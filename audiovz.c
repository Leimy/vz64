/*
 * virtio-sound (virtio 1.0) audio driver for Apple
 * Virtualization.framework guests.
 *
 * Presents the audio(3) contract (#A: /dev/audio, /dev/volume,
 * /dev/audioctl, /dev/audiostat) by registering an Audio card with
 * devaudio.c via addaudiocard().  devaudio.c owns the filesystem and
 * the volume-string parsing; this file owns the device.
 *
 * The device side is virtio-sound (virtio spec, "Sound Device").  Its
 * control protocol enumerates PCM streams (PCM_INFO) and drives each
 * through SET_PARAMS -> PREPARE -> START ... STOP -> RELEASE.  PCM data
 * rides its own virtqueue: each playback period is an xfer header
 * (stream id) + the PCM bytes, chained to a status word the device
 * writes on completion.
 *
 * PLAYBACK (txq, queue 2) and CAPTURE (rxq, queue 3).  Capture is the
 * mirror image of playback: pre-post empty buffers on the rxq, wait
 * for the device to fill them with recorded PCM, and copy the data
 * into a read ring for userspace.  The capture stream is found and
 * negotiated alongside the playback stream in pcmlifecycle().  The
 * host side must have the mic entitlement and an input stream
 * configured -- see section (b) of the working notes.
 *
 * Structure, by analogy:
 *   - the virtio 1.0 handshake (PCI cap walk, feature negotiation,
 *     queue setup, the sleep-on-used-ring discipline) is the same
 *     pattern as screen.c (virtio-gpu) and uartvz.c (virtio-console);
 *     initqueue / the Vqueue struct / ctlwait are copied from there.
 *   - the userspace-facing ring buffer and the read/write/buffered
 *     bookkeeping mirror audioac97.c's Ring.
 *
 * Bring-up: audioreset (called from devaudio's reset) runs the probe.
 * If there is no virtio-sound device (headless boot, host without an
 * audio device) the probe quietly fails and #A has no card -- exactly
 * like screen.c staying headless without a virtio-gpu.
 *
 * THE ONE REAL UNKNOWN is how strict Apple's virtio-sound backend is
 * about SET_PARAMS: which rate/format/channel combinations it accepts.
 * We probe a small candidate list the way screen.c's gpucreate2d()
 * probes pixel formats, most-likely-first, and remember what stuck.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"
#include "../port/error.h"
#include "../port/virtio10.h"
#include "../port/audioif.h"

enum {
	/* virtio-sound: virtio device id 25 -> modern PCI did 0x1040+25 */
	Viosndvid	= 0x1AF4,
	Viosnddid	= 0x1059,

	/* virtqueues */
	Vcontrolq	= 0,
	Veventq		= 1,
	Vtxq		= 2,
	Vrxq		= 3,
	Nqueue		= 4,

	/* descriptor flags */
	Dnext	= 1,
	Dwrite	= 2,

	/* struct sizes (vring) */
	VringSize	= 4,
	VdescSize	= 16,
	VusedSize	= 8,

	/*
	 * control request codes (virtio_snd spec).  Codes are grouped:
	 * jack 0x01xx, pcm 0x01xx (sic, separate base), chmap 0x03xx.
	 * Only the PCM set is needed for playback.
	 */
	RPcmInfo	= 0x0100,
	RPcmSetParams	= 0x0101,
	RPcmPrepare	= 0x0102,
	RPcmRelease	= 0x0103,
	RPcmStart	= 0x0104,
	RPcmStop	= 0x0105,

	/* control response status codes */
	SOk		= 0x8000,
	SErrBadMsg	= 0x8001,
	SErrNotSupp	= 0x8002,
	SErrIoErr	= 0x8003,

	/* pcm stream direction (virtio_snd_pcm_info.direction-ish) */
	Doutput		= 0,	/* VIRTIO_SND_D_OUTPUT (playback) */
	Dinput		= 1,	/* VIRTIO_SND_D_INPUT  (capture) */

	/*
	 * pcm formats (virtio_snd_pcm_set_params.format).  These are the
	 * VIRTIO_SND_PCM_FMT_* enum VALUES from the virtio spec
	 * (virtio_snd.h); the device's formats bitmap sets bit==value for
	 * each supported format.  The OLD values here were WRONG (S16 was
	 * 7, which is actually S18_3) -- that is why the advertised-bitmap
	 * filter rejected everything and printed "device advertised no
	 * format/rate we know" even though Apple advertises S16.
	 *
	 * Apple advertised formats 0xa0020 = bits 5 (S16), 17 (S32) and
	 * 19 (FLOAT), at rates 0x480 = bits 7 (48000) and 10 (96000).
	 */
	FmtU8		= 4,	/* VIRTIO_SND_PCM_FMT_U8  */
	FmtS16		= 5,	/* VIRTIO_SND_PCM_FMT_S16 */
	FmtS24		= 15,	/* VIRTIO_SND_PCM_FMT_S24 */
	FmtS32		= 17,	/* VIRTIO_SND_PCM_FMT_S32 */
	FmtFloat	= 19,	/* VIRTIO_SND_PCM_FMT_FLOAT */

	/* pcm sample rates (virtio_snd_pcm_set_params.rate enum value) */
	Rate44100	= 6,	/* VIRTIO_SND_PCM_RATE_44100 */
	Rate48000	= 7,	/* VIRTIO_SND_PCM_RATE_48000 */
	Rate96000	= 10,	/* VIRTIO_SND_PCM_RATE_96000 */

	/*
	 * virtio_snd_config (device-specific config space, behind the
	 * type-4 PCI cap).  jacks/streams/chmaps counts.
	 */
	Vsndcfg_jacks	= 0,
	Vsndcfg_streams	= 4,
	Vsndcfg_chmaps	= 8,

	/* features bitmap word 0 (virtio-sound has none we need) */

	/*
	 * EXACT virtio-sound wire sizes (bytes on the bus).  These are
	 * the lengths the device validates control descriptors against;
	 * they are NOT the C sizeof() of the matching structs.  The Plan
	 * 9 7c (arm64) compiler rounds EVERY struct's size up to a
	 * multiple of 8, so e.g. a one-word virtio_snd_hdr (4 bytes on the
	 * wire) has sizeof()==8, and virtio_snd_query_info (16 on the
	 * wire) had sizeof()==24 when it embedded a nested header struct.
	 * Sending sizeof() as the descriptor length makes the request
	 * over-long and Apple's strict virtio-sound backend rejects it
	 * with BAD_MSG -- which is exactly the [0..0] bad-msg we saw.
	 * So all control-descriptor lengths use these constants, never
	 * sizeof().  (The structs are still used to lay out fields, but
	 * because the trailing pad is past the last real field it does
	 * not corrupt anything we read/write; we just must not advertise
	 * it on the wire.)
	 */
	WSndhdr		= 4,	/* virtio_snd_hdr (just le32 code) */
	WSndquery	= 16,	/* virtio_snd_query_info */
	WSndpcminfo	= 32,	/* virtio_snd_pcm_info (per stream) */
	WSndsetparams	= 24,	/* virtio_snd_pcm_set_params */
	WSndpcmhdr	= 8,	/* virtio_snd_pcm_hdr */
	WSndxfer	= 4,	/* virtio_snd_pcm_xfer (just le32 stream id) */
	WSndstatus	= 8,	/* virtio_snd_pcm_status */

	/*
	 * playback/capture geometry.  S16_LE stereo => 4 bytes/frame.
	 * A period is one xfer; the ring holds a few periods so
	 * write()/read() can run ahead/behind the device.
	 */
	Chans		= 2,
	BytesPerFrame	= 2*Chans,	/* S16, 2ch */
	Period		= 4096,		/* bytes per xfer, frame-aligned */
	Nperiod		= 8,		/* periods buffered in the ring */
	Bufsize		= Period*Nperiod,

	/*
	 * Capture give-up cutoff.  When the host has no working input
	 * stream (no mic entitlement / no TCC consent -- the common case)
	 * EVERY rx period errors.  Without a cutoff capproc would re-post
	 * failing periods forever, burning CPU and (before the one-shot
	 * log flag) flooding /dev/kmesg.  After this many consecutive
	 * failures with nothing captured, capproc gives up and tears the
	 * capture stream down; a later read restarts it clean.
	 */
	Capmaxfail	= 32,
	/* backoff between failing rx periods (ms) so a dead mic does not spin */
	Capfailms	= 200,
};

typedef struct Vqueue Vqueue;
struct Vqueue
{
	Rendez;			/* slept on for used-ring completion */

	uint	qsize;
	uint	qmask;

	Vdesc	*desc;

	Vring	*avail;
	u16int	*availent;
	u16int	*availevent;

	Vring	*used;
	Vused	*usedent;
	u16int	*usedevent;
	u16int	lastused;

	Vio	notify;
};

/* control-queue message bodies (all little-endian on the wire; arm64
 * is LE so plain structs match) */
typedef struct Sndhdr Sndhdr;
struct Sndhdr
{
	u32int	code;		/* request code, or response status */
};

typedef struct Sndquery Sndquery;
struct Sndquery		/* virtio_snd_query_info: PCM_INFO request */
{
	u32int	code;		/* flattened from Sndhdr (see WSnd* note) */
	u32int	start_id;
	u32int	count;
	u32int	size;		/* per-entry size; device matches to its 32 */
};

typedef struct Sndpcminfo Sndpcminfo;
struct Sndpcminfo	/* virtio_snd_pcm_info (one returned per stream) */
{
	u32int	hda_fn_nid;	/* virtio_snd_info: the whole common header */
	u32int	features;
	u64int	formats;
	u64int	rates;
	u8int	direction;
	u8int	channels_min;
	u8int	channels_max;
	u8int	pad[5];
};

typedef struct Sndsetparams Sndsetparams;
struct Sndsetparams	/* virtio_snd_pcm_set_params */
{
	u32int	code;		/* was Sndhdr hdr; flattened */
	u32int	stream_id;
	u32int	buffer_bytes;
	u32int	period_bytes;
	u32int	features;
	u8int	channels;
	u8int	format;
	u8int	rate;
	u8int	pad;
};

typedef struct Sndpcmhdr Sndpcmhdr;
struct Sndpcmhdr	/* virtio_snd_pcm_hdr: prepare/start/stop/release */
{
	u32int	code;		/* was Sndhdr hdr; flattened */
	u32int	stream_id;
};

typedef struct Sndxfer Sndxfer;
struct Sndxfer		/* virtio_snd_pcm_xfer: prepended to each PCM period */
{
	u32int	stream_id;
};

typedef struct Sndstatus Sndstatus;
struct Sndstatus	/* virtio_snd_pcm_status: device writes after xfer */
{
	u32int	status;
	u32int	latency_bytes;
};

/* userspace-facing ring (mirrors audioac97.c) */
typedef struct Ring Ring;
struct Ring
{
	Rendez	r;
	uchar	*buf;
	ulong	nbuf;
	ulong	ri;
	ulong	wi;
};

static struct {
	int	present;	/* virtio-snd device discovered in PCI */
	int	ready;		/* playback PCM lifecycle done; audio may flow */
	int	failed;		/* activation/lifecycle gave up; no audio */
	int	intr;

	Pcidev	*pci;
	Vio	cfg;
	Vio	devcfg;		/* device-specific config (virtio_snd_config) */
	int	nstream;	/* total PCM streams from virtio_snd_config */
	Vio	isr;
	u32int	notifyoffmult;
	u32int	devfeat0;
	u32int	devfeat1;

	Vqueue	ctl;
	Vqueue	tx;
	Vqueue	rx;

	QLock	ctll;		/* serialises control-queue submitters */
	Lock	il;		/* brief vring index manipulation */

	/* playback stream */
	int	streamid;	/* playback stream id from PCM_INFO */
	u64int	formats;	/* PCM_INFO formats bitmap for the chosen stream */
	u64int	rates;		/* PCM_INFO rates bitmap for the chosen stream */
	int	chmin;		/* channel range advertised by the stream */
	int	chmax;
	int	started;	/* START issued, device draining */
	int	fmt;		/* accepted format code */
	int	rate;		/* accepted rate code */
	int	chans;		/* channel count we set up */
	int	speed;		/* rate in Hz (for /dev/audiostat) */

	/* capture stream */
	int	capstreamid;	/* capture stream id from PCM_INFO, or -1 */
	u64int	capformats;	/* PCM_INFO formats bitmap for capture */
	u64int	caprates;	/* PCM_INFO rates bitmap for capture */
	int	capchmin;
	int	capchmax;
	int	capstarted;	/* capture START issued */
	int	capready;	/* capture PCM lifecycle done; may read */
	int	capstop;	/* read side closed: capproc should tear down */
	int	caplogged;	/* rx-failure already logged once this session */
	int	capfmt;
	int	caprate;
	int	capchans;
	int	capspeed;

	Ring	outring;
	Ring	inring;		/* capture ring: device writes, user reads */

	/* a kproc drains outring -> txq */
	Rendez	kr;
	int	kproc;		/* drain kproc created (started on demand) */
	QLock	startl;		/* serialises lazy kproc creation */

	/* a kproc fills inring <- rxq */
	Rendez	ckr;
	int	ckproc;		/* capture kproc created */

	/* control scratch (single in-flight control command) */
	uchar	cmd[256];
	uchar	resp[256];

	/* tx scratch: one period in flight at a time on the data path */
	Sndxfer	txhdr;
	Sndstatus txstat;
	uchar	txbuf[Period];

	/* rx scratch: one period in flight at a time on the capture path */
	Sndxfer	rxhdr;
	Sndstatus rxstat;
	uchar	rxbuf[Period];

	Audio	*adev;
} vsnd;

static int audiovzprobe(Audio *);
static int sndactivate(void);
static int pcmlifecycle(void);
static int caplifecycle(void);
static void capteardown(void);
static int caprxready(void *);

/*
 * Wire-format size assertions.  We send the W* byte counts (not
 * sizeof) on the bus; these checks make sure each struct is at least
 * as large as the wire payload it represents, so storing the fields
 * never writes past the struct.  (sizeof() of these is generally
 * LARGER than the wire size because 7c rounds struct sizes up to a
 * multiple of 8 -- that trailing pad is harmless as long as we never
 * advertise it as the descriptor length, which is the whole point of
 * the W* constants.)
 */
typedef char Ckhdr[sizeof(Sndhdr) >= WSndhdr ? 1 : -1];
typedef char Ckquery[sizeof(Sndquery) >= WSndquery ? 1 : -1];
typedef char Ckpcminfo[sizeof(Sndpcminfo) >= WSndpcminfo ? 1 : -1];
typedef char Cksetparams[sizeof(Sndsetparams) >= WSndsetparams ? 1 : -1];
typedef char Ckpcmhdr[sizeof(Sndpcmhdr) >= WSndpcmhdr ? 1 : -1];
typedef char Ckxfer[sizeof(Sndxfer) >= WSndxfer ? 1 : -1];
typedef char Ckstatus[sizeof(Sndstatus) >= WSndstatus ? 1 : -1];

/*
 * ring helpers (verbatim shape from audioac97.c)
 */
static long
buffered(Ring *r)
{
	ulong ri, wi;

	ri = r->ri;
	wi = r->wi;
	if(wi >= ri)
		return wi - ri;
	return r->nbuf - (ri - wi);
}

static long
available(Ring *r)
{
	long m;

	m = (r->nbuf - BytesPerFrame) - buffered(r);
	if(m < 0)
		m = 0;
	return m;
}

static long
readring(Ring *r, uchar *p, long n)
{
	long n0, m;

	n0 = n;
	while(n > 0){
		if((m = buffered(r)) <= 0)
			break;
		if(m > n)
			m = n;
		if(p){
			if(r->ri + m > r->nbuf)
				m = r->nbuf - r->ri;
			memmove(p, r->buf + r->ri, m);
			p += m;
		}
		r->ri = (r->ri + m) % r->nbuf;
		n -= m;
	}
	return n0 - n;
}

static long
writering(Ring *r, uchar *p, long n)
{
	long n0, m;

	n0 = n;
	while(n > 0){
		if((m = available(r)) <= 0)
			break;
		if(m > n)
			m = n;
		if(p){
			if(r->wi + m > r->nbuf)
				m = r->nbuf - r->wi;
			memmove(r->buf + r->wi, p, m);
			p += m;
		}
		r->wi = (r->wi + m) % r->nbuf;
		n -= m;
	}
	return n0 - n;
}

/*
 * virtio queue setup (copied from screen.c / uartvz.c)
 */
static int
initqueue(Vqueue *q, int size)
{
	uchar *p;

	q->desc = mallocalign(VdescSize*size, 16, 0, 0);
	if(q->desc == nil)
		return -1;
	p = mallocalign(VringSize + 2*size + 2, 2, 0, 0);
	if(p == nil){
		free(q->desc);
		q->desc = nil;
		return -1;
	}
	q->avail = (void*)p;
	p += VringSize;
	q->availent = (void*)p;
	p += sizeof(u16int)*size;
	q->availevent = (void*)p;

	p = mallocalign(VringSize + VusedSize*size + 2, 4, 0, 0);
	if(p == nil){
		free(q->avail);
		free(q->desc);
		q->avail = nil;
		q->desc = nil;
		return -1;
	}
	q->used = (void*)p;
	p += VringSize;
	q->usedent = (void*)p;
	p += VusedSize*size;
	q->usedevent = (void*)p;

	q->qsize = size;
	q->qmask = size - 1;
	q->lastused = q->avail->idx = q->used->idx = 0;

	/*
	 * Leave used-ring completion interrupts ENABLED (avail flags 0).
	 * This whole call runs in the drain kproc (sndactivate), on a real
	 * KSTACK at spllo with up != nil, where qwait() sleeps on the
	 * completion interrupt (cansleep() true) and a device IRQ is
	 * harmless.  Boot-stack safety comes from never making the device
	 * live on the boot stack at all (sndfind does pure discovery, no
	 * DRIVER_OK), not from suppressing the interrupt here.
	 */
	q->avail->flags = 0;
	coherence();
	return 0;
}

static int
qhasroom(void *v)
{
	Vqueue *q = v;
	return q->used->idx != q->lastused;
}

static int
cansleep(void)
{
	return vsnd.intr && up != nil && islo();
}

/*
 * Wait for the single in-flight request on queue q to complete.
 * Sleeps on the used-ring interrupt when it can (process context,
 * interrupt wired, spllo); polls otherwise (early bring-up).  Never
 * forces interrupts off across the host round trip.
 */
static void
qwait(Vqueue *q)
{
	if(cansleep()){
		while(!qhasroom(q))
			tsleep(q, qhasroom, q, 5);
	} else {
		while(!qhasroom(q))
			coherence();
	}
	q->lastused = q->used->idx;
}

/*
 * used-ring completion interrupt: wake whoever waits on the control
 * or tx queue.  Reading the ISR status acknowledges it.
 */
static void
interrupt(Ureg*, void*)
{
	u32int isr;

	/*
	 * Read the ISR unconditionally: the read acknowledges and clears
	 * ALL asserted virtio interrupt sources (queue, bit 0, and config
	 * change, bit 1), de-asserting the level INTx line.  Only acking
	 * bit 0 would leave a config-change assertion latched, so the line
	 * would stay high and re-fire -- exactly the kind of un-acked level
	 * interrupt that nests on the stack.  This handler must also
	 * tolerate firing before the PCM lifecycle has run (queues are
	 * already initialised by sndactivate and the Rendez are valid, so
	 * the conditional wakeups are safe even when vsnd.ready is still 0).
	 */
	isr = vin8(&vsnd.isr, 0);
	if(isr & 1){
		if(qhasroom(&vsnd.ctl))
			wakeup(&vsnd.ctl);
		if(qhasroom(&vsnd.tx))
			wakeup(&vsnd.tx);
		if(qhasroom(&vsnd.rx))
			wakeup(&vsnd.rx);
	}
}

/*
 * Issue one control request synchronously: cmd (device-read) chained
 * to resp (device-write).  Returns the response status code, or 0 on a
 * queue error.  Caller holds vsnd.ctll (except early bring-up, which is
 * single-threaded).
 */
static u32int
ctlcmd(void *cmd, int cmdlen, void *resp, int resplen)
{
	Vqueue *q;
	Sndhdr *rh;
	int i;

	q = &vsnd.ctl;

	ilock(&vsnd.il);
	q->desc[0].addr = PADDR(cmd);
	q->desc[0].len = cmdlen;
	q->desc[0].flags = Dnext;
	q->desc[0].next = 1;

	q->desc[1].addr = PADDR(resp);
	q->desc[1].len = resplen;
	q->desc[1].flags = Dwrite;
	q->desc[1].next = 0;

	memset(resp, 0, resplen);

	i = q->avail->idx & q->qmask;
	q->availent[i] = 0;
	coherence();
	q->avail->idx++;
	coherence();
	vout16(&q->notify, 0, Vcontrolq);
	iunlock(&vsnd.il);

	qwait(q);

	rh = resp;
	return rh->code;
}

static char*
sndstatusname(u32int s)
{
	switch(s){
	case SOk:		return "ok";
	case SErrBadMsg:	return "bad-msg";
	case SErrNotSupp:	return "not-supported";
	case SErrIoErr:		return "io-error";
	case 0:			return "no-response";
	}
	return "unknown";
}

/*
 * PCM_INFO: enumerate all streams, find the first OUTPUT (playback)
 * and the first INPUT (capture) stream.  Sets vsnd.streamid (playback)
 * and vsnd.capstreamid (capture) independently; either can be -1 if
 * the device does not offer that direction.  Returns 0 on success
 * (at least playback found), -1 on total failure.
 */
static int
pcmfindstreams(void)
{
	Sndquery q;
	Sndpcminfo *info;
	u32int t;
	int total, perbatch, start, n, i, resplen;
	int gotout, gotin;

	/*
	 * The number of PCM streams is reported in virtio_snd_config; we
	 * must NOT guess it.  Apple's backend (unlike QEMU) strictly
	 * rejects a PCM_INFO whose start_id+count exceeds the real stream
	 * count with VIRTIO_SND_S_BAD_MSG -- which is the bad-msg we saw.
	 */
	total = vsnd.nstream;
	if(total <= 0){
		print("virtio-snd: device reports 0 streams\n");
		return -1;
	}

	/* how many info structs fit in one response buffer (wire sizes) */
	perbatch = (sizeof(vsnd.resp) - WSndhdr) / WSndpcminfo;
	if(perbatch < 1){
		print("virtio-snd: resp buffer too small for pcm-info\n");
		return -1;
	}

	gotout = 0;
	gotin = 0;
	vsnd.streamid = -1;
	vsnd.capstreamid = -1;

	for(start = 0; start < total; start += n){
		n = total - start;
		if(n > perbatch)
			n = perbatch;

		memset(&q, 0, sizeof q);
		q.code = RPcmInfo;
		q.start_id = start;
		q.count = n;
		q.size = WSndpcminfo;	/* device matches this to its own 32 */

		/*
		 * Size the device-writable response to EXACTLY the reply
		 * the device will produce, using WIRE sizes: the status
		 * header (WSndhdr=4) plus n info structs of WSndpcminfo=32.
		 * The request descriptor is the wire query size (WSndquery=
		 * 16), NOT sizeof q (which 7c rounds to 24).  Sending the
		 * over-long sizeof was the malformed PCM_INFO that Apple's
		 * strict backend rejected with BAD_MSG even for [0..0].
		 */
		resplen = WSndhdr + n*WSndpcminfo;
		t = ctlcmd(&q, WSndquery, vsnd.resp, resplen);
		if(t != SOk){
			print("virtio-snd: pcm-info[%d..%d]: %s (%#ux)\n",
				start, start+n-1, sndstatusname(t), t);
			return -1;
		}

		/* responses follow the status header (wire offset) in resp */
		info = (Sndpcminfo*)(vsnd.resp + WSndhdr);
		for(i = 0; i < n; i++, info++){
			print("virtio-snd: stream %d dir %d ch %d..%d "
				"formats %#.8ux%.8ux rates %#.8ux%.8ux\n",
				start+i, info->direction, info->channels_min,
				info->channels_max,
				(u32int)(info->formats>>32), (u32int)info->formats,
				(u32int)(info->rates>>32), (u32int)info->rates);
			if(info->direction == Doutput && !gotout){
				vsnd.formats = info->formats;
				vsnd.rates = info->rates;
				vsnd.chmin = info->channels_min;
				vsnd.chmax = info->channels_max;
				vsnd.streamid = start+i;
				gotout = 1;
			}
			if(info->direction == Dinput && !gotin){
				vsnd.capformats = info->formats;
				vsnd.caprates = info->rates;
				vsnd.capchmin = info->channels_min;
				vsnd.capchmax = info->channels_max;
				vsnd.capstreamid = start+i;
				gotin = 1;
			}
		}
	}
	if(vsnd.streamid < 0){
		print("virtio-snd: no playback stream\n");
		return -1;
	}
	return 0;
}

/*
 * Candidate (format, rate) pairs for SET_PARAMS, most-likely-first.
 * Apple's backend is pickier than QEMU's, so probe down the list the
 * way screen.c probes pixel formats rather than assume one combination.
 */
static struct {
	int	fmt;
	int	rate;
	int	hz;
} sndparams[] = {
	{ FmtS16, Rate48000, 48000 },
	{ FmtS16, Rate44100, 44100 },
};

static int
pcmsetparamstry(int sid, int fmt, int rate, int chans)
{
	Sndsetparams sp;

	memset(&sp, 0, sizeof sp);
	sp.code = RPcmSetParams;
	sp.stream_id = sid;
	sp.buffer_bytes = Bufsize;
	sp.period_bytes = Period;
	sp.features = 0;
	sp.channels = chans;
	sp.format = fmt;
	sp.rate = rate;
	/* wire sizes: request WSndsetparams (=24), reply a bare status hdr */
	return ctlcmd(&sp, WSndsetparams, vsnd.resp, WSndhdr);
}

/*
 * Negotiate SET_PARAMS for a given stream.  fmts/rates are the
 * device-advertised bitmaps; chmin/chmax the channel range.  On
 * success, stores the accepted format/rate/chans/speed in *ofmt etc.
 * label is "playback" or "capture" for diagnostics.
 */
static int
pcmsetparams(int sid, u64int fmts, u64int rates,
	int chmin, int chmax, int *ofmt, int *orate,
	int *ochans, int *ospeed, char *label)
{
	u32int t;
	int i, chans, tried;

	/*
	 * Drive SET_PARAMS only with format/rate/channel combinations the
	 * device advertised in PCM_INFO.  Apple's backend rejects anything
	 * outside the advertised formats/rates bitmaps with BAD_MSG, which
	 * is why the earlier blind {S16/48k, S16/44.1k} probe failed even
	 * though both look reasonable.  Prefer stereo, but fall back to the
	 * largest channel count in the advertised [chmin..chmax] range.
	 */
	chans = Chans;
	if(chans > chmax)
		chans = chmax;
	if(chans < chmin)
		chans = chmin;
	if(chans < 1)
		chans = 1;

	tried = 0;
	for(i = 0; i < nelem(sndparams); i++){
		/* skip combinations the device did not advertise */
		if((fmts & ((u64int)1<<sndparams[i].fmt)) == 0)
			continue;
		if((rates & ((u64int)1<<sndparams[i].rate)) == 0)
			continue;
		tried++;

		t = pcmsetparamstry(sid, sndparams[i].fmt, sndparams[i].rate, chans);
		if(t == SOk){
			*ofmt = sndparams[i].fmt;
			*orate = sndparams[i].rate;
			*ospeed = sndparams[i].hz;
			*ochans = chans;
			print("virtio-snd: %s set-params: ok (fmt %d rate %dHz "
				"%dch, candidate %d of %d)\n",
				label, sndparams[i].fmt, sndparams[i].hz,
				chans, i, nelem(sndparams));
			return 0;
		}
		print("virtio-snd: %s set-params fmt %d rate %dHz %dch: "
			"%s (%#ux)\n",
			label, sndparams[i].fmt, sndparams[i].hz, chans,
			sndstatusname(t), t);
		if(t != SErrNotSupp && t != SErrBadMsg)
			return -1;
	}
	if(tried == 0)
		print("virtio-snd: %s set-params: device advertised no "
			"format/rate we know (formats %#.8ux%.8ux rates "
			"%#.8ux%.8ux)\n", label,
			(u32int)(fmts>>32), (u32int)fmts,
			(u32int)(rates>>32), (u32int)rates);
	else
		print("virtio-snd: %s set-params: host rejected all %d "
			"advertised candidates\n", label, tried);
	return -1;
}

/*
 * Issue a simple PCM control command (PREPARE/START/STOP/RELEASE) for
 * the given stream id.  Returns 0 on SOk, -1 otherwise.
 */
static int
pcmsimpleid(int sid, int code)
{
	Sndpcmhdr h;
	u32int t;

	memset(&h, 0, sizeof h);
	h.code = code;
	h.stream_id = sid;
	/* wire sizes: request WSndpcmhdr (=8), reply a bare status header */
	t = ctlcmd(&h, WSndpcmhdr, vsnd.resp, WSndhdr);
	if(t != SOk){
		print("virtio-snd: pcm cmd %#ux (stream %d): %s (%#ux)\n",
			code, sid, sndstatusname(t), t);
		return -1;
	}
	return 0;
}

static int
pcmsimple(int code)
{
	return pcmsimpleid(vsnd.streamid, code);
}

/*
 * Post one period of PCM on txq and wait for completion.
 * Descriptor chain: xfer header (read) -> data (read) -> status (write).
 * Caller (the drain kproc) is the only tx submitter, so no extra lock.
 */
static int
txperiod(uchar *data, int len)
{
	Vqueue *q;
	int i;

	q = &vsnd.tx;
	if(q->qsize < 3)
		return -1;

	vsnd.txhdr.stream_id = vsnd.streamid;
	memset(&vsnd.txstat, 0, sizeof vsnd.txstat);

	ilock(&vsnd.il);
	q->desc[0].addr = PADDR(&vsnd.txhdr);
	q->desc[0].len = WSndxfer;	/* wire size, not sizeof (7c pads to 8) */
	q->desc[0].flags = Dnext;
	q->desc[0].next = 1;

	q->desc[1].addr = PADDR(data);
	q->desc[1].len = len;
	q->desc[1].flags = Dnext;
	q->desc[1].next = 2;

	q->desc[2].addr = PADDR(&vsnd.txstat);
	q->desc[2].len = WSndstatus;	/* wire size */
	q->desc[2].flags = Dwrite;
	q->desc[2].next = 0;

	i = q->avail->idx & q->qmask;
	q->availent[i] = 0;
	coherence();
	q->avail->idx++;
	coherence();
	vout16(&q->notify, 0, Vtxq);
	iunlock(&vsnd.il);

	qwait(q);

	if(vsnd.txstat.status != SOk)
		return -1;
	return 0;
}

/*
 * Post one empty period on rxq and wait for the device to fill it with
 * captured PCM.  Descriptor chain: xfer header (read by device, tells
 * it which stream) -> data buffer (written by device) -> status (written
 * by device).  Returns the number of bytes of PCM received, or -1 on
 * error.  Caller (the capture kproc) is the only rx submitter.
 */
static int
rxperiod(uchar *data, int len)
{
	Vqueue *q;
	int i;

	q = &vsnd.rx;
	if(q->qsize < 3)
		return -1;

	vsnd.rxhdr.stream_id = vsnd.capstreamid;
	memset(&vsnd.rxstat, 0, sizeof vsnd.rxstat);

	ilock(&vsnd.il);
	/* descriptor 0: xfer header (device-readable, tells stream id) */
	q->desc[0].addr = PADDR(&vsnd.rxhdr);
	q->desc[0].len = WSndxfer;
	q->desc[0].flags = Dnext;
	q->desc[0].next = 1;

	/* descriptor 1: PCM data buffer (device-writable) */
	q->desc[1].addr = PADDR(data);
	q->desc[1].len = len;
	q->desc[1].flags = Dwrite | Dnext;
	q->desc[1].next = 2;

	/* descriptor 2: status (device-writable) */
	q->desc[2].addr = PADDR(&vsnd.rxstat);
	q->desc[2].len = WSndstatus;
	q->desc[2].flags = Dwrite;
	q->desc[2].next = 0;

	i = q->avail->idx & q->qmask;
	q->availent[i] = 0;
	coherence();
	q->avail->idx++;
	coherence();
	vout16(&q->notify, 0, Vrxq);
	iunlock(&vsnd.il);

	/*
	 * Wait for the device to fill this period, but stay responsive to
	 * capstop (read side closed) and card teardown so a kill during a
	 * never-completing rx period (dead mic) does not hang the kproc.
	 * Like qwait, sleep on the completion interrupt when possible and
	 * poll otherwise; caprxready also wakes on capstop/!present/failed.
	 */
	if(cansleep()){
		while(!caprxready(q))
			tsleep(&q->Rendez, caprxready, q, Capfailms);
	} else {
		while(!caprxready(q))
			coherence();
	}
	if(!qhasroom(q)){
		/* woke for capstop/teardown, not a completion */
		return -1;
	}
	q->lastused = q->used->idx;

	if(vsnd.rxstat.status != SOk)
		return -1;
	/*
	 * The device returns the actual number of bytes captured in the
	 * used-ring entry's len field (total of all device-writable
	 * descriptors).  We subtract the status size to get the PCM bytes.
	 * However, the simplest approach: the device fills up to `len`
	 * bytes of the PCM buffer.  We return `len` since a period is a
	 * fixed-size chunk and the device should fill it completely at
	 * steady state.
	 */
	return len;
}

/*
 * Predicate for the drain kproc: there is data to push, or the card is
 * shutting down (vsnd.ready cleared) so the kproc should wake and exit.
 */
static int
outavail(void *)
{
	return buffered(&vsnd.outring) > 0 || !vsnd.ready;
}

/*
 * Predicate for a blocked writer (audiovzwrite): the ring has room
 * again, OR the card is gone/torn down (vsnd.present cleared) so the
 * writer should stop waiting and return.  NOTE this deliberately does
 * NOT key on vsnd.ready: before the kproc finishes the PCM lifecycle
 * (ready==0) the ring may legitimately be full and the writer must
 * SLEEP, not spin -- the kproc wakes it via wakeup(&outring.r) as soon
 * as it starts draining.  Keying the writer's wait on !ready (as the
 * drain predicate does) would busy-spin the writer until the lifecycle
 * completed.
 */
static int
writeroom(void *)
{
	return available(&vsnd.outring) > 0 || !vsnd.present || vsnd.failed;
}

/*
 * Drain kproc: pull period-sized chunks out of the ring and push them
 * to the device.  Sleeps when the ring is empty; woken by write().
 * This is the analogue of the ac97 interrupt advancing the DMA ring,
 * but virtio-sound is poll/complete per xfer so a dedicated context is
 * simpler than driving it from the completion interrupt.
 */
static void
drainproc(void *)
{
	long n;

	/*
	 * We are on a real KSTACK in process context at spllo with up !=
	 * nil.  THIS is where the device is first made live: sndactivate()
	 * runs the whole virtio handshake (DRIVER_OK and all), then
	 * pcmlifecycle() runs PCM_INFO/SET_PARAMS/PREPARE.  Any interrupt
	 * the device raises here nests one ordinary trap frame on the 8K
	 * KSTACK -- harmless, unlike the boot stack.
	 *
	 * The waserror() loop is established FIRST: control commands go
	 * through qwait()->tsleep(), which can raise (e.g. on a posted
	 * note), and we want any such error caught here rather than
	 * propagating out of a kproc with no handler.
	 */
	while(waserror())
		;

	if(sndactivate() < 0){
		print("virtio-snd: device activation failed; no audio\n");
		vsnd.failed = 1;
		vsnd.present = 0;
		wakeup(&vsnd.outring.r);
		pexit("audiovz activation failed", 1);
	}

	if(pcmlifecycle() < 0){
		print("virtio-snd: PCM lifecycle failed; no audio\n");
		/*
		 * The kproc is about to exit, so nothing will ever drain the
		 * ring.  Mark the card not-present and wake any writer blocked
		 * in audiovzwrite (writeroom keys on vsnd.present) so it
		 * returns short instead of sleeping forever with no draining
		 * kproc to wake it.  vsnd.ready stays 0; further writes fill
		 * the ring once and then return short.
		 */
		vsnd.failed = 1;
		vsnd.present = 0;
		wakeup(&vsnd.outring.r);
		pexit("audiovz pcm setup failed", 1);
	}
	if(vsnd.adev != nil)
		vsnd.adev->speed = vsnd.speed;

	/*
	 * Activation succeeded and the ring is now allocated (nbuf > 0).
	 * The writer that triggered audiovzstart() is asleep in
	 * audiovzwrite() on writeroom(), which could not have become true
	 * yet (nbuf was 0, so available()==0).  Wake it now that there is
	 * real room so it fills the ring; without this the writer would
	 * sleep forever and the drain loop below would sleep on an empty
	 * ring -- a deadlock.
	 */
	wakeup(&vsnd.outring.r);

	for(;;){
		while(buffered(&vsnd.outring) <= 0)
			sleep(&vsnd.kr, outavail, nil);
		if(!vsnd.ready)
			break;

		n = readring(&vsnd.outring, vsnd.txbuf, Period);
		if(n <= 0)
			continue;

		/* frame-align: pad a short tail with silence */
		if(n % BytesPerFrame){
			int pad = BytesPerFrame - (n % BytesPerFrame);
			if(n + pad <= Period){
				memset(vsnd.txbuf + n, 0, pad);
				n += pad;
			}
		}

		if(!vsnd.started){
			qlock(&vsnd.ctll);
			if(pcmsimple(RPcmStart) < 0){
				qunlock(&vsnd.ctll);
				print("virtio-snd: start failed; dropping audio\n");
				/* discard the ring so write() does not wedge */
				vsnd.outring.ri = vsnd.outring.wi;
				wakeup(&vsnd.outring.r);
				continue;
			}
			qunlock(&vsnd.ctll);
			vsnd.started = 1;
		}

		if(txperiod(vsnd.txbuf, n) < 0)
			print("virtio-snd: tx period failed\n");

		/* room freed: wake a blocked write() */
		wakeup(&vsnd.outring.r);
	}
	pexit("audiovz drain exiting", 1);
}

/*
 * Predicate for a blocked reader (audiovzread): the capture ring has
 * data to read, OR the card is gone, OR capture has stopped/given up
 * (capstop set, or the capture kproc is no longer running and capture
 * is not ready) so the reader should give up rather than hang.
 */
static int
indata(void *)
{
	return buffered(&vsnd.inring) > 0 || !vsnd.present || vsnd.failed
		|| vsnd.capstop || (!vsnd.capready && !vsnd.ckproc);
}

/*
 * Predicate for the capture kproc: an rx period completed, OR the read
 * side asked us to stop, OR the card went away.  capproc sleeps on this
 * (via the rx Rendez) instead of spinning, so a dead-mic failure path
 * backs off rather than busy-looping.
 */
static int
caprxready(void *)
{
	return qhasroom(&vsnd.rx) || vsnd.capstop || !vsnd.present || vsnd.failed;
}

/*
 * Quiesce the capture stream and reset capture state so a later read
 * can restart it from scratch.  Runs on the capture kproc's KSTACK as
 * it exits (or after a give-up).  Best-effort: STOP+RELEASE the capture
 * stream, drop any stale rx completions, reset the capture ring, and
 * clear the per-session flags (capstarted/capready/capstop/caplogged)
 * and ckproc so audiovzcapstart() will spawn a fresh kproc next time.
 */
static void
capteardown(void)
{
	if(vsnd.capstarted){
		qlock(&vsnd.ctll);
		pcmsimpleid(vsnd.capstreamid, RPcmStop);
		pcmsimpleid(vsnd.capstreamid, RPcmRelease);
		qunlock(&vsnd.ctll);
	}

	/* drop any rx completions the device may still post */
	vsnd.rx.lastused = vsnd.rx.used->idx;

	/* reset the capture ring so stale PCM is not read next session */
	vsnd.inring.ri = vsnd.inring.wi = 0;

	vsnd.capstarted = 0;
	vsnd.capready = 0;
	vsnd.capstop = 0;
	vsnd.caplogged = 0;
	vsnd.ckproc = 0;

	/* wake any reader still blocked so it returns short */
	wakeup(&vsnd.inring.r);
}

/*
 * Capture kproc: post empty periods on rxq, wait for the device to
 * fill them, and copy the captured PCM into the inring for userspace.
 * Mirror image of drainproc for the tx path -- but the capture path
 * must survive the common case of a host with NO working input stream
 * (no mic entitlement / no TCC consent), where every rx period errors.
 *
 * To avoid the old failure storm (which re-posted each failed period
 * instantly, spun the CPU, and flooded /dev/kmesg with "rx period
 * failed"), this loop:
 *   - sleeps on the rx Rendez (caprxready) rather than spinning;
 *   - logs the FIRST failure of the session ONCE (caplogged), never the
 *     rest, regardless of any intervening OK periods;
 *   - backs off Capfailms on a failing period instead of re-posting at
 *     line rate;
 *   - gives up after Capmaxfail consecutive failures with nothing
 *     captured, tearing the capture stream down (a later read restarts);
 *   - exits promptly when the read side closes (capstop) or the card
 *     goes away.
 */
static void
capproc(void *)
{
	int n, nfail;

	while(waserror())
		;

	/*
	 * The capture PCM lifecycle runs here, on a real KSTACK with
	 * up != nil.  The device is already activated (sndactivate ran
	 * in drainproc), so we just need SET_PARAMS/PREPARE/START for
	 * the capture stream.
	 */
	if(caplifecycle() < 0){
		print("virtio-snd: capture lifecycle failed; no mic\n");
		capteardown();
		pexit("audiovz capture failed", 1);
	}

	nfail = 0;
	for(;;){
		if(!vsnd.capready || vsnd.capstop || !vsnd.present || vsnd.failed)
			break;

		n = rxperiod(vsnd.rxbuf, Period);
		if(n <= 0){
			/*
			 * Persistent rx failure is the no-working-mic case.
			 * Log once per session, back off so we do not spin,
			 * and give up after Capmaxfail in a row.
			 */
			if(!vsnd.caplogged){
				print("virtio-snd: rx period failed "
					"(no working host input?); "
					"backing off\n");
				vsnd.caplogged = 1;
			}
			if(++nfail >= Capmaxfail){
				print("virtio-snd: capture gave up after %d "
					"failures; close and re-open to retry\n",
					nfail);
				break;
			}
			tsleep(&vsnd.ckr, return0, nil, Capfailms);
			continue;
		}
		nfail = 0;

		writering(&vsnd.inring, vsnd.rxbuf, n);
		wakeup(&vsnd.inring.r);
	}

	capteardown();
	pexit("audiovz capture exiting", 1);
}

/*
 * Lazy activation.  The whole device lifecycle (virtio handshake +
 * PCM_INFO/SET_PARAMS/PREPARE + draining) runs in the drain kproc.  We
 * must NOT create that kproc on the boot stack during chandevreset:
 * creating/readying a kproc in that pre-scheduler window (up == nil) is
 * itself a boot-stack trap source (see the vz64 PORT NOTES).
 *
 * The cure is to create the kproc only on FIRST USE of /dev/audio, from
 * one of the audio(3) callbacks (audiovzwrite et al), all of which run
 * in an ordinary process context after schedinit with up != nil.  Then
 * kproc()/ready() can never touch the boot stack.
 *
 * Idempotent and serialised by vsnd.startl: only the first caller wins
 * the race and spawns the kproc; later callers (or callers after a
 * failed activation) just return.
 */
static void
audiovzstart(void)
{
	if(!vsnd.present || vsnd.kproc || vsnd.failed)
		return;
	qlock(&vsnd.startl);
	if(vsnd.present && !vsnd.kproc && !vsnd.failed){
		vsnd.kproc = 1;
		kproc("audiovz", drainproc, nil);
	}
	qunlock(&vsnd.startl);
}

/*
 * Start the capture kproc.  The playback kproc (which runs
 * sndactivate) must be started first so the device is live.
 * The capture kproc runs caplifecycle (SET_PARAMS/PREPARE/START
 * for the capture stream) then loops posting rx periods.
 */
static void
audiovzcapstart(void)
{
	if(!vsnd.present || vsnd.ckproc || vsnd.failed)
		return;
	if(vsnd.capstreamid < 0)
		return;

	/* ensure device activation has been kicked off */
	audiovzstart();

	qlock(&vsnd.startl);
	if(vsnd.present && !vsnd.ckproc && !vsnd.failed && vsnd.capstreamid >= 0){
		vsnd.ckproc = 1;
		kproc("audiovzcap", capproc, nil);
	}
	qunlock(&vsnd.startl);
}

/*
 * audio(3) callbacks
 */
static long
audiovzread(Audio *, void *vp, long n, vlong)
{
	uchar *p, *e;
	Ring *ring;
	long m;

	/*
	 * First read triggers capture activation: spawn the capture
	 * kproc (which runs caplifecycle) now that we are in process
	 * context with up != nil.
	 */
	audiovzcapstart();

	p = vp;
	e = p + n;
	ring = &vsnd.inring;
	while(p < e){
		if((m = readring(ring, p, e - p)) <= 0){
			sleep(&ring->r, indata, nil);
			if(!vsnd.present || vsnd.failed)
				break;
			continue;
		}
		p += m;
	}
	return p - (uchar*)vp;
}

static long
audiovzwrite(Audio *, void *vp, long n, vlong)
{
	uchar *p, *e;
	Ring *ring;
	long m;

	/*
	 * First write triggers lazy activation: spawn the drain kproc
	 * (which runs the virtio handshake + PCM lifecycle) now that we
	 * are in process context with up != nil.  Before the kproc has
	 * allocated the ring, available()/writering() see nbuf==0 and
	 * return 0, so the loop below sleeps on writeroom() until the
	 * kproc frees room (or activation fails and clears present).
	 */
	audiovzstart();

	p = vp;
	e = p + n;
	ring = &vsnd.outring;
	while(p < e){
		if((m = writering(ring, p, e - p)) <= 0){
			/*
			 * Ring full.  Wake the drain proc and sleep until it
			 * frees room (writeroom), or until the card is torn
			 * down (vsnd.present cleared).  We wait on the SAME
			 * vsnd.present guard rather than vsnd.ready so a writer
			 * that arrives before the kproc has finished the PCM
			 * lifecycle blocks cleanly instead of busy-spinning;
			 * the kproc wakes us once it starts draining.
			 */
			wakeup(&vsnd.kr);
			sleep(&ring->r, writeroom, nil);
			if(!vsnd.present)
				break;
			continue;
		}
		p += m;
		wakeup(&vsnd.kr);
	}
	return p - (uchar*)vp;
}

static long
audiovzbuffered(Audio *)
{
	return buffered(&vsnd.outring);
}

static long
audiovzstatus(Audio *, void *a, long n, vlong)
{
	return snprint((char*)a, n,
		"bufsize %6d buffered %6ld speed %d chans %d\n",
		Period, buffered(&vsnd.outring), vsnd.speed, vsnd.chans);
}

static void
audiovzclose(Audio *, int mode)
{
	Ring *ring;

	if(mode == OWRITE || mode == ORDWR){
		/*
		 * Pad the ring out to a whole period so the final partial
		 * period is flushed rather than stranded, then let it drain.
		 */
		ring = &vsnd.outring;
		while(ring->wi % Period)
			if(writering(ring, (uchar*)"", 1) <= 0)
				break;
		wakeup(&vsnd.kr);
	}

	if(mode == OREAD || mode == ORDWR){
		/*
		 * Read side closed (e.g. `cat /dev/audio` was killed).  Tell
		 * the capture kproc to stop and tear down: set capstop and
		 * wake it (on the rx Rendez and its own ckr) plus any reader
		 * still blocked in audiovzread.  Without this the capture
		 * kproc would loop forever -- the failure-storm bug.
		 */
		if(vsnd.ckproc){
			vsnd.capstop = 1;
			wakeup(&vsnd.rx);
			wakeup(&vsnd.ckr);
			wakeup(&vsnd.inring.r);
		}
	}
}

/*
 * /dev/volume.  Apple's virtio-sound backend exposes no mixer controls
 * to the guest (no volume/jack remote controls), so volume is a no-op
 * stub: report a single 100% master so userspace tools that expect a
 * volume file do not choke.
 */
static Volume voltab[] =
{
	"master", 0, 100, Stereo, 0,
	0,
};

static int
volget(Audio *, int, int *v)
{
	v[0] = v[1] = 100;
	return 0;
}

static int
volset(Audio *, int, int *)
{
	return 0;	/* no host-side mixer; ignore */
}

static long
audiovzvolread(Audio *adev, void *a, long n, vlong off)
{
	return genaudiovolread(adev, a, n, off, voltab, volget, 0);
}

static long
audiovzvolwrite(Audio *adev, void *a, long n, vlong off)
{
	return genaudiovolwrite(adev, a, n, off, voltab, volset, 0);
}

/*
 * PCI capability walking (same as screen.c / uartvz.c)
 */
static int
matchvirtiocfgcap(Pcidev *p, int cap, int off, int typ)
{
	int bar;

	if(cap != 9 || pcicfgr8(p, off+3) != typ)
		return 1;
	bar = pcicfgr8(p, off+4);
	if(bar < 0 || bar >= nelem(p->mem)
	|| p->mem[bar].size == 0)
		return 1;
	return 0;
}

static int
virtiocap(Pcidev *p, int typ)
{
	return pcienumcaps(p, matchvirtiocfgcap, typ);
}

/*
 * sndfind: the ENTIRE boot-stack footprint of this driver.  It is a
 * pure PCI discovery stub: walk the bus for a virtio-sound device,
 * remember its Pcidev, mark the card present, and return.  It does NOT
 * touch the device in any way that could make it live: NO pcienable, NO
 * cap mapping, NO virtio status writes, NO feature negotiation, NO queue
 * setup, NO DRIVER_OK, NO bus-master enable, NO intrenable.  Nothing it
 * does can cause the device to raise an interrupt.
 *
 * WHY this is a bare stub: chandevreset() (devaudio's audioreset) runs
 * this with up == nil on the ~8K per-Mach boot stack, BEFORE
 * schedinit().  Bringing the device live there (DRIVER_OK + bus master)
 * lets it assert an interrupt that, taken on the tiny boot stack with no
 * driver context, nests trap frames until the stack overflows and
 * kenter() panics.  Keeping the device inert here makes that impossible
 * by any interrupt path.  (See the vz64 PORT NOTES, section (e) of
 * /usr/dave/9vz-audio-and-fullscreen.md, for the full story.)
 *
 * The full activation (pcienable, cap walk, reset, feature negotiation,
 * queue setup, intrenable, DRIVER_OK, bus master) is done by the drain
 * kproc in sndactivate(), after schedinit, on a real KSTACK at spllo
 * with up != nil -- where a device interrupt nests one ordinary trap
 * frame on an 8K KSTACK, harmless.
 *
 * Returns 0 if a virtio-sound device is present (Pcidev saved); -1 if
 * none is found.
 */
static int
sndfind(void)
{
	Pcidev *p;

	for(p = nil; p = pcimatch(p, Viosndvid, Viosnddid);){
		if(p->rid == 0)
			continue;
		/*
		 * Confirm it really is a modern virtio device with a
		 * common-config cap, but do NOT map it (mapping is a vmap,
		 * harmless, but we keep the boot-stack work to the bare
		 * minimum and leave ALL register access to the kproc).
		 */
		if(virtiocap(p, 1) < 0)
			continue;

		vsnd.pci = p;
		vsnd.present = 1;
		return 0;
	}
	return -1;
}

/*
 * sndactivate: the full virtio-sound handshake, run by the drain kproc
 * (sndactivate is called from drainproc) on a real KSTACK at spllo with
 * up != nil -- NEVER on the boot stack.  PCI enable, cap walk, device
 * reset, feature negotiation (VERSION_1 only), queue setup, interrupt
 * wiring, DRIVER_OK, and bus-master enable all happen here.  Because the
 * scheduler is running by the time this executes, any interrupt the
 * device raises after DRIVER_OK lands on this kproc's 8K KSTACK and
 * nests a single ordinary trap frame -- it cannot exhaust the tiny boot
 * stack the way it did when DRIVER_OK was written in sndfind().
 *
 * Returns 0 on success (queues live, DRIVER_OK set, handler wired,
 * vsnd.cfg/devcfg/isr mapped); -1 otherwise.
 */
static int
sndactivate(void)
{
	Pcidev *p;
	Vio cfg, notifybase;
	Vqueue *q;
	int cap, n, qi;

	p = vsnd.pci;
	if(p == nil)
		return -1;

	if((cap = virtiocap(p, 1)) < 0)
		return -1;
	if(virtiomapregs(p, cap, Vconf_sz, &cfg) == nil)
		return -1;
	vsnd.cfg = cfg;

	pcienable(p);
	print("virtio-snd: %T did %#ux rid %d\n", p->tbdf, p->did, p->rid);

	if(virtiomapregs(p, virtiocap(p, 3), 0, &vsnd.isr) == nil)
		goto Bad;
	/*
	 * Device-specific config (virtio_snd_config) is behind the type-4
	 * cap.  We need it to learn the real PCM stream count (see
	 * pcmfindplayback): Apple's backend rejects a PCM_INFO that
	 * overruns the stream count with BAD_MSG.
	 */
	if(virtiomapregs(p, virtiocap(p, 4), 0, &vsnd.devcfg) == nil)
		goto Bad;
	cap = virtiocap(p, 2);
	if(virtiomapregs(p, cap, 0, &notifybase) == nil)
		goto Bad;
	vsnd.notifyoffmult = pcicfgr32(p, cap+16);

	/* reset device */
	coherence();
	vout8(&cfg, Vconf_status, 0);
	while(vin8(&cfg, Vconf_status) != 0)
		delay(1);
	vout8(&cfg, Vconf_status, Sacknowledge|Sdriver);

	/*
	 * Negotiate features: accept only VIRTIO_F_VERSION_1 (bit 32 = bit
	 * 0 of word 1) and drive word 0 to zero.  Word 0 is where
	 * virtio-sound's optional CTLS feature lives; we want none of it
	 * (basic PCM playback only), matching the pure-2D choice screen.c
	 * makes for virtio-gpu.  The offered words are read only for the
	 * diagnostic print.
	 */
	vout32(&cfg, Vconf_devfeatsel, 0);
	vsnd.devfeat0 = vin32(&cfg, Vconf_devfeat);
	vout32(&cfg, Vconf_devfeatsel, 1);
	vsnd.devfeat1 = vin32(&cfg, Vconf_devfeat);

	vout32(&cfg, Vconf_drvfeatsel, 1);
	vout32(&cfg, Vconf_drvfeat, vsnd.devfeat1 & Fversion1);
	vout32(&cfg, Vconf_drvfeatsel, 0);
	vout32(&cfg, Vconf_drvfeat, 0);

	print("virtio-snd: devfeat %#.8ux:%#.8ux drvfeat %#.8ux:%#.8ux\n",
		vsnd.devfeat1, vsnd.devfeat0,
		vsnd.devfeat1 & Fversion1, 0);

	vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sfeaturesok);
	if((vin8(&cfg, Vconf_status) & Sfeaturesok) == 0){
		print("virtio-snd: device rejected features\n");
		goto Bad;
	}

	/*
	 * Set up controlq (0), txq (2), and rxq (3).  eventq (1) is not
	 * used.  The spec permits leaving unused queues disabled.
	 */
	for(qi = 0; qi < 3; qi++){
		int idx;

		switch(qi){
		case 0: idx = Vcontrolq; q = &vsnd.ctl; break;
		case 1: idx = Vtxq; q = &vsnd.tx; break;
		case 2: idx = Vrxq; q = &vsnd.rx; break;
		default: continue;
		}

		vout16(&cfg, Vconf_queuesel, idx);
		n = vin16(&cfg, Vconf_queuesize);
		if(n == 0 || (n & (n-1)) != 0){
			print("virtio-snd: queue %d bad size %d\n", idx, n);
			goto Bad;
		}
		if(initqueue(q, n) < 0){
			print("virtio-snd: queue %d alloc failed\n", idx);
			goto Bad;
		}
		q->notify = notifybase;
		if(q->notify.type == Vio_mem)
			q->notify.mem += vsnd.notifyoffmult
				* vin16(&cfg, Vconf_queuenotifyoff);
		else
			q->notify.port += vsnd.notifyoffmult
				* vin16(&cfg, Vconf_queuenotifyoff);
		coherence();
		vout64(&cfg, Vconf_queuedesc, PADDR(q->desc));
		vout64(&cfg, Vconf_queueavail, PADDR(q->avail));
		vout64(&cfg, Vconf_queueused, PADDR(q->used));
		vout16(&cfg, Vconf_queuesel, idx);
		vout16(&cfg, Vconf_queueenable, 1);
	}

	/*
	 * Wire the completion-interrupt handler BEFORE DRIVER_OK so the
	 * very first interrupt the device can raise is serviced (the
	 * handler reads the ISR, de-asserting a level line).  We are on a
	 * real KSTACK at spllo with up != nil, so this is also where
	 * qwait() switches to the sleeping path (cansleep() becomes true
	 * once vsnd.intr is set).  intrenable for a PCI device only adds
	 * interrupt() to pciqemu's vec[]; the slot's GIC SPI was already
	 * enabled at PCI-scan time.
	 */
	if(!vsnd.intr){
		intrenable(p->intl, interrupt, nil, p->tbdf, "virtio-snd");
		vsnd.intr = 1;
	}

	vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sdriverok);
	pcisetbme(p);

	/*
	 * Allocate the userspace rings now that activation succeeded.
	 */
	if(vsnd.outring.buf == nil){
		vsnd.outring.buf = mallocalign(Bufsize, BY2PG, 0, 0);
		if(vsnd.outring.buf == nil){
			print("virtio-snd: no memory for playback ring\n");
			goto Bad;
		}
		vsnd.outring.nbuf = Bufsize;
		vsnd.outring.ri = vsnd.outring.wi = 0;
	}
	if(vsnd.inring.buf == nil){
		vsnd.inring.buf = mallocalign(Bufsize, BY2PG, 0, 0);
		if(vsnd.inring.buf == nil){
			print("virtio-snd: no memory for capture ring\n");
			goto Bad;
		}
		vsnd.inring.nbuf = Bufsize;
		vsnd.inring.ri = vsnd.inring.wi = 0;
	}

	print("virtio-snd: DRIVER_OK set; queues live, handler wired\n");
	return 0;
Bad:
	pcidisable(p);
	return -1;
}

/*
 * The PCM lifecycle, run by the drain kproc on a real KSTACK at spllo
 * (NOT on the boot stack).  Reads the device-specific config, enumerates
 * streams (PCM_INFO), picks a playback stream, negotiates SET_PARAMS,
 * and issues PREPARE.  START is deferred to the first period written
 * (drainproc) so we do not run a hungry stream against an empty ring.
 * Returns 0 on success (vsnd.ready set), -1 otherwise.
 */
static int
pcmlifecycle(void)
{
	/*
	 * Read the device-specific config now that DRIVER_OK is set.
	 * streams is the total number of PCM streams; pcmfindstreams
	 * uses it so PCM_INFO never overruns the real count.
	 */
	vsnd.nstream = vin32(&vsnd.devcfg, Vsndcfg_streams);
	print("virtio-snd: jacks %d streams %d chmaps %d\n",
		vin32(&vsnd.devcfg, Vsndcfg_jacks),
		vsnd.nstream,
		vin32(&vsnd.devcfg, Vsndcfg_chmaps));

	/* PCM_INFO: enumerate all streams, find playback + capture */
	if(pcmfindstreams() < 0)
		return -1;
	print("virtio-snd: playback stream id %d\n", vsnd.streamid);
	if(vsnd.capstreamid >= 0)
		print("virtio-snd: capture stream id %d\n", vsnd.capstreamid);
	else
		print("virtio-snd: no capture stream\n");

	/* Playback: SET_PARAMS -> PREPARE (START deferred to first write) */
	if(pcmsetparams(vsnd.streamid, vsnd.formats, vsnd.rates,
		vsnd.chmin, vsnd.chmax, &vsnd.fmt, &vsnd.rate,
		&vsnd.chans, &vsnd.speed, "playback") < 0)
		return -1;
	if(pcmsimple(RPcmPrepare) < 0)
		return -1;

	vsnd.ready = 1;
	print("#A%d: virtio-snd ready (playback, %dHz %dch%s)\n",
		vsnd.adev != nil ? vsnd.adev->ctlrno : 0,
		vsnd.speed, vsnd.chans,
		vsnd.capstreamid >= 0 ? "; capture available" : "");
	return 0;
}

/*
 * Capture PCM lifecycle: SET_PARAMS -> PREPARE -> START for the capture
 * stream.  Run by capproc on a real KSTACK.  Unlike playback, START is
 * issued immediately because the device must be actively recording for
 * rxperiod to receive data.  Returns 0 on success, -1 on failure.
 */
static int
caplifecycle(void)
{
	Sndpcmhdr h;
	u32int t;

	if(vsnd.capstreamid < 0)
		return -1;

	/*
	 * Wait until the device is activated and the playback lifecycle
	 * is done (vsnd.ready or vsnd.failed).  The capture kproc starts
	 * in parallel with drainproc; we need sndactivate to have finished
	 * so the control queue is live.
	 */
	while(!vsnd.ready && !vsnd.failed)
		tsleep(&vsnd.ckr, return0, nil, 50);
	if(vsnd.failed)
		return -1;

	/*
	 * Serialise control-queue access with vsnd.ctll.  The playback
	 * side can issue RPcmStart from drainproc concurrently, and both
	 * paths share vsnd.resp and the single control queue.
	 */
	qlock(&vsnd.ctll);
	if(pcmsetparams(vsnd.capstreamid, vsnd.capformats, vsnd.caprates,
		vsnd.capchmin, vsnd.capchmax, &vsnd.capfmt, &vsnd.caprate,
		&vsnd.capchans, &vsnd.capspeed, "capture") < 0){
		qunlock(&vsnd.ctll);
		return -1;
	}

	/* PREPARE the capture stream */
	memset(&h, 0, sizeof h);
	h.code = RPcmPrepare;
	h.stream_id = vsnd.capstreamid;
	t = ctlcmd(&h, WSndpcmhdr, vsnd.resp, WSndhdr);
	if(t != SOk){
		print("virtio-snd: capture prepare: %s (%#ux)\n",
			sndstatusname(t), t);
		qunlock(&vsnd.ctll);
		return -1;
	}

	/* START the capture stream immediately so the device begins recording */
	memset(&h, 0, sizeof h);
	h.code = RPcmStart;
	h.stream_id = vsnd.capstreamid;
	t = ctlcmd(&h, WSndpcmhdr, vsnd.resp, WSndhdr);
	if(t != SOk){
		print("virtio-snd: capture start: %s (%#ux)\n",
			sndstatusname(t), t);
		qunlock(&vsnd.ctll);
		return -1;
	}
	qunlock(&vsnd.ctll);

	vsnd.capstarted = 1;
	vsnd.capready = 1;
	print("virtio-snd: capture ready (%dHz %dch)\n",
		vsnd.capspeed, vsnd.capchans);
	return 0;
}

/*
 * card probe, called by devaudio.c (audioreset) on the boot stack with
 * up == nil.  Does ONLY pure PCI discovery (sndfind) here -- it never
 * touches the device, so nothing can interrupt on the boot stack.  The
 * ENTIRE device lifecycle (virtio handshake AND PCM setup) is run later
 * by the drain kproc (sndactivate then pcmlifecycle), once the scheduler
 * is up and we are on a real KSTACK with up != nil.  The card is
 * registered as soon as the device is merely PRESENT in PCI -- before it
 * is even activated -- so #A/dev/audio exist; if the kproc's activation
 * or lifecycle later fails, writes simply drop (vsnd.ready stays 0).
 */
static int
audiovzprobe(Audio *adev)
{
	if(vsnd.present){
		/* one virtio-sound device only */
		return -1;
	}
	if(sndfind() < 0)
		return -1;

	vsnd.adev = adev;
	adev->ctlr = &vsnd;
	/*
	 * Provisional speed for /dev/audio until pcmlifecycle() negotiates
	 * the real rate; drainproc() updates adev->speed once SET_PARAMS
	 * has run.  devaudio also re-derives speed from volctl writes.
	 */
	adev->speed = 48000;
	adev->read = audiovzread;
	adev->write = audiovzwrite;
	adev->close = audiovzclose;
	adev->buffered = audiovzbuffered;
	adev->status = audiovzstatus;
	adev->volread = audiovzvolread;
	adev->volwrite = audiovzvolwrite;

	/*
	 * Do NOT create the drain kproc here.  This runs on the boot stack
	 * during chandevreset (audioreset), BEFORE schedinit, with up ==
	 * nil, where creating/readying a kproc is a boot-stack trap source
	 * (see the vz64 PORT NOTES).  Activation is deferred to first use
	 * of /dev/audio (audiovzstart, called from audiovzwrite), which
	 * runs in an ordinary process context after schedinit with up !=
	 * nil.  The card is registered as merely PRESENT now (so #A /
	 * /dev/audio exist); the virtio handshake + PCM lifecycle happen
	 * lazily in the drain kproc on first write.
	 */
	print("#A%d: virtio-snd present (activation deferred to first use)\n",
		adev->ctlrno);
	return 0;
}

void
audiovzlink(void)
{
	addaudiocard("virtio-snd", audiovzprobe);
}
