/*
 * devvsock.c -- virtio 1.x socket (vsock) driver for Apple
 * Virtualization.framework guests.
 *
 * Presents /net/vsock/{clone,N/ctl,N/data,N/status}.  Guest
 * userspace dials host services with
 *
 *	dial("vsock!2!port", nil, nil, nil)
 *
 * which works unmodified because the clone contract matches what
 * dial() already expects (see NOTES / devsock.md section 3.6):
 * open clone, read it back for the decimal conversation number,
 * write "connect CID!port" to that same descriptor, then open
 * N/data.
 *
 * Version 0: stream sockets only, guest-initiated connections
 * only (no guest-side listening -- every inbound REQUEST gets
 * RST).  See devsock.md for the full design.
 *
 * Modelled on ethervirtio10.c / uartvz.c / screen.c for the
 * virtio 1.0 ring plumbing (each of those reimplements its own
 * copy of the queue setup rather than sharing a library; this
 * driver follows the same local-copy convention).  The clone
 * directory is NOT built on the IP Proto/Conv framework (devip.c)
 * -- that framework carries IP-address/routing assumptions that
 * do not apply here, so this file implements its own small fixed
 * conversation table directly, per devsock.md section 3.7.
 *
 * Locking (devsock.md 3.9):
 *   convtab (Lock, taken with ilock) -- allocation/freeing, tuple
 *     lookup, generation changes.  Never held across blocking I/O.
 *   Vsconv (QLock) -- state transitions, credit counters, shutdown
 *     flags.  Blocking waits release it and recheck the predicate
 *     after wakeup (the sleep()/wakeup() idiom used throughout this
 *     port; see screen.c's ctlwait for the model).
 *   vsock.il (Lock) -- brief tx/rx/ev vring pointer manipulation,
 *     shared with interrupt().
 *   vsock.txl (QLock) -- serialises tx submitters (many writers,
 *     one shared tx virtqueue), analogous to uartvz.c's vcon.txl
 *     and screen.c's vgpu.ctll.
 * Lock order: convtab, then Vsconv, then vsock.il/txl; never the
 * reverse.  The generation counter in qid.vers is the "old slot"
 * guard: every operation re-validates conv->gen against the qid it
 * was handed before touching state, so a channel walked under a
 * stale generation gets Ehungup instead of addressing a later
 * occupant of the same table slot.
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

enum {
	Vsockvid	= 0x1AF4,
	Vsockdid	= 0x1053,	/* virtio device id 19 (socket) -> modern PCI did 0x1040+19 */

	HostCid		= 2,		/* well-known host CID */

	NCONV		= 64,		/* fixed conversation table size */
	/*
	 * Fixed per-conversation logical receive window: the buf_alloc
	 * we advertise to the peer.  Was 64*1024; bumped to 256*1024
	 * after vsbulk self-RSTs were traced (vsock-vs-tcp-notes.md
	 * section 8).  The traces show the HOST DOES NOT STRICTLY
	 * HONOR the advertised window: at the failure point it had
	 * sent past (our last reported fwd_cnt + buf_alloc), which a
	 * compliant peer never does -- fc is monotonic and reported
	 * after the fact, so a peer's stale view can only make it
	 * MORE conservative, and ../port/qio.c's one-overshoot
	 * admission (qpass() accepts the block that crosses the limit,
	 * rejecting only a FURTHER one) means every packet a compliant
	 * peer may legally send gets accepted, modulo a ~100B/block
	 * BALLOC accounting gap far smaller than any observed packet.
	 * The host instead appears to stream against its own ~256KB
	 * internal buffering (it advertises buf_alloc 262144 for its
	 * own receive side, and its post-RST pipelined backlog measured
	 * ~240KB), so matching that size makes its worst observed
	 * burst fit.  cv->rq is additionally opened with slack beyond
	 * this (see vsockopen) so moderate overruns are absorbed
	 * rather than RST.
	 */
	Credit		= 256*1024,

	/*
	 * Payload-buffer counts (upper bounds on Vqueue.nbuf), NOT
	 * ring sizes: rings are always allocated at the device's
	 * reported queue size (see the note in devvsocklink), and
	 * these cap how many buffers we provision / keep outstanding.
	 * Must be powers of two.
	 */
	Nrxdesc		= 32,
	Ntxdesc		= 32,
	Nevdesc		= 8,

	Rxbufsz		= Credit + 128,		/* header + up to one full window */
	/*
	 * Outbound packetization.  Every data write is chunked into
	 * RW packets of at most vstxchunk payload bytes, and each
	 * packet costs one avail-ring submission plus one MMIO kick
	 * (vout16 on the notify register) -- a trap out to the host
	 * VMM.  At the measured ~330MB/s bulk rate with 4096-byte
	 * chunks that is one kick every ~12us, which is suspiciously
	 * close to plausible per-trap cost, i.e. the kick rate (not
	 * memcpy, not the wire) may be what caps throughput.  To
	 * measure that without a rebuild per value, the chunk size is
	 * a runtime tunable (vstxchunk, "txchunk N" on /net/vsock/ctl,
	 * default Txchunk) clamped to Txchunkmax, the size the tx
	 * payload buffers are actually allocated at during boot.
	 * Txchunkmax at 64K costs 32 x (64K+128) ~= 2.1MB of boot-time
	 * allocation (vs ~130KB before); acceptable next to the rx
	 * buffers' ~8.4MB.
	 *
	 * The sweep this comment anticipated has been run (devsock.md
	 * section 4, "txchunk sweep"): 4096/8192/16384/32768/65536,
	 * three vsbulk runs each.  Throughput rose monotonically with
	 * chunk size through 32768 (mean ~529 -> 639 -> 690 -> 719
	 * MB/s) but 65536 -- one full Txbufsz per packet, halving the
	 * number of packets in flight for a given byte count relative
	 * to 32768 against the fixed Ntxdesc buffer pool -- went
	 * unstable (513-710 MB/s, ~6x the spread) instead of
	 * continuing to improve.  32768 wins on both mean and
	 * stability and is now the compiled-in default; the knob
	 * remains live for regression checks, not as a tunable anyone
	 * is expected to touch in normal use.
	 */
	Txchunk		= 32768,		/* default payload bytes per RW packet (see vstxchunk); devsock.md section 4 */
	Txchunkmax	= 64*1024,		/* hard cap; tx buffers are allocated this big */
	Txbufsz		= Txchunkmax + 128,
	Evbufsz		= 8,			/* struct virtio_vsock_event { le32 id; }, rounded up */

	Vshdrsz		= 44,		/* wire size of virtio_vsock_hdr; NEVER sizeof(Vshdr) -- see below */

	StypeStream	= 1,

	OpInvalid	= 0,
	OpRequest	= 1,
	OpResponse	= 2,
	OpRst		= 3,
	OpShutdown	= 4,
	OpRw		= 5,
	OpCreditUpdate	= 6,
	OpCreditRequest	= 7,

	ShutRecv	= 1,
	ShutSend	= 2,

	Sclosed		= 0,
	Sconnecting,
	Sestablished,
	Sclosing,
	Sreset,

	Qtopdir		= 0,	/* device root; single child "vsock" (Qprotodir) */
	Qprotodir,		/* the "vsock" directory: clone + numbered conversations */
	Qclone,
	Qgctl,			/* /net/vsock/ctl -- global control: "debug on|off", "txchunk N" */
	Qconvdir,
	Qctl,
	Qdata,
	Qstatus,

	Qtypebits	= 5,
	Qtypemask	= (1<<Qtypebits)-1,

	Vrxq		= 0,
	Vtxq		= 1,
	Vevq		= 2,

	/* descriptor / vring flags -- same as every other virtio10 driver here */
	Dnext		= 1,
	Dwrite		= 2,
	Rnointerrupt	= 1,

	VringSize	= 4,
	VdescSize	= 16,
	VusedSize	= 8,
};

#define QID(conv, type)	(((ulong)(conv)<<Qtypebits)|(type))
#define QTYPE(path)	((int)((path)&Qtypemask))
#define QCONV(path)	((int)((ulong)(path)>>Qtypebits))

/*
 * virtio_vsock_hdr, OASIS virtio 1.3 section 5.10.6.  The struct's
 * natural (ABI) size is tail-padded to 48 bytes on arm64 because of
 * the leading u64int members; the WIRE size is 44.  Every place
 * that sets a descriptor length for this header uses the Vshdrsz
 * constant, never sizeof(Vshdr) -- the 4 padding bytes are scratch,
 * not part of the format, and must never be transmitted or assumed
 * to precede the payload.
 */
typedef struct Vshdr Vshdr;
struct Vshdr
{
	u64int	srccid;
	u64int	dstcid;
	u32int	srcport;
	u32int	dstport;
	u32int	len;
	u16int	type;
	u16int	op;
	u32int	flags;
	u32int	bufalloc;
	u32int	fwdcnt;
};

typedef struct Vqueue Vqueue;
struct Vqueue
{
	Rendez;		/* woken by interrupt() */

	uint	qsize;	/* ring size: the DEVICE's reported queue size, never reduced */
	uint	qmask;
	uint	nbuf;	/* payload buffers we provision (<= qsize, power of two) */

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

typedef struct Vsconv Vsconv;
struct Vsconv
{
	QLock;
	Rendez	connr;		/* connect() waits here for RESPONSE/RST/hangup */
	Rendez	txr;		/* blocked writers wait here for credit */

	int	state;
	ulong	gen;		/* generation; bumped whenever the slot is freed */
	int	nref;		/* open ctl+data+status references */
	char	*owner;

	uvlong	rcid;		/* remote (peer) cid */
	u32int	rport;		/* remote (peer) port */
	u32int	lport;		/* our ephemeral local port */

	u32int	ourfwdcnt;	/* cumulative bytes delivered to the local reader */
	u32int	announced;	/* ourfwdcnt as of the last explicit CREDIT_UPDATE we sent */
	u32int	ourtxcnt;	/* cumulative payload bytes we have sent (wrapping tx_cnt) */
	u32int	peerbufalloc;	/* last-known peer buf_alloc */
	u32int	peerfwdcnt;	/* last-known peer fwd_cnt */
	int	creditreqsent;	/* a CREDIT_REQUEST is outstanding; suppress duplicates */

	int	shutwr;		/* we have sent SHUTDOWN with F_SEND */
	int	gotpeersend;	/* peer sent SHUTDOWN with F_SEND (their send side is closed) */
	int	gotpeerrecv;	/* peer sent SHUTDOWN with F_RECEIVE (informational) */

	uvlong	rxbytes, txbytes;	/* decimal traffic counters for status; NOT the wrapping credit counters */

	Queue	*rq;		/* payload bytes received from the peer, awaiting guest read */
};

static struct
{
	Lock;
	int	nused;		/* high-water mark of slots ever allocated; only
				 * directories 0..nused-1 are listed/walkable,
				 * matching devip.c's Proto.ac convention: /net/tcp
				 * shows no numbered directories until the first
				 * clone, and they persist (for reuse) afterwards. */
	Vsconv	conv[NCONV];
} convtab;

static struct
{
	int	ready;
	int	intr;

	Pcidev	*pci;
	Vio	cfg;
	Vio	isr;
	Vio	devcfg;		/* device-specific config: struct { le64 guest_cid; } */
	u32int	notifyoffmult;

	uvlong	guestcid;

	Vqueue	rx;
	Vqueue	tx;
	Vqueue	ev;

	uchar	*rxbuf[Nrxdesc];
	uchar	*txbuf[Ntxdesc];
	uchar	*evbuf[Nevdesc];

	QLock	txl;		/* serialises tx submitters */
	Lock	il;		/* brief vring pointer manipulation; shared with interrupt() */

	ulong	lportnext;
} vsock;

static char Ereset[] = "vsock: connection reset";
static char Erefused[] = "vsock: connection refused";

/*
 * Diagnostic tracing: one console line per packet sent and received
 * (op, tuple, len, credit fields).  Cheap enough for connect/hang
 * debugging but a print per RW packet dominates round-trip time, so
 * it must be OFF before taking any latency/throughput numbers.
 *
 * Toggled live via /net/vsock/ctl (Qgctl below), NOT a recompile:
 *	echo debug on >/net/vsock/ctl
 *	echo debug off >/net/vsock/ctl
 *	cat /net/vsock/ctl		# -> "debug 0" or "debug 1"
 * This is a separate file from each conversation's own N/ctl (which
 * only accepts connect/hangup) -- there is exactly one debug switch
 * for the whole device, not one per conversation.
 */
static int vsdebug = 0;

/*
 * Runtime outbound chunk size -- see the Txchunk/Txchunkmax comment
 * in the enum above.  Set via "txchunk N" on /net/vsock/ctl,
 * validated there to 512..Txchunkmax; read back via cat.  This is a
 * measurement knob in the same spirit as vsdebug: once the sweep
 * says what the right value is, that value should become the
 * compiled-in Txchunk default, and the knob stays for future
 * regression checks rather than as a thing anyone is expected to
 * tune in normal use.
 */
static int vstxchunk = Txchunk;

static void vssendraw(uvlong, u32int, uvlong, u32int, u16int, u32int, u32int, u32int, void*, u32int);
static void vssendctl(Vsconv*, u16int, u32int);
static void vsrst(uvlong, u32int, uvlong, u32int);
static void vsockhangup(Vsconv*);

/*
 * queue setup -- same shape as uartvz.c / ethervirtio10.c / screen.c
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
	/* interrupts stay enabled: we sleep on completions, never poll with them off */
	return 0;
}

static int
matchvirtiocfgcap(Pcidev *p, int cap, int off, int typ)
{
	int bar;

	if(cap != 9 || pcicfgr8(p, off+3) != typ)
		return 1;
	bar = pcicfgr8(p, off+4);
	if(bar < 0 || bar >= nelem(p->mem) || p->mem[bar].size == 0)
		return 1;
	return 0;
}

static int
virtiocap(Pcidev *p, int typ)
{
	return pcienumcaps(p, matchvirtiocfgcap, typ);
}

/*
 * tx: shared virtqueue, many submitters serialised by vsock.txl.
 * Buffers (and their same-numbered descriptors) are used strictly
 * in avail order, so one is free for reuse once (avail->idx -
 * lastused) < nbuf; txhasroom() retires completed entries
 * opportunistically before reporting room.  The bound is nbuf,
 * not qsize: the ring may be larger than the buffer set we
 * provision (see the queue-size note in devvsocklink).
 */
static int
txhasroom(void *v)
{
	Vqueue *q = v;
	int room;

	ilock(&vsock.il);
	while(q->used->idx != q->lastused)
		q->lastused++;
	room = (u16int)(q->avail->idx - q->lastused) < q->nbuf;
	iunlock(&vsock.il);
	return room;
}

static void
txwaitroom(void)
{
	while(!txhasroom(&vsock.tx)){
		if(up != nil && islo())
			tsleep(&vsock.tx, txhasroom, &vsock.tx, 1000);
		else
			coherence();
	}
}

static void
rxpost(int i)
{
	Vqueue *q = &vsock.rx;

	q->desc[i].addr = PADDR(vsock.rxbuf[i]);
	q->desc[i].len = Rxbufsz;
	q->desc[i].flags = Dwrite;
	q->desc[i].next = 0;
	q->availent[q->avail->idx & q->qmask] = i;
	coherence();
	q->avail->idx++;
}

static void
evpost(int i)
{
	Vqueue *q = &vsock.ev;

	q->desc[i].addr = PADDR(vsock.evbuf[i]);
	q->desc[i].len = Evbufsz;
	q->desc[i].flags = Dwrite;
	q->desc[i].next = 0;
	q->availent[q->avail->idx & q->qmask] = i;
	coherence();
	q->avail->idx++;
}

static void
interrupt(Ureg*, void*)
{
	if(vin8(&vsock.isr, 0) & 1){
		wakeup(&vsock.rx);	/* rxproc rechecks both rx and ev queues */
		wakeup(&vsock.tx);
	}
}

/*
 * conversation table: allocation, tuple lookup, ephemeral ports.
 * Always called with convtab ilocked, except vsockfind() which
 * callers must also ilock convtab around.
 */
static Vsconv*
vsockfind(uvlong rcid, u32int rport, u32int lport)
{
	Vsconv *cv;
	int i;

	for(i = 0; i < NCONV; i++){
		cv = &convtab.conv[i];
		if(cv->state == Sclosed)
			continue;
		if(cv->lport == lport && cv->rport == rport && cv->rcid == rcid)
			return cv;
	}
	return nil;
}

static u32int
vsockallocport(void)
{
	u32int port;
	int i, tries;

	for(tries = 0; tries < 65536; tries++){
		port = 1024 + (vsock.lportnext++ % 64512);
		for(i = 0; i < NCONV; i++)
			if(convtab.conv[i].state != Sclosed && convtab.conv[i].lport == port)
				break;
		if(i == NCONV)
			return port;
	}
	return 0;
}

/*
 * wrapping 32-bit peer credit arithmetic (devsock.md 3.3):
 *	peer_free = peer_buf_alloc - (tx_cnt - peer_fwd_cnt)
 */
static u32int
peerfree(Vsconv *cv)
{
	u32int inflight;

	inflight = cv->ourtxcnt - cv->peerfwdcnt;
	if(inflight > cv->peerbufalloc)
		return 0;
	return cv->peerbufalloc - inflight;
}

/*
 * low-level packet senders.  vssendraw serialises on the shared tx
 * queue (vsock.txl) and waits for a free descriptor (txwaitroom);
 * it never blocks on peer credit -- callers that send OpRw are
 * responsible for having already checked peerfree().
 */
static void
vssendraw(uvlong dcid, u32int dport, uvlong scid, u32int sport,
	u16int op, u32int flags, u32int bufalloc, u32int fwdcnt,
	void *payload, u32int len)
{
	Vshdr *h;
	uchar *buf;
	int slot;

	if(!vsock.ready)
		return;
	if(len > Txchunkmax)
		len = Txchunkmax;	/* buffer-capacity clamp; callers chunk to vstxchunk, which is <= this */

	if(vsdebug)
		print("vsock: tx op %ud %llud!%ud -> %llud!%ud len %ud ba %ud fc %ud\n",
			op, scid, sport, dcid, dport, len, bufalloc, fwdcnt);

	qlock(&vsock.txl);
	txwaitroom();

	ilock(&vsock.il);
	/*
	 * Buffer/descriptor index cycles over our provisioned set
	 * (nbuf, a power of two); the avail-ring slot below uses the
	 * device's true ring mask (qmask).  These differ when the
	 * device's ring is larger than Ntxdesc.
	 */
	slot = vsock.tx.avail->idx & (vsock.tx.nbuf - 1);
	buf = vsock.txbuf[slot];
	h = (Vshdr*)buf;
	h->srccid = scid;
	h->dstcid = dcid;
	h->srcport = sport;
	h->dstport = dport;
	h->len = len;
	h->type = StypeStream;
	h->op = op;
	h->flags = flags;
	h->bufalloc = bufalloc;
	h->fwdcnt = fwdcnt;
	if(len > 0)
		memmove(buf+Vshdrsz, payload, len);

	vsock.tx.desc[slot].addr = PADDR(buf);
	vsock.tx.desc[slot].len = Vshdrsz + len;
	vsock.tx.desc[slot].flags = 0;
	vsock.tx.desc[slot].next = 0;
	vsock.tx.availent[vsock.tx.avail->idx & vsock.tx.qmask] = slot;
	coherence();
	vsock.tx.avail->idx++;
	coherence();
	vout16(&vsock.tx.notify, 0, Vtxq);
	iunlock(&vsock.il);

	qunlock(&vsock.txl);
}

static void
vssendctl(Vsconv *cv, u16int op, u32int flags)
{
	vssendraw(cv->rcid, cv->rport, vsock.guestcid, cv->lport,
		op, flags, Credit, cv->ourfwdcnt, nil, 0);
}

static void
vssenddata(Vsconv *cv, void *payload, u32int len)
{
	vssendraw(cv->rcid, cv->rport, vsock.guestcid, cv->lport,
		OpRw, 0, Credit, cv->ourfwdcnt, payload, len);
	qlock(cv);
	cv->ourtxcnt += len;
	cv->txbytes += len;
	qunlock(cv);
}

/* reply RST to an arbitrary (possibly unknown) tuple; ourcid/ourport are OUR fields */
static void
vsrst(uvlong rcid, u32int rport, uvlong ourcid, u32int ourport)
{
	vssendraw(rcid, rport, ourcid, ourport, OpRst, 0, 0, 0, nil, 0);
}

/*
 * receive-side processing, run from a dedicated kproc (rxproc)
 * rather than from interrupt context: it may call qproduce/qhangup
 * and take cv's QLock, none of which is safe at interrupt level,
 * and a single slow conversation must not stall the shared rx ring
 * -- see the "malformed input must never panic" and "one slow
 * conversation must not stall others" requirements in devsock.md.
 */
static void
vsockrx(uchar *buf, u32int n)
{
	Vshdr *h;
	uchar *payload;
	u32int paylen;
	Vsconv *cv;

	if(n < Vshdrsz)
		return;		/* too short even for a header: drop, never panic */
	h = (Vshdr*)buf;
	payload = buf + Vshdrsz;
	paylen = n - Vshdrsz;
	if(h->len < paylen)
		paylen = h->len;

	if(vsdebug)
		print("vsock: rx op %ud %llud!%ud -> %llud!%ud len %ud ba %ud fc %ud\n",
			h->op, h->srccid, h->srcport, h->dstcid, h->dstport,
			h->len, h->bufalloc, h->fwdcnt);

	if(h->dstcid != vsock.guestcid || h->type != StypeStream){
		vsrst(h->srccid, h->srcport, h->dstcid, h->dstport);
		return;
	}

	ilock(&convtab);
	cv = vsockfind(h->srccid, h->srcport, h->dstport);
	iunlock(&convtab);

	if(cv == nil){
		if(vsdebug)
			print("vsock: rx no conversation for %llud!%ud -> port %ud\n",
				h->srccid, h->srcport, h->dstport);
		if(h->op != OpRst)
			vsrst(h->srccid, h->srcport, h->dstcid, h->dstport);
		return;
	}

	qlock(cv);
	switch(h->op){
	case OpResponse:
		if(cv->state == Sconnecting){
			cv->peerbufalloc = h->bufalloc;
			cv->peerfwdcnt = h->fwdcnt;
			cv->state = Sestablished;
			wakeup(&cv->connr);
		}
		break;

	case OpRw:
		if(cv->state == Sestablished || cv->state == Sclosing){
			cv->peerbufalloc = h->bufalloc;
			cv->peerfwdcnt = h->fwdcnt;
			if(paylen > 0){
				if(cv->rq == nil || qproduce(cv->rq, payload, paylen) != paylen){
					/*
					 * Either the queue is already closed, or
					 * the peer overran the credit it was
					 * given: cv->rq's admission limit exceeds
					 * the advertised buf_alloc (see
					 * vsockopen), and qio.c's qpass() always
					 * admits the block that crosses the
					 * limit, rejecting only a FURTHER one --
					 * so a peer honoring its window can never
					 * get here.  (The Apple host has been
					 * observed overrunning exactly this way;
					 * see the Credit comment.)  Reset rather
					 * than silently truncate or block the
					 * shared rx kproc.
					 */
					qunlock(cv);
					vssendctl(cv, OpRst, 0);
					qlock(cv);
					cv->state = Sreset;
					if(cv->rq != nil)
						qhangup(cv->rq, Ereset);
					wakeup(&cv->connr);
					wakeup(&cv->txr);
				}else{
					cv->rxbytes += paylen;
				}
			}
			wakeup(&cv->txr);
		}
		break;

	case OpCreditUpdate:
		if(cv->state == Sestablished || cv->state == Sclosing){
			cv->peerbufalloc = h->bufalloc;
			cv->peerfwdcnt = h->fwdcnt;
			cv->creditreqsent = 0;
			wakeup(&cv->txr);
		}
		break;

	case OpCreditRequest:
		if(cv->state == Sestablished || cv->state == Sclosing){
			qunlock(cv);
			vssendctl(cv, OpCreditUpdate, 0);
			qlock(cv);
		}
		break;

	case OpShutdown:
		if(h->flags & ShutSend){
			cv->gotpeersend = 1;
			if(cv->rq != nil)
				qhangup(cv->rq, nil);	/* clean EOF once drained */
		}
		if(h->flags & ShutRecv){
			cv->gotpeerrecv = 1;
			wakeup(&cv->txr);
		}
		if(cv->shutwr && cv->gotpeersend && cv->state != Sclosed){
			qunlock(cv);
			vssendctl(cv, OpRst, 0);
			qlock(cv);
			cv->state = Sclosed;
			wakeup(&cv->connr);
			wakeup(&cv->txr);
		}
		break;

	case OpRst:
		cv->state = Sreset;
		if(cv->rq != nil)
			qhangup(cv->rq, Ereset);
		wakeup(&cv->connr);
		wakeup(&cv->txr);
		break;

	case OpRequest:
		/* guest-side listening is version 1; always refuse */
		qunlock(cv);
		vsrst(h->srccid, h->srcport, h->dstcid, h->dstport);
		return;

	default:
		break;	/* unknown op: ignore, never panic */
	}
	qunlock(cv);
}

static void
vsockevent(uchar *buf, u32int n)
{
	u32int id;
	Vsconv *cv;
	int i;

	if(n < 4)
		return;
	memmove(&id, buf, sizeof id);
	if(id != 0)	/* only VIRTIO_VSOCK_EVENT_TRANSPORT_RESET (0) is defined */
		return;

	for(i = 0; i < NCONV; i++){
		cv = &convtab.conv[i];
		qlock(cv);
		if(cv->state != Sclosed){
			cv->state = Sreset;
			if(cv->rq != nil)
				qhangup(cv->rq, Ereset);
			wakeup(&cv->connr);
			wakeup(&cv->txr);
		}
		qunlock(cv);
	}

	/* a transport reset may present a different CID; re-read it */
	if(vsock.devcfg.type == Vio_mem || vsock.devcfg.type == Vio_port)
		vsock.guestcid = vin64(&vsock.devcfg, 0);
}

static int
rxhaswork(void*)
{
	return vsock.rx.used->idx != vsock.rx.lastused
		|| vsock.ev.used->idx != vsock.ev.lastused;
}

static void
rxproc(void*)
{
	Vqueue *q;
	u16int id;
	u32int len;

	/*
	 * Catch any error() raised under this kproc (e.g. from
	 * tsleep on a posted note) rather than letting it escape
	 * with no error label -- same guard as audiovz.c's
	 * drainproc/capproc.
	 */
	while(waserror())
		;

	for(;;){
		if(!rxhaswork(nil))
			tsleep(&vsock.rx, rxhaswork, nil, 1000);

		q = &vsock.rx;
		while(q->lastused != q->used->idx){
			id = q->usedent[q->lastused & q->qmask].id;
			len = q->usedent[q->lastused & q->qmask].len;
			q->lastused++;
			vsockrx(vsock.rxbuf[id], len);
			rxpost(id);
		}
		coherence();
		vout16(&q->notify, 0, Vrxq);

		q = &vsock.ev;
		while(q->lastused != q->used->idx){
			id = q->usedent[q->lastused & q->qmask].id;
			len = q->usedent[q->lastused & q->qmask].len;
			q->lastused++;
			vsockevent(vsock.evbuf[id], len);
			evpost(id);
		}
		coherence();
		vout16(&q->notify, 0, Vevq);
	}
}

/*
 * writer side: dial()'s data-file writes.  Blocks on the
 * conversation's own credit and on tx-ring room; a writer that has
 * exhausted peer credit is woken by credit arrival, shutdown,
 * reset, or (indirectly, via wakeup(&cv->txr) in vsockevent) a
 * transport reset.
 */
static int
txcredit(void *v)
{
	Vsconv *cv = v;

	return peerfree(cv) != 0 || (cv->state != Sestablished && cv->state != Sclosing);
}

static long
vsockdatawrite(Vsconv *cv, void *va, long n)
{
	uchar *p = va;
	long tot = 0;
	u32int chunk, free, tc;

	while(n > 0){
		qlock(cv);
		if(cv->state != Sestablished && cv->state != Sclosing){
			qunlock(cv);
			error(Ereset);
		}
		while((free = peerfree(cv)) == 0){
			if(cv->state != Sestablished && cv->state != Sclosing){
				qunlock(cv);
				error(Ereset);
			}
			if(!cv->creditreqsent){
				cv->creditreqsent = 1;
				qunlock(cv);
				vssendctl(cv, OpCreditRequest, 0);
				qlock(cv);
				continue;
			}
			qunlock(cv);
			if(waserror())
				nexterror();
			sleep(&cv->txr, txcredit, cv);
			poperror();
			qlock(cv);
		}
		qunlock(cv);

		/*
		 * Snapshot vstxchunk once per chunk so the length we
		 * account in vssenddata is the length vssendraw
		 * actually sends, even if the ctl knob changes under
		 * a write in progress.
		 */
		tc = vstxchunk;
		chunk = n < tc ? n : tc;
		if(chunk > free)
			chunk = free;

		vssenddata(cv, p, chunk);

		p += chunk;
		n -= chunk;
		tot += chunk;
	}
	return tot;
}

static int
connestab(void *v)
{
	Vsconv *cv = v;
	return cv->state != Sconnecting;
}

static char*
vsockconnect(Vsconv *cv, char *arg)
{
	char *p;
	uvlong cid;
	u32int port;

	/*
	 * Without a probed device vssendraw() is a silent no-op, so a
	 * connect would sleep forever waiting for a RESPONSE that can
	 * never come.  Fail the dial immediately instead (devsock.md:
	 * absence of the optional device must not wedge anything).
	 */
	if(!vsock.ready)
		return "vsock device not present";

	p = strchr(arg, '!');
	if(p == nil)
		return "bad vsock address";
	*p++ = 0;
	cid = strtoull(arg, nil, 0);
	port = strtoul(p, nil, 0);

	qlock(cv);
	if(cv->state != Sclosed){
		qunlock(cv);
		return "already connected";
	}
	cv->rcid = cid;
	cv->rport = port;
	cv->state = Sconnecting;
	cv->peerbufalloc = 0;
	cv->peerfwdcnt = 0;
	cv->ourtxcnt = 0;
	cv->ourfwdcnt = 0;
	cv->announced = 0;
	cv->creditreqsent = 0;
	cv->gotpeersend = 0;
	cv->gotpeerrecv = 0;
	cv->shutwr = 0;
	qunlock(cv);

	vssendctl(cv, OpRequest, 0);

	qlock(cv);
	while(cv->state == Sconnecting){
		qunlock(cv);
		if(waserror())
			nexterror();
		sleep(&cv->connr, connestab, cv);
		poperror();
		qlock(cv);
	}
	if(cv->state != Sestablished){
		qunlock(cv);
		return Erefused;
	}
	qunlock(cv);
	return nil;
}

/* idempotent full local shutdown, driven by ctl "hangup" and by final close */
static void
vsockhangup(Vsconv *cv)
{
	int state;

	qlock(cv);
	state = cv->state;
	qunlock(cv);

	switch(state){
	case Sclosed:
	case Sreset:
		return;

	case Sconnecting:
		vssendctl(cv, OpRst, 0);
		qlock(cv);
		cv->state = Sclosed;
		wakeup(&cv->connr);
		qunlock(cv);
		return;

	case Sestablished:
	case Sclosing:
		qlock(cv);
		if(cv->shutwr){
			qunlock(cv);
			return;
		}
		cv->shutwr = 1;
		qunlock(cv);

		vssendctl(cv, OpShutdown, ShutSend|ShutRecv);

		qlock(cv);
		if(cv->state == Sestablished)
			cv->state = Sclosing;
		if(cv->gotpeersend && cv->state != Sclosed){
			qunlock(cv);
			vssendctl(cv, OpRst, 0);
			qlock(cv);
			cv->state = Sclosed;
			wakeup(&cv->txr);
		}
		qunlock(cv);
		return;
	}
}

static long
vsockctlwrite(Vsconv *cv, void *a, long n)
{
	Cmdbuf *cb;
	char *err;

	cb = parsecmd(a, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	if(cb->nf < 1)
		error("short control request");
	if(strcmp(cb->f[0], "connect") == 0){
		if(cb->nf < 2)
			error("connect needs CID!port");
		err = vsockconnect(cv, cb->f[1]);
		if(err != nil)
			error(err);
	}else if(strcmp(cb->f[0], "hangup") == 0){
		vsockhangup(cv);
	}else
		error("unknown control request");
	free(cb);
	poperror();
	return n;
}

/*
 * Global control file (/net/vsock/ctl), distinct from each
 * conversation's own N/ctl.  Two knobs, both global to the device
 * (never per-conversation) and both safe to change on a live
 * system:
 *	debug on|off (or 1|0)	packet-trace prints (vsdebug)
 *	txchunk N		outbound packetization size in bytes,
 *				512..Txchunkmax (vstxchunk)
 * Deliberately NOT exposed here: anything whose storage is sized at
 * boot (Credit / rx buffer sizes, ring/buffer counts) -- a knob that
 * silently can't take effect, or worse takes effect unsafely, is a
 * trap, not a tunable.  Those change by recompile only.
 *
 * Not tied to any conversation's generation counter -- see the
 * special-case in vsockread/vsockwrite that dispatches here before
 * the per-conversation gen check, since this file has no associated
 * Vsconv and must keep working regardless of what has happened to
 * conversation slot 0.
 */
static long
vsockgctlwrite(void *a, long n)
{
	Cmdbuf *cb;
	int v;

	cb = parsecmd(a, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	if(cb->nf < 1)
		error("short control request");
	if(strcmp(cb->f[0], "debug") == 0){
		if(cb->nf < 2)
			error("debug needs on/off (or 1/0)");
		if(strcmp(cb->f[1], "on") == 0 || strcmp(cb->f[1], "1") == 0)
			vsdebug = 1;
		else if(strcmp(cb->f[1], "off") == 0 || strcmp(cb->f[1], "0") == 0)
			vsdebug = 0;
		else
			error("debug needs on/off (or 1/0)");
	}else if(strcmp(cb->f[0], "txchunk") == 0){
		if(cb->nf < 2)
			error("txchunk needs a byte count (512..65536)");
		v = strtol(cb->f[1], nil, 0);
		if(v < 512 || v > Txchunkmax)
			error("txchunk out of range (512..65536)");
		vstxchunk = v;
	}else
		error("unknown control request");
	free(cb);
	poperror();
	return n;
}

static long
vsockgctlread(void *a, long n, vlong off)
{
	char buf[64];

	snprint(buf, sizeof buf, "debug %d\ntxchunk %d\n", vsdebug, vstxchunk);
	return readstr(off, a, n, buf);
}

static long
vsockstatusread(Vsconv *cv, void *a, long n, vlong off)
{
	char buf[256];
	char *state;

	switch(cv->state){
	case Sclosed:		state = "Closed"; break;
	case Sconnecting:	state = "Connecting"; break;
	case Sestablished:	state = "Established"; break;
	case Sclosing:		state = "Closing"; break;
	case Sreset:		state = "Reset"; break;
	default:		state = "?"; break;
	}
	snprint(buf, sizeof buf, "%s %llud!%ud %llud!%ud %llud %llud\n",
		state, vsock.guestcid, cv->lport, cv->rcid, cv->rport,
		cv->rxbytes, cv->txbytes);
	return readstr(off, a, n, buf);
}

/*
 * teardown on last close.  Known limitation: the gen check a caller
 * performs before calling this is not atomic with the nref decrement
 * inside it, so a chan holding a very stale generation could in
 * theory race a slot's reuse; devip.c's Conv close path carries the
 * same class of race (it has no generation check at all) and this
 * table is small enough that the exposure is accepted here too.
 */
static void
vsockconvclose(Vsconv *cv)
{
	Queue *rq;

	qlock(cv);
	if(--cv->nref > 0){
		qunlock(cv);
		return;
	}
	qunlock(cv);

	vsockhangup(cv);

	/*
	 * cv's own QLock and convtab's ilock are taken one at a time
	 * here, never nested: qlock(cv) can block/sched(), which is
	 * illegal while convtab's spinlock is held (caught live by
	 * qlock()/eqlock()'s own m->ilockdepth check -- this exact
	 * nesting used to be here and was firing "qlock: ...
	 * ilockdepth 1" on the console during vsbulk teardown). See
	 * vsockopen()'s Qclone case for the same pattern already in
	 * use ("qopen can block/allocate; do it outside convtab's
	 * spinlock").
	 *
	 * This narrows, but does not fully close, the same class of
	 * race already accepted in the comment above this function:
	 * cv->state becomes Sclosed here slightly before cv->gen is
	 * bumped below, so in principle a concurrent vsockopen(clone)
	 * could observe state==Sclosed under its own ilock(&convtab)
	 * and start reusing this slot before gen++ has happened.
	 * Accepted for the same reason: NCONV is small, this is not a
	 * security boundary, and the alternative (nesting the locks)
	 * is an outright locking-order bug, not just a narrow race.
	 */
	qlock(cv);
	cv->state = Sclosed;
	rq = cv->rq;
	cv->rq = nil;
	qunlock(cv);

	ilock(&convtab);
	cv->gen++;
	free(cv->owner);
	cv->owner = nil;
	iunlock(&convtab);

	if(rq != nil)
		qfree(rq);
}

/*
 * Dev interface
 */
static Chan*
vsockattach(char *spec)
{
	return devattach('V', spec);
}

static int
vsockgen(Chan *c, char*, Dirtab*, int, int s, Dir *dp)
{
	Qid q;
	Vsconv *cv;
	int i;

	switch(QTYPE(c->qid.path)){
	default:
		return -1;

	/*
	 * Device root.  Its only child is the "vsock" directory
	 * (Qprotodir).  This extra level exists so '#V' can be
	 * unioned straight into /net the same way '#l'/'#I'/'#a'
	 * already are (bind -a '#V' /net) and land the clone/N
	 * hierarchy at /net/vsock -- exactly the way tcp/udp/il
	 * appear as named subdirectories of '#I' -- rather than
	 * dumping clone/0/1/... directly at /net's top level.
	 */
	case Qtopdir:
		if(s == DEVDOTDOT){
			mkqid(&q, QID(0, Qtopdir), 0, QTDIR);
			devdir(c, q, "vsock", 0, eve, DMDIR|0555, dp);
			return 1;
		}
		if(s != 0)
			return -1;
		mkqid(&q, QID(0, Qprotodir), 0, QTDIR);
		devdir(c, q, "vsock", 0, eve, DMDIR|0555, dp);
		return 1;

	case Qprotodir:
		if(s == DEVDOTDOT){
			mkqid(&q, QID(0, Qtopdir), 0, QTDIR);
			devdir(c, q, "vsock", 0, eve, DMDIR|0555, dp);
			return 1;
		}
		if(s == 0){
			mkqid(&q, QID(0, Qclone), 0, QTFILE);
			devdir(c, q, "clone", 0, eve, 0666, dp);
			return 1;
		}
		if(s == 1){
			mkqid(&q, QID(0, Qgctl), 0, QTFILE);
			devdir(c, q, "ctl", 0, eve, 0666, dp);
			return 1;
		}
		i = s - 2;
		if(i < 0 || i >= convtab.nused)
			return -1;
		cv = &convtab.conv[i];
		mkqid(&q, QID(i, Qconvdir), cv->gen, QTDIR);
		snprint(up->genbuf, sizeof up->genbuf, "%d", i);
		devdir(c, q, up->genbuf, 0, cv->owner? cv->owner: eve, DMDIR|0555, dp);
		return 1;

	case Qclone:
		if(s == DEVDOTDOT){
			mkqid(&q, QID(0, Qprotodir), 0, QTDIR);
			devdir(c, q, "vsock", 0, eve, DMDIR|0555, dp);
			return 1;
		}
		if(s > 0)
			return -1;
		mkqid(&q, QID(0, Qclone), 0, QTFILE);
		devdir(c, q, "clone", 0, eve, 0666, dp);
		return 1;

	case Qgctl:
		if(s == DEVDOTDOT){
			mkqid(&q, QID(0, Qprotodir), 0, QTDIR);
			devdir(c, q, "vsock", 0, eve, DMDIR|0555, dp);
			return 1;
		}
		if(s > 0)
			return -1;
		mkqid(&q, QID(0, Qgctl), 0, QTFILE);
		devdir(c, q, "ctl", 0, eve, 0666, dp);
		return 1;

	case Qconvdir:
		i = QCONV(c->qid.path);
		cv = &convtab.conv[i];
		if(s == DEVDOTDOT){
			mkqid(&q, QID(0, Qprotodir), 0, QTDIR);
			devdir(c, q, "vsock", 0, eve, DMDIR|0555, dp);
			return 1;
		}
		switch(s){
		case 0:
			mkqid(&q, QID(i, Qctl), cv->gen, QTFILE);
			devdir(c, q, "ctl", 0, cv->owner? cv->owner: eve, 0660, dp);
			return 1;
		case 1:
			mkqid(&q, QID(i, Qdata), cv->gen, QTFILE);
			devdir(c, q, "data", cv->rq? qlen(cv->rq): 0,
				cv->owner? cv->owner: eve, 0660, dp);
			return 1;
		case 2:
			mkqid(&q, QID(i, Qstatus), cv->gen, QTFILE);
			devdir(c, q, "status", 0, cv->owner? cv->owner: eve, 0440, dp);
			return 1;
		default:
			return -1;
		}

	case Qctl:
	case Qdata:
	case Qstatus:
		i = QCONV(c->qid.path);
		cv = &convtab.conv[i];
		if(s == DEVDOTDOT){
			mkqid(&q, QID(i, Qconvdir), cv->gen, QTDIR);
			snprint(up->genbuf, sizeof up->genbuf, "%d", i);
			devdir(c, q, up->genbuf, 0, cv->owner? cv->owner: eve, DMDIR|0555, dp);
			return 1;
		}
		if(s > 0)
			return -1;
		switch(QTYPE(c->qid.path)){
		case Qctl:
			mkqid(&q, QID(i, Qctl), cv->gen, QTFILE);
			devdir(c, q, "ctl", 0, cv->owner? cv->owner: eve, 0660, dp);
			return 1;
		case Qdata:
			mkqid(&q, QID(i, Qdata), cv->gen, QTFILE);
			devdir(c, q, "data", cv->rq? qlen(cv->rq): 0,
				cv->owner? cv->owner: eve, 0660, dp);
			return 1;
		case Qstatus:
			mkqid(&q, QID(i, Qstatus), cv->gen, QTFILE);
			devdir(c, q, "status", 0, cv->owner? cv->owner: eve, 0440, dp);
			return 1;
		default:
			return -1;
		}
	}
}

static Walkqid*
vsockwalk(Chan *c, Chan *nc, char **name, int nname)
{
	return devwalk(c, nc, name, nname, nil, 0, vsockgen);
}

static int
vsockstat(Chan *c, uchar *db, int n)
{
	return devstat(c, db, n, nil, 0, vsockgen);
}

static Chan*
vsockopen(Chan *c, int omode)
{
	Vsconv *cv;
	int i;

	if(c->qid.type & QTDIR){
		if(omode != OREAD)
			error(Eperm);
		c->mode = omode;
		c->flag |= COPEN;
		c->offset = 0;
		return c;
	}

	switch(QTYPE(c->qid.path)){
	case Qclone:
		ilock(&convtab);
		for(i = 0; i < NCONV; i++)
			if(convtab.conv[i].state == Sclosed && convtab.conv[i].nref == 0)
				break;
		if(i == NCONV){
			iunlock(&convtab);
			error("no free vsock conversations");
		}
		cv = &convtab.conv[i];
		cv->lport = vsockallocport();
		if(cv->lport == 0){
			iunlock(&convtab);
			error("no free vsock ports");
		}
		cv->nref = 1;
		cv->rxbytes = 0;
		cv->txbytes = 0;
		if(i+1 > convtab.nused)
			convtab.nused = i+1;
		iunlock(&convtab);

		/*
		 * qopen can block/allocate; do it outside convtab's
		 * spinlock.  The admission limit is 2*Credit while we
		 * advertise only Credit as buf_alloc: the limit is pure
		 * accounting (nothing is preallocated), qio.c counts
		 * BALLOC() bytes (payload plus ~Hdrspc+Tlrspc per block)
		 * against it and rejects outright at the edge, and the
		 * host has been observed overrunning the advertised
		 * window (see the Credit comment) -- the free slack
		 * absorbs moderate overruns instead of RSTing them.  A
		 * compliant peer never comes near the extra headroom.
		 */
		cv->rq = qopen(2*Credit, 0, nil, nil);
		if(cv->rq == nil){
			ilock(&convtab);
			cv->nref = 0;
			iunlock(&convtab);
			exhausted("vsock queue");
		}
		kstrdup(&cv->owner, up->user);
		mkqid(&c->qid, QID(i, Qctl), cv->gen, QTFILE);
		break;

	case Qgctl:
		/* global, no per-conversation state to allocate */
		break;

	case Qctl:
	case Qdata:
	case Qstatus:
		cv = &convtab.conv[QCONV(c->qid.path)];
		ilock(&convtab);
		if(cv->gen != c->qid.vers){
			iunlock(&convtab);
			error(Ehungup);
		}
		devpermcheck(cv->owner? cv->owner: eve, 0660, omode);
		cv->nref++;
		iunlock(&convtab);
		break;

	default:
		error(Eperm);
	}

	c->mode = openmode(omode);
	c->flag |= COPEN;
	c->offset = 0;
	return c;
}

static void
vsockclose(Chan *c)
{
	Vsconv *cv;
	int stale;

	if(c->qid.type & QTDIR)
		return;
	if((c->flag & COPEN) == 0)
		return;

	switch(QTYPE(c->qid.path)){
	case Qctl:
	case Qdata:
	case Qstatus:
		cv = &convtab.conv[QCONV(c->qid.path)];
		ilock(&convtab);
		stale = cv->gen != c->qid.vers;
		iunlock(&convtab);
		if(stale)
			return;
		vsockconvclose(cv);
		break;
	}
}

static long
vsockread(Chan *c, void *a, long n, vlong off)
{
	Vsconv *cv;
	char buf[32];

	if(c->qid.type & QTDIR)
		return devdirread(c, a, n, nil, 0, vsockgen);

	/*
	 * Qgctl is not tied to any conversation and must keep working
	 * regardless of what has happened to slot 0's generation, so
	 * it is special-cased ahead of the per-conversation gen check
	 * below (see the comment on vsockgctlwrite).
	 */
	if(QTYPE(c->qid.path) == Qgctl)
		return vsockgctlread(a, n, off);

	cv = &convtab.conv[QCONV(c->qid.path)];
	if(cv->gen != c->qid.vers)
		error(Ehungup);

	switch(QTYPE(c->qid.path)){
	case Qctl:
		snprint(buf, sizeof buf, "%d", QCONV(c->qid.path));
		return readstr(off, a, n, buf);

	case Qstatus:
		return vsockstatusread(cv, a, n, off);

	case Qdata:
		n = qread(cv->rq, a, n);
		if(n > 0){
			qlock(cv);
			cv->ourfwdcnt += n;
			if(cv->ourfwdcnt - cv->announced >= Credit/2
			&& (cv->state == Sestablished || cv->state == Sclosing)){
				cv->announced = cv->ourfwdcnt;
				qunlock(cv);
				vssendctl(cv, OpCreditUpdate, 0);
			}else
				qunlock(cv);
		}
		return n;

	default:
		error(Eperm);
	}
}

static long
vsockwrite(Chan *c, void *a, long n, vlong)
{
	Vsconv *cv;

	if(c->qid.type & QTDIR)
		error(Eisdir);

	if(QTYPE(c->qid.path) == Qgctl)
		return vsockgctlwrite(a, n);

	cv = &convtab.conv[QCONV(c->qid.path)];
	if(cv->gen != c->qid.vers)
		error(Ehungup);

	switch(QTYPE(c->qid.path)){
	case Qctl:
		return vsockctlwrite(cv, a, n);
	case Qdata:
		return vsockdatawrite(cv, a, n);
	default:
		error(Eperm);
	}
}

/*
 * devvsocklink -- called from links() after PCI is up (mkdevc
 * generates the call from the "devvsock" entry in the config's
 * link section; named devvsock rather than vsock to avoid
 * colliding with the "vsock" entry in the dev section, which
 * mkdevlist expects to resolve to a file named devvsock.c -- this
 * one).  Probes for the virtio-socket device, negotiates
 * VERSION_1, brings up rx/tx/event queues, reads the guest CID,
 * and starts the rx/event kproc.  If no device is present, /net/
 * vsock still attaches (so dial() fails normally rather than
 * hanging) but every conversation slot is permanently unusable
 * (clone still allocates a slot, but nothing will ever answer a
 * connect -- acceptable: absence of the optional device must not
 * wedge boot, and this driver does not special-case that further).
 */
void
devvsocklink(void)
{
	Pcidev *p;
	Vio cfg, notifybase;
	Vqueue *q;
	int cap, n, i, want;

	for(p = nil; p = pcimatch(p, Vsockvid, Vsockdid);){
		if(p->rid == 0)
			continue;
		if((cap = virtiocap(p, 1)) < 0)
			continue;
		if(virtiomapregs(p, cap, Vconf_sz, &cfg) == nil)
			continue;

		vsock.pci = p;
		vsock.cfg = cfg;
		pcienable(p);

		if(virtiomapregs(p, virtiocap(p, 3), 0, &vsock.isr) == nil)
			goto Bad;
		cap = virtiocap(p, 2);
		if(virtiomapregs(p, cap, 0, &notifybase) == nil)
			goto Bad;
		vsock.notifyoffmult = pcicfgr32(p, cap+16);

		if(virtiomapregs(p, virtiocap(p, 4), 8, &vsock.devcfg) == nil)
			goto Bad;

		coherence();
		vout8(&cfg, Vconf_status, 0);
		while(vin8(&cfg, Vconf_status) != 0)
			delay(1);
		vout8(&cfg, Vconf_status, Sacknowledge|Sdriver);

		vout32(&cfg, Vconf_devfeatsel, 1);
		vout32(&cfg, Vconf_drvfeatsel, 1);
		vout32(&cfg, Vconf_drvfeat, vin32(&cfg, Vconf_devfeat) & Fversion1);
		vout32(&cfg, Vconf_devfeatsel, 0);
		vout32(&cfg, Vconf_drvfeatsel, 0);
		vout32(&cfg, Vconf_drvfeat, 0);

		vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sfeaturesok);
		if((vin8(&cfg, Vconf_status) & Sfeaturesok) == 0){
			print("vsock: device rejected features\n");
			goto Bad;
		}

		for(i = 0; i < 3; i++){
			q = i == Vrxq? &vsock.rx: i == Vtxq? &vsock.tx: &vsock.ev;
			want = i == Vrxq? Nrxdesc: i == Vtxq? Ntxdesc: Nevdesc;

			vout16(&cfg, Vconf_queuesel, i);
			n = vin16(&cfg, Vconf_queuesize);
			if(n == 0 || (n & (n-1)) != 0){
				print("vsock: queue %d bad size %d\n", i, n);
				goto Bad;
			}
			/*
			 * The ring MUST be allocated at the device's
			 * reported size: the device masks its avail-ring
			 * index with ITS queue size, so a locally
			 * shrunken ring diverges from the device's view
			 * as soon as the index wraps our smaller mask.
			 * (Observed as every conversation wedging at
			 * exactly the 33rd packet when this driver
			 * clamped 256-entry rings to 32 without writing
			 * Vconf_queuesize back.)  ethervirtio10.c gets
			 * this right by never clamping.  We follow it,
			 * and instead cap only nbuf -- how many payload
			 * buffers we provision and allow outstanding --
			 * since the buffers, not the ring, are the
			 * memory that matters (Rxbufsz is 64K+).
			 */
			if(initqueue(q, n) < 0){
				print("vsock: queue %d alloc failed\n", i);
				goto Bad;
			}
			q->nbuf = n < want ? n : want;
			q->notify = notifybase;
			if(q->notify.type == Vio_mem)
				q->notify.mem += vsock.notifyoffmult
					* vin16(&cfg, Vconf_queuenotifyoff);
			else
				q->notify.port += vsock.notifyoffmult
					* vin16(&cfg, Vconf_queuenotifyoff);
			coherence();
			vout64(&cfg, Vconf_queuedesc, PADDR(q->desc));
			vout64(&cfg, Vconf_queueavail, PADDR(q->avail));
			vout64(&cfg, Vconf_queueused, PADDR(q->used));
			vout16(&cfg, Vconf_queuesel, i);
			vout16(&cfg, Vconf_queueenable, 1);
		}

		for(i = 0; i < vsock.rx.nbuf; i++){
			vsock.rxbuf[i] = mallocalign(Rxbufsz, 8, 0, 0);
			if(vsock.rxbuf[i] == nil){
				print("vsock: no memory for rx buffers\n");
				goto Bad;
			}
		}
		for(i = 0; i < vsock.tx.nbuf; i++){
			vsock.txbuf[i] = mallocalign(Txbufsz, 8, 0, 0);
			if(vsock.txbuf[i] == nil){
				print("vsock: no memory for tx buffers\n");
				goto Bad;
			}
		}
		for(i = 0; i < vsock.ev.nbuf; i++){
			vsock.evbuf[i] = mallocalign(Evbufsz, 8, 0, 0);
			if(vsock.evbuf[i] == nil){
				print("vsock: no memory for event buffers\n");
				goto Bad;
			}
		}

		vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sdriverok);

		vsock.guestcid = vin64(&vsock.devcfg, 0);

		for(i = 0; i < vsock.rx.nbuf; i++)
			rxpost(i);
		coherence();
		vout16(&vsock.rx.notify, 0, Vrxq);

		for(i = 0; i < vsock.ev.nbuf; i++)
			evpost(i);
		coherence();
		vout16(&vsock.ev.notify, 0, Vevq);

		pcisetbme(p);
		intrenable(p->intl, interrupt, nil, p->tbdf, "vsock");
		vsock.intr = 1;

		vsock.ready = 1;
		print("vsock: guest cid %llud\n", vsock.guestcid);

		/*
		 * The rx/event kproc is NOT started here.  links() runs
		 * on the per-Mach boot stack before schedinit(), with
		 * up == nil AND eve == nil (userinit() has not run yet);
		 * kproc() does kstrdup(&p->user, eve), so calling it
		 * here dereferences nil and the recursive fault (up is
		 * also nil in faultarm64's waserror) kills the VM with
		 * no output at all.  See audiovz.c's "Do NOT create the
		 * drain kproc here" note for the same rule.  The kproc
		 * is started from vsockinit() (Dev.init), which
		 * chandevinit() runs in proc0's context after
		 * schedinit.
		 */
		return;
Bad:
		pcidisable(p);
		vsock.pci = nil;
	}
	/* no virtio-socket device: /net/vsock still attaches but is inert */
}

/*
 * Dev.init -- run by chandevinit() from init0(), i.e. in proc0's
 * process context after schedinit with up != nil and eve set.
 * This is the earliest point where kproc() is safe on this port
 * (see the comment at the end of devvsocklink), and it is still
 * before any userspace can open /net/vsock, so no conversation
 * traffic can precede the rx kproc.
 */
static void
vsockinit(void)
{
	if(vsock.ready)
		kproc("vsockrx", rxproc, nil);
}

Dev vsockdevtab = {
	'V',
	"vsock",

	devreset,
	vsockinit,
	devshutdown,
	vsockattach,
	vsockwalk,
	vsockstat,
	vsockopen,
	devcreate,
	vsockclose,
	vsockread,
	devbread,
	vsockwrite,
	devbwrite,
	devremove,
	devwstat,
};
