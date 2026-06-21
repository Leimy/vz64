/*
 * virtio-gpu (virtio 1.0) framebuffer for Apple Virtualization.framework.
 *
 * Implements the devdraw "screen.c" contract (attachscreen,
 * flushmemscreen, cursor hooks) on top of a virtio-gpu device.
 *
 * The driver uses a single host 2D resource the size of the
 * scanout, with a guest-side linear framebuffer ("softscreen")
 * attached as backing.  devdraw renders into the framebuffer in
 * main memory; flushmemscreen issues TRANSFER_TO_HOST_2D +
 * RESOURCE_FLUSH for the dirty rectangle to push it to the host
 * scanout.
 *
 * The control queue (queue 0) completes asynchronously: the driver
 * posts the command(s), notifies the device, and SLEEPS on the
 * control-queue completion interrupt (a Rendez woken by
 * interrupt()).  An earlier version busy-waited on the used ring
 * with interrupts DISABLED (ilock across the whole host round
 * trip); because flushmemscreen runs on the hot devdraw path, that
 * blocked the virtual timer and the virtio-blk completion
 * interrupt on every flush and throttled file I/O by orders of
 * magnitude under -gui (see NOTES "GUI Bug 4").  The wait now runs
 * at spllo, yields the cpu, and pays one host round trip per flush
 * (transfer + flush are coalesced).  Early bring-up (screeninit,
 * before schedinit) and any !islo() console path fall back to
 * polling -- but never with interrupts forced off.
 *
 * The cursor is software (swcursor); we do not use the virtio-gpu
 * hardware cursor queue.
 *
 * Bring-up is two-phase like the console: screeninit() (called
 * from main, after links()/PCI) probes the device and sets up the
 * framebuffer.  If no virtio-gpu device is present (headless boot,
 * 9vz without -gui) screeninit quietly does nothing and the kernel
 * stays without a display.
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

#define	Image	IMAGE
#include <draw.h>
#include <memdraw.h>
#include <cursor.h>
#include "screen.h"

enum {
	/*
	 * Fallback framebuffer size, used only if GET_DISPLAY_INFO does
	 * not report a usable scanout and no vgasize= is set.  A 16:10
	 * laptop-native shape (1440x900) rather than the old 4:3 1024x768;
	 * in practice the host (9vz) advertises its own size and we adopt
	 * that in screeninit via gpudisplayinfo().
	 */
	Wid		= 1440,
	Ht		= 900,

	Tabstop		= 4,
	Scroll		= 8,

	/* descriptor flags */
	Dnext	= 1,
	Dwrite	= 2,

	/* vring avail flags */
	Rnointerrupt = 1,

	/* struct sizes */
	VringSize	= 4,
	VdescSize	= 16,
	VusedSize	= 8,

	/* virtio-gpu queues */
	Vctlq	= 0,	/* controlq */
	Vcursorq= 1,	/* cursorq (unused) */

	/* virtio-gpu 2D commands (le32) */
	CmdGetDisplayInfo	= 0x0100,
	CmdResourceCreate2d	= 0x0101,
	CmdResourceUnref	= 0x0102,
	CmdSetScanout		= 0x0103,
	CmdResourceFlush	= 0x0104,
	CmdTransferToHost2d	= 0x0105,
	CmdResourceAttachBacking= 0x0106,
	CmdResourceDetachBacking= 0x0107,

	/* responses */
	RespOkNodata		= 0x1100,
	RespOkDisplayInfo	= 0x1101,

	/* error responses (for diagnostics) */
	RespErrUnspec		= 0x1200,
	RespErrOutOfMemory	= 0x1201,
	RespErrInvalidScanoutId	= 0x1202,
	RespErrInvalidResourceId= 0x1203,
	RespErrInvalidContextId	= 0x1204,
	RespErrInvalidParameter	= 0x1205,

	/* max scanouts reported in GET_DISPLAY_INFO */
	MaxScanouts		= 16,

	/*
	 * virtio-gpu pixel formats (virtio_gpu_formats).  The host
	 * whitelists which of these RESOURCE_CREATE_2D accepts; Apple
	 * Virtualization.framework is pickier than QEMU, so we probe a
	 * small candidate list rather than assume B8G8R8A8.
	 *
	 * The Plan 9 framebuffer is XRGB32: 32bpp, little-endian word
	 * 0xAARRGGBB, i.e. bytes [B,G,R,X] in memory.  That byte order
	 * is what the host reads, so the matching host format is one of
	 * the B8G8R8x8 variants (X or A in the high byte).
	 */
	FmtB8G8R8A8Unorm	= 1,
	FmtB8G8R8X8Unorm	= 2,
	FmtA8R8G8B8Unorm	= 3,
	FmtX8R8G8B8Unorm	= 4,
	FmtR8G8B8A8Unorm	= 67,
	FmtX8B8G8R8Unorm	= 68,
	FmtA8B8G8R8Unorm	= 121,
	FmtR8G8B8X8Unorm	= 134,

	ResId	= 1,	/* our single scanout resource id */
};

typedef struct Vqueue Vqueue;
struct Vqueue
{
	Rendez;		/* slept on by ctlcmd, woken by interrupt() */

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

/* virtio-gpu command/response headers and bodies */
typedef struct Gpuhdr Gpuhdr;
struct Gpuhdr
{
	u32int	type;
	u32int	flags;
	u64int	fenceid;
	u32int	ctxid;
	u32int	pad;
};

typedef struct Gpurect Gpurect;
struct Gpurect
{
	u32int	x;
	u32int	y;
	u32int	width;
	u32int	height;
};

typedef struct Gpucreate2d Gpucreate2d;
struct Gpucreate2d
{
	Gpuhdr	hdr;
	u32int	resid;
	u32int	format;
	u32int	width;
	u32int	height;
};

typedef struct Gpumementry Gpumementry;
struct Gpumementry
{
	u64int	addr;
	u32int	length;
	u32int	pad;
};

typedef struct Gpuattachbacking Gpuattachbacking;
struct Gpuattachbacking
{
	Gpuhdr	hdr;
	u32int	resid;
	u32int	nentries;
	/* followed by nentries Gpumementry */
};

typedef struct Gpusetscanout Gpusetscanout;
struct Gpusetscanout
{
	Gpuhdr	hdr;
	Gpurect	r;
	u32int	scanoutid;
	u32int	resid;
};

typedef struct Gputransfer Gputransfer;
struct Gputransfer
{
	Gpuhdr	hdr;
	Gpurect	r;
	u64int	offset;
	u32int	resid;
	u32int	pad;
};

typedef struct Gpuflush Gpuflush;
struct Gpuflush
{
	Gpuhdr	hdr;
	Gpurect	r;
	u32int	resid;
	u32int	pad;
};

typedef struct Gpudisplayone Gpudisplayone;
struct Gpudisplayone
{
	Gpurect	r;
	u32int	enabled;
	u32int	flags;
};

typedef struct Gpudisplayinfo Gpudisplayinfo;
struct Gpudisplayinfo
{
	Gpuhdr	hdr;
	Gpudisplayone	pmodes[MaxScanouts];
};

static struct {
	int	ready;
	int	intr;		/* control-queue interrupt is wired up */
	int	nosleep;	/* console output holds screenlock: must poll */

	/*
	 * The control queue carries one command at a time.  ctll
	 * (a QLock) serialises submitters; it is a QLock, not an
	 * ilock, so the wait for the host to complete a command does
	 * NOT run with interrupts disabled.  The earlier ilock held
	 * splhi across two full host round trips (TRANSFER_TO_HOST_2D
	 * + RESOURCE_FLUSH) on EVERY flushmemscreen, blocking the
	 * virtual timer and the virtio-blk/-net completion interrupts
	 * for the whole round trip -- which throttled file I/O by
	 * orders of magnitude whenever the GUI was drawing.  See the
	 * "GUI Bug 4" note.
	 *
	 * il (an ilock) covers only the brief vring pointer/index
	 * manipulation that the interrupt handler also touches; it is
	 * dropped before sleeping/polling for completion.
	 */
	QLock	ctll;
	Lock	il;

	Pcidev	*pci;
	Vio	cfg;
	Vio	isr;
	u32int	notifyoffmult;
	u32int	devfeat0;	/* offered feature word 0 (gpu opts) */
	u32int	devfeat1;	/* offered feature word 1 (VERSION_1) */

	Vqueue	ctl;

	/* command scratch (single in-flight command) */
	uchar	cmd[1024];
	Gpuhdr	resp;
} vgpu;

Memimage *gscreen;

static Memdata xgdata;
static Memimage xgscreen =
{
	{ 0, 0, Wid, Ht },	/* r (overridden by host scanout/vgasize) */
	{ 0, 0, Wid, Ht },	/* clipr */
	32,			/* depth */
	4,			/* nchan */
	XRGB32,			/* chan */
	nil,			/* cmap */
	&xgdata,		/* data */
	0,			/* zero */
	0, 			/* width in words of a single scan line */
	0,			/* layer */
	0,			/* flags */
};

static Memimage *conscol;
static Memimage *back;
static Memsubfont *memdefont;

static Lock screenlock;

static Point	curpos;
static int	h, w;
static Rectangle window;

static void myscreenputs(char *s, int n);
static void screenputc(char *buf);
static void screenwin(void);
static char *gpurespname(u32int t);

/*
 * queue setup (same pattern as uartvz.c / ethervirtio10.c)
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
	 * Leave the queue interrupt ENABLED (do not set Rnointerrupt):
	 * ctlcmd waits for completion by sleeping on the used-ring
	 * interrupt rather than busy-spinning with interrupts off.
	 */
	return 0;
}

/* used-ring has a fresh entry (completion is ready) */
static int
ctlhasroom(void *v)
{
	Vqueue *q = v;
	return q->used->idx != q->lastused;
}

/*
 * Wait for the host to complete the single in-flight control
 * command.  Three contexts call into here:
 *   - normal draw/flush path (process context, islo()): sleep on
 *     the queue interrupt so the cpu is free to run other procs
 *     and, crucially, to take virtio-blk/-net/timer interrupts.
 *   - early bring-up (screeninit) before the interrupt handler is
 *     wired (vgpu.intr == 0): poll, but with interrupts ENABLED.
 *   - console output from interrupt/print context (!islo()): poll
 *     with interrupts in whatever state the caller left them; this
 *     is rare and short (a banner / panic line).
 * In every case interrupts are NOT forced off across the host
 * round trip, which was the throughput bug.
 */
static int
cansleep(void)
{
	/*
	 * We may sleep only with the interrupt wired, in process
	 * context (up != nil -- screeninit runs before schedinit, so
	 * up is nil there and we must poll), at spllo (not from
	 * interrupt level), and NOT while the console path holds the
	 * screenlock spinlock (vgpu.nosleep): sleeping with a spinlock
	 * held would wedge the scheduler.
	 */
	return vgpu.intr && up != nil && islo() && !vgpu.nosleep;
}

static void
ctlwait(Vqueue *q)
{
	if(cansleep()){
		/*
		 * tsleep, not sleep: a short timeout is a safety net in
		 * case the host's used-ring interrupt is not delivered
		 * the way we expect (then we wake on the timeout, re-poll
		 * the predicate, and make progress).  When the interrupt
		 * does arrive -- the normal case -- the wakeup returns us
		 * immediately, well under the timeout.  Either way the cpu
		 * is free to run other procs and take device interrupts
		 * while we wait, which is the whole point.
		 */
		while(!ctlhasroom(q))
			tsleep(q, ctlhasroom, q, 5);
	} else {
		while(!ctlhasroom(q))
			coherence();
	}
	q->lastused = q->used->idx;
}

/*
 * Issue one control command synchronously: two descriptors,
 * cmd (device-read) chained to resp (device-write).  Waits for
 * the used ring without disabling interrupts (see ctlwait).
 * Returns the response type, or 0 on a queue error.  Caller holds
 * vgpu.ctll.
 */
static u32int
ctlcmd(void *cmd, int cmdlen, void *resp, int resplen)
{
	Vqueue *q;
	Gpuhdr *rh;
	int i;

	q = &vgpu.ctl;

	ilock(&vgpu.il);
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
	vout16(&q->notify, 0, Vctlq);
	iunlock(&vgpu.il);

	ctlwait(q);

	rh = resp;
	return rh->type;
}

/*
 * Control-queue completion interrupt.  The host signals the ISR
 * when it has written a used-ring entry; wake whoever is sleeping
 * in ctlwait.  Reading the ISR status register acknowledges it.
 */
static void
interrupt(Ureg*, void*)
{
	Vqueue *q;

	if(vin8(&vgpu.isr, 0) & 1){
		q = &vgpu.ctl;
		if(ctlhasroom(q))
			wakeup(q);
	}
}

/*
 * Combined transfer + flush for a dirty rectangle.  TRANSFER_TO_
 * HOST_2D and RESOURCE_FLUSH are two SEPARATE virtio-gpu commands
 * (each a self-contained descriptor chain producing its own
 * used-ring entry).  We post BOTH available entries, then notify
 * ONCE and wait for BOTH completions in a single ctlwait spell.
 * This keeps the two commands' ordering (the device drains the
 * avail ring in order) while paying only one notify and one
 * wakeup per flushmemscreen instead of two -- halving the host
 * round trips on the hot draw path.  Caller holds vgpu.ctll.
 *
 * Descriptor layout (two independent chains):
 *   chain A: desc0 (xfer cmd, read) -> desc1 (xfer resp, write)
 *   chain B: desc2 (flush cmd, read) -> desc3 (flush resp, write)
 */
static int
gpuxferflush(int resid, Rectangle r, ulong stride)
{
	Vqueue *q;
	Gputransfer *xf;
	Gpuflush *fl;
	Gpuhdr *r0, *r1;
	uchar *p;
	int i;
	u16int want;

	q = &vgpu.ctl;
	if(q->qsize < 4)
		return -1;	/* need 4 descriptors + 2 avail slots */

	/*
	 * Lay out two commands and two response headers in the
	 * scratch buffer: [xfer][flush][resp0][resp1].
	 */
	p = vgpu.cmd;
	xf = (Gputransfer*)p;
	fl = (Gpuflush*)(p + sizeof(Gputransfer));
	r0 = (Gpuhdr*)(p + sizeof(Gputransfer) + sizeof(Gpuflush));
	r1 = r0 + 1;
	if((uchar*)(r1+1) > vgpu.cmd + sizeof(vgpu.cmd))
		return -1;

	memset(xf, 0, sizeof *xf);
	xf->hdr.type = CmdTransferToHost2d;
	xf->r.x = r.min.x;
	xf->r.y = r.min.y;
	xf->r.width = Dx(r);
	xf->r.height = Dy(r);
	xf->offset = (u64int)r.min.y * stride + (u64int)r.min.x * 4;
	xf->resid = resid;

	memset(fl, 0, sizeof *fl);
	fl->hdr.type = CmdResourceFlush;
	fl->r.x = r.min.x;
	fl->r.y = r.min.y;
	fl->r.width = Dx(r);
	fl->r.height = Dy(r);
	fl->resid = resid;

	memset(r0, 0, sizeof *r0);
	memset(r1, 0, sizeof *r1);

	ilock(&vgpu.il);
	/* chain A: xfer cmd -> xfer resp */
	q->desc[0].addr = PADDR(xf);
	q->desc[0].len = sizeof *xf;
	q->desc[0].flags = Dnext;
	q->desc[0].next = 1;

	q->desc[1].addr = PADDR(r0);
	q->desc[1].len = sizeof *r0;
	q->desc[1].flags = Dwrite;
	q->desc[1].next = 0;

	/* chain B: flush cmd -> flush resp */
	q->desc[2].addr = PADDR(fl);
	q->desc[2].len = sizeof *fl;
	q->desc[2].flags = Dnext;
	q->desc[2].next = 3;

	q->desc[3].addr = PADDR(r1);
	q->desc[3].len = sizeof *r1;
	q->desc[3].flags = Dwrite;
	q->desc[3].next = 0;

	/* post both available entries (heads 0 and 2) */
	i = q->avail->idx & q->qmask;
	q->availent[i] = 0;
	i = (q->avail->idx + 1) & q->qmask;
	q->availent[i] = 2;
	coherence();
	q->avail->idx += 2;
	want = q->avail->idx;
	coherence();
	vout16(&q->notify, 0, Vctlq);
	iunlock(&vgpu.il);

	/*
	 * Wait for both completions.  ctlhasroom(q) is true while the
	 * device has produced a used entry we have not yet retired;
	 * sleep on it (interrupts stay on) until we have retired both
	 * heads (lastused has caught up to want).
	 */
	for(;;){
		ilock(&vgpu.il);
		while(q->used->idx != q->lastused)
			q->lastused++;
		i = (u16int)(want - q->lastused) != 0;
		iunlock(&vgpu.il);
		if(!i)
			break;
		if(cansleep())
			tsleep(q, ctlhasroom, q, 5);	/* timeout = safety net */
		else
			coherence();
	}

	if(r0->type != RespOkNodata || r1->type != RespOkNodata)
		return -1;
	return 0;
}

/*
 * Build a virtio-gpu command in the scratch buffer and run it.
 * The response goes into vgpu.resp (header only -- all the
 * commands we issue return RESP_OK_NODATA).  Returns the
 * response type.
 */
static u32int
gpucmd(u32int type, void *body, int bodylen)
{
	Gpuhdr *h;

	if(bodylen > sizeof(vgpu.cmd))
		return 0;
	h = (Gpuhdr*)vgpu.cmd;
	memmove(vgpu.cmd, body, bodylen);
	h->type = type;
	h->flags = 0;
	h->fenceid = 0;
	h->ctxid = 0;
	h->pad = 0;
	return ctlcmd(vgpu.cmd, bodylen, &vgpu.resp, sizeof(vgpu.resp));
}

/*
 * Candidate host pixel formats for the XRGB32 framebuffer, most
 * likely first.  The guest framebuffer bytes are [B,G,R,X]; the
 * host reads the resource with whatever format we declare here.
 * Apple's virtio-gpu only accepts a subset, so RESOURCE_CREATE_2D
 * is retried down this list until the host stops returning
 * err-invalid-parameter.  The chosen format is remembered so a
 * later RESOURCE_UNREF + recreate would reuse it.
 */
static u32int gpufmts[] = {
	FmtB8G8R8X8Unorm,
	FmtB8G8R8A8Unorm,
	FmtX8R8G8B8Unorm,
	FmtA8R8G8B8Unorm,
	FmtR8G8B8X8Unorm,
	FmtR8G8B8A8Unorm,
	FmtX8B8G8R8Unorm,
	FmtA8B8G8R8Unorm,
};
static u32int gpufmt;	/* format the host accepted */

static int
gpucreate2dfmt(int resid, u32int fmt, int width, int height)
{
	Gpucreate2d c;

	memset(&c, 0, sizeof c);
	c.resid = resid;
	c.format = fmt;
	c.width = width;
	c.height = height;
	return gpucmd(CmdResourceCreate2d, &c, sizeof c);
}

static int
gpucreate2d(int resid, int width, int height)
{
	u32int t;
	int i;

	for(i = 0; i < nelem(gpufmts); i++){
		t = gpucreate2dfmt(resid, gpufmts[i], width, height);
		if(t == RespOkNodata){
			gpufmt = gpufmts[i];
			print("virtio-gpu: create-2d: ok, host accepted "
				"format %#ux (candidate %d of %d)\n",
				gpufmt, i, nelem(gpufmts));
			return 0;
		}
		/*
		 * Only an invalid-parameter error is worth retrying with
		 * a different format; anything else is a real failure.
		 */
		if(t != RespErrInvalidParameter){
			print("virtio-gpu: create-2d: %s (%#ux)\n",
				gpurespname(t), t);
			return -1;
		}
	}
	print("virtio-gpu: create-2d: host rejected all %d formats\n",
		nelem(gpufmts));
	return -1;
}

/*
 * Attach the guest framebuffer as backing for the host resource.
 * The framebuffer is a single kernel allocation, but the host
 * needs its physical scatter list.  Kernel allocations live in the
 * KZERO direct map, where PA = VA - KZERO is linear and therefore
 * physically contiguous -- so in practice one mementry suffices.
 * We nonetheless emit one entry per physical page so the code is
 * correct even if the allocation is not contiguous, and so a
 * non-direct-mapped framebuffer would be caught (PADDR panics
 * outside the direct map, which is the honest failure here).
 */
static int
gpuattachbacking(int resid, void *fb, ulong len)
{
	uchar *buf;
	Gpuattachbacking *c;
	Gpumementry *e;
	uintptr va, end;
	ulong n, sz, off;
	u32int t;
	int r;

	n = (PGROUND((uintptr)fb + len) - ((uintptr)fb & ~(BY2PG-1))) / BY2PG;
	sz = sizeof(Gpuattachbacking) + n*sizeof(Gpumementry);
	/*
	 * The attach-backing command is built in its OWN buffer (below),
	 * not the gpucmd scratch (vgpu.cmd), so its size is NOT bounded
	 * by sizeof(vgpu.cmd).  An earlier guard against that here was
	 * wrong: a full-screen framebuffer needs one mementry per page
	 * (768 entries for 1024x768x4 => ~12KB), far exceeding the 1KB
	 * scratch, and the bogus check silently failed attach-backing
	 * ("scanout setup failed" with no command-specific error).
	 */
	buf = mallocz(sz, 1);
	if(buf == nil){
		print("virtio-gpu: attach-backing: no memory for %lud entries\n", n);
		return -1;
	}

	c = (Gpuattachbacking*)buf;
	c->hdr.type = CmdResourceAttachBacking;
	c->resid = resid;
	c->nentries = n;
	e = (Gpumementry*)(buf + sizeof(Gpuattachbacking));

	va = (uintptr)fb;
	end = va + len;
	off = va & (BY2PG-1);		/* first page may start mid-page */
	while(va < end){
		ulong plen = BY2PG - off;
		if(va + plen > end)
			plen = end - va;
		e->addr = PADDR((void*)va);
		e->length = plen;
		e++;
		va += plen;
		off = 0;
	}

	/*
	 * The attach-backing command can be far larger than the
	 * gpucmd scratch buffer (one mementry per framebuffer page),
	 * so drive the control queue directly with our own buffer.
	 */
	t = ctlcmd(buf, sz, &vgpu.resp, sizeof(vgpu.resp));
	r = t == RespOkNodata ? 0 : -1;
	if(r < 0)
		print("virtio-gpu: attach-backing (%lud entries): %s (%#ux)\n",
			n, gpurespname(t), t);
	free(buf);
	return r;
}

static int
gpusetscanout(int scanout, int resid, int width, int height)
{
	Gpusetscanout c;
	u32int t;

	memset(&c, 0, sizeof c);
	c.r.x = 0;
	c.r.y = 0;
	c.r.width = width;
	c.r.height = height;
	c.scanoutid = scanout;
	c.resid = resid;
	t = gpucmd(CmdSetScanout, &c, sizeof c);
	if(t != RespOkNodata){
		print("virtio-gpu: set-scanout: %s (%#ux)\n", gpurespname(t), t);
		return -1;
	}
	return 0;
}

/*
 * Human-readable name for a virtio-gpu response type, for
 * bring-up diagnostics.
 */
static char*
gpurespname(u32int t)
{
	switch(t){
	case RespOkNodata:		return "ok-nodata";
	case RespOkDisplayInfo:		return "ok-displayinfo";
	case RespErrUnspec:		return "err-unspec";
	case RespErrOutOfMemory:	return "err-out-of-memory";
	case RespErrInvalidScanoutId:	return "err-invalid-scanout-id";
	case RespErrInvalidResourceId:	return "err-invalid-resource-id";
	case RespErrInvalidContextId:	return "err-invalid-context-id";
	case RespErrInvalidParameter:	return "err-invalid-parameter";
	case 0:				return "no-response";
	}
	return "unknown";
}

/*
 * Query the host scanout geometry.  Doubles as the first control-
 * queue round trip, so a failure here localises the problem to
 * queue setup rather than the later resource/scanout commands.
 * On success fills *width/*height with scanout 0's preferred size
 * (0 if the host reports the display disabled).
 */
static int
gpudisplayinfo(int *width, int *height)
{
	Gpudisplayinfo info;
	Gpuhdr h;
	u32int t;

	memset(&h, 0, sizeof h);
	h.type = CmdGetDisplayInfo;

	t = ctlcmd(&h, sizeof h, &info, sizeof info);
	if(t != RespOkDisplayInfo){
		print("virtio-gpu: get-display-info: %s (%#ux)\n",
			gpurespname(t), t);
		return -1;
	}
	if(width != nil)
		*width = info.pmodes[0].r.width;
	if(height != nil)
		*height = info.pmodes[0].r.height;
	print("virtio-gpu: scanout0 %udx%ud enabled=%ud\n",
		info.pmodes[0].r.width, info.pmodes[0].r.height,
		info.pmodes[0].enabled);
	return 0;
}

/*
 * PCI capability walking — same as uartvz.c / ethervirtio10.c
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
 * Probe for the virtio-gpu device (0x1AF4:0x1050), negotiate
 * VERSION_1, set up the single control queue.  Returns 0 on
 * success, -1 if no device / setup failed.
 */
static int
gpuprobe(void)
{
	Pcidev *p;
	Vio cfg, notifybase;
	Vqueue *q;
	int cap, n;

	for(p = nil; p = pcimatch(p, 0x1AF4, 0x1050);){
		if(p->rid == 0)
			continue;
		if((cap = virtiocap(p, 1)) < 0)
			continue;
		if(virtiomapregs(p, cap, Vconf_sz, &cfg) == nil)
			continue;

		vgpu.pci = p;
		vgpu.cfg = cfg;
		pcienable(p);

		print("virtio-gpu: %T did %#ux rid %d\n",
			p->tbdf, p->did, p->rid);

		if(virtiomapregs(p, virtiocap(p, 3), 0, &vgpu.isr) == nil)
			goto Bad;
		cap = virtiocap(p, 2);
		if(virtiomapregs(p, cap, 0, &notifybase) == nil)
			goto Bad;
		vgpu.notifyoffmult = pcicfgr32(p, cap+16);

		/* reset device */
		coherence();
		vout8(&cfg, Vconf_status, 0);
		while(vin8(&cfg, Vconf_status) != 0)
			delay(1);
		vout8(&cfg, Vconf_status, Sacknowledge|Sdriver);

		/*
		 * Negotiate features.  We are a pure 2D driver: accept
		 * only VIRTIO_F_VERSION_1 (bit 32, i.e. bit 0 of feature
		 * word 1) and drive ALL of feature word 0 to zero.  Word 0
		 * is where virtio-gpu's own optional features live --
		 * VIRGL(0) EDID(1) RESOURCE_UUID(2) RESOURCE_BLOB(3)
		 * CONTEXT_INIT(4) -- and we want none of them: no 3D/virgl,
		 * no host-visible blob resources, no hostmem window.  This
		 * keeps us on the plain GET_DISPLAY_INFO / CREATE_2D /
		 * ATTACH_BACKING / SET_SCANOUT / TRANSFER_TO_HOST_2D +
		 * RESOURCE_FLUSH path that QEMU's 2D backend implements and
		 * that Apple's virtio-gpu should too.  The device's offered
		 * word-0 bits are read only for the diagnostic print so we
		 * can SEE on the next boot whether VZ even offers virgl/
		 * blob (it should not for our purposes).
		 */
		vout32(&cfg, Vconf_devfeatsel, 0);
		vgpu.devfeat0 = vin32(&cfg, Vconf_devfeat);
		vout32(&cfg, Vconf_devfeatsel, 1);
		vgpu.devfeat1 = vin32(&cfg, Vconf_devfeat);

		vout32(&cfg, Vconf_drvfeatsel, 1);
		vout32(&cfg, Vconf_drvfeat, vgpu.devfeat1 & Fversion1);
		vout32(&cfg, Vconf_drvfeatsel, 0);
		vout32(&cfg, Vconf_drvfeat, 0);

		print("virtio-gpu: devfeat %#.8ux:%#.8ux drvfeat %#.8ux:%#.8ux\n",
			vgpu.devfeat1, vgpu.devfeat0,
			vgpu.devfeat1 & Fversion1, 0);

		vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sfeaturesok);

		/*
		 * §3.1.1: re-read status; if the device cleared
		 * FEATURES_OK it does not accept our feature subset
		 * and we must not proceed.
		 */
		if((vin8(&cfg, Vconf_status) & Sfeaturesok) == 0){
			print("virtio-gpu: device rejected features\n");
			goto Bad;
		}

		/* control queue (queue 0) */
		q = &vgpu.ctl;
		vout16(&cfg, Vconf_queuesel, Vctlq);
		n = vin16(&cfg, Vconf_queuesize);
		if(n == 0 || (n & (n-1)) != 0){
			print("virtio-gpu: bad controlq size %d\n", n);
			goto Bad;
		}
		if(initqueue(q, n) < 0){
			print("virtio-gpu: controlq alloc failed\n");
			goto Bad;
		}
		q->notify = notifybase;
		if(q->notify.type == Vio_mem)
			q->notify.mem += vgpu.notifyoffmult
				* vin16(&cfg, Vconf_queuenotifyoff);
		else
			q->notify.port += vgpu.notifyoffmult
				* vin16(&cfg, Vconf_queuenotifyoff);
		coherence();
		vout64(&cfg, Vconf_queuedesc, PADDR(q->desc));
		vout64(&cfg, Vconf_queueavail, PADDR(q->avail));
		vout64(&cfg, Vconf_queueused, PADDR(q->used));

		vout16(&cfg, Vconf_queuesel, Vctlq);
		vout16(&cfg, Vconf_queueenable, 1);

		/* driver ok */
		vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sdriverok);
		pcisetbme(p);

		/*
		 * Wire the control-queue completion interrupt so flushes
		 * can sleep instead of busy-polling.  Until this is set,
		 * ctlwait falls back to polling (the screeninit bring-up
		 * commands run before interrupts are wanted anyway, and
		 * with vgpu.intr==0 ctlwait stays in the poll branch).
		 */
		intrenable(p->intl, interrupt, nil, p->tbdf, "virtio-gpu");
		vgpu.intr = 1;

		vgpu.ready = 1;
		return 0;
Bad:
		pcidisable(p);
		vgpu.pci = nil;
	}
	return -1;
}

/*
 * devdraw hooks
 */
void
flushmemscreen(Rectangle r)
{
	if(!vgpu.ready || gscreen == nil)
		return;
	if(!rectclip(&r, gscreen->r))
		return;
	if(Dx(r) <= 0 || Dy(r) <= 0)
		return;

	/*
	 * Serialise control-queue submitters with a QLock (NOT an
	 * ilock): the wait for the host to finish must not run with
	 * interrupts disabled, or it stalls the timer and the
	 * virtio-blk/-net completion interrupts for the whole host
	 * round trip -- that interrupt-off busy-wait was the file-I/O
	 * throughput killer.  qlock can sleep, which is fine on the
	 * draw path (process context).
	 *
	 * Three context cases:
	 *  - up == nil: early boot (screenwin from screeninit),
	 *    single-threaded; no lock needed, gpuxferflush polls.
	 *  - islo() && up: the hot draw/console path; qlock and let
	 *    gpuxferflush sleep on the completion interrupt.
	 *  - !islo() (iprint/panic while a spinlock is held): must
	 *    not sleep; canqlock best-effort and poll.  If the lock is
	 *    held by a sleeper, skip the flush -- the next draw
	 *    repaints the region.
	 */
	if(up == nil){
		gpuxferflush(ResId, r, gscreen->width * sizeof(ulong));
		return;
	}
	if(!islo() || vgpu.nosleep){
		/*
		 * Must not block: either at interrupt/spinlock level
		 * (!islo()) or on the console output path holding
		 * screenlock (vgpu.nosleep).  canqlock is best-effort; if
		 * the queue is busy, skip the flush -- the next draw
		 * repaints the region.  gpuxferflush will poll (cansleep
		 * is false here), not sleep.
		 */
		if(!canqlock(&vgpu.ctll))
			return;
	} else
		qlock(&vgpu.ctll);
	gpuxferflush(ResId, r, gscreen->width * sizeof(ulong));
	qunlock(&vgpu.ctll);
}

Memdata*
attachscreen(Rectangle *r, ulong *chan, int *d, int *width, int *softscreen)
{
	if(gscreen == nil)
		return nil;

	*r = gscreen->r;
	*d = gscreen->depth;
	*chan = gscreen->chan;
	*width = gscreen->width;
	*softscreen = 1;

	gscreen->data->ref++;
	return gscreen->data;
}

void
getcolor(ulong p, ulong *pr, ulong *pg, ulong *pb)
{
	USED(p, pr, pg, pb);
}

int
setcolor(ulong p, ulong r, ulong g, ulong b)
{
	USED(p, r, g, b);
	return 0;
}

void
blankscreen(int)
{
}

/* software cursor */
void
cursoron(void)
{
	swcursorhide(0);
	swcursordraw(mousexy());
}

void
cursoroff(void)
{
	swcursorhide(0);
}

void
setcursor(Cursor* curs)
{
	swcursorload(curs);
}

int
hwdraw(Memdrawparam *par)
{
	Memimage *dst, *src, *mask;

	if((dst = par->dst) == nil || dst->data == nil)
		return 0;
	if((src = par->src) && src->data == nil)
		src = nil;
	if((mask = par->mask) && mask->data == nil)
		mask = nil;

	if(dst->data->bdata == xgdata.bdata)
		swcursoravoid(par->r);
	if(src && src->data->bdata == xgdata.bdata)
		swcursoravoid(par->sr);
	if(mask && mask->data->bdata == xgdata.bdata)
		swcursoravoid(par->mr);

	return 0;
}

/* mouse.c: accelerated/linear ctl */
enum
{
	CMaccelerated,
	CMlinear,
};

static Cmdtab mousectlmsg[] =
{
	CMaccelerated,	"accelerated",	0,
	CMlinear,	"linear",	1,
};

void
mousectl(Cmdbuf *cb)
{
	Cmdtab *ct;

	ct = lookupcmd(cb, mousectlmsg, nelem(mousectlmsg));
	switch(ct->index){
	case CMaccelerated:
		mouseaccelerate(cb->nf == 1? 1: atoi(cb->f[1]));
		break;
	case CMlinear:
		mouseaccelerate(0);
		break;
	}
}

/*
 * vgasize=WxHxD in the boot config overrides the default size.
 * Depth must be 32 (the only format we attach to the host).
 */
static int
screensize(void)
{
	char *p, *f[3];
	int width, height, depth;

	p = getconf("vgasize");
	if(p == nil || getfields(p, f, nelem(f), 0, "x") != nelem(f) ||
	    (width = atoi(f[0])) < 16 || (height = atoi(f[1])) <= 0 ||
	    (depth = atoi(f[2])) != 32)
		return -1;
	xgscreen.r.max = Pt(width, height);
	xgscreen.depth = depth;
	return 0;
}

void
screeninit(void)
{
	uchar *fb;
	ulong fbsz;
	int width, height;

	int dw, dh;

	if(gpuprobe() < 0)
		return;		/* no virtio-gpu: stay headless */

	/*
	 * First control-queue round trip: query host scanout
	 * geometry.  If the host advertises a usable size for
	 * scanout 0 and the boot config has no explicit vgasize,
	 * adopt it; otherwise fall back to the compiled default.
	 */
	/*
	 * No ctll/il wrapping for the bring-up commands below: this
	 * runs single-threaded from main() before schedinit, so up is
	 * nil (qlock would panic) and there is no concurrent
	 * submitter.  ctlcmd takes vgpu.il internally for the vring
	 * update, which is all the mutual exclusion needed here.
	 */
	dw = dh = 0;
	gpudisplayinfo(&dw, &dh);

	if(screensize() < 0 && dw >= 16 && dh > 0){
		xgscreen.r.max = Pt(dw, dh);
		xgscreen.depth = 32;
	}
	width = xgscreen.r.max.x;
	height = xgscreen.r.max.y;

	xgscreen.clipr = xgscreen.r;
	memsetchan(&xgscreen, XRGB32);

	/*
	 * Allocate the guest framebuffer (the softscreen devdraw
	 * renders into).  It is the backing store for the host 2D
	 * resource.  mallocalign on a page boundary keeps the
	 * single-entry scatter list simple and PADDR-contiguous.
	 */
	gscreen = &xgscreen;
	gscreen->width = wordsperline(gscreen->r, gscreen->depth);
	fbsz = gscreen->width * sizeof(ulong) * height;
	fb = mallocalign(fbsz, BY2PG, 0, 0);
	if(fb == nil){
		print("virtio-gpu: no memory for %dx%dx%d framebuffer\n",
			width, height, gscreen->depth);
		gscreen = nil;
		return;
	}
	memset(fb, 0, fbsz);
	xgdata.bdata = fb;
	xgdata.ref = 1;

	/*
	 * Create the host resource, attach backing, bind to scanout 0.
	 * Each step is logged so the boot transcript names exactly how
	 * far the lifecycle got (the helpers print their own err- line
	 * on failure; these mark the successful transitions between
	 * them).  This is the per-command trace the spec-driven debug
	 * pass asked for: create-2d -> attach-backing -> set-scanout.
	 */
	print("virtio-gpu: setup %dx%dx32 fb %#p (%lud bytes)\n",
		width, height, fb, fbsz);
	if(gpucreate2d(ResId, width, height) < 0){
		print("virtio-gpu: scanout setup failed at create-2d\n");
		gscreen = nil;
		return;
	}
	if(gpuattachbacking(ResId, fb, fbsz) < 0){
		print("virtio-gpu: scanout setup failed at attach-backing\n");
		gscreen = nil;
		return;
	}
	print("virtio-gpu: attach-backing: ok\n");
	if(gpusetscanout(0, ResId, width, height) < 0){
		print("virtio-gpu: scanout setup failed at set-scanout\n");
		gscreen = nil;
		return;
	}
	print("virtio-gpu: set-scanout: ok; scanout 0 bound to res %d\n", ResId);

	conf.monitor = 1;

	memimageinit();
	memdefont = getmemdefont();
	/*
	 * screenwin() paints the console banner into the framebuffer
	 * and calls flushmemscreen(), which is the first exercise of
	 * the TRANSFER_TO_HOST_2D + RESOURCE_FLUSH data path.  If the
	 * control lifecycle above all logged "ok" but the window stays
	 * black, the divergence is here (transfer offset/stride or the
	 * flush rectangle), not in resource setup -- the trace lets us
	 * tell the two apart on the next boot.
	 */
	print("virtio-gpu: control path up; painting console banner\n");
	screenwin();
	print("virtio-gpu: first flush issued; window should paint now\n");
	myscreenputs(kmesg.buf, kmesg.n);
	screenputs = myscreenputs;
	swcursorinit();
}

/*
 * console-on-screen plumbing (same shape as bcm/screen.c)
 */
static void
myscreenputs(char *s, int n)
{
	int i;
	Rune r;
	char buf[4];

	if(!islo()){
		if(!canlock(&screenlock))
			return;
	} else {
		while(!canlock(&screenlock))
			;
	}

	/*
	 * screenlock is a spinlock; the screenputc -> flushmemscreen
	 * calls below must therefore NOT sleep or block on a qlock.
	 * Flag the no-sleep window so cansleep()/flushmemscreen poll
	 * (with interrupts on) and best-effort the control queue.
	 */
	vgpu.nosleep++;
	while(n > 0){
		i = chartorune(&r, s);
		if(i == 0){
			s++;
			--n;
			continue;
		}
		memmove(buf, s, i);
		buf[i] = 0;
		n -= i;
		s += i;
		screenputc(buf);
	}
	vgpu.nosleep--;
	unlock(&screenlock);
}

static void
screenwin(void)
{
	char *greet;
	Memimage *orange;
	Point p, q;
	Rectangle r;

	back = memwhite;
	conscol = memblack;

	orange = allocmemimage(Rect(0, 0, 1, 1), XRGB32);
	orange->flags |= Frepl;
	orange->clipr = gscreen->r;
	memfillcolor(orange, 0xFF7F00FF);

	w = memdefont->info[' '].width;
	h = memdefont->height;

	r = insetrect(gscreen->r, 4);

	memimagedraw(gscreen, r, memblack, ZP, memopaque, ZP, S);
	window = insetrect(r, 4);
	memimagedraw(gscreen, window, memwhite, ZP, memopaque, ZP, S);

	memimagedraw(gscreen, Rect(window.min.x, window.min.y,
		window.max.x, window.min.y + h + 5 + 6), orange, ZP, nil, ZP, S);
	freememimage(orange);
	window = insetrect(window, 5);

	greet = " Plan 9 Console ";
	p = addpt(window.min, Pt(10, 0));
	q = memsubfontwidth(memdefont, greet);
	USED(q);
	memimagestring(gscreen, p, conscol, ZP, memdefont, greet);
	flushmemscreen(r);
	window.min.y += h + 6;
	curpos = window.min;
	window.max.y = window.min.y + ((window.max.y - window.min.y) / h) * h;
}

static void
scroll(void)
{
	int o;
	Point p;
	Rectangle r;

	o = Scroll*h;
	r = Rpt(window.min, Pt(window.max.x, window.max.y-o));
	p = Pt(window.min.x, window.min.y+o);
	memimagedraw(gscreen, r, gscreen, p, nil, p, S);
	flushmemscreen(r);
	r = Rpt(Pt(window.min.x, window.max.y-o), window.max);
	memimagedraw(gscreen, r, back, ZP, nil, ZP, S);
	flushmemscreen(r);

	curpos.y -= o;
}

static void
screenputc(char *buf)
{
	int w;
	uint pos;
	Point p;
	Rectangle r;
	static int *xp;
	static int xbuf[256];

	if(xp < xbuf || xp >= &xbuf[nelem(xbuf)])
		xp = xbuf;

	switch(buf[0]){
	case '\n':
		if(curpos.y + h >= window.max.y)
			scroll();
		curpos.y += h;
		screenputc("\r");
		break;
	case '\r':
		xp = xbuf;
		curpos.x = window.min.x;
		break;
	case '\t':
		p = memsubfontwidth(memdefont, " ");
		w = p.x;
		if(curpos.x >= window.max.x - Tabstop * w)
			screenputc("\n");
		pos = (curpos.x - window.min.x) / w;
		pos = Tabstop - pos % Tabstop;
		*xp++ = curpos.x;
		r = Rect(curpos.x, curpos.y, curpos.x + pos * w, curpos.y + h);
		memimagedraw(gscreen, r, back, back->r.min, nil, back->r.min, S);
		flushmemscreen(r);
		curpos.x += pos * w;
		break;
	case '\b':
		if(xp <= xbuf)
			break;
		xp--;
		r = Rect(*xp, curpos.y, curpos.x, curpos.y + h);
		memimagedraw(gscreen, r, back, back->r.min, nil, back->r.min, S);
		flushmemscreen(r);
		curpos.x = *xp;
		break;
	case '\0':
		break;
	default:
		p = memsubfontwidth(memdefont, buf);
		w = p.x;
		if(curpos.x >= window.max.x - w)
			screenputc("\n");
		*xp++ = curpos.x;
		r = Rect(curpos.x, curpos.y, curpos.x + w, curpos.y + h);
		memimagedraw(gscreen, r, back, back->r.min, nil, back->r.min, S);
		memimagestring(gscreen, curpos, conscol, ZP, memdefont, buf);
		flushmemscreen(r);
		curpos.x += w;
		break;
	}
}
