/*
 * virtio-console driver for Apple Virtualization.framework guests.
 *
 * Two-phase bring-up:
 *   uartconsinit (early, before PCI)
 *       Installs a null console; putc writes to a ring buffer.
 *   uartvzlink (links(), after pciqemulink has scanned PCI)
 *       Finds the virtio-console PCI device (0x1AF4:0x1043),
 *       initialises rx/tx queues, flushes the ring buffer,
 *       and switches putc/kick to real virtio I/O.
 *
 * TX: putc (polling, for iprint/panic) uses desc 0.
 *     kick (queued print path) uses desc 1.
 *     Both poll the used ring for completion.
 * RX: pre-posted buffers on receiveq, interrupt-driven via uartrecv.
 *     getc polls (for rdb).
 */
#include "u.h"
#include "../port/lib.h"
#include "../port/error.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"
#include "../port/virtio10.h"

extern PhysUart vzphysuart;

enum {
	Vrxq	= 0,
	Vtxq	= 1,

	Nrxbuf	= 16,
	Rxbufsz	= 64,
	Txbufsz	= 256,

	/* descriptor flags */
	Dnext	= 1,
	Dwrite	= 2,

	/* vring avail flags */
	Rnointerrupt = 1,

	/* struct sizes */
	VringSize	= 4,
	VdescSize	= 16,
	VusedSize	= 8,
};

typedef struct Vqueue Vqueue;
struct Vqueue
{
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

static struct {
	int	ready;

	/*
	 * Serialises the TX vring across cpus.  putc (iprint/
	 * panic) and kick (queued print output) both post to and
	 * wait on the single tx queue; without this lock two cpus
	 * can interleave descriptor/avail updates and corrupt or
	 * wedge the ring.  It is an ilock because putc runs from
	 * interrupt context.
	 */
	Lock	txl;

	Pcidev	*pci;

	Vio	cfg;
	Vio	isr;

	u32int	notifyoffmult;

	Vqueue	rx;
	Vqueue	tx;

	char	rxbufs[Nrxbuf][Rxbufsz];
	char	putcbuf[1];
	char	kickbuf[Txbufsz];
} vcon;

/* ring buffer for output before virtio is ready */
static char ringbuf[16*1024];
static ulong ringw;

static Uart vzuart = {
	.regs	= nil,
	.name	= "uart0",
	.freq	= 24*Mhz,
	.baud	= 115200,
	.phys	= &vzphysuart,
};

static Uart*
pnp(void)
{
	return &vzuart;
}

/*
 * queue management — follows ethervirtio10.c patterns
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

	return 0;
}

/*
 * TX — polling, synchronous
 *
 * putc uses desc 0 + putcbuf (single character, for iprint/panic).
 * kick uses desc 1 + kickbuf (batch, for normal print path).
 *
 * Both poll the used ring for completion before returning,
 * so descriptors are always free on entry.
 */
static void
txpost(Vqueue *q, int descid, void *buf, int len)
{
	q->desc[descid].addr = PADDR(buf);
	q->desc[descid].len = len;
	q->desc[descid].flags = 0;
	q->desc[descid].next = 0;

	q->availent[q->avail->idx & q->qmask] = descid;
	coherence();
	q->avail->idx++;
	coherence();
	vout16(&q->notify, 0, Vtxq);
}

static void
txwait(Vqueue *q)
{
	/*
	 * Wait for the device to consume everything we posted.
	 * avail->idx is ours, used->idx is the device's; both are
	 * free-running u16 counters, equal when the queue is idle.
	 * Without a real wait here, putc/kick reuse the descriptor
	 * and buffer while the device is still reading them, which
	 * duplicates and corrupts output (and can wedge the queue).
	 * coherence() doubles as a compiler barrier so the loads
	 * are not cached in a register.
	 */
	while(q->used->idx != q->avail->idx)
		coherence();
	q->lastused = q->used->idx;
}

/*
 * RX — interrupt-driven
 */
static void
rxpost(int i)
{
	Vqueue *q = &vcon.rx;

	q->desc[i].addr = PADDR(vcon.rxbufs[i]);
	q->desc[i].len = Rxbufsz;
	q->desc[i].flags = Dwrite;
	q->desc[i].next = 0;

	q->availent[q->avail->idx & q->qmask] = i;
	coherence();
	q->avail->idx++;
}

static void
rxfill(void)
{
	int i, n;

	n = vcon.rx.qsize;
	if(n > Nrxbuf)
		n = Nrxbuf;
	for(i = 0; i < n; i++)
		rxpost(i);
	coherence();
	vout16(&vcon.rx.notify, 0, Vrxq);
}

static void
interrupt(Ureg*, void*)
{
	Vqueue *q;
	int id;
	u32int len;
	uint i;

	if(vin8(&vcon.isr, 0) & 1){
		q = &vcon.rx;
		while(q->lastused != q->used->idx){
			id = q->usedent[q->lastused & q->qmask].id;
			len = q->usedent[q->lastused & q->qmask].len;
			for(i = 0; i < len; i++)
				uartrecv(&vzuart, vcon.rxbufs[id][i]);
			q->lastused++;
			rxpost(id);
		}
		coherence();
		vout16(&q->notify, 0, Vrxq);
	}
}

/*
 * PhysUart operations
 */
static void
enable(Uart*, int)
{
}

static void
disable(Uart*)
{
}

static void
kick(Uart *uart)
{
	int n;

	if(!vcon.ready){
		for(;;){
			if(uart->op >= uart->oe && uartstageoutput(uart) == 0)
				break;
			ringbuf[ringw++ % sizeof(ringbuf)] = *(uart->op++);
		}
		return;
	}

	ilock(&vcon.txl);
	n = 0;
	for(;;){
		if(uart->op >= uart->oe && uartstageoutput(uart) == 0)
			break;
		vcon.kickbuf[n++] = *(uart->op++);
		if(n >= Txbufsz){
			txwait(&vcon.tx);
			txpost(&vcon.tx, 1, vcon.kickbuf, n);
			txwait(&vcon.tx);
			n = 0;
		}
	}
	if(n > 0){
		txwait(&vcon.tx);
		txpost(&vcon.tx, 1, vcon.kickbuf, n);
		txwait(&vcon.tx);
	}
	iunlock(&vcon.txl);
}

static void
dobreak(Uart*, int)
{
}

static int
baud(Uart *uart, int n)
{
	if(n <= 0)
		return -1;
	uart->baud = n;
	return 0;
}

static int
bits(Uart *uart, int n)
{
	uart->bits = n;
	return 0;
}

static int
stop(Uart *uart, int n)
{
	uart->stop = n;
	return 0;
}

static int
parity(Uart *uart, int n)
{
	uart->parity = n;
	return 0;
}

static void
donothing(Uart*, int)
{
}

static void
rts(Uart*, int)
{
}

static int
getc(Uart*)
{
	Vqueue *q;
	int id;
	u32int len;

	if(!vcon.ready)
		return -1;

	q = &vcon.rx;
	while(q->lastused == q->used->idx)
		;
	id = q->usedent[q->lastused & q->qmask].id;
	len = q->usedent[q->lastused & q->qmask].len;
	q->lastused++;
	rxpost(id);
	coherence();
	vout16(&q->notify, 0, Vrxq);

	/* return first byte; discard rest (getc is one char at a time) */
	return (uchar)vcon.rxbufs[id][0];
}

static void
putc(Uart*, int c)
{
	if(!vcon.ready){
		ringbuf[ringw++ % sizeof(ringbuf)] = c;
		return;
	}
	ilock(&vcon.txl);
	txwait(&vcon.tx);
	vcon.putcbuf[0] = c;
	txpost(&vcon.tx, 0, vcon.putcbuf, 1);
	txwait(&vcon.tx);
	iunlock(&vcon.txl);
}

void
uartconsinit(void)
{
	consuart = &vzuart;
	consuart->console = 1;
}

/*
 * PCI capability walking — same as ethervirtio10.c
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
 * uartvzlink — called from links() after PCI is up.
 * Probes for virtio-console (0x1AF4:0x1043), initialises
 * queues, flushes the ring buffer, enables RX interrupts.
 */
void
uartvzlink(void)
{
	Pcidev *p;
	Vio cfg;
	Vqueue *q;
	int cap, n, i;

	for(p = nil; p = pcimatch(p, 0x1AF4, 0x1043);){
		if(p->rid == 0)
			continue;
		if((cap = virtiocap(p, 1)) < 0)
			continue;
		if(virtiomapregs(p, cap, Vconf_sz, &cfg) == nil)
			continue;

		vcon.pci = p;
		vcon.cfg = cfg;
		pcienable(p);

		if(virtiomapregs(p, virtiocap(p, 3), 0, &vcon.isr) == nil)
			goto Bad;
		cap = virtiocap(p, 2);
		{
			Vio notifybase;
			if(virtiomapregs(p, cap, 0, &notifybase) == nil)
				goto Bad;
			vcon.notifyoffmult = pcicfgr32(p, cap+16);

			/* reset device */
			coherence();
			vout8(&cfg, Vconf_status, 0);
			while(vin8(&cfg, Vconf_status) != 0)
				delay(1);
			vout8(&cfg, Vconf_status, Sacknowledge|Sdriver);

			/* negotiate features — just VERSION_1 */
			vout32(&cfg, Vconf_devfeatsel, 1);
			vout32(&cfg, Vconf_drvfeatsel, 1);
			vout32(&cfg, Vconf_drvfeat, vin32(&cfg, Vconf_devfeat) & Fversion1);
			vout32(&cfg, Vconf_devfeatsel, 0);
			vout32(&cfg, Vconf_drvfeatsel, 0);
			vout32(&cfg, Vconf_drvfeat, 0);

			vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | Sfeaturesok);

			/* init rx and tx queues */
			for(i = 0; i < 2; i++){
				q = (i == Vrxq) ? &vcon.rx : &vcon.tx;
				vout16(&cfg, Vconf_queuesel, i);
				n = vin16(&cfg, Vconf_queuesize);
				if(n == 0 || (n & (n-1)) != 0){
					print("uartvz: queue %d bad size %d\n", i, n);
					goto Bad;
				}
				if(initqueue(q, n) < 0){
					print("uartvz: queue %d alloc failed\n", i);
					goto Bad;
				}

				q->notify = notifybase;
				if(q->notify.type == Vio_mem)
					q->notify.mem += vcon.notifyoffmult
						* vin16(&cfg, Vconf_queuenotifyoff);
				else
					q->notify.port += vcon.notifyoffmult
						* vin16(&cfg, Vconf_queuenotifyoff);

				coherence();
				vout64(&cfg, Vconf_queuedesc, PADDR(q->desc));
				vout64(&cfg, Vconf_queueavail, PADDR(q->avail));
				vout64(&cfg, Vconf_queueused, PADDR(q->used));
			}

			/* enable queues */
			for(i = 0; i < 2; i++){
				vout16(&cfg, Vconf_queuesel, i);
				vout16(&cfg, Vconf_queueenable, 1);
			}

			/* driver ok */
			vout8(&cfg, Vconf_status,
				vin8(&cfg, Vconf_status) | Sdriverok);

			/* post rx buffers */
			rxfill();

			/* enable interrupt */
			pcisetbme(p);

			vcon.ready = 1;

			/* flush ring buffer through real console */
			if(ringw > 0){
				ulong total, i, n;

				total = ringw;
				if(total > sizeof(ringbuf))
					total = sizeof(ringbuf);
				n = 0;
				for(i = ringw - total; i < ringw; i++){
					vcon.kickbuf[n++] = ringbuf[i % sizeof(ringbuf)];
					if(n >= Txbufsz){
						txpost(&vcon.tx, 1, vcon.kickbuf, n);
						txwait(&vcon.tx);
						n = 0;
					}
				}
				if(n > 0){
					txpost(&vcon.tx, 1, vcon.kickbuf, n);
					txwait(&vcon.tx);
				}
			}

			intrenable(p->intl, interrupt, nil, p->tbdf,
				"virtio-console");

		}
		return;
Bad:
		pcidisable(p);
	}
	/* no device found — stay on null console */
	print("uartvz: no virtio-console device found\n");
}

PhysUart vzphysuart = {
	.name		= "vz",
	.pnp		= pnp,
	.enable		= enable,
	.disable	= disable,
	.kick		= kick,
	.dobreak	= dobreak,
	.baud		= baud,
	.bits		= bits,
	.stop		= stop,
	.parity		= parity,
	.modemctl	= donothing,
	.rts		= rts,
	.dtr		= donothing,
	.fifo		= donothing,
	.getc		= getc,
	.putc		= putc,
};
