#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"
#include "ureg.h"
#include "sysreg.h"
#include "../port/error.h"

enum {
	GICD_CTLR	= 0x000/4,	/* RW, Distributor Control Register */
	GICD_TYPER	= 0x004/4,	/* RO, Interrupt Controller Type */
	GICD_IIDR	= 0x008/4,	/* RO, Distributor Implementer Identification Register */

	GICD_IGROUPR0	= 0x080/4,	/* RW, Interrupt Group Registers (0x80-0xBC) */

	GICD_ISENABLER0	= 0x100/4,	/* RW, Interrupt Set-Enable Registers (0x100-0x13C) */
	GICD_ICENABLER0	= 0x180/4,	/* RW, Interrupt Clear-Enable Registers (0x180-0x1BC) */

	GICD_ISPENDR0	= 0x200/4,	/* RW, Interrupt Set-Pending Registers (0x200-0x23C) */
	GICD_ICPENDR0	= 0x280/4,	/* RW, Interrupt Clear-Pending Registers (0x280-0x2BC) */

	GICD_ISACTIVER0	= 0x300/4,	/* RW, Interrupt Set-Active Registers (0x300-0x33C) */
	GICD_ICACTIVER0 = 0x380/4,	/* RW, Interrupt Clear-Active Registers (0x380-0x3BC) */

	GICD_IPRIORITYR0= 0x400/4,	/* RW, Interrupt Priority Registers (0x400-0x5FC) */
	GICD_TARGETSR0	= 0x800/4,	/* RW, Interrupt Target Registers (0x800-0x9FC) */
	GICD_ICFGR0	= 0xC00/4,	/* RW, Interrupt Configuration Registers (0xC00-0xC7C) */

	GICD_ISR0	= 0xD00/4,
	GICD_PPISR	= GICD_ISR0,	/* RO, Private Peripheral Interrupt Status Register */
	GICD_SPISR0	= GICD_ISR0+1,	/* RO, Shared Peripheral Interrupt Status Register */
	GICD_SGIR	= 0xF00/4,	/* WO, Software Generated Interrupt Register */

	GICD_CPENDSGIR0	= 0xF10/4,	/* RW, SGI Clear-Pending Registers (0xF10-0xF1C) */
	GICD_SPENDSGIR0	= 0xF20/4,	/* RW, SGI Set-Pending Registers (0xF20-0xF2C) */

	GICD_PIDR4	= 0xFD0/4,	/* RO, Perpheral ID Registers */
	GICD_PIDR5	= 0xFD4/4,
	GICD_PIDR6	= 0xFD8/4,
	GICD_PIDR7	= 0xFDC/4,
	GICD_PIDR0	= 0xFE0/4,
	GICD_PIDR1	= 0xFE4/4,
	GICD_PIDR2	= 0xFE8/4,
	GICD_PIDR3	= 0xFEC/4,

	GICD_CIDR0	= 0xFF0/4,	/* RO, Component ID Registers */
	GICD_CIDR1	= 0xFF4/4,
	GICD_CIDR2	= 0xFF8/4,
	GICD_CIDR3	= 0xFFC/4,

	RD_base		= 0x00000,
	GICR_CTLR	= (RD_base+0x000)/4,
	GICR_IIDR	= (RD_base+0x004)/4,
	GICR_TYPER	= (RD_base+0x008)/4,
	GICR_STATUSR	= (RD_base+0x010)/4,
	GICR_WAKER	= (RD_base+0x014)/4,
	GICR_SETLPIR	= (RD_base+0x040)/4,
	GICR_CLRLPIR	= (RD_base+0x048)/4,
	GICR_PROPBASER	= (RD_base+0x070)/4,
	GICR_PENDBASER	= (RD_base+0x078)/4,
	GICR_INVLPIR	= (RD_base+0x0A0)/4,
	GICR_INVALLR	= (RD_base+0x0B0)/4,
	GICR_SYNCR	= (RD_base+0x0C0)/4,

	SGI_base	= 0x10000,
	GICR_IGROUPR0	= (SGI_base+0x080)/4,
	GICR_ISENABLER0	= (SGI_base+0x100)/4,
	GICR_ICENABLER0	= (SGI_base+0x180)/4,
	GICR_ISPENDR0	= (SGI_base+0x200)/4,
	GICR_ICPENDR0	= (SGI_base+0x280)/4,
	GICR_ISACTIVER0	= (SGI_base+0x300)/4,
	GICR_ICACTIVER0	= (SGI_base+0x380)/4,
	GICR_IPRIORITYR0= (SGI_base+0x400)/4,
	GICR_ICFGR0	= (SGI_base+0xC00)/4,
	GICR_ICFGR1	= (SGI_base+0xC04)/4,
	GICR_IGRPMODR0	= (SGI_base+0xD00)/4,
	GICR_NSACR	= (SGI_base+0xE00)/4,
};

typedef struct Vctl Vctl;
struct Vctl {
	Vctl	*next;
	void	(*f)(Ureg*, void*);
	void	*a;
	int	irq;
	u32int	intid;
};

static Lock vctllock;
static Vctl *vctl[MAXMACH][32], *vfiq;
static u32int *dregs = (u32int*)VIRTIO;

/*
 * Pack an MPIDR_EL1 value into the 32-bit affinity layout used by
 * GICR_TYPER[63:32] (Aff3<<24 | Aff2<<16 | Aff1<<8 | Aff0).  MPIDR
 * keeps Aff3 up at bits [39:32]; the GIC affinity word keeps it at
 * [31:24], so it has to be shifted down separately from Aff2..0.
 */
static u32int
mpidraff(uvlong mpidr)
{
	return (u32int)((mpidr & 0xFFFFFFULL)		/* Aff2:Aff1:Aff0 */
		| ((mpidr >> 32) & 0xFFULL) << 24);	/* Aff3 */
}

/*
 * Find the GIC redistributor for a given cpu.
 *
 * On a conforming GICv3 each redistributor is identified by the
 * MPIDR affinity the GIC reports in GICR_TYPER[63:32], and that is
 * our preferred match (works on real hardware and on qemu).
 *
 * Apple VZ does not cooperate: its redistributor GICR_TYPER reads
 * back as ALL ZEROES for every cpu -- both the affinity word
 * [63:32] and the low word (Processor_Number, Last, VLPIS) are 0
 * (see the dumprregs output: cpu0 and cpu1 both show aff 0,
 * typer.lo 0).  So neither affinity nor processor-number nor the
 * Last bit can tell the frames apart.  The frames are nonetheless
 * mapped one-per-cpu, in cpu order, at a fixed GICRFRAME stride,
 * with the region terminated by unmapped all-ones GIC space (the
 * frame just past the last cpu reads 0xffffffff).
 *
 * Therefore getrregs walks the redistributor region in GICRFRAME
 * steps and, when affinity matching turns up nothing (the VZ
 * case), falls back to positional indexing: cpu N gets the Nth
 * mapped frame.  That is exactly how GICv3 lays consecutive
 * redistributors out and matches the observed VZ layout
 * (frame0=cpu0, frame1=cpu1, frame2 unmapped).
 *
 * getrregs is only ever called for the currently-running cpu (both
 * intrinit and the PPI/SGI path of intrenable run on the cpu being
 * set up), so the positional index is just this cpu's machno.
 *
 * The walk is bounded by the redistributor region the device tree
 * advertises (gicrbase/gicrsize) when available, falling back to
 * the compiled-in PHYSIO offset otherwise, so it can never run off
 * the end of the mapped device window.
 */
enum {
	GICRFRAME = 0x20000,	/* one RD+SGI pair (smallest redistributor) */
};

/*
 * Debug aid: walk the whole advertised redistributor region and
 * print one line per frame so we can see exactly what layout VZ
 * presents (frame offset, the GICR_TYPER affinity word, the low
 * TYPER word with its Last/VLPIS bits, and whether the frame reads
 * back as all-ones unmapped GIC space).  Bounded by the same region
 * getrregs() uses.  Called once from cpu0's intrinit, and again
 * from getrregs() just before it panics, so a failed secondary
 * bringup leaves a full map in the log.
 */
static void
rregsbase(u32int **basep, uintptr *limp)
{
	if(gicrsize != 0 && gicrbase >= PHYSIO){
		*basep = (u32int*)(VIRTIO + (gicrbase - PHYSIO));
		*limp = gicrsize;
	} else {
		*basep = (u32int*)(VIRTIO + 0x10000);
		*limp = PHYSIOEND - (PHYSIO + 0x10000);
	}
}

void
dumprregs(void)
{
	u32int *rregs, *base;
	u32int lo, aff;
	uintptr off, lim;
	int n;

	rregsbase(&base, &lim);
	iprint("dumprregs: cpu%d mpidr %#llux want-aff %#ux base %#p lim %#p"
		" (gicrbase %#llux gicrsize %#llux)\n",
		m->machno, sysrd(MPIDR_EL1), mpidraff(sysrd(MPIDR_EL1)),
		base, (void*)lim, gicrbase, gicrsize);
	n = 0;
	for(off = 0; off + GICRFRAME <= lim; off += GICRFRAME){
		rregs = (u32int*)((uintptr)base + off);
		lo = rregs[GICR_TYPER];
		aff = rregs[GICR_TYPER + 1];
		iprint("  frame %2d off %#p aff %#ux typer.lo %#ux%s%s%s\n",
			n, (void*)off, aff, lo,
			(lo & (1<<4)) ? " Last" : "",
			(lo & (1<<0)) ? " PLPIS" : "",
			(lo & (1<<1)) ? " VLPIS" : "");
		n++;
		if(lo == ~0u && aff == ~0u){
			iprint("  ... frame reads all-ones (unmapped); stopping\n");
			break;
		}
		if(lo & (1<<4))			/* Last */
			break;
	}
}

static u32int*
getrregs(int machno)
{
	u32int *rregs, *base;
	u32int want, aff, typer;
	uintptr off, lim;
	int nframes;

	/*
	 * Prefer the DTB-advertised redistributor region; fall
	 * back to the compiled-in PHYSIO offset if the device tree
	 * did not give us one.  Either way the region must lie in
	 * the mapped device window [PHYSIO, PHYSIOEND).
	 */
	rregsbase(&base, &lim);

	/*
	 * First try to match by MPIDR affinity, the architecturally
	 * correct discriminator (GICR_TYPER[63:32]).  This is what
	 * works on real GICv3 and is kept as the preferred path.
	 *
	 * Under Apple VZ, however, the redistributor TYPER affinity
	 * word reads back as 0 for *every* frame (see dumprregs:
	 * cpu0 and cpu1 both report aff 0, typer.lo 0).  The frames
	 * are nonetheless laid out one-per-cpu in cpu order at a
	 * fixed GICRFRAME stride, terminated by unmapped all-ones
	 * GIC space.  So if affinity matching fails we fall back to
	 * indexing the Nth mapped frame for cpu N -- which is also
	 * how the GICv3 architecture lays consecutive redistributors
	 * out, and matches the observed VZ layout (frame0=cpu0,
	 * frame1=cpu1, frame2 unmapped).
	 */
	want = mpidraff(sysrd(MPIDR_EL1));
	aff = typer = 0;
	nframes = 0;
	for(off = 0; off + GICRFRAME <= lim; off += GICRFRAME){
		rregs = (u32int*)((uintptr)base + off);
		typer = rregs[GICR_TYPER];
		aff = rregs[GICR_TYPER + 1];
		if(typer == ~0u && aff == ~0u)	/* unmapped GIC space */
			break;
		if(aff != 0 && aff == want)	/* real affinity match */
			return rregs;
		nframes++;
		if(typer & (1<<4))		/* Last */
			break;
	}

	/*
	 * Affinity match failed (VZ zeroes the field).  Fall back to
	 * positional indexing: cpu N uses the Nth redistributor.
	 *
	 * The per-cpu redistributor size (stride) is GICRFRAME unless
	 * the device tree advertised a redistributor region whose size
	 * divides evenly by the cpu count -- in which case that quotient
	 * is the authoritative stride (it accounts for GICv4/VLPI
	 * layouts where each cpu's redistributor is 0x40000, not
	 * 0x20000).  This is the guest-visible equivalent of Apple's
	 * hv_gic_get_redistributor_size(): total region / ncpu.
	 */
	{
		uintptr stride = GICRFRAME;

		if(gicrsize != 0 && conf.nmach > 0){
			uintptr s = (uintptr)(gicrsize / conf.nmach);
			if(s >= GICRFRAME && (s % GICRFRAME) == 0)
				stride = s;
		}
		off = (uintptr)machno * stride;
		if(machno >= 0 && off + GICRFRAME <= lim){
			rregs = (u32int*)((uintptr)base + off);
			typer = rregs[GICR_TYPER];
			aff = rregs[GICR_TYPER + 1];
			if(!(typer == ~0u && aff == ~0u))
				return rregs;
		}
	}

	iprint("getrregs cpu%d want %#ux: %d mapped frames, last aff %#ux typer %#ux\n",
		machno, want, nframes, aff, typer);
	dumprregs();
	panic("getrregs: no re-distributor for cpu %d (aff %#ux)\n", machno, want);
}

void
intrcpushutdown(void)
{
	/* disable cpu interface */
	syswr(ICC_IGRPEN0_EL1, 0);
	syswr(ICC_IGRPEN1_EL1, 0);
	coherence();
}

void
intrsoff(void)
{
	/* disable distributor */
	dregs[GICD_CTLR] = 0;
	coherence();
	while(dregs[GICD_CTLR]&(1<<31))
		;
}

void
intrinit(void)
{
	u32int *rregs;
	int i, n;

	if(m->machno == 0){
		intrsoff();

		/* clear all interrupts */
		n = ((dregs[GICD_TYPER] & 0x1F)+1) << 5;
		for(i = 32; i < n; i += 32){
			dregs[GICD_IGROUPR0 + (i/32)] = -1;

			dregs[GICD_ISENABLER0 + (i/32)] = -1;
			while(dregs[GICD_CTLR]&(1<<31))
				;
			dregs[GICD_ICENABLER0 + (i/32)] = -1;
			while(dregs[GICD_CTLR]&(1<<31))
				;
			dregs[GICD_ICACTIVER0 + (i/32)] = -1;
		}
		for(i = 0; i < n; i += 4){
			dregs[GICD_IPRIORITYR0 + (i/4)] = 0;
			dregs[GICD_TARGETSR0 + (i/4)] = 0;
		}
		for(i = 32; i < n; i += 16){
			dregs[GICD_ICFGR0 + (i/16)] = 0;
		}
		coherence();
		while(dregs[GICD_CTLR]&(1<<31))
			;
		dregs[GICD_CTLR] = (1<<0) | (1<<1) | (1<<4);

		/*
		 * One-time map of the redistributor layout VZ presents.
		 * Quiet by default now that SMP works; set *gicdebug in
		 * the kernel config to print it (and the per-frame walk)
		 * for the next bring-up that needs it.
		 */
		if(getconf("*gicdebug") != nil)
			dumprregs();
	}

	rregs = getrregs(m->machno);

	/*
	 * Wake this cpu's redistributor before touching its SGI/PPI
	 * frame.  A secondary brought up by PSCI CPU_ON comes out of
	 * reset with GICR_WAKER.ProcessorSleep set; until it is
	 * cleared (and ChildrenAsleep observed low) the redistributor
	 * ignores SGI/PPI configuration.  cpu0 is already awake, so
	 * this is a harmless no-op there.  WAKER bits: ProcessorSleep
	 * = bit 1, ChildrenAsleep = bit 2.
	 */
	rregs[GICR_WAKER] &= ~(1<<1);
	coherence();
	while(rregs[GICR_WAKER] & (1<<2))
		;

	n = 32;
	for(i = 0; i < n; i += 32){
		rregs[GICR_IGROUPR0 + (i/32)] = -1;

		rregs[GICR_ISENABLER0 + (i/32)] = -1;
		while(rregs[GICR_CTLR]&(1<<3))
			;
		rregs[GICR_ICENABLER0 + (i/32)] = -1;
		while(dregs[GICD_CTLR]&(1<<31))
			;
		rregs[GICR_ICACTIVER0 + (i/32)] = -1;
	}
	for(i = 0; i < n; i += 4){
		rregs[GICR_IPRIORITYR0 + (i/4)] = 0;
	}
	coherence();
	while(rregs[GICR_CTLR]&(1<<3))
		;

	coherence();

	/* enable cpu interface */
	syswr(ICC_CTLR_EL1, 0);
	syswr(ICC_BPR1_EL1, 7);
	syswr(ICC_PMR_EL1, 0xFF);

	coherence();
}


/*
 *  called by trap to handle irq interrupts.
 *  returns true iff a clock interrupt, thus maybe reschedule.
 */
int
irq(Ureg* ureg)
{
	Vctl *v;
	int clockintr;
	u32int intid;

	m->intr++;
	intid = sysrd(ICC_IAR1_EL1) & 0xFFFFFF;
// iprint("i<%d>", intid);
	if((intid & ~3) == 1020)
		return 0; // spurious
	clockintr = 0;
	for(v = vctl[m->machno][intid%32]; v != nil; v = v->next)
		if(v->intid == intid){
			coherence();
			v->f(ureg, v->a);
			coherence();
			if(v->irq == IRQcntvns)
				clockintr = 1;
		}
	coherence();
	syswr(ICC_EOIR1_EL1, intid);
	return clockintr;
}

/*
 * called direct from lexception.s to handle fiq interrupt.
 */
void
fiq(Ureg *ureg)
{
	Vctl *v;
	u32int intid;

	m->intr++;
	intid = sysrd(ICC_IAR1_EL1) & 0xFFFFFF;
// iprint("f<%d>", intid);
	if((intid & ~3) == 1020)
		return;	// spurious
	v = vfiq;
	if(v != nil && v->intid == intid && m->machno == 0){
		coherence();
		v->f(ureg, v->a);
		coherence();
	}
	syswr(ICC_EOIR1_EL1, intid);
}

void
intrenable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *)
{
	Vctl *v;
	u32int intid;
	int cpu, prio;

	if(BUSTYPE(tbdf) == BusPCI){
		pciintrenable(tbdf, f, a);
		return;
	}

	if(tbdf != BUSUNKNOWN)
		return;

	prio = 0x80;
	intid = irq;
	if((v = xalloc(sizeof(Vctl))) == nil)
		panic("intrenable: no mem");
	v->irq = irq;
	v->intid = intid;
	v->f = f;
	v->a = a;

	lock(&vctllock);
	if(intid < SPI)
		cpu = m->machno;
	else
		cpu = 0;
	if(irq == IRQfiq){
		vfiq = v;
		prio = 0;
	}else{
		v->next = vctl[cpu][intid%32];
		vctl[cpu][intid%32] = v;
	}
	syswr(ICC_IGRPEN1_EL1, sysrd(ICC_IGRPEN1_EL1)|1);
	coherence();

	syswr(ICC_EOIR1_EL1, intid);
	coherence();

	/* setup */
	if(intid < 32){
		u32int *rregs = getrregs(cpu);
		rregs[GICR_IPRIORITYR0 + (intid/4)] |= prio << ((intid%4) << 3);
		coherence();
		rregs[GICR_ISENABLER0] = 1 << (intid%32);
		coherence();
		while(rregs[GICR_CTLR]&(1<<3))
			;
	} else {
		dregs[GICD_IPRIORITYR0 + (intid/4)] |= prio << ((intid%4) << 3);
		dregs[GICD_TARGETSR0 + (intid/4)] |= (1<<cpu) << ((intid%4) << 3);
		coherence();
		dregs[GICD_ISENABLER0 + (intid/32)] = 1 << (intid%32);
		coherence();
		while(dregs[GICD_CTLR]&(1<<31))
			;
	}
	unlock(&vctllock);
}

void
intrdisable(int, void (*f)(Ureg*, void*), void *a, int tbdf, char*)
{
	if(BUSTYPE(tbdf) == BusPCI){
		pciintrdisable(tbdf, f, a);
		return;
	}
}
