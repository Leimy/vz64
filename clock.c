#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"
#include "sysreg.h"

static uvlong freq;

enum {
	Enable	= 1<<0,
	Imask	= 1<<1,
	Istatus = 1<<2,
};

void
clockshutdown(void)
{
}

static void
localclockintr(Ureg *ureg, void *)
{
	timerintr(ureg, 0);
}

void
clockinit(void)
{
	/*
	 * No PMU under Apple VZ -- writing PMCR_EL0 etc. traps.
	 * We don't need it anyway: cycles()/userspace use the
	 * virtual counter CNTVCT_EL0 (see fns.h and below).
	 *
	 * syswr(PMCR_EL0, 1<<6 | 7);
	 * syswr(PMCNTENSET, 1<<31);
	 * syswr(PMUSERENR_EL0, 1<<2);
	 */
	syswr(CNTKCTL_EL1, 1<<1);

	syswr(CNTV_TVAL_EL0, ~0UL);
	syswr(CNTV_CTL_EL0, Enable);

	if(m->machno == 0){
		freq = sysrd(CNTFRQ_EL0);
		print("timer frequency %lld Hz\n", freq);
	}

	/*
	 * we are using virtual counter register CNTVCT_EL0
	 * instead of the performance counter in userspace.
	 */
	m->cyclefreq = freq;

	intrenable(IRQcntvns, localclockintr, nil, BUSUNKNOWN, "clock");
}

void
timerset(uvlong next)
{
	uvlong now;
	long period;

	now = fastticks(nil);
	period = next - now;
	syswr(CNTV_TVAL_EL0, period);
}

uvlong
fastticks(uvlong *hz)
{
	if(hz)
		*hz = freq;
	return sysrd(CNTVCT_EL0);
}

ulong
perfticks(void)
{
	return fastticks(nil);
}

ulong
µs(void)
{
	uvlong hz;
	uvlong t = fastticks(&hz);
	return (t * 1000000ULL) / hz;
}

void
microdelay(int n)
{
	ulong now;

	now = µs();
	while(µs() - now < n);
}

void
delay(int n)
{
	while(--n >= 0)
		microdelay(1000);
}

/*
 * Barrier across all cpus.  Called by cpu0 at the end of mpinit
 * and by every secondary as it comes up.  HISTORICAL TRAP: if any
 * cpu dies (panics/exits) before reaching its synccycles, the
 * survivors spin here forever with NO console output -- the
 * classic "SMP boot just hangs" with nothing to show for it.
 *
 * To make that visible, the spins are bounded by a loop count and,
 * if a cpu waits too long, it announces which barrier it is stuck
 * on and how many cpus it is still waiting for (so you can tell
 * "cpu1 never arrived" from "cpu0 never released us").  CNTVCT_EL0
 * is up by the time synccycles runs (clockinit ran first on both
 * paths), so we could time it, but a raw spin count needs no timer
 * and works even if the virtual counter is misbehaving.  The
 * threshold is deliberately huge; a healthy barrier closes in
 * microseconds, so reaching it always means something is wrong.
 */
static void
syncwait(Ref *r, char *which)
{
	uvlong i;
	int complained;

	complained = 0;
	for(i = 0; r->ref != conf.nmach; i++){
		if(i >= 2000000000ULL && !complained){
			iprint("synccycles: cpu%d stuck at %s barrier:"
				" have %ld of %ld cpus\n",
				m->machno, which, r->ref, conf.nmach);
			complained = 1;
			i = 0;
		}
		coherence();
	}
}

void
synccycles(void)
{
	static Ref r1, r2;
	int s;

	s = splhi();
	r2.ref = 0;
	incref(&r1);
	syncwait(&r1, "enter");
//	syswr(PMCR_EL0, 1<<6 | 7);
	incref(&r2);
	syncwait(&r2, "leave");
	r1.ref = 0;
	splx(s);
}
