#include "u.h"
#include "tos.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "pool.h"
#include "io.h"
#include "sysreg.h"
#include "ureg.h"

#include "rebootcode.i"

Conf conf;

int
isaconfig(char *, int, ISAConf *)
{
	return 0;
}

/*
 *  starting place for first process
 */
void
init0(void)
{
	char buf[2*KNAMELEN], **sp;

	chandevinit();

	if(!waserror()){
		snprint(buf, sizeof(buf), "%s %s", "ARM64", conffile);
		ksetenv("terminal", buf, 0);
		ksetenv("cputype", "arm64", 0);
		if(cpuserver)
			ksetenv("service", "cpu", 0);
		else
			ksetenv("service", "terminal", 0);
		setconfenv();
		poperror();
	}
	kproc("alarm", alarmkproc, 0);

	sp = (char**)(USTKTOP-sizeof(Tos) - 8 - sizeof(sp[0])*4);
	sp[3] = sp[2] = sp[1] = nil;
	strcpy(sp[1] = (char*)&sp[4], "boot");
	sp[0] = (void*)&sp[1];

	splhi();
	fpukexit(nil);
	touser((uintptr)sp);
}

void
confinit(void)
{
	int userpcnt;
	ulong kpages;
	char *p;
	int i;

	conf.nmach = 1;
	if(p = getconf("*ncpu"))
		conf.nmach = strtol(p, 0, 0);
	if(conf.nmach > MAXMACH)
		conf.nmach = MAXMACH;

	if(p = getconf("service")){
		if(strcmp(p, "cpu") == 0)
			cpuserver = 1;
		else if(strcmp(p,"terminal") == 0)
			cpuserver = 0;
	}

	if(p = getconf("*kernelpercent"))
		userpcnt = 100 - strtol(p, 0, 0);
	else
		userpcnt = 0;

	if(userpcnt < 10)
		userpcnt = 60 + cpuserver*10;

	conf.npage = 0;
	for(i = 0; i < nelem(conf.mem); i++)
		conf.npage += conf.mem[i].npage;

	kpages = conf.npage - (conf.npage*userpcnt)/100;
	if(kpages > ((uintptr)-VDRAM)/BY2PG)
		kpages = ((uintptr)-VDRAM)/BY2PG;

	conf.upages = conf.npage - kpages;
	conf.ialloc = (kpages/2)*BY2PG;

	/* set up other configuration parameters */
	conf.nproc = 100 + ((conf.npage*BY2PG)/MB)*5;
	if(cpuserver)
		conf.nproc *= 3;
	if(conf.nproc > 4000)
		conf.nproc = 4000;
	conf.nswap = conf.npage*3;
	conf.nswppo = 4096;
	conf.nimage = 200;

	conf.copymode = conf.nmach > 1;

	/*
	 * Guess how much is taken by the large permanent
	 * datastructures. Mntcache and Mntrpc are not accounted for.
	 */
	kpages = conf.npage - conf.upages;
	kpages *= BY2PG;
	kpages -= conf.upages*sizeof(Page)
		+ conf.nproc*sizeof(Proc*)
		+ conf.nimage*sizeof(Image)
		+ conf.nswap
		+ conf.nswppo*sizeof(Page*);
	mainmem->maxsize = kpages;
	imagmem->maxsize = kpages;
}

void
machinit(void)
{
	m->ticks = 1;
	m->perf.period = 1;
	active.machs[m->machno] = 1;
}

void
mpinit(void)
{
	extern void _start(void);
	uvlong mpid, self;
	int i;

	self = sysrd(MPIDR_EL1) & 0xFF00FFFFFFULL;	/* aff3..aff0, drop flags */

	for(i = 1; i < conf.nmach; i++){
		Ureg u = {0};

		MACHP(i)->machno = i;
		cachedwbinvse(MACHP(i), MACHSIZE);

		/*
		 * target_cpu is an MPIDR affinity value.  Prefer the
		 * one the device tree advertised for this cpu; the
		 * Apple/VZ affinity layout does not necessarily put
		 * the cpu index in aff0, so synthesising aff0 = i
		 * (as the qemu port does) can name the wrong cpu or
		 * none at all.  Fall back to aff0 = i only if the DTB
		 * gave us nothing.
		 */
		if(i < ncpumpid)
			mpid = cpumpid[i] & 0xFF00FFFFFFULL;
		else
			mpid = (self & ~0xFFULL) | i;

		u.r0 = 0xC4000003;	/* CPU_ON (SMC64: 64-bit entry arg; VZ rejects the SMC32 0x84000003 with NOT_SUPPORTED) */
		u.r1 = mpid;
		u.r2 = PADDR(_start);
		u.r3 = i;		/* context_id: secondary's machno, passed in R0 */
		if(getconf("*smpdebug") != nil)
			iprint("mpinit: CPU_ON cpu%d mpid %#llux (dtb cpumpid[%d] %#llux ncpumpid %d) entry %#llux\n",
				i, mpid, i, (i < ncpumpid) ? cpumpid[i] : 0ULL,
				ncpumpid, (uvlong)PADDR(_start));
		hvccall(&u);
		if(u.r0 != 0)
			print("mpinit: PSCI CPU_ON cpu%d (mpid %#llux) failed: %lld\n",
				i, mpid, (vlong)u.r0);
	}
	synccycles();
}

void
cpuidprint(void)
{
	iprint("cpu%d: Apple Virtualization.framework\n", m->machno);
}

void
main(uintptr dtb)
{
	/*
	 * Zero BSS (belt and braces with the l.s clear).
	 * CPU 0 ONLY: a secondary entering main() must never
	 * wipe live BSS out from under the running system.
	 * (m->machno is valid here: cpu0's Mach was zeroed by
	 * the l.s mach clear; secondaries get theirs from mpinit.)
	 */
	if(m->machno == 0){
		char *p;
		for(p = edata; p < end; p++)
			*p = 0;
	}
	machinit();
	if(m->machno){
		/*
		 * Secondary bringup.  With *smpdebug set, each stage
		 * announces itself BEFORE running, so if a secondary
		 * wedges or faults silently the last "cpuN main:" line
		 * on the console names the stage that hung (a stuck
		 * secondary otherwise produces no output at all, and
		 * can also hang cpu0 in the synccycles barrier below --
		 * see clock.c).  iprint is used: it does not depend on
		 * this cpu's clock/scheduler being up yet.  Quiet by
		 * default now that SMP boots cleanly.
		 */
		int dbg = getconf("*smpdebug") != nil;

		if(dbg) iprint("cpu%d main: trapinit\n", m->machno);
		trapinit();
		if(dbg) iprint("cpu%d main: fpuinit\n", m->machno);
		fpuinit();
		if(dbg) iprint("cpu%d main: intrinit\n", m->machno);
		intrinit();
		if(dbg) iprint("cpu%d main: clockinit\n", m->machno);
		clockinit();
		cpuidprint();
		if(dbg) iprint("cpu%d main: synccycles\n", m->machno);
		synccycles();
		if(dbg) iprint("cpu%d main: timersinit\n", m->machno);
		timersinit();
		if(dbg) iprint("cpu%d main: mmu1init\n", m->machno);
		mmu1init();
		m->ticks = MACHP(0)->ticks;
		schedinit();
	}
	uartconsinit();
	quotefmtinstall();
	bootargsinit(dtb);

	meminit();
	confinit();
	xinit();
	printinit();
	print("\nPlan 9\n");
	trapinit();
	fpuinit();
	intrinit();
	clockinit();
	cpuidprint();
	timersinit();
	pageinit();
	procinit0();
	initseg();
	links();
	chandevreset();
	userinit();
	mpinit();
	mmu1init();
	schedinit();
}

void
exit(int)
{
	Ureg u = { .r0 = 0x84000002 };	/* CPU_OFF */

	cpushutdown();
	splfhi();

	if(m->machno == 0){
		/* clear secrets */
		zeroprivatepages();
		poolreset(secrmem);

		u.r0 = 0x84000009;	/* SYSTEM RESET */
	}
	hvccall(&u);
}

static void
rebootjump(void *entry, void *code, ulong size)
{
	void (*f)(void*, void*, ulong);

	intrcpushutdown();

	/* redo identity map */
	setttbr(PADDR(L1BOT));

	/* setup reboot trampoline function */
	f = (void*)REBOOTADDR;
	memmove(f, rebootcode, sizeof(rebootcode));

	cachedwbinvse(f, sizeof(rebootcode));
	cacheiinvse(f, sizeof(rebootcode));

	(*f)(entry, code, size);

	for(;;);
}

void
reboot(void*, void *code, ulong size)
{
	writeconf();
	while(m->machno != 0){
		procwired(up, 0);
		sched();
	}

	cpushutdown();
	delay(2000);

	splfhi();

	/* turn off buffered serial console */
	serialoq = nil;

	/* shutdown devices */
	chandevshutdown();

	/* stop the clock */
	clockshutdown();
	intrsoff();

	/* clear secrets */
	zeroprivatepages();
	poolreset(secrmem);

	/* off we go - never to return */
	rebootjump((void*)(KTZERO-KZERO), code, size);
}

void
dmaflush(int clean, void *p, ulong len)
{
	uintptr s = (uintptr)p;
	uintptr e = (uintptr)p + len;

	if(clean){
		s &= ~(BLOCKALIGN-1);
		e += BLOCKALIGN-1;
		e &= ~(BLOCKALIGN-1);
		cachedwbse((void*)s, e - s);
		return;
	}
	if(s & BLOCKALIGN-1){
		s &= ~(BLOCKALIGN-1);
		cachedwbinvse((void*)s, BLOCKALIGN);
		s += BLOCKALIGN;
	}
	if(e & BLOCKALIGN-1){
		e &= ~(BLOCKALIGN-1);
		if(e < s)
			return;
		cachedwbinvse((void*)e, BLOCKALIGN);
	}
	if(s < e)
		cachedinvse((void*)s, e - s);
}
