#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"

#define MMIOBASE 0x50000000	/* VZ 32-bit window 0x50000000-0x6ffdffff */
#define ECAMBASE 0x40000000	/* VZ ECAM */
#define ECAMSIZE 0x01000000	/* bus 0 only */

typedef struct Intvec Intvec;

struct Intvec {
	Pcidev *p;
	void (*f)(Ureg*, void*);
	void *a;
};

static uchar *ecam;
static Intvec vec[32];

static void*
cfgaddr(int tbdf, int rno)
{
	return ecam + (BUSBNO(tbdf)<<20 | BUSDNO(tbdf)<<15 | BUSFNO(tbdf)<<12) + rno;
}

int
pcicfgrw32(int tbdf, int rno, int data, int read)
{
	u32int *p;

	if((p = cfgaddr(tbdf, rno & ~3)) != nil){
		if(read)
			data = *p;
		else
			*p = data;
	} else {
		data = -1;
	}
	return data;
}

int
pcicfgrw16(int tbdf, int rno, int data, int read)
{
	u16int *p;

	if((p = cfgaddr(tbdf, rno & ~1)) != nil){
		if(read)
			data = *p;
		else
			*p = data;
	} else {
		data = -1;
	}
	return data;
}

int
pcicfgrw8(int tbdf, int rno, int data, int read)
{
	u8int *p;

	if((p = cfgaddr(tbdf, rno)) != nil){
		if(read)
			data = *p;
		else
			*p = data;
	} else {
		data = -1;
	}
	return data;
}

static void
pciinterrupt(Ureg *ureg, void *)
{
	int i;
	Intvec *v;

	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->f)
			v->f(ureg, v->a);
	}
}

/*
 * Per-slot SPI gating.  intrenable() for an SPI both registers the
 * handler AND enables (unmasks) the SPI at the GIC distributor.  But a
 * device's interrupt MUST NOT be enabled before a driver is ready to
 * service it: chandevreset() drives device probes with up == nil on the
 * tiny per-Mach boot stack, BEFORE schedinit(), and trap() does splflo()
 * (unmasking IRQ) on its way in.  If a device asserts its level INTx
 * the instant its PCI memory space goes live (the generic pcienable()
 * during pcibusmap, well before any driver runs) and that slot's SPI is
 * already enabled, the unhandled, unacknowledged interrupt re-fires the
 * moment splflo() unmasks and nests trap frames down the whole boot
 * stack until kenter()'s guard panics ("kenter: hppir <slot SPI>
 * lastintid 0xffffffff" -- the IRQ is pending but irq() never even runs
 * to EOI it).  This bit us as soon as a slot-5 device (intid 69) started
 * asserting during the audio bring-up window.
 *
 * Fix: register the shared pciinterrupt handler for every slot's SPI at
 * init time (so irq() can dispatch it), but leave each slot's SPI MASKED
 * at the distributor.  A slot's SPI is unmasked lazily, the first time a
 * real driver registers an INTx handler for that slot via
 * pciintrenable(), and re-masked when its last handler goes away.  A
 * device with no driver therefore can never deliver an interrupt on the
 * boot stack, no matter what it asserts when its memory space is
 * enabled.
 */
static u32int slotspienabled;	/* bitmap of slots whose SPI is unmasked */

static void
pciintrinit(void)
{
	/* VZ: each device slot d gets INTA on SPI 32+d (from DTB interrupt-map) */
	u32int seen = 0;
	Pcidev *p;
	int d;

	for(p = pcimatch(nil, 0, 0); p != nil; p = pcimatch(p, 0, 0)){
		d = BUSDNO(p->tbdf);
		if(seen & 1<<d)
			continue;
		seen |= 1<<d;
		/*
		 * Register the dispatch handler, then immediately mask the
		 * SPI: it stays off until a driver claims the slot.  (The
		 * register+enable is one operation in intrenable(); we undo
		 * the enable half right away with gicspimask.)
		 */
		intrenable(SPI+32+d, pciinterrupt, nil, BUSUNKNOWN, "pci");
		gicspimask(SPI+32+d, 1);
	}
}

void
pciintrenable(int tbdf, void (*f)(Ureg*, void*), void *a)
{
	Pcidev *p;
	int i, d;
	Intvec *v;

	if((p = pcimatchtbdf(tbdf)) == nil){
		print("pciintrenable: %T: unknown device\n", tbdf);
		return;
	}

	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->f == nil){
			v->p = p;
			v->f = f;
			v->a = a;
			/*
			 * A real driver now owns this slot, so it is safe to
			 * let the device's SPI through.  Unmask it once per
			 * slot (idempotent across multiple handlers on the
			 * same slot).
			 */
			d = BUSDNO(tbdf);
			if((slotspienabled & 1<<d) == 0){
				slotspienabled |= 1<<d;
				gicspimask(SPI+32+d, 0);
				print("pci: %T slot %d: driver claimed; "
					"SPI %d unmasked\n",
					tbdf, d, SPI+32+d);
			}
			return;
		}
	}
}

void
pciintrdisable(int tbdf, void (*f)(Ureg*, void*), void *a)
{
	Pcidev *p;
	int i, d, any;
	Intvec *v;

	if((p = pcimatchtbdf(tbdf)) == nil){
		print("pciintrdisable: %T: unknown device\n", tbdf);
		return;
	}

	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->p == p && v->f == f && v->a == a)
			v->f = nil;
	}

	/*
	 * If no handler remains for this slot, mask its SPI again so a
	 * driverless device cannot deliver interrupts.
	 */
	d = BUSDNO(tbdf);
	any = 0;
	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->f != nil && v->p != nil && BUSDNO(v->p->tbdf) == d){
			any = 1;
			break;
		}
	}
	if(!any && (slotspienabled & 1<<d) != 0){
		slotspienabled &= ~(1<<d);
		gicspimask(SPI+32+d, 1);
	}
}

static void
pcicfginit(void)
{
	char *p;
	Pcidev *pciroot;
	ulong ioa;
	uvlong base;

	fmtinstall('T', tbdffmt);

	pcimaxdno = 32;
	if(p = getconf("*pcimaxdno"))
		pcimaxdno = strtoul(p, 0, 0);

	pciscan(0, &pciroot, nil);
	if(pciroot == nil)
		return;

	/* VZ: no firmware ever wrote PciINTL; route per-slot SPIs */
	{
		Pcidev *pd;
		for(pd = pcimatch(nil, 0, 0); pd != nil; pd = pcimatch(pd, 0, 0))
			pd->intl = SPI+32+BUSDNO(pd->tbdf);
	}

	pciintrinit();

	ioa = 0;
	base = MMIOBASE;
	pcibusmap(pciroot, &base, &ioa, 1);

	if(getconf("*pcihinv"))
		pcihinv(pciroot);
}

void
pciqemulink(void)
{
	ecam = vmap(ECAMBASE, ECAMSIZE);
	pcicfginit();
}
