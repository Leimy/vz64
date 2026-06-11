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
		intrenable(SPI+32+d, pciinterrupt, nil, BUSUNKNOWN, "pci");
	}
}

void
pciintrenable(int tbdf, void (*f)(Ureg*, void*), void *a)
{
	Pcidev *p;
	int i;
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
			return;
		}
	}
}

void
pciintrdisable(int tbdf, void (*f)(Ureg*, void*), void *a)
{
	Pcidev *p;
	int i;
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
