#include	"u.h"
#include	"../port/lib.h"
#include	"mem.h"
#include	"dat.h"
#include	"fns.h"

#define	MAXCONF 64
static char *confname[MAXCONF];
static char *confval[MAXCONF];
static int nconf;
static char maxmem[256];
static int cpus;
static char ncpu[256];

/*
 * MPIDR/affinity value of each cpu, taken from the `reg`
 * property of the /cpus/cpu@N device-tree nodes.  Under VZ
 * (and on real Apple silicon) the affinity layout is not
 * guaranteed to put the cpu index in aff0, so mpinit must
 * use these real values when starting secondaries with PSCI
 * CPU_ON rather than synthesising aff0 = machno.
 */
uvlong	cpumpid[MAXMACH];
int	ncpumpid;

/*
 * GIC distributor and redistributor regions, taken from the
 * `reg` property of the interrupt-controller (arm,gic-v3) node.
 * The redistributor base/size let getrregs() bound its frame
 * walk to the region the hypervisor actually advertises instead
 * of guessing the per-cpu stride (which differs between GICv3
 * and GICv4/VLPI layouts).  Zero size means "not found, fall
 * back to the compiled-in PHYSIO offset".
 */
uvlong	gicdbase, gicdsize;
uvlong	gicrbase, gicrsize;

static int
findconf(char *k)
{
	int i;

	for(i = 0; i < nconf; i++)
		if(cistrcmp(confname[i], k) == 0)
			return i;
	return -1;
}

static void
addconf(char *k, char *v)
{
	int i;

	i = findconf(k);
	if(i < 0){
		if(nconf >= MAXCONF)
			return;
		i = nconf++;
		confname[i] = k;
	}
	confval[i] = v;
}

static void
plan9iniinit(char *s, int cmdline)
{
	char *toks[MAXCONF];
	int i, c, n;
	char *v;

	if((c = *s) < ' ' || c >= 0x80)
		return;
	if(cmdline)
		n = tokenize(s, toks, MAXCONF);
	else
		n = getfields(s, toks, MAXCONF, 1, "\n");
	for(i = 0; i < n; i++){
		if(toks[i][0] == '#')
			continue;
		v = strchr(toks[i], '=');
		if(v == nil)
			continue;
		*v++ = '\0';
		addconf(toks[i], v);
	}
}

typedef struct Devtree Devtree;
struct Devtree
{
	uchar	*base;
	uchar	*end;
	char	*stab;
	char	path[1024];
};

enum {
	DtHeader	= 0xd00dfeed,
	DtBeginNode	= 1,
	DtEndNode	= 2,
	DtProp		= 3,
	DtEnd		= 9,
};

static u32int
beget4(uchar *p)
{
	return (u32int)p[0]<<24 | (u32int)p[1]<<16 | (u32int)p[2]<<8 | (u32int)p[3];
}

static uvlong
beget8(uchar *p)
{
	return (uvlong)beget4(p)<<32 | beget4(p+4);
}

/*
 * Remember which device-tree node is the GICv3 interrupt
 * controller.  We can't rely on the node *name* (VZ may call it
 * intc@, gic@, interrupt-controller@, or something Apple-specific),
 * so we latch onto the node whose `compatible` property contains
 * "gic-v3".  Properties of a node are visited in order and
 * `compatible` conventionally precedes `reg`, so by the time we
 * see this node's `reg` we have already recorded its path here.
 */
static char gicpath[1024];

static int
ingicnode(char *path)
{
	int n;

	if(gicpath[0] == 0)
		return 0;
	n = strlen(gicpath);
	return strncmp(path, gicpath, n) == 0
		&& (path[n] == 0 || path[n] == '/');
}

static int
memhas(void *buf, int len, char *s)
{
	char *p, *e;
	int n;

	n = strlen(s);
	p = buf;
	for(e = p + len - n; p <= e; p++)
		if(memcmp(p, s, n) == 0)
			return 1;
	return 0;
}

static void
devtreeprop(char *path, char *key, void *val, int len)
{
	uvlong addr;
	uchar *p = val;

	/*
	 * Identify the GIC node by its compatible string.  `compatible`
	 * is a list of NUL-separated strings; match a substring so
	 * "arm,gic-v3" (and Apple variants) are caught.
	 */
	if(strcmp(key, "compatible") == 0 && memhas(val, len, "gic-v3")){
		if(strlen(path) < sizeof(gicpath))
			strecpy(gicpath, gicpath+sizeof(gicpath), path);
		return;
	}

	/*
	 * GICv3 interrupt-controller node.  Its `reg` is a list of
	 * (base,size) regions encoded with the parent bus's
	 * #address-cells/#size-cells, which is 2/2 under VZ (64-bit
	 * cells).  Region 0 is the distributor, region 1 the
	 * redistributors.  We match the node by the compatible-string
	 * path latched above; fall back to a name match for DTBs we
	 * happened to see before `compatible`.
	 */
	if(strcmp(key, "reg") == 0
	&& (ingicnode(path)
	   || (gicpath[0] == 0
	      && (strstr(path, "/intc") != nil
	         || strstr(path, "/interrupt-controller") != nil
	         || strstr(path, "/gic") != nil)))){
		if(len >= 32){		/* 2/2 cells: two 64-bit (base,size) pairs */
			gicdbase = beget8(p+0);
			gicdsize = beget8(p+8);
			gicrbase = beget8(p+16);
			gicrsize = beget8(p+24);
		} else if(len >= 16){	/* 1/1 cells: two 32-bit (base,size) pairs */
			gicdbase = beget4(p+0);
			gicdsize = beget4(p+4);
			gicrbase = beget4(p+8);
			gicrsize = beget4(p+12);
		}
		return;
	}

	if((strncmp(path, "/memory", 7) == 0 || strncmp(path, "/memory@0", 9) == 0)
	&& strcmp(key, "reg") == 0){
		if(findconf("*maxmem") < 0 && len == 16){
			p += 4;	/* ignore */
			addr = (uvlong)beget4(p+4)<<32 | beget4(p);
			addr += beget4(p+8);
			snprint(maxmem, sizeof(maxmem), "%#llux", addr);
			addconf("*maxmem", maxmem);
		}
		return;
	}
	if(strncmp(path, "/cpus/cpu", 9) == 0 && strcmp(key, "reg") == 0){
		/*
		 * The cpu node's reg is the MPIDR affinity value.
		 * It is normally a single 32-bit cell, but a
		 * #address-cells=2 layout encodes it as two cells
		 * (big-endian, high cell first).  Record it so
		 * mpinit can target the real cpu with PSCI CPU_ON.
		 */
		if(ncpumpid < MAXMACH){
			if(len >= 8)
				cpumpid[ncpumpid] = (uvlong)beget4(p)<<32 | beget4(p+4);
			else if(len >= 4)
				cpumpid[ncpumpid] = beget4(p);
			else
				cpumpid[ncpumpid] = cpus;
			ncpumpid++;
		}
		cpus++;
		return;
	}
	if(strncmp(path, "/chosen", 7) == 0 && strcmp(key, "bootargs") == 0){
		if(len > BOOTARGSLEN)
			len = BOOTARGSLEN;
		memmove(BOOTARGS, val, len);
		plan9iniinit(BOOTARGS, 1);
		return;
	}
}

static uchar*
devtreenode(Devtree *t, uchar *p, char *cp)
{
	uchar *e = (uchar*)t->stab;
	char *s;
	int n;

	if(p+4 > e || beget4(p) != DtBeginNode)
		return nil;
	p += 4;
	if((s = memchr((char*)p, 0, e - p)) == nil)
		return nil;
	n = s - (char*)p;
	cp += n;
	if(cp >= &t->path[sizeof(t->path)])
		return nil;
	memmove(cp - n, (char*)p, n);
	*cp = 0;
	p += (n + 4) & ~3;
	while(p+12 <= e && beget4(p) == DtProp){
		n = beget4(p+4);
		if(p + 12 + n > e)
			return nil;
		s = t->stab + beget4(p+8);
		if(s < t->stab || s >= (char*)t->end
		|| memchr(s, 0, (char*)t->end - s) == nil)
			return nil;
		devtreeprop(t->path, s, p+12, n);
		p += 12 + ((n + 3) & ~3);
	}
	while(p+4 <= e && beget4(p) == DtBeginNode){
		*cp = '/';
		p = devtreenode(t, p, cp+1);
		if(p == nil)
			return nil;
	}
	if(p+4 > e || beget4(p) != DtEndNode)
		return nil;
	return p+4;
}

static int
parsedevtree(uchar *base, uintptr len)
{
	Devtree t[1];
	u32int total;

	if(len < 28 || beget4(base) != DtHeader)
		return -1;
	total = beget4(base+4);
	if(total < 28 || total > len)
		return -1;
	t->base = base;
	t->end = t->base + total;
	t->stab = (char*)base + beget4(base+12);
	if(t->stab >= (char*)t->end)
		return -1;
	devtreenode(t, base + beget4(base+8), t->path);
	return  0;
}

void
bootargsinit(uintptr dtb)
{
	void *va;
	uintptr len;

	if(dtb < VDRAM-KZERO || (len = cankaddr(dtb)) == 0){
		plan9iniinit(BOOTARGS, 0);
		return;
	}
	va = KADDR(dtb);

	plan9iniinit(BOOTARGS, 0);
	if(parsedevtree(va, len) == 0){
		/* user can provide fewer ncpu */
		if(findconf("*ncpu") < 0){
			snprint(ncpu, sizeof(ncpu), "%d", cpus);
			addconf("*ncpu", ncpu);
		}
	}
	/*
	 * Quiet by default now that SMP works; set *gicdebug in the
	 * kernel config to dump the captured GIC layout (handy if the
	 * DTB parse ever needs revisiting on a new VZ version).
	 */
	if(getconf("*gicdebug") != nil)
		iprint("bootargsinit: gic node %q\n"
			"  gicd base %#llux size %#llux\n"
			"  gicr base %#llux size %#llux  (%d cpus -> per-cpu %#llux)\n",
			gicpath[0] ? gicpath : "(not found by compatible)",
			gicdbase, gicdsize, gicrbase, gicrsize,
			cpus, (cpus > 0 && gicrsize != 0) ? gicrsize/cpus : 0ULL);
}

char*
getconf(char *name)
{
	int i;

	if((i = findconf(name)) < 0)
		return nil;
	return confval[i];
}

void
setconfenv(void)
{
	int i;

	if(nconf < 0){
		/* use defaults when there was no configuration */
		ksetenv("console", "0", 1);
		return;
	}

	for(i = 0; i < nconf; i++){
		if(confname[i][0] != '*')
			ksetenv(confname[i], confval[i], 0);
		ksetenv(confname[i], confval[i], 1);
	}
}

void
writeconf(void)
{
	char *p, *q;
	int n;

	p = getconfenv();
	if(waserror()) {
		free(p);
		nexterror();
	}

	/* convert to name=value\n format */
	for(q=p; *q; q++) {
		q += strlen(q);
		*q = '=';
		q += strlen(q);
		*q = '\n';
	}
	n = q - p + 1;
	if(n >= BOOTARGSLEN)
		error("kernel configuration too large");
	memmove(BOOTARGS, p, n);
	memset(BOOTARGS+n, 0, BOOTARGSLEN-n);
	poperror();
	free(p);
}
