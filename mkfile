CONF=vz
CONFLIST=vz

kzero=0xffffffff00000000
loadaddr=0xffffffff70100000

objtype=arm64
</$objtype/mkfile
p=9

DEVS=`{rc ../port/mkdevlist $CONF}

PORT=\
	alarm.$O\
	alloc.$O\
	allocb.$O\
	auth.$O\
	cache.$O\
	chan.$O\
	dev.$O\
	edf.$O\
	fault.$O\
	mul64fract.$O\
	page.$O\
	parse.$O\
	pgrp.$O\
	portclock.$O\
	print.$O\
	proc.$O\
	qio.$O\
	qlock.$O\
	rdb.$O\
	rebootcmd.$O\
	segment.$O\
	syscallfmt.$O\
	sysfile.$O\
	sysproc.$O\
	taslock.$O\
	tod.$O\
	xalloc.$O\
	userinit.$O\

OBJ=\
	l.$O\
	cache.v8.$O\
	clock.$O\
	fpu.$O\
	main.$O\
	mmu.$O\
	mem.$O\
	sysreg.$O\
	random.$O\
	trap.$O\
	bootargs.$O\
	$CONF.root.$O\
	$CONF.rootc.$O\
	$DEVS\
	$PORT\

# HFILES=

LIB=\
	/$objtype/lib/libmemlayer.a\
	/$objtype/lib/libmemdraw.a\
	/$objtype/lib/libdraw.a\
	/$objtype/lib/libip.a\
	/$objtype/lib/libsec.a\
	/$objtype/lib/libmp.a\
	/$objtype/lib/libc.a\
#	/$objtype/lib/libdtracy.a\

9:V:	$p$CONF $p$CONF.u $p$CONF.bin

$p$CONF.u:D:	$p$CONF
	aux/aout2uimage -Z$kzero $p$CONF

# 9vz.bin is the raw boot payload the Apple Virtualization (mac)
# runner wants: the uImage with aux/aout2uimage's 64-byte uImage
# header stripped off.  l.s already embeds the ARM64 Linux-style
# boot header, so this file boots directly:  9vz -kernel 9vz.bin
# (see BUILD.md / NOTES).  Strips exactly the bytes we used to drop
# by hand with: dd if=9vz.u of=9vz.bin bs=64 skip=1
$p$CONF.bin:D:	$p$CONF.u
	{dd -bs 64 -skip 1 <$prereq} >$target

$p$CONF:D:	$OBJ $CONF.$O $LIB
	$LD -o $target -T$loadaddr -l $prereq
	size $target

$OBJ: $HFILES

install:V: /$objtype/$p$CONF

/$objtype/$p$CONF:D: $p$CONF $p$CONF.u
	cp -x $p$CONF $p$CONF.u /$objtype/

<../boot/bootmkfile
<../port/portmkfile
<|../port/mkbootrules $CONF

main.$O: rebootcode.i

pciqemu.$O: ../port/pci.h

initcode.out:		init9.$O initcode.$O /$objtype/lib/libc.a
	$LD -l -R1 -s -o $target $prereq

rebootcode.out:		rebootcode.$O cache.v8.$O
	$LD -l -H6 -R1 -T0x70020000 -s -o $target $prereq

$CONF.clean:
	rm -rf $p$CONF $p$CONF.u $p$CONF.bin errstr.h $CONF.c boot$CONF.c
