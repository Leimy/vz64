# vz64 — 9front for Apple Virtualization.framework

A port of sys/src/9/arm64 (qemu-virt) to VZ's hardware model, derived
from a device-tree survey of a VZ Linux guest.

## What changed vs arm64/

  mem.h       KZERO 0xFFFFFFFF00000000; VDRAM -> PA 0x70000000 (the
              port relies on PA = VA - KZERO everywhere, so the VA
              window slides to retarget physical RAM). PHYSIO window
              now 0x10000000-0x20070000 (GICv3 + PL031/PL061).
              DTBADDR removed.
  l.s         unchanged — entry already saves x0 (DTB) in R26 and
              hands it to main() in R0.
  main.c      main(uintptr dtb); passes dtb to bootargsinit.
  bootargs.c  parses the DTB at the address VZ provides (via x0)
              instead of a fixed 0x40000000. -cmdline from the 9vz
              harness lands in /chosen/bootargs = plan9.ini content.
  gic.c       GICv3 redistributors at +0x10000 (VZ: GICD 0x10000000,
              GICR 0x10010000).
  devrtc.c    PL031 at PA 0x20050000.
  pciqemu.c   ECAM 0x40000000, BAR window 0x50000000; INTx is
              per-slot: slot d -> SPI 32+d (no QEMU swizzle).
  uartvz.c    NEW: null console (VZ has no UART hardware at all).
              Output goes to a 16K ring buffer in kernel memory.
              Real fix later: virtio-console driver.
  vz          kernel config (uartqemu -> uartvz).
  mkfile      CONF=vz, kzero/loadaddr for the new layout,
              rebootcode at 0x70020000.

## Building (inside a 9front system — kernel needs the native toolchain)

Boot your QEMU baseline 9front, get this directory to
/sys/src/9/vz64 (drawterm, 9fs, u9fs — dealer's choice), then:

    cd /sys/src/9/vz64
    mk 'CONF=vz'

Produces `9vz` (Plan 9 a.out), `9vz.u` (uImage via aout2uimage,
load/entry 0x70100000), and `9vz.bin` (the uImage with its 64-byte
header stripped -- this is what the mac runner boots directly).

## Running under VZ (on the Mac)

`9vz.bin` is built ready to boot: l.s embeds the ARM64 Linux-style
boot header, so no header tool is needed on the mac side.  Just
copy 9vz.bin over and:

    ./9vz -kernel 9vz.bin -disk 9front.raw \
          -cmdline 'console=0'                  # bootargs = plan9.ini

(The old two-step `dd if=9vz.u ... bs=64 skip=1` + `mkarm64hdr.py`
flow is gone: the dd strip is now a build target, and the python
header is superseded by the embedded header in l.s.  Do NOT run
mkarm64hdr.py on 9vz.bin -- see NOTES.)

## Expectations, round 1

- Console is null: success is NOT text. Success is the VM *staying in
  state running* instead of erroring out, and (better) virtio-blk
  reads hitting the disk image as the kernel tries to mount root.
- Expect compile errors first (this port was written dry against the
  tree, never compiled). Paste them back to Claude.
- Memory cap: keep -mem 2 (the VA layout reserves the top 256MB above
  VDRAM+2GB for vmap; >2GB RAM will collide).
- Then: kernel checkpoint bisection if it errors, virtio-console
  driver (uartvirtio10.c, cribbing port/virtio10mem.c) once it lives.
