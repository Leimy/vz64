vz64 - a 9front kernel for Apple Virtualization.framework
==========================================================

This is a native 9front/arm64 kernel that boots directly under
Apple's Virtualization.framework (VZ) on Apple silicon Macs, via
VZLinuxBootLoader -- no firmware, no U-Boot, no emulated legacy
hardware.  It boots to an interactive rc prompt on an hjfs root
filesystem over virtio-blk, with working virtio console, GICv3
interrupts, ARM virtual timer, and virtio networking, and supports
graphical use via drawterm from the host.

This port was generated with Claude Opus (Anthropic) and Fable,
working with a human who ran every build and boot.  The kernel
debugging happened over a claude9 session running ON a 9front
system; the host-side tooling was built with Fable on the Mac.
The bugs were real and the transcript was long.

Companion host-side runner: 9vz (separate repository), a small Go
harness over the Code-Hex/vz bindings that supplies the VM, the
virtio devices, and the serial console on stdio.

Status
------
Working: single-CPU boot to rc prompt and drawterm-in, on a stock
9front arm64 disk image (hjfs).  SMP is plumbed but not yet
working under VZ -- see TODO at the bottom.

Booting it
----------
Build (on a Plan 9/9front system, with this directory bound over
/sys/src/9/vz64):

    cd /sys/src/9/vz64
    mk

This produces 9vz (a.out) and 9vz.u (uImage).  On the Mac:

    dd if=9vz.u of=9vz.bin bs=64 skip=1     # strip uImage wrapper
    ./check_kernel.sh 9vz.bin               # (in the 9vz repo) should
                                            # report the arm64 header
    ./9vz -kernel 9vz.bin -disk 9front.raw -cmdline 'console=0
    *ncpu=1
    nobootprompt=local!/dev/sdF0/fs'

The cmdline parameters that matter:

    console=0                    serial console (the only console)
    *ncpu=1                      stay single-cpu (SMP is TODO)
    nobootprompt=local!/dev/sdF0/fs
                                 root from the first virtio-blk
                                 disk.  sdvirtio10 letters virtio-
                                 blk disks from 'F', so the first
                                 -disk is ALWAYS sdF0.

The disk image must carry an arm64 userland (e.g. the 9front
arm64 release image, raw format).  The kernel on the disk image
is ignored: VZ loads the kernel from the host file every launch,
so kernel upgrades never require touching the guest filesystem.
Run fshalt in the guest before stopping the VM; hjfs buffers
writes.

How it works
------------
The kernel is derived from the 9front arm64 (qemu-virt) port.
VZ guests start at EL1 under Apple's hypervisor with a virtual
GICv3, ARM generic timers, PSCI power management over HVC, and
virtio 1.0 devices on a PCI ECAM.  There is no UART, no VGA, no
PS/2 -- everything is virtio.

Boot protocol: VZLinuxBootLoader requires the Linux arm64 Image
header.  This kernel embeds the 64-byte header directly at
_start in l.s: code0 is a raw WORD-encoded branch (b .+64) over
the header words, text_offset 0x100000 places the image at
0x70100000 = KTZERO, and the magic "ARM\x64" sits at offset 56.
The header must be data words, not assembler branch mnemonics:
the Plan 9 7l linker's branch-following pass will otherwise
relocate code over the header.  The kernel command line arrives
via the device tree /chosen node and is parsed by bootargsinit.

VZ-specific adaptations, each of which was a bring-up bug first:

  * tas() and cmpswap() use ARMv8.1 LSE atomics (SWPW/CASW)
    instead of LDXR/STXR.  Apple's hypervisor clears the
    exclusive monitor on every VM exit, so classic load/store-
    exclusive spinlocks can livelock with zero contention.
  * No PMU.  VZ guests trap on PMCR_EL0 and friends; all cycle
    counting uses the virtual counter CNTVCT_EL0 (24 MHz).
  * PSCI over HVC for power control: SYSTEM_OFF/SYSTEM_RESET for
    exit/reboot, CPU_ON for secondary cpus (see TODO).
  * The console is virtio-console, brought up in two phases:
    before PCI is scanned, print output buffers in a memory
    ring; uartvzlink() (during links()) probes the device,
    starts the vrings, and flushes the ring.  The TX path must
    genuinely wait for the device's used index before reusing
    descriptors -- VZ's backend is a concurrent host thread,
    not a synchronous emulator.
  * mmu0init block-maps all of guest RAM up front (VZ places the
    DTB near the top of RAM); l1map() later skips entries that
    are already valid 2MB blocks.

Memory map (see mem.h): guest RAM at IPA 0x70000000 (VDRAM),
kernel text at 0x70100000 (KTZERO), GICv3 at 0x10000000, PL031
RTC at 0x20050000, PCI BARs around 0x50000000, KZERO =
0xFFFFFFFF00000000.

For the full story -- including the debugging methodology (PSCI
SYSTEM_OFF as a one-bit signal for bisecting a kernel with no
console) and the complete war stories -- read PORT.txt.  NOTES is
the raw bring-up log.

Graphics
--------
There is no framebuffer device (and no virtio-gpu driver yet).
The practical path is to run the guest as a cpu server and
connect with drawterm from the Mac over the NAT subnet:
rcpu/tcp17019, or a simpler listen1-based setup.  The Mac can
reach the guest's DHCP address directly (Apple NAT, bridge100;
note 192.168.64.1 is the HOST, not the guest).

TODO
----
  * SMP.  mpinit issues PSCI CPU_ON for secondaries; under VZ the
    SMC32 function id (0x84000003) returns -1 NOT_SUPPORTED:

        mpinit: PSCI CPU_ON cpu1 failed: -1

    The code now uses the SMC64 id (0xC4000003), which is what
    Linux uses on arm64 and matches the failure signature (the
    argument-less PSCI calls, which only exist as SMC32, all
    work).  This fix is UNTESTED -- boot without *ncpu=1 to try
    it.  The rest of the secondary path (BSS clear gated to
    cpu0, no WFE parking, per-cpu GIC/timer init) has been
    audited but never executed under VZ.
  * uartvz TX locking: putc (iprint/panic) and kick (queued
    output) can race the TX vring from two cpus; fine today on
    one cpu, needs an ilock for SMP.
  * virtio-gpu + devdraw/devmouse for native graphics.
  * The early console ring flush vs rx interrupt ordering.

Files
-----
    l.s          entry + arm64 boot header, LSE atomics, vectors,
                 cache/TLB ops, hvccall, vzstop (PSCI SYSTEM_OFF
                 debug aid)
    main.c       boot sequence, confinit, mpinit
    mem.c mmu.c  page tables, kmapram, identity/kernel maps
    trap.c gic.c clock.c fpu.c    traps, GICv3, virtual timer, FP
    uartvz.c     virtio-console driver
    bootargs.c   DTB / plan9.ini-style config
    pciqemu.c    PCI ECAM access
    sysreg.c devrtc.c             sysreg helpers, PL031 RTC
    dat.h mem.h fns.h io.h        machine definitions
    vz           kernel device configuration
    mkfile       build rules
    PORT.txt     full port documentation
    NOTES        raw bring-up log
