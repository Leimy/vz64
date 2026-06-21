vz64 - a 9front kernel for Apple Virtualization.framework
==========================================================

This is a native 9front/arm64 kernel that boots directly under
Apple's Virtualization.framework (VZ) on Apple silicon Macs, via
VZLinuxBootLoader -- no firmware, no U-Boot, no emulated legacy
hardware.  It boots to an interactive rc prompt on an hjfs root
filesystem over virtio-blk, with working virtio console, GICv3
interrupts, ARM virtual timer, virtio networking, and SMP (multi-
cpu), and supports graphical use via drawterm from the host.

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
Working: boot to rc prompt and drawterm-in, on a stock 9front
arm64 disk image (hjfs).  SMP works: a multi-cpu guest (9vz
default -cpus 2, *ncpu unset) boots cleanly with the secondary
cpus online.  *ncpu=1 still forces a single-cpu boot.

Booting it
----------
Build (on a Plan 9/9front system, with this directory bound over
/sys/src/9/vz64):

    cd /sys/src/9/vz64
    mk

This produces 9vz (a.out), 9vz.u (uImage), and 9vz.bin (the
uImage header stripped -- ready to boot; the dd that used to be a
manual mac-side step is now a build target).  On the Mac:

    ./check_kernel.sh 9vz.bin               # (in the 9vz repo) should
                                            # report the arm64 header
    ./9vz -kernel 9vz.bin -disk 9front.raw -cmdline 'console=0
    nobootprompt=local!/dev/sdF0/fs'

The cmdline parameters that matter:

    console=0                    serial console (the only console)
    *ncpu=1                      optional: force single-cpu (SMP
                                 works; omit to use all -cpus N)
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
    exit/reboot, CPU_ON (SMC64 id 0xC4000003) for secondary cpus.
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
Native graphics works via a virtio-gpu framebuffer.  Launch the
host runner with -gui (9vz -gui ...); it attaches a virtio-gpu
device with one scanout plus a USB keyboard and a USB absolute
pointing device, and opens a real macOS window.  The kernel side
is:

  * screen.c   a virtio-gpu driver implementing the devdraw
               screen.c contract (attachscreen / flushmemscreen /
               software cursor).  It creates one host 2D resource
               the size of the scanout, attaches a guest linear
               framebuffer ("softscreen") as backing, binds it to
               scanout 0, and on each flush issues
               TRANSFER_TO_HOST_2D + RESOURCE_FLUSH for the dirty
               rectangle.  The single control queue is driven
               synchronously (polling the used ring) exactly like
               the console TX path -- VZ's backend is a concurrent
               host thread, so the waits are real.
  * #i (draw), #m (mouse), swcursor are now in the vz config.
  * USB keyboard and pointer are handled in userspace by nusb
    (usbd + nusb/kb): kb writes absolute mouse events to
    /dev/mousein with the 'A'/'a' prefix, which devmouse routes
    through absmousetrack/scmousetrack.

If 9vz is run without -gui there is no virtio-gpu device;
screeninit() finds nothing and the kernel stays headless (the
serial console still works).  vgasize=WxHx32 in -cmdline overrides
the default 1024x768 scanout size (depth must be 32).

The older path -- run the guest as a cpu server and connect with
drawterm from the Mac over the NAT subnet (rcpu/tcp17019, or a
listen1 setup; the Mac reaches the guest's DHCP address directly
on Apple NAT bridge100, note 192.168.64.1 is the HOST) -- still
works and needs no display device.

Done since first bring-up
-------------------------
  * SMP.  Verified: a 2-cpu guest boots to the rc prompt with
    cpu1 online.  The decisive fixes were the SMC64 PSCI CPU_ON
    id (0xC4000003 -- the SMC32 0x84000003 returns -1
    NOT_SUPPORTED), a POSITIONAL getrregs fallback (VZ's
    GICR_TYPER reads all-zero, so affinity matching is
    impossible -- cpu N takes the Nth redistributor frame),
    clearing GICR_WAKER on the secondary, and matching the GIC
    DTB node by `compatible` = "gic-v3".  CPU count is not
    capped at two; omit *ncpu and use 9vz -cpus N.  See PORT.txt
    section 6.8 and NOTES for the full diagnosis.
  * uartvz TX locking: putc (iprint/panic) and kick (queued
    output) now share the vcon.txl ilock around the TX vring, so
    two cpus cannot interleave descriptor/avail updates.

TODO
----
  * virtio-gpu hardware cursor (currently swcursor).
  * virtio-gpu RESIZE / multi-scanout (VZ exposes one scanout;
    the host -width/-height set it; guest-driven mode set via
    GET_DISPLAY_INFO is not wired up).
  * The early console ring flush vs rx interrupt ordering
    (cosmetic: input arriving during flush could be dropped).
  * Higher cpu counts (>2) are plausible but unverified -- the
    code has no 2-cpu assumption, but only -cpus 2 has been
    exercised.
  * rebootcode/reboot path untested under VZ (SYSTEM_RESET works
    for exit; warm reboot into a new kernel is unexplored).

Files
-----
    l.s          entry + arm64 boot header, LSE atomics, vectors,
                 cache/TLB ops, hvccall, vzstop (PSCI SYSTEM_OFF
                 debug aid)
    main.c       boot sequence, confinit, mpinit
    mem.c mmu.c  page tables, kmapram, identity/kernel maps
    trap.c gic.c clock.c fpu.c    traps, GICv3, virtual timer, FP
    uartvz.c     virtio-console driver
    screen.c     virtio-gpu framebuffer (devdraw screen.c contract)
    screen.h     devmouse/screen/swcursor prototypes
    bootargs.c   DTB / plan9.ini-style config
    pciqemu.c    PCI ECAM access
    sysreg.c devrtc.c             sysreg helpers, PL031 RTC
    dat.h mem.h fns.h io.h        machine definitions
    vz           kernel device configuration
    mkfile       build rules
    PORT.txt     full port documentation
    NOTES        raw bring-up log
