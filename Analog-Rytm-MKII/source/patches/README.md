# Patches

Diffs against [Unicorn Engine](https://github.com/unicorn-engine/unicorn)
2.1.4, tag commit `8028ec436f2d9376525352dd38ed9ed6b9f6be10`. Since
2026-10-05 `tools/install-patched-unicorn.sh` applies fourteen (see *The
Syntakt's thirteen* and *emac-sat* at the end); the first two were:

- `unicorn-2.1.4-m68k-hook-ccr-sync.patch` touches
  `qemu/target/m68k/translate.c` and `qemu/target/m68k/unicorn.c`.
- `unicorn-2.1.4-m68k-emac-mac-load.patch` touches
  `qemu/target/m68k/translate.c`.

Both files are QEMU sources vendored inside Unicorn. **The patches are
derivative works of that code and carry its licence, not this repository's
choice of one** — take their terms from Unicorn and QEMU upstream. This is why
the repository as a whole is GPL-2.0-or-later rather than something
permissive; see the licence section of the top-level [README](../README.md).

## What the CCR patch fixes

Unicorn's m68k translator keeps condition codes lazily and commits them when a
translation block ends normally. A code hook, or a `count=` stop, can return to
the host *mid-block*, before that commit — so a guest `CMP` followed by a
hook-visible `SR` read, or by a counted stop on the next instruction, exposes
stale flags. Guest branches then take the wrong arm.

The patch commits the pending condition codes before handing control back.

## What the EMAC patch fixes

`DISAS_INSN(mac)` handles MAC and MSAC with load (ColdFire Programmer's
Reference Manual, p.6-3 to 6-5 and p.6-22 to 6-23) wrongly in four ways.
QEMU master had the same code on 2026-09-15.

- It takes extension-word bits 1..0 as a dual-accumulate request and, on a
  core without EMAC_B such as the CFV4E, raises an illegal-instruction
  exception. In the load forms those bits are part of Ry, so every Ry whose
  register number has bit 0 or 1 set faulted. The patch honours the request
  only on EMAC_B cores.
- It reads a data-register Rx from operation-word bits 14..12, which are
  always 2 for these opcodes, so Rx was always D2. Rx is extension-word bits
  15..12.
- It reads the MSAC bit from operation-word bit 8, which is always 0 for
  these opcodes, so MSAC added. The bit is extension-word bit 8. This also
  affects MSAC without load.
- It ANDs MASK into every load address. Extension-word bit 5 says whether
  MASK is used.

The Digitakt II SHARC frame build reaches one of these instructions at
`0x400db9e0`.

## Checks

`emu/unicorn_compat.py` exercises the CCR shapes and the `0x400db9e0`
instruction, and refuses to run on an interpreter whose Unicorn lacks either
patch, rather than letting a subtly wrong emulation pass for a working one.
`tests/test_unicorn_emac.py` checks the MAC and MSAC load forms against the
manual.

## Applying them

Do not apply these by hand. `tools/install-patched-unicorn.sh` pins the
upstream commit, verifies each patch's SHA-256, builds only the m68k target,
replaces the dynamic library the Python bindings actually load, and then runs
the compat check. `uv sync` can restore the stock wheel, which puts the
emulator back to refusing to start until the installer is rerun.

## Patches added for the Analog Rytm (2026-09-27)

- `unicorn-2.1.4-m68k-emac-fractional.patch`, `unicorn-2.1.4-m68k-mac-regs.patch`,
  `unicorn-2.1.4-m68k-notdirty-pages.patch`: copied unchanged from dnkit
  (`../../digitone/dnkit/patches/README.md` explains them). The EMAC one is a
  correctness fix: stock Unicorn gets fractional and signed MAC arithmetic
  wrong, and the Rytm's audio routine (vector 191) is EMAC code.
- `unicorn-2.1.4-page-scoped-mem-hooks.patch` (this project): touches
  `qemu/accel/tcg/cputlb.c`, `qemu/include/exec/exec-all.h`,
  `qemu/tcg/aarch64/tcg-target.inc.c`, `qemu/tcg/i386/tcg-target.inc.c` and
  `uc.c`. Stock Unicorn sends EVERY guest load and store to the slow path
  (`load_helper`/`store_helper`) as soon as one memory hook exists anywhere
  (`tcg_uc_has_hookmem`). The patch keeps the inline fast path and instead
  flags, in the TLB, only the pages a READ/READ_AFTER/WRITE hook overlaps
  (`TLB_WATCHPOINT`, whose check is empty in Unicorn), so those pages alone
  take the slow path where the hooks run; adding or deleting a memory hook
  flushes the TLB. Measured on the Rytm with the audio model on: 0.34x ->
  0.63x real time, FPGA frames, SSI TX and screen byte-identical to the
  build without it.

## The Syntakt's thirteen (in the .venv since 2026-09-28, here since 2026-10-05)

On 2026-09-28 the `.venv` got the library of digikit/Digitone/Syntakt (sha1
`3cc012af…`, see the top-level README, *Tempo reale con altri emulatori
aperti*), built from the Syntakt's 13 patches, but this repository kept
listing its own six, so `tools/install-patched-unicorn.sh` rebuilt a different
(older, slower) library from what ran. The eight missing ones are now copied
here unchanged from `../../model_cycles/emulator/patches` (same SHA-256 as the
Syntakt's): `hookmem-pages` (which replaces this project's
`page-scoped-mem-hooks`, kept here for the record but no longer applied),
`ldst-exit`, `emac-fused`, `emac-inline`, `jit-toggle`, `tb-budget`,
`tlb-min`, `emac-spec`. Their explanations are in
`../../syntakt/emulator/patches/README.md`.

## What the emac-sat patch fixes (from the Model:Cycles, a5f9fbd; here 2026-10-05)

Copied unchanged from `../../model_cycles/emulator/patches` (its README has
the full explanation, with the Cycles measurements). With MACSR.OMC = 1 QEMU
got the sign of the saturated value wrong: `helper_get_macs` (MOVE/MOVCLR
ACC,Rx in signed integer mode, CFPRM p.6-10) returned `0x7FFFFFF8` (+1.0)
for every negative overflow instead of `0x80000000`; `helper_macsats`,
`helper_macsatf` and the inline fractional MAC of `emac-inline`/`emac-spec`
took the sign from the wrapped 48-bit result (and `macsatf` used +-(2^47 - 1)
instead of `0x007f_ffff_ff00` / `0xff80_0000_0000`, MCF5441x RM 5-16/5-17).

On the Rytm (`scratch/rytmemac.py`, results in `scratch/rytmemac_esiti.txt`,
0.5 s emulated each on `live_main.snap` after PLAY and on the saved session
already playing): of ~14M MAC/MSAC and ~5M moves from ACC, 192,000 MAC and
192,000 MOVCLR run in exactly the `get_macs` mode (MACSR `0x80`: the routine
at `0x4010e950`, `msac.l d0,d3,<<1` with d0 = 1, i.e. -2x, then
`movclr.l ACCn,d4`), and ~6.5-7M MAC in fractional mode with OMC (`0xa0`).
**None of them overflows**: no ACC outside 32 bits at a MOVCLR in mode `0x80`,
no PAVx bit set at any move from ACC. So with these runs the patch changes
nothing: `tools/rytmdet.py` FPGA-frame, SSI-TX, engine-wav and event-trace
hashes are identical with the old and the new library (`live_main` 4 s with
PLAY, `--gui` and without, the session 8 s), and so are 17 screenshots of two
`tools/uiscript.py` scenarios. It would matter only for a signal reaching
+-2^30 at `0x4010e950` (old library: a negative clip read as +full scale).

`emu/unicorn_compat.py` (`emac_movclr_saturation`) refuses a library without
this patch; `tests/test_unicorn_emac.py` (`SAT_CASES`, 3 cases) fails on the
old one. The Rytm has no Windows build, so there is no DLL to rebuild.
