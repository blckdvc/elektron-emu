# Patches

Fifteen diffs against [Unicorn Engine](https://github.com/unicorn-engine/unicorn)
2.1.4, tag commit `8028ec436f2d9376525352dd38ed9ed6b9f6be10`, applied in this
order:

- `unicorn-2.1.4-m68k-hook-ccr-sync.patch` touches
  `qemu/target/m68k/translate.c` and `qemu/target/m68k/unicorn.c`.
- `unicorn-2.1.4-m68k-emac-mac-load.patch` touches
  `qemu/target/m68k/translate.c`.
- `unicorn-2.1.4-m68k-emac-fractional.patch` touches `qemu/target/m68k/helper.c`
  and `qemu/target/m68k/translate.c` (EMAC fractional and signed arithmetic).
- `unicorn-2.1.4-m68k-mac-regs.patch` touches `include/unicorn/m68k.h` and
  `qemu/target/m68k/unicorn.c` (MACSR, MASK, ACC0-3 via `uc_reg_read/write`).
- `unicorn-2.1.4-m68k-notdirty-pages.patch` touches `qemu/accel/tcg/cputlb.c`,
  `qemu/accel/tcg/translate-all.c`, `qemu/accel/tcg/translate-all.h` and
  `qemu/m68k.h` (speed only; the same file as in `digitone/dnkit/patches`).
- `unicorn-2.1.4-m68k-hookmem-pages.patch` touches `qemu/accel/tcg/cputlb.c`,
  `qemu/include/exec/cpu-all.h`, `qemu/include/tcg/tcg.h` and `uc.c` (speed
  only, see below).
- `unicorn-2.1.4-m68k-ldst-exit.patch` touches `qemu/accel/tcg/cputlb.c`,
  `qemu/tcg/tcg-op.c` and `qemu/target/m68k/cpu.c` (speed, and ISA_C on cfv4e).
- `unicorn-2.1.4-m68k-emac-fused.patch` touches `qemu/target/m68k/helper.c`,
  `helper.h` and `translate.c` (speed).
- `unicorn-2.1.4-m68k-emac-inline.patch` touches `qemu/target/m68k/translate.c`
  (speed).
- `unicorn-2.1.4-m68k-jit-toggle.patch` touches `qemu/accel/tcg/cpu-exec.c`,
  `cpu-exec-common.c`, `cputlb.c` and `tcg-runtime.c` (speed, macOS arm64).
- `unicorn-2.1.4-m68k-tb-budget.patch` touches `uc.c`, `include/uc_priv.h`,
  `include/unicorn/unicorn.h`, `qemu/accel/tcg/translator.c`,
  `tcg-runtime.c/.h` and `qemu/include/exec/gen-icount.h` (speed; new API).
- `unicorn-2.1.4-m68k-tlb-min.patch` touches `qemu/include/exec/cpu-defs.h`
  (speed).
- `unicorn-2.1.4-m68k-emac-spec.patch` touches `qemu/target/m68k/translate.c`
  (speed).
- `unicorn-2.1.4-m68k-hook-index.patch` touches `uc.c` and `include/uc_priv.h`
  (speed only, see below).
- `unicorn-2.1.4-m68k-emac-sat.patch` touches `qemu/target/m68k/helper.c` and
  `translate.c` (EMAC saturation with MACSR.OMC: sign and constants). Same file
  as `model_cycles/emulator/patches` (commit a5f9fbd there), SHA-256 `3bf897c3...`.

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

## What the hookmem-pages patch changes

Stock Unicorn 2 sends **every** guest load and store through the slow helper
(`load_helper` / `store_helper`) as soon as one `UC_HOOK_MEM_READ`,
`UC_HOOK_MEM_READ_AFTER` or `UC_HOOK_MEM_WRITE` hook exists anywhere, whatever
its address range (`tcg_uc_has_hookmem` turns the TLB-hit branch of the
generated code into an unconditional jump). The Syntakt co-simulation has a
few dozen narrow hooks on MMIO registers and DMA ports, so the whole firmware
ran on the slow path: sampled, 70% of the time was in those two helpers.

The patch makes the slow path a property of the page, kept in the TLB:

- a page with a read hook on any of its bytes gets `TLB_UC_HOOK` (a new flag)
  in `addr_read`; `load_helper` treats it as plain RAM once the hooks ran;
- a RAM page with a write hook keeps `TLB_NOTDIRTY` in `addr_write`, and
  `notdirty_write` no longer cleans a page that a write hook covers anywhere
  (the notdirty patch tested only the address written);
- the generated code keeps the normal TLB-hit fast path;
- `uc_hook_add` / `uc_hook_del` of those hook types flush the TLB.

Hooks run exactly as before. Checked on the Syntakt: `tools/cosimdet.py`
(memory and registers of both CPUs after 300 slices) and a 2 s render of
`tools/stwav.py` are bit-identical with and without the patch; the render goes
from 25 s to 8 s.

## What the ldst-exit patch changes

Stock Unicorn emits a `check_exit_request` helper call after every guest load
and store, so that a memory hook asking to stop takes effect at once. With the
hookmem-pages patch every hook runs on the slow path, so the same check (same
test, same state restore) now sits at the end of the slow-path load/store
helpers, and the generated code no longer calls it after each access. The check
at the start of each translation block stays.

The same patch enables ISA_C on the cfv4e model: the MCF5441x's V4e core has
FF1, BITREV and BYTEREV, which QEMU already implements (FF1 also sets N and Z
and clears V and C, as the ColdFire manual says). `emu/harness.py` then stops
emulating them with per-address Python hooks (`native_isa_c`).

## What the emac-fused patch changes

A MAC or MSAC without dual accumulate used to be three helper calls (multiply,
saturate, set flags), each spilling and reloading the guest registers. It is
now one helper doing the same steps in the same order; the mode (fractional,
signed, unsigned, and MACSR.SU for the scale shift) is still taken from MACSR
at translation time. The dual-accumulate form (EMAC_B only) is unchanged.

## What the emac-inline patch changes

The fractional MAC/MSAC (MACSR.FI at translation time, no dual accumulate) is
generated as TCG ops instead of the emac-fused helper: signed Q31 product >> 23
with RT rounding read from MACSR at run time, scale factor, accumulate,
macsatf with OMC read at run time, and the FI flags of mac_set_flags. Moves
from an accumulator in FI mode with SU = 0 (get_macf) likewise. Other modes
keep the helpers.

## What the jit-toggle patch changes

On macOS arm64 Unicorn switched the JIT area between writable and executable
(`pthread_jit_write_protect_np`) around EVERY exit from generated code
(cpu_tb_exec, cpu_loop_exit, the exit-request check): two switches per
interrupt, RTE or stop -- about 11% of the time here. The JIT now stays
executable between blocks; every path that writes it (tb_gen_code,
tb_add_jump, tb_phys_invalidate, tb_flush) already unlocks it itself, and
restore_jit_state puts the thread's entry state back when uc_emu_start returns.

## What the tb-budget patch changes

`uc_set_block_budget(uc, int64_t *state)` (new API, used by
`emu/nativehooks.py`'s NativeStepper): at every block start the generated code
itself does what the stepper's UC_HOOK_BLOCK did -- `left <= 0`: stop (the
block does not run), else `left -= 1, blocks += 1` -- on the same
`{left, blocks}` the idle-spin skip also updates; the block's PC is synced first,
as the hook's pc_sync did, so a stop resumes at the right place. The
exit-request test at block start is inline too (upstream QEMU's form), calling
the helper only when a request is pending. `DT2_TB_BUDGET=0` goes back to the
block hook.

## What the tlb-min patch changes

QEMU resizes a CPU's soft TLB only when it flushes it. The one flush Unicorn
does, at CPU reset, sees an empty usage window and shrinks the TLB to its
minimum, 64 direct-mapped entries; nothing flushes afterwards (hooks are added
before the run), so the emulator spends its whole life there. The index is the
low bits of the page number, and this firmware's code (`0x400xxxxx`), SRAM
(`0x800xxxxx`), SDRAM data and I/O (`0xfc0xxxxx`) pages share them: counted
with an instrumented build, 128k refills per emulated second, every one a
conflict with another page (`tlb_set_page_with_attrs`, `load_helper`,
`store_helper` among the top of the profile). Minimum and default are now 4096
entries (256 KB per MMU mode on a 64-bit host): 220 refills per second.
Alternating A/B, 3 s emulated with the pattern playing: 3.02-3.14 s -> 2.81-2.89 s
wall (main CPU 1.73-1.82 s -> 1.54-1.58 s); `tools/cosimdet.py` and the audio,
DSP and main hashes identical.

## What the emac-spec patch changes

Dumping the host code of the hottest translated blocks (a profiling build
writing `tb->tc.ptr` for chosen guest PCs, disassembled with capstone) showed
~55 arm64 instructions per fractional MAC and 25-48 host instructions per
guest instruction in the mixer and voice loops. Three things were work the
translator can settle once:

- RT and OMC are TB flags already (`cpu_get_tb_cpu_state` takes MACSR bits
  7..4), but the inline MAC and `get_macf` read them from MACSR at run time:
  the rounding is now generated only when RT = 1, and the saturation select
  without the OMC test;
- MACSR N, Z, V and EV were computed for every MAC, and the next MAC clears and
  recomputes all four: when the next instruction is a MAC or a move-to-ACC in
  the same TB, on the same page, not an exit address and not under a code hook
  (nothing can observe the state in between), they are skipped; the sticky PAV
  bits are still accumulated;
- `move.l <ea>,ACCx` called `helper_mac_set_flags` (with a MACSR spill and
  reload around it): inline for the fractional mode, and skipped under the
  same condition.

Host code of the hot blocks -25-30%. Measured on a quiet machine, 3 s emulated
with the pattern playing, CPU time: 2.05 s -> 1.32 s (DSP 0.95 -> 0.51, main
1.08 -> 0.79). `tools/cosimdet.py` and the audio, DSP and main hashes identical.

## Measured (Syntakt, audio chain on, pattern playing)

`tools/stbench.py`, wall seconds per emulated second: 5 patches 10.0 s;
+hookmem-pages 3.2; +DSP idle park and whole-slice DSP steps 1.9;
+ldst-exit 1.5; +emac-fused 1.36; +the DSP's DMA in C 1.22 (82% of real
time); then (a quieter machine, CPU time) native timers 1.06 -> 1.01, idle DSP
slices skipped 0.99, emac-inline 0.96, jit-toggle 0.92, tb-budget 0.83 (120%).
The GUI with live audio keeps real time (100% paced, 103% unthrottled). A 2 s render of `tools/stwav.py` is bit-identical at every step, and
`tools/cosimdet.py` too (except the FF1 change: same registers, a different
free-running DTIM0 count, because the block-bounded stepper estimates time from
block entries).

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

## What the notdirty patch changes (speed)

Stock Unicorn 2.1.4 never clears `TLB_NOTDIRTY` on m68k: every TLB entry is
executable (no MMU) and the fault hook covers the whole address space, so every
guest store took the slow path (`store_helper` -> `notdirty_write` ->
`tb_invalidate_phys_page_fast`). The patch applies QEMU's rule instead: clear the
flag when the page holds no translated block and no `UC_HOOK_MEM_WRITE` covers
it; `tb_gen_code` sets it again on every page that receives a new block, so
self-modifying code is still caught. Full reasoning in
`digitone/dnkit/patches/README.md` and `digitone/dnkit/docs/FINDINGS.md` d22.

Measured here (2026-09-27, Mac M3 Pro): main CPU 62 -> 68 M instructions/s
(`tools/realtime.py`), two-CPU co-simulation 3.5 -> 3.2 s for 1500 slices;
`tools/cosimdet.py` hashes (memory and registers of both CPUs, mailbox)
identical with and without the patch.

The installer writes only this project's `.venv`. `setup.sh` copies the stock
two-patch build from `../../.toolchain` when that passes the compat check, so
run `tools/install-patched-unicorn.sh` after `setup.sh` to get this one; the
shared toolchain copy (also used by Analog Rytm) is left as it is.

## Why the two EMAC patches are here too (2026-09-27)

The DSP CPU's program (section 7, decompressed at `0x40000400`) contains about 3700
EMAC instructions (line-A words such as `a1c3`/`a3c3` moves from an accumulator and
`a8a8 a801` MACs with an extension word, from `0x4000117a` on): the digital audio is
computed with EMAC, as on the Analog Rytm and the Digitone II. Stock Unicorn gets
fractional and signed MAC wrong (`digitone/dnkit/patches/README.md`), so any audio
this DSP produces needs `emac-fractional`. Today the DSP idles waiting for the audio
sync that the unmodelled FPGA/codec would give, and executes none of them:
`tools/cosimdet.py` hashes are identical with and without the patch.

## What the hook-index patch changes

`helper_uc_tracecode` is the TCG helper the translator emits before every
instruction that lies inside some `UC_HOOK_CODE` range. Upstream it walks the
whole code-hook list on every call and bound-checks each entry. With the
point hooks these emulators install (idle loops, hot loops, the block sender,
tracers) that walk was 13% of the ColdFire thread's own time in a live profile
(`digitone/dnkit/docs/FINDINGS.md`, d32).

The patch adds a direct-mapped cache in `uc_struct` (1024 entries keyed by the
address, up to 6 hooks each): on a miss it walks the list once and stores the
matching hooks in list order; on a hit it calls only those. A generation
counter bumped by `uc_hook_add`, `uc_hook_del` and the deferred purge
(`clear_deleted_hooks`) drops the whole cache. Stop handling after each
callback is the same as upstream; `size == 0` (invalid instruction) and
addresses with more than 6 matching hooks take the upstream walk.

One documented difference: a code hook added *from inside* a callback used to
be visited for the same instruction when it was appended to the list; with the
cache it first fires at the next hooked instruction.

## What the emac-sat patch fixes (from the Model:Cycles, 2026-10-05)

With MACSR.OMC = 1 the EMAC saturates instead of wrapping, and QEMU got the
sign of the saturated value wrong: `helper_get_macs` (MOVE/MOVCLR ACCx,Rx in
signed integer mode, CFPRM p.6-10) returned `0x7FFFFFF8` (+1.0) for every
negative overflow; `helper_macsats`, `helper_macsatf` and the inline fractional
MAC (`emac-inline`/`emac-spec`) took the sign from the wrapped 48-bit result.
Full analysis in `model_cycles/emulator/patches/README.md`, where it made
32-sample blocks of -FS in the middle of a signal near +FS (pattern A02).

On the Syntakt (`scratch/stemacsat_esiti.txt`):

- The DSP (tracks 1-8) never runs in the mode `get_macs` saturates in: its
  code sets MACSR only to `0x20` and `0xa0` (fractional, with and without OMC;
  `scratch/stmacsrscan.py`), and in PLAY (A01 A02 A05 A08 A13 A15, ~4M moves
  from ACC per 0.3 s, `scratch/stemacsat.py`) no move from ACC ever sees a PAVx
  bit set: no 48-bit accumulation overflow either. The DSP's audio does not
  touch any of the four fixed paths.
- The main CPU has two functions with MACSR `0x80` (signed integer, OMC):
  `0x4008eeca` (never saturates in the patterns tried) and `0x400964a0`, a x40
  gain (`mac.l` with `d1 = 0x28`, `movclr.l`, `lsr.l #8`, in place) on TX slots
  4, 1, 0 and 6 (table `0x401c0fe0`), which are not part of the modelled main
  out. It does saturate: A08 32 positive, A13 5 negative, A15 22 negative and 4
  positive in 0.3 s.
- `scratch/stptnaudio.py N 4` (A01-A16 from the factory project, PLAY, 4 s),
  old -> new library: the main out (`staudio.mix`, slots 5+7 / 3+2, peaks at
  most 0.564 FS) is bit-identical in all 16 patterns. Only slot 1 (A13, 15
  samples) and slot 4 (A15, 74) change, every one from +8388607 to -8388608;
  jumps > 0.5 FS on those slots 6 -> 0 and 56 -> 33, second difference max
  2.091 -> 0.776 and 3.804 -> 1.088. Old -> 14 patches with hook-index but
  without emac-sat: identical, so the change is all emac-sat's.
- Speed unchanged (`scratch/stbenchab.py 4 3 --lib`: CPU median 0.709 -> 0.706 s
  per emulated second).

The library that was in the `.venv` before had 13 patches: the installer
already listed hook-index (14th) but the installed dylib was the one from
before it (sha256 `7d50f4d0...`, the same as `libunicorn.2.dylib.pre-hook-index`).
Installed now with all 15: sha256 `5b372d7c...`; the previous one is kept as
`libunicorn.2.dylib.pre-emac-sat`. `emu/unicorn_compat.py`
(`emac_movclr_saturation`) refuses a library without this patch;
`tests/test_unicorn_emac.py` (`SAT_CASES`) checks the three helpers. No Windows
build exists for the Syntakt.
