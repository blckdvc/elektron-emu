# Tests of the Elektron Emu view

`tests/run-tests.sh [dt dn st ar mc]` builds `seqtest` for each machine against its own
core and runs two kinds of checks (`seqtest.cpp`):

- **layout** (no firmware): the view is drawn with a made-up pattern; every control must
  be inside the view, controls must not overlap, every button needs a key code.
- **engine** (with the firmware): the real OS runs, and the view is driven like the
  plug-in: a trig written then erased, a drag along a row (write and erase), a cell on
  another track, a cell on page 2 of a track that has one, a mute on and off, a pattern
  chosen in the list then *previous*, play (the position moves), the page loop
  (the position stays on page 2) and stop.

The firmwares are read from `.re/` in the repo (or `FW_DIR`), with the names of the
Elektron files (`Digitakt_II_OS1.16.syx`, `Digitone_II_OS1.11.syx`, `Syntakt_OS1.41.syx`,
`Analog-Rytm_MKII_OS1.73.syx`, `model-cycles_OS1.13.syx`). The first run makes a cold
boot state per machine (a few minutes each, kept in `~/.cache/elektron-emu-tests`).

Results: `tests/out/report.txt` (PASS / FAIL / SKIP, one line per check) and
`tests/out/report.html` (with a picture of each view). The exit code is the number of
failures. On GitHub the layout tests run at each push (`.github/workflows/tests.yml`).
