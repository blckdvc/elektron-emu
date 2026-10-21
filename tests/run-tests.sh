#!/usr/bin/env bash
# The tests of the Elektron Emu view, for the five machines (tests/seqtest.cpp):
#
#   tests/run-tests.sh [dt dn st ar mc]
#
# For each machine: builds the test against its core, checks the layout of the view
# (no firmware needed) and, when the firmware is there, drives the view on the real OS
# (grid, drag, other track, page 2, mutes, pattern select, play, page loop).
#
# Variables:
#   FW_DIR   the firmwares (.syx), default .re/ in the repo (the names of the Elektron
#            files: Digitakt_II_OS1.16.syx, Digitone_II_OS1.11.syx, Syntakt_OS1.41.syx,
#            Analog-Rytm_MKII_OS1.73.syx, model-cycles_OS1.13.syx)
#   WORK     builds, cold boot states (made once, a few minutes each), default
#            ~/.cache/elektron-emu-tests
#   UNICORN_<M>  the patched Unicorn of a machine (UNICORN_DT...); by default the one
#            build-au-mac.sh made, otherwise built here
#
# Results: tests/out/report.txt (one line per check), tests/out/report.html (with a
# picture of each view); exit code: the number of failures.
set -uo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
fw_dir=${FW_DIR:-$root/.re}
work=${WORK:-$HOME/.cache/elektron-emu-tests}
out=$root/tests/out
mkdir -p "$work" "$out"
machines=("$@")
[[ ${#machines[@]} -gt 0 ]] || machines=(dt dn st ar mc)
: > "$out/report.txt"

dir_of() { case $1 in dt) echo Digitakt-II;; dn) echo Digitone-II;; st) echo Syntakt;; ar) echo Analog-Rytm-MKII;; mc) echo Model-Cycles;; esac; }
name_of() { case $1 in dt) echo "Digitakt II";; dn) echo "Digitone II";; st) echo Syntakt;; ar) echo "Analog Rytm MKII";; mc) echo "Model:Cycles";; esac; }
fw_of() { case $1 in dt) echo Digitakt_II_OS1.16.syx;; dn) echo Digitone_II_OS1.11.syx;; st) echo Syntakt_OS1.41.syx;; ar) echo Analog-Rytm_MKII_OS1.73.syx;; mc) echo model-cycles_OS1.13.syx;; esac; }
# the Unicorn that build-au-mac.sh (vst/scripts/build-mac.sh) leaves
mac_unicorn_of() { case $1 in dt) echo ~/.cache/dt2-vst/unicorn-mac;; dn) echo ~/.cache/dn-vst/unicorn-mac;; st) echo ~/.cache/st-vst/unicorn-m68k-st;; ar) echo ~/.cache/ar-vst/unicorn-mac;; mc) echo ~/.cache/mc-vst/unicorn-mac;; esac; }

total_fail=0
for m in "${machines[@]}"; do
  d=$(dir_of "$m")
  [[ -n $d ]] || { echo "unknown machine: $m" >&2; continue; }
  echo "== $(name_of "$m")"
  echo "== $(name_of "$m")" >> "$out/report.txt"
  var=UNICORN_$(echo "$m" | tr a-z A-Z)
  uc=${!var:-$(mac_unicorn_of "$m")}
  if [[ ! -f $uc/lib/libunicorn.a ]]; then
    uc=$work/unicorn-$m
    if [[ ! -f $uc/lib/libunicorn.a ]]; then
      echo "   building the patched Unicorn (once)..."
      "$root/$d/source/core/scripts/build-unicorn.sh" "$uc" "$work/unicorn-work-$m" > "$work/unicorn-$m.log" 2>&1 \
        || { echo "FAIL build.unicorn: see $work/unicorn-$m.log" | tee -a "$out/report.txt"; total_fail=$((total_fail + 1)); continue; }
    fi
  fi
  b=$work/build-$m
  if ! { cmake -S "$root/tests" -B "$b" -G Ninja -DMACHINE="$m" -DDTCORE_UNICORN_PREFIX="$uc" > "$work/build-$m.log" 2>&1 &&
         cmake --build "$b" --target seqtest >> "$work/build-$m.log" 2>&1; }; then
    echo "FAIL build.seqtest: see $work/build-$m.log" | tee -a "$out/report.txt"
    tail -n 20 "$work/build-$m.log"
    total_fail=$((total_fail + 1))
    continue
  fi
  "$b/seqtest" --layout "$out/view-$m.svg" "$out/report.txt" | sed 's/^/   /'
  total_fail=$((total_fail + ${PIPESTATUS[0]}))
  fw=$fw_dir/$(fw_of "$m")
  if [[ -f $fw ]]; then
    (cd "$work" && "$b/seqtest" --engine "$fw" "$work/state-$m.dtsz" "$out/report.txt" "$out/engine-$m.svg") | sed 's/^/   /'
    total_fail=$((total_fail + ${PIPESTATUS[0]}))
  else
    echo "SKIP engine: no firmware $fw" | tee -a "$out/report.txt"
  fi
done

# the HTML report: the lines and the pictures
{
  echo "<!doctype html><meta charset='utf-8'><title>Elektron Emu tests</title>"
  echo "<style>body{font:14px system-ui;background:#16171a;color:#ecebe4;margin:24px}"
  echo ".PASS{color:#6fd08c}.FAIL{color:#ff7a6b;font-weight:600}.SKIP{color:#9a9da3}h2{margin-top:32px}"
  echo "img,svg{max-width:100%;border:1px solid #333;margin:8px 0}</style>"
  echo "<h1>Elektron Emu tests</h1><p>$(date)</p>"
  while IFS= read -r line; do
    case $line in
      "== "*) echo "<h2>${line#== }</h2>";;
      *) s=${line%% *}; echo "<div class='$s'>$(printf '%s' "$line" | sed 's/&/\&amp;/g;s/</\&lt;/g')</div>";;
    esac
  done < "$out/report.txt"
  for m in "${machines[@]}"; do
    for f in "$out/engine-$m.svg" "$out/view-$m.svg"; do
      [[ -f $f ]] && echo "<h3>$(basename "$f")</h3><img src='$(basename "$f")'>"
    done
  done
} > "$out/report.html"

pass=$(grep -c '^PASS' "$out/report.txt" || true)
fail=$(grep -c '^FAIL' "$out/report.txt" || true)
skip=$(grep -c '^SKIP' "$out/report.txt" || true)
echo
echo "$pass passed, $fail failed, $skip skipped -> $out/report.html"
exit "$fail"
