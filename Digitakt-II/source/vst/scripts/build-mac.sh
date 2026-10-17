#!/usr/bin/env bash
# Il VST3 del Digitakt II per macOS (arm64); con INSTALL=1 installato per l'utente in
#   ~/Library/Audio/Plug-Ins/VST3/digitakt2.vst3
# (si sviluppa solo il VST3, per Mac e Windows: vst/README.md)
#
#   vst/scripts/build-mac.sh [WORK]        (WORK predefinito: ~/.cache/dt2-vst)
#
# Variabili: DTCORE_UNICORN_PREFIX (la Unicorn con le patch, core/scripts/build-unicorn.sh),
# DT_PANEL_IMAGE (foto del pannello, predefinita out/gui/digitakt2_panel.png),
# INSTALL=1 per installare anche in ~/Library/Audio/Plug-Ins/VST3 (copia di sviluppo:
# toglie quella del .pkg in /Library, se c'e' e si puo').
# Il bundle contiene la foto del pannello (decisione dell'utente, GITHUB.md del repo
# indice) ma non firmware ne' stati: il firmware lo mette l'utente in
# ~/Library/Application Support/Elektron Emu/Digitakt II/Firmware (vst/README.md).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/dt2-vst}
mkdir -p "$work"
work=$(cd "$work" && pwd)
unicorn=${DTCORE_UNICORN_PREFIX:-$work/unicorn-mac}
[[ -f $unicorn/lib/libunicorn.a ]] || "$root/core/scripts/build-unicorn.sh" "$unicorn" "$work/unicorn-work"
dpf_commit=4238e1c7f0351bbe488d79f0899c540543ac7583

if [[ ! -d $work/dpf/.git ]]; then
  git clone --quiet https://github.com/DISTRHO/DPF.git "$work/dpf"
fi
if [[ $(git -C "$work/dpf" rev-parse HEAD) != "$dpf_commit" ]]; then
  git -C "$work/dpf" fetch --quiet origin "$dpf_commit" 2>/dev/null || git -C "$work/dpf" fetch --quiet --unshallow || true
  git -C "$work/dpf" checkout --quiet "$dpf_commit"
fi
git -C "$work/dpf" submodule update --init --recursive --quiet
if ! git -C "$work/dpf" apply -R --check "$root/vst/patches/dpf-offline.patch" 2>/dev/null; then
  git -C "$work/dpf" checkout --quiet -- .
  git -C "$work/dpf" apply "$root/vst/patches/dpf-offline.patch"
fi

build=$work/mac/vst-build
cmake -S "$root/vst" -B "$build" -G Ninja -DDPF_ROOT="$work/dpf" -DDTCORE_UNICORN_PREFIX="$unicorn" \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 >/dev/null
cmake --build "$build"

for b in "$build/bin/digitakt2.vst3"; do
  [[ -d $b ]] || continue
  # le build fino al 2026-10-16 copiavano qui firmware e stato: via
  rm -f "$b/Contents/Resources/firmware.syx" "$b/Contents/Resources/state.dtst"
  # foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
  img=${DT_PANEL_IMAGE:-$root/vst/resources/digitakt2_panel.png}
  [[ -f $img || -n ${DT_PANEL_IMAGE:-} ]] || img=$root/out/gui/digitakt2_panel.png
  mkdir -p "$b/Contents/Resources"
  if [[ -f $img ]]; then cp "$img" "$b/Contents/Resources/digitakt2_panel.png"
  else rm -f "$b/Contents/Resources/digitakt2_panel.png"; echo "warning: foto del pannello assente ($img): pannello schematico" >&2; fi
  "$root/vst/scripts/check-bundle.sh" "$b"
  codesign -f -s - "$b" 2>/dev/null
done

# una sola copia per plugin: di norma la build non installa (si usa il .pkg, che va in
# /Library e toglie la copia in ~/Library). INSTALL=1: copia di sviluppo in ~/Library,
# e via quella del .pkg in /Library (se si puo' senza sudo, altrimenti avvisa)
if [[ ${INSTALL:-0} != 1 || ${NO_INSTALL:-0} == 1 ]]; then
  echo "bundle pronto (non installato; INSTALL=1 per la copia di sviluppo in ~/Library)"; exit 0
fi
if [[ -d "/Library/Audio/Plug-Ins/VST3/digitakt2.vst3" ]]; then
  rm -rf "/Library/Audio/Plug-Ins/VST3/digitakt2.vst3" 2>/dev/null \
    || echo "warning: resta /Library/Audio/Plug-Ins/VST3/digitakt2.vst3 (serve sudo): la DAW vedra' due copie" >&2
fi
plug=$HOME/Library/Audio/Plug-Ins
for pair in "VST3:digitakt2.vst3"; do
  dir=$plug/${pair%%:*}; name=${pair#*:}
  [[ -d $build/bin/$name ]] || continue
  mkdir -p "$dir"
  rm -rf "${dir:?}/$name"
  cp -R "$build/bin/$name" "$dir/$name"
  echo "installato $dir/$name"
done
