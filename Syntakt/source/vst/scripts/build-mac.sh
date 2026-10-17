#!/usr/bin/env bash
# Il VST3 della Syntakt per macOS (arm64); con INSTALL=1 installato per l'utente in
#   ~/Library/Audio/Plug-Ins/VST3/syntakt.vst3
#
#   vst/scripts/build-mac.sh [WORK]        (WORK predefinito: ~/.cache/st-vst)
#
# Il bundle NON contiene il firmware ne' stati derivati: il plugin li cerca nelle
# cartelle dell'utente (vst/README.md). Per comodita' della build locale:
#   ST_PRECOPY=1   copia il firmware (ST_FIRMWARE, predefinito ../firmware/Syntakt_OS1.41.syx)
#                  in ~/Library/Application Support/Elektron Emu/Syntakt/Firmware/ (e, se c'e'
#                  ST_STATE_FILE, non lo mette in cache: lo stato nasce dal boot freddo).
# Altre variabili: DTCORE_UNICORN_PREFIX (la Unicorn con le patch,
# core/scripts/build-unicorn.sh), DT_PANEL_IMAGE (la foto del pannello, facoltativa:
# senza il plugin disegna un pannello schematico), INSTALL=1 (copia di sviluppo in ~/Library).
# Alla fine controlla che nel bundle non ci sia niente del firmware (check-bundle.py).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/st-vst}
mkdir -p "$work"
work=$(cd "$work" && pwd)
unicorn=${DTCORE_UNICORN_PREFIX:-$work/unicorn-m68k-st}
if [[ ! -f $unicorn/lib/libunicorn.a ]]; then
  "$root/core/scripts/build-unicorn.sh" "$unicorn" "$work/unicorn-work"
fi
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
dtcore=$("$root/vst/scripts/dtcore.sh" "$work")

build=$work/mac/vst-build
cmake -S "$root/vst" -B "$build" -G Ninja -DDPF_ROOT="$work/dpf" -DDTCORE_DIR="$dtcore" \
  -DDTCORE_UNICORN_PREFIX="$unicorn" -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 >/dev/null
cmake --build "$build"

bundle=$build/bin/syntakt.vst3
res=$bundle/Contents/Resources
mkdir -p "$res"
# foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
img=${DT_PANEL_IMAGE:-$root/vst/resources/syntakt_panel.png}
[[ -f $img || -n ${DT_PANEL_IMAGE:-} ]] || img=$root/out/gui/syntakt_panel.png
if [[ -f $img ]]; then cp "$img" "$res/syntakt_panel.png"; else rm -f "$res/syntakt_panel.png"; echo "warning: foto del pannello assente ($img)"; fi
codesign -f -s - "$bundle" 2>/dev/null

fw=${ST_FIRMWARE:-$root/../firmware/Syntakt_OS1.41.syx}
"$root/vst/scripts/check-bundle.py" "$bundle" "$fw" "$root"/sections/*.bin

if [[ ${ST_PRECOPY:-0} == 1 && -f $fw ]]; then
  fwdir=$HOME/Library/Application\ Support/Elektron Emu/Syntakt/Firmware
  mkdir -p "$fwdir"
  cp "$fw" "$fwdir/"
  echo "firmware copiato in $fwdir (fuori dal bundle)"
fi

# una sola copia per plugin: di norma la build non installa (si usa il .pkg, che va in
# /Library e toglie la copia in ~/Library). INSTALL=1: copia di sviluppo in ~/Library,
# e via quella del .pkg in /Library (se si puo' senza sudo, altrimenti avvisa)
if [[ ${INSTALL:-0} != 1 || ${NO_INSTALL:-0} == 1 ]]; then
  echo "bundle pronto (non installato; INSTALL=1 per la copia di sviluppo in ~/Library)"; exit 0
fi
if [[ -d "/Library/Audio/Plug-Ins/VST3/syntakt.vst3" ]]; then
  rm -rf "/Library/Audio/Plug-Ins/VST3/syntakt.vst3" 2>/dev/null \
    || echo "warning: resta /Library/Audio/Plug-Ins/VST3/syntakt.vst3 (serve sudo): la DAW vedra' due copie" >&2
fi
dir=$HOME/Library/Audio/Plug-Ins/VST3
mkdir -p "$dir"
rm -rf "${dir:?}/syntakt.vst3"
cp -R "$bundle" "$dir/syntakt.vst3"
echo "installato $dir/syntakt.vst3"
