#!/usr/bin/env bash
# Il VST3 del Model:Cycles per macOS (arm64); con INSTALL=1 installato per l'utente in
#   ~/Library/Audio/Plug-Ins/VST3/ModelCycles.vst3
#
#   vst/scripts/build-mac.sh [WORK]        (WORK predefinito: ~/.cache/mc-vst)
#
# Variabili: MC_UNICORN_PREFIX (la Unicorn con le patch; se manca la costruisce
# core/scripts/build-unicorn.sh), INSTALL=1 per installare in ~/Library (sviluppo).
#
# Il bundle NON contiene firmware o stati (solo la foto del pannello,
# Contents/Resources/cycles_panel.png, se c'e'): il firmware Elektron lo mette
# l'utente in ~/Library/Application Support/Elektron Emu/Model Cycles/Firmware/ (vst/README.md).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/mc-vst}
mkdir -p "$work"
work=$(cd "$work" && pwd)
unicorn=${MC_UNICORN_PREFIX:-$work/unicorn-mac}
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
mkdir -p "$work/mac"
cmake -S "$root/vst" -B "$build" -G Ninja -DDPF_ROOT="$work/dpf" -DDTCORE_UNICORN_PREFIX="$unicorn" \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 >/dev/null
cmake --build "$build"

b=$build/bin/ModelCycles.vst3
[[ -d $b ]] || { echo "error: $b non esiste" >&2; exit 1; }
# il bundle e' pulito: niente firmware ne' dati derivati (controllo in vst/scripts/check-clean.sh)
rm -rf "$b/Contents/Resources"
BUNDLE_RES=$b/Contents/Resources
# la foto del pannello (materiale locale, mai nel repo): MC_PANEL_IMAGE, oppure
# vst/resources/cycles_panel.png, oppure out/gui/cycles_panel.png (fatta qui da
# vst/scripts/make-panel-photo.py se manca e c'e' la foto sorgente
# ../default_webp_Elektron_Model_Cycles_11.webp). Senza: pannello schematico.
img=${MC_PANEL_IMAGE:-$root/vst/resources/cycles_panel.png}
if [[ ! -f $img && -z ${MC_PANEL_IMAGE:-} ]]; then
  img=$root/out/gui/cycles_panel.png
  [[ -f $img || ! -f $root/../default_webp_Elektron_Model_Cycles_11.webp ]] || python3 "$root/vst/scripts/make-panel-photo.py" >/dev/null
fi
if [[ -f $img ]]; then
  mkdir -p "$BUNDLE_RES"; cp "$img" "$BUNDLE_RES/cycles_panel.png"
else
  echo "warning: foto del pannello assente ($img): pannello schematico (vst/README.md)" >&2
fi
codesign -f -s - "$b" 2>/dev/null
"$root/vst/scripts/check-clean.sh" "$b"

# una sola copia per plugin: di norma la build non installa (si usa il .pkg, che va in
# /Library e toglie la copia in ~/Library). INSTALL=1: copia di sviluppo in ~/Library,
# e via quella del .pkg in /Library (se si puo' senza sudo, altrimenti avvisa)
if [[ ${INSTALL:-0} != 1 || ${NO_INSTALL:-0} == 1 ]]; then
  echo "bundle pronto (non installato; INSTALL=1 per la copia di sviluppo in ~/Library)"; exit 0
fi
if [[ -d "/Library/Audio/Plug-Ins/VST3/ModelCycles.vst3" ]]; then
  rm -rf "/Library/Audio/Plug-Ins/VST3/ModelCycles.vst3" 2>/dev/null \
    || echo "warning: resta /Library/Audio/Plug-Ins/VST3/ModelCycles.vst3 (serve sudo): la DAW vedra' due copie" >&2
fi
dir=$HOME/Library/Audio/Plug-Ins/VST3
mkdir -p "$dir"
rm -rf "${dir:?}/ModelCycles.vst3"
cp -R "$b" "$dir/ModelCycles.vst3"
echo "installato $dir/ModelCycles.vst3"
