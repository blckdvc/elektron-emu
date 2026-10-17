#!/usr/bin/env bash
# Il VST3 dell'Analog Rytm MKII per macOS (arm64); con INSTALL=1 installato per l'utente in
#   ~/Library/Audio/Plug-Ins/VST3/analogrytm.vst3
#
#   vst/scripts/build-mac.sh [WORK]        (WORK predefinito: ~/.cache/ar-vst)
#
# Il bundle contiene la foto del pannello ma niente firmware, niente stati derivati, niente codice
# ricompilato dal firmware (vst/README.md). Il firmware e' dell'utente e sta in
# ~/Library/Application Support/Elektron Emu/Analog Rytm MKII/Firmware/.
#
# Variabili:
#   DTCORE_UNICORN_PREFIX  la Unicorn con le patch (core/scripts/build-unicorn.sh PREFIX),
#                          predefinito WORK/unicorn-mac (la costruisce se manca)
#   INSTALL=1              installa anche in ~/Library/Audio/Plug-Ins/VST3 (sviluppo)
#   AR_PANEL_IMAGE         foto del pannello da mettere in Contents/Resources (predefinita
#                          out/gui/rytm_panel_unlit2.png, quella della GUI v2; va anche nei pacchetti: decisione dell'utente,
#                          GITHUB.md del repo indice; senza, il pannello si disegna schematico)
#   AR_LOCAL_SETUP=1       SOLO PER LE PROVE sul proprio Mac: copia il firmware (AR_FIRMWARE,
#                          predefinito ../firmware/Analog-Rytm_MKII_OS1.73.syx) nella cartella
#                          Firmware dell'utente e, se c'e', AR_STATE come stato del primo avvio
#                          in cache. Il bundle resta pulito.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/ar-vst}
mkdir -p "$work"
work=$(cd "$work" && pwd)
unicorn=${DTCORE_UNICORN_PREFIX:-$work/unicorn-mac}
if [[ ! -f $unicorn/lib/libunicorn.a ]]; then
  "$root/core/scripts/build-unicorn.sh" "$unicorn" "$work/unicorn-work" >/dev/null
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

build=$work/mac/vst-build
cmake -S "$root/vst" -B "$build" -G Ninja -DDPF_ROOT="$work/dpf" -DDTCORE_UNICORN_PREFIX="$unicorn" \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 >/dev/null
cmake --build "$build"

bundle=$build/bin/analogrytm.vst3
res=$bundle/Contents/Resources
mkdir -p "$res"
# foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
img=${AR_PANEL_IMAGE:-$root/vst/resources/rytm_panel.png}
[[ -f $img || -n ${AR_PANEL_IMAGE:-} ]] || img=$root/out/gui/rytm_panel_unlit2.png
if [[ -f $img ]]; then cp "$img" "$res/rytm_panel.png"; else rm -f "$res/rytm_panel.png"; echo "warning: foto del pannello assente ($img): pannello schematico" >&2; fi
codesign -f -s - "$bundle" 2>/dev/null

if [[ ${AR_LOCAL_SETUP:-0} == 1 ]]; then
  fw=${AR_FIRMWARE:-$root/../firmware/Analog-Rytm_MKII_OS1.73.syx}
  data="$HOME/Library/Application Support/Elektron Emu/Analog Rytm MKII"
  mkdir -p "$data/Firmware"
  cp "$fw" "$data/Firmware/"
  echo "firmware copiato in $data/Firmware (solo prove locali)"
  if [[ -n ${AR_STATE:-} && -f ${AR_STATE} ]]; then
    sha8=$(shasum -a 256 "$fw" | cut -c1-16)
    mkdir -p "$HOME/Library/Caches/Elektron Emu/Analog Rytm MKII"
    cp "$AR_STATE" "$HOME/Library/Caches/Elektron Emu/Analog Rytm MKII/primo-avvio-$sha8-g1.dtsz"
    echo "stato del primo avvio in cache (solo prove locali)"
  fi
fi

# una sola copia per plugin: di norma la build non installa (si usa il .pkg, che va in
# /Library e toglie la copia in ~/Library). INSTALL=1: copia di sviluppo in ~/Library,
# e via quella del .pkg in /Library (se si puo' senza sudo, altrimenti avvisa)
if [[ ${INSTALL:-0} != 1 || ${NO_INSTALL:-0} == 1 ]]; then
  echo "bundle pronto (non installato; INSTALL=1 per la copia di sviluppo in ~/Library)"; exit 0
fi
if [[ -d "/Library/Audio/Plug-Ins/VST3/analogrytm.vst3" ]]; then
  rm -rf "/Library/Audio/Plug-Ins/VST3/analogrytm.vst3" 2>/dev/null \
    || echo "warning: resta /Library/Audio/Plug-Ins/VST3/analogrytm.vst3 (serve sudo): la DAW vedra' due copie" >&2
fi
dir=$HOME/Library/Audio/Plug-Ins/VST3
mkdir -p "$dir"
rm -rf "$dir/analogrytm.vst3"
cp -R "$bundle" "$dir/analogrytm.vst3"
echo "installato $dir/analogrytm.vst3"
