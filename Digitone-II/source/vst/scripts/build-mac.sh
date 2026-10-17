#!/usr/bin/env bash
# Il VST3 del Digitone II per macOS (arm64); con INSTALL=1 installato per l'utente in
#   ~/Library/Audio/Plug-Ins/VST3/Digitone II.vst3
#
#   vst/scripts/build-mac.sh [WORK]        (WORK predefinito: ~/.cache/dn-vst)
#
# Variabili:
#   DTCORE_UNICORN_PREFIX  la Unicorn a 15 patch (core/scripts/build-unicorn.sh PREFIX);
#                          predefinita WORK/unicorn-mac
#   DNCORE_DSP_DIR         il DSP ricompilato dal firmware (libdndsp.a): SOLO sviluppo e uso
#                          personale, il bundle che ne viene fuori NON si distribuisce
#                          (scrive DSP_BACKEND=static accanto alla build; gli installer lo
#                          rifiutano). Senza, il DSP e' l'interprete (core/dsp/dn_interp.c).
#   DEV_DSP=1              scorciatoia: DNCORE_DSP_DIR=WORK/dncore-dsp (finche' l'interprete
#                          non e' pronto)
#   INSTALL=1              installa anche in ~/Library/Audio/Plug-Ins/VST3 (sviluppo)
#   DN_PANEL_IMAGE         la foto del pannello (predefinita out/gui/digitone2_panel.png): va in
#                          Contents/Resources, anche nei pacchetti (decisione dell'utente,
#                          GITHUB.md del repo indice)
# Il bundle non contiene firmware, stati ne' DSP derivato dal firmware (con l'interprete):
# check-bundle.sh lo controlla. Il firmware lo mette l'utente in
# ~/Library/Application Support/Elektron Emu/Digitone II/Firmware (vst/README.md).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/dn-vst}
mkdir -p "$work"
work=$(cd "$work" && pwd)
unicorn=${DTCORE_UNICORN_PREFIX:-$work/unicorn-mac}
[[ -f $unicorn/lib/libunicorn.a ]] || "$root/core/scripts/build-unicorn.sh" "$unicorn" "$work/unicorn-work"
dpf_commit=4238e1c7f0351bbe488d79f0899c540543ac7583

dsp_dir=${DNCORE_DSP_DIR:-}
if [[ -z $dsp_dir && ${DEV_DSP:-0} == 1 ]]; then
  dsp_dir=$work/dncore-dsp
  echo "AVVISO: DSP ricompilato di sviluppo ($dsp_dir): bundle SOLO per uso personale, non si distribuisce" >&2
fi

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
  -DDNCORE_DSP_DIR="$dsp_dir" -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 >/dev/null
cmake --build "$build"
if [[ -n $dsp_dir ]]; then echo static > "$work/mac/DSP_BACKEND"; else echo interpreter > "$work/mac/DSP_BACKEND"; fi

bundle=$build/bin/digitone2.vst3
[[ -d $bundle ]] || { echo "manca $bundle" >&2; exit 1; }
# il nome che l'utente vede: "Digitone II.vst3"
rm -rf "$build/bin/Digitone II.vst3"
mv "$bundle" "$build/bin/Digitone II.vst3"
bundle=$build/bin/Digitone\ II.vst3
mv "$bundle/Contents/MacOS/digitone2" "$bundle/Contents/MacOS/Digitone II"
plutil -replace CFBundleExecutable -string "Digitone II" "$bundle/Contents/Info.plist"
# foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
img=${DN_PANEL_IMAGE:-$root/vst/resources/digitone2_panel.png}
[[ -f $img || -n ${DN_PANEL_IMAGE:-} ]] || img=$root/out/gui/digitone2_panel.png
mkdir -p "$bundle/Contents/Resources"
if [[ -f $img ]]; then cp "$img" "$bundle/Contents/Resources/digitone2_panel.png"
else rm -f "$bundle/Contents/Resources/digitone2_panel.png"; echo "warning: foto del pannello assente ($img): pannello schematico" >&2; fi
ALLOW_STATIC_DSP=$([[ -n $dsp_dir ]] && echo 1 || echo 0) "$root/vst/scripts/check-bundle.sh" "$bundle"
# JIT di Unicorn: hardened runtime non serve in locale; per la firma vera vst/INSTALLER.md
codesign -f -s - "$bundle" 2>/dev/null

# una sola copia per plugin: di norma la build non installa (si usa il .pkg, che va in
# /Library e toglie la copia in ~/Library). INSTALL=1: copia di sviluppo in ~/Library,
# e via quella del .pkg in /Library (se si puo' senza sudo, altrimenti avvisa)
if [[ ${INSTALL:-0} != 1 || ${NO_INSTALL:-0} == 1 ]]; then
  echo "bundle pronto (non installato; INSTALL=1 per la copia di sviluppo in ~/Library)"; exit 0
fi
if [[ -d "/Library/Audio/Plug-Ins/VST3/Digitone II.vst3" ]]; then
  rm -rf "/Library/Audio/Plug-Ins/VST3/Digitone II.vst3" 2>/dev/null \
    || echo "warning: resta /Library/Audio/Plug-Ins/VST3/Digitone II.vst3 (serve sudo): la DAW vedra' due copie" >&2
fi
dir=$HOME/Library/Audio/Plug-Ins/VST3
mkdir -p "$dir"
rm -rf "$dir/Digitone II.vst3"
cp -R "$bundle" "$dir/Digitone II.vst3"
codesign -f -s - "$dir/Digitone II.vst3" 2>/dev/null
echo "installato $dir/Digitone II.vst3"
