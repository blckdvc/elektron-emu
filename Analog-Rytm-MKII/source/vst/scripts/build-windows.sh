#!/usr/bin/env bash
# Il VST3 dell'Analog Rytm MKII per Windows x64, costruito dal Mac con mingw-w64.
#
#   vst/scripts/build-windows.sh [WORK]          (WORK predefinito: ~/.cache/ar-vst)
#
# Fa: dipendenze statiche (build-deps-windows.sh), DPF a un commit fisso, il
# plugin e lo zip WORK/AnalogRytm-VST3-win64.zip. Nel bundle la foto del pannello
# (decisione dell'utente, GITHUB.md del repo indice) ma niente firmware ne' stati
# derivati (vst/README.md). L'installer vero (.exe) lo fa
# vst/scripts/make-installer-windows.sh da questo bundle.
# Variabili: AR_PANEL_IMAGE (foto del pannello, predefinita out/gui/rytm_panel_unlit2.png, la foto coi LED spenti della GUI v2).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/ar-vst}
mkdir -p "$work"
work=$(cd "$work" && pwd)
prefix=$work/win64
dpf_commit=4238e1c7f0351bbe488d79f0899c540543ac7583

"$root/vst/scripts/build-deps-windows.sh" "$prefix" >/dev/null

if [[ ! -d $work/dpf/.git ]]; then
  git clone --quiet https://github.com/DISTRHO/DPF.git "$work/dpf"
fi
if [[ $(git -C "$work/dpf" rev-parse HEAD) != "$dpf_commit" ]]; then
  git -C "$work/dpf" fetch --quiet origin "$dpf_commit" 2>/dev/null || git -C "$work/dpf" fetch --quiet --unshallow || true
  git -C "$work/dpf" checkout --quiet "$dpf_commit"
fi
git -C "$work/dpf" submodule update --init --recursive --quiet
# Plugin::isRenderingOffline() dal process_mode del VST3 (export della DAW)
if ! git -C "$work/dpf" apply -R --check "$root/vst/patches/dpf-offline.patch" 2>/dev/null; then
  git -C "$work/dpf" checkout --quiet -- .
  git -C "$work/dpf" apply "$root/vst/patches/dpf-offline.patch"
fi

cmake -S "$root/vst" -B "$prefix/vst-build" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$prefix/mingw.cmake" \
  -DDPF_ROOT="$work/dpf" -DDTCORE_UNICORN_PREFIX="$prefix" -DZLIB_ROOT="$prefix" >/dev/null
cmake --build "$prefix/vst-build"

bundle=$prefix/vst-build/bin/analogrytm.vst3
res=$bundle/Contents/Resources
mkdir -p "$res"
# foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
img=${AR_PANEL_IMAGE:-$root/vst/resources/rytm_panel.png}
[[ -f $img || -n ${AR_PANEL_IMAGE:-} ]] || img=$root/out/gui/rytm_panel_unlit2.png
if [[ -f $img ]]; then cp "$img" "$res/rytm_panel.png"; else rm -f "$res/rytm_panel.png"; echo "warning: foto del pannello assente ($img): pannello schematico" >&2; fi

dll=$bundle/Contents/x86_64-win/analogrytm.vst3
# niente simboli di debug (la libstdc++ statica con std::filesystem li porta: 20 MB invece di 4)
x86_64-w64-mingw32-strip "$dll"
if x86_64-w64-mingw32-objdump -p "$dll" | grep "DLL Name" | grep -viE "api-ms-win-crt|kernel32|user32|gdi32|opengl32|ole32|shell32|comdlg32|dwmapi"; then
  echo "error: il plugin dipende da DLL non di sistema (sopra)" >&2; exit 1
fi

out=$work/AnalogRytm-VST3-win64.zip
rm -f "$out"
(cd "$prefix/vst-build/bin" && zip -qr "$out" analogrytm.vst3)
ls -la "$out"
