#!/usr/bin/env bash
# Il VST3 del Digitone II per Windows x64, costruito dal Mac con mingw-w64.
#
#   vst/scripts/build-windows.sh [WORK]          (WORK predefinito: ~/.cache/dn-vst)
#
# Fa: dipendenze statiche (build-deps-windows.sh in WORK/win64), DPF a un commit fisso, il plugin
# (WORK/win64/vst-build/bin/Digitone II.vst3) e lo zip WORK/Digitone-II-VST3-win64.zip. Nel bundle
# la foto del pannello ma niente firmware ne' stati (check-bundle.sh): il firmware lo mette l'utente in
# %APPDATA%\Elektron Emu\Digitone II\Firmware. DNCORE_DSP_DIR non si usa a Windows (la libreria del DSP
# ricompilato e' solo Mac arm64): il DSP e' l'interprete (core/dsp/dn_interp.c).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/dn-vst}
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
if ! git -C "$work/dpf" apply -R --check "$root/vst/patches/dpf-offline.patch" 2>/dev/null; then
  git -C "$work/dpf" checkout --quiet -- .
  git -C "$work/dpf" apply "$root/vst/patches/dpf-offline.patch"
fi

cmake -S "$root/vst" -B "$prefix/vst-build" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$prefix/mingw.cmake" \
  -DDPF_ROOT="$work/dpf" -DDTCORE_UNICORN_PREFIX="$prefix" -DZLIB_ROOT="$prefix" >/dev/null
cmake --build "$prefix/vst-build"

out=$prefix/vst-build/bin
rm -rf "$out/Digitone II.vst3"
mv "$out/digitone2.vst3" "$out/Digitone II.vst3"
mv "$out/Digitone II.vst3/Contents/x86_64-win/digitone2.vst3" "$out/Digitone II.vst3/Contents/x86_64-win/Digitone II.vst3"
bundle=$out/Digitone\ II.vst3
dll=$bundle/Contents/x86_64-win/Digitone\ II.vst3
x86_64-w64-mingw32-strip "$dll"
# solo DLL di sistema (il resto e' statico)
if x86_64-w64-mingw32-objdump -p "$dll" | grep "DLL Name" | grep -viE "api-ms-win-crt|kernel32|user32|gdi32|opengl32|ole32|shell32|comdlg32|dwmapi|advapi32|msvcrt|winhttp"; then
  echo "error: il plugin dipende da DLL non di sistema (sopra)" >&2; exit 1
fi

# la foto del pannello (decisione dell'utente, GITHUB.md del repo indice)
# foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
img=${DN_PANEL_IMAGE:-$root/vst/resources/digitone2_panel.png}
[[ -f $img || -n ${DN_PANEL_IMAGE:-} ]] || img=$root/out/gui/digitone2_panel.png
mkdir -p "$bundle/Contents/Resources"
if [[ -f $img ]]; then cp "$img" "$bundle/Contents/Resources/digitone2_panel.png"
else rm -f "$bundle/Contents/Resources/digitone2_panel.png"; echo "warning: foto del pannello assente ($img): pannello schematico" >&2; fi
"$root/vst/scripts/check-bundle.sh" "$bundle"

zip=$work/Digitone-II-VST3-win64.zip
rm -f "$zip"
cp "$root/vst/installer/windows/README.txt" "$out/README.txt"
(cd "$out" && zip -qr "$zip" "Digitone II.vst3" README.txt)
ls -la "$zip"
