#!/usr/bin/env bash
# Il VST3 della Syntakt per Windows x64, costruito dal Mac con mingw-w64.
#
#   vst/scripts/build-windows.sh [WORK]          (WORK predefinito: ~/.cache/st-vst)
#
# Fa: dipendenze statiche (build-deps-windows.sh), DPF a un commit fisso, il motore
# comune del Digitakt a un commit fisso (dtcore.sh), il plugin, poi lo zip
# WORK/Syntakt-VST3-win64.zip (bundle + LEGGIMI). Il bundle NON contiene firmware ne'
# stati: il plugin li cerca in %APPDATA%\Elektron Emu\Syntakt\Firmware (vst/README.md). La foto
# del pannello (DT_PANEL_IMAGE, predefinita out/gui/syntakt_panel.png) e' facoltativa e
# solo per la build locale; senza, la vista disegna un pannello schematico.
# Alla fine controlla che il bundle sia pulito (check-bundle.py) e che la DLL dipenda
# solo da DLL di sistema.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/st-vst}
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
dtcore=$("$root/vst/scripts/dtcore.sh" "$work")

cmake -S "$root/vst" -B "$prefix/vst-build" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$prefix/mingw.cmake" \
  -DDPF_ROOT="$work/dpf" -DDTCORE_DIR="$dtcore" -DDTCORE_UNICORN_PREFIX="$prefix" -DZLIB_ROOT="$prefix" >/dev/null
cmake --build "$prefix/vst-build"

bundle=$prefix/vst-build/bin/syntakt.vst3
res=$bundle/Contents/Resources
mkdir -p "$res"
# foto del pannello: vst/resources/ (export pubblico) o out/gui/ (repo di lavoro)
img=${DT_PANEL_IMAGE:-$root/vst/resources/syntakt_panel.png}
[[ -f $img || -n ${DT_PANEL_IMAGE:-} ]] || img=$root/out/gui/syntakt_panel.png
if [[ -f $img ]]; then cp "$img" "$res/syntakt_panel.png"; echo "copiata la foto del pannello"
else rm -f "$res/syntakt_panel.png"; echo "warning: foto del pannello assente ($img)"; fi

dll=$bundle/Contents/x86_64-win/syntakt.vst3
# niente simboli di debug (la libstdc++ statica con std::filesystem li porta: 20 MB invece di 4)
x86_64-w64-mingw32-strip "$dll"
if x86_64-w64-mingw32-objdump -p "$dll" | grep "DLL Name" | grep -viE "api-ms-win-crt|kernel32|user32|gdi32|opengl32|ole32|shell32|comdlg32|dwmapi|advapi32|shlwapi"; then
  echo "error: il plugin dipende da DLL non di sistema (sopra)" >&2; exit 1
fi

fw=${ST_FIRMWARE:-$root/../firmware/Syntakt_OS1.41.syx}
"$root/vst/scripts/check-bundle.py" "$bundle" "$fw" "$root"/sections/*.bin

out=$work/Syntakt-VST3-win64.zip
rm -f "$out"
cp "$root/vst/README-USER.txt" "$prefix/vst-build/bin/README.txt"
(cd "$prefix/vst-build/bin" && zip -qr "$out" syntakt.vst3 README.txt)
ls -la "$out"
