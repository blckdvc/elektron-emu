#!/usr/bin/env bash
# Il VST3 del Model:Cycles per Windows x64, costruito dal Mac con mingw-w64.
#
#   vst/scripts/build-windows.sh [WORK]          (WORK predefinito: ~/.cache/mc-vst)
#
# Fa: dipendenze statiche (build-deps-windows.sh, prefisso WORK/win64), DPF a
# un commit fisso, il plugin e lo zip WORK/ModelCycles-VST3-win64.zip. Nel
# bundle niente firmware o stati (solo la foto del pannello): check-bundle.sh e il
# controllo sui file attesi fanno fallire la build. Il firmware lo mette
# l'utente in %APPDATA%\Elektron Emu\Model Cycles\Firmware (vst/README-WINDOWS.txt).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:-$HOME/.cache/mc-vst}
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
cmake --build "$prefix/vst-build" --target ModelCycles-vst3

bundle=$prefix/vst-build/bin/ModelCycles.vst3
# il bundle e' codice + la foto del pannello: niente altro in Resources
rm -rf "$bundle/Contents/Resources"
BUNDLE_RES=$bundle/Contents/Resources
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

dll=$bundle/Contents/x86_64-win/ModelCycles.vst3
[[ -f $dll ]] || { echo "error: manca $dll" >&2; exit 1; }
# niente simboli di debug (la libstdc++ statica con std::filesystem li porta)
x86_64-w64-mingw32-strip "$dll"
if x86_64-w64-mingw32-objdump -p "$dll" | grep "DLL Name" | grep -viE "api-ms-win-crt|kernel32|user32|gdi32|opengl32|ole32|shell32|comdlg32|dwmapi"; then
  echo "error: il plugin dipende da DLL non di sistema (sopra)" >&2; exit 1
fi

# file attesi: la DLL e la foto (piu' al massimo moduleinfo.json/desktop.ini); il resto fa fallire
unexpected=$(cd "$bundle" && find . -type f ! -path './Contents/x86_64-win/ModelCycles.vst3' \
  ! -path './Contents/Resources/cycles_panel.png' ! -name moduleinfo.json ! -name desktop.ini -print)
if [[ -n $unexpected ]]; then
  echo "error: file inattesi nel bundle:" >&2; echo "$unexpected" >&2; exit 1
fi
# le stringhe che stanno anche nelle librerie di runtime della toolchain (libstdc++ del
# MAIN OS compilato con GCC) non sono del firmware
rtlibs=
for l in libstdc++.a libgcc.a libgcc_eh.a libmingwex.a libmingw32.a libmsvcrt.a; do
  rtlibs="$rtlibs $(x86_64-w64-mingw32-g++ -print-file-name=$l)"
done
export MC_RUNTIME_LIBS=$rtlibs
"$root/vst/scripts/check-bundle.sh" "$bundle"

out=$work/ModelCycles-VST3-win64.zip
rm -f "$out"
cp "$root/vst/README-WINDOWS.txt" "$prefix/vst-build/bin/README.txt"
(cd "$prefix/vst-build/bin" && zip -qr "$out" ModelCycles.vst3 README.txt)
"$root/vst/scripts/check-bundle.sh" "$out"
ls -la "$out"
