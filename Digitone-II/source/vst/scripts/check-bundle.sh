#!/usr/bin/env bash
# Controlla che un bundle del plugin (cartella .vst3 o zip di distribuzione) non
# contenga firmware o stati della macchina. La foto del pannello
# (Contents/Resources/*_panel.png) e' ammessa: decisione dell'utente (GITHUB.md del repo indice).
#
#   vst/scripts/check-bundle.sh BUNDLE.vst3|ARCHIVIO.zip
#
# 1. nessun file con estensione vietata (.syx .swu .dtst .dtsz .snap .raw .bin,
#    immagini .png .jpg .jpeg .webp tranne Resources/*_panel.png);
# 2. nessun file oltre 32 MB;
# 3. se in locale ci sono le sezioni estratte del firmware (sections/, mai nel
#    repo): nessuna stringa del firmware lunga 20+ caratteri nei binari del bundle.
# Esce con 1 al primo problema. Lo chiamano build-mac.sh e build-windows.sh.
# ALLOW_STATIC_DSP=1: build di sviluppo col DSP ricompilato dal firmware dentro il binario
# (derivato dal firmware: uso personale): salta i punti 2 e 3 e lo dice. Gli installer
# non accettano queste build.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
[[ $# -eq 1 ]] || { echo "uso: $0 BUNDLE.vst3|ARCHIVIO.zip" >&2; exit 2; }
target=$1
tmp=
cleanup() { [[ -n $tmp ]] && rm -rf "$tmp"; return 0; }
trap cleanup EXIT
if [[ -f $target && $target == *.zip ]]; then
  tmp=$(mktemp -d)
  unzip -q "$target" -d "$tmp"
  target=$tmp
fi
[[ -d $target ]] || { echo "check-bundle: $target non esiste" >&2; exit 2; }

bad=$(find "$target" -type f \( -iname '*.syx' -o -iname '*.swu' -o -iname '*.dtst' -o -iname '*.dtsz' \
  -o -iname '*.snap' -o -iname '*.raw' -o -iname '*.bin' -o -iname '*.png' -o -iname '*.jpg' \
  -o -iname '*.jpeg' -o -iname '*.webp' \) ! -path '*/Contents/Resources/*_panel.png' -print)
if [[ -n $bad ]]; then
  echo "check-bundle: file vietati nel bundle:" >&2
  echo "$bad" >&2
  exit 1
fi
if [[ ${ALLOW_STATIC_DSP:-0} == 1 ]]; then
  echo "check-bundle: BUILD PER USO PERSONALE (DSP derivato dal firmware nel binario): $target"
  exit 0
fi
big=$(find "$target" -type f -size +32M -print)
if [[ -n $big ]]; then
  echo "check-bundle: file oltre 32 MB (firmware o stato incorporato?):" >&2
  echo "$big" >&2
  exit 1
fi

sections=$root/sections
if compgen -G "$sections/section_*.bin" >/dev/null; then
  # 1. stringhe lunghe del firmware, tolte quelle della libreria C++ (le stesse in qualunque
  #    programma C++ e nel firmware, che e' C++: simboli RTTI, std::, __cxxabi)
  fw=$(mktemp)
  mine=$(mktemp)
  strings -n 20 "$sections"/section_*.bin | sort -u >"$fw"
  find "$target" -type f -exec strings -n 20 {} + | sort -u >"$mine"
  # le stringhe che sono anche nelle librerie del compilatore (libstdc++, libgcc, mingw) o nella
  # libc++ del sistema non vengono dal firmware, anche se il firmware (C++) ne ha di uguali
  allow=$(mktemp)
  for lib in libstdc++.a libgcc.a libgcc_eh.a libwinpthread.a libmingwex.a libmingw32.a libmsvcrt.a libstdc++.dll.a; do
    f=$(x86_64-w64-mingw32-gcc -print-file-name=$lib 2>/dev/null || true)
    [[ -f $f ]] && strings -n 20 "$f"
  done | sort -u >"$allow"
  [[ -f /usr/lib/libc++.1.dylib ]] && strings -n 20 /usr/lib/libc++.1.dylib | sort -u >>"$allow"
  sort -u -o "$allow" "$allow"
  common=$(comm -12 "$fw" "$mine" | comm -23 - "$allow" | grep -vxE 'ABCDEFGHIJKLMNOPQRSTUVWXYZ|terminate called recursively|vector::_M_default_append|vector::_M_realloc_insert' || true)
  rm -f "$allow"
  rm -f "$fw" "$mine"
  if [[ -n $common ]]; then
    echo "check-bundle: stringhe del firmware nei binari del bundle:" >&2
    echo "$common" | head -20 >&2
    exit 1
  fi
  # 2. finestre di byte del firmware (codice o dati dentro il binario)
  pub=()
  for lib in libstdc++.a libgcc.a libgcc_eh.a libwinpthread.a libmingwex.a libmingw32.a libmsvcrt.a libsupc++.a; do
    f=$(x86_64-w64-mingw32-gcc -print-file-name=$lib 2>/dev/null || true)
    [[ -f $f ]] && pub+=("$f")
  done
  for f in "${DTCORE_UNICORN_PREFIX:-$HOME/.cache/dn-vst/unicorn-mac}/lib/libunicorn.a" "$HOME/.cache/dn-vst/win64/lib/libunicorn.a" "$HOME/.cache/dn-vst/win64/lib/libz.a"; do
    [[ -f $f ]] && pub+=("$f")
  done
  pub+=("@$root/core/src/dn_engine.c" "@$root/core/src/native/dn_native.c")
  python3 "$root/vst/scripts/check-bytes.py" "$sections" "$target" "${pub[@]}" || exit 1
  echo "check-bundle: ok ($target; nessuna stringa ne' blocco di byte delle sezioni del firmware)"
else
  echo "check-bundle: ok ($target; sezioni del firmware assenti, controllo delle stringhe saltato)"
fi
