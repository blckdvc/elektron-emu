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
#    repo): nessuna stringa del firmware lunga 20+ caratteri nei binari del bundle,
#    tolte quelle dei runtime C++ di GCC (check-bundle-allow.txt e le librerie di mingw).
# Esce con 1 al primo problema. Lo chiamano build-mac.sh e build-windows.sh.
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
big=$(find "$target" -type f -size +32M -print)
if [[ -n $big ]]; then
  echo "check-bundle: file oltre 32 MB (firmware o stato incorporato?):" >&2
  echo "$big" >&2
  exit 1
fi

sections=$root/sections
if compgen -G "$sections/section_*.bin" >/dev/null; then
  fw=$(mktemp)
  mine=$(mktemp)
  strings -n 20 "$sections"/section_*.bin | sort -u >"$fw"
  find "$target" -type f -exec strings -n 20 {} + | sort -u >"$mine"
  # il firmware e' compilato con GCC: le stringhe dei runtime C++ (libstdc++,
  # libsupc++, libgcc, che il .vst3 Windows ha statici) non contano
  rt=$(mktemp)
  {
    grep -v '^#' "$root/vst/scripts/check-bundle-allow.txt"
    if command -v x86_64-w64-mingw32-g++ >/dev/null; then
      for l in libstdc++.a libsupc++.a libgcc.a libgcc_eh.a; do
        f=$(x86_64-w64-mingw32-g++ -print-file-name=$l)
        [[ -f $f ]] && strings -n 20 "$f"
      done
    fi
  } | sort -u >"$rt"
  common=$(comm -12 "$fw" "$mine" | comm -23 - "$rt")
  rm -f "$fw" "$mine" "$rt"
  if [[ -n $common ]]; then
    echo "check-bundle: stringhe del firmware nei binari del bundle:" >&2
    echo "$common" | head -20 >&2
    exit 1
  fi
  echo "check-bundle: ok ($target; nessuna stringa delle sezioni del firmware)"
else
  echo "check-bundle: ok ($target; sezioni del firmware assenti, controllo delle stringhe saltato)"
fi
