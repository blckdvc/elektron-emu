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
#    Il MAIN OS e' compilato con GCC e porta stringhe della libstdc++ (nomi dei
#    tipi, "basic_string::append"...), che stanno anche nel plugin: le stringhe
#    che compaiono in MC_RUNTIME_LIBS (file .a della toolchain, separati da
#    spazio) non contano. Se ne restano, il controllo fallisce.
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
  common=$(comm -12 "$fw" "$mine")
  if [[ -n $common && -n ${MC_RUNTIME_LIBS:-} ]]; then
    rt=$(mktemp)
    for lib in $MC_RUNTIME_LIBS; do strings -n 4 "$lib"; done | sort -u >"$rt"
    left=
    while IFS= read -r line; do
      grep -qF -- "$line" "$rt" || left="$left$line"$'\n'
    done <<<"$common"
    common=${left%$'\n'}
    rm -f "$rt"
  fi
  rm -f "$fw" "$mine"
  # messaggi generici della libstdc++ (check-bundle-allow.txt)
  allow=$(dirname "${BASH_SOURCE[0]}")/check-bundle-allow.txt
  if [[ -n $common && -f $allow ]]; then
    common=$(grep -vxFf <(grep -v '^#' "$allow") <<<"$common" || true)
  fi
  if [[ -n $common ]]; then
    echo "check-bundle: stringhe del firmware nei binari del bundle:" >&2
    echo "$common" | head -20 >&2
    exit 1
  fi
  echo "check-bundle: ok ($target; nessuna stringa delle sezioni del firmware)"
else
  echo "check-bundle: ok ($target; sezioni del firmware assenti, controllo delle stringhe saltato)"
fi
