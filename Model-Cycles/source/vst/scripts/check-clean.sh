#!/usr/bin/env bash
# Controlla che un bundle (o un installer, o uno zip) NON contenga il firmware:
# estrae le sezioni dal .syx dell'utente e cerca nel bundle 200 blocchi da 32
# byte presi a caso dal MAIN OS (decompresso), dal container e dalle altre
# sezioni, piu' l'impronta SHA-256 del MAIN OS in esadecimale. Controlla anche
# che non ci siano .syx, stati (.mcsz/.dtst/.snap) ne' immagini, tranne la foto del
# pannello Contents/Resources/*_panel.png (ammessa: decisione dell'utente, GITHUB.md del repo indice).
#
#   vst/scripts/check-clean.sh BUNDLE_O_FILE... [--syx FIRMWARE.syx]
#
# Esce con 1 se trova qualcosa. Il firmware predefinito e' quello di prova locale.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
syx=$root/../firmware/model-cycles_OS1.13.syx
targets=()
while [[ $# -gt 0 ]]; do
  case $1 in --syx) syx=$2; shift 2;; *) targets+=("$1"); shift;; esac
done
[[ ${#targets[@]} -gt 0 ]] || { echo "uso: $0 BUNDLE... [--syx F.syx]" >&2; exit 2; }
if [[ ! -f $syx ]]; then
  # build da un clone (export per GitHub): senza firmware si controllano solo i tipi di file
  echo "check-clean: firmware assente ($syx): controllati solo i tipi di file"
  bad=0
  for t in "${targets[@]}"; do
    [[ -d $t ]] || continue
    extra=$(find "$t" \( -name '*.syx' -o -name '*.mcsz' -o -name '*.dtst' -o -name '*.dtsz' -o -name '*.snap' -o -name '*.png' \) ! -path '*/Contents/Resources/*_panel.png' | head -5)
    [[ -z $extra ]] || { echo "FALLITO: $t contiene: $extra"; bad=1; }
  done
  exit $bad
fi
cli=${MCCORE:-$HOME/.cache/mc-vst/core-build/mccore}
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
"$cli" extract "$syx" "$tmp" >/dev/null
bad=0
for t in "${targets[@]}"; do
  if [[ -d $t ]]; then
    extra=$(find "$t" \( -name '*.syx' -o -name '*.mcsz' -o -name '*.dtst' -o -name '*.dtsz' -o -name '*.snap' -o -name '*.png' \) ! -path '*/Contents/Resources/*_panel.png' | head -5)
    [[ -z $extra ]] || { echo "FALLITO: $t contiene: $extra"; bad=1; }
  fi
  python3 - "$t" "$tmp" "$syx" <<'PY' || bad=1
import hashlib, os, random, sys
target, sec, syx = sys.argv[1:4]
blobs = b''
if os.path.isdir(target):
    for d, _, fs in os.walk(target):
        for f in fs:
            blobs += open(os.path.join(d, f), 'rb').read()
else:
    blobs = open(target, 'rb').read()
random.seed(7)
needles = []
for f in sorted(os.listdir(sec)):
    if f.startswith('.'):
        continue
    d = open(os.path.join(sec, f), 'rb').read()
    for _ in range(200 if 'MAIN' in f else 40):
        if len(d) > 64:
            o = random.randrange(0, len(d) - 32)
            n = d[o:o + 32]
            if len(set(n)) > 4:               # niente blocchi di zeri o ripetizioni
                needles.append((f, n))
raw = open(syx, 'rb').read()
for _ in range(100):
    o = random.randrange(0, len(raw) - 32)
    n = raw[o:o + 32]
    if len(set(n)) > 4:
        needles.append(('syx', n))
hits = [f for f, n in needles if n in blobs]
mainsha = hashlib.sha256(open(os.path.join(sec, [x for x in os.listdir(sec) if 'MAIN' in x][0]), 'rb').read()).hexdigest().encode()
# l'impronta SHA-256 del MAIN OS e' nella tabella dei profili (serve a riconoscere la versione):
# e' un identificativo, non contenuto del firmware; si segnala soltanto
note = ' (con l\'impronta SHA-256 della versione nel profilo)' if mainsha in blobs else ''
if raw[:64] in blobs:
    hits.append('intestazione')
if hits:
    print('FALLITO: %s contiene %d blocchi del firmware (%s)' % (target, len(hits), sorted(set(hits))[:4]))
    sys.exit(1)
print('PULITO: %s (%d blocchi cercati, %.1f MB)%s' % (target, len(needles), len(blobs) / 1e6, note))
PY
done
exit $bad
