#!/usr/bin/env bash
# Esporta il motore comune del Digitakt II (digitakt/emulator/core) a un commit
# FISSO in WORK/dtcore-<commit>/core e ne stampa il percorso. Il VST3 della
# Syntakt si costruisce su quello, non sulla cartella viva del Digitakt: ci
# lavora un altro agente e una modifica a meta' romperebbe questa build.
# Per passare a un commit piu' nuovo (dopo aver provato `stcore rt-test`):
# DTCORE_COMMIT=<sha> o cambia il valore qui sotto.
#
#   vst/scripts/dtcore.sh WORK
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work=${1:?uso: dtcore.sh WORK}
# export pubblico: il motore comune e' gia' copiato in dtcore/ (stesso commit)
if [[ -z ${DTCORE_COMMIT:-} && -f $root/dtcore/src/machine.c ]]; then echo "$root/dtcore"; exit 0; fi
commit=${DTCORE_COMMIT:-547ee0c2175039b1040cf791cad4444699e26ae6}
repo=${DTCORE_REPO:-$root/../../digitakt/emulator}
dest=$work/dtcore-$commit
if [[ ! -f $dest/core/src/machine.c ]]; then
  mkdir -p "$dest"
  git -C "$repo" archive "$commit" core | tar -x -C "$dest"
fi
echo "$dest/core"
