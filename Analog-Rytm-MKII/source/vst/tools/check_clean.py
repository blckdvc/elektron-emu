#!/usr/bin/env python3
"""Controlla che un bundle (o un'intera cartella) NON contenga firmware Elektron:
nessun blocco da 64 byte non nullo di una sezione del firmware (MAIN OS, pannello,
FPGA, bootstrap) e del .syx grezzo compare in nessun file. Le sezioni si ricavano con
`arcore extract` (core/cli); le stringhe lunghe del firmware (>= 16 caratteri) si
cercano in tutti i file. Esce con 1 se trova qualcosa.

    vst/tools/check_clean.py FIRMWARE.syx ARCORE_BINARIO PERCORSO [PERCORSO...]
"""
import glob
import os
import re
import subprocess
import sys
import tempfile

# Il firmware (C++ per ColdFire) contiene anche la libreria standard (libstdc++): i suoi
# testi -- nomi di eccezioni, "basic_string::at", ... -- stanno anche in ogni plugin C++
# e non sono contenuto di Elektron. Non contano ne' le stringhe di questo elenco ne' i
# blocchi fatti (quasi) solo di testo stampabile: contano il codice e i dati.
GENERIC = re.compile(rb'^(St\d|N\d+__|NSt|\*N|basic_string|std::|typeinfo|TLS |construction|covariant|global |'
                     rb'guard variable|hidden alias|non-(virtual|transaction)|reference temporary|reinterpret_cast|'
                     rb'terminate called|unsigned |vector::|decltype|\(anonymous|0123456789|ABCDEFGHIJ| !"#|'
                     rb'Invalid argument|Analog Rytm MKII|bad_function_call|\s+$)')


def texty(b):
    return sum(1 for c in b if 32 <= c < 127 or c == 0) >= 60


def main(syx, arcore, paths):
    tmp = tempfile.mkdtemp()
    subprocess.run([arcore, 'extract', syx, tmp], check=True, stdout=subprocess.DEVNULL)
    secs = [open(f, 'rb').read() for f in sorted(glob.glob(os.path.join(tmp, '*.bin')))]
    secs.append(open(syx, 'rb').read())
    blocks = set()
    strs = set()
    for d in secs:
        for i in range(0, len(d) - 64, 64):
            b = d[i:i + 64]
            if b.count(0) <= 16 and not texty(b):
                blocks.add(b)
        for m in re.finditer(rb'[ -~]{16,}', d):
            if not GENERIC.match(m.group()):
                strs.add(m.group())
    files = []
    for p in paths:
        if os.path.isdir(p):
            for r, _, fs in os.walk(p):
                files += [os.path.join(r, f) for f in fs]
        else:
            files.append(p)
    bad = 0
    for f in files:
        try:
            data = open(f, 'rb').read()
        except OSError:
            continue
        nb = 0
        # i blocchi del firmware possono stare a qualunque offset: cerco ogni blocco come sottostringa
        # (insieme grande: confronto sulle finestre allineate a 64 e a 1 byte con un indice sui primi 8 byte)
        idx = {}
        for b in blocks:
            idx.setdefault(b[:8], []).append(b)
        for i in range(0, len(data) - 63):
            c = idx.get(data[i:i + 8])
            if c:
                w = data[i:i + 64]
                if w in c:
                    nb += 1
        ns = sum(1 for s in strs if s in data)
        print('%-70s %8d byte  blocchi del firmware: %d  stringhe del firmware: %d'
              % (os.path.relpath(f), len(data), nb, ns))
        bad += nb + ns
    print('PULITO' if not bad else 'TROVATO MATERIALE DEL FIRMWARE')
    return 1 if bad else 0


if __name__ == '__main__':
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3:]))
