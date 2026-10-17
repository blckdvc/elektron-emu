#!/usr/bin/env python3
"""Controlla che un bundle/zip del plugin non contenga il firmware ne' dati derivati.

    vst/scripts/check-bundle.py BUNDLE_O_ZIP [FIRMWARE.syx [SEZIONE.bin ...]]

Per ogni file del bundle: nessuna estensione da firmware o stato (.syx .stst .dtst
.stsz .dtsz .snap .bin), nessun file che contenga pezzi del .syx o delle sue sezioni
decompresse (finestre da 32 byte prese a intervalli regolari), nessun file con
l'impronta del .syx. Le SEZIONE.bin sono le sezioni decompresse del firmware
(`python -m emu.extract`, cartella sections/ del repo): codice ricompilato o RAM del
firmware ne conterrebbero dei pezzi. Uscita 0 = pulito, 1 = trovato qualcosa.
"""
import hashlib
import os
import sys
import zipfile

BAD_EXT = {'.syx', '.stst', '.dtst', '.stsz', '.dtsz', '.snap', '.bin', '.sec'}
WINDOW = 32
STEP = 4099


def windows(blob, n=400):
    """~n finestre da WINDOW byte, con almeno qualche entropia (non tutte zeri)."""
    out = []
    if len(blob) < WINDOW:
        return out
    step = max(STEP, len(blob) // n)
    for off in range(0, len(blob) - WINDOW, step):
        w = blob[off:off + WINDOW]
        if len(set(w)) > 8:
            out.append(w)
    return out


def main(argv):
    target = argv[1]
    syx = argv[2] if len(argv) > 2 and os.path.exists(argv[2]) else None
    files = {}
    if os.path.isdir(target):
        for d, _, fs in os.walk(target):
            for f in fs:
                p = os.path.join(d, f)
                files[os.path.relpath(p, target)] = open(p, 'rb').read()
    else:
        with zipfile.ZipFile(target) as z:
            for n in z.namelist():
                if not n.endswith('/'):
                    files[n] = z.read(n)
    needles = []
    syx_sha = None
    if syx:
        raw = open(syx, 'rb').read()
        syx_sha = hashlib.sha256(raw).digest()
        needles += [('syx', w) for w in windows(raw)]
        for p in argv[3:]:
            needles += [('sezione', w) for w in windows(open(p, 'rb').read())]
    bad = 0
    for name, data in sorted(files.items()):
        ext = os.path.splitext(name)[1].lower()
        if ext in BAD_EXT:
            print('TROVATO file sospetto:', name)
            bad += 1
        if syx_sha and hashlib.sha256(data).digest() == syx_sha:
            print('TROVATO il .syx intero:', name)
            bad += 1
        hits = sum(1 for _, w in needles if w in data)
        if hits:
            print('TROVATO %d pezzi di firmware in %s' % (hits, name))
            bad += 1
    print('check-bundle: %d file, %d finestre cercate (%s) -> %s'
          % (len(files), len(needles), 'con firmware' if syx else 'senza firmware: solo nomi',
             'PULITO' if not bad else 'NON PULITO'))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
