#!/usr/bin/env python3
"""Cerca nel bundle blocchi di byte che vengono dal firmware (sections/ del repo, mai committate).

    check-bytes.py SEZIONI_DIR CARTELLA_BUNDLE [LIBRERIA_PUBBLICA|@SORGENTE.c...]

Blocchi da 32 byte (4 finestre da 8 consecutive, la prima con almeno 6 valori diversi): quelli delle
sezioni del firmware, uno ogni 4 byte, contro tutti i blocchi dei file del bundle. Sequenze corte
comuni (prologhi di funzione ColdFire contro codice x86) non arrivano a 32 byte uguali. Esce con 1
se ce ne sono. I blocchi che stanno anche nelle librerie pubbliche indicate (libstdc++, zlib, Unicorn:
tabelle uguali al firmware per caso o perche' pubbliche) e quelli quasi solo ASCII (28 byte su 32) (li guarda il controllo
delle stringhe) non contano. Serve numpy.
"""
import glob
import os
import sys

import numpy as np


def windows(buf, step):
    """hash di blocchi da 32 byte a ogni `step` byte (finestre da 8 byte senza copie: strides=1)"""
    a = np.frombuffer(buf, dtype=np.uint8)
    n = len(a) - 31
    if n <= 0:
        return np.zeros(0, dtype=np.uint64), a, np.zeros(0, dtype=np.int64)
    w = np.ndarray(shape=(len(a) - 7,), dtype="<u8", buffer=buf, offset=0, strides=(1,))
    mul = [np.uint64(0x9E3779B97F4A7C15), np.uint64(0xC2B2AE3D27D4EB4F), np.uint64(0x165667B19E3779F9), np.uint64(0x27D4EB2F165667C5)]
    with np.errstate(over="ignore"):
        h = w[0:n] * mul[0] + w[8:n + 8] * mul[1] + w[16:n + 16] * mul[2] + w[24:n + 24] * mul[3]
    idx = np.arange(0, n, step)
    return h[::step], a, idx


def inset(values, sorted_set):
    """per ogni elemento di sorted_set (ordinato): e' in values? (cerca values dentro sorted_set)"""
    if len(values) == 0 or len(sorted_set) == 0:
        return np.zeros(len(sorted_set), dtype=bool)
    pos = np.searchsorted(sorted_set, values)
    pos[pos == len(sorted_set)] = len(sorted_set) - 1
    hit = np.zeros(len(sorted_set), dtype=bool)
    hit[pos[sorted_set[pos] == values]] = True
    return hit


def distinct_ok(a, idx):
    cols = np.stack([a[idx + k] for k in range(8)], axis=1)
    s = np.sort(cols, axis=1)
    return (np.diff(s, axis=1) != 0).sum(axis=1) + 1 >= 6


def c_arrays(path):
    """i vettori di byte {0x..,0x..} dei sorgenti C del core (firme dei punti da verificare: stanno gia'
    nel repo, nel motore dell'AU; il bundle li contiene per forza)"""
    import re
    txt = open(path, errors="replace").read()
    out = b""
    for m in re.finditer(r"\{((?:\s*0x[0-9a-fA-F]{2}\s*,?)+)\}", txt):
        out += bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", m.group(1))) + b"\x00" * 32
    return out


def main(sec_dir, bundle, libs):
    fw = []
    for p in sorted(glob.glob(os.path.join(sec_dir, "section_*.bin"))):
        d = open(p, "rb").read()
        if len(d) < 32:
            continue
        w, a, idx = windows(d, 4)
        ok = distinct_ok(a, idx)
        cs = np.concatenate([[0], np.cumsum((a >= 32) & (a < 127), dtype=np.int64)])
        ascii_only = (cs[idx + 32] - cs[idx]) >= 28
        fw.append(w[ok & ~ascii_only])
    if not fw:
        print("check-bytes: sezioni assenti, controllo saltato")
        return 0
    fwset = np.unique(np.concatenate(fw))
    for lib in libs:
        if lib.startswith("@") and os.path.isfile(lib[1:]):
            lw, _, _ = windows(c_arrays(lib[1:]), 1)
            fwset = fwset[~inset(lw, fwset)]
        elif os.path.isfile(lib) and os.path.getsize(lib) > 32:
            lw, _, _ = windows(open(lib, "rb").read(), 1)
            fwset = fwset[~inset(lw, fwset)]
    bad = 0
    for root, _, files in os.walk(bundle):
        for f in files:
            p = os.path.join(root, f)
            d = open(p, "rb").read()
            if len(d) < 32:
                continue
            w, a, idx = windows(d, 1)
            pos = np.searchsorted(fwset, w)
            pos[pos == len(fwset)] = len(fwset) - 1
            n = int((fwset[pos] == w).sum()) if len(fwset) else 0
            if n:
                print("check-bytes: %d blocchi da 32 byte del firmware in %s" % (n, p))
                bad += n
    if bad:
        return 1
    print("check-bytes: ok (nessun blocco da 32 byte delle sezioni del firmware nel bundle)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3:]))
