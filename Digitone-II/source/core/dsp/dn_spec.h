/* Gestori specializzati dell'esecutore senza SIMD (dn_exec.inc): per le classi di calcolo piu'
 * frequenti del DSP (unita' e codice operativo del campo COMPUTE) il gestore e' istanziato con
 * la classe come costante, cosi' le funzioni di rt.h si riducono al solo caso (niente switch su
 * alu/mul/shift); i registri e gli immediati restano nella Ins. Le liste sono una scelta di
 * progetto (le classi di calcolo che un programma SHARC+ usa di piu'), non dati del firmware: una
 * classe non elencata passa dal gestore generico, con lo stesso risultato.
 *
 * Plain: comp = (K << 12) | registri(12 bit), K = unita'(2) | codice(8). Multi: funzioni multiple
 * (bit 22 del COMPUTE), comp = (K << 16) | registri(16 bit), K = 0x40 | op6. */
#ifndef DN_SPEC_H
#define DN_SPEC_H

#define SPEC_KP(X) \
    X(0x130) X(0x081) X(0x082) X(0x001) X(0x029) X(0x0cd) X(0x0a2) X(0x021) X(0x0da) X(0x0b0) \
    X(0x0d9) X(0x040) X(0x00a) X(0x002) X(0x0c9) X(0x0ca) X(0x0e3) X(0x0a1) X(0x0e2) X(0x08a) \
    X(0x0e1) X(0x00b) X(0x022) X(0x0c4) X(0x02a) X(0x0ad) X(0x0bd) X(0x0c1) X(0x061) X(0x1bc) \
    X(0x288) X(0x109) X(0x17c) X(0x1fc) X(0x200) X(0x204) X(0x041) X(0x042) X(0x043) X(0x062)
#define SPEC_KM(X) X(0x58) X(0x59) X(0x40) X(0x60) X(0x70) X(0x78) X(0x48) X(0x50)

/* i quattro modi della memoria (post-modifica u, scrittura d) per OP_3 e OP_4 */
#define SPEC_UD(X) X(0,0) X(0,1) X(1,0) X(1,1)

#endif
