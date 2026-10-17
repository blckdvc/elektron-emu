/* DSP del Digitone II senza Python, per il plugin AU (docs/DSP-EMULATOR.md, "Il DSP senza
 * Python per il plugin").
 *
 * Il programma SHARC+ ricompilato (gli stessi sorgenti C che bridge.py compila, scritti da
 * tools/dsp_gensrc.py) e lo stato di partenza da uno .dnds (tools/dsp_export.py). Ogni
 * istanza ha la sua memoria, il suo stato e la sua tabella dei segmenti: piu' istanze
 * nello stesso processo, ognuna chiamata sempre dallo stesso thread.
 *
 *   dn_dsp *d = dn_dsp_new(buf, len, err, sizeof err);
 *   for each frame DSPI2 del ColdFire:
 *       dn_dsp_frame(d, frame, n);              // = DspSynth.on_frame (live=False)
 *       got = dn_dsp_pull(d, pcm, 4096);         // pcm[2*got] int32, slot TDM 0/1
 *   dn_dsp_reply(d, out, 2748);                 // risposta del DSP sull'SPI2
 *   dn_dsp_free(d);
 */
#ifndef DN_DSP_H
#define DN_DSP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Il DSP ha piu' motori intercambiabili (dn_dspback.c sceglie, il resto del motore chiama solo
 * dn_dsp_*): "interprete" (dn_interp.c, parte dal solo .syx a runtime e sta nel plugin),
 * "nativo" (core/dsp/dn_dsp.c con il C ricompilato dal firmware, libdndsp.a o la libreria
 * fatta sulla macchina dell'utente: solo uso personale, mai distribuita). Gli stati DNDS sono
 * gli stessi per entrambi (stesse St e Periph di rt.h/periph.h); `key` e' la chiave dei
 * sorgenti compilati ("" per l'interprete: accetta qualunque stato di quelle dimensioni). */
struct dn_dsp;
typedef struct dn_dsp_ops {
    const char *name;
    const char *key;
    void *(*create)(const uint8_t *dnds, size_t len, char *err, size_t errlen);
    void (*destroy)(void *d);
    int (*frame)(void *d, const uint8_t *frame, size_t n);
    int (*pull)(void *d, int32_t *out, int max);
    uint32_t (*reply)(void *d, uint8_t *out, uint32_t n);
    const char *(*error)(const void *d);
    int64_t (*cycles)(const void *d);
    const uint8_t *(*memory)(const void *d, int which, size_t *len);
    void (*analyze)(void *d);            /* solo l'interprete: analisi dei flag sul codice gia' decodificato */
} dn_dsp_ops;


void dn_dsp_analyze(struct dn_dsp *d);   /* l'analisi dei flag dell'interprete (no-op per gli altri) */
/* i motori (uno per file): il nativo e' in dn_dsp.c (libdndsp.a), l'interprete in dn_interp.c */
const dn_dsp_ops *dnn_dsp_get_ops(void);
const dn_dsp_ops *dni_dsp_get_ops(void);

/* Il boot del DSP dalla sezione 7 (stream di boot ADI) del firmware: `cycles` cicli del DSP con
   frame nulli (docs/DSP-INTERPRETE.md), poi lo stato DNDS (malloc'd, free) per dn_dsp_new. */
int dn_dsp_boot(const uint8_t *sec7, size_t len, int64_t cycles, volatile int *cancel,
                void (*progress)(void *ctx, int percent, const char *what), void *ctx,
                uint8_t **dnds, size_t *dnds_len, char *err, size_t errlen);

/* Il traduttore SHARC+ -> C (dn_gensrc.inc): dallo stato DNDS di un boot dell'interprete scrive i
   sorgenti C del DSP (gli stessi di tools/dsp_gensrc.py) chiamando `sink` per file (dsp_main.c,
   dsp_001.c ...); 0 = ok. Il risultato e' derivato dal firmware: solo nella cartella dell'utente. */
typedef int (*dn_gen_sink_fn)(void *ctx, const char *name, const char *data, size_t len);
int dn_dsp_gensrc(const uint8_t *dnds, size_t len, dn_gen_sink_fn sink, void *ctx, char *err, size_t errlen);

typedef struct dn_dsp dn_dsp;           /* il fronte: un motore + la sua istanza */
/* sceglie il motore per le istanze nuove: NULL = quello predefinito (nativo caricato, poi
   nativo statico, poi interprete) */
void dn_dsp_set_ops(const dn_dsp_ops *ops);
const dn_dsp_ops *dn_dsp_get_ops(void);

/* Carica uno stato DNDS v1 (il buffer si puo' liberare subito dopo). NULL e messaggio in
 * err se il file e' rotto o fatto con un altro codice (chiave o dimensioni diverse). */
dn_dsp *dn_dsp_new(const uint8_t *dnds, size_t len, char *err, size_t errlen);
void dn_dsp_free(dn_dsp *d);

/* Un frame DSPI2 come lo manda il ColdFire (big endian, fino a 2748 byte): il DSP gira
 * finche' non chiede il frame successivo. Ritorna 0 (vuole un altro frame), 1 (limite di
 * 2^40 cicli raggiunto), <0 errore: -4 errore del motore (dn_dsp_error), -3 PC non
 * compilato, -1 ritorno inatteso di rt_run. */
int dn_dsp_frame(dn_dsp *d, const uint8_t *frame, size_t n);

/* Fino a `max` campioni stereo int32 (out[2*i], out[2*i+1]); ritorna quanti. */
int dn_dsp_pull(dn_dsp *d, int32_t *out, int max);

/* L'ultima risposta del DSP (DMA26), byte come in memoria del DSP (parole a 16 bit
 * little-endian: il ColdFire le vuole scambiate, vedi DspSynth.reply). Ritorna il numero
 * di sequenza della risposta. */
uint32_t dn_dsp_reply(dn_dsp *d, uint8_t *out, uint32_t n);

/* Diagnostica */
const char *dn_dsp_error(const dn_dsp *d);          /* ultimo errore, "" se nessuno */
int64_t dn_dsp_cycles(const dn_dsp *d);             /* cicli del DSP emulati */
const char *dn_dsp_key(void);                       /* chiave dei sorgenti compilati ("" per l'interprete) */
/* memoria del DSP: 0 = L1, 1 = L2, 2 = DDR */
const uint8_t *dn_dsp_memory(const dn_dsp *d, int which, size_t *len);

#ifdef __cplusplus
}
#endif
#endif
