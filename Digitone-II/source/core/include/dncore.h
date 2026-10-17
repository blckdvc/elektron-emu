/* dncore: il motore del Digitone II per il plugin AU (docs/AU-PORT.md).
 *
 * Implementa l'API dt_engine_* di dtcore.h (digitakt/emulator/core/include), cosi'
 * il motore in tempo reale rt.c del Digitakt II (dt_rt_*) gira sul Digitone senza
 * cambiare. Qui le aggiunte del Digitone: lo stato del DSP e i suoi LED. */
#ifndef DNCORE_H
#define DNCORE_H

#include "dtcore.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DNCORE_VERSION "0.1.0"
#define DN_LEDS 53                  /* emu/panel_leds.py N_LEDS */

/* Lo stato di partenza del DSP (DNDS, tools/dsp_export.py) per i motori creati dopo;
   copiato. Senza, dt_engine_new legge il file di DNCORE_DSP_STATE. */
DT_API int dn_set_dsp_state(const uint8_t *dnds, size_t len);

/* Lo stato DNDS del DSP installato da dn_set_dsp_state / dn_prepare_dsp (non liberare; NULL se non c'e'). */
DT_API const uint8_t *dn_dsp_state_get(size_t *len);

/* Il DSP nativo costruito sul computer dell'utente (vst/NativeBuild.cpp): l'impronta SHA-256
   (esadecimale) della sezione DSP del firmware, che lega la libreria al suo firmware. */
DT_API int dn_dsp_program_hash(const uint8_t *syx, size_t len, char hex[65], char *err, size_t errlen);

/* Il traduttore SHARC+ -> C (core/dsp/dn_gensrc.inc): dallo stato DNDS di un boot dell'interprete
   scrive i sorgenti C del DSP (dsp_main.c, dsp_001.c ...), un file per chiamata di `sink`
   (0 = continua). Il risultato e' derivato dal firmware: resta nella cartella dell'utente. */
typedef int (*dn_gensrc_sink_fn)(void *ctx, const char *name, const char *data, size_t len);
DT_API int dn_dsp_gensrc(const uint8_t *dnds, size_t len, dn_gensrc_sink_fn sink, void *ctx, char *err, size_t errlen);

/* SHA-256 a pezzi (per verificare i download): init, update, final -> 32 byte */
typedef struct { uint64_t opaque[40]; } dn_sha256_ctx;
DT_API void dn_sha256_begin(dn_sha256_ctx *c);
DT_API void dn_sha256_add(dn_sha256_ctx *c, const void *data, size_t n);
DT_API void dn_sha256_end(dn_sha256_ctx *c, uint8_t out[32]);

/* tutti e 53 i LED (dt_engine_leds ne da' 50, quanti ne aspetta rt.c) */
DT_API uint64_t dn_engine_leds(dt_engine *e, uint8_t out[DN_LEDS]);
/* SHA-256 della memoria come la legge l'host del Python (la riga 'ram' della traccia) */
DT_API int dn_engine_digest(dt_engine *e, uint8_t out[32], uint32_t *nregions);
DT_API int64_t dn_engine_dsp_frames(dt_engine *e);

/* ---- per il plugin (sopra dt_rt di rt.c) ---- */
/* tutti e 53 i LED, da qualunque thread; -> la versione */
DT_API uint64_t dn_rt_leds(dt_rt *rt, uint8_t out[DN_LEDS]);
/* byte verso il MIDI IN emulato da un thread qualunque tranne quello audio (l'invio
   di SysEx del pannello +DRIVE): passano dalla coda RPC di rt.c con l'id DN_RPC_MIDI */
#define DN_RPC_MIDI 0xFF
DT_API int dn_rt_midi_inject(dt_rt *rt, const uint8_t *data, size_t n);

/* Il +Drive del Digitone II con l'API Data del firmware (emu/transfer.py DataClient,
   emu/drivepanel.py DataDrivePanel): preset, kit, progetti, soundbank. */
typedef struct {
    char name[64];
    uint8_t children;       /* ha figli (una cartella) */
    uint8_t type;           /* 1 cartella (count), 2 voce (index, size, ops, valid, meta) */
    uint8_t valid, meta;
    uint32_t count, index, size;
    uint16_t ops;
} dn_data_entry;
DT_API int dn_data_ls(dt_rt *rt, const char *path, dn_data_entry **out, size_t *count, char *err, size_t errlen);
/* DataReadOpen / DataReadPartial / DataReadClose: i byte dell'oggetto (free) */
DT_API int dn_data_read(dt_rt *rt, const char *path, uint8_t **out, size_t *len, char *err, size_t errlen);
DT_API int dn_data_clear(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dn_data_rename(dt_rt *rt, const char *path, const char *name, char *err, size_t errlen);


/* ---- per il plugin (vst/): dal solo .syx dell'utente allo stato iniziale ----
 * Niente del firmware (ne' di cio' che ne deriva) sta nel plugin: tutto si ricava a runtime
 * dal .syx che l'utente mette nella cartella del firmware, e va in una cartella di cache
 * dell'utente (`cache_dir`, creata se manca). Tutte le funzioni lunghe controllano `cancel`
 * (o NULL), possono chiamare `progress` (o NULL) da un thread qualunque, ritornano 0 o un
 * codice negativo di dtcore.h con il testo in `err`. */
typedef void (*dn_progress_fn)(void *ctx, int percent, const char *what);

typedef struct {
    int known;               /* 1 = MAIN OS con profilo di indirizzi (provato), 0 = versione sconosciuta */
    char machine[24];        /* "Digitone II" se il container lo dice, altrimenti "" */
    char version[32];        /* "1.11" se leggibile, altrimenti "" */
    char sha256[65];         /* del file .syx (esadecimale) */
    char main_sha256[65];    /* della sezione MAIN OS */
} dn_fw_info;
/* Controllo di formato e versione. 0 = e' un .syx valido (known dice se la versione e' quella
   provata: se 0 il plugin prova comunque, avvisando); <0 = non e' un firmware leggibile. */
DT_API int dn_fw_check(const uint8_t *syx, size_t len, dn_fw_info *info, char *err, size_t errlen);

/* Lo stato iniziale del DSP SHARC+ (DNDS) dal firmware: dalla cache, o fatto adesso (il boot
   del DSP, decine di secondi la prima volta) e messo in cache; poi installato (come
   dn_set_dsp_state) per i motori creati dopo. Va chiamata prima di dt_rt_new*. */
DT_API int dn_prepare_dsp(const uint8_t *syx, size_t len, const char *cache_dir, volatile int *cancel,
                          dn_progress_fn progress, void *ctx, char *err, size_t errlen);
/* La sessione iniziale della ColdFire (DTSZ) dal firmware: dalla cache, o dal boot freddo
   (~decine di secondi) e messa in cache. `*out` malloc'd (free). */
DT_API int dn_make_session(const uint8_t *syx, size_t len, const char *cache_dir, volatile int *cancel,
                           dn_progress_fn progress, void *ctx, uint8_t **out, size_t *out_len,
                           char *err, size_t errlen);
/* Il boot freddo senza cache (dn_make_session lo usa): vedi docs/COLDBOOT.md.
   La sessione iniziale (DTSZ compresso se compress != 0) dal solo firmware: il boot freddo dal
   reset (la scala di emu/dspboot.py, di seguito fino a 120M), poi il motore come tools/guirun.py
   per `instrs` istruzioni (0 = DN_COLDBOOT_INSTRS; Main OS avviato, +Drive montato) e, se lo
   stato del DSP e' impostato (dn_set_dsp_state), un secondo emulato con l'audio perche' il
   collegamento col DSP sia gia' partito. `*out` malloc'd (free); `cancel` (o NULL) a 1
   interrompe; `progress` (o NULL) e' chiamata ogni tanto (0..100). */
#define DN_COLDBOOT_INSTRS 600000000ll      /* dopo 120M: tutti i lavori del firmware finiti (il Python: 790M) */
DT_API int dn_coldboot_session(const dt_firmware *fw, int64_t instrs, int compress, uint8_t **out, size_t *len,
                               volatile int *cancel, void (*progress)(void *ctx, int percent), void *ctx,
                               char *err, size_t errlen);

/* Un gradino della scala di boot di emu/dspboot.py (boot60M.snap ... boot400M.snap): lo stato (DTST)
   a `point` istruzioni dal reset, solo il ColdFire (senza DSP ne' motore). `flags`:
   DT_COLDBOOT_FACTORY_INDEX = l'indice di fabbrica vuoto sull'eMMC (il +Drive si monta). */
#ifndef DT_COLDBOOT_FACTORY_INDEX
#define DT_COLDBOOT_FACTORY_INDEX 1
#endif
DT_API int dn_coldboot_rung(const dt_firmware *fw, uint64_t point, int compress, int flags, uint8_t **out,
                            size_t *len, char *err, size_t errlen);

/* Il motore del DSP in uso: "interprete" (nel plugin, parte dal solo .syx), "nativo" (la
   libreria ricompilata dal firmware, solo uso personale: dn_dsp_native_load) o "statico" (build
   di sviluppo con libdndsp.a). */
DT_API const char *dn_dsp_backend_name(void);
/* Carica la libreria del DSP ricompilato fatta sulla macchina dell'utente (vst/scripts/dsp-nativo.sh);
   0 = in uso da ora per i motori nuovi, <0 = non utilizzabile (il plugin resta sull'interprete). */
DT_API int dn_dsp_native_load(const char *path, char *err, size_t errlen);
DT_API void dn_dsp_native_unload(void);

#ifdef __cplusplus
}
#endif
#endif
