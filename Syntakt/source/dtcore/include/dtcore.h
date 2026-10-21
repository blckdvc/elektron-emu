/* dtcore: il motore dell'emulatore del Digitakt II in C, senza Python.
 *
 * Piano e milestone: docs/AU-PORT.md. Questo header espone SOLO cio' che esiste
 * (M1: firmware, stato, macchina di base); l'API completa del motore (audio per
 * traccia, MIDI, pannello) e' descritta nel piano e arriva con le milestone.
 *
 * Regole: nessuna parte del firmware (ne' suoi derivati: sezioni, stati) e'
 * dentro la libreria; tutto arriva dall'utente a runtime. Ogni funzione e'
 * equivalente byte per byte al Python indicato nel suo commento, e i test in
 * tests/test_dtcore.py lo verificano.
 */
#ifndef DTCORE_H
#define DTCORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && defined(DTCORE_SHARED_BUILD)
#define DT_API __declspec(dllexport)
#else
#define DT_API
#endif

#define DTCORE_VERSION "0.8.0-m8"

/* Codici di ritorno: 0 ok, <0 errore (il testo in `err`, se dato). */
enum {
    DT_OK = 0,
    DT_ERR_ARG = -1,
    DT_ERR_FORMAT = -2,      /* .syx, container o stato malformati */
    DT_ERR_NOMEM = -3,
    DT_ERR_IO = -4,
    DT_ERR_UNICORN = -5,
    DT_ERR_UNSUPPORTED = -6  /* es. Unicorn senza le patch richieste */
};

/* ---------------------------------------------------------------- firmware
 * .syx -> sezioni decompresse: dt2/container.py (8-in-7, ELE3) + dt2/elz.py +
 * emu/extract.py classify(). Stessi byte di `python -m emu.extract`. */

#define DT_MAX_SECTIONS 16

typedef struct {
    uint32_t id;             /* 2 bootstrap, 3 MAIN OS, 4 updater, 5 meta, 7, 8 */
    uint32_t dest;           /* indirizzo di caricamento (sez. 2: versione) */
    uint32_t offset, clen;   /* nel container ELE3 */
    int32_t packed;          /* 1 = era compressa (ELZ) */
    uint8_t *data;           /* contenuto decompresso */
    size_t len;
} dt_section;

typedef struct {
    int32_t count;
    dt_section sec[DT_MAX_SECTIONS];
    uint8_t syx_sha256[32];  /* del file .syx (emu/extract.py record_source) */
    uint8_t *container;      /* il container ELE3 (dt2/container.py container): la flash */
    size_t container_len;
} dt_firmware;

DT_API int dt_fw_extract(const uint8_t *syx, size_t len, dt_firmware *fw, char *err, size_t errlen);
DT_API void dt_fw_free(dt_firmware *fw);
DT_API const dt_section *dt_fw_section(const dt_firmware *fw, uint32_t id);
/* "MAIN_OS" ecc.: i nomi di emu/extract.py NAMES (SECTION per gli altri). */
DT_API const char *dt_fw_section_name(uint32_t id);

/* ---------------------------------------------------------------- macchina
 * La CPU ColdFire (Unicorn m68k cfv4e con le patch di patches/) e cio' che
 * emu/harness.py Machine aggiunge: pagine da 1 MB mappate a richiesta,
 * letture MMIO forzate (Machine.mmio), MOVEC (install_isa_patches_scoped; FF1,
 * BITREV e BYTEREV li esegue Unicorn), eccezioni via VBR e RTE
 * (install_exceptions, raise_vector senza srtrap), gli idle spin `bra *` col
 * yield del vettore 32 (longrun.build do_halt, senza canale UART TX). Nessun
 * modello di periferica: arrivano nelle milestone M2-M4. */

#define DT_MAIN_LOAD 0x40000400u
#define DT_VBR 0x40000000u

typedef struct dt_machine dt_machine;

typedef struct {
    int32_t idle_yield;      /* vettore 32 ogni N passaggi negli idle spin; 0 = niente hook */
    /* chiamata dopo l'hook MEM_INVALID e prima degli hook MOVEC (l'ordine di
       dspboot.run: il suo hook globale di copertura viene prima) */
    void (*pre_isa)(void *uc, void *ctx);
    void *pre_isa_ctx;
} dt_machine_config;

typedef struct {
    uint64_t movec, idle_spins, raised, rte, faults_mapped;
    int32_t halt_vec;        /* -1 = nessuno; altrimenti vettore senza gestore */
    int32_t movec_sites, idle_sites, mmio_hooks;
} dt_counters;

/* Ordine degli hook come tools/dtcore_ref.py (e longrun.build): MEM_INVALID,
   MOVEC, idle spin, INTR; poi dt_machine_load_state: stato, hook MMIO. */
DT_API dt_machine *dt_machine_new(const dt_firmware *fw, const dt_machine_config *cfg,
                                  char *err, size_t errlen);
DT_API void dt_machine_free(dt_machine *m);

/* Stato portabile (DTST v1, tools/snap2dtst.py dai .snap del Python): pagine,
   registri, EMAC, registri di controllo MOVEC, MMIO forzati, contatori. I
   componenti dei modelli (timer, eSDHC, ...) per ora si ignorano (M2). */
DT_API int dt_machine_load_state(dt_machine *m, const char *path, char *err, size_t errlen);

/* Esegue esattamente `count` istruzioni (uc_emu_start con count=, come
   uc.emu_start(pc, 0, count=N) del Python). -> 0 o l'errore di Unicorn
   (negativo: -1000 - uc_err). */
DT_API int dt_machine_run_count(dt_machine *m, uint64_t count);

/* d0..d7, a0..a7, pc, sr (l'ordine di emu/snapshot.py REGS) */
DT_API void dt_machine_regs(dt_machine *m, uint32_t out[18]);
DT_API void dt_machine_counters(dt_machine *m, dt_counters *out);
/* SHA-256 del contenuto di tutte le regioni mappate, in ordine di indirizzo
   (lo stesso calcolo di tools/dtcore_ref.py). */
DT_API int dt_machine_digest(dt_machine *m, uint8_t out[32], uint32_t *nregions, uint64_t *nbytes);
DT_API int dt_machine_mem_read(dt_machine *m, uint32_t addr, void *buf, size_t n);

/* ---------------------------------------------------------------- motore
 * Il worker della GUI v2 con --audio (emu/gui.py Emulator): CPU, timer, SSI0,
 * DSPI1/2, MIDI DIN, pannello, LED, eSDHC (+Drive), audio HLE. Lo stato e' un
 * DTST v2 (tools/snap2dtst.py da una sessione della GUI). Un motore si usa da
 * un solo thread; le funzioni di ingresso valgono fra un giro e l'altro. */
typedef struct dt_engine dt_engine;

typedef struct {
    int32_t panel_dwell;     /* PANEL_DWELL_CHUNKS: 16 */
    int64_t post_intro_ips;  /* 0 = AUDIO_IPS (150M istruzioni per secondo emulato) */
} dt_engine_config;

enum { DT_PANEL_PRESS = 0, DT_PANEL_RELEASE = 1, DT_PANEL_RELEASE_ALL = 2, DT_PANEL_ENCODER = 3 };

DT_API dt_engine *dt_engine_new(const dt_firmware *fw, const char *state_path,
                                const dt_engine_config *cfg, char *err, size_t errlen);
DT_API dt_engine *dt_engine_new_mem(const dt_firmware *fw, const uint8_t *state, size_t len,
                                    const dt_engine_config *cfg, char *err, size_t errlen);
DT_API void dt_engine_free(dt_engine *e);
/* un giro: ips in attesa, ingresso del pannello, spin(400000), schermo, LED. -> 0, 1 fermato */
DT_API int dt_engine_chunk(dt_engine *e);
DT_API int dt_engine_midi_in(dt_engine *e, const uint8_t *data, size_t n);
DT_API int dt_engine_panel(dt_engine *e, int kind, int code, int arg);
DT_API int dt_engine_rpc(dt_engine *e, const uint8_t *payload, size_t n);
DT_API size_t dt_engine_rpc_replies(dt_engine *e, uint8_t *buf, size_t max);
DT_API size_t dt_engine_midi_out(dt_engine *e, uint8_t *buf, size_t max);
/* ogni 512 campioni a 48 kHz: il mix (quello di emu/audio.py) e, se `tracks`,
   16 x 512 campioni per traccia (le stesse voci, MASTER senza limitatore) */
DT_API void dt_engine_set_audio_sink(dt_engine *e,
                                     void (*sink)(void *ctx, const float *mix, const float *tracks,
                                                  size_t n),
                                     void *ctx, int tracks);
DT_API void dt_engine_set_master(dt_engine *e, double master);   /* Main Volume: 0..0.5, 0.25 */
DT_API int64_t dt_engine_audio_samples(dt_engine *e);
DT_API uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64]);
DT_API uint64_t dt_engine_leds(dt_engine *e, uint8_t out[50]);
DT_API int64_t dt_engine_instrs(dt_engine *e);
DT_API const char *dt_engine_status(dt_engine *e);
DT_API int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n);
DT_API dt_machine *dt_engine_machine(dt_engine *e);
/* lo stato (DTST v2, come tools/snap2dtst.py; compress: "DTSZ" + zlib), fra due giri */
DT_API int dt_engine_save_mem(dt_engine *e, int compress, uint8_t **out, size_t *len);
DT_API int dt_engine_save(dt_engine *e, const char *path, int compress);

/* ------------------------------------------------------------ tempo reale
 * dt_rt: il motore con il suo thread, per un plugin (rt.c, docs/AU-PORT.md
 * 4.3). dt_rt_render e dt_rt_midi dal thread audio (niente lock ne'
 * allocazioni; `offline` != 0 aspetta il motore), dt_rt_panel,
 * dt_rt_set_master e dt_rt_screen dal thread della GUI. Uscite: 17 canali
 * mono (0 = il mix di emu/audio.py, 1..16 le tracce). */
typedef struct dt_rt dt_rt;
DT_API dt_rt *dt_rt_new(const uint8_t *syx, size_t syx_len, const char *state_path, double rate,
                        uint32_t max_frames, char *err, size_t errlen);
DT_API dt_rt *dt_rt_new_mem(const uint8_t *syx, size_t syx_len, const uint8_t *state, size_t state_len,
                            double rate, uint32_t max_frames, char *err, size_t errlen);
DT_API void dt_rt_free(dt_rt *rt);
/* lo stato (DTSZ) dal thread del motore; dal thread della GUI o dell'host, mai da quello audio */
DT_API int dt_rt_save(dt_rt *rt, uint8_t **out, size_t *len);
/* +Drive via MIDI RPC (come Elektron Transfer): richiesta e risposta, dal thread della GUI */
DT_API int dt_rt_rpc(dt_rt *rt, uint8_t rid, const uint8_t *body, size_t n, uint8_t *reply_id, uint8_t **reply,
                     size_t *reply_len, int timeout_ms);
DT_API uint32_t dt_rt_latency(dt_rt *rt);                 /* campioni dell'host */
DT_API uint32_t dt_rt_latency_for(double rate, uint32_t max_frames);
DT_API int dt_rt_midi(dt_rt *rt, uint32_t offset, const uint8_t *data, size_t n);
DT_API int dt_rt_render(dt_rt *rt, uint32_t n, float *const *outs, int nouts, int offline);
DT_API int dt_rt_panel(dt_rt *rt, int kind, int code, int arg);
DT_API void dt_rt_set_master(dt_rt *rt, double master);
DT_API uint64_t dt_rt_screen(dt_rt *rt, uint8_t out[128 * 64], uint8_t leds[50]);

/* lettura dei pattern per la vista Elektron Emu (SeqMem.hpp del plugin): ogni 8 giri il
   thread del motore chiama `fn` sulla macchina (fra due giri: la memoria e' ferma) e
   pubblica i byte che scrive (al piu' DT_SEQ_MAX); la GUI li legge con dt_rt_seq. */
#define DT_SEQ_MAX 256
typedef size_t (*dt_seq_reader)(dt_machine *m, uint8_t *out, size_t cap);
DT_API void dt_rt_set_seq_reader(dt_rt *rt, dt_seq_reader fn);
/* i byte dell'ultima lettura (-> quanti, 0 se nessuna); `version` (o NULL) cambia con loro */
DT_API size_t dt_rt_seq(dt_rt *rt, uint8_t *out, size_t cap, uint64_t *version);
DT_API size_t dt_rt_midi_out(dt_rt *rt, uint8_t *buf, size_t max, uint32_t *offset, uint32_t frames);
DT_API int64_t dt_rt_underruns(dt_rt *rt);
DT_API const char *dt_rt_status(dt_rt *rt);
DT_API dt_engine *dt_rt_engine(dt_rt *rt);
DT_API int64_t dt_rt_max_lag(dt_rt *rt);
DT_API void dt_rt_offsets(dt_rt *rt, int64_t *E, int64_t *X);   /* in campioni a 48 kHz */

/* il boot freddo dal reset (emu/dspboot.py run): lo stato a `point` istruzioni,
   come i pioli della scala (emu/checkpoint.py make, snapshots/<fw>/boot*M.snap) */
DT_API int dt_coldboot(const dt_firmware *fw, uint64_t point, int compress, uint8_t **out, size_t *len,
                       char *err, size_t errlen);
/* la prima sessione senza Python: il gradino `rung` della scala, poi il motore
   della GUI (gui.py --audio, come tools/dtcore_guiref.py RUNG --instrs N --save)
   per `instrs` istruzioni senza tempo reale: l'intro, il passaggio al sistema e
   il cambio di cadenza sono fatti. `cancel` (o NULL) a 1 interrompe. */
/* flags di dt_coldboot_ex / dt_coldboot_session_ex */
#define DT_COLDBOOT_FACTORY_INDEX 1     /* indice di fabbrica vuoto sull'eMMC: il +Drive si monta (i164) */
DT_API int dt_coldboot_ex(const dt_firmware *fw, uint64_t point, int compress, int flags, uint8_t **out,
                          size_t *len, char *err, size_t errlen);
DT_API int dt_coldboot_session_ex(const dt_firmware *fw, uint64_t rung, int64_t instrs, int compress, int flags,
                                  uint8_t **out, size_t *len, volatile int *cancel, char *err, size_t errlen);
#define DT_COLDBOOT_RUNG 60000000ull
#define DT_COLDBOOT_GUI 1000000000ll
DT_API int dt_coldboot_session(const dt_firmware *fw, uint64_t rung, int64_t instrs, int compress,
                               uint8_t **out, size_t *len, volatile int *cancel, char *err, size_t errlen);

/* +Drive dal computer (emu/transfer.py Client, tools/wav2dt.py): drive.c */
typedef struct {
    char name[64];
    char type;               /* 'D' cartella, 'F' file */
    uint8_t protected_;
    uint32_t size, hash;
} dt_drive_entry;
DT_API int dt_drive_ls(dt_rt *rt, const char *path, dt_drive_entry **out, size_t *count, char *err, size_t errlen);
DT_API int dt_drive_mkdir(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dt_drive_rmdir(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dt_drive_rm(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dt_drive_put(dt_rt *rt, const char *path, const uint8_t *data, size_t len,
                        void (*progress)(void *ctx, size_t off, size_t total), void *ctx,
                        char *err, size_t errlen);
/* WAV -> il container dei sample del firmware (mono/stereo, ricampionato a 48 kHz) */
DT_API int dt_wav2dt(const uint8_t *wav, size_t len, uint8_t **out, size_t *out_len, char *err, size_t errlen);

DT_API const char *dt_version(void);
/* 1 se il processo puo' allocare memoria eseguibile per il JIT di Unicorn
   (macOS: entitlement com.apple.security.cs.allow-jit col runtime protetto).
   dt_machine_new la controlla prima di uc_open, che altrimenti fa exit(1). */
DT_API int dt_jit_available(void);

#ifdef __cplusplus
}
#endif
#endif
