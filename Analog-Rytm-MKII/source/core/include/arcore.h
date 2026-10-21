/* arcore: il motore dell'emulatore dell'Analog Rytm MKII in C, senza Python.
 * Stessa API (prefisso dt_) di dtcore (Digitakt II, digitakt/emulator/core) e
 * di mccore (Model:Cycles), da cui deriva: docs/AU-PORT.md.
 *
 * Regole: nessuna parte del firmware (ne' suoi derivati: sezioni, stati,
 * calibrazione) e' dentro la libreria; tutto arriva dall'utente a runtime.
 * Ogni funzione e' equivalente byte per byte al Python indicato nel suo
 * commento, e i test in tests/test_arcore.py lo verificano.
 */
#ifndef ARCORE_H
#define ARCORE_H

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

#define ARCORE_VERSION "0.1.0"
#define DTCORE_VERSION ARCORE_VERSION

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
    uint32_t id;             /* 1 FPGA, 2 bootstrap, 3 MAIN OS, 4 updater, 8 pannello */
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
DT_API const char *dt_fw_section_name(uint32_t id);

/* ---------------------------------------------------------------- macchina
 * La CPU ColdFire (Unicorn m68k cfv4e con le patch di patches/) e cio' che
 * emu/harness.py Machine aggiunge: pagine da 1 MB mappate a richiesta,
 * letture MMIO forzate, MOVEC, FF1/BITREV/BYTEREV (gli hook per indirizzo di
 * install_isa_patches_scoped: questa Unicorn non li esegue come il Rytm li
 * vuole), eccezioni via VBR e RTE (install_exceptions, raise_vector senza
 * srtrap). */

#define DT_MAIN_LOAD 0x40000400u
#define DT_VBR 0x40000000u

typedef struct dt_machine dt_machine;

typedef struct {
    int32_t idle_yield;      /* vettore 32 ogni N passaggi negli idle spin; 0 = niente hook */
    /* != 0: gli hook ISA/MOVEC non si registrano in Unicorn ma restano in
       m->sites (il boot freddo li smista da un solo hook globale, come la
       `table` di dspboot.run: con migliaia di hook per indirizzo e uno globale
       ogni istruzione li scorre tutti) */
    int32_t defer_sites;
} dt_machine_config;

typedef struct {
    uint64_t movec, idle_spins, raised, rte, faults_mapped;
    int32_t halt_vec;        /* -1 = nessuno; altrimenti vettore senza gestore */
    int32_t movec_sites, idle_sites, mmio_hooks;
} dt_counters;

DT_API dt_machine *dt_machine_new(const dt_firmware *fw, const dt_machine_config *cfg,
                                  char *err, size_t errlen);
DT_API void dt_machine_free(dt_machine *m);
DT_API int dt_machine_load_state(dt_machine *m, const char *path, char *err, size_t errlen);
DT_API int dt_machine_run_count(dt_machine *m, uint64_t count);
/* d0..d7, a0..a7, pc, sr (l'ordine di emu/snapshot.py REGS) */
DT_API void dt_machine_regs(dt_machine *m, uint32_t out[18]);
DT_API void dt_machine_counters(dt_machine *m, dt_counters *out);
DT_API int dt_machine_digest(dt_machine *m, uint8_t out[32], uint32_t *nregions, uint64_t *nbytes);
DT_API int dt_machine_mem_read(dt_machine *m, uint32_t addr, void *buf, size_t n);

/* ---------------------------------------------------------------- motore
 * Il thread del motore della GUI v2 del Rytm con --audio (emu/gui.py Emulator
 * con configure_rytm_audio): CPU, timer, SSI1 (lo scambio con l'FPGA a 1500
 * Hz), le copie eDMA del software, interrupt forzati, MIDI DIN, pannello su
 * UART8 (tasti, encoder, pad), LED, eSDHC (+Drive), la flash SPI (calibrazione)
 * e il motore delle voci (emu/rytmsynth.py + emu/rytmdsp.py: le nostre voci,
 * suonate dal firmware). Lo stato e' un DTST v2 (tools/snap2dtst.py da una
 * sessione salvata da tools/arcore_ref.py --save). Un motore si usa da un solo
 * thread; le funzioni di ingresso valgono fra un giro e l'altro. */
typedef struct dt_engine dt_engine;

typedef struct {
    int32_t panel_dwell;     /* gui.PANEL_DWELL_CHUNKS (4); 0 = predefinito */
} dt_engine_config;

/* Emulator.inbox: press/release/encoder (codici di devices/analog-rytm-mkii.toml),
   release_all; i pad con DT_PANEL_PAD (code = pad 0-11, arg = velocity 1-127;
   arg 0 = rilascio): rytm_pads.press/release, byte grezzi sul filo */
enum { DT_PANEL_PRESS = 0, DT_PANEL_RELEASE = 1, DT_PANEL_RELEASE_ALL = 2, DT_PANEL_ENCODER = 3,
       DT_PANEL_PAD = 4 };

DT_API dt_engine *dt_engine_new(const dt_firmware *fw, const char *state_path,
                                const dt_engine_config *cfg, char *err, size_t errlen);
DT_API dt_engine *dt_engine_new_mem(const dt_firmware *fw, const char *state_path,
                                    const uint8_t *state_buf, size_t state_len,
                                    const dt_engine_config *cfg, char *err, size_t errlen);
DT_API void dt_engine_free(dt_engine *e);
/* un giro del worker: ips in attesa, ingressi del pannello, BUDGET (400.000)
   istruzioni, schermo, LED. -> 0, 1 fermato */
DT_API int dt_engine_chunk(dt_engine *e);
DT_API int dt_engine_midi_in(dt_engine *e, const uint8_t *data, size_t n);
DT_API int dt_engine_panel(dt_engine *e, int kind, int code, int arg);
/* byte grezzi sul filo del pannello (Emulator.inbox ('raw', None, bytes)) */
DT_API int dt_engine_panel_raw(dt_engine *e, const uint8_t *data, size_t n);
DT_API size_t dt_engine_midi_out(dt_engine *e, uint8_t *buf, size_t max);
/* l'audio del motore delle voci: un blocco a ogni tick di scambio (32 frame
   stereo float32 interlacciati L, R, gia' moltiplicati per il volume), come
   rytmsynth.RytmSynth.on_frame lo scrive al WavSink. `voices`: 8 x 32 frame
   stereo (le voci BD BT SD LT CH MT CY RS prima degli effetti) o NULL. */
DT_API void dt_engine_set_audio_sink(dt_engine *e,
                                     void (*sink)(void *ctx, const float *lr, const float *voices,
                                                  size_t frames),
                                     void *ctx);
DT_API void dt_engine_set_volume(dt_engine *e, double volume);   /* rytmsynth.MASTER: 0..1 */
DT_API int64_t dt_engine_audio_frames(dt_engine *e);
/* 128 x 64 pixel (0/1) dell'OLED come la GUI li mostra -> versione */
DT_API uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64]);
/* i LED (emu/rytm_leds.py RytmLedState): per ognuno dei 128 LED il valore
   mostrato (VV = colore << 4 | livello, RytmLedState.lit), e i 12 pad RGB a
   4 bit -> versione */
DT_API uint64_t dt_engine_leds(dt_engine *e, uint8_t lit[128], uint8_t pads[12][3]);
DT_API int64_t dt_engine_instrs(dt_engine *e);
DT_API const char *dt_engine_status(dt_engine *e);
DT_API int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n);
DT_API dt_machine *dt_engine_machine(dt_engine *e);
/* lo stato (DTST v2, come tools/snap2dtst.py; compress: "DTSZ" + zlib), fra due giri */
DT_API int dt_engine_save_mem(dt_engine *e, int compress, uint8_t **out, size_t *len);
DT_API int dt_engine_save(dt_engine *e, const char *path, int compress);
/* +Drive (emu/rytm_rpc.py SysexRpcLink): una richiesta RPC (id + corpo, come
   Transfer) entra dalla MIDI IN come SysEx; le risposte si prendono al flush
   della porta USB. dt_engine_rpc_replies: [u16 seq, u8 id, u32 n, n byte] ... */
DT_API int dt_engine_rpc(dt_engine *e, int id, const uint8_t *body, size_t n, uint16_t *seq);
DT_API size_t dt_engine_rpc_replies(dt_engine *e, uint8_t *buf, size_t max);

/* ------------------------------------------------------------ primo avvio
 * Lo stato iniziale del plugin NON sta nel plugin ne' nel repo: si genera al
 * primo avvio dal solo firmware (.syx dell'utente) e si tiene in cache. Le due
 * funzioni sono la catena completa: boot freddo dal reset, +Drive formattato
 * dal firmware, calibrazione analogica sui nostri modelli, impostazioni SYNC
 * (CLOCK/TRANSPORT RECEIVE): lo stesso stato che prima veniva dal Python
 * (docs/AU-PORT.md 3 e 6b). `progress`: frazione 0..1 dell'insieme e la fase.
 * `cancel`: se diventa != 0 si ferma (DT_ERR_IO, "interrotto"). */
typedef void (*dt_progress_fn)(void *ctx, double frac, const char *what);

/* calibrazione analogica del firmware (tools/calloop.py): su un motore a
 * riposo (schermata principale), la flash SPI riceve il record RBOW */
DT_API int dt_engine_calibrate(dt_engine *e, volatile int *cancel, dt_progress_fn progress, void *ctx,
                               char *err, size_t errlen);
/* firmware -> stato DTST (compress: "DTSZ" + zlib) pronto per dt_rt_new_mem */
DT_API int dt_coldboot_session(const dt_firmware *fw, int compress, uint8_t **out, size_t *len,
                               volatile int *cancel, dt_progress_fn progress, void *ctx, char *err,
                               size_t errlen);
/* solo la prima fase: lo stato DTST del boot freddo dal reset a `point`
   istruzioni (RAM, registri, MMIO ed eSDHC: come `python -m emu.checkpoint make`) */
DT_API int dt_coldboot(const dt_firmware *fw, uint64_t point, int compress, uint8_t **out, size_t *len,
                       char *err, size_t errlen);
/* 1 se il firmware ha un profilo di indirizzi (si puo' emulare), 0 se sconosciuto;
   `version` (se dato) riceve il testo della versione (es. "1.73") o "" */
DT_API int dt_fw_supported(const dt_firmware *fw, char *version, size_t vlen);

/* ------------------------------------------------------------ tempo reale
 * dt_rt: il motore con il suo thread, per un plugin (rt.c, docs/AU-PORT.md).
 * dt_rt_render e dt_rt_midi dal thread audio (niente lock ne' allocazioni;
 * `offline` != 0 aspetta il motore), dt_rt_panel, dt_rt_set_volume e
 * dt_rt_screen dal thread della GUI. Uscite: canali 0-1 il mix (L, R), poi
 * 2 x 8 per le voci separate se richieste (nouts > 2). */
typedef struct dt_rt dt_rt;
#define DT_RT_CHANNELS 18
DT_API dt_rt *dt_rt_new(const uint8_t *syx, size_t syx_len, const char *state_path, double rate,
                        uint32_t max_frames, char *err, size_t errlen);
DT_API dt_rt *dt_rt_new_mem(const uint8_t *syx, size_t syx_len, const uint8_t *state, size_t state_len,
                            double rate, uint32_t max_frames, char *err, size_t errlen);
DT_API void dt_rt_free(dt_rt *rt);
DT_API int dt_rt_save(dt_rt *rt, uint8_t **out, size_t *len);
DT_API uint32_t dt_rt_latency(dt_rt *rt);                 /* campioni dell'host */
DT_API int dt_rt_midi(dt_rt *rt, uint32_t offset, const uint8_t *data, size_t n);
DT_API int dt_rt_render(dt_rt *rt, uint32_t n, float *const *outs, int nouts, int offline);
DT_API int dt_rt_panel(dt_rt *rt, int kind, int code, int arg);
DT_API void dt_rt_set_volume(dt_rt *rt, double volume);
/* schermo (128 x 64) e LED (come dt_engine_leds) -> versione */
DT_API uint64_t dt_rt_screen(dt_rt *rt, uint8_t out[128 * 64], uint8_t lit[128], uint8_t pads[12][3]);

/* lettura dei pattern per la vista Elektron Emu (SeqMem.hpp del plugin): ogni 8 giri il
   thread del motore chiama `fn` sulla macchina (fra due giri: la memoria e' ferma) e
   pubblica i byte che scrive (al piu' DT_SEQ_MAX); la GUI li legge con dt_rt_seq. */
#define DT_SEQ_MAX 512
typedef size_t (*dt_seq_reader)(dt_machine *m, uint8_t *out, size_t cap);
DT_API void dt_rt_set_seq_reader(dt_rt *rt, dt_seq_reader fn);
/* i byte dell'ultima lettura (-> quanti, 0 se nessuna); `version` (o NULL) cambia con loro */
DT_API size_t dt_rt_seq(dt_rt *rt, uint8_t *out, size_t cap, uint64_t *version);
DT_API size_t dt_rt_midi_out(dt_rt *rt, uint8_t *buf, size_t max, uint32_t *offset, uint32_t frames);
DT_API int dt_rt_rpc(dt_rt *rt, int id, const uint8_t *body, size_t n, uint16_t *seq);
DT_API size_t dt_rt_rpc_replies(dt_rt *rt, uint8_t *buf, size_t max);
DT_API int64_t dt_rt_underruns(dt_rt *rt);
DT_API const char *dt_rt_status(dt_rt *rt);
DT_API dt_engine *dt_rt_engine(dt_rt *rt);
DT_API int64_t dt_rt_max_lag(dt_rt *rt);
DT_API void dt_rt_offsets(dt_rt *rt, int64_t *E, int64_t *X);   /* in campioni a 48 kHz */

/* +Drive da un computer (drive.c: emu/transfer.py Client sopra dt_rt_rpc;
   bloccano, mai dal thread audio). */
typedef struct {
    uint32_t hash, size;
    int protected_;
    char type;                /* 'D' cartella, 'F' file */
    char name[256];
} dt_drive_entry;
DT_API int dt_drive_ls(dt_rt *rt, const char *path, dt_drive_entry **out, size_t *count, char *err,
                       size_t errlen);
DT_API int dt_drive_mkdir(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dt_drive_rmdir(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dt_drive_rm(dt_rt *rt, const char *path, char *err, size_t errlen);
DT_API int dt_drive_put(dt_rt *rt, const char *path, const uint8_t *data, size_t len,
                        void (*progress)(void *ctx, size_t off, size_t total), void *ctx, char *err,
                        size_t errlen);
/* WAV -> il contenitore dei sample del Rytm (rytm_rpc.wav_to_rytm) */
DT_API int dt_wav2rytm(const uint8_t *wav, size_t len, uint8_t **out, size_t *out_len, char *err,
                       size_t errlen);

DT_API const char *dt_version(void);
/* 1 se il processo puo' allocare memoria eseguibile per il JIT di Unicorn
   (macOS: entitlement com.apple.security.cs.allow-jit col runtime protetto). */
DT_API int dt_jit_available(void);

#ifdef __cplusplus
}
#endif
#endif
