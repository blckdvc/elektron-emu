/* stcore: il motore dell'emulatore della Syntakt in C, senza Python.
 *
 * Implementa l'API del motore di dtcore (digitakt/emulator/core/include/
 * dtcore.h: dt_engine_*, e con rt.c dt_rt_*), quindi il resto si riusa dal
 * Digitakt II: la macchina ColdFire, l'estrazione del firmware, il tempo
 * reale, l'adattatore AU. Qui solo cio' che e' della Syntakt. Piano e
 * verifiche: docs/AU-PORT.md.
 *
 * Stato: un contenitore STST (tools/snap2dtst.py) con le due CPU, MAIN e DSP,
 * ciascuna in DTST v2. Uscite audio (sink di dt_engine_set_audio_sink e
 * canali di dt_rt_render): 0 Main L, 1 Main R, 2-3 uscita senza Analog FX
 * (slot 5/3), 4-5 mandata all'Analog FX (slot 7/2). */
#ifndef STCORE_H
#define STCORE_H

#include "dtcore.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STCORE_VERSION "0.1.0"
#define ST_OUT_CHANNELS 6

DT_API const char *st_version(void);
/* la macchina della CPU DSP (la MAIN e' dt_engine_machine) */
DT_API dt_machine *st_engine_dsp(dt_engine *e);
DT_API void st_engine_pcs(dt_engine *e, uint32_t *pc, uint32_t *dpc);
/* i PC del giro nei registri delle due CPU (per leggerli con dt_machine_regs) */
DT_API void st_engine_sync_pc(dt_engine *e);

/* ------------------------------------------------------------ boot freddo
 * Lo stato iniziale a runtime, dal solo firmware (docs/AU-PORT.md, "Boot
 * freddo in C"). */

/* avanzamento della catena in corso, 0..100 (leggibile da un altro thread) */
DT_API int st_coldboot_progress(void);
/* la catena intera: dal solo firmware allo stato iniziale della sessione (STST,
   due CPU; `compress` come dt_engine_save_mem). `cancel` (o NULL) a 1
   interrompe con DT_ERR_IO; DT_ERR_UNSUPPORTED per un firmware senza profilo.
   *out si libera con free(). */
DT_API int st_coldboot_session(const dt_firmware *fw, int compress, uint8_t **out, size_t *len,
                               volatile int *cancel, char *err, size_t errlen);
/* diagnostica: boot + CoSim senza audio (cadenza 4,68M) + `script` ("run:N cXbY info
   screen"), e lo stato che ne esce (la forma di ui_ready) */
DT_API int st_coldboot_cold(const dt_firmware *fw, const char *script, int compress, uint8_t **out, size_t *len,
                            volatile int *cancel, char *err, size_t errlen);
/* solo il boot del MAIN dal reset fino a `point` istruzioni: uno STST con il
   MAIN e un DSP vuoto (diagnostica: confronto con i pioli della scala) */
DT_API int st_coldboot_main(const dt_firmware *fw, uint64_t point, uint8_t **out, size_t *len,
                            volatile int *cancel, char *err, size_t errlen);

#ifdef __cplusplus
}
#endif
#endif
