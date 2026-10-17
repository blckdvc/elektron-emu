/* La macchina di base (machine.c), vista dal motore (engine.c). Non API. */
#ifndef DT_MACHINE_H
#define DT_MACHINE_H

#include "dtcore.h"

#include <stdbool.h>
#include <stdint.h>
#include <unicorn/unicorn.h>

#define PAGE 0x100000u
#define NPAGES 4096u                /* 4 GB / 1 MB */
#define EXCP_RTE 0x100u
#define MAX_HANDLER 0x48000000u

typedef struct {
    uint32_t key, value;
} kv32;

typedef struct {
    struct dt_machine *m;
    uint32_t addr;
    uint16_t w;
} movec_site;

typedef struct {
    struct dt_machine *m;
    uint32_t addr;
    uint32_t value;
} mmio_site;

/* chiamato per i record DTST che la macchina non conosce (componenti) */
typedef int (*dtm_record_fn)(void *ctx, const char tag[4], const uint8_t *data, uint64_t len,
                             char *err, size_t errlen);

struct dt_machine {
    uc_engine *uc;
    uint8_t mapped[NPAGES / 8];
    uint8_t *host[NPAGES];          /* la RAM di ogni pagina mappata, nostra (uc_mem_map_ptr) */
    kv32 *ctl;
    size_t nctl, capctl;
    mmio_site *mmio;                /* Machine.mmio: indirizzo -> valore (ordine del dizionario) */
    size_t nmmio, capmmio;
    movec_site *movec;
    size_t nmovec;
    int32_t idle_yield;
    uint64_t idle_n;
    uint64_t movec_count;           /* Machine.movec_count (dallo stato, poi + i MOVEC eseguiti) */
    uint64_t ff1_count;
    dt_counters cnt;
    int state_loaded;
    uint8_t main_sha256[32];        /* del MAIN OS: lo stato (record FWID) deve combaciare */
    uint8_t *page;                  /* buffer da 1 MB per stato e impronta */
    int32_t stop_flag;              /* 1: un hook ha chiamato uc_emu_stop (dt2_spin lo guarda) */
    int32_t no_mmio_at_load;        /* il motore installa gli hook MMIO da se' (install_mmio) */
};


uint32_t dtm_rd32(uc_engine *uc, int reg);
void dtm_wr32(uc_engine *uc, int reg, uint32_t v);
void dtm_ensure(dt_machine *m, uint64_t addr);
int dtm_is_mapped(dt_machine *m, uint32_t base);
/* la memoria della pagina da 1 MB che contiene base (NULL se non mappata): la si
   legge direttamente, senza uc_mem_read (la fotografia dello stato, rt.c) */
uint8_t *dtm_host(dt_machine *m, uint32_t base);
/* Machine.raise_vector (senza srtrap); level < 0 = None. -> 1 preso, 0 no */
int dtm_raise_vector(dt_machine *m, uint32_t vec, int from_instruction, int level);
/* Machine.mmio: imposta/legge (un indirizzo nuovo si aggiunge in coda) */
int dtm_mmio_set(dt_machine *m, uint32_t addr, uint32_t v);
uint32_t *dtm_mmio_ref(dt_machine *m, uint32_t addr);
/* Machine.install_mmio: un hook di lettura per indirizzo, nell'ordine del dizionario */
void dtm_install_mmio(dt_machine *m);
/* stato DTST: i record della macchina; gli altri a `cb` (puo' essere NULL) */
int dtm_load_state(dt_machine *m, const char *path, dtm_record_fn cb, void *ctx, char *err, size_t errlen);
int dtm_load_state_mem(dt_machine *m, const uint8_t *buf, size_t len, dtm_record_fn cb, void *ctx,
                       char *err, size_t errlen);
uint8_t *dtm_inflate_state(const uint8_t *buf, size_t len, size_t *out_len);
void dtm_stop(dt_machine *m);

#endif
