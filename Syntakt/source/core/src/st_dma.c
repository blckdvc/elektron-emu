/* Le copie eDMA da memoria a memoria avviate dal software (emu/nativedma.py):
 * lo stesso C che compila il Python (native/st_dma.c), in una sua unita' di
 * traduzione (ha nomi propri: be32, TCD_BASE, run, ...), con il buffer di
 * lavoro per thread (piu' istanze del plugin, ciascuna nel suo thread). Come
 * model_cycles/emulator/core/src/dma.c. */
#define DT2_TLS _Thread_local
#include "native/st_dma.c"

#include <stdlib.h>
#include <unicorn/unicorn.h>

#include "st_dma.h"

void *st_dma_new(void)
{
    dma_state *s = (dma_state *)calloc(1, sizeof *s);
    if (s) {
        s->read = (mem_fn)uc_mem_read;
        s->write = (mem_fn)(void *)uc_mem_write;
    }
    return s;
}

void st_dma_install(void *uc, void *state)
{
    int ch;
    uc_hook h;
    for (ch = 0; ch < 64; ch++) {
        uint64_t csr = TCD_BASE + (uint64_t)ch * 0x20u + 0x1C;
        uc_hook_add((uc_engine *)uc, &h, UC_HOOK_MEM_WRITE, (void *)dt2_dma_on_csr, state, csr, csr + 3);
        uc_hook_add((uc_engine *)uc, &h, UC_HOOK_MEM_READ, (void *)dt2_dma_on_read, state, csr, csr + 3);
    }
    uc_hook_add((uc_engine *)uc, &h, UC_HOOK_MEM_WRITE, (void *)dt2_dma_on_ssrt, state, 0xFC04401Eu,
                0xFC04401Eu);
}

void st_dma_counters(void *state, int64_t out[4])
{
    dma_state *s = (dma_state *)state;
    out[0] = s->transfers;
    out[1] = s->bytes;
    out[2] = s->skipped;
    out[3] = s->irqs;
}
