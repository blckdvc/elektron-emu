/* Le copie eDMA da memoria a memoria avviate dal software (emu/nativedma.py):
 * lo stesso C che compila il Python (native/dt2_dma.c), in una sua unita' di
 * traduzione (ha nomi propri: be32, TCD_BASE, ...), con il buffer di lavoro
 * per thread (piu' istanze del plugin, ciascuna nel suo thread). */
#define DT2_TLS _Thread_local
#include "native/dt2_dma.c"

#include <stdlib.h>
#include <unicorn/unicorn.h>

#include "dma.h"

void *mc_dma_new(void)
{
    dma_state *s = (dma_state *)calloc(1, sizeof *s);
    if (s) {
        s->read = (mem_fn)uc_mem_read;
        s->write = (mem_fn)(void *)uc_mem_write;
    }
    return s;
}

void mc_dma_counters(void *st, int64_t out[4])
{
    dma_state *s = (dma_state *)st;
    out[0] = s->transfers;
    out[1] = s->bytes;
    out[2] = s->skipped;
    out[3] = s->irqs;
}

/* NativeMemDma._ssrt: SSRT (un byte, il canale) come una scrittura di CSR con START */
void mc_dma_on_ssrt(void *uc, int type, uint64_t address, int size, int64_t value, void *user)
{
    uint8_t b[2] = {0, 0};
    uint32_t a;
    int ch;
    (void)type; (void)address;
    if (size != 1 || (value & 0x40))
        return;
    ch = (int)(value & 0x3F);
    a = TCD_BASE + (uint32_t)ch * 0x20u + 0x1E;
    uc_mem_read((uc_engine *)uc, a, b, 2);
    dt2_dma_on_csr(uc, 17, a, 2, (int64_t)(((uint32_t)b[0] << 8 | b[1]) | 0x0001), user);
}
