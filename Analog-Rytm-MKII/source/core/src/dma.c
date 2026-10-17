/* Le copie eDMA da memoria a memoria avviate dal software (emu/nativedma.py):
 * lo stesso C che compila il Python (native/dt2_dma.c), in una sua unita' di
 * traduzione (ha nomi propri: be32, TCD_BASE, ...), con il buffer di lavoro
 * per thread (piu' istanze del plugin, ciascuna nel suo thread). Dal Model:Cycles
 * (core/src/dma.c); il Rytm non ha l'hook di SSRT. */
#define DT2_TLS _Thread_local
#include "native/dt2_dma.c"

#include <stdlib.h>
#include <unicorn/unicorn.h>

#include "dma.h"

void *ar_dma_new(void)
{
    dma_state *s = (dma_state *)calloc(1, sizeof *s);
    if (s) {
        s->read = (mem_fn)uc_mem_read;
        s->write = (mem_fn)(void *)uc_mem_write;
    }
    return s;
}

void ar_dma_counters(void *st, int64_t out[4])
{
    dma_state *s = (dma_state *)st;
    out[0] = s->transfers;
    out[1] = s->bytes;
    out[2] = s->skipped;
    out[3] = s->irqs;
}

/* longrun.finish_pending_dma: una sessione salvata fra lo START di una copia
   e la lettura che pubblica DONE riparte con START acceso; quelle copie si
   completano al caricamento (MemDma(m).run(ch), poi il CSR finale a +0x1E).
   -> il numero di copie completate */
int ar_dma_finish_pending(void *st, void *uc)
{
    dma_state *s = (dma_state *)st;
    dma_state tmp = *s;                   /* MemDma(m): contatori a parte */
    int ch, n = 0;
    tmp.transfers = tmp.bytes = tmp.skipped = tmp.irqs = 0;
    for (ch = 0; ch < 64; ch++) {
        uint8_t b[2];
        uint32_t a = TCD_BASE + (uint32_t)ch * 0x20u + 0x1E;
        uint16_t csr;
        int final;
        if (uc_mem_read((uc_engine *)uc, a, b, 2) != UC_ERR_OK)
            return n;
        csr = be16(b);
        if (!(csr & 0x0001) || (csr & 0x00C0))
            continue;
        final = run(&tmp, uc, ch, -1, 0);
        if (final >= 0) {
            put16(b, (uint16_t)final);
            uc_mem_write((uc_engine *)uc, a, b, 2);
            n++;
        }
    }
    return n;
}
