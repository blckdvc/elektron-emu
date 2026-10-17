/* dma.c verso engine.c (non API). */
#ifndef AR_DMA_H
#define AR_DMA_H
#include <stdint.h>
void *ar_dma_new(void);
void ar_dma_counters(void *st, int64_t out[4]);
int ar_dma_finish_pending(void *st, void *uc);
void dt2_dma_on_csr(void *uc, int type, uint64_t address, int size, int64_t value, void *user);
void dt2_dma_on_read(void *uc, int type, uint64_t address, int size, int64_t value, void *user);
#endif
