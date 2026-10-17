/* dma.c verso engine.c (non API). */
#ifndef MC_DMA_H
#define MC_DMA_H
#include <stdint.h>
void *mc_dma_new(void);
void mc_dma_counters(void *st, int64_t out[4]);
void dt2_dma_on_csr(void *uc, int type, uint64_t address, int size, int64_t value, void *user);
void dt2_dma_on_read(void *uc, int type, uint64_t address, int size, int64_t value, void *user);
void mc_dma_on_ssrt(void *uc, int type, uint64_t address, int size, int64_t value, void *user);
#endif
