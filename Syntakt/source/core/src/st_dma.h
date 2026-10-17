/* st_dma.c verso st_engine.c (non API). */
#ifndef ST_DMA_H
#define ST_DMA_H
#include <stdint.h>
void *st_dma_new(void);
/* NativeMemDma.__init__: per canale l'hook di scrittura e di lettura di CSR, poi SSRT */
void st_dma_install(void *uc, void *state);
void st_dma_counters(void *state, int64_t out[4]);
#endif
