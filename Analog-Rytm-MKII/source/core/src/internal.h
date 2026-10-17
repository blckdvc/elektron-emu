/* Parti interne di dtcore condivise fra i file (non API). */
#ifndef DT_INTERNAL_H
#define DT_INTERNAL_H

#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

static inline void dt_errf(char *err, size_t errlen, const char *fmt, ...)
{
    va_list ap;
    if (!err || !errlen)
        return;
    va_start(ap, fmt);
    vsnprintf(err, errlen, fmt, ap);
    va_end(ap);
}

/* il salvataggio in due fasi (e_state.inc.c): la fotografia sul thread del
   motore, la compressione dove si vuole (rt.c dt_rt_save) */
struct dt_engine;
int dt_engine_save_raw(struct dt_engine *e, uint8_t **out_p, size_t *out_n, size_t *cap_io);
int dt_state_finish(const uint8_t *raw, size_t n, int compress, uint8_t **out_p, size_t *out_n);

#endif
