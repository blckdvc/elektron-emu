/* Parti interne di dtcore condivise fra i file (non API). */
#ifndef DT_INTERNAL_H
#define DT_INTERNAL_H

#include <stdarg.h>
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

#endif
