/* compat-gcc.h -- builtin di clang che gcc (mingw-w64, Windows) non ha, per i sorgenti del core
 * che il plugin compila (core/dsp/rt/rt.h: l'interprete del DSP). Si include con -include
 * dal CMake del plugin solo con GCC; il core non si tocca. */
#ifndef DN_COMPAT_GCC_H
#define DN_COMPAT_GCC_H
#if defined(__GNUC__) && !defined(__clang__)
#include <stdint.h>
static inline uint32_t dn_compat_bitrev32(uint32_t v)
{
    v = ((v >> 1) & 0x55555555u) | ((v & 0x55555555u) << 1);
    v = ((v >> 2) & 0x33333333u) | ((v & 0x33333333u) << 2);
    v = ((v >> 4) & 0x0F0F0F0Fu) | ((v & 0x0F0F0F0Fu) << 4);
    return __builtin_bswap32(v);
}
#define __builtin_bitreverse32(x) dn_compat_bitrev32(x)
#endif
#endif
