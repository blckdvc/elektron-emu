/* Gestori dell'esecutore a blocchi con SIMD (PEYEN): come dn_jit.c ma rt.h senza EXP_NOSIMD. */
#include "dn_interp_names.h"
#define IDLE_WAITING 0u
#define IDLE_READY0 0u
#include "rt/rt.h"
#include "dn_interp_int.h"

typedef void (*hplain_fn)(St *, const Ins *);
typedef uint32_t (*hflow_fn)(St *, const Ins *, uint32_t);
#define HSUF(x) x##_s
#include "dn_jit_h.inc"
const hplain_fn *dni_tplain_s(void) { return tplain_s; }
const hflow_fn *dni_tflow_s(void) { return tflow_s; }
