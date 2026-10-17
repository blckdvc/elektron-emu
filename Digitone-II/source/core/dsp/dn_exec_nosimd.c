/* Esecutore dell'interprete, variante senza SIMD (PEYEN = 0) (vedi dn_exec.inc). */
#include "dn_interp_names.h"
#define EXP_NOSIMD
#define IDLE_WAITING 0u
#define IDLE_READY0 0u
#include "rt/rt.h"
#include "dn_interp_int.h"
#define EXEC_NAME dni_exec_nosimd
#include "dn_exec.inc"
