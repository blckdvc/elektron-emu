// ColdBootStub.cpp -- il primo avvio finto, finche' stcore.h non dichiara
// st_coldboot_session (core/src/st_boot.inc.c, il boot freddo in C della Syntakt).
// Risponde "non disponibile". Non entra in nessun bundle quando il boot freddo c'e'.
// (ST_DEV_STATE, per le prove, e' gestito dal plugin: SyntaktPlugin.cpp.)
#include "dtcore.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" int st_coldboot_session(const dt_firmware*, int, uint8_t** out, size_t* len, volatile int*, char* err,
                                   size_t errlen)
{
    *out = nullptr;
    *len = 0;
    std::snprintf(err, errlen, "cold boot is not available in this build");
    return DT_ERR_UNSUPPORTED;
}
