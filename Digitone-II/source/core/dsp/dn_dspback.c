/* Il fronte del DSP: dn_dsp_* (dn_dsp.h) sopra il motore scelto (dn_dsp_ops).
 *
 * Ordine di scelta per le istanze nuove: un motore impostato con dn_dsp_set_ops; la libreria
 * nativa caricata con dn_dsp_native_load (uso personale: C ricompilato dal firmware, fatto
 * sulla macchina dell'utente, mai distribuito); il nativo linkato (build di sviluppo, DN_STATIC_DSP);
 * l'interprete (nel plugin, parte dal solo .syx). */
#include "dn_dsp.h"
#include "dncore.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zlib.h>
#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

struct dn_dsp {
    const dn_dsp_ops *ops;
    void *impl;
};

static const dn_dsp_ops *g_forced;
static const dn_dsp_ops *g_dylib_ops;
static void *g_dylib;

void dn_dsp_set_ops(const dn_dsp_ops *ops) { g_forced = ops; }

const dn_dsp_ops *dn_dsp_get_ops(void)
{
    if (g_forced)
        return g_forced;
    if (g_dylib_ops)
        return g_dylib_ops;
    {
        const char *e = getenv("DN_DSP_BACKEND");          /* prove: interp | native */
        if (e && !strcmp(e, "interp"))
            return dni_dsp_get_ops();
#ifdef DN_STATIC_DSP
        if (e && !strcmp(e, "native"))
            return dnn_dsp_get_ops();
#endif
    }
#ifdef DN_STATIC_DSP
    return dnn_dsp_get_ops();
#else
    return dni_dsp_get_ops();
#endif
}

const char *dn_dsp_backend_name(void)
{
    const dn_dsp_ops *o = dn_dsp_get_ops();
    return o ? o->name : "nessuno";
}

/* la uncompress del plugin per le librerie costruite sul computer dell'utente (DN_EXT_INFLATE:
   niente zlib da collegare nella libreria) */
static int host_uncompress(unsigned char *dst, unsigned long *dstlen, const unsigned char *src, unsigned long srclen)
{
    uLongf n = (uLongf)*dstlen;
    int r = uncompress(dst, &n, src, (uLong)srclen);
    *dstlen = (unsigned long)n;
    return r;
}

int dn_dsp_native_load(const char *path, char *err, size_t errlen)
{
    typedef const dn_dsp_ops *(*get_ops_fn)(void);
    typedef void (*set_unc_fn)(int (*)(unsigned char *, unsigned long *, const unsigned char *, unsigned long));
    void *h;
    get_ops_fn fn;
    set_unc_fn su;
    const dn_dsp_ops *o;
    if (g_dylib)
        return 0;
#if defined(_WIN32)
    {
        wchar_t wp[1100];
        if (!MultiByteToWideChar(CP_UTF8, 0, path, -1, wp, 1100)) {
            if (err && errlen)
                snprintf(err, errlen, "%s: bad path", path);
            return -1;
        }
        h = (void *)LoadLibraryW(wp);
        if (!h) {
            if (err && errlen)
                snprintf(err, errlen, "%s: LoadLibrary error %lu", path, (unsigned long)GetLastError());
            return -1;
        }
        fn = (get_ops_fn)(void *)GetProcAddress((HMODULE)h, "dnn_dsp_get_ops");
        su = (set_unc_fn)(void *)GetProcAddress((HMODULE)h, "dnn_set_uncompress");
    }
#else
    h = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        if (err && errlen)
            snprintf(err, errlen, "%s", dlerror());
        return -1;
    }
    fn = (get_ops_fn)dlsym(h, "dnn_dsp_get_ops");
    su = (set_unc_fn)dlsym(h, "dnn_set_uncompress");
#endif
    if (su)
        su(host_uncompress);
    o = fn ? fn() : NULL;
    if (!o) {
        if (err && errlen)
            snprintf(err, errlen, "%s: not the native DSP library", path);
#if defined(_WIN32)
        FreeLibrary((HMODULE)h);
#else
        dlclose(h);
#endif
        return -1;
    }
    g_dylib = h;
    g_dylib_ops = o;
    return 0;
}

void dn_dsp_native_unload(void)
{
    g_dylib_ops = NULL;
    /* la libreria resta caricata: le istanze create la usano ancora */
}

dn_dsp *dn_dsp_new(const uint8_t *dnds, size_t len, char *err, size_t errlen)
{
    const dn_dsp_ops *o = dn_dsp_get_ops();
    dn_dsp *d;
    if (!o) {
        if (err && errlen)
            snprintf(err, errlen, "nessun motore del DSP disponibile");
        return NULL;
    }
    d = (dn_dsp *)calloc(1, sizeof *d);
    if (!d)
        return NULL;
    d->ops = o;
    if (o->key[0] && len >= 48 && !memcmp(dnds + 8, "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", 16)) {
        /* stato fatto dall'interprete (chiave a zero) per il motore nativo: la sua chiave */
        uint8_t *cp = (uint8_t *)malloc(len);
        if (!cp) { free(d); return NULL; }
        memcpy(cp, dnds, len);
        memcpy(cp + 8, o->key, 16);
        d->impl = o->create(cp, len, err, errlen);
        free(cp);
    } else
        d->impl = o->create(dnds, len, err, errlen);
    if (!d->impl) {
        free(d);
        return NULL;
    }
    return d;
}

void dn_dsp_free(dn_dsp *d)
{
    if (!d)
        return;
    d->ops->destroy(d->impl);
    free(d);
}

int dn_dsp_frame(dn_dsp *d, const uint8_t *frame, size_t n) { return d->ops->frame(d->impl, frame, n); }
int dn_dsp_pull(dn_dsp *d, int32_t *out, int max) { return d->ops->pull(d->impl, out, max); }
uint32_t dn_dsp_reply(dn_dsp *d, uint8_t *out, uint32_t n) { return d->ops->reply(d->impl, out, n); }
const char *dn_dsp_error(const dn_dsp *d) { return d->ops->error(d->impl); }
int64_t dn_dsp_cycles(const dn_dsp *d) { return d->ops->cycles(d->impl); }
const uint8_t *dn_dsp_memory(const dn_dsp *d, int which, size_t *len) { return d->ops->memory(d->impl, which, len); }
const char *dn_dsp_key(void)
{
    const dn_dsp_ops *o = dn_dsp_get_ops();
    return o ? o->key : "";
}

void dn_dsp_analyze(dn_dsp *d)
{
    if (d && d->ops->analyze)
        d->ops->analyze(d->impl);
}
