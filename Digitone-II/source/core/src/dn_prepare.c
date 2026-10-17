/* dncore: dal solo .syx dell'utente allo stato iniziale (API per il plugin, dncore.h).
 *
 * Nulla del firmware sta nel plugin: il controllo del file, lo stato iniziale del DSP
 * (l'interprete SHARC+ fa il boot dallo stream della sezione 7) e la sessione iniziale della
 * ColdFire (boot freddo, dn_coldboot_session) si fanno a runtime e vanno in una cartella di
 * cache dell'utente. Vedi docs/DSP-INTERPRETE.md e docs/COLDBOOT.md. */
#include "dtcore.h"
#include "dncore.h"
#include "internal.h"
#include "sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#if defined(_WIN32)
#include <direct.h>
#define dn_mkdir(p) _mkdir(p)
#else
#define dn_mkdir(p) mkdir((p), 0755)
#endif

#include "dn_profiles.inc.c"

static void hex_of(const uint8_t *h, char out[65])
{
    static const char *d = "0123456789abcdef";
    for (int i = 0; i < 32; i++) {
        out[2 * i] = d[h[i] >> 4];
        out[2 * i + 1] = d[h[i] & 15];
    }
    out[64] = 0;
}

/* il container ELE3 decodificato: +7 il codice prodotto (5 caratteri), +0x14 la versione */
int dn_fw_check(const uint8_t *syx, size_t len, dn_fw_info *info, char *err, size_t errlen)
{
    dt_firmware fw;
    int r;
    const dt_section *m;
    uint8_t sha[32];
    dt_sha256 h;
    if (!syx || !info)
        return DT_ERR_ARG;
    memset(info, 0, sizeof *info);
    r = dt_fw_extract(syx, len, &fw, err, errlen);
    if (r)
        return r;
    hex_of(fw.syx_sha256, info->sha256);
    if (fw.container && fw.container_len >= 0x20) {
        char prod[6], ver[13];
        memcpy(prod, fw.container + 7, 5);
        prod[5] = 0;
        memcpy(ver, fw.container + 0x14, 12);
        ver[12] = 0;
        for (int i = 11; i >= 0 && (ver[i] == ' ' || ver[i] == 0); i--)
            ver[i] = 0;
        for (int i = 0; ver[i]; i++)
            if ((unsigned char)ver[i] < 32 || (unsigned char)ver[i] > 126)
                ver[i] = 0;
        snprintf(info->version, sizeof info->version, "%s", ver);
        if (!strcmp(prod, "40059"))
            snprintf(info->machine, sizeof info->machine, "Digitone II");
        else if (!strcmp(prod, "+0079"))
            snprintf(info->machine, sizeof info->machine, "Digitakt II");
        else if (!strcmp(prod, "J0094"))
            snprintf(info->machine, sizeof info->machine, "Syntakt");
    }
    m = dt_fw_section(&fw, 3);
    if (!m) {
        dt_errf(err, errlen, "il firmware non ha la sezione MAIN OS");
        dt_fw_free(&fw);
        return DT_ERR_FORMAT;
    }
    dt_sha256_init(&h);
    dt_sha256_update(&h, m->data, m->len);
    dt_sha256_final(&h, sha);
    hex_of(sha, info->main_sha256);
    for (size_t i = 0; i < DT_NPROFILES; i++)
        if (!strcmp(DT_PROFILES[i].main_sha256, info->main_sha256))
            info->known = 1;
    dt_fw_free(&fw);
    return DT_OK;
}

static int mkdirs(const char *dir)
{
    char tmp[1024];
    size_t n = strlen(dir);
    if (n == 0 || n >= sizeof tmp)
        return -1;
    memcpy(tmp, dir, n + 1);
    for (size_t i = 1; i < n; i++)
        if (tmp[i] == '/' || tmp[i] == '\\') {
            char c = tmp[i];
            tmp[i] = 0;
            dn_mkdir(tmp);
            tmp[i] = c;
        }
    dn_mkdir(tmp);
    struct stat st;
    return stat(tmp, &st) == 0 ? 0 : -1;
}

static uint8_t *read_all(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *p = NULL;
    long n;
    if (!f)
        return NULL;
    if (fseek(f, 0, SEEK_END) == 0 && (n = ftell(f)) > 0 && fseek(f, 0, SEEK_SET) == 0) {
        p = (uint8_t *)malloc((size_t)n);
        if (p && fread(p, 1, (size_t)n, f) != (size_t)n) {
            free(p);
            p = NULL;
        }
        *len = (size_t)n;
    }
    fclose(f);
    return p;
}

static int write_atomic(const char *path, const uint8_t *data, size_t len)
{
    char tmp[1100];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "wb");
    if (!f)
        return -1;
    if (fwrite(data, 1, len, f) != len) {
        fclose(f);
        remove(tmp);
        return -1;
    }
    fclose(f);
#if defined(_WIN32)
    remove(path);
#endif
    return rename(tmp, path);
}

static void cache_path(char *out, size_t n, const char *dir, const char *kind, const char *sha, const char *ext)
{
    snprintf(out, n, "%s/%s-%.8s%s", dir, kind, sha, ext);
}

/* ------------------------------------------------------------------ stato del DSP */
#include "dn_dsp.h"

/* Il DSP parte da uno stato DNDS fatto dall'interprete SHARC+ con il boot della sezione 7 del
   firmware (2e9 cicli del DSP, ~10 s): dalla cache dell'utente se c'e', altrimenti fatto adesso
   e messo in cache (`dsp-<sha8>-i1.dnds`; i1 = versione dell'interprete). Nulla di tutto cio'
   sta nel plugin. */
#define DSP_BOOT_CYCLES 2000000000LL

int dn_prepare_dsp(const uint8_t *syx, size_t len, const char *cache_dir, volatile int *cancel,
                   dn_progress_fn progress, void *ctx, char *err, size_t errlen)
{
    dn_fw_info info;
    dt_firmware fw;
    char path[1100] = "";
    uint8_t *st;
    size_t n = 0;
    int r;
    r = dn_fw_check(syx, len, &info, err, errlen);
    if (r)
        return r;
    if (cache_dir && mkdirs(cache_dir) == 0) {
        cache_path(path, sizeof path, cache_dir, "dsp", info.sha256, "-i1.dnds");
        st = read_all(path, &n);
        if (st) {
            char e2[200];
            dn_dsp *probe = dn_dsp_new(st, n, e2, sizeof e2);     /* sana? */
            if (probe) {
                dn_dsp_free(probe);
                r = dn_set_dsp_state(st, n);
                free(st);
                if (progress)
                    progress(ctx, 100, "DSP");
                return r;
            }
            free(st);                                              /* cache rotta: si rifa */
        }
    }
    {
        const char *dev = getenv("DN_DEV_DSP_STATE");              /* solo sviluppo */
        if (dev && !getenv("DN_DEV_DSP_BOOT")) {
            st = read_all(dev, &n);
            if (st) {
                r = dn_set_dsp_state(st, n);
                free(st);
                return r;
            }
        }
    }
    r = dt_fw_extract(syx, len, &fw, err, errlen);
    if (r)
        return r;
    {
        const dt_section *s7 = dt_fw_section(&fw, 7);
        uint8_t *dnds = NULL;
        size_t dlen = 0;
        if (!s7) {
            dt_errf(err, errlen, "the firmware has no DSP section");
            dt_fw_free(&fw);
            return DT_ERR_FORMAT;
        }
        r = dn_dsp_boot(s7->data, s7->len, DSP_BOOT_CYCLES, cancel, progress, ctx, &dnds, &dlen, err, errlen);
        dt_fw_free(&fw);
        if (r)
            return DT_ERR_UNSUPPORTED;
        if (cache_dir && path[0])
            write_atomic(path, dnds, dlen);
        r = dn_set_dsp_state(dnds, dlen);
        free(dnds);
        return r;
    }
}

/* L'impronta della sezione DSP del firmware: lega la libreria nativa costruita dal plugin al suo firmware. */
int dn_dsp_program_hash(const uint8_t *syx, size_t len, char hex[65], char *err, size_t errlen)
{
    dt_firmware fw;
    const dt_section *s7;
    dt_sha256 h;
    uint8_t sha[32];
    int r = dt_fw_extract(syx, len, &fw, err, errlen);
    if (r)
        return r;
    s7 = dt_fw_section(&fw, 7);
    if (!s7) {
        dt_errf(err, errlen, "the firmware has no DSP section");
        dt_fw_free(&fw);
        return DT_ERR_FORMAT;
    }
    dt_sha256_init(&h);
    dt_sha256_update(&h, s7->data, s7->len);
    dt_sha256_final(&h, sha);
    hex_of(sha, hex);
    dt_fw_free(&fw);
    return 0;
}

_Static_assert(sizeof(dt_sha256) <= sizeof(dn_sha256_ctx), "dn_sha256_ctx too small");

void dn_sha256_begin(dn_sha256_ctx *c)
{
    dt_sha256_init((dt_sha256 *)c);
}
void dn_sha256_add(dn_sha256_ctx *c, const void *data, size_t n)
{
    dt_sha256_update((dt_sha256 *)c, data, n);
}
void dn_sha256_end(dn_sha256_ctx *c, uint8_t out[32])
{
    dt_sha256_final((dt_sha256 *)c, out);
}

/* ------------------------------------------------------------------ sessione ColdFire */
/* La sessione iniziale: dalla cache `session-<sha8>.dtsz`, o dal boot freddo (dn_coldboot_session,
   docs/COLDBOOT.md, ~10 s) e messa in cache. Se il DSP e' pronto (dn_prepare_dsp prima) il boot
   finisce con un secondo emulato col DSP. DN_DEV_SESSION (sviluppo): un file al posto del boot. */
typedef struct {
    dn_progress_fn fn;
    void *ctx;
} sess_prog;

static void sess_progress(void *ctx, int pct)
{
    sess_prog *p = (sess_prog *)ctx;
    p->fn(p->ctx, pct, "sessione");
}

int dn_make_session(const uint8_t *syx, size_t len, const char *cache_dir, volatile int *cancel,
                    dn_progress_fn progress, void *ctx, uint8_t **out, size_t *out_len, char *err, size_t errlen)
{
    dn_fw_info info;
    char path[1100] = "";
    int r;
    *out = NULL;
    *out_len = 0;
    r = dn_fw_check(syx, len, &info, err, errlen);
    if (r)
        return r;
    if (cache_dir && mkdirs(cache_dir) == 0) {
        cache_path(path, sizeof path, cache_dir, "session", info.sha256, ".dtsz");
        *out = read_all(path, out_len);
        if (*out)
            return DT_OK;
    } else
        path[0] = 0;
    {
        const char *dev = getenv("DN_DEV_SESSION");
        if (dev) {
            *out = read_all(dev, out_len);
            if (!*out) {
                dt_errf(err, errlen, "DN_DEV_SESSION illeggibile: %s", dev);
                return DT_ERR_IO;
            }
            if (progress)
                progress(ctx, 100, "sessione");
            return DT_OK;
        }
    }
    {
        dt_firmware fw;
        sess_prog sp;
        sp.fn = progress;
        sp.ctx = ctx;
        r = dt_fw_extract(syx, len, &fw, err, errlen);
        if (r)
            return r;
        r = dn_coldboot_session(&fw, 0, 1, out, out_len, cancel, progress ? sess_progress : NULL, &sp, err, errlen);
        dt_fw_free(&fw);
        if (r)
            return r;
        if (path[0])
            write_atomic(path, *out, *out_len);           /* la cache e' solo un acceleratore */
    }
    return DT_OK;
}
