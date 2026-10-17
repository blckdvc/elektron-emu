/* Il motore del Digitone II per il plugin (dncore): il worker della GUI v2 con
 * --audio (emu/gui.py Emulator + longrun.build + i modelli) in C, senza Python,
 * con il DSP vero accoppiato in modo sincrono (emu/dsplive.py DspSynth, live=False).
 * Implementa l'API dt_engine_* di dtcore.h (digitakt/emulator/core/include), cosi'
 * rt.c del Digitakt II gira su questo motore senza cambiare. Piano e verifiche:
 * docs/AU-PORT.md.
 *
 * Una sola unita' di traduzione: il C nativo che usa anche il Python
 * (native/dn_native.c, lo stesso testo che emu/nativehooks.py compila), poi le parti
 * dn_*.inc.c, derivate da quelle del Digitakt II (digitakt/emulator/core/src/e_*) e
 * adattate al Digitone: bus MMIO (emu/mmiobus.py), BITREV/BYTEREV con un hook, cicli
 * caldi in C, DSPI2 col percorso DMA, il DSP vero, Dspi1Tail. Gli hook si registrano
 * nell'ordine del Python (scratch/dn_hooktopo.py); il riferimento e'
 * tools/dncore_guiref.py e `dncore play` deve dare la sua stessa traccia. */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include "native/dn_native.c"

#include "dtcore.h"
#include "dncore.h"
#include "internal.h"
#include "machine.h"
#include "sha256.h"
#include "dn_dsp.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zlib.h>
#include <pthread.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread/qos.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "dn_profiles.inc.c"
typedef struct dt_engine dt_engine;
#include "dn_base.inc.c"
#include "dn_bus.inc.c"
#include "dn_build.inc.c"
#include "dn_timers.inc.c"
#include "dn_exact.inc.c"
#include "dn_dspi.inc.c"
#include "dn_periph.inc.c"
#include "dn_panel.inc.c"
#include "dn_state.inc.c"

static void leds_publish(dt_engine *e);

#define E_BUDGET 400000
#define E_PER_BLOCK 3.87
#define E_IDLE_YIELD 20000
#define E_SSI_HZ 96000
#define E_AUDIO_IPS 150000000
#define E_SSI_FORCE_RTE_VEC 191

/* ---------------------------------------------------- componenti DTST */
typedef struct {
    dt_engine *e;
    int have_ssi, have_etx, have_tim, have_sd;
    uint8_t *ctim;
    uint64_t ctim_len;
    uint32_t pages[512];          /* basi delle pagine da 1 MB dello stato */
    int npages;
} e_load;

static int comp_record(void *ctx, const char tag[4], const uint8_t *d, uint64_t n, char *err, size_t errlen)
{
    e_load *L = (e_load *)ctx;
    dt_engine *e = L->e;
    if (!memcmp(tag, "PAGX", 4)) {
        /* PAGE, scritta come l'host Python: le pagine del bus vanno allo store */
        uLongf out = PAGE;
        uint32_t base;
        if (n < 8 || e_le32(d + 4) != PAGE) {
            dt_errf(err, errlen, "malformed PAGE record");
            return DT_ERR_FORMAT;
        }
        base = e_le32(d);
        if (base % PAGE || !dtm_is_mapped(e->m, base)) {
            dt_errf(err, errlen, "page 0x%08x not in MAPS", base);
            return DT_ERR_FORMAT;
        }
        if (uncompress(e->m->page, &out, d + 8, (uLong)(n - 8)) != Z_OK || out != PAGE) {
            dt_errf(err, errlen, "bad page data at 0x%08x", base);
            return DT_ERR_FORMAT;
        }
        if (e_write(e, base, e->m->page, PAGE)) {
            dt_errf(err, errlen, "cannot write page 0x%08x", base);
            return DT_ERR_UNICORN;
        }
        if (L->npages < (int)(sizeof L->pages / sizeof L->pages[0]))
            L->pages[L->npages++] = base;
    } else if (!memcmp(tag, "CUIN", 4)) {
        e->uart_in.n = 0;
        e->uart_in.head = 0;
        if (n && e_fifo_push(&e->uart_in, d, (size_t)n))
            return DT_ERR_NOMEM;
    } else if (!memcmp(tag, "CETX", 4)) {
        if (n != 32 || e_le32(d) != 35 || e_le32(d + 4) != 155) {
            dt_errf(err, errlen, "bad edma_tx component");
            return DT_ERR_FORMAT;
        }
        e->etx.pending = (int32_t)e_le64(d + 8);
        e->etx.bytes = (int64_t)e_le64(d + 16);
        e->etx.transfers = (int64_t)e_le64(d + 24);
        e->etx.restored = 1;
        L->have_etx = 1;
    } else if (!memcmp(tag, "CSSI", 4)) {
        int64_t rh, num, den;
        ssi_state *c = &e->sst;
        if (n != 40 + 4 + 4 + 48 + 4 + 16) {
            dt_errf(err, errlen, "bad ssi0_dma component (%llu bytes)", (unsigned long long)n);
            return DT_ERR_FORMAT;
        }
        rh = (int64_t)e_le64(d);
        if (rh != E_SSI_HZ) {
            dt_errf(err, errlen, "Ssi0Dma request-rate mismatch");
            return DT_ERR_FORMAT;
        }
        c->ips = (int64_t)e_le64(d + 8);
        c->now = (int64_t)e_le64(d + 16);
        num = (int64_t)e_le64(d + 24);
        den = (int64_t)e_le64(d + 32);
        if (den) {
            __int128 v = (__int128)num * rh;
            if (v % den) {
                dt_errf(err, errlen, "Ssi0Dma: next non multiplo di 1/request_hz");
                return DT_ERR_FORMAT;
            }
            c->nextN = (int64_t)(v / den);
            c->has_next = 1;
        } else
            c->has_next = 0;
        c->en = (int32_t)(e_le32(d + 40) & 3);
        c->int50_asserted = d[44];
        c->int50_delivered = d[45];
        c->force_asserted = d[46];
        c->force_delivered = d[47];
        c->requests = (int64_t)e_le64(d + 48);
        c->major_rx = (int64_t)e_le64(d + 56);
        c->major_tx = (int64_t)e_le64(d + 64);
        c->sg_rx = (int64_t)e_le64(d + 72);
        c->sg_tx = (int64_t)e_le64(d + 80);
        c->tx_bytes = (int64_t)e_le64(d + 88);
        c->tx_crc32 = e_le32(d + 96);
        c->vector170 = (int64_t)e_le64(d + 100);
        c->vector191 = (int64_t)e_le64(d + 108);
        L->have_ssi = 1;
    } else if (!memcmp(tag, "CSDH", 4)) {
        e_esdhc *sd = &e->sd;
        uint64_t off = 0, i, cnt;
        if (n < 33) {                           /* 4+4+8+8+4+1+4: anche con l'overlay vuoto */
            dt_errf(err, errlen, "bad esdhc component");
            return DT_ERR_FORMAT;
        }
        sd->pattern = e_le32(d);
        sd->armed = (int32_t)e_le32(d + 4);
        if (sd->armed != SD_DMA_CHAN)
            sd->armed = -1;
        sd->dma_bytes = (int64_t)e_le64(d + 8);
        if ((int64_t)e_le64(d + 16) != sd->blocks) {
            dt_errf(err, errlen, "Esdhc card capacity mismatch");
            return DT_ERR_FORMAT;
        }
        sd->rca = e_le32(d + 24);
        sd->selected = d[28];
        cnt = e_le32(d + 29);
        off = 33;
        ov_free(&sd->ov);
        for (i = 0; i < cnt; i++) {
            uint64_t sec;
            int hasmask;
            e_sector *s;
            if (off + 9 + 512 > n)
                goto bad_sd;
            sec = e_le64(d + off);
            hasmask = d[off + 8];
            off += 9;
            s = ov_add(&sd->ov, sec);
            if (!s)
                return DT_ERR_NOMEM;
            memcpy(s->data, d + off, 512);
            off += 512;
            if (hasmask) {
                int k;
                if (off + 512 > n)
                    goto bad_sd;
                s->mask = (uint8_t *)malloc(512);
                if (!s->mask)
                    return DT_ERR_NOMEM;
                memcpy(s->mask, d + off, 512);
                s->present = 0;
                for (k = 0; k < 512; k++)
                    s->present += s->mask[k] != 0;
                off += 512;
            }
        }
        L->have_sd = 1;
        return 0;
    bad_sd:
        dt_errf(err, errlen, "truncated esdhc overlay");
        return DT_ERR_FORMAT;
    } else if (!memcmp(tag, "CTIM", 4)) {
        L->ctim = (uint8_t *)malloc((size_t)n);
        if (!L->ctim)
            return DT_ERR_NOMEM;
        memcpy(L->ctim, d, (size_t)n);
        L->ctim_len = n;
        L->have_tim = 1;
    }
    /* CSMP (dsp_samples): sul Digitone non c'e' SampleMemory (si scarta, come il Python) */
    return 0;
}

/* timers dalla CTIM: Pits(...), Dtims(..., clear_stale=False) e il loro stato */
static int timers_restore(dt_engine *e, const uint8_t *d, uint64_t n)
{
    uint64_t off = 4;
    uint32_t ns, i;
    if (n < 4)
        return -1;
    ns = e_le32(d);
    if (ns != 2)
        return -1;
    for (i = 0; i < ns; i++) {
        e_tsrc *s = &e->tim[i];
        uint32_t kind, nch, ch[4];
        int64_t ips, now;
        int held, c;
        const uint8_t *p = d + off;
        if (off + 8 + 16 + 17 + 36 + 64 + 8 > n)
            return -1;
        kind = e_le32(p);
        nch = e_le32(p + 4);
        for (c = 0; c < 4; c++)
            ch[c] = e_le32(p + 8 + 4 * c);
        ips = (int64_t)e_le64(p + 24);
        now = (int64_t)e_le64(p + 32);
        held = p[40];
        if (kind != i || nch > 4)
            return -1;
        tsrc_init(e, s, (int)kind, (int)nch, ch, ips, held);
        s->now = now;
        for (c = 0; c < 4; c++) {
            const uint8_t *q = p + 41 + 9 * c;
            s->t.has[c] = q[0] == 1;
            if (q[0] == 1)
                memcpy(&s->t.next[c], q + 1, 8);
        }
        for (c = 0; c < 4; c++) {
            s->t.fired[c] = (int64_t)e_le64(p + 77 + 8 * c);
            s->t.missed[c] = (int64_t)e_le64(p + 109 + 8 * c);
        }
        s->arm = e_le32(p + 141);
        s->stale = e_le32(p + 145);
        off += 149;
    }
    e->ntim = 2;
    return 0;
}

/* ------------------------------------------------- hook della bus page */
/* longrun do_halt via nativehooks._IdleState.slow: tx.deliver(), ogni yield il 32 */
static void idle_slow_c(int64_t n)
{
    dt_engine *e = e_cur;
    tx_deliver(e, &e->etx);
    if (n % E_IDLE_YIELD == 0)
        e_raise(e, 32, -1);
}

/* Ssi0Dma._on_serq (WriteFilter 48/50, un byte, mai 0x80, tutti con 0x40) */
static void h_ssi_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a;
    if (size != 1 || (v & 0x80))
        return;
    if (v & 0x40)
        e->sst.en |= 3;
    else if ((v & 0x3F) == 48)
        e->sst.en |= 1;
    else if ((v & 0x3F) == 50)
        e->sst.en |= 2;
    if (e->sst.en && !e->sst.has_next) {          /* next = Fraction(now) + period */
        e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
        e->sst.has_next = 1;
    }
}

/* dsp.Fifo on_read (fast_writes): READY nei due byte di STATUS a ogni lettura */
static void h_dsp_status(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    static const uint8_t ready[2] = {0x00, 0x01};
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e_write((dt_engine *)ud, 0x8C000002u, ready, 2);
}

/* il terminal loop (solo la 1.15C del Digitakt) */
/* confronto a byte per il codice che non e' del firmware (NOP, BRA *) */
static int verify(dt_engine *e, uint32_t addr, const uint8_t *want, size_t n)
{
    uint8_t b[64];
    return n <= sizeof b && !e_read(e, addr, b, n) && !memcmp(b, want, n);
}

static int verify_h(dt_engine *e, uint32_t addr, const dn_vh *v)
{
    uint8_t b[64];
    return v->n <= sizeof b && !e_read(e, addr, b, v->n) && vh_match(b, v);
}

/* SSI0: il banco RX (Ssi0Dma.RX_BANK0): l'operando dell'addi.l del gestore di
   sincronizzazione del vettore 170, (costante del profilo: nel binario non c'e' la firma del codice) */
static uint32_t ssi_rx_bank0(dt_engine *e)
{
    (void)e;
    return 0x4E6DF100u;          /* Digitone II 1.11 (il profilo e' unico per quel MAIN OS) */
}

/* ------------------------------------------------------------- setup */
static dt_engine *engine_new(const dt_firmware *fw, const char *state, const uint8_t *state_buf,
                             size_t state_len, const dt_engine_config *cfg, char *err, size_t errlen);

dt_engine *dt_engine_new(const dt_firmware *fw, const char *state, const dt_engine_config *cfg,
                         char *err, size_t errlen)
{
    return engine_new(fw, state, NULL, 0, cfg, err, errlen);
}

dt_engine *dt_engine_new_mem(const dt_firmware *fw, const uint8_t *state, size_t len,
                             const dt_engine_config *cfg, char *err, size_t errlen)
{
    return engine_new(fw, NULL, state, len, cfg, err, errlen);
}

/* il DSP: lo stato DNDS da dn_engine_set_dsp_state (o da DNCORE_DSP_STATE) */
static uint8_t *g_dsp_state;
static size_t g_dsp_state_len;

int dn_set_dsp_state(const uint8_t *dnds, size_t len)
{
    uint8_t *p = (uint8_t *)malloc(len ? len : 1);
    if (!p)
        return DT_ERR_NOMEM;
    memcpy(p, dnds, len);
    free(g_dsp_state);
    g_dsp_state = p;
    g_dsp_state_len = len;
    return DT_OK;
}

const uint8_t *dn_dsp_state_get(size_t *len)
{
    if (len)
        *len = g_dsp_state_len;
    return g_dsp_state;
}

static int dsp_state_load(char *err, size_t errlen)
{
    const char *path;
    FILE *f;
    long n;
    if (g_dsp_state)
        return 0;
    path = getenv("DNCORE_DSP_STATE");
    if (!path) {
        dt_errf(err, errlen, "no DSP state (dn_set_dsp_state or DNCORE_DSP_STATE)");
        return -1;
    }
    f = fopen(path, "rb");
    if (!f) {
        dt_errf(err, errlen, "cannot open DSP state %s", path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    g_dsp_state = (uint8_t *)malloc(n > 0 ? (size_t)n : 1);
    if (!g_dsp_state || fread(g_dsp_state, 1, (size_t)n, f) != (size_t)n) {
        fclose(f);
        free(g_dsp_state);
        g_dsp_state = NULL;
        dt_errf(err, errlen, "cannot read DSP state %s", path);
        return -1;
    }
    fclose(f);
    g_dsp_state_len = (size_t)n;
    return 0;
}

/* lo stato DTST con i record PAGE rinominati PAGX (li scrive comp_record dal bus) */
static uint8_t *state_retag(const uint8_t *buf, size_t len, size_t *out_len, char *err, size_t errlen)
{
    uint8_t *p;
    size_t n, pos;
    uint32_t nrec, r;
    if (len >= 4 && !memcmp(buf, "DTSZ", 4)) {
        p = dtm_inflate_state(buf, len, &n);
        if (!p) {
            dt_errf(err, errlen, "bad compressed state");
            return NULL;
        }
    } else {
        p = (uint8_t *)malloc(len ? len : 1);
        if (!p)
            return NULL;
        memcpy(p, buf, len);
        n = len;
    }
    if (n < 16 || memcmp(p, "DTST", 4)) {
        free(p);
        dt_errf(err, errlen, "not a DTST state");
        return NULL;
    }
    nrec = e_le32(p + 8);
    pos = 16;
    for (r = 0; r < nrec && pos + 16 <= n; r++) {
        uint64_t rl = e_le64(p + pos + 8);
        if (!memcmp(p + pos, "PAGE", 4))
            memcpy(p + pos, "PAGX", 4);
        pos += 16 + (size_t)rl;
    }
    *out_len = n;
    return p;
}

static uint8_t *read_all(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *p;
    long n;
    if (!f)
        return NULL;
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    p = (uint8_t *)malloc(n > 0 ? (size_t)n : 1);
    if (p && fread(p, 1, (size_t)n, f) != (size_t)n) {
        free(p);
        p = NULL;
    }
    fclose(f);
    *len = (size_t)n;
    return p;
}

static dt_engine *engine_new(const dt_firmware *fw, const char *state, const uint8_t *state_buf,
                             size_t state_len, const dt_engine_config *cfg, char *err, size_t errlen)
{
    dt_engine *e;
    dt_machine_config mc = {0};
    uint8_t sha[32];
    char hex[65];
    dt_sha256 h;
    size_t i, off;
    e_load L;
    const uint8_t *img;
    size_t img_len;
    uint8_t *st_copy = NULL, *st_file = NULL;
    size_t st_len = 0;
    int r, k, ssi_legacy = 0, intro = 0;

    e = (dt_engine *)calloc(1, sizeof *e);
    if (!e) {
        dt_errf(err, errlen, "out of memory");
        return NULL;
    }
    e->fw = fw;
    e->main_os = dt_fw_section(fw, 3);
    if (!e->main_os) {
        dt_errf(err, errlen, "firmware without MAIN OS");
        goto fail;
    }
    img = e->main_os->data;
    img_len = e->main_os->len;
    dt_sha256_init(&h);
    dt_sha256_update(&h, img, img_len);
    dt_sha256_final(&h, sha);
    dt_sha256_hex(sha, hex);
    for (i = 0; i < DT_NPROFILES; i++)
        if (!strcmp(DT_PROFILES[i].main_sha256, hex))
            e->p = &DT_PROFILES[i];
    if (!e->p) {
        dt_errf(err, errlen, "unsupported firmware (MAIN OS %.16s...): no address profile", hex);
        goto fail;
    }
    e->dwell_chunks = cfg ? cfg->panel_dwell : 16;
    e->au.gain = 1.0;
    e->dsp_sync = getenv("DN_DSP_SYNC") && !strcmp(getenv("DN_DSP_SYNC"), "1");
    e->dsp_wait = getenv("DN_DSP_WAIT") ? atoi(getenv("DN_DSP_WAIT")) : 2;
    e->creator = pthread_self();
    e->boot_plain = g_boot_plain;

    /* il DSP (emu/dsplive.py DspSynth: lo snapshot dopo il boot, go_native); il boot freddo
       (dn_boot.inc.c) gira come tools/guirun.py, senza DSP ne' SSI0 */
    if (!e->boot_plain) {
        if (dsp_state_load(err, errlen))
            goto fail;
        e->au.dsp = dn_dsp_new(g_dsp_state, g_dsp_state_len, err, errlen);
        if (!e->au.dsp)
            goto fail;
    }

    /* dspboot.build_flash: 16 MB, container a 0x80000 */
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000) {
        dt_errf(err, errlen, "cannot build the flash image");
        goto fail;
    }
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);

    /* Machine(): MEM_INVALID; MOVEC (machine.c); il bus MMIO (vuoto); l'INTR */
    mc.idle_yield = 0;
    e->m = dt_machine_new(fw, &mc, err, errlen);
    if (!e->m)
        goto fail;
    e->m->no_mmio_at_load = 1;
    e->uc = e->m->uc;
    e_cur = e;

    /* lo stato nativo (nativehooks.install_exceptions, NativeSpin) */
    e->exc.reg_read = (reg_rw_fn)uc_reg_read;
    e->exc.reg_write = (reg_rw_fn)uc_reg_write;
    e->exc.mem_read = (mem_rw_fn)uc_mem_read;
    e->exc.mem_write = (mem_rw_fn)uc_mem_write;
    e->exc.vbr = DT_VBR;
    e->exc.r_a7 = UC_M68K_REG_A7;
    e->exc.r_sr = UC_M68K_REG_SR;
    e->exc.r_pc = UC_M68K_REG_PC;
    e->exc.bus = &e->bmap;
    e->q.exc = &e->exc;
    e->q.py_raise = py_raise_c;
    e->stp.stop = (stop_fn)uc_emu_stop;

    /* ---- longrun.build ---- */
    if (isa_hooks(e))                                   /* install_isa_patches_scoped */
        goto nomem;
    e_hook_code(e, (void *)h_flash_read, e, e->p->flash_read);
    e_hook_code(e, (void *)h_pend_call, e, e->p->pend_call);
    if (e->p->task_create)
        e_hook_code(e, (void *)h_task_create, e, e->p->task_create);
    e_hook_code(e, (void *)h_print, e, E_PRINT);
    if (e->p->set_pixel)
        e_hook_code(e, (void *)h_setpixel_count, e, e->p->set_pixel);
    if (e->p->px_copy)
        e_hook_code(e, (void *)h_pxcopy_count, e, e->p->px_copy);
    /* switch_to dietro un cancello, chiuso dalla GUI prima del primo passo */
    e->sw_gate.open = 0;
    e_hook_code(e, (void *)dt2_gate_code, &e->sw_gate, E_SWITCH_TO);
    e_hook_code(e, (void *)h_depack_clamp, e, e->p->depack_copy);
    /* edma */
    tx_init(&e->etx, 35, 155, &e->uart_out, &e->uart_out_total, &e->uart_out_crc);
    e_hook_code(e, (void *)h_edma_wait, e, e->p->uart8_tx_wait);
    if (bus_hook(e, BUS_W, E_SERQ, E_SERQ, (mmio_c_fn)h_edma_serq, e, bus_flt(0, 0, 1), 1ull << 35))
        goto busfail;
    /* ssi (Ssi0Dma.__init__): SERQ, CINT, INTFRCH1 sul bus; l'RTE forzato */
    if (!e->p->ssi0_dma_force_rte) {
        dt_errf(err, errlen, "SSI0 DMA model requires the force-ISR RTE symbol");
        goto fail;
    }
    e->sst.mem_read = (mem_rw_fn)uc_mem_read;
    e->sst.mem_write = (mem_rw_fn)uc_mem_write;
    e->sst.rh = E_SSI_HZ;
    e->sst.ips = 4680000;                     /* emu.pit.INSTR_PER_SEC, poi il checkpoint */
    e->sst.rx_peer = 1;
    e->sst.level_retry = 1;
    e->sst.rx_bank_bytes = 0x800;
    e->sst.rx_banks = 2;
    e->sst.marker = 0x007FFFFFu;
    e->sst.slack = 256;
    e->sst.level_retry_step = 8000;
    e->sst.q = &e->q;
    e->sst.tx_vec = 170;
    e->sst.force_vec = E_SSI_FORCE_RTE_VEC;
    if (!e->boot_plain) {
        if (bus_hook(e, BUS_W, E_SERQ, E_SERQ, (mmio_c_fn)h_ssi_serq, e, bus_flt(1, 1, 1),
                     (1ull << 48) | (1ull << 50))
            || bus_hook(e, BUS_W, 0xFC04401Cu, 0xFC04401Cu, (mmio_c_fn)dt2_ssi_on_cint, &e->sst, 0, 0)
            || bus_hook(e, BUS_W, 0xFC04C010u, 0xFC04C013u, (mmio_c_fn)dt2_ssi_on_intfrch1, &e->sst, 0, 0))
            goto busfail;
        e->ssi_gate.open = 1;
        e->ssi_gate.cfn = dt2_ssi_force_rte;
        e->ssi_gate.cdata = &e->sst;
        e_hook_code(e, (void *)dt2_gate_code, &e->ssi_gate, e->p->ssi0_dma_force_rte);
    }
    /* dsp (Fifo fast): la lettura di STATUS */
    if (bus_hook(e, BUS_R, 0x8C000002u, 0x8C000003u, (mmio_c_fn)h_dsp_status, e, 0, 0))
        goto busfail;
    /* DT2_SKIP_DELAYS: re.finditer(rb'\x51\xfc\x53[\x80-\x87]\x66\xfa') */
    for (off = 0; off + 6 <= img_len;) {
        if (img[off] == 0x51 && img[off + 1] == 0xFC && img[off + 2] == 0x53 && img[off + 3] >= 0x80
            && img[off + 3] <= 0x87 && img[off + 4] == 0x66 && img[off + 5] == 0xFA
            && e->ndelays < (int)(sizeof e->delays / sizeof e->delays[0])) {
            e_delay *d = &e->delays[e->ndelays++];
            d->e = e;
            d->reg = UC_M68K_REG_D0 + (img[off + 3] & 7);
            e_hook_code(e, (void *)h_skip_delay, d, DT_MAIN_LOAD + (uint32_t)off);
            off += 6;
        } else
            off++;
    }
    /* idle spin: in C alla fine (replace_idle_spins) */
    for (off = 0; off + 1 < img_len; off += 2)
        if (img[off] == 0x60 && img[off + 1] == 0xFE && e->nidle < 32)
            e->idle_addrs[e->nidle++] = DT_MAIN_LOAD + (uint32_t)off;
    /* softfloat, nell'ordine di ROUTINES */
    {
        const uint32_t addrs[7] = {e->p->sf_mulsf3, e->p->sf_subsf3, e->p->sf_addsf3, e->p->sf_divsf3,
                                   e->p->sf_abssf2, e->p->sf_fixsfsi, e->p->sf_cmpsf2};
        for (k = 0; k < 7; k++)
            if (addrs[k]) {
                e->sfs[k].e = e;
                e->sfs[k].op = k;
                e_hook_code(e, (void *)h_softfloat, &e->sfs[k], addrs[k]);
            }
    }
    /* bitmap */
    if (e->p->set_pixel)
        e_hook_code(e, (void *)h_bmp_set, e, e->p->set_pixel);
    if (e->p->get_pixel)
        e_hook_code(e, (void *)h_bmp_get, e, e->p->get_pixel);
    /* unblock (skip: i semafori; il router MIDI del Digitone gia' qui) */
    if (e->p->display_sem)
        e_skip_add(e, e->p->display_sem);
    if (e->p->sd_cmd_sem)
        e_skip_add(e, e->p->sd_cmd_sem);
    if (e->p->sd_data_sem)
        e_skip_add(e, e->p->sd_data_sem);
    if (e->p->sd_dma_sem)
        e_skip_add(e, e->p->sd_dma_sem);
    {
        size_t o = MI_ROUTER_PEND - DT_MAIN_LOAD;
        if (o + MI_ROUTER_VERIFY.n <= img_len && vh_match(img + o, &MI_ROUTER_VERIFY))
            e_skip_add(e, MI_ROUTER_SEM);
    }
    {
        const uint32_t rc[5] = {e->p->queue_recv, e->p->intro_park, e->p->display_wait, e->p->pump_wait,
                                e->p->tick_pend};
        for (k = 0; k < 5; k++)
            if (rc[k])
                e->skip_callers[e->nskip_callers++] = rc[k];
    }
    e_hook_code(e, (void *)h_satisfy, e, e->p->sem_pend);
    if (e->p->pend_b)
        e_hook_code(e, (void *)h_satisfy, e, e->p->pend_b);
    if (e->p->intro_done && e->p->frame_sem)
        e_hook_code(e, (void *)h_intro_done_skip, e, e->p->intro_done);
    /* rilascio del gate del frame */
    if (e->p->frame_sem && e->p->ctx_switch) {
        e->ctx_gate.open = 1;
        e->ctx_gate.py = g_release_on_entry;
        e_hook_code(e, (void *)dt2_gate_code, &e->ctx_gate, e->p->ctx_switch);
        if (e->p->intro_done)
            e_hook_code(e, (void *)h_intro_done_zero, e, e->p->intro_done);
    }
    /* UART8 */
    if (bus_hook(e, BUS_R, E_USR8, E_UDR8 + 3, (mmio_c_fn)h_uart8_read, e, 0, 0)
        || bus_hook(e, BUS_W, E_USR8, E_UDR8 + 3, (mmio_c_fn)h_uart8_write, e, 0, 0))
        goto busfail;
    mmio_set(e, 0xFC05C02Cu, 0x100000F0u);
    mmio_set(e, 0xEC03802Cu, 0x80000000u);
    /* esdhc (prima del ripristino: i valori di reset, poi lo stato) */
    sd_init(e);
    if (bus_hook(e, BUS_R, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3, (mmio_c_fn)h_sd_sysctl, e, 0, 0)
        || bus_hook(e, BUS_W, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3, (mmio_c_fn)h_sd_xfertyp, e, 0, 0)
        || bus_hook(e, BUS_W, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3, (mmio_c_fn)h_sd_datport, e, 0, 0)
        || bus_hook(e, BUS_W, E_SERQ, E_SERQ, (mmio_c_fn)h_sd_serq, e, bus_flt(0, 0, 0), 1ull << SD_DMA_CHAN))
        goto busfail;

    /* ---- restore_into ---- */
    memset(&L, 0, sizeof L);
    L.e = e;
    if (state) {
        st_file = read_all(state, &st_len);
        if (!st_file) {
            dt_errf(err, errlen, "cannot read %s", state);
            goto fail;
        }
        state_buf = st_file;
        state_len = st_len;
    }
    st_copy = state_retag(state_buf, state_len, &st_len, err, errlen);
    free(st_file);
    if (!st_copy)
        goto fail;
    r = dtm_load_state_mem(e->m, st_copy, st_len, comp_record, &L, err, errlen);
    free(st_copy);
    if (r) {
        free(L.ctim);
        goto fail;
    }
    /* Machine.mmio: restore_into fonde i valori dello stato (dict.update) */
    for (i = 0; i < e->m->nmmio; i++)
        mmio_set(e, e->m->mmio[i].addr, e->m->mmio[i].value);
    if (!L.have_ssi && !e->boot_plain) {
        /* uno stato senza il componente (main_os.snap, un gradino della scala): prima
           l'aggiornamento "legacy" (Ssi0Dma.arm_legacy: TCD48/50 gia' programmati dal
           firmware), altrimenti "cold" (arm_cold: TCD vuoti, i canali li accendono le SERQ
           del firmware), come emu/gui.py */
        ssi_legacy = dn_ssi_upgrade(e);
        if (!ssi_legacy) {
            dt_errf(err, errlen, "SSI0 DMA topology is absent; use an explicit legacy upgrade");
            free(L.ctim);
            goto fail;
        }
    }
    intro = e_intro_running(e);

    /* DT2_DSP_PEER: dspi2 (SR, SERQ) + audio (il DSP vero) */
    if (!e->boot_plain) {
        mmio_set(e, D2_SR, D2_TCF | D2_EOQF);
        e->dspi2_sr = mmio_ref(e, D2_SR);
        /* il W1C della SR: la cella e' quella di Machine.mmio (replace_mmio_forced) */
        if (bus_hook(e, BUS_W, D2_SR, D2_SR + 3, (mmio_c_fn)h_d2_sr_w1c, e, 0, 0)
            || bus_hook(e, BUS_W, E_SERQ, E_SERQ, (mmio_c_fn)h_d2_serq, e, bus_flt(1, 0, -1), 1ull << 28))
            goto busfail;
    }
    /* intfrc: gli hook di scrittura (in C) e gli RTE dietro un cancello */
    e->frc.q = &e->q;
    e->frc.retry = 8000;
    e->frc.skip[1] |= 1ull << 63;                 /* vettore 191 (emu/ssi.py) */
    e->frc.dirty = 1;
    {
        static const uint32_t ib[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
        for (k = 0; k < 3; k++)
            if (bus_hook(e, BUS_W, ib[k] + 0x10, ib[k] + 0x17, (mmio_c_fn)dt2_frc_on_write, &e->frc, 0, 0))
                goto busfail;
    }
    e->frc_gate.open = 1;
    e->frc_gate.cfn = dt2_frc_on_rte;
    e->frc_gate.cdata = &e->frc;
    {
        uint32_t a;
        uint8_t *buf = (uint8_t *)malloc(0x40320000u - 0x40000400u);
        if (!buf)
            goto nomem;
        e_read(e, 0x40000400u, buf, 0x40320000u - 0x40000400u);
        for (a = 0; a + 1 < 0x40320000u - 0x40000400u; a += 2)
            if (buf[a] == 0x4E && buf[a + 1] == 0x73)
                e_hook_code(e, (void *)dt2_gate_code, &e->frc_gate, 0x40000400u + a);
        free(buf);
    }
    /* midi */
    if (verify_h(e, MI_RX_HANDLER, &MI_RX_HANDLER_VERIFY)) {
        e_midi *mi = &e->midi;
        e->midi_ring_ptr = e_r32(e, MI_RX_HANDLER + MI_RING_BASE_OPERAND);
        mi->installed = 1;
        tx_init(&mi->tx, 37, 157, &mi->tx_out, &mi->tx_out_total, &mi->tx_out_crc);
        mi->hint = 1;
        if (bus_hook(e, BUS_W, E_SERQ, E_SERQ, (mmio_c_fn)h_mi_serq, e, bus_flt(0, 0, 1), 1ull << 37))
            goto busfail;
        mi->uimr = e_r8(e, MI_UIMR9);
        if (bus_hook(e, BUS_W, MI_UDR9, MI_UDR9, (mmio_c_fn)h_mi_udr, e, 0, 0)
            || bus_hook(e, BUS_W, MI_UIMR9, MI_UIMR9, (mmio_c_fn)h_mi_uimr, e, 0, 0)
            || bus_hook(e, BUS_R, MI_USR9, MI_USR9, (mmio_c_fn)h_mi_usr, e, 0, 0))
            goto busfail;
        mi->uart_masked = 1;
        if (bus_hook(e, BUS_W, MI_SIMR1, MI_CIMR1, (mmio_c_fn)h_mi_imr, e, 0, 0))
            goto busfail;
        if (verify_h(e, MI_ROUTER_PEND, &MI_ROUTER_VERIFY))
            e_skip_add(e, MI_ROUTER_SEM);
    }
    /* dspi1: la coda del frame (Dspi1Tail) */
    if (verify_h(e, D1_TAIL_SEND, &D1_TAIL_VERIFY)) {
        e->d1.installed = 1;
        if (bus_hook(e, BUS_W, E_SERQ, E_SERQ, (mmio_c_fn)h_d1_serq, e, bus_flt(1, 0, 0), 1ull << D1_TX_CHAN))
            goto busfail;
    }
    /* install_mmio: un hook di lettura per indirizzo forzato, nell'ordine del dizionario
       (replace_mmio_forced: la cella in C) */
    for (k = 0; k < e->nmmio; k++) {
        uint32_t a = e->mmio_addr[k];
        mmio_cell *c = &e->mmio_cells[k];
        uint8_t *stp;
        if (!bus_page_of(e, a & ~0xFFFu))           /* la pagina nasce con l'hook (MmioBus._page) */
            goto busfail;
        stp = bus_page(&e->bmap, a & ~0xFFFu);
        if (!stp || ((a + 3) & ~0xFFFu) != (a & ~0xFFFu)) {
            dt_errf(err, errlen, "forced MMIO 0x%08x outside a bus page", a);
            goto fail;
        }
        c->reg = stp + (a & 0xFFFu);
        c->cell = &e->mmio_val[k];
        c->addr = a;
        if (bus_hook(e, BUS_R, a, a + 3, (mmio_c_fn)dt2_mmio_force, c, 0, 0))
            goto busfail;
    }
    e->dspi2_sr = mmio_ref(e, D2_SR);
    /* edma: kick solo senza stato del modello */
    if (!e->etx.restored && e->p->uart8_tx_state && e_r32(e, e->p->uart8_tx_state))
        tx_run(e, &e->etx);

    /* ---- Emulator.run ---- */
    if (verify_h(e, PN_SET_LED, &PN_SET_LED_VERIFY)) {
        e_read(e, PN_LED_TABLE, e->leds.values, DN_NLEDS);
        e->leds.version++;
    }
    leds_publish(e);
    if (verify_h(e, TR_USB_SEND, &TR_USB_SEND_VERIFY)
        && verify_h(e, TR_ENQUEUE, &TR_ENQUEUE_BYTES)
        && verify_h(e, TR_QUEUE_VERIFY_AT, &TR_QUEUE_VERIFY)) {
        static const uint8_t nops[16] = {0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71,
                                         0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71};
        e->tr.installed = 1;
        dtm_ensure(e->m, TR_PAGE);
        dtm_ensure(e->m, TR_PAGE);
        e_write(e, TR_PAGE + 0x800, nops, sizeof nops);
        e_hook_code(e, (void *)h_tr_usb_send, e, TR_USB_SEND);
    }
    /* timers: restore_checkpoint_timers (Pits, poi Dtims con l'hook `arm` e i DTCN), o
       Timers(Pits(m, hold=intro), Dtims(m, channels=(3,), hold=intro)) nuovi (un gradino del
       boot freddo, senza il componente) */
    if (L.have_tim) {
        if (timers_restore(e, L.ctim, L.ctim_len)) {
            dt_errf(err, errlen, "bad timers component");
            free(L.ctim);
            goto fail;
        }
    } else {
        static const uint32_t pch[3] = {3, 2, 0}, dch[1] = {3};
        double p;
        int tr;
        tsrc_init(e, &e->tim[0], 0, 3, pch, 4680000, intro);
        tsrc_init(e, &e->tim[1], 1, 1, dch, 4680000, intro);
        e->ntim = 2;
        tr = tim_period(e->uc, &e->tim[1].t, 3, &p);            /* Dtims.clear_stale */
        if (tr < 0) {
            dt_errf(err, errlen, "cannot read DTIM3");
            goto fail;
        }
        if (tr > 0) {
            e_w16(e, DTIM_BASES[3], 0);
            e->tim[1].stale |= 1u << 3;
        }
    }
    free(L.ctim);
    L.ctim = NULL;
    if (bus_hook(e, BUS_W, DTIM_BASES[3], DTIM_BASES[3] + 0x0F, (mmio_c_fn)h_dtim_arm, &e->tim[1], 0, 0))
        goto busfail;
    if (e_r16(e, DTIM_BASES[0]) == 0) {
        e_w16(e, DTIM_BASES[0], 0x0003);
        e->dtcn_has[0] = 1;
        e->dtcn_origin[0] = 0;
    }
    for (i = 0; i < 4; i++) {
        e->dtcn_ud[i].e = e;
        e->dtcn_ud[i].ch = (int)i;
        if (bus_hook(e, BUS_R, DTIM_BASES[i] + 0x0C, DTIM_BASES[i] + 0x0F, (mmio_c_fn)h_dtcn_read,
                     &e->dtcn_ud[i], 0, 0)
            || bus_hook(e, BUS_W, DTIM_BASES[i], DTIM_BASES[i] + 0x0F, (mmio_c_fn)h_dtcn_write,
                        &e->dtcn_ud[i], 0, 0))
            goto busfail;
    }
    if (ssi_legacy)                              /* non ripristinato: ssi0.ips = pits.sources[0].ips */
        e->sst.ips = e->tim[0].ips;
    e->sst.now = e->tim[0].now;                  /* ssi0.align(pits.now) */
    if (ssi_legacy == 1) {                       /* acceso e non ripristinato: next = now + period */
        e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
        e->sst.has_next = 1;
    }
    if (!intro && (e->tim[0].t.held || e->tim[1].t.held)) {
        /* timer trattenuti su una base gia' dopo l'intro: rilasciati (tools/guirun.py
           release_intro_timers: anche i canali PIT dopo l'intro) */
        if (e->tim[0].t.nch == 1 && e->tim[0].t.ch[0] == 3) {
            e->tim[0].t.nch = 3;
            e->tim[0].t.ch[1] = 2;
            e->tim[0].t.ch[2] = 0;
        }
        e->tim[0].t.held = e->tim[1].t.held = 0;
    }
    e->sst.rx_bank0 = ssi_rx_bank0(e);           /* Ssi0Dma.RX_BANK0, dall'immagine in memoria */
    e->post_intro_ips = cfg && cfg->post_intro_ips ? cfg->post_intro_ips : E_AUDIO_IPS;
    e->use_panel = !(e->tim[0].t.held && e->tim[1].t.held);
    if (!e->use_panel) {
        if (e->p->intro_done)                    /* at(profile.intro_done, handover) */
            e_hook_code(e, (void *)h_handover, e, e->p->intro_done);
    } else {
        e->has_pending_ips = 1;                  /* _pending_ips.append((0, AUDIO_IPS)) */
        e->pending_ips_at = 0;
        e->pending_ips_n = e->post_intro_ips;
    }
    if (e->p->mainloop)
        e_hook_code(e, (void *)h_mainloop, e, e->p->mainloop);
    if (e->p->job_pump)
        e_hook_code(e, (void *)h_job_pump, e, e->p->job_pump);
    {
        static const uint8_t bra[2] = {0x60, 0xFE};
        if (verify(e, PN_TERMINAL, bra, 2))
            e_hook_code(e, (void *)h_terminal, e, PN_TERMINAL);
    }
    if (e->p->panel_diff && e->p->fb_front)
        e_hook_code(e, (void *)h_panel_latch, e, e->p->panel_diff);
    /* nativehooks.install: lo stepper (tb-budget) */
    if (uc_set_block_budget(e->uc, (int64_t *)&e->stp) != UC_ERR_OK) {
        dt_errf(err, errlen, "uc_set_block_budget failed (Unicorn without the tb-budget patch)");
        goto fail;
    }
    /* replace_idle_spins */
    e->idle.n = 0;
    e->idle.yield_every = E_IDLE_YIELD;
    e->idle.tx_pending = &e->etx.pending;
    e->idle.slow = idle_slow_c;
    e->idle.step = &e->stp;
    e->idle.exc = &e->exc;
    for (i = 0; i < (size_t)e->nidle; i++)
        e_hook_code(e, (void *)dt2_on_idle, &e->idle, e->idle_addrs[i]);
    /* replace_hot_loops: i byte del codice devono essere quelli attesi */
    {
        static const dn_vh c_fill = {18, {0xbd, 0x47, 0x38, 0x9c, 0xa8, 0xb8, 0xdd, 0xbe, 0x2b, 0x40, 0x52, 0x5f, 0x80, 0xf1, 0x41, 0x44, 0x2e, 0xea, 0xa8, 0xdd, 0x3c, 0x97, 0x58, 0x50, 0x09, 0x2b, 0x02, 0xf5, 0x01, 0xc0, 0x17, 0xf0}};
        static const dn_vh c_emac = {40, {0xc1, 0x6a, 0x34, 0xc8, 0x06, 0x5c, 0xb4, 0x5d, 0x44, 0x37, 0x98, 0xb4, 0xc2, 0x6b, 0x49, 0xc3, 0x3a, 0xb0, 0x93, 0x4a, 0xa7, 0x23, 0xa3, 0x4d, 0xae, 0x50, 0x34, 0xdf, 0x15, 0x15, 0x41, 0xeb}};
        static const struct { uint32_t hook, check, exit_pc; const dn_vh *vh; void *fn; } L2[2] = {
            {0x400CF91Cu, 0x400CF918u, 0x400CF918u, &c_fill, (void *)dt2_loop_dspi2_fill},
            {0x400DB160u, 0x400DB160u, 0x400DB188u, &c_emac, (void *)dt2_loop_emac_pairs}};
        for (k = 0; k < 2; k++) {
            loop_state *ls = &e->loops[k];
            if (!verify_h(e, L2[k].check, L2[k].vh))
                continue;
            ls->reg_read = (reg_rw_fn)uc_reg_read;
            ls->reg_write = (reg_rw_fn)uc_reg_write;
            ls->mem_read = (mem_rw_fn)uc_mem_read;
            ls->mem_write = (mem_rw_fn)uc_mem_write;
            ls->step = &e->stp;
            ls->exit_pc = L2[k].exit_pc;
            e_hook_code(e, L2[k].fn, ls, L2[k].hook);
        }
    }
    /* i timer leggono i registri DTIM dallo store del bus (timnative._setup) */
    tsrc_stores(e, &e->tim[0]);
    tsrc_stores(e, &e->tim[1]);
    /* nativespin.install: le sorgenti (ssi0, intc_force, midi, dspi1, transfer) */
    e->S.emu_start = (emu_start_fn)uc_emu_start;
    e->S.q = &e->q;
    e->S.stp = &e->stp;
    e->S.per_block = E_PER_BLOCK;
    e->S.ntim = 2;
    e->S.tim[0] = &e->tim[0].t;
    e->S.tim[1] = &e->tim[1].t;
    e->S.ssi = &e->sst;
    e->S.frc = &e->frc;
    e->S.nev = 0;
    if (!e->boot_plain)                         /* guirun: nessun Ssi0Dma */
        e->S.ev[e->S.nev++] = EV_SSI;
    e->S.ev[e->S.nev++] = EV_FRC;
    if (e->midi.installed) {
        int kk = e->S.nev++;
        e->S.ev[kk] = EV_PY;
        e->S.hint[kk] = &e->midi.hint;
        e->S.extra[kk] = &e->midi.tx.pending;
        e->S.py_step[kk] = mi_step;
        e->S.py_service[kk] = mi_service;
    }
    if (e->d1.installed) {
        int kk = e->S.nev++;
        e->S.ev[kk] = EV_PY;
        e->S.hint[kk] = &e->d1.hint;
        e->S.py_step[kk] = d1_step;
        e->S.py_service[kk] = d1_service;
    }
    if (e->tr.installed) {
        int kk = e->S.nev++;
        e->S.ev[kk] = EV_PY;
        e->S.hint[kk] = &e->tr.hint;
        e->S.py_step[kk] = tr_step;
        e->S.py_service[kk] = tr_service;
    }
    e->S.stop_flag = &e->m->stop_flag;
    /* l'hook sull'area RX della DSPI2 (il DSP in un thread suo): dal TCD 28 dello stato */
    if (!e->dsp_sync && !e->boot_plain) {
        uint32_t dst = e_r32(e, E_TCD_BASE + 28 * 0x20u + 0x10);
        uint32_t rn = (e_r16(e, E_TCD_BASE + 28 * 0x20u + 0x14) & 0x7FFF) * 2u;
        if (dst && rn && rn <= sizeof e->dspw.raw) {
            e->dspw.hooked_lo[0] = dst;
            e->dspw.hooked_n[0] = rn;
            e->dspw.nhooked = 1;
            e_hook_mem(e, UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, (void *)h_rx_access, e, dst, (uint64_t)dst + rn - 1);
        }
    }
    e->pc = e_rd(e, UC_M68K_REG_PC);
    snprintf(e->status, sizeof e->status, "running");
    return e;
busfail:
    dt_errf(err, errlen, "MMIO bus: cannot map or hook a peripheral page");
    goto fail;
nomem:
    dt_errf(err, errlen, "out of memory");
fail:
    dt_engine_free(e);
    return NULL;
}

void dt_engine_free(dt_engine *e)
{
    size_t i;
    int k;
    if (!e)
        return;
    if (getenv("DN_STATS"))
        fprintf(stderr, "dncore: DSP %lld frame, %.2f s nel DSP, il ColdFire ha aspettato %.2f s in %lld attese "
                "(%lld dall'hook RX)\n", (long long)e->au.frames, e->dspw.dsp_ns / 1e9, e->dspw.wait_ns / 1e9,
                (long long)e->dspw.waits, (long long)e->dspw.rx_flushes);
    dspw_stop(e);
    if (e->m)
        dt_machine_free(e->m);
    for (k = 0; k < e->nbpages; k++)
        free(e->bpages[k]);
    free(e->revs);
    free(e->flash);
    free(e->uart_out.p);
    free(e->uart_in.p);
    ov_free(&e->sd.ov);
    if (e->au.dsp)
        dn_dsp_free(e->au.dsp);
    free(e->midi.rx.p);
    free(e->midi.tx_out.p);
    free(e->midi.host_out.p);
    for (i = 0; i < e->tr.n; i++)
        free(e->tr.q[(e->tr.head + i) % e->tr.cap].data);
    free(e->tr.q);
    free(e->tr.last_reply.p);
    free(e->tr.parts.p);
    free(e->inbox);
    if (e_cur == e)
        e_cur = NULL;
    free(e);
}

/* --------------------------------------------------------------- spin */
/* longrun.spin(m, pc, BUDGET, pits=timers, fast=True, async_events) via
   NativeSpin.spin. -> 0 limite, 1 fermato (halt, pc zero, errore). */
static int engine_spin(dt_engine *e, int64_t instrs, int64_t *executed)
{
    spin_state *S = &e->S;
    int code, exact;
    S->base = e->tim[0].now;
    S->done = 0;
    S->instrs = instrs;
    S->pc = e->pc;
    S->now = S->base;
    S->steps = 0;
    S->err = S->uc_err = 0;
    e->q.err = 0;
    e->sst.err = 0;
    e->m->stop_flag = 0;
    e->m->cnt.halt_vec = -1;
    e->running = 1;
    exact = ex_needed(e);                       /* Dtims trattenuti (l'intro) o scadenze in Fraction */
    for (;;) {
        code = exact ? ex_spin(e, e->uc, S) : dt2_spin(e->uc, S);
        if (code == SPIN_LIMIT)
            break;
        if (code == SPIN_PC_ZERO) {
            snprintf(e->status, sizeof e->status, "halted: pc zero");
            break;
        }
        if (code == SPIN_UC_ERROR) {
            snprintf(e->status, sizeof e->status, "halted: %s", uc_strerror((uc_err)S->uc_err));
            break;
        }
        if (code == SPIN_STOPPED) {
            if (e->au.err[0]) {
                snprintf(e->status, sizeof e->status, "halted: %s", e->au.err);
                break;
            }
            if (e->m->cnt.halt_vec >= 0) {
                snprintf(e->status, sizeof e->status, "halted: unhandled vector %d at %#010x",
                         e->m->cnt.halt_vec, S->pc);
                break;
            }
            if (S->pc == 0) {
                snprintf(e->status, sizeof e->status, "halted: pc zero");
                break;
            }
            e->m->stop_flag = 0;
            S->done += S->executed;
            if ((exact ? ex_spin_service(e, e->uc, S) : dt2_spin_service(e->uc, S)) < 0) {
                snprintf(e->status, sizeof e->status, "halted: native error %d/%d", S->err, e->q.err);
                break;
            }
            continue;
        }
        snprintf(e->status, sizeof e->status, "halted: native error %d/%d/%d/%d", S->err, e->q.err,
                 e->sst.err, e->frc.err);
        break;
    }
    e->running = 0;
    e->tim[0].now = e->tim[1].now = S->base + S->done;
    e->pc = S->pc;
    *executed = S->done;
    return strncmp(e->status, "halted", 6) == 0;
}

/* il ritardo massimo dell'audio sul tempo del motore prima del riempimento con zeri */
#define AU_PAD_LAG 2048

/* Un giro del ciclo del worker della GUI (Emulator.run). -> 0, 1 fermato */
int dt_engine_chunk(dt_engine *e)
{
    int64_t executed = 0;
    e_cur = e;
    if (!cf_tuned)
        cf_tune_thread(e);
    if (e->halted)
        return 1;
    if (e->has_pending_ips && e->pending_ips_at <= e->instrs) {
        int kk;
        e->has_pending_ips = 0;
        for (kk = 0; kk < e->ntim; kk++)
            if (tsrc_set_ips(e, &e->tim[kk], e->pending_ips_n)) {
                snprintf(e->status, sizeof e->status, "halted: %s", e->err);
                e->halted = 1;
                return 1;
            }
        e->sst.ips = e->pending_ips_n;
        e->midi.ips = e->pending_ips_n;
    }
    e->pc = panel_drain(e, e->pc);
    if (engine_spin(e, E_BUDGET, &executed)) {
        e->halted = 1;
        return 1;
    }
    e->instrs += executed;
    /* il frame in corso del DSP resta in corso: si applica alla consegna successiva, al
       primo accesso all'RX, o se qualcuno legge lo stato (traccia, impronta, salvataggio) */
    if (e->au.err[0] && !e->halted) {
        snprintf(e->status, sizeof e->status, "halted: %s", e->au.err);
        e->halted = 1;
        return 1;
    }
    panel_publish(e);
    if (e->uart_out.n) {
        if (leds_feed(&e->leds, e->uart_out.p, e->uart_out.n))
            leds_publish(e);
        e->uart_out.n = 0;
    }
    /* l'uscita verso il tempo reale: il blocco in corso, e zeri se il DSP non produce
       (collegamento non ancora partito): l'audio resta allineato al tempo del motore */
    if (e->au.sink) {
        int64_t t48 = e->instrs / 3125;
        sink_flush(&e->au);
        while (e->au.emitted < t48 - AU_PAD_LAG) {
            int64_t nz = t48 - AU_PAD_LAG / 2 - e->au.emitted;
            if (nz > 512)
                nz = 512;
            memset(e->au.sink_l, 0, (size_t)nz * sizeof(float));
            memset(e->au.sink_r[0], 0, (size_t)nz * sizeof(float));
            e->au.sink_n = (size_t)nz;
            sink_flush(&e->au);
        }
    }
    e->chunk++;
    return 0;
}

/* ---------------------------------------------------------- ingressi */
int dt_engine_midi_in(dt_engine *e, const uint8_t *data, size_t n)
{
    if (!e->midi.installed)
        return DT_ERR_UNSUPPORTED;
    if (e_fifo_push(&e->midi.rx, data, n))
        return DT_ERR_NOMEM;
    e->midi.hint = 1;
    return DT_OK;
}

int dt_engine_panel(dt_engine *e, int kind, int code, int arg)
{
    e_input in;
    in.kind = kind;
    in.code = code;
    in.arg = arg;
    return inbox_push_back(e, in) ? DT_ERR_NOMEM : DT_OK;
}

int dt_engine_rpc(dt_engine *e, const uint8_t *payload, size_t n)
{
    e_transfer *t = &e->tr;
    e_msg msg;
    /* dn_rt_midi_inject: byte MIDI dal thread della GUI attraverso la coda RPC di rt.c
       (id DN_RPC_MIDI), per l'invio di SysEx dal pannello +DRIVE */
    if (n >= 5 && payload[4] == DN_RPC_MIDI)
        return dt_engine_midi_in(e, payload + 5, n - 5);
    if (!t->installed)
        return DT_ERR_UNSUPPORTED;
    msg.data = (uint8_t *)malloc(n ? n : 1);
    if (!msg.data)
        return DT_ERR_NOMEM;
    memcpy(msg.data, payload, n);
    msg.n = n;
    if (t->n == t->cap) {
        size_t cap = t->cap ? 2 * t->cap : 16, kk;
        e_msg *q = (e_msg *)malloc(cap * sizeof *q);
        if (!q) {
            free(msg.data);
            return DT_ERR_NOMEM;
        }
        for (kk = 0; kk < t->n; kk++)
            q[kk] = t->q[(t->head + kk) % t->cap];
        free(t->q);
        t->q = q;
        t->cap = cap;
        t->head = 0;
    }
    t->q[(t->head + t->n) % t->cap] = msg;
    t->n++;
    t->hint = 1;
    return DT_OK;
}

size_t dt_engine_rpc_replies(dt_engine *e, uint8_t *buf, size_t max)
{
    size_t n = e->tr.last_reply.n < max ? e->tr.last_reply.n : max;
    if (buf && n)
        memcpy(buf, e->tr.last_reply.p, n);
    if (n == e->tr.last_reply.n)
        e->tr.last_reply.n = 0;
    else
        e_buf_drop(&e->tr.last_reply, n);
    return n;
}

/* ------------------------------------------------------------ uscite */
size_t dt_engine_midi_out(dt_engine *e, uint8_t *buf, size_t max)
{
    size_t n = e->midi.host_out.n < max ? e->midi.host_out.n : max;
    if (buf && n)
        memcpy(buf, e->midi.host_out.p, n);
    e_buf_drop(&e->midi.host_out, buf ? n : e->midi.host_out.n);
    return n;
}

void dt_engine_set_audio_sink(dt_engine *e, void (*sink)(void *ctx, const float *mix, const float *tracks,
                                                         size_t n),
                              void *ctx, int tracks)
{
    (void)tracks;
    e->au.sink = sink;
    e->au.sink_ctx = ctx;
}

/* Main Volume: audio.MASTER (0..0.5, 0.25 al centro); l'uscita dal vivo e' a
   guadagno MASTER / MASTER_FULL (emu/audio.py DspClockSink) */
void dt_engine_set_master(dt_engine *e, double master)
{
    e->au.gain = master / 0.5;
}

int64_t dt_engine_audio_samples(dt_engine *e)
{
    dsp_flush(e);
    return e->au.samples;
}

uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64])
{
    if (out)
        memcpy(out, e->fb, sizeof e->fb);
    return e->panel_version;
}

/* la copia dei LED per la GUI del plugin (dal thread del motore) */
static void leds_publish(dt_engine *e)
{
    atomic_fetch_add_explicit(&e->leds_seq, 1, memory_order_acq_rel);
    memcpy(e->leds_pub, e->leds.values, DN_NLEDS);
    atomic_fetch_add_explicit(&e->leds_seq, 1, memory_order_acq_rel);
}

/* tutti e 53 i LED, da qualunque thread (rt.c ne pubblica 50) */
uint64_t dn_rt_leds(dt_rt *rt, uint8_t out[DN_LEDS])
{
    dt_engine *e = dt_rt_engine(rt);
    uint64_t a, b;
    if (!e)
        return 0;
    do {
        a = atomic_load_explicit(&e->leds_seq, memory_order_acquire);
        memcpy(out, e->leds_pub, DN_LEDS);
        b = atomic_load_explicit(&e->leds_seq, memory_order_acquire);
    } while (a != b || (a & 1));
    return (uint64_t)e->leds.version;
}

/* byte MIDI verso il MIDI IN emulato da un thread qualunque (non quello audio): la
   coda delle richieste RPC di rt.c, che il motore svuota fra un giro e l'altro */
int dn_rt_midi_inject(dt_rt *rt, const uint8_t *data, size_t n)
{
    uint8_t id = 0, *rep = NULL;
    size_t rn = 0;
    int r, k;
    /* timeout 0: si accoda e basta (-1 = accodata, senza risposta); con la coda piena
       (DT_ERR_NOMEM, 64 richieste) si riprova finche' il motore la svuota, fino a 2 s */
    for (k = 0; k < 400; k++) {
        r = dt_rt_rpc(rt, DN_RPC_MIDI, data, n, &id, &rep, &rn, 0);
        free(rep);
        rep = NULL;
        if (r != DT_ERR_NOMEM)
            return DT_OK;
        {
            struct timespec ts = {0, 5000000};
            nanosleep(&ts, NULL);
        }
    }
    return DT_ERR_NOMEM;
}

/* rt.c chiede 50 LED (il Digitakt): dn_engine_leds li da' tutti e 53 */
uint64_t dt_engine_leds(dt_engine *e, uint8_t out[50])
{
    if (out)
        memcpy(out, e->leds.values, 50);
    return (uint64_t)e->leds.version;
}

uint64_t dn_engine_leds(dt_engine *e, uint8_t out[DN_LEDS])
{
    if (out)
        memcpy(out, e->leds.values, DN_LEDS);
    return (uint64_t)e->leds.version;
}

int64_t dt_engine_instrs(dt_engine *e)
{
    return e->instrs;
}

const char *dt_engine_status(dt_engine *e)
{
    return e->status;
}

/* la riga 'chunk' di tools/dncore_guiref.py */
int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n)
{
    int64_t frc = 0;
    int kk;
    dsp_flush(e);
    for (kk = 0; kk < 192; kk++)
        frc += e->frc.delivered[kk];
    return snprintf(buf, n,
                    "%s %lld instrs=%lld pc=%08x pit=%lld/%lld/%lld:%lld/%lld/%lld dtim3=%lld:%lld "
                    "ssi=%lld/%lld/%lld/%lld frc=%lld/%lld idle=%lld sw=%lld sat=%lld uart=%llu:%08x "
                    "midi=%lld/%lld/%lld d2=%lld:%08x audio=%lld:%08x fb=%lld:%08x leds=%lld:%08x sd=%lld/%lld "
                    "d1=%lld:%08x/%d tr=%lld/%lld stat=%s",
                    tag, (long long)chunk, (long long)e->instrs, e_rd(e, UC_M68K_REG_PC),
                    (long long)e->tim[0].t.fired[0], (long long)e->tim[0].t.fired[2],
                    (long long)e->tim[0].t.fired[3], (long long)e->tim[0].t.missed[0],
                    (long long)e->tim[0].t.missed[2], (long long)e->tim[0].t.missed[3],
                    (long long)e->tim[1].t.fired[3], (long long)e->tim[1].t.missed[3],
                    (long long)e->sst.requests, (long long)e->sst.vector170, (long long)e->sst.vector191,
                    (long long)e->sst.rx_markers, (long long)frc, (long long)e->frc.at_rte,
                    (long long)e->idle.n, 0LL, (long long)e->satisfied,
                    (unsigned long long)e->uart_out_total, e->uart_out_crc, (long long)e->midi.tx_sent,
                    (long long)e->midi.rx_bytes, (long long)e->midi.uart_irqs, (long long)e->dspi2_frames,
                    e->d2_crc, (long long)e->au.samples, e->au.crc, (long long)e->frames, e->fb_crc,
                    (long long)e->leds.version, e_crc(0, e->leds.values, DN_NLEDS), (long long)e->sd.log_n,
                    (long long)e->sd.dma_bytes, (long long)e->d1.frames, e->d1.last_crc,
                    (int)e->d1.pending_irq, (long long)e->tr.sent, (long long)e->tr.replies,
                    e->halted ? "halted" : "ok");
}

dt_machine *dt_engine_machine(dt_engine *e)
{
    return e->m;
}

/* SHA-256 di tutte le regioni mappate in ordine di indirizzo, lette come l'host del
   Python (le pagine del bus dallo store): la riga 'ram' di tools/dncore_guiref.py */
int dn_engine_digest(dt_engine *e, uint8_t out[32], uint32_t *nregions)
{
    uc_mem_region *regs = NULL;
    uint32_t n = 0, i;
    dt_sha256 h;
    uint8_t *buf;
    dsp_flush(e);
    if (uc_mem_regions(e->uc, &regs, &n) != UC_ERR_OK)
        return -1;
    buf = (uint8_t *)malloc(PAGE);
    if (!buf) {
        uc_free(regs);
        return -1;
    }
    /* uc_mem_regions e' gia' in ordine di indirizzo */
    dt_sha256_init(&h);
    for (i = 0; i < n; i++) {
        uint64_t a = regs[i].begin, end = regs[i].end + 1;
        while (a < end) {
            size_t k = (size_t)(end - a < PAGE ? end - a : PAGE);
            if (e_read(e, a, buf, k)) {
                free(buf);
                uc_free(regs);
                return -1;
            }
            dt_sha256_update(&h, buf, k);
            a += k;
        }
    }
    dt_sha256_final(&h, out);
    *nregions = n;
    free(buf);
    uc_free(regs);
    return 0;
}

int64_t dn_engine_dsp_frames(dt_engine *e)
{
    return e->au.frames;
}

/* il boot freddo dal reset (docs/COLDBOOT.md) */
#include "dn_boot.inc.c"
