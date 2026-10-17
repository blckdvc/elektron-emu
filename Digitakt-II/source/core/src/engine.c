/* Il motore della GUI v2 con --audio (emu/gui.py Emulator + longrun.build +
 * i modelli), in C, senza Python. Piano e verifiche: docs/AU-PORT.md.
 *
 * Una sola unita' di traduzione: il C nativo che usa anche il Python
 * (native/dt2_hooks.c, native/dt2_spin.c, lo stesso testo che
 * emu/nativehooks.py compila) e poi le parti e_*.inc.c. Gli hook si registrano
 * nell'ordine del Python (l'ordine conta dove due hook coprono lo stesso
 * indirizzo: SERQ, USR8, gli RTE); il riferimento e' tools/dtcore_guiref.py e
 * `dtcore play` deve dare la sua stessa traccia. */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

#include "native/dt2_hooks.c"
#include "native/dt2_spin.c"

#include "dtcore.h"
#include "internal.h"
#include "machine.h"
#include "sha256.h"

#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "machines/digitakt2/profiles.inc.c"
typedef struct dt_engine dt_engine;
#include "e_base.inc.c"
#include "e_build.inc.c"
#include "e_timers.inc.c"
#include "e_exact.inc.c"
#include "e_periph.inc.c"
#include "machines/digitakt2/dt2_dspi.inc.c"
#include "machines/digitakt2/dt2_audio.inc.c"
#include "e_tanh.inc.c"
#include "e_panel.inc.c"
#include "e_state.inc.c"
#include "e_boot.inc.c"

#define E_BUDGET 400000
#define E_PER_BLOCK 3.87
#define E_IDLE_YIELD 20000
#define E_SSI_HZ 96000
#define E_AUDIO_IPS 150000000

/* ---------------------------------------------------- componenti DTST */
typedef struct {
    dt_engine *e;
    int have_ssi, have_etx, have_tim, have_smp, have_sd;
    uint8_t *ctim;
    uint64_t ctim_len;
} e_load;

static int comp_record(void *ctx, const char tag[4], const uint8_t *d, uint64_t n, char *err, size_t errlen)
{
    e_load *L = (e_load *)ctx;
    dt_engine *e = L->e;
    if (!memcmp(tag, "CUIN", 4)) {
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
        if (n != 40 + 4 + 4 + 48 + 4 + 16) {
            dt_errf(err, errlen, "bad ssi0_dma component (%llu bytes)", (unsigned long long)n);
            return DT_ERR_FORMAT;
        }
        rh = (int64_t)e_le64(d);
        if (rh != E_SSI_HZ) {
            dt_errf(err, errlen, "Ssi0Dma request-rate mismatch");
            return DT_ERR_FORMAT;
        }
        e->sst.ips = (int64_t)e_le64(d + 8);
        e->sst.now = (int64_t)e_le64(d + 16);
        num = (int64_t)e_le64(d + 24);
        den = (int64_t)e_le64(d + 32);
        if (den) {
            __int128 v = (__int128)num * rh;
            if (v % den) {
                dt_errf(err, errlen, "SSI deadline is not a whole number of requests");
                return DT_ERR_FORMAT;
            }
            e->sst.nextN = (int64_t)(v / den);
            e->sst.has_next = 1;
        } else
            e->sst.has_next = 0;
        e->sst.en = (int32_t)(e_le32(d + 40) & 3);
        e->sst.int50_asserted = d[44];
        e->sst.int50_delivered = d[45];
        e->sst.force_asserted = d[46];
        e->sst.force_delivered = d[47];
        e->sst.requests = (int64_t)e_le64(d + 48);
        e->sst.major_rx = (int64_t)e_le64(d + 56);
        e->sst.major_tx = (int64_t)e_le64(d + 64);
        e->sst.sg_rx = (int64_t)e_le64(d + 72);
        e->sst.sg_tx = (int64_t)e_le64(d + 80);
        e->sst.tx_bytes = (int64_t)e_le64(d + 88);
        e->sst.tx_crc32 = e_le32(d + 96);
        e->sst.vector170 = (int64_t)e_le64(d + 100);
        e->sst.vector191 = (int64_t)e_le64(d + 108);
        L->have_ssi = 1;
    } else if (!memcmp(tag, "CSDH", 4)) {
        e_esdhc *sd = &e->sd;
        uint64_t off = 0, i, cnt;
        if (n < 33) {
            dt_errf(err, errlen, "bad esdhc component");
            return DT_ERR_FORMAT;
        }
        sd->pattern = e_le32(d);
        sd->armed = (int32_t)e_le32(d + 4);
        if (sd->armed != SD_DMA_CHAN)
            sd->armed = -1;
        sd->dma_bytes = (int64_t)e_le64(d + 8);
        if ((int64_t)e_le64(d + 16) == SD_DT2_BLOCKS && sd->blocks != SD_DT2_BLOCKS)
            sd_identity_dt2(sd);                 /* uno stato nato con DT_COLDBOOT_EMMC_DT2 */
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
    } else if (!memcmp(tag, "CSMP", 4)) {
        uint64_t off = 4, i, np = n >= 4 ? e_le32(d) : 0, ns;
        for (i = 0; i < np; i++) {
            e_page *pg;
            if (off + 4 + AU_PAGE > n)
                goto bad_smp;
            pg = au_page_put(&e->au, e_le32(d + off));
            if (!pg)
                return DT_ERR_NOMEM;
            memcpy(pg->data, d + off + 4, AU_PAGE);
            off += 4 + AU_PAGE;
        }
        if (off + 4 > n)
            goto bad_smp;
        ns = e_le32(d + off);
        off += 4;
        for (i = 0; i < ns; i++) {
            uint8_t blk[20];
            if (off + 20 > n)
                goto bad_smp;
            /* SampleMemory.slots[slot] = (left, right, nbytes, nsamp) */
            e_put32(blk, e_le32(d + off));
            e_put32(blk + 4, e_le32(d + off + 4));
            e_put32(blk + 8, e_le32(d + off + 8));
            e_put32(blk + 12, e_le32(d + off + 12));
            e_put32(blk + 16, e_le32(d + off + 16));
            {
                int64_t blocks = e->au.blocks;
                au_on_block(&e->au, AU_CMD, blk);
                e->au.blocks = blocks;
            }
            off += 20;
        }
        L->have_smp = 1;
        return 0;
    bad_smp:
        dt_errf(err, errlen, "truncated dsp_samples component");
        return DT_ERR_FORMAT;
    } else if (!memcmp(tag, "CTIM", 4)) {
        L->ctim = (uint8_t *)malloc((size_t)n);
        if (!L->ctim)
            return DT_ERR_NOMEM;
        memcpy(L->ctim, d, (size_t)n);
        L->ctim_len = n;
        L->have_tim = 1;
    }
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
            s->t.has[c] = q[0] == 1 || q[0] == 2;
            s->frac[c] = q[0] == 2;                  /* Fraction: n/d nell'estensione */
            if (s->t.has[c])
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
    /* estensione: per ogni scadenza in Fraction (sorgente, canale in ordine)
       numeratore e denominatore, i128 little-endian */
    for (i = 0; i < ns; i++) {
        int c, k;
        for (c = 0; c < 4; c++) {
            e_tsrc *s = &e->tim[i];
            unsigned __int128 v[2] = {0, 0};
            if (!s->frac[c])
                continue;
            if (off + 32 > n)
                return -1;
            for (k = 0; k < 32; k++)
                v[k / 16] |= (unsigned __int128)d[off + k] << (8 * (k % 16));
            s->fn[c] = (__int128)v[0];
            s->fd[c] = (__int128)v[1];
            if (s->fd[c] <= 0)
                return -1;
            off += 32;
        }
    }
    e->ntim = 2;
    return 0;
}

/* ---------------------------------------------------------- idle slow */
static void idle_slow_c(int64_t n)
{
    dt_engine *e = e_cur;
    tx_deliver(e, &e->etx);
    if (n % E_IDLE_YIELD == 0)
        dtm_raise_vector(e->m, 32, 0, -1);
}

/* ----------------------------------------------------------- SSI SERQ */
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
    if (e->sst.en && !e->sst.has_next) {          /* next = now + Fraction(ips, rh) */
        e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
        e->sst.has_next = 1;
    }
}

static int verify(dt_engine *e, uint32_t addr, const uint8_t *want, size_t n)
{
    uint8_t b[64];
    return n <= sizeof b && !e_read(e, addr, b, n) && !memcmp(b, want, n);
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
    int ssi_legacy = 0, intro = 0;
    const uint8_t *img;
    size_t img_len;
    int r;

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

    /* dspboot.build_flash: 16 MB, container a 0x80000 */
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000) {
        dt_errf(err, errlen, "cannot build the flash image");
        goto fail;
    }
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);

    /* Machine(): MEM_INVALID; install_isa_patches_scoped (MOVEC); l'INTR */
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
    e->exc.fallback = intr_fallback_c;
    e->q.exc = &e->exc;
    e->q.py_raise = py_raise_c;
    e->stp.stop = stop_c;

    /* ---- longrun.build ---- */
    e_hook_code(e, (void *)h_flash_read, e, e->p->flash_read);
    e_hook_code(e, (void *)h_pend_call, e, e->p->pend_call);
    if (e->p->task_create)
        e_hook_code(e, (void *)h_task_create, e, e->p->task_create);
    e_hook_code(e, (void *)h_print, e, E_PRINT);
    if (e->p->set_pixel)
        e_hook_code(e, (void *)h_setpixel_count, e, e->p->set_pixel);
    if (e->p->px_copy)
        e_hook_code(e, (void *)h_pxcopy_count, e, e->p->px_copy);
    /* switch_to: in C alla fine (nativehooks.replace_switch_to) */
    e_hook_code(e, (void *)h_depack_clamp, e, e->p->depack_copy);
    /* edma */
    tx_init(&e->etx, 35, 155, &e->uart_out, &e->uart_out_total, &e->uart_out_crc);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_edma_serq, e, E_SERQ, E_SERQ);
    e_hook_code(e, (void *)h_edma_wait, e, e->p->uart8_tx_wait);
    /* ssi */
    if (!e->p->ssi0_dma_force_rte) {
        dt_errf(err, errlen, "SSI0 DMA model requires the force-ISR RTE symbol");
        goto fail;
    }
    e->sst.mem_read = (mem_rw_fn)uc_mem_read;
    e->sst.mem_write = (mem_rw_fn)uc_mem_write;
    e->sst.rh = E_SSI_HZ;
    e->sst.ips = 4680000;                     /* emu.pit.INSTR_PER_SEC, poi il checkpoint */
    e->sst.rx_ch = 48;
    e->sst.tx_ch = 50;
    e->sst.level_retry = 1;
    e->sst.slack = 256;
    e->sst.level_retry_step = 8000;
    e->sst.txbuf = NULL;
    e->sst.txcap = 1 << 20;
    e->sst.q = &e->q;
    e->sst.tx_vec = 170;
    e->sst.force_vec = 191;
    e->sst.rx_peer = 1;
    e->sst.nbanks = 2;
    e->sst.bank0 = 0x4FE57100u;
    e->sst.bank_bytes = 0x800;
    e->zt = (uint64_t *)calloc(1 << 14, sizeof *e->zt);
    if (!e->zt)
        goto nomem;
    e->sst.zt = e->zt;
    e->sst.zcap = 1 << 14;
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_ssi_serq, e, E_SERQ, E_SERQ);
    /* CINT e INTFRCH1: in C alla fine (nativespin._own_ssi) */
    e->ssi_gate.open = 1;
    e->ssi_gate.cfn = dt2_sst_force_rte;
    e->ssi_gate.cdata = &e->sst;
    e_hook_code(e, (void *)dt2_gate_code, &e->ssi_gate, e->p->ssi0_dma_force_rte);
    /* dsp (fast): la lettura di STATUS in C alla fine */
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
    /* idle spin: in C alla fine */
    for (off = 0; off + 1 < img_len; off += 2)
        if (img[off] == 0x60 && img[off + 1] == 0xFE && e->nidle < 32)
            e->idle_addrs[e->nidle++] = DT_MAIN_LOAD + (uint32_t)off;
    /* softfloat, nell'ordine di ROUTINES */
    {
        const uint32_t addrs[7] = {e->p->sf_mulsf3, e->p->sf_subsf3, e->p->sf_addsf3, e->p->sf_divsf3,
                                   e->p->sf_abssf2, e->p->sf_fixsfsi, e->p->sf_cmpsf2};
        int k;
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
    /* unblock */
    if (e->p->display_sem)
        e_skip_add(e, e->p->display_sem);
    if (e->p->sd_cmd_sem)
        e_skip_add(e, e->p->sd_cmd_sem);
    if (e->p->sd_data_sem)
        e_skip_add(e, e->p->sd_data_sem);
    if (e->p->sd_dma_sem)
        e_skip_add(e, e->p->sd_dma_sem);
    {
        const uint32_t rc[5] = {e->p->queue_recv, e->p->intro_park, e->p->display_wait, e->p->pump_wait,
                                e->p->tick_pend};
        int k;
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
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_uart8_read, e, E_USR8, E_UDR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_uart8_write, e, E_USR8, E_UDR8 + 3);
    dtm_mmio_set(e->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(e->m, 0xEC03802Cu, 0x80000000u);
    /* esdhc (prima del ripristino) */
    sd_init(e);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_serq, e, E_SERQ, E_SERQ);

    /* ---- restore_into ---- */
    memset(&L, 0, sizeof L);
    L.e = e;
    r = state ? dtm_load_state(e->m, state, comp_record, &L, err, errlen)
              : dtm_load_state_mem(e->m, state_buf, state_len, comp_record, &L, err, errlen);
    if (r) {
        free(L.ctim);
        goto fail;
    }
    if (!L.have_ssi) {
        /* gui.py con --audio su uno stato senza il componente: prima
           l'aggiornamento "legacy" (Ssi0Dma.arm_legacy: TCD48/50 gia'
           programmati dal firmware, come a boot400M) */
        uint8_t tcd[2][0x20];
        int k, legacy = 1;
        if (e_read(e, 0xFC045000u + 48u * 0x20u, tcd[0], 0x20)
            || e_read(e, 0xFC045000u + 50u * 0x20u, tcd[1], 0x20))
            legacy = 0;
        for (k = 0; k < 2 && legacy; k++) {
            const uint8_t *t = tcd[k];
            uint32_t periph = k ? e_be32(t + 16) : e_be32(t);
            uint16_t attr = (uint16_t)(t[4] << 8 | t[5]), csr = (uint16_t)(t[30] << 8 | t[31]);
            int16_t soff = (int16_t)(t[6] << 8 | t[7]), doff = (int16_t)(t[22] << 8 | t[23]);
            uint32_t nbytes = e_be32(t + 8);
            uint16_t citer = (uint16_t)(t[20] << 8 | t[21]), biter = (uint16_t)(t[28] << 8 | t[29]);
            if (k == 0 ? !(periph == 0xFC0BC008u && attr == 0x0202 && soff == 0 && nbytes == 0x20
                           && doff == 4 && (csr & 0x12) == 0x10)
                       : !(periph == 0xFC0BC000u && attr == 0x0202 && soff == 4 && nbytes == 0x20
                           && doff == 0 && (csr & 0x12) == 0x12))
                legacy = 0;
            if (!citer || !biter)
                legacy = 0;
        }
        if (legacy) {
            e->sst.en = 3;                         /* enabled.update((48, 50)) */
            e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
            e->sst.has_next = 1;
            ssi_legacy = 1;
        } else {
            /* se fallisce, quello di un gradino prima dell'SSI0 (boot60M:
               Ssi0Dma.arm_cold): TCD vuoti, i canali li accendono le SERQ */
            int b;
            for (k = 0; k < 2; k++)
                for (b = 0; b < 0x20; b++)
                    if (tcd[k][b]) {
                        dt_errf(err, errlen, "SSI0 DMA topology is absent; use an explicit legacy upgrade");
                        free(L.ctim);
                        goto fail;
                    }
            e->sst.en = 0;
            e->sst.has_next = 0;
            ssi_legacy = 2;                         /* non ripristinato, spento */
        }
    }

    /* DT2_DSP_PEER: dspi2 + audio */
    dtm_mmio_set(e->m, D2_SR, D2_TCF | D2_EOQF);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_d2_sr, e, D2_SR, D2_SR + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_d2_serq, e, E_SERQ, E_SERQ);
    if (!verify(e, D2_SEND, D2_SEND_VERIFY, sizeof D2_SEND_VERIFY)) {
        dt_errf(err, errlen, "DSPI2 send routine not where expected");
        goto fail;
    }
    if (!verify(e, AU_BLOCK_SEND, AU_BLOCK_VERIFY, sizeof AU_BLOCK_VERIFY)) {
        dt_errf(err, errlen, "DSP block-send routine not where expected");
        goto fail;
    }
    e_hook_code(e, (void *)h_audio_block_send, e, AU_BLOCK_SEND);
    e->au.kick = au_kick();
    if (!e->au.kick)
        goto nomem;
    e->au.master = 0.25f;
    /* intfrc: gli hook di scrittura in C alla fine; gli RTE ora */
    e->frc.q = &e->q;
    e->frc.retry = 8000;
    e->frc.skip[1] |= 1ull << 63;                 /* vettore 191 (emu/ssi.py) */
    e->frc.dirty = 1;
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
    if (verify(e, MI_RX_HANDLER, MI_RX_HANDLER_VERIFY, sizeof MI_RX_HANDLER_VERIFY)) {
        e_midi *mi = &e->midi;
        mi->installed = 1;
        tx_init(&mi->tx, 37, 157, &mi->tx_out, &mi->tx_out_total, &mi->tx_out_crc);
        mi->hint = 1;
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_serq, e, E_SERQ, E_SERQ);
        mi->uimr = e_r8(e, MI_UIMR9);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_udr, e, MI_UDR9, MI_UDR9);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_uimr, e, MI_UIMR9, MI_UIMR9);
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_mi_usr, e, MI_USR9, MI_USR9);
        mi->uart_masked = 1;
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_imr, e, MI_SIMR1, MI_CIMR1);
        if (verify(e, MI_ROUTER_PEND, MI_ROUTER_VERIFY, sizeof MI_ROUTER_VERIFY))
            e_skip_add(e, MI_ROUTER_SEM);
    }
    /* dspi1 */
    if (verify(e, D1_SEND, D1_SEND_VERIFY, sizeof D1_SEND_VERIFY)) {
        e->d1.installed = 1;
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_d1_serq, e, E_SERQ, E_SERQ);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_d1_pin, e, D1_PIN_CLR, D1_PIN_CLR);
        if (verify(e, D1_REPLY_VERIFY_AT, D1_REPLY_VERIFY, sizeof D1_REPLY_VERIFY))
            e_skip_add(e, D1_REPLY_SEM);
    }
    /* install_mmio */
    dtm_install_mmio(e->m);
    e->dspi2_sr = dtm_mmio_ref(e->m, D2_SR);
    /* edma: kick solo senza stato del modello */
    if (!e->etx.restored && e->p->uart8_tx_state && e_r32(e, e->p->uart8_tx_state))
        tx_run(e, &e->etx);

    /* ---- Emulator.run ---- */
    if (verify(e, PN_SET_LED, PN_SET_LED_VERIFY, sizeof PN_SET_LED_VERIFY)) {
        e_read(e, PN_LED_TABLE, e->leds.values, 50);
        e->leds.version++;
    }
    if (verify(e, TR_USB_SEND, TR_USB_SEND_VERIFY, sizeof TR_USB_SEND_VERIFY)
        && verify(e, TR_ENQUEUE, TR_ENQUEUE_BYTES, sizeof TR_ENQUEUE_BYTES)) {
        static const uint8_t nops[16] = {0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71,
                                         0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71, 0x4e, 0x71};
        e->tr.installed = 1;
        dtm_ensure(e->m, TR_PAGE);
        dtm_ensure(e->m, TR_PAGE);
        e_write(e, TR_PAGE + 0x800, nops, sizeof nops);
        e_hook_code(e, (void *)h_tr_usb_send, e, TR_USB_SEND);
    }
    intro = e_intro_running(e);
    /* timers: restore_checkpoint_timers, o Timers(Pits(m, hold=intro),
       Dtims(m, channels=(3,), hold=intro)) nuovi (gradino del boot freddo) */
    if (L.have_tim) {
        if (timers_restore(e, L.ctim, L.ctim_len)) {
            dt_errf(err, errlen, "bad timers component");
            free(L.ctim);
            goto fail;
        }
    } else {
        static const uint32_t pch[3] = {3, 2, 0}, dch[1] = {3};
        double p;
        int r;
        tsrc_init(e, &e->tim[0], 0, 3, pch, 4680000, intro);
        tsrc_init(e, &e->tim[1], 1, 1, dch, 4680000, intro);
        e->ntim = 2;
        r = tim_period(e->uc, &e->tim[1].t, 3, &p);            /* Dtims.clear_stale */
        if (r < 0) {
            dt_errf(err, errlen, "cannot read DTIM3");
            goto fail;
        }
        if (r > 0) {
            e_w16(e, DTIM_BASES[3], 0);
            e->tim[1].stale |= 1u << 3;
        }
    }
    free(L.ctim);
    L.ctim = NULL;
    /* Dtims.__init__: l'hook `arm` e dtim.install_counters */
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dtim_arm, &e->tim[1], DTIM_BASES[3], DTIM_BASES[3] + 0x0F);
    if (e_r16(e, DTIM_BASES[0]) == 0) {
        e_w16(e, DTIM_BASES[0], 0x0003);
        e->dtcn_has[0] = 1;
        e->dtcn_origin[0] = 0;
    }
    for (i = 0; i < 4; i++) {
        e->dtcn_ud[i].e = e;
        e->dtcn_ud[i].ch = (int)i;
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_dtcn_read, &e->dtcn_ud[i], DTIM_BASES[i] + 0x0C,
                   DTIM_BASES[i] + 0x0F);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dtcn_write, &e->dtcn_ud[i], DTIM_BASES[i],
                   DTIM_BASES[i] + 0x0F);
    }
    if (ssi_legacy)                              /* non ripristinato: ssi0.ips = pits.sources[0].ips */
        e->sst.ips = e->tim[0].ips;
    e->sst.now = e->tim[0].now;                  /* ssi0.align(pits.now) */
    if (ssi_legacy == 1) {                       /* acceso e non ripristinato: next = now + period */
        e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
        e->sst.has_next = 1;
    }
    if (!intro && (e->tim[0].t.held || e->tim[1].t.held)) {
        /* timer trattenuti su una base gia' dopo l'intro: rilasciati */
        if (e->tim[0].t.nch == 1 && e->tim[0].t.ch[0] == 3) {
            e->tim[0].t.nch = 3;
            e->tim[0].t.ch[1] = 2;
            e->tim[0].t.ch[2] = 0;
        }
        e->tim[0].t.held = e->tim[1].t.held = 0;
    }
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
    e_hook_code(e, (void *)h_terminal, e, PN_TERMINAL);
    if (e->p->panel_diff && e->p->fb_front)
        e_hook_code(e, (void *)h_panel_latch, e, e->p->panel_diff);
    /* nativehooks.install: lo stepper (tb-budget) */
    if (uc_set_block_budget(e->uc, (int64_t *)&e->stp) != UC_ERR_OK) {
        dt_errf(err, errlen, "uc_set_block_budget failed (Unicorn without the tb-budget patch)");
        goto fail;
    }
    /* replace_dsp_read */
    e->dspw.write = (memw_fn)uc_mem_write;
    e->dspw.address = 0x8C000002u;
    e->dspw.size = 2;
    e->dspw.bytes[0] = 0x00;
    e->dspw.bytes[1] = 0x01;
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)dt2_mem_write_const, &e->dspw, 0x8C000002u, 0x8C000003u);
    /* replace_idle_spins */
    e->idle.n = 0;
    e->idle.yield_every = E_IDLE_YIELD;
    e->idle.tx_pending = &e->etx.pending;
    e->idle.slow = idle_slow_c;
    e->idle.step = &e->stp;
    e->idle.exc = &e->exc;
    for (i = 0; i < (size_t)e->nidle; i++)
        e_hook_code(e, (void *)dt2_on_idle_c, &e->idle, e->idle_addrs[i]);
    /* replace_switch_to */
    e->sw.reg_read = (reg_rw_fn)uc_reg_read;
    e->sw.r_a0 = UC_M68K_REG_A0;
    e_hook_code(e, (void *)dt2_on_switch, &e->sw, E_SWITCH_TO);
    /* replace_dspi2_hle */
    e->hle.reg_read = (reg_rw_fn)uc_reg_read;
    e->hle.reg_write = (reg_rw_fn)uc_reg_write;
    e->hle.mem_read = (mem_rw_fn)uc_mem_read;
    e->hle.mem_write = (mem_rw_fn)uc_mem_write;
    e->hle.r_a7 = UC_M68K_REG_A7;
    e->hle.r_pc = UC_M68K_REG_PC;
    e->hle.frame_len_var = 0x42948B08u;
    e->hle.pushr_area = 0x80001BC0u;
    e->hle.rx_area = 0x80001000u;
    e->hle.max_bytes = 0xBC0;
    e->hle.py = hle_py_c;
    e_hook_code(e, (void *)dt2_dspi2_hle, &e->hle, D2_SEND);
    /* nativespin.install: SSI e interrupt forzati posseduti dal C */
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_cint, &e->sst, 0xFC04401Cu, 0xFC04401Cu);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_intfrch1, &e->sst, 0xFC04C010u, 0xFC04C013u);
    {
        static const uint32_t ib[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
        int k;
        for (k = 0; k < 3; k++)
            e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_frc_on_write, &e->frc, ib[k] + 0x10, ib[k] + 0x17);
    }
    e->S.emu_start = (emu_start_fn)uc_emu_start;
    e->S.q = &e->q;
    e->S.stp = &e->stp;
    e->S.per_block = E_PER_BLOCK;
    e->S.ntim = 2;
    e->S.tim[0] = &e->tim[0].t;
    e->S.tim[1] = &e->tim[1].t;
    e->S.sst = &e->sst;
    e->S.frc = &e->frc;
    e->S.nev = 0;
    e->S.ev[e->S.nev++] = EV_SSI;
    e->S.ev[e->S.nev++] = EV_FRC;
    if (e->midi.installed) {
        int k = e->S.nev++;
        e->S.ev[k] = EV_PY;
        e->S.hint[k] = &e->midi.hint;
        e->S.hint2[k] = &e->midi.tx.pending;
        e->S.pstep[k] = mi_step;
        e->S.pservice[k] = mi_service;
    }
    if (e->d1.installed) {
        int k = e->S.nev++;
        e->S.ev[k] = EV_PY;
        e->S.hint[k] = &e->d1.hint;
        e->S.pstep[k] = d1_step;
        e->S.pservice[k] = d1_service;
    }
    if (e->tr.installed) {
        int k = e->S.nev++;
        e->S.ev[k] = EV_PY;
        e->S.hint[k] = &e->tr.hint;
        e->S.pstep[k] = tr_step;
        e->S.pservice[k] = tr_service;
    }
    e->S.stop_flag = &e->m->stop_flag;
    e->S.tx_flush = 1 << 19;
    e->pc = e_rd(e, UC_M68K_REG_PC);
    snprintf(e->status, sizeof e->status, "running");
    return e;
nomem:
    dt_errf(err, errlen, "out of memory");
fail:
    dt_engine_free(e);
    return NULL;
}

void dt_engine_free(dt_engine *e)
{
    size_t i;
    if (!e)
        return;
    if (e->m)
        dt_machine_free(e->m);
    free(e->flash);
    free(e->zt);
    free(e->uart_out.p);
    free(e->uart_in.p);
    ov_free(&e->sd.ov);
    audio_free(&e->au);
    free(e->midi.rx.p);
    free(e->midi.tx_out.p);
    free(e->midi.host_out.p);
    for (i = 0; i < e->tr.n; i++)
        free(e->tr.q[(e->tr.head + i) % e->tr.cap].data);
    free(e->tr.q);
    free(e->tr.last_reply.p);
    free(e->sw.seq);
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
    exact = ex_needed(e);
    for (;;) {
        code = exact ? ex_spin(e, e->uc, S) : dt2_spin(e->uc, S);
        if (code == SPIN_LIMIT)
            break;
        if (code == SPIN_FLUSH)
            continue;
        if (code == SPIN_PC_ZERO) {
            snprintf(e->status, sizeof e->status, "halted: pc zero");
            break;
        }
        if (code == SPIN_UC_ERROR) {
            snprintf(e->status, sizeof e->status, "halted: %s", uc_strerror((uc_err)S->uc_err));
            break;
        }
        if (code == SPIN_STOPPED) {
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

/* Un giro del ciclo del worker della GUI (Emulator.run). -> 0, 1 fermato */
int dt_engine_chunk(dt_engine *e)
{
    int64_t executed = 0;
    e_cur = e;
    if (e->halted)
        return 1;
    if (e->has_pending_ips && e->pending_ips_at <= e->instrs) {
        int k;
        e->has_pending_ips = 0;
        for (k = 0; k < e->ntim; k++)
            tsrc_set_ips(e, &e->tim[k], e->pending_ips_n);
        e->sst.ips = e->pending_ips_n;
        e->midi.ips = e->pending_ips_n;
    }
    e->pc = panel_drain(e, e->pc);
    if (engine_spin(e, E_BUDGET, &executed)) {
        e->halted = 1;
        return 1;
    }
    e->instrs += executed;
    panel_publish(e);
    if (e->uart_out.n) {
        leds_feed(&e->leds, e->uart_out.p, e->uart_out.n);
        e->uart_out.n = 0;
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
    if (!t->installed)
        return DT_ERR_UNSUPPORTED;
    msg.data = (uint8_t *)malloc(n ? n : 1);
    if (!msg.data)
        return DT_ERR_NOMEM;
    memcpy(msg.data, payload, n);
    msg.n = n;
    if (t->n == t->cap) {
        size_t cap = t->cap ? 2 * t->cap : 16, k;
        e_msg *q = (e_msg *)malloc(cap * sizeof *q);
        if (!q) {
            free(msg.data);
            return DT_ERR_NOMEM;
        }
        for (k = 0; k < t->n; k++)
            q[k] = t->q[(t->head + k) % t->cap];
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

/* le risposte del firmware alle richieste RPC (USB_SEND), dall'ultima chiamata:
   [u32 n, host][n byte di frame] ... ; -> byte copiati, svuota la coda */
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
    e->au.sink = sink;
    e->au.sink_ctx = ctx;
    e->au.tracks_on = tracks;
}

void dt_engine_set_master(dt_engine *e, double master)
{
    e->au.master = (float)master;                 /* audio.MASTER: float32 nel numpy */
}

int64_t dt_engine_audio_samples(dt_engine *e)
{
    return e->au.samples;
}

uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64])
{
    if (out)
        memcpy(out, e->fb, sizeof e->fb);
    return e->panel_version;
}

uint64_t dt_engine_leds(dt_engine *e, uint8_t out[50])
{
    if (out)
        memcpy(out, e->leds.values, 50);
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

/* la riga 'chunk' di tools/dtcore_guiref.py */
int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n)
{
    int64_t frc = 0, sw = 0;
    int k;
    for (k = 0; k < 192; k++)
        frc += e->frc.delivered[k];
    for (k = 0; k < e->sw.n; k++)
        sw += e->sw.cnt[k];
    return snprintf(buf, n,
                    "%s %lld instrs=%lld pc=%08x pit=%lld/%lld/%lld:%lld/%lld/%lld dtim3=%lld:%lld "
                    "ssi=%lld/%lld/%lld/%lld frc=%lld/%lld idle=%lld sw=%lld sat=%lld uart=%llu:%08x "
                    "midi=%lld/%lld/%lld d2=%lld audio=%lld:%08x fb=%lld:%08x leds=%lld:%08x sd=%lld/%lld "
                    "d1=%lld/%lld/%lld tr=%lld/%lld stat=%s",
                    tag, (long long)chunk, (long long)e->instrs, e_rd(e, UC_M68K_REG_PC),
                    (long long)e->tim[0].t.fired[0], (long long)e->tim[0].t.fired[2],
                    (long long)e->tim[0].t.fired[3], (long long)e->tim[0].t.missed[0],
                    (long long)e->tim[0].t.missed[2], (long long)e->tim[0].t.missed[3],
                    (long long)e->tim[1].t.fired[3], (long long)e->tim[1].t.missed[3],
                    (long long)e->sst.requests, (long long)e->sst.vector170, (long long)e->sst.vector191,
                    (long long)e->sst.rx_markers, (long long)frc, (long long)e->frc.at_rte,
                    (long long)e->idle.n, (long long)sw, (long long)e->satisfied,
                    (unsigned long long)e->uart_out_total, e->uart_out_crc, (long long)e->midi.tx_sent,
                    (long long)e->midi.rx_bytes, (long long)e->midi.uart_irqs, (long long)e->dspi2_frames,
                    (long long)e->au.samples, e->au.crc, (long long)e->frames, e->fb_crc,
                    (long long)e->leds.version, e_crc(0, e->leds.values, 50), (long long)e->sd.log_n,
                    (long long)e->sd.dma_bytes, (long long)e->d1.transfers,
                    (long long)(e->d1.replies[0] + e->d1.replies[1] + e->d1.replies[2] + e->d1.replies[3]),
                    (long long)e->d1.ready_irqs, (long long)e->tr.sent, (long long)e->tr.replies,
                    e->halted ? "halted" : "ok");
}

dt_machine *dt_engine_machine(dt_engine *e)
{
    return e->m;
}

/* ---------------------------------------------- solo per i test (ctypes) */
DT_API void dtt_tanhf_array(const float *in, float *out, size_t n);
void dtt_tanhf_array(const float *in, float *out, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++)
        out[i] = au_tanhf(in[i]);
}

/* emu/audio.py Synth(bank={t: sample}, slot_sample=None): i frame in fila,
   l'uscita (float32) di ogni blocco di sintesi in `out` (n_frames/16*512). */
typedef struct {
    float *out;
    size_t n, cap;
} dtt_sink;

static void dtt_sink_fn(void *ctx, const float *block, const float *tracks, size_t n)
{
    (void)tracks;
    dtt_sink *s = (dtt_sink *)ctx;
    size_t k = n < s->cap - s->n ? n : s->cap - s->n;
    memcpy(s->out + s->n, block, k * sizeof(float));
    s->n += k;
}

DT_API size_t dtt_synth_run(const float *sample, size_t n, const uint8_t *frames, size_t nframes,
                            size_t frame_len, float *out, size_t out_cap);
size_t dtt_synth_run(const float *sample, size_t n, const uint8_t *frames, size_t nframes,
                     size_t frame_len, float *out, size_t out_cap)
{
    dt_engine *e = (dt_engine *)calloc(1, sizeof *e);
    dtt_sink sink = {out, 0, out_cap};
    au_arr *a;
    size_t i;
    if (!e)
        return 0;
    a = au_new(n);
    memcpy(a->d, sample, n * sizeof(float));
    e->au.kick = a;
    e->au.no_slots = 1;
    e->au.master = 0.25f;
    e->au.sink = dtt_sink_fn;
    e->au.sink_ctx = &sink;
    for (i = 0; i < nframes; i++)
        audio_on_frame(e, frames + i * frame_len, frame_len);
    audio_free(&e->au);
    free(e);
    return sink.n;
}

/* emu/dspi1.py Dspi1._reply(request, n_words) con una SampleMemory costruita
   dai blocchi `blocks` (ognuno: u32 BE cmd + 4096 byte), come on_block. */
DT_API void dtt_dspi1_reply(const uint8_t *blocks, size_t nblocks, uint32_t cmd, uint32_t slot,
                            uint32_t n_words, uint8_t *out);
void dtt_dspi1_reply(const uint8_t *blocks, size_t nblocks, uint32_t cmd, uint32_t slot,
                     uint32_t n_words, uint8_t *out)
{
    dt_engine *e = (dt_engine *)calloc(1, sizeof *e);
    size_t i;
    if (!e)
        return;
    for (i = 0; i < nblocks; i++)
        au_on_block(&e->au, e_be32(blocks + i * 4100), blocks + i * 4100 + 4);
    d1_reply(e, cmd, slot, out, n_words);
    audio_free(&e->au);
    free(e);
}
