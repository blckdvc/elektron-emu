/* Il motore della GUI v2 del Rytm con --audio (emu/gui.py Emulator con
 * configure_rytm_audio + longrun.build + i modelli + emu/rytmsynth.py), in C,
 * senza Python. Piano e verifiche: docs/AU-PORT.md.
 *
 * Una sola unita' di traduzione: il C nativo che usa anche il Python
 * (native/dt2_hooks.c, native/dt2_spin.c, lo stesso testo che
 * emu/nativehooks.py compila) e poi le parti e_*.inc.c. Il motore delle voci
 * (native/rytm_dsp.c) e le copie eDMA (native/dt2_dma.c, in dma.c) sono unita'
 * a parte, compilate come le compila il Python. Gli hook si registrano
 * nell'ordine del Python (l'ordine conta dove due hook coprono lo stesso
 * indirizzo: SERQ, gli RTE, i registri dei DTIM); il riferimento e'
 * tools/arcore_ref.py e `arcore play` deve dare la sua stessa traccia.
 *
 * Dal motore del Digitakt II (digitakt/emulator/core, il worker della stessa
 * GUI) e del Model:Cycles (model_cycles/emulator/core, lo stesso C nativo). */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

#include "native/dt2_hooks.c"
#include "native/dt2_spin.c"

#include "arcore.h"
#include "internal.h"
#include "machine.h"
#include "sha256.h"
#include "dma.h"

#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#include "profiles.inc.c"
typedef struct dt_engine dt_engine;
#include "e_base.inc.c"
#include "e_build.inc.c"
#include "e_timers.inc.c"
#include "e_periph.inc.c"
#include "e_rytm.inc.c"
#include "e_panel.inc.c"
#include "e_state.inc.c"
#include "e_calib.inc.c"      /* dt_engine_calibrate */
#include "e_boot.inc.c"       /* dt_coldboot_session, dt_fw_supported */

#define E_BUDGET 400000                  /* gui.BUDGET */
#define E_PER_BLOCK 3.87                 /* longrun._FastStepper.PER_BLOCK */
#define E_IDLE_YIELD 20000
#define E_SSI_HZ 96000                   /* gui.AUDIO_SSI_HZ */
#define E_IPS 60000000                   /* gui.RYTM_AUDIO_IPS */
#define E_PANEL_DWELL 4                  /* gui.PANEL_DWELL_CHUNKS */

/* ---------------------------------------------------- componenti DTST */
typedef struct {
    dt_engine *e;
    int have_ssi, have_etx, have_tim, have_sd, have_nor, have_led;
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
        e->sst.en = (int32_t)(e_le32(d + 40) & 3);   /* bit 0 = 52, bit 1 = 54 (snap2dtst) */
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
        e->ssi_tx_crc = e_le32(d + 96);
        e->sst.vector170 = (int64_t)e_le64(d + 100);
        e->sst.vector191 = (int64_t)e_le64(d + 108);
        L->have_ssi = 1;
    } else if (!memcmp(tag, "CSDH", 4)) {
        e_esdhc *sd = &e->sd;
        uint64_t off = 0, i, cnt;
        if (n < 33) {                               /* l'intestazione; senza settori scritti e' tutto */
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
    } else if (!memcmp(tag, "CNOR", 4)) {
        uint64_t off = 4, i, cnt = n >= 4 ? e_le32(d) : 0;
        for (i = 0; i < cnt; i++) {
            uint32_t b;
            if (off + 4 + NOR_BLOCK > n) {
                dt_errf(err, errlen, "truncated NOR component");
                return DT_ERR_FORMAT;
            }
            b = e_le32(d + off);
            if (b % NOR_BLOCK || b / NOR_BLOCK >= NOR_MAXBLK) {
                dt_errf(err, errlen, "bad NOR block 0x%x", b);
                return DT_ERR_FORMAT;
            }
            free(e->nor.blk[b / NOR_BLOCK]);
            e->nor.blk[b / NOR_BLOCK] = (uint8_t *)malloc(NOR_BLOCK);
            if (!e->nor.blk[b / NOR_BLOCK])
                return DT_ERR_NOMEM;
            memcpy(e->nor.blk[b / NOR_BLOCK], d + off + 4, NOR_BLOCK);
            off += 4 + NOR_BLOCK;
        }
        L->have_nor = 1;
    } else if (!memcmp(tag, "CLED", 4)) {
        int r, l;
        if (n != 4 * 128 * 2 + 36) {
            dt_errf(err, errlen, "bad LED component");
            return DT_ERR_FORMAT;
        }
        for (r = 0; r < 4; r++)
            for (l = 0; l < 128; l++)
                e->leds.regs[r][l] = (int16_t)(d[2 * (r * 128 + l)] | (d[2 * (r * 128 + l) + 1] << 8));
        for (l = 0; l < 12; l++)
            memcpy(e->leds.pads[l], d + 1024 + 3 * l, 3);
        L->have_led = 1;
    }
    return 0;
}

/* timers dalla CTIM: Pits((3, 2, 0)), Dtims((3,), clear_stale=False) e il loro stato */
static int timers_restore(dt_engine *e, const uint8_t *d, uint64_t n)
{
    static const uint32_t want_ch[2][3] = {{3, 2, 0}, {3, 0, 0}};
    static const int want_n[2] = {3, 1};
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
        if (off + 149 > n)
            return -1;
        kind = e_le32(p);
        nch = e_le32(p + 4);
        for (c = 0; c < 4; c++)
            ch[c] = e_le32(p + 8 + 4 * c);
        ips = (int64_t)e_le64(p + 24);
        now = (int64_t)e_le64(p + 32);
        held = p[40];
        if (kind != i || (int)nch != want_n[i])
            return -1;
        for (c = 0; c < (int)nch; c++)
            if (ch[c] != want_ch[i][c])
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
        if (kind == 1)
            e->dtim_arm = e_le32(p + 141);
        s->stale = e_le32(p + 145);
        off += 149;
    }
    e->ntim = 2;
    return 0;
}

/* Timers(Pits(m, hold=intro), Dtims(m, channels=(3,), hold=intro)) nuovi, come
   Emulator.run per uno stato senza il componente dei timer; il Rytm non ha
   l'intro (intro_pit3_isr non risolto: hold = False) */
static int timers_fresh(dt_engine *e)
{
    static const uint32_t pch[3] = {3, 2, 0}, dch[3] = {3, 0, 0};
    tsrc_init(e, &e->tim[0], 0, 3, pch, 4680000, 0);     /* emu.pit.INSTR_PER_SEC */
    tsrc_init(e, &e->tim[1], 1, 1, dch, 4680000, 0);
    e->dtim_arm = 0;
    e->ntim = 2;
    return 0;
}

/* ---------------------------------------------------------- idle slow */
/* replace_idle_spins _slow(n): tx.deliver(), ogni yield_every il vettore 32 */
static void idle_slow_c(int64_t n)
{
    dt_engine *e = e_cur;
    tx_deliver(e, &e->etx);
    if (n % E_IDLE_YIELD == 0)
        dtm_raise_vector(e->m, 32, 0, -1);
}

/* --------------------------------------------- il multiplexer di SERQ
   harness.mux_write_hook: un solo hook Unicorn, i gestori nell'ordine in cui
   il Python li registra: edma (35), Ssi0Dma._on_serq (col flush dei pezzi del
   TX), la scoperta del cablaggio del Rytm (gia' fatta: niente), Esdhc, la
   porta MIDI, il frame del motore delle voci. */
static void ssi_on_serq(dt_engine *e, int size, int64_t value)
{
    sst_state *s = &e->sst;
    ssi_flush(e);
    if (size != 1 || (value & 0x80))
        return;
    if (value & 0x40)
        s->en |= 3;
    else if ((value & 0x3F) == 52)
        s->en |= 1;
    else if ((value & 0x3F) == 54)
        s->en |= 2;
    if (s->en && !s->has_next) {                  /* next = _after(now) */
        s->nextN = s->now * s->rh + s->ips;
        s->has_next = 1;
    }
}

static void h_serq_mux(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a;
    edma_on_serq(e, size, v);
    ssi_on_serq(e, size, v);
    sd_on_serq(e, size, v);
    if (e->midi.installed)
        mi_on_serq(e, size, v);
    syn_on_serq(e, size, v);
}

/* ------------------------------------------------------------- setup */
dt_engine *dt_engine_new(const dt_firmware *fw, const char *state, const dt_engine_config *cfg,
                         char *err, size_t errlen)
{
    return dt_engine_new_mem(fw, state, NULL, 0, cfg, err, errlen);
}

static int verify(dt_engine *e, uint32_t addr, const uint8_t *want, size_t n)
{
    uint8_t b[64];
    return n <= sizeof b && !e_read(e, addr, b, n) && !memcmp(b, want, n);
}

dt_engine *dt_engine_new_mem(const dt_firmware *fw, const char *state, const uint8_t *state_buf,
                             size_t state_len, const dt_engine_config *cfg, char *err, size_t errlen)
{
    dt_engine *e;
    dt_machine_config mc = {0};
    uint8_t sha[32];
    char hex[65];
    dt_sha256 h;
    size_t i, off;
    e_load L;
    int base;
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
    if (!e->p->ssi0_dma_force_rte) {
        dt_errf(err, errlen, "SSI0 DMA model requires the force-ISR RTE symbol");
        goto fail;
    }
    if (e->p->frame_sem || e->p->intro_done || e->p->px_copy) {
        dt_errf(err, errlen, "firmware profile with hooks this engine does not model");
        goto fail;
    }
    e->dwell_chunks = cfg && cfg->panel_dwell ? cfg->panel_dwell : E_PANEL_DWELL;
    rl_init(&e->leds);

    /* dspboot.build_flash: 16 MB, container a 0x80000 */
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000) {
        dt_errf(err, errlen, "cannot build the flash image");
        goto fail;
    }
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);

    /* Machine(): MEM_INVALID; install_isa_patches_scoped (FF1/BITREV/BYTEREV,
       MOVEC); l'INTR di machine.c, poi sostituito da dt2_on_intr */
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
    e->exc.mem_write = (mem_rw_fn)(void *)uc_mem_write;
    e->exc.vbr = DT_VBR;
    e->exc.r_a7 = UC_M68K_REG_A7;
    e->exc.r_sr = UC_M68K_REG_SR;
    e->exc.r_pc = UC_M68K_REG_PC;
    e->exc.fallback = intr_fallback_c;
    e->q.exc = &e->exc;
    e->q.py_raise = py_raise_c;
    e->stp.stop = stop_c;

    /* ---- longrun.build ---- */
    /* rytm_nor.install */
    if (nor_verify(e)) {
        e->nor.installed = 1;
        e_hook_code(e, (void *)h_nor_read, e, NOR_READ);
        e_hook_code(e, (void *)h_nor_write, e, NOR_WRITE);
        e_hook_code(e, (void *)h_nor_erase, e, NOR_ERASE);
    }
    e_hook_code(e, (void *)h_flash_read, e, e->p->flash_read);
    e_hook_code(e, (void *)h_pend_call, e, e->p->pend_call);
    if (e->p->task_create)
        e_hook_code(e, (void *)h_task_create, e, e->p->task_create);
    e_hook_code(e, (void *)h_print, e, E_PRINT);
    if (e->p->set_pixel)
        e_hook_code(e, (void *)h_setpixel_count, e, e->p->set_pixel);
    e_hook_code(e, (void *)h_switch_count, e, E_SWITCH_TO);
    e_hook_code(e, (void *)h_depack_clamp, e, e->p->depack_copy);
    /* edma: il canale 35 (il primo nel multiplexer di SERQ) */
    tx_init(&e->etx, 35, 155, &e->uart_out, &e->uart_out_total, &e->uart_out_crc);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_serq_mux, e, E_SERQ, E_SERQ);
    if (e->p->uart8_tx_wait)
        e_hook_code(e, (void *)h_edma_wait, e, e->p->uart8_tx_wait);
    /* ssi: lo stato (CINT e INTFRCH1 in C alla fine: nativespin._own_ssi) */
    e->sst.mem_read = (mem_rw_fn)uc_mem_read;
    e->sst.mem_write = (mem_rw_fn)(void *)uc_mem_write;
    e->sst.rh = E_SSI_HZ;
    e->sst.ips = 4680000;                     /* emu.pit.INSTR_PER_SEC, poi il checkpoint */
    e->sst.rx_ch = 52;                        /* il cablaggio del Rytm (_run_detect) */
    e->sst.tx_ch = 54;
    e->sst.level_retry = 1;
    e->sst.slack = 256;
    e->sst.level_retry_step = 8000;
    e->txbuf = (uint8_t *)malloc(AR_TXCAP);
    if (!e->txbuf)
        goto nomem;
    e->sst.txbuf = e->txbuf;                  /* tx_sink: SampleTap */
    e->sst.txcap = AR_TXCAP;
    e->sst.q = &e->q;
    e->sst.tx_vec = 174;
    e->sst.force_vec = 191;
    e->ssi_gate.open = 0;
    e->ssi_gate.cfn = dt2_sst_force_rte;
    e->ssi_gate.cdata = &e->sst;
    e_hook_code(e, (void *)dt2_gate_code, &e->ssi_gate, e->p->ssi0_dma_force_rte);
    /* nativedma: le copie eDMA avviate dal software (vettore 191) */
    e->dma = ar_dma_new();
    if (!e->dma)
        goto nomem;
    for (i = 0; i < 64; i++) {
        uint32_t csr = E_TCD_BASE + (uint32_t)i * 0x20u + 0x1C;
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_dma_on_csr, e->dma, csr, csr + 3);
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)dt2_dma_on_read, e->dma, csr, csr + 3);
    }
    /* dsp (fast): la lettura di STATUS in C alla fine (replace_dsp_read) */
    /* idle spin: in C alla fine (replace_idle_spins) */
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
    /* UART8 */
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_uart8_read, e, E_USR8, E_UDR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_uart8_write, e, E_USR8, E_UDR8 + 3);
    dtm_mmio_set(e->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(e->m, 0xEC03802Cu, 0x80000000u);
    /* esdhc (prima del ripristino; il suo SERQ e' nel multiplexer) */
    sd_init(e);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);

    /* ---- restore_into ---- */
    memset(&L, 0, sizeof L);
    L.e = e;
    r = state_buf ? dtm_load_state_mem(e->m, state_buf, state_len, comp_record, &L, err, errlen)
                  : dtm_load_state(e->m, state, comp_record, &L, err, errlen);
    if (r) {
        free(L.ctim);
        goto fail;
    }
    /* uno stato del boot freddo (un gradino della scala: solo macchina ed
       eSDHC) non ha ne' SSI ne' timer: come il worker della GUI, che li
       costruisce nuovi (Pits/Dtims, SSI0 da armare), si parte da zero */
    base = !L.have_ssi && !L.have_tim;
    if (!base && !L.have_ssi) {
        dt_errf(err, errlen, "state without the SSI component: make it with tools/arcore_ref.py --save");
        free(L.ctim);
        goto fail;
    }
    if (!base && !L.have_tim) {
        dt_errf(err, errlen, "state without timers (make it with tools/arcore_ref.py --save)");
        goto fail;
    }
    /* m.premap_sdram(); longrun.finish_pending_dma */
    for (i = 0x40000000u; i < 0x50000000u; i += PAGE)
        dtm_ensure(e->m, i);
    ar_dma_finish_pending(e->dma, e->uc);
    /* intfrc: gli hook di scrittura in C alla fine; gli RTE ora (un cancello) */
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
    /* midi (il SERQ nel multiplexer) */
    if (mi_find_rx_handler(e, &e->midi.ring_base_ptr)) {
        e_midi *mi = &e->midi;
        mi->installed = 1;
        tx_init(&mi->tx, 37, 157, &mi->tx_out, &mi->tx_out_total, &mi->tx_out_crc);
        mi->hint = 1;
        mi->uimr = e_r8(e, MI_UIMR9);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_udr, e, MI_UDR9, MI_UDR9);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_uimr, e, MI_UIMR9, MI_UIMR9);
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_mi_usr, e, MI_USR9, MI_USR9);
        mi->uart_masked = 1;
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mi_imr, e, MI_SIMR1, MI_CIMR1);
    }
    /* install_mmio */
    dtm_install_mmio(e->m);
    /* edma: kick solo senza stato del modello */
    if (!e->etx.restored && e->p->uart8_tx_state && e_r32(e, e->p->uart8_tx_state))
        tx_run(e, &e->etx);

    /* ---- Emulator.run ---- */
    /* RytmLedState: la tabella del firmware, poi i registri salvati (CLED) */
    {
        e_rleds saved = e->leds;
        rl_init(&e->leds);
        rl_load(e);
        if (L.have_led) {
            int rr, l;
            for (rr = 0; rr < 4; rr++)
                for (l = 0; l < 128; l++)
                    if (saved.regs[rr][l] >= 0)
                        e->leds.regs[rr][l] = saved.regs[rr][l];
            memcpy(e->leds.pads, saved.pads, sizeof saved.pads);
            e->leds.version++;
        }
    }
    /* rytm_rpc.install */
    if (e->midi.installed && verify(e, RPC_ENTRY, RPC_ENTRY_VERIFY, sizeof RPC_ENTRY_VERIFY)
        && verify(e, RPC_FLUSH, RPC_FLUSH_VERIFY, sizeof RPC_FLUSH_VERIFY)) {
        e->rpc.installed = 1;
        e_hook_code(e, (void *)h_rpc_entry, e, RPC_ENTRY);
        e_hook_code(e, (void *)h_rpc_flush, e, RPC_FLUSH);
    }
    /* timers: restore_checkpoint_timers (Pits, Dtims con l'hook arm,
       install_counters, rearm_cleared) */
    if (base ? timers_fresh(e) : timers_restore(e, L.ctim, L.ctim_len)) {
        dt_errf(err, errlen, "bad timers component (Pits (3,2,0) + Dtims (3,) expected)");
        free(L.ctim);
        goto fail;
    }
    free(L.ctim);
    L.ctim = NULL;
    if (e->tim[0].t.held || e->tim[1].t.held) {
        dt_errf(err, errlen, "timers held: the intro is not supported");
        goto fail;
    }
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dtim_arm, e, DTIM_BASES[3], DTIM_BASES[3] + 0x0F);
    dtcn_install(e);
    e->dtcn.py_read[0] = e->dtcn.py_read[1] = e->dtcn.py_read[2] = e->dtcn.py_read[3] = dtcn_py_read;
    for (i = 0; i < 4; i++) {
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)dt2_dtcn_on_read, &e->dtcn_sites[i],
                   DTIM_BASES[i] + DTCN_OFF, DTIM_BASES[i] + DTCN_OFF + 3);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dtcn_write, &e->dtcn_sites[i], DTIM_BASES[i],
                   DTIM_BASES[i] + 0x0F);
    }
    dtim_rearm_cleared(e);
    /* rytmsynth.install: il frame nel multiplexer di SERQ; SampleTap = tx_sink */
    if (syn_init(e))
        goto nomem;
    e->sst.chunk_end = e->syn.chunk_end;
    e->sst.chunk_src = e->syn.chunk_src;
    e->sst.nchunk = 0;
    e->sst.chunkcap = AR_CHUNKCAP;
    /* ssi0.align(pits.now) */
    e->sst.now = e->tim[0].now;
    /* _pending_ips.append((0, RYTM_AUDIO_IPS)) */
    e->has_pending_ips = 1;
    e->pending_ips_at = 0;
    e->pending_ips_n = E_IPS;
    /* use_panel: il frame mostrato adesso (fb_back) */
    if (e->p->fb_back && panel_read_at(e, e->p->fb_back, e->latch))
        e->has_latch = 1;
    if (e->p->job_pump)
        e_hook_code(e, (void *)h_job_pump, e, e->p->job_pump);
    e_hook_code(e, (void *)h_terminal, e, PN_TERMINAL);
    if (e->p->panel_diff && e->p->fb_front)
        e_hook_code(e, (void *)h_panel_latch, e, e->p->panel_diff);
    /* nativehooks.install: lo stepper (tb-budget) e l'INTR in C */
    if (uc_set_block_budget(e->uc, (int64_t *)&e->stp) != UC_ERR_OK) {
        dt_errf(err, errlen, "uc_set_block_budget failed (Unicorn without the tb-budget patch)");
        goto fail;
    }
    uc_hook_del(e->uc, e->m->intr_hook);
    e->m->intr_hook = e_hook_mem(e, UC_HOOK_INTR, (void *)dt2_on_intr, &e->exc, 1, 0);
    /* replace_dsp_read */
    e->dspw.write = (memw_fn)(void *)uc_mem_write;
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
    for (i = 0; i < (size_t)e->nidle; i++)
        e_hook_code(e, (void *)dt2_on_idle, &e->idle, e->idle_addrs[i]);
    /* nativespin.install: SSI e interrupt forzati posseduti dal C, l'arm dei Dtims */
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_cint, &e->sst, 0xFC04401Cu, 0xFC04401Cu);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_intfrch1, &e->sst, 0xFC04C010u, 0xFC04C013u);
    e->ssi_gate.open = 1;                         /* _own_ssi: sempre aperto, decide il C */
    {
        static const uint32_t ib[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
        int k;
        for (k = 0; k < 3; k++)
            e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_frc_on_write, &e->frc, ib[k] + 0x10, ib[k] + 0x17);
    }
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_tim_on_write, &e->tim[1].t, DTIM_BASES[3],
               DTIM_BASES[3] + 0x0F);
    /* il primo step(): Ssi0Dma._run_detect -> _use_rytm (contatori dei
       major loop e degli scatter/gather azzerati), enabled |= {52, 54} */
    e->sst.en = 3;
    e->sst.major_rx = e->sst.major_tx = e->sst.sg_rx = e->sst.sg_tx = 0;
    /* NativeSpin */
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
        e->S.ev[e->S.nev++] = EV_MIDI;
        e->S.midi_hint = &e->midi.hint;
        e->S.midi_txpend = &e->midi.tx.pending;
        e->S.midi_step = mi_step;
        e->S.midi_service = mi_service;
    }
    if (e->rpc.installed)
        e->S.ev[e->S.nev++] = EV_NONE;           /* SysexRpcLink: step None, service nulla */
    e->S.stop_flag = &e->m->stop_flag;
    e->S.tx_flush = AR_TXCAP / 2;
    /* gli RTE vedono i timer del C; DTCN letto dal C */
    e->frc.ntim = 2;
    e->frc.tim[0] = &e->tim[0].t;
    e->frc.tim[1] = &e->tim[1].t;
    e->dtcn.per_block = E_PER_BLOCK;
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
    int i;
    if (!e)
        return;
    if (e->m)
        dt_machine_free(e->m);
    free(e->flash);
    free(e->txbuf);
    free(e->dma);
    free(e->uart_out.p);
    free(e->uart_in.p);
    ov_free(&e->sd.ov);
    free(e->midi.rx.p);
    free(e->midi.tx_out.p);
    free(e->midi.host_out.p);
    for (i = 0; i < NOR_MAXBLK; i++)
        free(e->nor.blk[i]);
    free(e->rpc.rx.p);
    free(e->rpc.replies.p);
    free(e->syn.dsp);
    free(e->syn.chunk_end);
    free(e->syn.chunk_src);
    free(e->syn.tap_data.p);
    free(e->syn.tap_end);
    free(e->syn.tap_src);
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
    int code;
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
    e->dtcn.ips = e->tim[1].ips;
    e->dtcn.live_now = &S->now;
    for (;;) {
        code = dt2_spin(e->uc, S);
        if (code == SPIN_LIMIT)
            break;
        if (code == SPIN_FLUSH) {
            ssi_flush(e);
            continue;
        }
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
            if (dt2_spin_service(e->uc, S) < 0) {
                snprintf(e->status, sizeof e->status, "halted: native error %d/%d", S->err, e->q.err);
                break;
            }
            continue;
        }
        snprintf(e->status, sizeof e->status, "halted: native error %d/%d/%d/%d", S->err, e->q.err,
                 e->sst.err, e->frc.err);
        break;
    }
    e->dtcn.live_now = NULL;
    e->running = 0;
    e->tim[0].now = e->tim[1].now = S->base + S->done;
    /* Dtims.step svuota arm dei canali; l'ultimo passo li ha scritti */
    e->dtim_arm = (e->dtim_arm & ~(1u << 3)) | (e->tim[1].t.armw & (1u << 3));
    e->pc = S->pc;
    *executed = S->done;
    ssi_flush(e);
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
            if (tsrc_set_ips(e, &e->tim[k], e->pending_ips_n)) {
                snprintf(e->status, sizeof e->status, "halted: %s", e->err);
                e->halted = 1;
                return 1;
            }
        e->sst.ips = e->pending_ips_n;
    }
    e->pc = panel_drain(e, e->pc);
    if (engine_spin(e, E_BUDGET, &executed)) {
        e->instrs += executed;
        e->halted = 1;
        return 1;
    }
    e->instrs += executed;
    panel_publish(e);
    if (e->uart_out.n) {
        rl_feed(&e->leds, e->uart_out.p, e->uart_out.n);
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

/* rytm_pads.raw_for / press / release */
static int pad_curve(int raw)
{
    int v = (int)((double)(raw - 10) * 11.0 / 4.0);
    v = (int)((double)v * 255.0 / 245.0);
    if (v > 255)
        v = 255;
    return v > 0 ? (v * v * 0x102) >> 16 : 0;
}

static int pad_raw_for(int velocity)
{
    int want = 2 * velocity + 1, out, raw;
    if (want < 1)
        want = 1;
    if (want > 255)
        want = 255;
    for (out = want; out < 256; out++)
        for (raw = 0; raw < 256; raw++)          /* _RAW[curve(raw)] = il raw piu' piccolo */
            if (pad_curve(raw) == out)
                return raw;
    return 255;
}

int dt_engine_panel(dt_engine *e, int kind, int code, int arg)
{
    e_input in;
    memset(&in, 0, sizeof in);
    if (kind == DT_PANEL_PAD) {
        if (code < 0 || code > 11)
            return DT_ERR_ARG;
        in.kind = 5;
        if (arg > 0) {
            in.raw[0] = (uint8_t)(0xC0 | code);
            in.raw[1] = 0x01;
            in.raw[2] = (uint8_t)(0xA0 | code);
            in.raw[3] = (uint8_t)pad_raw_for(arg > 127 ? 127 : arg);
        } else {
            in.raw[0] = (uint8_t)(0xC0 | code);
            in.raw[1] = 0x00;
            in.raw[2] = (uint8_t)(0xB0 | code);
            in.raw[3] = 0x00;
        }
        in.nraw = 4;
    } else {
        in.kind = kind;
        in.code = code;
        in.arg = arg;
    }
    return inbox_push_back(e, in) ? DT_ERR_NOMEM : DT_OK;
}

int dt_engine_panel_raw(dt_engine *e, const uint8_t *data, size_t n)
{
    while (n) {
        e_input in;
        size_t k = n > sizeof in.raw ? sizeof in.raw : n;
        memset(&in, 0, sizeof in);
        in.kind = 5;
        memcpy(in.raw, data, k);
        in.nraw = (int)k;
        if (inbox_push_back(e, in))
            return DT_ERR_NOMEM;
        data += k;
        n -= k;
    }
    return DT_OK;
}

/* SysexRpcLink.rpc, la parte che manda: il SysEx sulla MIDI IN */
int dt_engine_rpc(dt_engine *e, int id, const uint8_t *body, size_t n, uint16_t *seq_out)
{
    e_rpc *r = &e->rpc;
    uint8_t *pl, *msg;
    size_t plen = 5 + n, mlen;
    uint16_t seq;
    int rc;
    if (!r->installed)
        return DT_ERR_UNSUPPORTED;
    r->seq = (r->seq % 0x3FFF) + 1;
    seq = (uint16_t)r->seq;
    pl = (uint8_t *)malloc(plen);
    msg = (uint8_t *)malloc(sizeof RPC_HEADER + plen + plen / 7 + 2 + 1);
    if (!pl || !msg) {
        free(pl);
        free(msg);
        return DT_ERR_NOMEM;
    }
    e_put16(pl, seq);
    e_put16(pl + 2, 0);
    pl[4] = (uint8_t)id;
    if (n)
        memcpy(pl + 5, body, n);
    memcpy(msg, RPC_HEADER, sizeof RPC_HEADER);
    mlen = sizeof RPC_HEADER + rpc_encode7(pl, plen, msg + sizeof RPC_HEADER);
    msg[mlen++] = 0xF7;
    r->inflight++;
    rc = dt_engine_midi_in(e, msg, mlen);
    free(pl);
    free(msg);
    if (seq_out)
        *seq_out = seq;
    return rc;
}

size_t dt_engine_rpc_replies(dt_engine *e, uint8_t *buf, size_t max)
{
    size_t n = e->rpc.replies.n < max ? e->rpc.replies.n : max;
    if (buf && n)
        memcpy(buf, e->rpc.replies.p, n);
    e_buf_drop(&e->rpc.replies, buf ? n : e->rpc.replies.n);
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

void dt_engine_set_audio_sink(dt_engine *e,
                              void (*sink)(void *ctx, const float *lr, const float *voices, size_t frames),
                              void *ctx)
{
    e->syn.sink = sink;
    e->syn.sink_ctx = ctx;
}

void dt_engine_set_volume(dt_engine *e, double volume)
{
    e->syn.master = (float)volume;                /* rytmsynth.MASTER: un float del Python */
}

int64_t dt_engine_audio_frames(dt_engine *e)
{
    return e->syn.samples;
}

uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64])
{
    if (out)
        memcpy(out, e->fb, sizeof e->fb);
    return e->panel_version;
}

uint64_t dt_engine_leds(dt_engine *e, uint8_t lit[128], uint8_t pads[12][3])
{
    int i;
    if (lit)
        for (i = 0; i < 128; i++)
            lit[i] = rl_lit(&e->leds, i);
    if (pads)
        memcpy(pads, e->leds.pads, sizeof e->leds.pads);
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

/* la riga 'chunk' di tools/arcore_ref.py */
int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n)
{
    int64_t frc = 0, dma[4];
    int k;
    for (k = 0; k < 192; k++)
        frc += e->frc.delivered[k];
    ar_dma_counters(e->dma, dma);
    return snprintf(buf, n,
                    "%s %lld instrs=%lld pc=%08x pit=%lld/%lld/%lld:%lld/%lld/%lld dtim3=%lld:%lld "
                    "ssi=%lld/%lld/%lld/%lld frc=%lld/%lld idle=%lld sw=%lld uart=%llu:%08x "
                    "midi=%lld/%lld/%lld audio=%lld:%08x fb=%lld:%08x leds=%lld:%08x sd=%lld/%lld "
                    "dma=%lld/%lld trig=%lld/%lld rpc=%lld stat=%s",
                    tag, (long long)chunk, (long long)e->instrs, e_rd(e, UC_M68K_REG_PC),
                    (long long)e->tim[0].t.fired[0], (long long)e->tim[0].t.fired[2],
                    (long long)e->tim[0].t.fired[3], (long long)e->tim[0].t.missed[0],
                    (long long)e->tim[0].t.missed[2], (long long)e->tim[0].t.missed[3],
                    (long long)e->tim[1].t.fired[3], (long long)e->tim[1].t.missed[3],
                    (long long)e->sst.requests, (long long)e->sst.vector170, (long long)e->sst.vector191,
                    (long long)e->sst.tx_bytes, (long long)frc, (long long)e->frc.at_rte,
                    (long long)e->idle.n, (long long)e->switches,
                    (unsigned long long)e->uart_out_total, e->uart_out_crc, (long long)e->midi.tx_sent,
                    (long long)e->midi.rx_bytes, (long long)e->midi.uart_irqs,
                    (long long)e->syn.samples, e->syn.crc, (long long)e->frames, e->fb_crc,
                    (long long)e->leds.version, rl_crc(&e->leds), (long long)e->sd.log_n,
                    (long long)e->sd.dma_bytes, (long long)dma[0], (long long)dma[1],
                    (long long)e->syn.frames, (long long)e->syn.triggers, (long long)0,
                    e->halted ? "halted" : "ok");
}

dt_machine *dt_engine_machine(dt_engine *e)
{
    return e->m;
}
