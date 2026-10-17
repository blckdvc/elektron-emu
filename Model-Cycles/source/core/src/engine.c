/* Il motore del Model:Cycles in C, senza Python: la macchina che
 * emu/mcengine.py resume() costruisce (longrun.build + mchw + timer + C nativo)
 * e il giro del thread del motore della GUI (Engine._run: ingressi, una fetta
 * di 5 ms emulati, schermo, LED, audio dall'SSI). Piano e verifiche:
 * docs/AU-PORT.md.
 *
 * Una sola unita' di traduzione: il C nativo che usa anche il Python
 * (native/dt2_hooks.c, native/dt2_spin.c, lo stesso testo che
 * emu/nativehooks.py compila) e poi le parti e_*.inc.c. Gli hook si registrano
 * nell'ordine del Python (l'ordine conta dove due hook coprono lo stesso
 * indirizzo: SERQ, 0x8C000002, DTIM3); il riferimento e' tools/mccore_ref.py e
 * `mccore play` deve dare la sua stessa traccia.
 *
 * Dal motore del Digitakt II (digitakt/emulator/core, commit eb7fc7c): stessa
 * macchina, stesso C nativo d'origine, stessi modelli di eDMA/eSDHC/MIDI/timer;
 * qui niente DSP (la sintesi e' del firmware, l'audio esce dall'SSI) e la
 * scheda del Cycles (e_mc.inc.c). */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

#include "native/dt2_hooks.c"
#include "native/dt2_spin.c"

#include "mccore.h"
#include "internal.h"
#include "machine.h"
#include "sha256.h"
#include "dma.h"

#include <stdlib.h>
#include <string.h>
#include <zlib.h>
#include <time.h>

#include "profiles.inc.c"
typedef struct dt_engine dt_engine;
#include "e_base.inc.c"
#include "e_build.inc.c"
#include "e_timers.inc.c"
#include "e_periph.inc.c"
#include "e_mc.inc.c"
#include "e_state.inc.c"
#include "e_boot.inc.c"

#define E_PER_BLOCK 3.87                 /* longrun._FastStepper.PER_BLOCK */
#define E_IDLE_YIELD 20000
#define E_SSI_HZ 48000                   /* mcengine.RATE */
#define E_IPS 40000000                   /* mcengine.IPS */
#define E_SLICE (E_IPS / 200)            /* Engine.SLICE = 5 ms: 200.000 istruzioni */
#define E_TXCAP (1 << 20)                /* nativespin.TXCAP */

/* 1 se il MAIN OS del firmware ha un profilo di indirizzi (e quindi il motore lo fa girare) */
int dt_fw_profile_known(const dt_firmware *fw)
{
    const dt_section *s = dt_fw_section(fw, 3);
    uint8_t sha[32];
    char hex[65];
    dt_sha256 h;
    size_t i;
    if (!s)
        return 0;
    dt_sha256_init(&h);
    dt_sha256_update(&h, s->data, s->len);
    dt_sha256_final(&h, sha);
    dt_sha256_hex(sha, hex);
    for (i = 0; i < DT_NPROFILES; i++)
        if (!strcmp(DT_PROFILES[i].main_sha256, hex))
            return 1;
    return 0;
}

/* ---------------------------------------------------- componenti DTST */
typedef struct {
    dt_engine *e;
    int have_ssi, have_etx, have_tim, have_sd;
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
        /* tx_crc32 (d + 96): solo diagnostica del Python, il C non lo tiene */
        e->sst.vector170 = (int64_t)e_le64(d + 100);
        e->sst.vector191 = (int64_t)e_le64(d + 108);
        L->have_ssi = 1;
    } else if (!memcmp(tag, "CSDH", 4)) {
        e_esdhc *sd = &e->sd;
        uint64_t off = 0, i, cnt;
        if (n < 37) {
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
    } else if (!memcmp(tag, "CSDE", 4)) {
        /* card_erase: l'intervallo CMD35/CMD36 in attesa del CMD38 */
        if (n != 18) {
            dt_errf(err, errlen, "bad esdhc erase range");
            return DT_ERR_FORMAT;
        }
        e->sd.has_erase_start = d[0];
        e->sd.erase_start = (uint32_t)e_le64(d + 1);
        e->sd.has_erase_end = d[9];
        e->sd.erase_end = (uint32_t)e_le64(d + 10);
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
        s->arm = e_le32(p + 141);
        s->stale = e_le32(p + 145);
        off += 149;
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

static int verify(dt_engine *e, uint32_t addr, const uint8_t *want, size_t n)
{
    uint8_t b[64];
    return n <= sizeof b && !e_read(e, addr, b, n) && !memcmp(b, want, n);
}

/* ssi._looks_like_cycles: TCD50 -> STX0 con CITER, TCD48 vuoto */
static int looks_like_cycles(dt_engine *e)
{
    return e_r32(e, E_TCD_BASE + 50 * 0x20u + 0x10) == 0xFC0BC000u
           && e_r32(e, E_TCD_BASE + 48 * 0x20u + 0x00) == 0
           && e_r16(e, E_TCD_BASE + 50 * 0x20u + 0x14) != 0;
}

/* ------------------------------------------------------------- setup */
dt_engine *dt_engine_new(const dt_firmware *fw, const char *state, const dt_engine_config *cfg,
                         char *err, size_t errlen)
{
    return dt_engine_new_mem(fw, state, NULL, 0, cfg, err, errlen);
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
    const uint8_t *img;
    size_t img_len;
    int r, ssi_legacy = 0;

    (void)cfg;
    e = (dt_engine *)calloc(1, sizeof *e);
    if (!e) {
        dt_errf(err, errlen, "out of memory");
        return NULL;
    }
    e->fw = fw;
    e->volume = 1.0f;
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
    if (e->p->frame_sem || e->p->intro_done || e->p->uart8_tx_wait || e->p->px_copy) {
        dt_errf(err, errlen, "firmware profile with hooks this engine does not model");
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
    e_hook_code(e, (void *)h_flash_read, e, e->p->flash_read);
    e_hook_code(e, (void *)h_pend_call, e, e->p->pend_call);
    if (e->p->task_create)
        e_hook_code(e, (void *)h_task_create, e, e->p->task_create);
    e_hook_code(e, (void *)h_print, e, E_PRINT);
    if (e->p->set_pixel)
        e_hook_code(e, (void *)h_setpixel_count, e, e->p->set_pixel);
    e_hook_code(e, (void *)h_switch_count, e, E_SWITCH_TO);
    e_hook_code(e, (void *)h_depack_clamp, e, e->p->depack_copy);
    /* edma: il canale 35 nel multiplexer di SERQ (un solo hook, i gestori in ordine) */
    tx_init(&e->etx, 35, 155, &e->uart_out, &e->uart_out_total, &e->uart_out_crc);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_serq_mux, e, E_SERQ, E_SERQ);
    /* ssi: lo stato (CINT e INTFRCH1 in C alla fine: nativespin._own_ssi) */
    e->sst.mem_read = (mem_rw_fn)uc_mem_read;
    e->sst.mem_write = (mem_rw_fn)(void *)uc_mem_write;
    e->sst.rh = E_SSI_HZ;
    e->sst.ips = 4680000;                     /* emu.pit.INSTR_PER_SEC, poi il checkpoint */
    e->sst.rx_ch = 48;
    e->sst.tx_ch = 50;
    e->sst.level_retry = 1;
    e->sst.slack = 256;
    e->sst.level_retry_step = 8000;
    e->txbuf = (uint8_t *)malloc(E_TXCAP);
    if (!e->txbuf)
        goto nomem;
    e->sst.txbuf = e->txbuf;                  /* tx_sink: la cattura del TX */
    e->sst.txcap = E_TXCAP;
    e->sst.q = &e->q;
    e->sst.tx_vec = 170;
    e->sst.force_vec = 191;
    e->ssi_gate.open = 0;
    e->ssi_gate.cfn = dt2_sst_force_rte;
    e->ssi_gate.cdata = &e->sst;
    e_hook_code(e, (void *)dt2_gate_code, &e->ssi_gate, e->p->ssi0_dma_force_rte);
    /* nativedma: le copie eDMA avviate dal software (vettore 191) */
    e->dma = mc_dma_new();
    if (!e->dma)
        goto nomem;
    for (i = 0; i < 64; i++) {
        uint32_t csr = E_TCD_BASE + (uint32_t)i * 0x20u + 0x1C;
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_dma_on_csr, e->dma, csr, csr + 3);
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)dt2_dma_on_read, e->dma, csr, csr + 3);
    }
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)mc_dma_on_ssrt, e->dma, 0xFC04401Eu, 0xFC04401Eu);
    /* dsp (Fifo, poll_delay 0, non fast) */
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_dsp_read, e, DSP_STATUS, DSP_STATUS + 1);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dsp_write, e, DSP_STATUS, DSP_STATUS + 1);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dsp_latch, e, DSP_LATCH, DSP_LATCH + 1);
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
    /* UART8 (il Cycles non ha il micro del pannello: resta muta) */
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_uart8_read, e, E_USR8, E_UDR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_uart8_write, e, E_USR8, E_UDR8 + 3);
    dtm_mmio_set(e->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(e->m, 0xEC03802Cu, 0x80000000u);
    /* esdhc (prima del ripristino) */
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
    if (!L.have_ssi) {
        /* uno stato del boot freddo (dt_coldboot, snapshots/<fw>/cold200M.snap):
           senza il componente, l'aggiornamento "legacy" di Ssi0Dma.arm_legacy
           (il Model:Cycles: TCD50 verso STX0, TCD48 mai programmato) */
        if (!looks_like_cycles(e)) {
            dt_errf(err, errlen, "state without the SSI0 component: make it with tools/mccore_ref.py --save");
            free(L.ctim);
            goto fail;
        }
        ssi_legacy = 1;
    }
    /* m.premap_sdram() */
    for (i = 0x40000000u; i < 0x50000000u; i += PAGE)
        dtm_ensure(e->m, i);
    /* board.install, board.repair_halt */
    board_install(e);
    /* mchw.install: I2C0, PIT1 a polling, il pannello */
    mc_board_hooks(e);
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
    /* edma: kick solo senza stato del modello (uart8_tx_state non risolto qui) */
    if (!e->etx.restored && e->p->uart8_tx_state && e_r32(e, e->p->uart8_tx_state))
        tx_run(e, &e->etx);

    /* ---- mcengine.resume: i timer ---- */
    /* Dtims.__init__ (canale 3): l'hook `arm`, install_counters, rearm_cleared;
       il loro ips e' quello del checkpoint (resume li costruisce cosi') */
    if (L.have_tim) {
        if (timers_restore(e, L.ctim, L.ctim_len)) {
            dt_errf(err, errlen, "bad timers component (Pits (3,2,0) + Dtims (3,) expected)");
            free(L.ctim);
            goto fail;
        }
    } else {
        /* Timers(Pits(m), Dtims(m, channels=(3,))) nuovi, gia' alla cadenza del
           motore (il Python li crea a 4,68M istr/s e li porta a 40M con le
           scadenze razionali: qui direttamente a 40M) */
        static const uint32_t pch[3] = {3, 2, 0}, dch[1] = {3};
        tsrc_init(e, &e->tim[0], 0, 3, pch, E_IPS, 0);
        tsrc_init(e, &e->tim[1], 1, 1, dch, E_IPS, 0);
        e->ntim = 2;
        e->sst.ips = E_IPS;
    }
    free(L.ctim);
    L.ctim = NULL;
    if (e->tim[0].ips != E_IPS || e->tim[1].ips != E_IPS || e->sst.ips != E_IPS) {
        dt_errf(err, errlen, "state not at 40M instructions/s: make it with tools/mccore_ref.py --save");
        goto fail;
    }
    if (e->tim[0].t.held || e->tim[1].t.held) {
        dt_errf(err, errlen, "timers held: the intro is not supported");
        goto fail;
    }
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dtim_arm, &e->tim[1], DTIM_BASES[3], DTIM_BASES[3] + 0x0F);
    dtcn_install(e, E_IPS, 0);
    for (i = 0; i < 4; i++) {
        e->dtcn_ud[i].e = e;
        e->dtcn_ud[i].ch = (int)i;
        e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_dtcn_read, &e->dtcn_ud[i], DTIM_BASES[i] + DTCN_OFF,
                   DTIM_BASES[i] + DTCN_OFF + 3);
        e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_dtcn_write, &e->dtcn_ud[i], DTIM_BASES[i],
                   DTIM_BASES[i] + 0x0F);
    }
    dtim_rearm_cleared(e);
    /* ssi.ips = audio_ips; ssi.align(timers.now); midi.ips */
    e->sst.now = e->tim[0].now;
    if (ssi_legacy) {                             /* arm_legacy + align: next = now + period */
        e->sst.en |= 2;
        e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
        e->sst.has_next = 1;
    }
    e->midi.ips = E_IPS;
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
    for (i = 0; i < (size_t)e->nidle; i++)
        e_hook_code(e, (void *)dt2_on_idle, &e->idle, e->idle_addrs[i]);
    /* nativespin.install: SSI e interrupt forzati posseduti dal C */
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_cint, &e->sst, 0xFC04401Cu, 0xFC04401Cu);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_intfrch1, &e->sst, 0xFC04C010u, 0xFC04C013u);
    e->ssi_gate.open = 1;                         /* nativespin._own_ssi: sempre aperto, decide il C */
    {
        static const uint32_t ib[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
        int k;
        for (k = 0; k < 3; k++)
            e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)dt2_frc_on_write, &e->frc, ib[k] + 0x10, ib[k] + 0x17);
    }
    /* il controllo del cablaggio (Ssi0Dma._run_detect) */
    if (looks_like_cycles(e))
        e->sst.en |= 2;
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
    e->S.stop_flag = &e->m->stop_flag;
    e->S.tx_flush = E_TXCAP / 2;
    e->pc = e_rd(e, UC_M68K_REG_PC);
    memset(e->leds_acc, 0xFF, sizeof e->leds_acc);
    if (getenv("MCCORE_SLICE"))                   /* solo diagnostica: fette piu' corte */
        e->slice = strtoll(getenv("MCCORE_SLICE"), NULL, 0);
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
    free(e->inbox);
    free(e->abuf);
    if (e_cur == e)
        e_cur = NULL;
    free(e);
}

/* ------------------------------------------------------------- audio */
/* TxOut: i byte del TX dell'SSI0 (frame da 8 byte, due parole BE L, R col
   campione nei 24 bit bassi) -> float32 stereo per il sink: ((w << 8) >> 8)
   in float32 per float32(volume / 2**23), come il numpy. */
static void tx_out(dt_engine *e, const uint8_t *data, size_t n)
{
    size_t frames, i, k = 0;
    float g = (float)((double)e->volume / (double)(1 << 23));
    e->tx_bytes_out += (int64_t)n;
    e->tx_crc = e_crc(e->tx_crc, data, n);
    frames = (e->tx_npend + n) / 8;
    if (frames && e->sink) {
        if (e->abuf_cap < 2 * frames) {
            float *p = (float *)realloc(e->abuf, 2 * frames * sizeof(float));
            if (!p)
                return;
            e->abuf = p;
            e->abuf_cap = 2 * frames;
        }
    }
    for (i = 0; i < n; i++) {
        e->tx_pend[e->tx_npend++] = data[i];
        if (e->tx_npend == 8) {
            if (e->sink) {
                int32_t l = (int32_t)(e_be32(e->tx_pend) << 8) >> 8;
                int32_t r = (int32_t)(e_be32(e->tx_pend + 4) << 8) >> 8;
                e->abuf[2 * k] = (float)l * g;
                e->abuf[2 * k + 1] = (float)r * g;
            }
            k++;
            e->tx_npend = 0;
        }
    }
    e->audio_frames += (int64_t)k;
    if (k && e->sink)
        e->sink(e->sink_ctx, e->abuf, k);
}

static void tx_flush(dt_engine *e)
{
    if (e->sst.txlen) {
        size_t n = (size_t)e->sst.txlen;
        e->sst.txlen = 0;
        tx_out(e, e->txbuf, n);
    }
}

/* --------------------------------------------------------------- spin */
/* longrun.spin(m, pc, n, pits=timers, fast=True, async_events) via
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
    for (;;) {
        code = dt2_spin(e->uc, S);
        if (code == SPIN_LIMIT)
            break;
        if (code == SPIN_FLUSH) {
            tx_flush(e);
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
                if (strncmp(e->status, "halted", 6))
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
    e->running = 0;
    e->tim[0].now = e->tim[1].now = S->base + S->done;
    e->pc = S->pc;
    *executed = S->done;
    tx_flush(e);
    return strncmp(e->status, "halted", 6) == 0;
}

/* ---------------------------------------------------------- pannello */
static int inbox_push_back(dt_engine *e, e_input in)
{
    if (e->inbox_n == e->inbox_cap) {
        size_t cap = e->inbox_cap ? 2 * e->inbox_cap : 64, k;
        e_input *q = (e_input *)malloc(cap * sizeof *q);
        if (!q)
            return -1;
        for (k = 0; k < e->inbox_n; k++)
            q[k] = e->inbox[(e->inbox_head + k) % e->inbox_cap];
        free(e->inbox);
        e->inbox = q;
        e->inbox_cap = cap;
        e->inbox_head = 0;
    }
    e->inbox[(e->inbox_head + e->inbox_n) % e->inbox_cap] = in;
    e->inbox_n++;
    return 0;
}

/* Engine._run: `self.inbox.get_nowait()(p)` finche' ce n'e' */
static void panel_drain(dt_engine *e)
{
    while (e->inbox_n) {
        e_input in = e->inbox[e->inbox_head];
        e->inbox_head = (e->inbox_head + 1) % e->inbox_cap;
        e->inbox_n--;
        switch (in.kind) {
        case DT_PANEL_PRESS: mc_press(&e->pn, in.code); break;
        case DT_PANEL_RELEASE: mc_release(&e->pn, in.code); break;
        case DT_PANEL_ENCODER: mc_turn(&e->pn, in.code, in.arg); break;
        case DT_PANEL_PAD: mc_pad(&e->pn, in.code, in.arg); break;
        default: break;
        }
    }
}

/* screen(m) = panel.read(m, fb_back) e i LED (Engine._run, take_leds) */
static void panel_publish(dt_engine *e)
{
    uint8_t b[4], buf[1024];
    uint32_t ptr;
    int x, y, i;
    for (i = 0; i < 8; i++)
        e->leds_acc[i] &= e->pn.leds[i];
    e->leds_version++;
    if (!e->p->fb_back || e_read(e, e->p->fb_back, b, 4))
        return;
    ptr = e_be32(b);
    if (!(0x40000000u <= ptr && ptr < 0x50000000u) || e_read(e, ptr, buf, 1024))
        return;
    if (e->has_fb && !memcmp(buf, e->fb_raw, sizeof buf))
        return;
    memcpy(e->fb_raw, buf, sizeof buf);
    e->has_fb = 1;
    for (y = 0; y < 64; y++)
        for (x = 0; x < 128; x++)
            e->fb[y * 128 + x] = (uint8_t)((buf[x * 8 + (7 - y / 8)] >> (y % 8)) & 1);
    e->frames++;
    e->fb_crc = e_crc(0, buf, sizeof buf);
    e->panel_version++;
}

/* Un giro del thread del motore (Engine._run). -> 0, 1 fermato */
int dt_engine_chunk(dt_engine *e)
{
    int64_t executed = 0;
    e_cur = e;
    if (e->halted)
        return 1;
    panel_drain(e);
    if (engine_spin(e, e->slice ? e->slice : E_SLICE, &executed)) {
        e->instrs += executed;
        e->halted = 1;
        return 1;
    }
    e->instrs += executed;
    panel_publish(e);
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

/* ------------------------------------------------------------ uscite */
size_t dt_engine_midi_out(dt_engine *e, uint8_t *buf, size_t max)
{
    size_t n = e->midi.host_out.n < max ? e->midi.host_out.n : max;
    if (buf && n)
        memcpy(buf, e->midi.host_out.p, n);
    e_buf_drop(&e->midi.host_out, buf ? n : e->midi.host_out.n);
    return n;
}

void dt_engine_set_audio_sink(dt_engine *e, void (*sink)(void *ctx, const float *lr, size_t frames),
                              void *ctx)
{
    e->sink = sink;
    e->sink_ctx = ctx;
}

void dt_engine_set_volume(dt_engine *e, double volume)
{
    e->volume = (float)volume;
}

int64_t dt_engine_audio_frames(dt_engine *e)
{
    return e->audio_frames;
}

uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64])
{
    if (out)
        memcpy(out, e->fb, sizeof e->fb);
    return e->panel_version;
}

uint64_t dt_engine_leds(dt_engine *e, uint8_t out[8], uint8_t acc[8])
{
    if (out)
        memcpy(out, e->pn.leds, 8);
    if (acc) {
        memcpy(acc, e->leds_acc, 8);
        memset(e->leds_acc, 0xFF, sizeof e->leds_acc);
    }
    return e->leds_version;
}

int64_t dt_engine_instrs(dt_engine *e)
{
    return e->instrs;
}

const char *dt_engine_status(dt_engine *e)
{
    return e->status;
}

/* la riga 'chunk' di tools/mccore_ref.py */
int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n)
{
    int64_t frc = 0, dma[4], reads = 0, adc = 0;
    int k;
    for (k = 0; k < 192; k++)
        frc += e->frc.delivered[k];
    for (k = 0; k < 8; k++) {
        reads += e->pn.reads[k];
        adc += e->pn.adc_reads[k];
    }
    mc_dma_counters(e->dma, dma);
    return snprintf(buf, n,
                    "%s %lld instrs=%lld pc=%08x pit=%lld/%lld/%lld:%lld/%lld/%lld dtim3=%lld:%lld "
                    "ssi=%lld/%lld/%lld/%lld frc=%lld/%lld idle=%lld uart=%llu:%08x midi=%lld/%lld/%lld "
                    "audio=%lld:%08x fb=%lld:%08x leds=%08x scan=%lld/%lld/%lld sd=%lld/%lld dma=%lld/%lld "
                    "i2c=%02x/%02x poll=%lld dsp=%lld/%lld/%lld stat=%s",
                    tag, (long long)chunk, (long long)e->instrs, e_rd(e, UC_M68K_REG_PC),
                    (long long)e->tim[0].t.fired[0], (long long)e->tim[0].t.fired[2],
                    (long long)e->tim[0].t.fired[3], (long long)e->tim[0].t.missed[0],
                    (long long)e->tim[0].t.missed[2], (long long)e->tim[0].t.missed[3],
                    (long long)e->tim[1].t.fired[3], (long long)e->tim[1].t.missed[3],
                    (long long)e->sst.requests, (long long)e->sst.vector170, (long long)e->sst.vector191,
                    (long long)e->sst.tx_bytes, (long long)frc, (long long)e->frc.at_rte,
                    (long long)e->idle.n, (unsigned long long)e->uart_out_total, e->uart_out_crc,
                    (long long)e->midi.tx_sent, (long long)e->midi.rx_bytes, (long long)e->midi.uart_irqs,
                    (long long)e->tx_bytes_out, e->tx_crc, (long long)e->frames, e->fb_crc,
                    e_crc(0, e->pn.leds, 8), (long long)e->pn.scans, (long long)reads, (long long)adc,
                    (long long)e->sd.log_n, (long long)e->sd.dma_bytes, (long long)dma[0], (long long)dma[1],
                    e->i2c.cr, e->i2c.sr, (long long)e->pit1_polls, (long long)e->dsp_polls,
                    (long long)e->dsp_words, (long long)e->dsp_bursts, e->halted ? "halted" : "ok");
}

dt_machine *dt_engine_machine(dt_engine *e)
{
    return e->m;
}
