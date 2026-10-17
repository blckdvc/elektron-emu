/* Il motore della GUI v2 della Syntakt con --audio (emu/gui.py Emulator con
 * emu/cosim.py CoSim: le due ColdFire in lock-step, la catena audio del
 * firmware), in C, senza Python. Implementa l'API del motore di dtcore
 * (digitakt/emulator/core/include/dtcore.h: dt_engine_*), cosi' il tempo
 * reale (rt.c), la macchina (machine.c), il firmware (fw.c) e l'adattatore
 * AU del Digitakt si riusano cosi' come sono. Piano e verifiche:
 * docs/AU-PORT.md.
 *
 * Una sola unita' di traduzione: il C nativo che usa anche il Python
 * (native/st_hooks.c, st_spin.c, st_cosim.c: lo stesso testo che
 * emu/nativehooks.py compila) e poi le parti st_*.inc.c. Gli hook si
 * registrano nell'ordine del Python (scratch/st_hooktopo.py); il riferimento
 * e' tools/stcore_guiref.py e `stcore play` deve dare la sua stessa traccia. */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

#include "native/st_hooks.c"
#include "native/st_spin.c"
#include "native/st_cosim.c"

#include "dtcore.h"
#include "internal.h"
#include "machine.h"
#include "sha256.h"
#include "stcore.h"
#include "st_dma.h"

#include <stdlib.h>
#include <string.h>
#include <zlib.h>
#if !defined(_WIN32)
#include <pthread.h>
#endif
#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#endif

typedef struct dt_engine dt_engine;
#include "st_profiles.inc.c"
#include "st_base.inc.c"
#include "st_build.inc.c"
#include "st_timers.inc.c"
#include "st_periph.inc.c"
#include "st_dsp.inc.c"
#include "st_panel.inc.c"
#include "st_state.inc.c"

#define E_BUDGET 400000
#define E_IDLE_YIELD 20000
#define E_AUDIO_IPS 150000000
#define E_TXCAP (1 << 20)

static int verify(st_cpu *c, uint32_t addr, const uint8_t *want, size_t n)
{
    uint8_t b[64];
    return n <= sizeof b && !c_read(c, addr, b, n) && !memcmp(b, want, n);
}

/* ---------------------------------------------------------- idle slow */
static void idle_slow_c(int64_t n)
{
    dt_engine *e = e_cur;
    tx_deliver(MAINC(e), &e->etx);
    if (n % E_IDLE_YIELD == 0)
        c_raise(MAINC(e), 32, -1);
}

/* Il thread di emulazione di rt.c in tempo reale, come quello della GUI
   (emu/gui.py time_constraint_thread): macOS THREAD_TIME_CONSTRAINT_POLICY,
   periodo 10 ms, calcolo 5 ms. Con la sola QoS interattiva il thread restava
   pronto ma fermo per decine di ms dietro gli altri thread (e sui core E il
   motore delle due CPU non tiene il tempo reale): buchi nell'audio. Si chiede
   solo per un thread diverso da quello che ha creato il motore (rt.c), una
   volta per thread; DT2_RT_CONSTRAINT=0 lo spegne. */
static _Thread_local int st_thread_tuned;

static void st_tune_thread(dt_engine *e)
{
    const char *env;
    st_thread_tuned = 1;
    env = getenv("DT2_RT_CONSTRAINT");
    if (env && !strcmp(env, "0"))
        return;
#if defined(__APPLE__)
    if (!pthread_equal(pthread_self(), e->creator)) {
        mach_timebase_info_data_t tb;
        thread_time_constraint_policy_data_t pol;
        mach_timebase_info(&tb);
#define ST_MS(ms) ((uint32_t)((ms) * 1e6 * tb.denom / tb.numer))
        pol.period = ST_MS(10.0);
        pol.computation = ST_MS(5.0);
        pol.constraint = ST_MS(10.0);
        pol.preemptible = 1;
#undef ST_MS
        e->rt_tuned = thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY,
                                        (thread_policy_t)&pol, THREAD_TIME_CONSTRAINT_POLICY_COUNT) == KERN_SUCCESS;
    }
#else
    (void)e;
#endif
}

/* il DSP: la sezione 7 come "sezione 3" per machine.c (MOVEC e impronta) */
static void dsp_fwview(dt_firmware *v, const dt_section *s7)
{
    memset(v, 0, sizeof *v);
    v->count = 1;
    v->sec[0] = *s7;
    v->sec[0].id = 3;
}

/* ------------------------------------------------------------- setup */
/* La catena del boot freddo (st_boot.inc.c) costruisce un motore "freddo": lo
   stato di partenza e' un piolo della scala (nessun SSI, nessun interrupt
   forzato, nessun MIDI, nessuna porta FPGA, cadenza 4,68M, DSP dal reset), cioe'
   la CoSim di emu/cosim.py senza ST_AUDIO. Si chiede con questo flag, letto una
   volta da engine_new. */
static int st_cold_next;
/* il passaggio alla cadenza della GUI (ips -> 150M) lo fa la catena dopo il
   caricamento: lo stato di partenza ha ancora la cadenza dei timer di 4,68M */
static int st_legacy_ips_next;

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

static uint8_t *read_all(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *p = NULL;
    long n;
    if (!f)
        return NULL;
    if (fseek(f, 0, SEEK_END) == 0 && (n = ftell(f)) >= 0 && fseek(f, 0, SEEK_SET) == 0) {
        p = (uint8_t *)malloc(n ? (size_t)n : 1);
        if (p && fread(p, 1, (size_t)n, f) != (size_t)n) {
            free(p);
            p = NULL;
        }
        *len = (size_t)n;
    }
    fclose(f);
    return p;
}

/* il MAIN: Machine(), longrun.build fino al ripristino */
static int build_main(dt_engine *e, char *err, size_t errlen)
{
    st_cpu *c = MAINC(e);
    dt_machine_config mc = {0};
    const uint8_t *img = e->main_os->data;
    size_t img_len = e->main_os->len, off;
    int k;

    mc.idle_yield = 0;
    c->e = e;
    c->dsp = 0;
    c->m = dt_machine_new(e->fw, &mc, err, errlen);       /* MEM_INVALID, MOVEC, INTR */
    if (!c->m)
        return -1;
    c->m->no_mmio_at_load = 1;
    c->uc = c->m->uc;
    cpu_native_init(c);

    /* ---- longrun.build ---- */
    c_hook_code(c, (void *)h_flash_read, e, e->p->flash_read);
    c_hook_code(c, (void *)h_pend_call, e, e->p->pend_call);
    if (e->p->task_create)
        c_hook_code(c, (void *)h_task_create, e, e->p->task_create);
    c_hook_code(c, (void *)h_print, e, E_PRINT);
    if (e->p->set_pixel)
        c_hook_code(c, (void *)h_setpixel_count, e, e->p->set_pixel);
    if (e->p->px_copy)
        c_hook_code(c, (void *)h_pxcopy_count, e, e->p->px_copy);
    /* switch_to: tolto dalla GUI prima di partire (DT2_ST_SWITCH_TRACE) */
    c_hook_code(c, (void *)h_depack_clamp, e, e->p->depack_copy);
    /* edma: l'hook di SERQ (mux_write_hook, con tutti gli ascoltatori) e l'attesa */
    tx_init(&e->etx, 35, 155, &e->uart_out, &e->uart_out_total, &e->uart_out_crc);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_serq, e, E_SERQ, E_SERQ);
    c_hook_code(c, (void *)h_edma_wait, e, e->p->uart8_tx_wait);
    /* ssi (cablaggio del Rytm: SSI1, TCD 52/54, vettori 174/191) */
    if (!e->p->ssi0_dma_force_rte) {
        dt_errf(err, errlen, "SSI DMA model requires the force-ISR RTE symbol");
        return -1;
    }
    e->sst.mem_read = (mem_rw_fn)uc_mem_read;
    e->sst.mem_write = (mem_rw_fn)uc_mem_write;
    e->sst.rh = 48000;
    e->sst.ips = 4680000;
    e->sst.rx_ch = 52;
    e->sst.tx_ch = 54;
    e->sst.tx_vec = 174;
    e->sst.force_vec = 191;
    e->sst.level_retry = 1;
    e->sst.slack = 256;
    e->sst.level_retry_step = 8000;
    e->sst.txbuf = (uint8_t *)malloc(E_TXCAP);
    e->sst.txcap = E_TXCAP;
    e->sst.q = &c->q;
    if (!e->sst.txbuf)
        goto nomem;
    /* CINT e INTFRCH1: in C dopo (nativespin._own_ssi); il gate dell'RTE ora */
    e->ssi_gate.open = !e->cold;
    e->ssi_gate.cfn = dt2_sst_force_rte;
    e->ssi_gate.cdata = &e->sst;
    c_hook_code(c, (void *)dt2_gate_code, &e->ssi_gate, e->p->ssi0_dma_force_rte);
    /* le copie eDMA da memoria a memoria (emu/nativedma.py) */
    c->dma = st_dma_new();
    if (!c->dma)
        goto nomem;
    st_dma_install(c->uc, c->dma);
    /* dsp=True (fast=False): la linea ready della porta 0x8C000000 */
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_fifo_read, e, FIFO_STATUS, FIFO_STATUS + 1);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_fifo_write, e, FIFO_STATUS, FIFO_STATUS + 1);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_fifo_latch, e, FIFO_LATCH, FIFO_LATCH + 1);
    /* idle spin: in C dopo (replace_idle_spins), qui gli indirizzi */
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
                c_hook_code(c, (void *)h_softfloat, &e->sfs[k], addrs[k]);
            }
    }
    /* bitmap */
    if (e->p->set_pixel)
        c_hook_code(c, (void *)h_bmp_set, e, e->p->set_pixel);
    if (e->p->get_pixel)
        c_hook_code(c, (void *)h_bmp_get, e, e->p->get_pixel);
    /* unblock=False; frame_sem non risolto: niente gate del frame */
    /* UART8 */
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_uart8_read, e, E_USR8, E_UDR8 + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_uart8_write, e, E_USR8, E_UDR8 + 3);
    dtm_mmio_set(c->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(c->m, 0xEC03802Cu, 0x80000000u);
    /* esdhc (prima del ripristino) */
    sd_init(e);
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);
    return 0;
nomem:
    dt_errf(err, errlen, "out of memory");
    return -1;
}

/* il MAIN dopo restore_into: premap, board, intfrc, MIDI, install_mmio */
static int build_main_after(dt_engine *e, char *err, size_t errlen)
{
    st_cpu *c = MAINC(e);
    uint32_t a;
    for (a = 0x40000000u; a < 0x50000000u; a += 0x100000u)          /* premap_sdram */
        dtm_ensure(c->m, a);
    if (!e->cold)
        board_install_repair(c);
    /* intfrc: un gate su ogni RTE dell'immagine (dopo il ripristino: legge la RAM) */
    e->frc.q = &c->q;
    e->frc.retry = 8000;
    e->frc.skip[1] |= 1ull << 63;             /* vettore 191 (emu/ssi.py) */
    e->frc.dirty = 1;
    e->frc_gate.open = 1;
    e->frc_gate.cfn = dt2_frc_on_rte;
    e->frc_gate.cdata = &e->frc;
    if (!e->cold) {
        uint8_t *buf = (uint8_t *)malloc(0x40320000u - 0x40000400u);
        if (!buf) {
            dt_errf(err, errlen, "out of memory");
            return -1;
        }
        c_read(c, 0x40000400u, buf, 0x40320000u - 0x40000400u);
        for (a = 0; a + 1 < 0x40320000u - 0x40000400u; a += 2)
            if (buf[a] == 0x4E && buf[a + 1] == 0x73)
                c_hook_code(c, (void *)dt2_gate_code, &e->frc_gate, 0x40000400u + a);
        free(buf);
    }
    /* midi (DT2_MIDI=1) */
    if (!e->cold && mi_find_rx_handler(c, &e->midi.ring_base_ptr)) {
        e_midi *mi = &e->midi;
        mi->installed = 1;
        tx_init(&mi->tx, 37, 157, &mi->tx_out, &mi->tx_out_total, &mi->tx_out_crc);
        mi->hint = 1;
        mi->uimr = c_r8(c, MI_UIMR9);
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_mi_udr, e, MI_UDR9, MI_UDR9);
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_mi_uimr, e, MI_UIMR9, MI_UIMR9);
        c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_mi_usr, e, MI_USR9, MI_USR9);
        mi->uart_masked = 1;
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_mi_imr, e, MI_SIMR1, MI_CIMR1);
    }
    /* install_mmio */
    if (c->m->nmmio != 2) {
        dt_errf(err, errlen, "unexpected MMIO set in the MAIN state (%zu)", c->m->nmmio);
        return -1;
    }
    dtm_install_mmio(c->m);
    return 0;
}

/* la CPU DSP: dspcpu.build + prepare_timed, fino al ripristino */
static int build_dsp(dt_engine *e, char *err, size_t errlen)
{
    st_cpu *c = DSPC(e);
    dt_machine_config mc = {0};
    const uint8_t *img = e->dsp_os->data;
    size_t img_len = e->dsp_os->len;
    uint32_t a;
    long off;
    int k;
    static const uint32_t PITB[4] = {0xFC080000u, 0xFC084000u, 0xFC088000u, 0xFC08C000u};
    static const uint32_t DCH[4] = {0, 1, 2, 3};

    mc.idle_yield = 0;
    c->e = e;
    c->dsp = 1;
    dsp_fwview(&c->fwview, e->dsp_os);
    c->m = dt_machine_new(&c->fwview, &mc, err, errlen);  /* MEM_INVALID, MOVEC, INTR */
    if (!c->m)
        return -1;
    c->m->no_mmio_at_load = 1;
    c->uc = c->m->uc;
    cpu_native_init(c);
    for (a = 0x40000000u; a < 0x50000000u; a += 0x100000u)          /* premap_sdram */
        dtm_ensure(c->m, a);
    for (a = DSP_LOAD & ~0xFFFFFu; a < DSP_LOAD + img_len; a += 0x100000u)   /* m.load(img, LOAD) */
        dtm_ensure(c->m, a);
    c_write(c, DSP_LOAD, img, img_len);
    dtm_ensure(c->m, 0x80000000u);                                   /* SRAM */
    for (k = 0; k < 4; k++) {                                       /* install_pit_delay */
        dtm_ensure(c->m, PITB[k]);
        c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_pit_delay_read, (void *)(uintptr_t)PITB[k], PITB[k],
                   PITB[k] + 1);
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_pit_delay_write, NULL, PITB[k], PITB[k] + 1);
    }
    dtm_mmio_set(c->m, 0xEC09401Cu, 0x01800000u);
    dtm_ensure(c->m, DSP_DSPI1);                                     /* install_dspi1_fpga */
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_dspi1_sr, e, DSP_DSPI1 + 0x2C, DSP_DSPI1 + 0x2F);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_dspi1_push, e, DSP_DSPI1 + 0x34, DSP_DSPI1 + 0x37);
    c->dma = st_dma_new();                                           /* nativedma.install */
    if (!c->dma) {
        dt_errf(err, errlen, "out of memory");
        return -1;
    }
    st_dma_install(c->uc, c->dma);
    dtm_install_mmio(c->m);                                          /* m.install_mmio() */
    c_wr(c, UC_M68K_REG_SR, 0x2700);
    c_wr(c, UC_M68K_REG_A7, 0x80010000u);
    /* prepare_timed: memset HLE e i due parcheggi */
    off = find_bytes(img, img_len, SIG_BZERO, sizeof SIG_BZERO);
    if (off >= 0)
        c_hook_code(c, (void *)h_bzero, e, DSP_LOAD + (uint32_t)off);
    off = find_bytes(img, img_len, SIG_IDLE_WAIT, sizeof SIG_IDLE_WAIT);
    if (off < 0) {
        dt_errf(err, errlen, "DSP image without the wait-for-edge loop");
        return -1;
    }
    e->idle_flag = e_be32(img + off + 2);
    e->idle_wait = DSP_LOAD + (uint32_t)off;
    c_hook_code(c, (void *)h_park, e, e->idle_wait);
    off = find_bytes(img, img_len, SIG_AUDIO_IDLE, sizeof SIG_AUDIO_IDLE);
    if (off < 0) {
        dt_errf(err, errlen, "DSP image without the audio idle loop");
        return -1;
    }
    e->audio_idle = DSP_LOAD + (uint32_t)off + 10;
    c_hook_code(c, (void *)h_park_audio, e, e->audio_idle);
    /* Timers(Pits(m), Dtims(m, channels=(0, 1, 2, 3))): gli hook del Dtims */
    tsrc_init(c, &c->tim[0], 0, 3, (const uint32_t[]){3, 2, 0}, 4680000, 0);
    tsrc_init(c, &c->tim[1], 1, 4, DCH, 4680000, 0);
    dtim_arm_hooks(c, &c->tim[1]);
    dtcn_install(c, 4680000);
    dtim_rearm(c, &c->tim[1]);
    if (uc_set_block_budget(c->uc, (int64_t *)&c->stp) != UC_ERR_OK) {
        dt_errf(err, errlen, "uc_set_block_budget failed (Unicorn without the tb-budget patch)");
        return -1;
    }
    return 0;
}

static dt_engine *engine_new(const dt_firmware *fw, const char *state, const uint8_t *state_buf,
                             size_t state_len, const dt_engine_config *cfg, char *err, size_t errlen)
{
    dt_engine *e;
    st_cpu *m, *d;
    uint8_t sha[32];
    char hex[65];
    dt_sha256 h;
    size_t i;
    e_load L;
    uint8_t *file = NULL;
    const uint8_t *part[2];
    size_t plen[2];
    int r;

    e = (dt_engine *)calloc(1, sizeof *e);
    if (!e) {
        dt_errf(err, errlen, "out of memory");
        return NULL;
    }
    e_cur = e;
#if !defined(_WIN32)
    e->creator = pthread_self();
#endif
    e->cold = st_cold_next;
    e->legacy_ips = st_legacy_ips_next;
    st_cold_next = st_legacy_ips_next = 0;
    m = MAINC(e);
    d = DSPC(e);
    e->fw = fw;
    e->main_os = dt_fw_section(fw, 3);
    e->dsp_os = dt_fw_section(fw, 7);
    if (!e->main_os || !e->dsp_os) {
        dt_errf(err, errlen, "firmware without MAIN OS or DSP image (sections 3 and 7): not a Syntakt");
        goto fail;
    }
    dt_sha256_init(&h);
    dt_sha256_update(&h, e->main_os->data, e->main_os->len);
    dt_sha256_final(&h, sha);
    dt_sha256_hex(sha, hex);
    for (i = 0; i < ST_NPROFILES; i++)
        if (!strcmp(ST_PROFILES[i].main_sha256, hex))
            e->p = &ST_PROFILES[i];
    if (!e->p) {
        dt_errf(err, errlen, "unsupported firmware (MAIN OS %.16s...): no address profile", hex);
        goto fail;
    }
    e->dwell_chunks = cfg && cfg->panel_dwell ? cfg->panel_dwell : PN_DWELL_CHUNKS;
    e->volume = 0.8f;                         /* staudio.VOLUME */

    /* dspboot.build_flash: 16 MB, container a 0x80000 */
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000) {
        dt_errf(err, errlen, "cannot build the flash image");
        goto fail;
    }
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);

    if (state) {
        file = read_all(state, &state_len);
        if (!file) {
            dt_errf(err, errlen, "cannot read %s", state);
            goto fail;
        }
        state_buf = file;
    }
    if (stst_split(state_buf, state_len, part, plen, err, errlen))
        goto fail;

    /* ---- MAIN: longrun.build + restore_into ---- */
    if (build_main(e, err, errlen))
        goto fail;
    memset(&L, 0, sizeof L);
    L.e = e;
    r = dtm_load_state_mem(m->m, part[0], plen[0], main_record, &L, err, errlen);
    if (r) {
        free(L.ctim);
        goto fail;
    }
    if (!L.have_ssi || !L.have_tim || !L.have_etx || !L.have_sd) {
        dt_errf(err, errlen, "MAIN state without the GUI components (ssi0_dma, timers, edma_tx, esdhc)");
        free(L.ctim);
        goto fail;
    }
    if (build_main_after(e, err, errlen)) {
        free(L.ctim);
        goto fail;
    }

    /* ---- CoSim: restore_checkpoint_timers (Dtims costruito ora) ---- */
    r = timers_restore(m, L.ctim, L.ctim_len, err, errlen);
    free(L.ctim);
    L.ctim = NULL;
    if (r)
        goto fail;
    if (m->tim[0].t.nch != 3 || m->tim[1].t.nch != 1 || m->tim[1].t.ch[0] != 3) {
        dt_errf(err, errlen, "MAIN timers: expected Pits (3,2,0) and Dtims (3,)");
        goto fail;
    }
    dtim_arm_hooks(m, &m->tim[1]);
    dtcn_install(m, m->tim[1].ips);
    dtim_rearm(m, &m->tim[1]);
    /* nativehooks.install: lo stepper (tb-budget); le eccezioni sono di machine.c */
    if (uc_set_block_budget(m->uc, (int64_t *)&m->stp) != UC_ERR_OK) {
        dt_errf(err, errlen, "uc_set_block_budget failed (Unicorn without the tb-budget patch)");
        goto fail;
    }
    /* replace_idle_spins */
    e->idle.n = 0;
    e->idle.yield_every = E_IDLE_YIELD;
    e->idle.tx_pending = &e->etx.pending;
    e->idle.slow = idle_slow_c;
    e->idle.step = &m->stp;
    for (i = 0; i < (size_t)e->nidle; i++)
        c_hook_code(m, (void *)dt2_on_idle, &e->idle, e->idle_addrs[i]);
    /* FpgaPort (ST_AUDIO), riparazioni dello snapshot */
    if (!e->cold) {
        fpga_install(m);
        if (c_r8(m, 0x43BDD73Cu) != 0 && !(c_r16(m, 0xFC0455FEu) & 0x0080)) {
            dt_errf(err, errlen, "snapshot with eDMA 47 started but not done: needs CoSim._repair_pending_dma47");
            goto fail;
        }
    }
    e->sst.now = m->now;                                  /* ssi0.align(mt.now) */
    /* nativespin.install(m, mt, (ssi0, intc_force, midi)) */
    if (!e->cold) {
        c_hook_mem(m, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_cint, &e->sst, 0xFC04401Cu, 0xFC04401Cu);
        c_hook_mem(m, UC_HOOK_MEM_WRITE, (void *)dt2_sst_on_intfrch1, &e->sst, 0xFC04C010u, 0xFC04C013u);
    }
    if (!e->cold) {
        static const uint32_t ib[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
        int k;
        for (k = 0; k < 3; k++)
            c_hook_mem(m, UC_HOOK_MEM_WRITE, (void *)dt2_frc_on_write, &e->frc, ib[k] + 0x10, ib[k] + 0x17);
    }
    m->S.ntim = 2;
    m->S.tim[0] = &m->tim[0].t;
    m->S.tim[1] = &m->tim[1].t;
    m->S.sst = &e->sst;
    m->S.frc = &e->frc;
    m->S.nev = 0;
    if (!e->cold) {
        m->S.ev[m->S.nev++] = EV_SSI;
        m->S.ev[m->S.nev++] = EV_FRC;
    }
    if (e->midi.installed) {
        m->S.ev[m->S.nev++] = EV_MIDI;
        m->S.midi_hint = &e->midi.hint;
        m->S.midi_txpend = &e->midi.tx.pending;
        m->S.midi_step = mi_step;
        m->S.midi_service = mi_service;
    }

    /* ---- DspCompanion ---- */
    {
        uint8_t *mbox = (uint8_t *)calloc(1, MAILBOX_SIZE);
        if (!mbox || mailbox_share(m, MAIN_MAILBOX, mbox)) {
            free(mbox);
            dt_errf(err, errlen, "cannot share the mailbox");
            goto fail;
        }
        e->mailbox = mbox;
    }
    if (build_dsp(e, err, errlen))
        goto fail;
    memset(&L, 0, sizeof L);
    L.e = e;
    if (e->cold) {
        /* DspCompanion(m, dsp_snap=None): il DSP dal reset (build_dsp), PC all'ingresso */
        c_wr(d, UC_M68K_REG_PC, e_be32(e->dsp_os->data));
        L.have_tim = L.have_dsp = 1;
    } else {
        r = dtm_load_state_mem(d->m, part[1], plen[1], dsp_record, &L, err, errlen);
        if (r)
            goto fail;
    }
    if (!L.have_tim || !L.have_dsp) {
        dt_errf(err, errlen, "DSP state without timers or DSP component");
        goto fail;
    }
    if (d->tim[0].t.nch != 3 || d->tim[1].t.nch != 4 || d->tim[0].ips != 4680000 || d->tim[1].ips != 4680000) {
        dt_errf(err, errlen, "DSP timers: expected Pits (3,2,0) and Dtims (0,1,2,3) at 4.68M");
        goto fail;
    }
    if (d->m->nmmio != 1) {
        dt_errf(err, errlen, "unexpected MMIO set in the DSP state");
        goto fail;
    }
    if (mailbox_share(d, DSP_MAILBOX, e->mailbox)) {
        dt_errf(err, errlen, "cannot share the mailbox with the DSP");
        goto fail;
    }
    d->S.ntim = 2;
    d->S.tim[0] = &d->tim[0].t;
    d->S.tim[1] = &d->tim[1].t;
    d->S.nev = 0;
    e->last_edges = e->dtin0_edges;
    /* SpiLink: il lato DSP (il lato MAIN e' nell'hook di SERQ) */
    dtm_ensure(d->m, SL_DSPI0);
    c_hook_mem(d, UC_HOOK_MEM_WRITE, (void *)h_sl_push, e, SL_DSPI0 + 0x34, SL_DSPI0 + 0x37);
    c_hook_mem(d, UC_HOOK_MEM_READ, (void *)h_sl_sr, e, SL_DSPI0 + 0x2C, SL_DSPI0 + 0x2F);
    c_hook_mem(d, UC_HOOK_MEM_READ, (void *)h_sl_pop, e, SL_DSPI0 + 0x38, SL_DSPI0 + 0x3B);
    /* use_main_sync: la linea A4 del MAIN, in C */
    e->sync.level = c_r8(m, 0xEC094000u) & 0x10;
    e->sync.edges = 0;
    c_hook_mem(m, UC_HOOK_MEM_WRITE, (void *)dt2_sync_set, &e->sync, 0xEC094018u, 0xEC094018u);
    c_hook_mem(m, UC_HOOK_MEM_WRITE, (void *)dt2_sync_clr, &e->sync, 0xEC094024u, 0xEC094024u);
    /* _repair_lost_upload_end (solo con l'audio, CoSim.audio) */
    if (!e->cold && c_r32(m, 0x41BB22B0u) == 6 && e->mailbox[2] == 0 && e->mailbox[3] == 2 && e->mailbox[8] == 0
        && e->mailbox[9] == 0x0a) {
        e->mailbox[2] = 0;
        e->mailbox[3] = 3;
    }

    /* ---- Emulator.run ---- */
    stleds_load(e);
    if (e->p->mainloop)
        c_hook_code(m, (void *)h_mainloop, e, e->p->mainloop);
    if (e->p->job_pump)
        c_hook_code(m, (void *)h_job_pump, e, e->p->job_pump);
    c_hook_code(m, (void *)h_terminal, e, 0x4012D2FAu);
    if (e->p->panel_diff && e->p->fb_front)
        c_hook_code(m, (void *)h_panel_latch, e, e->p->panel_diff);
    if (!e->cold && !e->legacy_ips
        && (m->tim[0].ips != E_AUDIO_IPS || m->tim[1].ips != E_AUDIO_IPS || e->sst.ips != E_AUDIO_IPS)) {
        dt_errf(err, errlen, "session at %lld instructions per second: expected 150M (a GUI --audio session)",
                (long long)m->tim[0].ips);
        goto fail;
    }
    e->midi.ips = E_AUDIO_IPS;                /* _pending_ips (0, AUDIO_IPS): gia' cosi' per i timer */
    /* il frame in mostra al resume: fb_back */
    if (e->p->fb_back && panel_read(e, e->p->fb_back, e->latch))
        e->has_latch = 1;
    /* il Cosim C */
    e->Z.ucm = m->uc;
    e->Z.ucd = d->uc;
    e->Z.Sm = &m->S;
    e->Z.Sd = &d->S;
    e->Z.sync = &e->sync;
    e->Z.link_complete = &e->link_complete;
    e->Z.park = &e->park;
    e->Z.dstop = &d->m->stop_flag;
    e->Z.slice = ST_SLICE;
    e->Z.audio_idle = e->audio_idle;
    e->pc = c_rd(m, UC_M68K_REG_PC);
    e->dpc = c_rd(d, UC_M68K_REG_PC);
    free(file);
    snprintf(e->status, sizeof e->status, "running");
    return e;
fail:
    free(file);
    dt_engine_free(e);
    return NULL;
}

void dt_engine_free(dt_engine *e)
{
    int k;
    if (!e)
        return;
    for (k = 0; k < 2; k++) {
        if (e->cpu[k].m)
            dt_machine_free(e->cpu[k].m);
        free(e->cpu[k].dma);
    }
    free(e->mailbox);
    free(e->flash);
    free(e->sst.txbuf);
    free(e->uart_out.p);
    free(e->uart_in.p);
    ov_free(&e->sd.ov);
    free(e->midi.rx.p);
    free(e->midi.tx_out.p);
    free(e->midi.host_out.p);
    free(e->link_main_tx.p);
    free(e->link_pending.p);
    free(e->link_dsp_rx.p);
    free(e->fpga_config.p);
    free(e->inbox);
    if (e_cur == e)
        e_cur = NULL;
    free(e);
}

/* Un giro del ciclo del worker della GUI (Emulator.run). -> 0, 1 fermato */
int dt_engine_chunk(dt_engine *e)
{
    int64_t executed = 0;
    e_cur = e;
    if (!st_thread_tuned)
        st_tune_thread(e);
    if (e->halted)
        return 1;
    e->pc = panel_drain(e, e->pc);
    if (engine_cosim(e, E_BUDGET, &executed)) {
        e->instrs += executed;
        e->halted = 1;
        return 1;
    }
    e->instrs += executed;
    panel_publish(e);
    if (e->uart_out.n) {
        stleds_feed(&e->leds, e->uart_out.p, e->uart_out.n);
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

/* il +Drive via RPC (emu/transfer.py) non e' installato sulla Syntakt: gli
   indirizzi di Transfer sono quelli del Digitakt II e la verifica li scarta */
int dt_engine_rpc(dt_engine *e, const uint8_t *payload, size_t n)
{
    (void)e; (void)payload; (void)n;
    return DT_ERR_UNSUPPORTED;
}

size_t dt_engine_rpc_replies(dt_engine *e, uint8_t *buf, size_t max)
{
    (void)e; (void)buf; (void)max;
    return 0;
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

/* sink(ctx, mix, tracks, n) a blocchi di al piu' 512 campioni a 48 kHz:
   mix = Main L (staudio.TxOut), tracks[0] = Main R, tracks[1..2] = uscita
   principale senza il blocco Analog FX (slot 5/3), tracks[3..4] = la mandata
   all'Analog FX (slot 7/2); tutte col Main Volume */
void dt_engine_set_audio_sink(dt_engine *e, void (*sink)(void *ctx, const float *mix, const float *tracks,
                                                         size_t n),
                              void *ctx, int tracks)
{
    (void)tracks;
    e->sink = sink;
    e->sink_ctx = ctx;
}

void dt_engine_set_master(dt_engine *e, double master)
{
    if (master < 0)
        return;
    e->volume = (float)master;                /* staudio.VOLUME, 0..1 (GUI v2 VOLUME_MAX) */
}

int64_t dt_engine_audio_samples(dt_engine *e)
{
    return e->samples;
}

uint64_t dt_engine_screen(dt_engine *e, uint8_t out[128 * 64])
{
    if (out)
        memcpy(out, e->fb, sizeof e->fb);
    return e->panel_version;
}

uint64_t dt_engine_leds(dt_engine *e, uint8_t out[50])
{
    int k;
    if (out)
        for (k = 0; k < 50; k++)
            out[k] = stleds_lit(&e->leds, k);
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

/* la riga 'chunk' di tools/stcore_guiref.py */
int dt_engine_trace(dt_engine *e, const char *tag, int64_t chunk, char *buf, size_t n)
{
    st_cpu *m = MAINC(e), *d = DSPC(e);
    int64_t frc = 0;
    uint8_t lit[ST_NLEDS], box[64];
    int k;
    for (k = 0; k < 192; k++)
        frc += e->frc.delivered[k];
    for (k = 0; k < ST_NLEDS; k++)
        lit[k] = stleds_lit(&e->leds, k);
    memcpy(box, e->mailbox, sizeof box);
    return snprintf(buf, n,
                    "%s %lld instrs=%lld pc=%08x dpc=%08x pit=%lld/%lld/%lld:%lld/%lld/%lld dtim3=%lld:%lld "
                    "ssi=%lld/%lld/%lld/%lld frc=%lld/%lld idle=%lld uart=%llu:%08x midi=%lld/%lld/%lld "
                    "tx=%llu:%08x audio=%lld:%08x fb=%lld:%08x leds=%lld:%08x sd=%lld/%lld "
                    "link=%lld/%lld/%lld irq4=%lld edges=%lld dtim=%lld/%lld/%lld/%lld fpga=%lld/%llu "
                    "mbox=%08x stat=%s",
                    tag, (long long)chunk, (long long)e->instrs, e->pc, e->dpc,
                    (long long)m->tim[0].t.fired[0], (long long)m->tim[0].t.fired[2],
                    (long long)m->tim[0].t.fired[3], (long long)m->tim[0].t.missed[0],
                    (long long)m->tim[0].t.missed[2], (long long)m->tim[0].t.missed[3],
                    (long long)m->tim[1].t.fired[3], (long long)m->tim[1].t.missed[3],
                    (long long)e->sst.requests, (long long)e->sst.vector170, (long long)e->sst.vector191,
                    (long long)e->sst.tx_bytes, (long long)frc, (long long)e->frc.at_rte,
                    (long long)e->idle.n, (unsigned long long)e->uart_out_total, e->uart_out_crc,
                    (long long)e->midi.tx_sent, (long long)e->midi.rx_bytes, (long long)e->midi.uart_irqs,
                    (unsigned long long)e->tx_total, e->tx_crc, (long long)e->samples, e->audio_crc,
                    (long long)e->frames, e->fb_crc, (long long)e->leds.version, e_crc(0, lit, sizeof lit),
                    (long long)e->sd.log_n, (long long)e->sd.dma_bytes, (long long)e->link_sent,
                    (long long)e->link_received, (long long)e->link_dma_done, (long long)e->irq4,
                    (long long)e->dtin0_edges, (long long)d->tim[1].t.fired[0], (long long)d->tim[1].t.fired[1],
                    (long long)d->tim[1].t.fired[2], (long long)d->tim[1].t.fired[3],
                    (long long)e->fpga_sent, (unsigned long long)e->fpga_config_total,
                    e_crc(0, box, sizeof box), e->halted ? "halted" : "ok");
}

dt_machine *dt_engine_machine(dt_engine *e)
{
    return e->cpu[0].m;
}

#include "st_boot.inc.c"

/* ------------------------------------------------------- stcore.h */
dt_machine *st_engine_dsp(dt_engine *e)
{
    return e->cpu[1].m;
}

void st_engine_pcs(dt_engine *e, uint32_t *pc, uint32_t *dpc)
{
    *pc = e->pc;
    *dpc = e->dpc;
}

void st_engine_sync_pc(dt_engine *e)
{
    c_wr(MAINC(e), UC_M68K_REG_PC, e->pc);
    c_wr(DSPC(e), UC_M68K_REG_PC, e->dpc);
}

const char *st_version(void)
{
    return STCORE_VERSION;
}
