/* st_engine.c, parte 8: il boot freddo dal firmware, senza stato iniziale
 * (docs/AU-PORT.md, "Boot freddo in C"). Il Python che porta e' la catena
 *
 *   emu/dspboot.py run + emu/checkpoint.py make   (MAIN dal reset: gradino 60M)
 *   emu/cosim.py CoSim                            (MAIN + DSP dal reset)
 *   tools/uiscript.py                             (il dialogo "RUN WARM-UP
 *                                                  CALIBRATION" si chiude con NO)
 *   emu/gui.py Emulator --audio                   (la cadenza a 150M, l'SSI)
 *
 * Questo file e' incluso da st_engine.c dopo st_state.inc.c: usa i modelli e i
 * salvataggi del motore (eSDHC, hook di build, save_machine), niente di
 * derivato dal firmware: tutto viene dal .syx dell'utente, a runtime. */

/* ------------------------------------------------------------ avanzamento */
/* 0..100, per la UI: lo scrive la catena, lo legge chi vuole (anche da un
   altro thread). Una sola catena alla volta per processo. */
static volatile int st_boot_progress_pct;

int st_coldboot_progress(void)
{
    return st_boot_progress_pct;
}

#define BOOT_PPDSDR_C 0xEC09401Au
#define BOOT_PPDSDR_D 0xEC09401Bu
#define BOOT_PCLRR_D 0xEC094027u

/* emu/panelmcu.py: la UART8 verso il micro del pannello nel boot freddo */
#define PM_USR8 0xEC070004u
#define PM_UDR8 0xEC07000Cu
#define PM_RXRDY 0x01
#define PM_TXRDY 0x04
#define PM_TXEMP 0x08

typedef struct {
    dt_engine *e;
    uint64_t n, point;
    int64_t spin;
    int driven;                   /* emu/gpio.py SdGate */
    volatile int *cancel;
    int cancelled;
    /* PanelMcu('syntakt'): una sola richiesta in attesa di risposta */
    uint8_t rx[16];
    int nrx;
    uint8_t txlast[4];
    uint64_t ntx;
    int reply_left;
} st_boot;

static void hb_cover(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    st_boot *b = (st_boot *)ud;
    (void)a; (void)s;
    b->n++;
    if ((b->n & 0xFFFFF) == 0) {
        if (b->cancel && *b->cancel) {
            b->cancelled = 1;
            uc_emu_stop(uc);
            b->e->cpu[0].m->stop_flag = 1;
            return;
        }
        st_boot_progress_pct = (int)(10 * b->n / b->point);       /* il boot MAIN: 0..10 */
    }
    if (b->n >= b->point) {
        uc_emu_stop(uc);
        b->e->cpu[0].m->stop_flag = 1;
    }
}

static void hb_pre_isa(void *uc, void *ctx)
{
    uc_hook h;
    uc_hook_add((uc_engine *)uc, &h, UC_HOOK_CODE, (void *)hb_cover, ctx, 1, 0);
}

static void hb_halt(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    st_boot *b = (st_boot *)ud;
    (void)uc; (void)a; (void)s;
    b->spin++;
    if (b->spin % 20000 == 0)
        dtm_raise_vector(MAINC(b->e)->m, 32, 0, -1);
}

/* emu/gpio.py SdGate */
static void hb_gate_set(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (v & 0x10)
        ((st_boot *)ud)->driven = 1;
}

static void hb_gate_clear(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (!(v & 0x10))
        ((st_boot *)ud)->driven = 0;
}

static void hb_gate_sense(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    st_boot *b = (st_boot *)ud;
    st_cpu *c = MAINC(b->e);
    uint8_t cur = (uint8_t)(c_r8(c, BOOT_PPDSDR_C) & ~0x08);
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    c_w8(c, BOOT_PPDSDR_C, (uint8_t)(cur | (b->driven ? 0x08 : 0)));
}

/* Esdhc._on_serq nel boot: solo il canale 59 (il mux completo e' di longrun) */
static void hb_sd_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (!(v & 0x40) && (v & 0x3F) == SD_DMA_CHAN)
        ((st_boot *)ud)->e->sd.armed = SD_DMA_CHAN;
}

/* PanelMcu: _poke scrive un byte; se la pagina non c'e' (ancora) l'errore e' ignorato */
static void hb_pm_poke(st_boot *b, uint32_t addr, uint8_t v)
{
    uc_mem_write(MAINC(b->e)->uc, addr, &v, 1);
}

static uint8_t hb_pm_status(st_boot *b)
{
    return (uint8_t)(PM_TXRDY | PM_TXEMP | (b->nrx ? PM_RXRDY : 0));
}

static void hb_pm_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    st_boot *b = (st_boot *)ud;
    (void)uc; (void)ty; (void)size; (void)v;
    if (a >= PM_USR8 && a < PM_USR8 + 4) {
        hb_pm_poke(b, PM_USR8, hb_pm_status(b));
    } else {
        uint8_t x = 0;
        if (b->nrx) {
            x = b->rx[0];
            memmove(b->rx, b->rx + 1, (size_t)--b->nrx);
        }
        hb_pm_poke(b, PM_UDR8, x);
        hb_pm_poke(b, PM_USR8, hb_pm_status(b));
    }
}

static void hb_pm_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    static const uint8_t REQ[4] = {0x60, 0x02, 0x70, 0x00};            /* Syntakt OS 1.41: versione del pannello */
    static const uint8_t REPLY[5] = {0x70, 0x0b, 0xff, 0x00, 0x02};
    st_boot *b = (st_boot *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
    memmove(b->txlast, b->txlast + 1, 3);
    b->txlast[3] = (uint8_t)v;
    b->ntx++;
    if (b->reply_left && b->ntx >= 4 && !memcmp(b->txlast, REQ, 4)) {
        memcpy(b->rx + b->nrx, REPLY, 5);
        b->nrx += 5;
        b->reply_left = 0;
    }
    hb_pm_poke(b, PM_USR8, hb_pm_status(b));
}

/* ------------------------------------------------------------ lo stato */
/* il componente CSDH (l'overlay dell'eMMC emulata), come lo scrive dt_engine_save_mem */
static int boot_rec_csdh(dt_engine *e, e_buf *out, uint32_t *nrec)
{
    e_esdhc *sd = &e->sd;
    e_buf b = {0};
    e_sector **v = (e_sector **)malloc((sd->ov.n ? sd->ov.n : 1) * sizeof *v);
    size_t n = 0, j;
    int rc;
    if (!v)
        return -1;
    for (j = 0; j < sd->ov.cap; j++)
        if (sd->ov.tab[j].used)
            v[n++] = &sd->ov.tab[j];
    qsort(v, n, sizeof *v, cmp_sec);
    w_le32(&b, sd->pattern);
    w_le32(&b, (uint32_t)sd->armed);
    w_le64(&b, (uint64_t)sd->dma_bytes);
    w_le64(&b, (uint64_t)sd->blocks);
    w_le32(&b, sd->rca);
    w_u8(&b, (uint8_t)!!sd->selected);
    w_le32(&b, (uint32_t)n);
    for (j = 0; j < n; j++) {
        w_le64(&b, v[j]->sec);
        w_u8(&b, v[j]->mask ? 1 : 0);
        e_buf_put(&b, v[j]->data, 512);
        if (v[j]->mask)
            e_buf_put(&b, v[j]->mask, 512);
    }
    free(v);
    /* il lettore (main_record, st_state.inc.c) rifiuta un CSDH di meno di 37 byte, ma
       senza settori scritti il record ne ha 33: quattro zeri in coda, che il
       lettore ignora (lo stato del boot a 60M non ha ancora scritto sull'eMMC) */
    while (b.n < 37)
        w_u8(&b, 0);
    rc = w_rec(out, nrec, "CSDH", &b);
    free(b.p);
    return rc;
}

/* un DTST (non compresso) vuoto: l'intestazione, i record seguono; il numero
   dei record si scrive alla fine (offset 8) */
static int dtst_begin(e_buf *out)
{
    return e_buf_put(out, "DTST", 4) || w_le32(out, 1) || w_le32(out, 0) || w_le32(out, 0);
}

static void dtst_end(e_buf *out, uint32_t nrec)
{
    out->p[8] = (uint8_t)nrec;
    out->p[9] = (uint8_t)(nrec >> 8);
    out->p[10] = (uint8_t)(nrec >> 16);
    out->p[11] = (uint8_t)(nrec >> 24);
}

/* ------------------------------------------------------------ MAIN dal reset */
/* Il boot freddo del MAIN, emu/dspboot.py run (fast=True) fino all'istruzione
   `point` (emu/checkpoint.py make: lo stato si fotografa PRIMA di eseguirla).
   -> il DTST del MAIN, non compresso (REGS ... MAPS PAGE CSDH FWID), come un
   piolo della scala (snapshots/<fw>/boot60M.snap). */
static int boot_main(const dt_firmware *fw, uint64_t point, volatile int *cancel, e_buf *dtst, char *err,
                     size_t errlen)
{
    dt_engine *e = (dt_engine *)calloc(1, sizeof *e);
    st_boot *b = (st_boot *)calloc(1, sizeof *b);
    st_cpu *c;
    dt_machine_config mc;
    uint8_t sha[32];
    char hex[65];
    dt_sha256 h;
    size_t i, off;
    uint32_t nrec = 0;
    int r = DT_ERR_NOMEM;
    const uint8_t *img;
    e_buf out = {0};

    if (!e || !b)
        goto out;
    e_cur = e;
    c = MAINC(e);
    b->e = e;
    b->point = point;
    b->cancel = cancel;
    b->reply_left = 1;
    e->fw = fw;
    e->main_os = dt_fw_section(fw, 3);
    e->dsp_os = dt_fw_section(fw, 7);
    if (!e->main_os || !e->dsp_os) {
        dt_errf(err, errlen, "firmware without MAIN OS or DSP image (sections 3 and 7): not a Syntakt");
        r = DT_ERR_UNSUPPORTED;
        goto out;
    }
    img = e->main_os->data;
    dt_sha256_init(&h);
    dt_sha256_update(&h, img, e->main_os->len);
    dt_sha256_final(&h, sha);
    dt_sha256_hex(sha, hex);
    for (i = 0; i < ST_NPROFILES; i++)
        if (!strcmp(ST_PROFILES[i].main_sha256, hex))
            e->p = &ST_PROFILES[i];
    if (!e->p) {
        dt_errf(err, errlen, "unsupported firmware (MAIN OS %.16s...): no address profile (only Syntakt OS 1.41)",
                hex);
        r = DT_ERR_UNSUPPORTED;
        goto out;
    }
    /* dspboot.build_flash: 16 MB, il container a 0x80000 */
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000) {
        dt_errf(err, errlen, "cannot build the flash image");
        goto out;
    }
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);

    memset(&mc, 0, sizeof mc);
    mc.idle_yield = 0;
    mc.pre_isa = hb_pre_isa;
    mc.pre_isa_ctx = b;
    c->e = e;
    c->dsp = 0;
    c->m = dt_machine_new(fw, &mc, err, errlen);
    if (!c->m) {
        r = DT_ERR_UNICORN;
        goto out;
    }
    c->m->no_mmio_at_load = 1;
    c->uc = c->m->uc;

    /* emu/board.py install (la revisione della scheda a 0x0): NON qui. La scala
       degli snapshot e la catena che ha fatto ui_ready sono nate prima del
       registro (il firmware lo legge come 0 e il controllo di avvio dichiara
       "TOO OLD HARDWARE", ma il collegamento col DSP si completa); il registro
       entra al passaggio all'audio, come fa longrun.build su ogni snapshot
       (board_install_repair: tolto anche il blocco che il controllo aveva alzato) */
    /* gli hook di dspboot.run (fast=True) che cambiano lo stato; call_sites,
       task_create_sites e task_start sono osservazioni (nessun effetto) */
    c_hook_code(c, (void *)h_flash_read, e, e->p->flash_read);
    c_hook_code(c, (void *)h_depack_clamp, e, e->p->depack_copy);
    c_hook_code(c, (void *)h_pend_call, e, e->p->pend_call);
    for (off = 0; off + 1 < e->main_os->len; off += 2)
        if (img[off] == 0x60 && img[off + 1] == 0xFE)
            c_hook_code(c, (void *)hb_halt, b, DT_MAIN_LOAD + (uint32_t)off);
    dtm_mmio_set(c->m, 0xFC05C02Cu, 0x100000F0u);                    /* DSPI0 SR */
    dtm_mmio_set(c->m, 0xEC03802Cu, 0x80000000u);
    /* SdGate */
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)hb_gate_set, b, BOOT_PPDSDR_D, BOOT_PPDSDR_D);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)hb_gate_clear, b, BOOT_PCLRR_D, BOOT_PCLRR_D);
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)hb_gate_sense, b, BOOT_PPDSDR_C, BOOT_PPDSDR_C);
    /* Esdhc(card 256 MB, status/semafori del profilo) */
    sd_init(e);
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)hb_sd_serq, b, E_SERQ, E_SERQ);
    /* PanelMcu('syntakt') */
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)hb_pm_read, b, PM_USR8, PM_USR8 + 3);
    c_hook_mem(c, UC_HOOK_MEM_READ, (void *)hb_pm_read, b, PM_UDR8, PM_UDR8 + 3);
    c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)hb_pm_write, b, PM_UDR8, PM_UDR8 + 3);
    hb_pm_poke(b, PM_USR8, hb_pm_status(b));                          /* pagina non ancora mappata: nessun effetto */
    dtm_install_mmio(c->m);
    /* Machine.load + premap_sdram + reset */
    for (off = 0; off < e->main_os->len; off += 0x100000)
        dtm_ensure(c->m, DT_MAIN_LOAD + off);
    dtm_ensure(c->m, DT_MAIN_LOAD);
    c_write(c, DT_MAIN_LOAD, img, e->main_os->len);
    {
        uint32_t a;
        for (a = 0x40000000u; a < 0x50000000u; a += 0x100000u)
            dtm_ensure(c->m, a);
    }
    dtm_ensure(c->m, 0x40800000u);
    c_wr(c, UC_M68K_REG_SR, 0x2700);
    c_wr(c, UC_M68K_REG_A7, 0x40800000u);
    c->m->state_loaded = 1;
    {
        uc_err ue = uc_emu_start(c->uc, e->p->entry, 0, 0, (size_t)(point + 1000000));
        if (b->cancelled) {
            dt_errf(err, errlen, "interrotto");
            r = DT_ERR_IO;
            goto out;
        }
        if (ue != UC_ERR_OK || b->n < point) {
            dt_errf(err, errlen, "cold boot stopped at %llu instructions: %s", (unsigned long long)b->n,
                    uc_strerror(ue));
            r = DT_ERR_UNICORN;
            goto out;
        }
    }
    e->pc = c_rd(c, UC_M68K_REG_PC);
    /* il DTST del MAIN: i record della macchina e l'eMMC */
    if (dtst_begin(&out))
        goto out;
    if (save_machine(c, e->pc, sha, &out, &nrec) || boot_rec_csdh(e, &out, &nrec))
        goto out;
    dtst_end(&out, nrec);
    *dtst = out;
    out.p = NULL;
    r = DT_OK;
out:
    if (e) {
        int k;
        for (k = 0; k < 2; k++)
            if (e->cpu[k].m)
                dt_machine_free(e->cpu[k].m);
        free(e->flash);
        ov_free(&e->sd.ov);
        if (e_cur == e)
            e_cur = NULL;
        free(e);
    }
    free(b);
    free(out.p);
    return r;
}

/* il contenitore STST di MAIN + una CPU DSP vuota (DTST senza record): la
   forma che legge engine_new, con il DSP da costruire dal reset */
static int stst_wrap(const e_buf *main_dtst, const e_buf *dsp_dtst, e_buf *dst)
{
    static const uint8_t EMPTY[16] = {'D', 'T', 'S', 'T', 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (e_buf_put(dst, "STST", 4) || w_le32(dst, 1) || w_le32(dst, 2) || w_le32(dst, 0))
        return -1;
    if (w_le64(dst, main_dtst->n) || e_buf_put(dst, main_dtst->p, main_dtst->n))
        return -1;
    if (dsp_dtst) {
        if (w_le64(dst, dsp_dtst->n) || e_buf_put(dst, dsp_dtst->p, dsp_dtst->n))
            return -1;
    } else if (w_le64(dst, sizeof EMPTY) || e_buf_put(dst, EMPTY, sizeof EMPTY)) {
        return -1;
    }
    return 0;
}

/* diagnostica: solo il boot del MAIN fino a `point` istruzioni, in uno STST
   (MAIN + DSP vuoto) confrontabile con tools/snap2dtst.py di un piolo della
   scala (tools/dtst_diff.py) */
int st_coldboot_main(const dt_firmware *fw, uint64_t point, uint8_t **out, size_t *len, volatile int *cancel,
                     char *err, size_t errlen)
{
    e_buf m = {0}, st = {0};
    int r;
    *out = NULL;
    *len = 0;
    st_boot_progress_pct = 0;
    r = boot_main(fw, point, cancel, &m, err, errlen);
    if (r)
        return r;
    if (stst_wrap(&m, NULL, &st)) {
        free(m.p);
        free(st.p);
        return DT_ERR_NOMEM;
    }
    free(m.p);
    *out = st.p;
    *len = st.n;
    st_boot_progress_pct = 100;
    return DT_OK;
}

/* ============================================================ componenti freschi */
/* aggiunge un record al DTST (e ne aggiorna il conteggio nell'intestazione) */
static int dtst_add(e_buf *d, const char *tag, const e_buf *body)
{
    uint32_t n = e_le32(d->p + 8);
    if (e_buf_put(d, tag, 4) || w_le32(d, 0) || w_le64(d, body ? body->n : 0)
        || (body && body->n && e_buf_put(d, body->p, body->n)))
        return -1;
    n++;
    d->p[8] = (uint8_t)n; d->p[9] = (uint8_t)(n >> 8); d->p[10] = (uint8_t)(n >> 16); d->p[11] = (uint8_t)(n >> 24);
    return 0;
}

/* Il piolo della scala e' un file "vecchio": non ha i registri EMAC, e il
   ripristino del Python (emu/snapshot.py, legacy_macsr) rimette solo MACSR =
   0x20 (il modo frazionario che il firmware asserisce, sm/fade.c) e lascia gli
   altri ai valori della macchina nuova. Il boot in C ha MACSR = 0 e gli stessi
   zeri negli altri: il record EMAC del piolo ha dunque la sola MACSR. */
static int dtst_legacy_emac(e_buf *d)
{
    /* id UC_M68K_REG_MACSR (0x22) u32, u32 0, valore u64 0x20 (little endian) */
    static const uint8_t ENT[16] = {0x22, 0, 0, 0, 0, 0, 0, 0, 0x20, 0, 0, 0, 0, 0, 0, 0};
    size_t off = 16;
    uint32_t n = e_le32(d->p + 8), i;
    for (i = 0; i < n; i++) {
        uint64_t len = e_le64(d->p + off + 8);
        if (!memcmp(d->p + off, "EMAC", 4)) {
            size_t tail = d->n - (off + 16 + (size_t)len);
            memmove(d->p + off + 16 + sizeof ENT, d->p + off + 16 + (size_t)len, tail);
            memcpy(d->p + off + 16, ENT, sizeof ENT);
            memset(d->p + off + 8, 0, 8);
            d->p[off + 8] = sizeof ENT;
            d->n = off + 16 + sizeof ENT + tail;
            return 0;
        }
        off += 16 + (size_t)len;
    }
    return -1;
}

/* Timers(Pits(m, hold=False), Dtims(m, channels=(3,), hold=False)) appena costruiti
   (CoSim senza snapshot dei timer): niente scadenze armate, contatori a zero */
static int rec_timers_fresh(e_buf *d)
{
    e_buf b = {0};
    int k, s, rc;
    w_le32(&b, 2);
    for (s = 0; s < 2; s++) {
        w_le32(&b, (uint32_t)s);                              /* 0 Pits, 1 Dtims */
        w_le32(&b, s ? 1u : 3u);
        w_le32(&b, 3); w_le32(&b, s ? 0u : 2u); w_le32(&b, 0); w_le32(&b, 0);   /* (3,2,0) / (3,) */
        w_le64(&b, 4680000);
        w_le64(&b, 0);
        w_u8(&b, 0);
        for (k = 0; k < 4; k++) {
            w_u8(&b, 0);
            w_f64(&b, 0.0);
        }
        for (k = 0; k < 8; k++)
            w_le64(&b, 0);
        w_le32(&b, 0);
        w_le32(&b, 0);
    }
    rc = dtst_add(d, "CTIM", &b);
    free(b.p);
    return rc;
}

/* i componenti che CoSim costruisce da s\'e' (longrun.build senza snapshot di
   componenti): coda UART8 vuota, eDMA 35 a zero, l'SSI0 non attivo. L'eMMC
   (CSDH) viene dal piolo. */
static int boot_fresh_components(e_buf *d)
{
    e_buf b = {0};
    int rc = 0, k;
    rc |= dtst_add(d, "CUIN", &b);
    w_le32(&b, 35); w_le32(&b, 155); w_le64(&b, 0); w_le64(&b, 0); w_le64(&b, 0);
    rc |= dtst_add(d, "CETX", &b);
    b.n = 0;
    w_le64(&b, 48000); w_le64(&b, 4680000); w_le64(&b, 0); w_le64(&b, 0); w_le64(&b, 0);
    w_le32(&b, 52); w_le32(&b, 54); w_le32(&b, 0);
    for (k = 0; k < 4; k++)
        w_u8(&b, 0);
    for (k = 0; k < 6; k++)
        w_le64(&b, 0);
    w_le32(&b, 0);
    w_le64(&b, 0); w_le64(&b, 0);
    rc |= dtst_add(d, "CSSX", &b);
    free(b.p);
    return rc | rec_timers_fresh(d) | dtst_legacy_emac(d);
}

/* ============================================================ CoSim senza audio */
static int cold_dbg;                /* diagnostica: ogni fetta del DSP su stderr (dbg:1), le scritture della mailbox (dbg:2) */
static void h_mb_trace(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty;
    if (cold_dbg & 2)
        fprintf(stderr, "    %s+%x=%llx (size %d)\n", ud ? "M" : "D", (unsigned)(a & 0xFFFF), (long long)(v & ((size == 4) ? 0xFFFFFFFFll : (size == 2) ? 0xFFFFll : 0xFFll)), size);
}
#define COLD_SLICE 50000            /* emu/cosim.py SLICE */
#define COLD_HOLD_MAX 4000          /* fette (200M istruzioni) oltre le quali il DSP riparte comunque */
#define COLD_EDGE_EVERY 4680        /* emu/dspcpu.py EDGE_EVERY: un fronte di DTIN0 ogni 4680 istruzioni */

static void h_watch(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    uint32_t pc = 0;
    (void)ty; (void)ud;
    uc_reg_read(uc, UC_M68K_REG_PC, &pc);
    {
        uint32_t sp = 0, st[8] = {0};
        uint8_t b[32];
        int i;
        uc_reg_read(uc, UC_M68K_REG_A7, &sp);
        if (uc_mem_read(uc, sp, b, 32) == UC_ERR_OK)
            for (i = 0; i < 8; i++)
                st[i] = (uint32_t)b[4 * i] << 24 | (uint32_t)b[4 * i + 1] << 16 | (uint32_t)b[4 * i + 2] << 8 | b[4 * i + 3];
        fprintf(stderr, "    [%lld] watch %08x <- %llx (size %d) pc=%08x sp=%08x stack %08x %08x %08x %08x %08x %08x\n", e_cur ? (long long)e_cur->instrs : 0LL, (unsigned)a,
                (long long)(v & ((size == 4) ? 0xFFFFFFFFll : (size == 2) ? 0xFFFFll : 0xFFll)), size, pc, sp, st[0], st[1],
                st[2], st[3], st[4], st[5]);
    }
}

/* il saluto del DSP nella mailbox: "HOHA" a +0..+3 e lo stato 4 a +8 (emu/cosim.py) */
static int cold_hoha(const dt_engine *e)
{
    return e->mailbox[0] == 0x48 && e->mailbox[1] == 0x4f && e->mailbox[2] == 0x48 && e->mailbox[3] == 0x41
           && e->mailbox[8] == 0 && e->mailbox[9] == 4;
}

/* emu/cosim.py CoSim.step(n) per `count` volte: il MAIN per n istruzioni
   (spin con i suoi timer), poi DspCompanion.step(n) senza la sincronia dal MAIN
   (dspcpu.step_timed: fette di EDGE_EVERY, un fronte dopo ciascuna; il DSP
   fermo nel ciclo d'attesa salta al fronte). Stessi passi del lock-step in C di
   dt2_cosim (native/st_cosim.c), che e' per la sincronia esterna (audio).
   -> 0, 1 fermato (e->status) */
static int cold_steps(dt_engine *e, int64_t count, int64_t n, volatile int *cancel)
{
    st_cpu *m = MAINC(e), *d = DSPC(e);
    cz_state *Z = &e->Z;
    spin_state *Sm = &m->S, *Sd = &d->S;
    exc_state *em = &m->exc, *ed = &d->exc;
    int stop = 0, code;
    int64_t k, ex_total = 0;
    e_cur = e;
    cpu_enter(m, e->pc);
    cpu_enter(d, e->dpc);
    e->cz_calls++;
    Z->executed = 0;
    Z->pc = e->pc;
    Z->dpc = e->dpc;
    Z->mnow = Sm->base;
    Z->dnow = Sd->base;
    Z->stop = 0;
    Z->err = 0;
    Z->dtin0_edges = e->dtin0_edges;
    Z->last_edges = e->last_edges;
    Z->irq4 = e->irq4;
    Z->idle_skipped = e->idle_skipped;
    for (k = 0; k < count && !stop; k++) {
        int64_t ex, r, done, chunk, dd;
        int shortcut = 0, moved = 0;
        if (cancel && *cancel && (k & 255) == 0) {
            snprintf(e->status, sizeof e->status, "interrotto");
            stop = 2;
            break;
        }
        /* spin(m, pc, n, pits=mt, fast=True) */
        Sm->base = Z->mnow; Sm->done = 0; Sm->instrs = n; Sm->pc = Z->pc;
        cz_reset(Sm);
        code = dt2_spin(Z->ucm, Sm);
        if (code != SPIN_LIMIT && cpu_drive(e, m, code)) {
            stop = 1;
            break;
        }
        ex = Sm->done;
        Z->mnow = Sm->base + Sm->done;
        Z->pc = Sm->pc;
        ex_total += ex;
        /* il DSP ha salutato (HOHA) e aspetta la risposta 0x4230 del MAIN: finche' non arriva
           (o per al piu' COLD_HOLD_MAX fette) il DSP non gira e non segnala (IRQ4) al MAIN: il
           MAIN legge lo stato del DSP solo quando il DSP si e' mosso */
        if (e->cold_hold) {
            if ((e->mailbox[0] == 0x42 && e->mailbox[1] == 0x30) || ++e->cold_hold_n > COLD_HOLD_MAX) {
                e->cold_hold = 0;
            } else {
                Z->dnow += n;
                Z->idle_skipped++;
                shortcut = 2;
                /* finche' il MAIN non ha letto il saluto (stato < 2) il segnale continua */
                if (c_r32(m, 0x41BB22B0u) < 2) {
                    Z->dtin0_edges++;
                    shortcut = 3;
                }
            }
        }
        /* DspCompanion.step(n): il DSP parcheggiato nel ciclo audio, senza timer in scadenza */
        if (!shortcut && Z->dpc == Z->audio_idle) {
            if (cz_dsp_tim_step(Z, Z->dnow, n, &r) < 0) { stop = 1; break; }
            if (r >= n) {
                Z->dnow += n;
                Z->idle_skipped++;
                shortcut = 1;
            }
        }
        if (!shortcut) {
            done = 0;
            while (done < n) {                              /* step_timed(n), every = EDGE_EVERY */
                chunk = n - done < COLD_EDGE_EVERY ? n - done : COLD_EDGE_EVERY;
                Z->d_t0 = Z->dnow;
                Z->d_chunk = chunk;
                *Z->park = 0;
                Sd->base = Z->dnow; Sd->done = 0; Sd->instrs = chunk; Sd->pc = Z->dpc;
                cz_reset(Sd);
                *Z->dstop = 0;
                code = dt2_spin(Z->ucd, Sd);
                if (code == SPIN_LIMIT) {
                    Z->dnow = Sd->base + Sd->done;
                    dd = Sd->done;
                } else if (code == SPIN_STOPPED && *Z->park) {
                    if (dt2_cz_park(Z) < 0) { stop = 1; break; }
                    dd = chunk;
                    /* il DSP ha scritto "HOHA" (stato 4) e aspetta il fronte: la sua fetta finisce
                       qui, cosi' il MAIN legge la mailbox con il saluto dentro (docs/AU-PORT.md) */
                    if (cold_hoha(e)) {
                        dd = n - done;
                        e->cold_hold = 1;
                        e->cold_hold_n = 0;
                    }
                } else {
                    int64_t before = Z->d_done;
                    Z->d_done = 0;
                    if (cosim_dsp_rest(e, code) < 0) { stop = 1; break; }
                    dd = Z->d_done;
                    Z->d_done = before;
                }
                done += dd > 1 ? dd : 1;
                if (cold_dbg) {
                    int i;
                    fprintf(stderr, "  dsp chunk=%lld dd=%lld park=%d dnow=%lld dpc=%08x mbox=", (long long)chunk,
                            (long long)dd, *Z->park, (long long)Z->dnow, c_rd(d, UC_M68K_REG_PC));
                    for (i = 0; i < 10; i++)
                        fprintf(stderr, "%02x", e->mailbox[i]);
                    fprintf(stderr, "\n");
                }
                if (cz_edge(Z) < 0) { stop = 1; break; }       /* edge() */
                if (ed->reg_read(Z->ucd, ed->r_pc, &Z->dpc)) { Z->err = 56; stop = 1; break; }
            }
            if (stop)
                break;
        }
        if (*Z->link_complete)
            moved = sl_deliver(e) ? 1 : 0;
        if ((!shortcut || shortcut == 3) && Z->dtin0_edges != Z->last_edges) {
            Z->last_edges = Z->dtin0_edges;
            if (cz_irq4(Z) < 0) { stop = 1; break; }
            moved = 1;
        }
        if (moved && em->reg_read(Z->ucm, em->r_pc, &Z->pc)) { Z->err = 57; stop = 1; break; }
        if ((k & 63) == 63)
            e->uart_out.n = 0;                               /* i comandi dei LED: nessuno li legge qui */
    }
    if (stop == 1 && !e->status[0])
        snprintf(e->status, sizeof e->status, "halted: lock-step error %d", Z->err);
    m->running = 0;
    d->running = 0;
    m->now = Z->mnow;
    d->now = Z->dnow;
    e->pc = Z->pc;
    e->dpc = Z->dpc;
    e->dtin0_edges = Z->dtin0_edges;
    e->last_edges = Z->last_edges;
    e->irq4 = Z->irq4;
    e->idle_skipped = Z->idle_skipped;
    e->instrs += ex_total;
    e->uart_out.n = 0;
    if (stop == 1)
        e->halted = 1;
    return stop;
}

/* tools/uiscript.py: run(n), cXbY (un tasto: giu', run(hold) due volte, su,
   run(200000), run(after)) e feed() con la PC del giro */
static int cold_run(dt_engine *e, int64_t n, volatile int *cancel)
{
    int64_t full = n / COLD_SLICE, rem = n % COLD_SLICE;
    int r = 0;
    if (full)
        r = cold_steps(e, full, COLD_SLICE, cancel);
    if (!r && rem)
        r = cold_steps(e, 1, rem, cancel);
    return r;
}

static void cold_feed(dt_engine *e, const uint8_t *data, size_t n)
{
    c_wr(MAINC(e), UC_M68K_REG_PC, e->pc);
    e->pc = panel_feed(e, data, n);
}

#define UI_HOLD 300000
#define UI_AFTER 8000000
static int cold_press(dt_engine *e, int ch, int bit, volatile int *cancel)
{
    uint8_t d[2];
    int r;
    d[0] = (uint8_t)((0x2 << 4) | ch);
    d[1] = (uint8_t)(1 << bit);
    cold_feed(e, d, 2);
    if ((r = cold_run(e, UI_HOLD, cancel)) || (r = cold_run(e, UI_HOLD, cancel)))
        return r;
    d[1] = 0;
    cold_feed(e, d, 2);
    return cold_run(e, 200000, cancel) ? 1 : cold_run(e, UI_AFTER, cancel);
}

/* diagnostica: una riga con lo stato delle due CPU e del collegamento */
static void cold_info(dt_engine *e, FILE *f, const char *tag)
{
    uint32_t regs[18], nreg;
    uint64_t nbytes;
    uint8_t dig[32];
    char hx[65];
    int k;
    st_cpu *m = MAINC(e), *d = DSPC(e);
    c_wr(m, UC_M68K_REG_PC, e->pc);
    c_wr(d, UC_M68K_REG_PC, e->dpc);
    fprintf(f, "%s instrs=%lld pc=%08x dpc=%08x hs=%u mbox=", tag, (long long)e->instrs, e->pc, e->dpc,
            c_r32(m, 0x41BB22B0u));
    for (k = 0; k < 16; k++)
        fprintf(f, "%02x", e->mailbox[k]);
    fprintf(f, " irq4=%lld dma29=%lld edges=%lld nowM=%lld nowD=%lld fired=%lld/%lld/%lld,%lld sd=%lld",
            (long long)e->irq4, (long long)e->link_dma_done, (long long)e->dtin0_edges, (long long)m->now,
            (long long)d->now, (long long)m->tim[0].t.fired[0], (long long)m->tim[0].t.fired[2],
            (long long)m->tim[0].t.fired[3], (long long)m->tim[1].t.fired[3], (long long)e->sd.log_n);
    dt_machine_regs(m->m, regs);
    dt_machine_digest(m->m, dig, &nreg, &nbytes);
    dt_sha256_hex(dig, hx);
    hx[16] = 0;
    fprintf(f, " MAIN %s", hx);
    dt_machine_regs(d->m, regs);
    dt_machine_digest(d->m, dig, &nreg, &nbytes);
    dt_sha256_hex(dig, hx);
    hx[16] = 0;
    fprintf(f, " DSP %s\n", hx);
}

static void cold_screen(dt_engine *e, FILE *f)
{
    int x, y;
    panel_publish(e);
    fprintf(f, "schermo (frame %lld):\n", (long long)e->frames);
    for (y = 0; y < 64; y += 2) {
        for (x = 0; x < 128; x++) {
            int a = e->fb[y * 128 + x], b = e->fb[(y + 1) * 128 + x];
            fputs(a && b ? "#" : a ? "'" : b ? "," : " ", f);
        }
        fputc('\n', f);
    }
}

/* lo script: token separati da spazi: run:N, cXbY, info, screen. -> 0 o l'errore */
static int cold_script(dt_engine *e, const char *script, volatile int *cancel, FILE *log, int64_t plan,
                       int64_t done0)
{
    const char *p = script;
    int64_t done = done0;
    for (;;) {
        char tok[64];
        size_t n = 0;
        int r = 0;
        while (*p == ' ')
            p++;
        if (!*p)
            return 0;
        while (*p && *p != ' ' && n < sizeof tok - 1)
            tok[n++] = *p++;
        tok[n] = 0;
        if (!strncmp(tok, "run:", 4)) {
            int64_t k = strtoll(tok + 4, NULL, 10);
            while (k > 0 && !r) {
                int64_t step = k > 5000000 ? 5000000 : k;       /* a pezzi: avanzamento e annullamento */
                r = cold_run(e, step, cancel);
                k -= step;
                done += step;
                st_boot_progress_pct = 10 + (int)(80 * done / (plan ? plan : 1));
                if (log && getenv("ST_COLD_TRACE") && ((done / 5000000) % atoi(getenv("ST_COLD_TRACE"))) == 0)
                    cold_info(e, log, "trace");
            }
        } else if (tok[0] == 'c' && strchr(tok, 'b')) {
            int ch = atoi(tok + 1), bit = atoi(strchr(tok, 'b') + 1);
            r = cold_press(e, ch, bit, cancel);
            done += 2 * UI_HOLD + 200000 + UI_AFTER;
            st_boot_progress_pct = 10 + (int)(80 * done / (plan ? plan : 1));
        } else if (!strncmp(tok, "watch:", 6)) {
            uint32_t a = (uint32_t)strtoul(tok + 6, NULL, 16);
            c_hook_mem(MAINC(e), UC_HOOK_MEM_WRITE, (void *)h_watch, (void *)(uintptr_t)a, a, a + 3);
        } else if (!strncmp(tok, "dbg:", 4)) {
            static int hooked;
            cold_dbg = atoi(tok + 4);
            if (cold_dbg && !hooked) {
                hooked = 1;
                c_hook_mem(MAINC(e), UC_HOOK_MEM_WRITE, (void *)h_mb_trace, (void *)1, MAIN_MAILBOX, MAIN_MAILBOX + 0x1F);
                c_hook_mem(DSPC(e), UC_HOOK_MEM_WRITE, (void *)h_mb_trace, NULL, DSP_MAILBOX, DSP_MAILBOX + 0x1F);
            }
        } else if (!strncmp(tok, "mon:", 4)) {
            /* diagnostica: la mailbox e il PC del DSP a ogni fetta, per N fette */
            int64_t k, cnt = strtoll(tok + 4, NULL, 10);
            for (k = 0; k < cnt && !r; k++) {
                int i;
                if (cold_dbg)
                    fprintf(stderr, "--- fetta %lld\n", (long long)k);
                r = cold_steps(e, 1, COLD_SLICE, cancel);
                if (log) {
                    fprintf(log, "mon %lld dpc=%08x hs=%u mbox=", (long long)k, e->dpc, c_r32(MAINC(e), 0x41BB22B0u));
                    for (i = 0; i < 12; i++)
                        fprintf(log, "%02x", e->mailbox[i]);
                    fprintf(log, " nowD=%lld edges=%lld irq4=%lld tcb=%08x sr=%04x\n", (long long)DSPC(e)->now,
                            (long long)e->dtin0_edges, (long long)e->irq4, c_r32(MAINC(e), e->p->current_tcb),
                            c_rd(MAINC(e), UC_M68K_REG_SR) & 0xFFFF);
                }
            }
        } else if (!strcmp(tok, "info")) {
            if (log)
                cold_info(e, log, "info");
        } else if (!strcmp(tok, "screen")) {
            if (log)
                cold_screen(e, log);
        } else if (!strncmp(tok, "fb:", 3)) {
            FILE *f = fopen(tok + 3, "wb");               /* diagnostica: il frame in PGM (128x64) */
            if (f) {
                int i;
                panel_publish(e);
                fprintf(f, "P5\n128 64\n255\n");
                for (i = 0; i < 128 * 64; i++)
                    fputc(e->fb[i] ? 255 : 0, f);
                fclose(f);
            }
        } else {
            snprintf(e->status, sizeof e->status, "unknown script token %s", tok);
            return 3;
        }
        if (r)
            return r;
    }
}

/* le istruzioni che lo script spende (per l'avanzamento) */
static int64_t cold_script_plan(const char *script)
{
    const char *p = script;
    int64_t t = 0;
    while (*p) {
        if (!strncmp(p, "run:", 4))
            t += strtoll(p + 4, NULL, 10);
        else if (*p == 'c' && (p == script || p[-1] == ' ') && strchr(p, 'b'))
            t += 2 * UI_HOLD + 200000 + UI_AFTER;
        while (*p && *p != ' ')
            p++;
        while (*p == ' ')
            p++;
    }
    return t;
}

/* il motore freddo dal piolo del MAIN (DTST) */
static dt_engine *cold_new(const dt_firmware *fw, e_buf *main_dtst, char *err, size_t errlen)
{
    e_buf st = {0};
    dt_engine *e;
    if (boot_fresh_components(main_dtst) || stst_wrap(main_dtst, NULL, &st)) {
        free(st.p);
        dt_errf(err, errlen, "out of memory");
        return NULL;
    }
    st_cold_next = 1;
    e = engine_new(fw, NULL, st.p, st.n, NULL, err, errlen);
    free(st.p);
    if (!e)
        return NULL;
    /* edma.kick: lo stato "trasferimento in volo" di un piolo senza il componente */
    if (c_r32(MAINC(e), e->p->uart8_tx_state))
        tx_run(MAINC(e), &e->etx);
    return e;
}

/* diagnostica: boot + motore freddo + `script`, e lo stato del motore freddo (la
   forma di una sessione a cadenza 4,68M, senza SSI: quella di ui_ready) */
int st_coldboot_cold(const dt_firmware *fw, const char *script, int compress, uint8_t **out, size_t *len,
                     volatile int *cancel, char *err, size_t errlen)
{
    e_buf m = {0};
    dt_engine *e;
    int r;
    *out = NULL;
    *len = 0;
    st_boot_progress_pct = 0;
    r = boot_main(fw, 60000000ull, cancel, &m, err, errlen);
    if (r)
        return r;
    e = cold_new(fw, &m, err, errlen);
    free(m.p);
    if (!e)
        return DT_ERR_FORMAT;
    r = cold_script(e, script, cancel, stderr, cold_script_plan(script), 0);
    if (r) {
        if (r == 2)
            dt_errf(err, errlen, "interrotto");
        else
            dt_errf(err, errlen, "%s", e->status);
        dt_engine_free(e);
        return r == 2 ? DT_ERR_IO : DT_ERR_UNICORN;
    }
    r = dt_engine_save_mem(e, compress, out, len);
    dt_engine_free(e);
    return r;
}

/* ============================================================ passaggio all'audio */
static int bitlen128(unsigned __int128 v)
{
    int n = 0;
    while (v) {
        n++;
        v >>= 1;
    }
    return n;
}

/* float(Fraction(now) + (Fraction(f) - Fraction(now)) * Fraction(value, old)): il
   setter `ips` di emu/pit.py e emu/dtim.py (le scadenze in istruzioni scalano col
   rapporto fra le cadenze), arrotondato come lo arrotonda Python (al piu' vicino,
   pari). Aritmetica intera esatta; fuori dal campo (valori non positivi, enormi)
   ripiega sul calcolo in double. */
static double rescale_deadline(double f, int64_t now, int64_t old, int64_t value)
{
    int e, s, nfrac, i, L, drop;
    int64_t mant, g, a, b;
    __int128 A, B, I, rem, N0;
    unsigned __int128 Q;
    int sticky;
    double fr;
    a = value;
    b = old;
    while (b) {
        g = a % b;
        a = b;
        b = g;
    }
    g = a;
    fr = frexp(f, &e);
    mant = (int64_t)ldexp(fr, 53);
    e -= 53;
    if (!(f >= 0) || f != f || e < -60 || e > 20 || now < 0)
        return (double)now + (f - (double)now) * ((double)value / (double)old);
    s = e < 0 ? -e : 0;
    N0 = e >= 0 ? ((__int128)mant << e) : (__int128)mant;
    N0 -= (__int128)now << s;
    A = N0 * (value / g);
    B = (__int128)(old / g) << s;
    I = e_floordiv128(A, B);
    rem = A - I * B;
    I += now;
    if (I < 0)
        return (double)now + (f - (double)now) * ((double)value / (double)old);
    nfrac = 56 - bitlen128((unsigned __int128)I);
    if (nfrac < 2)
        nfrac = 2;
    Q = (unsigned __int128)I;
    for (i = 0; i < nfrac; i++) {
        rem <<= 1;
        Q <<= 1;
        if (rem >= B) {
            rem -= B;
            Q |= 1;
        }
    }
    sticky = rem != 0;
    L = bitlen128(Q);
    if (L <= 53)
        return ldexp((double)(uint64_t)Q, -nfrac);
    drop = L - 53;
    {
        unsigned __int128 low = Q & (((unsigned __int128)1 << drop) - 1), half = (unsigned __int128)1 << (drop - 1);
        Q >>= drop;
        if (low > half || (low == half && (sticky || (Q & 1))))
            Q++;
    }
    return ldexp((double)(uint64_t)Q, drop - nfrac);
}

/* emu/gui.py Emulator, `for source in pits.sources: source.ips = n` (e il rebase del
   DTCN di Dtims.ips), Ssi0Dma.ips e MidiPort.ips */
static void engine_set_ips(dt_engine *e, int64_t value)
{
    st_cpu *c = MAINC(e);
    int k, ch;
    for (k = 0; k < 2; k++) {
        e_tsrc *s = &c->tim[k];
        int64_t old = s->ips;
        if (old && old != value) {
            if (k == 1)
                for (ch = 0; ch < 4; ch++)
                    if (c->dtcn_has[ch]) {
                        __int128 x = (__int128)c->now - c->dtcn_origin[ch];
                        c->dtcn_origin[ch] = (int64_t)(c->now - (-e_floordiv128(-(x * value), old)));
                    }
            for (ch = 0; ch < 4; ch++)
                if (s->t.has[ch])
                    s->t.next[ch] = rescale_deadline(s->t.next[ch], c->now, old, value);
        }
        s->ips = value;
        s->t.ips = (double)value;
    }
    e->sst.ips = value;
    e->midi.ips = value;
}

/* Ssi0Dma.arm_legacy (il percorso del Rytm, che la Syntakt usa): i descrittori dei
   canali 52 e 54 devono essere quelli del firmware gia' avviato, poi i due canali
   sono abilitati e la prima richiesta cade a now + periodo (ips ancora a 4,68M) */
static int ssi_arm_legacy(dt_engine *e, char *err, size_t errlen)
{
    st_cpu *c = MAINC(e);
    sst_state *s = &e->sst;
    uint32_t rx = E_TCD_BASE + 52u * 0x20u, tx = E_TCD_BASE + 54u * 0x20u;
    if (c_r32(c, rx) != 0xFC0C8008u || c_r32(c, tx + 0x10) != 0xFC0C8000u) {
        dt_errf(err, errlen, "SSI legacy upgrade: TCD52/54 are not the firmware's SSI1 descriptors (state not past the audio init)");
        return -1;
    }
    if (!(c_r16(c, rx + 0x14) & 0x7FFF) || !(c_r16(c, rx + 0x1C) & 0x7FFF) || !(c_r16(c, tx + 0x14) & 0x7FFF)
        || !(c_r16(c, tx + 0x1C) & 0x7FFF)) {
        dt_errf(err, errlen, "SSI legacy upgrade found inactive TCD52/54");
        return -1;
    }
    s->en = 3;
    s->nextN = s->now * s->rh + s->ips;
    s->has_next = 1;
    return 0;
}

/* lo stato della sessione dalla catena fredda: il motore freddo salvato (ui_ready)
   -> motore con SSI (legacy upgrade), cadenza 150M, MIDI, porta FPGA e DSP sulla
   sincronia del MAIN, come Emulator(ui_ready, audio=True) -> `chunks` giri di
   assestamento -> stato */
static int audio_session(const dt_firmware *fw, const uint8_t *cold, size_t cold_len, int chunks, int compress,
                         uint8_t **out, size_t *len, volatile int *cancel, char *err, size_t errlen)
{
    dt_engine *e;
    int r, k;
    st_legacy_ips_next = 1;
    e = engine_new(fw, NULL, cold, cold_len, NULL, err, errlen);
    if (!e)
        return DT_ERR_FORMAT;
    if (ssi_arm_legacy(e, err, errlen)) {
        dt_engine_free(e);
        return DT_ERR_FORMAT;
    }
    engine_set_ips(e, E_AUDIO_IPS);
    e->legacy_ips = 0;
    for (k = 0; k < chunks; k++) {
        if (cancel && *cancel) {
            dt_errf(err, errlen, "interrotto");
            dt_engine_free(e);
            return DT_ERR_IO;
        }
        if (dt_engine_chunk(e)) {
            dt_errf(err, errlen, "il firmware si e' fermato: %s", dt_engine_status(e));
            dt_engine_free(e);
            return DT_ERR_UNICORN;
        }
        if ((k & 15) == 0)
            st_boot_progress_pct = 90 + (int)(10 * k / (chunks ? chunks : 1));
    }
    r = dt_engine_save_mem(e, compress, out, len);
    dt_engine_free(e);
    st_boot_progress_pct = 100;
    return r;
}

/* diagnostica: la catena intera con un altro script e `chunks` giri audio */
int st_coldboot_full(const dt_firmware *fw, const char *script, int chunks, int compress, uint8_t **out,
                     size_t *len, volatile int *cancel, char *err, size_t errlen)
{
    uint8_t *cold = NULL;
    size_t n = 0;
    int r;
    *out = NULL;
    *len = 0;
    r = st_coldboot_cold(fw, script, 0, &cold, &n, cancel, err, errlen);
    if (r)
        return r;
    r = audio_session(fw, cold, n, chunks, compress, out, len, cancel, err, errlen);
    free(cold);
    return r;
}

/* gli istanti della catena (docs/AU-PORT.md): la CoSim dal piolo 60M fino al
   dialogo di calibrazione, poi uiscript di ui_ready */
#define ST_COLD_SCRIPT "run:300000000 run:20000000 c3b3 run:30000000"
#define ST_AUDIO_SETTLE_CHUNKS 0

int st_coldboot_session(const dt_firmware *fw, int compress, uint8_t **out, size_t *len, volatile int *cancel,
                        char *err, size_t errlen)
{
    return st_coldboot_full(fw, ST_COLD_SCRIPT, ST_AUDIO_SETTLE_CHUNKS, compress, out, len, cancel, err, errlen);
}
