/* dn_engine.c, parte finale: il boot freddo del Digitone II dal reset, senza Python
 * (docs/COLDBOOT.md). Dal reset ai gradini della scala di emu/dspboot.py run /
 * emu/checkpoint.py make (boot60M ... boot400M), poi la sessione: il motore della
 * GUI (gui.Emulator con l'audio) per `instrs` istruzioni. Derivato da e_boot.inc.c
 * del Digitakt II (digitakt/emulator/core): gli hook di dspboot nel suo ordine, la
 * porta GPIO che abilita lo storage (emu/gpio.py SdGate), l'eSDHC; il MAIN OS
 * caricato a 0x40000400, SR 0x2700, A7 0x40800000, PC all'ingresso.
 *
 * Lo stato si salva DENTRO l'hook di copertura all'istruzione numero `point`
 * (prima di eseguirla), come fa checkpoint.make: qui l'hook ferma l'emulazione li'.
 * La scala del Python (60M con make, poi 120M ... 400M con una ripresa da boot60M,
 * scratch/dn_ladder_resume.py) coincide byte per byte con una corsa di seguito dal
 * reset: la ripresa non cambia nulla (docs/COLDBOOT.md), quindi qui si va di seguito.
 *
 * Nessuna parte del firmware e' qui: tutto arriva a runtime dal .syx dell'utente. */

#define BOOT_PPDSDR_C 0xEC09401Au
#define BOOT_PPDSDR_D 0xEC09401Bu
#define BOOT_PCLRR_D 0xEC094027u

typedef struct {
    dt_engine *e;
    uint64_t n, point;
    int64_t spin;
    int driven;
} dn_boot;

static void hb_cover(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dn_boot *b = (dn_boot *)ud;
    (void)a; (void)s;
    b->n++;
    if (b->n >= b->point) {
        uc_emu_stop(uc);
        b->e->m->stop_flag = 1;
    }
}

static void hb_pre_isa(void *uc, void *ctx)
{
    uc_hook h;
    uc_hook_add((uc_engine *)uc, &h, UC_HOOK_CODE, (void *)hb_cover, ctx, 1, 0);
}

static void hb_noop(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s; (void)ud;            /* osservazione in dspboot */
}

static void hb_halt(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dn_boot *b = (dn_boot *)ud;
    (void)uc; (void)a; (void)s;
    b->spin++;
    if (b->spin % 20000 == 0)
        dtm_raise_vector(b->e->m, 32, 0, -1);
}

static void hb_gate_set(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (v & 0x10)
        ((dn_boot *)ud)->driven = 1;
}

static void hb_gate_clear(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (!(v & 0x10))
        ((dn_boot *)ud)->driven = 0;
}

static void hb_gate_sense(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dn_boot *b = (dn_boot *)ud;
    uint8_t cur = (uint8_t)(e_r8(b->e, BOOT_PPDSDR_C) & ~0x08);
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e_w8(b->e, BOOT_PPDSDR_C, (uint8_t)(cur | (b->driven ? 0x08 : 0)));
}

/* L'indice della libreria di fabbrica VUOTO (FINDINGS i164 del Digitakt II): al settore
   0x458000, sotto il volume utente. Non e' contenuto Elektron: la sola intestazione
   'MaGj', zero voci. Campi big endian; a +4 il CRC-32 di +8..len in little endian. */
static void dn_factory_index(dt_engine *e)
{
    uint8_t s[512];
    uLong c;
    memset(s, 0, sizeof s);
    memcpy(s, "MaGj", 4);
    s[8] = 0x00; s[9] = 0x2b;                      /* u16 0x2b */
    s[10] = 0x00; s[11] = 0x03;                    /* u16 versione 3 */
    s[15] = 0x14;                                  /* u32 lunghezza: intestazione + contatore */
    c = crc32(0L, s + 8, 0x14 - 8);
    s[4] = (uint8_t)c; s[5] = (uint8_t)(c >> 8); s[6] = (uint8_t)(c >> 16); s[7] = (uint8_t)(c >> 24);
    ov_write(&e->sd.ov, 0x458000ull * SD_SECTOR, s, sizeof s);
}

/* -> lo stato a `point` istruzioni dal reset (DTST, come un piolo della scala) */
static int dn_coldboot_state(const dt_firmware *fw, uint64_t point, int compress, int flags,
                             uint8_t **out, size_t *len, char *err, size_t errlen)
{
    dt_engine *e = (dt_engine *)calloc(1, sizeof *e);
    dn_boot *b = (dn_boot *)calloc(1, sizeof *b);
    dt_machine_config mc;
    uint8_t sha[32];
    char hex[65];
    dt_sha256 h;
    size_t i, off;
    int r = DT_ERR_NOMEM;
    const uint8_t *img;
    if (!e || !b)
        goto out;
    b->e = e;
    b->point = point;
    e->fw = fw;
    e->main_os = dt_fw_section(fw, 3);
    if (!e->main_os) {
        dt_errf(err, errlen, "firmware senza MAIN OS");
        r = DT_ERR_FORMAT;
        goto out;
    }
    img = e->main_os->data;
    dt_sha256_init(&h);
    dt_sha256_update(&h, img, e->main_os->len);
    dt_sha256_final(&h, sha);
    dt_sha256_hex(sha, hex);
    for (i = 0; i < DT_NPROFILES; i++)
        if (!strcmp(DT_PROFILES[i].main_sha256, hex))
            e->p = &DT_PROFILES[i];
    if (!e->p) {
        dt_errf(err, errlen, "firmware non supportato (MAIN OS %.16s...): nessun profilo di indirizzi", hex);
        r = DT_ERR_UNSUPPORTED;
        goto out;
    }
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000)
        goto out;
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);
    memset(&mc, 0, sizeof mc);
    mc.idle_yield = 0;
    mc.pre_isa = hb_pre_isa;
    mc.pre_isa_ctx = b;
    e->m = dt_machine_new(fw, &mc, err, errlen);
    if (!e->m) {
        r = DT_ERR_UNICORN;
        goto out;
    }
    e->m->no_mmio_at_load = 1;
    e->uc = e->m->uc;
    e_cur = e;
    /* gli hook di dspboot.run (fast=True), nel suo ordine: prima la copertura (sopra, in
       dt_machine_new), poi MOVEC (machine.c), BITREV/BYTEREV e FF1 */
    if (isa_hooks(e)) {
        r = DT_ERR_NOMEM;
        goto out;
    }
    e_hook_code(e, (void *)h_flash_read, e, e->p->flash_read);
    for (i = 0; i < 32; i++)
        if (e->p->call_sites[i])
            e_hook_code(e, (void *)hb_noop, b, e->p->call_sites[i]);
    e_hook_code(e, (void *)h_depack_clamp, e, e->p->depack_copy);
    e_hook_code(e, (void *)h_pend_call, e, e->p->pend_call);
    for (i = 0; i < 32; i++)
        if (e->p->task_create_sites[i])
            e_hook_code(e, (void *)hb_noop, b, e->p->task_create_sites[i]);
    e_hook_code(e, (void *)hb_noop, b, e->p->task_start);
    for (off = 0; off + 1 < e->main_os->len; off += 2)
        if (img[off] == 0x60 && img[off + 1] == 0xFE)
            e_hook_code(e, (void *)hb_halt, b, DT_MAIN_LOAD + (uint32_t)off);
    dtm_mmio_set(e->m, 0xEC070004u, 0x0D000000u);
    dtm_mmio_set(e->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(e->m, 0xEC03802Cu, 0x80000000u);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_gate_set, b, BOOT_PPDSDR_D, BOOT_PPDSDR_D);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_gate_clear, b, BOOT_PCLRR_D, BOOT_PCLRR_D);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_gate_sense, b, BOOT_PPDSDR_C, BOOT_PPDSDR_C);
    sd_init(e);
    if (flags & DT_COLDBOOT_FACTORY_INDEX)
        dn_factory_index(e);
    /* dspboot passa anche dma_sem (come longrun): senza, la prima lettura CMD18 a DMA del
       boot freddo non finiva mai (FINDINGS d1) */
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_serq, e, E_SERQ, E_SERQ);
    dtm_install_mmio(e->m);
    /* Machine.load + reset */
    for (off = 0; off < e->main_os->len; off += 0x100000)
        dtm_ensure(e->m, DT_MAIN_LOAD + off);
    dtm_ensure(e->m, DT_MAIN_LOAD);
    e_write(e, DT_MAIN_LOAD, img, e->main_os->len);
    dtm_ensure(e->m, 0x40800000u);
    e_wr(e, UC_M68K_REG_SR, 0x2700);
    e_wr(e, UC_M68K_REG_A7, 0x40800000u);
    e->m->state_loaded = 1;
    {
        uc_err ue = uc_emu_start(e->uc, e->p->entry, 0, 0, (size_t)(point + 1000000));
        if (ue != UC_ERR_OK || b->n < point) {
            dt_errf(err, errlen, "il boot freddo si e' fermato a %llu istruzioni: %s", (unsigned long long)b->n,
                    uc_strerror(ue));
            r = DT_ERR_UNICORN;
            goto out;
        }
    }
    e->pc = e_rd(e, UC_M68K_REG_PC);
    e->boot_only = 1;
    /* Machine.mmio dello stato: i valori forzati della macchina */
    for (i = 0; i < e->m->nmmio; i++)
        mmio_set(e, e->m->mmio[i].addr, e->m->mmio[i].value);
    r = dt_engine_save_mem(e, compress, out, len);
out:
    if (e) {
        if (e->m)
            dt_machine_free(e->m);
        free(e->flash);
        free(e->revs);
        ov_free(&e->sd.ov);
        if (e_cur == e)
            e_cur = NULL;
        free(e);
    }
    free(b);
    return r;
}

/* un gradino della scala di emu/dspboot.py (boot*M.snap): lo stato a `point` istruzioni dal reset */
int dn_coldboot_rung(const dt_firmware *fw, uint64_t point, int compress, int flags, uint8_t **out, size_t *len,
                     char *err, size_t errlen)
{
    return dn_coldboot_state(fw, point, compress, flags, out, len, err, errlen);
}

/* ----------------------------------------------------------------- la sessione
 * Come la fa il Python (docs/DIGITONE-PORT.md, "Avvio"): la scala esatta fino a 120M (sopra),
 * poi tools/guirun.py da boot120M.snap (save-at 790M) = main_os.snap: Main OS avviato e +Drive
 * formattato dal firmware; poi la GUI con l'audio (emu/gui.py) la apre e la salva = session.snap.
 *
 * Fase B, "guirun": lo stesso motore di dt_engine_* con `boot_plain` (nessun DSP, nessuna SSI0):
 * guirun e' il worker della GUI SENZA audio. Con l'audio acceso l'intro non finisce mai (l'SSI0
 * gira prima che il firmware sia pronto: lo fa anche il Python, docs/COLDBOOT.md); i timer sono
 * trattenuti durante l'intro e al passaggio passano a 4 x INSTR_PER_SEC. Lo stato che ne esce e'
 * quello di main_os.snap: senza il componente Ssi0Dma, che il motore con l'audio aggiorna
 * ("legacy") all'apertura, come la GUI.
 *
 * Fase C, "riscaldamento": il motore con l'audio e il DSP gira un secondo emulato, cosi' il
 * collegamento col DSP e' gia' partito nella RAM (session.snap) e i primi suoni non aspettano
 * l'avvio del collegamento. Senza uno stato del DSP (dn_set_dsp_state o DNCORE_DSP_STATE) si salta. */
#define DN_BOOT_RUNG 120000000ull           /* l'ultimo gradino esatto prima di guirun */
#define DN_BOOT_IPS 18720000                /* tools/guirun.py --post-intro-ips (4 x INSTR_PER_SEC) */
#define DN_BOOT_SETTLE 200000000ll          /* fase C: 200M istruzioni (1,3 s emulati) */

typedef struct {
    volatile int *cancel;
    void (*progress)(void *ctx, int percent);
    void *ctx;
    int lo, hi;                              /* la fascia di avanzamento della fase */
    int last;
} dn_bprog;

/* i giri del motore fino a `target` istruzioni (come il ciclo di guirun: lo stato si salva al
   primo confine di giro a o dopo l'istruzione) */
static int dn_boot_run(dt_engine *e, int64_t target, dn_bprog *pg, char *err, size_t errlen)
{
    int64_t logged = -1;
    for (;;) {
        if (dt_engine_instrs(e) >= target)
            return 0;
        if (pg->cancel && *pg->cancel) {
            dt_errf(err, errlen, "interrotto");
            return DT_ERR_IO;
        }
        if (dt_engine_chunk(e)) {
            dt_errf(err, errlen, "il firmware si e' fermato: %s", dt_engine_status(e));
            return DT_ERR_UNICORN;
        }
        if (getenv("DN_COLDBOOT_LOG") && dt_engine_instrs(e) / 20000000 != logged) {
            logged = dt_engine_instrs(e) / 20000000;
            fprintf(stderr, "[coldboot] %lldM pc=%08x tasks=%lld mainloop=%lld jobs=%lld pit=%lld/%lld/%lld dtim3=%lld "
                    "sd=%lld/%lld sat=%lld ips=%lld %s\n", (long long)(dt_engine_instrs(e) / 1000000), e->pc,
                    (long long)e->task_creates, (long long)e->mainloop, (long long)e->jobs,
                    (long long)e->tim[0].t.fired[0], (long long)e->tim[0].t.fired[2], (long long)e->tim[0].t.fired[3],
                    (long long)e->tim[1].t.fired[3], (long long)e->sd.log_n, (long long)e->sd.dma_bytes,
                    (long long)e->satisfied, (long long)e->tim[0].ips, e->use_panel ? "os" : "intro");
        }
        if (pg->progress && target > 0) {
            int pct = pg->lo + (int)(((int64_t)(pg->hi - pg->lo) * dt_engine_instrs(e)) / target);
            if (pct != pg->last && pct < 100) {
                pg->last = pct;
                pg->progress(pg->ctx, pct);
            }
        }
    }
}

static int dn_have_dsp_state(void)
{
    return g_dsp_state || getenv("DNCORE_DSP_STATE");
}

int dn_coldboot_session(const dt_firmware *fw, int64_t instrs, int compress, uint8_t **out, size_t *len,
                        volatile int *cancel, void (*progress)(void *ctx, int percent), void *ctx,
                        char *err, size_t errlen)
{
    uint8_t *st = NULL;
    size_t n = 0;
    dt_engine *e;
    dt_engine_config cfg;
    dn_bprog pg;
    int r, settle = dn_have_dsp_state();
    *out = NULL;
    *len = 0;
    memset(&pg, 0, sizeof pg);
    pg.cancel = cancel;
    pg.progress = progress;
    pg.ctx = ctx;
    pg.last = -1;
    if (instrs <= 0)
        instrs = DN_COLDBOOT_INSTRS;
    if (progress)
        progress(ctx, 0);
    /* la scala fino a 120M */
    r = dn_coldboot_state(fw, DN_BOOT_RUNG, 0, 0, &st, &n, err, errlen);
    if (r)
        return r;
    if (cancel && *cancel) {
        free(st);
        dt_errf(err, errlen, "interrotto");
        return DT_ERR_IO;
    }
    /* fase B: guirun */
    pg.lo = 8;
    pg.hi = settle ? 85 : 99;
    memset(&cfg, 0, sizeof cfg);
    cfg.panel_dwell = 16;
    cfg.post_intro_ips = getenv("DN_COLDBOOT_IPS") ? atoll(getenv("DN_COLDBOOT_IPS")) : DN_BOOT_IPS;   /* prova */
    g_boot_plain = 1;
    e = dt_engine_new_mem(fw, st, n, &cfg, err, errlen);
    g_boot_plain = 0;
    free(st);
    if (!e)
        return DT_ERR_FORMAT;
    r = dn_boot_run(e, instrs, &pg, err, errlen);
    if (!r)
        r = dt_engine_save_mem(e, settle ? 0 : compress, out, len);
    dt_engine_free(e);
    if (r || !settle) {
        if (!r && progress)
            progress(ctx, 100);
        return r;
    }
    /* fase C: il motore con l'audio e il DSP */
    st = *out;
    n = *len;
    *out = NULL;
    *len = 0;
    e = dt_engine_new_mem(fw, st, n, NULL, err, errlen);
    free(st);
    if (!e)
        return DT_ERR_FORMAT;
    pg.lo = 85;
    pg.hi = 99;
    r = dn_boot_run(e, DN_BOOT_SETTLE, &pg, err, errlen);
    if (!r)
        r = dt_engine_save_mem(e, compress, out, len);
    dt_engine_free(e);
    if (!r && progress)
        progress(ctx, 100);
    return r;
}
