/* engine.c, parte 8: il boot freddo dal reset, come emu/dspboot.py run (e la
 * scala di emu/checkpoint.py make): la macchina con gli hook di dspboot
 * (flash, semaforo di completamento, depack, idle spin col tick del vettore
 * 32), la porta GPIO che abilita lo storage (emu/gpio.py SdGate) e l'eSDHC; il
 * MAIN OS caricato a 0x40000400, SR 0x2700, A7 0x40800000, PC all'ingresso.
 * Lo stato si salva DENTRO l'hook di copertura all'istruzione numero `point`
 * (prima di eseguirla), come fa make(): qui l'hook ferma l'emulazione li'. */

#define BOOT_PPDSDR_C 0xEC09401Au
#define BOOT_PPDSDR_D 0xEC09401Bu
#define BOOT_PCLRR_D 0xEC094027u

typedef struct {
    dt_engine *e;
    uint64_t n, point;
    int64_t spin;
    int driven;
} e_boot;

static void hb_cover(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_boot *b = (e_boot *)ud;
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
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)a; (void)s;
    b->spin++;
    if (b->spin % 20000 == 0)
        dtm_raise_vector(b->e->m, 32, 0, -1);
}

static void hb_gate_set(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (v & 0x10)
        ((e_boot *)ud)->driven = 1;
}

static void hb_gate_clear(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (!(v & 0x10))
        ((e_boot *)ud)->driven = 0;
}

static void hb_gate_sense(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_boot *b = (e_boot *)ud;
    uint8_t cur = (uint8_t)(e_r8(b->e, BOOT_PPDSDR_C) & ~0x08);
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e_w8(b->e, BOOT_PPDSDR_C, (uint8_t)(cur | (b->driven ? 0x08 : 0)));
}

/* -> lo stato a `point` istruzioni dal reset (DTST, come una piolo della scala) */
/* L'indice della libreria di fabbrica VUOTO (FINDINGS i164): al settore
   0x458000, sotto il volume utente. Il mount al boot (0x4015a450 -> 0x4015a124
   -> 0x4002cd6a) rifiuta qualsiasi volume se li' non trova un indice 'MaGj'
   valido: sul Digitakt vero c'e' la libreria di fabbrica, sull'eMMC emulato
   no, e senza +Drive montato ogni richiesta del pannello fallisce (CreateDir
   risponde 91). Zero voci: il firmware non installa nulla. Non e' contenuto
   Elektron. Campi big endian; a +4 il CRC-32 di +8..len in little endian (il
   residuo 0xDEBB20E3 che controlla il firmware, 0x4013e06c). */
static void e_factory_index(dt_engine *e)
{
    uint8_t s[512];
    uLong c;
    memset(s, 0, sizeof s);
    memcpy(s, "MaGj", 4);
    s[8] = 0x00; s[9] = 0x2b;                      /* u16 0x2b */
    s[10] = 0x00; s[11] = 0x03;                    /* u16 versione 3 */
    s[15] = 0x14;                                  /* u32 lunghezza: intestazione + contatore */
    /* +0x10 u32 0: zero voci */
    c = crc32(0L, s + 8, 0x14 - 8);
    s[4] = (uint8_t)c; s[5] = (uint8_t)(c >> 8); s[6] = (uint8_t)(c >> 16); s[7] = (uint8_t)(c >> 24);
    ov_write(&e->sd.ov, 0x458000ull * SD_SECTOR, s, sizeof s);
}

int dt_coldboot(const dt_firmware *fw, uint64_t point, int compress, uint8_t **out, size_t *len,
                char *err, size_t errlen)
{
    return dt_coldboot_ex(fw, point, compress, 0, out, len, err, errlen);
}

int dt_coldboot_ex(const dt_firmware *fw, uint64_t point, int compress, int flags, uint8_t **out, size_t *len,
                char *err, size_t errlen)
{
    dt_engine *e = (dt_engine *)calloc(1, sizeof *e);
    e_boot *b = (e_boot *)calloc(1, sizeof *b);
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
        dt_errf(err, errlen, "firmware without MAIN OS");
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
        dt_errf(err, errlen, "unsupported firmware: no address profile");
        r = DT_ERR_UNSUPPORTED;
        goto out;
    }
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || fw->container_len > e->flash_len - 0x80000)
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
    /* gli hook di dspboot.run (fast=True), nel suo ordine */
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
    if (flags & DT_COLDBOOT_EMMC_DT2)
        sd_identity_dt2(&e->sd);
    if (flags & DT_COLDBOOT_FACTORY_INDEX)
        e_factory_index(e);
    /* dspboot passa anche dma_sem (come longrun): senza, la prima lettura
       CMD18 a DMA del boot freddo non finiva mai (FINDINGS i208) */
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
            dt_errf(err, errlen, "cold boot stopped at %llu instructions: %s", (unsigned long long)b->n,
                    uc_strerror(ue));
            r = DT_ERR_UNICORN;
            goto out;
        }
    }
    e->pc = e_rd(e, UC_M68K_REG_PC);
    e->boot_only = 1;
    r = dt_engine_save_mem(e, compress, out, len);
out:
    if (e) {
        if (e->m)
            dt_machine_free(e->m);
        free(e->flash);
        ov_free(&e->sd.ov);
        if (e_cur == e)
            e_cur = NULL;
        free(e);
    }
    free(b);
    return r;
}

/* FORMAT +DRIVE dal menu del firmware (FINDINGS i212), come farebbe l'utente
   su un Digitakt II con il +Drive vuoto: SETTINGS > SYSTEM > FORMAT +DRIVE,
   spunta SAMPLES, ERASE SELECTED DATA, "ARE YOU SURE? Y/N" -> YES. Il lavoro
   lo fa il firmware (0x4009eaf6 -> 0x4015a424 -> 0x4015a164(1): superblocco
   'ekFS' a 0x5d8000, /incoming e, con l'indice di fabbrica, /factory). Poi
   NO tre volte (SUCCESS, SYSTEM, SETTINGS): la pagina principale. Gli eventi
   sono quelli di uno scenario di `dtcore play` (at N press/release K), con gli
   stessi istanti: lo stesso risultato si ottiene con `dtcore play` o con
   tools/dtcore_guiref.py dal gradino. Tasti: emu/guiv2.py KEYS. */
#define FMT_SETTINGS 8
#define FMT_YES 10
#define FMT_NO 12
#define FMT_DOWN 14
#define FMT_STEP 80000000ll     /* fra un tasto e il successivo (0,53 s a 150M istr/s) */
#define FMT_HOLD 20000000ll     /* tasto premuto */
#define FMT_WAIT 400000000ll    /* dopo la conferma: la formattazione ne usa ~45M */
#define FMT_SETTLE 200000000ll
static const uint8_t FMT_KEYS[] = {
    FMT_SETTINGS,                                                          /* SETTINGS: PROJECT */
    FMT_DOWN, FMT_DOWN, FMT_DOWN, FMT_DOWN, FMT_DOWN, FMT_DOWN, FMT_DOWN,  /* -> SYSTEM */
    FMT_YES,                                                               /* USB CONFIG */
    FMT_DOWN, FMT_DOWN, FMT_YES,                                           /* FORMAT +DRIVE */
    FMT_DOWN, FMT_YES,                                                     /* spunta SAMPLES */
    FMT_DOWN, FMT_YES,                                                     /* ERASE SELECTED DATA */
    FMT_YES};                                                              /* ARE YOU SURE? Y/N */
#define FMT_NKEYS (sizeof FMT_KEYS / sizeof FMT_KEYS[0])
#define FMT_NEXIT 3

typedef struct {
    int64_t at;
    int kind, code;
} e_fmt_ev;

/* -> numero di eventi in ev (2 per tasto), *end l'ultima istruzione */
static size_t e_fmt_events(int64_t base, e_fmt_ev *ev, int64_t *end)
{
    size_t i, n = 0;
    int64_t t = base;
    for (i = 0; i < FMT_NKEYS + FMT_NEXIT; i++) {
        int code = i < FMT_NKEYS ? FMT_KEYS[i] : FMT_NO;
        if (i == FMT_NKEYS)
            t += FMT_WAIT - FMT_STEP;
        ev[n].at = t; ev[n].kind = DT_PANEL_PRESS; ev[n].code = code; n++;
        ev[n].at = t + FMT_HOLD; ev[n].kind = DT_PANEL_RELEASE; ev[n].code = code; n++;
        t += FMT_STEP;
    }
    *end = t + FMT_SETTLE;
    return n;
}

/* il superblocco del +Drive sull'eMMC emulato (lo scrive la formattazione) */
static int e_has_ekfs(dt_engine *e)
{
    uint8_t s[4] = {0, 0, 0, 0};
    ov_read_into(&e->sd.ov, 0x5d8000ull * SD_SECTOR, s, 4);
    return !memcmp(s, "ekFS", 4);
}

/* dt_coldboot fino al gradino, poi il ciclo di `dtcore play` (e di
   tools/dtcore_guiref.py): un giro dopo aver raggiunto `instrs`, poi lo stato.
   Con DT_COLDBOOT_FORMAT_DRIVE il ciclo prosegue con gli eventi di
   e_fmt_events da max(instrs, DT_COLDBOOT_GUI) */
int dt_coldboot_session(const dt_firmware *fw, uint64_t rung, int64_t instrs, int compress,
                        uint8_t **out, size_t *len, volatile int *cancel, char *err, size_t errlen)
{
    return dt_coldboot_session_ex(fw, rung, instrs, compress, 0, out, len, cancel, err, errlen);
}

int dt_coldboot_session_ex(const dt_firmware *fw, uint64_t rung, int64_t instrs, int compress, int flags,
                           uint8_t **out, size_t *len, volatile int *cancel, char *err, size_t errlen)
{
    uint8_t *st = NULL;
    size_t n = 0, nev = 0, iev = 0;
    dt_engine *e;
    int r, last = 0;
    e_fmt_ev ev[2 * (FMT_NKEYS + FMT_NEXIT)];
    int64_t target = instrs;
    *out = NULL;
    *len = 0;
    if (flags & DT_COLDBOOT_FORMAT_DRIVE) {
        flags |= DT_COLDBOOT_EMMC_DT2;          /* con la carta di sd_init il firmware non monta ne' formatta */
        nev = e_fmt_events(instrs > DT_COLDBOOT_GUI ? instrs : DT_COLDBOOT_GUI, ev, &target);
    }
    r = dt_coldboot_ex(fw, rung, 0, flags, &st, &n, err, errlen);
    if (r)
        return r;
    if (cancel && *cancel) {
        free(st);
        dt_errf(err, errlen, "interrotto");
        return DT_ERR_IO;
    }
    e = dt_engine_new_mem(fw, st, n, NULL, err, errlen);
    free(st);
    if (!e)
        return DT_ERR_FORMAT;
    for (;;) {
        if (dt_engine_instrs(e) >= target)
            last = 1;
        if (cancel && *cancel) {
            dt_errf(err, errlen, "interrotto");
            dt_engine_free(e);
            return DT_ERR_IO;
        }
        while (iev < nev && ev[iev].at <= dt_engine_instrs(e)) {
            dt_engine_panel(e, ev[iev].kind, ev[iev].code, 0);
            iev++;
        }
        if (dt_engine_chunk(e)) {
            dt_errf(err, errlen, "il firmware si e' fermato: %s", dt_engine_status(e));
            dt_engine_free(e);
            return DT_ERR_UNICORN;
        }
        if (last)
            break;
    }
    if ((flags & DT_COLDBOOT_FORMAT_DRIVE) && !e_has_ekfs(e)) {
        dt_errf(err, errlen, "FORMAT +DRIVE: nessun superblocco ekFS a 0x5d8000");
        dt_engine_free(e);
        return DT_ERR_UNSUPPORTED;
    }
    r = dt_engine_save_mem(e, compress, out, len);
    dt_engine_free(e);
    return r;
}
