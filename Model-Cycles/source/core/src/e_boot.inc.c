/* engine.c, parte 8: il boot freddo dal reset, come emu/dspboot.py run() (fast,
 * con sdgate, esdhc e il pannello del Syntakt su UART8) e tools/mcsnap.py: la
 * macchina con gli hook di dspboot (flash, semaforo di completamento, depack,
 * idle spin col tick del vettore 32), la porta GPIO che abilita lo storage
 * (emu/gpio.py SdGate), l'eSDHC con la carta del Cycles, il micro del pannello
 * su UART8 (emu/panelmcu.py), la scheda (emu/board.py, emu/mchw.py); il MAIN OS
 * a 0x40000400, SR 0x2700, A7 0x40800000, PC all'ingresso, e in 0x40800004 la
 * parola che il bootstrap passa al MAIN OS (BOOT_FLAGS = 0x12: FACTORY RESET +
 * riformatta il +Drive, il menu STARTUP PATTERN + TRACK + TRIG 3).
 *
 * Il boot si ferma dopo `point` istruzioni (uc.emu_start(..., count=N) del
 * Python) e si salva lo stato: DTST con soli i record della macchina e la
 * carta (CSDH), come tools/snap2dtst.py da snapshots/<fw>/cold200M.snap.
 *
 * Niente firmware ne' stato derivato nel sorgente: tutto arriva a runtime
 * dal .syx dell'utente. */

#define BOOT_PPDSDR_C 0xEC09401Au
#define BOOT_PPDSDR_D 0xEC09401Bu
#define BOOT_PCLRR_D 0xEC094027u
#define BOOT_ENTRY 0x400004E8u               /* symbols.resolve(...).entry */
#define BOOT_FLAGS_ADDR 0x40800004u
#define BOOT_FLAGS 0x12u
#define BOOT_TICK_VEC 32
#define BOOT_TICK_EVERY 20000
#define BOOT_STEP 4000000ull                 /* quanto si esegue fra due controlli di `cancel` */
#define PM_RXRDY 0x01
#define PM_TXRDY 0x04
#define PM_TXEMP 0x08

typedef struct {
    dt_engine *e;
    int64_t spin;
    int driven;
    /* emu/panelmcu.py PanelMcu('syntakt'): UART8 con un RX FIFO e un pannello
       che risponde una volta sola alla richiesta di versione */
    uint8_t rx[16];
    int rxn;
    uint8_t tail[4];                         /* gli ultimi 4 byte trasmessi */
    int ntx;
    int replied;
} e_boot;

static void hb_halt(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)a; (void)s;
    b->spin++;
    if (b->spin % BOOT_TICK_EVERY == 0)
        dtm_raise_vector(b->e->m, BOOT_TICK_VEC, 0, -1);
}

/* SdGate: la porta C3 segue la D4 */
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

/* PanelMcu */
static void pm_usr8(e_boot *b)
{
    e_w8(b->e, E_USR8, (uint8_t)(PM_TXRDY | PM_TXEMP | (b->rxn ? PM_RXRDY : 0)));
}

static void hb_pm_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)ty; (void)size; (void)v;
    if (a >= E_USR8 && a < E_USR8 + 4) {
        pm_usr8(b);
    } else {
        uint8_t c = 0;
        if (b->rxn) {
            c = b->rx[0];
            memmove(b->rx, b->rx + 1, (size_t)--b->rxn);
        }
        e_w8(b->e, E_UDR8, c);
        pm_usr8(b);
    }
}

static void hb_pm_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    static const uint8_t req[4] = {0x60, 0x02, 0x70, 0x00};
    static const uint8_t reply[5] = {0x70, 0x0b, 0xff, 0x00, 0x02};
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
    memmove(b->tail, b->tail + 1, 3);
    b->tail[3] = (uint8_t)(v & 0xFF);
    if (b->ntx < 4)
        b->ntx++;
    if (!b->replied && b->ntx >= 4 && !memcmp(b->tail, req, 4)) {
        memcpy(b->rx + b->rxn, reply, sizeof reply);
        b->rxn += (int)sizeof reply;
        b->replied = 1;
    }
    pm_usr8(b);
}

static void hb_sd_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a;
    sd_on_serq((dt_engine *)ud, size, v);
}

/* -> lo stato a `point` istruzioni dal reset, DTST (compress: DTSZ) */
int dt_coldboot(const dt_firmware *fw, uint64_t point, int compress, uint8_t **out, size_t *len,
                volatile int *cancel, volatile int64_t *progress, char *err, size_t errlen)
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
    uint64_t done = 0;

    *out = NULL;
    *len = 0;
    if (!e || !b)
        goto out;
    b->e = e;
    e->fw = fw;
    e->volume = 1.0f;
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
        dt_errf(err, errlen, "unsupported firmware (MAIN OS %.16s...): no address profile", hex);
        r = DT_ERR_UNSUPPORTED;
        goto out;
    }
    e->flash_len = 0x1000000;                 /* dspboot.build_flash: 16 MB, container a 0x80000 */
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000)
        goto out;
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);
    memset(&mc, 0, sizeof mc);
    mc.idle_yield = 0;
    e->m = dt_machine_new(fw, &mc, err, errlen);
    if (!e->m) {
        r = DT_ERR_UNICORN;
        goto out;
    }
    e->m->no_mmio_at_load = 1;
    e->uc = e->m->uc;
    e_cur = e;
    /* board.install(m): il registro di revisione a 0x0 */
    board_install(e);
    /* gli hook di dspboot.run (fast=True), nel suo ordine */
    e_hook_code(e, (void *)h_flash_read, e, e->p->flash_read);
    e_hook_code(e, (void *)h_depack_clamp, e, e->p->depack_copy);
    e_hook_code(e, (void *)h_pend_call, e, e->p->pend_call);
    for (off = 0; off + 1 < e->main_os->len; off += 2)
        if (img[off] == 0x60 && img[off + 1] == 0xFE)
            e_hook_code(e, (void *)hb_halt, b, DT_MAIN_LOAD + (uint32_t)off);
    dtm_mmio_set(e->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(e->m, 0xEC03802Cu, 0x80000000u);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_gate_set, b, BOOT_PPDSDR_D, BOOT_PPDSDR_D);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_gate_clear, b, BOOT_PCLRR_D, BOOT_PCLRR_D);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_gate_sense, b, BOOT_PPDSDR_C, BOOT_PPDSDR_C);
    sd_init(e);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_sd_serq, e, E_SERQ, E_SERQ);
    /* PanelMcu */
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_pm_read, b, E_USR8, E_USR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_pm_read, b, E_UDR8, E_UDR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_pm_write, b, E_UDR8, E_UDR8 + 3);
    pm_usr8(b);
    /* mchw.install: I2C0, PIT1 a polling, il pannello */
    mc_board_hooks(e);
    dtm_install_mmio(e->m);
    /* Machine.load + premap_sdram + reset */
    for (off = 0; off < e->main_os->len; off += 0x100000)
        dtm_ensure(e->m, DT_MAIN_LOAD + off);
    dtm_ensure(e->m, DT_MAIN_LOAD);
    e_write(e, DT_MAIN_LOAD, img, e->main_os->len);
    for (i = 0x40000000u; i < 0x50000000u; i += PAGE)
        dtm_ensure(e->m, i);
    dtm_ensure(e->m, 0x40800000u);
    e_wr(e, UC_M68K_REG_SR, 0x2700);
    e_wr(e, UC_M68K_REG_A7, 0x40800000u);
    e_w32(e, BOOT_FLAGS_ADDR, BOOT_FLAGS);
    e->m->state_loaded = 1;
    e_wr(e, UC_M68K_REG_PC, BOOT_ENTRY);
    while (done < point) {
        uint64_t k = point - done < BOOT_STEP ? point - done : BOOT_STEP;
        uc_err ue;
        if (cancel && *cancel) {
            dt_errf(err, errlen, "interrotto");
            r = DT_ERR_IO;
            goto out;
        }
        ue = uc_emu_start(e->uc, e_rd(e, UC_M68K_REG_PC), 0, 0, (size_t)k);
        if (ue != UC_ERR_OK) {
            dt_errf(err, errlen, "cold boot stopped at %llu instructions: %s", (unsigned long long)done,
                    uc_strerror(ue));
            r = DT_ERR_UNICORN;
            goto out;
        }
        done += k;
        if (progress)
            *progress = (int64_t)done;
    }
    e->pc = e_rd(e, UC_M68K_REG_PC);
    e->boot_only = 1;
    e->instrs = (int64_t)done;
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

/* dt_coldboot fino a `rung`, poi il motore per `instrs` istruzioni senza tempo
   reale (giri da 5 ms emulati): l'animazione di avvio, la schermata principale e
   il +Drive pronti. Lo stato che esce parte a 40M istr/s, quello del plugin.
   `progress` (o NULL) riceve le istruzioni eseguite, `total` il totale previsto. */
int dt_coldboot_session(const dt_firmware *fw, uint64_t rung, int64_t instrs, int compress,
                        uint8_t **out, size_t *len, volatile int *cancel, volatile int64_t *progress,
                        char *err, size_t errlen)
{
    uint8_t *st = NULL;
    size_t n = 0;
    dt_engine *e;
    int r, last = 0;
    *out = NULL;
    *len = 0;
    r = dt_coldboot(fw, rung, 0, &st, &n, cancel, progress, err, errlen);
    if (r)
        return r;
    e = dt_engine_new_mem(fw, NULL, st, n, NULL, err, errlen);
    free(st);
    if (!e)
        return DT_ERR_FORMAT;
    for (;;) {
        if (dt_engine_instrs(e) >= instrs)
            last = 1;
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
        if (progress)
            *progress = (int64_t)rung + dt_engine_instrs(e);
        if (last)
            break;
    }
    r = dt_engine_save_mem(e, compress, out, len);
    dt_engine_free(e);
    return r;
}
