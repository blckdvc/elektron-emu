/* engine.c, parte 8: il primo avvio dal solo firmware (dt_coldboot_session) e
 * il riconoscimento del firmware (dt_fw_supported).
 *
 * Prima fase: il boot freddo dal reset, come emu/dspboot.py run (fast=True) e
 * la scala di emu/checkpoint.py make: la macchina con gli hook di dspboot
 * (flash, semaforo di completamento, depack, idle spin col tick del vettore
 * 32), la porta GPIO che abilita lo storage (emu/gpio.py SdGate), l'eSDHC e il
 * micro del pannello su UART8 (emu/panelmcu.py: l'unico scambio del boot
 * freddo e' la versione del pannello); il MAIN OS caricato a 0x40000400,
 * SR 0x2700, A7 0x40800000, PC all'ingresso (la prima longword
 * dell'immagine). Lo stato si salva all'istruzione numero `point`, prima di
 * eseguirla (make() lo fa nel hook di copertura): qui il hook di copertura
 * ferma l'emulazione li', e anche a ogni fetta di istruzioni, per dare la
 * mano a `progress` e `cancel`.
 *
 * Le fasi seguenti (ripresa sul motore completo, +Drive formattato dal
 * firmware, calibrazione, impostazioni SYNC) sono sotto, dopo il boot freddo.
 * Dal Digitakt II (digitakt/emulator/core/src/e_boot.inc.c), con le
 * specificita' del Rytm. */

#include <time.h>

#define BOOT_PPDSDR_C 0xEC09401Au
#define BOOT_PPDSDR_D 0xEC09401Bu
#define BOOT_PCLRR_D 0xEC094027u
#define BOOT_SLICE 5000000u                /* istruzioni fra due richiami di progress/cancel */

typedef struct {
    dt_engine *e;
    uint64_t n;                            /* st['n']: le istruzioni viste dal hook */
    uint64_t stop_at;                      /* ferma quando n arriva qui (poi n--) */
    dtm_site *tab;                         /* i siti, in ordine di indirizzo e di registrazione */
    size_t ntab;
    uint8_t *bits;                         /* un bit per ogni parola dell'immagine con un sito */
    uint32_t lo;
    uint64_t span;
    int64_t spin;
    int driven;
    /* emu/panelmcu.py PanelMcu('rytm') */
    uint8_t rx[16];
    int nrx;
    uint8_t tx2[2];                        /* gli ultimi due byte trasmessi */
    uint64_t ntx;
    int answered;
} e_boot;

/* gli hook per indirizzo di dspboot.run (la `table` dei siti): un solo hook
   globale conta le istruzioni e smista, con la mappa dei bit degli indirizzi
   che hanno un sito; migliaia di hook per indirizzo accanto a uno globale
   farebbero scorrere a Unicorn tutta la lista a ogni istruzione */
static void hb_cover(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_boot *b = (e_boot *)ud;
    uint64_t k;
    b->n++;
    if (b->n >= b->stop_at) {
        b->n--;                            /* l'istruzione non e' eseguita: ripartendo si ricontera' */
        uc_emu_stop(uc);
        b->e->m->stop_flag = 1;
        return;
    }
    k = (a - b->lo) >> 1;
    if (a >= b->lo && k < b->span && (b->bits[k >> 3] >> (k & 7) & 1)) {
        size_t lo = 0, hi = b->ntab;
        while (lo < hi) {                  /* il primo sito con indirizzo >= a */
            size_t mid = (lo + hi) / 2;
            if (b->tab[mid].addr < a)
                lo = mid + 1;
            else
                hi = mid;
        }
        for (; lo < b->ntab && b->tab[lo].addr == a; lo++)
            b->tab[lo].fn(uc, a, s, b->tab[lo].ud);
    }
}

typedef struct {
    dtm_site s;
    size_t seq;
} bsite_t;

static int cmp_site(const void *x, const void *y)
{
    const bsite_t *p = (const bsite_t *)x, *q = (const bsite_t *)y;
    if (p->s.addr != q->s.addr)
        return p->s.addr < q->s.addr ? -1 : 1;
    return p->seq < q->seq ? -1 : p->seq > q->seq;      /* stabile: l'ordine di registrazione */
}

/* i siti della macchina (ISA e MOVEC) poi quelli del boot, in ordine di
   indirizzo; la mappa dei bit per il filtro veloce */
static int boot_table(e_boot *b, dt_machine *m, const dtm_site *extra, size_t nextra, size_t img_len)
{
    size_t n = m->nsites + nextra, i;
    bsite_t *v = (bsite_t *)malloc((n ? n : 1) * sizeof *v);
    if (!v)
        return -1;
    for (i = 0; i < n; i++) {
        v[i].s = i < m->nsites ? m->sites[i] : extra[i - m->nsites];
        v[i].seq = i;
    }
    qsort(v, n, sizeof *v, cmp_site);
    b->tab = (dtm_site *)malloc((n ? n : 1) * sizeof *b->tab);
    b->lo = DT_MAIN_LOAD;
    b->span = img_len / 2 + 1;
    b->bits = (uint8_t *)calloc(b->span / 8 + 1, 1);
    if (!b->tab || !b->bits) {
        free(v);
        return -1;
    }
    for (i = 0; i < n; i++) {
        uint64_t k;
        b->tab[i] = v[i].s;
        k = (b->tab[i].addr - b->lo) >> 1;
        if (b->tab[i].addr >= b->lo && k < b->span)
            b->bits[k >> 3] |= (uint8_t)(1u << (k & 7));
    }
    b->ntab = n;
    free(v);
    return 0;
}

static void hb_halt(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)a; (void)s;
    b->spin++;
    if (b->spin % 20000 == 0)
        dtm_raise_vector(b->e->m, 32, 0, -1);
}

/* emu/gpio.py SdGate: la porta C bit 3 segue la porta D bit 4 */
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

/* emu/panelmcu.py PanelMcu: USR8 con RXRDY solo quando la FIFO ha dati; il
   solo scambio del boot freddo: 70 00 (versione del pannello) -> 70 07 00 00
   00 ("aggiornato"), come lo legge la convalida del firmware (0x4013ff38) */
static void pm_poke(e_boot *b, uint32_t addr, uint8_t v)
{
    uc_mem_write(b->e->uc, addr, &v, 1);       /* come _poke: errore ignorato se la pagina non c'e' */
}

static void hb_pm_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)ty; (void)size; (void)v;
    if (a >= E_USR8 && a < E_USR8 + 4)
        pm_poke(b, E_USR8, (uint8_t)(0x04 | 0x08 | (b->nrx ? 0x01 : 0)));
    else {
        uint8_t c = 0;
        if (b->nrx) {
            c = b->rx[0];
            memmove(b->rx, b->rx + 1, (size_t)--b->nrx);
        }
        pm_poke(b, E_UDR8, c);
        pm_poke(b, E_USR8, (uint8_t)(0x04 | 0x08 | (b->nrx ? 0x01 : 0)));
    }
}

static void hb_pm_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    static const uint8_t req[2] = {0x70, 0x00}, rep[5] = {0x70, 0x07, 0x00, 0x00, 0x00};
    e_boot *b = (e_boot *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
    b->tx2[0] = b->tx2[1];
    b->tx2[1] = (uint8_t)(v & 0xFF);
    b->ntx++;
    if (!b->answered && b->ntx >= 2 && !memcmp(b->tx2, req, 2)) {
        memcpy(b->rx + b->nrx, rep, sizeof rep);
        b->nrx += (int)sizeof rep;
        b->answered = 1;                   /* la risposta si da' una volta (replies.remove) */
    }
    pm_poke(b, E_USR8, (uint8_t)(0x04 | 0x08 | (b->nrx ? 0x01 : 0)));
}

/* Esdhc._on_serq, qui senza il multiplexer del motore */
static void hb_sd_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a;
    sd_on_serq(((e_boot *)ud)->e, size, v);
}

/* ---------------------------------------------------------- profili */
static const dt_profile *boot_profile(const dt_section *mo)
{
    dt_sha256 h;
    uint8_t sha[32];
    char hex[65];
    size_t i;
    if (!mo)
        return NULL;
    dt_sha256_init(&h);
    dt_sha256_update(&h, mo->data, mo->len);
    dt_sha256_final(&h, sha);
    dt_sha256_hex(sha, hex);
    for (i = 0; i < DT_NPROFILES; i++)
        if (!strcmp(DT_PROFILES[i].main_sha256, hex))
            return &DT_PROFILES[i];
    return NULL;
}

/* La versione dell'OS sta nell'intestazione del container ELE3: dopo il
   magic e una lunghezza, "<numero di build> <spazi> <versione>" (per l'OS
   1.73 "0173        1.73"). La sezione 2 ne ha un'altra, ma e' la versione
   del bootstrap (0x0701), non dell'OS. */
static void boot_version(const dt_firmware *fw, char *out, size_t n)
{
    char tok[32];
    size_t i, k = 0, last = 0;
    int dots = 0, digits = 0;
    if (!out || !n)
        return;
    out[0] = 0;
    if (!fw->container || fw->container_len < 40 || memcmp(fw->container, "ELE3", 4))
        return;
    for (i = 8; i < 40 && fw->container[i] >= 0x20 && fw->container[i] < 0x7F; i++) {
        if (fw->container[i] == ' ') {
            k = 0;
            continue;
        }
        if (k < sizeof tok - 1)
            tok[k++] = (char)fw->container[i];
        tok[k] = 0;
        last = k;
    }
    if (!last)
        return;
    for (i = 0; i < last; i++) {            /* ^[0-9]+\.[0-9]+[A-Za-z]*$ */
        char c = tok[i];
        if (c >= '0' && c <= '9')
            digits++;
        else if (c == '.' && !dots && digits)
            dots = 1;
        else if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) || !dots)
            return;
    }
    if (!dots || (tok[last - 1] == '.'))
        return;
    snprintf(out, n, "%s", tok);
}

int dt_fw_supported(const dt_firmware *fw, char *version, size_t vlen)
{
    if (version && vlen)
        version[0] = 0;
    if (!fw)
        return 0;
    if (version && vlen)
        boot_version(fw, version, vlen);
    return boot_profile(dt_fw_section(fw, 3)) != NULL;
}

/* ------------------------------------------------------- boot freddo */
typedef struct {
    dt_engine *e;
    e_boot *b;
    uint32_t pc;
} cold_t;

static void cold_free(cold_t *c)
{
    dt_engine *e = c->e;
    if (e) {
        if (e->m)
            dt_machine_free(e->m);
        free(e->flash);
        ov_free(&e->sd.ov);
        if (e_cur == e)
            e_cur = NULL;
        free(e);
    }
    if (c->b) {
        free(c->b->tab);
        free(c->b->bits);
    }
    free(c->b);
    memset(c, 0, sizeof *c);
}

/* la macchina di dspboot.run(fast=True, sdgate=True, esdhc=True) pronta al
   primo passo */
static int cold_new(cold_t *c, const dt_firmware *fw, char *err, size_t errlen)
{
    dt_engine *e = (dt_engine *)calloc(1, sizeof *e);
    e_boot *b = (e_boot *)calloc(1, sizeof *b);
    dt_machine_config mc;
    const uint8_t *img;
    size_t off;
    uint32_t entry;
    int r = DT_ERR_NOMEM;
    memset(c, 0, sizeof *c);
    c->e = e;
    c->b = b;
    if (!e || !b)
        goto fail;
    b->e = e;
    b->stop_at = ~0ull;
    e->fw = fw;
    e->main_os = dt_fw_section(fw, 3);
    if (!e->main_os) {
        dt_errf(err, errlen, "firmware senza MAIN OS");
        r = DT_ERR_FORMAT;
        goto fail;
    }
    e->p = boot_profile(e->main_os);
    if (!e->p) {
        dt_errf(err, errlen, "firmware non supportato: nessun profilo di indirizzi per questo MAIN OS");
        r = DT_ERR_UNSUPPORTED;
        goto fail;
    }
    img = e->main_os->data;
    entry = e_be32(img);                   /* la prima longword dell'immagine e' l'ingresso */
    if (e->main_os->len < 4 || entry < DT_MAIN_LOAD || entry >= DT_MAIN_LOAD + e->main_os->len) {
        dt_errf(err, errlen, "MAIN OS senza un ingresso valido");
        r = DT_ERR_FORMAT;
        goto fail;
    }
    e->flash_len = 0x1000000;
    e->flash = (uint8_t *)calloc(1, e->flash_len);
    if (!e->flash || !fw->container || fw->container_len > e->flash_len - 0x80000)
        goto fail;
    memcpy(e->flash + 0x80000, fw->container, fw->container_len);
    memset(&mc, 0, sizeof mc);
    mc.idle_yield = 0;
    mc.defer_sites = 1;
    e->m = dt_machine_new(fw, &mc, err, errlen);
    if (!e->m) {
        r = DT_ERR_UNICORN;
        goto fail;
    }
    e->m->no_mmio_at_load = 1;
    e->uc = e->m->uc;
    e_cur = e;
    /* dspboot.run: il hook globale (conteggio e smistamento), poi i siti ISA e
       MOVEC della macchina, poi i siti del firmware, nel loro ordine */
    {
        uc_hook h;
        dtm_site *x = (dtm_site *)malloc((4 + (e->main_os->len / 2)) * sizeof *x);
        size_t nx = 0;
        if (!x)
            goto fail;
        x[nx++] = (dtm_site){e->p->flash_read, h_flash_read, e};
        x[nx++] = (dtm_site){e->p->depack_copy, h_depack_clamp, e};
        x[nx++] = (dtm_site){e->p->pend_call, h_pend_call, e};
        for (off = 0; off + 1 < e->main_os->len; off += 2)
            if (img[off] == 0x60 && img[off + 1] == 0xFE)
                x[nx++] = (dtm_site){DT_MAIN_LOAD + (uint32_t)off, hb_halt, b};
        if (boot_table(b, e->m, x, nx, e->main_os->len)) {
            free(x);
            goto fail;
        }
        free(x);
        uc_hook_add(e->uc, &h, UC_HOOK_CODE, (void *)hb_cover, b, 1, 0);
    }
    /* ...le tre porte dei registri (USR8 e' del micro del pannello, piu' sotto)... */
    dtm_mmio_set(e->m, 0xFC05C02Cu, 0x100000F0u);
    dtm_mmio_set(e->m, 0xEC03802Cu, 0x80000000u);
    /* ...SdGate, Esdhc (anche dma_sem: senza, la prima CMD18 a DMA non finiva), PanelMcu */
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_gate_set, b, BOOT_PPDSDR_D, BOOT_PPDSDR_D);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_gate_clear, b, BOOT_PCLRR_D, BOOT_PCLRR_D);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_gate_sense, b, BOOT_PPDSDR_C, BOOT_PPDSDR_C);
    sd_init(e);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_sd_sysctl, e, SD_BASE + SD_SYSCTL, SD_BASE + SD_SYSCTL + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_xfertyp, e, SD_BASE + SD_XFERTYP, SD_BASE + SD_XFERTYP + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_sd_datport, e, SD_BASE + SD_DATPORT, SD_BASE + SD_DATPORT + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_sd_serq, b, E_SERQ, E_SERQ);
    pm_poke(b, E_USR8, 0x04 | 0x08);       /* PanelMcu.__init__: _poke(USR8, TXRDY|TXEMP) */
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_pm_read, b, E_USR8, E_USR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)hb_pm_read, b, E_UDR8, E_UDR8 + 3);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)hb_pm_write, b, E_UDR8, E_UDR8 + 3);
    dtm_install_mmio(e->m);
    /* Machine.load + premap_sdram + ensure(0x40800000) + reset */
    for (off = 0; off < e->main_os->len; off += 0x100000)
        dtm_ensure(e->m, DT_MAIN_LOAD + off);
    dtm_ensure(e->m, DT_MAIN_LOAD);
    e_write(e, DT_MAIN_LOAD, img, e->main_os->len);
    for (off = 0x40000000u; off < 0x50000000u; off += PAGE)
        dtm_ensure(e->m, off);
    dtm_ensure(e->m, 0x40800000u);
    e_wr(e, UC_M68K_REG_SR, 0x2700);
    e_wr(e, UC_M68K_REG_A7, 0x40800000u);
    e->m->state_loaded = 1;
    e->boot_only = 1;
    c->pc = entry;
    return DT_OK;
fail:
    cold_free(c);
    return r;
}

/* avanti fino a `point` istruzioni dal reset (fra una fetta e l'altra,
   progress e cancel); -> 0, DT_ERR_IO se interrotto, un altro errore */
static int cold_run(cold_t *c, uint64_t point, volatile int *cancel, dt_progress_fn progress, void *ctx,
                    double f0, double f1, const char *what, char *err, size_t errlen)
{
    e_boot *b = c->b;
    dt_engine *e = c->e;
    uint64_t target = point ? point - 1 : 0, done;
    e_cur = e;
    for (;;) {
        uc_err ue;
        uint64_t to;
        done = b->n;
        if (progress)
            progress(ctx, f0 + (f1 - f0) * (point ? (double)done / (double)point : 1.0), what);
        if (cancel && *cancel) {
            dt_errf(err, errlen, "interrotto");
            return DT_ERR_IO;
        }
        if (done >= target)
            break;
        to = done + BOOT_SLICE < target ? done + BOOT_SLICE : target;
        b->stop_at = to + 1;               /* il hook ferma quando n arriva a to + 1 (to eseguite) */
        e->m->stop_flag = 0;
        ue = uc_emu_start(e->uc, c->pc, 0, 0, 0);
        c->pc = e_rd(e, UC_M68K_REG_PC);
        if (ue != UC_ERR_OK) {
            dt_errf(err, errlen, "boot freddo fermo a %llu istruzioni: %s", (unsigned long long)b->n,
                    uc_strerror(ue));
            return DT_ERR_UNICORN;
        }
        if (b->n < to) {
            dt_errf(err, errlen, "boot freddo fermo a %llu istruzioni (nessun errore di Unicorn)",
                    (unsigned long long)b->n);
            return DT_ERR_UNICORN;
        }
    }
    return DT_OK;
}

/* lo stato (DTST) del boot freddo a `point` istruzioni dal reset: lo stesso di
   `python -m emu.checkpoint make point PREFIX SYX` (pagine, registri, MMIO ed
   eSDHC) */
int dt_coldboot(const dt_firmware *fw, uint64_t point, int compress, uint8_t **out, size_t *len, char *err,
                size_t errlen)
{
    cold_t c;
    int r = cold_new(&c, fw, err, errlen);
    if (out) *out = NULL;
    if (len) *len = 0;
    if (r)
        return r;
    r = cold_run(&c, point, NULL, NULL, NULL, 0, 1, "", err, errlen);
    if (!r) {
        c.e->pc = c.pc;
        r = dt_engine_save_mem(c.e, compress, out, len);
    }
    cold_free(&c);
    return r;
}

/* ===================================================== la sessione */
/* Dopo il boot freddo: lo stato del boot entra nel motore completo (60M
 * istruzioni per secondo emulato, come il plugin), che porta il firmware alla
 * schermata principale; poi il +Drive si formatta DAL FIRMWARE con i tasti
 * simulati (README "Formattazione +Drive"), si calibra, si impostano CLOCK e
 * TRANSPORT RECEIVE (tests/scenarios/plugin_sync_setup.txt) e si salva. Il
 * motore e' deterministico: gli ingressi sono a istanti fissi di istruzioni,
 * e la formattazione si verifica poi dal +Drive (ReadDir via RPC).
 * I testi di `progress` sono frasi inglesi per l'utente del plugin. */
#define SESS_BOOT_RUNG 60000000ull          /* il gradino della scala: come tools/coldcheck.py 60000000 */
#define SESS_ALERT_AT 1000000000ll          /* l'alert "FAULTY FILESYSTEM" e' sullo schermo da ~914M */
#define SESS_PRESS 2000000ll                /* istruzioni fra pressione e rilascio */
#define SESS_GAP 20000000ll                 /* fra una pressione e la successiva */
#define SESS_REPORT 50000000ll              /* progress a ogni ~50M istruzioni */

/* codici dei tasti (devices/analog-rytm-mkii.toml [panel.names]) */
enum { K_YES = 41, K_NO = 33, K_UP = 42, K_DOWN = 35, K_FX = 7, K_SETTINGS = 53, K_SAMPLING = 52, K_STOP = 15 };

typedef struct {
    dt_engine *e;
    volatile int *cancel;
    dt_progress_fn progress;
    void *ctx;
    char *err;
    size_t errlen;
    double f0, f1;                          /* l'intervallo di frazioni della fase corrente */
    const char *what;
    int64_t start, span;                    /* istruzioni all'inizio della fase e attese della fase (0 = ignoto) */
    int64_t next_report;
    int verbose;
    double t_phase;
    int64_t i_phase;
    const char *phase_name;
} sess_t;

/* AR_BOOT_DUMP=cartella: lo stato (DTST) dopo ogni fase importante, per le prove */
static void sess_dump(sess_t *s, const char *name)
{
    const char *dir = getenv("AR_BOOT_DUMP");
    char path[1024];
    if (dir && dir[0]) {
        snprintf(path, sizeof path, "%s/%s.dtst", dir, name);
        dt_engine_save(s->e, path, 0);
    }
}

static double boot_now(void)                 /* solo per le misure di AR_BOOT_VERBOSE */
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

static void sess_report(sess_t *s, double frac)
{
    if (frac < 0)
        frac = 0;
    if (frac > 1)
        frac = 1;
    if (s->progress)
        s->progress(s->ctx, s->f0 + (s->f1 - s->f0) * frac, s->what);
}

/* inizia una fase (progress subito) */
static void sess_phase(sess_t *s, const char *what, double f0, double f1, int64_t span)
{
    double t = boot_now();
    if (s->verbose && s->phase_name)
        fprintf(stderr, "[coldboot] %-24s %8.1f M istruzioni %7.2f s\n", s->phase_name,
                (double)(dt_engine_instrs(s->e) - s->i_phase) / 1e6, t - s->t_phase);
    s->phase_name = what;
    s->t_phase = t;
    s->i_phase = dt_engine_instrs(s->e);
    s->what = what;
    s->f0 = f0;
    s->f1 = f1;
    s->start = dt_engine_instrs(s->e);
    s->span = span;
    s->next_report = s->start + SESS_REPORT;
    sess_report(s, 0);
}

/* un giro del motore, con cancel e progress; -> 0, o l'errore */
static int sess_chunk(sess_t *s)
{
    int64_t n;
    if (s->cancel && *s->cancel) {
        dt_errf(s->err, s->errlen, "interrotto");
        return DT_ERR_IO;
    }
    if (dt_engine_chunk(s->e)) {
        dt_errf(s->err, s->errlen, "the firmware stopped: %s", dt_engine_status(s->e));
        return DT_ERR_UNICORN;
    }
    n = dt_engine_instrs(s->e);
    if (n >= s->next_report) {
        s->next_report = n + SESS_REPORT;
        sess_report(s, s->span ? (double)(n - s->start) / (double)s->span : 0.0);
    }
    return DT_OK;
}

static int sess_run_to(sess_t *s, int64_t target)
{
    int r;
    while (dt_engine_instrs(s->e) < target)
        if ((r = sess_chunk(s)))
            return r;
    return DT_OK;
}

/* un tasto: pressione, SESS_PRESS istruzioni, rilascio, fino a `gap` dalla pressione */
static int sess_key(sess_t *s, int code, int64_t gap)
{
    int64_t t0 = dt_engine_instrs(s->e);
    int r;
    dt_engine_panel(s->e, DT_PANEL_PRESS, code, 0);
    if ((r = sess_run_to(s, t0 + SESS_PRESS)))
        return r;
    dt_engine_panel(s->e, DT_PANEL_RELEASE, code, 0);
    return sess_run_to(s, t0 + gap);
}

static int sess_keys(sess_t *s, const int *codes, int n, int64_t gap)
{
    int i, r;
    for (i = 0; i < n; i++)
        if ((r = sess_key(s, codes[i], gap)))
            return r;
    return DT_OK;
}

/* RPC ReadDir "/" (emu/transfer.py Client.ls): -> 1 se ci sono `factory` e `incoming`,
   0 se la risposta non c'e' o non li ha, < 0 errore. Aspetta fino a `wait` istruzioni. */
static int sess_drive_ok(sess_t *s, int64_t wait)
{
    static const uint8_t root[2] = {'/', 0};
    uint16_t seq = 0;
    uint8_t *buf = NULL, tmp[4096];
    size_t have = 0, cap = 0;
    int64_t until = dt_engine_instrs(s->e) + wait;
    int r, found = 0;
    if ((r = dt_engine_rpc(s->e, 0x10, root, sizeof root, &seq))) {
        dt_errf(s->err, s->errlen, "the +Drive link is not available");
        return r < 0 ? r : -1;
    }
    while (dt_engine_instrs(s->e) < until && !found) {
        size_t k;
        if ((r = sess_chunk(s))) {
            free(buf);
            return r;
        }
        k = dt_engine_rpc_replies(s->e, tmp, sizeof tmp);
        if (k) {
            if (have + k > cap) {
                uint8_t *q = (uint8_t *)realloc(buf, (cap = (have + k) * 2));
                if (!q) {
                    free(buf);
                    return DT_ERR_NOMEM;
                }
                buf = q;
            }
            memcpy(buf + have, tmp, k);
            have += k;
        }
        while (have >= 7) {                           /* [u16 seq, u8 id, u32 n, n byte] */
            uint32_t bl = (uint32_t)buf[3] << 24 | (uint32_t)buf[4] << 16 | (uint32_t)buf[5] << 8 | buf[6];
            if (have < 7 + (size_t)bl)
                break;
            if ((uint16_t)(buf[0] << 8 | buf[1]) == seq) {
                size_t i = 7, end = 7 + (size_t)bl;
                int fac = 0, inc = 0;
                while (i + 10 <= end) {               /* hash, size, protetto, tipo, nome\0 */
                    size_t j = i + 10;
                    while (j < end && buf[j])
                        j++;
                    if (j >= end)
                        break;
                    if (!strcmp((const char *)buf + i + 10, "factory"))
                        fac = 1;
                    if (!strcmp((const char *)buf + i + 10, "incoming"))
                        inc = 1;
                    i = j + 1;
                }
                found = fac && inc ? 1 : -1;
                break;
            }
            memmove(buf, buf + 7 + bl, have - 7 - bl);
            have -= 7 + bl;
        }
    }
    free(buf);
    return found > 0 ? 1 : 0;
}

/* tests/scenarios/plugin_sync_setup.txt: STOP, SETTINGS > MIDI CONFIG > SYNC >
   CLOCK RECEIVE acceso (TRANSPORT RECEIVE era gia' acceso), poi SETTINGS
   chiude; stessi istanti (il primo tasto a 3M, uno ogni 8M, premuto per 3M).
   In piu', 8 SU dopo SETTINGS: il menu si riapre dove l'ha lasciato (dopo il
   format, su SYSTEM) e il cursore si ferma in cima senza girare, cosi' i 3 GIU'
   arrivano sempre a MIDI CONFIG. */
static const uint8_t SYNC_KEYS[] = {
    K_STOP, K_SETTINGS,
    K_UP, K_UP, K_UP, K_UP, K_UP, K_UP, K_UP, K_UP,
    K_DOWN, K_DOWN, K_DOWN, K_FX, K_YES, K_FX, K_YES, K_FX, K_YES, K_FX,
    K_NO, K_NO, K_SETTINGS, K_FX,
};
#define SYNC_FIRST 3000000ll
#define SYNC_PERIOD 8000000ll
#define SYNC_HOLD 3000000ll
#define SYNC_SETTLE 30000000ll              /* dopo l'ultimo tasto: ad assestarsi */

static int sess_sync_setup(sess_t *s)
{
    size_t n = sizeof SYNC_KEYS, press = 0, release = 0;
    int64_t t0 = dt_engine_instrs(s->e);
    int64_t last = SYNC_FIRST + SYNC_PERIOD * (int64_t)(n - 1) + SYNC_HOLD + SYNC_SETTLE;
    int r;
    while (dt_engine_instrs(s->e) - t0 < last) {
        int64_t t = dt_engine_instrs(s->e) - t0;
        while (press < n && SYNC_FIRST + SYNC_PERIOD * (int64_t)press <= t) {
            dt_engine_panel(s->e, DT_PANEL_PRESS, SYNC_KEYS[press], 0);
            press++;
        }
        while (release < press && SYNC_FIRST + SYNC_PERIOD * (int64_t)release + SYNC_HOLD <= t) {
            dt_engine_panel(s->e, DT_PANEL_RELEASE, SYNC_KEYS[release], 0);
            release++;
        }
        if ((r = sess_chunk(s)))
            return r;
    }
    return DT_OK;
}

typedef struct {
    sess_t *s;
    double f0, f1;
} cal_fwd;

static void sess_cal_progress(void *c0, double frac, const char *what)
{
    cal_fwd *c = (cal_fwd *)c0;
    if (c->s->progress)
        c->s->progress(c->s->ctx, c->f0 + (c->f1 - c->f0) * (frac < 0 ? 0 : frac > 1 ? 1 : frac), what);
}

int dt_coldboot_session(const dt_firmware *fw, int compress, uint8_t **out, size_t *len, volatile int *cancel,
                        dt_progress_fn progress, void *ctx, char *err, size_t errlen)
{
    static const int alerts[] = {K_YES, K_YES, K_NO, K_NO};           /* l'alert, "MISSING CAL", "RUN NOW? Y/N" */
    /* SAMPLING poi SETTINGS (SETTINGS dalla schermata principale a volte non ridisegna), 7 giu' fino a
       SYSTEM, YES; 3 giu' fino a FORMAT +DRIVE, YES; giu' su SAMPLES, YES (spunta); giu' su ERASE
       SELECTED DATA, YES; YES su "ARE YOU SURE?". Dopo ogni spostamento FX forza il ridisegno. */
    static const int format_keys[] = {K_SAMPLING, K_SETTINGS, K_FX,
                                      K_DOWN, K_DOWN, K_DOWN, K_DOWN, K_DOWN, K_DOWN, K_DOWN, K_FX, K_YES, K_FX,
                                      K_DOWN, K_DOWN, K_DOWN, K_FX, K_YES, K_FX,
                                      K_DOWN, K_YES, K_FX, K_DOWN, K_FX, K_YES, K_FX, K_YES};
    static const int leave_keys[] = {K_NO, K_NO, K_NO, K_NO, K_NO};   /* SUCCESS, FORMAT, SYSTEM, GLOBAL, (resto) */
    cold_t c;
    sess_t S;
    dt_engine *e = NULL;
    uint8_t *st = NULL;
    size_t stn = 0;
    const char *skip;
    int r, tries;
    memset(&S, 0, sizeof S);
    S.cancel = cancel;
    S.progress = progress;
    S.ctx = ctx;
    S.err = err;
    S.errlen = errlen;
    S.verbose = getenv("AR_BOOT_VERBOSE") != NULL;
    if (out)
        *out = NULL;
    if (len)
        *len = 0;
    if (!fw || !out || !len)
        return DT_ERR_ARG;

    /* 1. il boot freddo dal reset (la scala: 60M istruzioni) */
    r = cold_new(&c, fw, err, errlen);
    if (r)
        return r;
    {
        double t0 = boot_now();
        r = cold_run(&c, SESS_BOOT_RUNG, cancel, progress, ctx, 0.0, 0.03, "Booting the firmware", err, errlen);
        if (!r) {
            c.e->pc = c.pc;
            r = dt_engine_save_mem(c.e, 0, &st, &stn);
        }
        if (S.verbose)
            fprintf(stderr, "[coldboot] %-24s %8.1f M istruzioni %7.2f s\n", "Booting the firmware",
                    (double)SESS_BOOT_RUNG / 1e6, boot_now() - t0);
    }
    cold_free(&c);
    if (r)
        return r;

    /* 2. la ripresa sul motore completo fino all'alert */
    e = dt_engine_new_mem(fw, NULL, st, stn, NULL, err, errlen);
    free(st);
    if (!e)
        return DT_ERR_FORMAT;
    S.e = e;
    sess_phase(&S, "Starting the engine", 0.03, 0.40, SESS_ALERT_AT);
    if ((r = sess_run_to(&S, SESS_ALERT_AT)))
        goto done;

    /* 3. il +Drive formattato dal firmware; si verifica dal +Drive stesso */
    sess_phase(&S, "Formatting the +Drive", 0.40, 0.72, 900000000ll);
    if ((r = sess_keys(&S, alerts, 4, SESS_GAP)))
        goto done;
    if ((r = sess_keys(&S, format_keys, (int)(sizeof format_keys / sizeof format_keys[0]), SESS_GAP)))
        goto done;
    for (tries = 0, r = 0; tries < 8; tries++) {         /* la formattazione dura ~100M istruzioni */
        if ((r = sess_run_to(&S, dt_engine_instrs(e) + 60000000ll)))
            goto done;
        r = sess_drive_ok(&S, 40000000ll);
        if (r)
            break;
    }
    if (r < 0)
        goto done;
    if (!r) {
        dt_errf(err, errlen, "the firmware did not format the +Drive");
        r = DT_ERR_UNICORN;
        goto done;
    }
    if ((r = sess_keys(&S, leave_keys, 5, SESS_GAP)))     /* torna alla schermata principale */
        goto done;

    /* 4. la calibrazione analogica sui nostri modelli (AR_SKIP_CALIB=1: senza, per le prove) */
    sess_phase(&S, "Calibrating the analog circuits", 0.72, 0.92, 0);
    skip = getenv("AR_SKIP_CALIB");
    sess_dump(&S, "dopo_format");
    if (!(skip && !strcmp(skip, "1"))) {
        cal_fwd cf;
        cf.s = &S;
        cf.f0 = 0.72;
        cf.f1 = 0.92;
        r = dt_engine_calibrate(e, cancel, sess_cal_progress, &cf, err, errlen);
        if (r)
            goto done;
        sess_dump(&S, "dopo_calibrazione");
    }

    /* 5. CLOCK/TRANSPORT RECEIVE, e il salvataggio */
    sess_phase(&S, "Setting up MIDI sync", 0.92, 0.98, 225000000ll);
    if ((r = sess_sync_setup(&S)))
        goto done;
    sess_phase(&S, "Saving the state", 0.98, 1.0, 0);
    r = dt_engine_save_mem(e, compress, out, len);
    sess_report(&S, 1.0);
    if (S.verbose)
        fprintf(stderr, "[coldboot] totale %.1f G istruzioni, stato %.1f MB\n",
                (double)dt_engine_instrs(e) / 1e9, (double)*len / 1e6);
done:
    dt_engine_free(e);
    return r;
}
