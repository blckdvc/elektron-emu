/* La macchina di base: emu/harness.py Machine in C, sopra Unicorn.
 *
 * Equivalenze (riga per riga, stesso ordine di letture e scritture):
 *   on_invalid    Machine._fault + ensure: mappa la pagina da 1 MB e riprova
 *   on_mmio       Machine.install_mmio (scoped): riscrive il valore forzato
 *                 (4 byte BE) prima di ogni lettura dell'indirizzo
 *   on_movec      install_isa_patches_scoped make_movec
 *   on_isa        install_isa_patches_scoped con gli hook C del Rytm
 *                 (nativehooks.isa_hooks, dt2_isa_site): FF1/BITREV/BYTEREV
 *                 senza codici di condizione, nello stesso ordine
 *   on_intr       install_exceptions on_intr: RTE (EXCP_RTE 0x100) e
 *                 dtm_raise_vector(vec, from_instruction=True)
 *   raise_vector  Machine.raise_vector senza srtrap
 *   on_idle       longrun.build do_halt senza canale TX: n += 1, ogni
 *                 idle_yield passaggi dtm_raise_vector(32)
 * Lo stato si carica da un file DTST v1 (tools/snap2dtst.py).
 */
#include "arcore.h"
#include "internal.h"
#include "sha256.h"
#include "machine.h"

#include <stdlib.h>
#include <string.h>

#include <zlib.h>

#if defined(__APPLE__)
#include <sys/mman.h>
#endif

/* ------------------------------------------------------------ utilita' */
uint32_t dtm_rd32(uc_engine *uc, int reg)
{
    uint32_t v = 0;
    uc_reg_read(uc, reg, &v);
    return v;
}

void dtm_wr32(uc_engine *uc, int reg, uint32_t v)
{
    uc_reg_write(uc, reg, &v);
}

static uint32_t be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(p[0] << 8 | p[1]);
}

int dtm_is_mapped(dt_machine *m, uint32_t base)
{
    uint32_t i = base >> 20;
    return (m->mapped[i >> 3] >> (i & 7)) & 1;
}

/* pagine da 1 MB nostre, a zero e allineate (come quelle che alloca Unicorn) */
static uint8_t *page_alloc(void)
{
    void *p = NULL;
#if defined(_WIN32)
    p = _aligned_malloc(PAGE, 65536);
#else
    if (posix_memalign(&p, 65536, PAGE))
        p = NULL;
#endif
    if (p)
        memset(p, 0, PAGE);
    return (uint8_t *)p;
}

static void page_free(uint8_t *p)
{
#if defined(_WIN32)
    _aligned_free(p);
#else
    free(p);
#endif
}

uint8_t *dtm_host(dt_machine *m, uint32_t base)
{
    return m->host[base >> 20];
}

/* Machine.ensure. La RAM e' nostra (uc_mem_map_ptr): lo stato si fotografa con
   una copia diretta invece di uc_mem_read (~10 volte piu' lento), cosi' il
   salvataggio non ferma il motore abbastanza da sentirsi (dal Digitakt II). */
void dtm_ensure(dt_machine *m, uint64_t addr)
{
    uint32_t base = (uint32_t)(addr & ~(uint64_t)(PAGE - 1)), i;
    uint8_t *h;
    if (dtm_is_mapped(m, base))
        return;
    h = page_alloc();
    if (!h)
        return;
    if (uc_mem_map_ptr(m->uc, base, PAGE, UC_PROT_ALL, h) == UC_ERR_OK) {
        i = base >> 20;
        m->mapped[i >> 3] |= (uint8_t)(1u << (i & 7));
        m->host[i] = h;
    } else {
        page_free(h);
    }
}

static int ctl_get(dt_machine *m, uint32_t rc, uint32_t *v)
{
    size_t i;
    for (i = 0; i < m->nctl; i++)
        if (m->ctl[i].key == rc) {
            *v = m->ctl[i].value;
            return 1;
        }
    return 0;
}

static int ctl_set(dt_machine *m, uint32_t rc, uint32_t v)
{
    size_t i;
    for (i = 0; i < m->nctl; i++)
        if (m->ctl[i].key == rc) {
            m->ctl[i].value = v;
            return 0;
        }
    if (m->nctl == m->capctl) {
        size_t cap = m->capctl ? 2 * m->capctl : 16;
        kv32 *p = (kv32 *)realloc(m->ctl, cap * sizeof *p);
        if (!p)
            return -1;
        m->ctl = p;
        m->capctl = cap;
    }
    m->ctl[m->nctl].key = rc;
    m->ctl[m->nctl].value = v;
    m->nctl++;
    return 0;
}

/* --------------------------------------------------------------- hook */
static bool on_invalid(uc_engine *uc, uc_mem_type type, uint64_t addr, int size,
                       int64_t value, void *ud)
{
    dt_machine *m = (dt_machine *)ud;
    (void)uc; (void)type; (void)size; (void)value;
    m->cnt.faults_mapped++;
    dtm_ensure(m, addr);
    return true;
}

static void on_mmio(uc_engine *uc, uc_mem_type type, uint64_t addr, int size,
                    int64_t value, void *ud)
{
    mmio_site *s = (mmio_site *)ud;
    uint8_t b[4] = {(uint8_t)(s->value >> 24), (uint8_t)(s->value >> 16),
                    (uint8_t)(s->value >> 8), (uint8_t)s->value};
    (void)type; (void)addr; (void)size; (void)value;
    uc_mem_write(uc, s->addr, b, 4);
}

static void on_movec(uc_engine *uc, uint64_t address, uint32_t size, void *ud)
{
    movec_site *s = (movec_site *)ud;
    dt_machine *m = s->m;
    uint8_t b[2] = {0, 0};
    uint32_t ext, rc, v;
    int reg;
    (void)address; (void)size;
    uc_mem_read(uc, s->addr + 2, b, 2);
    ext = be16(b);
    rc = ext & 0x0FFF;
    reg = UC_M68K_REG_D0 + (int)((ext >> 12) & 7);
    if (s->w == 0x4E7B) {
        ctl_set(m, rc, dtm_rd32(uc, reg));
    } else {
        v = 0;
        ctl_get(m, rc, &v);
        dtm_wr32(uc, reg, v);
    }
    dtm_wr32(uc, UC_M68K_REG_PC, s->addr + 4);
    m->movec_count++;
    m->cnt.movec++;
}

/* dt2_isa_site (native/dt2_hooks.c, nativehooks.isa_hooks): il registro,
   poi PC oltre l'istruzione di 2 byte, poi il contatore */
static void on_isa(uc_engine *uc, uint64_t address, uint32_t size, void *ud)
{
    mach_isa_site *t = (mach_isa_site *)ud;
    uint32_t v = 0, out = 0, pc = (uint32_t)address + 2u;
    int k;
    (void)size;
    if (uc_reg_read(uc, t->regid, &v) != UC_ERR_OK)
        return;
    if (t->kind == 0) {                 /* 32 - bit_length: FF1 di 0 e' 32 */
        out = 32;
        while (v) { out--; v >>= 1; }
    } else if (t->kind == 1) {
        for (k = 0; k < 32; k++)
            out |= ((v >> k) & 1u) << (31 - k);
    } else
        out = (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) | (v << 24);
    uc_reg_write(uc, t->regid, &out);
    uc_reg_write(uc, UC_M68K_REG_PC, &pc);
    t->m->isa_counts[t->kind]++;
}

/* Machine.raise_vector (srtrap spento). level < 0 = None. -> 1 preso, 0 no */
int dtm_raise_vector(dt_machine *m, uint32_t vec, int from_instruction, int level)
{
    uc_engine *uc = m->uc;
    uint8_t b[8];
    uint32_t handler, pc, sr, sp, live;
    uint16_t fmt;
    if (uc_mem_read(uc, DT_VBR + vec * 4u, b, 4) != UC_ERR_OK)
        return 0;
    handler = be32(b);
    if (handler == 0 || handler >= MAX_HANDLER)
        return 0;
    pc = dtm_rd32(uc, UC_M68K_REG_PC);
    if (from_instruction && uc_mem_read(uc, pc, b, 2) == UC_ERR_OK) {
        uint16_t w = be16(b);
        if (w >= 0x4E40 && w <= 0x4E4F)      /* trap #0 .. trap #15: riprende dopo */
            pc += 2;
    }
    sr = dtm_rd32(uc, UC_M68K_REG_SR);
    sp = dtm_rd32(uc, UC_M68K_REG_A7) - 8;
    dtm_ensure(m, sp);                            /* Machine._ensure_frame */
    dtm_ensure(m, (uint64_t)sp + 8 - 1);
    fmt = (uint16_t)(0x4000 | ((vec << 2) & 0x0FFC));
    b[0] = (uint8_t)(fmt >> 8); b[1] = (uint8_t)fmt;
    b[2] = (uint8_t)(sr >> 8); b[3] = (uint8_t)sr;
    b[4] = (uint8_t)(pc >> 24); b[5] = (uint8_t)(pc >> 16); b[6] = (uint8_t)(pc >> 8); b[7] = (uint8_t)pc;
    uc_mem_write(uc, sp, b, 8);
    dtm_wr32(uc, UC_M68K_REG_A7, sp);
    live = sr | 0x2000;
    if (level >= 0)
        live = (live & ~0x0700u) | ((uint32_t)(level & 7) << 8);
    dtm_wr32(uc, UC_M68K_REG_SR, live);
    dtm_wr32(uc, UC_M68K_REG_PC, handler);
    m->cnt.raised++;
    return 1;
}

static void on_intr(uc_engine *uc, uint32_t intno, void *ud)
{
    dt_machine *m = (dt_machine *)ud;
    if (intno == EXCP_RTE) {
        uint8_t b[8];
        uint32_t sp = dtm_rd32(uc, UC_M68K_REG_A7);
        if (uc_mem_read(uc, sp, b, 8) != UC_ERR_OK) {
            /* il Python solleverebbe UcError dentro l'hook: ci si ferma */
            m->cnt.halt_vec = (int32_t)intno;
            dtm_stop(m);
            return;
        }
        dtm_wr32(uc, UC_M68K_REG_SR, be16(b + 2));
        dtm_wr32(uc, UC_M68K_REG_PC, be32(b + 4));
        dtm_wr32(uc, UC_M68K_REG_A7, sp + 8);
        m->cnt.rte++;
        return;
    }
    if (!dtm_raise_vector(m, intno, 1, -1)) {
        m->cnt.halt_vec = (int32_t)intno;
        dtm_stop(m);
    }
}

static void on_idle(uc_engine *uc, uint64_t address, uint32_t size, void *ud)
{
    dt_machine *m = (dt_machine *)ud;
    (void)uc; (void)address; (void)size;
    m->idle_n++;
    m->cnt.idle_spins++;
    if (m->idle_n % (uint64_t)m->idle_yield == 0)
        dtm_raise_vector(m, 32, 0, -1);
}

void dtm_stop(dt_machine *m)
{
    m->stop_flag = 1;
    uc_emu_stop(m->uc);
}

/* ------------------------------------------------------- costruzione */
static int emac_supported(uc_engine *uc)
{
    /* emu/snapshot.py emac_supported: la Unicorn originale risponde ok a
       questi id senza leggere ne' scrivere; si prova a cambiare MASK. */
    uint32_t old = 0, flip, probe = 0;
    if (uc_reg_read(uc, UC_M68K_REG_MAC_MASK, &old) != UC_ERR_OK)
        return 0;
    flip = old ^ 0x5A5A;
    uc_reg_write(uc, UC_M68K_REG_MAC_MASK, &flip);
    uc_reg_read(uc, UC_M68K_REG_MAC_MASK, &probe);
    uc_reg_write(uc, UC_M68K_REG_MAC_MASK, &old);
    return probe == flip;
}

/* Unicorn chiama exit(1) se non puo' allocare la memoria del JIT ("Could not
   allocate dynamic translator buffer", qemu/accel/tcg/translate-all.c): in un
   plugin ucciderebbe l'host. Su macOS col runtime protetto serve
   l'entitlement com.apple.security.cs.allow-jit (misurato: senza, l'eseguibile
   firmato esce cosi'), quindi prima di uc_open si prova una pagina MAP_JIT. */
int dt_jit_available(void)
{
#if defined(__APPLE__) && defined(MAP_JIT)
    void *p = mmap(NULL, 16384, PROT_READ | PROT_WRITE | PROT_EXEC,
                   MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);
    if (p == MAP_FAILED)
        return 0;
    munmap(p, 16384);
#endif
    return 1;
}

/* un hook di codice per indirizzo, o (defer_sites) solo il suo posto in m->sites */
static int site_add(dt_machine *m, void *fn, void *ud, uint32_t a)
{
    uc_hook hh;
    if (!m->defer_sites)
        return uc_hook_add(m->uc, &hh, UC_HOOK_CODE, fn, ud, a, a) != UC_ERR_OK;
    if (m->nsites == m->capsites) {
        size_t cap = m->capsites ? 2 * m->capsites : 4096;
        dtm_site *p = (dtm_site *)realloc(m->sites, cap * sizeof *p);
        if (!p)
            return 1;
        m->sites = p;
        m->capsites = cap;
    }
    m->sites[m->nsites].addr = a;
    m->sites[m->nsites].fn = (void (*)(uc_engine *, uint64_t, uint32_t, void *))fn;
    m->sites[m->nsites].ud = ud;
    m->nsites++;
    return 0;
}

dt_machine *dt_machine_new(const dt_firmware *fw, const dt_machine_config *cfg, char *err, size_t errlen)
{
    const dt_section *main_os;
    dt_machine *m;
    uc_hook hh;
    size_t off, n;
    uc_err e;

    main_os = dt_fw_section(fw, 3);
    if (!main_os || main_os->len < 2) {
        dt_errf(err, errlen, "firmware without a MAIN OS section (3)");
        return NULL;
    }
    if (!dt_jit_available()) {
        dt_errf(err, errlen, "no JIT memory (macOS: the process needs com.apple.security.cs.allow-jit)");
        return NULL;
    }
    m = (dt_machine *)calloc(1, sizeof *m);
    if (!m)
        return NULL;
    m->cnt.halt_vec = -1;
    m->idle_yield = cfg ? cfg->idle_yield : 20000;
    m->defer_sites = cfg ? cfg->defer_sites : 0;
    {
        dt_sha256 h;
        dt_sha256_init(&h);
        dt_sha256_update(&h, main_os->data, main_os->len);
        dt_sha256_final(&h, m->main_sha256);
    }
    m->page = (uint8_t *)malloc(PAGE);
    if (!m->page)
        goto nomem;
    e = uc_open(UC_ARCH_M68K, UC_MODE_BIG_ENDIAN, &m->uc);
    if (e != UC_ERR_OK) {
        dt_errf(err, errlen, "uc_open: %s", uc_strerror(e));
        free(m->page);
        free(m);
        return NULL;
    }
    uc_ctl_set_cpu_model(m->uc, UC_CPU_M68K_CFV4E);
    if (!emac_supported(m->uc)) {
        dt_errf(err, errlen, "Unicorn without the mac-regs patch");
        dt_machine_free(m);
        return NULL;
    }
    /* Machine.__init__: il fault hook su tutto (begin=1, end=0) */
    uc_hook_add(m->uc, &hh, UC_HOOK_MEM_INVALID, (void *)on_invalid, m, 1, 0);

    /* install_isa_patches_scoped: un hook per ogni parola MOVEC, FF1, BITREV,
       BYTEREV del MAIN OS, nell'ordine degli indirizzi */
    for (n = 0, off = 0; off + 1 < main_os->len; off += 2) {
        uint16_t w = be16(main_os->data + off);
        if (w == 0x4E7A || w == 0x4E7B)
            n++;
        if ((w >= 0x00C0 && w <= 0x00C7) || (w >= 0x02C0 && w <= 0x02C7) || (w >= 0x04C0 && w <= 0x04C7))
            m->nisa++;
    }
    m->movec = (movec_site *)calloc(n ? n : 1, sizeof *m->movec);
    m->isa = (mach_isa_site *)calloc(m->nisa ? m->nisa : 1, sizeof *m->isa);
    if (!m->movec || !m->isa)
        goto nomem;
    m->nisa = 0;
    for (off = 0; off + 1 < main_os->len; off += 2) {
        uint16_t w = be16(main_os->data + off);
        uint32_t a = DT_MAIN_LOAD + (uint32_t)off;
        if ((w >= 0x00C0 && w <= 0x00C7) || (w >= 0x02C0 && w <= 0x02C7)) {
            mach_isa_site *t = &m->isa[m->nisa++];
            t->m = m;
            t->kind = w < 0x0200 ? 1 : 2;
            t->regid = UC_M68K_REG_D0 + (w & 7);
            site_add(m, (void *)on_isa, t, a);
        }
        if (w >= 0x04C0 && w <= 0x04C7) {
            mach_isa_site *t = &m->isa[m->nisa++];
            t->m = m;
            t->kind = 0;
            t->regid = UC_M68K_REG_D0 + (w & 7);
            site_add(m, (void *)on_isa, t, a);
        } else if (w == 0x4E7A || w == 0x4E7B) {
            movec_site *s = &m->movec[m->nmovec++];
            s->m = m;
            s->addr = a;
            s->w = w;
            site_add(m, (void *)on_movec, s, a);
        }
    }
    m->cnt.movec_sites = (int32_t)m->nmovec;

    /* longrun.build do_halt: dspboot.find_idle_spins, `bra.b *` (60 FE) */
    if (m->idle_yield > 0) {
        for (off = 0; off + 1 < main_os->len; off += 2) {
            if (main_os->data[off] == 0x60 && main_os->data[off + 1] == 0xFE) {
                uint32_t a = DT_MAIN_LOAD + (uint32_t)off;
                uc_hook_add(m->uc, &hh, UC_HOOK_CODE, (void *)on_idle, m, a, a);
                m->cnt.idle_sites++;
            }
        }
    }
    /* Machine.install_exceptions */
    uc_hook_add(m->uc, &m->intr_hook, UC_HOOK_INTR, (void *)on_intr, m, 1, 0);
    return m;
nomem:
    dt_errf(err, errlen, "out of memory");
    dt_machine_free(m);
    return NULL;
}

void dt_machine_free(dt_machine *m)
{
    if (!m)
        return;
    if (m->uc)
        uc_close(m->uc);
    {
        uint32_t i;
        for (i = 0; i < NPAGES; i++)
            if (m->host[i])
                page_free(m->host[i]);
    }
    free(m->ctl);
    free(m->mmio);
    free(m->movec);
    free(m->isa);
    free(m->sites);
    free(m->page);
    free(m);
}

/* ------------------------------------------------------- stato DTST v1
 * Formato (little endian), scritto da tools/snap2dtst.py:
 *   "DTST" u32 versione(1) u32 record u32 0
 *   record: char[4] tag, u32 flag, u64 lunghezza, dati
 *     REGS 18 x u32: d0-d7 a0-a7 pc sr
 *     EMAC n x (u32 regid, u32 0, u64 valore), nell'ordine di scrittura
 *     CTLR n x (u32 rc, u32 valore)     registri di controllo MOVEC
 *     MMIO n x (u32 indirizzo, u32 valore)
 *     CNTR u64 ff1_count, u64 movec_count
 *     MAPS n x u32 base di pagina da 1 MB
 *     PAGE u32 base, u32 1 MB, flusso zlib della pagina
 *     FWID SHA-256 del MAIN OS (dal manifesto): deve essere quello del firmware
 *     MANI JSON informativo (manifesto, componenti), ignorato qui
 *     COMP stato di un componente (JSON); ignorato in M1
 */
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint64_t le64(const uint8_t *p)
{
    return (uint64_t)le32(p) | (uint64_t)le32(p + 4) << 32;
}

int dtm_mmio_set(dt_machine *m, uint32_t addr, uint32_t v)
{
    size_t i;
    for (i = 0; i < m->nmmio; i++)
        if (m->mmio[i].addr == addr) {
            m->mmio[i].value = v;
            return 0;
        }
    if (m->nmmio == m->capmmio) {
        size_t cap = m->capmmio ? 2 * m->capmmio : 64;
        mmio_site *p = (mmio_site *)realloc(m->mmio, cap * sizeof *p);
        if (!p)
            return -1;
        m->mmio = p;
        m->capmmio = cap;
    }
    m->mmio[m->nmmio].m = m;
    m->mmio[m->nmmio].addr = addr;
    m->mmio[m->nmmio].value = v;
    m->nmmio++;
    return 0;
}

uint32_t *dtm_mmio_ref(dt_machine *m, uint32_t addr)
{
    size_t i;
    for (i = 0; i < m->nmmio; i++)
        if (m->mmio[i].addr == addr)
            return &m->mmio[i].value;
    return NULL;
}

void dtm_install_mmio(dt_machine *m)
{
    size_t i;
    for (i = 0; i < m->nmmio; i++) {
        uc_hook hh;
        uc_hook_add(m->uc, &hh, UC_HOOK_MEM_READ, (void *)on_mmio, &m->mmio[i],
                    m->mmio[i].addr, (uint64_t)m->mmio[i].addr + 3);
    }
    m->cnt.mmio_hooks = (int32_t)m->nmmio;
}

int dt_machine_load_state(dt_machine *m, const char *path, char *err, size_t errlen)
{
    return dtm_load_state(m, path, NULL, NULL, err, errlen);
}

typedef struct {
    const uint8_t *p;
    size_t n, pos;
} st_reader;

static size_t st_read(void *dst, size_t k, st_reader *r)
{
    if (k > r->n - r->pos)
        k = r->n - r->pos;
    memcpy(dst, r->p + r->pos, k);
    r->pos += k;
    return k;
}

/* "DTSZ" u64 lunghezza + flusso zlib di un DTST: -> il DTST (da liberare) */
uint8_t *dtm_inflate_state(const uint8_t *buf, size_t len, size_t *out_len)
{
    uint64_t raw;
    uint8_t *out;
    uLongf n;
    if (len < 12 || memcmp(buf, "DTSZ", 4))
        return NULL;
    raw = le64(buf + 4);
    if (raw > ((uint64_t)1 << 34))
        return NULL;
    out = (uint8_t *)malloc((size_t)raw ? (size_t)raw : 1);
    if (!out)
        return NULL;
    n = (uLongf)raw;
    if (uncompress(out, &n, buf + 12, (uLong)(len - 12)) != Z_OK || n != raw) {
        free(out);
        return NULL;
    }
    *out_len = (size_t)raw;
    return out;
}

int dtm_load_state(dt_machine *m, const char *path, dtm_record_fn cb, void *ctx, char *err, size_t errlen)
{
    FILE *f = fopen(path, "rb");
    uint8_t *buf;
    long n;
    int r;
    if (!f) {
        dt_errf(err, errlen, "cannot open %s", path);
        return DT_ERR_IO;
    }
    if (fseek(f, 0, SEEK_END) || (n = ftell(f)) < 0 || fseek(f, 0, SEEK_SET)) {
        fclose(f);
        dt_errf(err, errlen, "cannot read %s", path);
        return DT_ERR_IO;
    }
    buf = (uint8_t *)malloc(n ? (size_t)n : 1);
    if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        fclose(f);
        dt_errf(err, errlen, "cannot read %s", path);
        return DT_ERR_IO;
    }
    fclose(f);
    r = dtm_load_state_mem(m, buf, (size_t)n, cb, ctx, err, errlen);
    free(buf);
    return r;
}

int dtm_load_state_mem(dt_machine *m, const uint8_t *buf0, size_t len0, dtm_record_fn cb, void *ctx,
                       char *err, size_t errlen)
{
    st_reader rd, *f = &rd;
    uint8_t *inflated = NULL;
    uint8_t hdr[16], rh[16];
    uint8_t *data = NULL;
    uint32_t nrec, r;
    uint32_t regs[18];
    int have_regs = 0, rc = DT_ERR_FORMAT;
    uint8_t *emac = NULL;
    uint64_t emac_len = 0;
    size_t i;

    if (!m || !buf0)
        return DT_ERR_ARG;
    if (m->state_loaded) {
        dt_errf(err, errlen, "state already loaded");
        return DT_ERR_ARG;
    }
    rd.p = buf0;
    rd.n = len0;
    rd.pos = 0;
    if (len0 >= 4 && !memcmp(buf0, "DTSZ", 4)) {
        size_t n = 0;
        inflated = dtm_inflate_state(buf0, len0, &n);
        if (!inflated) {
            dt_errf(err, errlen, "bad compressed state");
            return DT_ERR_FORMAT;
        }
        rd.p = inflated;
        rd.n = n;
    }
    if (st_read(hdr, 16, f) != 16 || memcmp(hdr, "DTST", 4) || le32(hdr + 4) != 1) {
        dt_errf(err, errlen, "not a DTST v1 state");
        goto out;
    }
    nrec = le32(hdr + 8);
    for (r = 0; r < nrec; r++) {
        uint64_t len;
        if (st_read(rh, 16, f) != 16) {
            dt_errf(err, errlen, "truncated at record %u", r);
            goto out;
        }
        len = le64(rh + 8);
        if (len > (uint64_t)1 << 32) {
            dt_errf(err, errlen, "record %u too long", r);
            goto out;
        }
        free(data);
        data = (uint8_t *)malloc(len ? (size_t)len : 1);
        if (!data) {
            rc = DT_ERR_NOMEM;
            goto out;
        }
        if (len && st_read(data, (size_t)len, f) != len) {
            dt_errf(err, errlen, "truncated record %u", r);
            goto out;
        }
        if (!memcmp(rh, "REGS", 4)) {
            if (len != 18 * 4)
                goto bad;
            for (i = 0; i < 18; i++)
                regs[i] = le32(data + 4 * i);
            have_regs = 1;
        } else if (!memcmp(rh, "MAPS", 4)) {
            if (len % 4)
                goto bad;
            for (i = 0; i < len / 4; i++)
                dtm_ensure(m, le32(data + 4 * i));
        } else if (!memcmp(rh, "PAGE", 4)) {
            uLongf out = PAGE;
            uint32_t base;
            if (len < 8 || le32(data + 4) != PAGE)
                goto bad;
            base = le32(data);
            if (base % PAGE || !dtm_is_mapped(m, base)) {
                dt_errf(err, errlen, "page 0x%08x not in MAPS", base);
                goto out;
            }
            if (uncompress(m->page, &out, data + 8, (uLong)(len - 8)) != Z_OK || out != PAGE) {
                dt_errf(err, errlen, "bad page data at 0x%08x", base);
                goto out;
            }
            if (uc_mem_write(m->uc, base, m->page, PAGE) != UC_ERR_OK) {
                dt_errf(err, errlen, "cannot write page 0x%08x", base);
                rc = DT_ERR_UNICORN;
                goto out;
            }
        } else if (!memcmp(rh, "MMIO", 4)) {
            if (len % 8)
                goto bad;
            for (i = 0; i < len / 8; i++)
                if (dtm_mmio_set(m, le32(data + 8 * i), le32(data + 8 * i + 4))) {
                    rc = DT_ERR_NOMEM;
                    goto out;
                }
        } else if (!memcmp(rh, "CTLR", 4)) {
            if (len % 8)
                goto bad;
            for (i = 0; i < len / 8; i++)
                if (ctl_set(m, le32(data + 8 * i), le32(data + 8 * i + 4))) {
                    rc = DT_ERR_NOMEM;
                    goto out;
                }
        } else if (!memcmp(rh, "CNTR", 4)) {
            if (len != 16)
                goto bad;
            m->ff1_count = le64(data);
            m->movec_count = le64(data + 8);
        } else if (!memcmp(rh, "FWID", 4)) {
            if (len != 32)
                goto bad;
            if (memcmp(data, m->main_sha256, 32)) {
                dt_errf(err, errlen, "state made with another firmware (MAIN OS SHA-256 differs)");
                goto out;
            }
        } else if (!memcmp(rh, "EMAC", 4)) {
            if (len % 16)
                goto bad;
            emac = data;              /* si applica dopo i registri, come _restore_emac */
            emac_len = len;
            data = NULL;
        } else if (cb) {
            int r = cb(ctx, (const char *)rh, data, len, err, errlen);
            if (r) {
                rc = r;
                goto out;
            }
        }
        /* MANI e tag sconosciuti senza `cb`: si saltano */
    }
    if (!have_regs) {
        dt_errf(err, errlen, "no REGS record");
        goto out;
    }
    /* snapshot.restore_into: SR prima, poi d0..d7, a0..a7, pc */
    dtm_wr32(m->uc, UC_M68K_REG_SR, regs[17]);
    for (i = 0; i < 8; i++)
        dtm_wr32(m->uc, UC_M68K_REG_D0 + (int)i, regs[i]);
    for (i = 0; i < 8; i++)
        dtm_wr32(m->uc, UC_M68K_REG_A0 + (int)i, regs[8 + i]);
    dtm_wr32(m->uc, UC_M68K_REG_PC, regs[16]);
    for (i = 0; emac && i < emac_len / 16; i++) {
        uint32_t rid = le32(emac + 16 * i);
        uint64_t v = le64(emac + 16 * i + 8);
        if (rid == UC_M68K_REG_MACSR || rid == UC_M68K_REG_MAC_MASK) {
            uint32_t v32 = (uint32_t)v;
            uc_reg_write(m->uc, (int)rid, &v32);
        } else if (rid >= UC_M68K_REG_MACC0 && rid <= UC_M68K_REG_MACC3) {
            uc_reg_write(m->uc, (int)rid, &v);
        } else {
            dt_errf(err, errlen, "bad EMAC register id %u", rid);
            goto out;
        }
    }
    /* Machine.install_mmio (scoped), dopo il ripristino come in longrun.build */
    if (!m->no_mmio_at_load)
        dtm_install_mmio(m);
    m->state_loaded = 1;
    rc = DT_OK;
    goto out;
bad:
    dt_errf(err, errlen, "malformed %.4s record", (const char *)rh);
out:
    free(data);
    free(emac);
    free(inflated);
    return rc;
}

/* ---------------------------------------------------------- esecuzione */
int dt_machine_run_count(dt_machine *m, uint64_t count)
{
    uc_err e;
    uint32_t pc;
    if (!m || !m->state_loaded)
        return DT_ERR_ARG;
    pc = dtm_rd32(m->uc, UC_M68K_REG_PC);
    e = uc_emu_start(m->uc, pc, 0, 0, (size_t)count);
    return e == UC_ERR_OK ? DT_OK : -1000 - (int)e;
}

void dt_machine_regs(dt_machine *m, uint32_t out[18])
{
    int i;
    for (i = 0; i < 8; i++)
        out[i] = dtm_rd32(m->uc, UC_M68K_REG_D0 + i);
    for (i = 0; i < 8; i++)
        out[8 + i] = dtm_rd32(m->uc, UC_M68K_REG_A0 + i);
    out[16] = dtm_rd32(m->uc, UC_M68K_REG_PC);
    out[17] = dtm_rd32(m->uc, UC_M68K_REG_SR);
}

void dt_machine_counters(dt_machine *m, dt_counters *out)
{
    *out = m->cnt;
    out->movec = m->movec_count;      /* come Machine.movec_count */
}

static int cmp_region(const void *a, const void *b)
{
    const uc_mem_region *x = (const uc_mem_region *)a, *y = (const uc_mem_region *)b;
    return x->begin < y->begin ? -1 : x->begin > y->begin;
}

int dt_machine_digest(dt_machine *m, uint8_t out[32], uint32_t *nregions, uint64_t *nbytes)
{
    uc_mem_region *rg = NULL;
    uint32_t n = 0, i;
    uint64_t total = 0;
    dt_sha256 h;
    if (uc_mem_regions(m->uc, &rg, &n) != UC_ERR_OK)
        return DT_ERR_UNICORN;
    qsort(rg, n, sizeof *rg, cmp_region);
    dt_sha256_init(&h);
    for (i = 0; i < n; i++) {
        uint64_t a = rg[i].begin, end = rg[i].end + 1;
        while (a < end) {
            size_t k = end - a < PAGE ? (size_t)(end - a) : PAGE;
            if (uc_mem_read(m->uc, a, m->page, k) != UC_ERR_OK) {
                uc_free(rg);
                return DT_ERR_UNICORN;
            }
            dt_sha256_update(&h, m->page, k);
            a += k;
            total += k;
        }
    }
    uc_free(rg);
    dt_sha256_final(&h, out);
    if (nregions)
        *nregions = n;
    if (nbytes)
        *nbytes = total;
    return DT_OK;
}

int dt_machine_mem_read(dt_machine *m, uint32_t addr, void *buf, size_t n)
{
    return uc_mem_read(m->uc, addr, buf, n) == UC_ERR_OK ? DT_OK : DT_ERR_UNICORN;
}
