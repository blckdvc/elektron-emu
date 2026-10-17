/* dn_engine.c: il bus MMIO del Python (emu/mmiobus.py, DT2_MMIO_BUS=1) in C, e le
 * istruzioni ISA_C che il Digitone esegue con un hook (harness make_rev).
 *
 * Ogni hook di memoria su una pagina di periferica (indirizzo >= 0x8C000000) non va a
 * Unicorn: la pagina da 4 KB diventa una regione uc_mmio_map con uno store (i
 * registri), e gli hook della pagina li chiamano le callback C di native/dn_native.c
 * (dt2_mmio_read / dt2_mmio_write) nell'ordine di registrazione, col filtro sul valore
 * (WriteFilter) dove c'e'. Qui ogni hook e' una funzione C (mmio_page.rcfn/wcfn): nel
 * Python alcuni sono ancora Python (slow_r/slow_w), con lo stesso effetto. Gli accessi
 * dell'host (e_read/e_write) vanno allo store senza hook, come quelli del Python. */

typedef struct e_bpage {
    uint32_t base;
    mmio_page pg;
    uint8_t store[0x1000];
} e_bpage;

#define BUS_MMIO_BASE 0x8C000000u
#define BUS_R 1
#define BUS_W 2

/* MmioBus._page: la pagina del bus che contiene `base` (creata se manca) */
static mmio_page *bus_page_of(dt_engine *e, uint32_t base)
{
    e_bpage *b;
    int i;
    for (i = 0; i < e->nbpages; i++)
        if (e->bpages[i]->base == base)
            return &e->bpages[i]->pg;
    if (e->nbpages >= (int)(sizeof e->bpages / sizeof e->bpages[0]))
        return NULL;
    b = (e_bpage *)calloc(1, sizeof *b);
    if (!b)
        return NULL;
    dtm_ensure(e->m, base);                       /* self.ensure(base) */
    if (uc_mem_read(e->uc, base, b->store, 0x1000) != UC_ERR_OK
        || uc_mem_unmap(e->uc, base, 0x1000) != UC_ERR_OK) {
        free(b);
        return NULL;
    }
    b->base = base;
    b->pg.store = b->store;
    b->pg.base = base;
    if (uc_mmio_map(e->uc, base, 0x1000, (uc_cb_mmio_read_t)dt2_mmio_read, &b->pg,
                    (uc_cb_mmio_write_t)dt2_mmio_write, &b->pg) != UC_ERR_OK) {
        free(b);
        return NULL;
    }
    if (dt2_bus_add(&e->bmap, base, b->store)) {
        free(b);
        return NULL;
    }
    e->bpages[e->nbpages++] = b;
    return &b->pg;
}

/* MmioBus.hook_add per un hook C: -> 0, -1 */
static int bus_hook(dt_engine *e, int kinds, uint64_t begin, uint64_t end, mmio_c_fn fn, void *data,
                    int32_t flt, uint64_t chans)
{
    uint64_t base;
    for (base = begin & ~(uint64_t)0xFFF; base <= (end & ~(uint64_t)0xFFF); base += 0x1000) {
        mmio_page *p = bus_page_of(e, (uint32_t)base);
        if (!p)
            return -1;
        if (kinds & BUS_R) {
            if (p->nr >= MMIO_MAXH)
                return -1;
            p->rb[p->nr] = begin;
            p->re[p->nr] = end;
            p->rcfn[p->nr] = fn;
            p->rcdata[p->nr] = data;
            p->nr++;
        }
        if (kinds & BUS_W) {
            if (p->nw >= MMIO_MAXH)
                return -1;
            p->wb[p->nw] = begin;
            p->we[p->nw] = end;
            p->wcfn[p->nw] = fn;
            p->wcdata[p->nw] = data;
            p->wflt[p->nw] = flt;
            p->wchans[p->nw] = chans;
            p->nw++;
        }
    }
    return 0;
}

/* WriteFilter(fn, chans, size1, not80, all40) -> flt (emu/mmiobus.py) */
static int32_t bus_flt(int size1, int not80, int all40)
{
    int m40 = all40 < 0 ? 0 : (all40 ? 1 : 2);
    return 0x80 | (size1 ? 1 : 0) | (not80 ? 2 : 0) | (m40 << 2);
}

/* un hook di memoria come lo registra il Python: sul bus se cade in una pagina di
   periferica, altrimenti a Unicorn */
static int e_mem_hook(dt_engine *e, int kinds, uint64_t begin, uint64_t end, mmio_c_fn fn, void *data)
{
    if (begin >= BUS_MMIO_BASE && begin <= end)
        return bus_hook(e, kinds, begin, end, fn, data, 0, 0);
    e_hook_mem(e, (kinds & BUS_R ? UC_HOOK_MEM_READ : 0) | (kinds & BUS_W ? UC_HOOK_MEM_WRITE : 0),
               (void *)fn, data, begin, end);
    return 0;
}

/* --------------------------------------------- Machine.mmio (MmioValues) */
static int mmio_find(dt_engine *e, uint32_t addr)
{
    int i;
    for (i = 0; i < e->nmmio; i++)
        if (e->mmio_addr[i] == addr)
            return i;
    return -1;
}

/* m.mmio[addr] = v (un indirizzo nuovo in coda, come il dizionario) */
static int mmio_set(dt_engine *e, uint32_t addr, uint32_t v)
{
    int i = mmio_find(e, addr);
    if (i < 0) {
        if (e->nmmio >= (int)(sizeof e->mmio_addr / sizeof e->mmio_addr[0]))
            return -1;
        i = e->nmmio++;
        e->mmio_addr[i] = addr;
    }
    e->mmio_val[i] = v;
    return 0;
}

static uint32_t *mmio_ref(dt_engine *e, uint32_t addr)
{
    int i = mmio_find(e, addr);
    return i < 0 ? NULL : &e->mmio_val[i];
}

/* ------------------------------------------------- harness make_rev */
typedef struct e_rev {
    dt_engine *e;
    int reg;
    int byterev;
} e_rev;

static uint32_t bitrev32(uint32_t v)
{
    uint32_t r = 0;
    int i;
    for (i = 0; i < 32; i++)
        r |= ((v >> i) & 1u) << (31 - i);
    return r;
}

static void h_rev(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_rev *r = (e_rev *)ud;
    uint32_t v = 0, pc = (uint32_t)a + 2u;
    (void)s;
    uc_reg_read(uc, r->reg, &v);
    if (r->byterev) {
        v = (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) | (v << 24);
        r->e->byterev_count++;
    } else {
        v = bitrev32(v);
        r->e->bitrev_count++;
    }
    uc_reg_write(uc, r->reg, &v);
    uc_reg_write(uc, UC_M68K_REG_PC, &pc);
}

/* install_isa_patches_scoped: BITREV/BYTEREV (Python) e FF1 (C, native_ff1) per sito.
   I MOVEC li installa machine.c. Gli hook di codice di indirizzi diversi non
   interferiscono, quindi l'ordine rispetto ai MOVEC non conta. */
static int isa_hooks(dt_engine *e)
{
    const uint8_t *img = e->main_os->data;
    size_t len = e->main_os->len, off;
    int n = 0, k;
    for (off = 0; off + 1 < len; off += 2) {
        uint16_t w = (uint16_t)(img[off] << 8 | img[off + 1]);
        if ((w >= 0x00C0 && w <= 0x00C7) || (w >= 0x02C0 && w <= 0x02C7))
            n++;
    }
    e->revs = (e_rev *)calloc(n ? (size_t)n : 1, sizeof *e->revs);
    if (!e->revs)
        return -1;
    for (k = 0; k < 8; k++) {
        e->ff1[k].reg_read = (reg_rw_fn)uc_reg_read;
        e->ff1[k].reg_write = (reg_rw_fn)uc_reg_write;
        e->ff1[k].reg = UC_M68K_REG_D0 + k;
        e->ff1[k].r_pc = UC_M68K_REG_PC;
        e->ff1[k].count = &e->ff1_count;
    }
    for (off = 0; off + 1 < len; off += 2) {
        uint16_t w = (uint16_t)(img[off] << 8 | img[off + 1]);
        uint32_t addr = DT_MAIN_LOAD + (uint32_t)off;
        if ((w >= 0x00C0 && w <= 0x00C7) || (w >= 0x02C0 && w <= 0x02C7)) {
            e_rev *r = &e->revs[e->nrevs++];
            r->e = e;
            r->reg = UC_M68K_REG_D0 + (w & 7);
            r->byterev = w >= 0x0200;
            e_hook_code(e, (void *)h_rev, r, addr);
        }
        if (w >= 0x04C0 && w <= 0x04C7)
            e_hook_code(e, (void *)dt2_ff1, &e->ff1[w & 7], addr);
    }
    return 0;
}
