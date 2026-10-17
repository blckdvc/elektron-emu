/* engine.c, parte 6: il pannello -- ingresso (emu/panelin.py + Emulator._drain_input
 * con Held e l'attesa fra tasti, i byte grezzi dei pad) e schermo (latch a
 * panel_diff, Emulator._publish_panel). Dal Digitakt II; i LED del Rytm sono
 * in e_rytm.inc.c. */

#define PN_TCD34_DADDR 0xFC045450u
#define PN_RX_VECTOR 154
#define PN_TERMINAL 0x4012D2FAu

/* panel.read(m, ptr_addr): il latch a panel_diff legge fb_front */
static int panel_read_at(dt_engine *e, uint32_t ptr_addr, uint8_t out[1024])
{
    uint8_t b[4];
    uint32_t ptr;
    if (!ptr_addr || e_read(e, ptr_addr, b, 4))
        return 0;
    ptr = e_be32(b);
    if (!(0x40000000u <= ptr && ptr < 0x50000000u))
        return 0;
    return e_read(e, ptr, out, 1024) == 0;
}

static int panel_read(dt_engine *e, uint8_t out[1024])
{
    return panel_read_at(e, e->p->fb_front, out);
}

static void h_panel_latch(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t buf[1024];
    (void)uc; (void)a; (void)s;
    if (panel_read(e, buf)) {
        memcpy(e->latch, buf, sizeof buf);
        e->has_latch = 1;
    }
}

static void h_job_pump(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->jobs++;
}

static void h_terminal(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->terminal = 1;
}

/* Emulator._publish_panel (use_panel: dopo l'intro) */
static void panel_publish(dt_engine *e)
{
    uint8_t buf[1024];
    int x, y, px = 0;
    if (e->has_latch)
        memcpy(buf, e->latch, sizeof buf);
    else if (!panel_read(e, buf))
        return;
    if (e->has_last_panel && !memcmp(buf, e->last_panel, sizeof buf))
        return;
    for (y = 0; y < 64; y++)
        for (x = 0; x < 128; x++)
            px += (buf[x * 8 + (7 - y / 8)] >> (y % 8)) & 1;
    if (!px && !e->panel_live)
        return;
    e->panel_live = 1;
    memcpy(e->last_panel, buf, sizeof buf);
    e->has_last_panel = 1;
    for (y = 0; y < 64; y++)
        for (x = 0; x < 128; x++)
            e->fb[y * 128 + x] = (uint8_t)((buf[x * 8 + (7 - y / 8)] >> (y % 8)) & 1);
    e->frames++;
    e->fb_crc = e_crc(0, e->fb, sizeof e->fb);
    e->panel_version++;
}

/* ------------------------------------------------------------- input */
/* Device.wire_for (devices/analog-rytm-mkii.toml: linear_channels = 9,
   nessuna eccezione) */
static int wire_for(int code, int *channel, int *bit)
{
    if (code >= 1 && code <= 72) {
        *channel = (code - 1) / 8;
        *bit = (code - 1) % 8;
        return 1;
    }
    return 0;
}

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

static void inbox_push_front(dt_engine *e, e_input in)
{
    if (e->inbox_n == e->inbox_cap && inbox_push_back(e, in) == 0) {
        /* crescita: rimettere in testa l'ultimo inserito */
        e->inbox_n--;
    }
    e->inbox_head = (e->inbox_head + e->inbox_cap - 1) % e->inbox_cap;
    e->inbox[e->inbox_head] = in;
    e->inbox_n++;
}

static e_input inbox_pop(dt_engine *e)
{
    e_input in = e->inbox[e->inbox_head];
    e->inbox_head = (e->inbox_head + 1) % e->inbox_cap;
    e->inbox_n--;
    return in;
}

/* panelin.feed: -> nuovo PC */
static uint32_t panel_feed(dt_engine *e, const uint8_t *data, size_t n)
{
    uint32_t base = e_r32(e, e->p->uart8_ring_ptr), daddr = e_r32(e, PN_TCD34_DADDR);
    size_t i;
    for (i = 0; i < n; i++) {
        e_w8(e, daddr, data[i]);
        daddr = base + (((daddr - base) + 1u) & 0x3FFu);
    }
    e_w32(e, PN_TCD34_DADDR, daddr);
    dtm_raise_vector(e->m, PN_RX_VECTOR, 0, -1);
    return e_rd(e, UC_M68K_REG_PC);
}

/* Emulator._drain_input */
static uint32_t panel_drain(dt_engine *e, uint32_t pc)
{
    uint8_t out[512];
    size_t n = 0;
    int took_button = 0, paced = e->dwell_chunks > 0;
    e_input deferred[256];
    int nd = 0, i;
    if (paced && e->delivered_before && e->chunks_since_delivery < e->dwell_chunks) {
        e->chunks_since_delivery++;
        return pc;
    }
    while (e->inbox_n && n + 32 < sizeof out) {
        e_input in = inbox_pop(e);
        if (in.kind == 5) {                                /* 'raw': i byte cosi' come sono (pad) */
            memcpy(out + n, in.raw, (size_t)in.nraw);
            n += (size_t)in.nraw;
        } else if (in.kind == 3) {                         /* encoder */
            if (in.code >= 1 && in.code <= 9 && in.arg >= -128 && in.arg <= 127) {
                out[n++] = (uint8_t)((0x3 << 4) | (in.code - 1));
                out[n++] = (uint8_t)(in.arg & 0xFF);
            }
        } else if (paced && took_button) {
            if (nd < (int)(sizeof deferred / sizeof deferred[0]))
                deferred[nd++] = in;
        } else {
            int ch, bit;
            took_button = 1;
            if (in.kind == 0 || in.kind == 1) {
                if (wire_for(in.code, &ch, &bit)) {
                    if (in.kind == 0)
                        e->held[ch] |= (uint8_t)(1u << bit);
                    else
                        e->held[ch] &= (uint8_t)~(1u << bit);
                    out[n++] = (uint8_t)((0x2 << 4) | ch);
                    out[n++] = e->held[ch];
                }
            } else if (in.kind == 2) {                     /* release_all */
                for (ch = 0; ch < 16; ch++)
                    if (e->held[ch]) {
                        out[n++] = (uint8_t)((0x2 << 4) | ch);
                        out[n++] = 0;
                    }
                memset(e->held, 0, sizeof e->held);
            }
        }
    }
    for (i = nd - 1; i >= 0; i--)
        inbox_push_front(e, deferred[i]);
    if (!n)
        return pc;
    e->chunks_since_delivery = 0;
    e->delivered_before = 1;
    return panel_feed(e, out, n);
}
