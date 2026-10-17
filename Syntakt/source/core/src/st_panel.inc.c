/* st_engine.c, parte 6: il pannello -- ingresso (emu/panelin.py +
 * Emulator._drain_input con Held e l'attesa fra tasti), i LED della Syntakt
 * (emu/st_leds.py StLedState, il protocollo del Rytm), lo schermo (latch a
 * panel_diff, Emulator._publish_panel) -- e l'uscita audio dal TX dell'SSI1
 * (emu/staudio.py TxOut). Pannello e schermo da digitakt/emulator/core/src/
 * e_panel.inc.c (lo stesso Python). */

#define PN_TCD34_DADDR 0xFC045450u
#define PN_RX_VECTOR 154
#define PN_DWELL_CHUNKS 4             /* gui.PANEL_DWELL_CHUNKS della Syntakt */

/* ------------------------------------------------------------ LED */
#define SL_REG_BASE 0x455653D9u
#define SL_GROUPS_SENT 0x444BEAF8u
#define SL_N_GROUPS 11

static void stleds_group(e_stleds *l, int g, uint8_t v)
{
    int k;
    for (k = 0; k < 4; k++)
        if (4 * g + k < ST_NLEDS)
            l->state[4 * g + k] = (uint8_t)((v >> (2 * k)) & 3);
}

/* StLedState.load: lo stato allo snapshot dalle tabelle del firmware */
static void stleds_load(dt_engine *e)
{
    e_stleds *l = &e->leds;
    uint8_t raw[4 * 44], g[SL_N_GROUPS];
    int p, r;
    for (r = 0; r < 4; r++)
        for (p = 0; p < ST_NLEDS; p++)
            l->regs[r][p] = -1;
    memset(l->state, 0, sizeof l->state);
    c_read(MAINC(e), SL_REG_BASE, raw, sizeof raw);
    for (p = 0; p < 44; p++)
        for (r = 0; r < 4; r++)
            l->regs[r][p] = raw[4 * p + r] == 0xFF ? -1 : raw[4 * p + r];
    c_read(MAINC(e), SL_GROUPS_SENT, g, sizeof g);
    for (p = 0; p < SL_N_GROUPS; p++)
        stleds_group(l, p, g[p]);
    l->version++;
}

/* StLedState.lit: VV del LED adesso, 0 se spento */
static uint8_t stleds_lit(const e_stleds *l, int led)
{
    int v = l->regs[l->state[led]][led];
    return v > 0 ? (uint8_t)v : 0;
}

/* StLedState.feed */
static int stleds_feed(e_stleds *l, const uint8_t *data, size_t n0)
{
    uint8_t s[4096 + 16];
    size_t n, i;
    int changed = 0;
    while (n0) {
        size_t k = n0 > 4096 ? 4096 : n0;
        memcpy(s, l->buf, l->nbuf);
        memcpy(s + l->nbuf, data, k);
        n = l->nbuf + k;
        data += k;
        n0 -= k;
        i = 0;
        while (i < n) {
            uint8_t h = s[i];
            if (h == 0xB8) {
                i += 1;
            } else if (h >= 0x10 && h <= 0x17) {
                if (i + 10 > n)
                    break;
                i += 10;
            } else if (h >= 0xB0 && h <= 0xB3) {
                if (i + 3 > n)
                    break;
                if (s[i + 1] < ST_NLEDS)
                    l->regs[h - 0xB0][s[i + 1]] = s[i + 2];
                changed++;
                i += 3;
            } else if ((h >= 0x20 && h <= 0x2F) || (h >= 0xA0 && h <= 0xAF)) {
                if (i + 2 > n)
                    break;
                stleds_group(l, (h & 0x0F) + (h >= 0xA0 ? 16 : 0), s[i + 1]);
                changed++;
                i += 2;
            } else {
                l->unknown++;
                i += 1;
            }
        }
        l->nbuf = n - i;
        memcpy(l->buf, s + i, l->nbuf);
    }
    if (changed)
        l->version++;
    return changed;
}

/* ------------------------------------------------------------ schermo */
/* panel.read(m, ptr_addr) */
static int panel_read(dt_engine *e, uint32_t ptr_addr, uint8_t out[1024])
{
    uint8_t b[4];
    uint32_t ptr;
    st_cpu *c = MAINC(e);
    if (!ptr_addr || c_read(c, ptr_addr, b, 4))
        return 0;
    ptr = e_be32(b);
    if (!(0x40000000u <= ptr && ptr < 0x50000000u))
        return 0;
    return c_read(c, ptr, out, 1024) == 0;
}

static void h_panel_latch(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t buf[1024];
    (void)uc; (void)a; (void)s;
    if (panel_read(e, e->p->fb_front, buf)) {
        memcpy(e->latch, buf, sizeof buf);
        e->has_latch = 1;
    }
}

static void h_mainloop(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->mainloop++;
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

/* Emulator._publish_panel (use_panel: la Syntakt riparte dopo l'intro) */
static void panel_publish(dt_engine *e)
{
    uint8_t buf[1024];
    int x, y, px = 0;
    if (e->has_latch)
        memcpy(buf, e->latch, sizeof buf);
    else if (!panel_read(e, e->p->fb_front, buf))
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
/* device.wire_for della Syntakt: codici lineari, 7 canali, nessuna eccezione */
static int wire_for(int code, int *channel, int *bit)
{
    if (code >= 1 && code <= 7 * 8) {
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
    if (e->inbox_n == e->inbox_cap && inbox_push_back(e, in) == 0)
        e->inbox_n--;
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
    st_cpu *c = MAINC(e);
    uint32_t base = c_r32(c, e->p->uart8_ring_ptr), daddr = c_r32(c, PN_TCD34_DADDR);
    size_t i;
    for (i = 0; i < n; i++) {
        c_w8(c, daddr, data[i]);
        daddr = base + (((daddr - base) + 1u) & 0x3FFu);
    }
    c_w32(c, PN_TCD34_DADDR, daddr);
    dtm_raise_vector(c->m, PN_RX_VECTOR, 0, -1);
    return c_rd(c, UC_M68K_REG_PC);
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
        if (in.kind == 3) {                                /* encoder */
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

/* ------------------------------------------------- audio: TxOut */
#define TX_MAIN_L 5
#define TX_MAIN_R 3
#define TX_AFX_L 7
#define TX_AFX_R 2

static void sink_flush(dt_engine *e)
{
    if (e->sink_n && e->sink)
        e->sink(e->sink_ctx, e->sink_mix, &e->sink_tracks[0][0], e->sink_n);
    e->sink_n = 0;
}

/* staudio.TxOut.__call__ su frame interi: (n, 2) float32 */
static void txout_frames(dt_engine *e, const uint8_t *p, size_t nframes)
{
    float scale = (float)(e->volume / (double)(1 << 23));
    size_t f;
    for (f = 0; f < nframes; f++, p += 32) {
        int32_t s[8];
        float l, r, dl, dr, al, ar;
        int k;
        for (k = 0; k < 8; k++)
            s[k] = (int32_t)((uint32_t)e_be32(p + 4 * k) << 8) >> 8;     /* 24 bit con segno */
        dl = (float)s[TX_MAIN_L];
        dr = (float)s[TX_MAIN_R];
        al = (float)s[TX_AFX_L];
        ar = (float)s[TX_AFX_R];
        l = dl + al;                                      /* MAIN + ANALOG_FX * AFX */
        r = dr + ar;
        l = l * scale;
        r = r * scale;
        {
            float lr[2] = {l, r};
            e->audio_crc = e_crc(e->audio_crc, lr, 8);    /* block.tobytes(): float32 LE */
        }
        e->sink_mix[e->sink_n] = l;
        e->sink_tracks[0][e->sink_n] = r;
        e->sink_tracks[1][e->sink_n] = dl * scale;
        e->sink_tracks[2][e->sink_n] = dr * scale;
        e->sink_tracks[3][e->sink_n] = al * scale;
        e->sink_tracks[4][e->sink_n] = ar * scale;
        e->sink_n++;
        e->samples++;
        if (e->sink_n == 512)
            sink_flush(e);
    }
}

/* i byte del TX catturati dal C (Ssi0Dma._flush -> tx_sink) */
static void tx_take(dt_engine *e)
{
    sst_state *s = &e->sst;
    const uint8_t *p = s->txbuf;
    size_t n = (size_t)s->txlen;
    if (!n)
        return;
    s->txlen = 0;
    e->tx_total += n;
    e->tx_crc = e_crc(e->tx_crc, p, n);
    if (e->tx_npend) {
        size_t k = 32 - e->tx_npend;
        if (k > n)
            k = n;
        memcpy(e->tx_pend + e->tx_npend, p, k);
        e->tx_npend += k;
        p += k;
        n -= k;
        if (e->tx_npend < 32)
            return;
        txout_frames(e, e->tx_pend, 1);
        e->tx_npend = 0;
    }
    txout_frames(e, p, n / 32);
    p += n / 32 * 32;
    n %= 32;
    memcpy(e->tx_pend, p, n);
    e->tx_npend = n;
    sink_flush(e);
}
