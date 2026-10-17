/* engine.c, parte 5: cio' che e' solo del Rytm, riga per riga sul Python:
 *   la flash SPI scrivibile (emu/rytm_nor.py: la calibrazione "RBOW");
 *   l'RPC del +Drive in SysEx (emu/rytm_rpc.py SysexRpcLink);
 *   i LED (emu/rytm_leds.py RytmLedState);
 *   il motore delle voci: il frame verso l'FPGA a ogni tick di scambio
 *   (emu/rytmsynth.py install/on_frame, percorso offline: un blocco reso nel
 *   thread dell'emulatore) e lo strato dei sample (SampleTap), intorno a
 *   native/rytm_dsp.c (emu/rytmdsp.py), compilato a parte con i flag del
 *   Python (-O2 -ffast-math). */

/* native/rytm_dsp.c (la sua unita' di traduzione) */
void rytm_init(void *d);
void rytm_trigger(void *d, int vi, const float *p, float vel);
void rytm_set_fx(void *d, const float *fx);
void rytm_set_input(void *d, int vi, const float *x);
void rytm_set_strike(void *d, int vi, const float *x);
void rytm_frame(void *d, const uint16_t *w, int n);
void rytm_render(void *d, float *out);
int rytm_size(void);
const float *rytm_voices(void *d);

/* ================================================================ NOR */
#define NOR_READ 0x40136FAAu
#define NOR_WRITE 0x40136FAEu
#define NOR_ERASE 0x4013702Au
static const uint8_t NOR_READ_VERIFY[] = {0x60, 0x00, 0xf3, 0x0a};
static const uint8_t NOR_WRITE_VERIFY[] = {0x4f, 0xef, 0xff, 0xec, 0x48, 0xd7, 0x04, 0x3c};
static const uint8_t NOR_ERASE_VERIFY[] = {0x4f, 0xef, 0xff, 0xf4, 0x48, 0xd7, 0x04, 0x0c};

/* rytm_nor.install: le firme nell'immagine principale */
static int nor_verify(dt_engine *e)
{
    const uint8_t *img = e->main_os->data;
    size_t n = e->main_os->len;
#define NV(a, v) ((a) - DT_MAIN_LOAD + sizeof v <= n && !memcmp(img + (a) - DT_MAIN_LOAD, v, sizeof v))
    return NV(NOR_READ, NOR_READ_VERIFY) && NV(NOR_WRITE, NOR_WRITE_VERIFY) && NV(NOR_ERASE, NOR_ERASE_VERIFY);
#undef NV
}

/* Nor._block: un blocco dall'immagine di base (completato con zeri) */
static uint8_t *nor_block(dt_engine *e, uint32_t b)
{
    uint32_t i = b / NOR_BLOCK;
    if (i >= NOR_MAXBLK)
        return NULL;
    if (!e->nor.blk[i]) {
        uint8_t *p = (uint8_t *)calloc(1, NOR_BLOCK);
        if (!p)
            return NULL;
        if (b < e->flash_len)
            memcpy(p, e->flash + b, e->flash_len - b < NOR_BLOCK ? e->flash_len - b : NOR_BLOCK);
        e->nor.blk[i] = p;
    }
    return e->nor.blk[i];
}

/* Nor.read(off, n) -> in memoria emulata a `dest` (nor.read + mem_write) */
static void nor_read(dt_engine *e, uint32_t off, uint32_t n, uint32_t dest)
{
    uint8_t *buf = (uint8_t *)malloc(n ? n : 1);
    uint32_t o = off, k, left = n;
    uint8_t *p = buf;
    if (!buf)
        return;
    while (left) {
        uint32_t b = o - o % NOR_BLOCK, i = b / NOR_BLOCK;
        k = b + NOR_BLOCK - o < left ? b + NOR_BLOCK - o : left;
        if (i < NOR_MAXBLK && e->nor.blk[i])
            memcpy(p, e->nor.blk[i] + (o - b), k);
        else {
            uint32_t j;
            for (j = 0; j < k; j++)
                p[j] = (uint64_t)o + j < e->flash_len ? e->flash[o + j] : 0;
        }
        o += k;
        p += k;
        left -= k;
    }
    e_write(e, dest, buf, n);
    free(buf);
}

/* rytm_nor args(uc, n): sp, il ritorno e n argomenti */
static uint32_t nor_args(dt_engine *e, uint32_t *ret, uint32_t *a, int n)
{
    uint8_t b[16];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7);
    int i;
    e_read(e, sp, b, 4u * (uint32_t)(n + 1));
    *ret = e_be32(b);
    for (i = 0; i < n; i++)
        a[i] = e_be32(b + 4 + 4 * i);
    return sp;
}

static void nor_ret(dt_engine *e, uint32_t sp, uint32_t r)
{
    e_wr(e, UC_M68K_REG_D0, 0);
    e_wr(e, UC_M68K_REG_A7, sp + 4);
    e_wr(e, UC_M68K_REG_PC, r);
}

static void h_nor_read(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t r, v[3], sp = nor_args(e, &r, v, 3);
    (void)uc; (void)a; (void)s;
    if (v[1] && v[2]) {
        uint64_t p;
        for (p = 0; p < (uint64_t)v[1] + 0x100000u; p += 0x100000u)
            dtm_ensure(e->m, (uint64_t)v[2] + p);
        nor_read(e, v[0], v[1], v[2]);
    }
    e->nor.reads++;
    nor_ret(e, sp, r);
}

static void h_nor_write(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t r, v[3], sp = nor_args(e, &r, v, 3), off = v[0], n = v[1];
    uint8_t *data = (uint8_t *)malloc(n ? n : 1), *q = data;
    (void)uc; (void)a; (void)s;
    if (data && n)
        e_read(e, v[2], data, n);
    while (data && n) {                         /* NOR: la programmazione spegne i bit */
        uint32_t b = off - off % NOR_BLOCK, k = b + NOR_BLOCK - off < n ? b + NOR_BLOCK - off : n, i;
        uint8_t *blk = nor_block(e, b);
        if (blk)
            for (i = 0; i < k; i++)
                blk[off - b + i] &= q[i];
        off += k;
        q += k;
        n -= k;
    }
    free(data);
    e->nor.writes++;
    nor_ret(e, sp, r);
}

static void h_nor_erase(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t r, v[1], sp = nor_args(e, &r, v, 1), b = v[0] - v[0] % NOR_BLOCK;
    uint8_t *blk;
    (void)uc; (void)a; (void)s;
    blk = nor_block(e, b);
    if (blk)
        memset(blk, 0xFF, NOR_BLOCK);
    e->nor.erases++;
    nor_ret(e, sp, r);
}

/* ================================================================ RPC */
#define RPC_ENTRY 0x400830F6u
#define RPC_FLUSH 0x400859ACu
static const uint8_t RPC_ENTRY_VERIFY[] = {0x2f, 0x02, 0x74, 0x02, 0x22, 0x2f, 0x00, 0x08,
                                           0x20, 0x6f, 0x00, 0x0c, 0x20, 0x2f, 0x00, 0x10};
static const uint8_t RPC_FLUSH_VERIFY[] = {0x42, 0x00, 0x4f, 0xef, 0xff, 0xe8, 0x48, 0xd7,
                                           0x0c, 0x1c, 0x28, 0x2f, 0x00, 0x1c, 0x24, 0x2f};
static const uint8_t RPC_HEADER[] = {0xF0, 0x00, 0x20, 0x3C, 0x10, 0x00};
#define RPC_USB_PORT 2

static void h_rpc_entry(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    e->rpc.entries++;
    if (e->rpc.inflight)
        e_w32(e, e_rd(e, UC_M68K_REG_A7) + 12, RPC_USB_PORT);
}

/* rytm_rpc.decode7 */
static size_t rpc_decode7(const uint8_t *d, size_t n, uint8_t *out)
{
    size_t i, k = 0;
    for (i = 0; i < n; i += 8) {
        uint8_t top = d[i];
        size_t j;
        for (j = 0; j < 7 && i + 1 + j < n; j++)
            out[k++] = (uint8_t)(d[i + 1 + j] | ((top & (1u << (6 - j))) ? 0x80 : 0));
    }
    return k;
}

/* SysexRpcLink._on_flush: le risposte della porta USB, messaggi F0..F7 */
static void h_rpc_flush(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    e_rpc *r = &e->rpc;
    uint8_t b[16];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), buf, n, port;
    (void)uc; (void)a; (void)s;
    e_read(e, sp + 4, b, 16);
    buf = e_be32(b); n = e_be32(b + 4); port = e_be32(b + 12);
    if (port != RPC_USB_PORT || !(0 < n && n <= 0x10000))
        return;
    r->flushes++;
    {
        uint8_t *p = (uint8_t *)malloc(n);
        if (!p)
            return;
        e_read(e, buf, p, n);
        e_buf_put(&r->rx, p, n);
        free(p);
    }
    for (;;) {
        size_t i, start = (size_t)-1, end = (size_t)-1;
        for (i = 0; i < r->rx.n; i++)
            if (r->rx.p[i] == 0xF0) { start = i; break; }
        if (start != (size_t)-1)
            for (i = start + 1; i < r->rx.n; i++)
                if (r->rx.p[i] == 0xF7) { end = i; break; }
        if (end == (size_t)-1) {
            if (start != (size_t)-1 && start > 0)
                e_buf_drop(&r->rx, start);
            return;
        }
        {
            size_t mlen = end + 1 - start, plen;
            uint8_t *msg = (uint8_t *)malloc(mlen), *pl;
            if (!msg)
                return;
            memcpy(msg, r->rx.p + start, mlen);
            e_buf_drop(&r->rx, end + 1);
            if (mlen < sizeof RPC_HEADER + 1 || memcmp(msg, RPC_HEADER, sizeof RPC_HEADER)) {
                free(msg);
                continue;
            }
            pl = (uint8_t *)malloc(mlen);
            plen = pl ? rpc_decode7(msg + sizeof RPC_HEADER, mlen - sizeof RPC_HEADER - 1, pl) : 0;
            if (pl && plen >= 5) {
                uint8_t hdr[7];
                uint32_t blen = (uint32_t)(plen - 5);
                e_put16(hdr, e_be16(pl + 2));            /* seq */
                hdr[2] = pl[4];                          /* id | 0x80 */
                e_put32(hdr + 3, blen);
                e_buf_put(&r->replies, hdr, 7);
                if (blen)
                    e_buf_put(&r->replies, pl + 5, blen);
                r->nreplies++;
                if (r->inflight)
                    r->inflight--;
            }
            free(pl);
            free(msg);
        }
    }
}

/* rytm_rpc.encode7 */
static size_t rpc_encode7(const uint8_t *d, size_t n, uint8_t *out)
{
    size_t i, k = 0;
    for (i = 0; i < n; i += 7) {
        size_t j, m = n - i < 7 ? n - i : 7, top = k++;
        out[top] = 0;
        for (j = 0; j < m; j++) {
            if (d[i + j] & 0x80)
                out[top] |= (uint8_t)(1u << (6 - j));
            out[k++] = d[i + j] & 0x7F;
        }
    }
    return k;
}

/* ================================================================ LED */
#define RL_GROUPS 0x41920534u
#define RL_GROUPS_CHECK 0x41920549u

static void rl_group(e_rleds *l, int g, uint8_t v)
{
    int k;
    for (k = 0; k < 4; k++)
        if (4 * g + k < 128)
            l->state[4 * g + k] = (uint8_t)((v >> (2 * k)) & 3);
}

static void rl_init(e_rleds *l)
{
    memset(l, 0, sizeof *l);
    memset(l->regs, 0xFF, sizeof l->regs);       /* -1: None */
}

/* RytmLedState.load: lo stato 2x dalla tabella del firmware */
static int rl_load(dt_engine *e)
{
    uint8_t a[21], b[21];
    int g;
    if (e_read(e, RL_GROUPS, a, 21) || e_read(e, RL_GROUPS_CHECK, b, 21) || memcmp(a, b, 21))
        return 0;
    for (g = 0; g < 21; g++)
        rl_group(&e->leds, g, a[g]);
    e->leds.version++;
    return 1;
}

/* RytmLedState.lit */
static uint8_t rl_lit(const e_rleds *l, int led)
{
    int r = l->state[led];
    int16_t v = l->regs[r][led];
    if (v < 0) {
        static const uint8_t REG_DEFAULT[4] = {0x04, 0x0F, 0x00, 0x0F};
        if (led == 22) { static const uint8_t d[4] = {0x1F, 0x1F, 0x00, 0x1F}; return d[r]; }
        if (led == 23) return 0;
        if (led == 21) { static const uint8_t d[4] = {0x0F, 0x0F, 0x00, 0x0F}; return d[r]; }
        return REG_DEFAULT[r];
    }
    return (uint8_t)v;
}

/* RytmLedState.feed */
static int rl_feed(e_rleds *l, const uint8_t *data, size_t n0)
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
            if (h == 0xB8)
                i += 1;
            else if (h >= 0x10 && h <= 0x17) {
                if (i + 10 > n)
                    break;
                i += 10;
            } else if (h >= 0xB0 && h <= 0xB3) {
                if (i + 3 > n)
                    break;
                if (s[i + 1] < 128)
                    l->regs[h - 0xB0][s[i + 1]] = s[i + 2];
                changed++;
                i += 3;
            } else if (h == 0xB5) {
                int pad;
                if (i + 5 > n)
                    break;
                pad = s[i + 1] - 0x74;
                if (pad >= 0 && pad < 12)
                    memcpy(l->pads[pad], s + i + 2, 3);
                changed++;
                i += 5;
            } else if (h >= 0x20 && h <= 0x3F) {
                if (i + 2 > n)
                    break;
                rl_group(l, h - 0x20, s[i + 1]);
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

/* crc32 dello stato (tools/arcore_ref.py leds_crc): registri (0xFF = None), 2x, pad */
static uint32_t rl_crc(const e_rleds *l)
{
    uint8_t b[4 * 128 + 128 + 36];
    int r, i;
    for (r = 0; r < 4; r++)
        for (i = 0; i < 128; i++)
            b[r * 128 + i] = l->regs[r][i] < 0 ? 0xFF : (uint8_t)l->regs[r][i];
    memcpy(b + 512, l->state, 128);
    for (i = 0; i < 12; i++)
        memcpy(b + 640 + 3 * i, l->pads[i], 3);
    return e_crc(0, b, sizeof b);
}

/* ============================================================ il motore */
#define SYN_VOICE_TRACK 0x40290DD8u
#define SYN_KIT_PTR 0x41B6E830u
#define SYN_FRAME_TCD_SADDR 0xFC0451E0u
#define SYN_TRACK_STRIDE 0xA2u
#define SYN_WARMUP 4
#define SYN_FRAME_WORDS 332
#define SYN_FX_EVERY 150
#define SYN_TEMPO_ADDR 0x8000E432u
#define SYN_FX_DELAY 0x7C6u
#define SYN_TX_FRAME0 0x80000920u

static const float SYN_FX_DEFAULT[32] = {23, 0, 127, 49, 0, 127, 0, 64, 0, 0,
                                         8, 72, 64, 64, 0, 127, 64, 0,
                                         0, 64, 0, 127, 0, 64, 0, 0, 0, 0, 127, 127.0f, 0, 0};

static int syn_init(dt_engine *e)
{
    e_synth *y = &e->syn;
    y->dsp = calloc(1, (size_t)rytm_size());
    y->chunk_end = (int64_t *)malloc(AR_CHUNKCAP * sizeof(int64_t));
    y->chunk_src = (int64_t *)malloc(AR_CHUNKCAP * sizeof(int64_t));
    if (!y->dsp || !y->chunk_end || !y->chunk_src)
        return -1;
    rytm_init(y->dsp);                                 /* Dsp() */
    memcpy(y->fx, SYN_FX_DEFAULT, sizeof y->fx);
    rytm_set_fx(y->dsp, y->fx);
    y->master = 1.0f;
    y->tap_on = 1;
    return 0;
}

/* RytmSynth.on_frame(w): w = parole 0..331 del frame (16 bit) */
static void syn_on_frame(dt_engine *e, const uint16_t *w)
{
    e_synth *y = &e->syn;
    const uint16_t *vw = w + 84, *prev;
    uint8_t vt[8];
    int v, have_vt = 0;
    rytm_frame(y->dsp, w, SYN_FRAME_WORDS);
    if (y->frames % SYN_FX_EVERY == 0) {               /* set_fx(fx_params()) */
        uint32_t name = e_r32(e, SYN_KIT_PTR) + 4;
        uint8_t raw[58];
        double tempo;
        int i;
        e_read(e, name + SYN_FX_DELAY, raw, sizeof raw);
        for (i = 0; i < 32; i++)
            y->fx[i] = SYN_FX_DEFAULT[i];
        for (i = 0; i < 29; i++)
            y->fx[i] = (float)((double)raw[2 * i] + (double)raw[2 * i + 1] / 256.0);
        tempo = (double)e_r16(e, SYN_TEMPO_ADDR) / 120.0;
        y->fx[29] = (float)(30.0 <= tempo && tempo <= 300.0 ? tempo : 120.0);
        rytm_set_fx(y->dsp, y->fx);
    }
    prev = y->has_prev ? y->prev : vw;
    for (v = 0; v < 8; v++) {
        if (y->frames >= SYN_WARMUP && (vw[4 * v] & 0x4000) && !(prev[4 * v] & 0x4000)) {
            uint32_t name, track, vel;
            uint8_t raw[0xA2 + 0x10];
            int i, machine, model;
            static const uint8_t FLT_OFF[8] = {0x66, 0x6A, 0x68, 0x6C, 0x6E, 0x70, 0x72, 0x74};
            static const uint8_t AMP_OFF[8] = {0x76, 0x78, 0x7A, 0x7C, 0x7E, 0x80, 0x82, 0x84};
            if (!have_vt) {
                e_read(e, SYN_VOICE_TRACK, vt, 8);
                have_vt = 1;
            }
            track = vt[v];
            if (track >= 12)
                continue;
            vel = vw[4 * v + 1] & 0x7F;
            if (!vel)
                vel = 100;
            name = e_r32(e, SYN_KIT_PTR) + 4;
            e_read(e, name + track * SYN_TRACK_STRIDE, raw, sizeof raw);
#define U(off) ((double)raw[off] + (double)raw[(off) + 1] / 256.0)
            for (i = 0; i < 8; i++)
                y->pbuf[i] = (float)U(0x46 + 2 * i);
            for (i = 0; i < 8; i++)
                y->pbuf[8 + i] = (float)U(FLT_OFF[i]);
            for (i = 0; i < 8; i++)
                y->pbuf[16 + i] = (float)U(AMP_OFF[i]);
#undef U
            machine = raw[0xA6];
            switch (machine) {                         /* rytmdsp.MODEL_OF */
            case 35: model = 0; break;
            case 36: model = 1; break;
            case 37: model = 2; break;
            case 38: model = 3; break;
            case 39: model = 4; break;
            case 40: model = 5; break;
            case 41: model = 13; break;
            case 42: model = 14; break;
            case 34: model = 7; break;
            default: model = machine;
            }
            y->pbuf[24] = (float)model;
            y->pbuf[25] = (float)track;
            rytm_trigger(y->dsp, v, y->pbuf, (float)vel);
            y->triggers++;
        }
    }
    memcpy(y->prev, vw, sizeof y->prev);
    y->has_prev = 1;
    {                                                  /* offline: render() * MASTER -> sink */
        int i;
        rytm_render(y->dsp, y->out);
        for (i = 0; i < 2 * AR_BLOCK; i++)
            y->out[i] = y->out[i] * y->master;
        y->samples += AR_BLOCK;
        y->crc = e_crc(y->crc, y->out, sizeof y->out);    /* float32 LE (arm64/x86) */
        if (y->sink) {                                   /* le voci separate: col volume, fuori dal mix */
            const float *v = rytm_voices(y->dsp);
            for (i = 0; i < 8 * 2 * AR_BLOCK; i++)
                y->voices[i] = v[i] * y->master;
            y->sink(y->sink_ctx, y->out, y->voices, AR_BLOCK);
        }
    }
    y->frames++;
}

/* ------------------------------------------------------------- SampleTap
   Il TX dell'SSI1 arriva a pezzi: un pezzo per service che ha catturato
   byte, col suo tx_src (native/dt2_hooks.c dt2_sst_service). Il Python li
   consegna al tx_sink (SampleTap.__call__, che li tiene) in
   _OwnedSsi0Dma._flush: al SERQ (prima degli altri ascoltatori), a buffer
   pieno (SPIN_FLUSH) e a fine chiamata; il CRC dell'SSI (tx_crc32) li conta
   li'. SampleTap.flush() li consuma al frame. */
static int tap_put(e_synth *y, const uint8_t *data, size_t n, int64_t src)
{
    if (y->tap_n == y->tap_cap) {
        size_t cap = y->tap_cap ? 2 * y->tap_cap : 256;
        int64_t *a = (int64_t *)realloc(y->tap_end, cap * sizeof *a);
        int64_t *b;
        if (!a)
            return -1;
        y->tap_end = a;
        b = (int64_t *)realloc(y->tap_src, cap * sizeof *b);
        if (!b)
            return -1;
        y->tap_src = b;
        y->tap_cap = cap;
    }
    if (e_buf_put(&y->tap_data, data, n))
        return -1;
    y->tap_end[y->tap_n] = (int64_t)y->tap_data.n;
    y->tap_src[y->tap_n] = src;
    y->tap_n++;
    return 0;
}

/* _OwnedSsi0Dma._flush */
static void ssi_flush(dt_engine *e)
{
    sst_state *s = &e->sst;
    int64_t i, start = 0;
    if (s->nchunk) {
        for (i = 0; i < s->nchunk; i++) {
            int64_t end = s->chunk_end[i];
            e->ssi_tx_crc = e_crc(e->ssi_tx_crc, e->txbuf + start, (size_t)(end - start));
            if (e->syn.tap_on)
                tap_put(&e->syn, e->txbuf + start, (size_t)(end - start), s->chunk_src[i]);
            start = end;
        }
        s->nchunk = 0;
        s->txlen = 0;
    } else if (s->txlen)
        s->txlen = 0;
}

/* SampleTap.flush (la via veloce; i pezzi di lunghezza non multipla di 4
   andrebbero a _flush_ref: qui non capitano, i minor loop sono a parole) */
static void tap_flush(dt_engine *e)
{
    e_synth *y = &e->syn;
    size_t nw, n, i, start = 0;
    const uint8_t *raw = y->tap_data.p;
    int k;
    if (!y->tap_n)
        return;
    for (i = 0; i < y->tap_n; i++) {
        if ((y->tap_end[i] - (int64_t)start) & 3)
            y->tap_odd++;
        start = (size_t)y->tap_end[i];
    }
    nw = y->tap_data.n / 4;
    n = nw / 16;
    start = 0;
    if (!n || n * 16 != nw)
        goto drop;                                   /* misaligned (or nothing): drop this tick */
    for (i = 0; i < y->tap_n; i++) {                 /* the start slot, from the first chunk >= 4 */
        if (y->tap_end[i] - (int64_t)start >= 4) {
            if (((((uint32_t)y->tap_src[i] - SYN_TX_FRAME0) & 0xFFFu) >> 2) & 15u)
                goto drop;
            break;
        }
        start = (size_t)y->tap_end[i];
    }
    for (i = 0; i < n; i++) {
        int j;
        if (y->nacc == AR_BLOCK) {                   /* (non capita: si svuota sotto) */
            y->tap_odd++;
            break;
        }
        for (j = 0; j < 16; j++) {
            const uint8_t *q = raw + 4 * (16 * i + (size_t)j);
            int32_t v = (int32_t)(e_be32(q) << 8) >> 8;
            y->acc[y->nacc][j] = (float)v * (1.0f / 8388608.0f);
        }
        y->nacc++;
        if (y->nacc == AR_BLOCK) {                   /* un blocco: le voci e i tom */
            int lit[16] = {0};
            float col[AR_BLOCK];
            int r;
            for (r = 0; r < AR_BLOCK; r++)
                for (j = 0; j < 16; j++)
                    if (y->acc[r][j] != 0.0f)
                        lit[j] = 1;
            for (k = 0; k < 8; k++)                  /* slot 8-15: voci 0-7, * SAMPLE_GAIN */
                if (lit[8 + k]) {
                    for (r = 0; r < AR_BLOCK; r++)
                        col[r] = y->acc[r][8 + k] * 2.0f;
                    rytm_set_input(y->dsp, k, col);
                }
            for (k = 0; k < 3; k++)                  /* slot 0-2: i tom BT, LT, MT */
                if (lit[k]) {
                    for (r = 0; r < AR_BLOCK; r++)
                        col[r] = y->acc[r][k];
                    rytm_set_strike(y->dsp, 1 + 2 * k, col);
                }
            y->tap_blocks++;
            y->nacc = 0;
        }
    }
drop:
    y->tap_n = 0;
    y->tap_data.n = 0;
}

static void cal_on_frame(dt_engine *e, const uint16_t *w);      /* e_calib.inc.c */

/* rytmsynth.install on_serq: il frame (SERQ del canale 15), poi tap.flush() */
static void syn_on_serq(dt_engine *e, int size, int64_t v)
{
    uint8_t b[4 * SYN_FRAME_WORDS];
    uint16_t w[SYN_FRAME_WORDS];
    uint32_t src;
    int i;
    if (size != 1 || v != 15)
        return;
    src = e_r32(e, SYN_FRAME_TCD_SADDR);
    e_read(e, src + 4, b, sizeof b);
    for (i = 0; i < SYN_FRAME_WORDS; i++)
        w[i] = e_be16(b + 4 * i + 2);                /* '>u4' & 0xFFFF */
    if (e->cal) {                                    /* calibrazione: il frame va ai circuiti, non alle voci */
        cal_on_frame(e, w);
        return;
    }
    syn_on_frame(e, w);
    tap_flush(e);
}
