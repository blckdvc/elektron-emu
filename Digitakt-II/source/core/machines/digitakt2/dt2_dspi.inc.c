/* Digitakt II: i collegamenti col DSP SHARC (non emulato), lato DSP --
 * DSPI2 (emu/dspi2.py: i frame dei parametri, HLE dell'invio) e DSPI1
 * (emu/dspi1.py: la forma d'onda). Indirizzi del firmware 1.16, verificati
 * dai byte all'avvio. */

/* ================================================================ DSPI2 */
#define D2_SR 0xEC03802Cu
#define D2_TCF 0x80000000u
#define D2_EOQF 0x10000000u
#define D2_SEND 0x400CD2BCu
static const uint8_t D2_SEND_VERIFY[] = {0x2f, 0x0a, 0x2f, 0x02, 0x22, 0x2f, 0x00, 0x14, 0x20,
                                         0x2f, 0x00, 0x18, 0x4a, 0xb9, 0x42, 0x94, 0x8b, 0x08};

static void audio_on_frame(dt_engine *e, const uint8_t *frame, size_t n);

static void d2_deliver(dt_engine *e, const uint8_t *frame, size_t n)
{
    e->dspi2_frames++;
    audio_on_frame(e, frame, n);
}

static void h_d2_sr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty;
    if (size == 4 && a == D2_SR)
        *e->dspi2_sr &= ~(uint32_t)v;
}

static void d2_complete(dt_engine *e, uint32_t chan)
{
    uint32_t base = E_TCD_BASE + chan * 0x20u;
    uint16_t csr;
    e_w16(e, base + 0x14, e_r16(e, base + 0x1C));
    csr = e_r16(e, base + 0x1E);
    e_w16(e, base + 0x1E, (uint16_t)((csr | 0x0080) & ~0x0040));
}

static void h_d2_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t src, count, i, n, rx_dst, rx_count;
    uint8_t *raw, *words;
    (void)uc; (void)ty; (void)a;
    if (!(size == 1 && (v & 0x3F) == 28))
        return;
    src = e_r32(e, E_TCD_BASE + 29 * 0x20u);
    count = e_r16(e, E_TCD_BASE + 29 * 0x20u + 0x14) & 0x7FFF;
    if (!count)
        return;
    raw = (uint8_t *)malloc(count * 4u);
    words = (uint8_t *)malloc(count * 2u);
    if (!raw || !words) {
        free(raw);
        free(words);
        return;
    }
    e_read(e, src, raw, count * 4u);
    n = count;
    for (i = 0; i < count; i++)
        if (raw[4 * i] & 0x08) {
            n = i + 1;
            break;
        }
    for (i = 0; i < n; i++) {
        words[2 * i] = raw[4 * i + 2];
        words[2 * i + 1] = raw[4 * i + 3];
    }
    d2_deliver(e, words, n * 2u);
    rx_dst = e_r32(e, E_TCD_BASE + 28 * 0x20u + 0x10);
    rx_count = e_r16(e, E_TCD_BASE + 28 * 0x20u + 0x14) & 0x7FFF;
    if (rx_count) {
        uint8_t *z = (uint8_t *)calloc(rx_count, 2);
        if (z) {
            e_write(e, rx_dst, z, rx_count * 2u);
            free(z);
        }
    }
    d2_complete(e, 29);
    d2_complete(e, 28);
    *e->dspi2_sr |= D2_TCF | D2_EOQF;
    free(raw);
    free(words);
}

/* hle_state.py: la consegna del frame dall'HLE in C */
static int hle_py_c(const uint8_t *words, int64_t n)
{
    dt_engine *e = e_cur;
    if (!(*e->dspi2_sr & D2_EOQF))
        return 0;
    d2_deliver(e, words, (size_t)n);
    *e->dspi2_sr |= D2_TCF | D2_EOQF;
    return 1;
}

/* ================================================================ DSPI1 */
#define D1_VECTOR 86
#define D1_READY_VECTOR 65
#define D1_EPFR 0xFC090006u
#define D1_PIN_CLR 0xEC094025u
#define D1_REPLY_SEM 0x40965B84u
#define D1_REPLY_VERIFY_AT 0x400124E6u
static const uint8_t D1_REPLY_VERIFY[] = {0x20, 0x3c, 0x40, 0x96, 0x5b, 0x84};
#define D1_SEND 0x400CD48Au
static const uint8_t D1_SEND_VERIFY[] = {0x22, 0x3c, 0xfc, 0x03, 0xc0, 0x34, 0x23, 0xc1, 0xfc, 0x04, 0x51, 0xf0};
#define D1_RETRY 8000

static void d1_update_hint(e_dspi1 *d)
{
    d->hint = (d->pending_irq || d->pending_ready) ? 1 : 0;
}

static void h_d1_pin(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
    if (e->d1.awaiting_pulse && !(v & 0x10)) {
        e->d1.awaiting_pulse = 0;
        e->d1.pending_ready = 1;
        d1_update_hint(&e->d1);
    }
}

static int audio_channels(dt_engine *e, uint32_t slot, float **left, float **right, size_t *n);

/* Dspi1._reply: i picchi del sample (np.linspace / max / min / clip) */
static void d1_reply(dt_engine *e, uint32_t cmd, uint32_t slot, uint8_t *out, uint32_t n_words)
{
    float *ch[2] = {NULL, NULL};
    size_t len = 0;
    uint32_t cols, c, k;
    int16_t *tab;
    memset(out, 0, n_words * 2u);
    if ((cmd != 1 && cmd != 2) || n_words % 4)
        return;
    if (!audio_channels(e, slot, &ch[0], &ch[1], &len))
        return;
    cols = n_words / 4;
    tab = (int16_t *)calloc(4 * (size_t)cols, sizeof *tab);
    if (!tab)
        goto out;
    for (c = 0; c < 2; c++) {
        double step;
        if (!len)
            continue;
        step = (double)len / (double)cols;
        for (k = 0; k < cols; k++) {
            int64_t e0 = (int64_t)((double)k * step + 0.0);
            int64_t e1 = k + 1 == cols ? (int64_t)len : (int64_t)((double)(k + 1) * step + 0.0);
            int64_t hi = e1 > e0 + 1 ? e1 : e0 + 1, i;
            float mx, mn;
            if (hi > (int64_t)len)
                hi = (int64_t)len;
            mx = mn = ch[c][e0];
            for (i = e0 + 1; i < hi; i++) {
                if (ch[c][i] > mx) mx = ch[c][i];
                if (ch[c][i] < mn) mn = ch[c][i];
            }
            mx = mx < -1.0f ? -1.0f : (mx > 1.0f ? 1.0f : mx);
            mn = mn < -1.0f ? -1.0f : (mn > 1.0f ? 1.0f : mn);
            tab[c * cols + k] = (int16_t)(int32_t)(mx * 32767.0f);
            tab[(2 + c) * cols + k] = (int16_t)(int32_t)(mn * 32767.0f);
        }
    }
    for (k = 0; k < 4 * cols; k++)
        e_put16(out + 2 * k, (uint16_t)tab[k]);
    free(tab);
out:
    if (ch[1] != ch[0])
        free(ch[1]);
    free(ch[0]);
}

static void h_d1_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    e_dspi1 *d = &e->d1;
    uint32_t n_tx, n_rx, i, chan;
    uint8_t *tx, *reply;
    (void)uc; (void)ty; (void)a;
    if (!(size == 1 && (v & 0x3F) == 14 && !(v & 0x40)))
        return;
    n_tx = e_r16(e, E_TCD_BASE + 15 * 0x20u + 0x14) & 0x7FFF;
    n_rx = e_r16(e, E_TCD_BASE + 14 * 0x20u + 0x14) & 0x7FFF;
    if (!n_tx || !n_rx)
        return;
    tx = (uint8_t *)malloc(n_tx * 4u);
    reply = (uint8_t *)calloc(n_rx, 2);
    if (!tx || !reply) {
        free(tx);
        free(reply);
        return;
    }
    e_read(e, e_r32(e, E_TCD_BASE + 15 * 0x20u), tx, n_tx * 4u);
    if (n_rx == 2 && n_tx == 2) {
        d->has_request = 1;
        d->req_cmd = e_be16(tx + 2);
        d->req_arg = e_be16(tx + 6);
        d->awaiting_pulse = 1;
    } else if (d->has_request) {
        d1_reply(e, d->req_cmd, d->req_arg, reply, n_rx);
        if (d->req_cmd < 4)
            d->replies[d->req_cmd]++;
        d->has_request = 0;
    }
    e_write(e, e_r32(e, E_TCD_BASE + 14 * 0x20u + 0x10), reply, n_rx * 2u);
    for (i = 0; i < 2; i++) {
        uint32_t base;
        uint16_t csr;
        chan = i == 0 ? 15 : 14;
        base = E_TCD_BASE + chan * 0x20u;
        e_w16(e, base + 0x14, e_r16(e, base + 0x1C));
        csr = e_r16(e, base + 0x1E);
        e_w16(e, base + 0x1E, (uint16_t)((csr | 0x0080) & ~0x0040));
    }
    d->transfers++;
    d->pending_irq = 1;
    d1_update_hint(d);
    free(tx);
    free(reply);
}

static int d1_raise(dt_engine *e, int vector)
{
    int lvl = e_irq_level(e, vector, 1);
    if (!lvl)
        return 0;
    if (e_ipl(e) >= lvl)
        return 0;
    return e_raise(e, vector, lvl);
}

static int64_t d1_step(int64_t done)
{
    e_dspi1 *d = &e_cur->d1;
    (void)done;
    return (d->pending_irq || d->pending_ready) ? D1_RETRY : -1;
}

static int d1_service(int64_t done)
{
    dt_engine *e = e_cur;
    e_dspi1 *d = &e->d1;
    (void)done;
    if (d->pending_irq) {
        if (d1_raise(e, D1_VECTOR)) {
            d->pending_irq = 0;
            d1_update_hint(d);
        }
    } else if (d->pending_ready) {
        uint8_t flags = e_r8(e, D1_EPFR);
        e_w8(e, D1_EPFR, flags | 0x02);
        if (d1_raise(e, D1_READY_VECTOR)) {
            d->pending_ready = 0;
            d1_update_hint(d);
            d->ready_irqs++;
        }
    }
    d1_update_hint(d);
    return 0;
}

