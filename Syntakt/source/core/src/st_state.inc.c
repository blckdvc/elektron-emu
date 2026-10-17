/* st_engine.c, parte 7: lo stato delle due CPU -- lettura del contenitore
 * STST (tools/snap2dtst.py) e scrittura dallo stesso formato (DTST v2 per CPU,
 * come digitakt/emulator/core/src/e_state.inc.c), con i componenti della
 * Syntakt. La scrittura e' quella di Emulator._save_session: il MAIN con
 * checkpoint.save_longrun, il DSP con DspCompanion.save. */

typedef struct {
    dt_engine *e;
    int have_ssi, have_etx, have_tim, have_sd, have_dsp;
    uint8_t *ctim;
    uint64_t ctim_len;
} e_load;

static int main_record(void *ctx, const char tag[4], const uint8_t *d, uint64_t n, char *err, size_t errlen)
{
    e_load *L = (e_load *)ctx;
    dt_engine *e = L->e;
    if (!memcmp(tag, "CUIN", 4)) {
        e->uart_in.n = 0;
        e->uart_in.head = 0;
        if (n && e_fifo_push(&e->uart_in, d, (size_t)n))
            return DT_ERR_NOMEM;
    } else if (!memcmp(tag, "CETX", 4)) {
        if (n != 32 || e_le32(d) != 35 || e_le32(d + 4) != 155) {
            dt_errf(err, errlen, "bad edma_tx component");
            return DT_ERR_FORMAT;
        }
        e->etx.pending = (int32_t)e_le64(d + 8);
        e->etx.bytes = (int64_t)e_le64(d + 16);
        e->etx.transfers = (int64_t)e_le64(d + 24);
        e->etx.restored = 1;
        L->have_etx = 1;
    } else if (!memcmp(tag, "CSSX", 4)) {
        sst_state *s = &e->sst;
        int64_t rh, num, den;
        if (n != 40 + 12 + 4 + 48 + 4 + 16) {
            dt_errf(err, errlen, "bad ssi0_dma component (%llu bytes)", (unsigned long long)n);
            return DT_ERR_FORMAT;
        }
        rh = (int64_t)e_le64(d);
        if (rh != 48000) {
            dt_errf(err, errlen, "Ssi0Dma request-rate mismatch");
            return DT_ERR_FORMAT;
        }
        if (e_le32(d + 40) != 52 || e_le32(d + 44) != 54) {
            dt_errf(err, errlen, "SSI channels %u/%u: not the Syntakt's SSI1 (52/54)", e_le32(d + 40),
                    e_le32(d + 44));
            return DT_ERR_UNSUPPORTED;
        }
        s->rh = rh;
        s->ips = (int64_t)e_le64(d + 8);
        s->now = (int64_t)e_le64(d + 16);
        num = (int64_t)e_le64(d + 24);
        den = (int64_t)e_le64(d + 32);
        if (den) {
            __int128 v = (__int128)num * rh;
            if (v % den) {
                dt_errf(err, errlen, "SSI deadline is not a whole number of requests");
                return DT_ERR_FORMAT;
            }
            s->nextN = (int64_t)(v / den);
            s->has_next = 1;
        } else
            s->has_next = 0;
        s->en = (int32_t)(e_le32(d + 48) & 3);
        s->int50_asserted = d[52];
        s->int50_delivered = d[53];
        s->force_asserted = d[54];
        s->force_delivered = d[55];
        s->requests = (int64_t)e_le64(d + 56);
        /* major_loops e scatter_gathers ripartono da 0: _own_ssi li riprende coi
           canali 48/50 (non ci sono) e _use_rytm li azzera (diagnostica) */
        s->major_rx = s->major_tx = s->sg_rx = s->sg_tx = 0;
        s->tx_bytes = (int64_t)e_le64(d + 96);
        e->tx_crc32_saved = e_le32(d + 104);      /* Ssi0Dma.tx_crc32: il C non lo aggiorna (come il Python) */
        s->vector170 = (int64_t)e_le64(d + 108);
        s->vector191 = (int64_t)e_le64(d + 116);
        L->have_ssi = 1;
    } else if (!memcmp(tag, "CSDH", 4)) {
        e_esdhc *sd = &e->sd;
        uint64_t off = 0, i, cnt;
        if (n < 37) {
            dt_errf(err, errlen, "bad esdhc component");
            return DT_ERR_FORMAT;
        }
        sd->pattern = e_le32(d);
        sd->armed = (int32_t)e_le32(d + 4);
        if (sd->armed != SD_DMA_CHAN)
            sd->armed = -1;
        sd->dma_bytes = (int64_t)e_le64(d + 8);
        if ((int64_t)e_le64(d + 16) != sd->blocks) {
            dt_errf(err, errlen, "Esdhc card capacity mismatch");
            return DT_ERR_FORMAT;
        }
        sd->rca = e_le32(d + 24);
        sd->selected = d[28];
        cnt = e_le32(d + 29);
        off = 33;
        ov_free(&sd->ov);
        for (i = 0; i < cnt; i++) {
            uint64_t sec;
            int hasmask;
            e_sector *s;
            if (off + 9 + 512 > n)
                goto bad_sd;
            sec = e_le64(d + off);
            hasmask = d[off + 8];
            off += 9;
            s = ov_add(&sd->ov, sec);
            if (!s)
                return DT_ERR_NOMEM;
            memcpy(s->data, d + off, 512);
            off += 512;
            if (hasmask) {
                int k;
                if (off + 512 > n)
                    goto bad_sd;
                s->mask = (uint8_t *)malloc(512);
                if (!s->mask)
                    return DT_ERR_NOMEM;
                memcpy(s->mask, d + off, 512);
                s->present = 0;
                for (k = 0; k < 512; k++)
                    s->present += s->mask[k] != 0;
                off += 512;
            }
        }
        L->have_sd = 1;
        return 0;
    bad_sd:
        dt_errf(err, errlen, "truncated esdhc overlay");
        return DT_ERR_FORMAT;
    } else if (!memcmp(tag, "CTIM", 4)) {
        L->ctim = (uint8_t *)malloc((size_t)n);
        if (!L->ctim)
            return DT_ERR_NOMEM;
        memcpy(L->ctim, d, (size_t)n);
        L->ctim_len = n;
        L->have_tim = 1;
    } else if (!memcmp(tag, "CSSI", 4) || !memcmp(tag, "CSMP", 4)) {
        dt_errf(err, errlen, "a Digitakt II state, not a Syntakt one");
        return DT_ERR_FORMAT;
    }
    return 0;
}

static int dsp_record(void *ctx, const char tag[4], const uint8_t *d, uint64_t n, char *err, size_t errlen)
{
    e_load *L = (e_load *)ctx;
    dt_engine *e = L->e;
    if (!memcmp(tag, "CTIM", 4)) {
        int r = timers_restore(DSPC(e), d, n, err, errlen);
        if (r)
            return r;
        L->have_tim = 1;
    } else if (!memcmp(tag, "CDSP", 4)) {
        uint32_t k;
        if (n < 12 || (k = e_le32(d + 8)) != n - 12) {
            dt_errf(err, errlen, "bad DSP component");
            return DT_ERR_FORMAT;
        }
        e->dtin0_edges = (int64_t)e_le64(d);
        e->fpga_config.n = 0;
        if (k && e_buf_put(&e->fpga_config, d + 12, k))
            return DT_ERR_NOMEM;
        e->fpga_config_total = k;
        L->have_dsp = 1;
    }
    return 0;
}

/* il contenitore: "STST" u32 1, u32 2, u32 0, poi per CPU u64 lunghezza e lo stato */
static int stst_split(const uint8_t *buf, size_t len, const uint8_t **part, size_t *plen, char *err,
                      size_t errlen)
{
    size_t off = 16;
    int i;
    if (len < 16 || memcmp(buf, "STST", 4) || e_le32(buf + 4) != 1 || e_le32(buf + 8) != 2) {
        dt_errf(err, errlen, "not a Syntakt state (STST v1 with 2 CPUs): see tools/snap2dtst.py");
        return DT_ERR_FORMAT;
    }
    for (i = 0; i < 2; i++) {
        uint64_t n;
        if (off + 8 > len)
            goto trunc;
        n = e_le64(buf + off);
        off += 8;
        if (n > len - off)
            goto trunc;
        part[i] = buf + off;
        plen[i] = (size_t)n;
        off += (size_t)n;
    }
    return 0;
trunc:
    dt_errf(err, errlen, "truncated STST state");
    return DT_ERR_FORMAT;
}

/* ------------------------------------------------------------ scrittura */
static int w_le32(e_buf *b, uint32_t v)
{
    uint8_t x[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    return e_buf_put(b, x, 4);
}

static int w_le64(e_buf *b, uint64_t v)
{
    return w_le32(b, (uint32_t)v) || w_le32(b, (uint32_t)(v >> 32));
}

static int w_u8(e_buf *b, uint8_t v)
{
    return e_buf_put(b, &v, 1);
}

static int w_f64(e_buf *b, double d)
{
    uint64_t v;
    memcpy(&v, &d, 8);
    return w_le64(b, v);
}

static int w_rec(e_buf *out, uint32_t *nrec, const char *tag, const e_buf *body)
{
    if (e_buf_put(out, tag, 4) || w_le32(out, 0) || w_le64(out, body->n)
        || (body->n && e_buf_put(out, body->p, body->n)))
        return -1;
    (*nrec)++;
    return 0;
}

static int64_t gcd64(int64_t a, int64_t b)
{
    if (a < 0) a = -a;
    if (b < 0) b = -b;
    while (b) {
        int64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : x > y;
}

static int cmp_sec(const void *a, const void *b)
{
    const e_sector *x = *(const e_sector *const *)a, *y = *(const e_sector *const *)b;
    return x->sec < y->sec ? -1 : x->sec > y->sec;
}

/* i record della macchina: REGS, EMAC, CTLR, MMIO, CNTR, MAPS, PAGE, FWID */
/* una sola allocazione per quello che si sa gia' che servira': niente raddoppi con
 * copia di decine di MB mentre il motore e' fermo */
static int e_buf_reserve(e_buf *b, size_t total)
{
    uint8_t *q;
    if (total <= b->cap)
        return 0;
    q = (uint8_t *)realloc(b->p, total);
    if (!q)
        return -1;
    b->p = q;
    b->cap = total;
    return 0;
}

/* il buffer di lavoro della fotografia: ogni motore gira sul suo thread, quindi uno per
 * thread; si tiene fra un salvataggio e l'altro (pagine gia' toccate: niente errori di
 * pagina su 80 MB a ogni salvataggio) */
static _Thread_local e_buf st_save_scratch;

/* raw = 1: le pagine di memoria si copiano grezze in record "PAGR" (la fotografia
 * del salvataggio in due fasi, dt_engine_save_raw); la compressione zlib nei
 * "PAGE" la fa dt_state_finish, fuori dal motore */
static int save_machine_ex(st_cpu *c, uint32_t pc, const uint8_t fwid[32], e_buf *out, uint32_t *nrec, int raw)
{
    e_buf b = {0};
    uint32_t regs[18], *bases = NULL, nb = 0, i;
    uc_mem_region *rg = NULL;
    uint8_t *page = NULL, *z = NULL;
    dt_machine *m = c->m;
    int rc = -1, k;
    c_wr(c, UC_M68K_REG_PC, pc);
    dt_machine_regs(m, regs);
    for (k = 0; k < 18; k++)
        w_le32(&b, regs[k]);
    if (w_rec(out, nrec, "REGS", &b))
        goto fail;
    b.n = 0;
    {
        static const int ids[6] = {UC_M68K_REG_MACSR, UC_M68K_REG_MAC_MASK, UC_M68K_REG_MACC0,
                                   UC_M68K_REG_MACC1, UC_M68K_REG_MACC2, UC_M68K_REG_MACC3};
        for (k = 0; k < 6; k++) {
            uint64_t v = 0;
            if (k < 2) {
                uint32_t v32 = 0;
                uc_reg_read(c->uc, ids[k], &v32);
                v = v32;
            } else
                uc_reg_read(c->uc, ids[k], &v);
            w_le32(&b, (uint32_t)ids[k]);
            w_le32(&b, 0);
            w_le64(&b, v);
        }
    }
    if (w_rec(out, nrec, "EMAC", &b))
        goto fail;
    b.n = 0;
    for (i = 0; i < m->nctl; i++) {
        w_le32(&b, m->ctl[i].key);
        w_le32(&b, m->ctl[i].value);
    }
    if (w_rec(out, nrec, "CTLR", &b))
        goto fail;
    b.n = 0;
    for (i = 0; i < m->nmmio; i++) {
        w_le32(&b, m->mmio[i].addr);
        w_le32(&b, m->mmio[i].value);
    }
    if (w_rec(out, nrec, "MMIO", &b))
        goto fail;
    b.n = 0;
    w_le64(&b, m->ff1_count);
    w_le64(&b, m->movec_count);
    if (w_rec(out, nrec, "CNTR", &b))
        goto fail;
    if (uc_mem_regions(c->uc, &rg, &nb) != UC_ERR_OK)
        goto fail;
    bases = (uint32_t *)malloc((nb ? nb : 1) * sizeof *bases);
    page = (uint8_t *)malloc(0x100000);
    z = (uint8_t *)malloc(compressBound(0x100000));
    if (!bases || !page || !z)
        goto fail;
    for (i = 0; i < nb; i++)
        bases[i] = (uint32_t)rg[i].begin;
    qsort(bases, nb, sizeof *bases, cmp_u32);
    b.n = 0;
    for (i = 0; i < nb; i++)
        w_le32(&b, bases[i]);
    if (w_rec(out, nrec, "MAPS", &b))
        goto fail;
    if (raw && e_buf_reserve(out, out->n + (size_t)nb * (0x100000 + 24) + (1u << 20)))
        goto fail;
    for (i = 0; i < nb; i++) {
        uLongf zn = compressBound(0x100000);
        size_t j;
        int nz = 0;
        if (raw) {                      /* la pagina letta dritta nel record: una sola copia */
            uint8_t *rec = out->p + out->n;
            const uint64_t *w = (const uint64_t *)(const void *)(rec + 24);
            if (uc_mem_read(c->uc, bases[i], rec + 24, 0x100000) != UC_ERR_OK)
                goto fail;
            for (j = 0; j < 0x100000 / 8; j++)
                if (w[j]) {
                    nz = 1;
                    break;
                }
            if (!nz)
                continue;
            memcpy(rec, "PAGR", 4);
            memset(rec + 4, 0, 4);
            for (j = 0; j < 8; j++)
                rec[8 + j] = (uint8_t)((uint64_t)(8 + 0x100000) >> (8 * j));
            for (j = 0; j < 4; j++) {
                rec[16 + j] = (uint8_t)(bases[i] >> (8 * j));
                rec[20 + j] = (uint8_t)(0x100000u >> (8 * j));
            }
            out->n += 24 + 0x100000;
            (*nrec)++;
            continue;
        }
        if (uc_mem_read(c->uc, bases[i], page, 0x100000) != UC_ERR_OK)
            goto fail;
        for (j = 0; j < 0x100000; j++)
            if (page[j]) {
                nz = 1;
                break;
            }
        if (!nz)
            continue;
        b.n = 0;
        w_le32(&b, bases[i]);
        w_le32(&b, 0x100000);
        if (compress2(z, &zn, page, 0x100000, 6) != Z_OK)
            goto fail;
        e_buf_put(&b, z, zn);
        if (w_rec(out, nrec, "PAGE", &b))
            goto fail;
    }
    b.n = 0;
    e_buf_put(&b, fwid, 32);
    if (w_rec(out, nrec, "FWID", &b))
        goto fail;
    rc = 0;
fail:
    if (rg)
        uc_free(rg);
    free(bases);
    free(page);
    free(z);
    free(b.p);
    return rc;
}

/* come prima del salvataggio in due fasi (lo usa anche il boot freddo): pagine gia' compresse */
static int save_machine(st_cpu *c, uint32_t pc, const uint8_t fwid[32], e_buf *out, uint32_t *nrec)
{
    return save_machine_ex(c, pc, fwid, out, nrec, 0);
}

static int save_timers(st_cpu *c, e_buf *out, uint32_t *nrec)
{
    e_buf b = {0};
    int k, rc;
    w_le32(&b, 2);
    for (k = 0; k < 2; k++) {
        e_tsrc *s = &c->tim[k];
        int ch;
        w_le32(&b, (uint32_t)s->kind);
        w_le32(&b, (uint32_t)s->t.nch);
        for (ch = 0; ch < 4; ch++)
            w_le32(&b, ch < s->t.nch ? (uint32_t)s->t.ch[ch] : 0);
        w_le64(&b, (uint64_t)s->ips);
        w_le64(&b, (uint64_t)c->now);
        w_u8(&b, (uint8_t)!!s->t.held);
        for (ch = 0; ch < 4; ch++) {
            w_u8(&b, s->t.has[ch] ? 1 : 0);
            w_f64(&b, s->t.has[ch] ? s->t.next[ch] : 0.0);
        }
        for (ch = 0; ch < 4; ch++)
            w_le64(&b, (uint64_t)s->t.fired[ch]);
        for (ch = 0; ch < 4; ch++)
            w_le64(&b, (uint64_t)s->t.missed[ch]);
        w_le32(&b, s->arm | (k == 1 ? (uint32_t)c->arm_bits : 0));
        w_le32(&b, s->stale);
    }
    rc = w_rec(out, nrec, "CTIM", &b);
    free(b.p);
    return rc;
}

static int finish_dtst(e_buf *out, uint32_t nrec, int compress, e_buf *dst)
{
    memcpy(out->p + 8, &nrec, 4);
    if (compress) {
        uLongf zn = compressBound((uLong)out->n);
        uint8_t *zz = (uint8_t *)malloc(12 + zn);
        uint64_t raw = out->n;
        int k;
        if (!zz || compress2(zz + 12, &zn, out->p, (uLong)out->n, 1) != Z_OK) {
            free(zz);
            return -1;
        }
        memcpy(zz, "DTSZ", 4);
        for (k = 0; k < 8; k++)
            zz[4 + k] = (uint8_t)(raw >> (8 * k));
        if (w_le64(dst, 12 + zn) || e_buf_put(dst, zz, 12 + zn)) {
            free(zz);
            return -1;
        }
        free(zz);
        return 0;
    }
    return e_buf_reserve(dst, dst->n + 8 + out->n) || w_le64(dst, out->n) || e_buf_put(dst, out->p, out->n);
}

static int save_impl(dt_engine *e, int compress, int raw, uint8_t **out_p, size_t *out_n, size_t *cap_io)
{
    e_buf dst = {0}, out = {0}, b = {0};
    uint32_t nrec = 0;
    uint8_t sha[32];
    dt_sha256 h;
    int rc = DT_ERR_NOMEM;
    size_t j;

    if (cap_io) {                       /* il buffer da riusare di rt.c (gia' toccato) */
        dst.p = *out_p;
        dst.cap = *cap_io;
    }
    if (raw) {
        out = st_save_scratch;
        out.n = 0;
        st_save_scratch.p = NULL;
    }
    *out_p = NULL;
    *out_n = 0;
    if (e_buf_put(&dst, "STST", 4) || w_le32(&dst, 1) || w_le32(&dst, 2) || w_le32(&dst, 0))
        goto fail;
    /* ---- MAIN: save_longrun ---- */
    if (e_buf_put(&out, "DTST", 4) || w_le32(&out, 1) || w_le32(&out, 0) || w_le32(&out, 0))
        goto fail;
    dt_sha256_init(&h);
    dt_sha256_update(&h, e->main_os->data, e->main_os->len);
    dt_sha256_final(&h, sha);
    if (save_machine_ex(MAINC(e), e->pc, sha, &out, &nrec, raw))
        goto fail;
    b.n = 0;
    for (j = 0; j < e->uart_in.n; j++)
        w_u8(&b, e->uart_in.p[(e->uart_in.head + j) % e->uart_in.cap]);
    if (w_rec(&out, &nrec, "CUIN", &b))
        goto fail;
    b.n = 0;
    w_le32(&b, 35);
    w_le32(&b, 155);
    w_le64(&b, (uint64_t)(int64_t)e->etx.pending);
    w_le64(&b, (uint64_t)e->etx.bytes);
    w_le64(&b, (uint64_t)e->etx.transfers);
    if (w_rec(&out, &nrec, "CETX", &b))
        goto fail;
    {
        sst_state *s = &e->sst;
        int64_t num = 0, den = 0;
        if (s->has_next) {
            int64_t g = gcd64(s->nextN, s->rh);
            num = s->nextN / g;
            den = s->rh / g;
        }
        b.n = 0;
        w_le64(&b, (uint64_t)s->rh);
        w_le64(&b, (uint64_t)s->ips);
        w_le64(&b, (uint64_t)s->now);
        w_le64(&b, (uint64_t)num);
        w_le64(&b, (uint64_t)den);
        w_le32(&b, (uint32_t)s->rx_ch);
        w_le32(&b, (uint32_t)s->tx_ch);
        w_le32(&b, (uint32_t)s->en);
        w_u8(&b, (uint8_t)!!s->int50_asserted);
        w_u8(&b, (uint8_t)!!s->int50_delivered);
        w_u8(&b, (uint8_t)!!s->force_asserted);
        w_u8(&b, (uint8_t)!!s->force_delivered);
        w_le64(&b, (uint64_t)s->requests);
        w_le64(&b, (uint64_t)s->major_rx);
        w_le64(&b, (uint64_t)s->major_tx);
        w_le64(&b, (uint64_t)s->sg_rx);
        w_le64(&b, (uint64_t)s->sg_tx);
        w_le64(&b, (uint64_t)s->tx_bytes);
        w_le32(&b, e->tx_crc32_saved);                     /* tx_crc32: quello ripristinato */
        w_le64(&b, (uint64_t)s->vector170);
        w_le64(&b, (uint64_t)s->vector191);
        if (w_rec(&out, &nrec, "CSSX", &b))
            goto fail;
    }
    {
        e_esdhc *sd = &e->sd;
        e_sector **v = (e_sector **)malloc((sd->ov.n ? sd->ov.n : 1) * sizeof *v);
        size_t n = 0;
        if (!v)
            goto fail;
        for (j = 0; j < sd->ov.cap; j++)
            if (sd->ov.tab[j].used)
                v[n++] = &sd->ov.tab[j];
        qsort(v, n, sizeof *v, cmp_sec);
        b.n = 0;
        w_le32(&b, sd->pattern);
        w_le32(&b, (uint32_t)sd->armed);
        w_le64(&b, (uint64_t)sd->dma_bytes);
        w_le64(&b, (uint64_t)sd->blocks);
        w_le32(&b, sd->rca);
        w_u8(&b, (uint8_t)!!sd->selected);
        w_le32(&b, (uint32_t)n);
        for (j = 0; j < n; j++) {
            w_le64(&b, v[j]->sec);
            w_u8(&b, v[j]->mask ? 1 : 0);
            e_buf_put(&b, v[j]->data, 512);
            if (v[j]->mask)
                e_buf_put(&b, v[j]->mask, 512);
        }
        free(v);
        if (w_rec(&out, &nrec, "CSDH", &b))
            goto fail;
    }
    if (save_timers(MAINC(e), &out, &nrec))
        goto fail;
    {
        char mani[256];
        int n = snprintf(mani, sizeof mani, "{\"cpu\": \"main\", \"writer\": \"stcore %s\", \"instrs\": %lld}",
                         STCORE_VERSION, (long long)e->instrs);
        b.n = 0;
        e_buf_put(&b, mani, (size_t)n);
        if (w_rec(&out, &nrec, "MANI", &b))
            goto fail;
    }
    if (finish_dtst(&out, nrec, compress, &dst))
        goto fail;
    /* ---- DSP: DspCompanion.save ---- */
    out.n = 0;
    nrec = 0;
    if (e_buf_put(&out, "DTST", 4) || w_le32(&out, 1) || w_le32(&out, 0) || w_le32(&out, 0))
        goto fail;
    dt_sha256_init(&h);
    dt_sha256_update(&h, e->dsp_os->data, e->dsp_os->len);
    dt_sha256_final(&h, sha);
    if (save_machine_ex(DSPC(e), e->dpc, sha, &out, &nrec, raw))
        goto fail;
    if (save_timers(DSPC(e), &out, &nrec))
        goto fail;
    {
        size_t k = e->fpga_config.n > 54666 ? 54666 : e->fpga_config.n;
        b.n = 0;
        w_le64(&b, (uint64_t)e->dtin0_edges);
        w_le32(&b, (uint32_t)k);
        e_buf_put(&b, e->fpga_config.p + (e->fpga_config.n - k), k);
        if (w_rec(&out, &nrec, "CDSP", &b))
            goto fail;
    }
    {
        char mani[256];
        int n = snprintf(mani, sizeof mani, "{\"cpu\": \"dsp\", \"writer\": \"stcore %s\"}", STCORE_VERSION);
        b.n = 0;
        e_buf_put(&b, mani, (size_t)n);
        if (w_rec(&out, &nrec, "MANI", &b))
            goto fail;
    }
    if (finish_dtst(&out, nrec, compress, &dst))
        goto fail;
    *out_p = dst.p;
    *out_n = dst.n;
    if (cap_io)
        *cap_io = dst.cap;
    dst.p = NULL;
    rc = DT_OK;
fail:
    free(dst.p);
    if (raw) {                          /* il buffer di lavoro resta per la prossima fotografia */
        free(st_save_scratch.p);
        st_save_scratch = out;
        out.p = NULL;
    }
    free(out.p);
    free(b.p);
    return rc;
}

int dt_engine_save(dt_engine *e, const char *path, int compress)
{
    uint8_t *buf;
    size_t n;
    FILE *f;
    int r = dt_engine_save_mem(e, compress, &buf, &n);
    char tmp[4096];
    if (r)
        return r;
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    f = fopen(tmp, "wb");
    if (!f || fwrite(buf, 1, n, f) != n) {
        if (f)
            fclose(f);
        free(buf);
        return DT_ERR_IO;
    }
    fclose(f);
    free(buf);
    return rename(tmp, path) ? DT_ERR_IO : DT_OK;
}

int dt_engine_save_mem(dt_engine *e, int compress, uint8_t **out_p, size_t *out_n)
{
    return save_impl(e, compress, 0, out_p, out_n, NULL);
}

/* Il salvataggio in due fasi di rt.c (digitakt/emulator/core/src/e_state.inc.c):
 * dt_engine_save_raw sul thread del motore e' solo una fotografia (lo STST con le
 * pagine di memoria grezze in "PAGR"), dt_state_finish, su qualunque thread, le
 * comprime nei "PAGE" e, con compress, fa il DTSZ di ciascuna CPU. Il file finale
 * e' lo stesso di dt_engine_save_mem(e, 1, ...) (prova: stcore state-test). */
int dt_engine_save_raw(dt_engine *e, uint8_t **out_p, size_t *out_n, size_t *cap_io)
{
    /* out_p e cap_io: il buffer da riusare (gia' toccato), o NULL/0 */
    int r = save_impl(e, 0, 1, out_p, out_n, cap_io);
    if (r != DT_OK) {
        *out_p = NULL;
        *out_n = 0;
        *cap_io = 0;
    }
    return r;
}

/* un DTST con "PAGR" -> lo stesso con "PAGE" (zlib 6, come save_machine) */
static int dtst_finish(const uint8_t *raw, size_t n, int compress, e_buf *dst)
{
    e_buf out = {0}, b = {0};
    uint32_t nrec = 0, k;
    size_t at = 16;
    uint8_t *z = (uint8_t *)malloc(compressBound(0x100000));
    int rc = DT_ERR_NOMEM;
    if (!z || n < 16 || memcmp(raw, "DTST", 4)) {
        rc = z ? DT_ERR_FORMAT : DT_ERR_NOMEM;
        goto fail;
    }
    if (e_buf_put(&out, raw, 16))
        goto fail;
    for (k = e_le32(raw + 8); k > 0; k--) {
        uint64_t len;
        if (at + 16 > n)
            goto bad;
        len = e_le64(raw + at + 8);
        if (len > n - at - 16)
            goto bad;
        if (!memcmp(raw + at, "PAGR", 4)) {
            uLongf zn = compressBound(0x100000);
            if (len != 8 + 0x100000)
                goto bad;
            if (compress2(z, &zn, raw + at + 24, 0x100000, 6) != Z_OK)
                goto fail;
            b.n = 0;
            if (e_buf_put(&b, raw + at + 16, 8) || e_buf_put(&b, z, zn) || w_rec(&out, &nrec, "PAGE", &b))
                goto fail;
        } else {
            if (e_buf_put(&out, raw + at, 16 + (size_t)len))
                goto fail;
            nrec++;
        }
        at += 16 + (size_t)len;
    }
    rc = finish_dtst(&out, nrec, compress, dst) ? DT_ERR_NOMEM : DT_OK;
    goto fail;
bad:
    rc = DT_ERR_FORMAT;
fail:
    free(z);
    free(b.p);
    free(out.p);
    return rc;
}

int dt_state_finish(const uint8_t *raw, size_t n, int compress, uint8_t **out_p, size_t *out_n)
{
    const uint8_t *part[2];
    size_t plen[2], i;
    e_buf dst = {0};
    int rc = DT_ERR_NOMEM;
    *out_p = NULL;
    *out_n = 0;
    if (stst_split(raw, n, part, plen, NULL, 0))
        return DT_ERR_FORMAT;
    if (e_buf_put(&dst, "STST", 4) || w_le32(&dst, 1) || w_le32(&dst, 2) || w_le32(&dst, 0))
        goto fail;
    for (i = 0; i < 2; i++) {
        rc = dtst_finish(part[i], plen[i], compress, &dst);
        if (rc)
            goto fail;
    }
    *out_p = dst.p;
    *out_n = dst.n;
    return DT_OK;
fail:
    free(dst.p);
    return rc;
}
