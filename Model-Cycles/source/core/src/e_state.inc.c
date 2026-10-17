/* engine.c, parte 7: salvare lo stato del motore (DTST v2, lo stesso formato di
 * tools/snap2dtst.py) -- come Emulator._save_session -> checkpoint.save_longrun
 * del Python: pagine, registri, EMAC, MOVEC, MMIO, contatori e i componenti
 * (uart_in, edma_tx, ssi0_dma, esdhc + l'intervallo di cancellazione, timers).
 * Da chiamare fra un giro e l'altro, sul thread del motore. Dal Digitakt II. */

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

/* un record: tag, flag 0, lunghezza, dati */
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

int dt_engine_save_mem(dt_engine *e, int compress, uint8_t **out_p, size_t *out_n)
{
    e_buf out = {0}, b = {0};
    uint32_t nrec = 0, regs[18], *bases = NULL, nb = 0, i;
    uc_mem_region *rg = NULL;
    uint8_t *page = NULL, *z = NULL;
    int rc = DT_ERR_NOMEM, k;
    dt_machine *m = e->m;

    *out_p = NULL;
    *out_n = 0;
    if (e->fast_save) {                         /* niente copie da realloc: ~40 pagine da 1 MB */
        out.p = (uint8_t *)malloc((size_t)48 << 20);
        out.cap = out.p ? (size_t)48 << 20 : 0;
    }
    if (e_buf_put(&out, "DTST", 4) || w_le32(&out, 1) || w_le32(&out, 0) || w_le32(&out, 0))
        goto fail;
    /* REGS (PC: quello del giro, come _save_session) */
    e_wr(e, UC_M68K_REG_PC, e->pc);
    dt_machine_regs(m, regs);
    b.n = 0;
    for (k = 0; k < 18; k++)
        w_le32(&b, regs[k]);
    if (w_rec(&out, &nrec, "REGS", &b))
        goto fail;
    /* EMAC: macsr, mask, acc0-3 (snapshot.EMAC_REGS) */
    b.n = 0;
    {
        static const int ids[6] = {UC_M68K_REG_MACSR, UC_M68K_REG_MAC_MASK, UC_M68K_REG_MACC0,
                                   UC_M68K_REG_MACC1, UC_M68K_REG_MACC2, UC_M68K_REG_MACC3};
        for (k = 0; k < 6; k++) {
            uint64_t v = 0;
            if (k < 2) {
                uint32_t v32 = 0;
                uc_reg_read(e->uc, ids[k], &v32);
                v = v32;
            } else
                uc_reg_read(e->uc, ids[k], &v);
            w_le32(&b, (uint32_t)ids[k]);
            w_le32(&b, 0);
            w_le64(&b, v);
        }
    }
    if (w_rec(&out, &nrec, "EMAC", &b))
        goto fail;
    /* CTLR, MMIO, CNTR */
    b.n = 0;
    for (i = 0; i < m->nctl; i++) {
        w_le32(&b, m->ctl[i].key);
        w_le32(&b, m->ctl[i].value);
    }
    if (w_rec(&out, &nrec, "CTLR", &b))
        goto fail;
    b.n = 0;
    for (i = 0; i < m->nmmio; i++) {
        w_le32(&b, m->mmio[i].addr);
        w_le32(&b, m->mmio[i].value);
    }
    if (w_rec(&out, &nrec, "MMIO", &b))
        goto fail;
    b.n = 0;
    w_le64(&b, m->ff1_count);
    w_le64(&b, m->movec_count);
    if (w_rec(&out, &nrec, "CNTR", &b))
        goto fail;
    /* MAPS e PAGE (pagine non nulle, zlib 6, in ordine di indirizzo) */
    if (uc_mem_regions(e->uc, &rg, &nb) != UC_ERR_OK)
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
    if (w_rec(&out, &nrec, "MAPS", &b))
        goto fail;
    for (i = 0; i < nb; i++) {
        uLongf zn = compressBound(0x100000);
        size_t j;
        int nz = 0;
        if (uc_mem_read(e->uc, bases[i], page, 0x100000) != UC_ERR_OK)
            goto fail;
        for (j = 0; j < 0x100000 && !nz; j += 4096) {        /* a blocchi da 4 KB, senza uscite anticipate: vettorizza */
            uint64_t acc = 0;
            size_t q;
            for (q = 0; q < 4096; q += 8) {
                uint64_t w;
                memcpy(&w, page + j + q, 8);
                acc |= w;
            }
            nz = acc != 0;
        }
        if (!nz)
            continue;
        if (compress2(z, &zn, page, 0x100000, e->fast_save ? 0 : 6) != Z_OK)
            goto fail;
        b.n = 0;
        w_le32(&b, bases[i]);
        w_le32(&b, 0x100000);
        e_buf_put(&b, z, zn);
        if (w_rec(&out, &nrec, "PAGE", &b))
            goto fail;
    }
    /* FWID */
    {
        dt_sha256 h;
        uint8_t sha[32];
        dt_sha256_init(&h);
        dt_sha256_update(&h, e->main_os->data, e->main_os->len);
        dt_sha256_final(&h, sha);
        b.n = 0;
        e_buf_put(&b, sha, 32);
        if (w_rec(&out, &nrec, "FWID", &b))
            goto fail;
    }
    if (e->boot_only)
        goto csdh;
    /* CUIN */
    b.n = 0;
    for (i = 0; i < e->uart_in.n; i++)
        w_u8(&b, e->uart_in.p[(e->uart_in.head + i) % e->uart_in.cap]);
    if (w_rec(&out, &nrec, "CUIN", &b))
        goto fail;
    /* CETX */
    b.n = 0;
    w_le32(&b, 35);
    w_le32(&b, 155);
    w_le64(&b, (uint64_t)(int64_t)e->etx.pending);
    w_le64(&b, (uint64_t)e->etx.bytes);
    w_le64(&b, (uint64_t)e->etx.transfers);
    if (w_rec(&out, &nrec, "CETX", &b))
        goto fail;
    /* CSSI */
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
        w_le32(&b, 0);                         /* tx_crc32: diagnostica del Python */
        w_le64(&b, (uint64_t)s->vector170);
        w_le64(&b, (uint64_t)s->vector191);
        if (w_rec(&out, &nrec, "CSSI", &b))
            goto fail;
    }
    /* CSDH: overlay in ordine di settore */
csdh:
    {
        e_esdhc *sd = &e->sd;
        e_sector **v = (e_sector **)malloc((sd->ov.n ? sd->ov.n : 1) * sizeof *v);
        size_t n = 0, j;
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
    /* CSDE: l'intervallo di cancellazione in attesa (card_erase) */
    b.n = 0;
    w_u8(&b, (uint8_t)!!e->sd.has_erase_start);
    w_le64(&b, e->sd.has_erase_start ? e->sd.erase_start : 0);
    w_u8(&b, (uint8_t)!!e->sd.has_erase_end);
    w_le64(&b, e->sd.has_erase_end ? e->sd.erase_end : 0);
    if (w_rec(&out, &nrec, "CSDE", &b))
        goto fail;
    if (e->boot_only)
        goto mani;
    /* CTIM */
    b.n = 0;
    w_le32(&b, (uint32_t)e->ntim);
    for (k = 0; k < e->ntim; k++) {
        e_tsrc *s = &e->tim[k];
        int c;
        w_le32(&b, (uint32_t)s->kind);
        w_le32(&b, (uint32_t)s->t.nch);
        for (c = 0; c < 4; c++)
            w_le32(&b, c < s->t.nch ? (uint32_t)s->t.ch[c] : 0);
        w_le64(&b, (uint64_t)s->ips);
        w_le64(&b, (uint64_t)s->now);
        w_u8(&b, (uint8_t)!!s->t.held);
        for (c = 0; c < 4; c++) {
            w_u8(&b, s->t.has[c] ? 1 : 0);
            w_f64(&b, s->t.has[c] ? s->t.next[c] : 0.0);
        }
        for (c = 0; c < 4; c++)
            w_le64(&b, (uint64_t)s->t.fired[c]);
        for (c = 0; c < 4; c++)
            w_le64(&b, (uint64_t)s->t.missed[c]);
        w_le32(&b, s->arm);
        w_le32(&b, s->stale);
    }
    if (w_rec(&out, &nrec, "CTIM", &b))
        goto fail;
    /* MANI (informativo) */
mani:
    {
        char mani[256];
        int n = snprintf(mani, sizeof mani, "{\"writer\": \"mccore %s\", \"instrs\": %lld}", MCCORE_VERSION,
                         (long long)e->instrs);
        b.n = 0;
        e_buf_put(&b, mani, (size_t)n);
        if (w_rec(&out, &nrec, "MANI", &b))
            goto fail;
    }
    memcpy(out.p + 8, &nrec, 4);
    if (compress) {
        uLongf zn = compressBound((uLong)out.n);
        uint8_t *zz = (uint8_t *)malloc(12 + zn);
        uint64_t raw = out.n;
        if (!zz || compress2(zz + 12, &zn, out.p, (uLong)out.n, 1) != Z_OK) {
            free(zz);
            goto fail;
        }
        memcpy(zz, "DTSZ", 4);
        for (k = 0; k < 8; k++)
            zz[4 + k] = (uint8_t)(raw >> (8 * k));
        free(out.p);
        out.p = zz;
        out.n = 12 + zn;
    }
    *out_p = out.p;
    *out_n = out.n;
    out.p = NULL;
    rc = DT_OK;
fail:
    if (rg)
        uc_free(rg);
    free(bases);
    free(page);
    free(z);
    free(b.p);
    free(out.p);
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

/* Il salvataggio per il plugin in due fasi (come dt_state_finish del Digitakt): il
 * thread del motore fa solo la fotografia, con le pagine in deflate "stored"
 * (praticamente una copia: il motore si ferma per qualche decina di millisecondi
 * invece di mezzo secondo), e chi salva le ricomprime e comprime il tutto. Lo
 * stato che esce ha gli stessi record e gli stessi dati di dt_engine_save_mem. */
int dt_engine_save_fast(dt_engine *e, uint8_t **out_p, size_t *out_n)
{
    int r;
    e->fast_save = 1;
    r = dt_engine_save_mem(e, 0, out_p, out_n);
    e->fast_save = 0;
    return r;
}

int dt_state_pack(const uint8_t *in, size_t n, uint8_t **out_p, size_t *out_n)
{
    e_buf out = {0}, b = {0};
    size_t off = 16;
    uint32_t nrec = 0;
    uint8_t *raw = NULL, *z = NULL;
    int rc = DT_ERR_FORMAT, k;
    *out_p = NULL;
    *out_n = 0;
    if (n < 16 || memcmp(in, "DTST", 4))
        return DT_ERR_FORMAT;
    raw = (uint8_t *)malloc(0x100000);
    z = (uint8_t *)malloc(compressBound(0x100000));
    if (!raw || !z) {
        rc = DT_ERR_NOMEM;
        goto fail;
    }
    if (e_buf_put(&out, in, 16))
        goto nomem;
    while (off + 16 <= n) {
        uint64_t len = e_le64(in + off + 8);
        const uint8_t *d = in + off + 16;
        if (off + 16 + len > n)
            goto fail;
        if (!memcmp(in + off, "PAGE", 4) && len > 8) {
            uLongf rn = 0x100000, zn = compressBound(0x100000);
            if (uncompress(raw, &rn, d + 8, (uLong)(len - 8)) != Z_OK || rn != 0x100000
                || compress2(z, &zn, raw, 0x100000, 3) != Z_OK)
                goto fail;
            b.n = 0;
            e_buf_put(&b, d, 8);
            e_buf_put(&b, z, zn);
            if (w_rec(&out, &nrec, "PAGE", &b))
                goto nomem;
        } else {
            if (e_buf_put(&out, in + off, 16) || (len && e_buf_put(&out, d, (size_t)len)))
                goto nomem;
            nrec++;
        }
        off += 16 + (size_t)len;
    }
    memcpy(out.p + 8, &nrec, 4);
    {
        uLongf zn = compressBound((uLong)out.n);
        uint8_t *zz = (uint8_t *)malloc(12 + zn);
        uint64_t rawn = out.n;
        if (!zz || compress2(zz + 12, &zn, out.p, (uLong)out.n, 1) != Z_OK) {
            free(zz);
            goto fail;
        }
        memcpy(zz, "DTSZ", 4);
        for (k = 0; k < 8; k++)
            zz[4 + k] = (uint8_t)(rawn >> (8 * k));
        *out_p = zz;
        *out_n = 12 + zn;
    }
    rc = DT_OK;
    goto fail;
nomem:
    rc = DT_ERR_NOMEM;
fail:
    free(out.p);
    free(b.p);
    free(raw);
    free(z);
    return rc;
}
