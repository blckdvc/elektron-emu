/* .syx -> sezioni decompresse, senza Python.
 *
 * Traduzione riga per riga di:
 *   dt2/container.py  decode_syx (SysEx da 128 byte, 8-in-7), container (ELE3),
 *                     sections (tabella da 16 byte a 0x20, conteggio a 0x1C);
 *   emu/extract.py    classify (compressa solo se l'intestazione [len][somma]
 *                     torna), NAMES;
 *   dt2/elz.py        depack / depack_section (il codec LZ delle sezioni).
 * Verifica: tests/test_dtcore.py confronta ogni sezione con emu.extract.
 */
#include "mccore.h"
#include "internal.h"
#include "sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------- buffer */
typedef struct {
    uint8_t *p;
    size_t n, cap;
} buf_t;

static int buf_push(buf_t *b, uint8_t v)
{
    if (b->n == b->cap) {
        size_t cap = b->cap ? b->cap * 2 : 1 << 16;
        uint8_t *q = (uint8_t *)realloc(b->p, cap);
        if (!q)
            return -1;
        b->p = q;
        b->cap = cap;
    }
    b->p[b->n++] = v;
    return 0;
}

/* ------------------------------------------------------- container.py */
static int decode_syx(const uint8_t *d, size_t len, buf_t *out, char *err, size_t errlen)
{
    size_t i = 0;
    while (i < len) {
        size_t j, k;
        const uint8_t *body, *payload;
        if (d[i] != 0xF0) {
            dt_errf(err, errlen, "expected F0 at offset %zu", i);
            return DT_ERR_FORMAT;
        }
        for (j = i; j < len && d[j] != 0xF7; j++)
            ;
        if (j == len) {                       /* d.index(0xF7, i): ValueError */
            dt_errf(err, errlen, "no F7 after offset %zu", i);
            return DT_ERR_FORMAT;
        }
        body = d + i + 1;
        k = j - (i + 1);                      /* len(body) */
        i = j + 1;
        if (k != 126)                         /* marcatori di inizio/fine da 14 byte */
            continue;
        payload = body + 9;                   /* 9 di intestazione, 116 di dati, 1 di checksum */
        k = 0;
        while (k < 116) {
            uint8_t ms = payload[k++];
            int n;
            for (n = 0; n < 7; n++) {
                if (k >= 116)
                    break;
                if (buf_push(out, (uint8_t)(payload[k] | (((ms >> (6 - n)) & 1) ? 0x80 : 0))))
                    return DT_ERR_NOMEM;
                k++;
            }
        }
    }
    return DT_OK;
}

/* --------------------------------------------------------------- elz.py */
#define ELZ_BIAS 767u
#define ELZ_REUSE 2u
#define ELZ_FAR 3328u

typedef struct {
    const uint8_t *d;
    size_t p, end;
    uint32_t tag;
    int eof;
} bits_t;

static uint32_t elz_byte(bits_t *s)
{
    if (s->p >= s->end) {
        s->eof = 1;
        return 0;
    }
    return s->d[s->p++];
}

static uint32_t elz_bit(bits_t *s)
{
    s->tag = (s->tag << 1) & 0x1FF;
    if ((s->tag & 0xFF) == 0) {
        uint32_t b = elz_byte(s);
        s->tag = (b << 1) | 1;
        return b >> 7;
    }
    return s->tag >> 8;
}

static int elz_gamma(bits_t *s, uint64_t *out)
{
    uint64_t v = 1;
    for (;;) {
        v = (v << 1) | elz_bit(s);
        if (elz_bit(s)) {
            *out = v;
            return 0;
        }
        if (s->eof)
            return -1;
        if (v > 0x02000000u)
            return -2;
    }
}

/* -> 0 e *endpos, o <0 */
static int elz_depack(const uint8_t *data, size_t pos, size_t end, buf_t *out, size_t *endpos,
                      char *err, size_t errlen)
{
    bits_t s = {data, pos, end, 0, 0};
    uint64_t last = 1;
    for (;;) {
        uint64_t g, off, length, i;
        uint32_t shortlen;
        if (elz_bit(&s)) {
            uint32_t b = elz_byte(&s);
            if (s.eof)
                goto eof;
            if (buf_push(out, (uint8_t)b))
                return DT_ERR_NOMEM;
            continue;
        }
        if (s.eof)
            goto eof;
        if (elz_gamma(&s, &g)) {
            if (s.eof)
                goto eof;
            dt_errf(err, errlen, "gamma overflow");
            return DT_ERR_FORMAT;
        }
        if (g == ELZ_REUSE) {
            off = last;
        } else {
            uint64_t raw = ((g << 8) + elz_byte(&s)) & 0xFFFFFFFFu;
            if (s.eof)
                goto eof;
            if (raw == ELZ_BIAS) {
                *endpos = s.p;
                return DT_OK;
            }
            off = (raw - ELZ_BIAS) & 0xFFFFFFFFu;
            last = off;
        }
        shortlen = 2 * elz_bit(&s);
        shortlen += elz_bit(&s);
        if (shortlen) {
            length = shortlen;
        } else {
            uint64_t gg;
            if (elz_gamma(&s, &gg)) {
                if (s.eof)
                    goto eof;
                dt_errf(err, errlen, "gamma overflow");
                return DT_ERR_FORMAT;
            }
            length = gg + 2;
        }
        if (s.eof)
            goto eof;
        if (off > ELZ_FAR)
            length += 1;
        if (off == 0 || off > out->n) {
            dt_errf(err, errlen, "offset %llu outside %zu bytes of output",
                    (unsigned long long)off, out->n);
            return DT_ERR_FORMAT;
        }
        for (i = 0; i < length + 1; i++)
            if (buf_push(out, out->p[out->n - off]))
                return DT_ERR_NOMEM;
    }
eof:
    dt_errf(err, errlen, "compressed stream ends early");
    return DT_ERR_FORMAT;
}

static uint32_t be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/* ------------------------------------------------------------ extract.py */
const char *dt_fw_section_name(uint32_t id)
{
    switch (id) {
    case 2: return "DSP";          /* il bootstrap: nome di elektron-firmware-tool */
    case 3: return "MAIN_OS";
    case 4: return "UPDATER";
    case 5: return "META";
    case 7: return "BLOB";
    default: return "SECTION";
    }
}

int dt_fw_extract(const uint8_t *syx, size_t len, dt_firmware *fw, char *err, size_t errlen)
{
    buf_t dec = {0, 0, 0};
    const uint8_t *c;
    size_t clen, off, k, n;
    int r;
    dt_sha256 h;

    if (!syx || !fw)
        return DT_ERR_ARG;
    memset(fw, 0, sizeof *fw);
    dt_sha256_init(&h);
    dt_sha256_update(&h, syx, len);
    dt_sha256_final(&h, fw->syx_sha256);

    r = decode_syx(syx, len, &dec, err, errlen);
    if (r)
        goto fail;
    for (off = 0; off + 4 <= dec.n; off++)
        if (memcmp(dec.p + off, "ELE3", 4) == 0)
            break;
    if (off + 4 > dec.n) {
        dt_errf(err, errlen, "no ELE3 magic; unsupported container");
        r = DT_ERR_FORMAT;
        goto fail;
    }
    c = dec.p + off;
    clen = dec.n - off;
    if (clen < 0x20) {
        dt_errf(err, errlen, "container too short");
        r = DT_ERR_FORMAT;
        goto fail;
    }
    n = be32(c + 0x1C);
    if (n > DT_MAX_SECTIONS || 0x20 + 16 * n > clen) {
        dt_errf(err, errlen, "bad section count %zu", n);
        r = DT_ERR_FORMAT;
        goto fail;
    }
    for (k = 0; k < n; k++) {
        const uint8_t *e = c + 0x20 + 16 * k;
        dt_section *s = &fw->sec[k];
        const uint8_t *stream;
        size_t slen, so;
        s->id = be32(e);
        s->offset = be32(e + 4);
        s->clen = be32(e + 8);
        s->dest = be32(e + 12);
        /* c[off:off + clen], con lo slicing del Python (taglia alla fine) */
        so = s->offset < clen ? s->offset : clen;
        slen = s->clen < clen - so ? s->clen : clen - so;
        stream = c + so;
        s->packed = 0;
        if (slen >= 8) {
            uint32_t ln = be32(stream), sm = be32(stream + 4), sum = 0;
            size_t i;
            if ((uint64_t)ln + 8 <= slen) {
                for (i = 0; i < ln; i++)
                    sum += stream[8 + i];
                if (sum == sm)
                    s->packed = 1;
            }
            if (s->packed) {
                buf_t out = {0, 0, 0};
                size_t endpos = 0;
                r = elz_depack(stream, 8, 8 + (size_t)ln, &out, &endpos, err, errlen);
                if (r == DT_OK && endpos != 8 + (size_t)ln) {
                    dt_errf(err, errlen, "end marker at %zu of %u stream bytes", endpos - 8, ln);
                    r = DT_ERR_FORMAT;
                }
                if (r) {
                    free(out.p);
                    fw->count = (int32_t)k;
                    goto fail;
                }
                s->data = out.p;
                s->len = out.n;
                continue;
            }
            if (sm == 0) {                     /* raw con intestazione da togliere (sez. 4) */
                stream += 8;
                slen -= 8;
            }
        }
        s->data = (uint8_t *)malloc(slen ? slen : 1);
        if (!s->data) {
            r = DT_ERR_NOMEM;
            fw->count = (int32_t)k;
            goto fail;
        }
        memcpy(s->data, stream, slen);
        s->len = slen;
    }
    fw->count = (int32_t)n;
    fw->container = (uint8_t *)malloc(clen);
    if (!fw->container) {
        r = DT_ERR_NOMEM;
        goto fail;
    }
    memcpy(fw->container, c, clen);
    fw->container_len = clen;
    free(dec.p);
    return DT_OK;
fail:
    free(dec.p);
    dt_fw_free(fw);
    return r;
}

void dt_fw_free(dt_firmware *fw)
{
    int i;
    if (!fw)
        return;
    for (i = 0; i < DT_MAX_SECTIONS; i++) {
        free(fw->sec[i].data);
        fw->sec[i].data = NULL;
        fw->sec[i].len = 0;
    }
    fw->count = 0;
    free(fw->container);
    fw->container = NULL;
    fw->container_len = 0;
}

const dt_section *dt_fw_section(const dt_firmware *fw, uint32_t id)
{
    int i;
    for (i = 0; fw && i < fw->count; i++)
        if (fw->sec[i].id == id)
            return &fw->sec[i];
    return NULL;
}

const char *dt_version(void)
{
    return DTCORE_VERSION;
}
