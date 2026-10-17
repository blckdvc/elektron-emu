/* engine.c, parte 5: emu/audio.py (SampleMemory, Synth, WavSink) in C.
 *
 * Stessi risultati del numpy, campione per campione: float32 dove il numpy
 * lavora in float32 (con gli scalari Python "deboli" di NEP 50 convertiti a
 * float32), float64 dove lavora in float64, nello stesso ordine di
 * operazioni; niente FMA (CMake: -ffp-contract=off). tanh: vedi au_tanhf.
 * Verifica: tests/test_dtcore_audio.py contro la Synth di Python. */

#define AU_BLOCK 32
#define AU_CHUNK 16
#define AU_N (AU_BLOCK * AU_CHUNK)
#define AU_PAGE 0x1000u
#define AU_CMD 0xFFFFFFFFu
#define AU_BLOCK_SEND 0x400CD638u
static const uint8_t AU_BLOCK_VERIFY[] = {0x4f, 0xef, 0xff, 0xf4, 0x48, 0xd7, 0x0c, 0x04, 0x24, 0x6f,
                                          0x00, 0x10, 0x26, 0x6f, 0x00, 0x14, 0x24, 0x2f, 0x00, 0x18};
#define AU_TRACK_SLOT 0x8000339Au
#define AU_TRACK_STRIDE 0x8E
#define AU_SLOT_DESC 0x405B5368u
#define AU_SLOTS 1024
#define AU_SLOT_LEN 0x405BA768u
#define AU_DESC_VEC 0x405A0F64u
#define AU_DESC_SIZE 0x34u

/* un array float32 condiviso (in Python: l'oggetto numpy, vivo finche' qualcuno lo usa) */
typedef struct {
    int32_t refs;
    size_t n;
    float d[];
} au_arr;

static au_arr *au_new(size_t n)
{
    au_arr *a = (au_arr *)malloc(sizeof(au_arr) + (n ? n : 1) * sizeof(float));
    if (a) {
        a->refs = 1;
        a->n = n;
    }
    return a;
}

static void au_unref(au_arr *a)
{
    if (a && --a->refs == 0)
        free(a);
}

/* ---------------------------------------------------- SampleMemory */
static size_t au_hash(uint32_t k, size_t cap)
{
    return (size_t)(((uint64_t)k * 0x9E3779B97F4A7C15ull) >> 20) & (cap - 1);
}

static e_page *au_page_find(e_audio *a, uint32_t idx)
{
    size_t i;
    if (!a->pcap)
        return NULL;
    for (i = au_hash(idx, a->pcap);; i = (i + 1) & (a->pcap - 1)) {
        if (!a->pages[i].data)
            return NULL;
        if (a->pages[i].idx == idx)
            return &a->pages[i];
    }
}

static e_page *au_page_put(e_audio *a, uint32_t idx)
{
    e_page *p = au_page_find(a, idx);
    size_t i;
    if (p)
        return p;
    if (2 * (a->pn + 1) > a->pcap) {
        size_t cap = a->pcap ? 2 * a->pcap : 256, k;
        e_page *t = (e_page *)calloc(cap, sizeof *t);
        if (!t)
            return NULL;
        for (k = 0; k < a->pcap; k++)
            if (a->pages[k].data) {
                for (i = au_hash(a->pages[k].idx, cap); t[i].data; i = (i + 1) & (cap - 1))
                    ;
                t[i] = a->pages[k];
            }
        free(a->pages);
        a->pages = t;
        a->pcap = cap;
    }
    for (i = au_hash(idx, a->pcap); a->pages[i].data; i = (i + 1) & (a->pcap - 1))
        ;
    a->pages[i].idx = idx;
    a->pages[i].data = (uint8_t *)calloc(1, AU_PAGE);
    if (!a->pages[i].data)
        return NULL;
    a->pn++;
    return &a->pages[i];
}

/* gli slot: tabella piccola a scansione (al piu' 1024) */
typedef struct {
    uint32_t slot, left, right, nbytes, nsamp;
    au_arr *cache;
} au_slot;

static au_slot *au_slot_find(e_audio *a, uint32_t slot)
{
    size_t i;
    au_slot *s = (au_slot *)a->slots;
    for (i = 0; i < a->sn; i++)
        if (s[i].slot == slot)
            return &s[i];
    return NULL;
}

static void au_cache_clear(e_audio *a)
{
    size_t i;
    au_slot *s = (au_slot *)a->slots;
    for (i = 0; i < a->sn; i++) {
        au_unref(s[i].cache);
        s[i].cache = NULL;
    }
}

static void au_mem_read(e_audio *a, uint32_t addr, uint8_t *out, size_t n)
{
    while (n > 0) {
        uint32_t page = addr / AU_PAGE, off = addr % AU_PAGE;
        size_t k = n < AU_PAGE - off ? n : AU_PAGE - off;
        e_page *p = au_page_find(a, page);
        if (p)
            memcpy(out, p->data + off, k);
        else
            memset(out, 0, k);
        out += k;
        addr += (uint32_t)k;
        n -= k;
    }
}

/* SampleMemory.on_block */
static void au_on_block(e_audio *a, uint32_t cmd, const uint8_t *data)
{
    a->blocks++;
    if (cmd == AU_CMD) {
        uint32_t slot = e_be32(data), left = e_be32(data + 4), right = e_be32(data + 8);
        uint32_t nbytes = e_be32(data + 12), nsamp = e_be32(data + 16);
        au_slot *s = au_slot_find(a, slot);
        if (s) {                                         /* _cache.pop(slot) */
            au_unref(s->cache);
            s->cache = NULL;
        }
        if (left >= 0x80000000u || !nsamp) {
            if (s) {                                     /* slots.pop(slot) */
                au_slot *t = (au_slot *)a->slots;
                size_t i = (size_t)(s - t);
                memmove(&t[i], &t[i + 1], (a->sn - i - 1) * sizeof *t);
                a->sn--;
            }
        } else {
            if (!s) {
                if (a->sn == a->scap) {
                    size_t cap = a->scap ? 2 * a->scap : 64;
                    au_slot *t = (au_slot *)realloc(a->slots, cap * sizeof *t);
                    if (!t)
                        return;
                    a->slots = t;
                    a->scap = cap;
                }
                s = &((au_slot *)a->slots)[a->sn++];
                s->cache = NULL;
            }
            s->slot = slot; s->left = left; s->right = right; s->nbytes = nbytes; s->nsamp = nsamp;
        }
    } else {
        e_page *p = au_page_put(a, cmd);
        if (p)
            memcpy(p->data, data, AU_PAGE);
        au_cache_clear(a);
    }
}

/* SampleMemory.pcm: float32 mono in [-1, 1], o NULL */
static au_arr *au_pcm(e_audio *a, uint32_t slot)
{
    au_slot *s = au_slot_find(a, slot);
    size_t n, i;
    int stereo;
    uint8_t *raw;
    au_arr *r;
    if (!s)
        return NULL;
    if (s->cache)
        return s->cache;
    stereo = s->right != s->left;
    n = stereo ? s->nsamp / 2 : s->nsamp;
    r = au_new(n);
    raw = (uint8_t *)malloc(2 * n + 1);
    if (!r || !raw) {
        free(raw);
        au_unref(r);
        return NULL;
    }
    au_mem_read(a, s->left, raw, 2 * n);
    for (i = 0; i < n; i++)
        r->d[i] = (float)(int16_t)e_be16(raw + 2 * i);
    if (stereo) {
        au_mem_read(a, s->right, raw, 2 * n);
        for (i = 0; i < n; i++)
            r->d[i] = (r->d[i] + (float)(int16_t)e_be16(raw + 2 * i)) / 2.0f;
    }
    for (i = 0; i < n; i++)
        r->d[i] /= 32768.0f;
    free(raw);
    s->cache = r;
    return r;
}

/* SampleMemory.channels (per emu/dspi1.py): 1 e i due array (destro == sinistro se mono) */
static int audio_channels(dt_engine *e, uint32_t slot, float **left, float **right, size_t *n)
{
    e_audio *a = &e->au;
    au_slot *s = au_slot_find(a, slot);
    size_t k, i;
    int stereo;
    uint8_t *raw;
    if (!s)
        return 0;
    stereo = s->right != s->left;
    k = stereo ? s->nsamp / 2 : s->nsamp;
    *left = (float *)malloc((k ? k : 1) * sizeof(float));
    raw = (uint8_t *)malloc(2 * k + 1);
    if (!*left || !raw) {
        free(*left);
        free(raw);
        *left = NULL;
        return 0;
    }
    au_mem_read(a, s->left, raw, 2 * k);
    for (i = 0; i < k; i++)
        (*left)[i] = (float)(int16_t)e_be16(raw + 2 * i) / 32768.0f;
    if (stereo) {
        *right = (float *)malloc((k ? k : 1) * sizeof(float));
        if (!*right) {
            free(raw);
            free(*left);
            *left = NULL;
            return 0;
        }
        au_mem_read(a, s->right, raw, 2 * k);
        for (i = 0; i < k; i++)
            (*right)[i] = (float)(int16_t)e_be16(raw + 2 * i) / 32768.0f;
    } else
        *right = *left;
    free(raw);
    *n = k;
    return 1;
}

/* ----------------------------------------------------------- Synth */
typedef struct {
    int track;
    au_arr *sample;
    double pos, step;
    double gain;
    int offset, end;
} au_voice;

/* gui._write_kick + audio._load_wav_mono_float: il segnaposto */
static au_arr *au_kick(void)
{
    const int rate = 48000;
    int n = (int)(0.4 * rate), i;
    double phase = 0.0;
    au_arr *a = au_new((size_t)n);
    if (!a)
        return NULL;
    for (i = 0; i < n; i++) {
        double t = (double)i / rate;
        double f = 45 + 75 * exp(-t / 0.04);
        double v;
        int16_t s16;
        phase += 2 * M_PI * f / rate;
        v = sin(phase) * exp(-t / 0.18);
        s16 = (int16_t)(int64_t)(v * 28000);
        a->d[i] = (float)s16 / 32768.0f;
    }
    return a;
}

/* np.tanh su float32: implementata nel numpy (non e' tanhf della libc). Vedi
   au_tanhf in e_tanh.inc.c, verificata su tutti i float32 positivi. */
static float au_tanhf(float x);

static void au_emit(dt_engine *e, const float *out)
{
    e_audio *a = &e->au;
    uint8_t buf[AU_N * 2];
    int i;
    for (i = 0; i < AU_N; i++) {
        float y = out[i] * 32767.0f;
        e_put16(buf + 2 * i, 0);
        {
            int16_t v = (int16_t)(int32_t)y;          /* astype('<i2') */
            buf[2 * i] = (uint8_t)((uint16_t)v & 0xFF);
            buf[2 * i + 1] = (uint8_t)((uint16_t)v >> 8);
        }
    }
    a->samples += AU_N;
    a->crc = e_crc(a->crc, buf, sizeof buf);
    if (a->sink)
        a->sink(a->sink_ctx, out, a->tracks_on ? &a->tracks[0][0] : NULL, AU_N);
}

static void au_render(dt_engine *e)
{
    e_audio *a = &e->au;
    au_voice *v = (au_voice *)a->active;
    float *out = a->out;
    size_t keep = 0, i;
    const float KNEE = 0.800000011920928955078125f;          /* float32(0.8) */
    const float ONE_MINUS_KNEE = 0.20000000298023223876953125f;  /* float32(1 - 0.8) */
    memset(out, 0, sizeof a->out);
    if (a->tracks_on)
        memset(a->tracks, 0, sizeof a->tracks);
    if (!a->nactive) {
        au_emit(e, out);
        return;
    }
    for (i = 0; i < a->nactive; i++) {
        int s0 = v[i].offset, s1 = v[i].end >= 0 ? v[i].end : AU_N, count = s1 - s0;
        const float *smp = v[i].sample->d;
        int64_t last = (int64_t)v[i].sample->n - 1;
        float g = (float)v[i].gain;
        if (count > 0) {
            if (v[i].step == 1.0) {
                int64_t i0 = (int64_t)v[i].pos, k, nseg;
                nseg = (int64_t)v[i].sample->n - i0;
                if (nseg > count)
                    nseg = count;
                for (k = 0; k < nseg; k++) {
                    float add = smp[i0 + k] * g;
                    out[s0 + k] = out[s0 + k] + add;
                    if (a->tracks_on)
                        a->tracks[v[i].track][s0 + k] += add;
                }
            } else {
                int k;
                for (k = 0; k < count; k++) {
                    double pos = v[i].pos + (double)k * v[i].step;
                    int64_t i0;
                    float frac, seg;
                    if (!(pos < (double)last))
                        break;
                    i0 = (int64_t)pos;
                    frac = (float)(pos - (double)i0);
                    seg = smp[i0] * (1.0f - frac) + smp[i0 + 1] * frac;
                    out[s0 + k] = out[s0 + k] + seg * g;
                    if (a->tracks_on)
                        a->tracks[v[i].track][s0 + k] += seg * g;
                }
            }
            v[i].pos += (double)count * v[i].step;
        }
        if (v[i].end < 0 && v[i].pos < (double)last) {
            v[i].offset = 0;
            v[keep++] = v[i];
        } else
            au_unref(v[i].sample);
    }
    a->nactive = keep;
    if (a->tracks_on) {
        int t;
        for (t = 0; t < 16; t++)
            for (i = 0; i < AU_N; i++)
                a->tracks[t][i] *= a->master;
    }
    for (i = 0; i < AU_N; i++) {
        float x = out[i] * a->master, mag = fabsf(x);
        if (mag > KNEE) {
            float sg = x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f);
            x = sg * (KNEE + ONE_MINUS_KNEE * au_tanhf((mag - KNEE) / ONE_MINUS_KNEE));
        }
        out[i] = x;
    }
    au_emit(e, out);
}

static int au_slot_loaded(dt_engine *e, uint32_t slot)
{
    uint32_t begin = e_r32(e, AU_DESC_VEC), end = e_r32(e, AU_DESC_VEC + 4);
    int64_t count = end >= begin ? (int64_t)((end - begin) / AU_DESC_SIZE) : 0;
    int32_t desc = (int32_t)e_r32(e, AU_SLOT_DESC + 4 * slot);
    uint32_t length = e_r32(e, AU_SLOT_LEN + 4 * slot);
    return 0 <= desc && desc < count && 0 < length && length < 0x7FFFFFFFu;
}

/* slot_sample(t): -> pcm, NULL (None) o (au_arr *)1 (EMPTY) */
#define AU_EMPTY ((au_arr *)1)
static au_arr *au_slot_sample(dt_engine *e, int t)
{
    uint32_t slot;
    if (e->au.no_slots)                          /* test: Synth(bank, slot_sample=None) */
        return NULL;
    slot = e_r16(e, AU_TRACK_SLOT + (uint32_t)t * AU_TRACK_STRIDE);
    au_arr *p = au_pcm(&e->au, slot);
    if (p)
        return p;
    if (slot < AU_SLOTS && !au_slot_loaded(e, slot))
        return AU_EMPTY;
    return NULL;
}

/* Synth.on_frame */
static void audio_on_frame(dt_engine *e, const uint8_t *frame, size_t n)
{
    e_audio *a = &e->au;
    if (n >= 0x26 && (frame[0x22] | frame[0x23] | frame[0x24] | frame[0x25])) {
        uint32_t on = e_be16(frame + 0x22), off = e_be16(frame + 0x24);
        int offset = a->fill * AU_BLOCK, t;
        for (t = 0; t < 16; t++) {
            au_arr *sample;
            uint32_t note, vel;
            size_t i;
            if ((off >> t) & 1)
                a->note_offs[t]++;
            if (!((on >> t) & 1))
                continue;
            a->note_ons[t]++;
            sample = au_slot_sample(e, t);
            if (sample == AU_EMPTY) {
                a->silent_hits[t]++;
                continue;
            }
            if (sample)
                a->real_hits[t]++;
            else
                sample = (au_arr *)a->kick;
            if (!sample || !sample->n)
                continue;
            for (i = 0; i < a->nactive; i++) {
                au_voice *v = &((au_voice *)a->active)[i];
                if (v->track == t && v->end < 0)
                    v->end = offset;
            }
            note = 0x02 + 2u * (uint32_t)t < n ? frame[0x02 + 2 * t] : 0;
            vel = 0x34 + 2u * (uint32_t)t < n ? frame[0x34 + 2 * t] : 0;
            if (a->nactive == a->capactive) {
                size_t cap = a->capactive ? 2 * a->capactive : 32;
                au_voice *nv = (au_voice *)realloc(a->active, cap * sizeof *nv);
                if (!nv)
                    continue;
                a->active = nv;
                a->capactive = cap;
            }
            {
                au_voice *v = &((au_voice *)a->active)[a->nactive++];
                v->track = t;
                v->sample = sample;
                sample->refs++;
                v->step = pow(2.0, ((double)note - 60) / 12.0);
                v->gain = (double)vel / 127.0;
                v->pos = 0.0;
                v->offset = offset;
                v->end = -1;
            }
        }
    }
    a->fill++;
    a->sblocks++;
    if (a->fill < AU_CHUNK)
        return;
    a->fill = 0;
    au_render(e);
}

static void h_audio_block_send(uc_engine *uc, uint64_t ad, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[12], page[AU_PAGE];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7);
    (void)uc; (void)ad; (void)s;
    e_read(e, sp, b, 12);
    e_read(e, e_be32(b + 8), page, AU_PAGE);
    au_on_block(&e->au, e_be32(b + 4), page);
}

static void audio_free(e_audio *a)
{
    size_t i;
    for (i = 0; i < a->nactive; i++)
        au_unref(((au_voice *)a->active)[i].sample);
    free(a->active);
    au_cache_clear(a);
    free(a->slots);
    for (i = 0; i < a->pcap; i++)
        free(a->pages[i].data);
    free(a->pages);
    au_unref((au_arr *)a->kick);
    memset(a, 0, sizeof *a);
}
