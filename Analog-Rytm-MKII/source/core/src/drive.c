/* Il +Drive visto da un computer (emu/transfer.py Client, emu/rytm_rpc.py
 * wav_to_rytm), in C, per il pannello +DRIVE del plugin: le richieste RPC
 * vanno in SysEx agli handler veri del firmware emulato (dt_rt_rpc). Dal
 * Digitakt II (digitakt/emulator/core/src/drive.c). */
#include "arcore.h"
#include "internal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void sleep_ms(int ms)
{
    struct timespec ts = {0, (long)ms * 1000000L};
    nanosleep(&ts, NULL);
}

/* SysexRpcLink.rpc: la richiesta, poi l'attesa della risposta con lo stesso
   seq (le altre si scartano: c'e' un solo cliente). -> 0, la risposta in *rep */
static int rpc_call(dt_rt *rt, uint8_t rid, const uint8_t *body, size_t n, uint8_t *id, uint8_t **rep,
                    size_t *rn, int timeout_ms)
{
    uint16_t seq = 0;
    uint8_t *buf = NULL;
    size_t have = 0, cap = 0;
    int waited = 0;
    *rep = NULL;
    *rn = 0;
    if (dt_rt_rpc(rt, rid, body, n, &seq))
        return DT_ERR_IO;
    while (waited < timeout_ms) {
        uint8_t tmp[16384];
        size_t k = dt_rt_rpc_replies(rt, tmp, sizeof tmp);
        if (k) {
            if (have + k > cap) {
                cap = (have + k) * 2;
                buf = (uint8_t *)realloc(buf, cap);
                if (!buf)
                    return DT_ERR_NOMEM;
            }
            memcpy(buf + have, tmp, k);
            have += k;
        }
        while (have >= 7) {                         /* [u16 seq, u8 id, u32 n, n byte] */
            uint32_t bl = (uint32_t)buf[3] << 24 | (uint32_t)buf[4] << 16 | (uint32_t)buf[5] << 8 | buf[6];
            uint16_t s = (uint16_t)(buf[0] << 8 | buf[1]);
            if (have < 7 + bl)
                break;
            if (s == seq) {
                *id = buf[2];
                *rep = (uint8_t *)malloc(bl ? bl : 1);
                if (!*rep) {
                    free(buf);
                    return DT_ERR_NOMEM;
                }
                memcpy(*rep, buf + 7, bl);
                *rn = bl;
                free(buf);
                return DT_OK;
            }
            memmove(buf, buf + 7 + bl, have - 7 - bl);
            have -= 7 + bl;
        }
        if (!k) {
            sleep_ms(2);
            waited += 2;
        }
    }
    free(buf);
    return DT_ERR_IO;
}

#define RD_READ_DIR 0x10
#define RD_CREATE_DIR 0x11
#define RD_DELETE_DIR 0x12
#define RD_DELETE_FILE 0x20
#define RD_OPEN_WRITE 0x40
#define RD_CLOSE_WRITE 0x41
#define RD_WRITE_FILE 0x42
#define RD_CHUNK 0x2000
#define RD_TIMEOUT 20000

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/* Client._ok: risposta che comincia con 1. -> 0, o un errore con il testo */
static int rpc_ok(dt_rt *rt, uint8_t rid, const uint8_t *body, size_t n, uint8_t **rep, size_t *rn,
                  char *err, size_t errlen, const char *what)
{
    uint8_t id = 0;
    int r = rpc_call(rt, rid, body, n, &id, rep, rn, RD_TIMEOUT);
    if (r) {
        dt_errf(err, errlen, "%s: il firmware non risponde (RPC 0x%02x)", what, rid);
        return r;
    }
    if (!*rn || (*rep)[0] != 1) {
        dt_errf(err, errlen, "%s: il firmware risponde %02x %02x", what, id, *rn ? (*rep)[0] : 0);
        free(*rep);
        *rep = NULL;
        return DT_ERR_IO;
    }
    return DT_OK;
}

static int path_call(dt_rt *rt, uint8_t rid, const char *path, char *err, size_t errlen, const char *what)
{
    uint8_t *rep = NULL;
    size_t rn = 0;
    int r = rpc_ok(rt, rid, (const uint8_t *)path, strlen(path) + 1, &rep, &rn, err, errlen, what);
    free(rep);
    return r;
}

int dt_drive_mkdir(dt_rt *rt, const char *path, char *err, size_t errlen)
{
    return path_call(rt, RD_CREATE_DIR, path, err, errlen, "crea cartella");
}

int dt_drive_rmdir(dt_rt *rt, const char *path, char *err, size_t errlen)
{
    return path_call(rt, RD_DELETE_DIR, path, err, errlen, "elimina cartella");
}

int dt_drive_rm(dt_rt *rt, const char *path, char *err, size_t errlen)
{
    return path_call(rt, RD_DELETE_FILE, path, err, errlen, "elimina");
}

/* Client.ls + parse_dir */
int dt_drive_ls(dt_rt *rt, const char *path, dt_drive_entry **out, size_t *count, char *err, size_t errlen)
{
    uint8_t id = 0, *rep = NULL;
    size_t rn = 0, i = 0, n = 0, cap = 0;
    dt_drive_entry *v = NULL;
    *out = NULL;
    *count = 0;
    if (rpc_call(rt, RD_READ_DIR, (const uint8_t *)path, strlen(path) + 1, &id, &rep, &rn, RD_TIMEOUT)) {
        dt_errf(err, errlen, "lettura di %s: il firmware non risponde", path);
        return DT_ERR_IO;
    }
    while (i + 10 <= rn) {
        size_t j = i + 10;
        while (j < rn && rep[j])
            j++;
        if (j >= rn)
            break;
        if (n == cap) {
            cap = cap ? 2 * cap : 32;
            v = (dt_drive_entry *)realloc(v, cap * sizeof *v);
            if (!v) {
                free(rep);
                return DT_ERR_NOMEM;
            }
        }
        memset(&v[n], 0, sizeof v[n]);
        v[n].hash = get32(rep + i);
        v[n].size = get32(rep + i + 4);
        v[n].protected_ = rep[i + 8];
        v[n].type = (char)rep[i + 9];
        snprintf(v[n].name, sizeof v[n].name, "%.*s", (int)(j - i - 10), (const char *)rep + i + 10);
        n++;
        i = j + 1;
    }
    free(rep);
    *out = v;
    *count = n;
    return DT_OK;
}

/* Client.put: Open, Write a blocchi da 0x2000, Close */
int dt_drive_put(dt_rt *rt, const char *path, const uint8_t *data, size_t len,
                 void (*progress)(void *ctx, size_t off, size_t total), void *ctx, char *err, size_t errlen)
{
    uint8_t *body, *rep = NULL;
    size_t rn = 0, off = 0, pl = strlen(path) + 1;
    uint32_t handle;
    int r;
    body = (uint8_t *)malloc(12 + RD_CHUNK + pl);
    if (!body)
        return DT_ERR_NOMEM;
    put32(body, (uint32_t)len);
    memcpy(body + 4, path, pl);
    r = rpc_ok(rt, RD_OPEN_WRITE, body, 4 + pl, &rep, &rn, err, errlen, "apri");
    if (r) {
        free(body);
        return r;
    }
    if (rn < 5) {
        free(rep);
        free(body);
        dt_errf(err, errlen, "apri %s: risposta corta", path);
        return DT_ERR_IO;
    }
    handle = get32(rep + 1);
    free(rep);
    while (off < len) {
        size_t k = len - off < RD_CHUNK ? len - off : RD_CHUNK;
        put32(body, handle);
        put32(body + 4, (uint32_t)k);
        put32(body + 8, (uint32_t)off);
        memcpy(body + 12, data + off, k);
        r = rpc_ok(rt, RD_WRITE_FILE, body, 12 + k, &rep, &rn, err, errlen, "scrivi");
        free(rep);
        rep = NULL;
        if (r) {
            free(body);
            return r;
        }
        off += k;
        if (progress)
            progress(ctx, off, len);
    }
    put32(body, handle);
    put32(body + 4, (uint32_t)len);
    r = rpc_ok(rt, RD_CLOSE_WRITE, body, 8, &rep, &rn, err, errlen, "chiudi");
    free(rep);
    free(body);
    return r;
}

/* ------------------------------------- rytm_rpc.wav_to_rytm (con tools/wav2dt.py) */
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | p[1] << 8);
}

int dt_wav2rytm(const uint8_t *wav, size_t len, uint8_t **out, size_t *out_len, char *err, size_t errlen)
{
    size_t pos = 12, data_off = 0, data_len = 0, nframes, nout, i;
    int channels = 0, width = 0, have_fmt = 0, c;
    uint32_t rate = 0;
    int16_t *fr;
    uint8_t *blob;
    *out = NULL;
    *out_len = 0;
    if (len < 12 || memcmp(wav, "RIFF", 4) || memcmp(wav + 8, "WAVE", 4)) {
        dt_errf(err, errlen, "non e' un file WAV");
        return DT_ERR_FORMAT;
    }
    while (pos + 8 <= len) {
        uint32_t sz = le32(wav + pos + 4);
        const uint8_t *d = wav + pos + 8;
        size_t avail = len - pos - 8 < sz ? len - pos - 8 : sz;
        if (!memcmp(wav + pos, "fmt ", 4) && avail >= 16) {
            uint16_t tag = le16(d);
            if (tag == 0xFFFE && avail >= 26)
                tag = le16(d + 24);                  /* WAVE_FORMAT_EXTENSIBLE: il sottoformato */
            if (tag != 1) {
                dt_errf(err, errlen, "WAV non PCM (formato %u)", tag);
                return DT_ERR_FORMAT;
            }
            channels = le16(d + 2);
            rate = le32(d + 4);
            width = (le16(d + 14) + 7) / 8;
            have_fmt = 1;
        } else if (!memcmp(wav + pos, "data", 4)) {
            data_off = pos + 8;
            data_len = avail;
        }
        pos += 8 + sz + (sz & 1);
    }
    if (!have_fmt || !data_off) {
        dt_errf(err, errlen, "WAV senza fmt o data");
        return DT_ERR_FORMAT;
    }
    if (channels != 1 && channels != 2) {
        dt_errf(err, errlen, "%d canali: solo mono e stereo", channels);
        return DT_ERR_FORMAT;
    }
    if (width < 1 || width > 4) {
        dt_errf(err, errlen, "campioni da %d byte non supportati", width);
        return DT_ERR_FORMAT;
    }
    nframes = data_len / (size_t)(width * channels);       /* readframes: frame interi */
    fr = (int16_t *)malloc((nframes ? nframes : 1) * channels * sizeof *fr);
    if (!fr)
        return DT_ERR_NOMEM;
    for (i = 0; i < nframes; i++)
        for (c = 0; c < channels; c++) {
            const uint8_t *s = wav + data_off + (i * channels + c) * width;
            int32_t v;
            if (width == 1)
                v = (s[0] - 128) * 256;
            else if (width == 2)
                v = (int16_t)le16(s);
            else if (width == 3)
                v = ((int32_t)((uint32_t)s[0] << 8 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 24) >> 8) >> 8;
            else
                v = (int32_t)le32(s) >> 16;
            fr[i * channels + c] = (int16_t)v;
        }
    /* _resample: lineare, round() di Python (meta' al pari) */
    if (rate != 48000 && nframes) {
        double ratio;
        int16_t *o;
        nout = (size_t)nearbyint((double)((int64_t)nframes * 48000) / rate);
        if (nout < 1)
            nout = 1;
        ratio = nout > 1 ? (double)(nframes - 1) / (double)(nout - 1) : 0;
        o = (int16_t *)malloc(nout * channels * sizeof *o);
        if (!o) {
            free(fr);
            return DT_ERR_NOMEM;
        }
        for (i = 0; i < nout; i++) {
            double p = (double)i * ratio, t;
            size_t lo = (size_t)p, hi = lo + 1 < nframes ? lo + 1 : nframes - 1;
            t = p - (double)lo;
            for (c = 0; c < channels; c++) {
                double a = fr[lo * channels + c], b = fr[hi * channels + c];
                o[i * channels + c] = (int16_t)nearbyint(a + (b - a) * t);
            }
        }
        free(fr);
        fr = o;
        nframes = nout;
    }
    /* wav_to_rytm: mono = int(sum(f) / len(f)) (troncato verso zero), 64 byte di
       intestazione (u32 BE: 0, byte del PCM, 48000, 0, frame - 1, 0x7f), PCM a 16 bit BE */
    *out_len = 0x40 + nframes * 2;
    blob = (uint8_t *)calloc(1, *out_len);
    if (!blob) {
        free(fr);
        return DT_ERR_NOMEM;
    }
    put32(blob + 4, (uint32_t)(nframes * 2));
    put32(blob + 8, 48000);
    put32(blob + 16, nframes ? (uint32_t)(nframes - 1) : 0);
    put32(blob + 20, 0x7F);
    for (i = 0; i < nframes; i++) {
        double sum = 0;
        int16_t m;
        for (c = 0; c < channels; c++)
            sum += fr[i * channels + c];
        m = (int16_t)(sum / channels);
        blob[0x40 + 2 * i] = (uint8_t)((uint16_t)m >> 8);
        blob[0x40 + 2 * i + 1] = (uint8_t)m;
    }
    free(fr);
    *out = blob;
    return DT_OK;
}
