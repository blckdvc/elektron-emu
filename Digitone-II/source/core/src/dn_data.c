/* Il +Drive del Digitone II visto da un computer: l'API Data del firmware (emu/transfer.py
 * DataClient, parse_data_list), in C per il pannello +DRIVE del plugin. Le richieste MIDI
 * RPC vanno agli handler veri del firmware emulato (dt_rt_rpc di rt.c). */
#include "dncore.h"
#include "internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DATA_LIST 0x53
#define DATA_READ_OPEN 0x54
#define DATA_READ_PARTIAL 0x55
#define DATA_READ_CLOSE 0x56
#define DATA_CLEAR 0x5C
#define DATA_RENAME 0x5E
#define DATA_TIMEOUT 40000

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

/* Client._ok: la risposta comincia con 1 */
static int call_ok(dt_rt *rt, uint8_t rid, const uint8_t *body, size_t n, uint8_t **rep, size_t *rn,
                   char *err, size_t errlen, const char *what)
{
    uint8_t id = 0;
    if (dt_rt_rpc(rt, rid, body, n, &id, rep, rn, DATA_TIMEOUT)) {
        dt_errf(err, errlen, "%s: il firmware non risponde (RPC 0x%02x)", what, rid);
        return DT_ERR_IO;
    }
    if (!*rn || (*rep)[0] != 1) {
        dt_errf(err, errlen, "%s: il firmware risponde %02x %02x", what, id, *rn ? (*rep)[0] : 0);
        free(*rep);
        *rep = NULL;
        return DT_ERR_IO;
    }
    return DT_OK;
}

static uint8_t *cstr2(const char *a, const char *b, size_t *n)
{
    size_t la = strlen(a) + 1, lb = b ? strlen(b) + 1 : 0;
    uint8_t *p = (uint8_t *)malloc(la + lb + 8);
    if (!p)
        return NULL;
    memcpy(p, a, la);
    if (b)
        memcpy(p + la, b, lb);
    *n = la + lb;
    return p;
}

/* data_ls + parse_data_list */
int dn_data_ls(dt_rt *rt, const char *path, dn_data_entry **out, size_t *count, char *err, size_t errlen)
{
    uint8_t *rep = NULL;
    size_t rn = 0, i = 13, n = 0, cap = 0;
    dn_data_entry *v = NULL;
    int r = call_ok(rt, DATA_LIST, (const uint8_t *)path, strlen(path) + 1, &rep, &rn, err, errlen, path);
    *out = NULL;
    *count = 0;
    if (r)
        return r;
    while (i < rn) {
        size_t j = i;
        dn_data_entry *e;
        while (j < rn && rep[j])
            j++;
        if (j >= rn)
            break;
        if (n == cap) {
            dn_data_entry *nv;
            cap = cap ? 2 * cap : 64;
            nv = (dn_data_entry *)realloc(v, cap * sizeof *v);
            if (!nv) {
                free(v);
                free(rep);
                return DT_ERR_NOMEM;
            }
            v = nv;
        }
        e = &v[n];
        memset(e, 0, sizeof *e);
        snprintf(e->name, sizeof e->name, "%.*s", (int)(j - i), (const char *)rep + i);
        i = j + 1;
        if (i + 2 > rn)
            break;
        e->children = rep[i] != 0;
        e->type = rep[i + 1];
        i += 2;
        if (e->type == 1) {
            if (i + 4 > rn)
                break;
            e->count = get32(rep + i);
            i += 4;
        } else {
            if (i + 12 > rn)
                break;
            e->index = get32(rep + i);
            e->size = get32(rep + i + 4);
            e->ops = (uint16_t)(rep[i + 8] << 8 | rep[i + 9]);
            e->valid = rep[i + 10];
            e->meta = rep[i + 11];
            i += 12;
        }
        n++;
    }
    free(rep);
    *out = v;
    *count = n;
    return DT_OK;
}

/* data_read: Open (blocco 0x800), Partial dalla sequenza 1, Close; gli ultimi `size` byte */
int dn_data_read(dt_rt *rt, const char *path, uint8_t **out, size_t *len, char *err, size_t errlen)
{
    const uint32_t chunk = 0x800;
    uint8_t *rep = NULL, *body, buf[8], *data = NULL;
    size_t rn = 0, bn, dn = 0, dcap = 0;
    uint32_t jid, seq = 1, size;
    int r;
    *out = NULL;
    *len = 0;
    bn = strlen(path) + 1;
    body = (uint8_t *)malloc(bn + 5);
    if (!body)
        return DT_ERR_NOMEM;
    memcpy(body, path, bn);
    put32(body + bn, chunk);
    body[bn + 4] = 1;
    r = call_ok(rt, DATA_READ_OPEN, body, bn + 5, &rep, &rn, err, errlen, "apri");
    free(body);
    if (r)
        return r;
    if (rn < 5) {
        free(rep);
        dt_errf(err, errlen, "apri %s: risposta corta", path);
        return DT_ERR_IO;
    }
    jid = get32(rep + 1);
    free(rep);
    for (;;) {
        uint8_t id = 0;
        put32(buf, jid);
        put32(buf + 4, seq);
        rep = NULL;
        rn = 0;
        if (dt_rt_rpc(rt, DATA_READ_PARTIAL, buf, 8, &id, &rep, &rn, DATA_TIMEOUT))
            break;
        if (!rn || rep[0] != 1 || rn <= 13) {
            free(rep);
            break;
        }
        if (dn + rn - 13 > dcap) {
            uint8_t *nd;
            dcap = (dn + rn - 13) * 2;
            nd = (uint8_t *)realloc(data, dcap);
            if (!nd) {
                free(rep);
                free(data);
                return DT_ERR_NOMEM;
            }
            data = nd;
        }
        memcpy(data + dn, rep + 13, rn - 13);
        dn += rn - 13;
        free(rep);
        if (rn - 13 < chunk)
            break;
        seq++;
    }
    put32(buf, jid);
    rep = NULL;
    r = call_ok(rt, DATA_READ_CLOSE, buf, 4, &rep, &rn, err, errlen, "chiudi");
    if (r) {
        free(data);
        return r;
    }
    size = rn >= 9 ? get32(rep + 5) : (uint32_t)dn;
    free(rep);
    if (size <= dn && size) {
        memmove(data, data + dn - size, size);
        dn = size;
    }
    *out = data;
    *len = dn;
    return DT_OK;
}

int dn_data_clear(dt_rt *rt, const char *path, char *err, size_t errlen)
{
    uint8_t *rep = NULL;
    size_t rn = 0;
    int r = call_ok(rt, DATA_CLEAR, (const uint8_t *)path, strlen(path) + 1, &rep, &rn, err, errlen, "svuota");
    free(rep);
    return r;
}

int dn_data_rename(dt_rt *rt, const char *path, const char *name, char *err, size_t errlen)
{
    uint8_t *rep = NULL, *body;
    size_t rn = 0, bn = 0;
    int r;
    body = cstr2(path, name, &bn);
    if (!body)
        return DT_ERR_NOMEM;
    r = call_ok(rt, DATA_RENAME, body, bn, &rep, &rn, err, errlen, "rinomina");
    free(body);
    free(rep);
    return r;
}
