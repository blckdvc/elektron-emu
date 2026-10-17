
#include <stdint.h>
#include <string.h>

typedef int (*mem_fn)(void *uc, uint64_t address, void *bytes, uint64_t size);

typedef struct {
    mem_fn read;
    mem_fn write;
    uint32_t pending_mask_lo, pending_mask_hi;   /* channels with a CSR to publish */
    uint16_t final_csr[64];
    int64_t transfers, bytes, skipped, irqs;
} dma_state;

#define TCD_BASE 0xFC045000u
#define START 0x0001u
#define INTMAJOR 0x0002u
#define ESG 0x0010u
#define MAJORELINK 0x0020u
#define ACTIVE 0x0040u
#define DONE 0x0080u
#define MAX_CHAIN 4096

static int is_memory(uint32_t a)
{
    return (a >= 0x40000000u && a < 0x50000000u) || (a >= 0x80000000u && a < 0x80010000u);
}

static uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = (uint8_t)v; }

/* a buffer per thread in the engine in C (several plugin instances); empty for the Python */
#ifndef DT2_TLS
#define DT2_TLS
#endif
static DT2_TLS uint8_t big[1 << 16];

/* -> final CSR (>= 0), or -1 when the transfer is not modelled */
static int run(dma_state *s, void *uc, int ch, int csr_written, int depth)
{
    uint32_t tcd = TCD_BASE + (uint32_t)ch * 0x20u;
    uint8_t regs[32];
    int step;
    s->read(uc, tcd, regs, 32);
    if (csr_written >= 0)
        put16(regs + 0x1E, (uint16_t)csr_written);
    for (step = 0; step < MAX_CHAIN; step++) {
        uint32_t src = be32(regs + 0), nbytes = be32(regs + 8), dst = be32(regs + 0x10);
        uint16_t attr = be16(regs + 4), citer_raw = be16(regs + 0x14), biter_raw = be16(regs + 0x1C);
        uint16_t csr = be16(regs + 0x1E);
        int16_t soff = (int16_t)be16(regs + 6), doff = (int16_t)be16(regs + 0x16);
        int32_t slast = (int32_t)be32(regs + 0x0C), dlast = (int32_t)be32(regs + 0x18);
        uint32_t citer, ssize, dsize, smod, dmod, smask, dmask, sbase, dbase, total, it;
        int32_t mloff = 0;
        int smloe = 0, dmloe = 0, elink;
        if (!is_memory(dst) || !(is_memory(src) || src >= 0xFFFF0000u)) {
            s->skipped++;
            return -1;
        }
        elink = (citer_raw & 0x8000) != 0;
        citer = citer_raw & (elink ? 0x01FFu : 0x7FFFu);
        if (elink && ((citer_raw >> 9) & 0x3F) != (unsigned)ch) {
            s->skipped++;
            return -1;
        }
        if (!citer) citer = 1;
        ssize = 1u << ((attr >> 8) & 7);
        dsize = 1u << (attr & 7);
        smod = (attr >> 11) & 0x1F;
        dmod = (attr >> 3) & 0x1F;
        smask = smod ? ((1u << smod) - 1) : 0xFFFFFFFFu;
        dmask = dmod ? ((1u << dmod) - 1) : 0xFFFFFFFFu;
        sbase = src & ~smask;
        dbase = dst & ~dmask;
        if (nbytes & 0xC0000000u) {
            uint32_t m = (nbytes >> 10) & 0xFFFFFu;
            mloff = (m & 0x80000u) ? (int32_t)m - (1 << 20) : (int32_t)m;
            smloe = (nbytes & 0x80000000u) != 0;
            dmloe = (nbytes & 0x40000000u) != 0;
            nbytes &= 0x3FFu;
        }
        if (!nbytes || nbytes % ssize || nbytes % dsize) {
            s->skipped++;
            return -1;
        }
        total = citer * nbytes;
        if ((int32_t)soff == (int32_t)ssize && (int32_t)doff == (int32_t)dsize && !smloe && !dmloe
            && (uint64_t)(src & smask) + total <= (uint64_t)smask + 1
            && (uint64_t)(dst & dmask) + total <= (uint64_t)dmask + 1
            && is_memory(src) && is_memory(src + total - 1) && total <= sizeof(big)) {
            s->read(uc, src, big, total);
            s->write(uc, dst, big, total);
            src = sbase | ((src + total) & smask);
            dst = dbase | ((dst + total) & dmask);
        } else {
            for (it = 0; it < citer; it++) {
                uint8_t data[1024];
                uint32_t k;
                for (k = 0; k < nbytes; k += ssize) {
                    if (is_memory(src)) s->read(uc, src, data + k, ssize);
                    else memset(data + k, 0, ssize);
                    src = sbase | ((src + (uint32_t)(int32_t)soff) & smask);
                }
                for (k = 0; k < nbytes; k += dsize) {
                    s->write(uc, dst, data + k, dsize);
                    dst = dbase | ((dst + (uint32_t)(int32_t)doff) & dmask);
                }
                if (smloe) src = sbase | ((src + (uint32_t)mloff) & smask);
                if (dmloe) dst = dbase | ((dst + (uint32_t)mloff) & dmask);
            }
        }
        src = sbase | ((src + (uint32_t)slast) & smask);
        s->transfers++;
        s->bytes += total;
        if (csr & INTMAJOR) s->irqs++;
        if ((csr & MAJORELINK) && depth < 8)
            run(s, uc, (csr >> 8) & 0x3F, -1, depth + 1);
        if (csr & ESG) {
            uint16_t ncsr;
            s->read(uc, (uint32_t)dlast, regs, 32);
            ncsr = be16(regs + 0x1E);
            if (ncsr & START) continue;
            s->write(uc, tcd, regs, 32);
            return ncsr;
        }
        dst = dbase | ((dst + (uint32_t)dlast) & dmask);
        {
            uint16_t final = (uint16_t)((csr & ~(START | ACTIVE)) | DONE);
            put32(regs + 0, src);
            put32(regs + 0x10, dst);
            put16(regs + 0x14, biter_raw);
            put16(regs + 0x1E, final);
            s->write(uc, tcd, regs, 32);
            return final;
        }
    }
    return -1;
}

void dt2_dma_on_csr(void *uc, int type, uint64_t address, int size, int64_t value, void *user)
{
    dma_state *s = (dma_state *)user;
    uint32_t rel = (uint32_t)address - TCD_BASE;
    int ch = (int)(rel / 0x20), off = (int)(rel % 0x20), csr;
    (void)type;
    if (off == 0x1E && size >= 2) csr = (int)(value & 0xFFFF);
    else if (off == 0x1C && size == 4) csr = (int)(value & 0xFFFF);
    else if (off == 0x1F && size == 1) csr = (int)(value & 0xFF);
    else return;
    if (csr & START) {
        int final = run(s, uc, ch, csr, 0);
        if (final >= 0) {
            s->final_csr[ch] = (uint16_t)final;
            if (ch < 32) s->pending_mask_lo |= 1u << ch;
            else s->pending_mask_hi |= 1u << (ch - 32);
        }
    }
}

void dt2_dma_on_read(void *uc, int type, uint64_t address, int size, int64_t value, void *user)
{
    dma_state *s = (dma_state *)user;
    int ch;
    (void)type; (void)address; (void)size; (void)value;
    if (!(s->pending_mask_lo | s->pending_mask_hi))
        return;
    for (ch = 0; ch < 64; ch++) {
        uint32_t bit = ch < 32 ? (s->pending_mask_lo >> ch) & 1u : (s->pending_mask_hi >> (ch - 32)) & 1u;
        if (bit) {
            uint8_t b[2];
            put16(b, s->final_csr[ch]);
            s->write(uc, TCD_BASE + (uint32_t)ch * 0x20u + 0x1E, b, 2);
        }
    }
    s->pending_mask_lo = s->pending_mask_hi = 0;
}
