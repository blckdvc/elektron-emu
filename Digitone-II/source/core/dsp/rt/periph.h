/* COPIA di emu/sharc/rec/periph.h per l'interprete in C del plugin (docs/DSP-INTERPRETE.md):
 * stesso Periph (layout identico: gli stati DNDS sono intercambiabili), senza il thread dal
 * vivo e il profiler (non servono: dn_dsp_frame gira nel thread di chi chiama) e portabile su
 * Windows. Vedi rt.h per la regola sulle modifiche.
 *
 * Periferiche del DSP in C (porta 1:1 di emu/sharc/periph.py), per far girare il
 * motore ricompilato senza il Python: SEC0, i sei canali DMA delle SPORT e
 * dell'SPI2, CGU/DMC, i registri non modellati (letti come scritti) e la coda
 * degli eventi. Incluso solo nel file del dispatcher (RT_MAIN).
 *
 * I dati escono ed entrano da anelli (Rings) letti e riempiti da fuori:
 *   frame DSPI2 dal ColdFire -> DMA27 (SPI2 RX), parole a 16 bit scambiate;
 *   DMA26 (SPI2 TX)  -> ultima risposta del DSP al ColdFire;
 *   DMA0  (SPORT0A)  -> audio TDM16 a 32 bit (anello audio);
 *   DMA10 (SPORT4A)  -> contato e scartato; DMA1/DMA11 (RX) ricevono zeri.
 * Se un buffer dell'SPI2 RX deve completarsi e non c'e' un frame, rt_run si
 * ferma con 5 (serve un frame) senza consumare l'evento.
 */
#ifndef SHARC_PERIPH_H
#define SHARC_PERIPH_H

#define NDMA 6
#define SEC_BASE 0x31089000u
#define SEC_CCTL0 0x400u
#define SEC_END 0x0Cu
#define SECI_ID_ADDR (CMMR_BASE + 4 * (0x300EBu - 0x30000u))
enum { D_NXT = 0x00, D_START = 0x04, D_CFG = 0x08, D_XCNT = 0x0C, D_XMOD = 0x10, D_YCNT = 0x14,
       D_YMOD = 0x18, D_CUR = 0x24, D_STAT = 0x30 };
enum { DK_AUDIO = 0, DK_RX_ZERO = 1, DK_DISCARD = 2, DK_REPLY = 3, DK_FRAMES = 4 };

typedef struct {
    uint32_t base, sec_id, extra_sid, kind, sport;   /* sport: base SPORT per la cadenza, 0 = SPI2 */
    uint32_t regs[32];                                /* offset / 4, 0x00..0x7C */
    uint32_t stat;
    int64_t word_cycles;
    uint64_t buffers;
} Dma;

typedef struct { int64_t due; uint64_t seq; int ch; } Ev;

#define NEV 64
#define GEN_SIZE 4096                                 /* registri non modellati: indirizzo -> valore */
#define FRAME_BYTES 2748
#define FRAME_RING 64
#define AUDIO_RING (1 << 16)                          /* campioni stereo */
#define SPARSE_PAGES 64

typedef struct {
    Dma dma[NDMA];
    /* SEC0 */
    uint32_t sec_regs[1024];                          /* offset / 4 */
    uint8_t sec_pending[256];
    int32_t sec_active;                               /* -1 = nessuno */
    uint64_t sec_raised[256];
    /* CGU0/1, DMC0 */
    uint32_t cgu_regs[2][64]; int cgu_pllen[2], cgu_bypass[2];
    uint32_t dmc_regs[1024], dmc_phy[1024], cdu_regs[1024];
    /* registri e pagine non modellati (come la RAM di sfondo di Memory) */
    uint32_t gen_addr[GEN_SIZE], gen_val[GEN_SIZE]; uint8_t gen_used[GEN_SIZE];
    uint32_t sparse_base[SPARSE_PAGES]; uint8_t *sparse[SPARSE_PAGES]; int nsparse;
    /* eventi */
    Ev ev[NEV]; int nev; uint64_t evseq;
    /* anelli */
    uint8_t frames[FRAME_RING][FRAME_BYTES]; volatile uint32_t fr_head, fr_tail;
    uint8_t reply[FRAME_BYTES]; volatile uint32_t reply_seq;
    int32_t audio[AUDIO_RING][2]; volatile uint32_t au_head, au_tail;
    uint64_t audio_dropped, frames_used, sport4_blocks;
    uint32_t out_ch[2];                               /* slot TDM del canale sinistro e destro */
    int spi_lock;                                     /* SPI2 agganciata ai blocchi della SPORT4 (d14) */
    /* SPI2 (periph.py SPI): trasferimento solo con REN/TEN, RF/TF in STAT (W1C) */
    uint32_t spi_regs[64], spi_stat, spi_imsk;
    uint64_t spi_dropped;
    /* ultimo frame consumato, con gli impulsi nota azzerati: lo ripete il thread dal
       vivo quando il ColdFire e' in ritardo (FINDINGS d16) */
    uint8_t held[FRAME_BYTES]; int has_held; uint64_t held_used;
} Periph;
/* byte del frame (ordine del ColdFire) che valgono un blocco solo: note-on, note-off e i
   due flag che li accompagnano (+0x22..+0x29, FINDINGS d15) */
#define EV_LO 0x22
#define EV_HI 0x2A
#define SPI_BASE 0x31030000u
#define SPI_RXCTL 0x08u
#define SPI_TXCTL 0x0Cu
#define SPI_IMSK 0x30u
#define SPI_IMSK_CLR 0x34u
#define SPI_IMSK_SET 0x38u
#define SPI_STAT 0x40u
#define SPI_RF (1u << 10)
#define SPI_TF (1u << 11)
#define CH_SPORT4B 3
#define CH_SPI_TX 4
#define CH_SPI_RX 5

#define PER(st) ((Periph *)(st)->per)

/* ------------------------------------------------------------ pagine sparse */
static uint8_t *sparse_page(Periph *P, uint32_t b, int create) {
    uint32_t pb = b & ~0xFFFFu;
    for (int k = 0; k < P->nsparse; k++)
        if (P->sparse_base[k] == pb) return P->sparse[k];
    if (!create || P->nsparse >= SPARSE_PAGES) return 0;
    uint8_t *p = (uint8_t *)calloc(1, 0x10000);
    P->sparse_base[P->nsparse] = pb; P->sparse[P->nsparse] = p; P->nsparse++;
    return p;
}
static uint32_t gen_find(Periph *P, uint32_t a, int create) {
    uint32_t h = (a >> 2) * 2654435761u % GEN_SIZE;
    for (uint32_t i = 0; i < GEN_SIZE; i++) {
        uint32_t k = (h + i) % GEN_SIZE;
        if (P->gen_used[k] && P->gen_addr[k] == a) return k;
        if (!P->gen_used[k]) {
            if (!create) return GEN_SIZE;
            P->gen_used[k] = 1; P->gen_addr[k] = a; P->gen_val[k] = 0;
            return k;
        }
    }
    return GEN_SIZE;
}

/* memoria di sistema vista dai DMA (indirizzi a byte di sistema) */
static uint8_t *sys_ptr(St *st, uint32_t b, uint32_t n) {
    uint8_t *p = host(st, b, n);
    if (p) return p;
    uint8_t *pg = sparse_page(PER(st), b, 1);
    return pg ? pg + (b & 0xFFFF) : 0;
}
static uint32_t sys_ld32(St *st, uint32_t b) {
    uint8_t *p = sys_ptr(st, b, 4); uint32_t v = 0;
    if (p) memcpy(&v, p, 4);
    return v;
}

/* ------------------------------------------------------------------ eventi */
static void ev_update(St *st) {
    Periph *P = PER(st);
    int64_t m = -1;
    for (int i = 0; i < P->nev; i++) if (m < 0 || P->ev[i].due < m) m = P->ev[i].due;
    st->next_event = m;
}
static void add_event(St *st, int64_t due, int ch) {
    Periph *P = PER(st);
    if (P->nev >= NEV) { DN_ERR(st, 0xA0001); return; }
    P->ev[P->nev].due = due; P->ev[P->nev].seq = ++P->evseq; P->ev[P->nev].ch = ch; P->nev++;
    ev_update(st);
    st->deadline = st->cycles;
}

/* ----------------------------------------------------------------------- SEC */
static uint32_t sec_sctl(Periph *P, uint32_t sid) { return P->sec_regs[(0x800 + 8 * sid) / 4]; }
static int sec_ready_min(Periph *P) {
    for (int sid = 0; sid < 256; sid++)
        if (P->sec_pending[sid] && (sec_sctl(P, sid) & 1)) return sid;
    return -1;
}
static void sec_update(St *st) {
    Periph *P = PER(st);
    if (P->sec_active < 0 && (P->sec_regs[SEC_CCTL0 / 4] & 1) && sec_ready_min(P) >= 0) {
        st->irptl |= 1u << SECI;
        st->deadline = st->cycles;
    }
}
static void sec_raise(St *st, uint32_t sid) {
    Periph *P = PER(st);
    if (sid >= 256) return;
    P->sec_raised[sid]++;
    if (sec_sctl(P, sid) & 4) { P->sec_pending[sid] = 1; sec_update(st); }
}
static uint32_t sec_read_id(St *st) {
    Periph *P = PER(st);
    int r = sec_ready_min(P);
    if (P->sec_active < 0 && r >= 0) { P->sec_active = r; P->sec_pending[r] = 0; }
    return P->sec_active < 0 ? 0 : (uint32_t)P->sec_active;
}
static void sec_write(St *st, uint32_t off, uint32_t v) {
    Periph *P = PER(st);
    if (off / 4 < 1024) P->sec_regs[off / 4] = v;
    if (off == SEC_END) { P->sec_active = -1; sec_update(st); }
    else if (off == SEC_CCTL0 || off >= 0x800) sec_update(st);
}

/* ----------------------------------------------------------------------- DMA */
static int64_t sport_cycles(St *st, uint32_t base) {
    uint32_t mctl = sys_ld32(st, base + 0x08), words = 2;
    if (mctl & 1) {
        words = 0;
        for (uint32_t o = 0x0C; o <= 0x18; o += 4) words += (uint32_t)__builtin_popcount(sys_ld32(st, base + o));
        if (!words) words = 1;
    }
    int64_t c = 1000000000LL / (48000LL * words);
    return c > 0 ? c : 1;
}
static void dma_load_desc(St *st, Dma *d, uint32_t ptr) {
    static const uint32_t order[7] = {D_NXT, D_START, D_CFG, D_XCNT, D_XMOD, D_YCNT, D_YMOD};
    uint32_t cfg = d->regs[D_CFG / 4];
    uint32_t n = ((cfg >> 16) & 7) + 1;
    for (uint32_t k = 0; k < n && k < 7; k++) d->regs[order[k] / 4] = sys_ld32(st, ptr + 4 * k);
    d->regs[D_CUR / 4] = ptr;
}
static void dma_schedule(St *st, int ch) {
    Dma *d = &PER(st)->dma[ch];
    /* agganciata: i buffer dell'SPI2 si chiudono col blocco della SPORT4 (dma_done) */
    if (PER(st)->spi_lock && (ch == CH_SPI_TX || ch == CH_SPI_RX)) return;
    uint32_t n = d->regs[D_XCNT / 4];
    add_event(st, st->cycles + (int64_t)(n ? n : 1) * d->word_cycles, ch);
}
static void dma_start(St *st, int ch) {
    Dma *d = &PER(st)->dma[ch];
    if (d->sport) d->word_cycles = sport_cycles(st, d->sport);
    uint32_t cfg = d->regs[D_CFG / 4], flow = (cfg >> 12) & 7;
    if (flow >= 4) dma_load_desc(st, d, d->regs[D_NXT / 4]);
    d->stat |= 1u << 8;
    dma_schedule(st, ch);
}
/* ritorna 0 se il buffer non puo' completarsi ora (manca un frame) */
static int dma_done(St *st, int ch);
static int spi_enabled(Periph *P, int ch) { return (P->dma[ch].regs[D_CFG / 4] & 1) != 0; }

static int dma_done(St *st, int ch) {
    Periph *P = PER(st);
    Dma *d = &P->dma[ch];
    uint32_t cfg = d->regs[D_CFG / 4];
    if (!(cfg & 1)) return 1;
    /* il ColdFire manda un frame per blocco audio: prima il frame, poi il blocco */
    if (P->spi_lock && ch == CH_SPORT4B) {
        if (spi_enabled(P, CH_SPI_RX)) dma_done(st, CH_SPI_RX);
        if (spi_enabled(P, CH_SPI_TX)) dma_done(st, CH_SPI_TX);
    }
    uint32_t msize = 1u << ((cfg >> 8) & 7), xcnt = d->regs[D_XCNT / 4];
    int32_t xmod = (int32_t)d->regs[D_XMOD / 4];
    uint32_t start = d->regs[D_START / 4], nbytes = xcnt * msize;
    if (ch == CH_SPI_RX || ch == CH_SPI_TX) {
        uint32_t en = P->spi_regs[(ch == CH_SPI_RX ? SPI_RXCTL : SPI_TXCTL) / 4] & 1;
        if (!en) {                                    /* SPI2 spenta: il frame va perso */
            if (ch == CH_SPI_RX && P->fr_head != P->fr_tail) {
                __atomic_store_n(&P->fr_tail, P->fr_tail + 1, __ATOMIC_RELEASE);
                P->spi_dropped++;
            }
            dma_schedule(st, ch);
            return 1;
        }
    }
    if (cfg & 2) {                                    /* periferica -> memoria */
        const uint8_t *src = 0;
        uint8_t tmp[FRAME_BYTES];
        if (d->kind == DK_FRAMES) {
            if (P->fr_head == P->fr_tail) return 0;   /* serve un frame dal ColdFire */
            const uint8_t *fr = P->frames[P->fr_tail % FRAME_RING];
            uint32_t n = nbytes < FRAME_BYTES ? nbytes : FRAME_BYTES;
            for (uint32_t k = 0; k + 1 < n; k += 2) { tmp[k] = fr[k + 1]; tmp[k + 1] = fr[k]; }
            src = tmp;
            memcpy(P->held, fr, FRAME_BYTES);
            memset(P->held + EV_LO, 0, EV_HI - EV_LO);
            P->has_held = 1;
            __atomic_store_n(&P->fr_tail, P->fr_tail + 1, __ATOMIC_RELEASE);
            P->frames_used++;
        }
        for (uint32_t k = 0; k < xcnt; k++) {
            uint8_t *q = sys_ptr(st, start + (uint32_t)((int32_t)k * xmod), msize);
            if (!q) continue;
            if (src && (k + 1) * msize <= FRAME_BYTES) memcpy(q, src + k * msize, msize);
            else memset(q, 0, msize);
        }
    } else {                                          /* memoria -> periferica */
        if (d->kind == DK_AUDIO) {
            /* TDM: parole per frame dalla SPORT; tiene due slot (sinistro, destro) */
            uint32_t words = (uint32_t)(1000000000LL / (48000LL * (d->word_cycles ? d->word_cycles : 1)));
            if (words < 1) words = 1;
            for (uint32_t k = 0; k + words <= xcnt; k += words) {
                uint32_t nh = P->au_head;
                if (nh - P->au_tail >= AUDIO_RING) { P->audio_dropped++; continue; }
                for (int c = 0; c < 2; c++) {
                    uint32_t idx = k + P->out_ch[c];
                    uint8_t *q = sys_ptr(st, start + (uint32_t)((int32_t)idx * xmod), 4);
                    int32_t v = 0; if (q) memcpy(&v, q, 4);
                    P->audio[nh % AUDIO_RING][c] = v;
                }
                __atomic_store_n(&P->au_head, nh + 1, __ATOMIC_RELEASE);
            }
        } else if (d->kind == DK_REPLY) {
            uint32_t n = nbytes < FRAME_BYTES ? nbytes : FRAME_BYTES;
            for (uint32_t k = 0; k < xcnt && (k + 1) * msize <= n; k++) {
                uint8_t *q = sys_ptr(st, start + (uint32_t)((int32_t)k * xmod), msize);
                if (q) memcpy(P->reply + k * msize, q, msize);
            }
            __atomic_store_n(&P->reply_seq, P->reply_seq + 1, __ATOMIC_RELEASE);
        } else if (d->kind == DK_DISCARD) {
            P->sport4_blocks++;
        }
    }
    d->buffers++;
    if (ch == CH_SPI_RX) P->spi_stat |= SPI_RF;
    if (ch == CH_SPI_TX) P->spi_stat |= SPI_TF;
    if (((cfg >> 20) & 3) == 1) { d->stat |= 1; sec_raise(st, d->sec_id); }
    if (d->extra_sid) sec_raise(st, d->extra_sid);
    uint32_t flow = (cfg >> 12) & 7;
    if (flow == 0) { d->regs[D_CFG / 4] = cfg & ~1u; d->stat &= ~(7u << 8); return 1; }
    if (flow >= 4) dma_load_desc(st, d, d->regs[D_NXT / 4]);
    dma_schedule(st, ch);
    return 1;
}
static uint32_t dma_read(Dma *d, uint32_t off) { return off == D_STAT ? d->stat : (off / 4 < 32 ? d->regs[off / 4] : 0); }
static void dma_write(St *st, int ch, uint32_t off, uint32_t v) {
    Dma *d = &PER(st)->dma[ch];
    if (off == D_STAT) { d->stat &= ~(v & 7); return; }
    if (off / 4 < 32) d->regs[off / 4] = v;
    if (off == D_CFG) {
        if (v & 1) dma_start(st, ch);
        else d->stat &= ~(7u << 8);
    }
}

/* esegue gli eventi scaduti in ordine (ciclo, sequenza); 0 = manca un frame */
static int run_events(St *st) {
    Periph *P = PER(st);
    for (;;) {
        int best = -1;
        for (int i = 0; i < P->nev; i++)
            if (P->ev[i].due <= st->cycles &&
                (best < 0 || P->ev[i].due < P->ev[best].due ||
                 (P->ev[i].due == P->ev[best].due && P->ev[i].seq < P->ev[best].seq)))
                best = i;
        if (best < 0) break;
        Ev e = P->ev[best];
        int wants = P->spi_lock ? (e.ch == CH_SPORT4B && (P->dma[CH_SPORT4B].regs[D_CFG / 4] & 1)
                                   && (P->dma[CH_SPI_RX].regs[D_CFG / 4] & 3) == 3)
                                : (P->dma[e.ch].kind == DK_FRAMES && (P->dma[e.ch].regs[D_CFG / 4] & 3) == 3);
        if (wants && P->fr_head == P->fr_tail)
            return 0;                                 /* l'evento resta in coda finche' arriva un frame */
        P->ev[best] = P->ev[--P->nev];
        dma_done(st, e.ch);
    }
    ev_update(st);
    return 1;
}

/* ------------------------------------------------------------------- MMIO */
static uint32_t shift_read(uint32_t v, uint32_t a) { return v >> (8 * (a & 3)); }

uint32_t periph_read(St *st, uint32_t b, uint32_t n) {
    Periph *P = PER(st);
    uint32_t mask = n >= 4 ? M32 : ((1u << (8 * n)) - 1);
    for (int ch = 0; ch < NDMA; ch++) {
        Dma *d = &P->dma[ch];
        if (b - d->base < 0x80) return shift_read(dma_read(d, (b - d->base) & ~3u), b) & mask;
    }
    if (b - SEC_BASE < 0x1000) {
        uint32_t off = (b - SEC_BASE) & ~3u;
        return shift_read(P->sec_regs[off / 4], b) & mask;
    }
    if (b - SECI_ID_ADDR < 4) return sec_read_id(st) & mask;
    if (b - SPI_BASE < 0x100) {
        uint32_t off = (b - SPI_BASE) & ~3u;
        uint32_t v = off == SPI_STAT ? P->spi_stat : (off == SPI_IMSK ? P->spi_imsk : P->spi_regs[off / 4]);
        return shift_read(v, b) & mask;
    }
    for (int k = 0; k < 2; k++) {
        uint32_t base = k ? 0x3108E000u : 0x3108D000u;
        if (b - base < 0x1000) {
            uint32_t off = (b - base) & ~3u;
            uint32_t v = off == 0x08 ? (uint32_t)(P->cgu_pllen[k] | (P->cgu_bypass[k] << 1) | (P->cgu_pllen[k] << 2))
                                     : (off / 4 < 64 ? P->cgu_regs[k][off / 4] : 0);
            return shift_read(v, b) & mask;
        }
    }
    if (b - 0x31070000u < 0x1000) {
        uint32_t off = (b - 0x31070000u) & ~3u;
        uint32_t v = off == 0x08 ? ((1u << 0) | (1u << 2) | (1u << 13)) : P->dmc_regs[off / 4];
        return shift_read(v, b) & mask;
    }
    if (b - 0x31071000u < 0x1000) return shift_read(P->dmc_phy[((b - 0x31071000u) & ~3u) / 4], b) & mask;
    if (b - 0x3108F000u < 0x1000) {
        uint32_t off = (b - 0x3108F000u) & ~3u;
        return shift_read(off == 0x40 ? 0 : P->cdu_regs[off / 4], b) & mask;
    }
    /* non modellato: la RAM di sfondo */
    uint8_t *pg = sparse_page(P, b, 0);
    if (!pg) return 0;
    uint32_t v = 0; memcpy(&v, pg + (b & 0xFFFF), n > 4 ? 4 : n);
    return v & mask;
}

void periph_write(St *st, uint32_t b, uint32_t n, uint32_t v) {
    Periph *P = PER(st);
    for (int ch = 0; ch < NDMA; ch++) {
        Dma *d = &P->dma[ch];
        if (b - d->base < 0x80) { dma_write(st, ch, (b - d->base) & ~3u, v); return; }
    }
    if (b - SEC_BASE < 0x1000) { sec_write(st, (b - SEC_BASE) & ~3u, v); return; }
    if (b - SECI_ID_ADDR < 4) return;
    if (b - SPI_BASE < 0x100) {
        uint32_t off = (b - SPI_BASE) & ~3u;
        if (off == SPI_STAT) P->spi_stat &= ~v;
        else if (off == SPI_IMSK_CLR) P->spi_imsk &= ~v;
        else if (off == SPI_IMSK_SET) P->spi_imsk |= v;
        else P->spi_regs[off / 4] = v;
        return;
    }
    for (int k = 0; k < 2; k++) {
        uint32_t base = k ? 0x3108E000u : 0x3108D000u;
        if (b - base < 0x1000) {
            uint32_t off = (b - base) & ~3u;
            if (off == 0x04) {
                if (v & 1) P->cgu_bypass[k] = 1;
                if (v & 2) P->cgu_bypass[k] = 0;
                if (v & 4) P->cgu_pllen[k] = 0;
                if (v & 8) P->cgu_pllen[k] = 1;
            }
            if (off / 4 < 64) P->cgu_regs[k][off / 4] = v;
            return;
        }
    }
    if (b - 0x31070000u < 0x1000) {
        uint32_t off = (b - 0x31070000u) & ~3u, clr = 0;
        if (off == 0x04) clr = 1u << 26;
        else if (off == 0x64) clr = 1u << 7;
        else if (off == 0x6C) clr = 1u << 2;
        P->dmc_regs[off / 4] = v & ~clr;
        return;
    }
    if (b - 0x31071000u < 0x1000) { P->dmc_phy[((b - 0x31071000u) & ~3u) / 4] = v; return; }
    if (b - 0x3108F000u < 0x1000) { P->cdu_regs[((b - 0x3108F000u) & ~3u) / 4] = v; return; }
    uint8_t *pg = sparse_page(P, b, 1);
    if (pg) memcpy(pg + (b & 0xFFFF), &v, n > 4 ? 4 : n);
}

/* PORT: il thread dal vivo del Python e il profiler a campionamento non ci sono: la St e'
   guidata da un solo thread (dn_dsp_frame) e rt_push_frame/rt_pull_audio non prendono lock. */
int rt_run(St *st, int64_t limit);

/* --------------------------------------------------------------- interfaccia */
void *rt_periph_new(void) { return calloc(1, sizeof(Periph)); }
long rt_periph_size(void) { return (long)sizeof(Periph); }
/* configura un canale: indice, base, ID SEC, ID extra (0), tipo, base SPORT (0 = SPI2), cicli per parola */
void rt_dma_config(St *st, int ch, uint32_t base, uint32_t sid, uint32_t extra, uint32_t kind, uint32_t sport, int64_t wc) {
    Dma *d = &PER(st)->dma[ch];
    d->base = base; d->sec_id = sid; d->extra_sid = extra; d->kind = kind; d->sport = sport; d->word_cycles = wc;
}
void rt_dma_set(St *st, int ch, uint32_t off, uint32_t v) {
    Dma *d = &PER(st)->dma[ch];
    if (off == D_STAT) d->stat = v; else if (off / 4 < 32) d->regs[off / 4] = v;
}
void rt_event_add(St *st, int64_t due, int ch) { add_event(st, due, ch); }
void rt_sec_set(St *st, uint32_t off, uint32_t v) { if (off / 4 < 1024) PER(st)->sec_regs[off / 4] = v; }
void rt_sec_state(St *st, int32_t active, const uint8_t *pending) {
    PER(st)->sec_active = active;
    memcpy(PER(st)->sec_pending, pending, 256);
}
void rt_block_set(St *st, uint32_t which, uint32_t off, uint32_t v) {
    Periph *P = PER(st);
    switch (which) {
    case 0: case 1: if (off / 4 < 64) P->cgu_regs[which][off / 4] = v; break;
    case 2: P->dmc_regs[off / 4 % 1024] = v; break;
    case 3: P->dmc_phy[off / 4 % 1024] = v; break;
    case 4: P->cdu_regs[off / 4 % 1024] = v; break;
    }
}
void rt_cgu_state(St *st, int k, int pllen, int bypass) { PER(st)->cgu_pllen[k] = pllen; PER(st)->cgu_bypass[k] = bypass; }
void rt_spi_state(St *st, uint32_t off, uint32_t v) {
    Periph *P = PER(st);
    if (off == SPI_STAT) P->spi_stat = v;
    else if (off == SPI_IMSK) P->spi_imsk = v;
    else if (off / 4 < 64) P->spi_regs[off / 4] = v;
}
void rt_sparse_load(St *st, uint32_t page_base, const uint8_t *data) {
    uint8_t *p = sparse_page(PER(st), page_base, 1);
    if (p) memcpy(p, data, 0x10000);
}
void rt_out_channels(St *st, uint32_t l, uint32_t r) { PER(st)->out_ch[0] = l; PER(st)->out_ch[1] = r; }
/* agganciare l'SPI2 ai blocchi della SPORT4 toglie gli eventi a tempo dei suoi canali */
void rt_spi_lock(St *st, int on) {
    Periph *P = PER(st);
    P->spi_lock = on;
    if (!on) return;
    int k = 0;
    for (int i = 0; i < P->nev; i++)
        if (P->ev[i].ch != CH_SPI_TX && P->ev[i].ch != CH_SPI_RX) P->ev[k++] = P->ev[i];
    P->nev = k;
    ev_update(st);
}
/* un frame DSPI2 come lo manda il ColdFire (big endian); 0 se l'anello e' pieno */
int rt_push_frame(St *st, const uint8_t *fr, uint32_t n) {
    Periph *P = PER(st);
    if (P->fr_head - P->fr_tail >= FRAME_RING) return 0;
    uint8_t *dst = P->frames[P->fr_head % FRAME_RING];
    memset(dst, 0, FRAME_BYTES);
    memcpy(dst, fr, n < FRAME_BYTES ? n : FRAME_BYTES);
    __atomic_store_n(&P->fr_head, P->fr_head + 1, __ATOMIC_RELEASE);
    return 1;
}
int rt_frames_queued(St *st) { return (int)(PER(st)->fr_head - PER(st)->fr_tail); }
/* fino a `max` campioni stereo int32 -> out[2*i], out[2*i+1]; ritorna quanti */
int rt_pull_audio(St *st, int32_t *out, int max) {
    Periph *P = PER(st);
    int n = 0;
    uint32_t head = __atomic_load_n(&P->au_head, __ATOMIC_ACQUIRE);
    while (n < max && P->au_tail != head) {
        out[2 * n] = P->audio[P->au_tail % AUDIO_RING][0];
        out[2 * n + 1] = P->audio[P->au_tail % AUDIO_RING][1];
        P->au_tail++; n++;
    }
    return n;
}
uint32_t rt_reply(St *st, uint8_t *out, uint32_t n) {
    memcpy(out, PER(st)->reply, n < FRAME_BYTES ? n : FRAME_BYTES);
    return PER(st)->reply_seq;
}
uint64_t rt_stat(St *st, int k) {
    Periph *P = PER(st);
    switch (k) {
    case 0: return P->frames_used;
    case 1: return P->audio_dropped;
    case 2: return P->sport4_blocks;
    case 3: return P->dma[0].buffers;
    case 4: return (uint64_t)P->nev;
    case 5: return P->spi_dropped;
    }
    return 0;
}

#endif
