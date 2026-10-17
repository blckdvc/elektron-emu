
#include <stdint.h>
typedef int (*stop_fn)(void *uc);
typedef struct {
    int64_t left;
    int64_t blocks;
    stop_fn stop;
} step_state;

/* Stessa logica di longrun._FastStepper._on_block. */
void dt2_on_block(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    step_state *s = (step_state *)user_data;
    (void)address; (void)size;
    if (s->left <= 0) {
        s->stop(uc);
        return;
    }
    s->blocks += 1;
    s->left -= 1;
}

/* Stessa logica di dsp.Fifo on_read (fast_writes): a OGNI lettura di STATUS
   riscrive READY (2 byte) prima che la lettura avvenga. */
typedef int (*memw_fn)(void *uc, uint64_t address, const void *bytes, uint64_t size);
typedef struct {
    memw_fn write;
    uint64_t address;
    uint64_t size;
    uint8_t bytes[8];
} memw_state;

void dt2_mem_write_const(void *uc, int type, uint64_t address, int size,
                         int64_t value, void *user_data)
{
    memw_state *s = (memw_state *)user_data;
    (void)type; (void)address; (void)size; (void)value;
    s->write(uc, s->address, s->bytes, s->size);
}

/* Stessa logica di longrun.build:do_halt, l'hook degli idle spin (`bra *`).
   Python faceva, a OGNI passaggio: n += 1 ; tx.deliver() ; ogni `yield_every`
   passaggi raise_vector(32). deliver() esce subito se non c'e' nulla in coda,
   quindi il caso caldo e' solo il contatore: qui resta in C, e Python viene
   chiamato SOLO quando serve -- completamento UART in coda, oppure passaggio
   multiplo di yield_every -- con il valore di n, per fare esattamente
   quello che faceva prima, nello stesso istante. */
typedef void (*idle_slow_fn)(int64_t n);
typedef struct {
    int64_t n;
    int64_t yield_every;
    const int32_t *tx_pending;      /* NULL se non c'e' un canale TX */
    idle_slow_fn slow;
    step_state *step;               /* lo stepper nativo, o NULL */
    int64_t skipped;                /* giri avanzati senza eseguirli (diagnostica) */
    void *exc;                      /* exc_state: il yield (vettore 32) in C, o NULL */
    int64_t c_yields;               /* yield consegnati dal C (diagnostica) */
} idle_state;

void dt2_on_idle(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    idle_state *s = (idle_state *)user_data;
    int64_t to_yield, j;
    (void)uc; (void)address; (void)size;
    s->n += 1;
    if ((s->tx_pending && *s->tx_pending > 0) || (s->n % s->yield_every) == 0) {
        s->slow(s->n);
        return;                      /* un evento: niente salto su questo giro */
    }
    /* SALTO DELL'ATTESA. Un idle spin e' `bra *`: un giro non cambia NULLA
       dello stato della macchina. Dentro un chunk, i soli eventi possibili
       sono (a) lo stop dello stepper quando `left` arriva a 0, (b) il
       prossimo multiplo di yield_every (raise_vector 32), (c) un
       completamento UART in coda -- (c) e' escluso qui sopra e nessuno puo'
       accodarne uno mentre la CPU gira su se stessa. Quindi si avanzano n,
       left e blocks di tutti i giri fino all'ULTIMO prima dell'evento, e
       quell'ultimo lo si esegue davvero: stessi contatori, stesso istante
       dell'evento, stesso numero di istruzioni contate, dell'esecuzione giro
       per giro. */
    if (s->step == 0 || s->step->left <= 1)
        return;
    to_yield = s->yield_every - (s->n % s->yield_every);   /* >= 1 */
    j = (s->step->left < to_yield ? s->step->left : to_yield) - 1;
    if (j > 0) {
        s->n += j;
        s->step->left -= j;
        s->step->blocks += j;
        s->skipped += j;
    }
}

/* ---- PIT e DTIM in C (dal Digitone, digitone/dnkit FINDINGS d22) ---- */
#include <string.h>
#include <math.h>
typedef int (*mem_rw_fn)(void *uc, uint64_t address, void *bytes, uint64_t size);
static inline uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline uint16_t rbe16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
/* ---- PIT e DTIM (emu/pit.py Pits, emu/dtim.py Dtims) in C, d22 ----
   Il caso comune di step/service: lettura dei registri, armo delle scadenze, passo. Stessa
   aritmetica del Python (double: int / int e float * int sono esatti come in Python per
   valori < 2^53). Se un canale e' scaduto o ha un interrupt pendente, service risponde 1
   senza toccare nulla e lo fa il Python. */
typedef struct {
    mem_rw_fn mem_read;
    int32_t kind;                   /* 0 PIT, 1 DTIM */
    int32_t nch, held, level_irq, err;
    int32_t ch[4];
    uint32_t base[4];
    uint8_t *store[4];              /* registri del canale nello store MMIO, o NULL = RAM */
    double ips;
    double next[4];
    int32_t has[4], pend[4];
    /* emu/nativespin.py: the whole service in C (dt2_tim_service_full) */
    int64_t fired[4], missed[4], merged[4];
    struct irq_ctx *q;
    int32_t vec0, pad2;
} tim_state;

static int tim_read(void *uc, tim_state *s, int c, uint32_t off, uint8_t *b, int n)
{
    if (s->store[c]) { memcpy(b, s->store[c] + off, (size_t)n); return 0; }
    if (s->mem_read(uc, s->base[c] + off, b, (uint64_t)n)) { s->err = 1; return -1; }
    return 0;
}

/* -> 1 con *p, 0 se il canale non puo' scattare, -1 errore */
static int tim_period(void *uc, tim_state *s, int c, double *p)
{
    uint8_t b[8];
    if (s->kind == 0) {
        uint16_t pcsr, pmr;
        uint64_t prescale;
        if (tim_read(uc, s, c, 0, b, 4)) return -1;
        pcsr = rbe16(b); pmr = rbe16(b + 2);
        if (!(pcsr & 0x01) || !(pcsr & 0x08)) return 0;
        prescale = 1ull << (((pcsr >> 8) & 0xF) + 1);
        *p = (double)(prescale * ((uint64_t)pmr + 1)) / 132000000.0 * s->ips;
        return 1;
    } else {
        uint16_t dtmr; uint8_t dtxmr; uint32_t dtrr, clk, div;
        uint64_t ticks;
        if (tim_read(uc, s, c, 0, b, 8)) return -1;
        dtmr = rbe16(b); dtxmr = b[2]; dtrr = be32(b + 4);
        if (!(dtmr & 0x01) || !(dtmr & 0x10) || (dtxmr & 0x80)) return 0;
        clk = (dtmr >> 1) & 3;
        if (clk == 0 || clk == 3) return 0;
        div = clk == 1 ? 1 : 16;
        ticks = ((uint64_t)dtrr + 1) * (((dtmr >> 8) & 0xFF) + 1) * div;
        *p = (double)ticks / 132000000.0 * s->ips;
        return 1;
    }
}

int64_t dt2_tim_step(void *uc, tim_state *s, int64_t done, int64_t remaining)
{
    double best = 0, p;
    int have = 0, i, c, r;
    int64_t n;
    if (!s->held) {
        for (i = 0; i < s->nch; i++) {
            c = s->ch[i];
            r = tim_period(uc, s, c, &p);
            if (r < 0) return -1;
            if (r == 0) { s->has[c] = 0; continue; }
            if (!s->has[c]) { s->next[c] = (double)done + p; s->has[c] = 1; }
            if (!have || s->next[c] < best) { best = s->next[c]; have = 1; }
        }
    }
    if (!have) n = 1000000;
    else { double d = ceil(best - (double)done); n = (int64_t)d; if (n < 1) n = 1; }
    if (!have && remaining >= 0) n = remaining;
    if (remaining >= 0 && remaining < n) n = remaining;
    if (s->level_irq && (s->pend[0] | s->pend[1] | s->pend[2] | s->pend[3]) && n > 2000) n = 2000;
    return n < 1 ? 1 : n;
}

/* -> 0 fatto, 1 serve il Python (nulla e' stato toccato), -1 errore */
int dt2_tim_service(void *uc, tim_state *s, int64_t done)
{
    double p[4];
    int r[4], i, c;
    if (s->held) return 0;
    for (i = 0; i < s->nch; i++) {
        c = s->ch[i];
        r[i] = tim_period(uc, s, c, &p[i]);
        if (r[i] < 0) return -1;
        if (r[i] == 0 || !s->has[c]) continue;
        if ((double)done < s->next[c]) {
            if (s->level_irq && s->pend[c]) return 1;
            continue;
        }
        return 1;
    }
    for (i = 0; i < s->nch; i++) {
        c = s->ch[i];
        if (r[i] == 0) s->has[c] = 0;
        else if (!s->has[c]) { s->next[c] = (double)done + p[i]; s->has[c] = 1; }
    }
    return 0;
}

/* ---- Eccezioni: Machine.raise_vector (senza srtrap) e l'RTE di
   Machine.install_exceptions, in C. Stessa sequenza del Python; -1 = il
   Python lo rifa' (es. pagina dello stack non mappata: la mappa lui). ---- */
typedef int (*reg_rw_fn)(void *uc, int regid, void *value);
typedef void (*intr_py_fn)(void *uc, uint32_t intno);
typedef struct {
    reg_rw_fn reg_read, reg_write;
    mem_rw_fn mem_read, mem_write;
    uint32_t vbr;
    int32_t r_a7, r_sr, r_pc;
    intr_py_fn fallback;
    int64_t raised, rte, fallbacks;
} exc_state;

int dt2_raise_vector(void *uc, exc_state *s, int vec, int level, int from_instr)
{
    uint8_t b[8];
    uint32_t handler, pc = 0, sr = 0, sp = 0, live;
    uint16_t fmt;
    if (s->mem_read(uc, s->vbr + (uint32_t)vec * 4u, b, 4)) return -1;
    handler = be32(b);
    if (handler == 0 || handler >= 0x48000000u) return 0;
    if (s->reg_read(uc, s->r_pc, &pc)) return -1;
    if (from_instr && !s->mem_read(uc, pc, b, 2)) {
        uint16_t w = rbe16(b);
        if (w >= 0x4E40 && w <= 0x4E4F) pc += 2;          /* trap #N: riprende dopo */
    }
    if (s->reg_read(uc, s->r_sr, &sr) || s->reg_read(uc, s->r_a7, &sp)) return -1;
    sp -= 8;
    fmt = (uint16_t)(0x4000 | ((vec << 2) & 0x0FFC));
    b[0] = fmt >> 8; b[1] = fmt; b[2] = (uint8_t)(sr >> 8); b[3] = (uint8_t)sr;
    b[4] = pc >> 24; b[5] = pc >> 16; b[6] = pc >> 8; b[7] = pc;
    if (s->mem_write(uc, sp, b, 8)) return -1;
    if (s->reg_write(uc, s->r_a7, &sp)) return -1;
    live = sr | 0x2000;
    if (level >= 0) live = (live & ~0x0700u) | ((uint32_t)(level & 7) << 8);
    if (s->reg_write(uc, s->r_sr, &live) || s->reg_write(uc, s->r_pc, &handler)) return -1;
    s->raised++;
    return 1;
}

/* dt2_on_idle col yield in C (Digitakt II, 2026-10-05): quando l'unico
   evento del giro e' il multiplo di yield_every (nessun completamento UART
   in coda: tx.deliver() non farebbe nulla), raise_vector(32) lo fa
   dt2_raise_vector come lo farebbe Machine.raise_vector (che passa dallo
   stesso C); se il C non puo' (-1), il Python di sempre. */
void dt2_on_idle_c(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    idle_state *s = (idle_state *)user_data;
    int txp = s->tx_pending && *s->tx_pending > 0;
    if (!txp && s->exc && ((s->n + 1) % s->yield_every) == 0) {
        s->n += 1;
        if (dt2_raise_vector(uc, (exc_state *)s->exc, 32, -1, 0) >= 0) {
            s->c_yields++;
            return;
        }
        s->slow(s->n);
        return;
    }
    dt2_on_idle(uc, address, size, user_data);
}

/* longrun.build:switch_to (Digitakt II, 2026-10-05): a ogni cambio di
   contesto (0x4000044a) il TCB in A0 si conta (ev['switch'], un Counter) e si
   aggiunge alla sequenza se diverso dall'ultimo (ev['switch_seq']). Stessi
   dati, in C: ~2000 chiamate Python al secondo emulato in meno. */
#include <stdlib.h>
#define SWITCH_MAX 512
typedef struct {
    reg_rw_fn reg_read;
    int32_t r_a0, n;
    uint32_t tcb[SWITCH_MAX];
    int64_t cnt[SWITCH_MAX];
    uint32_t *seq;
    int64_t seqlen, seqcap;
    int32_t err, pad;
} switch_state;

void dt2_on_switch(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    switch_state *s = (switch_state *)user_data;
    uint32_t a0 = 0;
    int i;
    (void)address; (void)size;
    if (s->reg_read(uc, s->r_a0, &a0)) { s->err = 1; return; }
    for (i = 0; i < s->n; i++)
        if (s->tcb[i] == a0) break;
    if (i == s->n) {
        if (s->n == SWITCH_MAX) { s->err = 2; return; }
        s->tcb[i] = a0; s->cnt[i] = 0; s->n++;
    }
    s->cnt[i]++;
    if (s->seqlen == 0 || s->seq[s->seqlen - 1] != a0) {
        if (s->seqlen == s->seqcap) {
            int64_t cap = s->seqcap ? s->seqcap * 2 : 4096;
            uint32_t *p = (uint32_t *)realloc(s->seq, (size_t)cap * sizeof(uint32_t));
            if (!p) { s->err = 3; return; }
            s->seq = p; s->seqcap = cap;
        }
        s->seq[s->seqlen++] = a0;
    }
}

/* emu/dspi2.py _install_hle (DT2_DSPI2_HLE): la routine d'invio dei
   parametri al DSP (0x400cd2bc), 1500 chiamate al secondo emulato. Le letture
   dello stack e della SRAM, il frame, la copia dell'ultimo RX, gli zeri e
   l'`rts` in C; il Python solo per il registro SR del DSPI2 (Machine.mmio) e
   per consegnare il frame agli ascoltatori: py(frame, n) -> 1 consegnato
   (EOQF acceso: RX a zero dopo), 0 EOQF spento, -1 errore. */
typedef int (*hle_py_fn)(const uint8_t *words, int64_t n);
typedef struct {
    reg_rw_fn reg_read, reg_write;
    mem_rw_fn mem_read, mem_write;
    int32_t r_a7, r_pc;
    uint32_t frame_len_var, pushr_area, rx_area, max_bytes;
    hle_py_fn py;
    int64_t calls, frames;
    int32_t err, pad;
    uint8_t words[0x1000], raw[0x2000];
} hle_state;

void dt2_dspi2_hle(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    hle_state *s = (hle_state *)user_data;
    uint8_t b[20];
    uint32_t sp = 0, ret, payload, rxbuf, total;
    int32_t n_tx, rxlen;
    (void)address; (void)size;
    s->calls++;
    if (s->reg_read(uc, s->r_a7, &sp) || s->mem_read(uc, sp, b, 20)) { s->err = 1; return; }
    ret = be32(b); n_tx = (int32_t)be32(b + 4); payload = be32(b + 8);
    rxlen = (int32_t)be32(b + 12); rxbuf = be32(b + 16);
    if (s->mem_read(uc, s->frame_len_var, b, 4)) { s->err = 1; return; }
    total = be32(b);
    if (total) {
        if (rxbuf && rxlen > 0) {
            int64_t left = rxlen, off = 0;
            while (left > 0) {            /* uc.mem_write(rxbuf, uc.mem_read(RX_AREA, rxlen)) */
                int64_t k = left > (int64_t)sizeof s->raw ? (int64_t)sizeof s->raw : left;
                if (s->mem_read(uc, s->rx_area + (uint32_t)off, s->raw, (uint64_t)k)
                        || s->mem_write(uc, rxbuf + (uint32_t)off, s->raw, (uint64_t)k)) { s->err = 2; return; }
                left -= k; off += k;
            }
        }
        {
            uint32_t t = total < s->max_bytes ? total : s->max_bytes;
            int64_t entries = t / 2, n_pay, nw, i, r;
            n_pay = n_tx > 0 ? n_tx : 0;
            if (n_pay > (int64_t)s->max_bytes) n_pay = s->max_bytes;
            n_pay /= 2;
            nw = entries > n_pay ? entries : n_pay;     /* words[:n_pay*2] = ... puo' allungare */
            memset(s->words, 0, (size_t)(nw * 2));
            if (n_pay && s->mem_read(uc, payload, s->words, (uint64_t)(n_pay * 2))) { s->err = 3; return; }
            if (entries > n_pay) {
                int64_t k = entries - n_pay;
                if (s->mem_read(uc, s->pushr_area + (uint32_t)(n_pay * 4), s->raw, (uint64_t)(k * 4))) { s->err = 3; return; }
                for (i = 0; i < k; i++) {
                    s->words[(n_pay + i) * 2] = s->raw[i * 4 + 2];
                    s->words[(n_pay + i) * 2 + 1] = s->raw[i * 4 + 3];
                }
            }
            r = s->py(s->words, nw * 2);
            if (r < 0) { s->err = 4; return; }
            if (r == 1) {
                s->frames++;
                memset(s->raw, 0, (size_t)(entries * 2));
                if (entries && s->mem_write(uc, s->rx_area, s->raw, (uint64_t)(entries * 2))) { s->err = 5; return; }
            }
        }
    }
    sp += 4;                                            /* rts */
    if (s->reg_write(uc, s->r_a7, &sp) || s->reg_write(uc, s->r_pc, &ret)) s->err = 6;
}

void dt2_on_intr(void *uc, uint32_t intno, void *user_data)
{
    exc_state *s = (exc_state *)user_data;
    if (intno == 0x100) {                                  /* EXCP_RTE */
        uint8_t b[8];
        uint32_t sp = 0, sr, pc;
        if (!s->reg_read(uc, s->r_a7, &sp) && !s->mem_read(uc, sp, b, 8)) {
            sr = rbe16(b + 2); pc = be32(b + 4);
            s->reg_write(uc, s->r_sr, &sr);
            s->reg_write(uc, s->r_pc, &pc);
            sp += 8;
            s->reg_write(uc, s->r_a7, &sp);
            s->rte++;
            return;
        }
    }
    s->fallbacks++;
    s->fallback(uc, intno);
}

/* ---- SSI (emu/ssi.py Ssi0Dma, Syntakt/Rytm: SSI1, eDMA RX/TX configurabili,
   INT_HALF, SMOD sulla sorgente, cattura del TX) in C. Stessa logica di
   step / service / _run_minor / _remaining_minors in modo batch, senza peer
   RX (rx_peer e rx_fill spenti: l'RX fa solo avanzare gli indirizzi). Tempi
   interi scalati per request_hz: next = nextN / rh, periodo = ips / rh. Le
   consegne dei vettori restano in Python. ---- */
typedef struct {
    mem_rw_fn mem_read, mem_write;
    int64_t rh, ips, nextN;
    int32_t has_next, en;               /* en: bit0 RX, bit1 TX abilitati */
    int32_t rx_ch, tx_ch;
    int32_t level_retry, int50_asserted, int50_delivered, force_asserted, force_delivered;
    int64_t now, requests, major_rx, major_tx, sg_rx, sg_tx, tx_bytes;
    int64_t slack, level_retry_step;
    int32_t err, pad;
    uint8_t *txbuf;                     /* != NULL: i byte del TX vanno qui (tx_sink) */
    int64_t txcap, txlen;
    /* emu/nativespin.py: consegne dei vettori e hook in C (stato posseduto dal C) */
    int64_t vector170, vector191;
    struct irq_ctx *q;
    int32_t tx_vec, force_vec;
    /* Digitakt II: il peer DSP minimo (Ssi0Dma.rx_peer, DT2_DSP_PEER=1) -- il
       marcatore 0x007fffff all'inizio di ogni banco RX, zeri altrove; gli
       intervalli gia' azzerati (Ssi0Dma._rx_zeroed) in una tabella di chiavi
       (lo << 32 | hi), 0 = vuota. E il CRC del TX sul percorso parola per
       parola, come Ssi0Dma._run_minor (tx_crc32, diagnostica). */
    int32_t rx_peer, nbanks;
    uint32_t bank0, bank_bytes;
    int64_t rx_markers;
    uint64_t *zt;
    int64_t zcap, zlen;
    uint32_t tx_crc32, pad3;
} sst_state;

/* zlib.crc32(data, crc) */
static uint32_t sst_crc32(uint32_t crc, const uint8_t *p, int64_t n)
{
    int64_t i; int k;
    crc = ~crc;
    for (i = 0; i < n; i++) {
        crc ^= p[i];
        for (k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

/* Ssi0Dma._rx_word: la parola che il peer DSP mette all'indirizzo RX `dest` */
static uint32_t sst_rx_word(sst_state *s, uint32_t dest)
{
    int64_t rel = (int64_t)dest - (int64_t)s->bank0;
    if (rel >= 0 && rel < (int64_t)s->bank_bytes * s->nbanks && rel % s->bank_bytes == 0) {
        s->rx_markers++;
        return 0x007FFFFFu;
    }
    return 0;
}

static int sst_put32(void *uc, sst_state *s, uint32_t a, uint32_t v)
{
    uint8_t b[4]; b[0] = v >> 24; b[1] = v >> 16; b[2] = v >> 8; b[3] = v;
    if (s->mem_write(uc, a, b, 4)) { s->err = 11; return -1; }
    return 0;
}

/* -> 1 se la chiave e' nuova (e la aggiunge), 0 se c'era, -1 tabella piena */
static int sst_zt_add(sst_state *s, uint64_t key)
{
    uint64_t h = (key * 0x9E3779B97F4A7C15ull) >> 20;
    int64_t i, j;
    if (!s->zt || s->zcap <= 0) return -1;
    for (i = 0; i < s->zcap; i++) {
        j = (int64_t)((h + (uint64_t)i) % (uint64_t)s->zcap);
        if (s->zt[j] == key) return 0;
        if (s->zt[j] == 0) {
            if (s->zlen + 1 >= s->zcap) return -1;
            s->zt[j] = key; s->zlen++;
            return 1;
        }
    }
    return -1;
}

/* Python: gli intervalli che il Python aveva gia' azzerato. -> come sst_zt_add */
int dt2_sst_zt_add(sst_state *s, uint64_t key)
{
    return sst_zt_add(s, key);
}

/* Ssi0Dma._run_minor, ramo contiguo, RX col peer: zeri una volta per
   intervallo, poi i marcatori sugli inizi di banco che cadono nel blocco. */
static int sst_rx_block(void *uc, sst_state *s, uint32_t dest, int64_t words)
{
    uint64_t lo = dest, hi = (uint64_t)dest + (uint64_t)words * 4u;
    int64_t b;
    int r;
    if (hi > 0xFFFFFFFFull) { s->err = 12; return -1; }
    r = sst_zt_add(s, (lo << 32) | hi);
    if (r < 0) { s->err = 12; return -1; }
    if (r == 1) {
        static const uint8_t z[256];
        uint64_t a = lo;
        while (a < hi) {
            uint64_t n = hi - a > sizeof z ? sizeof z : hi - a;
            if (s->mem_write(uc, a, (void *)z, n)) { s->err = 11; return -1; }
            a += n;
        }
    }
    for (b = 0; b < s->nbanks; b++) {
        uint64_t start = (uint64_t)s->bank0 + (uint64_t)b * s->bank_bytes;
        if (lo <= start && start < hi) {
            if (sst_put32(uc, s, (uint32_t)start, 0x007FFFFFu)) return -1;
            s->rx_markers++;
        }
    }
    return 0;
}

#define SST_TCD(ch) (0xFC045000u + (uint32_t)(ch) * 0x20u)

static int sst_u16(void *uc, sst_state *s, int ch, uint32_t off, uint16_t *v)
{
    uint8_t b[2];
    if (s->mem_read(uc, SST_TCD(ch) + off, b, 2)) { s->err = 10; return -1; }
    *v = rbe16(b);
    return 0;
}
static int sst_w32(void *uc, sst_state *s, int ch, uint32_t off, uint32_t v)
{
    uint8_t b[4]; b[0] = v >> 24; b[1] = v >> 16; b[2] = v >> 8; b[3] = v;
    if (s->mem_write(uc, SST_TCD(ch) + off, b, 4)) { s->err = 11; return -1; }
    return 0;
}
static int sst_w16(void *uc, sst_state *s, int ch, uint32_t off, uint16_t v)
{
    uint8_t b[2]; b[0] = v >> 8; b[1] = v;
    if (s->mem_write(uc, SST_TCD(ch) + off, b, 2)) { s->err = 11; return -1; }
    return 0;
}

static int64_t sst_remaining(void *uc, sst_state *s)
{
    int tx = (s->en & 2) != 0;
    int ch = tx ? s->tx_ch : s->rx_ch;
    uint16_t c, csr, biter;
    if (!(s->en & (tx ? 2 : 1))) return 1;
    if (sst_u16(uc, s, ch, 0x14, &c)) return 1;
    c &= 0x7FFF;
    if (c && tx) {                      /* INT_HALF: anche la meta' e' un confine */
        if (sst_u16(uc, s, ch, 0x1E, &csr) || sst_u16(uc, s, ch, 0x1C, &biter)) return 1;
        if (csr & 0x0004) {
            uint16_t half = (biter & 0x7FFF) / 2;
            if (c > half) c -= half;
        }
    }
    return c ? c : 1;
}

static int64_t sst_fdiv(int64_t a, int64_t b) { int64_t q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
static int64_t sst_cdiv(int64_t a, int64_t b) { return -sst_fdiv(-a, b); }

int64_t dt2_sst_step(void *uc, sst_state *s, int64_t done, int64_t remaining)
{
    int64_t targetN, step;
    s->now = done;
    if (!s->en) return remaining;
    if (!s->has_next) { s->nextN = done * s->rh + s->ips; s->has_next = 1; }
    targetN = s->nextN + (sst_remaining(uc, s) - 1) * s->ips;
    step = sst_cdiv(targetN - done * s->rh, s->rh);
    if (step < 1) step = 1;
    if (s->level_retry && ((s->int50_asserted && !s->int50_delivered) ||
                           (s->force_asserted && !s->force_delivered)))
        if (step > s->level_retry_step) step = s->level_retry_step;
    return (remaining >= 0 && remaining < step) ? remaining : step;
}

static int sst_cap(void *uc, sst_state *s, uint32_t a, int64_t n)
{
    if (s->txlen + n > s->txcap) { s->err = 6; return -1; }
    if (s->mem_read(uc, a, s->txbuf + s->txlen, (uint64_t)n)) { s->err = 10; return -1; }
    s->txlen += n;
    return 0;
}

/* -> 1 major loop finito, 0 no, -1 errore (s->err) */
static int sst_run_minor(void *uc, sst_state *s, int ch, int capture_tx, int64_t count)
{
    uint8_t t[32];
    uint32_t source, nbytes, dest, dlast, slast, smask, sbase, smod;
    uint16_t attr, citer_raw, biter_raw, csr, citer, half;
    int16_t soff, doff;
    int64_t words, w;
    int is_tx = ch == s->tx_ch;
    if (!(s->en & (is_tx ? 2 : 1))) return 0;
    if (s->mem_read(uc, SST_TCD(ch), t, 32)) { s->err = 10; return -1; }
    source = be32(t); attr = rbe16(t + 4); soff = (int16_t)rbe16(t + 6); nbytes = be32(t + 8);
    slast = be32(t + 12); dest = be32(t + 16); citer_raw = rbe16(t + 20); doff = (int16_t)rbe16(t + 22);
    dlast = be32(t + 24); biter_raw = rbe16(t + 28); csr = rbe16(t + 30);
    if ((citer_raw & 0x8000) || (biter_raw & 0x8000)) { s->err = 1; return -1; }
    citer = citer_raw & 0x7FFF;
    if (!citer) return 0;
    if ((1u << (attr & 7)) != 4 || (1u << ((attr >> 8) & 7)) != 4) { s->err = 2; return -1; }
    if (nbytes & 0xC0000000u) {
        if ((nbytes >> 10) & 0xFFFFFu) { s->err = 7; return -1; }
        nbytes &= 0x3FFu;
    }
    smod = (attr >> 11) & 0x1F;
    smask = smod ? ((1u << smod) - 1u) : 0xFFFFFFFFu;
    sbase = source & ~smask;
    if (!nbytes || nbytes % 4) { s->err = 3; return -1; }
    if (count < 1) count = 1;
    if (count > citer) count = citer;
    words = count * (int64_t)(nbytes / 4);
    if (count > 1 && (soff == 0 || soff == 4) && (doff == 0 || doff == 4)) {
        if (!capture_tx && s->rx_peer) {
            if (doff) {
                if (sst_rx_block(uc, s, dest, words)) return -1;
            } else {
                for (w = 0; w < words; w++)
                    if (sst_put32(uc, s, dest, sst_rx_word(s, dest))) return -1;
            }
        }
        if (capture_tx) {
            if (s->txbuf) {
                if (soff) {
                    int64_t n = words * 4; uint32_t a = source;
                    while (n) {
                        int64_t k = (int64_t)smask + 1 - (a & smask);
                        if (k > n) k = n;
                        if (sst_cap(uc, s, a, k)) return -1;
                        n -= k;
                        a = sbase | ((a + (uint32_t)k) & smask);
                    }
                } else {
                    for (w = 0; w < words; w++) if (sst_cap(uc, s, source, 4)) return -1;
                }
            }
            s->tx_bytes += words * 4;
        }
        source = sbase | ((source + (uint32_t)((int64_t)soff * words)) & smask);
        dest += (uint32_t)((int64_t)doff * words);
    } else {
        for (w = 0; w < words; w++) {
            if (capture_tx) {
                uint8_t wb[4];
                if (s->txbuf && sst_cap(uc, s, source, 4)) return -1;
                if (s->mem_read(uc, source, wb, 4)) { s->err = 10; return -1; }
                s->tx_crc32 = sst_crc32(s->tx_crc32, wb, 4);
                s->tx_bytes += 4;
            } else if (s->rx_peer) {
                if (sst_put32(uc, s, dest, sst_rx_word(s, dest))) return -1;
            }
            source = sbase | ((source + (uint32_t)(int32_t)soff) & smask);
            dest += (uint32_t)(int32_t)doff;
        }
    }
    if (sst_w32(uc, s, ch, 0x00, source) || sst_w32(uc, s, ch, 0x10, dest)) return -1;
    half = (biter_raw & 0x7FFF) / 2;
    if (is_tx && (csr & 0x0004) && citer > half && half >= citer - count) {
        s->int50_asserted = 1; s->int50_delivered = 0;
    }
    citer = (uint16_t)(citer - count);
    if (sst_w16(uc, s, ch, 0x14, citer)) return -1;
    if (citer) return 0;
    source = sbase | ((source + slast) & smask);
    if (sst_w32(uc, s, ch, 0x00, source)) return -1;
    if (is_tx) s->major_tx++; else s->major_rx++;
    if (csr & 0x0010) {                               /* E_SG */
        uint8_t d[32];
        if (dlast & 0x1F) { s->err = 5; return -1; }
        if (s->mem_read(uc, dlast, d, 32)) { s->err = 10; return -1; }
        if (s->mem_write(uc, SST_TCD(ch), d, 32)) { s->err = 11; return -1; }
        if (is_tx) s->sg_tx++; else s->sg_rx++;
    } else {
        dest += dlast;
        if (sst_w32(uc, s, ch, 0x10, dest) || sst_w16(uc, s, ch, 0x14, biter_raw)) return -1;
    }
    if (is_tx && (csr & 0x0002)) { s->int50_asserted = 1; s->int50_delivered = 0; }
    return 1;
}

/* la parte di Ssi0Dma.service prima delle consegne dei vettori; -> 0, o -1 (s->err) */
int dt2_sst_service(void *uc, sst_state *s, int64_t done)
{
    int64_t horizonN, due, k, left, c;
    s->now = done;
    horizonN = (done + s->slack) * s->rh;
    if (s->has_next && horizonN >= s->nextN) {
        due = sst_fdiv(horizonN - s->nextN, s->ips) + 1;
        k = sst_remaining(uc, s);
        if (due < k) k = due;
        s->requests += k;
        left = k;
        while (left > 0 && (s->en & 1)) {
            uint16_t ci;
            if (sst_u16(uc, s, s->rx_ch, 0x14, &ci)) return -1;
            c = ci & 0x7FFF;
            if (!c || c > left) c = left;
            if (sst_run_minor(uc, s, s->rx_ch, 0, c) < 0) return -1;
            left -= c;
        }
        if (sst_run_minor(uc, s, s->tx_ch, 1, k) < 0) return -1;
        s->nextN += k * s->ips;
        if (s->nextN <= done * s->rh) s->nextN = done * s->rh + s->ips;
    }
    return s->err ? -1 : 0;
}
/* ---- gated code hook: Python only when the model has something to do ----
   The model keeps `open` up to date (a property of the Python object); a
   hook with `open` == 0 costs one C call instead of a trip into Python.
   `cfn` (emu/nativespin.py): the model's hook is in C, called instead of
   `py` -- the hook keeps its place in Unicorn's list, so the order of the
   hooks on the same address does not change. */
typedef void (*dt2_gate_py)(void *uc, uint64_t address, uint32_t size);
typedef void (*dt2_gate_c)(void *uc, uint64_t address, uint32_t size, void *data);
typedef struct { int32_t open; int32_t pad; dt2_gate_py py; int64_t calls, passed;
                 dt2_gate_c cfn; void *cdata; } dt2_gate;

void dt2_gate_code(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    dt2_gate *g = (dt2_gate *)user_data;
    g->calls++;
    if (!g->open)
        return;
    g->passed++;
    if (g->cfn)
        g->cfn(uc, address, size, g->cdata);
    else
        g->py(uc, address, size);
}
