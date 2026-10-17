
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef int (*stop_fn)(void *uc);
typedef struct {
    int64_t left;
    int64_t blocks;
    stop_fn stop;
} step_state;

/* ---- Pagine MMIO del bus (emu/mmiobus.py) viste dall'host ----
   Un accesso dell'host (uc.mem_read/mem_write dal Python) a una pagina del bus va
   allo store, SENZA gli hook del guest: e' la semantica del bus. uc_mem_read di
   Unicorn su una regione mmio chiamerebbe invece le callback della pagina (gli
   hook), quindi il C che legge registri di periferica passa da hread/hwrite. */
typedef int (*mem_rw_fn)(void *uc, uint64_t address, void *bytes, uint64_t size);
typedef int (*reg_rw_fn)(void *uc, int regid, void *value);
#define BUS_SLOTS 256
typedef struct {
    uint32_t base[BUS_SLOTS];           /* base della pagina | 1; 0 = vuoto */
    uint8_t *store[BUS_SLOTS];
    int32_t n, pad;
} bus_map;

static uint8_t *bus_page(const bus_map *b, uint32_t pg)
{
    uint32_t i, k;
    if (!b || !b->n) return 0;
    for (i = 0; i < BUS_SLOTS; i++) {
        k = ((pg >> 12) + i) & (BUS_SLOTS - 1);
        if (b->base[k] == 0) return 0;
        if (b->base[k] == (pg | 1u)) return b->store[k];
    }
    return 0;
}

int dt2_bus_add(bus_map *b, uint32_t pg, uint8_t *store)
{
    uint32_t i, k;
    for (i = 0; i < BUS_SLOTS; i++) {
        k = ((pg >> 12) + i) & (BUS_SLOTS - 1);
        if (b->base[k] == (pg | 1u)) { b->store[k] = store; return 0; }
        if (b->base[k] == 0) {
            if (b->n >= BUS_SLOTS * 3 / 4) return -1;
            b->base[k] = pg | 1u; b->store[k] = store; b->n++;
            return 0;
        }
    }
    return -1;
}

/* ---- Eccezioni: Machine.raise_vector (senza srtrap) e l'RTE di
   Machine.install_exceptions, in C (dal Model:Cycles, 9ae8a87). Stessa sequenza
   del Python; -1 = lo rifa' il Python (es. pagina dello stack non mappata: la
   mappa lui con _ensure_frame). ---- */
typedef void (*intr_py_fn)(void *uc, uint32_t intno);
typedef struct {
    reg_rw_fn reg_read, reg_write;
    mem_rw_fn mem_read, mem_write;
    uint32_t vbr;
    int32_t r_a7, r_sr, r_pc;
    intr_py_fn fallback;
    int64_t raised, rte, fallbacks;
    bus_map *bus;
} exc_state;

static int hread(void *uc, exc_state *x, uint32_t a, void *buf, uint32_t n)
{
    uint8_t *d = (uint8_t *)buf, *st;
    while (n) {
        uint32_t pg = a & ~0xFFFu, k = 0x1000u - (a & 0xFFFu);
        if (k > n) k = n;
        st = bus_page(x->bus, pg);
        if (st) memcpy(d, st + (a & 0xFFFu), k);
        else if (x->mem_read(uc, a, d, k)) return -1;
        a += k; d += k; n -= k;
    }
    return 0;
}

static int hwrite(void *uc, exc_state *x, uint32_t a, const void *buf, uint32_t n)
{
    const uint8_t *d = (const uint8_t *)buf;
    uint8_t *st;
    while (n) {
        uint32_t pg = a & ~0xFFFu, k = 0x1000u - (a & 0xFFFu);
        if (k > n) k = n;
        st = bus_page(x->bus, pg);
        if (st) memcpy(st + (a & 0xFFFu), d, k);
        else if (x->mem_write(uc, a, (void *)d, k)) return -1;
        a += k; d += k; n -= k;
    }
    return 0;
}

static inline uint32_t xbe32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline uint16_t xbe16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

int dt2_raise_vector(void *uc, exc_state *s, int vec, int level, int from_instr)
{
    uint8_t b[8];
    uint32_t handler, pc = 0, sr = 0, sp = 0, live;
    uint16_t fmt;
    if (hread(uc, s, s->vbr + (uint32_t)vec * 4u, b, 4)) return -1;
    handler = xbe32(b);
    if (handler == 0 || handler >= 0x48000000u) return 0;
    if (s->reg_read(uc, s->r_pc, &pc)) return -1;
    if (from_instr && !hread(uc, s, pc, b, 2)) {
        uint16_t w = xbe16(b);
        if (w >= 0x4E40 && w <= 0x4E4F) pc += 2;          /* trap #N: riprende dopo */
    }
    if (s->reg_read(uc, s->r_sr, &sr) || s->reg_read(uc, s->r_a7, &sp)) return -1;
    sp -= 8;
    fmt = (uint16_t)(0x4000 | ((vec << 2) & 0x0FFC));
    b[0] = fmt >> 8; b[1] = fmt; b[2] = (uint8_t)(sr >> 8); b[3] = (uint8_t)sr;
    b[4] = pc >> 24; b[5] = pc >> 16; b[6] = pc >> 8; b[7] = pc;
    if (hwrite(uc, s, sp, b, 8)) return -1;
    if (s->reg_write(uc, s->r_a7, &sp)) return -1;
    live = (sr & 0xFFFFu) | 0x2000;
    if (level >= 0) live = (live & ~0x0700u) | ((uint32_t)(level & 7) << 8);
    if (s->reg_write(uc, s->r_sr, &live) || s->reg_write(uc, s->r_pc, &handler)) return -1;
    s->raised++;
    return 1;
}

void dt2_on_intr(void *uc, uint32_t intno, void *user_data)
{
    exc_state *s = (exc_state *)user_data;
    if (intno == 0x100) {                                  /* EXCP_RTE */
        uint8_t b[8];
        uint32_t sp = 0, sr, pc;
        if (!s->reg_read(uc, s->r_a7, &sp) && !hread(uc, s, sp, b, 8)) {
            sr = xbe16(b + 2); pc = xbe32(b + 4);
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
    exc_state *exc;                 /* != NULL: raise_vector(32) in C (dt2_raise_vector) */
    int64_t yields_c;               /* diagnostica: yield consegnati dal C */
} idle_state;

void dt2_on_idle(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    idle_state *s = (idle_state *)user_data;
    int64_t to_yield, j;
    (void)uc; (void)address; (void)size;
    s->n += 1;
    if ((s->tx_pending && *s->tx_pending > 0) || (s->n % s->yield_every) == 0) {
        /* do_halt: tx.deliver() (niente da fare senza completamenti in coda),
           poi m.raise_vector(32): in C se si puo', altrimenti il Python */
        if (s->exc && !(s->tx_pending && *s->tx_pending > 0)
                && dt2_raise_vector(uc, s->exc, 32, -1, 0) >= 0) {
            s->yields_c++;
            return;
        }
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

/* ---- cicli caldi del firmware eseguiti in C (FINDINGS d16) ----
   Funzioni di Unicorn passate dal Python (niente link a libunicorn). */
typedef struct {
    reg_rw_fn reg_read, reg_write;
    mem_rw_fn mem_read, mem_write;
    step_state *step;               /* lo stepper nativo: blocchi da accreditare */
    uint64_t exit_pc;               /* dove riprende Unicorn */
    int64_t calls, iters;           /* diagnostica */
} loop_state;

enum { R_A0 = 1, R_A3 = 4, R_D0 = 9, R_D2 = 11, R_PC = 18, R_SR = 17 };

/* Riempimento del buffer DSPI2 (Digitone II OS 1.11, 0x400cf918..0x400cf928):
       cmp.l d2,d0 ; ble exit ; move.w #$8001,(a0) ; addq.l #1,d2 ; addq.l #4,a0 ;
       move.w (a3)+,-2(a0) ; bra cmp
   Hook sul corpo (0x400cf91c): il blocco del corpo e' gia' contato. Si fanno
   l'iterazione corrente e altre j, con j limitato dal budget dello stepper (2 blocchi
   per iterazione: il corpo e il confronto), si accreditano 2j blocchi e si riparte dal
   confronto, che Unicorn esegue (N, Z, V, C). X e' quello dell'ultimo addq.l #1,d2. */
void dt2_loop_dspi2_fill(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    loop_state *s = (loop_state *)user_data;
    uint32_t d0, d2, a0, a3, sr;
    int64_t n, j, k, i;
    uint8_t src[4096], dst[8192];
    (void)address; (void)size;
    s->reg_read(uc, R_D0, &d0); s->reg_read(uc, R_D2, &d2);
    s->reg_read(uc, R_A0, &a0); s->reg_read(uc, R_A3, &a3);
    n = (int64_t)(int32_t)d0 - (int64_t)(int32_t)d2;          /* iterazioni rimaste, >= 1 */
    if (n < 1 || s->step == 0)
        return;
    j = n - 1;
    if (j > s->step->left / 2) j = s->step->left / 2;
    k = 1 + j;
    if (k > 2048) { k = 2048; j = k - 1; }
    /* sorgente e destinazione non devono sovrapporsi */
    if (!((uint64_t)a3 + 2 * k <= a0 || (uint64_t)a0 + 4 * k <= a3))
        return;
    if (s->mem_read(uc, a3, src, 2 * k) != 0)
        return;
    for (i = 0; i < k; i++) {
        dst[4 * i] = 0x80; dst[4 * i + 1] = 0x01;
        dst[4 * i + 2] = src[2 * i]; dst[4 * i + 3] = src[2 * i + 1];
    }
    if (s->mem_write(uc, a0, dst, 4 * k) != 0)
        return;
    d2 += (uint32_t)k; a0 += (uint32_t)(4 * k); a3 += (uint32_t)(2 * k);
    s->reg_write(uc, R_D2, &d2); s->reg_write(uc, R_A0, &a0); s->reg_write(uc, R_A3, &a3);
    s->reg_read(uc, R_SR, &sr);
    sr = (d2 - 1 == 0xFFFFFFFFu) ? (sr | 0x10) : (sr & ~0x10u);   /* X dell'ultimo addq */
    s->reg_write(uc, R_SR, &sr);
    s->step->left -= 2 * j;
    s->step->blocks += 2 * j;
    s->calls += 1; s->iters += k;
    {
        uint32_t pc = (uint32_t)s->exit_pc;
        s->reg_write(uc, R_PC, &pc);
    }
}

/* ---- ciclo EMAC di 0x400db160..0x400db186 (Main OS del Digitone II 1.11) ----
   Semantica di QEMU/Unicorn 2.1.4 patchato con MACSR = 0x20 (FI, con segno, niente RT,
   niente OMC; la cfv4e non ha EMAC_B): DISAS_INSN(mac), macmulf, macsatf,
   mac_set_flags, get_macf, movclr. Per iterazione:
     mac.w  d0.u,d6.u,(a1),d4,acc0     mac.l  d4,d5,acc0      move.l d1,(a2)+
     mac.w  d0.l,d6.u,(4,a1),d0,acc1   mac.l  d0,d5,(a0)+,d0,acc1
     movclr acc0,d1   move.l d1,(a1)+   movclr acc1,d4   move.l d4,(a1)+
     add.l d2,d4 ; swap d4 ; add.l d2,d1 ; move.w d4,d1 ; subq.l #2,d3 ; bgt   */
enum { R_A1 = 2, R_A2 = 3, R_D1 = 10, R_D3 = 12, R_D4 = 13, R_D5 = 14, R_D6 = 15,
       R_MACSR = 34, R_MACC0 = 36, R_MACC1 = 37 };
#define M_PAV0 0x100u
#define M_V 0x002u
#define M_Z 0x004u
#define M_N 0x008u
#define M_EV 0x001u

static inline uint64_t emac_mulf(uint32_t a, uint32_t b)        /* RT = 0 */
{
    int64_t p = (int64_t)(int32_t)a * (int32_t)b;
    return (uint64_t)(p >> 23);
}
static inline void emac_mac(uint64_t *acc, uint32_t *macsr, int n, uint64_t prod)
{
    int64_t sum, res;
    uint64_t v;
    *macsr &= ~(M_V | M_Z | M_N | M_EV);                       /* gen_mac_clear_flags */
    *acc += prod;
    sum = (int64_t)*acc;                                        /* macsatf */
    res = (int64_t)((uint64_t)sum << 16) >> 16;
    if (res != sum) *macsr |= M_V;
    if (*macsr & M_V) *macsr |= M_PAV0 << n;
    *acc = (uint64_t)res;
    v = *acc;                                                   /* mac_set_flags */
    if (v == 0) *macsr |= M_Z;
    else if (v & (1ull << 47)) *macsr |= M_N;
    if (*macsr & (M_PAV0 << n)) *macsr |= M_V;
    {
        int64_t e = (int64_t)v >> 40;                          /* FI */
        if (e != 0 && e != -1) *macsr |= M_EV;
    }
}
static inline uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline void wbe32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static inline int disjoint(uint32_t a, uint32_t an, uint32_t b, uint32_t bn)
{
    return (uint64_t)a + an <= b || (uint64_t)b + bn <= a;
}

#define EMAC1_MAX 1024
void dt2_loop_emac_pairs(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    loop_state *s = (loop_state *)user_data;
    uint32_t D[8], A0, A1, A2, macsr, sr, old, res;
    uint64_t acc0, acc1;
    int64_t rem, j, k, i;
    /* per thread: piu' motori (il plugin, piu' istanze) girano in thread diversi */
    static _Thread_local uint8_t in0[4 * EMAC1_MAX], in1[8 * EMAC1_MAX], out2[4 * EMAC1_MAX];
    int cont = 0;
    (void)address; (void)size;
    if (s->step == 0) return;
    s->reg_read(uc, R_MACSR, &macsr);
    if ((macsr & 0xF0u) != 0x20u) return;                       /* altro modo: lo fa Unicorn */
    for (i = 0; i < 8; i++) s->reg_read(uc, R_D0 + (int)i, &D[i]);
    s->reg_read(uc, R_A0, &A0); s->reg_read(uc, R_A1, &A1); s->reg_read(uc, R_A2, &A2);
    if ((int32_t)D[3] <= 0) return;
    rem = ((int64_t)(int32_t)D[3] + 1) / 2;                     /* iterazioni fino all'uscita */
    j = rem - 1;
    if (j > s->step->left) j = s->step->left;
    k = 1 + j;
    if (k > EMAC1_MAX) { k = EMAC1_MAX; j = k - 1; }
    if (!disjoint(A0, 4 * k, A1, 8 * k) || !disjoint(A2, 4 * k, A1, 8 * k) || !disjoint(A2, 4 * k, A0, 4 * k))
        return;
    if (s->mem_read(uc, A0, in0, 4 * k) || s->mem_read(uc, A1, in1, 8 * k)) return;
    s->reg_read(uc, R_MACC0, &acc0); s->reg_read(uc, R_MACC1, &acc1);
    for (i = 0; i < k; i++) {
        uint32_t L;
        L = be32(in1 + 8 * i);                                          /* mac.w (a1) -> d4 */
        emac_mac(&acc0, &macsr, 0, emac_mulf(D[0] & 0xffff0000u, D[6] & 0xffff0000u));
        D[4] = L;
        emac_mac(&acc0, &macsr, 0, emac_mulf(D[4], D[5]));             /* mac.l d4,d5 */
        wbe32(out2 + 4 * i, D[1]);                                      /* move.l d1,(a2)+ */
        L = be32(in1 + 8 * i + 4);                                      /* mac.w (4,a1) -> d0 */
        emac_mac(&acc1, &macsr, 1, emac_mulf(D[0] << 16, D[6] & 0xffff0000u));
        D[0] = L;
        L = be32(in0 + 4 * i);                                          /* mac.l (a0)+ -> d0 */
        emac_mac(&acc1, &macsr, 1, emac_mulf(D[0], D[5]));
        D[0] = L;
        D[1] = (uint32_t)((int64_t)acc0 >> 8); acc0 = 0; macsr &= ~M_PAV0;          /* movclr */
        wbe32(in1 + 8 * i, D[1]);
        D[4] = (uint32_t)((int64_t)acc1 >> 8); acc1 = 0; macsr &= ~(M_PAV0 << 1);
        wbe32(in1 + 8 * i + 4, D[4]);
        D[4] += D[2];
        D[4] = (D[4] << 16) | (D[4] >> 16);
        D[1] += D[2];
        D[1] = (D[1] & 0xffff0000u) | (D[4] & 0xffffu);
        old = D[3]; D[3] = old - 2;
    }
    res = D[3];
    s->reg_read(uc, R_SR, &sr);                                         /* CCR di subq.l #2,d3 */
    sr &= ~0x1Fu;
    if (old < 2u) sr |= 0x11u;                                         /* C e X */
    if (res & 0x80000000u) sr |= 0x08u;
    if (res == 0) sr |= 0x04u;
    if (((old ^ 2u) & (old ^ res)) & 0x80000000u) sr |= 0x02u;
    cont = !(sr & 0x04u) && (((sr >> 3) & 1) == ((sr >> 1) & 1));     /* bgt */
    if (s->mem_write(uc, A1, in1, 8 * k) || s->mem_write(uc, A2, out2, 4 * k)) return;
    A0 += 4 * (uint32_t)k; A1 += 8 * (uint32_t)k; A2 += 4 * (uint32_t)k;
    for (i = 0; i < 8; i++) s->reg_write(uc, R_D0 + (int)i, &D[i]);
    s->reg_write(uc, R_A0, &A0); s->reg_write(uc, R_A1, &A1); s->reg_write(uc, R_A2, &A2);
    s->reg_write(uc, R_MACC0, &acc0); s->reg_write(uc, R_MACC1, &acc1);
    s->reg_write(uc, R_MACSR, &macsr);
    s->reg_write(uc, R_SR, &sr);
    s->step->left -= j;
    s->step->blocks += j;
    s->calls += 1; s->iters += k;
    {
        uint32_t pc = cont ? 0x400db160u : (uint32_t)s->exit_pc;
        s->reg_write(uc, R_PC, &pc);
    }
}

/* ---- pagine MMIO (emu/mmiobus.py) con le callback in C ----
   Stessa regola del bus Python: gli hook della pagina scattano se begin <= indirizzo <= end,
   prima della lettura / prima della scrittura, nell'ordine di registrazione (li esegue il
   Python, chiamato solo se almeno uno copre l'indirizzo); il registro sta nello store della
   pagina (il bytearray del bus), big-endian. */
typedef void (*mmio_slow_r_fn)(void *uc, uint64_t addr, unsigned size, void *ud);
typedef void (*mmio_slow_w_fn)(void *uc, uint64_t addr, unsigned size, uint64_t value, void *ud);
/* un hook in C sulla pagina: la firma degli hook di memoria di Unicorn */
typedef void (*mmio_c_fn)(void *uc, int type, uint64_t addr, int size, int64_t value, void *data);
#define MMIO_MAXH 32
typedef struct {
    uint8_t *store;
    uint64_t base;
    int32_t nr, nw;
    uint64_t rb[MMIO_MAXH], re[MMIO_MAXH], wb[MMIO_MAXH], we[MMIO_MAXH];
    mmio_slow_r_fn slow_r;
    mmio_slow_w_fn slow_w;
    int64_t reads, writes, slow_reads, slow_writes;
    /* per hook, nell'ordine di registrazione: la chiave del Python (l'handle del
       bus), una funzione C al posto del Python (cfn, cdata), e per le scritture un
       filtro sul valore (SERQ: il modello agisce solo per i suoi canali; flt bit7
       acceso = filtro attivo, bit0 size == 1, bit1 niente bit 0x80, bit2-3 come
       trattare il bit 0x40: 0 ignorato, 1 = tutti i canali, 2 = mai) */
    int64_t rkey[MMIO_MAXH], wkey[MMIO_MAXH];
    mmio_c_fn rcfn[MMIO_MAXH], wcfn[MMIO_MAXH];
    void *rcdata[MMIO_MAXH], *wcdata[MMIO_MAXH];
    uint64_t wchans[MMIO_MAXH];
    int32_t wflt[MMIO_MAXH];
    int64_t filtered, c_calls;
} mmio_page;

static int mmio_pass(const mmio_page *p, unsigned i, unsigned size, uint64_t v)
{
    int f = p->wflt[i];
    if (!(f & 0x80)) return 1;
    if ((f & 1) && size != 1) return 0;
    if ((f & 2) && (v & 0x80)) return 0;
    if (v & 0x40) {
        int m40 = (f >> 2) & 3;
        if (m40 == 1) return 1;
        if (m40 == 2) return 0;
    }
    return (int)((p->wchans[i] >> (v & 0x3F)) & 1u);
}

uint64_t dt2_mmio_read(void *uc, uint64_t off, unsigned size, void *ud)
{
    mmio_page *p = (mmio_page *)ud;
    uint64_t a = p->base + off, v = 0;
    unsigned i, n = 0;
    unsigned idx[MMIO_MAXH];
    int64_t key[MMIO_MAXH];
    mmio_c_fn fn[MMIO_MAXH];
    void *dt[MMIO_MAXH];
    p->reads++;
    /* prima l'elenco, poi le chiamate: un hook che ne toglie un altro non sposta
       quelli ancora da chiamare (come la lista del Python) */
    for (i = 0; i < (unsigned)p->nr; i++)
        if (p->rb[i] <= a && a <= p->re[i]) {
            idx[n] = i; key[n] = p->rkey[i]; fn[n] = p->rcfn[i]; dt[n] = p->rcdata[i]; n++;
        }
    for (i = 0; i < n; i++) {
        if (fn[i]) { p->c_calls++; fn[i](uc, 16, a, (int)size, 0, dt[i]); }
        else { p->slow_reads++; p->slow_r(uc, a, size, (void *)(intptr_t)key[i]); }
    }
    (void)idx;
    for (i = 0; i < size; i++)
        v = (v << 8) | p->store[off + i];
    return v;
}

void dt2_mmio_write(void *uc, uint64_t off, unsigned size, uint64_t value, void *ud)
{
    mmio_page *p = (mmio_page *)ud;
    uint64_t a = p->base + off;
    unsigned i, n = 0;
    int64_t key[MMIO_MAXH];
    mmio_c_fn fn[MMIO_MAXH];
    void *dt[MMIO_MAXH];
    p->writes++;
    if (size < 8) value &= (1ull << (8 * size)) - 1;
    for (i = 0; i < (unsigned)p->nw; i++)
        if (p->wb[i] <= a && a <= p->we[i]) {
            if (!mmio_pass(p, i, size, value)) { p->filtered++; continue; }
            key[n] = p->wkey[i]; fn[n] = p->wcfn[i]; dt[n] = p->wcdata[i]; n++;
        }
    for (i = 0; i < n; i++) {
        if (fn[i]) { p->c_calls++; fn[i](uc, 17, a, (int)size, (int64_t)value, dt[i]); }
        else { p->slow_writes++; p->slow_w(uc, a, size, value, (void *)(intptr_t)key[i]); }
    }
    for (i = 0; i < size; i++)
        p->store[off + i] = (uint8_t)(value >> (8 * (size - 1 - i)));
}

/* ---- SSI0 / eDMA48-50 (emu/ssi.py, modo a blocchi) in C, d22 ----
   Stessa logica di Ssi0Dma.step / service / _run_minor / _remaining_minors. I tempi sono
   interi scalati per request_hz: next = nextN / rh, periodo = ips / rh (esatti come le
   Fraction del Python). Le consegne dei vettori restano in Python. */
typedef struct {
    mem_rw_fn mem_read, mem_write;
    int64_t rh, ips;
    int64_t nextN;                  /* next * rh */
    int32_t has_next;
    int32_t en;                     /* bit0 RX (48) abilitato, bit1 TX (50) */
    int32_t rx_peer, level_retry;
    int32_t int50_asserted, int50_delivered, force_asserted, force_delivered;
    int64_t now, requests, major_rx, major_tx, sg_rx, sg_tx, tx_bytes, rx_markers;
    uint32_t rx_bank0, rx_bank_bytes, rx_banks, marker;
    int64_t slack, level_retry_step;
    int32_t err;                    /* != 0: errore, lo solleva il Python */
    int32_t nzero;
    uint32_t zlo[16384], zhi[16384]; /* intervalli RX gia' azzerati (_rx_zeroed), hash; zhi 0 = vuoto */
    uint32_t tx_crc32;              /* zlib.crc32 del TX letto parola per parola */
    /* emu/nativespin.py: stato posseduto dal C (consegne dei vettori e hook in C) */
    uint32_t pad2;
    int64_t vector170, vector191;
    struct irq_ctx *q;
    int32_t tx_vec, force_vec;
} ssi_state;

static uint32_t ssi_crc32(uint32_t crc, const uint8_t *p, int64_t n)
{
    int64_t i; int k;
    crc = ~crc;
    for (i = 0; i < n; i++) {
        crc ^= p[i];
        for (k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    }
    return ~crc;
}

#define SSI_TCD(ch) (0xFC045000u + (uint32_t)(ch) * 0x20u)

static inline uint16_t rbe16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline void wbe16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }

static int ssi_u16(void *uc, ssi_state *s, int ch, uint32_t off, uint16_t *v)
{
    uint8_t b[2];
    if (s->mem_read(uc, SSI_TCD(ch) + off, b, 2)) { s->err = 10; return -1; }
    *v = rbe16(b);
    return 0;
}
static int ssi_w32(void *uc, ssi_state *s, int ch, uint32_t off, uint32_t v)
{
    uint8_t b[4]; wbe32(b, v);
    if (s->mem_write(uc, SSI_TCD(ch) + off, b, 4)) { s->err = 11; return -1; }
    return 0;
}
static int ssi_w16(void *uc, ssi_state *s, int ch, uint32_t off, uint16_t v)
{
    uint8_t b[2]; wbe16(b, v);
    if (s->mem_write(uc, SSI_TCD(ch) + off, b, 2)) { s->err = 11; return -1; }
    return 0;
}

static int64_t ssi_remaining(void *uc, ssi_state *s)
{
    int ch = (s->en & 2) ? 50 : 48;
    uint16_t c;
    if (!(s->en & (ch == 50 ? 2 : 1))) return 1;
    if (ssi_u16(uc, s, ch, 0x14, &c)) return 1;
    c &= 0x7FFF;
    return c ? c : 1;
}

static int64_t fdiv(int64_t a, int64_t b) { int64_t q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
static int64_t cdiv(int64_t a, int64_t b) { return -fdiv(-a, b); }

int64_t dt2_ssi_step(void *uc, ssi_state *s, int64_t done, int64_t remaining)
{
    int64_t targetN, step;
    s->now = done;
    if (!s->en) return remaining;
    if (!s->has_next) { s->nextN = done * s->rh + s->ips; s->has_next = 1; }
    targetN = s->nextN + (ssi_remaining(uc, s) - 1) * s->ips;
    step = cdiv(targetN - done * s->rh, s->rh);
    if (step < 1) step = 1;
    if (s->level_retry && ((s->int50_asserted && !s->int50_delivered) ||
                           (s->force_asserted && !s->force_delivered)))
        if (step > s->level_retry_step) step = s->level_retry_step;
    return (remaining >= 0 && remaining < step) ? remaining : step;
}

/* insieme degli intervalli azzerati: 1 se (lo, hi) c'era, 0 se aggiunto ora, -1 se pieno */
int dt2_ssi_zeroed(ssi_state *s, uint32_t lo, uint32_t hi)
{
    uint32_t h = (lo * 2654435761u) ^ (hi * 40503u), i, n;
    for (n = 0; n < 16384; n++) {
        i = (h + n) & 16383u;
        if (s->zhi[i] == 0) {
            if (s->nzero >= 12288) return -1;
            s->zlo[i] = lo; s->zhi[i] = hi; s->nzero++;
            return 0;
        }
        if (s->zlo[i] == lo && s->zhi[i] == hi) return 1;
    }
    return -1;
}

/* -> 1 se il major loop e' finito, 0 se no, -1 errore (s->err) */
static int ssi_run_minor(void *uc, ssi_state *s, int ch, int capture_tx, int64_t count)
{
    uint8_t t[32], z[4096];
    uint32_t source, nbytes, dest, dlast, slast;
    uint16_t attr, citer_raw, biter_raw, csr, citer;
    int16_t soff, doff;
    uint32_t ssz, dsz;
    int64_t words, w;
    if (!(s->en & (ch == 50 ? 2 : 1))) return 0;
    if (s->mem_read(uc, SSI_TCD(ch), t, 32)) { s->err = 10; return -1; }
    source = be32(t); attr = rbe16(t + 4); soff = (int16_t)rbe16(t + 6); nbytes = be32(t + 8);
    slast = be32(t + 12); dest = be32(t + 16); citer_raw = rbe16(t + 20); doff = (int16_t)rbe16(t + 22);
    dlast = be32(t + 24); biter_raw = rbe16(t + 28); csr = rbe16(t + 30);
    if ((citer_raw & 0x8000) || (biter_raw & 0x8000)) { s->err = 1; return -1; }
    citer = citer_raw & 0x7FFF;
    if (!citer) return 0;
    ssz = 1u << (attr & 7); dsz = 1u << ((attr >> 8) & 7);
    if (ssz != 4 || dsz != 4) { s->err = 2; return -1; }
    if (!nbytes || nbytes % ssz) { s->err = 3; return -1; }
    if (count < 1) count = 1;
    if (count > citer) count = citer;
    words = count * (int64_t)(nbytes / ssz);
    if (count > 1 && (soff == 0 || soff == (int16_t)ssz) && (doff == 0 || doff == (int16_t)dsz)) {
        if (capture_tx) {
            s->tx_bytes += words * ssz;               /* senza DT2_SSI_TX_CRC: si conta */
        } else if (s->rx_peer) {
            if (doff) {
                uint32_t lo = dest, hi = dest + (uint32_t)(words * 4), b, start;
                int found = dt2_ssi_zeroed(s, lo, hi);
                if (found < 0) { s->err = 4; return -1; }
                if (!found) {
                    memset(z, 0, sizeof z);
                    for (w = 0; w < words * 4; w += (int64_t)sizeof z) {
                        int64_t n = words * 4 - w < (int64_t)sizeof z ? words * 4 - w : (int64_t)sizeof z;
                        if (s->mem_write(uc, dest + (uint32_t)w, z, (uint64_t)n)) { s->err = 11; return -1; }
                    }
                }
                for (b = 0; b < s->rx_banks; b++) {
                    start = s->rx_bank0 + b * s->rx_bank_bytes;
                    if (lo <= start && start < hi) {
                        uint8_t mk[4]; wbe32(mk, s->marker);
                        if (s->mem_write(uc, start, mk, 4)) { s->err = 11; return -1; }
                        s->rx_markers++;
                    }
                }
            } else {
                for (w = 0; w < words; w++) {
                    uint32_t rel = dest - s->rx_bank0, v = 0;
                    uint8_t b4[4];
                    if (rel < s->rx_bank_bytes * s->rx_banks && rel % s->rx_bank_bytes == 0) {
                        s->rx_markers++; v = s->marker;
                    }
                    wbe32(b4, v);
                    if (s->mem_write(uc, dest, b4, 4)) { s->err = 11; return -1; }
                }
            }
        }
        source += (uint32_t)((int64_t)soff * words);
        dest += (uint32_t)((int64_t)doff * words);
    } else {
        for (w = 0; w < words; w++) {
            if (capture_tx) {
                uint8_t b4[4];
                if (s->mem_read(uc, source, b4, 4)) { s->err = 10; return -1; }
                s->tx_bytes += ssz;
                s->tx_crc32 = ssi_crc32(s->tx_crc32, b4, 4);
            } else if (s->rx_peer) {
                uint32_t rel = dest - s->rx_bank0, v = 0;
                uint8_t b4[4];
                if (rel < s->rx_bank_bytes * s->rx_banks && rel % s->rx_bank_bytes == 0) {
                    s->rx_markers++; v = s->marker;
                }
                wbe32(b4, v);
                if (s->mem_write(uc, dest, b4, 4)) { s->err = 11; return -1; }
            }
            source += (uint32_t)(int32_t)soff;
            dest += (uint32_t)(int32_t)doff;
        }
    }
    if (ssi_w32(uc, s, ch, 0x00, source) || ssi_w32(uc, s, ch, 0x10, dest)) return -1;
    citer = (uint16_t)(citer - count);
    if (ssi_w16(uc, s, ch, 0x14, citer)) return -1;
    if (citer) return 0;
    source += slast;
    if (ssi_w32(uc, s, ch, 0x00, source)) return -1;
    if (ch == 48) s->major_rx++; else s->major_tx++;
    if (csr & 0x0010) {                               /* E_SG */
        uint8_t d[32];
        if (dlast & 0x1F) { s->err = 5; return -1; }
        if (s->mem_read(uc, dlast, d, 32)) { s->err = 10; return -1; }
        if (s->mem_write(uc, SSI_TCD(ch), d, 32)) { s->err = 11; return -1; }
        if (ch == 48) s->sg_rx++; else s->sg_tx++;
    } else {
        dest += dlast;
        if (ssi_w32(uc, s, ch, 0x10, dest) || ssi_w16(uc, s, ch, 0x14, biter_raw)) return -1;
    }
    if (ch == 50 && (csr & 0x0002)) { s->int50_asserted = 1; s->int50_delivered = 0; }
    return 1;
}

/* la parte di Ssi0Dma.service prima delle consegne dei vettori; -> 0, o -1 (s->err) */
int dt2_ssi_service(void *uc, ssi_state *s, int64_t done)
{
    int64_t horizonN, due, k, left, c;
    s->now = done;
    horizonN = (done + s->slack) * s->rh;
    if (s->has_next && horizonN >= s->nextN) {
        due = fdiv(horizonN - s->nextN, s->ips) + 1;
        k = ssi_remaining(uc, s);
        if (due < k) k = due;
        s->requests += k;
        left = k;
        while (left > 0 && (s->en & 1)) {
            uint16_t ci;
            if (ssi_u16(uc, s, 48, 0x14, &ci)) return -1;
            c = ci & 0x7FFF;
            if (!c || c > left) c = left;
            if (ssi_run_minor(uc, s, 48, 0, c) < 0) return -1;
            left -= c;
        }
        if (ssi_run_minor(uc, s, 50, 1, k) < 0) return -1;
        s->nextN += k * s->ips;
        if (s->nextN <= done * s->rh) s->nextN = done * s->rh + s->ips;
    }
    return 0;
}

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
    /* emu/nativespin.py (dt2_tim_service_full): contatori del ciclo C, sommati ai
       Counter Python all'uscita */
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

/* ---- DTCN (emu/dtim.py install_counters, on_read) ----
   L'hook Python di lettura di DTCNn non fa nulla quando value() e' None: timer in
   reset, clock fermo o pin esterno (CLK 0 o 3). Qui lo si decide in C dallo store
   di DTMRn e si chiama il Python (lo stesso hook, per handle) solo negli altri casi. */
typedef struct {
    const uint8_t *dtmr;                /* DTMRn nello store della pagina del bus */
    mmio_slow_r_fn slow;
    int64_t key;
    int64_t calls, passed;
} dtcn_gate;

void dt2_dtcn_read(void *uc, int type, uint64_t addr, int size, int64_t value, void *data)
{
    dtcn_gate *g = (dtcn_gate *)data;
    uint16_t m = (uint16_t)((g->dtmr[0] << 8) | g->dtmr[1]);
    int clk = (m >> 1) & 3;
    (void)type; (void)value;
    g->calls++;
    if (!(m & 0x01) || clk == 0 || clk == 3)
        return;
    g->passed++;
    g->slow(uc, addr, (unsigned)size, (void *)(intptr_t)g->key);
}

/* ---- valori forzati in lettura (harness.install_mmio) e SR della DSPI2 ----
   Machine.mmio[addr] vive in una cella C (emu/harness.py MmioValues): l'hook di
   lettura la scrive (big endian) nello store prima della lettura, come
   uc.mem_write(addr, pack('>I', mmio[addr])) del Python. dt2_mmio_w1c e'
   Dspi2._on_sr: con una scrittura di 4 byte all'indirizzo esatto i bit scritti a 1
   si spengono nella cella. */
typedef struct {
    uint8_t *reg;                       /* il registro nello store della pagina */
    uint32_t *cell;
    uint64_t addr;
    int64_t calls;
} mmio_cell;

void dt2_mmio_force(void *uc, int type, uint64_t addr, int size, int64_t value, void *data)
{
    mmio_cell *c = (mmio_cell *)data;
    uint32_t v = *c->cell;
    (void)uc; (void)type; (void)addr; (void)size; (void)value;
    c->calls++;
    c->reg[0] = (uint8_t)(v >> 24); c->reg[1] = (uint8_t)(v >> 16);
    c->reg[2] = (uint8_t)(v >> 8); c->reg[3] = (uint8_t)v;
}

void dt2_mmio_w1c(void *uc, int type, uint64_t addr, int size, int64_t value, void *data)
{
    mmio_cell *c = (mmio_cell *)data;
    (void)uc; (void)type;
    c->calls++;
    if (size == 4 && addr == c->addr)
        *c->cell &= ~(uint32_t)value;
}

/* ---- uscita audio dal vivo senza Python (emu/audio.py DspClockSink) ----
   Il callback di PortAudio in C: lo stesso lavoro di DspClockSink._cb +
   DspSynth.pull_into + grow_buffer, ma senza il GIL. Col callback Python la scheda
   audio aspettava il GIL ad ogni blocco (187 al secondo): con le altre macchine e la
   finestra che lo tengono, un callback in ritardo e' un blocco perso dalla scheda,
   l'anello del DSP resta pieno, il DSP non consuma i frame del ColdFire e il ColdFire
   rallenta (calo del tempo reale) anche senza un buco contato. */
typedef uint32_t (*au_pull_fn)(void *st, int32_t *out, int max);
typedef uint32_t (*au_level_fn)(void *st);
typedef void (*au_clock_fn)(int on, uint32_t lo, uint32_t hi);
#define AU_MAX 4096
typedef struct {
    au_pull_fn pull;
    au_level_fn level;
    au_clock_fn live_clock;
    void *st;
    double gain;                        /* MASTER / MASTER_FULL * 2^-31, lo aggiorna il Python */
    int32_t mute, started;
    int32_t lo, hi, hi_max, hi_step;    /* anello del DSP (DspSynth.clock_lo/hi, grow_buffer) */
    int64_t callbacks, underruns, late, samples;
    int32_t buf[2 * AU_MAX];
} au_state;

int dt2_pa_callback(const void *input, void *output, unsigned long frames,
                    const void *time_info, unsigned long flags, void *user_data)
{
    au_state *a = (au_state *)user_data;
    float *o = (float *)output;
    unsigned long i, n = frames > AU_MAX ? AU_MAX : frames;
    uint32_t got;
    double g;
    (void)input; (void)time_info;
    a->callbacks++;
    if (flags & 0x4)                    /* paOutputUnderflow: un blocco arrivato tardi */
        a->late++;
    if (!a->started) {                  /* si parte quando il DSP ha riempito il suo anello */
        if (a->level(a->st) < 2 * frames) {
            memset(o, 0, frames * 2 * sizeof(float));
            return 0;
        }
        a->started = 1;
    }
    got = a->pull(a->st, a->buf, (int)n);
    g = a->gain;
    for (i = 0; i < got; i++) {
        o[2 * i] = (float)((double)a->buf[2 * i] * g);
        o[2 * i + 1] = (float)((double)a->buf[2 * i + 1] * g);
    }
    for (i = got; i < frames; i++)
        o[2 * i] = o[2 * i + 1] = 0.0f;
    if (a->mute)
        memset(o, 0, frames * 2 * sizeof(float));
    a->samples += got;
    if (got < frames) {
        a->underruns++;
        if (a->hi < a->hi_max) {        /* DspSynth.grow_buffer */
            a->hi = a->hi + a->hi_step < a->hi_max ? a->hi + a->hi_step : a->hi_max;
            a->lo = a->hi / 2;
            a->live_clock(1, (uint32_t)a->lo, (uint32_t)a->hi);
        }
    }
    return 0;                           /* paContinue */
}

/* ================= dt2_spin (emu/nativespin.py) =================
   Il ciclo dei passi di longrun.spin(fast=True, pits=Timers, async_events=...) in
   C, con i servizi dei timer, dell'SSI0 e degli interrupt forzati (porting dal
   Model:Cycles, commit 9ae8a87/50eb8c0 del suo repository, adattato al bus MMIO e
   all'SSI col peer DSP di questa macchina). */
typedef int (*emu_start_fn)(void *uc, uint64_t begin, uint64_t until, uint64_t timeout, size_t count);
typedef int (*py_raise_fn)(int32_t vec, int32_t level);   /* Machine.raise_vector: 1, 0, -1 errore */
typedef int64_t (*py_step_fn)(int64_t now);               /* evento Python: step >=1, -1 None, -2 errore */
typedef int (*py_service_fn)(int64_t now);                /* evento Python: service 0, -1 errore */

/* Cio' che serve per consegnare un interrupt: dt2_raise_vector e il Python di
   ripiego quando il C non puo' (pagina dello stack non mappata). */
typedef struct irq_ctx {
    exc_state *exc;
    py_raise_fn py_raise;
    int64_t py_raises;
    int32_t err, pad;
} irq_ctx;

static const uint32_t INTC_BASE[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};

static int irq_raise(void *uc, irq_ctx *q, int vec, int level)
{
    int r = dt2_raise_vector(uc, q->exc, vec, level, 0);
    if (r >= 0)
        return r;
    q->py_raises++;
    r = q->py_raise(vec, level);
    if (r < 0) { q->err = 20; return -1; }
    return r;
}

/* emu/pit.py interrupt_level: il livello programmato (ICR & 7), 0 = None
   (spento o mascherato), -1 errore. */
static int irq_level(void *uc, irq_ctx *q, int vec, int respect_mask)
{
    uint8_t b[8];
    int i, src, icr;
    uint32_t hi, lo, masked;
    if (vec < 64 || vec >= 256)
        return 0;
    i = (vec - 64) / 64;
    src = vec - 64 - 64 * i;
    if (hread(uc, q->exc, INTC_BASE[i] + 0x40u + (uint32_t)src, b, 1)) { q->err = 21; return -1; }
    icr = b[0] & 7;
    if (!icr || !respect_mask)
        return icr;
    if (hread(uc, q->exc, INTC_BASE[i] + 0x08u, b, 8)) { q->err = 21; return -1; }
    hi = be32(b); lo = be32(b + 4);
    masked = src < 32 ? (lo >> src) & 1u : (hi >> (src - 32)) & 1u;
    return masked ? 0 : icr;
}

static int irq_ipl(void *uc, irq_ctx *q)
{
    uint32_t sr = 0;
    if (q->exc->reg_read(uc, q->exc->r_sr, &sr)) { q->err = 22; return -1; }
    return (int)((sr >> 8) & 7u);
}

/* ---- Pits / Dtims: service intero (pit.Pits.service, dtim.Dtims.service,
   con e senza DT2_LEVEL_IRQ), riga per riga. ---- */
static int tim_try(void *uc, tim_state *t, int c)        /* _try_pending */
{
    int vec = t->vec0 + c, lvl, ipl, r;
    lvl = irq_level(uc, t->q, vec, 1);
    if (lvl < 0) return -1;
    ipl = irq_ipl(uc, t->q);
    if (ipl < 0) return -1;
    if (lvl == 0 || ipl >= lvl) { t->missed[c]++; return 0; }
    r = irq_raise(uc, t->q, vec, lvl);
    if (r < 0) return -1;
    if (r) { t->fired[c]++; t->pend[c] = 0; }
    return r;
}

int dt2_tim_service_full(void *uc, tim_state *t, int64_t done)
{
    double p, d = (double)done;
    int i, c, r;
    if (t->held)
        return 0;
    for (i = 0; i < t->nch; i++) {
        c = t->ch[i];
        r = tim_period(uc, t, c, &p);
        if (r < 0) return -1;
        if (r == 0) { t->has[c] = 0; continue; }
        if (!t->has[c]) { t->next[c] = d + p; t->has[c] = 1; continue; }
        if (d < t->next[c]) {
            if (t->level_irq && t->pend[c] && tim_try(uc, t, c) < 0) return -1;
            continue;
        }
        t->next[c] += p;
        if (t->next[c] <= d)
            t->next[c] = d + p;          /* drop the backlog, do not chase it */
        if (t->kind == 1) {              /* DTER |= REF prima dell'interrupt */
            uint8_t b;
            if (t->store[c]) t->store[c][3] |= 0x02;
            else {
                if (hread(uc, t->q->exc, t->base[c] + 3u, &b, 1)) { t->err = 2; return -1; }
                b |= 0x02;
                if (hwrite(uc, t->q->exc, t->base[c] + 3u, &b, 1)) { t->err = 2; return -1; }
            }
        }
        if (t->level_irq) {
            if (t->pend[c]) t->merged[c]++;
            t->pend[c] = 1;
            if (tim_try(uc, t, c) < 0) return -1;
            continue;
        }
        {
            int vec = t->vec0 + c, lvl, ipl;
            lvl = irq_level(uc, t->q, vec, 1);
            if (lvl < 0) return -1;
            ipl = irq_ipl(uc, t->q);
            if (ipl < 0) return -1;
            if (lvl == 0 || ipl >= lvl)
                t->missed[c]++;
            else {
                r = irq_raise(uc, t->q, vec, lvl);
                if (r < 0) return -1;
                if (r) t->fired[c]++;
            }
        }
    }
    return 0;
}

/* ---- SSI0: _deliver_vector170 / _deliver_vector191 e i tre hook
   (_on_cint, _on_intfrch1, _on_force_rte) di emu/ssi.py. ---- */
static int ssi_deliver(void *uc, ssi_state *s)
{
    irq_ctx *q = s->q;
    int lvl, ipl, r;
    if (s->int50_asserted && !s->int50_delivered) {
        lvl = irq_level(uc, q, s->tx_vec, 1);
        if (lvl < 0) return -1;
        if (lvl) {
            ipl = irq_ipl(uc, q);
            if (ipl < 0) return -1;
            if (ipl < lvl) {
                r = irq_raise(uc, q, s->tx_vec, lvl);
                if (r < 0) return -1;
                if (r) { s->int50_delivered = 1; s->vector170++; }
            }
        }
    }
    if (s->force_asserted && !s->force_delivered) {
        lvl = irq_level(uc, q, s->force_vec, 0);
        if (lvl < 0) return -1;
        if (lvl) {
            ipl = irq_ipl(uc, q);
            if (ipl < 0) return -1;
            if (ipl < lvl) {
                r = irq_raise(uc, q, s->force_vec, lvl);
                if (r < 0) return -1;
                if (r) { s->force_delivered = 1; s->vector191++; }
            }
        }
    }
    return 0;
}

/* Ssi0Dma.service: le richieste scadute e poi le consegne. -> 0, -1 errore */
int dt2_ssi_service_full(void *uc, ssi_state *s, int64_t done)
{
    if (dt2_ssi_service(uc, s, done) != 0)
        return -1;
    return ssi_deliver(uc, s);
}

/* Ssi0Dma._on_cint: hook di scrittura su CINT (il gestore del vettore 170) */
void dt2_ssi_on_cint(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    ssi_state *s = (ssi_state *)user_data;
    (void)uc; (void)type; (void)address;
    if (size == 1 && ((value & 0x40) || (value & 0x3F) == 50)) {
        s->int50_asserted = 0;
        s->int50_delivered = 0;
    }
}

/* Ssi0Dma._on_intfrch1: hook di scrittura su INTFRCH1 (prima della scrittura) */
void dt2_ssi_on_intfrch1(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    ssi_state *s = (ssi_state *)user_data;
    uint8_t b[4];
    int off = (int)(address - 0xFC04C010u), k, asserted;
    (void)type;
    if (hread(uc, s->q->exc, 0xFC04C010u, b, 4)) { s->err = 10; return; }
    for (k = 0; k < size && off + k < 4; k++)
        if (off + k >= 0)
            b[off + k] = (uint8_t)((uint64_t)value >> (8 * (size - 1 - k)));
    asserted = (be32(b) & 0x80000000u) != 0;
    if (asserted && !s->force_asserted)
        s->force_delivered = 0;
    s->force_asserted = asserted;
    if (!asserted)
        s->force_delivered = 0;
}

/* Ssi0Dma._on_force_rte: all'RTE del gestore del canale 50 il vettore 191
   forzato entra se l'IPL che l'RTE ripristina lo consente (via dt2_gate cfn). */
void dt2_ssi_force_rte(void *uc, uint64_t address, uint32_t size, void *data)
{
    ssi_state *s = (ssi_state *)data;
    irq_ctx *q = s->q;
    uint32_t sp = 0;
    uint8_t b[2];
    int lvl, r;
    (void)address; (void)size;
    if (!s->force_asserted || s->force_delivered)
        return;
    lvl = irq_level(uc, q, s->force_vec, 0);
    if (lvl <= 0)
        return;
    if (q->exc->reg_read(uc, q->exc->r_a7, &sp) || hread(uc, q->exc, sp + 2u, b, 2)) {
        q->err = 23;
        return;
    }
    if ((int)((rbe16(b) >> 8) & 7u) < lvl) {
        r = irq_raise(uc, q, s->force_vec, lvl);
        if (r == 1) { s->force_delivered = 1; s->vector191++; }
    }
}

/* ---- Interrupt forzati (emu/intfrc.py ForcedIrqs) in C ----
   inflight e skip: un bit per sorgente di ciascun INTC; delle sorgenti in
   attesa basta la massima (livello, vettore), l'unica che si consegna. */
typedef struct {
    irq_ctx *q;
    uint64_t inflight[3], skip[3];
    int32_t dirty, has_pend, best_lvl, best_vec;
    int64_t now, at_rte, retry;
    int64_t delivered[192];             /* indice: vettore - 64 */
    int32_t err, pad;
} frc_state;

static int frc_asserted(void *uc, frc_state *f)       /* ForcedIrqs._asserted */
{
    uint8_t b[8];
    int i, src;
    if (!f->dirty)
        return f->has_pend;
    f->dirty = 0;
    f->has_pend = 0; f->best_lvl = 0; f->best_vec = 0;
    for (i = 0; i < 3; i++) {
        uint32_t hi, lo;
        if (hread(uc, f->q->exc, INTC_BASE[i] + 0x10u, b, 8)) { f->err = 30; return -1; }
        hi = be32(b); lo = be32(b + 4);
        if (!(hi | lo))
            continue;
        for (src = 0; src < 64; src++) {
            uint32_t bit = src >= 32 ? (hi >> (src - 32)) & 1u : (lo >> src) & 1u;
            int vec = 64 + 64 * i + src, lvl;
            if (!bit || ((f->skip[i] >> src) & 1u) || ((f->inflight[i] >> src) & 1u))
                continue;
            if (hread(uc, f->q->exc, INTC_BASE[i] + 0x40u + (uint32_t)src, b, 1)) { f->err = 30; return -1; }
            lvl = b[0] & 7;
            if (!lvl)
                continue;
            if (!f->has_pend || lvl > f->best_lvl || (lvl == f->best_lvl && vec > f->best_vec)) {
                f->best_lvl = lvl;
                f->best_vec = vec;
            }
            f->has_pend = 1;
        }
    }
    return f->has_pend;
}

static void frc_took(frc_state *f, int vec, int at_rte)
{
    int i = (vec - 64) / 64, src = vec - 64 - 64 * i;
    f->inflight[i] |= 1ull << src;
    f->dirty = 1;
    f->delivered[vec - 64]++;
    if (at_rte)
        f->at_rte++;
}

/* ForcedIrqs.step: retry se c'e' qualcosa in attesa, -1 None, -2 errore */
int64_t dt2_frc_step(void *uc, frc_state *f)
{
    int a = frc_asserted(uc, f);
    if (a < 0) return -2;
    return a ? f->retry : -1;
}

/* ForcedIrqs.service -> 0, -1 errore */
int dt2_frc_service(void *uc, frc_state *f, int64_t done)
{
    int a, ipl, r, lvl, vec;
    f->now = done;
    a = frc_asserted(uc, f);
    if (a <= 0)
        return a;
    lvl = f->best_lvl; vec = f->best_vec;
    ipl = irq_ipl(uc, f->q);
    if (ipl < 0) return -1;
    if (lvl <= ipl && lvl != 7)
        return 0;
    r = irq_raise(uc, f->q, vec, lvl);
    if (r < 0) return -1;
    if (r) frc_took(f, vec, 0);
    return 0;
}

/* ForcedIrqs._on_write: hook di scrittura su INTFRCH/INTFRCL (prima della scrittura) */
void dt2_frc_on_write(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    frc_state *f = (frc_state *)user_data;
    uint8_t b[8];
    uint32_t hi, lo;
    uint64_t m;
    int i, off, k, src;
    (void)type;
    for (i = 0; i < 3; i++)
        if (address >= INTC_BASE[i] + 0x10u && address < INTC_BASE[i] + 0x18u)
            break;
    if (i == 3)
        return;
    if (hread(uc, f->q->exc, INTC_BASE[i] + 0x10u, b, 8)) { f->err = 30; return; }
    off = (int)(address - INTC_BASE[i] - 0x10u);
    for (k = 0; k < size && off + k < 8; k++)
        b[off + k] = (uint8_t)((uint64_t)value >> (8 * (size - 1 - k)));
    hi = be32(b); lo = be32(b + 4);
    f->dirty = 1;
    m = f->inflight[i];
    for (src = 0; src < 64; src++) {
        if (!((m >> src) & 1u))
            continue;
        if (!(src >= 32 ? (hi >> (src - 32)) & 1u : (lo >> src) & 1u))
            f->inflight[i] &= ~(1ull << src);
    }
}

/* ForcedIrqs._on_rte (via dt2_gate cfn, sugli RTE dell'immagine) */
void dt2_frc_on_rte(void *uc, uint64_t address, uint32_t size, void *data)
{
    frc_state *f = (frc_state *)data;
    irq_ctx *q = f->q;
    uint32_t pc = 0, sp = 0;
    uint8_t b[2];
    int a, lvl, vec, r;
    (void)size;
    a = frc_asserted(uc, f);
    if (a <= 0)
        return;
    if (q->exc->reg_read(uc, q->exc->r_pc, &pc) || pc != (uint32_t)address)
        return;
    if (q->exc->reg_read(uc, q->exc->r_a7, &sp) || hread(uc, q->exc, sp + 2u, b, 2)) {
        q->err = 23;
        return;
    }
    lvl = f->best_lvl; vec = f->best_vec;
    if (lvl <= (int)((rbe16(b) >> 8) & 7u))
        return;
    r = irq_raise(uc, q, vec, lvl);
    if (r == 1)
        frc_took(f, vec, 1);
}

/* ---- gated code hook: Python solo quando il modello ha qualcosa da fare ----
   Il modello tiene `open` aggiornato (una proprieta' dell'oggetto Python); con
   `open` == 0 l'hook costa una chiamata C invece di un giro nel Python. `cfn`
   (emu/nativespin.py): l'hook del modello e' in C e si chiama al posto di `py`,
   cosi' l'hook tiene il suo posto nell'elenco di Unicorn (stesso ordine). */
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

/* ---- FF1 (harness.install_isa_patches_scoped, make_ff1) in C ----
   Stesso effetto del Python: il registro prende il numero di zeri in testa
   (32 per 0), il PC salta l'istruzione, il contatore sale. */
typedef struct {
    reg_rw_fn reg_read, reg_write;
    int32_t reg, r_pc;
    int64_t *count;
} ff1_state;

void dt2_ff1(void *uc, uint64_t address, uint32_t size, void *user_data)
{
    ff1_state *f = (ff1_state *)user_data;
    uint32_t v = 0, out, pc = (uint32_t)address + 2u;
    (void)size;
    f->reg_read(uc, f->reg, &v);
    out = v ? (uint32_t)__builtin_clz(v) : 32u;
    f->reg_write(uc, f->reg, &out);
    f->reg_write(uc, f->r_pc, &pc);
    (*f->count)++;
}

/* ---- il ciclo ---- */
enum { SPIN_LIMIT = 0, SPIN_PC_ZERO = 1, SPIN_STOPPED = 2, SPIN_UC_ERROR = 3 };
enum { EV_SSI = 1, EV_FRC = 2, EV_PY = 3 };
#define SPIN_MAXEV 8

typedef struct {
    emu_start_fn emu_start;
    irq_ctx *q;
    step_state *stp;                    /* lo stepper nativo: {left, blocks} */
    double per_block;
    int32_t ntim, nev;
    tim_state *tim[4];                  /* Timers.sources, in ordine */
    int32_t ev[SPIN_MAXEV];             /* async_events, in ordine: EV_* */
    ssi_state *ssi;
    frc_state *frc;
    /* eventi Python (EV_PY): occupati se *hint o *extra; altrimenti step None e
       service senza effetto, e il C non li chiama */
    const int32_t *hint[SPIN_MAXEV], *extra[SPIN_MAXEV];
    py_step_fn py_step[SPIN_MAXEV];
    py_service_fn py_service[SPIN_MAXEV];
    const int32_t *stop_flag;           /* 1: un hook Python ha chiamato uc.emu_stop() */
    int64_t base, done, instrs, now, executed, steps, py_calls;
    uint32_t pc;
    int32_t uc_err, err, timing;
    int64_t emu_ns;                     /* timing != 0: tempo dentro emu_start (diagnostica) */
} spin_state;

/* orologio monotono per `timing` */
#ifdef _WIN32
#include <windows.h>
static int64_t spin_clock_ns(void)
{
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (int64_t)((double)c.QuadPart * 1e9 / (double)f.QuadPart);
}
#else
#include <time.h>
static int64_t spin_clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}
#endif

/* CPython float_floor_div (Objects/floatobject.c, _float_div_mod): `a // b`
   tra float, esattamente come lo fa Python (non e' floor(a / b)). */
static double py_floordiv(double vx, double wx)
{
    double mod = fmod(vx, wx), div = (vx - mod) / wx, floordiv;
    if (mod) {
        if ((wx < 0) != (mod < 0)) { mod += wx; div -= 1.0; }
    }
    if (div) {
        floordiv = floor(div);
        if (div - floordiv > 0.5)
            floordiv += 1.0;
    } else
        floordiv = copysign(0.0, vx / wx);
    return floordiv;
}

/* NativeStepper.run: left = max(1, -int(-step // per_block)) */
int64_t dt2_spin_left(int64_t step, double per_block)
{
    int64_t left = -(int64_t)py_floordiv(-(double)step, per_block);
    return left < 1 ? 1 : left;
}

static int py_busy(spin_state *S, int i)
{
    return (S->hint[i] && *S->hint[i]) || (S->extra[i] && *S->extra[i]);
}

/* spin: `for event in async_events: event.service(); pits.service()` */
static int spin_services(void *uc, spin_state *S, int64_t now)
{
    int i;
    for (i = 0; i < S->nev; i++) {
        switch (S->ev[i]) {
        case EV_SSI:
            if (dt2_ssi_service_full(uc, S->ssi, now) < 0) return -1;
            break;
        case EV_FRC:
            if (dt2_frc_service(uc, S->frc, now) < 0) return -1;
            break;
        case EV_PY:
            if (py_busy(S, i)) {
                S->py_calls++;
                if (S->py_service[i](now) < 0) { S->err = 40; return -1; }
            }
            break;
        }
    }
    for (i = 0; i < S->ntim; i++)
        if (dt2_tim_service_full(uc, S->tim[i], now) < 0) { S->err = 41; return -1; }
    return 0;
}

static int spin_hook_error(spin_state *S)
{
    return S->q->err || (S->ssi && S->ssi->err) || (S->frc && S->frc->err);
}

/* I servizi di un passo gia' eseguito (S->done aggiornato da chi chiama),
   poi il PC: la coda di un'iterazione che il Python ha ripreso (SPIN_STOPPED). */
int dt2_spin_service(void *uc, spin_state *S)
{
    S->now = S->base + S->done;
    if (spin_services(uc, S, S->now) < 0)
        return -1;
    if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &S->pc)) { S->err = 42; return -1; }
    return 0;
}

/* longrun.spin(m, pc, instrs, pits=timers, fast=True, async_events=...):
   lo stesso ciclo, gli stessi passi. Esce con SPIN_* (o -1, errore in
   S->err / q->err / ssi->err / frc->err / tim->err). */
int dt2_spin(void *uc, spin_state *S)
{
    uint32_t pc = S->pc;
    while (S->done < S->instrs) {
        int64_t now, step = -1, r;
        int i, err;
        if (pc == 0) { S->pc = 0; return SPIN_PC_ZERO; }
        now = S->base + S->done;
        S->now = now;
        /* steps = [pits.step(now)] + [e.step(now) for e in async_events if not None] */
        for (i = 0; i < S->ntim; i++) {
            r = dt2_tim_step(uc, S->tim[i], now, -1);
            if (r < 0) { S->err = 43; return -1; }
            if (step < 0 || r < step) step = r;
        }
        for (i = 0; i < S->nev; i++) {
            r = -1;
            switch (S->ev[i]) {
            case EV_SSI:
                r = dt2_ssi_step(uc, S->ssi, now, -1);
                if (S->ssi->err) return -1;
                if (!S->ssi->en) r = -1;
                break;
            case EV_FRC:
                r = dt2_frc_step(uc, S->frc);
                if (r == -2) return -1;
                break;
            case EV_PY:
                if (py_busy(S, i)) {
                    S->py_calls++;
                    r = S->py_step[i](now);
                    if (r == -2) { S->err = 40; return -1; }
                }
                break;
            }
            if (r >= 0 && r < step) step = r;
        }
        /* NativeStepper.run(pc, step) */
        S->stp->blocks = 0;
        S->stp->left = dt2_spin_left(step, S->per_block);
        if (S->timing) {
            int64_t t0 = spin_clock_ns();
            err = S->emu_start(uc, pc, 0, 0, 0);
            S->emu_ns += spin_clock_ns() - t0;
        } else
            err = S->emu_start(uc, pc, 0, 0, 0);
        S->steps++;
        S->executed = (int64_t)((double)S->stp->blocks * S->per_block);
        if (S->executed < 1) S->executed = 1;
        if (err) { S->uc_err = err; S->pc = pc; return SPIN_UC_ERROR; }
        if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &pc)) { S->err = 42; return -1; }
        if (*S->stop_flag) { S->pc = pc; return SPIN_STOPPED; }
        if (spin_hook_error(S)) { S->pc = pc; return -1; }
        if (pc == 0) { S->pc = 0; return SPIN_PC_ZERO; }
        S->done += S->executed;
        now = S->base + S->done;
        S->now = now;
        if (spin_services(uc, S, now) < 0) { S->pc = pc; return -1; }
        if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &pc)) { S->err = 42; return -1; }
    }
    S->pc = pc;
    return SPIN_LIMIT;
}
