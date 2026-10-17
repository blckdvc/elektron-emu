/* engine.c, parte 1: tipi comuni, utilita', la struct del motore.
 * Incluso da engine.c DOPO native/dt2_hooks.c e native/dt2_spin.c (stessa
 * unita' di traduzione: i tipi tim_state, sst_state, ... sono li').
 * Dal motore del Digitakt II (digitakt/emulator/core/src/e_base.inc.c), con i
 * modelli del Rytm: SSI1, la flash SPI, l'RPC in SysEx, i LED del Rytm, il
 * motore delle voci. */

/* ------------------------------------------------------------ byte buffer */
typedef struct {
    uint8_t *p;
    size_t n, cap;
} e_buf;

static int e_buf_put(e_buf *b, const void *data, size_t n)
{
    if (b->n + n > b->cap) {
        size_t cap = b->cap ? b->cap : 4096;
        uint8_t *q;
        while (cap < b->n + n)
            cap *= 2;
        q = (uint8_t *)realloc(b->p, cap);
        if (!q)
            return -1;
        b->p = q;
        b->cap = cap;
    }
    memcpy(b->p + b->n, data, n);
    b->n += n;
    return 0;
}

static void e_buf_drop(e_buf *b, size_t n)          /* toglie n byte in testa */
{
    if (n >= b->n) {
        b->n = 0;
        return;
    }
    memmove(b->p, b->p + n, b->n - n);
    b->n -= n;
}

/* coda di byte (deque di Python) */
typedef struct {
    uint8_t *p;
    size_t head, n, cap;
} e_fifo;

static int e_fifo_push(e_fifo *f, const uint8_t *data, size_t n)
{
    size_t i;
    if (f->n + n > f->cap) {
        size_t cap = f->cap ? f->cap : 1024, k;
        uint8_t *q;
        while (cap < f->n + n)
            cap *= 2;
        q = (uint8_t *)malloc(cap);
        if (!q)
            return -1;
        for (k = 0; k < f->n; k++)
            q[k] = f->p[(f->head + k) % f->cap];
        free(f->p);
        f->p = q;
        f->cap = cap;
        f->head = 0;
    }
    for (i = 0; i < n; i++)
        f->p[(f->head + f->n + i) % f->cap] = data[i];
    f->n += n;
    return 0;
}

static int e_fifo_pop(e_fifo *f)
{
    int v;
    if (!f->n)
        return -1;
    v = f->p[f->head];
    f->head = (f->head + 1) % f->cap;
    f->n--;
    return v;
}

/* --------------------------------------------------------------- big-endian */
static inline uint32_t e_be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static inline uint16_t e_be16(const uint8_t *p)
{
    return (uint16_t)(p[0] << 8 | p[1]);
}

static inline void e_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

static inline void e_put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}

static inline uint32_t e_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static inline uint64_t e_le64(const uint8_t *p)
{
    return (uint64_t)e_le32(p) | (uint64_t)e_le32(p + 4) << 32;
}

/* zlib.crc32 incrementale */
static uint32_t e_crc(uint32_t crc, const void *p, size_t n)
{
    return (uint32_t)crc32(crc, (const Bytef *)p, (uInt)n);
}

/* floor division di Python fra interi (anche negativi) */
static inline __int128 e_floordiv128(__int128 a, __int128 b)
{
    __int128 q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        q--;
    return q;
}

/* ------------------------------------------------------------- modelli */
/* emu/edma.py TxChannel */
typedef struct {
    uint32_t chan, vector, tcd;
    int32_t pending;              /* la "cella" ctypes di Python: idle_state.tx_pending */
    int64_t bytes, transfers;
    int restored;
    e_buf *out;                   /* dove vanno i byte (uart_out, tx_out del MIDI) */
    uint64_t *out_total;          /* conteggio totale (len(bytearray) di Python) */
    uint32_t *out_crc;            /* crc32 di tutto il bytearray */
} e_txch;

/* emu/pit.py Pits / emu/dtim.py Dtims: la parte Python intorno a tim_state */
typedef struct {
    tim_state t;                  /* next/has/pend/fired/missed/merged: qui */
    int kind;                     /* 0 Pits, 1 Dtims */
    int64_t ips;                  /* _ips */
    int64_t now;
    uint32_t arm, stale;          /* Dtims */
} e_tsrc;

/* emu/esdhc.py: la carta, overlay a settori */
typedef struct {
    uint64_t sec;
    uint8_t *data;                /* 512 */
    uint8_t *mask;                /* 512 o NULL (settore intero) */
    int32_t present;              /* byte presenti se mask */
    int32_t used;
} e_sector;

typedef struct {
    e_sector *tab;                /* hash aperto su sec */
    size_t cap, n;
} e_overlay;

typedef struct {
    uint32_t drv_status, cmd_sem, data_sem, dma_sem;
    uint32_t pattern;
    int32_t armed;                /* -1 = None */
    int64_t dma_bytes;
    int64_t log_n;
    /* Card */
    int64_t blocks;
    uint8_t ext_csd[512];
    uint32_t rca, ocr;
    int32_t selected;
    uint32_t cid[4], csd[4];
    e_overlay ov;
} e_esdhc;

/* emu/midi.py MidiPort (il Rytm: i byte entrano tutti insieme, senza il
   ritmo del DIN; tx_listeners: il collegamento RPC) */
typedef struct {
    e_fifo rx;
    int64_t rx_bytes;
    e_buf tx_out;
    uint64_t tx_out_total;
    uint32_t tx_out_crc;
    e_txch tx;
    int64_t tx_sent;
    uint8_t uimr;
    int64_t uart_irqs;
    int32_t uart_masked;
    int32_t hint;
    e_buf host_out;               /* byte verso l'host (dt_engine_midi_out) */
    uint32_t ring_base_ptr;       /* dal gestore RX (midi._find_rx_handler) */
    int installed;
} e_midi;

/* ===================================================== il Rytm */
#define AR_BLOCK 32                       /* rytmsynth.BLOCK: frame per tick di scambio */
#define AR_TXCAP (1 << 20)                /* nativespin.TXCAP */
#define AR_CHUNKCAP 8192                  /* nativespin.CHUNKCAP */

/* emu/rytm_leds.py RytmLedState */
typedef struct {
    int16_t regs[4][128];         /* -1 = None (mai ricevuto) */
    uint8_t state[128];           /* 2x: il registro mostrato */
    uint8_t pads[12][3];
    int64_t version, unknown;
    uint8_t buf[16];
    size_t nbuf;
} e_rleds;

/* emu/rytm_nor.py Nor: la flash SPI, blocchi da 256 KB scritti (overlay) */
#define NOR_BLOCK 0x40000u
#define NOR_MAXBLK 64
typedef struct {
    uint8_t *blk[NOR_MAXBLK];     /* NULL = dall'immagine di base (e->flash) */
    int64_t reads, writes, erases;
    int installed;
} e_nor;

/* emu/rytm_rpc.py SysexRpcLink */
typedef struct {
    int64_t inflight;             /* richieste nostre in volo (il porto diventa USB) */
    int64_t seq;
    e_buf rx;                     /* i byte del flush della porta 2 */
    e_buf replies;                /* [u16 seq, u8 id, u32 n, n byte] per dt_engine_rpc_replies */
    int64_t entries, flushes, nreplies;
    int installed;
} e_rpc;

/* emu/rytmsynth.py RytmSynth + SampleTap, intorno a native/rytm_dsp.c */
typedef struct {
    void *dsp;                    /* dsp_t (rytm_size() byte) */
    int64_t frames;               /* RytmSynth.frames: tick di scambio */
    int64_t triggers;             /* len(RytmSynth.triggers) */
    uint16_t prev[32];            /* le parole 84..115 del frame precedente */
    int has_prev;
    float fx[32];                 /* Dsp.fx: i valori dell'ultimo set_fx */
    float pbuf[32];               /* Dsp.pbuf: l'ultimo vettore di trigger */
    /* SampleTap */
    int64_t *chunk_end, *chunk_src;   /* i record dei pezzi dell'SSI (sst_state) */
    e_buf tap_data;               /* SampleTap.chunks: i byte ... */
    int64_t *tap_end, *tap_src;   /* ... e dove finisce/comincia ogni pezzo */
    size_t tap_n, tap_cap;
    float acc[AR_BLOCK][16];      /* SampleTap.acc: righe da 16 slot */
    int nacc;
    int64_t tap_blocks, tap_odd;
    int tap_on;
    /* uscita */
    float out[2 * AR_BLOCK];
    float voices[8 * 2 * AR_BLOCK];  /* le 8 voci, prima degli effetti (uscite separate) */
    float master;                 /* rytmsynth.MASTER */
    int64_t samples;              /* frame resi */
    uint32_t crc;                 /* crc32 dei blocchi float32 LE */
    void (*sink)(void *ctx, const float *lr, const float *voices, size_t frames);
    void *sink_ctx;
} e_synth;

/* Emulator._drain_input: la coda dei comandi del pannello */
typedef struct {
    int kind;                     /* DT_PANEL_*; 5 = byte grezzi */
    int code, arg;
    uint8_t raw[8];
    int nraw;
} e_input;

typedef struct {
    dt_engine *e;
    int op;                       /* 0 mul 1 sub 2 add 3 div 4 abs 5 fix 6 cmp */
} e_sf;

/* la calibrazione in corso (e_calib.inc.c); NULL fuori da dt_engine_calibrate */
typedef struct e_cal e_cal;

/* il motore */
struct dt_engine {
    const dt_firmware *fw;
    const dt_section *main_os;
    dt_machine *m;
    uc_engine *uc;
    const dt_profile *p;
    uint8_t *flash;               /* dspboot.build_flash: 16 MB, container a 0x80000 */
    size_t flash_len;
    char err[256];

    /* stato nativo (le struct di native/) */
    step_state stp;
    exc_state exc;
    irq_ctx q;
    spin_state S;
    e_tsrc tim[2];                /* Timers(Pits, Dtims) */
    int ntim;
    sst_state sst;
    uint8_t *txbuf;               /* il TX dell'SSI1 (tx_sink) */
    int64_t ssi_rh;
    uint32_t ssi_tx_crc;          /* Ssi0Dma.tx_crc32 (dei pezzi consegnati) */
    frc_state frc;
    dt2_gate frc_gate, ssi_gate;
    idle_state idle;
    memw_state dspw;
    dtcn_state dtcn;
    dtcn_site dtcn_sites[4];
    void *dma;                    /* dt2_dma.c dma_state */

    /* longrun.build */
    e_fifo uart_in;               /* inq */
    e_buf uart_out;               /* byte non ancora dati ai LED */
    uint64_t uart_out_total;
    uint32_t uart_out_crc;
    e_txch etx;                   /* eDMA 35 */
    int64_t depack_clamps;
    int64_t setpixel_calls, prints, task_creates, switches;
    int64_t sf_calls, sf_deferred, bmp_set, bmp_get;
    e_sf sfs[7];

    /* modelli */
    e_esdhc sd;
    e_midi midi;
    e_nor nor;
    e_rpc rpc;
    e_synth syn;
    uint32_t idle_addrs[32];
    int nidle;
    int running;                  /* dentro una chiamata di spin (stepper.running) */
    uint32_t dtim_arm;            /* Dtims.arm (maschera dei canali) */

    /* GUI (emu/gui.py Emulator) */
    int64_t instrs;               /* stats['instrs'] */
    uint32_t pc;
    int halted;
    char status[128];
    int64_t pending_ips_at;       /* _pending_ips: un solo elemento (when, n) */
    int64_t pending_ips_n;
    int has_pending_ips;
    int64_t jobs;
    int terminal;
    e_rleds leds;
    uint8_t latch[1024];
    int has_latch;
    uint8_t last_panel[1024];
    int has_last_panel;
    int panel_live;
    uint8_t fb[128 * 64];
    int64_t frames;
    uint32_t fb_crc;
    uint64_t panel_version;
    /* input del pannello */
    e_input *inbox;
    size_t inbox_head, inbox_n, inbox_cap;
    uint8_t held[16];
    int dwell_chunks, chunks_since_delivery, delivered_before;
    int64_t chunk;
    int boot_only;                /* il boot freddo: salva solo l'eSDHC (come checkpoint.make) */
    e_cal *cal;                  /* i circuiti degli ADC durante la calibrazione */
};

/* il motore del thread che sta eseguendo (le callback del C nativo non hanno contesto) */
static _Thread_local dt_engine *e_cur;

#define E_UC(e) ((e)->uc)

static uint32_t e_rd(dt_engine *e, int reg)
{
    return dtm_rd32(e->uc, reg);
}

static void e_wr(dt_engine *e, int reg, uint32_t v)
{
    dtm_wr32(e->uc, reg, v);
}

static int e_read(dt_engine *e, uint64_t a, void *b, size_t n)
{
    return uc_mem_read(e->uc, a, b, n) == UC_ERR_OK ? 0 : -1;
}

static int e_write(dt_engine *e, uint64_t a, const void *b, size_t n)
{
    return uc_mem_write(e->uc, a, b, n) == UC_ERR_OK ? 0 : -1;
}

static uint32_t e_r32(dt_engine *e, uint64_t a)
{
    uint8_t b[4] = {0, 0, 0, 0};
    e_read(e, a, b, 4);
    return e_be32(b);
}

static uint16_t e_r16(dt_engine *e, uint64_t a)
{
    uint8_t b[2] = {0, 0};
    e_read(e, a, b, 2);
    return e_be16(b);
}

static uint8_t e_r8(dt_engine *e, uint64_t a)
{
    uint8_t b = 0;
    e_read(e, a, &b, 1);
    return b;
}

static void e_w32(dt_engine *e, uint64_t a, uint32_t v)
{
    uint8_t b[4];
    e_put32(b, v);
    e_write(e, a, b, 4);
}

static void e_w16(dt_engine *e, uint64_t a, uint16_t v)
{
    uint8_t b[2];
    e_put16(b, v);
    e_write(e, a, b, 2);
}

static void e_w8(dt_engine *e, uint64_t a, uint8_t v)
{
    e_write(e, a, &v, 1);
}

static uc_hook e_hook_code(dt_engine *e, void *fn, void *ud, uint32_t addr)
{
    uc_hook h = 0;
    uc_hook_add(e->uc, &h, UC_HOOK_CODE, fn, ud, addr, addr);
    return h;
}

static uc_hook e_hook_mem(dt_engine *e, int type, void *fn, void *ud, uint64_t begin, uint64_t end)
{
    uc_hook h = 0;
    uc_hook_add(e->uc, &h, type, fn, ud, begin, end);
    return h;
}

/* emu/pit.py interrupt_level: 0 = None */
static int e_irq_level(dt_engine *e, int vec, int respect_mask)
{
    static const uint32_t base[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
    int i, src, icr;
    uint8_t b[8];
    if (vec < 64 || vec >= 256)
        return 0;
    i = (vec - 64) / 64;
    src = vec - 64 - 64 * i;
    icr = e_r8(e, base[i] + 0x40u + (uint32_t)src) & 7;
    if (!icr || !respect_mask)
        return icr;
    e_read(e, base[i] + 0x08u, b, 8);
    {
        uint32_t hi = e_be32(b), lo = e_be32(b + 4);
        uint32_t masked = src < 32 ? (lo >> src) & 1u : (hi >> (src - 32)) & 1u;
        return masked ? 0 : icr;
    }
}

static int e_ipl(dt_engine *e)
{
    return (int)((e_rd(e, UC_M68K_REG_SR) >> 8) & 7u);
}

static int e_raise(dt_engine *e, int vec, int level)
{
    return dtm_raise_vector(e->m, (uint32_t)vec, 0, level);
}
