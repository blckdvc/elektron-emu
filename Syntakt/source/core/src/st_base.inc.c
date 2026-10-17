/* st_engine.c, parte 1: tipi, utilita' e la struct del motore della Syntakt.
 * Incluso DOPO native/st_hooks.c, st_spin.c, st_cosim.c (stessa unita' di
 * traduzione: tim_state, sst_state, frc_state, spin_state, cz_state...).
 *
 * Le utilita' (buffer, code, big endian, crc) e i modelli comuni col Digitakt
 * II (edma, softfloat, bitmap, UART8, eSDHC, MIDI, pannello) vengono da
 * digitakt/emulator/core/src (e_*.inc.c), adattati: il Python della Syntakt e'
 * lo stesso di quello del Digitakt per quei moduli, con le differenze
 * annotate dove ci sono (docs/AU-PORT.md). */

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
    if (n)
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

/* emu/midi.py MidiPort */
typedef struct {
    e_fifo rx;
    int64_t rx_bytes;
    int64_t ips;                  /* 0 = None */
    int32_t has_rx_at;
    double rx_at;
    int64_t now;
    e_buf tx_out;
    uint64_t tx_out_total;
    uint32_t tx_out_crc;
    e_txch tx;
    int64_t tx_sent;
    uint8_t uimr;
    int64_t uart_irqs;
    int32_t uart_masked;
    int32_t hint;
    uint32_t ring_base_ptr;       /* _find_rx_handler: l'operando del movea.l a +0x26 */
    e_buf host_out;               /* byte verso l'host (dt_engine_midi_out) */
    int installed;
} e_midi;

/* emu/st_leds.py StLedState */
#define ST_NLEDS 64
typedef struct {
    int16_t regs[4][ST_NLEDS];    /* -1 = None */
    uint8_t state[ST_NLEDS];
    int64_t version, unknown;
    uint8_t buf[16];
    size_t nbuf;
} e_stleds;

/* Emulator._drain_input: la coda dei comandi del pannello */
typedef struct {
    int kind;                     /* 0 press, 1 release, 2 release_all, 3 encoder */
    int code, arg;
} e_input;

typedef struct {
    dt_engine *e;
    int op;                       /* 0 mul 1 sub 2 add 3 div 4 abs 5 fix 6 cmp */
} e_sf;

/* una delle due CPU: la macchina, il suo ciclo dei passi e i suoi timer */
typedef struct st_cpu st_cpu;

typedef struct {
    st_cpu *c;
    int ch;
} e_dtcn;

struct st_cpu {
    dt_engine *e;
    int dsp;                      /* 0 MAIN, 1 DSP */
    dt_machine *m;
    uc_engine *uc;
    dt_firmware fwview;           /* DSP: la sezione 7 come "sezione 3" per machine.c */
    step_state stp;               /* NativeStepper (tb-budget) */
    exc_state exc;
    irq_ctx q;
    spin_state S;                 /* NativeSpin.S */
    e_tsrc tim[2];                /* Timers(Pits, Dtims) */
    int64_t now;                  /* Timers.now */
    int running;                  /* dentro una chiamata C (stepper.running, _spin_live) */
    /* dtim.install_counters */
    int dtcn_has[4];
    int64_t dtcn_origin[4];
    e_dtcn dtcn_ud[4];
    /* Dtims.arm (dt2_set_bit) */
    int32_t arm_bits;
    setbit_state arm[4];
    void *dma;                    /* emu/nativedma.py (st_dma.c) */
};

/* il motore */
struct dt_engine {
    const dt_firmware *fw;
    const dt_section *main_os, *dsp_os;
    const st_profile *p;
    uint8_t *flash;               /* dspboot.build_flash: 16 MB, container a 0x80000 */
    size_t flash_len;
    char err[256];
    st_cpu cpu[2];                /* 0 MAIN, 1 DSP */

    /* MAIN: SSI1 (emu/ssi.py, cablaggio del Rytm), interrupt forzati, idle */
    sst_state sst;
    frc_state frc;
    dt2_gate frc_gate, ssi_gate;
    idle_state idle;
    uint32_t idle_addrs[32];
    int nidle;

    /* MAIN: longrun.build */
    e_fifo uart_in;               /* inq */
    e_buf uart_out;               /* byte non ancora dati ai LED */
    uint64_t uart_out_total;
    uint32_t uart_out_crc;
    e_txch etx;                   /* eDMA 35 */
    int64_t depack_clamps;
    int64_t setpixel_calls, pxcopy_calls, prints, task_creates;
    int64_t sf_calls, sf_deferred, bmp_set, bmp_get;
    e_sf sfs[7];
    e_esdhc sd;
    e_midi midi;
    int64_t fifo_polls, fifo_words, fifo_bursts;        /* emu/dsp.py Fifo (diagnostica) */
    int64_t fpga_sent, fpga_bad;                        /* emu/fpgaport.py */
    sync_state sync;                                    /* la linea A4 (DspCompanion.use_main_sync) */

    /* emu/spilink.py SpiLink */
    e_fifo link_main_tx, link_pending, link_dsp_rx;
    int32_t link_complete;
    int link_dma_active;
    uint32_t link_saddr, link_n;
    int64_t link_dma_done, link_sent, link_received;
    uint32_t link_sent_crc, link_received_crc;

    /* DSP (emu/dspcpu.py) */
    int32_t park;                 /* m.park_flag */
    uint32_t audio_idle, idle_wait, idle_flag;
    int64_t dtin0_edges, last_edges, irq4, idle_skipped;
    e_buf fpga_config;            /* m.fpga_config (le ultime 54666 parole si salvano) */
    uint64_t fpga_config_total;
    int64_t bzero_calls;
    cz_state Z;
    uint8_t *mailbox;             /* la RAM condivisa (cosim.share): 1 MB, MAIN 0x10000000, DSP 0 */

    /* GUI (emu/gui.py Emulator) */
    int64_t instrs;               /* stats['instrs'] */
    uint32_t pc, dpc;
    int halted;
    char status[160];
    int64_t mainloop, jobs;
    int terminal;
    e_stleds leds;
    uint8_t latch[1024];
    int has_latch;
    uint8_t last_panel[1024];
    int has_last_panel;
    int panel_live;
    uint8_t fb[128 * 64];
    int64_t frames;
    uint32_t fb_crc;
    uint64_t panel_version;
    e_input *inbox;
    size_t inbox_head, inbox_n, inbox_cap;
    uint8_t held[16];
    int dwell_chunks, chunks_since_delivery, delivered_before;
    int64_t chunk;
    int64_t cz_calls, cz_main_exits, cz_dsp_exits, cz_link_exits;
#if !defined(_WIN32)
    pthread_t creator;            /* il thread che ha creato il motore (st_tune_thread) */
#endif
    int rt_tuned;
    int cold;                     /* motore della catena del boot freddo (st_boot.inc.c) */
    int cold_hold;                /* motore freddo: il DSP aspetta la risposta del MAIN al suo saluto */
    int64_t cold_hold_n;
    int legacy_ips;               /* stato con la cadenza dei timer di 4,68M, da portare a 150M */

    /* audio: il TX dell'SSI1 (emu/staudio.py TxOut) */
    uint8_t tx_pend[32];
    size_t tx_npend;
    uint64_t tx_total;            /* byte TX consegnati al TxOut */
    uint32_t tx_crc;
    uint32_t tx_crc32_saved;      /* Ssi0Dma.tx_crc32 dello stato (fermo nel modello in C) */
    float volume;                 /* staudio.VOLUME (Main Volume, analogico) */
    int64_t samples;              /* frame stereo prodotti */
    uint32_t audio_crc;           /* crc32 dei blocchi float32 (n, 2) */
    void (*sink)(void *ctx, const float *mix, const float *tracks, size_t n);
    void *sink_ctx;
    float sink_mix[512];
    float sink_tracks[16][512];
    size_t sink_n;
};

/* il motore del thread che sta eseguendo (le callback del C nativo non hanno contesto) */
static _Thread_local dt_engine *e_cur;

static uint32_t c_rd(st_cpu *c, int reg)
{
    return dtm_rd32(c->uc, reg);
}

static void c_wr(st_cpu *c, int reg, uint32_t v)
{
    dtm_wr32(c->uc, reg, v);
}

static int c_read(st_cpu *c, uint64_t a, void *b, size_t n)
{
    return uc_mem_read(c->uc, a, b, n) == UC_ERR_OK ? 0 : -1;
}

static int c_write(st_cpu *c, uint64_t a, const void *b, size_t n)
{
    return uc_mem_write(c->uc, a, b, n) == UC_ERR_OK ? 0 : -1;
}

static uint32_t c_r32(st_cpu *c, uint64_t a)
{
    uint8_t b[4] = {0, 0, 0, 0};
    c_read(c, a, b, 4);
    return e_be32(b);
}

static uint16_t c_r16(st_cpu *c, uint64_t a)
{
    uint8_t b[2] = {0, 0};
    c_read(c, a, b, 2);
    return e_be16(b);
}

static uint8_t c_r8(st_cpu *c, uint64_t a)
{
    uint8_t b = 0;
    c_read(c, a, &b, 1);
    return b;
}

static void c_w32(st_cpu *c, uint64_t a, uint32_t v)
{
    uint8_t b[4];
    e_put32(b, v);
    c_write(c, a, b, 4);
}

static void c_w16(st_cpu *c, uint64_t a, uint16_t v)
{
    uint8_t b[2];
    e_put16(b, v);
    c_write(c, a, b, 2);
}

static void c_w8(st_cpu *c, uint64_t a, uint8_t v)
{
    c_write(c, a, &v, 1);
}

static uc_hook c_hook_code(st_cpu *c, void *fn, void *ud, uint32_t addr)
{
    uc_hook h = 0;
    uc_hook_add(c->uc, &h, UC_HOOK_CODE, fn, ud, addr, addr);
    return h;
}

static uc_hook c_hook_mem(st_cpu *c, int type, void *fn, void *ud, uint64_t begin, uint64_t end)
{
    uc_hook h = 0;
    uc_hook_add(c->uc, &h, type, fn, ud, begin, end);
    return h;
}

/* emu/pit.py interrupt_level: 0 = None */
static int c_irq_level(st_cpu *c, int vec, int respect_mask)
{
    static const uint32_t base[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};
    int i, src, icr;
    uint8_t b[8];
    if (vec < 64 || vec >= 256)
        return 0;
    i = (vec - 64) / 64;
    src = vec - 64 - 64 * i;
    icr = c_r8(c, base[i] + 0x40u + (uint32_t)src) & 7;
    if (!icr || !respect_mask)
        return icr;
    c_read(c, base[i] + 0x08u, b, 8);
    {
        uint32_t hi = e_be32(b), lo = e_be32(b + 4);
        uint32_t masked = src < 32 ? (lo >> src) & 1u : (hi >> (src - 32)) & 1u;
        return masked ? 0 : icr;
    }
}

static int c_ipl(st_cpu *c)
{
    return (int)((c_rd(c, UC_M68K_REG_SR) >> 8) & 7u);
}

/* Machine.raise_vector (senza srtrap); level < 0 = None */
static int c_raise(st_cpu *c, int vec, int level)
{
    return dtm_raise_vector(c->m, (uint32_t)vec, 0, level);
}

#define MAINC(e) (&(e)->cpu[0])
#define DSPC(e) (&(e)->cpu[1])
