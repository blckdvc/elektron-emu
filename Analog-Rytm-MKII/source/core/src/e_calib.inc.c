/* engine.c, parte 8: la calibrazione analogica del Rytm in C (dt_engine_calibrate),
 * riga per riga su tools/calloop.py (e docs/CALIBRAZIONE.md).
 *
 * Il firmware calibra misurando i propri circuiti con gli ADC: manda i suoi
 * controlli nel frame verso l'FPGA e legge, dall'anello RX dell'SSI1
 * (0x80003000: 4 blocchi x 32 frame x 12 slot x 4 byte, campioni a 24 bit
 * allineati a destra), cio' che i circuiti rispondono. Qui i circuiti sono i
 * nostri modelli (`Circuits` del Python: filtri autooscillanti, VCO XD
 * lineari, tom a risonatore), e le tabelle che il firmware ricava sono
 * quelle che rendono i modelli fedeli ai suoi obiettivi. Niente dati del
 * firmware qui dentro: gli indirizzi sono quelli dell'OS 1.73 (profilo) e le
 * tabelle nascono al primo avvio dell'utente, sul suo firmware.
 *
 * Il percorso:
 *   1. il frame (SERQ del canale 15, syn_on_serq) va ai circuiti e non piu'
 *      alle voci; il peer RX dell'SSI (dt2_sst_rx_fill, native/dt2_hooks.c)
 *      scrive nell'anello i campioni dei circuiti a ogni richiesta, con la
 *      stessa matematica e la stessa semantica del Python (il record RBOW
 *      che ne esce e' uguale parola per parola a quello di calloop.py);
 *   2. cal_inject(): una chiamata a una funzione del firmware dal contesto di
 *      un task (IPL 0), con ritorno su una trappola `bra.s *`;
 *   3. 0x401096de(modo 0, sel -1) avvia; si lascia girare finche' lo stato
 *      0x42fb4218 arriva a 7; 0x4010957a (SAVING RESULTS) scrive il record
 *      RBOW nella flash SPI (h_nor_write) e chiama la pulizia 0x401090f6
 *      (vettore 191, USB e modo audio rimessi);
 *   4. si toglie tutto: i hook, il peer, il frame torna alle voci. */

static int engine_spin(dt_engine *e, int64_t instrs, int64_t *executed);     /* engine.c */

#define CAL_RING 0x80003000u
#define CAL_SLOTS 12
#define CAL_SR 48000.0
#define CAL_FULL 2147483648.0                 /* float(1 << 31) */
#define CAL_TWO_PI (2.0 * 3.14159265358979323846)

#define CAL_START 0x401096DEu
#define CAL_STOP 0x401090F6u
#define CAL_SAVE 0x4010957Au
#define CAL_JOBS 0x40288FB4u
#define CAL_LIST_PTR 0x419C6BDCu              /* la lista dei lavori in uso (modo 0: CAL_JOBS) */
#define CAL_STATE 0x42FB4218u
#define CAL_JOB 0x42FB46D8u
#define CAL_TICK 0x42FB46D4u
#define CAL_USBCMD 0xFC0B01B4u
#define CAL_RECORD 0x4196E0A8u                /* copia in RAM del record RBOW */
#define CAL_ERRORS 0x42FB4224u                /* i record di errore: 14 x 0x3C byte */
#define CAL_TOM_STRIKE 0x401063B6u            /* TOMS: il colpo nel buffer del DAC */
#define CAL_TOM_JOB 0x40105EC8u               /* TOMS: la funzione di misura */
#define CAL_SENTINEL 0x4F300000u              /* `bra.s *`: dove ritornano le chiamate iniettate */
#define CAL_NJOBS 10                          /* i lavori del modo 0 (SIGNALS e RMS non girano) */
/* ritardo massimo (istruzioni) con cui l'SSI riprova a consegnare la ISR del tick quando e' mascherata: gli 8000 del
   motore normale (6,4 frame) fanno leggere al firmware blocchi a meta' e la bisezione dei tom sbaglia un passo ogni tanto */
#define CAL_RETRY_STEP 300
#define CAL_BUDGET 400000                     /* istruzioni per giro (gui.BUDGET) */

/* ------------------------------------------------------------- i circuiti */
typedef struct {
    double re, im;
} cal_c;

static cal_c cal_cmul(cal_c a, cal_c b)
{
    cal_c r;
    r.re = a.re * b.re - a.im * b.im;
    r.im = a.re * b.im + a.im * b.re;
    return r;
}

/* numpy npy_cpow con esponente intero positivo piccolo: elevamento a potenza
   per quadrati (n < 100); oltre, exp(n log z) (non capita: n <= 32) */
static cal_c cal_cpowi(cal_c a, int n)
{
    cal_c aa = {1.0, 0.0}, p = a;
    int mask = 1;
    if (n == 1)
        return a;
    if (n >= 100) {
        double m = hypot(a.re, a.im), th = atan2(a.im, a.re), lm = log(m), x = exp(n * lm);
        cal_c r;
        r.re = x * cos(n * th);
        r.im = x * sin(n * th);
        return r;
    }
    for (;;) {
        if (n & mask)
            aa = cal_cmul(aa, p);
        mask <<= 1;
        if (n < mask || mask > 100)
            break;
        p = cal_cmul(p, p);
    }
    return aa;
}

struct e_cal {
    uint16_t w[SYN_FRAME_WORDS];              /* Circuits.frame: l'ultimo frame del firmware */
    uint32_t job;                             /* la funzione di misura del lavoro in corso, 0 = nessuna */
    double phase[8];
    double osc_phase[3][2];
    cal_c tom_z[3];
    double tom_kick[3], pending[3];
    int wait[3];
    int64_t kicks[3];
    int64_t rx_fills, rx_frames;
    uc_hook h_strike, h_usb;
    uint32_t *buf;                            /* nfr x 12 campioni */
    size_t buf_cap;
};

#define CAL_XD_HZ_PER_CODE 0.5
#define CAL_TOM_BT_LOW (48000 / 5601.0)
#define CAL_TOM_A 1.0
#define CAL_TOM_CV_OCT 3200.0
#define CAL_TOM_N 120.0
#define CAL_TOM_KICK 100.0
#define CAL_TOM_LEVEL 0.25

static int16_t cal_s16(uint16_t v)
{
    return (int16_t)v;
}

/* Circuits.samples(nframes) -> (nframes, 12) uint32 */
static void cal_samples(e_cal *c, int nframes, uint32_t *res)
{
    int64_t *out = (int64_t *)calloc((size_t)nframes * CAL_SLOTS, sizeof *out);
    const uint16_t *w = c->w;
    int v, k, o, t;
    static const int xd_voice[3] = {0, 2, 7};
    static const double tom_hz0[3] = {48000 / 2627.0, 48000 / 2627.0, 48000 / 2627.0};
    if (!out) {
        memset(res, 0, (size_t)nframes * CAL_SLOTS * 4);
        return;
    }
    for (v = 0; v < 8; v++) {                 /* i filtri: autooscillano se la risonanza e' alta */
        int code = cal_s16(w[29 + v]);
        int reso = w[37 + v];
        if (reso >= 0x3000) {
            double f = 23000.0 * pow(2.0, -code / 1600.0), ph = 0;
            if (f < 10.0) f = 10.0;           /* clip(., 10, 23000) */
            if (f > 23000.0) f = 23000.0;
            for (t = 1; t <= nframes; t++) {
                ph = c->phase[v] + (double)t * f / CAL_SR;
                out[(size_t)(t - 1) * CAL_SLOTS + v] = (int64_t)(sin(CAL_TWO_PI * ph) * 0.3 * CAL_FULL);
            }
            c->phase[v] = fmod(ph, 1.0);
        }
    }
    for (k = 0; k < 3; k++) {                 /* le voci XD: due VCO lineari ciascuna */
        v = xd_voice[k];
        for (o = 0; o < 2; o++) {
            int code = cal_s16(w[45 + 4 * k + o]);
            double lev = (double)w[47 + 4 * k + o] / 0x2000, f, ph = 0;
            if (lev <= 0 || code == 0)
                continue;
            f = CAL_XD_HZ_PER_CODE * (double)(code < 0 ? -code : code);
            if (f > 20000.0) f = 20000.0;
            for (t = 1; t <= nframes; t++) {
                ph = c->osc_phase[k][o] + (double)t * f / CAL_SR;
                out[(size_t)(t - 1) * CAL_SLOTS + v] +=
                    (int64_t)(sin(CAL_TWO_PI * ph) * 0.25 * (lev < 1.0 ? lev : 1.0) * CAL_FULL);
            }
            c->osc_phase[k][o] = fmod(ph, 1.0);
        }
    }
    if (c->job == CAL_TOM_JOB) {              /* i tom (BT, LT, MT sugli slot 1, 3, 5): risonatori */
        static const int tom_slot[3] = {1, 3, 5};
        for (k = 0; k < 3; k++) {
            double f0 = (k == 0 && (w[82] & 8)) ? CAL_TOM_BT_LOW : tom_hz0[k];
            int word = w[59 + 2 * k];
            double f, alpha, mag, sc;
            cal_c z0, step, z;
            f = f0 * sqrt((double)(word > 1 ? word : 1) / 256.0);
            alpha = f * CAL_TOM_A * pow(2.0, cal_s16(w[60 + 2 * k]) / CAL_TOM_CV_OCT) - CAL_TOM_N;
            z0.re = c->tom_z[k].re + CAL_TOM_KICK * c->tom_kick[k];
            z0.im = c->tom_z[k].im;
            c->tom_kick[k] = 0.0;
            if (hypot(z0.re, z0.im) < 1e-5) {         /* rumore del circuito */
                z0.re = 1e-5;
                z0.im = 0.0;
            }
            {
                double a = -alpha / CAL_SR, b = CAL_TWO_PI * f / CAL_SR, x = exp(a);
                step.re = x * cos(b);
                step.im = x * sin(b);
            }
            z.re = z.im = 0;
            for (t = 1; t <= nframes; t++) {
                z = cal_cmul(z0, cal_cpowi(step, t));
                mag = hypot(z.re, z.im);
                if (mag > 1.0) {                        /* il limitatore */
                    sc = 1.0 / (mag > 1e-30 ? mag : 1e-30);
                    z.re *= sc;
                    z.im *= sc;
                }
                out[(size_t)(t - 1) * CAL_SLOTS + tom_slot[k]] +=
                    (int64_t)(z.re * CAL_TOM_LEVEL * CAL_FULL);
            }
            c->tom_z[k] = z;
        }
    }
    /* campioni a 24 bit ALLINEATI A DESTRA: le misure li caricano e fanno asl.l #8 */
    for (t = 0; t < nframes * CAL_SLOTS; t++) {
        int64_t x = out[t];
        if (x < -(int64_t)CAL_FULL) x = -(int64_t)CAL_FULL;
        if (x > (int64_t)CAL_FULL - 1) x = (int64_t)CAL_FULL - 1;
        res[t] = (uint32_t)(x >> 8);
    }
    free(out);
}

/* rx_fill(dest, words): il peer RX (dt2_sst_rx_fill), come calloop.py */
static void cal_rx_fill(void *ctx, uint32_t dest, int64_t words)
{
    dt_engine *e = (dt_engine *)ctx;
    e_cal *c = e->cal;
    int64_t rel = ((int64_t)dest - (int64_t)CAL_RING) / 4, first = ((rel % CAL_SLOTS) + CAL_SLOTS) % CAL_SLOTS;
    int64_t nfr = (first + words + CAL_SLOTS - 1) / CAL_SLOTS, i;
    uint8_t *b;
    if (!c || words <= 0)
        return;
    if ((size_t)(nfr * CAL_SLOTS) > c->buf_cap) {
        uint32_t *q = (uint32_t *)realloc(c->buf, (size_t)(nfr * CAL_SLOTS) * 4);
        if (!q)
            return;
        c->buf = q;
        c->buf_cap = (size_t)(nfr * CAL_SLOTS);
    }
    cal_samples(c, (int)nfr, c->buf);
    b = (uint8_t *)malloc((size_t)words * 4);
    if (!b)
        return;
    for (i = 0; i < words; i++)
        e_put32(b + 4 * i, c->buf[first + i]);
    {
        uint64_t p;
        for (p = 0; p < (uint64_t)words * 4 + 0x100000u; p += 0x100000u)
            dtm_ensure(e->m, (uint64_t)dest + p);
    }
    e_write(e, dest, b, (size_t)words * 4);
    free(b);
    c->rx_fills++;
    c->rx_frames += nfr;
}

/* il frame che il firmware ha appena mandato (SERQ del canale 15): on_serq di calloop.py */
static void cal_on_frame(dt_engine *e, const uint16_t *w)
{
    e_cal *c = e->cal;
    int k;
    memcpy(c->w, w, sizeof c->w);
    /* il lavoro in corso, dalla sua funzione di misura (la lista puo' essere una a parte) */
    {
        uint32_t st = e_r32(e, CAL_STATE), job = e_r32(e, CAL_JOB);
        c->job = (st >= 2 && st <= 4 && job < 16) ? e_r32(e, e_r32(e, CAL_LIST_PTR) + 0x24 * job + 0x18) : 0;
    }
    for (k = 0; k < 3; k++)                   /* deliver(): il DAC suona il colpo dopo il frame coi suoi CV */
        if (c->wait[k] && !--c->wait[k]) {
            c->tom_kick[k] += c->pending[k];
            c->pending[k] = 0.0;
        }
}

/* TOMS colpisce ogni tom con una rampa di 32 campioni nel buffer del DAC
   (move.l d0,(a0,d5.l), d4 = tom * 4): presa li' */
static void cal_strike(uc_engine *uc, uint64_t addr, uint32_t size, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    e_cal *c = e->cal;
    uint32_t k = e_rd(e, UC_M68K_REG_D4) / 4;
    int32_t v = (int32_t)e_rd(e, UC_M68K_REG_D0);
    (void)uc; (void)addr; (void)size;
    if (c && k < 3) {
        c->pending[k] += (double)v / CAL_FULL;
        c->wait[k] = 2;
        c->kicks[k]++;
    }
}

/* il PHY USB risponde subito: i bit 1 e 17 si azzerano alla lettura */
static void cal_usb_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t cur = e_r32(e, CAL_USBCMD);
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    if (cur & 0x00020002u)
        e_w32(e, CAL_USBCMD, cur & ~0x00020002u);
}

/* ------------------------------------------------- girare e iniettare */
static int cal_run(dt_engine *e, int64_t n, char *err, size_t errlen)
{
    int64_t ex = 0;
    int halted;
    e_cur = e;
    halted = engine_spin(e, n, &ex);
    e->instrs += ex;
    if (halted) {
        e->halted = 1;
        dt_errf(err, errlen, "motore fermo durante la calibrazione: %s", e->status);
        return DT_ERR_UNICORN;
    }
    panel_publish(e);
    if (e->uart_out.n) {
        rl_feed(&e->leds, e->uart_out.p, e->uart_out.n);
        e->uart_out.n = 0;
    }
    e->chunk++;
    return 0;
}

static int cal_cancelled(volatile int *cancel, char *err, size_t errlen)
{
    if (cancel && *cancel) {
        dt_errf(err, errlen, "interrotto");
        return 1;
    }
    return 0;
}

/* Chiama una funzione del firmware dal contesto di un task (IPL 0), lasciando
   girare il motore normalmente (interrupt, timer) finche' non ritorna.
   Registri e PC si rimettono come erano. -> 0, errore (DT_ERR_*); *returned,
   *d0 se dati. */
static int cal_inject(dt_engine *e, volatile int *cancel, char *err, size_t errlen, uint32_t entry,
                      int nargs, const uint32_t *args, int *returned, uint32_t *d0_out)
{
    static const int REGS[17] = {UC_M68K_REG_SR, UC_M68K_REG_D0, UC_M68K_REG_D1, UC_M68K_REG_D2,
                                 UC_M68K_REG_D3, UC_M68K_REG_D4, UC_M68K_REG_D5, UC_M68K_REG_D6,
                                 UC_M68K_REG_D7, UC_M68K_REG_A0, UC_M68K_REG_A1, UC_M68K_REG_A2,
                                 UC_M68K_REG_A3, UC_M68K_REG_A4, UC_M68K_REG_A5, UC_M68K_REG_A6,
                                 UC_M68K_REG_A7};
    uint32_t saved[17], saved_pc, sp, d0;
    uint8_t buf[4 * 8];
    static const uint8_t trap[2] = {0x60, 0xFE};
    int i, r, ok = 0;
    /* il contesto di un task (IPL 0): dopo ogni giro il motore e' fermo su un
       evento appena servito, cioe' dentro la sua ISR; si eseguono poche
       istruzioni alla volta, senza servire eventi, finche' l'ISR ritorna al
       task (il Python lo faceva a caso: spin(5000) finisce nella ISR) */
    for (i = 0; i < 20000 && e_ipl(e) != 0; i++) {
        int ue;
        if (i % 200 == 199 && (r = cal_run(e, 20000, err, errlen)))      /* un'ISR che aspetta un evento: serviamolo */
            return r;
        e->stp.left = INT64_MAX / 2;
        e->stp.blocks = 0;
        e->m->stop_flag = 0;
        ue = (int)uc_emu_start(e->uc, e->pc, 0, 0, 40);
        if (ue != UC_ERR_OK) {
            dt_errf(err, errlen, "Unicorn: %s", uc_strerror((uc_err)ue));
            return DT_ERR_UNICORN;
        }
        e->pc = e_rd(e, UC_M68K_REG_PC);
        e->instrs += 40;
    }
    e->m->stop_flag = 0;
    if (e_ipl(e) != 0) {
        dt_errf(err, errlen, "il motore non esce dalla ISR (IPL %d): niente chiamata in un task", e_ipl(e));
        return DT_ERR_UNSUPPORTED;
    }
    for (i = 0; i < 17; i++)
        saved[i] = e_rd(e, REGS[i]);
    saved_pc = e->pc;
    dtm_ensure(e->m, CAL_SENTINEL);
    e_write(e, CAL_SENTINEL, trap, 2);
    sp = saved[16] - 80;
    e_put32(buf, CAL_SENTINEL);
    for (i = 0; i < nargs && i < 7; i++)
        e_put32(buf + 4 + 4 * i, args[i]);
    e_write(e, sp, buf, 4u + 4u * (unsigned)nargs);
    e_wr(e, UC_M68K_REG_A7, sp);
    e->pc = entry;
    for (i = 0; i < 4000; i++) {
        if (cal_cancelled(cancel, err, errlen))
            return DT_ERR_IO;
        if ((r = cal_run(e, 100000, err, errlen)))
            return r;
        if (e->pc == CAL_SENTINEL) {
            ok = 1;
            break;
        }
    }
    d0 = e_rd(e, UC_M68K_REG_D0);
    for (i = 0; i < 17; i++)
        e_wr(e, REGS[i], saved[i]);
    e->pc = saved_pc;
    if (returned)
        *returned = ok;
    if (d0_out)
        *d0_out = d0;
    return 0;
}

/* ------------------------------------------------------ lo svolgimento */
/* durata (tick, 1500 al secondo) di ogni lavoro del modo 0, per la frazione di avanzamento: misurata
   (docs/CALIBRAZIONE.md, "Calibrazione in C al primo avvio") */
static const int32_t CAL_JOB_TICKS[CAL_NJOBS] = {
    32545, 38551, 187505, 396536, 414461, 29250, 126076, 126151, 126151, 126076};

static void cal_title(dt_engine *e, int job, char *out, size_t n)
{
    uint32_t list = e_r32(e, CAL_LIST_PTR), p = e_r32(e, list + 0x24 * (uint32_t)job + 4);
    size_t i;
    out[0] = 0;
    for (i = 0; i + 1 < n; i++) {
        uint8_t ch = e_r8(e, p + i);
        if (ch < 0x20 || ch >= 0x7F)
            break;
        out[i] = (char)ch;
        out[i + 1] = 0;
    }
}

/* i record di errore dei controlli (0x42fb4224, 0x3c byte ciascuno: lavoro, 1 = superato, poi codici di 4
   lettere come OC1T, CVLO, TOME): -> quanti lavori non superati, il testo in `msg` */
static int cal_errors(dt_engine *e, char *msg, size_t n)
{
    uint8_t rb[0x3C * 14];
    int j, k, bad = 0;
    size_t len = 0;
    e_read(e, CAL_ERRORS, rb, sizeof rb);
    msg[0] = 0;
    for (j = 0; j < 14; j++) {
        const uint8_t *r = rb + 0x3C * j;
        int any = 0, codes = 0;
        for (k = 0; k < 0x3C; k++)
            any |= r[k];
        if (!any)
            continue;
        for (k = 8; k < 0x3C; k++)
            codes |= r[k];
        if (e_be32(r + 4) == 1 && !codes)
            continue;
        bad++;
        len += (size_t)snprintf(msg + len, len < n ? n - len : 0, "%slavoro %u:", len ? "; " : "", e_be32(r));
        for (k = 2; k < 15; k++) {
            uint32_t x = e_be32(r + 4 * k);
            if (x && len < n)
                len += (size_t)snprintf(msg + len, n - len, " %c%c%c%c", (x >> 24) & 0x7F, (x >> 16) & 0x7F, (x >> 8) & 0x7F, x & 0x7F);
        }
    }
    return bad;
}

static int cal_main(dt_engine *e, volatile int *cancel, dt_progress_fn progress, void *ctx, char *err,
                    size_t errlen)
{
    int r, returned = 0, only = 0;
    uint32_t d0 = 0, args[2], state = 0, job = 0, tick = 0, last_job = 0xFFFFFFFFu, job_tick0 = 0;
    char title[48] = "", what[96] = "Calibrating the analog circuits", emsg[200];
    int64_t limit = 1500LL * 60000000LL, t_start;      /* 1500 s emulati al massimo (il firmware ha il suo: 20 min) */
    if (progress)
        progress(ctx, 0.0, "Calibrating the analog circuits: starting");
    if ((r = cal_run(e, 3000000, err, errlen)))             /* 3.000.000 istruzioni di assestamento */
        return r;
    args[0] = 0;                                           /* modo 0 (completa), sel -1 (automatico) */
    args[1] = 0xFFFFFFFFu;
    if (getenv("ARCORE_CAL_ONLY")) {                       /* prove: uno o piu' lavori (modo 1) */
        const char *q = getenv("ARCORE_CAL_ONLY");
        uint8_t recs[0x24 * CAL_NJOBS];
        uint32_t n = 0;
        while (*q && n < CAL_NJOBS) {
            e_read(e, CAL_JOBS + 0x24u * (uint32_t)atoi(q), recs + 0x24u * n, 0x24);
            n++;
            while (*q && *q != ',') q++;
            if (*q == ',') q++;
        }
        dtm_ensure(e->m, 0x42F96F94u);
        e_write(e, 0x42F96F94u, recs, 0x24u * n);
        e_w32(e, 0x419BFB30u, n);
        args[0] = 1;
        only = 1;
    }
    if ((r = cal_inject(e, cancel, err, errlen, CAL_START, 2, args, &returned, &d0)))
        return r;
    if (d0 != 1) {
        dt_errf(err, errlen, "calibrazione non avviata (d0 = %u)", d0);
        return DT_ERR_UNSUPPORTED;
    }
    t_start = e->instrs;
    while (e->instrs - t_start < limit) {
        if (cal_cancelled(cancel, err, errlen))
            return DT_ERR_IO;
        if ((r = cal_run(e, 3000000, err, errlen)))
            return r;
        state = e_r32(e, CAL_STATE);
        job = e_r32(e, CAL_JOB);
        tick = e_r32(e, CAL_TICK);
        if (state >= 7)
            break;
        if (job != last_job && job < CAL_NJOBS) {
            last_job = job;
            job_tick0 = tick;
            cal_title(e, (int)job, title, sizeof title);
            snprintf(what, sizeof what, "Calibrating the analog circuits, job %u of %d: %s", job + 1, CAL_NJOBS, title);
        }
        if (progress) {
            double f = 0, tot = 0;
            uint32_t j;
            for (j = 0; j < CAL_NJOBS; j++) {
                tot += CAL_JOB_TICKS[j];
                if (j < job)
                    f += CAL_JOB_TICKS[j];
            }
            if (job < CAL_NJOBS)
                f += tick - job_tick0 < (uint32_t)CAL_JOB_TICKS[job] ? tick - job_tick0 : CAL_JOB_TICKS[job];
            f = only ? 0.5 : f / tot;
            progress(ctx, f > 0.98 ? 0.98 : f, what);
        }
    }
    if (state < 7) {
        dt_errf(err, errlen, "calibrazione senza fine (stato %u, lavoro %u)", state, job);
        return DT_ERR_UNSUPPORTED;
    }
    /* i controlli dei lavori: se uno non e' superato niente flash, solo la pulizia (0x401090f6) */
    if (!only && cal_errors(e, emsg, sizeof emsg)) {
        cal_inject(e, cancel, NULL, 0, CAL_STOP, 0, NULL, &returned, &d0);
        cal_run(e, 30000000, NULL, 0);
        dt_errf(err, errlen, "controlli non superati: %s", emsg);
        return DT_ERR_FORMAT;
    }
    if (progress)
        progress(ctx, 0.98, "Calibrating the analog circuits: saving the results");
    /* SAVING RESULTS: i controlli di nuovo, il buffer di lavoro nel record,
       il record nella flash SPI; poi la pulizia (vettore 191, USB, modo audio) */
    if ((r = cal_inject(e, cancel, err, errlen, CAL_SAVE, 0, NULL, &returned, &d0)))
        return r;
    if ((r = cal_run(e, 30000000, err, errlen)))
        return r;
    if (!only && (e_r32(e, CAL_RECORD + 0x10) >> 16) == 0) {
        dt_errf(err, errlen, "record RBOW non valido dopo il salvataggio");
        return DT_ERR_FORMAT;
    }
    return 0;
}

int dt_engine_calibrate(dt_engine *e, volatile int *cancel, dt_progress_fn progress, void *ctx, char *err,
                        size_t errlen)
{
    e_cal *c;
    int r, tap_was;
    int64_t retry_was;
    if (!e || e->halted) {
        dt_errf(err, errlen, "motore fermo");
        return DT_ERR_ARG;
    }
    if (!e->nor.installed) {
        dt_errf(err, errlen, "firmware senza la flash SPI modellata: niente calibrazione");
        return DT_ERR_UNSUPPORTED;
    }
    if (e->has_pending_ips && e->pending_ips_at <= e->instrs) {      /* il tempo del motore (primo giro di dt_engine_chunk) */
        int k;
        e->has_pending_ips = 0;
        for (k = 0; k < e->ntim; k++)
            if (tsrc_set_ips(e, &e->tim[k], e->pending_ips_n)) {
                dt_errf(err, errlen, "%s", e->err);
                return DT_ERR_UNICORN;
            }
        e->sst.ips = e->pending_ips_n;
    }
    c = (e_cal *)calloc(1, sizeof *c);
    if (!c) {
        dt_errf(err, errlen, "memoria esaurita");
        return DT_ERR_NOMEM;
    }
    e_cur = e;
    e->cal = c;
    tap_was = e->syn.tap_on;
    e->syn.tap_on = 0;                                     /* il TX non va allo strato dei sample */
    retry_was = e->sst.level_retry_step;
    e->sst.level_retry_step = CAL_RETRY_STEP;
    dt2_sst_rx_fill = cal_rx_fill;
    dt2_sst_rx_ctx = e;
    c->h_strike = e_hook_code(e, (void *)cal_strike, e, CAL_TOM_STRIKE);
    c->h_usb = e_hook_mem(e, UC_HOOK_MEM_READ, (void *)cal_usb_read, e, CAL_USBCMD, CAL_USBCMD + 3);
    r = cal_main(e, cancel, progress, ctx, err, errlen);
    /* si toglie tutto, anche dopo un errore: nessun hook rimasto */
    uc_hook_del(e->uc, c->h_strike);
    uc_hook_del(e->uc, c->h_usb);
    e->sst.level_retry_step = retry_was;
    dt2_sst_rx_fill = NULL;
    dt2_sst_rx_ctx = NULL;
    e->syn.tap_on = tap_was;
    e->syn.tap_n = 0;                                      /* i pezzi di TX di prima: scartati */
    e->syn.tap_data.n = 0;
    e->syn.nacc = 0;
    e->syn.has_prev = 0;                                   /* nessun trigger spurio al primo frame */
    e->sst.nchunk = 0;
    e->sst.txlen = 0;
    e->cal = NULL;
    free(c->buf);
    free(c);
    if (r == 0 && progress)
        progress(ctx, 1.0, "Calibrating the analog circuits: done");
    return r;
}
