/* dt_rt: il motore in tempo reale per un plugin (M5, docs/AU-PORT.md 4.3).
 *
 * Un thread di emulazione per istanza esegue i giri di dt_engine; il thread
 * audio dell'host chiama dt_rt_render, che non blocca, non alloca e non
 * prende lock (tranne in modo offline, dove aspetta il motore).
 *
 * Tempo. Il motore ha il suo tempo in campioni a 48 kHz: istruzioni * 3 / 2500
 * (mcengine.IPS = 40M istruzioni per secondo emulato). L'host ha il suo, contato dai
 * campioni resi. Con R la frequenza dell'host, h48(h) = floor(h * 48000 / R):
 *   - un evento MIDI all'istante h dell'host entra nel motore al primo confine
 *     di giro in cui il tempo del motore e' >= h48(h) + E;
 *   - il campione k dell'uscita del motore (l'SSI0, a mezzi buffer) esce
 *     all'istante h48 = k + X - E.
 * E (anticipo del motore, nessun costo di latenza) da' respiro al thread di
 * emulazione; X e' la latenza dichiarata all'host (in campioni a 48 kHz,
 * convertita). Cosi' l'uscita e' funzione solo degli eventi e dei loro
 * istanti: in tempo reale (senza buchi) e offline e' la stessa, a qualunque
 * dimensione di blocco (tests/test_dtcore_rt.py).
 */
#include "mccore.h"
#include "internal.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <pthread.h>
#include <time.h>
#if defined(__APPLE__)
#include <dispatch/dispatch.h>
#include <pthread/qos.h>
#else
#include <semaphore.h>
#endif
#endif

#define RT_CH 2                     /* L, R: il mix dell'SSI0 */
#define RT_ACAP (1 << 16)           /* campioni a 48 kHz nell'anello dell'audio */
#define RT_CHUNK48 240              /* un giro: 200000 istruzioni = 240 campioni */
#define RT_BURST48 512              /* l'audio esce a mezzi buffer del DMA dell'SSI0 */
#define RT_EVCAP (1 << 16)
#define RT_PANCAP 1024
#define RT_MOCAP (1 << 14)
#define RT_TAPS 16                  /* semi-larghezza del filtro di ricampionamento */
#define RT_PHASES 512

/* ------------------------------------------------------------- semaforo */
typedef struct {
#if defined(_WIN32)
    HANDLE h;
#elif defined(__APPLE__)
    dispatch_semaphore_t s;
#else
    sem_t s;
#endif
} rt_sem;

static int sem_init_(rt_sem *s)
{
#if defined(_WIN32)
    s->h = CreateSemaphore(NULL, 0, 0x7FFFFFFF, NULL);
    return s->h ? 0 : -1;
#elif defined(__APPLE__)
    s->s = dispatch_semaphore_create(0);
    return s->s ? 0 : -1;
#else
    return sem_init(&s->s, 0, 0);
#endif
}

static void sem_post_(rt_sem *s)
{
#if defined(_WIN32)
    ReleaseSemaphore(s->h, 1, NULL);
#elif defined(__APPLE__)
    dispatch_semaphore_signal(s->s);
#else
    sem_post(&s->s);
#endif
}

static void sem_wait_ms(rt_sem *s, int ms)
{
#if defined(_WIN32)
    WaitForSingleObject(s->h, (DWORD)ms);
#elif defined(__APPLE__)
    dispatch_semaphore_wait(s->s, dispatch_time(DISPATCH_TIME_NOW, (int64_t)ms * 1000000));
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += (long)ms * 1000000L;
    ts.tv_sec += ts.tv_nsec / 1000000000L;
    ts.tv_nsec %= 1000000000L;
    sem_timedwait(&s->s, &ts);
#endif
}

static void sem_destroy_(rt_sem *s)
{
#if defined(_WIN32)
    CloseHandle(s->h);
#elif defined(__APPLE__)
    (void)s;                         /* ARC/dispatch_release dipende dalla build: si lascia */
#else
    sem_destroy(&s->s);
#endif
}

/* orologio monotono in ns (durata dei giri del motore) */
static int64_t mono_ns(void)
{
#if defined(_WIN32)
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (int64_t)((double)c.QuadPart * 1e9 / (double)f.QuadPart);
#else
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
#endif
}

/* ------------------------------------------------------------ dt_rt */
typedef struct {
    int kind, code, arg;
} rt_panel;

struct dt_rt {
    dt_firmware fw;
    dt_engine *e;
    double rate;                    /* frequenza dell'host */
    uint32_t max_frames;
    int64_t E, X;                   /* in campioni a 48 kHz */
    int64_t host_frames;            /* campioni resi (thread audio) */
    _Atomic int64_t limit48;        /* il motore gira finche' t48 + 128 <= limit48 */
    _Atomic int64_t engine_t48;     /* tempo del motore (letto dal thread audio) */
    _Atomic int quit, halted, offline_wait;
    _Atomic int save_req;           /* 1 = salva al prossimo confine di giro */
    uint8_t *save_buf;
    size_t save_len;
    int save_rc;
    rt_sem saved;
    _Atomic int64_t underruns, chunks;
    int64_t max_lag;                /* tempo del motore - campioni d'uscita pronti (diagnostica) */
    _Atomic int64_t max_step_ns;
    rt_sem wake, progress;
    int has_thread;
#if defined(_WIN32)
    HANDLE thread;
#else
    pthread_t thread;
#endif
    /* audio: motore -> thread audio */
    float *aring;                   /* RT_ACAP x RT_CH */
    _Atomic int64_t awritten;       /* campioni scritti (indice assoluto) */
    /* eventi MIDI: thread audio -> motore; anello di byte {i64 t48, u16 n, dati} */
    uint8_t *ev;
    _Atomic uint64_t ev_head, ev_tail;
    int64_t ev_next_t;              /* il prossimo evento letto ma non ancora dovuto */
    uint8_t ev_next[260];
    size_t ev_next_n;
    int has_ev_next;
    /* pannello: thread della GUI -> motore */
    rt_panel pan[RT_PANCAP];
    _Atomic uint64_t pan_head, pan_tail;
    /* MIDI in uscita: motore -> thread audio; {i64 t48, u16 n, dati} */
    uint8_t *mo;
    _Atomic uint64_t mo_head, mo_tail;
    /* schermo e LED: motore -> GUI (seqlock) */
    /* pattern (vista Elektron Emu): motore -> GUI (seqlock), ogni 8 giri */
    _Atomic(dt_seq_reader) seq_fn;
    _Atomic(dt_seq_hook) hook_fn;   /* ogni 2 giri, con hook_arg - 1 (-1: spento) */
    _Atomic int hook_arg;
    _Atomic uint64_t sq;
    uint8_t sqbuf[DT_SEQ_MAX];
    size_t sqlen;
    uint64_t sqver;
    unsigned sqtick;
    _Atomic uint64_t seq;
    uint8_t screen[128 * 64], leds[8];
    uint64_t screen_ver, leds_ver;
    _Atomic uint64_t leds_taken;    /* la GUI ha letto i LED accumulati: si riparte */
    uint64_t leds_seen;
    /* volume (Main Volume): GUI -> motore */
    _Atomic uint64_t master_bits;   /* il volume (TxOut.volume) */
    double master_applied;
    /* ricampionamento */
    float *kernel;                  /* RT_PHASES + 1 righe x 2*RT_TAPS */
    double ratio;                   /* 48000 / rate */
    char status[160];
};

static void rt_sink(void *ctx, const float *lr, size_t n)
{
    dt_rt *rt = (dt_rt *)ctx;
    int64_t w = atomic_load_explicit(&rt->awritten, memory_order_relaxed);
    size_t i;
    for (i = 0; i < n; i++) {
        float *d = rt->aring + (size_t)((w + (int64_t)i) & (RT_ACAP - 1)) * RT_CH;
        d[0] = lr[2 * i];
        d[1] = lr[2 * i + 1];
    }
    atomic_store_explicit(&rt->awritten, w + (int64_t)n, memory_order_release);
}

static int64_t rt_engine_t48(dt_rt *rt)
{
    return dt_engine_instrs(rt->e) * 3 / 2500;
}

/* il prossimo evento MIDI dall'anello (thread del motore) */
static int ev_peek(dt_rt *rt)
{
    uint64_t head, tail;
    if (rt->has_ev_next)
        return 1;
    head = atomic_load_explicit(&rt->ev_head, memory_order_acquire);
    tail = atomic_load_explicit(&rt->ev_tail, memory_order_relaxed);
    if (head == tail)
        return 0;
    {
        uint8_t hdr[10];
        size_t k;
        for (k = 0; k < 10; k++)
            hdr[k] = rt->ev[(tail + k) & (RT_EVCAP - 1)];
        memcpy(&rt->ev_next_t, hdr, 8);
        rt->ev_next_n = (size_t)(hdr[8] | hdr[9] << 8);
        for (k = 0; k < rt->ev_next_n; k++)
            rt->ev_next[k] = rt->ev[(tail + 10 + k) & (RT_EVCAP - 1)];
        atomic_store_explicit(&rt->ev_tail, tail + 10 + rt->ev_next_n, memory_order_release);
        rt->has_ev_next = 1;
    }
    return 1;
}

/* schermo e LED per la GUI (seqlock). I LED si accumulano (AND dei byte,
   invertiti) finche' la GUI non li legge, come Engine.take_leds: un lampo piu'
   breve del ridisegno (TEMPO, ~40 ms) resta visibile. */
/* la lettura dei pattern (dt_rt_set_seq_reader): fra due giri, sul thread del motore */
static void rt_seq_publish(dt_rt *rt)
{
    uint8_t tmp[DT_SEQ_MAX];
    size_t n;
    dt_seq_reader fn = atomic_load_explicit(&rt->seq_fn, memory_order_acquire);
    dt_seq_hook hk = atomic_load_explicit(&rt->hook_fn, memory_order_acquire);
    const unsigned tick = rt->sqtick++;
    if (hk && !(tick & 1u))
        hk(dt_engine_machine(rt->e), atomic_load_explicit(&rt->hook_arg, memory_order_relaxed) - 1);
    if (!fn || (tick & 7u))
        return;
    n = fn(dt_engine_machine(rt->e), tmp, sizeof tmp);
    if (n > sizeof tmp)
        n = 0;
    if (n == rt->sqlen && !memcmp(tmp, rt->sqbuf, n))
        return;
    atomic_fetch_add_explicit(&rt->sq, 1, memory_order_acq_rel);
    memcpy(rt->sqbuf, tmp, n);
    rt->sqlen = n;
    rt->sqver++;
    atomic_fetch_add_explicit(&rt->sq, 1, memory_order_acq_rel);
}

void dt_rt_set_seq_reader(dt_rt *rt, dt_seq_reader fn)
{
    if (rt)
        atomic_store_explicit(&rt->seq_fn, fn, memory_order_release);
}

void dt_rt_set_seq_hook(dt_rt *rt, dt_seq_hook fn)
{
    if (rt)
        atomic_store_explicit(&rt->hook_fn, fn, memory_order_release);
}

void dt_rt_set_seq_arg(dt_rt *rt, int arg)
{
    if (rt)
        atomic_store_explicit(&rt->hook_arg, arg + 1, memory_order_relaxed);
}

size_t dt_rt_seq(dt_rt *rt, uint8_t *out, size_t cap, uint64_t *version)
{
    if (!rt)
        return 0;
    for (;;) {
        uint64_t s0 = atomic_load_explicit(&rt->sq, memory_order_acquire);
        size_t n;
        if (s0 & 1)
            continue;
        n = rt->sqlen < cap ? rt->sqlen : cap;
        if (out && n)
            memcpy(out, rt->sqbuf, n);
        if (version)
            *version = rt->sqver;
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&rt->sq, memory_order_relaxed) == s0)
            return n;
    }
}

static void rt_publish(dt_rt *rt)
{
    uint8_t scr[128 * 64], leds[8];
    uint64_t sv = dt_engine_screen(rt->e, NULL), taken = atomic_load(&rt->leds_taken);
    int i, fresh = taken != rt->leds_seen, changed;
    dt_engine_leds(rt->e, leds, NULL);
    changed = fresh;
    for (i = 0; i < 8 && !changed; i++)
        changed = (rt->leds[i] & leds[i]) != rt->leds[i];
    if (sv == rt->screen_ver && !changed)
        return;
    dt_engine_screen(rt->e, scr);
    atomic_fetch_add_explicit(&rt->seq, 1, memory_order_acq_rel);
    memcpy(rt->screen, scr, sizeof scr);
    for (i = 0; i < 8; i++)
        rt->leds[i] = fresh ? leds[i] : (uint8_t)(rt->leds[i] & leds[i]);
    rt->leds_seen = taken;
    rt->screen_ver = sv;
    if (changed)
        rt->leds_ver++;
    atomic_fetch_add_explicit(&rt->seq, 1, memory_order_acq_rel);
}

static void rt_midi_out(dt_rt *rt)
{
    uint8_t buf[256];
    size_t n;
    int64_t t = rt_engine_t48(rt);
    while ((n = dt_engine_midi_out(rt->e, buf, sizeof buf)) > 0) {
        uint64_t head = atomic_load_explicit(&rt->mo_head, memory_order_relaxed);
        uint64_t tail = atomic_load_explicit(&rt->mo_tail, memory_order_acquire);
        size_t k;
        uint8_t hdr[10];
        if (RT_MOCAP - (head - tail) < 10 + n)
            continue;                                 /* pieno: si perde (il thread audio e' fermo) */
        memcpy(hdr, &t, 8);
        hdr[8] = (uint8_t)n;
        hdr[9] = (uint8_t)(n >> 8);
        for (k = 0; k < 10; k++)
            rt->mo[(head + k) & (RT_MOCAP - 1)] = hdr[k];
        for (k = 0; k < n; k++)
            rt->mo[(head + 10 + k) & (RT_MOCAP - 1)] = buf[k];
        atomic_store_explicit(&rt->mo_head, head + 10 + n, memory_order_release);
    }
}

/* un passo del motore se il tempo lo consente. -> 1 fatto, 0 no */
static int rt_step(dt_rt *rt)
{
    int64_t t = rt_engine_t48(rt), lim = atomic_load_explicit(&rt->limit48, memory_order_acquire);
    uint64_t ph, pt;
    uint64_t mb;
    int r_chunk;
    if (atomic_load(&rt->halted) || t + RT_CHUNK48 > lim)
        return 0;
    /* MIDI: tutti gli eventi dovuti (istante <= tempo del motore) */
    while (ev_peek(rt) && rt->ev_next_t <= t) {
        dt_engine_midi_in(rt->e, rt->ev_next, rt->ev_next_n);
        rt->has_ev_next = 0;
    }
    /* pannello */
    ph = atomic_load_explicit(&rt->pan_head, memory_order_acquire);
    pt = atomic_load_explicit(&rt->pan_tail, memory_order_relaxed);
    while (pt != ph) {
        rt_panel *p = &rt->pan[pt & (RT_PANCAP - 1)];
        dt_engine_panel(rt->e, p->kind, p->code, p->arg);
        pt++;
    }
    atomic_store_explicit(&rt->pan_tail, pt, memory_order_release);
    mb = atomic_load(&rt->master_bits);
    {
        double m;
        memcpy(&m, &mb, 8);
        if (m != rt->master_applied) {
            dt_engine_set_volume(rt->e, m);
            rt->master_applied = m;
        }
    }
    {
        int64_t lag = t - atomic_load(&rt->awritten);
        if (lag > rt->max_lag)
            rt->max_lag = lag;
    }
    {
        int64_t ns = mono_ns();
        r_chunk = dt_engine_chunk(rt->e);
        ns = mono_ns() - ns;
        if (ns > atomic_load(&rt->max_step_ns))
            atomic_store(&rt->max_step_ns, ns);
    }
    if (r_chunk) {
        atomic_store(&rt->halted, 1);
        snprintf(rt->status, sizeof rt->status, "%s", dt_engine_status(rt->e));
    }
    atomic_store_explicit(&rt->engine_t48, rt_engine_t48(rt), memory_order_release);
    atomic_fetch_add(&rt->chunks, 1);
    rt_publish(rt);
    rt_seq_publish(rt);
    rt_midi_out(rt);
    return 1;
}

#if defined(_WIN32)
static DWORD WINAPI rt_thread(void *arg)
#else
static void *rt_thread(void *arg)
#endif
{
    dt_rt *rt = (dt_rt *)arg;
#if defined(__APPLE__)
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    while (!atomic_load(&rt->quit)) {
        int did = 0;
        while (!atomic_load(&rt->quit)) {
            if (atomic_load(&rt->save_req)) {             /* fra due giri */
                rt->save_rc = dt_engine_save_fast(rt->e, &rt->save_buf, &rt->save_len);   /* solo la fotografia */
                atomic_store(&rt->save_req, 0);
                sem_post_(&rt->saved);
            }
            if (!rt_step(rt))
                break;
            did = 1;
        }
        if (did)
            sem_post_(&rt->progress);
        sem_wait_ms(&rt->wake, 20);
    }
#if defined(_WIN32)
    return 0;
#else
    return NULL;
#endif
}

/* ------------------------------------------------------ ricampionamento */
static double bessel_i0(double x)
{
    double s = 1, t = 1;
    int k;
    for (k = 1; k < 40; k++) {
        t *= (x / (2 * k)) * (x / (2 * k));
        s += t;
    }
    return s;
}

static int rt_kernel(dt_rt *rt)
{
    double cutoff = rt->rate < 48000.0 ? 0.92 * rt->rate / 48000.0 : 0.92, beta = 8.0;
    int p, k;
    rt->kernel = (float *)malloc(sizeof(float) * (RT_PHASES + 1) * 2 * RT_TAPS);
    if (!rt->kernel)
        return -1;
    for (p = 0; p <= RT_PHASES; p++) {
        double frac = (double)p / RT_PHASES, sum = 0;
        float *row = rt->kernel + (size_t)p * 2 * RT_TAPS;
        for (k = 0; k < 2 * RT_TAPS; k++) {
            double x = (double)(k - RT_TAPS + 1) - frac, w, s;
            double r = x / RT_TAPS;
            s = x == 0 ? cutoff : sin(M_PI * cutoff * x) / (M_PI * x);
            w = fabs(r) >= 1 ? 0 : bessel_i0(beta * sqrt(1 - r * r)) / bessel_i0(beta);
            row[k] = (float)(s * w);
            sum += s * w;
        }
        for (k = 0; k < 2 * RT_TAPS; k++)
            row[k] = (float)(row[k] / sum);
    }
    return 0;
}

/* il campione k del canale ch (0 se non c'e' ancora o e' troppo vecchio) */
static float rt_sample(dt_rt *rt, int64_t k, int ch, int64_t written, int *missing)
{
    if (k < 0)
        return 0.0f;
    if (k >= written) {
        *missing = 1;
        return 0.0f;
    }
    if (k < written - RT_ACAP)
        return 0.0f;
    return rt->aring[(size_t)(k & (RT_ACAP - 1)) * RT_CH + ch];
}

/* -------------------------------------------------------------- API */
static dt_rt *rt_new(const uint8_t *syx, size_t syx_len, const char *state_path, const uint8_t *state_buf,
                     size_t state_len, double rate, uint32_t max_frames, char *err, size_t errlen);

dt_rt *dt_rt_new(const uint8_t *syx, size_t syx_len, const char *state_path, double rate,
                 uint32_t max_frames, char *err, size_t errlen)
{
    return rt_new(syx, syx_len, state_path, NULL, 0, rate, max_frames, err, errlen);
}

dt_rt *dt_rt_new_mem(const uint8_t *syx, size_t syx_len, const uint8_t *state, size_t state_len, double rate,
                     uint32_t max_frames, char *err, size_t errlen)
{
    return rt_new(syx, syx_len, NULL, state, state_len, rate, max_frames, err, errlen);
}

/* lo stato del motore (DTSZ), salvato dal suo thread fra due giri. Blocca chi
   chiama (non il thread audio) finche' e' pronto. -> 0, il buffer in *out (free) */
int dt_rt_save(dt_rt *rt, uint8_t **out, size_t *len)
{
    int k;
    atomic_store(&rt->save_req, 1);
    sem_post_(&rt->wake);
    for (k = 0; k < 600 && atomic_load(&rt->save_req); k++)
        sem_wait_ms(&rt->saved, 50);
    if (atomic_load(&rt->save_req))
        return DT_ERR_IO;
    if (rt->save_rc == DT_OK && rt->save_buf) {      /* la compressione e' di chi salva, non del motore */
        int r = dt_state_pack(rt->save_buf, rt->save_len, out, len);
        free(rt->save_buf);
        rt->save_buf = NULL;
        return r;
    }
    *out = rt->save_buf;
    *len = rt->save_len;
    rt->save_buf = NULL;
    return rt->save_rc;
}

static dt_rt *rt_new(const uint8_t *syx, size_t syx_len, const char *state_path, const uint8_t *state_buf,
                     size_t state_len, double rate, uint32_t max_frames, char *err, size_t errlen)
{
    dt_rt *rt = (dt_rt *)calloc(1, sizeof *rt);
    int r;
    int64_t n48;
    double m = 0.25;
    if (!rt) {
        dt_errf(err, errlen, "out of memory");
        return NULL;
    }
    if (rate < 8000 || rate > 384000 || !max_frames) {
        dt_errf(err, errlen, "bad sample rate or block size");
        free(rt);
        return NULL;
    }
    r = dt_fw_extract(syx, syx_len, &rt->fw, err, errlen);
    if (r) {
        free(rt);
        return NULL;
    }
    rt->e = dt_engine_new_mem(&rt->fw, state_path, state_buf, state_len, NULL, err, errlen);
    if (!rt->e) {
        dt_fw_free(&rt->fw);
        free(rt);
        return NULL;
    }
    rt->rate = rate;
    rt->max_frames = max_frames;
    rt->ratio = 48000.0 / rate;
    /* il blocco in tempo reale: al piu' 1024 (un blocco piu' grande -- l'offline
       di molti host -- aspetta il motore e non conta per la latenza) */
    n48 = (int64_t)ceil((max_frames < 1024 ? max_frames : 1024) * rt->ratio);
    /* latenza: un blocco dell'host, un mezzo buffer dell'SSI0, due giri (240),
       il filtro e un margine; E: quanto il motore puo' correre avanti */
    /* + un margine per i picchi del motore: la traduzione JIT di codice nuovo
       (il primo PLAY, una pagina nuova) misurata fino a 8,5 ms per un giro */
    rt->X = n48 + RT_BURST48 + 2 * RT_CHUNK48 + RT_TAPS + 64 + 1024;
    rt->E = 8192;
    rt->aring = (float *)calloc((size_t)RT_ACAP * RT_CH, sizeof(float));
    rt->ev = (uint8_t *)calloc(RT_EVCAP, 1);
    rt->mo = (uint8_t *)calloc(RT_MOCAP, 1);
    if (!rt->aring || !rt->ev || !rt->mo || (rate != 48000.0 && rt_kernel(rt))) {
        dt_errf(err, errlen, "out of memory");
        dt_rt_free(rt);
        return NULL;
    }
    memcpy((void *)&rt->master_bits, &m, 8);
    rt->master_applied = 0.25;
    dt_engine_set_volume(rt->e, 0.25);
    dt_engine_set_audio_sink(rt->e, rt_sink, rt);
    memset(rt->leds, 0xFF, sizeof rt->leds);
    atomic_store(&rt->limit48, rt->E);
    /* il motore parte gia' avanti di E: il primo blocco dell'host lo trova pronto */
    while (rt_step(rt))
        ;
    rt_publish(rt);
    snprintf(rt->status, sizeof rt->status, "running");
    if (sem_init_(&rt->wake) || sem_init_(&rt->progress) || sem_init_(&rt->saved)) {
        dt_errf(err, errlen, "cannot create semaphores");
        dt_rt_free(rt);
        return NULL;
    }
#if defined(_WIN32)
    rt->thread = CreateThread(NULL, 0, rt_thread, rt, 0, NULL);
    rt->has_thread = rt->thread != NULL;
#else
    rt->has_thread = pthread_create(&rt->thread, NULL, rt_thread, rt) == 0;
#endif
    if (!rt->has_thread) {
        dt_errf(err, errlen, "cannot start the emulation thread");
        dt_rt_free(rt);
        return NULL;
    }
    return rt;
}

void dt_rt_free(dt_rt *rt)
{
    if (!rt)
        return;
    if (rt->has_thread) {
        atomic_store(&rt->quit, 1);
        sem_post_(&rt->wake);
#if defined(_WIN32)
        WaitForSingleObject(rt->thread, INFINITE);
        CloseHandle(rt->thread);
#else
        pthread_join(rt->thread, NULL);
#endif
        sem_destroy_(&rt->wake);
        sem_destroy_(&rt->progress);
        sem_destroy_(&rt->saved);
    }
    dt_engine_free(rt->e);
    dt_fw_free(&rt->fw);
    free(rt->aring);
    free(rt->ev);
    free(rt->mo);
    free(rt->kernel);
    free(rt->save_buf);
    free(rt);
}

/* la stessa latenza prima che il motore esista (il primo avvio dal firmware) */
uint32_t dt_rt_latency_for(double rate, uint32_t max_frames)
{
    double ratio = 48000.0 / rate;
    int64_t n48 = (int64_t)ceil((max_frames < 1024 ? max_frames : 1024) * ratio);
    int64_t X = n48 + RT_BURST48 + 2 * RT_CHUNK48 + RT_TAPS + 64 + 1024;
    return (uint32_t)ceil((double)X / ratio);
}

uint32_t dt_rt_latency(dt_rt *rt)
{
    double l = (double)rt->X / rt->ratio;
    return (uint32_t)ceil(l);
}

static int64_t to48(dt_rt *rt, int64_t h)
{
    if (rt->rate == 48000.0)
        return h;
    return (int64_t)floor((double)h * rt->ratio);
}

int dt_rt_midi(dt_rt *rt, uint32_t offset, const uint8_t *data, size_t n)
{
    uint64_t head = atomic_load_explicit(&rt->ev_head, memory_order_relaxed);
    uint64_t tail = atomic_load_explicit(&rt->ev_tail, memory_order_acquire);
    int64_t t = to48(rt, rt->host_frames + offset) + rt->E;
    uint8_t hdr[10];
    size_t k;
    if (n > 250 || RT_EVCAP - (head - tail) < 10 + n)
        return DT_ERR_NOMEM;
    memcpy(hdr, &t, 8);
    hdr[8] = (uint8_t)n;
    hdr[9] = (uint8_t)(n >> 8);
    for (k = 0; k < 10; k++)
        rt->ev[(head + k) & (RT_EVCAP - 1)] = hdr[k];
    for (k = 0; k < n; k++)
        rt->ev[(head + 10 + k) & (RT_EVCAP - 1)] = data[k];
    atomic_store_explicit(&rt->ev_head, head + 10 + n, memory_order_release);
    return DT_OK;
}

int dt_rt_render(dt_rt *rt, uint32_t n, float *const *outs, int nouts, int offline)
{
    int64_t c = rt->host_frames, need_k, written;
    int missing = 0, ch;
    uint32_t i;
    int64_t new_limit = to48(rt, c + n) + rt->E;
    atomic_store_explicit(&rt->limit48, new_limit, memory_order_release);
    sem_post_(&rt->wake);
    /* l'ultimo campione a 48 kHz che serve */
    need_k = to48(rt, c + n - 1) + rt->E - rt->X + (rt->rate != 48000.0 ? RT_TAPS : 0);
    if (offline) {
        while ((written = atomic_load_explicit(&rt->awritten, memory_order_acquire)) <= need_k
               && !atomic_load(&rt->halted)) {
            int64_t t = atomic_load(&rt->engine_t48);
            if (t + RT_CHUNK48 > new_limit)
                break;                               /* il motore non puo' andare oltre */
            sem_post_(&rt->wake);
            sem_wait_ms(&rt->progress, 50);
        }
    }
    written = atomic_load_explicit(&rt->awritten, memory_order_acquire);
    for (i = 0; i < n; i++) {
        int64_t h = c + i;
        for (ch = 0; ch < nouts && ch < RT_CH; ch++) {
            float v;
            if (!outs[ch])
                continue;
            if (rt->rate == 48000.0) {
                v = rt_sample(rt, h + rt->E - rt->X, ch, written, &missing);
            } else {
                double p = (double)h * rt->ratio + (double)(rt->E - rt->X);
                int64_t k0 = (int64_t)floor(p);
                double frac = p - (double)k0;
                int ph = (int)(frac * RT_PHASES + 0.5), k;
                const float *row = rt->kernel + (size_t)ph * 2 * RT_TAPS;
                float acc = 0;
                for (k = 0; k < 2 * RT_TAPS; k++)
                    acc += row[k] * rt_sample(rt, k0 + k - RT_TAPS + 1, ch, written, &missing);
                v = acc;
            }
            outs[ch][i] = v;
        }
    }
    if (missing)
        atomic_fetch_add(&rt->underruns, 1);
    rt->host_frames = c + n;
    return missing ? 1 : 0;
}

int dt_rt_panel(dt_rt *rt, int kind, int code, int arg)
{
    uint64_t head = atomic_load_explicit(&rt->pan_head, memory_order_relaxed);
    uint64_t tail = atomic_load_explicit(&rt->pan_tail, memory_order_acquire);
    if (head - tail >= RT_PANCAP)
        return DT_ERR_NOMEM;
    rt->pan[head & (RT_PANCAP - 1)].kind = kind;
    rt->pan[head & (RT_PANCAP - 1)].code = code;
    rt->pan[head & (RT_PANCAP - 1)].arg = arg;
    atomic_store_explicit(&rt->pan_head, head + 1, memory_order_release);
    sem_post_(&rt->wake);
    return DT_OK;
}

void dt_rt_set_volume(dt_rt *rt, double master)
{
    uint64_t b;
    memcpy(&b, &master, 8);
    atomic_store(&rt->master_bits, b);
}

uint64_t dt_rt_screen(dt_rt *rt, uint8_t out[128 * 64], uint8_t leds[8])
{
    for (;;) {
        uint64_t s0 = atomic_load_explicit(&rt->seq, memory_order_acquire), v;
        if (s0 & 1)
            continue;
        if (out)
            memcpy(out, rt->screen, sizeof rt->screen);
        if (leds)
            memcpy(leds, rt->leds, sizeof rt->leds);
        v = rt->screen_ver * 1000003u + rt->leds_ver;
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&rt->seq, memory_order_relaxed) == s0) {
            if (leds)
                atomic_fetch_add(&rt->leds_taken, 1);      /* take_leds: si riaccumula */
            return v;
        }
    }
}

size_t dt_rt_midi_out(dt_rt *rt, uint8_t *buf, size_t max, uint32_t *offset, uint32_t frames)
{
    uint64_t head = atomic_load_explicit(&rt->mo_head, memory_order_acquire);
    uint64_t tail = atomic_load_explicit(&rt->mo_tail, memory_order_relaxed);
    uint8_t hdr[10];
    size_t n, k;
    int64_t t, h48;
    if (head == tail)
        return 0;
    for (k = 0; k < 10; k++)
        hdr[k] = rt->mo[(tail + k) & (RT_MOCAP - 1)];
    memcpy(&t, hdr, 8);
    n = (size_t)(hdr[8] | hdr[9] << 8);
    if (n > max)
        return 0;
    for (k = 0; k < n; k++)
        buf[k] = rt->mo[(tail + 10 + k) & (RT_MOCAP - 1)];
    atomic_store_explicit(&rt->mo_tail, tail + 10 + n, memory_order_release);
    /* istante dell'host di chi ascolta: t48 - E + X (come l'audio) */
    h48 = t - rt->E + rt->X;
    {
        int64_t c48 = to48(rt, rt->host_frames - frames);
        int64_t off = h48 - c48;
        if (off < 0)
            off = 0;
        if (off >= (int64_t)frames)
            off = frames ? frames - 1 : 0;
        *offset = (uint32_t)off;
    }
    return n;
}

int64_t dt_rt_underruns(dt_rt *rt)
{
    return atomic_load(&rt->underruns);
}

const char *dt_rt_status(dt_rt *rt)
{
    return atomic_load(&rt->halted) ? rt->status : "running";
}

dt_engine *dt_rt_engine(dt_rt *rt)
{
    return rt->e;
}

void dt_rt_offsets(dt_rt *rt, int64_t *E, int64_t *X)
{
    *E = rt->E;
    *X = rt->X;
}

int64_t dt_rt_max_lag(dt_rt *rt)
{
    return rt->max_lag * 1000000 + atomic_load(&rt->max_step_ns) / 1000;   /* diagnostica: lag e us */
}
