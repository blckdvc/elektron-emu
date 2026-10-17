/* dn_engine.c: i collegamenti col DSP SHARC del Digitone II, lato DSP.
 *
 *   DSPI2 (emu/dspi2.py Dspi2, percorso DMA: sul Digitone l'HLE dell'invio non si
 *   installa, la routine e' a un altro indirizzo): a ogni SERQ del canale 28 il frame
 *   dei parametri va al DSP vero, che gira subito fino alla richiesta del frame
 *   successivo (emu/dsplive.py DspSynth, live=False: il modo deterministico del
 *   Python); la sua risposta torna al ColdFire nell'RX.
 *   DSPI1 (emu/dspi1.py Dspi1Tail): la coda del frame, consegnata a nessuno
 *   (emu/audio.py non la usa), completata con il suo interrupt.
 * Il DSP e' core/dsp/dn_dsp.c (il programma SHARC+ ricompilato alla build). */

#define D2_SR 0xEC03802Cu
#define D2_TCF 0x80000000u
#define D2_EOQF 0x10000000u
#define AU_OUT_SCALE (1.0 / 2147483648.0)          /* emu/dsplive.py OUT_SCALE */
#define MIDI_HOLD_FRAMES 150                        /* emu/audio.py */

/* verso il motore in tempo reale (rt.c): canale 0 = sinistro, traccia 0 = destro */
static void sink_flush(e_audio *a)
{
    if (a->sink_n && a->sink)
        a->sink(a->sink_ctx, a->sink_l, &a->sink_r[0][0], a->sink_n);
    a->emitted += (int64_t)a->sink_n;
    a->sink_n = 0;
}

/* l'audio di un frame del DSP: contatori e CRC di tools/dncore_guiref.py
   (float32 LE come DspSynth.on_frame), poi verso il sink */
static void au_apply(dt_engine *e, const int32_t *pcm, int got)
{
    e_audio *a = &e->au;
    int i;
    if (got <= 0)
        return;
    for (i = 0; i < 2 * got; i++)
        a->out[i] = (float)((double)pcm[i] * AU_OUT_SCALE);
    a->samples += got;
    a->crc = e_crc(a->crc, a->out, (size_t)got * 2 * sizeof(float));
    if (a->sink) {
        float g = (float)a->gain;
        for (i = 0; i < got; i++) {
            a->sink_l[a->sink_n] = a->out[2 * i] * g;
            a->sink_r[0][a->sink_n] = a->out[2 * i + 1] * g;
            if (++a->sink_n == 512)
                sink_flush(a);
        }
    }
}

/* Dspi2.reply_bytes(n): DspSynth.reply, le parole a 16 bit scambiate per il ColdFire */
static void reply_swap(uint8_t *out, const uint8_t *raw, uint32_t n)
{
    uint32_t k;
    for (k = 0; k + 1 < n; k += 2) {
        out[k] = raw[k + 1];
        out[k + 1] = raw[k];
    }
    if (n & 1)
        out[n - 1] = raw[n - 1];
}

static void au_fail(dt_engine *e, int r, const char *msg)
{
    snprintf(e->au.err, sizeof e->au.err, "DSP: %s (%d)", msg, r);
    if (e->m)
        dtm_stop(e->m);
}

/* ---------------------------------------------- il DSP in un thread suo
 * Il ColdFire consegna il frame N e prosegue; il DSP lo elabora intanto. La risposta
 * del frame N (che il Python scrive nell'RX al trasferimento) si scrive "pigra":
 * prima del primo accesso del guest all'area RX (un hook di memoria su quell'area,
 * misurato senza costo), prima del frame N+1, a fine giro e prima di salvare. Il
 * firmware la legge ~88.650 istruzioni dopo il trasferimento (scratch/dn_rxprobe.py:
 * memcpy da 0x80001a20, 1374 parole), quindi il DSP lavora in parallelo per quasi
 * tutto il frame. Le uscite sono le stesse del modo sincrono, per costruzione (stesse
 * chiamate al DSP nello stesso ordine; stessi byte nella RAM prima di ogni accesso).
 * DN_DSP_SYNC=1: il modo sincrono, per l'A/B. */
#define DSPW_SPIN_NS 300000             /* il DSP aspetta il frame girando a vuoto fino a 0,3 ms */
#define DSPW_WAIT_SPIN_NS 20000000       /* il ColdFire, dopo il sonno stimato: fino a 20 ms */
#define DSPW_SLEEP_MIN_NS 120000
#define DSPW_SLEEP_MARGIN_NS 70000

static int64_t dspw_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

static void dsp_tune_thread(void)
{
#if defined(__APPLE__)
    const char *env = getenv("DN_DSP_CONSTRAINT");
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);   /* sui core di prestazione */
    mach_timebase_info_data_t tb;
    thread_time_constraint_policy_data_t pol;
    if (env && !strcmp(env, "0"))         /* DN_DSP_CONSTRAINT=0: solo la QoS */
        return;
    mach_timebase_info(&tb);
#define DN_MS(ms) ((uint32_t)((ms) * 1e6 * tb.denom / tb.numer))
    pol.period = DN_MS(256.0 / 48.0);                /* come emu/rtthread.py per il thread del DSP */
    /* calcolo quasi tutto il periodo: il DSP lavora ~80% del tempo (con 2,5 ms, come
       emu/rtthread.py, il kernel lo frena: 8,0 contro 6,9 s per 13.007 frame) */
    pol.computation = DN_MS(getenv("DN_DSP_COMP") ? atof(getenv("DN_DSP_COMP")) : 5.0);
    pol.constraint = DN_MS(256.0 / 48.0);
    pol.preemptible = 1;
#undef DN_MS
    thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol,
                      THREAD_TIME_CONSTRAINT_POLICY_COUNT);
#endif
}

/* il thread del ColdFire (chi chiama dt_engine_chunk), se DN_CF_CONSTRAINT=1 */
static _Thread_local int cf_tuned;
static void cf_tune_thread(dt_engine *e)
{
    const char *env = getenv("DN_CF_CONSTRAINT");
    cf_tuned = 1;
    /* il thread di emulazione di rt.c (non chi ha creato il motore): la classe tempo
       reale, come la Syntakt (senza, buchi sotto carico); DN_CF_CONSTRAINT=1 anche
       nel thread che l'ha creato (la riga di comando), =0 mai */
    if (env && !strcmp(env, "0"))
        return;
    if ((env && !strcmp(env, "1")) || !pthread_equal(pthread_self(), e->creator))
        dsp_tune_thread();
}

static void *dspw_main(void *arg)
{
    dt_engine *e = (dt_engine *)arg;
    e_dspw *w = &e->dspw;
    dsp_tune_thread();
    for (;;) {
        int64_t t0 = dspw_ns();
        uint64_t posted;
        while ((posted = atomic_load_explicit(&w->posted, memory_order_acquire))
                   == atomic_load_explicit(&w->done, memory_order_relaxed)
               && !atomic_load(&w->quit)) {
            if (dspw_ns() - t0 > DSPW_SPIN_NS) {
                pthread_mutex_lock(&w->mu);
                while (atomic_load(&w->posted) == atomic_load(&w->done) && !atomic_load(&w->quit))
                    pthread_cond_wait(&w->cv_work, &w->mu);
                pthread_mutex_unlock(&w->mu);
            }
        }
        if (atomic_load(&w->quit))
            break;
        {
            int64_t ta = dspw_ns(), d;
            atomic_store_explicit(&w->start_ns, ta, memory_order_relaxed);
            w->rc = dn_dsp_frame(e->au.dsp, w->frame, w->fn);
            d = dspw_ns() - ta;
            w->dsp_ns += d;
            /* durata tipica di un frame (media mobile), per l'attesa del ColdFire */
            atomic_store_explicit(&w->ema_ns, (atomic_load_explicit(&w->ema_ns, memory_order_relaxed) * 7 + d) / 8,
                                  memory_order_relaxed);
        }
        w->got = w->rc < 0 ? 0 : dn_dsp_pull(e->au.dsp, w->pcm, 4096);
        if (w->rn && w->rc >= 0) {
            memset(w->raw, 0, w->rn);
            dn_dsp_reply(e->au.dsp, w->raw, w->rn);
        }
        pthread_mutex_lock(&w->mu);
        atomic_store_explicit(&w->done, posted, memory_order_release);
        pthread_cond_signal(&w->cv_done);
        pthread_mutex_unlock(&w->mu);
    }
    return NULL;
}

static int dspw_start(dt_engine *e)
{
    e_dspw *w = &e->dspw;
    if (w->started)
        return 0;
    pthread_mutex_init(&w->mu, NULL);
    pthread_cond_init(&w->cv_work, NULL);
    pthread_cond_init(&w->cv_done, NULL);
    if (pthread_create(&w->th, NULL, dspw_main, e))
        return -1;
    w->started = 1;
    return 0;
}

static void dspw_stop(dt_engine *e)
{
    e_dspw *w = &e->dspw;
    if (!w->started)
        return;
    pthread_mutex_lock(&w->mu);
    atomic_store(&w->quit, 1);
    pthread_cond_signal(&w->cv_work);
    pthread_mutex_unlock(&w->mu);
    pthread_join(w->th, NULL);
    pthread_mutex_destroy(&w->mu);
    pthread_cond_destroy(&w->cv_work);
    pthread_cond_destroy(&w->cv_done);
    w->started = 0;
}

/* aspetta il frame in corso del DSP e ne applica i risultati: audio, risposta all'RX */
static void dsp_flush(dt_engine *e)
{
    e_dspw *w = &e->dspw;
    uint64_t posted = atomic_load_explicit(&w->posted, memory_order_relaxed);
    if (w->applied == posted)
        return;
    if (atomic_load_explicit(&w->done, memory_order_acquire) != posted) {
        /* il resto del frame del DSP si stima dalla durata tipica: si dorme fino a poco
           prima della fine, poi si gira a vuoto (un risveglio dal sonno costa decine di
           microsecondi su un frame di ~500) */
        int64_t t0 = dspw_ns();
        int64_t left = atomic_load_explicit(&w->ema_ns, memory_order_relaxed)
                       - (t0 - atomic_load_explicit(&w->start_ns, memory_order_relaxed));
        if (e->dsp_wait == 1 && left > DSPW_SLEEP_MIN_NS) {       /* sonno stimato, poi a vuoto */
            struct timespec ts;
            int64_t sl = left - DSPW_SLEEP_MARGIN_NS;
            ts.tv_sec = (time_t)(sl / 1000000000);
            ts.tv_nsec = (long)(sl % 1000000000);
            nanosleep(&ts, NULL);
        } else if (e->dsp_wait == 2) {                              /* il segnale del DSP */
            pthread_mutex_lock(&w->mu);
            while (atomic_load(&w->done) != posted)
                pthread_cond_wait(&w->cv_done, &w->mu);
            pthread_mutex_unlock(&w->mu);
        }
        while (atomic_load_explicit(&w->done, memory_order_acquire) != posted) {
            if (dspw_ns() - t0 > DSPW_WAIT_SPIN_NS) {        /* qualcosa non va: si dorme */
                pthread_mutex_lock(&w->mu);
                while (atomic_load(&w->done) != posted)
                    pthread_cond_wait(&w->cv_done, &w->mu);
                pthread_mutex_unlock(&w->mu);
            }
        }
        w->waits++;
        w->wait_ns += dspw_ns() - t0;
    }
    w->applied = posted;
    if (w->rc < 0) {
        au_fail(e, w->rc, dn_dsp_error(e->au.dsp));
        w->pending = 0;
        return;
    }
    au_apply(e, w->pcm, w->got);
    if (w->pending) {
        uint8_t *rep = (uint8_t *)malloc(w->rn ? w->rn : 1);
        if (rep) {
            reply_swap(rep, w->raw, w->rn);
            e_write(e, w->rx_dst, rep, w->rn);
            free(rep);
        }
        w->pending = 0;
    }
}

/* hook di memoria sull'area RX: un accesso del guest con la risposta in sospeso */
static void h_rx_access(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    e_dspw *w = &e->dspw;
    (void)uc; (void)ty; (void)v;
    if (w->pending && a + (uint64_t)size > w->rx_dst && a < (uint64_t)w->rx_dst + w->rn) {
        w->rx_flushes++;
        dsp_flush(e);
    }
}

/* DspSynth.on_frame (live=False) e il wrap di tools/dncore_guiref.py */
static void audio_on_frame(dt_engine *e, const uint8_t *frame, size_t n, uint32_t rx_dst, uint32_t rn)
{
    e_audio *a = &e->au;
    e_dspw *w = &e->dspw;
    a->frames++;
    if (e->dsp_sync) {
        int got, r;
        {
            int64_t ta = dspw_ns();
            r = dn_dsp_frame(a->dsp, frame, n);
            w->dsp_ns += dspw_ns() - ta;
        }
        if (r < 0) {
            au_fail(e, r, dn_dsp_error(a->dsp));
            return;
        }
        got = dn_dsp_pull(a->dsp, a->pcm, 4096);
        au_apply(e, a->pcm, got);
        if (rn) {
            uint8_t *raw = (uint8_t *)calloc(1, rn + 2), *rep = (uint8_t *)malloc(rn);
            if (raw && rep) {
                dn_dsp_reply(a->dsp, raw, rn);
                reply_swap(rep, raw, rn);
                e_write(e, rx_dst, rep, rn);
            }
            free(raw);
            free(rep);
        }
        return;
    }
    dsp_flush(e);                                  /* il frame precedente */
    if (dspw_start(e)) {
        au_fail(e, -10, "cannot start the DSP thread");
        return;
    }
    if (rn > sizeof w->raw)
        rn = sizeof w->raw;
    memset(w->frame, 0, sizeof w->frame);
    memcpy(w->frame, frame, n < sizeof w->frame ? n : sizeof w->frame);
    w->fn = n < sizeof w->frame ? n : sizeof w->frame;
    w->rn = rn;
    w->rx_dst = rx_dst;
    w->pending = rn != 0;
    if (rn) {                                      /* l'hook sull'area RX, una volta per area */
        int k, have = 0;
        for (k = 0; k < w->nhooked; k++)
            if (w->hooked_lo[k] == rx_dst && w->hooked_n[k] >= rn)
                have = 1;
        if (!have && w->nhooked < 8) {
            w->hooked_lo[w->nhooked] = rx_dst;
            w->hooked_n[w->nhooked++] = rn;
            e_hook_mem(e, UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, (void *)h_rx_access, e, rx_dst,
                       (uint64_t)rx_dst + rn - 1);
        }
    }
    pthread_mutex_lock(&w->mu);
    atomic_store_explicit(&w->posted, atomic_load(&w->posted) + 1, memory_order_release);
    pthread_cond_signal(&w->cv_work);
    pthread_mutex_unlock(&w->mu);
}

static void d2_deliver(dt_engine *e, const uint8_t *frame, size_t n, uint32_t rx_dst, uint32_t rn)
{
    e->dspi2_frames++;
    e->d2_crc = e_crc(e->d2_crc, frame, n);        /* il tap di tools/dncore_guiref.py */
    audio_on_frame(e, frame, n, rx_dst, rn);
}

/* Dspi2._on_sr: il W1C della SR (nel Python in C: dt2_mmio_w1c sulla cella) */
static void h_d2_sr_w1c(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty;
    if (size == 4 && a == D2_SR)
        *e->dspi2_sr &= ~(uint32_t)v;
}

static void d2_complete(dt_engine *e, uint8_t *tcd, int o)
{
    uint16_t csr;
    tcd[o + 0x14] = tcd[o + 0x1C];
    tcd[o + 0x15] = tcd[o + 0x1D];
    csr = e_be16(tcd + o + 0x1E);
    e_put16(tcd + o + 0x1E, (uint16_t)((csr | 0x0080) & ~0x0040));
}

/* Dspi2._on_serq (WriteFilter canale 28, un byte) -> _transfer */
static void h_d2_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t tcd[0x40];
    uint32_t src, count, i, n, rx_dst, rx_count;
    uint8_t *raw, *words;
    (void)uc; (void)ty; (void)a;
    if (!(size == 1 && (v & 0x3F) == 28))
        return;
    if (e_read(e, E_TCD_BASE + 28 * 0x20u, tcd, 0x40))
        return;
    src = e_be32(tcd + 0x20);
    count = e_be16(tcd + 0x20 + 0x14) & 0x7FFF;
    if (!count)
        return;
    raw = (uint8_t *)malloc(count * 4u);
    words = (uint8_t *)malloc(count * 2u);
    if (!raw || !words) {
        free(raw);
        free(words);
        return;
    }
    e_read(e, src, raw, count * 4u);
    n = count;
    for (i = 0; i < count; i++)
        if (raw[4 * i] & 0x08) {
            n = i + 1;
            break;
        }
    for (i = 0; i < n; i++) {
        words[2 * i] = raw[4 * i + 2];
        words[2 * i + 1] = raw[4 * i + 3];
    }
    /* la risposta del DSP: nel Python reply_bytes(rx_count * 2) subito dopo la consegna;
       qui dentro audio_on_frame (sincrono) o pigra (il DSP in un thread suo) */
    rx_dst = e_be32(tcd + 0x10);
    rx_count = e_be16(tcd + 0x14) & 0x7FFF;
    d2_deliver(e, words, n * 2u, rx_dst, rx_count * 2u);
    d2_complete(e, tcd, 0x20);                     /* _complete(TX), poi _complete(RX) */
    d2_complete(e, tcd, 0);
    e_write(e, E_TCD_BASE + 28 * 0x20u, tcd, 0x40);
    *e->dspi2_sr |= D2_TCF | D2_EOQF;
    free(raw);
    free(words);
}

/* emu/audio.py midi_hold: i byte MIDI restano in coda finche' il collegamento col DSP
   non e' partito (150 frame DSPI2 di questa esecuzione) */
static int midi_hold(dt_engine *e)
{
    return e->dspi2_frames < MIDI_HOLD_FRAMES;
}

/* ================================================== DSPI1: Dspi1Tail */
#define D1_TX_CHAN 15
#define D1_TAIL_VECTOR (64 + 8 + D1_TX_CHAN)
#define D1_TAIL_SEND 0x400CF694u
static const dn_vh D1_TAIL_VERIFY = {12, {0xa5, 0x99, 0x6c, 0x3c, 0xad, 0xa5, 0xc6, 0x5e, 0x27, 0xcb, 0xa9, 0xbd, 0x9c, 0x6e, 0x7a, 0xf9, 0x48, 0xbb, 0x2c, 0xd6, 0xa6, 0xa4, 0xeb, 0x6c, 0xda, 0x45, 0x5e, 0x08, 0x64, 0x2f, 0x7d, 0xa5}};   /* 12 byte del firmware: solo la loro impronta */
#define D1_RETRY 8000

static void h_d1_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t base = E_TCD_BASE + D1_TX_CHAN * 0x20u, n, src, i;
    uint8_t *raw, *words;
    uint16_t csr;
    uint32_t *sr;
    (void)uc; (void)ty; (void)a;
    if (!(size == 1 && !(v & 0x40) && (v & 0x3F) == D1_TX_CHAN))
        return;
    n = e_r16(e, base + 0x14) & 0x7FFF;
    if (!n)
        return;
    src = e_r32(e, base + 0x00);
    raw = (uint8_t *)malloc(n * 4u);
    words = (uint8_t *)malloc(n * 2u);
    if (!raw || !words) {
        free(raw);
        free(words);
        return;
    }
    e_read(e, src, raw, n * 4u);
    for (i = 0; i < n; i++) {
        words[2 * i] = raw[4 * i + 2];
        words[2 * i + 1] = raw[4 * i + 3];
    }
    e->d1.last_crc = e_crc(0, words, n * 2u);
    e->d1.frames++;
    e_w16(e, base + 0x14, e_r16(e, base + 0x1C));
    csr = e_r16(e, base + 0x1E);
    e_w16(e, base + 0x1E, (uint16_t)((csr | 0x0080) & ~0x0040));
    e->d1.pending_irq = (csr & 0x0002) != 0;          /* INTMAJOR */
    e->d1.hint = e->d1.pending_irq;
    sr = mmio_ref(e, 0xFC03C02Cu);
    if (sr)
        *sr |= 0x90000000u;
    else
        mmio_set(e, 0xFC03C02Cu, 0x90000000u);         /* TCF | EOQF su .get(sr, 0) */
    free(raw);
    free(words);
}

static int64_t d1_step(int64_t done)
{
    (void)done;
    return e_cur->d1.pending_irq ? D1_RETRY : -1;
}

static int d1_service(int64_t done)
{
    dt_engine *e = e_cur;
    int lvl;
    (void)done;
    if (e->d1.pending_irq) {
        lvl = e_irq_level(e, D1_TAIL_VECTOR, 1);
        if (!lvl)
            e->d1.pending_irq = 0;                     /* nessun gestore: niente da fare */
        else if (e_ipl(e) < lvl && e_raise(e, D1_TAIL_VECTOR, lvl))
            e->d1.pending_irq = 0;
    }
    e->d1.hint = e->d1.pending_irq;                    /* _update_hint */
    return 0;
}
