// DigitonePlugin.cpp -- vedi DigitonePlugin.hpp. La logica e' quella dell'AU
// (au/Extension/DigitoneAudioUnit.mm: bus, Main Volume, Sync DAW, MIDI in/out) con
// l'avvio dal solo firmware dell'utente (cartella, cache) e la macchina unica sul
// disco come il VST3 del Digitakt II.
#include "DigitonePlugin.hpp"
#include "NativeBuild.hpp"
#include "DistrhoPluginUtils.hpp"
#include "extra/Base64.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

START_NAMESPACE_DISTRHO

namespace {

bool logOn() { return std::getenv("DTCORE_LOG") != nullptr; }

int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* rt.c parte con il volume gia' "applicato" a 0,25: 0,25 esatto non arriverebbe al
   motore. Il successivo double va bene: il motore lo usa in float32 (0,25f). Come l'AU. */
double rtVolume(double v) { return v == 0.25 ? std::nextafter(0.25, 1.0) : v; }

} // namespace

// ------------------------------------------------------------------ firmware
bool DigitonePlugin::installFirmware(const std::string& path, std::string& msg)
{
    const bool ok = dnfw::install(dnplat::fs::path(path), dnplat::firmwareDir(), msg);
    if (ok) {
        fRetry = true;
        fStartPending = true;
        wake();
    }
    return ok;
}

void DigitonePlugin::wake()
{
    {
        std::lock_guard<std::mutex> g(fWakeMx);
    }
    fWakeCv.notify_all();
}

void DigitonePlugin::setStatus(Stage st, const std::string& text)
{
    if (logOn()) std::fprintf(stderr, "dncore: stato %d: %s\n", int(st), text.c_str());
    fStage = st;
    fPrepStartMs = st == kPreparing ? (fPrepStartMs.load() ? fPrepStartMs.load() : nowMs()) : 0;
    std::lock_guard<std::mutex> g(fLock);
    fStatus = text;
}

EngineStatus DigitonePlugin::status()
{
    EngineStatus s;
    {
        std::lock_guard<std::mutex> g(fLock);
        s.text = fStatus;
        s.detail = fBuildDetail;
        s.warning = fStage.load() == kNeedFirmware ? fFirmwareProblem : fRunningWarning;
        if (fStage.load() == kRunning && nowMs() < fSlowUntilMs.load())
            s.warning += (s.warning.empty() ? "" : "; ") + std::string(dnstr::kWarnSlowDsp);
    }
    s.firmwareDir = dnplat::firmwareDir().u8string();
    switch (fStage.load()) {
    case kIdle: s.kind = EngineStatus::Starting; break;
    case kNeedFirmware: s.kind = EngineStatus::NoFirmware; break;
    case kWaitOwner: s.kind = EngineStatus::Busy; break;
    case kPreparing: s.kind = EngineStatus::Booting; break;
    case kRunning: s.kind = EngineStatus::Running; break;
    case kError: s.kind = EngineStatus::Error; break;
    case kNeedTools: s.kind = EngineStatus::NeedTools; break;
    }
    if (const int64_t t0 = fPrepStartMs.load()) s.bootSeconds = (nowMs() - t0) / 1000.0;
    s.percent = fProgress.load();
    if (dt_rt* rt = fRt.load()) {
        const char* r = dt_rt_status(rt);
        if (std::strcmp(r, "running")) { s.kind = EngineStatus::Error; s.text = r; }
        else if (s.kind != EngineStatus::Running) { s.kind = EngineStatus::Running; s.text = "running"; }
    }
    return s;
}

// ------------------------------------------------------------------ HostSync
// Come HostSync dell'AU: gli host non mandano Start/Stop/clock ai plugin, li
// generiamo dal trasporto e li mettiamo nel MIDI IN emulato all'istante giusto.
// Il clock a 24 ppq si ricalcola a ogni blocco dalla posizione in beat.
void DigitonePlugin::HostSync::add(uint32_t off, uint8_t a, uint8_t b, uint8_t c, uint8_t n)
{
    if (count < kMax) ev[count++] = SyncEv{off, n, {a, b, c}};
}

void DigitonePlugin::HostSync::run(const TimePosition& t, uint32_t frames, double rate)
{
    count = 0;
    if (!t.bbt.valid || rate <= 0 || t.bbt.ticksPerBeat <= 0) return;
    const double q = t.bbt.beatType > 0 ? 4.0 / t.bbt.beatType : 1.0;          // beat -> semiminime
    const double beat = ((t.bbt.bar - 1) * (double)t.bbt.beatsPerBar + (t.bbt.beat - 1)
                         + t.bbt.tick / t.bbt.ticksPerBeat) * q;
    double tempo = t.bbt.beatsPerMinute > 0 ? t.bbt.beatsPerMinute * q : 120;
    const bool moving = t.playing;
    const double spb = tempo / 60.0 / rate;                                     // semiminime per campione
    auto sppContinue = [&](uint32_t off) {
        const int spp = static_cast<int>(beat * 4 + 1e-9);                       // in sedicesimi
        add(off, 0xF2, spp & 0x7F, (spp >> 7) & 0x7F, 3);
        add(off, 0xFB);                                                         // Continue
    };
    if (!started) {
        started = true;
        if (!moving) add(0, 0xFC);                                              // la DAW e' ferma: Stop
    }
    if (moving && !wasMoving) {
        if (beat < 1e-6) add(0, 0xFA);                                          // Start
        else sppContinue(0);
        nextTick = static_cast<int64_t>(std::ceil(beat * 24 - 1e-9));
    } else if (moving && std::fabs(beat - expectedBeat) > 1.0 / 48) {
        add(0, 0xFC);                                                           // loop o salto
        sppContinue(0);
        nextTick = static_cast<int64_t>(std::ceil(beat * 24 - 1e-9));
    }
    if (moving) {
        const double end = beat + frames * spb;
        for (; nextTick / 24.0 < end; ++nextTick) {                             // clock
            // + 1e-6: un clock che cade esattamente su un campione (120 BPM a 48 kHz: ogni
            // 1000) non deve finire un campione prima per l'errore di arrotondamento di beat
            double off = (nextTick / 24.0 - beat) / spb + 1e-6;
            if (off < 0) off = 0;
            uint32_t o = static_cast<uint32_t>(off);
            if (o >= frames) o = frames - 1;
            add(o, 0xF8);
        }
        expectedBeat = end;
    } else if (wasMoving) {
        add(0, 0xFC);                                                           // Stop
    }
    wasMoving = moving;
}

// ------------------------------------------------------------------ plugin
DigitonePlugin::DigitonePlugin()
    : Plugin(kParamCount, 0, 1)
{
    if (isDummyInstance()) return;
    dnplat::firmwareDir();                                 // la cartella c'e' sempre, anche vuota
    fDrive = std::make_shared<DriveModel>([this] { return fRt.load(); });
    fDrive->onChange = [this] { requestSave(); };          // +DRIVE cambiato dal pannello: si salva subito
    fWorker = std::thread([this] { workerLoop(); });
}

DigitonePlugin::~DigitonePlugin()
{
    fCancel = 1;
    {
        std::lock_guard<std::mutex> g(fWakeMx);
        fQuit = true;
    }
    fWakeCv.notify_all();
    if (fWorker.joinable()) fWorker.join();
    if (fDrive) fDrive->close();                 // il suo thread usa ancora il motore
    fDrive.reset();
    saveMachine();                               // si spegne: la macchina resta com'era
    swapEngine(nullptr);
    fMachineLock.release();
}

// Il thread di lavoro: avvia il motore (firmware, stato del DSP, sessione: mai nel
// thread della DAW), riguarda la cartella del firmware e la macchina unica mentre
// mancano, e salva: a richiesta (getState, +DRIVE) subito, altrimenti ogni minuto ma
// solo a Digitone fermo (trasporto fermo e niente MIDI da 5 s: la fotografia ferma il
// motore ~40 ms).
void DigitonePlugin::workerLoop()
{
    int since = 0;
    int64_t lastUnderruns = 0;
    int64_t tick = nowMs(), lastTry = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> g(fWakeMx);
            fWakeCv.wait_for(g, std::chrono::seconds(1), [this] {
                return fQuit || fSaveRequested.load() || fRetry.load() || fStartPending.load();
            });
            if (fQuit) return;
        }
        if (nowMs() - tick >= 1000) { tick = nowMs(); ++since; }
        const double want = fWantRate.load();
        const bool running = fRt.load() != nullptr && fRate.load() == want;
        if (want > 0 && !running) {
            const Stage st = fStage.load();
            const bool retryNow = fRetry.exchange(false);
            // manca il firmware o la macchina e' occupata: si riguarda ogni 2 s (o subito)
            const bool waiting = st == kNeedFirmware || st == kWaitOwner;
            const bool due = retryNow || st == kIdle || st == kRunning || (waiting && nowMs() - lastTry >= 2000) ||
                             (st == kError && fTriedRate != want);
            if (due) {
                fTriedRate = want;
                lastTry = nowMs();
                startEngine();
            }
        }
        if (fWantRate.load() == want && fStage.load() != kPreparing) fStartPending = false;
        // il DSP e' rimasto indietro in tempo reale (non offline, dove il motore aspetta): avviso per 8 s
        if (dt_rt* rt = fRt.load()) {
            const int64_t u = dt_rt_underruns(rt);
            if (u > lastUnderruns && !fOffline.load()) fSlowUntilMs = nowMs() + 8000;
            lastUnderruns = u;
        }
        if (fRt.load() && fOwner.load()) {
            if (fSaveRequested.exchange(false)) {
                saveMachine();
                since = 0;
            } else if (since >= 60 && nowMs() - fLastActivityMs.load() > 5000) {
                saveMachine();
                since = 0;
            }
        }
    }
}

// ------------------------------------------------------------------ la macchina unica
bool DigitonePlugin::MachineLock::acquire(const dnplat::fs::path& path)
{
#if defined(_WIN32)
    // aperto in esclusiva finche' l'istanza vive: Windows lo libera anche se il processo muore
    HANDLE f = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_ALWAYS,
                           FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) return false;
    h = f;
    return true;
#else
    const int f = ::open(path.c_str(), O_RDWR | O_CREAT, 0644);
    if (f < 0) return false;
    if (::flock(f, LOCK_EX | LOCK_NB) != 0) { ::close(f); return false; }   // anche fra istanze dello stesso processo
    fd = f;
    return true;
#endif
}

void DigitonePlugin::MachineLock::release()
{
#if defined(_WIN32)
    if (h) CloseHandle(static_cast<HANDLE>(h));
    h = nullptr;
#else
    if (fd >= 0) ::close(fd);
    fd = -1;
#endif
}

dnplat::fs::path DigitonePlugin::machinePath() const
{
    const auto dir = dnplat::machineDir();
    if (dir.empty() || fInfo.sha256.size() < 8) return dnplat::fs::path();
    // una macchina per firmware
    return dir / ("digitone2-" + fInfo.sha256.substr(0, 8) + ".dtsz");
}

bool DigitonePlugin::tryOwn()
{
    if (fOwner.load()) return true;
    if (isDummyInstance()) return false;
    const auto m = machinePath();
    if (m.empty()) return false;
    fOwner = fMachineLock.acquire(m.string() + ".lock");
    if (logOn()) std::fprintf(stderr, "dncore: machine %s (%s)\n", m.string().c_str(), fOwner ? "taken" : "busy");
    return fOwner.load();
}

void DigitonePlugin::requestSave()
{
    if (!fOwner.load()) return;
    {
        std::lock_guard<std::mutex> g(fWakeMx);
        fSaveRequested = true;
    }
    fWakeCv.notify_all();
}

bool DigitonePlugin::saveMachine()
{
    if (!fOwner.load()) return false;
    std::lock_guard<std::mutex> g(fSaveLock);
    dt_rt* rt = fRt.load();
    const auto m = machinePath();
    if (!rt || m.empty()) return false;
    uint8_t* buf = nullptr;
    size_t len = 0;
    bool ok = false;
    const auto t0 = std::chrono::steady_clock::now();
    if (dt_rt_save(rt, &buf, &len) == 0 && buf) ok = dnplat::writeFile(m, buf, len);
    std::free(buf);
    if (logOn())
        std::fprintf(stderr, "dncore: machine saved (%zu bytes, %.0f ms) %s\n", len,
                     std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(), ok ? "ok" : "ERROR");
    return ok;
}

// ------------------------------------------------------------------ parametri e stato
void DigitonePlugin::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    if (input) return;
    port.groupId = 0;
    port.name = index ? dnstr::kPortMainR : dnstr::kPortMainL;
    port.symbol = index ? "main_r" : "main_l";
}

void DigitonePlugin::initPortGroup(uint32_t, PortGroup& group)
{
    group.name = dnstr::kPortMain;
    group.symbol = "main";
}

void DigitonePlugin::initParameter(uint32_t index, Parameter& p)
{
    if (index == kParamVolume) {
        p.hints = kParameterIsAutomatable;
        p.name = dnstr::kParamVolume;
        p.symbol = "volume";
        p.ranges.min = 0.0f; p.ranges.max = 0.5f; p.ranges.def = 0.25f;
    } else if (index == kParamSync) {
        p.hints = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsInteger;
        p.name = dnstr::kParamSync;
        p.symbol = "syncdaw";
        p.ranges.min = 0.0f; p.ranges.max = 1.0f; p.ranges.def = 1.0f;
    }
}

void DigitonePlugin::initState(uint32_t, State& s)
{
    // come l'hardware la macchina e' una sola, sul disco (machinePath): il progetto della
    // DAW non la contiene. La chiave (quella dell'AU) resta per leggere i progetti vecchi:
    // se la macchina non c'e' ancora, il loro stato diventa la macchina.
    s.hints = kStateIsOnlyForDSP | kStateIsBase64Blob;
    s.key = "dncore_state";
    s.defaultValue = "";
    s.label = dnstr::kStateLabel;
}

float DigitonePlugin::getParameterValue(uint32_t index) const
{
    if (index == kParamVolume) return static_cast<float>(fMaster.load());
    if (index == kParamSync) return fSync.load() ? 1.0f : 0.0f;
    return 0.0f;
}

void DigitonePlugin::setParameterValue(uint32_t index, float value)
{
    if (index == kParamVolume) {
        const double v = value < 0 ? 0 : (value > 0.5f ? 0.5 : value);
        fMaster = v;
        if (dt_rt* rt = fRt.load()) dt_rt_set_master(rt, rtVolume(v));
    } else if (index == kParamSync) {
        fSync = value >= 0.5f;
    }
}

void DigitonePlugin::setMasterFromUI(double v)
{
    fMaster = v < 0 ? 0 : (v > 0.5 ? 0.5 : v);
    if (dt_rt* rt = fRt.load()) dt_rt_set_master(rt, rtVolume(fMaster.load()));
}

String DigitonePlugin::getState(const char* key) const
{
    // la DAW salva il progetto (o chiede lo stato per altro, anche spesso): la macchina
    // si salva nel thread di lavoro, mai qui
    if (!std::strcmp(key, "dncore_state")) const_cast<DigitonePlugin*>(this)->requestSave();
    return String();
}

void DigitonePlugin::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "dncore_state") || !value || !*value) return;
    std::vector<uint8_t> st = d_getChunkFromBase64String(value);
    if (st.size() < 16) return;
    std::lock_guard<std::mutex> g(fLock);
    fPendingState = std::move(st);     // conta solo se la macchina unica non c'e' ancora (startEngine)
}

// ------------------------------------------------------------------ avvio del motore
void DigitonePlugin::swapEngine(dt_rt* rt)
{
    if (rt) dt_rt_set_seq_reader(rt, seqmem::read);   // the pattern for the Elektron Emu view
    if (rt) {
        dt_rt_set_seq_hook(rt, seqmem::loop);                     // the page loop
        dt_rt_set_seq_arg(rt, fLoopPage.load());
    }
    dt_rt* old = fRt.exchange(rt);
    // run() ha caricato il vecchio puntatore prima dello scambio: si aspetta che finisca
    while (fInRun.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (old) dt_rt_free(old);
}

void DigitonePlugin::progressCb(void* ctx, int percent, const char* what)
{
    auto* self = static_cast<DigitonePlugin*>(ctx);
    self->fProgress = percent < 0 ? 0 : (percent > 100 ? 100 : percent);
    (void)what;
}

void DigitonePlugin::installTools()
{
    dnnative::openToolInstaller();
}

// Il DSP nativo: la libreria costruita su questo computer dal firmware dell'utente e legata ad esso
// (Native/dsp-<hash della sezione DSP>-n1.*). Se c'e', la si carica; se no la si costruisce adesso
// (NativeBuild.cpp: traduzione del programma DSP in C e compilazione col compilatore del computer). Finche' non
// e' pronta il primo avvio resta nella scheda "First start" con la barra; se non si puo' (nessun compilatore,
// rete, compilazione fallita) il DSP gira con l'interprete e la barra lo dice. DN_NATIVE=0: solo l'interprete
// (prove). La libreria personale di vst/scripts/dsp-nativo.sh (Native/dsp-native.dylib, solo Mac) si usa se
// non c'e' quella costruita dal plugin; non si tocca mai.
bool DigitonePlugin::ensureNative()
{
    if (fNativeFailed) return true;                                   // riavvio del motore (frequenza): non si ritenta
    fNativeNote.clear();
    fPersonalNative = false;
    {
        std::lock_guard<std::mutex> g(fLock);
        fBuildDetail.clear();
    }
    if (dnplat::env("DN_NATIVE") == "0") return true;
    if (std::strcmp(dn_dsp_backend_name(), "nativo") == 0) return true;      // gia' caricata in questo processo
    char err[512] = "", hash[65] = "";
    if (dn_dsp_program_hash(fSyx.data(), fSyx.size(), hash, err, sizeof err) != 0) return true;
    const dnplat::fs::path nd = dnplat::nativeDir();
    if (nd.empty()) return true;
    dnplat::makeDirs(nd);
    const dnplat::fs::path lib = nd / dnnative::libName(hash);
    auto load = [&](const dnplat::fs::path& p) {
        char e[256] = "";
        const int r = dn_dsp_native_load(p.string().c_str(), e, sizeof e);
        std::fprintf(stderr, "Digitone II: native DSP %s %s\n", r == 0 ? "loaded" : "not usable:", e);
        return r == 0;
    };
    if (dnplat::fileExists(lib)) {
        if (load(lib)) return true;
        std::error_code ec;
        dnplat::fs::remove(lib, ec);                                  // illeggibile: si rifa
    }
#if defined(__APPLE__)
    const dnplat::fs::path personal = nd / "dsp-native.dylib";
    if (dnplat::fileExists(personal) && load(personal)) {
        fPersonalNative = true;
        return true;
    }
#endif
    size_t dlen = 0;
    const uint8_t* dnds = dn_dsp_state_get(&dlen);
    if (!dnds) return true;
    dnnative::Hooks hooks;
    hooks.cancel = reinterpret_cast<volatile int*>(&fCancel);
    hooks.progress = [this](int pct, const std::string& text) {
        fProgress = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
        std::lock_guard<std::mutex> g(fLock);
        fBuildDetail = text;
    };
    for (;;) {
        if (fSkipNative.load()) {
            fNativeNote = dnstr::kWarnNoNativeSkipped;
            fNativeFailed = true;
            return true;
        }
        setStatus(kPreparing, dnstr::kStatusBuilding);
        std::string detail;
        const dnnative::Result r =
            dnnative::build(dnds, dlen, lib, dnplat::cacheDir() / "dsp-build", dnplat::toolchainDir(), hooks, detail);
        {
            std::lock_guard<std::mutex> g(fLock);
            fBuildDetail.clear();
        }
        fProgress = -1;
        if (r == dnnative::Result::Ready) {
            if (!load(lib)) fNativeNote = dnstr::warnNoNative("the library built here does not load");
            return true;
        }
        if (r == dnnative::Result::Cancelled || fCancel.load()) return false;
        if (r == dnnative::Result::NeedTools) {
            {
                std::lock_guard<std::mutex> g(fLock);
                fBuildDetail = detail;
            }
            setStatus(kNeedTools, dnstr::kStatusNeedTools);
            std::string why;
            while (!fCancel.load() && !fSkipNative.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                if (dnnative::compilerAvailable(dnplat::cacheDir() / "dsp-build" / "probe", why)) break;
            }
            if (fCancel.load()) return false;
            continue;                                                // gli strumenti ci sono (o "Use slow mode")
        }
        fNativeNote = dnstr::warnNoNative(detail);
        fNativeFailed = true;
        std::fprintf(stderr, "Digitone II: %s\n", fNativeNote.c_str());
        return true;
    }
}

bool DigitonePlugin::startEngine()
{
    const double rate = fWantRate.load();
    const uint32_t frames = fWantFrames.load();
    if (rate <= 0) return false;

    // 1. il firmware dell'utente
    if (fSyx.empty()) {
        std::vector<uint8_t> syx;
        dnfw::Info info;
        std::string problem;
        const bool found = fFinder.find(dnplat::firmwareDir(), syx, info, problem);
        {
            std::lock_guard<std::mutex> g(fLock);
            fFirmwareProblem = problem;
        }
        if (!found) {
            setStatus(kNeedFirmware, dnstr::kStatusFirmwareMissing);
            return false;
        }
        fSyx = std::move(syx);
        fInfo = info;
    }

    // 2. un solo Digitone: questa istanza aspetta
    if (!tryOwn()) {
        setStatus(kWaitOwner, dnstr::kStatusBusy);
        return false;
    }
    if (!dt_jit_available()) {
        setStatus(kError, dnstr::kStatusNoJit);
        return false;
    }
    // 3. lo stato del DSP: dalla cache, o fatto adesso (la prima volta, lungo)
    setStatus(kPreparing, dnstr::kStatusBooting);
    fProgress = 0;
    char err[512] = "";
    const std::string cache = dnplat::cacheDir().string();
    int r = dn_prepare_dsp(fSyx.data(), fSyx.size(), cache.c_str(), reinterpret_cast<volatile int*>(&fCancel),
                           progressCb, this, err, sizeof err);
    if (r != 0) {
        fProgress = -1;
        if (!fCancel.load()) setStatus(kError, dnstr::prepareError(err));
        return false;
    }
    // 3b. il DSP nativo (costruito qui la prima volta), prima della sessione cosi' anche il suo boot lo usa
    if (!ensureNative()) return false;
    setStatus(kPreparing, dnstr::kStatusBooting);
    fProgress = 0;

    // 4. lo stato di partenza del ColdFire: lo stato attuale (cambio di frequenza), la
    //    macchina unica, un progetto vecchio, la sessione iniziale (dalla cache o dal boot)
    std::vector<uint8_t> state;
    bool fromMachine = false, generated = false;
    if (dt_rt* cur = fRt.load()) {
        uint8_t* buf = nullptr;
        size_t len = 0;
        if (dt_rt_save(cur, &buf, &len) == 0 && buf) state.assign(buf, buf + len);
        std::free(buf);
    }
    const auto mpath = machinePath();
    if (state.empty() && !mpath.empty()) fromMachine = dnplat::readFile(mpath, state);
    if (state.empty()) {
        std::lock_guard<std::mutex> g(fLock);
        state = fPendingState;
    }
    dt_rt* rt = nullptr;
    for (int attempt = 0; attempt < 2 && !rt; ++attempt) {
        if (state.empty()) {
            uint8_t* buf = nullptr;
            size_t len = 0;
            r = dn_make_session(fSyx.data(), fSyx.size(), cache.c_str(), reinterpret_cast<volatile int*>(&fCancel),
                                progressCb, this, &buf, &len, err, sizeof err);
            if (r != 0) {
                fProgress = -1;
                if (!fCancel.load()) setStatus(kError, dnstr::prepareError(err));
                return false;
            }
            state.assign(buf, buf + len);
            std::free(buf);
            generated = true;
        }
        err[0] = 0;
        rt = dt_rt_new_mem(fSyx.data(), fSyx.size(), state.data(), state.size(), rate, frames, err, sizeof err);
        if (!rt) {
            if (logOn()) std::fprintf(stderr, "Digitone II: state discarded (%s)\n", err);
            if (fromMachine && !mpath.empty()) {       // illeggibile: da parte, si riparte dalla sessione iniziale
                std::error_code ec;
                dnplat::fs::rename(mpath, mpath.string() + ".bad", ec);
            }
            state.clear();
            fromMachine = false;
            if (attempt == 1 || generated) break;
        }
    }
    fProgress = -1;
    if (!rt) {
        setStatus(kError, dnstr::engineError(err));
        return false;
    }
    dt_rt_set_master(rt, rtVolume(fMaster.load()));
    swapEngine(rt);
    fRate = rate;
    fRtFrames = frames;
    setLatency(dt_rt_latency(rt));
    {
        std::string w = fInfo.warning;
        if (fPersonalNative) w += (w.empty() ? "" : "; ") + std::string(dnstr::kWarnNativeDsp);
        if (!fNativeNote.empty()) w += (w.empty() ? "" : "; ") + fNativeNote;
        std::lock_guard<std::mutex> g(fLock);
        fRunningWarning = w;
    }
    setStatus(kRunning, "running");
    if (!fromMachine) requestSave();                // la prima volta lo stato diventa la macchina
    return true;
}

void DigitonePlugin::activate()
{
    const uint32_t frames = getBufferSize();
    fMaxFrames = frames;
    fBuf.assign(static_cast<size_t>(2) * frames, 0.0f);
    for (int i = 0; i < 2; ++i) fMono[i] = fBuf.data() + static_cast<size_t>(i) * frames;
    fHostSync = HostSync();
    if (isDummyInstance()) return;
    fWantRate = getSampleRate();
    fWantFrames = frames;
    setLatency(dt_rt_latency_for(getSampleRate(), frames));   // subito: la compensazione dell'host non aspetta il motore
    fStartPending = true;
    wake();
}

void DigitonePlugin::run(const float**, float** outputs, uint32_t frames, const MidiEvent* midi, uint32_t midiCount)
{
    // export della DAW (VST3 V3_OFFLINE): mentre il motore parte si aspetta, invece di
    // perdere i primi campioni (non per mancanza di firmware o macchina occupata)
    if (isRenderingOffline() && !fRt.load()) {
        const auto t0 = std::chrono::steady_clock::now();
        while (!fRt.load() && fStartPending.load() && fStage.load() != kNeedFirmware && fStage.load() != kWaitOwner &&
               fStage.load() != kError && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(900))
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    fInRun.store(1);
    dt_rt* rt = fRt.load();
    if (rt && frames <= fMaxFrames && fRate.load() == getSampleRate()) {
        if (fSync.load()) fHostSync.run(getTimePosition(), frames, getSampleRate());
        else fHostSync.count = 0;
        if (midiCount || getTimePosition().playing) fLastActivityMs.store(nowMs());
        // Start/Stop/clock del trasporto e MIDI dell'host in un solo ordine di tempo (a parita'
        // di campione prima il trasporto): dt_rt_midi accoda nell'ordine in cui lo si chiama
        {
            int si = 0;
            const int sn = fHostSync.count;
            auto sendSync = [&](const SyncEv& e) { dt_rt_midi(rt, e.off, e.b, e.n); };
            for (uint32_t i = 0; i < midiCount; ++i) {
                const MidiEvent& ev = midi[i];
                const uint32_t f = ev.frame < frames ? ev.frame : frames - 1;
                while (si < sn && fHostSync.ev[si].off <= f) sendSync(fHostSync.ev[si++]);
                const uint8_t* d = ev.size > MidiEvent::kDataSize ? ev.dataExt : ev.data;
                if (d && ev.size) dt_rt_midi(rt, f, d, ev.size);
            }
            while (si < sn) sendSync(fHostSync.ev[si++]);
        }
        fOffline.store(isRenderingOffline());
        dt_rt_render(rt, frames, fMono, 2, isRenderingOffline() ? 1 : 0);
        uint8_t mb[256];
        uint32_t off = 0;
        size_t n;
        while ((n = dt_rt_midi_out(rt, mb, sizeof mb, &off, frames)) > 0) {
            MidiEvent ev;
            ev.frame = off < frames ? off : frames - 1;
            ev.size = static_cast<uint32_t>(n);
            if (n <= MidiEvent::kDataSize) {
                std::memcpy(ev.data, mb, n);
                ev.dataExt = nullptr;
            } else {
                ev.dataExt = mb;                 // SysEx: l'host la copia dentro writeMidiEvent
            }
            if (!writeMidiEvent(ev)) break;
        }
        fInRun.store(0);
        std::memcpy(outputs[0], fMono[0], frames * sizeof(float));
        std::memcpy(outputs[1], fMono[1], frames * sizeof(float));
        return;
    }
    fInRun.store(0);
    std::memset(outputs[0], 0, frames * sizeof(float));
    std::memset(outputs[1], 0, frames * sizeof(float));
}

// ------------------------------------------------------------------ per la vista
void DigitonePlugin::setLoopPage(int page)
{
    fLoopPage.store(page);
    if (dt_rt* rt = fRt.load()) dt_rt_set_seq_arg(rt, page);
}

bool DigitonePlugin::seqData(SeqData& d)
{
    dt_rt* rt = fRt.load();
    if (!rt) return false;
    uint8_t buf[DT_SEQ_MAX];
    uint64_t v = 0;
    if (dt_rt_seq(rt, buf, sizeof buf, &v) != sizeof d) return false;
    std::memcpy(&d, buf, sizeof d);
    return d.valid != 0;
}

uint64_t DigitonePlugin::copyScreen(uint8_t screen[128 * 64], uint8_t leds[DN_LEDS])
{
    dt_rt* rt = fRt.load();
    if (!rt) return 0;
    uint8_t leds50[50];
    const uint64_t sv = dt_rt_screen(rt, screen, leds50);
    const uint64_t lv = dn_rt_leds(rt, leds);               // tutti e 53
    return sv ^ (lv << 40);
}

void DigitonePlugin::panel(int kind, int code, int arg)
{
    if (dt_rt* rt = fRt.load()) dt_rt_panel(rt, kind, code, arg);
}

Plugin* createPlugin()
{
    return new DigitonePlugin();
}

END_NAMESPACE_DISTRHO
