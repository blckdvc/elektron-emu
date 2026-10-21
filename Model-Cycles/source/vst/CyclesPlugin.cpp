// CyclesPlugin.cpp -- vedi CyclesPlugin.hpp. La logica e' quella dell'AU
// (au/Extension/CyclesAudioUnit.mm) e del VST3 del Digitakt II: il firmware
// NON e' nel bundle ma nella cartella dell'utente (FirmwareStore.hpp), il primo
// avvio e' il boot freddo in C (dt_coldboot_session) con cache, la macchina
// unica sta sul disco, Sync DAW, MIDI in/out, un bus stereo.
#include "CyclesPlugin.hpp"
#include "DistrhoPluginUtils.hpp"
#include "extra/Base64.hpp"

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

bool logOn() { return std::getenv("MC_LOG") != nullptr; }

int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

// ------------------------------------------------------------------ HostSync
// Come HostSync dell'AU: gli host non mandano Start/Stop/clock ai plugin, li
// generiamo dal trasporto e li mettiamo nel MIDI IN emulato all'istante giusto.
// Il clock a 24 ppq si ricalcola a ogni blocco dalla posizione in beat.
void CyclesPlugin::HostSync::run(dt_rt* rt, const TimePosition& t, uint32_t frames, double rate)
{
    if (!t.bbt.valid || rate <= 0 || t.bbt.ticksPerBeat <= 0) return;
    const double q = t.bbt.beatType > 0 ? 4.0 / t.bbt.beatType : 1.0;          // beat -> semiminime
    const double beat = ((t.bbt.bar - 1) * (double)t.bbt.beatsPerBar + (t.bbt.beat - 1)
                         + t.bbt.tick / t.bbt.ticksPerBeat) * q;
    double tempo = t.bbt.beatsPerMinute > 0 ? t.bbt.beatsPerMinute * q : 120;
    const bool moving = t.playing;
    const double spb = tempo / 60.0 / rate;                                     // semiminime per campione
    uint8_t b[3];
    auto sppContinue = [&](uint32_t off) {
        const int spp = static_cast<int>(beat * 4 + 1e-9);                       // in sedicesimi
        b[0] = 0xF2; b[1] = spp & 0x7F; b[2] = (spp >> 7) & 0x7F;
        dt_rt_midi(rt, off, b, 3);
        b[0] = 0xFB;                                                            // Continue
        dt_rt_midi(rt, off, b, 1);
    };
    if (!started) {
        started = true;
        if (!moving) { b[0] = 0xFC; dt_rt_midi(rt, 0, b, 1); }                  // la DAW e' ferma: Stop
    }
    if (moving && !wasMoving) {
        if (beat < 1e-6) { b[0] = 0xFA; dt_rt_midi(rt, 0, b, 1); }               // Start
        else sppContinue(0);
        nextTick = static_cast<int64_t>(std::ceil(beat * 24 - 1e-9));
    } else if (moving && std::fabs(beat - expectedBeat) > 1.0 / 48) {
        b[0] = 0xFC;                                                            // loop o salto
        dt_rt_midi(rt, 0, b, 1);
        sppContinue(0);
        nextTick = static_cast<int64_t>(std::ceil(beat * 24 - 1e-9));
    }
    if (moving) {
        const double end = beat + frames * spb;
        b[0] = 0xF8;                                                            // clock
        for (; nextTick / 24.0 < end; ++nextTick) {
            double off = (nextTick / 24.0 - beat) / spb;
            if (off < 0) off = 0;
            uint32_t o = static_cast<uint32_t>(off);
            if (o >= frames) o = frames - 1;
            dt_rt_midi(rt, o, b, 1);
        }
        expectedBeat = end;
    } else if (wasMoving) {
        b[0] = 0xFC;                                                            // Stop
        dt_rt_midi(rt, 0, b, 1);
    }
    wasMoving = moving;
}

// ------------------------------------------------------------------ plugin
CyclesPlugin::CyclesPlugin()
    : Plugin(kParamCount, 0, 1)
{
    if (isDummyInstance()) return;
    // ogni secondo: la macchina si e' liberata? e' comparso il firmware? Ogni
    // minuto, ma solo a Cycles fermo (trasporto fermo e niente MIDI da 5 s): la
    // fotografia ferma il motore ~40 ms
    fAutosave = std::thread([this] {
        std::unique_lock<std::mutex> g(fAutosaveMx);
        int since = 0;
        int64_t tick = nowMs();
        while (!fAutosaveStop) {
            fAutosaveCv.wait_for(g, std::chrono::seconds(1),
                                 [this] { return fAutosaveStop || fSaveRequested.load() || fRescan.load(); });
            if (fAutosaveStop) break;
            g.unlock();
            if (nowMs() - tick >= 1000) { tick = nowMs(); ++since; }
            if (!fRt.load() && !fBooting.load() && fWantRate > 0) {
                // niente motore: firmware messo nella cartella, o macchina liberata
                const bool force = fRescan.exchange(false);
                const bool changed = mcfw::signature() != fFwSig;
                if (force || changed || fPhase.load() == kBusy) {
                    if (force || changed) {
                        std::lock_guard<std::recursive_mutex> sg(fStartLock);
                        fSyx.clear();
                    }
                    startEngine(fWantRate, fWantFrames);
                }
            } else if (fSaveRequested.exchange(false)) {   // chiesto (getState): subito
                saveMachine();
                since = 0;
            } else if (since >= 60 && nowMs() - fLastActivityMs.load() > 5000) {
                saveMachine();
                since = 0;
            }
            g.lock();
        }
    });
}

CyclesPlugin::~CyclesPlugin()
{
    {
        std::lock_guard<std::mutex> g(fAutosaveMx);
        fAutosaveStop = true;
    }
    fAutosaveCv.notify_all();
    if (fAutosave.joinable()) fAutosave.join();
    fBootCancel = 1;
    if (fBoot.joinable()) fBoot.join();
    saveMachine();                               // si spegne: la macchina resta com'era
    swapEngine(nullptr);
    fMachineLock.release();
}

void CyclesPlugin::setStatus(Phase p, const std::string& s)
{
    std::lock_guard<std::mutex> g(fLock);
    fStatus = s;
    fPhase = p;
}

// ------------------------------------------------------------------ la macchina unica
bool CyclesPlugin::MachineLock::acquire(const std::string& path)
{
#if defined(_WIN32)
    // aperto in esclusiva finche' l'istanza vive: Windows lo libera anche se il processo muore
    HANDLE f = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_ALWAYS,
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

void CyclesPlugin::MachineLock::release()
{
#if defined(_WIN32)
    if (h) CloseHandle(static_cast<HANDLE>(h));
    h = nullptr;
#else
    if (fd >= 0) ::close(fd);
    fd = -1;
#endif
}

std::string CyclesPlugin::machinePath() const
{
    const std::string dir = mcfw::dataDir();
    if (dir.empty() || fSyx.empty()) return std::string();
    // una macchina per firmware
    return dir + mcfw::kSep + "macchina-" + mcfw::shortSha(fSyx) + "-g1.mcsz";
}

bool CyclesPlugin::tryOwn()
{
    if (fOwner.load()) return true;
    if (isDummyInstance()) return false;
    const std::string m = machinePath();
    if (m.empty()) return false;
    fOwner = fMachineLock.acquire(m + ".lock");
    if (logOn()) std::fprintf(stderr, "mc: macchina %s (%s)\n", m.c_str(), fOwner ? "presa" : "occupata");
    return fOwner.load();
}

void CyclesPlugin::requestSave()
{
    if (!fOwner.load()) return;
    {
        std::lock_guard<std::mutex> g(fAutosaveMx);
        fSaveRequested = true;
    }
    fAutosaveCv.notify_all();
}

bool CyclesPlugin::saveMachine()
{
    if (!fOwner.load()) return false;
    std::lock_guard<std::mutex> g(fSaveLock);
    dt_rt* rt = fRt.load();
    const std::string m = machinePath();
    if (!rt || m.empty()) return false;
    uint8_t* buf = nullptr;
    size_t len = 0;
    bool ok = false;
    const auto t0 = std::chrono::steady_clock::now();
    if (dt_rt_save(rt, &buf, &len) == 0 && buf) ok = mcfw::writeFile(m, buf, len);
    std::free(buf);
    if (logOn())
        std::fprintf(stderr, "mc: macchina salvata (%zu byte, %.0f ms) %s\n", len,
                     std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(), ok ? "ok" : "ERRORE");
    return ok;
}

// ------------------------------------------------------------------ parametri e stato
void CyclesPlugin::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    if (input) return;
    port.name = index ? "Main R" : "Main L";
    port.symbol = index ? "main_r" : "main_l";
}

void CyclesPlugin::initParameter(uint32_t index, Parameter& p)
{
    if (index == kParamVolume) {
        p.hints = kParameterIsAutomatable;
        p.name = "Volume";
        p.symbol = "volume";
        p.ranges.min = 0.0f; p.ranges.max = 1.0f; p.ranges.def = 0.25f;
    } else if (index == kParamSync) {
        p.hints = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsInteger;
        p.name = "Sync DAW";
        p.symbol = "syncdaw";
        p.ranges.min = 0.0f; p.ranges.max = 1.0f; p.ranges.def = 1.0f;
    }
}

void CyclesPlugin::initState(uint32_t, State& s)
{
    // come l'hardware la macchina e' una sola, sul disco (machinePath): il progetto
    // della DAW non la contiene (e quindi niente RAM del firmware nei progetti). La
    // chiave resta per leggere i progetti dell'AU: se la macchina non c'e' ancora, il
    // loro stato diventa la macchina.
    s.hints = kStateIsOnlyForDSP | kStateIsBase64Blob;
    s.key = "mccore_state";
    s.defaultValue = "";
    s.label = "Model:Cycles state";
}

float CyclesPlugin::getParameterValue(uint32_t index) const
{
    if (index == kParamVolume) return static_cast<float>(fVolume.load());
    if (index == kParamSync) return fSync.load() ? 1.0f : 0.0f;
    return 0.0f;
}

void CyclesPlugin::setParameterValue(uint32_t index, float value)
{
    if (index == kParamVolume) {
        const double v = value < 0 ? 0 : (value > 1.0f ? 1.0 : value);
        fVolume = v;
        if (dt_rt* rt = fRt.load()) dt_rt_set_volume(rt, v);
    } else if (index == kParamSync) {
        fSync = value >= 0.5f;
    }
}

void CyclesPlugin::setVolumeFromUI(double v)
{
    fVolume = v < 0 ? 0 : (v > 1.0 ? 1.0 : v);
    if (dt_rt* rt = fRt.load()) dt_rt_set_volume(rt, fVolume.load());
}

String CyclesPlugin::getState(const char* key) const
{
    // la DAW salva il progetto (o chiede lo stato per altro, anche spesso): la
    // macchina si salva nel thread di salvataggio, mai qui
    if (!std::strcmp(key, "mccore_state")) const_cast<CyclesPlugin*>(this)->requestSave();
    return String();
}

void CyclesPlugin::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "mccore_state") || !value || !*value) return;
    // progetto dell'AU con lo stato dentro: conta solo se la macchina unica non c'e' ancora
    {
        const std::string m = machinePath();
        if (!m.empty() && mcfw::fileExists(m)) return;
    }
    std::vector<uint8_t> st = d_getChunkFromBase64String(value);
    if (st.size() < 16) return;
    {
        std::lock_guard<std::mutex> g(fLock);
        fPendingState = std::move(st);
    }
    std::lock_guard<std::recursive_mutex> sg(fStartLock);
    if (fRt.load() && fRate > 0) {                 // gia' in moto: si riparte da quello stato
        const double rate = fRate;
        fRate = 0;
        startEngine(rate, fRtFrames);
    }
}

// ------------------------------------------------------------------ firmware
std::string CyclesPlugin::firmwareFolder() const
{
    return mcfw::firmwareDir();
}

std::string CyclesPlugin::firmwareNote()
{
    std::lock_guard<std::mutex> g(fLock);
    return fFwNote;
}

bool CyclesPlugin::loadFirmware()
{
    mcfw::Firmware fw = mcfw::find();
    std::lock_guard<std::mutex> g(fLock);
    fFwSig = mcfw::signature();
    if (fw.path.empty()) {
        fSyx.clear();
        fFwPath.clear();
        fFwNote.clear();
        fStatus = "No firmware: " + fw.error;
        fPhase = kNoFirmware;
        return false;
    }
    fSyx = std::move(fw.data);
    fFwPath = fw.path;
    fFwNote = fw.note;
    return true;
}

std::string CyclesPlugin::installFirmware(const std::string& path)
{
    const std::string err = mcfw::install(path);
    if (err.empty()) rescanFirmware();
    return err;
}

void CyclesPlugin::rescanFirmware()
{
    {
        std::lock_guard<std::mutex> g(fAutosaveMx);
        fRescan = true;
    }
    fAutosaveCv.notify_all();
}

std::string CyclesPlugin::coldBootCachePath() const
{
    const std::string dir = mcfw::cacheDir();
    if (dir.empty() || fSyx.empty()) return std::string();
    char name[160];
    std::snprintf(name, sizeof name, "primo-avvio-%s-%llu-%lld-%s.mcsz", mcfw::shortSha(fSyx).c_str(),
                  (unsigned long long)DT_COLDBOOT_RUNG, (long long)DT_COLDBOOT_SESSION, MCCORE_VERSION);
    return dir + mcfw::kSep + name;
}

// ------------------------------------------------------------------ motore
void CyclesPlugin::swapEngine(dt_rt* rt)
{
    if (rt) dt_rt_set_seq_reader(rt, seqmem::read);   // the pattern for the Elektron Emu view
    dt_rt* old = fRt.exchange(rt);
    // run() ha caricato il vecchio puntatore prima dello scambio: si aspetta che finisca
    while (fInRun.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (old) dt_rt_free(old);
}

int CyclesPlugin::bootPercent() const
{
    const double total = double(DT_COLDBOOT_RUNG) + double(DT_COLDBOOT_SESSION);
    const double p = 100.0 * double(fBootDone.load()) / total;
    return p < 0 ? 0 : (p > 99 ? 99 : int(p));
}

bool CyclesPlugin::startEngine(double rate, uint32_t frames)
{
    std::lock_guard<std::recursive_mutex> sg(fStartLock);
    if (fRt.load() && rate == fRate && frames <= fRtFrames) return true;
    fWantRate = rate;
    fWantFrames = frames;
    std::vector<uint8_t> src;
    if (dt_rt* cur = fRt.load()) {                 // cambio di frequenza: si riparte dallo stato attuale
        uint8_t* buf = nullptr;
        size_t len = 0;
        if (dt_rt_save(cur, &buf, &len) == 0 && buf) src.assign(buf, buf + len);
        std::free(buf);
    }
    if (fSyx.empty() && !loadFirmware()) return false;
    if (!tryOwn()) {                               // un solo Cycles: questa istanza aspetta
        setStatus(kBusy, "Model:Cycles is already open in another instance: close that one and this one starts by itself");
        return false;
    }
    bool fromMachine = false;
    if (src.empty()) {                             // la macchina unica, se c'e'
        const std::string m = machinePath();
        fromMachine = !m.empty() && mcfw::readFile(m, src);
    }
    if (src.empty()) {
        std::lock_guard<std::mutex> g(fLock);
        src = fPendingState;                       // un progetto dell'AU, o il primo avvio appena fatto
    }
    if (src.empty()) {
        // senza stato: il boot freddo dal solo firmware, una volta sola, poi dalla cache
        const std::string cache = coldBootCachePath();
        if (cache.empty() || !mcfw::readFile(cache, src)) {
            coldBoot(rate, frames);
            return true;
        }
    }
    char err[256] = "";
    dt_rt* rt = dt_rt_new_mem(fSyx.data(), fSyx.size(), src.data(), src.size(), rate, frames, err, sizeof err);
    if (!rt) {
        std::string msg = std::string("engine: ") + err;
        if (!fFwNote.empty()) msg = fFwNote + " (" + msg + ")";
        setStatus(kError, msg);
        return false;
    }
    dt_rt_set_volume(rt, fVolume.load());
    swapEngine(rt);
    fRate = rate;
    fRtFrames = frames;
    setLatency(dt_rt_latency(rt));
    if (!fromMachine) requestSave();             // la prima volta lo stato diventa la macchina
    setStatus(kRunning, "running");
    return true;
}

void CyclesPlugin::coldBoot(double rate, uint32_t frames)
{
    if (fBooting.exchange(true)) return;
    if (fBoot.joinable()) fBoot.join();
    fBootDone = 0;
    setStatus(kBooting, "First start: booting the firmware (once only)");
    setLatency(dt_rt_latency_for(rate, frames));
    fBoot = std::thread([this, rate, frames] {
        dt_firmware fwd;
        char err[256] = "";
        uint8_t* buf = nullptr;
        size_t len = 0;
        int r = -1;
        if (dt_fw_extract(fSyx.data(), fSyx.size(), &fwd, err, sizeof err) == 0) {
            r = dt_coldboot_session(&fwd, DT_COLDBOOT_RUNG, DT_COLDBOOT_SESSION, 1, &buf, &len,
                                    reinterpret_cast<volatile int*>(&fBootCancel),
                                    reinterpret_cast<volatile int64_t*>(&fBootDone), err, sizeof err);
            dt_fw_free(&fwd);
        }
        if (r == 0 && buf && !fBootCancel.load()) {
            const std::string cache = coldBootCachePath();
            if (!cache.empty()) mcfw::writeFile(cache, buf, len);
            {
                std::lock_guard<std::mutex> g(fLock);
                if (fPendingState.empty()) fPendingState.assign(buf, buf + len);
            }
            std::free(buf);
            fBooting = false;
            startEngine(rate, frames);
            return;
        }
        std::free(buf);
        fBooting = false;
        if (!fBootCancel.load()) {
            std::string msg = std::string("cold boot: ") + err;
            if (!fFwNote.empty()) msg = fFwNote + " (" + msg + ")";
            setStatus(kError, msg);
        }
    });
}

void CyclesPlugin::activate()
{
    const uint32_t frames = getBufferSize();
    fMaxFrames = frames;
    fHostSync = HostSync();
    fWantRate = getSampleRate();
    fWantFrames = frames;
    if (!fBooting.load()) startEngine(getSampleRate(), frames);
}

void CyclesPlugin::run(const float**, float** outputs, uint32_t frames, const MidiEvent* midi, uint32_t midiCount)
{
    fInRun.store(1);
    dt_rt* rt = fRt.load();
    if (rt) {
        if (fSync.load()) fHostSync.run(rt, getTimePosition(), frames, getSampleRate());
        if (midiCount || getTimePosition().playing) fLastActivityMs.store(nowMs());
        for (uint32_t i = 0; i < midiCount; ++i) {
            const MidiEvent& ev = midi[i];
            const uint8_t* d = ev.size > MidiEvent::kDataSize ? ev.dataExt : ev.data;
            if (d && ev.size) dt_rt_midi(rt, ev.frame < frames ? ev.frame : frames - 1, d, ev.size);
        }
        // export della DAW (VST3 V3_OFFLINE): si aspetta il motore invece di perdere campioni
        dt_rt_render(rt, frames, outputs, 2, isRenderingOffline() ? 1 : 0);
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
        return;
    }
    fInRun.store(0);
    for (uint32_t i = 0; i < 2; ++i) std::memset(outputs[i], 0, frames * sizeof(float));
}

// ------------------------------------------------------------------ per la vista
bool CyclesPlugin::seqData(SeqData& d)
{
    dt_rt* rt = fRt.load();
    if (!rt) return false;
    uint8_t buf[DT_SEQ_MAX];
    uint64_t v = 0;
    if (dt_rt_seq(rt, buf, sizeof buf, &v) != sizeof d) return false;
    std::memcpy(&d, buf, sizeof d);
    return d.valid != 0;
}

uint64_t CyclesPlugin::copyScreen(uint8_t screen[128 * 64], uint8_t leds[8])
{
    dt_rt* rt = fRt.load();
    return rt ? dt_rt_screen(rt, screen, leds) : 0;
}

void CyclesPlugin::panel(int kind, int code, int arg)
{
    if (dt_rt* rt = fRt.load()) dt_rt_panel(rt, kind, code, arg);
}

std::string CyclesPlugin::statusText()
{
    if (dt_rt* rt = fRt.load()) {
        const char* s = dt_rt_status(rt);
        if (std::strcmp(s, "running")) return s;
    }
    if (fPhase.load() == kBooting) {
        char b[96];
        std::snprintf(b, sizeof b, "First start: booting the firmware %d%% (once only)", bootPercent());
        return b;
    }
    std::lock_guard<std::mutex> g(fLock);
    return fStatus;
}

Plugin* createPlugin()
{
    return new CyclesPlugin();
}

END_NAMESPACE_DISTRHO
