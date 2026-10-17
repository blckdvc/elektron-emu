// RytmPlugin.cpp -- vedi RytmPlugin.hpp. La logica e' quella dell'AU
// (au/Extension/RytmAudioUnit.mm) e del VST3 del Digitakt II: stato intero nella
// macchina unica sul disco, Sync DAW, MIDI in/out, Mix + 8 voci separate. Novita'
// rispetto a quelli: il firmware e lo stato iniziale NON stanno nel bundle. Il
// firmware e' un .syx nella cartella dell'utente (RytmPaths.hpp), lo stato
// iniziale nasce al primo avvio da dt_coldboot_session e va in cache.
#include "RytmPlugin.hpp"
#include "RytmPaths.hpp"
#include "DistrhoPluginUtils.hpp"
#include "extra/Base64.hpp"

extern "C" {
#include "sha256.h"
}

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

// generazione dei file di stato: si cambia quando cambia il modo di farli
// (macchina: RAM + +Drive + flash; primo avvio: catena di dt_coldboot_session)
const char* const kGen = "g1";

bool logOn() { return std::getenv("DTCORE_LOG") != nullptr || std::getenv("AR_LOG") != nullptr; }

int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string sha8(const uint8_t* d, size_t n)
{
    dt_sha256 h;
    uint8_t md[32];
    dt_sha256_init(&h);
    dt_sha256_update(&h, d, n);
    dt_sha256_final(&h, md);
    char hex[17];
    for (int i = 0; i < 8; ++i) std::snprintf(hex + 2 * i, 3, "%02x", md[i]);
    return hex;
}

// "1.73" -> 173 per ordinare le versioni
long versionKey(const std::string& v)
{
    long a = 0, b = 0;
    if (std::sscanf(v.c_str(), "%ld.%ld", &a, &b) >= 1) return a * 1000 + b;
    return -1;
}

// nomi e misure dei .syx della cartella: se non cambia, e' inutile rileggerli
std::string dirSignature()
{
    std::string sig;
    for (const std::string& f : rytm::listSyx(rytm::firmwareDir())) {
        std::vector<uint8_t> d;
        long n = -1;
        if (FILE* fp = std::fopen(f.c_str(), "rb")) {
            std::fseek(fp, 0, SEEK_END);
            n = std::ftell(fp);
            std::fclose(fp);
        }
        sig += f + ":" + std::to_string(n) + ";";
    }
    return sig;
}

} // namespace

// ------------------------------------------------------------------ HostSync
// Come HostSync dell'AU: gli host non mandano Start/Stop/clock ai plugin, li
// generiamo dal trasporto e li mettiamo nel MIDI IN emulato all'istante giusto.
// Il clock a 24 ppq si ricalcola a ogni blocco dalla posizione in beat.
void RytmPlugin::HostSync::run(dt_rt* rt, const TimePosition& t, uint32_t frames, double rate)
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
RytmPlugin::RytmPlugin()
    : Plugin(kParamCount, 0, 1)
{
    if (isDummyInstance()) return;
    fView.firmwareDir = rytm::firmwareDir();
    fView.text = "engine not started yet";
    fDrive = std::make_shared<DriveModel>([this] { return fRt.load(); });
    fDrive->onChange = [this] { requestSave(); };          // +Drive cambiato dal pannello: si salva subito
    // thread di servizio, ogni secondo: il firmware e' arrivato? la macchina si e' liberata?
    // Ogni minuto salva, ma solo a Rytm fermo (trasporto fermo e niente MIDI da 5 s)
    fService = std::thread([this] {
        std::unique_lock<std::mutex> g(fServiceMx);
        int since = 0, tick2 = 0;
        int64_t tick = nowMs();
        while (!fServiceStop) {
            fServiceCv.wait_for(g, std::chrono::seconds(1),
                                [this] { return fServiceStop || fSaveRequested.load() || fKick.load(); });
            if (fServiceStop) break;
            g.unlock();
            if (nowMs() - tick >= 1000) { tick = nowMs(); ++since; ++tick2; }
            if (fKick.exchange(false)) {
                if (fWantRate > 0 && !fRt.load()) startEngine(fWantRate, fWantFrames);
            } else if (!fRt.load() && !fBooting.load() && fWantRate > 0 && tick2 >= 2) {
                tick2 = 0;                                  // firmware messo a mano? macchina libera?
                startEngine(fWantRate, fWantFrames);
            } else if (fSaveRequested.exchange(false)) {   // chiesto (getState, +Drive): subito
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

RytmPlugin::~RytmPlugin()
{
    {
        std::lock_guard<std::mutex> g(fServiceMx);
        fServiceStop = true;
    }
    fServiceCv.notify_all();
    if (fService.joinable()) fService.join();
    fBootCancel = 1;
    if (fBoot.joinable()) fBoot.join();
    if (fDrive) fDrive->close();                 // il suo thread usa ancora il motore
    fDrive.reset();
    saveMachine();                               // si spegne: la macchina resta com'era
    swapEngine(nullptr);
    fMachineLock.release();
}

// ------------------------------------------------------------------ la macchina unica
bool RytmPlugin::MachineLock::acquire(const std::string& path)
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

void RytmPlugin::MachineLock::release()
{
#if defined(_WIN32)
    if (h) CloseHandle(static_cast<HANDLE>(h));
    h = nullptr;
#else
    if (fd >= 0) ::close(fd);
    fd = -1;
#endif
}

std::string RytmPlugin::machinePath() const
{
    const std::string dir = rytm::dataDir();
    if (dir.empty() || fFw.sha8.empty()) return std::string();
    // una macchina per firmware
    return rytm::join(dir, "macchina-" + fFw.sha8 + "-" + kGen + ".dtsz");
}

std::string RytmPlugin::coldBootCachePath() const
{
    const std::string dir = rytm::cacheDir();
    if (dir.empty() || fFw.sha8.empty()) return std::string();
    return rytm::join(dir, "primo-avvio-" + fFw.sha8 + "-" + kGen + ".dtsz");
}

bool RytmPlugin::tryOwn()
{
    if (fOwner.load()) return true;
    if (isDummyInstance()) return false;
    const std::string m = machinePath();
    if (m.empty()) return false;
    rytm::makeDirs(rytm::dataDir());
    fOwner = fMachineLock.acquire(m + ".lock");
    if (logOn()) std::fprintf(stderr, "arcore: macchina %s (%s)\n", m.c_str(), fOwner ? "presa" : "occupata");
    return fOwner.load();
}

void RytmPlugin::requestSave()
{
    if (!fOwner.load()) return;
    {
        std::lock_guard<std::mutex> g(fServiceMx);
        fSaveRequested = true;
    }
    fServiceCv.notify_all();
}

bool RytmPlugin::saveMachine()
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
    if (dt_rt_save(rt, &buf, &len) == 0 && buf) ok = rytm::writeFile(m, buf, len);
    std::free(buf);
    if (logOn())
        std::fprintf(stderr, "arcore: macchina salvata (%zu byte, %.0f ms) %s\n", len,
                     std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(), ok ? "ok" : "ERRORE");
    return ok;
}

// ------------------------------------------------------------------ porte e parametri
void RytmPlugin::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    if (input) return;
    static const char* const names[kBuses] = {"Mix", "BD", "BT", "SD", "LT", "CH/OH", "MT/HT", "CY/CB", "RS/CP"};
    static const char* const syms[kBuses] = {"mix", "bd", "bt", "sd", "lt", "ch_oh", "mt_ht", "cy_cb", "rs_cp"};
    const uint32_t bus = index / 2;
    port.groupId = bus;
    port.name = String(names[bus]) + (index % 2 ? " R" : " L");
    port.symbol = String(syms[bus]) + (index % 2 ? "_r" : "_l");
}

void RytmPlugin::initPortGroup(uint32_t groupId, PortGroup& group)
{
    static const char* const names[kBuses] = {"Mix", "BD", "BT", "SD", "LT", "CH/OH", "MT/HT", "CY/CB", "RS/CP"};
    static const char* const syms[kBuses] = {"mix", "bd", "bt", "sd", "lt", "ch_oh", "mt_ht", "cy_cb", "rs_cp"};
    group.name = names[groupId];
    group.symbol = syms[groupId];
}

void RytmPlugin::initParameter(uint32_t index, Parameter& p)
{
    if (index == kParamVolume) {
        // sul Rytm il Main Volume e' analogico, fuori dal firmware (rytmsynth.MASTER della GUI v2)
        p.hints = kParameterIsAutomatable;
        p.name = "Volume";
        p.symbol = "volume";
        p.ranges.min = 0.0f; p.ranges.max = 1.0f; p.ranges.def = 1.0f;
    } else if (index == kParamSync) {
        p.hints = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsInteger;
        p.name = "Sync DAW";
        p.symbol = "syncdaw";
        p.ranges.min = 0.0f; p.ranges.max = 1.0f; p.ranges.def = 1.0f;
    }
}

void RytmPlugin::initState(uint32_t, State& s)
{
    // come l'hardware la macchina e' una sola, sul disco (machinePath): il progetto
    // della DAW non contiene niente del firmware. La chiave serve solo perche' l'host
    // la chieda al salvataggio del progetto: allora la macchina si salva.
    s.hints = kStateIsOnlyForDSP | kStateIsBase64Blob;
    s.key = "arcore_machine";
    s.defaultValue = "";
    s.label = "Rytm machine";
}

float RytmPlugin::getParameterValue(uint32_t index) const
{
    if (index == kParamVolume) return static_cast<float>(fMaster.load());
    if (index == kParamSync) return fSync.load() ? 1.0f : 0.0f;
    return 0.0f;
}

void RytmPlugin::setParameterValue(uint32_t index, float value)
{
    if (index == kParamVolume) {
        const double v = value < 0 ? 0 : (value > 1.0f ? 1.0 : value);
        fMaster = v;
        if (dt_rt* rt = fRt.load()) dt_rt_set_volume(rt, v);
    } else if (index == kParamSync) {
        fSync = value >= 0.5f;
    }
}

void RytmPlugin::setMasterFromUI(double v)
{
    fMaster = v < 0 ? 0 : (v > 1.0 ? 1.0 : v);
    if (dt_rt* rt = fRt.load()) dt_rt_set_volume(rt, fMaster.load());
}

String RytmPlugin::getState(const char* key) const
{
    // la DAW salva il progetto (o chiede lo stato per altro, anche spesso): la
    // macchina si salva nel thread di servizio, mai qui (secondi su Windows)
    if (!std::strcmp(key, "arcore_machine")) const_cast<RytmPlugin*>(this)->requestSave();
    return String();
}

void RytmPlugin::setState(const char*, const char*)
{
    // niente: lo stato e' la macchina sul disco, non il progetto
}

// ------------------------------------------------------------------ vista
void RytmPlugin::setView(RytmViewInfo::Kind k, const std::string& text)
{
    if (logOn()) std::fprintf(stderr, "arcore: stato %d: %s\n", int(k), text.c_str());
    std::lock_guard<std::mutex> g(fLock);
    fView.kind = k;
    fView.text = text;
    ++fView.version;
}

RytmViewInfo RytmPlugin::viewInfo()
{
    RytmViewInfo s;
    {
        std::lock_guard<std::mutex> g(fLock);
        s = fView;
        s.warning = fWarning;
        s.firmwareLabel = fFw.label;
    }
    s.firmwareDir = rytm::firmwareDir();
    if (fBooting.load()) s.bootSeconds = (nowMs() - fBootStartMs.load()) / 1000.0;
    if (dt_rt* rt = fRt.load()) {
        const char* r = dt_rt_status(rt);
        if (std::strcmp(r, "running")) { s.kind = RytmViewInfo::kError; s.text = r; }
        else if (s.kind != RytmViewInfo::kRunning) { s.kind = RytmViewInfo::kRunning; s.text = "running"; }
    }
    return s;
}

std::string RytmPlugin::photoPath() const
{
    const std::string u = rytm::join(rytm::dataDir(), "rytm_panel.png");
    if (!rytm::dataDir().empty() && rytm::fileExists(u)) return u;
    if (const char* bundle = getBundlePath())
        if (const char* res = getResourcePath(bundle)) {
            const std::string p = std::string(res) + DISTRHO_OS_SEP_STR + "rytm_panel.png";
            if (rytm::fileExists(p)) return p;
        }
    return std::string();
}

// ------------------------------------------------------------------ il firmware dell'utente
// Cerca un .syx valido nella cartella Firmware: formato (container ELE3 e MAIN OS)
// e versione (il motore ha un profilo di indirizzi per ogni versione supportata).
// Una versione sconosciuta non si rifiuta: si prova e si avvisa.
bool RytmPlugin::scanFirmware(Firmware& out)
{
    const std::string dir = rytm::firmwareDir();
    rytm::makeDirs(dir);
    Firmware best;
    std::string firstError;
    for (const std::string& f : rytm::listSyx(dir)) {
        std::vector<uint8_t> d;
        Firmware c;
        c.path = f;
        c.name = rytm::baseName(f);
        if (!rytm::readFile(f, d)) { if (firstError.empty()) firstError = c.name + ": unreadable file"; continue; }
        dt_firmware fw;
        char err[256] = "";
        if (dt_fw_extract(d.data(), d.size(), &fw, err, sizeof err) != 0) {
            if (firstError.empty()) firstError = c.name + ": not a valid Elektron firmware (" + err + ")";
            continue;
        }
        char ver[64] = "";
        c.supported = dt_fw_supported(&fw, ver, sizeof ver) != 0;
        c.version = ver;
        c.valid = dt_fw_section(&fw, 3) != nullptr;
        dt_fw_free(&fw);
        if (!c.valid) { if (firstError.empty()) firstError = c.name + ": OS section missing (is this an Analog Rytm firmware?)"; continue; }
        c.sha8 = sha8(d.data(), d.size());
        c.label = !c.version.empty() ? "Analog Rytm MKII OS " + c.version : "firmware " + c.sha8;
        // preferito: supportato, poi la versione piu' alta, poi il primo in ordine di nome
        const bool better = !best.valid || (c.supported && !best.supported) ||
                            (c.supported == best.supported && versionKey(c.version) > versionKey(best.version));
        if (better) best = c;
    }
    if (!best.valid) {
        out = Firmware();
        out.error = firstError;
        return false;
    }
    out = best;
    return true;
}

bool RytmPlugin::installFirmware(const std::string& path, std::string& error)
{
    std::vector<uint8_t> d;
    if (!rytm::readFile(path, d)) { error = "unreadable file: " + path; return false; }
    dt_firmware fw;
    char err[256] = "";
    if (dt_fw_extract(d.data(), d.size(), &fw, err, sizeof err) != 0) {
        error = std::string("not a valid Elektron firmware (") + err + ")";
        return false;
    }
    const bool hasMain = dt_fw_section(&fw, 3) != nullptr;
    dt_fw_free(&fw);
    if (!hasMain) { error = "OS section missing (is this an Analog Rytm firmware?)"; return false; }
    const std::string dir = rytm::firmwareDir();
    if (!rytm::makeDirs(dir)) { error = "cannot create " + dir; return false; }
    std::string name = rytm::baseName(path);
    if (name.size() < 5 || rytm::lower(name.substr(name.size() - 4)) != ".syx") name += ".syx";
    const std::string dest = rytm::join(dir, name);
    if (!rytm::writeFile(dest, d.data(), d.size())) { error = "cannot write " + dest; return false; }
    {
        std::lock_guard<std::mutex> g(fServiceMx);
        fKick = true;
        fRescan = true;
    }
    fServiceCv.notify_all();
    return true;
}

// ------------------------------------------------------------------ il motore
void RytmPlugin::swapEngine(dt_rt* rt)
{
    dt_rt* old = fRt.exchange(rt);
    // run() ha caricato il vecchio puntatore prima dello scambio: si aspetta che finisca
    while (fInRun.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (old) dt_rt_free(old);
}

bool RytmPlugin::startEngine(double rate, uint32_t frames)
{
    std::lock_guard<std::recursive_mutex> sg(fStartLock);
    fWantRate = rate;
    fWantFrames = frames;
    if (fRt.load() && rate == fRate && frames <= fRtFrames) return true;
    if (fBooting.load()) return false;             // il thread del primo avvio richiama startEngine
    std::vector<uint8_t> src;
    if (dt_rt* cur = fRt.load()) {                 // cambio di frequenza: si riparte dallo stato attuale
        uint8_t* buf = nullptr;
        size_t len = 0;
        if (dt_rt_save(cur, &buf, &len) == 0 && buf) src.assign(buf, buf + len);
        std::free(buf);
    }
    // 1. il firmware dell'utente
    const std::string sig = dirSignature();
    if (fRescan.exchange(false)) {                 // l'utente ha scelto un file: da capo
        fSyx.clear(); fFw = Firmware();
        fSigFailed.clear(); fSigBad.clear();
    }
    if (!fSigBad.empty()) {                        // il firmware di prima non va: aspetto un file nuovo
        if (sig == fSigBad) return false;
        fSyx.clear(); fFw = Firmware();
        fSigBad.clear();
    }
    if (fSyx.empty()) {
        if (!fSigFailed.empty() && sig == fSigFailed) return false;   // niente di nuovo nella cartella
        fSigFailed.clear();
        Firmware fw;
        if (!scanFirmware(fw)) {
            fSigFailed = sig;
            {
                std::lock_guard<std::mutex> g(fLock);
                fWarning = fw.error;               // il motivo, se c'era un file ma non valido
                fFw = Firmware();
            }
            setView(RytmViewInfo::kNoFirmware, "firmware missing");
            return false;
        }
        if (!rytm::readFile(fw.path, fSyx)) {
            setView(RytmViewInfo::kError, "firmware unreadable: " + fw.path);
            return false;
        }
        {
            std::lock_guard<std::mutex> g(fLock);
            fFw = fw;
            fWarning = fw.supported ? std::string() : fw.name + ": unknown version (" + fw.label + "), trying anyway";
        }
        if (logOn()) std::fprintf(stderr, "arcore: firmware %s (%s, %s)\n", fw.path.c_str(), fw.version.c_str(),
                                  fw.supported ? "noto" : "versione sconosciuta");
    }
    // 2. un solo Rytm: questa istanza aspetta
    if (!tryOwn()) {
        setView(RytmViewInfo::kBusy, "Analog Rytm is already open in another instance: close that one and this one starts by itself");
        return false;
    }
    // 3. la macchina unica, se c'e'; poi lo stato del primo avvio (cache); poi il boot freddo
    bool fromMachine = false;
    if (src.empty()) {
        const std::string m = machinePath();
        fromMachine = !m.empty() && rytm::readFile(m, src);
    }
    if (src.empty()) {
        std::lock_guard<std::mutex> g(fLock);
        src = fPendingState;
    }
    if (src.empty()) {
        std::vector<uint8_t> boot;
        const std::string cache = coldBootCachePath();
        if (cache.empty() || !rytm::readFile(cache, boot)) {
            coldBoot(rate, frames);
            return true;
        }
        src = std::move(boot);
    }
    setView(RytmViewInfo::kStarting, "starting the engine");
    char err[256] = "";
    dt_rt* rt = dt_rt_new_mem(fSyx.data(), fSyx.size(), src.data(), src.size(), rate, frames, err, sizeof err);
    if (!rt) {
        const std::string e = err;
        if (e.find("no address profile") != std::string::npos || e.find("unsupported") != std::string::npos)
            fSigBad = sig;                           // aspetto un file diverso nella cartella
        setView(RytmViewInfo::kError, "engine: " + e);
        if (logOn()) std::fprintf(stderr, "arcore: motore: %s\n", err);
        return false;
    }
    dt_rt_set_volume(rt, fMaster.load());
    swapEngine(rt);
    fRate = rate;
    fRtFrames = frames;
    setLatency(dt_rt_latency(rt));
    if (!fromMachine) requestSave();               // la prima volta lo stato diventa la macchina
    setView(RytmViewInfo::kRunning, "running");
    return true;
}

// Il primo avvio dal solo firmware: una volta sola, in un thread (da 15 s a qualche
// minuto), poi lo stato va in cache e diventa la macchina.
void RytmPlugin::coldBoot(double rate, uint32_t frames)
{
    if (fBooting.exchange(true)) return;
    if (fBoot.joinable()) fBoot.join();
    fBootStartMs = nowMs();
    { std::lock_guard<std::mutex> g(fLock); fView.phase.clear(); fView.progress = 0; }
    setView(RytmViewInfo::kBooting, "First start: booting the firmware (once only)");
    fBoot = std::thread([this, rate, frames] {
        dt_firmware fwd;
        char err[256] = "";
        uint8_t* buf = nullptr;
        size_t len = 0;
        int r = -1;
        const auto t0 = std::chrono::steady_clock::now();
        if (dt_fw_extract(fSyx.data(), fSyx.size(), &fwd, err, sizeof err) == 0) {
            r = dt_coldboot_session(
                &fwd, 1, &buf, &len, reinterpret_cast<volatile int*>(&fBootCancel),
                [](void* ctx, double frac, const char* what) {
                    auto* self = static_cast<RytmPlugin*>(ctx);
                    std::lock_guard<std::mutex> g(self->fLock);
                    self->fView.phase = what ? what : "";
                    self->fView.progress = frac;
                    ++self->fView.version;
                },
                this, err, sizeof err);
            dt_fw_free(&fwd);
        }
        if (r == 0 && buf && !fBootCancel.load()) {
            if (logOn())
                std::fprintf(stderr, "arcore: primo avvio fatto in %.1f s (%zu byte)\n",
                             std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), len);
            const std::string cache = coldBootCachePath();
            if (!cache.empty()) {
                rytm::makeDirs(rytm::cacheDir());
                rytm::writeFile(cache, buf, len);
            }
            {
                std::lock_guard<std::mutex> g(fLock);
                if (fPendingState.empty()) fPendingState.assign(buf, buf + len);
            }
            std::free(buf);
            fBooting = false;                    // startEngine non parte finche' fBooting e' vero
            startEngine(rate, frames);
            return;
        }
        std::free(buf);
        if (!fBootCancel.load()) {
            setView(RytmViewInfo::kError, std::string("cold boot: ") + err);
            fSigBad = dirSignature();            // non si riprova finche' la cartella non cambia
        }
        fBooting = false;
    });
}

void RytmPlugin::ioChanged(uint16_t, uint16_t numOutputs)
{
    fNumOutputs = numOutputs < DISTRHO_PLUGIN_NUM_OUTPUTS ? numOutputs : DISTRHO_PLUGIN_NUM_OUTPUTS;
}

void RytmPlugin::activate()
{
    const uint32_t frames = getBufferSize();
    fMaxFrames = frames;
    fBuf.assign(static_cast<size_t>(DT_RT_CHANNELS) * frames, 0.0f);
    for (int i = 0; i < DT_RT_CHANNELS; ++i) fChan[i] = fBuf.data() + static_cast<size_t>(i) * frames;
    fHostSync = HostSync();
    if (!fBooting.load()) startEngine(getSampleRate(), frames);
}

void RytmPlugin::run(const float**, float** outputs, uint32_t frames, const MidiEvent* midi, uint32_t midiCount)
{
    fInRun.store(1);
    dt_rt* rt = fRt.load();
    if (rt && frames <= fMaxFrames) {
        if (fSync.load()) fHostSync.run(rt, getTimePosition(), frames, getSampleRate());
        if (midiCount || getTimePosition().playing) fLastActivityMs.store(nowMs());
        for (uint32_t i = 0; i < midiCount; ++i) {
            const MidiEvent& ev = midi[i];
            const uint8_t* d = ev.size > MidiEvent::kDataSize ? ev.dataExt : ev.data;
            if (d && ev.size) dt_rt_midi(rt, ev.frame < frames ? ev.frame : frames - 1, d, ev.size);
        }
        // export della DAW (VST3 V3_OFFLINE): si aspetta il motore invece di perdere campioni
        dt_rt_render(rt, frames, fChan, DT_RT_CHANNELS, isRenderingOffline() ? 1 : 0);
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
        for (uint32_t i = 0; i < fNumOutputs; ++i)
            std::memcpy(outputs[i], fChan[i], frames * sizeof(float));
        return;
    }
    fInRun.store(0);
    for (uint32_t i = 0; i < fNumOutputs; ++i) std::memset(outputs[i], 0, frames * sizeof(float));
}

// ------------------------------------------------------------------ per la vista
uint64_t RytmPlugin::copyScreen(uint8_t screen[128 * 64], uint8_t lit[128], uint8_t pads[12][3])
{
    dt_rt* rt = fRt.load();
    return rt ? dt_rt_screen(rt, screen, lit, pads) : 0;
}

void RytmPlugin::panel(int kind, int code, int arg)
{
    if (dt_rt* rt = fRt.load()) dt_rt_panel(rt, kind, code, arg);
}

Plugin* createPlugin()
{
    return new RytmPlugin();
}

END_NAMESPACE_DISTRHO
