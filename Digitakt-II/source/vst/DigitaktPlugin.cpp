// DigitaktPlugin.cpp -- vedi DigitaktPlugin.hpp. La logica e' quella dell'AU
// (au/Extension/DigitaktAudioUnit.mm): firmware dalla cartella dell'utente (mai
// nel bundle), primo avvio dal solo firmware (dt_coldboot_session) con cache,
// la macchina unica sul disco, Sync DAW, MIDI in/out, 17 uscite.
#include "DigitaktPlugin.hpp"
#include "BrandDir.hpp"
#include "StarterKit.hpp"
#include "DistrhoPluginUtils.hpp"
#include "extra/Base64.hpp"

extern "C" {
#include "sha256.h"
}
// i MAIN OS che dtcore sa far girare (un profilo di indirizzi ciascuno): solo
// questi firmware si accettano dalla cartella
#include "machines/digitakt2/profiles.inc.c"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>

#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

START_NAMESPACE_DISTRHO

namespace {

bool readFile(const std::string& path, std::vector<uint8_t>& out)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n <= 0) { std::fclose(f); return false; }
    out.resize(static_cast<size_t>(n));
    const bool ok = std::fread(out.data(), 1, out.size(), f) == out.size();
    std::fclose(f);
    return ok;
}

bool writeFile(const std::string& path, const uint8_t* data, size_t n)
{
    const std::string tmp = path + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(data, 1, n, f) == n;
    std::fclose(f);
    if (!ok) { std::remove(tmp.c_str()); return false; }
#if defined(_WIN32)
    return MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
#else
    return std::rename(tmp.c_str(), path.c_str()) == 0;     // atomico: il vecchio file resta fino all'ultimo
#endif
}

bool logOn() { return std::getenv("DTCORE_LOG") != nullptr; }

int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// cartella della macchina unica: %APPDATA%\dtcore, ~/Library/Application Support/dtcore,
// ~/.local/share/dtcore (DT_MACHINE_DIR la sostituisce, per le prove)
std::string machineDir()
{
    std::string d;
    if (const char* e = std::getenv("DT_MACHINE_DIR")) return e;
#if defined(_WIN32)
    if (const char* a = std::getenv("APPDATA")) d = std::string(a) + "\\dtcore";
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) d = std::string(h) + "/Library/Application Support/dtcore";
#else
    if (const char* h = std::getenv("HOME")) d = std::string(h) + "/.local/share/dtcore";
#endif
    return d;
}

void makeDir(const std::string& p)
{
#if defined(_WIN32)
    _mkdir(p.c_str());
#else
    mkdir(p.c_str(), 0755);
#endif
}

// cartella della cache del primo avvio: %LOCALAPPDATA%\dtcore, ~/Library/Caches/dtcore, ~/.cache/dtcore
std::string cacheDir()
{
    std::string d;
#if defined(_WIN32)
    if (const char* a = std::getenv("LOCALAPPDATA")) d = a;
    if (d.empty()) return d;
    d += "\\dtcore";
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) d = std::string(h) + "/Library/Caches";
    if (d.empty()) return d;
    d += "/dtcore";
#else
    if (const char* x = std::getenv("XDG_CACHE_HOME")) d = x;
    else if (const char* h = std::getenv("HOME")) d = std::string(h) + "/.cache";
    if (d.empty()) return d;
    makeDir(d);
    d += "/dtcore";
#endif
    makeDir(d);
    return d;
}

// un .syx e' buono se dtcore lo apre e ha il profilo del suo MAIN OS
bool checkFirmware(const std::vector<uint8_t>& syx, std::string& why)
{
    dt_firmware fw;
    char err[256] = "";
    if (dt_fw_extract(syx.data(), syx.size(), &fw, err, sizeof err) != 0) {
        why = std::string("not a readable Elektron firmware (") + err + ")";
        return false;
    }
    bool ok = false;
    if (const dt_section* s = dt_fw_section(&fw, 3)) {
        dt_sha256 h;
        uint8_t md[32];
        char hex[65];
        dt_sha256_init(&h);
        dt_sha256_update(&h, s->data, s->len);
        dt_sha256_final(&h, md);
        dt_sha256_hex(md, hex);
        for (size_t i = 0; i < DT_NPROFILES; ++i)
            if (!std::strcmp(DT_PROFILES[i].main_sha256, hex)) ok = true;
    }
    dt_fw_free(&fw);
    if (!ok) why = "not the Digitakt II OS 1.16 firmware (the only supported version)";
    return ok;
}

bool hasSyxExt(const std::filesystem::path& p)
{
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return e == ".syx";
}

} // namespace

// ------------------------------------------------------------------ cartella dell'utente
// Convenzione comune ai VST3: ~/Library/Application Support/Elektron Emu/<Nome>,
// %APPDATA%\Elektron Emu\<Nome>, ~/.local/share/Elektron Emu/<Nome> (DT_USER_DIR la sostituisce, per le prove).
std::string DigitaktPlugin::userDir()
{
    if (const char* e = std::getenv("DT_USER_DIR")) { makeDir(e); return e; }
    std::string d;
#if defined(_WIN32)
    if (const char* a = std::getenv("APPDATA")) d = a;
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) d = std::string(h) + "/Library/Application Support";
#else
    if (const char* x = std::getenv("XDG_DATA_HOME")) d = x;
    else if (const char* h = std::getenv("HOME")) { d = std::string(h) + "/.local"; makeDir(d); d += "/share"; }
#endif
    if (d.empty()) return d;
    makeDir(d);
    d += DISTRHO_OS_SEP_STR "Elektron Emu";
    makeDir(d);
    d += DISTRHO_OS_SEP_STR "Digitakt II";
    emu_brand::migrate(d);   // la vecchia .../Elektron Emu/Digitakt II
    makeDir(d);
    return d;
}

std::string DigitaktPlugin::firmwareDir()
{
    std::string d = userDir();
    if (d.empty()) return d;
    d += DISTRHO_OS_SEP_STR "Firmware";
    makeDir(d);
    return d;
}

std::string DigitaktPlugin::firmwareProblem()
{
    std::lock_guard<std::mutex> g(fLock);
    return fFirmwareProblem;
}

// il primo .syx valido della cartella, in ordine di nome. I file scartati non si
// rileggono finche' non cambiano (la cartella si riguarda ogni 2 s mentre manca).
bool DigitaktPlugin::locateFirmware()
{
    const std::string dir = firmwareDir();
    if (dir.empty()) return false;
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (auto it = std::filesystem::directory_iterator(std::filesystem::u8path(dir), ec);
         !ec && it != std::filesystem::directory_iterator(); it.increment(ec))
        if (it->is_regular_file(ec) && hasSyxExt(it->path())) files.push_back(it->path());
    std::sort(files.begin(), files.end());
    std::string problem;
    for (const auto& f : files) {
        const auto size = static_cast<unsigned long long>(std::filesystem::file_size(f, ec));
        const auto when = static_cast<long long>(std::chrono::duration_cast<std::chrono::seconds>(
            std::filesystem::last_write_time(f, ec).time_since_epoch()).count());
        const std::string key = f.u8string() + "|" + std::to_string(size) + "|" + std::to_string(when);
        std::string why = "unreadable";
        if (std::find(fRejected.begin(), fRejected.end(), key) != fRejected.end()) {
            if (problem.empty()) problem = f.filename().u8string() + ": rejected";
            continue;
        }
        std::vector<uint8_t> syx;
        if (readFile(f.u8string(), syx) && checkFirmware(syx, why)) {
            fSyx = std::move(syx);
            std::lock_guard<std::mutex> g(fLock);
            fFirmwareProblem.clear();
            return true;
        }
        fRejected.push_back(key);
        problem = f.filename().u8string() + ": " + why;
    }
    std::lock_guard<std::mutex> g(fLock);
    fFirmwareProblem = problem;
    return false;
}

bool DigitaktPlugin::installFirmware(const std::string& path, std::string& msg)
{
    std::vector<uint8_t> syx;
    if (!readFile(path, syx)) { msg = "cannot read the file"; return false; }
    std::string why;
    if (!checkFirmware(syx, why)) { msg = why; return false; }
    const std::string dir = firmwareDir();
    if (dir.empty()) { msg = "firmware folder not available"; return false; }
    std::filesystem::path name = std::filesystem::u8path(path).filename();
    if (!hasSyxExt(name)) name += ".syx";
    const std::string dest = dir + DISTRHO_OS_SEP_STR + name.u8string();
    std::vector<uint8_t> old;
    if (!(readFile(dest, old) && old == syx) && !writeFile(dest, syx.data(), syx.size())) {
        msg = "cannot write to " + dir;
        return false;
    }
    msg = "firmware copied: " + name.u8string() + ", starting...";
    fRetryStart = true;
    wakeWorker();
    return true;
}

void DigitaktPlugin::wakeWorker()
{
    {
        std::lock_guard<std::mutex> g(fAutosaveMx);
    }
    fAutosaveCv.notify_all();
}

// ------------------------------------------------------------------ HostSync
// Come HostSync dell'AU: gli host non mandano Start/Stop/clock ai plugin, li
// generiamo dal trasporto e li mettiamo nel MIDI IN emulato all'istante giusto.
// Il clock a 24 ppq si ricalcola a ogni blocco dalla posizione in beat.
void DigitaktPlugin::HostSync::run(dt_rt* rt, const TimePosition& t, uint32_t frames, double rate)
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
DigitaktPlugin::DigitaktPlugin()
    : Plugin(kParamCount, 0, 1)
{
    if (isDummyInstance()) return;
    firmwareDir();                                         // la cartella c'e' sempre, anche vuota
    fDrive = std::make_shared<DriveModel>([this] { return fRt.load(); });
    fDrive->onChange = [this] { requestSave(); };          // +Drive cambiato dal pannello: si salva subito
    // ogni secondo: la macchina si e' liberata? Ogni minuto, ma solo a Digitakt fermo
    // (trasporto fermo e niente MIDI da 5 s): la fotografia ferma il motore ~40 ms
    fAutosave = std::thread([this] {
        std::unique_lock<std::mutex> g(fAutosaveMx);
        int since = 0;
        int64_t tick = nowMs(), lastScan = 0;
        while (!fAutosaveStop) {
            fAutosaveCv.wait_for(g, std::chrono::seconds(1),
                                 [this] { return fAutosaveStop || fSaveRequested.load() || fRetryStart.load(); });
            if (fAutosaveStop) break;
            g.unlock();
            if (nowMs() - tick >= 1000) { tick = nowMs(); ++since; }
            if (fNeedFirmware.load() && fWantRate > 0) {
                // manca il firmware: la cartella si riguarda ogni 2 s (o subito dopo "Scegli firmware")
                if (fRetryStart.exchange(false) || nowMs() - lastScan >= 2000) {
                    lastScan = nowMs();
                    startEngine(fWantRate, fWantFrames);
                }
            } else if (!fOwner.load() && fWantRate > 0 && !fSyx.empty() && tryOwn()) {
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

DigitaktPlugin::~DigitaktPlugin()
{
    {
        std::lock_guard<std::mutex> g(fAutosaveMx);
        fAutosaveStop = true;
    }
    fAutosaveCv.notify_all();
    if (fAutosave.joinable()) fAutosave.join();
    fBootCancel = 1;
    if (fBoot.joinable()) fBoot.join();
    if (fDrive) fDrive->close();                 // il suo thread usa ancora il motore
    fDrive.reset();
    saveMachine();                               // si spegne: la macchina resta com'era
    swapEngine(nullptr);
    fMachineLock.release();
}

// ------------------------------------------------------------------ la macchina unica
bool DigitaktPlugin::MachineLock::acquire(const std::string& path)
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

void DigitaktPlugin::MachineLock::release()
{
#if defined(_WIN32)
    if (h) CloseHandle(static_cast<HANDLE>(h));
    h = nullptr;
#else
    if (fd >= 0) ::close(fd);
    fd = -1;
#endif
}

std::string DigitaktPlugin::machinePath() const
{
    const std::string dir = machineDir();
    if (dir.empty() || fSyx.empty()) return std::string();
    dt_sha256 h;
    uint8_t md[32];
    dt_sha256_init(&h);
    dt_sha256_update(&h, fSyx.data(), fSyx.size());
    dt_sha256_final(&h, md);
    char hex[17];
    for (int i = 0; i < 8; ++i) std::snprintf(hex + 2 * i, 3, "%02x", md[i]);
    // una macchina per firmware; "-g2": le macchine nate dal primo avvio puro (fino al
    // 2026-10-06) hanno il +Drive mai montato, si riparte dallo stato iniziale
    return dir + DISTRHO_OS_SEP_STR + "digitakt2-macchina-" + hex + "-g2.dtsz";
}

bool DigitaktPlugin::tryOwn()
{
    if (fOwner.load()) return true;
    if (isDummyInstance()) return false;
    const std::string m = machinePath();
    if (m.empty()) return false;
    makeDir(machineDir());
    fOwner = fMachineLock.acquire(m + ".lock");
    if (logOn()) std::fprintf(stderr, "dtcore: macchina %s (%s)\n", m.c_str(), fOwner ? "presa" : "occupata");
    return fOwner.load();
}

void DigitaktPlugin::requestSave()
{
    if (!fOwner.load()) return;
    {
        std::lock_guard<std::mutex> g(fAutosaveMx);
        fSaveRequested = true;
    }
    fAutosaveCv.notify_all();
}

bool DigitaktPlugin::saveMachine()
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
    if (dt_rt_save(rt, &buf, &len) == 0 && buf) ok = writeFile(m, buf, len);
    std::free(buf);
    if (logOn())
        std::fprintf(stderr, "dtcore: macchina salvata (%zu byte, %.0f ms) %s\n", len,
                     std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(), ok ? "ok" : "ERRORE");
    return ok;
}

void DigitaktPlugin::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    if (input) return;
    const uint32_t bus = index / 2;
    port.groupId = bus;
    port.name = String(bus == 0 ? "Mix" : "Track ") + (bus == 0 ? String() : String(bus))
                + (index % 2 ? " R" : " L");
    port.symbol = String(bus == 0 ? "mix" : "trk") + (bus == 0 ? String() : String(bus))
                  + (index % 2 ? "_r" : "_l");
}

void DigitaktPlugin::initPortGroup(uint32_t groupId, PortGroup& group)
{
    group.name = groupId == 0 ? String("Mix") : String("Track ") + String(groupId);
    group.symbol = groupId == 0 ? String("mix") : String("trk") + String(groupId);
}

void DigitaktPlugin::initParameter(uint32_t index, Parameter& p)
{
    if (index == kParamVolume) {
        p.hints = kParameterIsAutomatable;
        p.name = "Main Volume";
        p.symbol = "volume";
        p.ranges.min = 0.0f; p.ranges.max = 0.5f; p.ranges.def = 0.25f;
    } else if (index == kParamSync) {
        p.hints = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsInteger;
        p.name = "Sync DAW";
        p.symbol = "syncdaw";
        p.ranges.min = 0.0f; p.ranges.max = 1.0f; p.ranges.def = 1.0f;
    }
}

void DigitaktPlugin::initState(uint32_t, State& s)
{
    // come l'hardware la macchina e' una sola, sul disco (machinePath): il progetto
    // della DAW non la contiene. La chiave resta per leggere i progetti vecchi: se la
    // macchina non c'e' ancora, il loro stato diventa la macchina.
    s.hints = kStateIsOnlyForDSP | kStateIsBase64Blob;
    s.key = "dtcore_state";
    s.defaultValue = "";
    s.label = "Digitakt state";
}

float DigitaktPlugin::getParameterValue(uint32_t index) const
{
    if (index == kParamVolume) return static_cast<float>(fMaster.load());
    if (index == kParamSync) return fSync.load() ? 1.0f : 0.0f;
    return 0.0f;
}

void DigitaktPlugin::setParameterValue(uint32_t index, float value)
{
    if (index == kParamVolume) {
        const double v = value < 0 ? 0 : (value > 0.5f ? 0.5 : value);
        fMaster = v;
        if (dt_rt* rt = fRt.load()) dt_rt_set_master(rt, v);
    } else if (index == kParamSync) {
        fSync = value >= 0.5f;
    }
}

void DigitaktPlugin::setMasterFromUI(double v)
{
    fMaster = v < 0 ? 0 : (v > 0.5 ? 0.5 : v);
    if (dt_rt* rt = fRt.load()) dt_rt_set_master(rt, fMaster.load());
}

String DigitaktPlugin::getState(const char* key) const
{
    // la DAW salva il progetto (o chiede lo stato per altro, anche spesso): la
    // macchina si salva nel thread di salvataggio, mai qui (secondi su Windows)
    if (!std::strcmp(key, "dtcore_state")) const_cast<DigitaktPlugin*>(this)->requestSave();
    return String();
}

void DigitaktPlugin::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "dtcore_state") || !value || !*value) return;
    // progetto vecchio con lo stato dentro: conta solo se la macchina unica non c'e' ancora
    {
        const std::string m = machinePath();
        FILE* f = m.empty() ? nullptr : std::fopen(m.c_str(), "rb");
        if (f) { std::fclose(f); return; }
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

std::string DigitaktPlugin::coldBootCachePath() const
{
    const std::string dir = cacheDir();
    if (dir.empty() || fSyx.empty()) return std::string();
    dt_sha256 h;
    uint8_t md[32];
    dt_sha256_init(&h);
    dt_sha256_update(&h, fSyx.data(), fSyx.size());
    dt_sha256_final(&h, md);
    char hex[17];
    for (int i = 0; i < 8; ++i) std::snprintf(hex + 2 * i, 3, "%02x", md[i]);
    char name[160];
    std::snprintf(name, sizeof name, "primo-avvio-%s-%llu-%lld-f%d-kit%d-%s.dtsz", hex,
                  (unsigned long long)DT_COLDBOOT_RUNG, (long long)DT_COLDBOOT_GUI, DT_COLDBOOT_PLUSDRIVE,
                  dtkit::kVersion, DTCORE_VERSION);
    return dir + DISTRHO_OS_SEP_STR + name;
}

void DigitaktPlugin::swapEngine(dt_rt* rt)
{
    if (rt) dt_rt_set_seq_reader(rt, seqmem::read);   // the pattern for the Elektron Emu view
    dt_rt* old = fRt.exchange(rt);
    // run() ha caricato il vecchio puntatore prima dello scambio: si aspetta che finisca
    while (fInRun.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (old) dt_rt_free(old);
}

bool DigitaktPlugin::startEngine(double rate, uint32_t frames)
{
    std::lock_guard<std::recursive_mutex> sg(fStartLock);
    if (fRt.load() && rate == fRate && frames <= fRtFrames) return true;
    std::vector<uint8_t> src;
    if (dt_rt* cur = fRt.load()) {                 // cambio di frequenza: si riparte dallo stato attuale
        uint8_t* buf = nullptr;
        size_t len = 0;
        if (dt_rt_save(cur, &buf, &len) == 0 && buf) src.assign(buf, buf + len);
        std::free(buf);
    }
    fWantRate = rate;
    fWantFrames = frames;
    if (fSyx.empty() && !locateFirmware()) {
        fNeedFirmware = true;
        std::lock_guard<std::mutex> g(fLock);
        fStatus = "Digitakt II firmware (OS 1.16) missing: " + firmwareDir();
        return false;
    }
    fNeedFirmware = false;
    if (!tryOwn()) {                               // un solo Digitakt: questa istanza aspetta
        std::lock_guard<std::mutex> g(fLock);
        fStatus = "Digitakt already open in another instance: close it and this one starts by itself";
        fBusy = true;
        return false;
    }
    fBusy = false;
    bool fromMachine = false;
    if (src.empty()) {                             // la macchina unica, se c'e'
        const std::string m = machinePath();
        fromMachine = !m.empty() && readFile(m, src);
    }
    if (src.empty()) {
        std::lock_guard<std::mutex> g(fLock);
        src = fPendingState;                       // un progetto vecchio, o il primo avvio appena fatto
    }
    std::vector<uint8_t>& pending = src;
    if (pending.empty()) {
        // senza stato: la prima sessione dal firmware, una volta sola, poi dalla cache
        std::vector<uint8_t> boot;
        const std::string cache = coldBootCachePath();
        if (cache.empty() || !readFile(cache, boot)) {
            coldBoot(rate, frames);
            return true;
        }
        pending = std::move(boot);
    }
    char err[256] = "";
    dt_rt* rt = dt_rt_new_mem(fSyx.data(), fSyx.size(), pending.data(), pending.size(), rate, frames, err, sizeof err);
    if (!rt) {
        std::lock_guard<std::mutex> g(fLock);
        fStatus = err;
        fError = true;
        return false;
    }
    dt_rt_set_master(rt, fMaster.load());
    swapEngine(rt);
    fRate = rate;
    fRtFrames = frames;
    setLatency(dt_rt_latency(rt));
    if (!fromMachine) requestSave();             // la prima volta lo stato diventa la macchina
    std::lock_guard<std::mutex> g(fLock);
    fStatus = "running";
    fError = false;
    return true;
}

void DigitaktPlugin::coldBoot(double rate, uint32_t frames)
{
    if (fBooting.exchange(true)) return;
    if (fBoot.joinable()) fBoot.join();
    {
        std::lock_guard<std::mutex> g(fLock);
        fStatus = "first start: booting the firmware (about 15-60 s, only once)";
        fBootStartMs = nowMs();
    }
    setLatency(dt_rt_latency_for(rate, frames));
    fBoot = std::thread([this, rate, frames] {
        dt_firmware fwd;
        char err[256] = "";
        uint8_t* buf = nullptr;
        size_t len = 0;
        int r = -1;
        if (dt_fw_extract(fSyx.data(), fSyx.size(), &fwd, err, sizeof err) == 0) {
            // +Drive montato dal solo .syx: la eMMC del Digitakt II e FORMAT +DRIVE dal
            // menu del firmware (docs/FINDINGS.md i212); niente stato iniziale nel bundle
            r = dt_coldboot_session_ex(&fwd, DT_COLDBOOT_RUNG, DT_COLDBOOT_GUI, 1, DT_COLDBOOT_PLUSDRIVE, &buf, &len,
                                       reinterpret_cast<volatile int*>(&fBootCancel), err, sizeof err);
            dt_fw_free(&fwd);
        }
        std::vector<uint8_t> st;
        if (r == 0 && buf && !fBootCancel.load()) {
            // il +Drive appena formattato non ha i sample di fabbrica: le tracce del progetto
            // di fabbrica sarebbero mute. Si caricano 16 suoni nostri (StarterKit.hpp).
            {
                std::lock_guard<std::mutex> g(fLock);
                fStatus = "first start: loading the starter sounds";
            }
            const std::vector<uint8_t> boot(buf, buf + len);
            std::string kerr;
            if (dtkit::apply(fSyx.data(), fSyx.size(), boot, st, &fBootCancel, kerr) != 0) {
                std::snprintf(err, sizeof err, "starter sounds: %s", kerr.c_str());
                st.clear();
                r = -1;
            }
        }
        if (r == 0 && !st.empty() && !fBootCancel.load()) {
            const std::string cache = coldBootCachePath();
            if (!cache.empty()) writeFile(cache, st.data(), st.size());
            {
                std::lock_guard<std::mutex> g(fLock);
                if (fPendingState.empty()) fPendingState = st;
            }
            std::free(buf);
            startEngine(rate, frames);
            fBooting = false;                    // dopo: coldBoot() puo' fare join senza attese
            return;
        }
        std::free(buf);
        std::lock_guard<std::mutex> g(fLock);
        fStatus = std::string("cold boot: ") + err;
        fError = true;
        fBooting = false;
    });
}

void DigitaktPlugin::ioChanged(uint16_t, uint16_t numOutputs)
{
    fNumOutputs = numOutputs < DISTRHO_PLUGIN_NUM_OUTPUTS ? numOutputs : DISTRHO_PLUGIN_NUM_OUTPUTS;
}

void DigitaktPlugin::activate()
{
    const uint32_t frames = getBufferSize();
    fMaxFrames = frames;
    fBuf.assign(static_cast<size_t>(kBuses) * frames, 0.0f);
    for (int i = 0; i < kBuses; ++i) fMono[i] = fBuf.data() + static_cast<size_t>(i) * frames;
    fHostSync = HostSync();
    if (!fBooting.load()) startEngine(getSampleRate(), frames);
}

void DigitaktPlugin::run(const float**, float** outputs, uint32_t frames, const MidiEvent* midi, uint32_t midiCount)
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
        dt_rt_render(rt, frames, fMono, kBuses, isRenderingOffline() ? 1 : 0);
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
            std::memcpy(outputs[i], fMono[i / 2], frames * sizeof(float));
        return;
    }
    fInRun.store(0);
    for (uint32_t i = 0; i < fNumOutputs; ++i) std::memset(outputs[i], 0, frames * sizeof(float));
}

// ------------------------------------------------------------------ per la vista
bool DigitaktPlugin::seqData(SeqData& d)
{
    dt_rt* rt = fRt.load();
    if (!rt) return false;
    uint8_t buf[DT_SEQ_MAX];
    uint64_t v = 0;
    if (dt_rt_seq(rt, buf, sizeof buf, &v) != sizeof d) return false;
    std::memcpy(&d, buf, sizeof d);
    return d.valid != 0;
}

uint64_t DigitaktPlugin::copyScreen(uint8_t screen[128 * 64], uint8_t leds[50])
{
    dt_rt* rt = fRt.load();
    return rt ? dt_rt_screen(rt, screen, leds) : 0;
}

void DigitaktPlugin::panel(int kind, int code, int arg)
{
    if (dt_rt* rt = fRt.load()) dt_rt_panel(rt, kind, code, arg);
}

EngineStatus DigitaktPlugin::status()
{
    EngineStatus s;
    if (dt_rt* rt = fRt.load()) {
        const char* t = dt_rt_status(rt);
        s.kind = std::strcmp(t, "running") ? EngineStatus::Error : EngineStatus::Running;
        s.text = t;
        return s;
    }
    s.firmwareDir = firmwareDir();
    std::lock_guard<std::mutex> g(fLock);
    s.warning = fFirmwareProblem;
    s.text = fStatus;
    if (fNeedFirmware.load()) s.kind = EngineStatus::NoFirmware;
    else if (fBooting.load()) { s.kind = EngineStatus::Booting; s.bootSeconds = (nowMs() - fBootStartMs.load()) / 1000.0; }
    else if (fBusy.load()) s.kind = EngineStatus::Busy;
    else if (fError.load()) s.kind = EngineStatus::Error;
    else s.kind = EngineStatus::Starting;
    return s;
}

Plugin* createPlugin()
{
    return new DigitaktPlugin();
}

END_NAMESPACE_DISTRHO
