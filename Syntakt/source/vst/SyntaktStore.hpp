// SyntaktStore.hpp -- dove il plugin tiene le sue cose sul disco dell'utente, e
// come trova e controlla il firmware. Header solo, usato dal plugin, dalla
// vista e dagli strumenti di prova.
//
// REGOLA (legale, Elektron): il firmware e tutto cio' che ne deriva (stati con la
// RAM del firmware, il primo avvio in cache, le macchine) stanno SOLO qui, nelle
// cartelle dell'utente. Mai dentro il bundle del plugin.
//
//   dati       Mac  ~/Library/Application Support/Elektron Emu/Syntakt/
//              Win  %APPDATA%\Elektron Emu\Syntakt\
//   firmware   <dati>/Firmware/            (il plugin la crea se manca)
//   macchina   <dati>/syntakt-macchina-<sha>-g1.stsz   (+ .lock)
//   cache      Mac  ~/Library/Caches/Elektron Emu/Syntakt/
//              Win  %LOCALAPPDATA%\Elektron Emu\Syntakt\
//
// ST_DATA_DIR sposta tutto sotto una cartella sola (dati, Firmware, Cache): per le
// prove.
#pragma once

#include "dtcore.h"
extern "C" {
#include "sha256.h"
}

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "BrandDir.hpp"

#if defined(_WIN32)
#include <windows.h>
#include <shlobj.h>
#endif

namespace stplug {

namespace fs = std::filesystem;

inline std::string utf8(const fs::path& p)
{
    return p.u8string();
}

inline fs::path fromUtf8(const std::string& s)
{
    return fs::u8path(s);
}

#if defined(_WIN32)
inline fs::path knownFolder(const KNOWNFOLDERID& id, const char* envName)
{
    PWSTR w = nullptr;
    fs::path p;
    if (SUCCEEDED(SHGetKnownFolderPath(id, KF_FLAG_CREATE, nullptr, &w)) && w) p = fs::path(w);
    if (w) CoTaskMemFree(w);
    if (p.empty())
        if (const char* e = std::getenv(envName)) p = fromUtf8(e);
    return p;
}
#endif

// la cartella dei dati: macchina e Firmware/
inline fs::path dataDir()
{
    if (const char* e = std::getenv("ST_DATA_DIR")) return fromUtf8(e);
    fs::path base;
#if defined(_WIN32)
    base = knownFolder(FOLDERID_RoamingAppData, "APPDATA");
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) base = fs::path(h) / "Library" / "Application Support";
#else
    if (const char* x = std::getenv("XDG_DATA_HOME")) base = x;
    else if (const char* h = std::getenv("HOME")) base = fs::path(h) / ".local" / "share";
#endif
    if (base.empty()) return fs::path();
    const fs::path d = base / emu_brand::brand() / "Syntakt";
    emu_brand::migrate(d.string());   // la vecchia .../Elektron Emu/Syntakt
    return d;
}

inline fs::path firmwareDir()
{
    const fs::path d = dataDir();
    return d.empty() ? d : d / "Firmware";
}

// la cache del primo avvio e le preferenze della vista
inline fs::path cacheDir()
{
    if (const char* e = std::getenv("ST_DATA_DIR")) return fromUtf8(e) / "Cache";
    fs::path base;
#if defined(_WIN32)
    base = knownFolder(FOLDERID_LocalAppData, "LOCALAPPDATA");
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) base = fs::path(h) / "Library" / "Caches";
#else
    if (const char* x = std::getenv("XDG_CACHE_HOME")) base = x;
    else if (const char* h = std::getenv("HOME")) base = fs::path(h) / ".cache";
#endif
    if (base.empty()) return base;
    const fs::path d = base / emu_brand::brand() / "Syntakt";
    emu_brand::migrate(d.string());
    return d;
}

inline bool makeDirs(const fs::path& p)
{
    if (p.empty()) return false;
    std::error_code ec;
    fs::create_directories(p, ec);
    return fs::is_directory(p, ec);
}

inline bool readFile(const fs::path& path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return false;
    const std::streamoff n = f.tellg();
    if (n <= 0) return false;
    out.resize(static_cast<size_t>(n));
    f.seekg(0);
    f.read(reinterpret_cast<char*>(out.data()), n);
    return static_cast<bool>(f);
}

// scrive in un .tmp e lo rinomina: il vecchio file resta finche' il nuovo non e' intero
inline bool writeFile(const fs::path& path, const uint8_t* data, size_t n)
{
    fs::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(n));
        if (!f) { f.close(); std::error_code ec; fs::remove(tmp, ec); return false; }
    }
#if defined(_WIN32)
    if (MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0) return true;
    std::error_code ec;
    fs::remove(tmp, ec);
    return false;
#else
    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) { fs::remove(tmp, ec); return false; }
    return true;
#endif
}

inline std::string shaPrefix(const uint8_t* data, size_t n)
{
    dt_sha256 h;
    uint8_t md[32];
    dt_sha256_init(&h);
    dt_sha256_update(&h, data, n);
    dt_sha256_final(&h, md);
    char hex[17];
    for (int i = 0; i < 8; ++i) std::snprintf(hex + 2 * i, 3, "%02x", md[i]);
    return hex;
}

// ------------------------------------------------------------------ firmware
// I firmware che il plugin conosce (MAIN OS, SHA-256 della sezione 3 decompressa:
// le stesse impronte di core/src/st_profiles.inc.c). Un firmware valido ma non in
// elenco si prova lo stesso: il motore dira' se gli mancano i suoi indirizzi.
struct KnownFirmware { const char* mainSha256; const char* label; };
static const KnownFirmware kKnownFirmware[] = {
    {"8cf2b4b4ea80d58953da46ca453328454fb3fca5cb5f844a2f7bad4b72241f53", "Syntakt OS 1.41"},
};

struct FirmwareInfo {
    enum Kind { None, Invalid, Unknown, Known } kind = None;
    fs::path path;                  // il file scelto
    std::string label;              // "Syntakt OS 1.41", o l'impronta se sconosciuto
    std::string note;               // avviso o motivo dell'errore, per la vista
    std::vector<uint8_t> data;      // il .syx intero (solo se kind >= Unknown)
    std::string signature;          // elenco+data dei .syx della cartella: per non rileggere inutilmente
};

inline std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return s;
}

// formato e versione di un .syx: ELE3 leggibile, MAIN OS (sez. 3) e DSP (sez. 7) presenti
inline FirmwareInfo checkFirmware(const fs::path& path, std::vector<uint8_t>&& bytes)
{
    FirmwareInfo fi;
    fi.path = path;
    char err[256] = "";
    dt_firmware fw;
    if (dt_fw_extract(bytes.data(), bytes.size(), &fw, err, sizeof err) != 0) {
        fi.kind = FirmwareInfo::Invalid;
        fi.note = utf8(path.filename()) + ": not a valid Elektron firmware (" + err + ")";
        return fi;
    }
    const dt_section* main = dt_fw_section(&fw, 3);
    const dt_section* dsp = dt_fw_section(&fw, 7);
    if (!main || !dsp) {
        dt_fw_free(&fw);
        fi.kind = FirmwareInfo::Invalid;
        fi.note = utf8(path.filename()) + ": OS or DSP section missing (is this a Syntakt firmware?)";
        return fi;
    }
    dt_sha256 h;
    uint8_t md[32];
    dt_sha256_init(&h);
    dt_sha256_update(&h, main->data, main->len);
    dt_sha256_final(&h, md);
    char hex[65];
    dt_sha256_hex(md, hex);
    dt_fw_free(&fw);
    fi.kind = FirmwareInfo::Unknown;
    fi.label = std::string("firmware ") + std::string(hex, 12);
    for (const auto& k : kKnownFirmware)
        if (!std::strcmp(k.mainSha256, hex)) { fi.kind = FirmwareInfo::Known; fi.label = k.label; }
    if (fi.kind == FirmwareInfo::Unknown)
        fi.note = utf8(path.filename()) + ": unknown version (" + fi.label + "), trying anyway";
    fi.data = std::move(bytes);
    return fi;
}

// l'elenco dei .syx della cartella con data e misura (cambia se ne arriva uno)
inline std::string dirSignature(const fs::path& dir)
{
    std::string sig;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec) || lower(utf8(it->path().extension())) != ".syx") continue;
        sig += utf8(it->path().filename()) + ":" + std::to_string((unsigned long long)it->file_size(ec)) + ":" +
               std::to_string((long long)it->last_write_time(ec).time_since_epoch().count()) + ";";
    }
    return sig;
}

// il migliore .syx della cartella: prima i noti, poi gli sconosciuti, a parita' il piu' recente
inline FirmwareInfo findFirmware(const fs::path& dir)
{
    FirmwareInfo best;
    best.signature = dirSignature(dir);
    std::error_code ec;
    std::vector<fs::path> files;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec))
        if (it->is_regular_file(ec) && lower(utf8(it->path().extension())) == ".syx") files.push_back(it->path());
    std::sort(files.begin(), files.end(), [&](const fs::path& a, const fs::path& b) {
        std::error_code e1, e2;
        return fs::last_write_time(a, e1) > fs::last_write_time(b, e2);
    });
    std::string firstInvalid;
    for (const auto& f : files) {
        std::vector<uint8_t> bytes;
        std::error_code e1;
        const auto sz = fs::file_size(f, e1);
        if (e1 || sz < 1024 || sz > (64u << 20) || !readFile(f, bytes)) continue;
        FirmwareInfo fi = checkFirmware(f, std::move(bytes));
        if (fi.kind == FirmwareInfo::Invalid) { if (firstInvalid.empty()) firstInvalid = fi.note; continue; }
        if (fi.kind > best.kind) {
            const std::string sig = best.signature;
            best = std::move(fi);
            best.signature = sig;
        }
        if (best.kind == FirmwareInfo::Known) break;
    }
    if (best.kind == FirmwareInfo::None && !firstInvalid.empty()) {
        best.kind = FirmwareInfo::Invalid;
        best.note = firstInvalid;
    }
    return best;
}

} // namespace stplug
