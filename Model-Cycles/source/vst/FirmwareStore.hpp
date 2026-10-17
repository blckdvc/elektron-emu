// FirmwareStore.hpp -- dove il plugin cerca il firmware e dove tiene i suoi file.
//
// Il firmware Elektron NON e' nel plugin (ne' nel repo, ne' nei progetti della
// DAW): lo mette l'utente nella cartella
//   Mac      ~/Library/Application Support/Elektron Emu/Model Cycles/Firmware/
//   Windows  %APPDATA%\Elektron Emu\Model Cycles\Firmware\
// (convenzione comune a tutti i plugin della serie). Il plugin la crea se manca,
// ci cerca un .syx valido, e con quello fa partire il motore. Anche tutto cio'
// che deriva dal firmware (lo stato del primo avvio, la macchina unica) sta fuori
// dal bundle, nelle cartelle dell'utente:
//   macchina   <...>/Elektron Emu/Model Cycles/macchina-<sha>-g1.mcsz   (RAM e +Drive)
//   cache      ~/Library/Caches/Elektron Emu/Model Cycles/              (il primo avvio)
//              %LOCALAPPDATA%\Elektron Emu\Model Cycles\Cache\
// MC_DATA_DIR / MC_CACHE_DIR le sostituiscono (per le prove).
#pragma once

#include "mccore.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "BrandDir.hpp"

#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace mcfw {

#if defined(_WIN32)
static const char kSep = '\\';
#else
static const char kSep = '/';
#endif

inline bool makeDir(const std::string& p)
{
#if defined(_WIN32)
    return _mkdir(p.c_str()) == 0;
#else
    return mkdir(p.c_str(), 0755) == 0;
#endif
}

// crea il percorso, un livello alla volta
inline void makeDirs(const std::string& path)
{
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i] == '/' || path[i] == '\\') {
            const std::string sub = path.substr(0, i);
            if (sub.size() > 2 || (sub.size() == 2 && sub[1] != ':')) makeDir(sub);
        }
    }
    makeDir(path);
}

inline bool fileExists(const std::string& p)
{
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) return false;
    std::fclose(f);
    return true;
}

inline bool readFile(const std::string& path, std::vector<uint8_t>& out, size_t maxBytes = 0)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n <= 0 || (maxBytes && static_cast<size_t>(n) > maxBytes)) { std::fclose(f); return false; }
    out.resize(static_cast<size_t>(n));
    const bool ok = std::fread(out.data(), 1, out.size(), f) == out.size();
    std::fclose(f);
    return ok;
}

// scrive in un file temporaneo e lo rinomina: il vecchio file resta fino all'ultimo
inline bool writeFile(const std::string& path, const uint8_t* data, size_t n)
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
    return std::rename(tmp.c_str(), path.c_str()) == 0;
#endif
}

// ----------------------------------------------------------------- cartelle
// ~/Library/Application Support/Elektron Emu/Model Cycles, %APPDATA%\Elektron Emu\Model Cycles
inline std::string dataDir()
{
    std::string d;
    if (const char* e = std::getenv("MC_DATA_DIR")) { makeDirs(e); return e; }
#if defined(_WIN32)
    if (const char* a = std::getenv("APPDATA")) d = std::string(a) + "\\Elektron Emu\\Model Cycles";
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) d = std::string(h) + "/Library/Application Support/Elektron Emu/Model Cycles";
#else
    if (const char* x = std::getenv("XDG_DATA_HOME")) d = std::string(x) + "/Elektron Emu/Model Cycles";
    else if (const char* h = std::getenv("HOME")) d = std::string(h) + "/.local/share/Elektron Emu/Model Cycles";
#endif
    if (!d.empty()) { emu_brand::migrate(d); makeDirs(d); }   // la vecchia .../Elektron Emu/...
    return d;
}

// la cartella del firmware: la crea se manca
inline std::string firmwareDir()
{
    const std::string d = dataDir();
    if (d.empty()) return d;
    const std::string f = d + kSep + "Firmware";
    makeDirs(f);
    return f;
}

// ~/Library/Caches/Elektron Emu/Model Cycles, %LOCALAPPDATA%\Elektron Emu\Model Cycles\Cache
inline std::string cacheDir()
{
    std::string d;
    if (const char* e = std::getenv("MC_CACHE_DIR")) { makeDirs(e); return e; }
#if defined(_WIN32)
    if (const char* a = std::getenv("LOCALAPPDATA")) d = std::string(a) + "\\Elektron Emu\\Model Cycles\\Cache";
#elif defined(__APPLE__)
    if (const char* h = std::getenv("HOME")) d = std::string(h) + "/Library/Caches/Elektron Emu/Model Cycles";
#else
    if (const char* x = std::getenv("XDG_CACHE_HOME")) d = std::string(x) + "/Elektron Emu/Model Cycles";
    else if (const char* h = std::getenv("HOME")) d = std::string(h) + "/.cache/Elektron Emu/Model Cycles";
#endif
    if (!d.empty()) { emu_brand::migrate(d); makeDirs(d); }   // la vecchia .../Elektron Emu/...
    return d;
}

// ----------------------------------------------------------------- firmware
struct Firmware {
    std::string path;                 // il .syx scelto (vuoto: nessuno)
    std::vector<uint8_t> data;
    bool known = false;               // la versione che il motore conosce
    std::string note;                 // avviso per la vista ("versione sconosciuta ...")
    std::string error;                // perche' nessun file e' valido
};

inline std::string lower(std::string s)
{
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// i .syx della cartella (data di modifica, percorso), il piu' recente per primo
inline std::vector<std::pair<long long, std::string>> listSyx(const std::string& dir)
{
    std::vector<std::pair<long long, std::string>> v;
#if defined(_WIN32)
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA((dir + "\\*").c_str(), &fd);
    if (h != INVALID_HANDLE_VALUE) {
        do {
            if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
            const std::string n = fd.cFileName;
            if (lower(n).size() > 4 && lower(n).substr(n.size() - 4) == ".syx") {
                const long long t = (static_cast<long long>(fd.ftLastWriteTime.dwHighDateTime) << 32) | fd.ftLastWriteTime.dwLowDateTime;
                v.push_back({t, dir + "\\" + n});
            }
        } while (FindNextFileA(h, &fd));
        FindClose(h);
    }
#else
    if (DIR* d = opendir(dir.c_str())) {
        while (const dirent* e = readdir(d)) {
            const std::string n = e->d_name;
            if (n.size() > 4 && n[0] != '.' && lower(n).substr(n.size() - 4) == ".syx") {
                struct stat st;
                const std::string p = dir + "/" + n;
                if (stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode))
                    v.push_back({static_cast<long long>(st.st_mtime), p});
            }
        }
        closedir(d);
    }
#endif
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    return v;
}

// cambia quando cambia il contenuto della cartella: la vista non rilegge i file per niente
inline std::string signature()
{
    std::string s;
    for (const auto& f : listSyx(firmwareDir())) s += f.second + ":" + std::to_string(f.first) + ";";
    return s;
}

// controlla formato (container ELE3 con il MAIN OS) e versione. -> 0 non e' un
// firmware, 1 firmware di una versione che il motore non conosce (si prova
// comunque e si avvisa), 2 versione nota
inline int check(const std::vector<uint8_t>& syx, std::string& why)
{
    dt_firmware fw;
    char err[256] = "";
    if (dt_fw_extract(syx.data(), syx.size(), &fw, err, sizeof err) != 0) {
        why = err[0] ? err : "unrecognised format";
        return 0;
    }
    const int known = dt_fw_profile_known(&fw);
    dt_fw_free(&fw);
    return known ? 2 : 1;
}

// il firmware da usare: fra i .syx validi quello di versione nota (il piu'
// recente), altrimenti il piu' recente di versione sconosciuta
inline Firmware find()
{
    Firmware r;
    const std::string dir = firmwareDir();
    if (dir.empty()) { r.error = "firmware folder not reachable"; return r; }
    const auto files = listSyx(dir);
    if (files.empty()) { r.error = "no .syx file in the folder"; return r; }
    Firmware unknown;
    std::string why, first;
    for (const auto& entry : files) {
        const std::string& f = entry.second;
        std::vector<uint8_t> d;
        if (!readFile(f, d, 64u << 20)) continue;
        std::string w;
        const int c = check(d, w);
        if (c == 2) {
            r.path = f; r.data = std::move(d); r.known = true;
            return r;
        }
        if (c == 1 && unknown.path.empty()) {
            unknown.path = f; unknown.data = std::move(d);
            unknown.note = "Unknown firmware version: trying anyway";
        } else if (c == 0 && first.empty()) {
            first = f.substr(f.find_last_of("/\\") + 1) + ": " + w;
        }
    }
    if (!unknown.path.empty()) return unknown;
    r.error = first.empty() ? "no readable .syx file in the folder" : "not a valid firmware (" + first + ")";
    return r;
}

// copia il .syx scelto nella cartella del firmware. -> "" se riuscito, altrimenti il perche'
inline std::string install(const std::string& src)
{
    std::vector<uint8_t> d;
    if (!readFile(src, d, 64u << 20)) return "cannot read the file";
    std::string why;
    if (check(d, why) == 0) return "not a valid firmware (" + why + ")";
    const std::string dir = firmwareDir();
    if (dir.empty()) return "firmware folder not reachable";
    std::string name = src.substr(src.find_last_of("/\\") + 1);
    if (lower(name).size() < 5 || lower(name).substr(name.size() - 4) != ".syx") name += ".syx";
    if (!writeFile(dir + kSep + name, d.data(), d.size())) return "could not copy the file into the firmware folder";
    return std::string();
}

// SHA-256 corto del firmware, per i nomi dei file di stato
inline std::string shortSha(const std::vector<uint8_t>& syx)
{
    dt_firmware fw;
    char err[64];
    std::string out = "0000000000000000";
    if (dt_fw_extract(syx.data(), syx.size(), &fw, err, sizeof err) == 0) {
        static const char* hx = "0123456789abcdef";
        for (int i = 0; i < 8; ++i) {
            out[2 * i] = hx[fw.syx_sha256[i] >> 4];
            out[2 * i + 1] = hx[fw.syx_sha256[i] & 15];
        }
        dt_fw_free(&fw);
    }
    return out;
}

} // namespace mcfw
