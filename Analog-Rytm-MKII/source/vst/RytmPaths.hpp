// RytmPaths.hpp -- cartelle e file dell'utente per il VST3 dell'Analog Rytm MKII.
//
// Convenzione comune a tutti i plugin: il plugin NON contiene firmware ne'
// dati derivati dal firmware (stati con la RAM del firmware, codice ricompilato):
// e' tutto dell'utente, sul suo disco.
//
//   Mac      ~/Library/Application Support/Elektron Emu/Analog Rytm MKII/     (dati)
//            ~/Library/Caches/Elektron Emu/Analog Rytm MKII/                   (cache)
//   Windows  %APPDATA%\Elektron Emu\Analog Rytm MKII\                          (dati)
//            %LOCALAPPDATA%\Elektron Emu\Analog Rytm MKII\                     (cache)
//   Linux    ~/.local/share/Elektron Emu/Analog Rytm MKII/, ~/.cache/Elektron Emu/Analog Rytm MKII/
//
//   <dati>/Firmware/*.syx            il firmware dell'utente (il plugin crea la cartella)
//   <dati>/macchina-<sha>-g1.dtsz    la macchina unica, una per firmware (RAM + +Drive + flash)
//   <cache>/primo-avvio-<sha>-g1.dtsz lo stato del primo avvio (dal boot freddo del firmware)
//   <dati>/rytm_panel.png            foto del pannello (facoltativa: se manca, pannello schematico)
//
// Per le prove: AR_USER_DIR sostituisce <dati> e mette la cache in <dati>/Cache;
// AR_FIRMWARE_DIR sostituisce <dati>/Firmware.
#pragma once

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "BrandDir.hpp"

#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#include <shellapi.h>
#else
#include <dirent.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace rytm {

#if defined(_WIN32)
constexpr char kSep = '\\';
#else
constexpr char kSep = '/';
#endif

inline std::string join(const std::string& a, const std::string& b)
{
    if (a.empty()) return b;
    if (a.back() == '/' || a.back() == '\\') return a + b;
    return a + kSep + b;
}

inline bool dirExists(const std::string& p)
{
#if defined(_WIN32)
    const DWORD a = GetFileAttributesA(p.c_str());
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY);
#else
    struct stat st;
    return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

inline bool fileExists(const std::string& p)
{
    if (FILE* f = std::fopen(p.c_str(), "rb")) { std::fclose(f); return true; }
    return false;
}

// crea la cartella e le sue superiori
inline bool makeDirs(const std::string& p)
{
    if (p.empty()) return false;
    if (dirExists(p)) return true;
    size_t i = p.find_last_of("/\\");
    if (i != std::string::npos && i > 0) {
        const std::string parent = p.substr(0, i);
        if (parent.size() > 2 || (parent.size() == 2 && parent[1] != ':')) makeDirs(parent);
    }
#if defined(_WIN32)
    return _mkdir(p.c_str()) == 0 || dirExists(p);
#else
    return ::mkdir(p.c_str(), 0755) == 0 || dirExists(p);
#endif
}

inline std::string env(const char* n)
{
    const char* v = std::getenv(n);
    return v ? std::string(v) : std::string();
}

inline std::string dataDir()
{
    std::string t = env("AR_USER_DIR");
    if (!t.empty()) return t;
#if defined(_WIN32)
    const std::string a = env("APPDATA");
    return a.empty() ? std::string() : emu_brand::migrate(a + "\\Elektron Emu\\Analog Rytm MKII");
#elif defined(__APPLE__)
    const std::string h = env("HOME");
    return h.empty() ? std::string() : emu_brand::migrate(h + "/Library/Application Support/Elektron Emu/Analog Rytm MKII");
#else
    const std::string x = env("XDG_DATA_HOME"), h = env("HOME");
    if (!x.empty()) return emu_brand::migrate(x + "/Elektron Emu/Analog Rytm MKII");
    return h.empty() ? std::string() : emu_brand::migrate(h + "/.local/share/Elektron Emu/Analog Rytm MKII");
#endif
}

inline std::string firmwareDir()
{
    const std::string t = env("AR_FIRMWARE_DIR");
    if (!t.empty()) return t;
    const std::string d = dataDir();
    return d.empty() ? d : join(d, "Firmware");
}

inline std::string cacheDir()
{
    const std::string t = env("AR_USER_DIR");
    if (!t.empty()) return join(t, "Cache");
#if defined(_WIN32)
    const std::string a = env("LOCALAPPDATA");
    return a.empty() ? std::string() : emu_brand::migrate(a + "\\Elektron Emu\\Analog Rytm MKII");
#elif defined(__APPLE__)
    const std::string h = env("HOME");
    return h.empty() ? std::string() : emu_brand::migrate(h + "/Library/Caches/Elektron Emu/Analog Rytm MKII");
#else
    const std::string x = env("XDG_CACHE_HOME"), h = env("HOME");
    if (!x.empty()) return emu_brand::migrate(x + "/Elektron Emu/Analog Rytm MKII");
    return h.empty() ? std::string() : emu_brand::migrate(h + "/.cache/Elektron Emu/Analog Rytm MKII");
#endif
}

inline bool readFile(const std::string& path, std::vector<uint8_t>& out)
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

// scrive in un file temporaneo e lo rinomina: il vecchio resta fino all'ultimo
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

inline std::string lower(std::string s)
{
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// i file *.syx della cartella (nome completo), in ordine alfabetico
inline std::vector<std::string> listSyx(const std::string& dir)
{
    std::vector<std::string> out;
    if (dir.empty()) return out;
#if defined(_WIN32)
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(join(dir, "*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return out;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            const std::string n = fd.cFileName;
            if (n.size() > 4 && lower(n.substr(n.size() - 4)) == ".syx") out.push_back(join(dir, n));
        }
    } while (FindNextFileA(h, &fd));
    FindClose(h);
#else
    if (DIR* d = ::opendir(dir.c_str())) {
        while (dirent* e = ::readdir(d)) {
            const std::string n = e->d_name;
            if (n.size() > 4 && n[0] != '.' && lower(n.substr(n.size() - 4)) == ".syx") out.push_back(join(dir, n));
        }
        ::closedir(d);
    }
#endif
    std::sort(out.begin(), out.end());
    return out;
}

inline std::string baseName(const std::string& p)
{
    const size_t i = p.find_last_of("/\\");
    return i == std::string::npos ? p : p.substr(i + 1);
}

// copia un file (sovrascrive la destinazione)
inline bool copyFile(const std::string& from, const std::string& to)
{
    std::vector<uint8_t> d;
    return readFile(from, d) && writeFile(to, d.data(), d.size());
}

// apre la cartella nel Finder / Explorer / gestore file
inline void openFolder(const std::string& dir)
{
    makeDirs(dir);
#if defined(_WIN32)
    ShellExecuteA(nullptr, "open", dir.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#else
    pid_t pid;
#if defined(__APPLE__)
    const char* tool = "open";
#else
    const char* tool = "xdg-open";
#endif
    char* argv[] = {const_cast<char*>(tool), const_cast<char*>(dir.c_str()), nullptr};
    if (posix_spawnp(&pid, tool, nullptr, nullptr, argv, environ) == 0)
        std::thread([pid] { int st = 0; ::waitpid(pid, &st, 0); }).detach();   // niente zombie
#endif
}

} // namespace rytm
