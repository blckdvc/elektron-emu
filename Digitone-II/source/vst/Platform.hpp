// Platform.hpp -- file e cartelle dell'utente per il VST3 (Mac e Windows), senza
// DPF: lo usano il plugin, la vista e gli strumenti di prova (vst/tools).
//
//   firmware        Mac ~/Library/Application Support/Elektron Emu/Digitone II/Firmware
//                   Windows %APPDATA%\Elektron Emu\Digitone II\Firmware
//   macchina unica  .../Elektron Emu/Digitone II/Machine
//   DSP nativo      .../Elektron Emu/Digitone II/Native (la libreria costruita dal plugin, dsp-<hash>-n1.*;
//                   dsp-native.dylib = quella personale di vst/scripts/dsp-nativo.sh, solo Mac)
//   compilatore     Windows %LOCALAPPDATA%\Elektron Emu\toolchain (llvm-mingw scaricato al primo avvio)
//   cache           Mac ~/Library/Caches/Elektron Emu/Digitone II
//                   Windows %LOCALAPPDATA%\Elektron Emu\Digitone II\Cache
//
// I percorsi sono std::filesystem::path: su Windows le API "strette" (fopen, dncore)
// usano la tabella ANSI come getenv, quindi path(string) e .string() restano coerenti
// anche con nomi utente non ASCII; per mostrarli si usa .u8string().
// Variabili per le prove: DN_USER_DIR, DN_CACHE_DIR, DN_MACHINE_DIR.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "BrandDir.hpp"

namespace dnplat {

namespace fs = std::filesystem;

inline std::string env(const char* name)
{
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

inline void makeDirs(const fs::path& p)
{
    std::error_code ec;
    if (!p.empty()) fs::create_directories(p, ec);
}

inline fs::path userDir()
{
    fs::path d;
    const std::string o = env("DN_USER_DIR");
    if (!o.empty()) d = o;
#if defined(_WIN32)
    else if (!env("APPDATA").empty()) d = fs::path(env("APPDATA")) / emu_brand::brand() / "Digitone II";
#elif defined(__APPLE__)
    else if (!env("HOME").empty()) d = fs::path(env("HOME")) / "Library" / "Application Support" / emu_brand::brand() / "Digitone II";
#else
    else if (!env("XDG_DATA_HOME").empty()) d = fs::path(env("XDG_DATA_HOME")) / emu_brand::brand() / "Digitone II";
    else if (!env("HOME").empty()) d = fs::path(env("HOME")) / ".local" / "share" / emu_brand::brand() / "Digitone II";
#endif
    if (o.empty()) emu_brand::migrate(d.string());   // la vecchia .../Elektron Emu/Digitone II
    makeDirs(d);
    return d;
}

inline fs::path firmwareDir()
{
    fs::path d = userDir();
    if (d.empty()) return d;
    d /= "Firmware";
    makeDirs(d);
    return d;
}

inline fs::path machineDir()
{
    const std::string o = env("DN_MACHINE_DIR");
    fs::path d = !o.empty() ? fs::path(o) : userDir();
    if (d.empty()) return d;
    if (o.empty()) d /= "Machine";
    makeDirs(d);
    return d;
}

inline fs::path nativeDir()
{
    fs::path d = userDir();
    if (!d.empty()) d /= "Native";
    return d;
}

inline fs::path toolchainDir()
{
    fs::path d;
    const std::string o = env("DN_TOOLCHAIN_DIR");
    if (!o.empty()) d = o;
#if defined(_WIN32)
    else if (!env("LOCALAPPDATA").empty()) d = fs::path(env("LOCALAPPDATA")) / emu_brand::brand() / "toolchain";
#else
    else if (!env("HOME").empty()) d = fs::path(env("HOME")) / ".cache" / emu_brand::brand() / "toolchain";
#endif
    return d;
}

inline fs::path cacheDir()
{
    fs::path d;
    const std::string o = env("DN_CACHE_DIR");
    if (!o.empty()) d = o;
#if defined(_WIN32)
    else if (!env("LOCALAPPDATA").empty()) d = fs::path(env("LOCALAPPDATA")) / emu_brand::brand() / "Digitone II" / "Cache";
#elif defined(__APPLE__)
    else if (!env("HOME").empty()) d = fs::path(env("HOME")) / "Library" / "Caches" / emu_brand::brand() / "Digitone II";
#else
    else if (!env("XDG_CACHE_HOME").empty()) d = fs::path(env("XDG_CACHE_HOME")) / emu_brand::brand() / "Digitone II";
    else if (!env("HOME").empty()) d = fs::path(env("HOME")) / ".cache" / emu_brand::brand() / "Digitone II";
#endif
    if (o.empty()) emu_brand::migrate(d.string());   // la vecchia .../Elektron Emu/Digitone II
    makeDirs(d);
    return d;
}

inline bool readFile(const fs::path& path, std::vector<uint8_t>& out)
{
    std::FILE* f = std::fopen(path.string().c_str(), "rb");
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

// scrittura atomica: il vecchio file resta finche' il nuovo non e' completo
inline bool writeFile(const fs::path& path, const uint8_t* data, size_t n)
{
    const fs::path tmp = path.string() + ".tmp";
    std::FILE* f = std::fopen(tmp.string().c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(data, 1, n, f) == n;
    std::fclose(f);
    std::error_code ec;
    if (!ok) { fs::remove(tmp, ec); return false; }
    fs::rename(tmp, path, ec);          // std::filesystem sostituisce anche su Windows
    if (ec) { fs::remove(tmp, ec); return false; }
    return true;
}

inline bool fileExists(const fs::path& p)
{
    std::error_code ec;
    return fs::is_regular_file(p, ec);
}

} // namespace dnplat
