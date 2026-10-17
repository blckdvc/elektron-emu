// BrandDir.hpp -- le cartelle dell'utente stanno sotto "Elektron Emu" (fino al 2026-10-17
// "Simo"): stessa copia in tutti e 5 i VST3. migrate(p), con p = .../Elektron Emu/<Nome>[/...],
// sposta .../Simo/<Nome> in .../Elektron Emu/<Nome> se la vecchia c'e' e la nuova manca o e'
// vuota (solo cartelle vuote e README*, come le crea l'installer): rename nella stessa cartella
// madre, se non riesce copia. Restituisce p.
#pragma once

#include <filesystem>
#include <string>
#include <system_error>

namespace emu_brand {

inline const char* brand() { return "Elektron Emu"; }

// vera se in d non c'e' nessun file salvo i README* (cartelle vuote fatte dagli installer)
inline bool onlySkeleton(const std::filesystem::path& d)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    for (fs::recursive_directory_iterator it(d, ec), end; !ec && it != end; it.increment(ec)) {
        if (it->is_directory(ec)) continue;
        if (it->path().filename().string().rfind("README", 0) == 0) continue;
        return false;
    }
    return !ec;
}

inline std::string migrate(const std::string& p)
{
    namespace fs = std::filesystem;
    const std::string b = brand();
    const size_t i = p.rfind(b);
    if (i == std::string::npos || i == 0 || (p[i - 1] != '/' && p[i - 1] != '\\')) return p;
    const size_t j = i + b.size();
    if (j >= p.size() || (p[j] != '/' && p[j] != '\\')) return p;
    size_t k = p.find_first_of("/\\", j + 1);
    if (k == std::string::npos) k = p.size();
    const std::string newRoot = p.substr(0, k);
    const std::string oldParent = p.substr(0, i) + "Simo";
    const std::string oldRoot = oldParent + p.substr(j, k - j);
    std::error_code ec;
    if (!fs::is_directory(fs::path(oldRoot), ec)) return p;
    if (fs::exists(fs::path(newRoot), ec)) {
        if (!onlySkeleton(fs::path(newRoot))) return p;
        fs::remove_all(fs::path(newRoot), ec);
        if (ec) return p;
    }
    fs::create_directories(fs::path(p.substr(0, j)), ec);
    ec.clear();
    fs::rename(fs::path(oldRoot), fs::path(newRoot), ec);
    if (ec) {
        ec.clear();
        fs::copy(fs::path(oldRoot), fs::path(newRoot), fs::copy_options::recursive, ec);
    }
    fs::remove(fs::path(oldParent), ec);   // solo se e' rimasta vuota
    return p;
}

} // namespace emu_brand
