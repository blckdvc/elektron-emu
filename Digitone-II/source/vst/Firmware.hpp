// Firmware.hpp -- il firmware dell'utente: controllo di un .syx (dn_fw_check),
// ricerca nella cartella e copia da "Scegli firmware...". Senza DPF, cosi' si prova
// da solo (vst/tools/fwtest.cpp). Il firmware e' di Elektron: sta solo nella cartella
// dell'utente, mai nel plugin.
#pragma once

#include "Platform.hpp"
#include "Strings.hpp"
#include "dncore.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>

namespace dnfw {

namespace fs = std::filesystem;

struct Info {
    bool known = false;          // MAIN OS con profilo di indirizzi (provato)
    std::string machine, version, sha256;
    std::string warning;         // versione sconosciuta: si prova comunque
};

// true = e' un firmware del Digitone II che si puo' provare; false = `why` dice perche' no
inline bool check(const uint8_t* data, size_t len, Info& info, std::string& why)
{
    dn_fw_info fi;
    char err[256] = "";
    if (dn_fw_check(data, len, &fi, err, sizeof err) != 0) {
        why = dnstr::fwNotReadable(err[0] ? err : "?");
        return false;
    }
    info.known = fi.known != 0;
    info.machine = fi.machine;
    info.version = fi.version;
    info.sha256 = fi.sha256;
    if (!info.machine.empty() && info.machine != "Digitone II") {
        why = dnstr::fwWrongMachine(info.machine);
        return false;
    }
    if (info.machine.empty() && !info.known) {
        why = dnstr::kFwUnknownMachine;
        return false;
    }
    if (!info.known) info.warning = dnstr::fwUnknownVersion(info.version);
    return true;
}

inline bool hasSyxExt(const fs::path& p)
{
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return e == ".syx";
}

// il primo .syx valido della cartella, in ordine di nome. I file scartati non si
// rileggono finche' non cambiano (la cartella si riguarda ogni 2 s mentre manca).
struct Finder {
    std::vector<std::string> rejected;

    bool find(const fs::path& dir, std::vector<uint8_t>& syx, Info& info, std::string& problem)
    {
        std::vector<fs::path> files;
        std::error_code ec;
        for (auto it = fs::directory_iterator(dir, ec); !ec && it != fs::directory_iterator(); it.increment(ec))
            if (it->is_regular_file(ec) && hasSyxExt(it->path())) files.push_back(it->path());
        std::sort(files.begin(), files.end());
        problem.clear();
        for (const auto& f : files) {
            const auto size = static_cast<unsigned long long>(fs::file_size(f, ec));
            const auto when = static_cast<long long>(std::chrono::duration_cast<std::chrono::seconds>(
                fs::last_write_time(f, ec).time_since_epoch()).count());
            const std::string key = f.u8string() + "|" + std::to_string(size) + "|" + std::to_string(when);
            std::string why = dnstr::kFwUnreadable;
            if (std::find(rejected.begin(), rejected.end(), key) != rejected.end()) {
                if (problem.empty()) problem = dnstr::fwFileProblem(f.filename().u8string(), lastWhy[key]);
                continue;
            }
            std::vector<uint8_t> data;
            Info fi;
            if (dnplat::readFile(f, data) && check(data.data(), data.size(), fi, why)) {
                syx = std::move(data);
                info = fi;
                problem.clear();
                return true;
            }
            rejected.push_back(key);
            lastWhy[key] = why;
            if (problem.empty()) problem = dnstr::fwFileProblem(f.filename().u8string(), why);
        }
        return false;
    }

private:
    std::map<std::string, std::string> lastWhy;
};

// "Choose firmware...": controlla il file e lo copia nella cartella del firmware
inline bool install(const fs::path& src, const fs::path& dir, std::string& msg)
{
    std::vector<uint8_t> syx;
    if (!dnplat::readFile(src, syx)) { msg = dnstr::kFwUnreadable; return false; }
    Info info;
    std::string why;
    if (!check(syx.data(), syx.size(), info, why)) { msg = why; return false; }
    if (dir.empty()) { msg = dnstr::kFwNoFolder; return false; }
    dnplat::makeDirs(dir);
    fs::path name = src.filename();
    if (!hasSyxExt(name)) name += ".syx";
    const fs::path dest = dir / name;
    std::vector<uint8_t> old;
    if (!(dnplat::readFile(dest, old) && old == syx) && !dnplat::writeFile(dest, syx.data(), syx.size())) {
        msg = dnstr::fwCannotWrite(dir.u8string());
        return false;
    }
    msg = dnstr::fwCopied(name.u8string());
    if (!info.warning.empty()) msg += " (" + info.warning + ")";
    return true;
}

} // namespace dnfw
