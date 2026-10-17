// DriveModel.hpp -- il +DRIVE del Digitone II visto da un computer per il VST3: lo
// stato e le operazioni del pannello (au/Extension/DrivePanel.swift, emu/drivepanel.py
// DataDrivePanel). Niente sample come nel Digitakt: preset, kit, progetti e soundbank
// con l'API Data del firmware (dn_data_ls / dn_data_read / dn_data_clear via MIDI RPC) e
// CARICA .SYX = i file sul MIDI IN emulato (dn_rt_midi_inject, 256 byte ogni 8 ms).
// Ogni operazione aspetta il firmware: gira in un thread di lavoro, la vista legge lo
// stato con snapshot().
//
// Il +DRIVE e' quello della macchina unica (DigitonePlugin): dopo ogni modifica
// onChange() la fa salvare.
#pragma once

#include "dncore.h"
#include "Platform.hpp"
#include "Strings.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct DriveEntry {
    std::string raw;            // il nome come lo da' il firmware (Latin-1): serve per i percorsi
    std::string name;           // in UTF-8, per la vista
    bool dir = false, valid = false;
    uint32_t count = 0, index = 0, size = 0;
    int type = 0;
};

struct DriveSnapshot {
    std::string path = "/", message;
    bool error = false, busy = false;
    double progress = 0;
    std::vector<DriveEntry> entries;      // nell'ordine del firmware
    uint64_t version = 0;                 // cambia a ogni modifica visibile
};

class DriveModel {
public:
    // engine(): il dt_rt corrente (o nullptr se il motore non e' ancora partito)
    explicit DriveModel(std::function<dt_rt*()> engine)
        : fEngineFn(std::move(engine)), fWorker([this] { loop(); }) {}

    // il plugin si chiude: niente piu' motore, e si aspetta l'operazione in corso
    void close()
    {
        std::unique_lock<std::mutex> g(fMx);
        fClosed = true;
        fJobs.clear();
        fPending = false;
        fIdle.wait(g, [this] { return !fRunning; });
    }

    ~DriveModel()
    {
        {
            std::lock_guard<std::mutex> g(fMx);
            fQuit = true;
        }
        fCv.notify_all();
        fWorker.join();
    }

    DriveSnapshot snapshot() const
    {
        std::lock_guard<std::mutex> g(fMx);
        return fSnap;
    }

    // il motore c'e' ed e' in marcia (i tasti si spengono altrimenti)
    bool available() const
    {
        dt_rt* rt = fEngineFn();
        return rt && !std::strcmp(dt_rt_status(rt), "running");
    }

    void refresh() { post([this] { doRefresh(); }); }

    void enter(const DriveEntry& e)
    {
        {
            std::lock_guard<std::mutex> g(fMx);
            fSnap.path = childPath(fSnap.path, e);
            fPending = true;
        }
        fCv.notify_all();
    }

    void up()
    {
        {
            std::lock_guard<std::mutex> g(fMx);
            if (fSnap.path == "/") return;
            const size_t i = fSnap.path.rfind('/');
            fSnap.path = i == 0 ? "/" : fSnap.path.substr(0, i);
            fPending = true;
        }
        fCv.notify_all();
    }

    // CARICA .SYX: file e cartelle (i .syx dentro, in ordine di nome)
    void sendSyx(std::vector<std::string> paths) { post([this, paths] { doSendSyx(paths); }); }
    // SALVA: DataRead della voce su un file
    void save(DriveEntry e, std::string dest) { post([this, e, dest] { doSave(e, dest); }); }
    // SVUOTA: DataClear delle voci
    void clear(std::vector<DriveEntry> items) { post([this, items] { doClear(items); }); }

    // il +DRIVE e' cambiato (dal thread di lavoro): il plugin salva la macchina
    std::function<void()> onChange;

    // DataDrivePanel._child_path: le cartelle per nome, le voci per indice
    static std::string childPath(const std::string& base, const DriveEntry& e)
    {
        const std::string b = base == "/" ? std::string() : base;
        return b + "/" + (e.dir ? e.raw : std::to_string(e.index));
    }

    static std::string latin1ToUtf8(const char* s)
    {
        std::string o;
        for (; *s; ++s) {
            const unsigned char c = static_cast<unsigned char>(*s);
            if (c < 0x80) o.push_back(char(c));
            else { o.push_back(char(0xC0 | (c >> 6))); o.push_back(char(0x80 | (c & 0x3F))); }
        }
        return o;
    }

    // i .syx (anche dentro le cartelle, in ordine di nome), saltando il resto
    static std::vector<dnplat::fs::path> syxFiles(const std::vector<std::string>& paths)
    {
        namespace fs = dnplat::fs;
        std::vector<fs::path> out;
        auto isSyx = [](const fs::path& p) {
            std::string e = p.extension().string();
            for (auto& c : e) c = char(std::tolower(static_cast<unsigned char>(c)));
            const std::string n = p.filename().string();
            return e == ".syx" && !n.empty() && n[0] != '.';
        };
        for (const auto& s : paths) {
            std::error_code ec;
            const fs::path p(s);
            if (fs::is_directory(p, ec)) {
                std::vector<fs::path> found;
                for (auto it = fs::recursive_directory_iterator(p, ec); !ec && it != fs::recursive_directory_iterator();
                     it.increment(ec))
                    if (it->is_regular_file(ec) && isSyx(it->path())) found.push_back(it->path());
                std::sort(found.begin(), found.end(), [](const fs::path& a, const fs::path& b) {
                    std::string x = a.string(), y = b.string();
                    for (auto& c : x) c = char(std::tolower(static_cast<unsigned char>(c)));
                    for (auto& c : y) c = char(std::tolower(static_cast<unsigned char>(c)));
                    return x < y;
                });
                out.insert(out.end(), found.begin(), found.end());
            } else if (isSyx(p)) {
                out.push_back(p);
            }
        }
        return out;
    }

private:
    using Job = std::function<void()>;

    void post(Job j)
    {
        {
            std::lock_guard<std::mutex> g(fMx);
            if (fClosed) return;
            fJobs.push_back(std::move(j));
        }
        fCv.notify_all();
    }

    void loop()
    {
        for (;;) {
            Job j;
            {
                std::unique_lock<std::mutex> g(fMx);
                fCv.wait(g, [this] { return fQuit || !fJobs.empty() || fPending; });
                if (fQuit) return;
                if (!fJobs.empty()) { j = std::move(fJobs.front()); fJobs.pop_front(); }
                else if (fPending) { fPending = false; j = [this] { doRefresh(); }; }
                fSnap.busy = true;
                fRunning = true;
                ++fSnap.version;
            }
            j();
            {
                std::lock_guard<std::mutex> g(fMx);
                fRunning = false;
                fSnap.busy = false;
                fSnap.progress = 0;
                ++fSnap.version;
            }
            fIdle.notify_all();
        }
    }

    dt_rt* rtOrNull()
    {
        std::lock_guard<std::mutex> g(fMx);
        if (fClosed) return nullptr;
        dt_rt* rt = fEngineFn();
        return rt && !std::strcmp(dt_rt_status(rt), "running") ? rt : nullptr;
    }

    void say(const std::string& m, bool err = false)
    {
        std::lock_guard<std::mutex> g(fMx);
        fSnap.message = m;
        fSnap.error = err;
        ++fSnap.version;
    }

    void progress(double f)
    {
        std::lock_guard<std::mutex> g(fMx);
        fSnap.progress = f;
        ++fSnap.version;
    }

    std::string path() const
    {
        std::lock_guard<std::mutex> g(fMx);
        return fSnap.path;
    }

    void doRefresh()
    {
        dt_rt* rt = rtOrNull();
        if (!rt) { say(dnstr::kDriveNotReady, true); return; }
        const std::string p = path();
        say(dnstr::driveReading(p));
        dn_data_entry* v = nullptr;
        size_t n = 0;
        char err[256] = "";
        if (dn_data_ls(rt, p.c_str(), &v, &n, err, sizeof err)) { say(err[0] ? dnstr::coreMsg(err) : std::string("error"), true); return; }
        std::vector<DriveEntry> es;
        int dirs = 0;
        for (size_t i = 0; i < n; ++i) {
            DriveEntry e;
            e.raw = v[i].name;
            e.name = latin1ToUtf8(v[i].name);
            e.dir = v[i].children != 0 || v[i].type == 1;
            e.type = v[i].type;
            e.valid = v[i].valid != 0;
            e.count = v[i].count;
            e.index = v[i].index;
            e.size = v[i].size;
            dirs += e.dir ? 1 : 0;
            es.push_back(std::move(e));
        }
        std::free(v);
        std::lock_guard<std::mutex> g(fMx);
        fSnap.entries = std::move(es);
        fSnap.message = dnstr::driveCounts(dirs, int(n) - dirs);
        fSnap.error = false;
        ++fSnap.version;
    }

    // emu/drivepanel.py DataDrivePanel.upload_paths
    void doSendSyx(const std::vector<std::string>& paths)
    {
        const auto files = syxFiles(paths);
        if (files.empty()) { say(dnstr::kDriveOnlySyx, true); return; }
        std::vector<std::vector<uint8_t>> data;
        size_t total = 0;
        for (const auto& f : files) {
            std::vector<uint8_t> d;
            if (!dnplat::readFile(f, d)) { say(dnstr::fwFileProblem(f.filename().u8string(), dnstr::kFwUnreadable), true); return; }
            total += d.size();
            data.push_back(std::move(d));
        }
        if (total == 0) total = 1;
        say(dnstr::kDriveSending);
        size_t sent = 0;
        for (size_t k = 0; k < files.size(); ++k) {
            const auto& d = data[k];
            for (size_t o = 0; o < d.size(); o += 256) {
                dt_rt* rt = rtOrNull();
                if (!rt) { say(dnstr::kDriveNotReady, true); return; }
                const size_t n = std::min<size_t>(256, d.size() - o);
                dn_rt_midi_inject(rt, d.data() + o, n);
                progress(double(sent + o + n) / double(total));
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
            }
            sent += d.size();
            say(dnstr::driveSent(files[k].filename().u8string(), d.size()));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        changed();
        doRefresh();
        say(dnstr::driveSentAll(int(files.size())));
    }

    // emu/drivepanel.py DataDrivePanel.save_item: DataRead su file
    void doSave(const DriveEntry& e, const std::string& dest)
    {
        dt_rt* rt = rtOrNull();
        if (!rt) { say(dnstr::kDriveNotReady, true); return; }
        const std::string p = childPath(path(), e);
        say(dnstr::driveReading(p));
        uint8_t* d = nullptr;
        size_t n = 0;
        char err[256] = "";
        if (dn_data_read(rt, p.c_str(), &d, &n, err, sizeof err)) { say(err[0] ? dnstr::coreMsg(err) : std::string("error"), true); return; }
        const bool ok = dnplat::writeFile(dnplat::fs::path(dest), d, n);
        std::free(d);
        if (!ok) { say(dnstr::kDriveCannotWrite, true); return; }
        say(dnstr::driveSaved(n, dest));
    }

    // emu/drivepanel.py DataDrivePanel.delete: DataClear
    void doClear(const std::vector<DriveEntry>& items)
    {
        dt_rt* rt = rtOrNull();
        if (!rt) { say(dnstr::kDriveNotReady, true); return; }
        say(dnstr::kDriveClearing);
        const std::string base = path();
        for (const auto& e : items) {
            const std::string p = childPath(base, e);
            char err[256] = "";
            if (dn_data_clear(rt, p.c_str(), err, sizeof err)) { say(err[0] ? dnstr::coreMsg(err) : std::string("error"), true); return; }
        }
        changed();
        doRefresh();
    }

    void changed()
    {
        if (onChange) onChange();
    }

    std::function<dt_rt*()> fEngineFn;
    mutable std::mutex fMx;
    std::condition_variable fCv, fIdle;
    bool fClosed = false, fRunning = false;
    std::deque<Job> fJobs;
    bool fQuit = false, fPending = false;
    DriveSnapshot fSnap;
    std::thread fWorker;
};
