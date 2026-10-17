// DriveModel.hpp -- il +Drive visto da un computer per il VST3: lo stato e le
// operazioni del pannello (au/Extension/DrivePanel.swift, emu/drivepanel.py).
// Ogni operazione e' una RPC al firmware emulato (arcore drive.c) e aspetta la
// risposta: gira in un thread di lavoro, la vista legge lo stato con snapshot().
//
// Il +Drive e' quello della macchina unica (RytmPlugin: RAM e +Drive in un
// file sul disco, come l'hardware): dopo ogni modifica onChange() la fa salvare.
#pragma once

#include "arcore.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <filesystem>
#include <map>
#include <set>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>
#include <vector>

struct DriveEntry {
    std::string name;
    bool dir = false, readOnly = false;
    uint32_t size = 0;
};

struct DriveSnapshot {
    std::string path = "/", message, destFolder = "/incoming";
    bool error = false, busy = false, readOnly = false;
    double progress = 0;
    std::vector<DriveEntry> entries;      // cartelle prima, poi file, in ordine alfabetico
    uint64_t version = 0;                 // cambia a ogni modifica visibile
};

class DriveModel {
public:
    // engine(): il dt_rt corrente (o nullptr se il motore non e' ancora partito)
    explicit DriveModel(std::function<dt_rt*()> engine)
        : fEngineFn(std::move(engine)), fWorker([this] { loop(); }) {}

    // il plugin si chiude: niente piu' motore, e si aspetta l'operazione in corso
    // (una finestra di scelta file aperta puo' tenere vivo il modello dopo il plugin)
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
        DriveSnapshot s = fSnap;
        s.readOnly = isReadOnly(fSnap.path);
        s.destFolder = destFolder(fSnap.path);
        return s;
    }

    void refresh() { post([this] { doRefresh(); }); }

    void enter(const std::string& name)
    {
        {
            std::lock_guard<std::mutex> g(fMx);
            fSnap.path = child(fSnap.path, name);
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

    void goRoot()
    {
        {
            std::lock_guard<std::mutex> g(fMx);
            fSnap.path = "/";
            fPending = true;
        }
        fCv.notify_all();
    }

    void upload(std::vector<std::string> files)
    {
        post([this, files] { doUpload(files); });
    }

    void makeDir(std::string name)
    {
        post([this, name] { doMakeDir(name); });
    }

    void remove(std::vector<std::pair<std::string, bool>> items)       // nome, e' una cartella
    {
        post([this, items] { doRemove(items); });
    }

    // il +Drive e' cambiato (dal thread di lavoro): il plugin salva la macchina
    std::function<void()> onChange;

    static bool isReadOnly(const std::string& p) { return p == "/factory" || p.compare(0, 9, "/factory/") == 0; }
    static std::string destFolder(const std::string& p) { return p != "/" && !isReadOnly(p) ? p : "/incoming"; }
    static std::string child(const std::string& p, const std::string& n) { return p != "/" ? p + "/" + n : "/" + n; }

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

    dt_rt* fEngine()
    {
        std::lock_guard<std::mutex> g(fMx);
        return fClosed ? nullptr : fEngineFn();
    }

    void say(const std::string& m, bool err = false)
    {
        std::lock_guard<std::mutex> g(fMx);
        fSnap.message = m;
        fSnap.error = err;
        ++fSnap.version;
    }

    std::string path() const
    {
        std::lock_guard<std::mutex> g(fMx);
        return fSnap.path;
    }

    static bool list(dt_rt* rt, const std::string& p, std::vector<DriveEntry>& out, std::string& err)
    {
        dt_drive_entry* v = nullptr;
        size_t n = 0;
        char e[256] = "";
        if (dt_drive_ls(rt, p.c_str(), &v, &n, e, sizeof e)) { err = e[0] ? e : "error"; return false; }
        out.clear();
        for (size_t i = 0; i < n; ++i) {
            DriveEntry d;
            d.name = v[i].name;
            d.dir = v[i].type == 'D';
            d.readOnly = v[i].protected_ != 0;
            d.size = v[i].size;
            out.push_back(d);
        }
        std::free(v);
        auto lower = [](std::string s) { for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c))); return s; };
        std::stable_sort(out.begin(), out.end(), [&](const DriveEntry& a, const DriveEntry& b) {
            if (a.dir != b.dir) return a.dir;
            return lower(a.name) < lower(b.name);
        });
        return true;
    }

    void doRefresh()
    {
        dt_rt* rt = fEngine();
        if (!rt) { say("the emulator is not ready yet", true); return; }
        const std::string p = path();
        say("reading " + p + " ...");
        std::vector<DriveEntry> es;
        std::string err;
        if (!list(rt, p, es, err)) { say(err, true); return; }
        int dirs = 0, files = 0;
        for (auto& e : es) (e.dir ? dirs : files)++;
        std::lock_guard<std::mutex> g(fMx);
        fSnap.entries = std::move(es);
        fSnap.message = std::to_string(dirs) + " folders, " + std::to_string(files) + " files";
        fSnap.error = false;
        ++fSnap.version;
    }

    static std::string baseName(const std::string& f)
    {
        const size_t i = f.find_last_of("/\\");
        std::string n = i == std::string::npos ? f : f.substr(i + 1);
        const size_t d = n.rfind('.');
        if (d != std::string::npos) n.resize(d);
        if (n.size() > 32) n.resize(32);
        return n;
    }

    static bool isWav(const std::string& f)
    {
        const size_t d = f.rfind('.');
        if (d == std::string::npos) return false;
        std::string e = f.substr(d + 1);
        for (auto& c : e) c = char(std::tolower(static_cast<unsigned char>(c)));
        return e == "wav";
    }

    // emu/drivepanel.py plan_upload (DrivePanel.swift planUpload): file e cartelle
    // locali -> (WAV, cartella sul drive), cartelle da creare, file saltati
    struct Plan {
        std::vector<std::pair<std::string, std::string>> files;
        std::vector<std::string> dirs;
        int skipped = 0;
    };

    static std::string leaf(const std::string& p)
    {
        std::string s = p;
        while (s.size() > 1 && (s.back() == '/' || s.back() == '\\')) s.pop_back();
        const size_t i = s.find_last_of("/\\");
        s = i == std::string::npos ? s : s.substr(i + 1);
        if (s.size() > 32) s.resize(32);
        return s;
    }

    static void walk(const std::filesystem::path& local, const std::string& remote, Plan& plan)
    {
        std::vector<std::filesystem::path> items;
        std::error_code ec;
        for (auto it = std::filesystem::directory_iterator(local, ec); !ec && it != std::filesystem::directory_iterator();
             it.increment(ec))
            items.push_back(it->path());
        std::sort(items.begin(), items.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
            std::string x = a.filename().string(), y = b.filename().string();
            for (auto& c : x) c = char(std::tolower(static_cast<unsigned char>(c)));
            for (auto& c : y) c = char(std::tolower(static_cast<unsigned char>(c)));
            return x < y;
        });
        for (const auto& full : items) {
            const std::string name = full.filename().string();
            if (name.empty() || name[0] == '.') continue;
            if (std::filesystem::is_directory(full, ec)) {
                const std::string sub = remote + "/" + leaf(name);
                plan.dirs.push_back(sub);
                walk(full, sub, plan);
            } else if (isWav(name)) {
                plan.files.push_back({full.string(), remote});
            } else {
                ++plan.skipped;
            }
        }
    }

    static Plan planUpload(const std::vector<std::string>& paths, const std::string& folder)
    {
        Plan plan;
        std::error_code ec;
        for (const auto& p : paths) {
            if (std::filesystem::is_directory(p, ec)) {
                const std::string remote = (folder == "/" ? std::string() : folder) + "/" + leaf(p);
                plan.dirs.push_back(remote);
                walk(p, remote, plan);
            } else if (isWav(p)) {
                plan.files.push_back({p, folder});
            } else {
                ++plan.skipped;
            }
        }
        return plan;
    }

    void doUpload(const std::vector<std::string>& paths)
    {
        dt_rt* rt = fEngine();
        if (!rt) { say("the emulator is not ready yet", true); return; }
        const std::string folder = destFolder(path());
        const Plan plan = planUpload(paths, folder);
        if (plan.files.empty() && plan.dirs.empty()) {
            say("nothing to load: WAV files only (" + std::to_string(plan.skipped) + " skipped)", true);
            return;
        }
        // le cartelle che mancano, dalla piu' esterna
        std::map<std::string, std::set<std::string>> existing;
        for (const auto& d : plan.dirs) {
            const size_t i = d.rfind('/');
            const std::string parent = i == 0 ? "/" : d.substr(0, i), name = d.substr(i + 1);
            if (!existing.count(parent)) {
                std::vector<DriveEntry> es;
                std::string err;
                if (!list(rt, parent, es, err)) { say(err, true); return; }
                for (auto& e : es) existing[parent].insert(e.name);
            }
            if (!existing[parent].count(name)) {
                say("creating " + d + " ...");
                char err[256] = "";
                if (dt_drive_mkdir(rt, d.c_str(), err, sizeof err)) { say(err[0] ? err : "error", true); return; }
                existing[parent].insert(name);
            }
        }
        // avanzamento pesato sui byte
        std::vector<double> sizes;
        double total = 0;
        for (const auto& f : plan.files) {
            std::error_code ec;
            const double n = double(std::filesystem::file_size(f.first, ec));
            sizes.push_back(ec ? 0 : n);
            total += ec ? 0 : n;
        }
        if (total <= 0) total = 1;
        double sent = 0;
        for (size_t k = 0; k < plan.files.size(); ++k) {
            const std::string& local = plan.files[k].first;
            const std::string& dest = plan.files[k].second;
            say(std::to_string(k + 1) + "/" + std::to_string(plan.files.size()) + "  " + baseName(local) + "  ->  " + dest);
            std::vector<uint8_t> wav;
            if (FILE* f = std::fopen(local.c_str(), "rb")) {
                std::fseek(f, 0, SEEK_END);
                const long n = std::ftell(f);
                std::fseek(f, 0, SEEK_SET);
                if (n > 0) { wav.resize(size_t(n)); if (std::fread(wav.data(), 1, wav.size(), f) != wav.size()) wav.clear(); }
                std::fclose(f);
            }
            if (wav.empty()) { say("unreadable file: " + local, true); return; }
            uint8_t* blob = nullptr;
            size_t len = 0;
            char err[256] = "";
            if (dt_wav2rytm(wav.data(), wav.size(), &blob, &len, err, sizeof err)) {
                say(baseName(local) + ": " + (err[0] ? err : "invalid WAV"), true);
                return;
            }
            struct Ctx { DriveModel* m; double base, size, total; } ctx{this, sent, sizes[k], total};
            const std::string target = dest + "/" + baseName(local);
            const int r = dt_drive_put(rt, target.c_str(), blob, len,
                [](void* c, size_t off, size_t tot) {
                    auto* x = static_cast<Ctx*>(c);
                    std::lock_guard<std::mutex> g(x->m->fMx);
                    x->m->fSnap.progress = (x->base + x->size * (tot ? double(off) / double(tot) : 1.0)) / x->total;
                    ++x->m->fSnap.version;
                }, &ctx, err, sizeof err);
            std::free(blob);
            if (r) { say(err[0] ? err : "error", true); return; }
            sent += sizes[k];
        }
        {
            std::lock_guard<std::mutex> g(fMx);
            fSnap.path = folder;
        }
        changed();
        doRefresh();
        say("loaded " + std::to_string(plan.files.size()) + " samples into " + folder +
            (plan.dirs.empty() ? "" : " (" + std::to_string(plan.dirs.size()) + " folders)") +
            (plan.skipped ? "  (" + std::to_string(plan.skipped) + " non-WAV skipped)" : ""));
    }

    void doMakeDir(const std::string& name)
    {
        dt_rt* rt = fEngine();
        if (!rt) { say("the emulator is not ready yet", true); return; }
        std::string n = name.substr(0, 32);
        const std::string p = child(path(), n);
        char err[256] = "";
        if (dt_drive_mkdir(rt, p.c_str(), err, sizeof err)) { say(err[0] ? err : "error", true); return; }
        changed();
        doRefresh();
    }

    void doRemove(const std::vector<std::pair<std::string, bool>>& items)
    {
        dt_rt* rt = fEngine();
        if (!rt) { say("the emulator is not ready yet", true); return; }
        const std::string base = path();
        for (auto& it : items) {
            const std::string p = child(base, it.first);
            char err[256] = "";
            const int r = it.second ? dt_drive_rmdir(rt, p.c_str(), err, sizeof err) : dt_drive_rm(rt, p.c_str(), err, sizeof err);
            if (r) { say(err[0] ? err : "error", true); return; }
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
