// NativeBuild.cpp -- vedi NativeBuild.hpp.
#include "NativeBuild.hpp"

#include "NativeSrc.hpp"
#include "dncore.h"

#include <zlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winhttp.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace dnnative {

namespace {

// Versione del traduttore e del runtime: nel nome della libreria e nella chiave
const char* const kVersionTag = "n1";

#if defined(_WIN32)
// llvm-mingw: versione fissa, URL ufficiale (GitHub), impronta SHA-256 fissata qui
// (https://github.com/mstorsjo/llvm-mingw/releases/tag/20261006, ucrt, x86_64)
const char* const kTcUrl = "https://github.com/mstorsjo/llvm-mingw/releases/download/20261006/llvm-mingw-20261006-ucrt-x86_64.zip";
const char* const kTcSha256 = "317492c456aa27ee607a5919f1d2d38dcdc1112516a24d0bf4b00d078f52d17a";
const char* const kTcRoot = "llvm-mingw-20261006-ucrt-x86_64";
const uint64_t kTcSize = 190836485ull;
#endif

std::string firstLines(const std::string& s, size_t maxChars = 400)
{
    std::string out;
    int lines = 0;
    size_t i = 0;
    while (i < s.size() && lines < 6 && out.size() < maxChars) {
        size_t e = s.find('\n', i);
        if (e == std::string::npos) e = s.size();
        std::string l = s.substr(i, e - i);
        while (!l.empty() && (l.back() == '\r' || l.back() == ' ')) l.pop_back();
        if (!l.empty()) {
            if (!out.empty()) out += " | ";
            out += l;
            ++lines;
        }
        i = e + 1;
    }
    if (out.size() > maxChars) out.resize(maxChars);
    return out;
}

std::string trim(std::string s)
{
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r')) ++i;
    return s.substr(i);
}

bool cancelled(const Hooks& h)
{
    return h.cancel && *h.cancel;
}

void report(const Hooks& h, int pct, const std::string& text)
{
    if (h.progress) h.progress(pct, text);
}

std::string u8(const fs::path& p)
{
    return p.u8string();
}

fs::path fromU8(const std::string& s)
{
    return fs::u8path(s);
}

// ------------------------------------------------------------------ processi
struct ProcOut {
    bool started = false;
    int code = -1;
    std::string out;
};

#if defined(_WIN32)
std::wstring widen(const std::string& s)
{
    if (s.empty()) return std::wstring();
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), &w[0], n);
    return w;
}

std::wstring quoteArg(const std::wstring& a)
{
    if (!a.empty() && a.find_first_of(L" \t\"") == std::wstring::npos) return a;
    std::wstring q = L"\"";
    size_t bs = 0;
    for (wchar_t c : a) {
        if (c == L'\\') { ++bs; continue; }
        if (c == L'"') { q.append(bs * 2 + 1, L'\\'); q += L'"'; bs = 0; continue; }
        q.append(bs, L'\\');
        bs = 0;
        q += c;
    }
    q.append(bs * 2, L'\\');
    q += L'"';
    return q;
}

ProcOut runProc(const std::vector<std::string>& argv, volatile int* cancel, bool lowPriority)
{
    ProcOut r;
    SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
    HANDLE rd = nullptr, wr = nullptr;
    if (!CreatePipe(&rd, &wr, &sa, 0)) return r;
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);
    HANDLE nul = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, 0, nullptr);
    STARTUPINFOW si{};
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nul;
    si.hStdOutput = wr;
    si.hStdError = wr;
    std::wstring cmd;
    for (size_t i = 0; i < argv.size(); ++i) {
        if (i) cmd += L' ';
        cmd += quoteArg(widen(argv[i]));
    }
    std::vector<wchar_t> buf(cmd.begin(), cmd.end());
    buf.push_back(L'\0');
    PROCESS_INFORMATION pi{};
    const DWORD flags = CREATE_NO_WINDOW | (lowPriority ? BELOW_NORMAL_PRIORITY_CLASS : 0);
    const std::wstring exe = widen(argv[0]);
    const BOOL ok = CreateProcessW(exe.c_str(), buf.data(), nullptr, nullptr, TRUE, flags, nullptr, nullptr, &si, &pi);
    CloseHandle(wr);
    if (nul != INVALID_HANDLE_VALUE) CloseHandle(nul);
    if (!ok) {
        CloseHandle(rd);
        return r;
    }
    r.started = true;
    for (;;) {
        DWORD avail = 0;
        if (PeekNamedPipe(rd, nullptr, 0, nullptr, &avail, nullptr) && avail) {
            char tmp[4096];
            DWORD got = 0;
            if (ReadFile(rd, tmp, sizeof tmp, &got, nullptr) && got) {
                if (r.out.size() < (1u << 20)) r.out.append(tmp, got);
                continue;
            }
        }
        if (WaitForSingleObject(pi.hProcess, 50) == WAIT_OBJECT_0) {
            // quello che resta nel pipe
            DWORD got = 0;
            char tmp[4096];
            while (PeekNamedPipe(rd, nullptr, 0, nullptr, &avail, nullptr) && avail && ReadFile(rd, tmp, sizeof tmp, &got, nullptr) && got)
                if (r.out.size() < (1u << 20)) r.out.append(tmp, got);
            break;
        }
        if (cancel && *cancel) {
            TerminateProcess(pi.hProcess, 1);
            WaitForSingleObject(pi.hProcess, 2000);
            break;
        }
    }
    DWORD code = 1;
    GetExitCodeProcess(pi.hProcess, &code);
    r.code = static_cast<int>(code);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    CloseHandle(rd);
    return r;
}
#else
ProcOut runProc(const std::vector<std::string>& argv, volatile int* cancel, bool lowPriority)
{
    ProcOut r;
    int fds[2];
    if (pipe(fds) != 0) return r;
    fcntl(fds[0], F_SETFD, FD_CLOEXEC);
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, fds[1], 1);
    posix_spawn_file_actions_adddup2(&fa, fds[1], 2);
    posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
    std::vector<char*> args;
    for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
    args.push_back(nullptr);
    pid_t pid = 0;
    const int rc = posix_spawn(&pid, argv[0].c_str(), &fa, nullptr, args.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    close(fds[1]);
    if (rc != 0) {
        close(fds[0]);
        return r;
    }
    r.started = true;
    if (lowPriority) setpriority(PRIO_PROCESS, static_cast<id_t>(pid), 10);
    bool killed = false;
    for (;;) {
        pollfd p{fds[0], POLLIN, 0};
        const int pr = poll(&p, 1, 100);
        if (pr > 0) {
            char tmp[4096];
            const ssize_t n = read(fds[0], tmp, sizeof tmp);
            if (n > 0) {
                if (r.out.size() < (1u << 20)) r.out.append(tmp, static_cast<size_t>(n));
                continue;
            }
            if (n == 0) break;                        // EOF: il processo ha chiuso l'uscita
        }
        if (cancel && *cancel && !killed) {
            kill(pid, SIGKILL);
            killed = true;
        }
    }
    close(fds[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    r.code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
    return r;
}
#endif

// ------------------------------------------------------------------ zip (solo i file che servono)
uint32_t le16(const uint8_t* p) { return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8); }
uint32_t le32(const uint8_t* p) { return le16(p) | (le16(p + 2) << 16); }

struct ZipEntry {
    std::string name;
    uint32_t method = 0, crc = 0, csize = 0, usize = 0, offset = 0, attr = 0;
    bool unixMade = false;
};

bool readAt(std::FILE* f, uint64_t off, void* dst, size_t n)
{
#if defined(_WIN32)
    if (_fseeki64(f, static_cast<__int64>(off), SEEK_SET) != 0) return false;
#else
    if (fseeko(f, static_cast<off_t>(off), SEEK_SET) != 0) return false;
#endif
    return std::fread(dst, 1, n, f) == n;
}

bool zipDirectory(std::FILE* f, uint64_t size, std::vector<ZipEntry>& out, std::string& why)
{
    const size_t tail = static_cast<size_t>(std::min<uint64_t>(size, 1u << 17));
    std::vector<uint8_t> t(tail);
    if (!readAt(f, size - tail, t.data(), tail)) { why = "cannot read the zip file"; return false; }
    size_t e = std::string::npos;
    for (size_t i = tail >= 22 ? tail - 22 : 0; i + 4 <= tail; --i) {
        if (le32(&t[i]) == 0x06054b50u) { e = i; break; }
        if (i == 0) break;
    }
    if (e == std::string::npos) { why = "not a zip file"; return false; }
    const uint32_t count = le16(&t[e + 10]), cdSize = le32(&t[e + 12]), cdOff = le32(&t[e + 16]);
    if (count == 0xFFFF || cdSize == 0xFFFFFFFFu || cdOff == 0xFFFFFFFFu) { why = "zip64 is not supported"; return false; }
    std::vector<uint8_t> cd(cdSize);
    if (!readAt(f, cdOff, cd.data(), cdSize)) { why = "cannot read the zip directory"; return false; }
    size_t p = 0;
    for (uint32_t i = 0; i < count; ++i) {
        if (p + 46 > cd.size() || le32(&cd[p]) != 0x02014b50u) { why = "damaged zip directory"; return false; }
        ZipEntry z;
        z.unixMade = (cd[p + 5] == 3);
        z.method = le16(&cd[p + 10]);
        z.crc = le32(&cd[p + 16]);
        z.csize = le32(&cd[p + 20]);
        z.usize = le32(&cd[p + 24]);
        const uint32_t nl = le16(&cd[p + 28]), xl = le16(&cd[p + 30]), cl = le16(&cd[p + 32]);
        z.attr = le32(&cd[p + 38]);
        z.offset = le32(&cd[p + 42]);
        if (p + 46 + nl > cd.size()) { why = "damaged zip directory"; return false; }
        z.name.assign(reinterpret_cast<const char*>(&cd[p + 46]), nl);
        out.push_back(std::move(z));
        p += 46 + nl + xl + cl;
    }
    return true;
}

bool inflateEntry(std::FILE* f, const ZipEntry& z, const fs::path& dest, std::string& why)
{
    uint8_t lh[30];
    if (!readAt(f, z.offset, lh, 30) || le32(lh) != 0x04034b50u) { why = "damaged zip entry " + z.name; return false; }
    const uint64_t data = static_cast<uint64_t>(z.offset) + 30 + le16(lh + 26) + le16(lh + 28);
    dnplat::makeDirs(dest.parent_path());
    std::FILE* out = std::fopen(dest.string().c_str(), "wb");
    if (!out) {
#if defined(_WIN32)
        out = _wfopen(dest.wstring().c_str(), L"wb");
#endif
        if (!out) { why = "cannot write " + u8(dest); return false; }
    }
    bool ok = true;
    uint32_t crc = crc32(0, nullptr, 0);
    if (z.method == 0) {
        std::vector<uint8_t> b(1 << 16);
        uint64_t left = z.csize, off = data;
        while (left && ok) {
            const size_t n = static_cast<size_t>(std::min<uint64_t>(left, b.size()));
            ok = readAt(f, off, b.data(), n) && std::fwrite(b.data(), 1, n, out) == n;
            crc = crc32(crc, b.data(), static_cast<uInt>(n));
            off += n;
            left -= n;
        }
    } else if (z.method == 8) {
        z_stream s{};
        if (inflateInit2(&s, -15) != Z_OK) { std::fclose(out); why = "zlib"; return false; }
        std::vector<uint8_t> in(1 << 16), ob(1 << 17);
        uint64_t left = z.csize, off = data;
        int rc = Z_OK;
        while (ok && rc != Z_STREAM_END) {
            if (s.avail_in == 0) {
                const size_t n = static_cast<size_t>(std::min<uint64_t>(left, in.size()));
                if (n == 0) { ok = false; break; }
                ok = readAt(f, off, in.data(), n);
                off += n;
                left -= n;
                s.next_in = in.data();
                s.avail_in = static_cast<uInt>(n);
            }
            s.next_out = ob.data();
            s.avail_out = static_cast<uInt>(ob.size());
            rc = inflate(&s, Z_NO_FLUSH);
            if (rc != Z_OK && rc != Z_STREAM_END) { ok = false; break; }
            const size_t got = ob.size() - s.avail_out;
            if (got) {
                crc = crc32(crc, ob.data(), static_cast<uInt>(got));
                if (std::fwrite(ob.data(), 1, got, out) != got) ok = false;
            }
        }
        inflateEnd(&s);
    } else {
        ok = false;
    }
    if (std::fclose(out) != 0) ok = false;
    if (!ok || crc != z.crc) {
        why = "cannot unpack " + z.name;
        std::error_code ec;
        fs::remove(dest, ec);
        return false;
    }
    return true;
}

#if defined(_WIN32)
// i file di llvm-mingw che servono a compilare e collegare codice C per x86-64
bool wantedInToolchain(const std::string& rel)
{
    auto starts = [&](const char* p) { return rel.compare(0, std::strlen(p), p) == 0; };
    if (starts("bin/")) {
        static const char* const names[] = {"clang.exe", "clang-23.exe", "ld.lld.exe", "libLLVM-23.dll", "libclang-cpp.dll",
                                            "libc++.dll", "libunwind.dll", "libwinpthread-1.dll", "x86_64-w64-windows-gnu.cfg",
                                            "mingw32-common.cfg", "x86_64-w64-mingw32-common.cfg"};
        const std::string n = rel.substr(4);
        for (const char* x : names) if (n == x) return true;
        return false;
    }
    if (starts("x86_64-w64-mingw32/lib/")) return true;
    if (starts("lib/clang/") && rel.find("/include/") != std::string::npos) return true;
    if (starts("lib/clang/") && rel.find("/lib/windows/") != std::string::npos && rel.find("x86_64") != std::string::npos) return true;
    if (starts("include/") && !starts("include/c++/")) return true;
    return false;
}

bool extractToolchain(const fs::path& zip, const fs::path& dest, const Hooks& h, int p0, int p1, std::string& why)
{
    std::FILE* f = _wfopen(zip.wstring().c_str(), L"rb");
    if (!f) { why = "cannot open the downloaded file"; return false; }
    _fseeki64(f, 0, SEEK_END);
    const uint64_t size = static_cast<uint64_t>(_ftelli64(f));
    std::vector<ZipEntry> es;
    bool ok = zipDirectory(f, size, es, why);
    std::vector<const ZipEntry*> todo;
    const std::string root = std::string(kTcRoot) + "/";
    if (ok) {
        for (const auto& z : es) {
            if (z.name.compare(0, root.size(), root) != 0 || z.name.back() == '/') continue;
            const std::string rel = z.name.substr(root.size());
            if (rel.find("..") != std::string::npos) continue;
            if (z.unixMade && ((z.attr >> 16) & 0170000) == 0120000) continue;      // collegamenti simbolici
            if (wantedInToolchain(rel)) todo.push_back(&z);
        }
        if (todo.empty()) { why = "the compiler package has an unexpected layout"; ok = false; }
    }
    for (size_t i = 0; ok && i < todo.size(); ++i) {
        if (cancelled(h)) { why = "cancelled"; ok = false; break; }
        ok = inflateEntry(f, *todo[i], dest / fs::u8path(todo[i]->name.substr(root.size())), why);
        if (i % 50 == 0) report(h, p0 + static_cast<int>((p1 - p0) * i / todo.size()), "Unpacking the compiler");
    }
    std::fclose(f);
    return ok;
}

// ------------------------------------------------------------------ download (WinHTTP)
bool download(const std::string& url, const fs::path& part, uint64_t expectSize, const char* expectSha, const Hooks& h, int p0, int p1,
              std::string& why)
{
    const std::wstring wurl = widen(url);
    URL_COMPONENTSW uc{};
    uc.dwStructSize = sizeof uc;
    wchar_t host[256], path[2048];
    uc.lpszHostName = host;
    uc.dwHostNameLength = 256;
    uc.lpszUrlPath = path;
    uc.dwUrlPathLength = 2048;
    if (!WinHttpCrackUrl(wurl.c_str(), 0, 0, &uc)) { why = "bad download address"; return false; }
    HINTERNET ses = WinHttpOpen(L"ElektronEmu-Digitone/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME,
                                WINHTTP_NO_PROXY_BYPASS, 0);
    if (!ses) { why = "no network access"; return false; }
    WinHttpSetTimeouts(ses, 15000, 15000, 30000, 60000);
    HINTERNET con = WinHttpConnect(ses, host, uc.nPort, 0);
    HINTERNET req = con ? WinHttpOpenRequest(con, L"GET", path, nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                                             uc.nScheme == INTERNET_SCHEME_HTTPS ? WINHTTP_FLAG_SECURE : 0)
                        : nullptr;
    auto closeAll = [&] {
        if (req) WinHttpCloseHandle(req);
        if (con) WinHttpCloseHandle(con);
        WinHttpCloseHandle(ses);
    };
    if (!req || !WinHttpSendRequest(req, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
        !WinHttpReceiveResponse(req, nullptr)) {
        why = "cannot reach " + std::string(url.substr(0, url.find('/', 8)));
        closeAll();
        return false;
    }
    DWORD status = 0, sl = sizeof status;
    WinHttpQueryHeaders(req, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status, &sl,
                        WINHTTP_NO_HEADER_INDEX);
    if (status != 200) {
        why = "the download server answered " + std::to_string(status);
        closeAll();
        return false;
    }
    dnplat::makeDirs(part.parent_path());
    std::FILE* out = _wfopen(part.wstring().c_str(), L"wb");
    if (!out) { why = "cannot write " + u8(part); closeAll(); return false; }
    dn_sha256_ctx sha;
    dn_sha256_begin(&sha);
    uint64_t got = 0;
    bool ok = true;
    std::vector<char> buf(1 << 16);
    for (;;) {
        if (cancelled(h)) { why = "cancelled"; ok = false; break; }
        DWORD n = 0;
        if (!WinHttpReadData(req, buf.data(), static_cast<DWORD>(buf.size()), &n)) { why = "the download was interrupted"; ok = false; break; }
        if (n == 0) break;
        dn_sha256_add(&sha, buf.data(), n);
        if (std::fwrite(buf.data(), 1, n, out) != n) { why = "cannot write the downloaded file (disk full?)"; ok = false; break; }
        got += n;
        report(h, p0 + static_cast<int>((p1 - p0) * std::min<uint64_t>(got, expectSize) / expectSize),
               "Downloading the compiler: " + std::to_string(got >> 20) + " of " + std::to_string(expectSize >> 20) + " MB");
    }
    if (std::fclose(out) != 0) ok = false;
    closeAll();
    uint8_t d[32];
    dn_sha256_end(&sha, d);
    if (ok) {
        char hex[65];
        for (int i = 0; i < 32; ++i) std::snprintf(hex + 2 * i, 3, "%02x", d[i]);
        if (got != expectSize || std::strcmp(hex, expectSha) != 0) {
            why = "the downloaded compiler does not match its checksum";
            ok = false;
        }
    }
    if (!ok) {
        std::error_code ec;
        fs::remove(part, ec);
    }
    return ok;
}
#endif

// ------------------------------------------------------------------ il compilatore
struct Toolchain {
    std::string clang, sdk;
};

#if !defined(_WIN32)
bool findToolchain(const fs::path& scratch, Toolchain& tc, std::string& why)
{
    ProcOut f = runProc({"/usr/bin/xcrun", "--find", "clang"}, nullptr, false);
    if (!f.started) { why = "xcrun is not available"; return false; }
    if (f.code != 0) { why = firstLines(f.out); if (why.empty()) why = "xcrun cannot find clang"; return false; }
    tc.clang = trim(f.out);
    {
        const size_t nl = tc.clang.find_last_of('\n');          // l'ultima riga (eventuali avvisi prima)
        if (nl != std::string::npos) tc.clang = trim(tc.clang.substr(nl + 1));
    }
    ProcOut s = runProc({"/usr/bin/xcrun", "--show-sdk-path"}, nullptr, false);
    tc.sdk = s.code == 0 ? trim(s.out) : std::string();
    if (tc.clang.empty() || !dnplat::fileExists(fromU8(tc.clang))) { why = "xcrun cannot find clang"; return false; }
    // una prova vera: compila un file che include le intestazioni di sistema
    dnplat::makeDirs(scratch);
    const fs::path src = scratch / "probe.c", obj = scratch / "probe.o";
    {
        std::ofstream f2(src, std::ios::binary);
        f2 << "#include <string.h>\n#include <math.h>\n#include <stdlib.h>\nint main(void) { return (int)sqrt(4.0) - 2; }\n";
    }
    std::vector<std::string> a{tc.clang, "-c", "-O0", "-o", u8(obj), u8(src)};
    if (!tc.sdk.empty()) { a.insert(a.begin() + 1, tc.sdk); a.insert(a.begin() + 1, "-isysroot"); }
    ProcOut t = runProc(a, nullptr, false);
    std::error_code ec;
    fs::remove(src, ec);
    fs::remove(obj, ec);
    if (t.code != 0) {
        why = firstLines(t.out);
        if (why.empty()) why = "the compiler does not work";
        return false;
    }
    return true;
}
#else
fs::path winClang(const fs::path& toolRoot)
{
    return toolRoot / kTcRoot / "bin" / "clang.exe";
}

bool findToolchain(const fs::path& toolRoot, Toolchain& tc, std::string& why)
{
    const fs::path clang = winClang(toolRoot);
    if (!dnplat::fileExists(clang) || !dnplat::fileExists(toolRoot / (std::string(kTcRoot) + ".ok"))) {
        why = "the compiler is not installed yet";
        return false;
    }
    tc.clang = u8(clang);
    ProcOut v = runProc({tc.clang, "--version"}, nullptr, false);
    if (v.code != 0) {
        why = "the downloaded compiler does not run on this computer";
        return false;
    }
    return true;
}
#endif

// ------------------------------------------------------------------ compilazione
struct Job {
    fs::path src, obj;
};

bool writeText(const fs::path& p, const char* text)
{
    return dnplat::writeFile(p, reinterpret_cast<const uint8_t*>(text), std::strlen(text));
}

std::vector<std::string> compileArgs(const Toolchain& tc, const fs::path& srcDir, const Job& j)
{
    std::vector<std::string> a{tc.clang, "-O2", "-ffp-contract=off", "-fno-stack-protector", "-fomit-frame-pointer", "-fno-math-errno",
                               "-w", "-DDN_FIXED_IDLE", "-DDN_EXT_INFLATE", "-c"};
#if defined(__APPLE__)
    a.push_back("-fPIC");
#if defined(__aarch64__)
    a.push_back("-arch");
    a.push_back("arm64");
#endif
    if (!tc.sdk.empty()) {
        a.push_back("-isysroot");
        a.push_back(tc.sdk);
    }
#endif
    a.push_back("-I");
    a.push_back(srcDir.generic_u8string());
    a.push_back("-o");
    fs::path tmpobj = j.obj;
    tmpobj += ".tmp";
    a.push_back(tmpobj.generic_u8string());
    a.push_back(j.src.generic_u8string());
    return a;
}

} // namespace

std::string libName(const std::string& programHash)
{
#if defined(_WIN32)
    const char* ext = ".dll";
#else
    const char* ext = ".dylib";
#endif
    return "dsp-" + programHash.substr(0, 12) + "-" + kVersionTag + ext;
}

void openToolInstaller()
{
#if defined(__APPLE__)
    std::thread([] { runProc({"/usr/bin/xcode-select", "--install"}, nullptr, false); }).detach();
#endif
}

bool compilerAvailable(const fs::path& toolRoot, std::string& why)
{
    Toolchain tc;
#if defined(_WIN32)
    return findToolchain(toolRoot, tc, why);
#else
    return findToolchain(toolRoot.empty() ? fs::temp_directory_path() / "dn-probe" : toolRoot / "probe", tc, why);
#endif
}

Result build(const uint8_t* dnds, size_t dndsLen, const fs::path& lib, const fs::path& workRoot, const fs::path& toolRoot,
             const Hooks& hooks, std::string& detail)
{
    std::error_code ec;
    Toolchain tc;
    int base = 0;                                     // dove comincia la compilazione nella barra
    // 1. il compilatore
#if defined(_WIN32)
    if (!findToolchain(toolRoot, tc, detail)) {
        const fs::path part = toolRoot / "llvm-mingw.zip.part";
        std::string url = kTcUrl;
        if (const char* o = std::getenv("DN_TOOLCHAIN_URL")) url = o;       // prove: un server locale (l'impronta resta fissata)
        report(hooks, 0, "Downloading the compiler");
        dnplat::makeDirs(toolRoot);
        std::string why;
        if (!download(url, part, kTcSize, kTcSha256, hooks, 0, 28, why)) {
            detail = cancelled(hooks) ? std::string() : "could not download the compiler: " + why;
            return cancelled(hooks) ? Result::Cancelled : Result::Failed;
        }
        fs::remove_all(toolRoot / kTcRoot, ec);
        if (!extractToolchain(part, toolRoot / kTcRoot, hooks, 28, 35, why)) {
            fs::remove(part, ec);
            fs::remove_all(toolRoot / kTcRoot, ec);
            detail = cancelled(hooks) ? std::string() : "could not unpack the compiler: " + why;
            return cancelled(hooks) ? Result::Cancelled : Result::Failed;
        }
        fs::remove(part, ec);
        writeText(toolRoot / (std::string(kTcRoot) + ".ok"), "ok\n");
        if (!findToolchain(toolRoot, tc, why)) {
            detail = why;
            return Result::Failed;
        }
        base = 35;
    }
#else
    if (!findToolchain(workRoot / "probe", tc, detail)) return Result::NeedTools;
    (void)toolRoot;
#endif
    if (cancelled(hooks)) return Result::Cancelled;

    // 2. i sorgenti C del DSP (dal programma dell'interprete) e il runtime
    report(hooks, base, "Translating the DSP program");
    fs::remove_all(workRoot / "src", ec);
    fs::remove_all(workRoot / "obj", ec);
    const fs::path src = workRoot / "src", objd = workRoot / "obj";
    dnplat::makeDirs(src);
    dnplat::makeDirs(objd);
    struct SinkCtx {
        fs::path dir;
        std::vector<std::string> names;
        dn_sha256_ctx sha;
        bool bad = false;
    } sc;
    sc.dir = src;
    dn_sha256_begin(&sc.sha);
    auto sink = [](void* ctx, const char* name, const char* data, size_t len) -> int {
        auto* c = static_cast<SinkCtx*>(ctx);
        if (!dnplat::writeFile(c->dir / name, reinterpret_cast<const uint8_t*>(data), len)) { c->bad = true; return -1; }
        c->names.push_back(name);
        dn_sha256_add(&c->sha, data, len);
        return 0;
    };
    char err[256] = "";
    if (dn_dsp_gensrc(dnds, dndsLen, sink, &sc, err, sizeof err) != 0 || sc.bad) {
        detail = sc.bad ? "cannot write to the cache folder (disk full?)" : std::string("cannot translate the DSP program: ") + err;
        return Result::Failed;
    }
    struct Rt { const char* name; const char* text; };
    const Rt rts[] = {{"rt.h", kSrcRtH}, {"periph.h", kSrcPeriphH}, {"dn_dsp.h", kSrcDnDspH}, {"dn_dsp.c", kSrcDnDspC},
                      {"dn_rtmain.c", kSrcDnRtmainC}};
    for (const auto& r : rts) {
        if (!writeText(src / r.name, r.text)) { detail = "cannot write to the cache folder (disk full?)"; return Result::Failed; }
        dn_sha256_add(&sc.sha, r.text, std::strlen(r.text));
    }
    dn_sha256_add(&sc.sha, kVersionTag, std::strlen(kVersionTag));
    {
        uint8_t d[32];
        char key[17];
        dn_sha256_end(&sc.sha, d);
        for (int i = 0; i < 8; ++i) std::snprintf(key + 2 * i, 3, "%02x", d[i]);
        const std::string kh = std::string("#define DN_DSP_KEY \"") + key + "\"\n";
        if (!writeText(src / "dn_key.h", kh.c_str())) { detail = "cannot write to the cache folder (disk full?)"; return Result::Failed; }
    }

    // 3. gli oggetti (dsp_main.c e' dentro dn_rtmain.c)
    std::vector<Job> jobs;
    jobs.push_back({src / "dn_rtmain.c", objd / "dn_rtmain.o"});
    jobs.push_back({src / "dn_dsp.c", objd / "dn_dsp.o"});
    for (const auto& n : sc.names) {
        if (n == "dsp_main.c") continue;
        jobs.push_back({src / n, objd / (n.substr(0, n.size() - 2) + ".o")});
    }
    const size_t total = jobs.size();
    std::atomic<size_t> next{0}, done{0};
    std::atomic<bool> failed{false};
    std::mutex em;
    std::string firstErr;
    const unsigned hc = std::max(1u, std::thread::hardware_concurrency());
    const unsigned nthreads = hc <= 2 ? hc : hc - 1;      // un core resta alla DAW
    auto worker = [&] {
        for (;;) {
            if (failed.load() || cancelled(hooks)) return;
            const size_t i = next.fetch_add(1);
            if (i >= total) return;
            const Job& j = jobs[i];
            ProcOut r = runProc(compileArgs(tc, src, j), hooks.cancel, true);
            if (r.code != 0) {
                if (!cancelled(hooks)) {
                    std::lock_guard<std::mutex> g(em);
                    if (firstErr.empty()) firstErr = r.started ? firstLines(r.out) : "cannot run the compiler";
                    if (firstErr.empty()) firstErr = "the compiler failed";
                    failed = true;
                }
                return;
            }
            std::error_code e2;
            fs::path tmpobj = j.obj;
            tmpobj += ".tmp";
            fs::rename(tmpobj, j.obj, e2);
            const size_t d = done.fetch_add(1) + 1;
            report(hooks, base + static_cast<int>((96 - base) * d / total),
                   "Compiling the DSP: " + std::to_string(d) + " of " + std::to_string(total) + " parts");
        }
    };
    report(hooks, base, "Compiling the DSP: 0 of " + std::to_string(total) + " parts");
    {
        std::vector<std::thread> ts;
        for (unsigned t = 0; t < nthreads; ++t) ts.emplace_back(worker);
        for (auto& t : ts) t.join();
    }
    if (cancelled(hooks)) return Result::Cancelled;
    if (failed.load()) {
        detail = "the compiler failed: " + firstErr;
        return Result::Failed;
    }

    // 4. la libreria
    report(hooks, 97, "Linking the DSP");
    {
        std::string rsp;
        for (const auto& j : jobs) rsp += "\"" + j.obj.generic_u8string() + "\"\n";
        if (!dnplat::writeFile(workRoot / "objs.rsp", reinterpret_cast<const uint8_t*>(rsp.data()), rsp.size())) {
            detail = "cannot write to the cache folder (disk full?)";
            return Result::Failed;
        }
    }
    dnplat::makeDirs(lib.parent_path());
    fs::path tmp = lib;
    tmp += ".tmp";
    std::vector<std::string> la{tc.clang, "-shared"};
#if defined(__APPLE__)
#if defined(__aarch64__)
    la.push_back("-arch");
    la.push_back("arm64");
#endif
    if (!tc.sdk.empty()) {
        la.push_back("-isysroot");
        la.push_back(tc.sdk);
    }
#endif
    la.push_back("-o");
    la.push_back(tmp.generic_u8string());
    la.push_back("@" + (workRoot / "objs.rsp").generic_u8string());
    ProcOut lr = runProc(la, hooks.cancel, true);
    if (cancelled(hooks)) return Result::Cancelled;
    if (lr.code != 0) {
        detail = "the linker failed: " + firstLines(lr.out);
        return Result::Failed;
    }
    fs::rename(tmp, lib, ec);
    if (ec) {
        detail = "cannot install the library: " + ec.message();
        return Result::Failed;
    }
    // le altre librerie costruite dal plugin (firmware o versione diversi) non servono piu'; quella
    // personale dell'utente (dsp-native.dylib, vst/scripts/dsp-nativo.sh) non si tocca
    for (const auto& e : fs::directory_iterator(lib.parent_path(), ec)) {
        const std::string n = e.path().filename().string();
        const std::string ext = e.path().extension().string();
        if (e.path() != lib && n.rfind("dsp-", 0) == 0 && n != "dsp-native.dylib" && (ext == ".dylib" || ext == ".dll")) {
            std::error_code e3;
            fs::remove(e.path(), e3);
        }
    }
    report(hooks, 100, "Ready");
    detail.clear();
    fs::remove_all(workRoot, ec);                      // sorgenti e oggetti derivano dal firmware: non restano
    return Result::Ready;
}

} // namespace dnnative
