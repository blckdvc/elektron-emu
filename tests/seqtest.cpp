// seqtest.cpp -- automatic tests of the Elektron Emu view (SeqView.hpp), its pattern
// reader (SeqMem.hpp) and the keys of each machine (SeqSpec.hpp).
//
// Built once per machine (tests/CMakeLists.txt, MACHINE=dt|dn|st|ar|mc) against the
// machine's own core. Two modes:
//
//   seqtest --layout OUT.svg REPORT
//       no firmware: the view drawn with a made-up pattern; checks that every control
//       is inside the view and that controls of different kinds do not overlap.
//
//   seqtest --engine FIRMWARE.syx STATE.dtsz REPORT [OUT.svg]
//       the real OS: the view is driven like the plug-in does (clicks on its controls,
//       the queued key presses go to the panel, the pattern is read back from the
//       memory) and the results are checked: trigs written and erased, drag along a
//       row, another track, a second step page, mutes, pattern select, play / stop,
//       the page loop. STATE is made by a cold boot the first time (a few minutes).
//
// The report has one line per check: "PASS name" or "FAIL name: what happened",
// "SKIP name: why"; the exit code is the number of failures.

// the standard headers first: SeqView's private parts are opened for the tests below
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "stubvg.hpp"

#if defined(M_DN)
#include "dtcore.h"
#include "dncore.h"
#elif defined(M_ST)
#include "dtcore.h"
#include "stcore.h"
#elif defined(M_AR)
#include "arcore.h"
#elif defined(M_MC)
#include "mccore.h"
#else
#include "dtcore.h"
#endif
#include "panel_layout.hpp"
#include "SeqMem.hpp"

#define private public
#include "SeqView.hpp"
#undef private

namespace DISTRHO {
#include "SeqSpec.hpp"
}
using namespace DISTRHO;

// ------------------------------------------------------------------ report
static FILE* gReport = nullptr;
static int gFail = 0, gPass = 0, gSkip = 0;

static void report(const char* status, const std::string& name, const std::string& detail)
{
    const std::string line = std::string(status) + " " + name + (detail.empty() ? "" : ": " + detail);
    std::printf("%s\n", line.c_str());
    if (gReport) std::fprintf(gReport, "%s\n", line.c_str());
    if (!std::strcmp(status, "FAIL")) ++gFail;
    else if (!std::strcmp(status, "PASS")) ++gPass;
    else ++gSkip;
}

static void check(bool ok, const std::string& name, const std::string& detail = "")
{
    report(ok ? "PASS" : "FAIL", name, ok ? "" : detail);
}

static std::string fmt(const char* f, ...)
{
    char b[512];
    va_list a;
    va_start(a, f);
    std::vsnprintf(b, sizeof b, f, a);
    va_end(a);
    return b;
}

static void writeSvg(NanoVG& vg, float h, const char* path)
{
    if (!path) return;
    if (FILE* f = std::fopen(path, "w")) {
        std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='1000' height='%d'>"
                        "<rect width='1000' height='%d' fill='#16171A'/>%s</svg>", int(h), int(h), vg.out.c_str());
        std::fclose(f);
    }
}

// ------------------------------------------------------------------ layout
static int layoutTest(const char* svg)
{
    static uint8_t screen[128 * 64];
    for (int i = 0; i < 128 * 64; ++i) screen[i] = ((i / 128) % 9 == 0) ? 0xFF : 0;
    SeqHost h;
    h.send = [](int, int, int) {};
    h.keyLevel = [](int) { return 0.0f; };
    h.pageLevel = [](int i) { return i == 0 ? 1.0f : 0.3f; };
    h.screen = []() -> const uint8_t* { return screen; };
    const SeqSpec spec = seqSpec();
    SeqView v(spec, h);
    SeqGrid g;
    g.valid = true;
    g.tracks = spec.tracks;
    for (int t = 0; t < spec.tracks; ++t) g.trigs[t] = 0x1111ull << (t % 4);
    v.setGrid(g);
    const float H = 1000 * v.aspect();
    check(v.aspect() > 0.4f && v.aspect() < 0.75f, "layout.aspect", fmt("height/width %.3f", v.aspect()));

    for (int tab = 0; tab < 2; ++tab) {
        v.fFuncTab = tab == 1;
        NanoVG vg;
        v.draw(vg, 0, 0, 1000, H);
        if (tab == 0) writeSvg(vg, H, svg);
        const char* where = tab ? "func tab" : "pages tab";
        // inside the view
        int outside = 0;
        std::string first;
        for (const auto& t : v.fHits)
            if (t.x < -0.5f || t.y < -0.5f || t.x + t.w > 1000.5f || t.y + t.h > v.fHgt + 0.5f) {
                if (!outside++) first = fmt("kind %d index %d at %.0f,%.0f %.0fx%.0f", t.kind, t.index, t.x, t.y, t.w, t.h);
            }
        check(outside == 0, fmt("layout.inside (%s)", where), fmt("%d controls outside the view, first: %s", outside, first.c_str()));
        // no overlap between controls (the knob zones of the screen may touch each other)
        int overlaps = 0;
        for (size_t a = 0; a < v.fHits.size(); ++a)
            for (size_t b = a + 1; b < v.fHits.size(); ++b) {
                const auto &p = v.fHits[a], &q = v.fHits[b];
                const float ox = std::min(p.x + p.w, q.x + q.w) - std::max(p.x, q.x);
                const float oy = std::min(p.y + p.h, q.y + q.h) - std::max(p.y, q.y);
                if (ox > 1.0f && oy > 1.0f) {
                    if (!overlaps++) first = fmt("kind %d/%d index %d/%d", p.kind, q.kind, p.index, q.index);
                }
            }
        check(overlaps == 0, fmt("layout.no-overlap (%s)", where), fmt("%d overlapping pairs, first: %s", overlaps, first.c_str()));
    }
    // every key of the spec is a real code (no -1 left where a button is drawn)
    int bad = 0;
    for (int i = 0; i < spec.nPageKeys; ++i) bad += spec.pageKeys[i] < 0;
    for (int i = 0; i < spec.nFuncs; ++i) bad += spec.funcKeys[i] == 0;
    for (int i = 0; i < spec.tracks; ++i) bad += spec.trig[i] <= 0;
    check(bad == 0, "layout.spec-codes", fmt("%d buttons without a key code", bad));
    return 0;
}

// ------------------------------------------------------------------ engine
static uint8_t* readFile(const char* p, size_t* n)
{
    FILE* f = std::fopen(p, "rb");
    if (!f) return nullptr;
    std::fseek(f, 0, SEEK_END);
    const long l = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    uint8_t* b = (uint8_t*)std::malloc(l ? l : 1);
    if (std::fread(b, 1, size_t(l), f) != size_t(l)) { std::fclose(f); std::free(b); return nullptr; }
    std::fclose(f);
    *n = size_t(l);
    return b;
}

struct Rig {
    dt_engine* e = nullptr;
    dt_machine* m = nullptr;
    SeqView* v = nullptr;
    int loopPage = -1;
    double now = 0;
    long chunk = 0;
    uint8_t screen[128 * 64] = {};
#if defined(M_AR)
    uint8_t leds[128] = {}, pads[12][3] = {};
#elif defined(M_MC)
    uint8_t leds[8] = {}, acc[8] = {};
    bool lit[64] = {};
#elif defined(M_DN)
    uint8_t leds[64] = {};
#else
    uint8_t leds[50] = {};
#endif
    SeqData data;

    void readLeds()
    {
        dt_engine_screen(e, screen);
#if defined(M_AR)
        dt_engine_leds(e, leds, pads);
#elif defined(M_MC)
        dt_engine_leds(e, leds, acc);
        seqLit(leds, lit);
#elif defined(M_DN)
        dn_engine_leds(e, leds);
#else
        dt_engine_leds(e, leds);
#endif
    }

    float keyLevel(int code)
    {
#if defined(M_MC)
        return seqKeyLevel(lit, code);
#else
        return seqKeyLevel(leds, code);
#endif
    }

    float pageLevel(int i)
    {
#if defined(M_MC)
        return seqPageLevel(lit, i);
#else
        return seqPageLevel(leds, i);
#endif
    }

    // what the plug-in's UI does at 25 Hz
    void idle()
    {
        readLeds();
        v->screenChanged();
        uint8_t buf[DT_SEQ_MAX];
        SeqGrid g;
        if (seqmem::read(m, buf, sizeof buf) == sizeof(SeqData)) {
            std::memcpy(&data, buf, sizeof data);
            g.valid = true;
            g.pattern = data.pattern;
            g.tracks = data.tracks;
            g.mutes = data.mutes;
            for (int i = 0; i < 16; ++i) {
                g.trigs[i] = data.trigs[i];
                g.trigs2[i] = data.trigs2[i];
                g.step[i] = data.step[i] == 0xff ? -1 : data.step[i];
            }
        } else data.valid = 0;
        v->setGrid(g);
        v->tick(now);
        NanoVG vg;
        v->draw(vg, 0, 0, 1000, 1000 * v->aspect());
    }

    // the engine thread: the page loop every 2 chunks, the UI every 15 (40 ms)
    void run(int chunks)
    {
        for (int i = 0; i < chunks; ++i) {
            dt_engine_chunk(e);
            ++chunk;
            now += 128.0 / 48.0;                     // ms
            if (!(chunk & 1)) seqmem::loop(m, loopPage);
            if (chunk % 15 == 0) idle();
        }
    }

    // until the queued key presses are played, then a little more
    void settle(int extra = 120)
    {
        for (int i = 0; i < 4000 && v->busy(); ++i) run(15);
        run(extra);
    }

    const SeqView::Hit* find(int kind, int index, int code = -100000)
    {
        for (const auto& h : v->fHits)
            if (h.kind == kind && h.index == index && (code == -100000 || h.code == code)) return &h;
        return nullptr;
    }

    // a click in the middle of a control, as the window would send it
    bool click(int kind, int index, int code = -100000)
    {
        idle();
        const SeqView::Hit* h = find(kind, index, code);
        if (!h) return false;
        const float x = v->fOx + (h->x + h->w / 2) * v->fScale, y = v->fOy + (h->y + h->h / 2) * v->fScale;
        v->mouse(true, x, y, now);
        run(6);
        v->mouse(false, x, y, now);
        return true;
    }

    bool trig(int t, int s) const
    {
        if (s < 64) return (data.trigs[t] >> s) & 1;
        return (data.trigs2[t] >> (s - 64)) & 1;
    }
};

static int engineTest(const char* syxPath, const char* statePath, const char* svg)
{
    char err[512] = "";
    size_t sl = 0, stl = 0;
    uint8_t* syx = readFile(syxPath, &sl);
    if (!syx) { report("FAIL", "engine.firmware", fmt("cannot read %s", syxPath)); return 1; }
    dt_firmware fw;
    if (dt_fw_extract(syx, sl, &fw, err, sizeof err)) { report("FAIL", "engine.firmware", err); return 1; }
#if defined(M_DN)
    if (dn_prepare_dsp(syx, sl, "dncache", nullptr, nullptr, nullptr, err, sizeof err)) { report("FAIL", "engine.dsp", err); return 1; }
#endif
    uint8_t* st = readFile(statePath, &stl);
    if (!st) {
        std::printf("cold boot (first run, a few minutes)...\n");
        std::fflush(stdout);
#if defined(M_DN)
        const int rc = dn_make_session(syx, sl, "dncache", nullptr, nullptr, nullptr, &st, &stl, err, sizeof err);
#elif defined(M_ST)
        const int rc = st_coldboot_session(&fw, 1, &st, &stl, nullptr, err, sizeof err);
#elif defined(M_AR)
        const int rc = dt_coldboot_session(&fw, 1, &st, &stl, nullptr, nullptr, nullptr, err, sizeof err);
#elif defined(M_MC)
        const int rc = dt_coldboot_session(&fw, DT_COLDBOOT_RUNG, DT_COLDBOOT_SESSION, 1, &st, &stl, nullptr, nullptr, err, sizeof err);
#else
        const int rc = dt_coldboot_session_ex(&fw, DT_COLDBOOT_RUNG, DT_COLDBOOT_GUI, 1, DT_COLDBOOT_PLUSDRIVE, &st, &stl, nullptr, err, sizeof err);
#endif
        if (rc) { report("FAIL", "engine.coldboot", err); return 1; }
        if (FILE* f = std::fopen(statePath, "wb")) { std::fwrite(st, 1, stl, f); std::fclose(f); }
    }
#if defined(M_AR) || defined(M_MC)
    dt_engine* e = dt_engine_new_mem(&fw, nullptr, st, stl, nullptr, err, sizeof err);
#else
    dt_engine* e = dt_engine_new_mem(&fw, st, stl, nullptr, err, sizeof err);
#endif
    if (!e) { report("FAIL", "engine.start", err); return 1; }

    Rig r;
    r.e = e;
    r.m = dt_engine_machine(e);
    SeqHost h;
    h.send = [&r](int kind, int code, int arg) {
        if (std::getenv("SEQTEST_DEBUG")) std::printf("debug: send %d %d %d at %.0f ms\n", kind, code, arg, r.now);
        dt_engine_panel(r.e, kind, code, arg);
    };
    h.keyLevel = [&r](int code) { return r.keyLevel(code); };
    h.pageLevel = [&r](int i) { return r.pageLevel(i); };
    h.screen = [&r]() -> const uint8_t* { return r.screen; };
    h.setLoop = [&r](int p) { r.loopPage = p; };
    h.getLoop = [&r]() { return r.loopPage; };
    const SeqSpec spec = seqSpec();
    SeqView v(spec, h);
    r.v = &v;
    r.run(300);

    if (std::getenv("SEQTEST_DEBUG")) std::printf("debug: start REC %.2f PLAY %.2f\n", r.keyLevel(spec.rec), r.keyLevel(spec.play));
    // 1. the pattern is read
    check(r.data.valid, "read.valid", "seqmem::read gave nothing (addresses of another OS version?)");
    if (!r.data.valid) return 1;
    check(r.data.tracks == spec.tracks, "read.tracks", fmt("%d tracks read, %d in the spec", r.data.tracks, spec.tracks));
    const int pattern0 = r.data.pattern;
    if (std::getenv("SEQTEST_DEBUG"))
        for (int t = 0; t < spec.tracks; ++t) std::printf("debug: track %d trigs %016llx\n", t + 1, (unsigned long long)r.data.trigs[t]);

    // 2. a trig written and erased (the first track with an empty step on page 1)
    int s0 = -1, t0 = 0;
    for (int t = 0; t < spec.tracks && s0 < 0; ++t)
        for (int s = 0; s < 16 && s0 < 0; ++s) if (!r.trig(t, s)) { s0 = s; t0 = t; }
    if (s0 < 0) report("SKIP", "grid.write", "no empty step on page 1");
    else {
        r.click(SeqView::kBtnCell, t0, s0);
        r.settle();
        check(r.trig(t0, s0), "grid.write", fmt("track %d step %d still empty after a click", t0 + 1, s0 + 1));
        if (std::getenv("SEQTEST_DEBUG")) {
            std::printf("debug: REC level %.2f PLAY %.2f pages", r.keyLevel(spec.rec), r.keyLevel(spec.play));
            for (int k = 0; k < 6; ++k) { r.run(5); std::printf(" [%.2f %.2f %.2f %.2f]", r.pageLevel(0), r.pageLevel(1), r.pageLevel(2), r.pageLevel(3)); }
            std::printf(" osPage %d osPages %d\n", v.osPage(), v.osPages());
        }
        r.click(SeqView::kBtnCell, t0, s0);
        r.settle();
        check(!r.trig(t0, s0), "grid.erase", fmt("track %d step %d still set after a second click", t0 + 1, s0 + 1));
    }

    // 3. a drag along a row, four steps
    {
        // four empty steps in a row, on some track
        int t = -1, from = 0, to = 3;
        for (int k = 0; k < spec.tracks && t < 0; ++k)
            for (int f = 0; f + 3 < 16 && t < 0; ++f) {
                bool e = true;
                for (int s = f; s <= f + 3; ++s) e = e && !r.trig(k, s);
                if (e) { t = k; from = f; to = f + 3; }
            }
        if (t < 0) report("SKIP", "grid.drag", "no four empty steps in a row");
        else {
            r.idle();
            const SeqView::Hit* a = r.find(SeqView::kBtnCell, t, from);
            const SeqView::Hit* b = r.find(SeqView::kBtnCell, t, to);
            if (!a || !b) report("FAIL", "grid.drag", "cells not found");
            else {
                const float y = v.fOy + (a->y + a->h / 2) * v.fScale;
                const float x0 = v.fOx + (a->x + a->w / 2) * v.fScale, x1 = v.fOx + (b->x + b->w / 2) * v.fScale;
                v.mouse(true, x0, y, r.now);
                for (int k = 1; k <= 12; ++k) { v.motion(x0 + (x1 - x0) * k / 12, y, r.now); r.run(4); }
                v.mouse(false, x1, y, r.now);
                r.settle(200);
                int set = 0;
                for (int s = from; s <= to; ++s) set += r.trig(t, s);
                check(set == to - from + 1, "grid.drag", fmt("%d of %d steps set by the drag", set, to - from + 1));
                // erase them with the same drag
                r.idle();
                v.mouse(true, x0, y, r.now);
                for (int k = 1; k <= 12; ++k) { v.motion(x0 + (x1 - x0) * k / 12, y, r.now); r.run(4); }
                v.mouse(false, x1, y, r.now);
                r.settle(200);
                set = 0;
                for (int s = from; s <= to; ++s) set += r.trig(t, s);
                check(set == 0, "grid.drag-erase", fmt("%d steps left after the erasing drag", set));
            }
        }
    }

    // 4. another track: the click selects it and writes there
    {
        const int t = std::min(2, spec.tracks - 1);
        int s = -1;
        for (int k = 4; k < 16 && s < 0; ++k) if (!r.trig(t, k)) s = k;
        if (s < 0) report("SKIP", "grid.other-track", "no empty step");
        else {
            r.click(SeqView::kBtnCell, t, s);
            r.settle();
            check(r.trig(t, s), "grid.other-track", fmt("track %d step %d not set", t + 1, s + 1));
            check(v.fSel == t, "grid.select-track", fmt("track %d selected", v.fSel + 1));
            r.click(SeqView::kBtnCell, t, s);
            r.settle();
            check(!r.trig(t, s), "grid.other-track-erase", fmt("track %d step %d still set", t + 1, s + 1));
        }
    }

    // 5. a second step page, on a track that has one (the tracks can have their own length)
    int longTrack = -1;
    for (int t = 0; t < spec.tracks && longTrack < 0; ++t) {
        r.click(SeqView::kBtnTrack, t);
        r.settle(200);
        if (v.osPages() >= 2) longTrack = t;
    }
    if (longTrack < 0) report("SKIP", "grid.page2", "no track has a second page");
    else {
        const int t = longTrack;
        int s = -1;
        for (int k = 16; k < 32 && s < 0; ++k) if (!r.trig(t, k)) s = k;
        r.click(SeqView::kBtnViewPage, 1);
        r.settle();
        if (s < 0) report("SKIP", "grid.page2", "no empty step on page 2");
        else {
            r.click(SeqView::kBtnCell, t, s);
            r.settle();
            check(r.trig(t, s), "grid.page2", fmt("track %d step %d not set (OS on page %d)", t + 1, s + 1, v.osPage() + 1));
            r.click(SeqView::kBtnCell, t, s);
            r.settle();
            check(!r.trig(t, s), "grid.page2-erase", fmt("track %d step %d still set", t + 1, s + 1));
        }
        r.click(SeqView::kBtnViewPage, 0);
        r.settle();
    }

    // 6. mute and unmute track 2
    if (spec.mute == SeqSpec::kMuteNone) report("SKIP", "mute", "no mute keys in the spec");
    else {
        const int t = 1;
        const bool before = (r.data.mutes >> t) & 1;
        r.click(SeqView::kBtnMute, t);
        r.settle();
        const bool after = (r.data.mutes >> t) & 1;
        check(after != before, "mute.on", fmt("mute bit of track 2 stayed %d", int(before)));
        r.click(SeqView::kBtnMute, t);
        r.settle();
        check(((r.data.mutes >> t) & 1) == before, "mute.off", "the second click did not restore the mute");
    }

    // 7. pattern select: B03 from the list, then previous
    if (spec.ptnSel == SeqSpec::kPtnNone) report("SKIP", "pattern", "no pattern keys");
    else {
        r.click(SeqView::kBtnListToggle, 0);
        r.click(SeqView::kBtnBank, 1);
        r.click(SeqView::kBtnPattern, 18);
        r.settle(300);
        check(r.data.pattern == 18, "pattern.list", fmt("pattern %c%02d instead of B03", 'A' + r.data.pattern / 16, r.data.pattern % 16 + 1));
        r.click(SeqView::kBtnPrevNext, -1);
        r.settle(300);
        check(r.data.pattern == 17, "pattern.prev", fmt("pattern %c%02d instead of B02", 'A' + r.data.pattern / 16, r.data.pattern % 16 + 1));
        r.click(SeqView::kBtnListToggle, 0);                       // back to the first pattern
        r.click(SeqView::kBtnBank, pattern0 / 16);
        r.click(SeqView::kBtnPattern, pattern0);
        r.settle(300);
        check(r.data.pattern == pattern0, "pattern.back", fmt("pattern %d instead of %d", r.data.pattern, pattern0));
    }

    // 8. play: the play position moves, stop
    r.click(SeqView::kBtnKey, 0, spec.play);
    r.run(120);
    const int pt = longTrack >= 0 ? longTrack : 0;
    if (longTrack >= 0) { r.click(SeqView::kBtnTrack, pt); r.settle(60); }
    const int p0 = v.playPos(pt);
    if (std::getenv("SEQTEST_DEBUG")) std::printf("debug: playing PLAY %.2f REC %.2f\n", r.keyLevel(spec.play), r.keyLevel(spec.rec));
    r.run(600);
    const int p1 = v.playPos(pt);
    check(p0 >= 0 && p1 >= 0 && p1 != p0, "play.position", fmt("position %d then %d", p0, p1));

    // 9. page loop on page 2: once there, the play position stays in steps 17-32
    if (longTrack < 0) report("SKIP", "loop.page2", "no track has a second page");
    else {
        r.click(SeqView::kBtnLoop, 0);                              // LOOP PAGE (the page shown)
        r.click(SeqView::kBtnViewPage, 1);                          // ... page 2
        int outside = 0, inside = 0;
        for (int k = 0; k < 300; ++k) {
            r.run(10);
            const int p = v.playPos(pt);
            if (p < 0) continue;
            if (p >= 16 && p < 32) ++inside;
            else if (inside > 0) ++outside;
        }
        check(inside > 0 && outside == 0, "loop.page2", fmt("%d samples on page 2, %d outside after reaching it", inside, outside));
        r.click(SeqView::kBtnLoop, 0);
    }
    r.click(SeqView::kBtnKey, 0, spec.stop);
    r.run(400);
    check(v.playPos(pt) < 0, "play.stop", fmt("still playing at %d", v.playPos(pt)));

    if (svg) {
        NanoVG vg;
        v.draw(vg, 0, 0, 1000, 1000 * v.aspect());
        writeSvg(vg, 1000 * v.aspect(), svg);
    }
    return 0;
}

int main(int argc, char** argv)
{
    if (argc >= 4 && !std::strcmp(argv[1], "--layout")) {
        gReport = std::fopen(argv[3], "a");
        layoutTest(argv[2]);
    } else if (argc >= 5 && !std::strcmp(argv[1], "--engine")) {
        gReport = std::fopen(argv[4], "a");
        engineTest(argv[2], argv[3], argc > 5 ? argv[5] : nullptr);
    } else {
        std::fprintf(stderr, "usage: seqtest --layout OUT.svg REPORT | --engine FW.syx STATE.dtsz REPORT [OUT.svg]\n");
        return 2;
    }
    std::printf("%d passed, %d failed, %d skipped\n", gPass, gFail, gSkip);
    if (gReport) std::fclose(gReport);
    return gFail;
}
