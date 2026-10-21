// SeqView.hpp -- the "Modern" and "Elektron Emu" views shared by the five plug-ins
// (the same file in each vst/ folder, like dtcore/), and the View menu that chooses
// between them and the panel photo ("Classic").
//
// The views do not know the machine: they press the real keys and turn the real
// encoders of the emulated panel (UI::panel: 0 press, 1 release, 3 turn, 4 pad), and
// read back the LEDs, the 128x64 screen and, for "Elektron Emu", the pattern decoded
// from the memory of the OS (SeqMem.hpp). So the OS stays the reference.
//
//   Modern        tracks, the 16 steps of the selected track from the trig LEDs, the
//                 real screen, FUNC/NO/YES/arrows, the parameter pages and encoders.
//   Elektron Emu  the same, with the grid of every track of the pattern (from the
//                 memory: what is on the tracks that are not selected too), the mutes
//                 and the pattern name. A click in the grid selects the track, goes to
//                 the right step page and presses the trig key.
//
// The views are drawn in a 1000 x 640 frame scaled to the window.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

START_NAMESPACE_DISTRHO

struct SeqSpec {
    const char* title = "";
    int tracks = 16;                         // <= 16
    const char* trackNames[16] = {};
    int trig[16] = {};                       // trig key codes 1..16
    int nPageKeys = 0;                       // parameter pages (0: none)
    int pageKeys[8] = {};
    const char* pageNames[8] = {};
    int nEnc = 8;                            // encoders (A-H) or knobs
    int enc[12] = {};
    const char* encNames[12] = {};
    int data = -1;                           // Level/Data encoder (turn code), -1 none
    int func = -1, trk = -1, rec = -1, play = -1, stop = -1, page = -1;
    int yes = -1, no = -1, up = -1, down = -1, left = -1, right = -1;
    enum Select { kTrkTrig, kTrkPad, kPad } select = kTrkTrig;
    int padBase = 0;                         // pad id of track 1 (kTrkPad, kPad)
    int nPageLeds = 4;                       // step page LEDs
    const char* pagesTitle = "PARAMETER PAGES";
    unsigned char screenMask = 1;            // pixel lit when (value & mask) != 0
    // mute of a track: kModeTrig  FUNC + muteKey toggles the mute mode, then the trig key
    //                  kModePad   muteKey toggles the mute mode, then the pad
    //                  kHoldPad   FUNC held + the pad
    enum Mute { kMuteNone, kModeTrig, kModePad, kHoldPad } mute = kMuteNone;
    int muteKey = -1;
    const char* memo = "";                   // file name for the remembered view
};

// the pattern decoded from the memory (SeqMem.hpp) as the view needs it
struct SeqGrid {
    bool valid = false;
    int pattern = 0, tracks = 0;
    unsigned mutes = 0;
    uint64_t trigs[16] = {};
};

struct SeqHost {
    std::function<void(int kind, int code, int arg)> send;
    std::function<float(int code)> keyLevel;     // LED of a key, 0..1
    std::function<float(int i)> pageLevel;       // step page LED i, 0..1
    std::function<const uint8_t*()> screen;      // 128 x 64
};

// ------------------------------------------------------------------ View menu
// The button in the status bar and its menu: Classic (the panel photo), Modern,
// Elektron Emu. The choice is kept for the next windows (a small file per machine).
class ViewMenu {
public:
    enum { kClassic = 0, kModern = 1, kEmu = 2, kCount = 3 };
    int mode = kEmu;
    bool open = false;

    static const char* name(int m) { return m == kClassic ? "Classic" : m == kModern ? "Modern" : "Elektron Emu"; }
    static const char* help(int m)
    {
        return m == kClassic ? "The panel photo, as on the machine"
             : m == kModern  ? "Selected track, real screen, pages"
                             : "Every track of the pattern, mutes";
    }

    void load(const char* memo)
    {
        fPath = path(memo);
        if (FILE* f = fPath.empty() ? nullptr : std::fopen(fPath.c_str(), "r")) {
            int m = -1;
            if (std::fscanf(f, "%d", &m) == 1 && m >= 0 && m < kCount) mode = m;
            std::fclose(f);
        }
    }

    // the button: (x, y) is its left/top corner in the status bar
    void place(float x, float y, float w, float h) { fX = x; fY = y; fW = w; fH = h; }

    void drawButton(NanoVG& vg, float sf)
    {
        vg.beginPath();
        vg.roundedRect(fX, fY, fW, fH, 3 * sf);
        vg.fillColor(Color(0x2A, 0x2C, 0x31, 1.0f));
        vg.fill();
        vg.strokeColor(open ? Color(0xF2, 0xA9, 0x3B, 1.0f) : Color(0x4A, 0x4E, 0x56, 1.0f));
        vg.strokeWidth(1);
        vg.stroke();
        vg.fontFace(NANOVG_DEJAVU_SANS_TTF);
        vg.fontSize(11 * sf);
        vg.textAlign(NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        vg.fillColor(Color(0xEC, 0xEA, 0xE4, 1.0f));
        char t[48];
        std::snprintf(t, sizeof t, "View: %s", name(mode));
        vg.text(fX + 6 * sf, fY + fH / 2 + 1, t, nullptr);
        vg.beginPath();                                          // the arrow
        const float ax = fX + fW - 10 * sf, ay = fY + fH / 2;
        vg.moveTo(ax - 3 * sf, ay + 1.5f * sf);
        vg.lineTo(ax, ay - 1.5f * sf);
        vg.lineTo(ax + 3 * sf, ay + 1.5f * sf);
        vg.strokeColor(Color(0xEC, 0xEA, 0xE4, 1.0f));
        vg.strokeWidth(1.2f * sf);
        vg.stroke();
    }

    // the menu above the button
    void drawMenu(NanoVG& vg, float sf)
    {
        if (!open) return;
        const Box b = menuBox(sf);
        vg.beginPath();
        vg.roundedRect(b.x, b.y, b.w, b.h, 7 * sf);
        vg.fillColor(Color(0x1E, 0x20, 0x24, 0.98f));
        vg.fill();
        vg.strokeColor(Color(0x3A, 0x3D, 0x44, 1.0f));
        vg.strokeWidth(1);
        vg.stroke();
        vg.fontFace(NANOVG_DEJAVU_SANS_TTF);
        vg.fontSize(10 * sf);
        vg.textAlign(NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        vg.fillColor(Color(0x8E, 0x91, 0x98, 1.0f));
        vg.text(b.x + 12 * sf, b.y + 14 * sf, "VIEW", nullptr);
        for (int m = 0; m < kCount; ++m) {
            const Box r = itemBox(m, sf);
            if (m == mode) {
                vg.beginPath();
                vg.roundedRect(r.x, r.y, r.w, r.h, 5 * sf);
                vg.fillColor(Color(0x2C, 0x2F, 0x35, 1.0f));
                vg.fill();
                vg.beginPath();                                  // the tick
                vg.moveTo(r.x + 9 * sf, r.y + r.h / 2);
                vg.lineTo(r.x + 13 * sf, r.y + r.h / 2 + 4 * sf);
                vg.lineTo(r.x + 20 * sf, r.y + r.h / 2 - 5 * sf);
                vg.strokeColor(Color(0xF2, 0xA9, 0x3B, 1.0f));
                vg.strokeWidth(1.8f * sf);
                vg.stroke();
            }
            vg.fontSize(12 * sf);
            vg.fillColor(Color(0xEC, 0xEA, 0xE4, 1.0f));
            vg.text(r.x + 28 * sf, r.y + r.h / 2 - 7 * sf, name(m), nullptr);
            vg.fontSize(10.5f * sf);
            vg.fillColor(Color(0xA3, 0xA6, 0xAD, 1.0f));
            vg.text(r.x + 28 * sf, r.y + r.h / 2 + 8 * sf, help(m), nullptr);
        }
    }

    // a click: -1 not for the menu, otherwise handled (mode may have changed)
    int click(float px, float py, float sf)
    {
        const bool onButton = px >= fX && px <= fX + fW && py >= fY && py <= fY + fH;
        if (!open) {
            if (!onButton) return -1;
            open = true;
            return 1;
        }
        for (int m = 0; m < kCount; ++m) {
            const Box r = itemBox(m, sf);
            if (px >= r.x && px <= r.x + r.w && py >= r.y && py <= r.y + r.h) {
                mode = m;
                open = false;
                save();
                return 1;
            }
        }
        open = false;                                            // outside: it closes
        return 1;
    }

private:
    struct Box { float x, y, w, h; };

    Box menuBox(float sf) const
    {
        const float w = 250 * sf, h = (24 + kCount * 40 + 8) * sf;
        float x = fX + fW - w;
        if (x < 4 * sf) x = 4 * sf;
        return {x, fY - h - 4 * sf, w, h};
    }

    Box itemBox(int m, float sf) const
    {
        const Box b = menuBox(sf);
        return {b.x + 6 * sf, b.y + (24 + m * 40) * sf, b.w - 12 * sf, 38 * sf};
    }

    static std::string path(const char* memo)
    {
        if (!memo || !*memo) return std::string();
#if defined(_WIN32)
        const char* a = std::getenv("LOCALAPPDATA");
        if (!a) return std::string();
        std::string d = std::string(a) + "\\Elektron Emu";
        _mkdir(d.c_str());
        return d + "\\view-" + memo + ".txt";
#elif defined(__APPLE__)
        const char* h = std::getenv("HOME");
        if (!h) return std::string();
        std::string d = std::string(h) + "/Library/Caches/Elektron Emu";
        mkdir(d.c_str(), 0755);
        return d + "/view-" + memo + ".txt";
#else
        const char* h = std::getenv("HOME");
        if (!h) return std::string();
        std::string d = std::string(h) + "/.cache/Elektron Emu";
        mkdir(d.c_str(), 0755);
        return d + "/view-" + memo + ".txt";
#endif
    }

    void save() const
    {
        if (FILE* f = fPath.empty() ? nullptr : std::fopen(fPath.c_str(), "w")) {
            std::fprintf(f, "%d\n", mode);
            std::fclose(f);
        }
    }

    std::string fPath;
    float fX = 0, fY = 0, fW = 0, fH = 0;
};

// ------------------------------------------------------------------ the views
class SeqView {
public:
    enum Result { kNone, kHandled };

    SeqView(const SeqSpec& spec, SeqHost host) : fSpec(spec), fHost(std::move(host))
    {
        for (float& a : fAngle) a = 0;
    }

    void screenChanged() { fScreenDirty = true; }
    // the grid of every track when the pattern could be read, otherwise the Modern layout
    bool grid() const { return fStudio && fGrid.valid; }
    void setStudio(bool on) { fStudio = on; }
    bool studio() const { return fStudio; }

    // the pattern from the memory; -> true if it changed (to repaint)
    bool setGrid(const SeqGrid& g)
    {
        const bool same = g.valid == fGrid.valid && g.pattern == fGrid.pattern && g.mutes == fGrid.mutes &&
                          !std::memcmp(g.trigs, fGrid.trigs, sizeof g.trigs);
        fGrid = g;
        return !same;
    }

    // ------------------------------------------------------------ drawing
    void draw(NanoVG& vg, float x, float y, float w, float h)
    {
        fScale = std::max(0.2f, std::min(w / kW, h / kH));
        fOx = std::round(x + (w - kW * fScale) / 2);
        fOy = std::round(y + (h - kH * fScale) / 2);

        vg.beginPath();
        vg.rect(x, y, w, h);
        vg.fillColor(rgb(0x16171A));
        vg.fill();

        vg.save();
        vg.translate(fOx, fOy);
        vg.scale(fScale, fScale);
        vg.fontFace(NANOVG_DEJAVU_SANS_TTF);
        fHits.clear();

        drawHeader(vg);
        if (grid()) {
            drawGrid(vg);
        } else {
            drawTracks(vg);
            drawSteps(vg);
        }
        drawScreen(vg);
        drawNav(vg);
        drawParams(vg);

        vg.restore();
    }

    // ------------------------------------------------------------ input
    Result mouse(bool press, float x, float y, double now)
    {
        const float qx = (x - fOx) / fScale, qy = (y - fOy) / fScale;
        if (press) {
            const Hit* h = hitAt(qx, qy);
            if (!h) return kNone;
            fDown = *h;
            fDownActive = true;
            switch (h->kind) {
            case kBtnTrack: selectTrack(h->index, now); break;
            case kBtnStep: toggleStep(h->index, now); break;
            case kBtnCell: toggleCell(h->index, h->code, now); break;
            case kBtnMute: toggleMute(h->index, now); break;
            case kBtnViewPage: goToPage(h->index, now); break;
            case kBtnKey: keyDown(h->code); break;
            case kBtnFunc: setFunc(!fFunc); fDownActive = false; break;
            case kBtnKnob: fDragY = qy; fDragAcc = 0; fDragging = h->index; break;
            }
            return kHandled;
        }
        if (!fDownActive) return kNone;
        fDownActive = false;
        if (fDown.kind == kBtnKey) keyUp(fDown.code);
        if (fDown.kind == kBtnKnob) fDragging = -1;
        return kHandled;
    }

    bool motion(float /*x*/, float y)
    {
        if (fDragging < 0) return false;
        const float qy = (y - fOy) / fScale;
        fDragAcc += (fDragY - qy) / 3.0f;
        fDragY = qy;
        const int steps = int(fDragAcc);
        if (steps != 0) {
            fDragAcc -= steps;
            turnKnob(fDragging, steps);
        }
        return true;
    }

    bool scroll(float x, float y, float delta, bool fine)
    {
        const Hit* h = hitAt((x - fOx) / fScale, (y - fOy) / fScale);
        if (!h || h->kind != kBtnKnob || delta == 0) return false;
        int steps = int(std::max(-6.0f, std::min(6.0f, delta * 2)));
        if (steps == 0) steps = delta > 0 ? 1 : -1;
        if (fine) steps = steps > 0 ? 1 : -1;
        turnKnob(h->index, steps);
        return true;
    }

    // the queued key sequences (select a track, add a trig, mute); true if something was sent
    bool tick(double now)
    {
        bool sent = false;
        while (!fQueue.empty() && fQueue.front().at <= now) {
            const Ev e = fQueue.front();
            fQueue.erase(fQueue.begin());
            send(e.kind, e.code, e.arg);
            sent = true;
        }
        return sent;
    }

    bool busy() const { return !fQueue.empty(); }

private:
    static constexpr float kW = 1000, kH = 640;
    static constexpr double kGapMs = 70;

    enum Kind { kBtnTrack, kBtnStep, kBtnCell, kBtnMute, kBtnViewPage, kBtnKey, kBtnFunc, kBtnKnob };
    struct Hit { int kind = 0, index = 0, code = 0; float x = 0, y = 0, w = 0, h = 0; };
    struct Ev { double at; int kind, code, arg; };

    static Color rgb(unsigned v, float a = 1.0f)
    {
        return Color(int((v >> 16) & 0xFF), int((v >> 8) & 0xFF), int(v & 0xFF), a);
    }

    void send(int kind, int code, int arg = 0)
    {
        if (code >= 0 && fHost.send) fHost.send(kind, code, arg);
    }

    float level(int code) const { return code >= 0 && fHost.keyLevel ? std::max(0.0f, std::min(1.0f, fHost.keyLevel(code))) : 0.0f; }
    float pageLed(int i) const { return fHost.pageLevel ? std::max(0.0f, std::min(1.0f, fHost.pageLevel(i))) : 0.0f; }

    void queue(double& t, int kind, int code, int arg = 0)
    {
        if (code < 0) return;
        fQueue.push_back({t, kind, code, arg});
        t += kGapMs;
    }

    double queueStart(double now) const { return fQueue.empty() ? now : std::max(now, fQueue.back().at + kGapMs); }

    void queueSelect(double& at, int t)
    {
        switch (fSpec.select) {
        case SeqSpec::kTrkTrig:
            queue(at, 0, fSpec.trk); queue(at, 0, fSpec.trig[t]); queue(at, 1, fSpec.trig[t]); queue(at, 1, fSpec.trk);
            break;
        case SeqSpec::kTrkPad:
            queue(at, 0, fSpec.trk); queue(at, 4, fSpec.padBase + t, 1); queue(at, 4, fSpec.padBase + t, 0); queue(at, 1, fSpec.trk);
            break;
        case SeqSpec::kPad:
            queue(at, 4, fSpec.padBase + t, 1); queue(at, 4, fSpec.padBase + t, 0);
            break;
        }
    }

    void selectTrack(int t, double now)
    {
        if (t < 0 || t >= fSpec.tracks) return;
        fSel = t;
        double at = queueStart(now);
        queueSelect(at, t);
    }

    void queueGridRec(double& at)
    {
        if (fSpec.rec >= 0 && level(fSpec.rec) < 0.2f) { queue(at, 0, fSpec.rec); queue(at, 1, fSpec.rec); }
    }

    void toggleStep(int i, double now)
    {
        if (i < 0 || i >= 16) return;
        double at = queueStart(now);
        queueGridRec(at);
        queue(at, 0, fSpec.trig[i]); queue(at, 1, fSpec.trig[i]);
    }

    // the step page the OS shows (the brightest page LED) and how many pages the pattern has
    int osPage() const
    {
        int best = 0;
        float lv = -1;
        for (int i = 0; i < fSpec.nPageLeds; ++i)
            if (pageLed(i) > lv + 0.05f) { lv = pageLed(i); best = i; }
        return best;
    }

    int osPages() const
    {
        int n = 0;
        for (int i = 0; i < fSpec.nPageLeds; ++i) if (pageLed(i) > 0.05f) n = i + 1;
        return std::max(1, n);
    }

    // PAGE until the OS shows page p (PAGE goes round the pages of the pattern)
    void queuePage(double& at, int p)
    {
        const int n = osPages(), cur = osPage();
        if (fSpec.page < 0 || p >= n) return;
        const int k = ((p - cur) % n + n) % n;
        for (int i = 0; i < k; ++i) { queue(at, 0, fSpec.page); queue(at, 1, fSpec.page); }
    }

    void goToPage(int p, double now)
    {
        fViewPage = p;
        double at = queueStart(now);
        queuePage(at, p);
    }

    void toggleCell(int t, int s, double now)
    {
        if (t < 0 || t >= fSpec.tracks || s < 0 || s >= 64) return;
        double at = queueStart(now);
        if (t != fSel) { queueSelect(at, t); fSel = t; }
        queuePage(at, s / 16);
        queueGridRec(at);
        queue(at, 0, fSpec.trig[s % 16]); queue(at, 1, fSpec.trig[s % 16]);
    }

    void toggleMute(int t, double now)
    {
        if (t < 0 || t >= fSpec.tracks || fSpec.mute == SeqSpec::kMuteNone) return;
        double at = queueStart(now);
        switch (fSpec.mute) {
        case SeqSpec::kModeTrig:
            for (int pass = 0; pass < 2; ++pass) {                  // in, the trig, out
                queue(at, 0, fSpec.func); queue(at, 0, fSpec.muteKey); queue(at, 1, fSpec.muteKey); queue(at, 1, fSpec.func);
                if (pass == 0) { queue(at, 0, fSpec.trig[t]); queue(at, 1, fSpec.trig[t]); }
            }
            break;
        case SeqSpec::kModePad:
            queue(at, 0, fSpec.muteKey); queue(at, 1, fSpec.muteKey);
            queue(at, 4, fSpec.padBase + t, 100); queue(at, 4, fSpec.padBase + t, 0);
            queue(at, 0, fSpec.muteKey); queue(at, 1, fSpec.muteKey);
            break;
        case SeqSpec::kHoldPad:
            queue(at, 0, fSpec.func);
            queue(at, 4, fSpec.padBase + t, 100); queue(at, 4, fSpec.padBase + t, 0);
            queue(at, 1, fSpec.func);
            break;
        default:
            break;
        }
    }

    void keyDown(int code) { send(0, code); }

    void keyUp(int code)
    {
        send(1, code);
        if (fFunc) setFunc(false);                                  // FUNC + key, then FUNC goes up
    }

    void setFunc(bool on)
    {
        if (fSpec.func < 0 || on == fFunc) return;
        fFunc = on;
        send(on ? 0 : 1, fSpec.func);
    }

    int knobCode(int i) const
    {
        if (i < fSpec.nEnc) return fSpec.enc[i];
        return fSpec.data;
    }

    void turnKnob(int i, int steps)
    {
        const int code = knobCode(i);
        if (code < 0) return;
        steps = std::max(-30, std::min(30, steps));
        send(3, code, steps);
        if (i >= 0 && i < 13) fAngle[i] += steps * 0.12f;
    }

    const Hit* hitAt(float qx, float qy) const
    {
        for (const Hit& h : fHits)
            if (qx >= h.x && qx <= h.x + h.w && qy >= h.y && qy <= h.y + h.h) return &h;
        return nullptr;
    }

    void addHit(int kind, int index, int code, float x, float y, float w, float h)
    {
        Hit t; t.kind = kind; t.index = index; t.code = code; t.x = x; t.y = y; t.w = w; t.h = h;
        fHits.push_back(t);
    }

    // ------------------------------------------------------------ pieces
    void label(NanoVG& vg, float x, float y, const char* s, float size, unsigned color, int align)
    {
        vg.fontSize(size);
        vg.textAlign(align);
        vg.fillColor(rgb(color));
        vg.text(x, y, s, nullptr);
    }

    void button(NanoVG& vg, float x, float y, float w, float h, const char* text, float lit, bool accent = false, float size = 13)
    {
        vg.beginPath();
        vg.roundedRect(x, y, w, h, std::min(7.0f, h / 4));
        if (accent || lit > 0.05f) vg.fillColor(Color(rgb(0x2C2F35), rgb(0xF2A93B), std::max(accent ? 1.0f : 0.0f, std::min(1.0f, lit))));
        else vg.fillColor(rgb(0x2C2F35));
        vg.fill();
        const bool dark = accent || lit > 0.5f;
        label(vg, x + w / 2, y + h / 2 + 1, text, size, dark ? 0x16171A : 0xECEAE4, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
    }

    void drawHeader(NanoVG& vg)
    {
        label(vg, 24, 30, fSpec.title, 20, 0xECEAE4, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        label(vg, 24, 50, grid() ? "Elektron Emu view - the pattern read in the OS" : "Modern view - drives the machine's own keys",
              11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        if (grid()) {
            char p[8];
            std::snprintf(p, sizeof p, "%c%02d", 'A' + fGrid.pattern / 16, fGrid.pattern % 16 + 1);
            label(vg, 470, 26, "PATTERN", 10, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            label(vg, 470, 44, p, 20, 0xF2A93B, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        }
        float x = 760;
        const struct { const char* t; int code; } tr[] = {{"REC", fSpec.rec}, {"PLAY", fSpec.play}, {"STOP", fSpec.stop}};
        for (const auto& b : tr) {
            if (b.code < 0) continue;
            button(vg, x, 18, 64, 36, b.t, level(b.code));
            addHit(kBtnKey, 0, b.code, x, 18, 64, 36);
            x += 72;
        }
    }

    // Modern: the track list
    void drawTracks(NanoVG& vg)
    {
        label(vg, 24, 84, "TRACKS", 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        const int n = std::max(1, std::min(16, fSpec.tracks));
        const float top = 98, rowH = std::min(38.0f, (kH - 16 - top) / n);
        for (int t = 0; t < n; ++t) {
            const float y = top + t * rowH;
            const bool sel = t == fSel;
            vg.beginPath();
            vg.roundedRect(18, y + 1, 190, rowH - 2, 6);
            vg.fillColor(sel ? rgb(0x2C2F35) : rgb(0x1B1C20));
            vg.fill();
            if (sel) {
                vg.beginPath();
                vg.roundedRect(18, y + 1, 4, rowH - 2, 2);
                vg.fillColor(rgb(0xF2A93B));
                vg.fill();
            }
            char num[8];
            std::snprintf(num, sizeof num, "%02d", t + 1);
            label(vg, 34, y + rowH / 2, num, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            const char* name = fSpec.trackNames[t] ? fSpec.trackNames[t] : "";
            label(vg, 60, y + rowH / 2, name, 13, sel ? 0xECEAE4 : 0xC9CBD0, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnTrack, t, 0, 18, y + 1, 190, rowH - 2);
        }
    }

    void drawPageLeds(NanoVG& vg, float right, float y)
    {
        const int np = std::max(0, std::min(8, fSpec.nPageLeds));
        const float px = right - np * 14;
        for (int i = 0; i < np; ++i) {
            vg.beginPath();
            vg.roundedRect(px + i * 14, y, 10, 10, 2);
            vg.fillColor(Color(rgb(0x2A2C31), rgb(0xF2A93B), pageLed(i)));
            vg.fill();
        }
    }

    // Modern: the steps of the selected track from the trig LEDs
    void drawSteps(NanoVG& vg)
    {
        char t[48];
        std::snprintf(t, sizeof t, "STEPS - TRACK %d", fSel + 1);
        label(vg, 232, 84, t, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        drawPageLeds(vg, 500, 78);
        if (fSpec.page >= 0) {
            button(vg, 520, 70, 90, 28, "PAGE", 0);
            addHit(kBtnKey, 0, fSpec.page, 520, 70, 90, 28);
        }
        const float cw = 42, gap = 6, ch = 46;
        for (int i = 0; i < 16; ++i) {
            const float x = 232 + (i % 8) * (cw + gap), y = 108 + (i / 8) * (ch + 8);
            const float lv = level(fSpec.trig[i]);
            vg.beginPath();
            vg.roundedRect(x, y, cw, ch, 6);
            vg.fillColor(rgb(i % 4 == 0 ? 0x2A2C31 : 0x222429));
            vg.fill();
            if (lv > 0.03f) {
                vg.beginPath();
                vg.roundedRect(x, y, cw, ch, 6);
                vg.fillColor(rgb(0xF2A93B, std::min(1.0f, 0.25f + lv)));
                vg.fill();
            }
            char n[4];
            std::snprintf(n, sizeof n, "%d", i + 1);
            label(vg, x + cw / 2, y + ch - 10, n, 11, lv > 0.4f ? 0x16171A : 0x9A9DA3, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnStep, i, 0, x, y, cw, ch);
        }
        label(vg, 232, 232, "Click a step to add or remove a trig (grid recording turns on by itself).", 11, 0x7C8088,
              NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        label(vg, 232, 250, "The step row shows the trig LEDs of the machine.", 11, 0x7C8088, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    // Elektron Emu: every track of the pattern, from the memory
    void drawGrid(NanoVG& vg)
    {
        const int n = std::max(1, std::min(16, fSpec.tracks));
        const float left = 18, cells = 184, right = 612, top = 102, bottom = 418;
        const float rowH = std::min(30.0f, (bottom - top) / n);
        const int osp = osPage();
        if (fViewPage < 0) fViewPage = osp;
        const int page = fViewPage;

        // page tabs (the view follows the page the OS shows once it gets there)
        label(vg, left + 6, 82, "TRACKS", 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        const float tw = (right - cells - 3 * 6) / 4;
        for (int p = 0; p < 4; ++p) {
            const float x = cells + p * (tw + 6);
            char s[16];
            std::snprintf(s, sizeof s, "%d-%d", p * 16 + 1, p * 16 + 16);
            const bool cur = p == page;
            vg.beginPath();
            vg.roundedRect(x, 72, tw, 22, 5);
            vg.fillColor(rgb(cur ? 0x3A3D44 : 0x222429));
            vg.fill();
            if (p == osp) {
                vg.beginPath();
                vg.rect(x + 6, 91, tw - 12, 2);
                vg.fillColor(rgb(0xF2A93B));
                vg.fill();
            }
            label(vg, x + tw / 2, 83, s, 11, cur ? 0xECEAE4 : 0xA3A6AD, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnViewPage, p, 0, x, 72, tw, 22);
        }

        const float cw = (right - cells - 15 * 3) / 16;
        for (int t = 0; t < n; ++t) {
            const float y = top + t * rowH;
            const bool sel = t == fSel;
            const bool muted = fGrid.valid && ((fGrid.mutes >> t) & 1);
            vg.beginPath();
            vg.roundedRect(left, y + 1, cells - left - 36, rowH - 2, 5);
            vg.fillColor(sel ? rgb(0x2C2F35) : rgb(0x1B1C20));
            vg.fill();
            if (sel) {
                vg.beginPath();
                vg.roundedRect(left, y + 1, 3, rowH - 2, 1.5f);
                vg.fillColor(rgb(0xF2A93B));
                vg.fill();
            }
            char num[8];
            std::snprintf(num, sizeof num, "%02d", t + 1);
            const float fs = rowH < 24 ? 11 : 12;
            label(vg, left + 10, y + rowH / 2, num, fs - 1, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            label(vg, left + 32, y + rowH / 2, fSpec.trackNames[t] ? fSpec.trackNames[t] : "", fs, muted ? 0x7C8088 : (sel ? 0xECEAE4 : 0xC9CBD0),
                  NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnTrack, t, 0, left, y + 1, cells - left - 36, rowH - 2);

            if (fSpec.mute != SeqSpec::kMuteNone) {
                const float mx = cells - 32, mh = std::min(22.0f, rowH - 4);
                vg.beginPath();
                vg.roundedRect(mx, y + (rowH - mh) / 2, 26, mh, 4);
                vg.fillColor(rgb(muted ? 0xE8875A : 0x2A2C31));
                vg.fill();
                label(vg, mx + 13, y + rowH / 2 + 1, "M", 11, muted ? 0x16171A : 0xA3A6AD, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
                addHit(kBtnMute, t, 0, mx, y + (rowH - mh) / 2, 26, mh);
            }

            for (int i = 0; i < 16; ++i) {
                const int s = page * 16 + i;
                const float x = cells + i * (cw + 3);
                const bool on = fGrid.valid && ((fGrid.trigs[t] >> s) & 1);
                vg.beginPath();
                vg.roundedRect(x, y + 1.5f, cw, rowH - 3, 3);
                if (on) vg.fillColor(rgb(sel ? 0xF2A93B : 0xC98B34, muted ? 0.35f : 1.0f));
                else vg.fillColor(rgb(i % 4 == 0 ? 0x2A2C31 : 0x222429));
                vg.fill();
                addHit(kBtnCell, t, s, x, y + 1.5f, cw, rowH - 3);
            }
        }
        label(vg, cells, bottom + 8, "Click a cell to add or remove a trig. M mutes the track.", 11, 0x7C8088,
                  NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    void drawScreen(NanoVG& vg)
    {
        const float x = 632, y = 70, w = 344, h = 172;
        vg.beginPath();
        vg.roundedRect(x - 8, y - 8, w + 16, h + 16, 10);
        vg.fillColor(rgb(0x050506));
        vg.fill();
        const uint8_t* s = fHost.screen ? fHost.screen() : nullptr;
        if (!s) return;
        if (!fScreenImg.isValid()) {
            fScreenPx.assign(128 * 64 * 4, 0);
            fScreenImg = vg.createImageFromRGBA(128, 64, fScreenPx.data(), 1 << 5 /* NVG_IMAGE_NEAREST */);
            fScreenDirty = true;
        }
        if (fScreenDirty) {
            fScreenDirty = false;
            for (int i = 0; i < 128 * 64; ++i) {
                const bool on = (s[i] & fSpec.screenMask) != 0;
                uint8_t* p = &fScreenPx[size_t(i) * 4];
                p[0] = on ? 0xE8 : 0x0C; p[1] = on ? 0xF6 : 0x0E; p[2] = on ? 0xFF : 0x12; p[3] = 0xFF;
            }
            fScreenImg.update(fScreenPx.data());
        }
        vg.beginPath();
        vg.rect(x, y, w, h);
        vg.fillPaint(vg.imagePattern(x, y, w, h, 0, fScreenImg, 1.0f));
        vg.fill();
    }

    void drawNav(NanoVG& vg)
    {
        const float y = 262, h = 38;
        if (fSpec.func >= 0) {
            button(vg, 632, y, 72, h, "FUNC", fFunc ? 1.0f : 0.0f);
            addHit(kBtnFunc, 0, fSpec.func, 632, y, 72, h);
        }
        const struct { const char* t; int code; float x, w; } k[] = {
            {"NO", fSpec.no, 712, 52}, {"YES", fSpec.yes, 770, 52},
            {"<", fSpec.left, 836, 32}, {"^", fSpec.up, 872, 32}, {"v", fSpec.down, 908, 32}, {">", fSpec.right, 944, 32}};
        for (const auto& b : k) {
            if (b.code < 0) continue;
            button(vg, b.x, y, b.w, h, b.t, 0);
            addHit(kBtnKey, 0, b.code, b.x, y, b.w, h);
        }
        if (grid()) {                                              // PAGE and the page LEDs under the screen
            drawPageLeds(vg, 900, 326);
            if (fSpec.page >= 0) {
                button(vg, 906, 318, 70, 26, "PAGE", 0, false, 12);
                addHit(kBtnKey, 0, fSpec.page, 906, 318, 70, 26);
            }
        } else if (fSpec.func >= 0) {
            label(vg, 632, 316, "FUNC stays down for the next key.", 11, 0x7C8088, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        }
    }

    void drawParams(NanoVG& vg)
    {
        const float left = grid() ? 18 : 232, right = 976, width = right - left;
        float top = grid() ? 446 : 350;
        if (fSpec.nPageKeys > 0) {
            label(vg, left, top - 10, fSpec.pagesTitle, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            const int n = fSpec.nPageKeys;
            const float gap = 8, bw = std::min(110.0f, (width - gap * (n - 1)) / n), bh = grid() ? 28 : 34;
            for (int i = 0; i < n; ++i) {
                const float x = left + i * (bw + gap);
                button(vg, x, top, bw, bh, fSpec.pageNames[i] ? fSpec.pageNames[i] : "", level(fSpec.pageKeys[i]));
                addHit(kBtnKey, 0, fSpec.pageKeys[i], x, top, bw, bh);
            }
            top += bh + (grid() ? 10 : 20);
        }

        const int nk = std::min(12, fSpec.nEnc) + (fSpec.data >= 0 ? 1 : 0);
        if (nk <= 0) return;
        const int perRow = nk <= (grid() ? 13 : 9) ? nk : (nk + 1) / 2;
        const int rows = (nk + perRow - 1) / perRow;
        const float slot = width / perRow;
        const float rowH = std::min(140.0f, (kH - 22 - top) / rows);
        const float r = std::min(30.0f, std::min(slot, rowH) * 0.28f);
        for (int i = 0; i < nk; ++i) {
            const float cx = left + (i % perRow) * slot + slot / 2;
            const float cy = top + (i / perRow) * rowH + r + 10;
            const bool isData = i >= std::min(12, fSpec.nEnc);
            const char* name = isData ? "DATA" : (fSpec.encNames[i] ? fSpec.encNames[i] : "");
            const bool active = fDragging == i;
            vg.beginPath();
            vg.circle(cx, cy, r);
            vg.fillColor(rgb(0x26282D));
            vg.fill();
            vg.beginPath();
            vg.arc(cx, cy, r + 5, 0.75f * float(M_PI), 2.25f * float(M_PI), NanoVG::CW);
            vg.strokeColor(rgb(active ? 0xF2A93B : 0x33363C));
            vg.strokeWidth(4);
            vg.lineCap(NanoVG::ROUND);
            vg.stroke();
            const float a = (i < 13 ? fAngle[i] : 0) - float(M_PI) / 2;
            vg.beginPath();
            vg.moveTo(cx + std::cos(a) * r * 0.35f, cy + std::sin(a) * r * 0.35f);
            vg.lineTo(cx + std::cos(a) * r * 0.85f, cy + std::sin(a) * r * 0.85f);
            vg.strokeColor(rgb(isData ? 0x6FB7FF : 0xF2A93B));
            vg.strokeWidth(3);
            vg.stroke();
            label(vg, cx, cy + r + 16, name, 12, 0xC9CBD0, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnKnob, i, 0, cx - slot / 2 + 4, cy - r - 8, slot - 8, 2 * r + 34);
        }
        label(vg, left, kH - 10, "Drag up/down or use the wheel on a knob (shift: one step). The values are on the screen.", 11, 0x7C8088,
              NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    // ------------------------------------------------------------ state
    SeqSpec fSpec;
    SeqHost fHost;
    SeqGrid fGrid;
    bool fStudio = false;
    float fScale = 1, fOx = 0, fOy = 0;
    std::vector<Hit> fHits;
    Hit fDown;
    bool fDownActive = false, fFunc = false;
    int fSel = 0, fDragging = -1, fViewPage = -1;
    float fDragY = 0, fDragAcc = 0;
    float fAngle[13];
    std::vector<Ev> fQueue;
    NanoImage fScreenImg;
    std::vector<uint8_t> fScreenPx;
    bool fScreenDirty = true;
};

END_NAMESPACE_DISTRHO
