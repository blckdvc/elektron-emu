// SeqView.hpp -- the "Elektron Emu" view shared by the five plug-ins (the same file in
// each vst/ folder, like dtcore/), and the View menu that chooses between it and the
// panel photo ("Classic").
//
// The view does not know the machine: it presses the real keys and turns the real
// encoders of the emulated panel (UI::panel: 0 press, 1 release, 3 turn, 4 pad), and
// reads back the LEDs, the 128x64 screen and the pattern decoded from the memory of
// the OS (SeqMem.hpp). So the OS stays the reference.
//
//   left    the grid of every track of the pattern (trigs, mutes, play position),
//           the step pages; a click selects the track, goes to the step page and
//           presses the trig key
//   top     the pattern (a list of the banks and their 16 slots, previous / next),
//           FOLLOW (the grid follows the play position), REC / PLAY / STOP
//   right   the real screen, large: its parameters are turned by dragging them on
//           the screen (the screen shows A-D on the top row and E-H below, as the
//           encoders); FUNC / NO / YES / arrows, PAGE, the parameter pages
//
// Without the pattern (an OS whose addresses are not known) the left side shows the
// selected track from the trig LEDs.
//
// The view is drawn in a 1000 x 640 frame scaled to the window.
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
    // the parameters on the screen: 4 columns x 2 rows (A-D, E-H) in the screen
    // pixels x0..127, y0..yMid (top row), yMid..63 (bottom row); false: knobs instead
    bool screenKnobs = true;
    int knobX0 = 16, knobY0 = 10, knobYMid = 37, knobY1 = 64;
    // where the screen shows the track level (turned with the Level/Data encoder),
    // in screen pixels x0, y0, x1, y1; x1 = 0: none
    int levelZone[4] = {0, 36, 16, 64};
    // mute of a track: kModeTrig  FUNC + muteKey toggles the mute mode, then the trig key
    //                  kModePad   muteKey toggles the mute mode, then the pad
    //                  kHoldPad   FUNC held + the pad
    enum Mute { kMuteNone, kModeTrig, kModePad, kHoldPad } mute = kMuteNone;
    int muteKey = -1;
    // pattern select: kPtnArrows  PTN held, LEFT / RIGHT to the bank, the trig key
    //                 kBankKey    the bank key held, the trig key
    //                 kPtnPad     PTN held, the pad of the bank, the trig key
    enum PtnSel { kPtnNone, kPtnArrows, kBankKey, kPtnPad } ptnSel = kPtnNone;
    int ptnKey = -1;
    int bankKeys[8] = {};
    int nBanks = 8;
    const char* memo = "";                   // file name for the remembered settings
    // the FUNC + key functions, flat: a button each (FUNC is held, the key tapped;
    // a negative code: that key alone, for the menus with a key of their own)
    int nFuncs = 0;
    const char* funcNames[24] = {};
    int funcKeys[24] = {};
};

// the pattern decoded from the memory (SeqMem.hpp) as the view needs it
struct SeqGrid {
    bool valid = false;
    bool playing = false;
    int pattern = 0, tracks = 0;
    unsigned mutes = 0;
    uint64_t trigs[16] = {};                 // steps 1-64
    uint64_t trigs2[16] = {};                // steps 65-128 (Digitakt II, Digitone II)
    int step[16] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};   // play position, -1 none
};

struct SeqHost {
    std::function<void(int kind, int code, int arg)> send;
    std::function<float(int code)> keyLevel;     // LED of a key, 0..1
    std::function<float(int i)> pageLevel;       // step page LED i, 0..1
    std::function<const uint8_t*()> screen;      // 128 x 64
    std::function<void(int page)> setLoop;       // play one step page in a loop (-1: all the pages)
    std::function<int()> getLoop;
};

// a small file per machine for the remembered settings (view, follow)
inline std::string seqMemoPath(const char* kind, const char* memo)
{
    if (!memo || !*memo) return std::string();
#if defined(_WIN32)
    const char* a = std::getenv("LOCALAPPDATA");
    if (!a) return std::string();
    std::string d = std::string(a) + "\\Elektron Emu";
    _mkdir(d.c_str());
    return d + "\\" + kind + "-" + memo + ".txt";
#elif defined(__APPLE__)
    const char* h = std::getenv("HOME");
    if (!h) return std::string();
    std::string d = std::string(h) + "/Library/Caches/Elektron Emu";
    mkdir(d.c_str(), 0755);
    return d + "/" + kind + "-" + memo + ".txt";
#else
    const char* h = std::getenv("HOME");
    if (!h) return std::string();
    std::string d = std::string(h) + "/.cache/Elektron Emu";
    mkdir(d.c_str(), 0755);
    return d + "/" + kind + "-" + memo + ".txt";
#endif
}

inline int seqMemoLoad(const char* kind, const char* memo, int def)
{
    const std::string p = seqMemoPath(kind, memo);
    int v = def;
    if (FILE* f = p.empty() ? nullptr : std::fopen(p.c_str(), "r")) {
        if (std::fscanf(f, "%d", &v) != 1) v = def;
        std::fclose(f);
    }
    return v;
}

inline void seqMemoSave(const char* kind, const char* memo, int v)
{
    const std::string p = seqMemoPath(kind, memo);
    if (FILE* f = p.empty() ? nullptr : std::fopen(p.c_str(), "w")) {
        std::fprintf(f, "%d\n", v);
        std::fclose(f);
    }
}

// ------------------------------------------------------------------ View menu
// The button in the status bar and its menu: Classic (the panel photo) or Elektron Emu.
// The choice is kept for the next windows.
class ViewMenu {
public:
    enum { kClassic = 0, kEmu = 1, kCount = 2 };
    int mode = kEmu;
    bool open = false;

    static const char* name(int m) { return m == kClassic ? "Classic" : "Elektron Emu"; }
    static const char* help(int m)
    {
        return m == kClassic ? "The panel photo, as on the machine" : "The pattern, the screen, the pages";
    }

    void load(const char* memo)
    {
        fMemo = memo ? memo : "";
        const int m = seqMemoLoad("view", fMemo.c_str(), 2);
        mode = m == 0 ? kClassic : kEmu;             // 1 and 2 were Modern and Elektron Emu
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
                seqMemoSave("view", fMemo.c_str(), mode == kClassic ? 0 : 2);
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

    std::string fMemo;
    float fX = 0, fY = 0, fW = 0, fH = 0;
};

// ------------------------------------------------------------------ the view
class SeqView {
public:
    enum Result { kNone, kHandled };

    SeqView(const SeqSpec& spec, SeqHost host) : fSpec(spec), fHost(std::move(host))
    {
        for (float& a : fAngle) a = 0;
        fFollow = seqMemoLoad("follow", fSpec.memo, 1) != 0;
        fHgt = contentHeight();
        fLoop = fHost.getLoop ? fHost.getLoop() : -1;
        if (fLoop >= 0) fViewPage = fLoop;
    }

    void screenChanged() { fScreenDirty = true; }
    void setStudio(bool) {}                          // one view now (kept for the UIs)
    // the grid of every track when the pattern could be read, otherwise the trig LEDs
    bool grid() const { return fGrid.valid; }

    // the pattern from the memory; -> true if it changed (to repaint)
    bool setGrid(const SeqGrid& g)
    {
        sampleLeds();
        const bool same = g.valid == fGrid.valid && g.playing == fGrid.playing && g.pattern == fGrid.pattern &&
                          g.mutes == fGrid.mutes && !std::memcmp(g.trigs, fGrid.trigs, sizeof g.trigs) &&
                          !std::memcmp(g.trigs2, fGrid.trigs2, sizeof g.trigs2) &&
                          !std::memcmp(g.step, fGrid.step, sizeof g.step);
        if (level(fSpec.play) > kLit) fPlayLedSeen = true;
        // the positions move while it plays (and go back to 0 at STOP)
        if (std::memcmp(g.step, fGrid.step, sizeof g.step)) fStill = 0;
        else if (fStill < 1000) ++fStill;
        fGrid = g;
        const int s = playPos(fSel);
        if (fFollow && fLoop < 0 && s >= 0 && s / 16 != fViewPage) {   // the grid goes to the page that plays
            fViewPage = s / 16;
            return true;
        }
        return !same;
    }

    // playing: the positions moved lately; the step that plays on track t (each track
    // runs at its own length and speed, so the positions can differ from track to track)
    bool playing() const
    {
        if (fGrid.playing) return true;
        if (fPlayLedSeen) return level(fSpec.play) > kLit;          // the PLAY LED, once seen lit
        return fStill < 30;                                         // otherwise: the positions move
    }
    int playPos(int t) const
    {
        if (!fGrid.valid || !playing() || t < 0 || t > 15 || fGrid.step[t] < 0 || fGrid.step[t] >= 128) return -1;
        return fGrid.step[t];
    }
    bool trigOn(int t, int s) const
    {
        if (t < 0 || t > 15 || s < 0 || s >= 128) return false;
        return s < 64 ? ((fGrid.trigs[t] >> s) & 1) : ((fGrid.trigs2[t] >> (s - 64)) & 1);
    }

    // height / width of the view, to size the window (the width is the panel's)
    float aspect() const { return fHgt / kW; }

    // ------------------------------------------------------------ drawing
    void draw(NanoVG& vg, float x, float y, float w, float h)
    {
        fScale = std::max(0.2f, std::min(w / kW, h / fHgt));
        fOx = std::round(x + (w - kW * fScale) / 2);
        fOy = std::round(y + (h - fHgt * fScale) / 2);

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
        if (grid()) drawGrid(vg);
        else { drawTracks(vg); drawSteps(vg); }
        drawScreen(vg);
        drawNav(vg);
        drawParams(vg);
        drawPatternList(vg);                         // on top (and its hits first)

        vg.restore();
    }

    // ------------------------------------------------------------ input
    Result mouse(bool press, float x, float y, double now)
    {
        const float qx = (x - fOx) / fScale, qy = (y - fOy) / fScale;
        if (press) {
            const Hit* h = hitAt(qx, qy);
            if (fListOpen && (!h || (h->kind != kBtnPattern && h->kind != kBtnBank && h->kind != kBtnListToggle))) {
                fListOpen = false;                   // a click outside closes the list
                return kHandled;
            }
            if (!h) return kNone;
            fDown = *h;
            fDownActive = true;
            switch (h->kind) {
            case kBtnTrack: selectTrack(h->index, now); break;
            case kBtnStep: toggleStep(h->index, now); break;
            case kBtnCell:                                           // click, or click and drag on the row
                fPaint = true;
                fPaintTrack = h->index;
                fPaintOn = !trigOn(h->index, h->code);
                fPainted.assign(128, false);
                fPainted[size_t(h->code)] = true;
                toggleCell(h->index, h->code, now);
                break;
            case kBtnMute: toggleMute(h->index, now); break;
            case kBtnViewPage:
                if (fLoop >= 0) setLoop(h->index);
                goToPage(h->index, now);
                break;
            case kBtnLoop: setLoop(fLoop >= 0 ? -1 : std::max(0, fViewPage)); break;
            case kBtnTab: fFuncTab = h->index == 1; break;
            case kBtnFuncKey: {
                double at = queueStart(now);
                if (fFunc) setFunc(false);
                if (h->code < 0) queueTap(at, -h->code);                 // a key of its own (no FUNC)
                else { queue(at, 0, fSpec.func); queueTap(at, h->code); queue(at, 1, fSpec.func); }
                break;
            }
            case kBtnKey: keyDown(h->code); break;
            case kBtnFunc: setFunc(!fFunc); fDownActive = false; break;
            case kBtnKnob: fDragY = qy; fDownQY = qy; fMoved = false; fDragAcc = 0; fDragging = h->index; break;
            case kBtnFollow:
                fFollow = !fFollow;
                seqMemoSave("follow", fSpec.memo, fFollow ? 1 : 0);
                break;
            case kBtnListToggle: fListOpen = !fListOpen; fListBank = fGrid.pattern / 16; break;
            case kBtnBank: if (h->index >= 0) fListBank = h->index; break;
            case kBtnPattern: fListOpen = false; selectPattern(h->index, now); break;
            case kBtnPrevNext: {
                const int n = std::max(1, fSpec.nBanks) * 16;
                selectPattern(((fGrid.pattern + h->index) % n + n) % n, now);
                break;
            }
            }
            return kHandled;
        }
        fPaint = false;
        if (!fDownActive) return kNone;
        fDownActive = false;
        if (fDown.kind == kBtnKey) keyUp(fDown.code);
        if (fDown.kind == kBtnKnob) {
            fDragging = -1;
            if (!fMoved && fDown.code == 1 && fDown.index < fSpec.nEnc) {   // a click on the screen, no drag
                const int i = std::max(0, std::min(7, fDown.index));    // a switch: on, then off...
                turnKnob(fDown.index, fClickDir[i]);
                fClickDir[i] = -fClickDir[i];
            }
        }
        return kHandled;
    }

    // drag on a knob or on a parameter of the screen; the hover of the screen parameters
    bool motion(float x, float y, double now = 0, bool fine = false)
    {
        const float qx = (x - fOx) / fScale, qy = (y - fOy) / fScale;
        if (fPaint) {                                    // the cells of the row under the drag
            const float rowY = fDown.y + fDown.h / 2;
            const Hit* h = hitAt(qx, rowY);
            if (h && h->kind == kBtnCell && h->index == fPaintTrack && !fPainted[size_t(h->code)]) {
                fPainted[size_t(h->code)] = true;
                if (trigOn(h->index, h->code) != fPaintOn) {
                    toggleCell(h->index, h->code, now > 0 ? now : fLastNow);
                    return true;
                }
            }
            return false;
        }
        if (fDragging < 0) {
            const Hit* h = hitAt(qx, qy);
            const int hover = h && h->kind == kBtnKnob && h->code == 1 ? h->index : -1;
            if (hover == fHover) return false;
            fHover = hover;
            return true;
        }
        if (std::fabs(qy - fDownQY) > 2) fMoved = true;
        if (!fMoved) return false;
        fDragAcc += (fDragY - qy) / (fine ? 8.0f : 1.5f);   // shift: fine
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
        int steps = int(std::max(-16.0f, std::min(16.0f, delta * 4)));
        if (steps == 0) steps = delta > 0 ? 1 : -1;
        if (fine) steps = steps > 0 ? 1 : -1;               // shift: one step
        turnKnob(h->index, steps);
        return true;
    }

    // the queued key sequences (select a track, add a trig, mute, pattern); true if something was sent
    bool tick(double now)
    {
        fLastNow = now;
        bool sent = false;
        while (!fQueue.empty() && fQueue.front().at <= now) {
            const Ev e = fQueue.front();
            fQueue.erase(fQueue.begin());
            send(e.kind, e.code, e.arg);
            sent = true;
        }
        // what the queue changed (REC on, the step page) is kept a little after it played,
        // the time for the LEDs to show it; the page LEDs are sampled afresh
        if (sent && fQueue.empty()) { fPendUntil = now + 300; fHistN = 0; }
        if (fQueue.empty() && now >= fPendUntil) { fPendPage = -1; fPendRec = false; }
        return sent;
    }

    bool busy() const { return !fQueue.empty(); }

private:
    static constexpr float kW = 1000;
    static constexpr double kGapMs = 70;
    // the right column: the screen (3x) and what is under it
    static constexpr float kRX = 604, kSY = 70, kSW = 384, kSH = 192;

    enum Kind {
        kBtnTrack, kBtnStep, kBtnCell, kBtnMute, kBtnViewPage, kBtnKey, kBtnFunc, kBtnKnob,
        kBtnFollow, kBtnListToggle, kBtnBank, kBtnPattern, kBtnPrevNext, kBtnLoop, kBtnTab, kBtnFuncKey
    };
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

    void queueTap(double& t, int code) { queue(t, 0, code); queue(t, 1, code); }

    double queueStart(double now) const { return fQueue.empty() ? now : std::max(now, fQueue.back().at + kGapMs); }

    void queueSelect(double& at, int t)
    {
        switch (fSpec.select) {
        case SeqSpec::kTrkTrig:
            queue(at, 0, fSpec.trk); queueTap(at, fSpec.trig[t]); queue(at, 1, fSpec.trk);
            break;
        case SeqSpec::kTrkPad:
            queue(at, 0, fSpec.trk); queue(at, 4, fSpec.padBase + t, 1); queue(at, 4, fSpec.padBase + t, 0); queue(at, 1, fSpec.trk);
            break;
        case SeqSpec::kPad:                                         // a soft pad is not taken (Model:Cycles)
            queue(at, 4, fSpec.padBase + t, 100); queue(at, 4, fSpec.padBase + t, 0);
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
        if (fSpec.rec < 0 || fPendRec) return;
        if (level(fSpec.rec) < kLit) queueTap(at, fSpec.rec);
        fPendRec = true;                             // on once the queue has played
    }

    void toggleStep(int i, double now)
    {
        if (i < 0 || i >= 16) return;
        double at = queueStart(now);
        queueGridRec(at);
        queueTap(at, fSpec.trig[i]);
    }

    // the page LEDs over the last half second: the page that plays blinks, the page
    // the OS shows stays lit
    void sampleLeds()
    {
        const int n = std::max(0, std::min(8, fSpec.nPageLeds));
        for (int i = 0; i < n; ++i) fLedHist[i][fHistI] = pageLed(i);
        fHistI = (fHistI + 1) % kHist;
        if (fHistN < kHist) ++fHistN;
    }

    // the step page the OS shows: the page LED that stands out from the others (brighter
    // on the Digitakt, dimmer on the Syntakt), over the last half second
    int osPage() const
    {
        if (fPendPage >= 0) return fPendPage;
        const int n = std::min(8, fSpec.nPageLeds);
        float m[8] = {};
        int lit = 0;
        for (int i = 0; i < n; ++i) {
            if (fHistN == 0) m[i] = pageLed(i);
            else { for (int k = 0; k < fHistN; ++k) m[i] += fLedHist[i][k]; m[i] /= fHistN; }
            if (m[i] > 0.05f) ++lit;
        }
        if (lit < 2) {                                              // one page: the lit one
            for (int i = 0; i < n; ++i) if (m[i] > 0.05f) return i;
            return 0;
        }
        int best = 0;
        float far = -1;
        for (int i = 0; i < n; ++i) {
            if (m[i] <= 0.05f) continue;
            float others = 0;
            for (int j = 0; j < n; ++j) if (j != i && m[j] > 0.05f) others += m[j];
            const float d = std::fabs(m[i] - others / (lit - 1));
            if (d > far + 0.02f) { far = d; best = i; }
        }
        return best;
    }

    int osPages() const
    {
        int n = 0;
        for (int i = 0; i < std::min(8, fSpec.nPageLeds); ++i) {
            float m = pageLed(i);
            for (int k = 0; k < fHistN; ++k) m = std::max(m, fLedHist[i][k]);
            if (m > 0.05f) n = i + 1;
        }
        return std::max(1, n);
    }

    // PAGE until the OS shows page p (PAGE goes round the pages of the pattern)
    void queuePage(double& at, int p)
    {
        const int n = osPages(), cur = osPage();
        if (fSpec.page < 0 || p >= n) return;
        const int k = ((p - cur) % n + n) % n;
        for (int i = 0; i < k; ++i) queueTap(at, fSpec.page);
        fPendPage = p;
    }

    float gridRowH() const { return std::max(1, std::min(16, fSpec.tracks)) <= 8 ? 50.0f : 31.0f; }

    // what the view needs in height: the grid on the left, the screen, keys and
    // pages (and knobs) on the right
    float contentHeight() const
    {
        const int n = std::max(1, std::min(16, fSpec.tracks));
        const float leftH = 102 + n * gridRowH() + 40;
        const float base = kSY + kSH + 14 + 36 + 22;                 // under the keys
        const int nPages = std::max(fSpec.nPageKeys, 0), perRow = nPages > 6 ? 4 : 3;
        float pages = base + 12 + ((nPages + perRow - 1) / perRow) * 36 + 8;
        const int nk = (fSpec.screenKnobs ? 0 : std::min(12, fSpec.nEnc)) + (fSpec.data >= 0 ? 1 : 0);
        if (nk > 0) {
            const int kRow = nk > 8 ? (nk + 1) / 2 : nk;
            pages += ((nk + kRow - 1) / kRow) * 66 + 10;
        }
        const float funcs = fSpec.nFuncs > 0 ? base + 16 + ((fSpec.nFuncs + 3) / 4) * 36 + 10 : 0;
        return std::max(440.0f, std::ceil(std::max(leftH, std::max(pages, funcs))));
    }

    void setLoop(int p)
    {
        fLoop = p;
        if (p >= 0) fViewPage = p;
        if (fHost.setLoop) fHost.setLoop(p);
    }

    void goToPage(int p, double now)
    {
        fViewPage = p;
        double at = queueStart(now);
        queuePage(at, p);
    }

    void toggleCell(int t, int s, double now)
    {
        if (t < 0 || t >= fSpec.tracks || s < 0 || s >= 128) return;
        double at = queueStart(now);
        if (t != fSel) { queueSelect(at, t); fSel = t; }
        queuePage(at, s / 16);
        queueGridRec(at);
        queueTap(at, fSpec.trig[s % 16]);
    }

    void toggleMute(int t, double now)
    {
        if (t < 0 || t >= fSpec.tracks || fSpec.mute == SeqSpec::kMuteNone) return;
        double at = queueStart(now);
        switch (fSpec.mute) {
        case SeqSpec::kModeTrig:
            for (int pass = 0; pass < 2; ++pass) {                  // in, the trig, out
                queue(at, 0, fSpec.func); queueTap(at, fSpec.muteKey); queue(at, 1, fSpec.func);
                if (pass == 0) queueTap(at, fSpec.trig[t]);
            }
            break;
        case SeqSpec::kModePad:
            queueTap(at, fSpec.muteKey);
            queue(at, 4, fSpec.padBase + t, 100); queue(at, 4, fSpec.padBase + t, 0);
            queueTap(at, fSpec.muteKey);
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

    // pattern p (0 = A01, 16 = B01...) with the keys of the machine
    void selectPattern(int p, double now)
    {
        if (!fGrid.valid || fSpec.ptnSel == SeqSpec::kPtnNone) return;
        const int bank = p / 16, slot = p % 16, cur = fGrid.pattern / 16;
        if (bank < 0 || bank >= fSpec.nBanks) return;
        double at = queueStart(now);
        switch (fSpec.ptnSel) {
        case SeqSpec::kPtnArrows: {
            queue(at, 0, fSpec.ptnKey);
            const int d = bank - cur;
            for (int i = 0; i < std::abs(d); ++i) queueTap(at, d > 0 ? fSpec.right : fSpec.left);
            queueTap(at, fSpec.trig[slot]);
            queue(at, 1, fSpec.ptnKey);
            break;
        }
        case SeqSpec::kBankKey:
            queue(at, 0, fSpec.bankKeys[bank]);
            queueTap(at, fSpec.trig[slot]);
            queue(at, 1, fSpec.bankKeys[bank]);
            break;
        case SeqSpec::kPtnPad:
            queue(at, 0, fSpec.ptnKey);
            queue(at, 4, fSpec.padBase + bank, 100); queue(at, 4, fSpec.padBase + bank, 0);
            queueTap(at, fSpec.trig[slot]);
            queue(at, 1, fSpec.ptnKey);
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

    // the last drawn wins (the list is drawn last, on top)
    const Hit* hitAt(float qx, float qy) const
    {
        for (auto it = fHits.rbegin(); it != fHits.rend(); ++it)
            if (qx >= it->x && qx <= it->x + it->w && qy >= it->y && qy <= it->y + it->h) return &*it;
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

    static void patternName(char* out, size_t n, int p)
    {
        std::snprintf(out, n, "%c%02d", 'A' + p / 16, p % 16 + 1);
    }

    void drawHeader(NanoVG& vg)
    {
        label(vg, 24, 28, fSpec.title, 20, 0xECEAE4, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        label(vg, 24, 48, "Elektron Emu", 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);

        if (grid() && fSpec.ptnSel != SeqSpec::kPtnNone) {           // < A01 >
            char p[8];
            patternName(p, sizeof p, fGrid.pattern);
            button(vg, 250, 16, 34, 36, "<", 0);
            addHit(kBtnPrevNext, -1, 0, 250, 16, 34, 36);
            vg.beginPath();
            vg.roundedRect(288, 16, 104, 36, 7);
            vg.fillColor(rgb(fListOpen ? 0x3A3D44 : 0x2C2F35));
            vg.fill();
            label(vg, 300, 34, p, 20, 0xF2A93B, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            label(vg, 380, 35, fListOpen ? "^" : "v", 12, 0xA3A6AD, NanoVG::ALIGN_RIGHT | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnListToggle, 0, 0, 288, 16, 104, 36);
            button(vg, 396, 16, 34, 36, ">", 0);
            addHit(kBtnPrevNext, 1, 0, 396, 16, 34, 36);
        } else if (grid()) {
            char p[8];
            patternName(p, sizeof p, fGrid.pattern);
            label(vg, 288, 34, p, 20, 0xF2A93B, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        }
        if (grid()) {
            button(vg, 446, 18, 100, 32, fFollow ? "FOLLOW: ON" : "FOLLOW: OFF", fFollow ? 0.6f : 0.0f, false, 11);
            addHit(kBtnFollow, 0, 0, 446, 18, 100, 32);
        }

        float x = 760;
        const struct { const char* t; int code; } tr[] = {{"REC", fSpec.rec}, {"PLAY", fSpec.play}, {"STOP", fSpec.stop}};
        for (const auto& b : tr) {
            if (b.code < 0) continue;
            button(vg, x, 16, 64, 36, b.t, level(b.code));
            addHit(kBtnKey, 0, b.code, x, 16, 64, 36);
            x += 72;
        }
    }

    // the bank list: the banks on a row, the 16 slots of the bank below
    void drawPatternList(NanoVG& vg)
    {
        if (!fListOpen || !grid()) return;
        const float x = 250, y = 58, w = 340, h = 232;
        vg.beginPath();
        vg.roundedRect(x, y, w, h, 9);
        vg.fillColor(rgb(0x1E2024, 0.98f));
        vg.fill();
        vg.strokeColor(rgb(0x3A3D44));
        vg.strokeWidth(1);
        vg.stroke();
        addHit(kBtnBank, -1, 0, x, y, w, h);                         // the background keeps the list open

        label(vg, x + 12, y + 16, "BANK", 10, 0x8E9198, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        const int nb = std::max(1, std::min(8, fSpec.nBanks));
        const float bw = (w - 24 - (nb - 1) * 4) / nb;
        for (int b = 0; b < nb; ++b) {
            const float bx = x + 12 + b * (bw + 4);
            const char t[2] = {char('A' + b), 0};
            const bool cur = b == fListBank, playing = b == fGrid.pattern / 16;
            vg.beginPath();
            vg.roundedRect(bx, y + 26, bw, 28, 5);
            vg.fillColor(rgb(cur ? 0x3A3D44 : 0x26282D));
            vg.fill();
            if (playing) {
                vg.beginPath();
                vg.rect(bx + 6, y + 51, bw - 12, 2);
                vg.fillColor(rgb(0xF2A93B));
                vg.fill();
            }
            label(vg, bx + bw / 2, y + 41, t, 13, cur ? 0xECEAE4 : 0xA3A6AD, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnBank, b, 0, bx, y + 26, bw, 28);
        }

        label(vg, x + 12, y + 72, "PATTERN", 10, 0x8E9198, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        const float sw = (w - 24 - 3 * 6) / 4, sh = 32;
        for (int s = 0; s < 16; ++s) {
            const int p = fListBank * 16 + s;
            const float sx = x + 12 + (s % 4) * (sw + 6), sy = y + 82 + (s / 4) * (sh + 4);
            char t[8];
            patternName(t, sizeof t, p);
            const bool cur = p == fGrid.pattern;
            vg.beginPath();
            vg.roundedRect(sx, sy, sw, sh, 5);
            vg.fillColor(rgb(cur ? 0xF2A93B : 0x2C2F35));
            vg.fill();
            label(vg, sx + sw / 2, sy + sh / 2 + 1, t, 13, cur ? 0x16171A : 0xECEAE4, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnPattern, p, 0, sx, sy, sw, sh);
        }
    }

    // without the pattern: the track list
    void drawTracks(NanoVG& vg)
    {
        label(vg, 24, 84, "TRACKS", 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        const int n = std::max(1, std::min(16, fSpec.tracks));
        const float top = 98, rowH = std::min(34.0f, (600 - top) / n);
        for (int t = 0; t < n; ++t) {
            const float y = top + t * rowH;
            const bool sel = t == fSel;
            vg.beginPath();
            vg.roundedRect(18, y + 1, 180, rowH - 2, 6);
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
            label(vg, 60, y + rowH / 2, fSpec.trackNames[t] ? fSpec.trackNames[t] : "", 13, sel ? 0xECEAE4 : 0xC9CBD0,
                  NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnTrack, t, 0, 18, y + 1, 180, rowH - 2);
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

    // without the pattern: the steps of the selected track from the trig LEDs
    void drawSteps(NanoVG& vg)
    {
        char t[48];
        std::snprintf(t, sizeof t, "STEPS - TRACK %d", fSel + 1);
        label(vg, 218, 84, t, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        const float cw = 40, gap = 5, ch = 46;
        for (int i = 0; i < 16; ++i) {
            const float x = 218 + (i % 8) * (cw + gap), y = 102 + (i / 8) * (ch + 8);
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
        label(vg, 218, 224, "Click a step to add or remove a trig (the trig LEDs of the machine).", 11, 0x7C8088,
              NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        label(vg, 218, 242, "The pattern of every track is not read on this OS yet.", 11, 0x7C8088,
              NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    // every track of the pattern, from the memory
    void drawGrid(NanoVG& vg)
    {
        const int n = std::max(1, std::min(16, fSpec.tracks));
        const float left = 18, cells = 184, right = 584, top = 102;
        const float rowH = gridRowH();
        const int nTabs = std::max(1, std::min(8, std::max(4, fSpec.nPageLeds)));
        if (fViewPage < 0) fViewPage = osPage();
        const int page = std::max(0, std::min(nTabs - 1, fViewPage));

        // what plays: every page, or the selected page in a loop (then a page tab picks it)
        {
            char t[32];
            if (fLoop >= 0) std::snprintf(t, sizeof t, "LOOP PAGE %d", fLoop + 1);
            else std::snprintf(t, sizeof t, "PLAY ALL PAGES");
            button(vg, left, 70, cells - left - 8, 22, t, fLoop >= 0 ? 1.0f : 0.0f, false, 11);
            addHit(kBtnLoop, 0, 0, left, 70, cells - left - 8, 22);
        }
        const int have = osPages();
        const float tw = (right - cells - (nTabs - 1) * 4) / nTabs;
        for (int p = 0; p < nTabs; ++p) {
            const float x = cells + p * (tw + 4);
            char s[16];
            if (nTabs <= 4) std::snprintf(s, sizeof s, "%d-%d", p * 16 + 1, p * 16 + 16);
            else std::snprintf(s, sizeof s, "%d", p * 16 + 1);
            const bool cur = p == page;
            vg.beginPath();
            vg.roundedRect(x, 70, tw, 22, 5);
            vg.fillColor(rgb(cur ? 0x3A3D44 : 0x222429));
            vg.fill();
            if (p == fLoop) {                                         // the page that loops
                vg.strokeColor(rgb(0xF2A93B));
                vg.strokeWidth(1.5f);
                vg.stroke();
            }
            label(vg, x + tw / 2, 81, s, 11, cur ? 0xECEAE4 : (p < have ? 0xA3A6AD : 0x5A5D63), NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnViewPage, p, 0, x, 70, tw, 22);
        }

        const float cw = (right - cells - 15 * 3) / 16;
        for (int t = 0; t < n; ++t) {
            const float y = top + t * rowH;
            const bool sel = t == fSel;
            const bool muted = (fGrid.mutes >> t) & 1;
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
                const bool on = trigOn(t, s);
                vg.beginPath();
                vg.roundedRect(x, y + 1.5f, cw, rowH - 3, 3);
                if (on) vg.fillColor(rgb(sel ? 0xF2A93B : 0xC98B34, muted ? 0.35f : 1.0f));
                else vg.fillColor(rgb(i % 4 == 0 ? 0x2A2C31 : 0x222429));
                vg.fill();
                addHit(kBtnCell, t, s, x, y + 1.5f, cw, rowH - 3);
            }
        }

        // the play position: a light dot under the grid (the selected track)
        {
            const int ps = playPos(fSel);
            const float dy = top + n * rowH + 7;
            for (int i = 0; i < 16; ++i) {
                const float cx = cells + i * (cw + 3) + cw / 2;
                const bool here = ps >= 0 && ps / 16 == page && ps % 16 == i;
                vg.beginPath();
                vg.circle(cx, dy, here ? 3.5f : 1.5f);
                vg.fillColor(rgb(here ? 0x8CF0A8 : 0x2E3136));
                vg.fill();
            }
        }
        label(vg, left, top + n * rowH + 28, "Click or drag along a row to add or remove trigs. M mutes. LOOP PAGE plays the selected page only.", 11, 0x7C8088,
              NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    // the screen, 3x; its parameters (A-D top row, E-H bottom row) are turned by dragging
    void drawScreen(NanoVG& vg)
    {
        const float x = kRX, y = kSY, w = kSW, h = kSH;
        vg.beginPath();
        vg.roundedRect(x - 8, y - 8, w + 16, h + 16, 10);
        vg.fillColor(rgb(0x050506));
        vg.fill();
        const uint8_t* s = fHost.screen ? fHost.screen() : nullptr;
        if (s) {
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
        if (!fSpec.screenKnobs) return;

        // the 8 parameter zones
        const float sx = w / 128, sy = h / 64;
        const float x0 = x + fSpec.knobX0 * sx, zw = (x + w - x0) / 4;
        const float rows[3] = {y + fSpec.knobY0 * sy, y + fSpec.knobYMid * sy, y + fSpec.knobY1 * sy};
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 4; ++c) {
                const int i = r * 4 + c;
                if (i >= fSpec.nEnc) continue;
                const float zx = x0 + c * zw, zy = rows[r], zh = rows[r + 1] - rows[r];
                if (i == fHover || (i == fDragging && fMoved)) {
                    vg.beginPath();
                    vg.roundedRect(zx + 1, zy + 1, zw - 2, zh - 2, 5);
                    vg.fillColor(rgb(0xF2A93B, i == fDragging ? 0.16f : 0.08f));
                    vg.fill();
                    vg.strokeColor(rgb(0xF2A93B, 0.8f));
                    vg.strokeWidth(1.5f);
                    vg.stroke();
                    vg.beginPath();
                    vg.roundedRect(zx + 4, zy + 4, 16, 16, 3);
                    vg.fillColor(rgb(0xF2A93B));
                    vg.fill();
                    label(vg, zx + 12, zy + 12.5f, fSpec.encNames[i] ? fSpec.encNames[i] : "", 11, 0x16171A,
                          NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
                }
                addHit(kBtnKnob, i, 1, zx, zy, zw, zh);
            }

        // the level: the Level/Data encoder (drag or wheel)
        const int* lz = fSpec.levelZone;
        if (fSpec.data >= 0 && lz[2] > lz[0]) {
            const int i = std::min(12, fSpec.nEnc);
            const float zx = x + lz[0] * sx, zy = y + lz[1] * sy, zw = (lz[2] - lz[0]) * sx, zh = (lz[3] - lz[1]) * sy;
            if (i == fHover || (i == fDragging && fMoved)) {
                vg.beginPath();
                vg.roundedRect(zx + 1, zy + 1, zw - 2, zh - 2, 5);
                vg.fillColor(rgb(0x6FB7FF, i == fDragging ? 0.18f : 0.10f));
                vg.fill();
                vg.strokeColor(rgb(0x6FB7FF, 0.9f));
                vg.strokeWidth(1.5f);
                vg.stroke();
            }
            addHit(kBtnKnob, i, 1, zx, zy, zw, zh);
        }
    }

    void drawNav(NanoVG& vg)
    {
        const float y = kSY + kSH + 14, h = 36;
        if (fSpec.func >= 0) {
            button(vg, kRX, y, 70, h, "FUNC", fFunc ? 1.0f : 0.0f);
            addHit(kBtnFunc, 0, fSpec.func, kRX, y, 70, h);
        }
        const struct { const char* t; int code; float x, w; } k[] = {
            {"NO", fSpec.no, kRX + 78, 52}, {"YES", fSpec.yes, kRX + 136, 52},
            {"<", fSpec.left, kRX + 200, 42}, {"^", fSpec.up, kRX + 246, 42}, {"v", fSpec.down, kRX + 292, 42}, {">", fSpec.right, kRX + 338, 42}};
        for (const auto& b : k) {
            if (b.code < 0) continue;
            button(vg, b.x, y, b.w, h, b.t, 0);
            addHit(kBtnKey, 0, b.code, b.x, y, b.w, h);
        }
    }

    // the parameter pages (menus) and the knobs that are not on the screen
    void drawParams(NanoVG& vg)
    {
        const float left = kRX, width = kSW;
        float top = kSY + kSH + 14 + 36 + 22;
        // two tabs: the parameter pages (and knobs), or the FUNC functions as buttons
        if (fSpec.nFuncs > 0 && fSpec.func >= 0) {
            const char* names[2] = {fSpec.pagesTitle, "FUNC MENUS"};
            for (int i = 0; i < 2; ++i) {
                const float tx = left + i * 150, tw = 144;
                const bool cur = (i == 1) == fFuncTab;
                label(vg, tx + 2, top, names[i], 11, cur ? 0xECEAE4 : 0x7C8088, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
                if (cur) {
                    vg.beginPath();
                    vg.rect(tx + 2, top + 8, 60, 2);
                    vg.fillColor(rgb(0xF2A93B));
                    vg.fill();
                }
                addHit(kBtnTab, i, 0, tx, top - 10, tw, 22);
            }
            if (fFuncTab) {
                top += 16;
                const int n = fSpec.nFuncs, perRow = 4;
                const float gap = 6, bw = (width - gap * (perRow - 1)) / perRow;
                const int rows = (n + perRow - 1) / perRow;
                const float bh = std::min(30.0f, (fHgt - 12 - top) / rows - gap);
                for (int i = 0; i < n; ++i) {
                    const float x = left + (i % perRow) * (bw + gap), y = top + (i / perRow) * (bh + gap);
                    button(vg, x, y, bw, bh, fSpec.funcNames[i] ? fSpec.funcNames[i] : "", 0, false, 11);
                    addHit(kBtnFuncKey, i, fSpec.funcKeys[i], x, y, bw, bh);
                }
                return;
            }
            top += 12;
        } else if (fSpec.nPageKeys > 0) {
            label(vg, left, top, fSpec.pagesTitle, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            top += 12;
        }
        if (fSpec.nPageKeys > 0) {
            const int n = fSpec.nPageKeys, perRow = n > 6 ? 4 : 3;
            const float gap = 6, bw = (width - gap * (perRow - 1)) / perRow, bh = 30;
            for (int i = 0; i < n; ++i) {
                const float x = left + (i % perRow) * (bw + gap), y = top + (i / perRow) * (bh + gap);
                button(vg, x, y, bw, bh, fSpec.pageNames[i] ? fSpec.pageNames[i] : "", level(fSpec.pageKeys[i]), false, 12);
                addHit(kBtnKey, 0, fSpec.pageKeys[i], x, y, bw, bh);
            }
            top += ((n + perRow - 1) / perRow) * (bh + gap) + 8;
        }

        // the knobs: all of them when the screen does not show the parameters, otherwise DATA
        std::vector<int> knobs;
        if (!fSpec.screenKnobs)
            for (int i = 0; i < std::min(12, fSpec.nEnc); ++i) knobs.push_back(i);
        if (fSpec.data >= 0) knobs.push_back(std::min(12, fSpec.nEnc));
        if (knobs.empty()) return;
        const int nk = int(knobs.size());
        const int perRow = nk > 8 ? (nk + 1) / 2 : nk;
        const int rows = (nk + perRow - 1) / perRow;
        const float slot = width / perRow;
        const float rowH = std::min(80.0f, (fHgt - 24 - top) / rows);
        const float r = std::max(10.0f, std::min(20.0f, std::min(slot, rowH) * 0.27f));
        for (int k = 0; k < nk; ++k) {
            const int i = knobs[k];
            const float cx = left + (k % perRow) * slot + slot / 2;
            const float cy = top + (k / perRow) * rowH + r + 6;
            const bool isData = i >= std::min(12, fSpec.nEnc);
            const char* name = isData ? "DATA" : (fSpec.encNames[i] ? fSpec.encNames[i] : "");
            const bool active = fDragging == i;
            vg.beginPath();
            vg.circle(cx, cy, r);
            vg.fillColor(rgb(0x26282D));
            vg.fill();
            vg.beginPath();
            vg.arc(cx, cy, r + 4, 0.75f * float(M_PI), 2.25f * float(M_PI), NanoVG::CW);
            vg.strokeColor(rgb(active ? 0xF2A93B : 0x33363C));
            vg.strokeWidth(3);
            vg.lineCap(NanoVG::ROUND);
            vg.stroke();
            const float a = (i < 13 ? fAngle[i] : 0) - float(M_PI) / 2;
            vg.beginPath();
            vg.moveTo(cx + std::cos(a) * r * 0.35f, cy + std::sin(a) * r * 0.35f);
            vg.lineTo(cx + std::cos(a) * r * 0.85f, cy + std::sin(a) * r * 0.85f);
            vg.strokeColor(rgb(isData ? 0x6FB7FF : 0xF2A93B));
            vg.strokeWidth(2.5f);
            vg.stroke();
            label(vg, cx, cy + r + 13, name, 10.5f, 0xC9CBD0, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnKnob, i, 0, cx - slot / 2 + 2, cy - r - 6, slot - 4, 2 * r + 26);
        }
    }

    // ------------------------------------------------------------ state
    SeqSpec fSpec;
    SeqHost fHost;
    SeqGrid fGrid;
    float fScale = 1, fOx = 0, fOy = 0;
    std::vector<Hit> fHits;
    Hit fDown;
    bool fDownActive = false, fFunc = false, fFollow = true, fListOpen = false;
    int fStill = 1000, fLoop = -1;
    bool fPlayLedSeen = false;
    double fPendUntil = 0;
    // a key LED is lit above this (the Digitakt II shows REC on at 0.14 of its range)
    static constexpr float kLit = 0.05f;
    float fHgt = 640;
    bool fFuncTab = false, fMoved = false;
    float fDownQY = 0;
    int fClickDir[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    static constexpr int kHist = 12;
    float fLedHist[8][kHist] = {};
    int fHistI = 0, fHistN = 0, fPendPage = -1;
    bool fPendRec = false;
    bool fPaint = false, fPaintOn = true;
    int fPaintTrack = -1;
    std::vector<bool> fPainted;
    double fLastNow = 0;
    int fSel = 0, fDragging = -1, fHover = -1, fViewPage = -1, fListBank = 0;
    float fDragY = 0, fDragAcc = 0;
    float fAngle[13];
    std::vector<Ev> fQueue;
    NanoImage fScreenImg;
    std::vector<uint8_t> fScreenPx;
    bool fScreenDirty = true;
};

END_NAMESPACE_DISTRHO
