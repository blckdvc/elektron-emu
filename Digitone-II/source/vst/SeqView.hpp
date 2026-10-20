// SeqView.hpp -- the "sequencer" view shared by the five plug-ins (the same file
// in each vst/ folder, like dtcore/). It does not know the machine: it only
// presses the real keys and turns the real encoders of the emulated panel
// (UI::panel: 0 press, 1 release, 3 turn, 4 pad) and reads back the LEDs and the
// 128x64 screen. So the OS stays the reference: what the view shows is what the
// machine shows.
//
//   - tracks on the left: a click selects the track (TRK + trig, or TRK + pad);
//   - the 16 steps of the selected track, from the trig LEDs: a click presses
//     the trig key (grid recording is switched on first if its LED is off);
//   - the real screen in big, with FUNC, NO, YES and the arrows under it;
//   - the parameter pages (TRIG, SRC, FLTR...) and the encoders A-H: drag
//     up/down or use the wheel; the values are on the screen;
//   - REC, PLAY, STOP and PAGE with their LEDs.
//
// The whole view is drawn in a 1000 x 640 frame scaled to the window.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

struct SeqSpec {
    const char* title = "";
    int tracks = 16;                         // <= 16
    const char* trackNames[16] = {};
    int trig[16] = {};                       // trig key codes 1..16
    int nPageKeys = 0;                       // parameter pages (0: none, as on the Model:Cycles)
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
};

struct SeqHost {
    std::function<void(int kind, int code, int arg)> send;
    std::function<float(int code)> keyLevel;     // LED of a key, 0..1
    std::function<float(int i)> pageLevel;       // step page LED i, 0..1
    std::function<const uint8_t*()> screen;      // 128 x 64
};

class SeqView {
public:
    enum Result { kNone, kHandled, kToPanel };

    SeqView(const SeqSpec& spec, SeqHost host) : fSpec(spec), fHost(std::move(host))
    {
        fSel = 0;
        for (float& a : fAngle) a = 0;
    }

    void screenChanged() { fScreenDirty = true; }

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
        drawTracks(vg);
        drawSteps(vg);
        drawScreen(vg);
        drawNav(vg);
        drawParams(vg);

        vg.restore();
    }

    // the small "Sequencer" button the panel view shows in its corner
    static void drawToggle(NanoVG& vg, float x, float y, float sf)
    {
        const float w = 92 * sf, h = 24 * sf;
        vg.beginPath();
        vg.roundedRect(x, y, w, h, 6 * sf);
        vg.fillColor(Color(0x22, 0x24, 0x29, 0.92f));
        vg.fill();
        vg.strokeColor(rgb(0xF2A93B));
        vg.strokeWidth(1 * sf);
        vg.stroke();
        vg.fontFace(NANOVG_DEJAVU_SANS_TTF);
        vg.fontSize(12 * sf);
        vg.textAlign(NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
        vg.fillColor(rgb(0xECEAE4));
        vg.text(x + w / 2, y + h / 2, "Sequencer", nullptr);
    }

    static bool toggleHit(float px, float py, float x, float y, float sf)
    {
        return px >= x && px <= x + 92 * sf && py >= y && py <= y + 24 * sf;
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
            case kBtnPanel: fDownActive = false; return kToPanel;
            case kBtnTrack: selectTrack(h->index, now); break;
            case kBtnStep: toggleStep(h->index, now); break;
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

    // the queued key sequences (select a track, add a trig); true if something was sent
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

    enum Kind { kBtnPanel, kBtnTrack, kBtnStep, kBtnKey, kBtnFunc, kBtnKnob };
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

    void queue(double& t, int kind, int code, int arg = 0)
    {
        if (code < 0) return;
        fQueue.push_back({t, kind, code, arg});
        t += kGapMs;
    }

    double queueStart(double now) const { return fQueue.empty() ? now : std::max(now, fQueue.back().at + kGapMs); }

    void selectTrack(int t, double now)
    {
        if (t < 0 || t >= fSpec.tracks) return;
        fSel = t;
        double at = queueStart(now);
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

    void toggleStep(int i, double now)
    {
        if (i < 0 || i >= 16) return;
        double at = queueStart(now);
        if (fSpec.rec >= 0 && level(fSpec.rec) < 0.2f) {             // grid recording first
            queue(at, 0, fSpec.rec); queue(at, 1, fSpec.rec);
        }
        queue(at, 0, fSpec.trig[i]); queue(at, 1, fSpec.trig[i]);
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

    void button(NanoVG& vg, float x, float y, float w, float h, const char* text, float lit, bool accent = false)
    {
        vg.beginPath();
        vg.roundedRect(x, y, w, h, 7);
        if (accent || lit > 0.05f) vg.fillColor(Color(rgb(0x2C2F35), rgb(0xF2A93B), std::max(accent ? 1.0f : 0.0f, std::min(1.0f, lit))));
        else vg.fillColor(rgb(0x2C2F35));
        vg.fill();
        const bool dark = accent || lit > 0.5f;
        label(vg, x + w / 2, y + h / 2 + 1, text, 13, dark ? 0x16171A : 0xECEAE4, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
    }

    void drawHeader(NanoVG& vg)
    {
        label(vg, 24, 30, fSpec.title, 20, 0xECEAE4, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        label(vg, 24, 50, "Sequencer view - drives the machine's own keys", 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);

        float x = 640;
        const struct { const char* t; int code; } tr[] = {{"REC", fSpec.rec}, {"PLAY", fSpec.play}, {"STOP", fSpec.stop}};
        for (const auto& b : tr) {
            if (b.code < 0) continue;
            button(vg, x, 18, 64, 36, b.t, level(b.code));
            addHit(kBtnKey, 0, b.code, x, 18, 64, 36);
            x += 72;
        }
        vg.beginPath();
        vg.roundedRect(886, 18, 90, 36, 7);
        vg.strokeColor(rgb(0x4A4E56));
        vg.strokeWidth(1);
        vg.stroke();
        label(vg, 931, 37, "Panel", 13, 0xC9CBD0, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
        addHit(kBtnPanel, 0, 0, 886, 18, 90, 36);
    }

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

    void drawSteps(NanoVG& vg)
    {
        char t[48];
        std::snprintf(t, sizeof t, "STEPS - TRACK %d", fSel + 1);
        label(vg, 232, 84, t, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);

        // step page LEDs and PAGE
        const int np = std::max(0, std::min(8, fSpec.nPageLeds));
        float px = 500 - np * 14;
        for (int i = 0; i < np; ++i) {
            const float lv = fHost.pageLevel ? std::max(0.0f, std::min(1.0f, fHost.pageLevel(i))) : 0.0f;
            vg.beginPath();
            vg.roundedRect(px + i * 14, 78, 10, 10, 2);
            vg.fillColor(Color(rgb(0x2A2C31), rgb(0xF2A93B), lv));
            vg.fill();
        }
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
        if (fSpec.func >= 0)
            label(vg, 632, 316, "FUNC stays down for the next key.", 11, 0x7C8088, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    void drawParams(NanoVG& vg)
    {
        const float left = 232, right = 976, width = right - left;
        float top = 350;
        if (fSpec.nPageKeys > 0) {
            label(vg, left, top - 10, fSpec.pagesTitle, 11, 0x9A9DA3, NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
            const int n = fSpec.nPageKeys;
            const float gap = 8, bw = std::min(110.0f, (width - gap * (n - 1)) / n);
            for (int i = 0; i < n; ++i) {
                const float x = left + i * (bw + gap);
                button(vg, x, top, bw, 34, fSpec.pageNames[i] ? fSpec.pageNames[i] : "", level(fSpec.pageKeys[i]));
                addHit(kBtnKey, 0, fSpec.pageKeys[i], x, top, bw, 34);
            }
            top += 54;
        } else {
            top += 0;
        }

        const int nk = std::min(12, fSpec.nEnc) + (fSpec.data >= 0 ? 1 : 0);
        if (nk <= 0) return;
        const int perRow = nk <= 9 ? nk : (nk + 1) / 2;
        const int rows = (nk + perRow - 1) / perRow;
        const float slot = width / perRow;
        const float rowH = std::min(140.0f, (kH - 16 - top) / rows);
        const float r = std::min(30.0f, std::min(slot, rowH) * 0.28f);
        for (int i = 0; i < nk; ++i) {
            const float cx = left + (i % perRow) * slot + slot / 2;
            const float cy = top + (i / perRow) * rowH + r + 12;
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
            label(vg, cx, cy + r + 18, name, 12, 0xC9CBD0, NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            addHit(kBtnKnob, i, 0, cx - slot / 2 + 4, cy - r - 8, slot - 8, 2 * r + 36);
        }
        label(vg, left, kH - 12, "Drag up/down or use the wheel on a knob (shift: one step). The values are on the screen.", 11, 0x7C8088,
              NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
    }

    // ------------------------------------------------------------ state
    SeqSpec fSpec;
    SeqHost fHost;
    float fScale = 1, fOx = 0, fOy = 0;
    std::vector<Hit> fHits;
    Hit fDown;
    bool fDownActive = false, fFunc = false;
    int fSel = 0, fDragging = -1;
    float fDragY = 0, fDragAcc = 0;
    float fAngle[13];
    std::vector<Ev> fQueue;
    NanoImage fScreenImg;
    std::vector<uint8_t> fScreenPx;
    bool fScreenDirty = true;
};

END_NAMESPACE_DISTRHO
