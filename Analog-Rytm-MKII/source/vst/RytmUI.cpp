// RytmUI.cpp -- la faccia dell'Analog Rytm MKII nel VST3: la stessa dell'AU
// (au/Extension/PanelView.swift, a sua volta la GUI v2 emu/guiv2_rytm.py sopra
// emu/guiv2.py), disegnata con NanoVG. Layout dal JSON condiviso
// (vst/scripts/gen-layout.py), foto del pannello dalla cartella dell'utente o dal
// bundle (se manca, pannello schematico), schermo OLED nella cornice nera (clic:
// grande sopra i pad, secondo clic: piccolo), bagliori dei LED nei colori del
// firmware, pad RGB con velocity dall'altezza del clic, LED di pagina, Main
// Volume, FUNC/TRK agganciati per una combinazione, encoder con trascinamento e
// rotella, +DRIVE nella linguetta a destra. Modellata sulla vista del VST3 del
// Digitakt II (digitakt/emulator/vst/DigitaktUI.cpp).
//
// Senza firmware (o con il primo avvio in corso, o con un errore) sopra il
// pannello c'e' una scheda con il percorso della cartella Firmware, "Apri
// cartella" e "Choose firmware...".
//
// Misura: il pannello scala con proporzioni fisse ed e' centrato. La finestra si
// puo' trascinare dall'angolo (se l'host lo permette) e i tasti 75-150% nella
// riga di stato chiedono all'host una nuova misura (setSize -> in VST3
// IPlugFrame::resizeView).
#include "DistrhoUI.hpp"
#include "DistrhoPluginUtils.hpp"
#include "RytmPlugin.hpp"
#include "RytmPaths.hpp"
#include "DriveModel.hpp"
#include "panel_layout.hpp"
#include "SeqView.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#include <commdlg.h>
#include <ole2.h>
#include <shlobj.h>
#endif

START_NAMESPACE_DISTRHO

namespace {

using Clock = std::chrono::steady_clock;

double nowMs()
{
    return std::chrono::duration<double, std::milli>(Clock::now().time_since_epoch()).count();
}

const float kZooms[] = {0.75f, 1.0f, 1.25f, 1.5f};
const int kNZooms = 4;

// emu/guiv2.py _box_matrices: pesi della media d'area da n a out
std::vector<std::vector<std::pair<int, float>>> boxWeights(int n, int out)
{
    std::vector<std::vector<std::pair<int, float>>> w(out);
    const double k = double(n) / out;
    for (int i = 0; i < out; ++i) {
        const double lo = i * k, hi = (i + 1) * k;
        float s = 0;
        for (int j = int(lo); j < std::min(n, int(std::ceil(hi))); ++j) {
            const float f = float(std::min(hi, double(j + 1)) - std::max(lo, double(j)));
            w[i].push_back({j, f});
            s += f;
        }
        for (auto& p : w[i]) p.second /= s;
    }
    return w;
}

// file dello zoom scelto (vale per le prossime aperture, come le preferenze dell'AU)
std::string zoomFile()
{
    const std::string d = rytm::cacheDir();
    return d.empty() ? d : rytm::join(d, "vst-zoom.txt");
}

float loadZoom()
{
    float z = 0.75f;
    const std::string p = zoomFile();
    if (FILE* f = p.empty() ? nullptr : std::fopen(p.c_str(), "r")) {
        if (std::fscanf(f, "%f", &z) != 1) z = 0.75f;
        std::fclose(f);
    }
    return std::max(0.4f, std::min(2.0f, z));
}

void saveZoom(float z)
{
    const std::string p = zoomFile();
    if (p.empty()) return;
    rytm::makeDirs(rytm::cacheDir());
    if (FILE* f = std::fopen(p.c_str(), "w")) {
        std::fprintf(f, "%.3f\n", z);
        std::fclose(f);
    }
}

bool isLatching(int code)
{
    for (int c : arl::kLatching) if (c == code) return true;
    return false;
}

// emu/rytm_leds.py colour: VV -> (rgb, livello 0..1)
struct LedLook { arl::RGB c; double level; };

LedLook ledColour(uint8_t v)
{
    arl::RGB c = {235, 235, 225};
    bool found = false;
    for (const auto& k : arl::kColours) if (k.id == (v >> 4)) { c = k.c; found = true; break; }
    if (!found)
        for (const auto& k : arl::kColours) if (k.id == arl::kColourDefault) { c = k.c; break; }
    return {c, double(v & 0x0F) / 15.0};
}

// emu/rytm_leds.py pad_rgb: (r, g, b) a 4 bit -> (rgb pieno, livello 0..1)
LedLook padRGB(uint8_t r, uint8_t g, uint8_t b)
{
    const int top = std::max(r, std::max(g, b));
    if (top == 0) return {{0, 0, 0}, 0.0};
    auto c = [&](int x) { return (unsigned char)(x * 255 / top); };
    return {{c(r), c(g), c(b)}, double(top) / 15.0};
}

// the keys of the Analog Rytm MKII for the sequencer view (SeqView.hpp): codes of the
// panel layout JSON, read on the panel photo
SeqSpec seqSpec()
{
    static const char* const kNames[12] = {"BD", "SD", "RS", "CP", "BT", "LT", "MT", "HT", "CH", "OH", "CY", "CB"};
    static const char* const kPages[6] = {"TRIG", "SRC", "SMPL", "FLTR", "AMP", "LFO"};
    static const char* const kEnc[8] = {"A", "B", "C", "D", "E", "F", "G", "H"};
    SeqSpec s;
    s.title = "Analog Rytm MKII";
    s.tracks = 12;
    for (int i = 0; i < 12; ++i) s.trackNames[i] = kNames[i];
    for (int i = 0; i < 8; ++i) { s.trig[i] = 25 + i; s.trig[8 + i] = 17 + i; }
    s.nPageKeys = 6;
    for (int i = 0; i < 6; ++i) { s.pageKeys[i] = 43 + i; s.pageNames[i] = kPages[i]; }
    s.nEnc = 8;
    for (int i = 0; i < 8; ++i) { s.enc[i] = 1 + i; s.encNames[i] = kEnc[i]; }
    s.data = 9;
    s.func = 54; s.trk = 55; s.rec = 13; s.play = 14; s.stop = 15; s.page = 16;
    s.yes = 41; s.no = 33; s.up = 42; s.down = 35; s.left = 34; s.right = 36;
    s.select = SeqSpec::kTrkPad;                                   // TRK + pad, pad 0 = BD
    s.padBase = 0;
    s.nPageLeds = 4;
    s.screenMask = 1;
    s.mute = SeqSpec::kModePad;                                    // MUTE, then the pads
    s.muteKey = 49;
    s.ptnSel = SeqSpec::kBankKey;                                  // the bank key held, trig
    {
        static const int kBanks[8] = {1, 2, 3, 4, 9, 10, 11, 12};  // A-D, E-H
        for (int i = 0; i < 8; ++i) s.bankKeys[i] = kBanks[i];
    }
    s.nBanks = 8;
    s.knobX0 = 0;                                                  // no strip on the left
    s.knobYMid = 33;
    {                                                              // FUNC + key, flat
        static const char* const kFn[] = {"PTN: SEQUENTIAL", "PTN: DIR. START", "PTN: DIR. JUMP", "PTN: TEMP JUMP", "CHAIN", "MIDI", "TRIG MUTES", "ACCENT", "SWING", "SLIDE", "COPY", "CLEAR", "PASTE"};
        static const int kFk[] = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12, 13, 14, 15};
        s.nFuncs = int(sizeof kFk / sizeof kFk[0]);
        for (int i = 0; i < s.nFuncs; ++i) { s.funcNames[i] = kFn[i]; s.funcKeys[i] = kFk[i]; }
    }
    s.memo = "rytm";
    return s;
}

} // namespace

class RytmUI : public UI, public IdleCallback {
public:
    RytmUI()
        : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
    {
        fPlugin = static_cast<RytmPlugin*>(getPluginInstancePointer());
        loadSharedResources();
        const double sf = getScaleFactor();
        fStatusH = 22 * sf;
        if (fPlugin) {
            fVolume = fPlugin->masterVolume();
            fPhotoPath = fPlugin->photoPath();
        }
        applyConstraints();
        SeqHost host;
        host.send = [this](int kind, int code, int arg) { send(kind, code, arg); };
        host.keyLevel = [this](int code) -> float {
            for (const auto& m : arl::kLedToControl) if (m.code == code) return float(ledColour(fLit[m.led]).level);
            return 0.0f;
        };
        host.pageLevel = [this](int i) -> float {
            if (i < 0 || i >= int(sizeof arl::kPageLeds / sizeof arl::kPageLeds[0])) return 0.0f;
            return float(ledColour(fLit[arl::kPageLeds[i]]).level);
        };
        host.screen = [this]() -> const uint8_t* { return fScreen; };
        host.setLoop = [this](int page) { if (fPlugin) fPlugin->setLoopPage(page); };
        host.getLoop = [this]() -> int { return fPlugin ? fPlugin->loopPage() : -1; };
        fSeq.reset(new SeqView(seqSpec(), host));
        fView.load(seqSpec().memo);
        applyView();
        zoomTo(loadZoom(), false);
        addIdleCallback(this, 40);
    }

    ~RytmUI() override { removeIdleCallback(this); }

protected:
    // ------------------------------------------------------------ host
    void parameterChanged(uint32_t index, float value) override
    {
        if (index == RytmPlugin::kParamVolume) { fVolume = value; repaint(); }
    }

    void onResize(const ResizeEvent& ev) override
    {
        UI::onResize(ev);
        relayout();
    }

    void uiScaleFactorChanged(double sf) override
    {
        fStatusH = 22 * sf;
        applyConstraints();
        relayout();
    }

    // ------------------------------------------------------------ disegno
    void onNanoDisplay() override
    {
        // senza contesto OpenGL (NanoVG non creato) DPF va in crash sulle immagini
        if (getContext() == nullptr) return;
        if (!fPhotoPath.empty()) {                   // la foto al primo disegno, col contesto pronto
            fPhoto = createImageFromFile(fPhotoPath.c_str(), IMAGE_GENERATE_MIPMAPS);
            fPhotoPath.clear();
        }
        const float W = getWidth(), H = getHeight();
        if (fS <= 0) relayout();
        beginPath();
        rect(0, 0, W, H);
        fillColor(Color(arl::kBackground.r, arl::kBackground.g, arl::kBackground.b));
        fill();

        if (fSeqMode) {                                    // la vista sequencer (SeqView.hpp)
            fSeq->draw(*this, 0, 0, areaW(), H - fStatusH);
            drawOverlay();
            drawStatusBar(areaW(), H);
            drawDrive(W, H);
            return;
        }

        // il pannello e' la parte `view` della foto: tutto il resto si taglia
        const float pw = arl::kPanelW * fS, ph = arl::kPanelH * fS;
        scissor(fOffX, fOffY, pw, ph);
        if (fPhoto.isValid()) {
            beginPath();
            rect(px(0), py(0), arl::kImgW * fS, arl::kImgH * fS);
            fillPaint(imagePattern(px(0), py(0), arl::kImgW * fS, arl::kImgH * fS, 0, fPhoto, 1.0f));
            fill();
        } else {
            drawSchematic();
        }

        drawKeyLeds();
        // contorni dei tasti premuti e di FUNC/TRK agganciati
        for (const auto& k : arl::kKeys) {
            const bool latched = fLatched.count(k.code) != 0;
            const bool pressed = fDown.kind == 0 && fDown.active && fDown.code == k.code;
            if (!latched && !pressed) continue;
            const auto& c = latched ? arl::kOutlineLatched : arl::kOutlinePressed;
            outlineRect(k.x, k.y, k.w, k.h, c, arl::kOutlineWidth);
        }

        // tacca del Main Volume
        {
            const double a = (arl::kAngle0 + arl::kAngleSpan * std::min(1.0, fVolume / arl::kVolMax)) * M_PI / 180;
            const float dx = float(std::sin(a)), dy = float(-std::cos(a));
            beginPath();
            moveTo(px(arl::kVolX + dx * arl::kVolR * arl::kTickR0), py(arl::kVolY + dy * arl::kVolR * arl::kTickR0));
            lineTo(px(arl::kVolX + dx * arl::kVolR * arl::kTickR1), py(arl::kVolY + dy * arl::kVolR * arl::kTickR1));
            strokeColor(Color(arl::kVolTick.r, arl::kVolTick.g, arl::kVolTick.b));
            strokeWidth(std::max(1.0f, arl::kTickWidth * fS));
            lineCap(ROUND);
            stroke();
        }
        if (nowMs() < fVolShowUntil) {
            char t[32];
            std::snprintf(t, sizeof t, "vol %d%%", int(std::lround(fVolume / arl::kVolMax * 100)));
            fontFace(NANOVG_DEJAVU_SANS_TTF);
            fontSize(std::max(6.0f, std::round(11 * fS)));
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            fillColor(Color(arl::kVolText.r, arl::kVolText.g, arl::kVolText.b));
            text(px(arl::kVolX), py(arl::kVolY + arl::kVolR + arl::kVolTextDY), t, nullptr);
        }

        if (!fBig) drawScreenAndBezel();
        drawPadLeds();
        if (fBig) drawScreenAndBezel();
        resetScissor();

        drawOverlay();
        drawStatusBar(areaW(), H);
        drawDrive(W, H);
    }

    // ------------------------------------------------------------ input
    bool onMouse(const MouseEvent& ev) override
    {
        // the View menu (Classic / Elektron Emu) in the status bar
        if (ev.button == 1 && ev.press && fView.click(ev.pos.getX(), ev.pos.getY(), float(getScaleFactor())) >= 0) {
            applyView();
            repaint();
            return true;
        }
        if (ev.button != 1) return false;
        const float x = ev.pos.getX(), y = ev.pos.getY();
        if (x >= areaW()) {                                // linguetta e +DRIVE
            if (ev.press) driveClick(x, y, (ev.mod & kModifierControl) || (ev.mod & kModifierSuper));
            return true;
        }
        if (ev.press) {
            const int z = zoomHit(x, y);
            if (z >= 0) { zoomTo(kZooms[z], true); return true; }
            if (gripHit(x, y)) { fGrip = true; fGripX = x; fGripW = areaW(); return true; }
            if (overlayShown() && overlayClick(x, y)) return true;      // la scheda sopra il pannello
            if (fSeqMode) {
                fSeq->mouse(true, x, y, nowMs());
                repaint();
                return true;
            }
            const float qx = qxOf(x), qy = qyOf(y);
            if (onScreen(qx, qy)) {                                    // clic sullo schermo: grande/piccolo
                fBig = !fBig;
                fScreenDirty = true;
                fDown = Hit();
                repaint();
                return true;
            }
            if (const arl::Pad* p = padAt(qx, qy)) {                   // pad: velocity dall'altezza (piu' in alto = piu' forte)
                const double top = p->y - int(p->h) / 2;
                const double v = std::max(arl::kPadVelMin, std::min(arl::kPadVelMax,
                                    arl::kPadVelTop - (qy - top) * arl::kPadVelSpan / p->h));
                fDown = Hit();
                fDown.active = true; fDown.kind = 3; fDown.code = p->id;
                send(4, p->id, int(v));
                repaint();
                return true;
            }
            Hit h = hit(qx, qy);
            fDown = h;
            if (!h.active) return false;
            if (h.kind == 0) {
                if (isLatching(h.code)) {
                    if (fLatched.count(h.code)) { fLatched.erase(h.code); send(1, h.code); }
                    else { fLatched.insert(h.code); send(0, h.code); }
                } else {
                    send(0, h.code);
                }
            } else {
                fDragY = qy; fDragT = nowMs(); fDragMoved = false; fDragAcc = 0;
            }
            repaint();
            return true;
        }
        // rilascio
        if (fGrip) { fGrip = false; saveZoom(currentZoom()); return true; }
        if (fSeqMode) { fSeq->mouse(false, x, y, nowMs()); repaint(); return true; }
        if (!fDown.active) return false;
        const Hit d = fDown;
        fDown.active = false;
        if (d.kind == 3) {
            send(4, d.code, 0);
        } else if (d.kind == 0) {
            if (!isLatching(d.code)) {
                send(1, d.code);
                if (!fLatched.empty()) fReleaseLatchedAt = nowMs() + arl::kAutoReleaseMs;
            }
        } else if (d.kind == 1 && !fDragMoved) {
            send(0, d.push);
            fPushRelease.push_back({nowMs() + arl::kPushReleaseMs, d.push});
        }
        repaint();
        return true;
    }

    bool onMotion(const MotionEvent& ev) override
    {
        const float x = ev.pos.getX(), y = ev.pos.getY();
        if (fGrip) {
            const double sf = getScaleFactor();
            const float w = std::max<float>(arl::kPanelW * 0.4f * sf, fGripW + (x - fGripX));
            zoomTo(w / (arl::kPanelW * sf), false);
            return true;
        }
        if (x >= areaW()) { driveMotion(x, y); return true; }
        if (fSeqMode) {
            if (fSeq->motion(x, y, 0, (ev.mod & kModifierShift) != 0)) repaint();
            const bool over = gripHit(x, y) || zoomHit(x, y) >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return true;
        }
        const float qx = qxOf(x), qy = qyOf(y);
        if (!fDown.active || fDown.kind == 0 || fDown.kind == 3) {
            const bool over = hit(qx, qy).active || padAt(qx, qy) || onScreen(qx, qy) || zoomHit(x, y) >= 0 || gripHit(x, y) ||
                              overlayHit(x, y) >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return false;
        }
        const double t = nowMs();
        const double dy = fDragY - qy;
        const double speed = std::fabs(dy) / std::max(1e-3, t - fDragT);
        fDragAcc += dy / arl::kDragPx * (speed > arl::kFastSpeed ? arl::kFastGain : 1);
        const int steps = int(fDragAcc);                                    // verso lo zero
        if (steps != 0) {
            fDragAcc -= steps;
            if (fDown.kind == 2) volumeBy(steps);
            else turn(fDown.code, steps);
        }
        fDragMoved = fDragMoved || dy != 0;
        fDragY = qy; fDragT = t;
        return true;
    }

    bool onScroll(const ScrollEvent& ev) override
    {
        if (ev.pos.getX() >= areaW()) {
            fDriveScroll = std::max(0, fDriveScroll - int(std::lround(ev.delta.getY() * 3)));
            repaint();
            return true;
        }
        if (fSeqMode) {
            if (!fSeq->scroll(ev.pos.getX(), ev.pos.getY(), float(ev.delta.getY()), (ev.mod & kModifierShift) != 0)) return false;
            repaint();
            return true;
        }
        const Hit h = hit(qxOf(ev.pos.getX()), qyOf(ev.pos.getY()));
        const double delta = ev.delta.getY();
        if (!h.active || delta == 0) return false;
        if (h.kind == 1) {
            int step = int(std::max<double>(-arl::kWheelClamp, std::min<double>(arl::kWheelClamp, delta)));
            if (step == 0) step = delta > 0 ? 1 : -1;
            step *= arl::kWheelGain;
            if (ev.mod & kModifierShift) step *= arl::kShiftMult;
            turn(h.code, step);
        } else if (h.kind == 2) {
            volumeBy(delta > 0 ? arl::kWheelGain : -arl::kWheelGain);
        }
        return true;
    }

    bool onKeyboard(const KeyboardEvent& ev) override
    {
        // come la GUI v2: Cmd+S / Ctrl+S salva la macchina adesso (si salva comunque alla chiusura)
        if (ev.press && !fMkdirMode && (ev.mod & (kModifierControl | kModifierSuper)) && (ev.key == 's' || ev.key == 'S')) {
            if (fPlugin) fPlugin->requestSave();
            repaint();
            return true;
        }
        if (!fMkdirMode || !ev.press) return false;
        if (ev.key == kKeyEscape) { fMkdirMode = false; }
        else if (ev.key == kKeyEnter) {
            fMkdirMode = false;
            if (!fMkdirText.empty() && fDrive) fDrive->makeDir(fMkdirText);
        } else if (ev.key == kKeyBackspace) { if (!fMkdirText.empty()) fMkdirText.pop_back(); }
        else if (ev.key >= 32 && ev.key < 127 && ev.key != '/' && fMkdirText.size() < 32) fMkdirText.push_back(char(ev.key));
        else return false;
        repaint();
        return true;
    }

    void uiFileBrowserSelected(const char* filename) override
    {
        if (!filename || !*filename) return;
        if (fPickMode == 1) installFirmware(filename);
        else if (fDrive) fDrive->upload({filename});
    }

    // ------------------------------------------------------------ 25 Hz
    void idleCallback() override
    {
        if (!fPlugin) return;
        const double t = nowMs();
        bool dirty = false;
        const uint64_t v = fPlugin->copyScreen(fScreen, fLit, fPads);
        if (v != fVersion) { fVersion = v; fScreenDirty = true; fSeq->screenChanged(); dirty = true; }
        if (fSeq->tick(t)) dirty = true;
        if (fView.mode == ViewMenu::kEmu) {                // the pattern read in the OS memory
            SeqData sd;
            SeqGrid g;
            if (fPlugin->seqData(sd)) {
                g.valid = true;
                g.pattern = sd.pattern;
                g.tracks = sd.tracks;
                g.mutes = sd.mutes;
                g.playing = sd.playing != 0;
                for (int i = 0; i < 16; ++i) {
                    g.trigs[i] = sd.trigs[i];
                    g.trigs2[i] = sd.trigs2[i];
                    g.step[i] = sd.step[i] == 0xff ? -1 : sd.step[i];
                }
            }
            if (fSeq->setGrid(g)) dirty = true;
        }
        if (fReleaseLatchedAt > 0 && t >= fReleaseLatchedAt) {
            fReleaseLatchedAt = 0;
            for (int c : fLatched) send(1, c);
            fLatched.clear();
            dirty = true;
        }
        for (size_t i = 0; i < fPushRelease.size();) {
            if (t >= fPushRelease[i].first) { send(1, fPushRelease[i].second); fPushRelease.erase(fPushRelease.begin() + i); }
            else ++i;
        }
        if (fVolShowUntil > 0 && t >= fVolShowUntil) { fVolShowUntil = 0; dirty = true; }
        const RytmViewInfo es = fPlugin->viewInfo();
        std::string st = es.text;
        if (es.kind == RytmViewInfo::kRunning) {
            st = (es.firmwareLabel.empty() ? std::string("Analog Rytm MKII") : es.firmwareLabel) + " - emulator (VST3)";
            if (!es.warning.empty()) st += " - " + es.warning;
        }
        if (fNoteUntil > t) st = fNote;
        if (st != fStatus) { fStatus = st; dirty = true; }
        if (es.kind != fInfo.kind || es.firmwareDir != fInfo.firmwareDir || es.warning != fInfo.warning ||
            es.version != fInfo.version || (es.kind == RytmViewInfo::kBooting && int(es.bootSeconds) != int(fInfo.bootSeconds)))
            dirty = true;
        fInfo = es;
        if (fDriveOpen && fDrive) {
            const uint64_t dv = fDrive->snapshot().version;
            if (dv != fDriveVersion) { fDriveVersion = dv; dirty = true; }
        }
        if (fDelConfirmUntil > 0 && t >= fDelConfirmUntil) { fDelConfirmUntil = 0; dirty = true; }
        if (dirty) repaint();
    }

private:
    struct Hit { bool active = false; int kind = 0, code = 0, push = 0; };   // kind: 0 tasto, 1 encoder, 2 volume, 3 pad

    static constexpr float kTab = 24, kDriveW = 330;
    float extraW() const { return (kTab + (fDriveOpen ? kDriveW : 0)) * float(getScaleFactor()); }
    float areaW() const { return float(getWidth()) - extraW(); }

    // foto (coordinate della foto scalata) <-> vista: VIEW[0] + scostamento + s * (x - VIEW[0])
    float px(float x) const { return fOffX + (x - arl::kViewX) * fS; }
    float py(float y) const { return fOffY + (y - arl::kViewY) * fS; }
    float qxOf(float x) const { return arl::kViewX + (x - fOffX) / fS; }
    float qyOf(float y) const { return arl::kViewY + (y - fOffY) / fS; }

    void relayout()
    {
        const float w = areaW(), h = getHeight() - fStatusH;
        if (w <= 2 || h <= 2) return;
        float sc = std::max<float>(arl::kMinScale, std::min(w / arl::kPanelW, h / arl::kPanelH));
        if (std::fabs(sc - 1.0f) < 0.004f) sc = 1.0f;
        fS = sc;
        fOffX = std::round((w - arl::kPanelW * fS) / 2);
        fOffY = std::round((h - arl::kPanelH * fS) / 2);
        fScreenDirty = true;
        fGlow.clear();
        repaint();
    }

    void zoomTo(float z, bool remember)
    {
        const double sf = getScaleFactor();
        const uint w = uint(std::lround(arl::kPanelW * z * sf + extraW()));
        const uint h = uint(std::lround(arl::kPanelH * z * sf + fStatusH));
        setSize(w, h);
        if (remember) {
            saveZoom(z);
            // se l'host non ha cambiato misura (come Live con l'AUv3) lo si dice
            if (uint(getWidth()) != w) { fNote = "The host did not resize the window: the zoom applies the next time it opens"; fNoteUntil = nowMs() + 4000; fNoteError = false; }
        }
        relayout();
    }

    int zoomHit(float x, float y) const
    {
        const float H = getHeight(), W = areaW(), sf = getScaleFactor();
        if (y < H - fStatusH || x >= W) return -1;
        for (int i = 0; i < kNZooms; ++i) {
            const float zx = W - 210 * sf + i * 46 * sf;
            if (x >= zx - 2 && x <= zx + 44 * sf + 2) return i;
        }
        return -1;
    }

    bool gripHit(float x, float y) const
    {
        const float sf = getScaleFactor();
        return x > areaW() - 18 * sf && x < areaW() && y > getHeight() - 18 * sf && zoomHit(x, y) < 0;
    }

    // emu/guiv2_rytm.py: lo schermo piccolo o grande
    bool onScreen(float qx, float qy) const
    {
        const arl::Rect& r = fBig ? arl::kBigScreen : arl::kScreen;
        return r.x <= qx && qx < r.x + r.w && r.y <= qy && qy < r.y + r.h;
    }

    const arl::Pad* padAt(float qx, float qy) const
    {
        for (const auto& p : arl::kPads)
            if (std::fabs(qx - p.x) <= int(p.w) / 2 && std::fabs(qy - p.y) <= int(p.h) / 2) return &p;
        return nullptr;
    }

    Hit hit(float x, float y) const
    {
        Hit h;
        for (const auto& k : arl::kKeys)
            if (std::fabs(x - k.x) <= int(k.w) / 2 && std::fabs(y - k.y) <= int(k.h) / 2) {
                h.active = true; h.kind = 0; h.code = k.code; return h;
            }
        for (const auto& e : arl::kEncoders)
            if ((x - e.x) * (x - e.x) + (y - e.y) * (y - e.y) <= e.r * e.r) {
                h.active = true; h.kind = 1; h.code = e.turn; h.push = e.push; return h;
            }
        if ((x - arl::kVolX) * (x - arl::kVolX) + (y - arl::kVolY) * (y - arl::kVolY) <= arl::kVolR * arl::kVolR) {
            h.active = true; h.kind = 2;
        }
        return h;
    }

    void send(int kind, int code, int arg = 0)
    {
        if (fPlugin) fPlugin->panel(kind, code, arg);
    }

    void turn(int code, int steps)
    {
        send(3, code, std::max(-arl::kTurnClamp, std::min(arl::kTurnClamp, steps)));
    }

    // RytmApp._volume_by: MASTER +- 0,02 a scatto, 0..1; "vol N%" per 1,5 s
    void volumeBy(int steps)
    {
        fVolume = std::max(0.0, std::min<double>(arl::kVolMax, fVolume + arl::kVolumeStep * steps));
        if (fPlugin) fPlugin->setMasterFromUI(fVolume);
        editParameter(RytmPlugin::kParamVolume, true);
        setParameterValue(RytmPlugin::kParamVolume, float(fVolume));
        editParameter(RytmPlugin::kParamVolume, false);
        fVolShowUntil = nowMs() + arl::kVolHideMs;
        repaint();
    }

    // contorno di un tasto/pad centrato in (cx, cy) di misura (w, h) della foto
    void outlineRect(float cx, float cy, float w, float h, arl::RGB c, float width)
    {
        const float hw = int(w) / 2, hh = int(h) / 2;
        beginPath();
        rect(px(cx - hw), py(cy - hh), 2 * hw * fS, 2 * hh * fS);
        strokeColor(Color(c.r, c.g, c.b));
        strokeWidth(std::max(1.0f, width * fS));
        stroke();
    }

    // emu/guiv2.py _background, ramo senza foto
    void drawSchematic()
    {
        auto box = [&](float x, float y, float w, float h, arl::RGB f, arl::RGB s, bool oval) {
            beginPath();
            if (oval) ellipse(px(x + w / 2), py(y + h / 2), w / 2 * fS, h / 2 * fS);
            else rect(px(x), py(y), w * fS, h * fS);
            fillColor(Color(f.r, f.g, f.b));
            fill();
            strokeColor(Color(s.r, s.g, s.b));
            strokeWidth(1);
            stroke();
        };
        // il pannello vuoto (la parte mostrata) e i suoi controlli
        box(arl::kViewX, arl::kViewY, arl::kPanelW, arl::kPanelH, {0x16, 0x17, 0x1a}, {0x16, 0x17, 0x1a}, false);
        for (const auto& k : arl::kKeys) {
            const float hw = int(k.w) / 2, hh = int(k.h) / 2;
            box(k.x - hw, k.y - hh, 2 * hw, 2 * hh, {0x22, 0x23, 0x28}, {0x3a, 0x3c, 0x42}, false);
        }
        for (const auto& p : arl::kPads) {
            const float hw = int(p.w) / 2, hh = int(p.h) / 2;
            box(p.x - hw, p.y - hh, 2 * hw, 2 * hh, {0x2a, 0x2b, 0x30}, {0x3a, 0x3c, 0x42}, false);
        }
        for (const auto& e : arl::kEncoders)
            box(e.x - e.r, e.y - e.r, 2 * e.r, 2 * e.r, {0x18, 0x19, 0x1c}, {0x3a, 0x3c, 0x42}, true);
        box(arl::kVolX - arl::kVolR, arl::kVolY - arl::kVolR, 2 * arl::kVolR, 2 * arl::kVolR,
            {0x18, 0x19, 0x1c}, {0x3a, 0x3c, 0x42}, true);
    }

    // guiv2_rytm.RytmApp._set_screen: la cornice nera e lo schermo, piccolo (la
    // cornice copre il logo fino a kBezelBottom) o grande
    void drawScreenAndBezel()
    {
        const arl::Rect& r = fBig ? arl::kBigScreen : arl::kScreen;
        const float p = fBig ? arl::kBigPad : arl::kBezelPad;
        const float bottom = fBig ? r.y + r.h + p : std::max(r.y + r.h + p, arl::kBezelBottom);
        const float ax = px(r.x - p), ay = py(r.y - p), bx = px(r.x + r.w + p), by = py(bottom);
        beginPath();
        rect(ax, ay, bx - ax, by - ay);
        fillColor(Color(arl::kBezelFill.r, arl::kBezelFill.g, arl::kBezelFill.b));
        fill();
        strokeColor(Color(arl::kBezelOutline.r, arl::kBezelOutline.g, arl::kBezelOutline.b));
        strokeWidth(std::max(1.0f, arl::kBezelWidth * fS));
        stroke();
        drawScreen(r);
    }

    // emu/guiv2.py _draw_screen: 128x64 -> w x h con la media d'area
    void drawScreen(const arl::Rect& r)
    {
        const int w = std::max(1, int(std::lround(r.w * fS)));
        const int h = std::max(1, int(std::lround(r.h * fS)));
        if (w != fScreenW || h != fScreenH || !fScreenImg.isValid()) {
            fScreenW = w; fScreenH = h;
            fWy = boxWeights(64, h);
            fWx = boxWeights(128, w);
            fScreenPx.assign(size_t(w) * h * 4, 255);
            fScreenImg = createImageFromRGBA(w, h, fScreenPx.data(), 0);
            fScreenDirty = true;
        }
        if (fScreenDirty) {
            fScreenDirty = false;
            const float lut[2][3] = {{float(arl::kScreenOff.r), float(arl::kScreenOff.g), float(arl::kScreenOff.b)},
                                     {float(arl::kScreenOn.r), float(arl::kScreenOn.g), float(arl::kScreenOn.b)}};
            std::vector<float> tmp(size_t(h) * 128 * 3, 0.0f);
            for (int y = 0; y < h; ++y)
                for (const auto& sw : fWy[y])
                    for (int x = 0; x < 128; ++x) {
                        const float* c = lut[fScreen[sw.first * 128 + x] & 1];
                        float* t = &tmp[(size_t(y) * 128 + x) * 3];
                        t[0] += sw.second * c[0]; t[1] += sw.second * c[1]; t[2] += sw.second * c[2];
                    }
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x) {
                    float rr = 0, g = 0, b = 0;
                    for (const auto& sw : fWx[x]) {
                        const float* t = &tmp[(size_t(y) * 128 + sw.first) * 3];
                        rr += sw.second * t[0]; g += sw.second * t[1]; b += sw.second * t[2];
                    }
                    uint8_t* p = &fScreenPx[(size_t(y) * w + x) * 4];
                    p[0] = uint8_t(std::max(0.0f, std::min(255.0f, rr + 0.5f)));
                    p[1] = uint8_t(std::max(0.0f, std::min(255.0f, g + 0.5f)));
                    p[2] = uint8_t(std::max(0.0f, std::min(255.0f, b + 0.5f)));
                    p[3] = 255;
                }
            fScreenImg.update(fScreenPx.data());
        }
        const float x = px(r.x), y = py(r.y);
        beginPath();
        rect(x, y, w, h);
        fillPaint(imagePattern(x, y, w, h, 0, fScreenImg, 1.0f));
        fill();
    }

    // emu/guiv2.py _glow: bagliore w x h a intensita' level/(levels-1), bordi sfumati
    NanoImage& glow(float w, float h, int level, bool round, arl::RGB c)
    {
        const int gw = std::max(1, int(std::lround(w * fS))), gh = std::max(1, int(std::lround(h * fS)));
        const double soft = std::round(std::max(1.0, arl::kGlowSoft * fS) * 100) / 100;
        char key[96];
        std::snprintf(key, sizeof key, "%dx%d/%d/%d/%d,%d,%d", gw, gh, round, level, c.r, c.g, c.b);
        auto it = fGlow.find(key);
        if (it != fGlow.end()) return *it->second;
        std::vector<uint8_t> px(size_t(gw) * gh * 4);
        for (int y = 0; y < gh; ++y)
            for (int x = 0; x < gw; ++x) {
                double edge;
                if (round) {
                    const double dx = (x - (gw - 1) / 2.0) / (gw / 2.0), dy = (y - (gh - 1) / 2.0) / (gh / 2.0);
                    edge = std::min(1.0, std::max(0.0, (1.0 - std::sqrt(dx * dx + dy * dy)) * arl::kRoundEdge));
                } else {
                    const double m = std::min(std::min(x, y), std::min(gw - 1 - x, gh - 1 - y)) / soft;
                    edge = std::min(1.0, std::max(0.0, m));
                }
                const uint8_t a = uint8_t(edge * arl::kGlowAlpha * level / (arl::kLevels - 1));
                uint8_t* p = &px[(size_t(y) * gw + x) * 4];
                p[0] = c.r; p[1] = c.g; p[2] = c.b; p[3] = a;
            }
        auto img = std::make_unique<NanoImage>(createImageFromRGBA(gw, gh, px.data(), 0));
        NanoImage& ref = *img;
        fGlow[key] = std::move(img);
        return ref;
    }

    void drawImageAt(NanoImage& img, float cx, float cy)
    {
        const Size<uint> sz = img.getSize();
        const float x = std::round(cx - sz.getWidth() / 2.0f), y = std::round(cy - sz.getHeight() / 2.0f);
        beginPath();
        rect(x, y, sz.getWidth(), sz.getHeight());
        fillPaint(imagePattern(x, y, sz.getWidth(), sz.getHeight(), 0, img, 1.0f));
        fill();
    }

    // RytmApp._paint: lv = round(level * (LEVELS - 1)), niente bagliore a 0
    void paintGlow(float cx, float cy, float w, float h, const LedLook& look, bool round)
    {
        const int lv = int(std::nearbyint(look.level * (arl::kLevels - 1)));
        if (lv <= 0) return;
        drawImageAt(glow(w, h, lv, round, look.c), px(cx), py(cy));
    }

    // RytmApp._show_leds: tasti
    void drawKeyLeds()
    {
        for (const auto& m : arl::kLedToControl) {
            const arl::Key* k = nullptr;
            for (const auto& kk : arl::kKeys) if (kk.code == m.code) { k = &kk; break; }
            if (!k) continue;
            paintGlow(k->x, k->y, k->w, k->h, ledColour(fLit[m.led]), false);
        }
    }

    // pad RGB (con il loro contorno quando premuti) e LED di pagina rotondi
    void drawPadLeds()
    {
        for (const auto& p : arl::kPads)
            paintGlow(p.x, p.y, p.w, p.h, padRGB(fPads[p.id][0], fPads[p.id][1], fPads[p.id][2]), false);
        int i = 0;
        for (int led : arl::kPageLeds) {
            paintGlow(arl::kPageXY[i][0], arl::kPageXY[i][1], arl::kPageSize, arl::kPageSize, ledColour(fLit[led]), true);
            ++i;
        }
        if (fDown.active && fDown.kind == 3)
            for (const auto& p : arl::kPads)
                if (p.id == fDown.code) outlineRect(p.x, p.y, p.w, p.h, arl::kPadOutline, arl::kPadOutlineWidth);
    }

    // ------------------------------------------------------------ +DRIVE
    // coordinate in unita' di vista (come il pannello): x0 = bordo sinistro della colonna
    struct Btn { const char* label; float x, y, w; bool accent; };
    static constexpr int kRowH = 24, kBottom = 130;

    static Color hexc(unsigned v, int a = 255) { return Color(int(v >> 16 & 255), int(v >> 8 & 255), int(v & 255), a / 255.0f); }
    static constexpr unsigned cBg = 0x141517, cRow = 0x1b1c1f, cLine = 0x2a2c30, cFg = 0xe6e3dc, cDim = 0x9aa0a8,
                              cAccent = 0xe8b04a, cErr = 0xe0654f;

    void ensureDrive()
    {
        if (!fDrive && fPlugin) fDrive = fPlugin->drive();
    }

    // i tasti: 0 CARICA WAV, 1 CARICA CARTELLA (prima riga), 2 NUOVA CARTELLA, 3 ELIMINA, 4 AGGIORNA
    enum { kBtnWav, kBtnFolder, kBtnMkdir, kBtnDelete, kBtnRefresh, kNBtn };
    Btn btn(int i, float H) const
    {
        const float y1 = H / float(getScaleFactor()) - kBottom + 12;
        static const char* labels[] = {"LOAD WAV", "LOAD FOLDER", "NEW FOLDER", "DELETE", "REFRESH"};
        static const float widths[] = {104, 136, 124, 72, 84};
        static const float xs[] = {14, 14 + 104 + 6, 14, 14 + 124 + 6, 14 + 124 + 6 + 72 + 6};
        return {labels[i], xs[i], i < 2 ? y1 : y1 + 30, widths[i], i < 2};
    }

    bool btnEnabled(int i, const DriveSnapshot& s) const
    {
        const bool writable = !s.busy && s.path != "/" && !s.readOnly;
        if (i == kBtnWav || i == kBtnMkdir) return writable;
#if defined(_WIN32)
        if (i == kBtnFolder) return writable;
#else
        if (i == kBtnFolder) return false;          // il file browser di DPF non sceglie cartelle
#endif
        if (i == kBtnDelete) return writable && !fSel.empty();
        return !s.busy;
    }

    std::vector<const DriveEntry*> shownRows(const DriveSnapshot& s) const
    {
        std::vector<const DriveEntry*> rows;
        if (s.path != "/") rows.push_back(nullptr);                 // ".."
        for (const auto& e : s.entries) rows.push_back(&e);
        return rows;
    }

    // proporzioni libere: il pannello e' gia' centrato e in scala (relayout), e la
    // colonna del +DRIVE ha larghezza fissa. Con keepAspectRatio la finestra
    // allargata dal +DRIVE veniva stretta nella proporzione del solo pannello e
    // a ogni apertura/chiusura lo zoom scendeva.
    void applyConstraints()
    {
        const double sf = getScaleFactor();
        setGeometryConstraints(uint(arl::kPanelW * 0.4 * sf + extraW()), uint((arl::kPanelH * 0.4 + 22) * sf), false, false);
    }

    // lo zoom attuale: la scala vera del pannello (non la larghezza della finestra)
    float currentZoom() const { return fS / float(getScaleFactor()); }

    void toggleDrive()
    {
        ensureDrive();
        const float zoom = currentZoom();
        fDriveOpen = !fDriveOpen;
        applyConstraints();
        zoomTo(zoom, false);
        if (fDriveOpen) fDrive->refresh();
        repaint();
    }

    void pickFiles()
    {
        fPickMode = 0;
        ensureDrive();
        auto drive = fDrive;
        const std::string dest = DriveModel::destFolder(drive->snapshot().path);
#if defined(_WIN32)
        // selezione multipla con la finestra nativa, in un thread: e' bloccante
        std::thread([drive, dest] {
            std::vector<char> buf(1 << 16, 0);
            OPENFILENAMEA o{};
            o.lStructSize = sizeof o;
            o.lpstrFilter = "WAV files\0*.wav\0All files\0*.*\0";
            o.lpstrFile = buf.data();
            o.nMaxFile = DWORD(buf.size());
            o.lpstrTitle = "Load samples into the +Drive";
            o.Flags = OFN_ALLOWMULTISELECT | OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
            if (!GetOpenFileNameA(&o)) return;
            std::vector<std::string> files;
            const std::string first = buf.data();
            const char* p = buf.data() + first.size() + 1;
            if (*p == 0) files.push_back(first);                      // un solo file: percorso intero
            else for (; *p; p += std::strlen(p) + 1) files.push_back(first + "\\" + p);
            drive->upload(files);
        }).detach();
#else
        FileBrowserOptions opt;
        opt.title = "Load WAV samples into the +Drive";
        openFileBrowser(opt);
        (void)dest;
#endif
    }

    // una cartella intera: sottocartelle ricreate sul +Drive, solo i WAV (solo Windows)
    void pickFolder()
    {
#if defined(_WIN32)
        ensureDrive();
        auto drive = fDrive;
        std::thread([drive] {
            OleInitialize(nullptr);
            char dir[MAX_PATH] = "";
            BROWSEINFOA b{};
            b.pszDisplayName = dir;
            b.lpszTitle = "Folder of WAV samples to load into the +Drive";
            b.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE | BIF_NONEWFOLDERBUTTON;
            if (LPITEMIDLIST id = SHBrowseForFolderA(&b)) {
                if (SHGetPathFromIDListA(id, dir)) drive->upload({dir});
                CoTaskMemFree(id);
            }
            OleUninitialize();
        }).detach();
#endif
    }

    void driveClick(float x, float y, bool multi)
    {
        const float sf = float(getScaleFactor());
        const float H = getHeight();
        if (x >= getWidth() - kTab * sf) { toggleDrive(); return; }
        if (!fDriveOpen || !fDrive) return;
        const float lx = (x - areaW()) / sf, ly = y / sf;
        const DriveSnapshot s = fDrive->snapshot();
        if (fMkdirMode) fMkdirMode = false;                           // un clic altrove annulla
        // su di un livello
        if (lx >= 14 && lx <= 42 && ly >= 40 && ly <= 64) { if (!s.busy && s.path != "/") { fSel.clear(); fDriveScroll = 0; fDrive->up(); } return; }
        for (int i = 0; i < kNBtn; ++i) {
            const Btn b = btn(i, H);
            if (lx < b.x || lx > b.x + b.w || ly < b.y || ly > b.y + 24) continue;
            if (!btnEnabled(i, s)) return;
            if (i == kBtnWav) pickFiles();
            else if (i == kBtnFolder) pickFolder();
            else if (i == kBtnMkdir) { fMkdirMode = true; fMkdirText.clear(); }
            else if (i == kBtnDelete) {
                if (fDelConfirmUntil == 0) { fDelConfirmUntil = nowMs() + 3000; }   // secondo clic: conferma
                else {
                    fDelConfirmUntil = 0;
                    std::vector<std::pair<std::string, bool>> items;
                    for (const auto& e : s.entries) if (fSel.count(e.name)) items.push_back({e.name, e.dir});
                    fSel.clear();
                    fDrive->remove(items);
                }
            } else fDrive->refresh();
            repaint();
            return;
        }
        // lista
        const float top = 76, bottom = H / sf - kBottom;
        if (ly < top || ly > bottom) return;
        const auto rows = shownRows(s);
        const int r = int((ly - top) / kRowH) + fDriveScroll;
        if (r < 0 || r >= int(rows.size())) return;
        const double t = nowMs();
        const bool dbl = r == fLastClickRow && t - fLastClickT < 400;
        fLastClickRow = r; fLastClickT = t;
        if (dbl) {
            if (!rows[size_t(r)]) { fSel.clear(); fDriveScroll = 0; fDrive->up(); }
            else if (rows[size_t(r)]->dir) { const std::string n = rows[size_t(r)]->name; fSel.clear(); fDriveScroll = 0; fDrive->enter(n); }
            return;
        }
        if (!rows[size_t(r)]) return;
        if (!multi) fSel.clear();
        const std::string& n = rows[size_t(r)]->name;
        if (multi && fSel.count(n)) fSel.erase(n); else fSel.insert(n);
        fDelConfirmUntil = 0;
        repaint();
    }

    void driveMotion(float x, float y)
    {
        bool over = x >= getWidth() - kTab * float(getScaleFactor());
        if (!over && fDriveOpen) {
            const float sf = float(getScaleFactor());
            const float lx = (x - areaW()) / sf, ly = y / sf;
            over = ly >= 40 && ly <= getHeight() / sf - kBottom + 100 && lx >= 14;
        }
        if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
    }

    void flatButton(const Btn& b, bool enabled, bool hoverAccent, const char* label, float sf, float x0)
    {
        const float bx = x0 + b.x * sf, by = b.y * sf, bw = b.w * sf, bh = 24 * sf;
        beginPath();
        rect(bx, by, bw, bh);
        fillColor(enabled ? (b.accent ? hexc(cAccent) : hexc(cRow)) : hexc(cBg));
        fill();
        beginPath();
        rect(bx + 0.5f, by + 0.5f, bw - 1, bh - 1);
        strokeColor(hoverAccent && enabled ? hexc(cAccent) : hexc(cLine));
        strokeWidth(1);
        stroke();
        fontSize(10 * sf);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fillColor(enabled ? (b.accent ? hexc(cBg) : hexc(cFg)) : hexc(cDim));
        text(bx + bw / 2, by + bh / 2, label, nullptr);
    }

    void drawDrive(float W, float H)
    {
        const float sf = float(getScaleFactor());
        const float tabX = W - kTab * sf;
        // linguetta
        beginPath();
        rect(tabX, 0, kTab * sf, H);
        fillColor(hexc(cBg));
        fill();
        beginPath();
        rect(tabX, 0, 1, H);
        fillColor(hexc(cLine));
        fill();
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(16 * sf);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fillColor(hexc(cAccent));
        text(tabX + 12 * sf, 26 * sf, fDriveOpen ? ">" : "<", nullptr);
        save();
        translate(tabX + 12 * sf, 120 * sf);
        rotate(-float(M_PI) / 2);
        fontSize(10 * sf);
        fillColor(hexc(cFg));
        text(0, 0, "+DRIVE", nullptr);
        restore();
        if (!fDriveOpen || !fDrive) return;

        const float x0 = areaW();
        const DriveSnapshot s = fDrive->snapshot();
        beginPath();
        rect(x0, 0, kDriveW * sf, H);
        fillColor(hexc(cBg));
        fill();
        fontSize(11 * sf);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        fillColor(hexc(cFg));
        text(x0 + 14 * sf, 22 * sf, "+DRIVE", nullptr);

        Btn up{"^", 14, 40, 28, false};
        flatButton(up, !s.busy && s.path != "/", false, "^", sf, x0);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        fontSize(11 * sf);
        fillColor(hexc(cAccent));
        const std::string pathText = s.path + (s.readOnly ? "   (read only)" : "");
        scissor(x0 + 50 * sf, 40 * sf, (kDriveW - 64) * sf, 24 * sf);
        text(x0 + 50 * sf, 52 * sf, pathText.c_str(), nullptr);
        resetScissor();
        beginPath();
        rect(x0 + 14 * sf, 70 * sf, (kDriveW - 28) * sf, 1);
        fillColor(hexc(cLine));
        fill();

        // lista
        const auto rows = shownRows(s);
        const float top = 76 * sf, bottom = H - kBottom * sf;
        const int visible = std::max(1, int((bottom - top) / (kRowH * sf)));
        fDriveScroll = std::max(0, std::min(fDriveScroll, std::max(0, int(rows.size()) - visible)));
        scissor(x0, top, kDriveW * sf, bottom - top);
        for (int i = 0; i + fDriveScroll < int(rows.size()) && i <= visible; ++i) {
            const DriveEntry* e = rows[size_t(i + fDriveScroll)];
            const float y = top + i * kRowH * sf;
            const bool sel = e && fSel.count(e->name);
            if (sel) {
                beginPath();
                rect(x0 + 14 * sf, y, (kDriveW - 28) * sf, kRowH * sf);
                fillColor(hexc(cAccent, 70));
                fill();
            }
            fontSize(11 * sf);
            textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
            const std::string name = !e ? ".." : (e->dir ? "> " + e->name + "/" : "  " + e->name);
            fillColor(!e || e->dir ? hexc(cFg) : (e->readOnly ? hexc(cDim) : hexc(0xb9b6ae)));
            scissor(x0 + 14 * sf, y, (kDriveW - 28 - 106) * sf, kRowH * sf);
            text(x0 + 18 * sf, y + kRowH * sf / 2, name.c_str(), nullptr);
            scissor(x0, top, kDriveW * sf, bottom - top);
            if (e && !e->dir) {
                char sz[24];
                if (e->size < 1024) std::snprintf(sz, sizeof sz, "%u B", e->size);
                else if (e->size < 1024 * 1024) std::snprintf(sz, sizeof sz, "%.1f KB", e->size / 1024.0);
                else std::snprintf(sz, sizeof sz, "%.2f MB", e->size / 1024.0 / 1024.0);
                textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
                text(x0 + (kDriveW - 18) * sf, y + kRowH * sf / 2, sz, nullptr);
            }
        }
        resetScissor();

        // nome della nuova cartella
        if (fMkdirMode) {
            beginPath();
            rect(x0 + 14 * sf, bottom - 28 * sf, (kDriveW - 28) * sf, 24 * sf);
            fillColor(hexc(cRow));
            fill();
            beginPath();
            rect(x0 + 14.5f * sf, bottom - 27.5f * sf, (kDriveW - 29) * sf, 23 * sf);
            strokeColor(hexc(cAccent));
            strokeWidth(1);
            stroke();
            textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
            fillColor(hexc(cFg));
            fontSize(11 * sf);
            const std::string t = "name: " + fMkdirText + (std::fmod(nowMs(), 1000) < 500 ? "_" : " ");
            text(x0 + 20 * sf, bottom - 16 * sf, t.c_str(), nullptr);
        }

        beginPath();
        rect(x0 + 14 * sf, H - (kBottom - 4) * sf, (kDriveW - 28) * sf, 1);
        fillColor(hexc(cLine));
        fill();
        for (int i = 0; i < kNBtn; ++i) {
            const Btn b = btn(i, H);
            const bool confirm = i == kBtnDelete && fDelConfirmUntil > 0;
            flatButton(b, btnEnabled(i, s), confirm, confirm ? "CONFIRM?" : b.label, sf, x0);
        }
        const float py0 = H - kBottom * sf + 12 * sf + 62 * sf;
        beginPath();
        rect(x0 + 14 * sf, py0, (kDriveW - 28) * sf, 2);
        fillColor(hexc(cRow));
        fill();
        beginPath();
        rect(x0 + 14 * sf, py0, float(kDriveW - 28) * sf * float(std::max(0.0, std::min(1.0, s.progress))), 2);
        fillColor(hexc(cAccent));
        fill();
        fontSize(10 * sf);
        textAlign(ALIGN_LEFT | ALIGN_TOP);
        fillColor(s.error ? hexc(cErr) : hexc(cDim));
        textBox(x0 + 14 * sf, py0 + 8 * sf, (kDriveW - 28) * sf, s.message.c_str(), nullptr);
    }

    void drawStatusBar(float W, float H)
    {
        const float sf = getScaleFactor();
        const auto& sc = arl::kStatusColor;
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(11 * sf);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        fillColor(fNoteUntil > nowMs() && fNoteError ? hexc(cErr) : Color(sc.r, sc.g, sc.b));
        const float ty = H - fStatusH / 2 + 2 * sf;
        scissor(0, H - fStatusH, std::max(0.0f, W - 356 * sf), fStatusH);
        if (!fStatus.empty()) text(10 * sf, ty, fStatus.c_str(), nullptr);
        resetScissor();
        fView.place(W - 346 * sf, H - fStatusH + 3 * sf, 128 * sf, fStatusH - 5 * sf);
        fView.drawButton(*this, sf);
        fView.drawMenu(*this, sf);
        const float cur = currentZoom();
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        for (int i = 0; i < kNZooms; ++i) {
            const float zx = W - 210 * sf + i * 46 * sf;
            const bool on = std::fabs(kZooms[i] - cur) < 0.01f;
            if (on) {
                beginPath();
                roundedRect(zx, H - fStatusH + 4 * sf, 44 * sf, fStatusH - 6 * sf, 3 * sf);
                fillColor(Color(255, 255, 255, 31 / 255.0f));
                fill();
            }
            char t[8];
            std::snprintf(t, sizeof t, "%d%%", int(kZooms[i] * 100 + 0.5f));
            fillColor(on ? Color(255, 255, 255) : Color(sc.r, sc.g, sc.b));
            text(zx + 22 * sf, ty, t, nullptr);
        }
        beginPath();                                       // maniglia in basso a destra
        for (int k = 0; k < 3; ++k) {
            const float d = (4 + 4 * k) * sf;
            moveTo(W - 2 * sf, H - d);
            lineTo(W - d, H - 2 * sf);
        }
        strokeColor(Color(sc.r, sc.g, sc.b));
        strokeWidth(1);
        stroke();
    }

    // ------------------------------------------------------------ senza firmware (come la Syntakt)
    enum { kOvFolder, kOvChoose, kNOv };

    struct Card { float x, y, w, h; };

    // la scheda al centro del pannello: sempre sopra lo schermo e i tasti
    Card card() const
    {
        const float sf = float(getScaleFactor());
        const float W = areaW(), H = getHeight() - fStatusH;
        const float w = std::min(W - 24 * sf, 520 * sf), h = std::min(H - 24 * sf, 250 * sf);
        return {(W - w) / 2, (H - h) / 2, w, h};
    }

    bool overlayShown() const
    {
        return fInfo.kind != RytmViewInfo::kRunning && fInfo.kind != RytmViewInfo::kStarting;
    }

    bool wantsButtons() const
    {
        return fInfo.kind == RytmViewInfo::kNoFirmware || fInfo.kind == RytmViewInfo::kError;
    }

    // i due tasti in basso nella scheda
    Card button(int i) const
    {
        const float sf = float(getScaleFactor());
        const Card c = card();
        const float bw = 168 * sf, bh = 30 * sf, gap = 12 * sf;
        const float x0 = c.x + (c.w - (2 * bw + gap)) / 2;
        return {x0 + i * (bw + gap), c.y + c.h - bh - 18 * sf, bw, bh};
    }

    int overlayHit(float x, float y) const
    {
        if (!overlayShown() || !wantsButtons()) return -1;
        for (int i = 0; i < kNOv; ++i) {
            const Card b = button(i);
            if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) return i;
        }
        return -1;
    }

    bool overlayClick(float x, float y)
    {
        if (!overlayShown()) return false;
        const int b = overlayHit(x, y);
        if (b == kOvFolder) {
            rytm::openFolder(fInfo.firmwareDir);
        } else if (b == kOvChoose) {
            fPickMode = 1;
            FileBrowserOptions opt;
            opt.title = "Choose the Analog Rytm MKII firmware (.syx)";
            openFileBrowser(opt);
        } else {
            const Card c = card();
            if (x < c.x || x > c.x + c.w || y < c.y || y > c.y + c.h) return false;     // fuori scheda: i tasti sotto
        }
        return true;
    }

    void installFirmware(const std::string& path)
    {
        std::string err;
        const bool ok = fPlugin->installFirmware(path, err);
        fNote = ok ? "Firmware copied to the folder: the engine is starting" : err;
        fNoteUntil = nowMs() + (ok ? 5000 : 9000);
        fNoteError = !ok;
        repaint();
    }

    void drawOverlay()
    {
        if (!overlayShown()) return;
        const float sf = float(getScaleFactor());
        const Card c = card();
        beginPath();
        rect(0, 0, areaW(), getHeight() - fStatusH);
        fillColor(Color(0, 0, 0, 120 / 255.0f));
        fill();
        beginPath();
        roundedRect(c.x, c.y, c.w, c.h, 8 * sf);
        fillColor(hexc(cBg, 245));
        fill();
        beginPath();
        roundedRect(c.x + 0.5f, c.y + 0.5f, c.w - 1, c.h - 1, 8 * sf);
        strokeColor(hexc(cLine));
        strokeWidth(1);
        stroke();

        const char* title = "";
        std::string body;
        switch (fInfo.kind) {
        case RytmViewInfo::kNoFirmware:
            title = "Analog Rytm MKII firmware not found";
            body = "This plugin does not include the Elektron firmware. Put your Analog Rytm MKII firmware file (.syx) in the folder below and the engine starts by itself.";
            break;
        case RytmViewInfo::kBooting:
            title = "First start";
            body = "Preparing the machine from the firmware (this happens once, then it is cached). " +
                   std::to_string(int(fInfo.bootSeconds)) + " s elapsed.";
            break;
        case RytmViewInfo::kBusy:
            title = "Analog Rytm MKII already open";
            body = "There is only one machine, like the hardware: this instance stays silent and starts by itself when you close the other one.";
            break;
        default:
            title = "The engine cannot start";
            body = fInfo.text;
            break;
        }
        const float tx = c.x + 22 * sf, tw = c.w - 44 * sf;
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(16 * sf);
        textAlign(ALIGN_LEFT | ALIGN_TOP);
        fillColor(hexc(cAccent));
        text(tx, c.y + 18 * sf, title, nullptr);
        fontSize(11.5f * sf);
        fillColor(hexc(cFg));
        textBox(tx, c.y + 48 * sf, tw, body.c_str(), nullptr);
        float y = c.y + 48 * sf + 16 * sf * (1 + float(body.size()) * 6.5f * sf / std::max(1.0f, tw));
        if (fInfo.kind == RytmViewInfo::kBooting) {          // la fase e l'avanzamento (arcore)
            const std::string ph = fInfo.phase.empty() ? std::string()
                : fInfo.phase + " (" + std::to_string(int(fInfo.progress * 100 + 0.5)) + "%)";
            fontSize(10 * sf);
            fillColor(hexc(cDim));
            text(tx, y + 6 * sf, ph.c_str(), nullptr);
            beginPath();
            rect(tx, y + 26 * sf, tw, 4 * sf);
            fillColor(hexc(cRow));
            fill();
            beginPath();
            rect(tx, y + 26 * sf, tw * float(std::max(0.0, std::min(1.0, fInfo.progress))), 4 * sf);
            fillColor(hexc(cAccent));
            fill();
        }
        if (wantsButtons()) {
            fontSize(10 * sf);
            fillColor(hexc(cDim));
            text(tx, y, "Firmware folder:", nullptr);
            fontSize(11 * sf);
            fillColor(hexc(cAccent));
            textBox(tx, y + 14 * sf, tw, fInfo.firmwareDir.c_str(), nullptr);
            y += 14 * sf + 16 * sf * (1 + float(fInfo.firmwareDir.size()) * 6.5f * sf / std::max(1.0f, tw));
            if (!fInfo.warning.empty()) {
                fontSize(10 * sf);
                fillColor(hexc(cErr));
                textBox(tx, y + 4 * sf, tw, fInfo.warning.c_str(), nullptr);
            }
            static const char* labels[kNOv] = {"Open folder", "Choose firmware..."};
            for (int i = 0; i < kNOv; ++i) {
                const Card b = button(i);
                beginPath();
                rect(b.x, b.y, b.w, b.h);
                fillColor(i == kOvChoose ? hexc(cAccent) : hexc(cRow));
                fill();
                beginPath();
                rect(b.x + 0.5f, b.y + 0.5f, b.w - 1, b.h - 1);
                strokeColor(hexc(cLine));
                strokeWidth(1);
                stroke();
                fontSize(11.5f * sf);
                textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
                fillColor(i == kOvChoose ? hexc(cBg) : hexc(cFg));
                text(b.x + b.w / 2, b.y + b.h / 2, labels[i], nullptr);
                textAlign(ALIGN_LEFT | ALIGN_TOP);
            }
        }
    }

    RytmPlugin* fPlugin = nullptr;
    std::unique_ptr<SeqView> fSeq;
    ViewMenu fView;                                        // Classic / Elektron Emu
    void applyView()
    {
        fSeqMode = fView.mode != ViewMenu::kClassic;
        fSeq->setStudio(fView.mode == ViewMenu::kEmu);
    }
    bool fSeqMode = true;                                  // Elektron Emu (not the panel photo)
    NanoImage fPhoto, fScreenImg;
    std::string fPhotoPath;
    std::map<std::string, std::unique_ptr<NanoImage>> fGlow;
    std::vector<std::vector<std::pair<int, float>>> fWy, fWx;
    std::vector<uint8_t> fScreenPx;
    int fScreenW = 0, fScreenH = 0;
    bool fScreenDirty = true, fBig = false;
    uint8_t fScreen[128 * 64] = {};
    uint8_t fLit[128] = {};
    uint8_t fPads[12][3] = {};
    uint64_t fVersion = ~0ull;

    // firmware mancante / primo avvio / errore
    RytmViewInfo fInfo;
    int fPickMode = 0;                         // 0 = WAV del +Drive, 1 = firmware

    // +DRIVE
    std::shared_ptr<DriveModel> fDrive;
    bool fDriveOpen = false, fMkdirMode = false;
    int fDriveScroll = 0, fLastClickRow = -2;
    double fLastClickT = 0, fDelConfirmUntil = 0;
    uint64_t fDriveVersion = ~0ull;
    std::string fMkdirText;
    std::set<std::string> fSel;

    float fS = 0, fOffX = 0, fOffY = 0, fStatusH = 22;
    double fVolume = 1.0, fVolShowUntil = 0;
    std::set<int> fLatched;
    Hit fDown;
    double fDragY = 0, fDragT = 0, fDragAcc = 0;
    bool fDragMoved = false, fHand = false;
    double fReleaseLatchedAt = 0;
    std::vector<std::pair<double, int>> fPushRelease;
    bool fGrip = false;
    float fGripX = 0, fGripW = 0;
    std::string fStatus, fNote;
    double fNoteUntil = 0;
    bool fNoteError = false;

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RytmUI)
};

UI* createUI()
{
    return new RytmUI();
}

END_NAMESPACE_DISTRHO
