// DigitaktUI.cpp -- la faccia del Digitakt II nel VST3: la stessa dell'AU
// (au/Extension/PanelView.swift, a sua volta la GUI v2 emu/guiv2.py), disegnata
// con NanoVG. Layout dal JSON condiviso (vst/scripts/gen-layout.py),
// foto del pannello (dal bundle o dalla cartella dell'utente; schematico se manca),
// richiesta del firmware quando manca, schermo OLED con la
// media d'area, bagliori dei LED, LED di pagina, Main Volume, tasti con FUNC/TRK
// agganciati, encoder con trascinamento e rotella.
//
// Misura: il pannello scala con proporzioni fisse ed e' centrato. La finestra si
// puo' trascinare dall'angolo (se l'host lo permette) e i tasti 75-150% nella
// riga di stato chiedono all'host una nuova misura (setSize -> in VST3
// IPlugFrame::resizeView): e' la prova che con l'AUv3 Live non permetteva.
#include "DistrhoUI.hpp"
#include "DistrhoPluginUtils.hpp"
#include "DigitaktPlugin.hpp"
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
#include <shellapi.h>
#include <shlobj.h>
#else
#include <spawn.h>
#include <sys/wait.h>
extern char** environ;
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

// emu/panel_leds.py brightness
double brightness(uint8_t v)
{
    const int hi = v >> 4, lo = v & 0x0F;
    if (hi > 0) return std::min(1.0, hi / 7.0);
    return lo <= 1 ? 0.0 : 0.15 * lo / 15.0 + 0.05;
}

// file dello zoom scelto (vale per le prossime aperture, come le preferenze dell'AU)
std::string zoomFile()
{
#if defined(_WIN32)
    const char* a = std::getenv("LOCALAPPDATA");
    return a ? std::string(a) + "\\dtcore\\vst-zoom.txt" : std::string();
#elif defined(__APPLE__)
    const char* h = std::getenv("HOME");
    return h ? std::string(h) + "/Library/Caches/dtcore/vst-zoom.txt" : std::string();
#else
    const char* h = std::getenv("HOME");
    return h ? std::string(h) + "/.cache/dtcore/vst-zoom.txt" : std::string();
#endif
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
    if (FILE* f = p.empty() ? nullptr : std::fopen(p.c_str(), "w")) {
        std::fprintf(f, "%.3f\n", z);
        std::fclose(f);
    }
}

bool isLatching(int code)
{
    for (int c : dtl::kLatching) if (c == code) return true;
    return false;
}

// la cartella nel Finder / in Esplora risorse
void openFolder(const std::string& dir)
{
    if (dir.empty()) return;
#if defined(_WIN32)
    ShellExecuteA(nullptr, "open", dir.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#else
#if defined(__APPLE__)
    const char* tool = "/usr/bin/open";
#else
    const char* tool = "xdg-open";
#endif
    char* argv[] = {const_cast<char*>(tool), const_cast<char*>(dir.c_str()), nullptr};
    pid_t pid;
    if (posix_spawnp(&pid, tool, nullptr, nullptr, argv, environ) == 0) {
        std::thread([pid] { int st; waitpid(pid, &st, 0); }).detach();
    }
#endif
}

// the keys of the Digitakt II for the sequencer view (SeqView.hpp): codes of
// digitakt2_panel.json, read on the panel photo
SeqSpec seqSpec()
{
    static const char* const kNames[16] = {"Track 1", "Track 2", "Track 3", "Track 4", "Track 5", "Track 6", "Track 7", "Track 8",
                                           "Track 9", "Track 10", "Track 11", "Track 12", "Track 13", "Track 14", "Track 15", "Track 16"};
    static const char* const kPages[6] = {"TRIG", "SRC", "FLTR", "AMP", "FX", "MOD"};
    static const char* const kEnc[8] = {"A", "B", "C", "D", "E", "F", "G", "H"};
    SeqSpec s;
    s.title = "Digitakt II";
    s.tracks = 16;
    for (int i = 0; i < 16; ++i) { s.trackNames[i] = kNames[i]; s.trig[i] = 25 + i; }
    s.nPageKeys = 6;
    for (int i = 0; i < 6; ++i) { s.pageKeys[i] = 1 + i; s.pageNames[i] = kPages[i]; }
    s.nEnc = 8;
    for (int i = 0; i < 8; ++i) { s.enc[i] = 1 + i; s.encNames[i] = kEnc[i]; }
    s.data = 9;
    s.func = 17; s.trk = 16; s.rec = 19; s.play = 20; s.stop = 21; s.page = 22;
    s.yes = 10; s.no = 12; s.up = 11; s.down = 14; s.left = 13; s.right = 15;
    s.select = SeqSpec::kTrkTrig;
    s.nPageLeds = 8;
    s.screenMask = 1;
    s.mute = SeqSpec::kModeTrig;                                   // FUNC + TRK: mute mode
    s.muteKey = 16;
    s.ptnSel = SeqSpec::kPtnArrows;                                // PTN held, LEFT/RIGHT bank, trig
    s.ptnKey = 23;
    s.nBanks = 8;
    s.memo = "digitakt2";
    return s;
}

} // namespace

class DigitaktUI : public UI, public IdleCallback {
public:
    DigitaktUI()
        : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
    {
        fPlugin = static_cast<DigitaktPlugin*>(getPluginInstancePointer());
        loadSharedResources();
        const double sf = getScaleFactor();
        fStatusH = 22 * sf;
        applyConstraints();
        // la foto del pannello (950x777, out/gui/digitakt2_panel.png): una messa
        // dall'utente nella sua cartella vince, poi quella in Resources del bundle
        // (la copiano build-mac.sh/build-windows.sh); senza foto lo schematico
        {
            const std::string p = DigitaktPlugin::userDir() + DISTRHO_OS_SEP_STR + "digitakt2_panel.png";
            if (FILE* f = std::fopen(p.c_str(), "rb")) { std::fclose(f); fPhotoPath = p; }
        }
        if (fPhotoPath.empty())
            if (const char* b = getBundlePath())
                if (const char* r = getResourcePath(b)) {
                    const std::string p = std::string(r) + DISTRHO_OS_SEP_STR + "digitakt2_panel.png";
                    if (FILE* f = std::fopen(p.c_str(), "rb")) { std::fclose(f); fPhotoPath = p; }
                }
        SeqHost host;
        host.send = [this](int kind, int code, int arg) { send(kind, code, arg); };
        host.keyLevel = [this](int code) -> float {
            for (const auto& m : dtl::kLedToControl) if (m.code == code) return float(brightness(fLeds[m.led]));
            return 0.0f;
        };
        host.pageLevel = [this](int i) -> float {
            if (i < 0 || i >= int(sizeof dtl::kPageLeds / sizeof dtl::kPageLeds[0])) return 0.0f;
            const uint8_t v = fLeds[dtl::kPageLeds[i]];
            return (v >> 4) ? 1.0f : ((v & 0x0F) > 1 ? 0.3f : 0.0f);
        };
        host.screen = [this]() -> const uint8_t* { return fScreen; };
        fSeq.reset(new SeqView(seqSpec(), host));
        fView.load(seqSpec().memo);
        applyView();
        zoomTo(loadZoom(), false);
        addIdleCallback(this, 40);
    }

    ~DigitaktUI() override { removeIdleCallback(this); }

protected:
    // ------------------------------------------------------------ host
    void parameterChanged(uint32_t index, float value) override
    {
        if (index == DigitaktPlugin::kParamVolume) { fVolume = value; repaint(); }
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
        fillColor(Color(dtl::kBackground.r, dtl::kBackground.g, dtl::kBackground.b));
        fill();

        if (fSeqMode) {                                    // la vista sequencer (SeqView.hpp)
            fSeq->draw(*this, 0, 0, areaW(), H - fStatusH);
            drawOverlay();
            drawStatusBar(areaW(), H);
            drawDrive(W, H);
            return;
        }

        const float pw = dtl::kPanelW * fS, ph = dtl::kPanelH * fS;
        if (fPhoto.isValid()) {
            beginPath();
            rect(fOffX, fOffY, pw, ph);
            fillPaint(imagePattern(fOffX, fOffY, pw, ph, 0, fPhoto, 1.0f));
            fill();
        } else {
            drawSchematic();
        }

        drawScreen();
        drawLeds();

        // contorni dei tasti premuti e di FUNC/TRK agganciati
        for (const auto& k : dtl::kKeys) {
            const bool latched = fLatched.count(k.code) != 0;
            const bool pressed = fDown.kind == 0 && fDown.active && fDown.code == k.code;
            if (!latched && !pressed) continue;
            const auto& c = latched ? dtl::kOutlineLatched : dtl::kOutlinePressed;
            const float hw = int(k.w) / 2, hh = int(k.h) / 2;
            beginPath();
            rect(px(k.x - hw), py(k.y - hh), 2 * hw * fS, 2 * hh * fS);
            strokeColor(Color(c.r, c.g, c.b));
            strokeWidth(std::max(1.0f, dtl::kOutlineWidth * fS));
            stroke();
        }

        // tacca del Main Volume
        {
            const double a = (dtl::kAngle0 + dtl::kAngleSpan * std::min(1.0, fVolume / dtl::kVolMax)) * M_PI / 180;
            const float dx = float(std::sin(a)), dy = float(-std::cos(a));
            beginPath();
            moveTo(px(dtl::kVolX + dx * dtl::kVolR * dtl::kTickR0), py(dtl::kVolY + dy * dtl::kVolR * dtl::kTickR0));
            lineTo(px(dtl::kVolX + dx * dtl::kVolR * dtl::kTickR1), py(dtl::kVolY + dy * dtl::kVolR * dtl::kTickR1));
            strokeColor(Color(dtl::kVolTick.r, dtl::kVolTick.g, dtl::kVolTick.b));
            strokeWidth(std::max(1.0f, dtl::kTickWidth * fS));
            lineCap(ROUND);
            stroke();
        }
        if (nowMs() < fVolShowUntil) {
            char t[32];
            std::snprintf(t, sizeof t, "vol %d%%", int(std::lround(fVolume / dtl::kVolMax * 100)));
            fontFace(NANOVG_DEJAVU_SANS_TTF);
            fontSize(std::max(6.0f, std::round(11 * fS)));
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            fillColor(Color(dtl::kVolText.r, dtl::kVolText.g, dtl::kVolText.b));
            text(px(dtl::kVolX), py(dtl::kVolY + dtl::kVolR + dtl::kVolTextDY), t, nullptr);
        }

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
            if (overlayClick(x, y)) return true;
            if (fSeqMode) {
                fSeq->mouse(true, x, y, nowMs());
                repaint();
                return true;
            }
            const float qx = (x - fOffX) / fS, qy = (y - fOffY) / fS;
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
        if (d.kind == 0) {
            if (!isLatching(d.code)) {
                send(1, d.code);
                if (!fLatched.empty()) fReleaseLatchedAt = nowMs() + dtl::kAutoReleaseMs;
            }
        } else if (d.kind == 1 && !fDragMoved) {
            send(0, d.push);
            fPushRelease.push_back({nowMs() + dtl::kPushReleaseMs, d.push});
        }
        repaint();
        return true;
    }

    bool onMotion(const MotionEvent& ev) override
    {
        const float x = ev.pos.getX(), y = ev.pos.getY();
        if (fGrip) {
            const double sf = getScaleFactor();
            const float w = std::max<float>(dtl::kPanelW * 0.4f * sf, fGripW + (x - fGripX));
            zoomTo(w / (dtl::kPanelW * sf), false);
            return true;
        }
        if (x >= areaW()) { driveMotion(x, y); return true; }
        if (fSeqMode) {
            if (fSeq->motion(x, y)) repaint();
            const bool over = gripHit(x, y) || zoomHit(x, y) >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return true;
        }
        const float qx = (x - fOffX) / fS, qy = (y - fOffY) / fS;
        if (!fDown.active || fDown.kind == 0) {
            const bool over = hit(qx, qy).active || zoomHit(x, y) >= 0 || gripHit(x, y);
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return false;
        }
        const double t = nowMs();
        const double dy = fDragY - qy;
        const double speed = std::fabs(dy) / std::max(1e-3, t - fDragT);
        fDragAcc += dy / dtl::kDragPx * (speed > dtl::kFastSpeed ? dtl::kFastGain : 1);
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
        const float qx = (ev.pos.getX() - fOffX) / fS, qy = (ev.pos.getY() - fOffY) / fS;
        const Hit h = hit(qx, qy);
        const double delta = ev.delta.getY();
        if (!h.active || delta == 0) return false;
        if (h.kind == 1) {
            int step = int(std::max<double>(-dtl::kWheelClamp, std::min<double>(dtl::kWheelClamp, delta)));
            if (step == 0) step = delta > 0 ? 1 : -1;
            step *= dtl::kWheelGain;
            if (ev.mod & kModifierShift) step *= dtl::kShiftMult;
            turn(h.code, step);
        } else if (h.kind == 2) {
            volumeBy(delta > 0 ? dtl::kWheelGain : -dtl::kWheelGain);
        }
        return true;
    }

    bool onKeyboard(const KeyboardEvent& ev) override
    {
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
        if (fPickingFw) {
            fPickingFw = false;
            if (!filename || !*filename || !fPlugin) return;
            std::string msg;
            const bool ok = fPlugin->installFirmware(filename, msg);
            fNote = ok ? "Firmware copied to the folder: the engine is starting" : msg;
            fNoteUntil = nowMs() + (ok ? 5000 : 9000);
            fNoteError = !ok;
            repaint();
            return;
        }
        if (filename && *filename && fDrive) fDrive->upload({filename});
    }

    // ------------------------------------------------------------ 25 Hz
    void idleCallback() override
    {
        if (!fPlugin) return;
        const double t = nowMs();
        bool dirty = false;
        const uint64_t v = fPlugin->copyScreen(fScreen, fLeds);
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
        const EngineStatus es = fPlugin->status();
        std::string st = es.text;
        if (es.kind == EngineStatus::Running) st = "Digitakt II - emulator (VST3)";
        else if (es.kind != EngineStatus::Error) st = "Digitakt II - emulator (VST3)";
        if (!es.warning.empty() && es.kind == EngineStatus::Running) st += " - " + es.warning;
        if (fNoteUntil > t) st = fNote;
        if (st != fStatus) { fStatus = st; dirty = true; }
        if (fDriveOpen && fDrive) {
            const uint64_t dv = fDrive->snapshot().version;
            if (dv != fDriveVersion) { fDriveVersion = dv; dirty = true; }
        }
        if (fDelConfirmUntil > 0 && t >= fDelConfirmUntil) { fDelConfirmUntil = 0; dirty = true; }
        if (es.kind != fEngine.kind || es.firmwareDir != fEngine.firmwareDir || es.warning != fEngine.warning ||
            (es.kind == EngineStatus::Booting && int(es.bootSeconds) != int(fEngine.bootSeconds)))
            dirty = true;
        fEngine = es;
        if (dirty) repaint();
    }

private:
    struct Hit { bool active = false; int kind = 0, code = 0, push = 0; };   // kind: 0 tasto, 1 encoder, 2 volume

    static constexpr float kTab = 24, kDriveW = 330;
    float extraW() const { return (kTab + (fDriveOpen ? kDriveW : 0)) * float(getScaleFactor()); }
    float areaW() const { return float(getWidth()) - extraW(); }

    float px(float x) const { return fOffX + x * fS; }
    float py(float y) const { return fOffY + y * fS; }

    void relayout()
    {
        const float w = areaW(), h = getHeight() - fStatusH;
        if (w <= 2 || h <= 2) return;
        float sc = std::max<float>(dtl::kMinScale, std::min(w / dtl::kPanelW, h / dtl::kPanelH));
        if (std::fabs(sc - 1.0f) < 0.004f) sc = 1.0f;
        fS = sc;
        fOffX = std::round((w - dtl::kPanelW * fS) / 2);
        fOffY = std::round((h - dtl::kPanelH * fS) / 2);
        fScreenDirty = true;
        fGlow.clear();
        repaint();
    }

    void zoomTo(float z, bool remember)
    {
        const double sf = getScaleFactor();
        const uint w = uint(std::lround(dtl::kPanelW * z * sf + extraW()));
        const uint h = uint(std::lround(dtl::kPanelH * z * sf + fStatusH));
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

    Hit hit(float x, float y) const
    {
        Hit h;
        for (const auto& k : dtl::kKeys)
            if (std::fabs(x - k.x) <= int(k.w) / 2 && std::fabs(y - k.y) <= int(k.h) / 2) {
                h.active = true; h.kind = 0; h.code = k.code; return h;
            }
        for (const auto& e : dtl::kEncoders)
            if ((x - e.x) * (x - e.x) + (y - e.y) * (y - e.y) <= e.r * e.r) {
                h.active = true; h.kind = 1; h.code = e.turn; h.push = e.push; return h;
            }
        if ((x - dtl::kVolX) * (x - dtl::kVolX) + (y - dtl::kVolY) * (y - dtl::kVolY) <= dtl::kVolR * dtl::kVolR) {
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
        send(3, code, std::max(-dtl::kTurnClamp, std::min(dtl::kTurnClamp, steps)));
    }

    void volumeBy(int steps)
    {
        fVolume = std::max(0.0, std::min<double>(dtl::kVolMax, fVolume + dtl::kVolumeStep * steps));
        if (fPlugin) fPlugin->setMasterFromUI(fVolume);
        editParameter(DigitaktPlugin::kParamVolume, true);
        setParameterValue(DigitaktPlugin::kParamVolume, float(fVolume));
        editParameter(DigitaktPlugin::kParamVolume, false);
        fVolShowUntil = nowMs() + dtl::kVolHideMs;
        repaint();
    }

    // ------------------------------------------------------------ senza firmware
    // la scheda della Syntakt (syntakt/emulator/vst/SyntaktUI.cpp, il riferimento):
    // firmware assente, primo avvio, macchina occupata, motore fermo
    enum { kCardFolder, kCardChoose, kCardBtns };
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
        return fEngine.kind != EngineStatus::Running && fEngine.kind != EngineStatus::Starting;
    }

    bool wantsButtons() const
    {
        return fEngine.kind == EngineStatus::NoFirmware || fEngine.kind == EngineStatus::Error;
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
        for (int i = 0; i < kCardBtns; ++i) {
            const Card b = button(i);
            if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) return i;
        }
        return -1;
    }

    bool overlayClick(float x, float y)
    {
        if (!overlayShown() || x >= areaW()) return false;
        const int b = overlayHit(x, y);
        if (b == kCardFolder) {
            openFolder(fEngine.firmwareDir);
        } else if (b == kCardChoose) {
            FileBrowserOptions opt;
            opt.title = "Choose the Digitakt II firmware (.syx)";
            fPickingFw = true;
            if (!openFileBrowser(opt)) fPickingFw = false;
        } else {
            const Card c = card();
            if (x < c.x || x > c.x + c.w || y < c.y || y > c.y + c.h) return false;     // fuori scheda: i tasti sotto
        }
        return true;
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
        switch (fEngine.kind) {
        case EngineStatus::NoFirmware:
            title = "Digitakt II firmware not found";
            body = "This plugin does not include the Elektron firmware. Put your Digitakt II firmware file (OS 1.16, .syx) in the folder below and the engine starts by itself.";
            break;
        case EngineStatus::Booting:
            title = "First start";
            body = "Preparing the machine from the firmware (this happens once, then it is cached). " +
                   std::to_string(int(fEngine.bootSeconds)) + " s elapsed.";
            break;
        case EngineStatus::Busy:
            title = "Digitakt II already open";
            body = "There is only one machine, like the hardware: this instance stays silent and starts by itself when you close the other one.";
            break;
        default:
            title = "The engine cannot start";
            body = fEngine.text;
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
        if (wantsButtons()) {
            fontSize(10 * sf);
            fillColor(hexc(cDim));
            text(tx, y, "Firmware folder:", nullptr);
            fontSize(11 * sf);
            fillColor(hexc(cAccent));
            textBox(tx, y + 14 * sf, tw, fEngine.firmwareDir.c_str(), nullptr);
            y += 14 * sf + 16 * sf * (1 + float(fEngine.firmwareDir.size()) * 6.5f * sf / std::max(1.0f, tw));
            if (!fEngine.warning.empty()) {
                fontSize(10 * sf);
                fillColor(hexc(cErr));
                textBox(tx, y + 4 * sf, tw, fEngine.warning.c_str(), nullptr);
            }
            static const char* labels[kCardBtns] = {"Open folder", "Choose firmware..."};
            for (int i = 0; i < kCardBtns; ++i) {
                const Card b = button(i);
                beginPath();
                rect(b.x, b.y, b.w, b.h);
                fillColor(i == kCardChoose ? hexc(cAccent) : hexc(cRow));
                fill();
                beginPath();
                rect(b.x + 0.5f, b.y + 0.5f, b.w - 1, b.h - 1);
                strokeColor(hexc(cLine));
                strokeWidth(1);
                stroke();
                fontSize(11.5f * sf);
                textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
                fillColor(i == kCardChoose ? hexc(cBg) : hexc(cFg));
                text(b.x + b.w / 2, b.y + b.h / 2, labels[i], nullptr);
                textAlign(ALIGN_LEFT | ALIGN_TOP);
            }
        }
    }

    // emu/guiv2.py _background, ramo senza foto
    void drawSchematic()
    {
        auto box = [&](float x, float y, float w, float h, dtl::RGB f, dtl::RGB s, bool oval) {
            beginPath();
            if (oval) ellipse(px(x + w / 2), py(y + h / 2), w / 2 * fS, h / 2 * fS);
            else rect(px(x), py(y), w * fS, h * fS);
            fillColor(Color(f.r, f.g, f.b));
            fill();
            strokeColor(Color(s.r, s.g, s.b));
            strokeWidth(1);
            stroke();
        };
        box(165, 110, 295, 190, {5, 5, 6}, {0x33, 0x33, 0x33}, false);
        for (const auto& k : dtl::kKeys) {
            const float hw = int(k.w) / 2, hh = int(k.h) / 2;
            box(k.x - hw, k.y - hh, 2 * hw, 2 * hh, {0x22, 0x23, 0x28}, {0x3a, 0x3c, 0x42}, false);
        }
        for (const auto& e : dtl::kEncoders)
            box(e.x - e.r, e.y - e.r, 2 * e.r, 2 * e.r, {0x18, 0x19, 0x1c}, {0x3a, 0x3c, 0x42}, true);
        box(dtl::kVolX - dtl::kVolR, dtl::kVolY - dtl::kVolR, 2 * dtl::kVolR, 2 * dtl::kVolR,
            {0x18, 0x19, 0x1c}, {0x3a, 0x3c, 0x42}, true);
    }

    // emu/guiv2.py _draw_screen: 128x64 -> w x h con la media d'area
    void drawScreen()
    {
        const int w = std::max(1, int(std::lround(dtl::kScreenW * fS)));
        const int h = std::max(1, int(std::lround(dtl::kScreenH * fS)));
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
            const float lut[2][3] = {{float(dtl::kScreenOff.r), float(dtl::kScreenOff.g), float(dtl::kScreenOff.b)},
                                     {float(dtl::kScreenOn.r), float(dtl::kScreenOn.g), float(dtl::kScreenOn.b)}};
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
                    float r = 0, g = 0, b = 0;
                    for (const auto& sw : fWx[x]) {
                        const float* t = &tmp[(size_t(y) * 128 + sw.first) * 3];
                        r += sw.second * t[0]; g += sw.second * t[1]; b += sw.second * t[2];
                    }
                    uint8_t* p = &fScreenPx[(size_t(y) * w + x) * 4];
                    p[0] = uint8_t(std::max(0.0f, std::min(255.0f, r + 0.5f)));
                    p[1] = uint8_t(std::max(0.0f, std::min(255.0f, g + 0.5f)));
                    p[2] = uint8_t(std::max(0.0f, std::min(255.0f, b + 0.5f)));
                    p[3] = 255;
                }
            fScreenImg.update(fScreenPx.data());
        }
        const float x = px(dtl::kScreenX), y = py(dtl::kScreenY);
        beginPath();
        rect(x, y, w, h);
        fillPaint(imagePattern(x, y, w, h, 0, fScreenImg, 1.0f));
        fill();
    }

    // emu/guiv2.py _glow: bagliore w x h a intensita' level/(levels-1), bordi sfumati
    NanoImage& glow(float w, float h, int level, bool round, dtl::RGB c)
    {
        const int gw = std::max(1, int(std::lround(w * fS))), gh = std::max(1, int(std::lround(h * fS)));
        const double soft = std::round(std::max(1.0, dtl::kGlowSoft * fS) * 100) / 100;
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
                    edge = std::min(1.0, std::max(0.0, (1.0 - std::sqrt(dx * dx + dy * dy)) * dtl::kRoundEdge));
                } else {
                    const double m = std::min(std::min(x, y), std::min(gw - 1 - x, gh - 1 - y)) / soft;
                    edge = std::min(1.0, std::max(0.0, m));
                }
                const uint8_t a = uint8_t(edge * dtl::kGlowAlpha * level / (dtl::kLevels - 1));
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

    void drawLeds()
    {
        for (const auto& m : dtl::kLedToControl) {
            const dtl::Key* k = nullptr;
            for (const auto& kk : dtl::kKeys) if (kk.code == m.code) { k = &kk; break; }
            if (!k) continue;
            const int level = int(std::nearbyint(brightness(fLeds[m.led]) * (dtl::kLevels - 1)));
            if (level <= 0) continue;
            dtl::RGB c = dtl::kLedDefault;
            for (const auto& lc : dtl::kLedColor) if (lc.code == m.code) { c = lc.c; break; }
            drawImageAt(glow(k->w, k->h, level, false, c), px(k->x), py(k->y));
        }
        int i = 0;
        for (int idx : dtl::kPageLeds) {
            const uint8_t v = fLeds[idx];
            const int level = (v >> 4) ? dtl::kLevels - 1 : ((v & 0x0F) > 1 ? 2 : 0);
            if (level > 0)
                drawImageAt(glow(dtl::kPageSize, dtl::kPageSize, level, true, dtl::kLedDefault),
                            px(dtl::kPageXY[i][0]), py(dtl::kPageXY[i][1]));
            ++i;
        }
    }

    // ------------------------------------------------------------ +DRIVE
    // coordinate in unita' di vista (come il pannello): x0 = bordo sinistro della colonna
    struct Btn { const char* label; float x, y, w; bool accent; };
    static constexpr int kRowH = 24, kBottom = 130;

    // DGL: con i componenti interi l'alfa e' un float 0..1 (un 70 era opaco)
    static Color hexc(unsigned v, int a = 255) { return Color(int(v >> 16 & 255), int(v >> 8 & 255), int(v & 255), a / 255.0f); }
    static constexpr unsigned cBg = 0x141517, cRow = 0x1b1c1f, cLine = 0x2a2c30, cFg = 0xe6e3dc, cDim = 0x7d8189,
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
        setGeometryConstraints(uint(dtl::kPanelW * 0.4 * sf + extraW()), uint((dtl::kPanelH * 0.4 + 22) * sf), false, false);
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
        const auto& sc = dtl::kStatusColor;
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

    DigitaktPlugin* fPlugin = nullptr;
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
    bool fScreenDirty = true;
    uint8_t fScreen[128 * 64] = {};
    uint8_t fLeds[50] = {};
    uint64_t fVersion = ~0ull;

    // +DRIVE
    std::shared_ptr<DriveModel> fDrive;
    bool fDriveOpen = false, fMkdirMode = false;
    int fDriveScroll = 0, fLastClickRow = -2;
    double fLastClickT = 0, fDelConfirmUntil = 0;
    uint64_t fDriveVersion = ~0ull;
    std::string fMkdirText;
    std::set<std::string> fSel;

    float fS = 0, fOffX = 0, fOffY = 0, fStatusH = 22;
    double fVolume = 0.25, fVolShowUntil = 0;
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

    // firmware assente
    EngineStatus fEngine;
    bool fPickingFw = false, fNoteError = false;

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DigitaktUI)
};

UI* createUI()
{
    return new DigitaktUI();
}

END_NAMESPACE_DISTRHO
