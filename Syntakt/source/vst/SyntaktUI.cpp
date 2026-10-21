// SyntaktUI.cpp -- la faccia della Syntakt nel VST3: la stessa dell'AU
// (au/Extension/PanelView.swift, a sua volta la GUI v2 emu/guiv2.py), disegnata
// con NanoVG. Layout dal JSON condiviso (vst/scripts/gen-layout.py), foto del
// pannello dalle risorse del bundle (o pannello schematico), schermo OLED con la
// media d'area, bagliori dei LED nei colori della Syntakt (emu/st_leds.py), LED
// delle battute e quello sotto TRK, Main Volume, tasti con FUNC/TRK agganciati
// per una combinazione, encoder con trascinamento e rotella, tastiera del
// computer.
//
// Senza firmware (che NON e' nel plugin) la vista dice dove metterlo, con
// "Apri cartella" e "Scegli firmware...". Il +DRIVE non c'e': sulla Syntakt il
// collegamento di Transfer non e' ancora nel motore (docs/AU-PORT.md).
//
// Misura: il pannello scala con proporzioni fisse ed e' centrato. La finestra si
// puo' trascinare dall'angolo (se l'host lo permette) e i tasti 75-150% nella
// riga di stato chiedono all'host una nuova misura (setSize -> in VST3
// IPlugFrame::resizeView).
#include "DistrhoUI.hpp"
#include "DistrhoPluginUtils.hpp"
#include "SyntaktPlugin.hpp"
#include "panel_layout.hpp"
#include "SeqView.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#include <shellapi.h>
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
    const stplug::fs::path d = stplug::cacheDir();
    return d.empty() ? std::string() : stplug::utf8(d / "vst-zoom.txt");
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
    stplug::makeDirs(stplug::cacheDir());
    if (FILE* f = std::fopen(p.c_str(), "w")) {
        std::fprintf(f, "%.3f\n", z);
        std::fclose(f);
    }
}

bool isLatching(int code)
{
    for (int c : stl::kLatching) if (c == code) return true;
    return false;
}

bool isPageLed(int led)
{
    for (int l : stl::kPageLeds) if (l == led) return true;
    return false;
}

// emu/st_leds.py colore + livello: il valore VV di un LED fisico -> (rgb, livello 0..1)
stl::RGB ledRgb(uint8_t value, int led)
{
    for (const auto& k : stl::kKeyColour) if (k.led == led) return k.c;
    for (const auto& k : stl::kLedColours) if (k.led == (value >> 4)) return k.c;
    for (const auto& k : stl::kLedColours) if (k.led == 1) return k.c;
    return {235, 235, 225};
}

double ledLevel(uint8_t value, int led)
{
    if (isPageLed(led)) {
        for (const auto& p : stl::kPageLevels) if (p.value == value) return p.level;
        return 0;
    }
    const int lo = value & 0x0F;
    if (lo < stl::kLitMin) return 0;
    return std::min(1.0, 0.55 + 0.45 * double(lo - stl::kLitMin) / double(stl::kLitMax - stl::kLitMin));
}

// emu/guiv2.py _paint: il livello del bagliore (0 = niente)
int glowLevelOf(uint8_t value, int led)
{
    const int lv = int(std::nearbyint(std::sqrt(ledLevel(value, led)) * (stl::kLevels - 1)));
    if (isPageLed(led)) return lv;
    return (value & 0x0F) <= 2 ? 0 : lv;
}

// il nome di un tasto del computer ([panel.keys]) -> codice del DPF
int keyForName(const char* name)
{
    static const struct { const char* n; unsigned key; } special[] = {
        {"Return", kKeyEnter}, {"KP_Enter", kKeyEnter}, {"Escape", kKeyEscape}, {"BackSpace", kKeyBackspace},
        {"Up", kKeyUp}, {"Down", kKeyDown}, {"Left", kKeyLeft}, {"Right", kKeyRight}, {"Tab", kKeyTab},
        {"space", kKeySpace}, {"period", '.'}, {"F1", kKeyF1}, {"F2", kKeyF2}, {"F3", kKeyF3}, {"F4", kKeyF4},
        {"F5", kKeyF5}, {"F6", kKeyF6}, {"F7", kKeyF7}, {"F8", kKeyF8},
        {"Alt_L", kKeyAltL}, {"Alt_R", kKeyAltR}, {"Option_L", kKeyAltL}, {"Option_R", kKeyAltR}};
    for (const auto& s : special) if (!std::strcmp(s.n, name)) return int(s.key);
    return name[0] && !name[1] ? int(name[0]) : -1;
}

// apre la cartella nel Finder / Esplora risorse
void openFolder(const std::string& path)
{
#if defined(_WIN32)
    const stplug::fs::path p = stplug::fromUtf8(path);
    ShellExecuteW(nullptr, L"open", p.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#else
    std::string q = "'";
    for (char c : path) { if (c == '\'') q += "'\\''"; else q += c; }
    q += "'";
#if defined(__APPLE__)
    const std::string cmd = "open " + q + " >/dev/null 2>&1 &";
#else
    const std::string cmd = "xdg-open " + q + " >/dev/null 2>&1 &";
#endif
    if (std::system(cmd.c_str()) != 0) { /* niente da fare: la vista mostra comunque il percorso */ }
#endif
}

// the keys of the Syntakt for the sequencer view (SeqView.hpp): codes of the
// panel layout JSON, read on the panel photo
SeqSpec seqSpec()
{
    static const char* const kNames[12] = {"Digital 1", "Digital 2", "Digital 3", "Digital 4", "Digital 5", "Digital 6",
                                           "Digital 7", "Digital 8", "Analog 9", "Analog 10", "Analog 11", "Analog 12"};
    static const char* const kPages[6] = {"TRIG", "SYN", "FLTR", "AMP", "LFO", "FX TRACK"};
    static const char* const kEnc[8] = {"A", "B", "C", "D", "E", "F", "G", "H"};
    static const int kPageCodes[5] = {19, 20, 22, 23, 24};
    SeqSpec s;
    s.title = "Syntakt";
    s.tracks = 12;
    for (int i = 0; i < 12; ++i) s.trackNames[i] = kNames[i];
    for (int i = 0; i < 16; ++i) s.trig[i] = 1 + i;
    s.nPageKeys = 6;
    for (int i = 0; i < 5; ++i) { s.pageKeys[i] = kPageCodes[i]; s.pageNames[i] = kPages[i]; }
    s.pageKeys[5] = 26; s.pageNames[5] = kPages[5];               // the FX track
    s.nEnc = 8;
    for (int i = 0; i < 8; ++i) { s.enc[i] = 1 + i; s.encNames[i] = kEnc[i]; }
    s.data = 9;
    s.func = 18; s.trk = 17; s.rec = 36; s.play = 37; s.stop = 38; s.page = 32;
    s.yes = 39; s.no = 28; s.up = 40; s.down = 30; s.left = 29; s.right = 31;
    s.select = SeqSpec::kTrkTrig;
    s.nPageLeds = 4;
    s.screenMask = 1;
    s.mute = SeqSpec::kModeTrig;                                   // FUNC + BANK: mute mode
    s.muteKey = 33;
    s.ptnSel = SeqSpec::kPtnArrows;                                // PTN held, LEFT/RIGHT bank, trig
    s.ptnKey = 34;
    s.nBanks = 8;
    {                                                              // FUNC + key, flat
        static const char* const kFn[] = {"SETTINGS", "SONG MODE", "QUANTIZE", "MACHINE", "DELAY", "REVERB", "MIXER", "PRESETS", "ROUTING", "TAP TEMPO", "PAGE SETUP", "CLICK", "SOUND", "COPY", "CLEAR", "PASTE"};
        static const int kFk[] = {-25, -35, 19, 20, 22, 23, 24, 25, 26, 27, 32, 34, 35, 36, 37, 38};
        s.nFuncs = int(sizeof kFk / sizeof kFk[0]);
        for (int i = 0; i < s.nFuncs; ++i) { s.funcNames[i] = kFn[i]; s.funcKeys[i] = kFk[i]; }
    }
    s.memo = "syntakt";
    return s;
}

} // namespace

class SyntaktUI : public UI, public IdleCallback {
public:
    SyntaktUI()
        : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
    {
        fPlugin = static_cast<SyntaktPlugin*>(getPluginInstancePointer());
        loadSharedResources();
        const double sf = getScaleFactor();
        fStatusH = 22 * sf;
        applyConstraints();
        if (const char* b = getBundlePath())
            if (const char* r = getResourcePath(b)) {
                const std::string p = std::string(r) + DISTRHO_OS_SEP_STR + "syntakt_panel.png";
                if (FILE* f = std::fopen(p.c_str(), "rb")) { std::fclose(f); fPhotoPath = p; }
            }
        for (const auto& k : stl::kKeyboard) {
            const int dk = keyForName(k.name);
            if (dk >= 0) fKeyMap[unsigned(dk)] = k.code;
        }
        SeqHost host;
        host.send = [this](int kind, int code, int arg) { send(kind, code, arg); };
        host.keyLevel = [this](int code) -> float {
            for (const auto& m : stl::kLedToControl) if (m.code == code) return float(ledLevel(fLeds[m.led], m.led));
            return 0.0f;
        };
        host.pageLevel = [this](int i) -> float {
            if (i < 0 || i >= int(sizeof stl::kPageLeds / sizeof stl::kPageLeds[0])) return 0.0f;
            const int led = stl::kPageLeds[i];
            return float(ledLevel(fLeds[led], led));
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

    ~SyntaktUI() override { removeIdleCallback(this); }

protected:
    // ------------------------------------------------------------ host
    void parameterChanged(uint32_t index, float value) override
    {
        if (index == SyntaktPlugin::kParamVolume) { fVolume = value; repaint(); }
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
        fillColor(Color(stl::kBackground.r, stl::kBackground.g, stl::kBackground.b));
        fill();

        if (fSeqMode) {                                    // la vista sequencer (SeqView.hpp)
            fSeq->draw(*this, 0, 0, W, H - fStatusH);
            drawOverlay();
            drawStatusBar(W, H);
            return;
        }

        const float pw = stl::kPanelW * fS, ph = stl::kPanelH * fS;
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
        for (const auto& k : stl::kKeys) {
            const bool latched = fLatched.count(k.code) != 0;
            const bool pressed = (fDown.kind == 0 && fDown.active && fDown.code == k.code) || fKeysHeld.count(k.code);
            if (!latched && !pressed) continue;
            const auto& c = latched ? stl::kOutlineLatched : stl::kOutlinePressed;
            const float hw = int(k.w) / 2, hh = int(k.h) / 2;
            beginPath();
            rect(px(k.x - hw), py(k.y - hh), 2 * hw * fS, 2 * hh * fS);
            strokeColor(Color(c.r, c.g, c.b));
            strokeWidth(std::max(1.0f, stl::kOutlineWidth * fS));
            stroke();
        }

        // tacca del Main Volume
        {
            const double a = (stl::kAngle0 + stl::kAngleSpan * std::min(1.0, fVolume / stl::kVolMax)) * M_PI / 180;
            const float dx = float(std::sin(a)), dy = float(-std::cos(a));
            beginPath();
            moveTo(px(stl::kVolX + dx * stl::kVolR * stl::kTickR0), py(stl::kVolY + dy * stl::kVolR * stl::kTickR0));
            lineTo(px(stl::kVolX + dx * stl::kVolR * stl::kTickR1), py(stl::kVolY + dy * stl::kVolR * stl::kTickR1));
            strokeColor(Color(stl::kVolTick.r, stl::kVolTick.g, stl::kVolTick.b));
            strokeWidth(std::max(1.0f, stl::kTickWidth * fS));
            lineCap(ROUND);
            stroke();
        }
        if (nowMs() < fVolShowUntil) {
            char t[32];
            std::snprintf(t, sizeof t, "vol %d%%", int(std::lround(fVolume / stl::kVolMax * 100)));
            fontFace(NANOVG_DEJAVU_SANS_TTF);
            fontSize(std::max(6.0f, std::round(11 * fS)));
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            fillColor(Color(stl::kVolText.r, stl::kVolText.g, stl::kVolText.b));
            text(px(stl::kVolX), py(stl::kVolY + stl::kVolR + stl::kVolTextDY), t, nullptr);
        }

        drawOverlay();
        drawStatusBar(W, H);
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
        if (ev.press) {
            if (overlayClick(x, y)) return true;
            const int z = zoomHit(x, y);
            if (z >= 0) { zoomTo(kZooms[z], true); return true; }
            if (gripHit(x, y)) { fGrip = true; fGripX = x; fGripW = areaW(); return true; }
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
                // emu/guiv2.py _press: un tasto agganciabile si aggancia solo se non ce
                // n'e' gia' un altro (FUNC poi TRK = due clic: Keyboard on/off)
                if (isLatching(h.code) && otherLatched(h.code).empty()) {
                    if (fLatched.count(h.code)) { fLatched.erase(h.code); send(1, h.code); }
                    else { fLatched.insert(h.code); send(0, h.code); }
                } else {
                    if (isLatching(h.code)) fMomentary.insert(h.code);
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
            if (isLatching(d.code)) {
                if (!fMomentary.count(d.code)) { repaint(); return true; }
                fMomentary.erase(d.code);
            }
            send(1, d.code);
            if (!fLatched.empty()) fReleaseLatchedAt = nowMs() + stl::kAutoReleaseMs;
        } else if (d.kind == 1) {
            if (!fDragMoved) {
                send(0, d.push);
                fPushRelease.push_back({nowMs() + stl::kPushReleaseMs, d.push});
            }
            // l'encoder chiude l'accordo come un tasto (emu/guiv2.py _enc_release)
            if (!fLatched.empty()) fReleaseLatchedAt = nowMs() + stl::kPushReleaseMs + stl::kAutoReleaseMs;
        }
        repaint();
        return true;
    }

    bool onMotion(const MotionEvent& ev) override
    {
        const float x = ev.pos.getX(), y = ev.pos.getY();
        if (fGrip) {
            const double sf = getScaleFactor();
            const float w = std::max<float>(stl::kPanelW * 0.4f * sf, fGripW + (x - fGripX));
            zoomTo(w / (stl::kPanelW * sf), false);
            return true;
        }
        if (fSeqMode) {
            if (fSeq->motion(x, y, 0, (ev.mod & kModifierShift) != 0)) repaint();
            const bool over = gripHit(x, y) || zoomHit(x, y) >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return true;
        }
        const float qx = (x - fOffX) / fS, qy = (y - fOffY) / fS;
        if (!fDown.active || fDown.kind == 0) {
            const bool over = hit(qx, qy).active || zoomHit(x, y) >= 0 || gripHit(x, y) || overlayHit(x, y) >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return false;
        }
        const double t = nowMs();
        const double dy = fDragY - qy;
        const double speed = std::fabs(dy) / std::max(1e-3, t - fDragT);
        fDragAcc += dy / stl::kDragPx * (speed > stl::kFastSpeed ? stl::kFastGain : 1);
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
            int step = int(std::max<double>(-stl::kWheelClamp, std::min<double>(stl::kWheelClamp, delta)));
            if (step == 0) step = delta > 0 ? 1 : -1;
            step *= stl::kWheelGain;
            if (ev.mod & kModifierShift) step *= stl::kShiftMult;
            turn(h.code, step);
        } else if (h.kind == 2) {
            volumeBy(delta > 0 ? stl::kVolWheelSteps : -stl::kVolWheelSteps);
        }
        return true;
    }

    // tastiera del computer ([panel.keys] di devices/syntakt.toml, emu/guiv2.py
    // _on_key_press): tasto tenuto = tasto del pannello tenuto; l'auto-repeat no
    bool onKeyboard(const KeyboardEvent& ev) override
    {
        unsigned key = ev.key;
        if (key >= 'A' && key <= 'Z') key += 'a' - 'A';
        const auto it = fKeyMap.find(key);
        if (it == fKeyMap.end()) return false;
        const int code = it->second;
        if (ev.press) {
            if (fKeysDown.count(key)) return true;
            fKeysDown[key] = code;
            fKeysHeld.insert(code);
            send(0, code);
        } else {
            const auto d = fKeysDown.find(key);
            if (d == fKeysDown.end()) return false;
            fKeysHeld.erase(d->second);
            send(1, d->second);
            fKeysDown.erase(d);
        }
        repaint();
        return true;
    }

    void uiFileBrowserSelected(const char* filename) override
    {
        if (!filename || !*filename || !fPlugin) return;
        const std::string err = fPlugin->installFirmware(filename);
        fNote = err.empty() ? "Firmware copied to the folder: the engine is starting" : err;
        fNoteUntil = nowMs() + (err.empty() ? 5000 : 9000);
        fNoteError = !err.empty();
        repaint();
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
        if (es.kind == EngineStatus::Running) {
            st = (es.firmwareLabel.empty() ? std::string("Syntakt") : es.firmwareLabel) + " - emulator (VST3)";
            if (!es.warning.empty()) st += " - " + es.warning;
        }
        if (fNoteUntil > t) st = fNote;
        if (st != fStatus) { fStatus = st; dirty = true; }
        if (es.kind != fEngine.kind || es.firmwareDir != fEngine.firmwareDir || es.warning != fEngine.warning ||
            (es.kind == EngineStatus::Booting && int(es.bootSeconds) != int(fEngine.bootSeconds)))
            dirty = true;
        fEngine = es;
        if (dirty) repaint();
    }

private:
    struct Hit { bool active = false; int kind = 0, code = 0, push = 0; };   // kind: 0 tasto, 1 encoder, 2 volume

    float areaW() const { return float(getWidth()); }

    float px(float x) const { return fOffX + x * fS; }
    float py(float y) const { return fOffY + y * fS; }

    void relayout()
    {
        const float w = areaW(), h = getHeight() - fStatusH;
        if (w <= 2 || h <= 2) return;
        float sc = std::max<float>(stl::kMinScale, std::min(w / stl::kPanelW, h / panelH()));
        if (std::fabs(sc - 1.0f) < 0.004f) sc = 1.0f;
        fS = sc;
        fOffX = std::round((w - stl::kPanelW * fS) / 2);
        fOffY = std::round((h - stl::kPanelH * fS) / 2);
        fScreenDirty = true;
        fGlow.clear();
        repaint();
    }

    void zoomTo(float z, bool remember)
    {
        const double sf = getScaleFactor();
        const uint w = uint(std::lround(stl::kPanelW * z * sf));
        const uint h = uint(std::lround(panelH() * z * sf + fStatusH));
        setSize(w, h);
        if (remember) {
            saveZoom(z);
            // se l'host non ha cambiato misura (come Live con l'AUv3) lo si dice
            if (uint(getWidth()) != w) {
                fNote = "The host did not resize the window: the zoom applies the next time it opens";
                fNoteUntil = nowMs() + 4000;
                fNoteError = false;
            }
        }
        relayout();
    }

    void applyConstraints()
    {
        const double sf = getScaleFactor();
        setGeometryConstraints(uint(stl::kPanelW * 0.4 * sf), uint((stl::kPanelH * 0.4 + 22) * sf), false, false);
    }

    // lo zoom attuale: la scala vera del pannello (non la larghezza della finestra)
    float currentZoom() const { return fS / float(getScaleFactor()); }

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
        for (const auto& k : stl::kKeys)
            if (std::fabs(x - k.x) <= int(k.w) / 2 && std::fabs(y - k.y) <= int(k.h) / 2) {
                h.active = true; h.kind = 0; h.code = k.code; return h;
            }
        for (const auto& e : stl::kEncoders)
            if ((x - e.x) * (x - e.x) + (y - e.y) * (y - e.y) <= e.r * e.r) {
                h.active = true; h.kind = 1; h.code = e.turn; h.push = e.push; return h;
            }
        if ((x - stl::kVolX) * (x - stl::kVolX) + (y - stl::kVolY) * (y - stl::kVolY) <= stl::kVolR * stl::kVolR) {
            h.active = true; h.kind = 2;
        }
        return h;
    }

    std::set<int> otherLatched(int code) const
    {
        std::set<int> o = fLatched;
        o.erase(code);
        return o;
    }

    void send(int kind, int code, int arg = 0)
    {
        if (fPlugin) fPlugin->panel(kind, code, arg);
    }

    void turn(int code, int steps)
    {
        send(3, code, std::max(-stl::kTurnClamp, std::min(stl::kTurnClamp, steps)));
    }

    void volumeBy(int steps)
    {
        fVolume = std::max(0.0, std::min<double>(stl::kVolMax, fVolume + stl::kVolumeStep * steps));
        if (fPlugin) fPlugin->setMasterFromUI(fVolume);
        editParameter(SyntaktPlugin::kParamVolume, true);
        setParameterValue(SyntaktPlugin::kParamVolume, float(fVolume));
        editParameter(SyntaktPlugin::kParamVolume, false);
        fVolShowUntil = nowMs() + stl::kVolHideMs;
        repaint();
    }

    // emu/guiv2.py _background, ramo senza foto
    void drawSchematic()
    {
        auto box = [&](float x, float y, float w, float h, stl::RGB f, stl::RGB s, bool oval) {
            beginPath();
            if (oval) ellipse(px(x + w / 2), py(y + h / 2), w / 2 * fS, h / 2 * fS);
            else rect(px(x), py(y), w * fS, h * fS);
            fillColor(Color(f.r, f.g, f.b));
            fill();
            strokeColor(Color(s.r, s.g, s.b));
            strokeWidth(1);
            stroke();
        };
        box(stl::kScreenX - 4, stl::kScreenY - 4, stl::kScreenW + 8, stl::kScreenH + 8, {5, 5, 6}, {0x33, 0x33, 0x33}, false);
        for (const auto& k : stl::kKeys) {
            const float hw = int(k.w) / 2, hh = int(k.h) / 2;
            box(k.x - hw, k.y - hh, 2 * hw, 2 * hh, {0x22, 0x23, 0x28}, {0x3a, 0x3c, 0x42}, false);
        }
        for (const auto& e : stl::kEncoders)
            box(e.x - e.r, e.y - e.r, 2 * e.r, 2 * e.r, {0x18, 0x19, 0x1c}, {0x3a, 0x3c, 0x42}, true);
        box(stl::kVolX - stl::kVolR, stl::kVolY - stl::kVolR, 2 * stl::kVolR, 2 * stl::kVolR,
            {0x18, 0x19, 0x1c}, {0x3a, 0x3c, 0x42}, true);
    }

    // emu/guiv2.py _draw_screen: 128x64 -> w x h con la media d'area
    void drawScreen()
    {
        const int w = std::max(1, int(std::lround(stl::kScreenW * fS)));
        const int h = std::max(1, int(std::lround(stl::kScreenH * fS)));
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
            const float lut[2][3] = {{float(stl::kScreenOff.r), float(stl::kScreenOff.g), float(stl::kScreenOff.b)},
                                     {float(stl::kScreenOn.r), float(stl::kScreenOn.g), float(stl::kScreenOn.b)}};
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
        const float x = px(stl::kScreenX), y = py(stl::kScreenY);
        beginPath();
        rect(x, y, w, h);
        fillPaint(imagePattern(x, y, w, h, 0, fScreenImg, 1.0f));
        fill();
    }

    // emu/guiv2.py _glow: bagliore w x h a intensita' level/(levels-1), bordi sfumati
    NanoImage& glow(float w, float h, int level, bool round, stl::RGB c)
    {
        const int gw = std::max(1, int(std::lround(w * fS))), gh = std::max(1, int(std::lround(h * fS)));
        const double soft = std::round(std::max(1.0, stl::kGlowSoft * fS) * 100) / 100;
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
                    edge = std::min(1.0, std::max(0.0, (1.0 - std::sqrt(dx * dx + dy * dy)) * stl::kRoundEdge));
                } else {
                    const double m = std::min(std::min(x, y), std::min(gw - 1 - x, gh - 1 - y)) / soft;
                    edge = std::min(1.0, std::max(0.0, m));
                }
                const uint8_t a = uint8_t(edge * stl::kGlowAlpha * level / (stl::kLevels - 1));
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

    // emu/guiv2.py _show_leds
    void drawLeds()
    {
        for (const auto& m : stl::kLedToControl) {
            const stl::Key* k = nullptr;
            for (const auto& kk : stl::kKeys) if (kk.code == m.code) { k = &kk; break; }
            if (!k) continue;
            const int lv = glowLevelOf(fLeds[m.led], m.led);
            if (lv <= 0) continue;
            drawImageAt(glow(k->w, k->h, lv, false, ledRgb(fLeds[m.led], m.led)), px(k->x), py(k->y));
        }
        int i = 0;
        for (int led : stl::kPageLeds) {
            const int lv = glowLevelOf(fLeds[led], led);
            if (lv > 0)
                drawImageAt(glow(stl::kPageSize, stl::kPageSize, lv, true, ledRgb(fLeds[led], led)),
                            px(stl::kPageXY[i][0]), py(stl::kPageXY[i][1]));
            ++i;
        }
        const int lv = glowLevelOf(fLeds[stl::kTrkLed], stl::kTrkLed);       // il LED Keyboard sotto TRK
        if (lv > 0)
            drawImageAt(glow(stl::kTrkSize, stl::kTrkSize, lv, true, ledRgb(fLeds[stl::kTrkLed], stl::kTrkLed)),
                        px(stl::kTrkX), py(stl::kTrkY));
    }

    // ------------------------------------------------------------ senza firmware
    static Color hexc(unsigned v, int a = 255) { return Color(int(v >> 16 & 255), int(v >> 8 & 255), int(v & 255), a / 255.0f); }
    static constexpr unsigned cBg = 0x141517, cRow = 0x1b1c1f, cLine = 0x2a2c30, cFg = 0xe6e3dc, cDim = 0x9aa0a8,
                              cAccent = 0xe8b04a, cErr = 0xe0654f;

    enum { kBtnFolder, kBtnChoose, kNBtn };

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
        for (int i = 0; i < kNBtn; ++i) {
            const Card b = button(i);
            if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) return i;
        }
        return -1;
    }

    bool overlayClick(float x, float y)
    {
        if (!overlayShown()) return false;
        const int b = overlayHit(x, y);
        if (b == kBtnFolder) {
            openFolder(fEngine.firmwareDir);
        } else if (b == kBtnChoose) {
            FileBrowserOptions opt;
            opt.title = "Choose the Syntakt firmware (.syx)";
            openFileBrowser(opt);
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
            title = "Syntakt firmware not found";
            body = "This plugin does not include the Elektron firmware. Put your Syntakt firmware file (.syx) in the folder below and the engine starts by itself.";
            break;
        case EngineStatus::Booting:
            title = "First start";
            body = "Preparing the machine from the firmware (this happens once, then it is cached). " +
                   std::to_string(int(fEngine.bootSeconds)) + " s elapsed.";
            break;
        case EngineStatus::Busy:
            title = "Syntakt already open";
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
            text(tx, y, fEngine.kind == EngineStatus::NoFirmware ? "Firmware folder:" : "Firmware folder:", nullptr);
            fontSize(11 * sf);
            fillColor(hexc(cAccent));
            textBox(tx, y + 14 * sf, tw, fEngine.firmwareDir.c_str(), nullptr);
            y += 14 * sf + 16 * sf * (1 + float(fEngine.firmwareDir.size()) * 6.5f * sf / std::max(1.0f, tw));
            if (!fEngine.warning.empty()) {
                fontSize(10 * sf);
                fillColor(hexc(cErr));
                textBox(tx, y + 4 * sf, tw, fEngine.warning.c_str(), nullptr);
            }
            static const char* labels[kNBtn] = {"Open folder", "Choose firmware..."};
            for (int i = 0; i < kNBtn; ++i) {
                const Card b = button(i);
                beginPath();
                rect(b.x, b.y, b.w, b.h);
                fillColor(i == kBtnChoose ? hexc(cAccent) : hexc(cRow));
                fill();
                beginPath();
                rect(b.x + 0.5f, b.y + 0.5f, b.w - 1, b.h - 1);
                strokeColor(hexc(cLine));
                strokeWidth(1);
                stroke();
                fontSize(11.5f * sf);
                textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
                fillColor(i == kBtnChoose ? hexc(cBg) : hexc(cFg));
                text(b.x + b.w / 2, b.y + b.h / 2, labels[i], nullptr);
                textAlign(ALIGN_LEFT | ALIGN_TOP);
            }
        }
    }

    void drawStatusBar(float W, float H)
    {
        const float sf = getScaleFactor();
        const auto& sc = stl::kStatusColor;
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

    SyntaktPlugin* fPlugin = nullptr;
    std::unique_ptr<SeqView> fSeq;
    ViewMenu fView;                                        // Classic / Elektron Emu
    // the height of the content for the zoom: the panel photo, or the Elektron Emu
    // view (as wide as the photo, as tall as it needs)
    float panelH() const { return fSeqMode && fSeq ? stl::kPanelW * fSeq->aspect() : stl::kPanelH; }
    void applyView()
    {
        const bool was = fSeqMode;
        fSeqMode = fView.mode != ViewMenu::kClassic;
        fSeq->setStudio(fView.mode == ViewMenu::kEmu);
        if (was != fSeqMode && fS > 0) zoomTo(currentZoom(), false);   // the window follows the view
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

    EngineStatus fEngine;

    float fS = 0, fOffX = 0, fOffY = 0, fStatusH = 22;
    double fVolume = stl::kVolDefault, fVolShowUntil = 0;
    std::set<int> fLatched, fMomentary, fKeysHeld;
    std::map<unsigned, int> fKeyMap, fKeysDown;            // tasto del computer -> codice del pannello
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

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SyntaktUI)
};

UI* createUI()
{
    return new SyntaktUI();
}

END_NAMESPACE_DISTRHO
