// CyclesUI.cpp -- la faccia del Model:Cycles nel VST3: la foto del dispositivo
// (Contents/Resources/cycles_panel.png, la foto frontale ritagliata e "spenta" da
// vst/scripts/make-panel-photo.py: tasti accesi ricolorati, LED spenti, schermo nero)
// con sopra, disegnati con NanoVG, lo schermo OLED 128x64 vivo (media d'area) nel
// rettangolo dello schermo della foto, i LED accesi (tasti, pad, quadratini accanto
// alle manopole, LED di pagina) e i contorni dei tasti premuti (bianco) e di
// FUNC/TRACK/RETRIG agganciati (giallo). Le posizioni sono in vst/PhotoLayout.hpp
// (pixel della foto), i LED di ogni controllo e le costanti dell'interazione in
// panel_layout.hpp (da core/layout/mc_panel.json, vst/scripts/gen-layout.py).
// Senza la foto (bundle costruito senza) un pannello schematico con la stessa
// geometria. Sotto: la riga di 22 px della Syntakt con lo stato e i tasti di zoom
// 75-150% (chiedono all'host il resize) e la maniglia.
//
// Interazione come l'AU/emu/mcgui.py: FUNC/TRACK/RETRIG agganciati per una
// combinazione, clic destro/Ctrl = tieni premuto, LEVEL/DATA cliccabile,
// trascinamento e rotella sulle manopole, pad con velocity, tastiera, +/- volume.
//
// Il firmware NON e' nel plugin: finche' phase() != kRunning una scheda sopra il
// pannello dice a che punto e' il motore (con la barra % al primo avvio) e, se il
// firmware manca, mostra il percorso esatto della cartella, "Open folder" e
// "Choose firmware...". Tutti i testi visibili sono in inglese.
//
// Misura: il pannello scala con proporzioni fisse ed e' centrato (kViewW x kViewH
// = la foto, 797x532, + riga in basso). Lo zoom si ricorda in
// <dataDir>/vst-zoom.txt (la cartella di Elektron Emu/Model Cycles, mai quella di altri).
#include "DistrhoUI.hpp"
#include "DistrhoPluginUtils.hpp"
#include "CyclesPlugin.hpp"
#include "panel_layout.hpp"
#include "PhotoLayout.hpp"
#include "SeqView.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
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

// il pannello in pixel della foto; la riga in basso (zoom, avvisi) e' alta 22 px
// del dispositivo, come nella Syntakt
constexpr float kViewW = mcp::kPhotoW;
constexpr float kViewH = mcp::kPhotoH;

const float kZooms[] = {0.75f, 1.0f, 1.25f, 1.5f};
const int kNZooms = 4;

// ------------------------------------------------------------------ preferenze
// lo zoom scelto vale per le prossime aperture: un file nella cartella dati di
// Elektron Emu/Model Cycles (MC_DATA_DIR nelle prove)
std::string zoomFile()
{
    const std::string d = mcfw::dataDir();
    return d.empty() ? std::string() : d + mcfw::kSep + "vst-zoom.txt";
}

float loadZoom()
{
    float z = 1.0f;
    const std::string p = zoomFile();
    if (FILE* f = p.empty() ? nullptr : std::fopen(p.c_str(), "r")) {
        if (std::fscanf(f, "%f", &z) != 1) z = 1.0f;
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
    for (int c : mcl::kLatching) if (c == code) return true;
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

// la scelta del firmware fatta in un altro thread (finestra nativa di Windows)
struct PickResult {
    std::mutex m;
    std::string path;
};

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

// the keys of the Model:Cycles for the sequencer view (SeqView.hpp): codes of the
// panel layout JSON, read on the panel photo
SeqSpec seqSpec()
{
    static const char* const kNames[6] = {"Track 1", "Track 2", "Track 3", "Track 4", "Track 5", "Track 6"};
    static const char* const kKeysN[8] = {"MACHINES", "PUNCH", "GATE", "LFO", "RETRIG", "PATTERN", "TEMPO", "SETTINGS"};
    static const int kKeyCodes[8] = {5, 6, 7, 8, 4, 3, 14, 13};
    static const char* const kKnobN[12] = {"PITCH", "DECAY", "COLOR", "SHAPE", "SWEEP", "CONTOUR",
                                           "DELAY", "REVERB", "LFO SPD", "VOL+DIST", "SWING", "CHANCE"};
    static const int kKnobCodes[12] = {12, 0, 1, 2, 13, 4, 5, 6, 14, 8, 9, 10};    // "knobs" of mc_panel.json
    SeqSpec s;
    s.title = "Model:Cycles";
    s.tracks = 6;
    for (int i = 0; i < 6; ++i) s.trackNames[i] = kNames[i];
    for (int i = 0; i < 16; ++i) s.trig[i] = 16 + i;
    s.nPageKeys = 8;
    s.pagesTitle = "KEYS";
    for (int i = 0; i < 8; ++i) { s.pageKeys[i] = kKeyCodes[i]; s.pageNames[i] = kKeysN[i]; }
    s.nEnc = 12;
    for (int i = 0; i < 12; ++i) { s.enc[i] = kKnobCodes[i]; s.encNames[i] = kKnobN[i]; }
    s.data = mcl::kDataEnc;
    s.func = 1; s.trk = 2; s.rec = 9; s.play = 10; s.stop = 11; s.page = 15;
    s.no = 12;                                                     // BACK
    s.select = SeqSpec::kPad;                                      // a pad selects its track
    s.padBase = 1;
    s.nPageLeds = 4;
    s.screenMask = 0xFF;
    s.mute = SeqSpec::kHoldPad;                                    // FUNC held + the pad
    s.ptnSel = SeqSpec::kPtnPad;                                   // PATTERN held, pad = bank, trig
    s.ptnKey = 3;
    s.nBanks = 6;
    s.screenKnobs = false;                                         // the knobs are not on the screen
    s.memo = "cycles";
    return s;
}

} // namespace

class CyclesUI : public UI, public IdleCallback {
public:
    CyclesUI()
        : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
    {
        fPlugin = static_cast<CyclesPlugin*>(getPluginInstancePointer());
        loadSharedResources();
        // la foto del pannello nel bundle (Contents/Resources/cycles_panel.png), se c'e'
        if (const char* bp = getBundlePath())
            if (const char* r = getResourcePath(bp)) {
                const std::string ph = std::string(r) + DISTRHO_OS_SEP_STR + "cycles_panel.png";
                if (mcfw::fileExists(ph)) fPhotoPath = ph;
            }
        std::memset(fLeds, 0xFF, sizeof fLeds);
        if (fPlugin) {
            fVolume = fPlugin->outputVolume();
            fFolder = fPlugin->firmwareFolder();
        }
        applyConstraints();
        SeqHost host;
        host.send = [this](int kind, int code, int arg) { send(kind, code, arg); };
        host.keyLevel = [this](int code) -> float {
            for (const auto& k : mcl::kKeyLeds) if (k.code == code) return lit(k.led) ? 1.0f : 0.0f;
            return 0.0f;
        };
        host.pageLevel = [this](int i) -> float {
            if (i < 0 || i >= int(sizeof mcl::kPageLedIds / sizeof mcl::kPageLedIds[0])) return 0.0f;
            return lit(mcl::kPageLedIds[i]) ? 1.0f : 0.0f;
        };
        host.screen = [this]() -> const uint8_t* { return fScreen; };
        fSeq.reset(new SeqView(seqSpec(), host));
        fView.load(seqSpec().memo);
        applyView();
        zoomTo(loadZoom(), false);
        addIdleCallback(this, 33);
    }

    ~CyclesUI() override { removeIdleCallback(this); }

protected:
    // ------------------------------------------------------------ host
    void parameterChanged(uint32_t index, float value) override
    {
        if (index == CyclesPlugin::kParamVolume) { fVolume = value; fDirty = true; }
    }

    void onResize(const ResizeEvent& ev) override
    {
        UI::onResize(ev);
        relayout();
    }

    void uiScaleFactorChanged(double) override
    {
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
        if (fS <= 0) relayout();
        const float W = getWidth(), H = getHeight();
        beginPath();
        rect(0, 0, W, H);
        fillColor(Color(mcl::kBackground.r, mcl::kBackground.g, mcl::kBackground.b));
        fill();

        if (fSeqMode) {                                    // la vista sequencer (SeqView.hpp)
            fSeq->draw(*this, 0, 0, W, H - fStatusH);
            drawCard();
            drawBar(W, H);
            return;
        }

        if (fPhoto.isValid()) {
            beginPath();
            rect(fOffX, fOffY, kViewW * fS, kViewH * fS);
            fillPaint(imagePattern(fOffX, fOffY, kViewW * fS, kViewH * fS, 0, fPhoto, 1.0f));
            fill();
        } else {
            drawSchematic();
        }
        drawScreen();
        drawLeds();
        drawOutlines();
        drawVolume();
        drawCard();
        drawBar(W, H);
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
        const float x = ev.pos.getX(), y = ev.pos.getY();
        const float qx = (x - fOffX) / fS, qy = (y - fOffY) / fS;
        if (!ev.press) {
            if (ev.button != 1) return false;
            if (fGrip) { fGrip = false; saveZoom(currentZoom()); return true; }
        if (fSeqMode) { fSeq->mouse(false, x, y, nowMs()); repaint(); return true; }
            return mouseUp();
        }
        // la riga in basso: zoom e maniglia (anche con la scheda del firmware)
        if (ev.button == 1) {
            const int z = zoomHit(x, y);
            if (z >= 0) { zoomTo(kZooms[z], true); return true; }
            if (gripHit(x, y)) { fGrip = true; fGripX = x; fGripW = getWidth(); return true; }
        }
        if (cardShown()) {
            if (ev.button == 1) cardClick(x, y);
            return true;
        }
            if (fSeqMode) {
                fSeq->mouse(true, x, y, nowMs());
                repaint();
                return true;
            }
        if (!running() || y >= getHeight() - fStatusH) return false;
        if (ev.button == 2 || ev.button == 3 || (ev.button == 1 && (ev.mod & kModifierControl))) {
            latch(qx, qy);
            fDrag.kind = -1;
            return true;
        }
        if (ev.button != 1) return false;
        return mouseDown(qx, qy, y);
    }

    bool onMotion(const MotionEvent& ev) override
    {
        const float x = ev.pos.getX(), y = ev.pos.getY();
        if (fGrip) {
            const double sf = getScaleFactor();
            const float w = std::max<float>(kViewW * 0.4f * sf, fGripW + (x - fGripX));
            zoomTo(w / (kViewW * sf), false);
            return true;
        }
        if (fSeqMode) {
            if (fSeq->motion(x, y)) repaint();
            const bool over = gripHit(x, y) || zoomHit(x, y) >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return true;
        }
        const float qx = (x - fOffX) / fS, qy = (y - fOffY) / fS;
        if (fDrag.kind != 1) {
            bool over = zoomHit(x, y) >= 0 || gripHit(x, y) || cardHit(x, y) >= 0;
            if (!over && running() && y < getHeight() - fStatusH) over = hitAt(qx, qy).kind >= 0;
            if (over != fHand) { fHand = over; setCursor(over ? kMouseCursorHand : kMouseCursorArrow); }
            return false;
        }
        // trascinamento di una manopola: un passo ogni drag_px (pixel della vista)
        const double step = mcl::kDragPx * getScaleFactor();
        const int steps = int((fDrag.y - y) / step);                   // verso lo zero
        if (steps != 0) {
            turn(fDrag.code, steps);
            fDrag.y -= steps * step;
            fDrag.moved = true;
        }
        return true;
    }

    bool onScroll(const ScrollEvent& ev) override
    {
        if (fSeqMode) {
            if (!fSeq->scroll(ev.pos.getX(), ev.pos.getY(), float(ev.delta.getY()), (ev.mod & kModifierShift) != 0)) return false;
            repaint();
            return true;
        }
        if (!running()) return false;
        const float qx = (ev.pos.getX() - fOffX) / fS, qy = (ev.pos.getY() - fOffY) / fS;
        const Hit h = hitAt(qx, qy);
        if (h.kind != 1) return false;
        const double t = nowMs();
        if (t - fWheelT > 400) fWheelAcc = 0;                           // un gesto nuovo
        fWheelT = t;
        fWheelAcc += ev.delta.getY();
        const int d = int(fWheelAcc);                                   // una tacca = una riga
        if (d == 0) return true;
        fWheelAcc -= d;
        turn(h.value, d * ((ev.mod & kModifierShift) ? mcl::kShiftMult : 1));
        closeChord(mcl::kWheelReleaseMs);
        return true;
    }

    bool onKeyboard(const KeyboardEvent& ev) override
    {
        if (!running()) return false;
        if (ev.mod & (kModifierControl | kModifierSuper)) return false;   // scorciatoie dell'host
        unsigned k = ev.key;
        if (k >= 'A' && k <= 'Z') k += 32;
        if (ev.press) {
            if (const int n = padKey(k)) {
                if (fHeldPads.insert(k).second) { send(4, n, mcl::kPadKeyVelocity); fPadsDown.insert(n); fDirty = true; }
            } else if (const int code = keyCode(k)) {
                press(code);
            } else if (k == kKeyUp || k == kKeyDown) {
                turn(mcl::kDataEnc, k == kKeyUp ? 1 : -1);
            } else if (k == '+' || k == '=' || k == '-') {
                setVolume(fVolume + (k == '-' ? -mcl::kVolumeStep : mcl::kVolumeStep));
            } else {
                return false;
            }
            return true;
        }
        if (const int n = padKey(k)) {
            fHeldPads.erase(k);
            send(4, n, 0);
            fPadsDown.erase(n);
            fDirty = true;
        } else if (const int code = keyCode(k)) {
            if (fLatched.count(code)) return true;
            release(code);
            if (isLatching(code)) return true;
        } else {
            return false;
        }
        closeChord(mcl::kAutoReleaseMs);
        return true;
    }

    void uiFileBrowserSelected(const char* filename) override
    {
        if (fPickingFw) {
            fPickingFw = false;
            if (filename && *filename) installFirmware(filename);
        }
    }

    // ------------------------------------------------------------ 30 Hz
    void idleCallback() override
    {
        if (!fPlugin) return;
        const double t = nowMs();
        bool dirty = fDirty;
        fDirty = false;

        // schermo e LED: cambiano solo quando cambia la versione
        const uint64_t v = fPlugin->copyScreen(fScreen, fLeds);
        if (v != fVersion) {
            fVersion = v;
            fScreenDirty = true;
            applyLeds();
            fSeq->screenChanged();
            dirty = true;
        }
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

        // gli eventi a tempo: rilascio delle combinazioni, del push di DATA
        if (fChordDue > 0 && t >= fChordDue) { endChord(); dirty = true; }
        for (size_t i = 0; i < fPushRelease.size();) {
            if (t >= fPushRelease[i].first) {
                release(fPushRelease[i].second);
                fPushRelease.erase(fPushRelease.begin() + long(i));
                dirty = true;
            } else {
                ++i;
            }
        }

        // stato del motore e riga in basso (i testi come nella Syntakt)
        const CyclesPlugin::Phase ph = fPlugin->phase();
        const std::string st = fPlugin->statusText();
        const std::string note = fPlugin->firmwareNote();
        if (ph != fPhase || st != fSt || note != fNote) {
            if (ph == CyclesPlugin::kBooting && fPhase != CyclesPlugin::kBooting) fBootT0 = t;
            fPhase = ph; fSt = st; fNote = note;
            dirty = true;
        }
        if (fPhase == CyclesPlugin::kBooting) {
            const int secs = int((t - fBootT0) / 1000), pct = fPlugin->bootPercent();
            if (secs != fBootSecs || pct != fPct) { fBootSecs = secs; fPct = pct; dirty = true; }
        }
        std::string bar = std::string("Model:Cycles - emulator (VST3)");
        if (!fNote.empty()) bar += " - " + fNote;
        if (fNoteUntil > t) bar = fHostNote;
        if (cardShown()) bar = fSt;
        if (bar != fBar) { fBar = bar; dirty = true; }
        if (fVolShowUntil > 0 && t >= fVolShowUntil) { fVolShowUntil = 0; dirty = true; }
        if (fNoteUntil > 0 && t >= fNoteUntil) { fNoteUntil = 0; dirty = true; }

        // la scelta del firmware fatta nella finestra di Windows (altro thread)
        std::string picked;
        {
            std::lock_guard<std::mutex> g(fPick->m);
            picked.swap(fPick->path);
        }
        if (!picked.empty()) { installFirmware(picked); dirty = true; }

        if (dirty) repaint();
    }

private:
    struct Hit { int kind = -1, value = 0; };      // kind: 0 tasto, 1 manopola, 2 pad
    struct Drag { int kind = -1; int code = 0; double y = 0; bool moved = false; };   // 0 tasto, 1 manopola, 2 pad

    bool running() const { return fPlugin && fPlugin->phase() == CyclesPlugin::kRunning; }

    float px(float x) const { return fOffX + x * fS; }
    float py(float y) const { return fOffY + y * fS; }
    float currentZoom() const { return fS / float(getScaleFactor()); }

    void applyConstraints()
    {
        const double sf = getScaleFactor();
        fStatusH = 22 * float(sf);
        setGeometryConstraints(uint(kViewW * 0.4 * sf), uint((kViewH * 0.4 + 22) * sf), false, false);
    }

    void relayout()
    {
        const float w = getWidth(), h = getHeight() - fStatusH;
        if (w <= 2 || h <= 2) return;
        const float sf = float(getScaleFactor());
        float sc = std::max<float>(mcl::kMinScale * sf, std::min(w / kViewW, h / kViewH));
        for (float z : kZooms)                                      // sui 4 zoom, scala esatta
            if (std::fabs(sc / sf - z) < 0.004f) sc = z * sf;
        fS = sc;
        fOffX = std::round((w - kViewW * fS) / 2);
        fOffY = std::round((h - kViewH * fS) / 2);
        fScreenDirty = true;
        fDirty = true;
        repaint();
    }

    void zoomTo(float z, bool remember)
    {
        const double sf = getScaleFactor();
        const uint w = uint(std::lround(kViewW * z * sf));
        const uint h = uint(std::lround(kViewH * z * sf + fStatusH));
        setSize(w, h);
        if (remember) {
            saveZoom(z);
            // se l'host non ha cambiato misura lo si dice
            if (uint(getWidth()) != w) {
                fHostNote = "The host did not resize the window: the zoom applies the next time it opens";
                fNoteUntil = nowMs() + 4000;
            }
        }
        relayout();
    }

    // la riga in basso (pixel del dispositivo, come nella Syntakt): i tasti di zoom e la maniglia
    int zoomHit(float x, float y) const
    {
        const float H = getHeight(), W = getWidth(), sf = float(getScaleFactor());
        if (y < H - fStatusH) return -1;
        for (int i = 0; i < kNZooms; ++i) {
            const float zx = W - 210 * sf + i * 46 * sf;
            if (x >= zx - 2 && x <= zx + 44 * sf + 2) return i;
        }
        return -1;
    }

    bool gripHit(float x, float y) const
    {
        const float sf = float(getScaleFactor());
        return x > getWidth() - 18 * sf && y > getHeight() - 18 * sf && zoomHit(x, y) < 0;
    }

    // il font della riga in basso e della scheda: DejaVu Sans di DPF, come la Syntakt
    void dejavu(float size)
    {
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(size);
    }

    // ------------------------------------------------------------ pannello
    static Color col(const mcl::RGB& c, float a = 1.0f) { return Color(c.r, c.g, c.b, a); }

    void keyPath(const mcp::Key& k, float inset, float radius)
    {
        beginPath();
        roundedRect(px(k.x - k.w / 2 + inset), py(k.y - k.h / 2 + inset),
                    (k.w - 2 * inset) * fS, (k.h - 2 * inset) * fS, radius * fS);
    }

    // senza la foto: la stessa geometria disegnata (corpo, tasti, pad, manopole, LED)
    void drawSchematic()
    {
        beginPath();
        roundedRect(fOffX, fOffY, kViewW * fS, kViewH * fS, 26 * fS);
        fillColor(col(mcl::kC_BODY));
        fill();
        beginPath();
        rect(px(mcp::kScreenX - 4), py(mcp::kScreenY - 4), (mcp::kScreenW + 8) * fS, (mcp::kScreenH + 8) * fS);
        fillColor(col(mcl::kC_BEZEL));
        fill();
        for (const auto& k : mcp::kKeys) {
            keyPath(k, 1, 6);
            fillColor(col(mcl::kC_KEY_EDGE));
            fill();
            keyPath(k, 3, 5);
            fillColor(col(mcl::kC_KEY));
            fill();
        }
        for (const auto& k : mcp::kPads) {
            keyPath(k, 1, 7);
            fillColor(col(mcl::kC_KEY_EDGE));
            fill();
            keyPath(k, 3, 6);
            fillColor(col(mcl::kC_PAD));
            fill();
        }
        for (const auto& n : mcp::kKnobs) {
            beginPath();
            circle(px(n.x), py(n.y), n.r * fS);
            fillColor(col(n.enc == mcl::kDataEnc ? mcl::kC_DATA_BLUE : n.enc == 15 ? mcl::kC_VOL_WHITE : mcl::kC_KNOB));
            fill();
        }
        for (const auto& d : mcp::kKnobDots) {
            beginPath();
            rect(px(d.x - mcp::kDotSize / 2), py(d.y - mcp::kDotSize / 2), mcp::kDotSize * fS, mcp::kDotSize * fS);
            fillColor(col(mcl::kC_LED_OFF));
            fill();
        }
        for (const auto& xy : mcp::kPageLedXY) {
            beginPath();
            rect(px(xy[0] - mcp::kPageLedSize / 2), py(xy[1] - mcp::kPageLedSize / 2),
                 mcp::kPageLedSize * fS, mcp::kPageLedSize * fS);
            fillColor(col(mcl::kC_LED_OFF));
            fill();
        }
    }

    // mcgui.py _led_ids + _draw_leds: led_bits dice quale bit e' quale LED; i 54 LED
    // sono 8 byte con 1 = spento (invertiti)
    void applyLeds()
    {
        const int nbits = int(sizeof mcl::kLedBits / sizeof mcl::kLedBits[0]);
        for (int id = 0; id < 64; ++id) {
            const int bit = id < nbits ? mcl::kLedBits[id] : -1;
            fLit[id] = bit >= 0 && bit / 8 < 8 && ((fLeds[bit / 8] >> (bit % 8)) & 1) == 0;
        }
    }

    bool lit(int led) const { return led >= 0 && led < 64 && fLit[led]; }

    static int ledOf(const mcl::CtlLed* t, int n, int code)
    {
        for (int i = 0; i < n; ++i) if (t[i].code == code) return t[i].led;
        return -1;
    }

    // un tasto acceso come nella foto: la faccia rossa (un riquadro all'interno del
    // bordo) con un alone morbido, sopra la foto spenta (la scritta traspare appena)
    void drawLitKey(const mcp::Key& k, bool pad)
    {
        const float in = 6, x = px(k.x - k.w / 2 + in), y = py(k.y - k.h / 2 + in);
        const float w = (k.w - 2 * in) * fS, h = (k.h - 2 * in) * fS, r = 3 * fS, f = 5 * fS;
        // i colori della foto (faccia accesa ~ (228,150,145), centro dei pad piu' chiaro)
        // su una faccia spenta grigia: alfa 0.85, la scritta traspare appena
        const Color edge(246, 150, 144, 0.85f), mid = pad ? Color(248, 200, 186, 0.85f) : Color(248, 162, 156, 0.85f);
        beginPath();
        rect(x - f, y - f, w + 2 * f, h + 2 * f);
        fillPaint(boxGradient(x, y, w, h, r, f, Color(255, 130, 115, 0.45f), Color(255, 130, 115, 0.0f)));
        fill();
        beginPath();
        roundedRect(x, y, w, h, r);
        fillPaint(boxGradient(x + w * 0.3f, y + h * 0.3f, w * 0.4f, h * 0.4f, r, std::max(2.0f, w * 0.3f), mid, edge));
        fill();
    }

    void drawLedSquare(float cx, float cy, float size, Color core, Color halo)
    {
        const float s = size * fS, x = px(cx) - s / 2, y = py(cy) - s / 2, f = 4 * fS;
        beginPath();
        rect(x - f, y - f, s + 2 * f, s + 2 * f);
        fillPaint(boxGradient(x, y, s, s, 1, f, halo, Color(halo.red, halo.green, halo.blue, 0.0f)));
        fill();
        beginPath();
        rect(x + 0.5f * fS, y + 0.5f * fS, s - fS, s - fS);
        fillColor(core);
        fill();
    }

    void drawLeds()
    {
        const int nk = int(sizeof mcl::kKeyLeds / sizeof mcl::kKeyLeds[0]);
        for (const auto& k : mcp::kKeys)
            if (lit(ledOf(mcl::kKeyLeds, nk, k.code))) drawLitKey(k, false);
        const int np = int(sizeof mcl::kPadLeds / sizeof mcl::kPadLeds[0]);
        for (const auto& k : mcp::kPads)
            if (lit(ledOf(mcl::kPadLeds, np, k.code))) drawLitKey(k, true);
        const int nn = int(sizeof mcl::kKnobLeds / sizeof mcl::kKnobLeds[0]);
        for (const auto& d : mcp::kKnobDots)
            if (lit(ledOf(mcl::kKnobLeds, nn, d.enc)))
                drawLedSquare(d.x, d.y, mcp::kDotSize, col(mcl::kC_LED_ON), Color(255, 80, 60, 0.5f));
        for (int i = 0; i < 4; ++i)                      // i LED di pagina: bianco caldo come nella foto
            if (lit(mcl::kPageLedIds[i]))
                drawLedSquare(mcp::kPageLedXY[i][0], mcp::kPageLedXY[i][1], mcp::kPageLedSize,
                              Color(250, 238, 170), Color(255, 240, 160, 0.5f));
    }

    // i contorni: tasti premuti (bianco), FUNC/TRACK/RETRIG agganciati o tenuti col
    // clic destro (giallo), pad premuti, LEVEL/DATA premuto (cerchio)
    void drawOutlines()
    {
        const float lw = std::max(1.0f, mcl::kOutlineWidth * fS);
        auto outline = [&](const mcp::Key& k, const mcl::RGB& c) {
            keyPath(k, 0.5f, 6);
            strokeColor(col(c));
            strokeWidth(lw);
            stroke();
        };
        for (const auto& k : mcp::kKeys) {
            if (fChord.count(k.code) || fLatched.count(k.code)) outline(k, mcl::kOutlineChord);
            else if (fHeld.count(k.code)) outline(k, mcl::kOutlineDown);
        }
        for (const auto& k : mcp::kPads)
            if (fPadsDown.count(k.code)) outline(k, mcl::kOutlineDown);
        if (fHeld.count(mcl::kDataPush))
            for (const auto& n : mcp::kKnobs)
                if (n.enc == mcl::kDataEnc) {
                    beginPath();
                    circle(px(n.x), py(n.y), (n.r + 1.5f) * fS);
                    strokeColor(col(fLatched.count(mcl::kDataPush) ? mcl::kOutlineChord : mcl::kOutlineDown));
                    strokeWidth(lw);
                    stroke();
                }
    }

    // emu/guiv2.py _draw_screen: 128x64 -> w x h con la media d'area
    void drawScreen()
    {
        const int w = std::max(1, int(std::lround(mcp::kScreenW * fS)));
        const int h = std::max(1, int(std::lround(mcp::kScreenH * fS)));
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
            const float lut[2][3] = {{float(mcl::kScreenOff.r), float(mcl::kScreenOff.g), float(mcl::kScreenOff.b)},
                                     {float(mcl::kScreenOn.r), float(mcl::kScreenOn.g), float(mcl::kScreenOn.b)}};
            std::vector<float> tmp(size_t(h) * 128 * 3, 0.0f);
            for (int y = 0; y < h; ++y)
                for (const auto& sw : fWy[y])
                    for (int x = 0; x < 128; ++x) {
                        const float* c = lut[fScreen[sw.first * 128 + x] != 0];
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
                    uint8_t* q = &fScreenPx[(size_t(y) * w + x) * 4];
                    q[0] = uint8_t(std::max(0.0f, std::min(255.0f, r + 0.5f)));
                    q[1] = uint8_t(std::max(0.0f, std::min(255.0f, g + 0.5f)));
                    q[2] = uint8_t(std::max(0.0f, std::min(255.0f, b + 0.5f)));
                    q[3] = 255;
                }
            fScreenImg.update(fScreenPx.data());
        }
        const float x = std::round(px(mcp::kScreenX)), y = std::round(py(mcp::kScreenY));
        beginPath();
        rect(x, y, float(w), float(h));
        fillPaint(imagePattern(x, y, float(w), float(h), 0, fScreenImg, 1.0f));
        fill();
    }

    // ------------------------------------------------------------ riga in basso
    // Come la Syntakt (SyntaktUI.cpp drawStatusBar): 22 px, DejaVu Sans 11, a sinistra
    // "Model:Cycles - emulator (VST3)" (+ " - avviso"), a destra i 4 zoom e la maniglia.
    static constexpr unsigned kStatusRGB = 0x8d97a5;
    static Color hexc(unsigned v, int a = 255) { return Color(int(v >> 16 & 255), int(v >> 8 & 255), int(v & 255), a / 255.0f); }
    static constexpr unsigned cBg = 0x141517, cRow = 0x1b1c1f, cLine = 0x2a2c30, cFg = 0xe6e3dc, cDim = 0x9aa0a8,
                              cAccent = 0xe8b04a, cErr = 0xe0654f;

    void drawBar(float W, float H)
    {
        const float sf = float(getScaleFactor());
        beginPath();
        rect(0, H - fStatusH, W, fStatusH);
        fillColor(Color(mcl::kBackground.r, mcl::kBackground.g, mcl::kBackground.b));
        fill();
        dejavu(11 * sf);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        fillColor(hexc(kStatusRGB));
        const float ty = H - fStatusH / 2 + 2 * sf;
        scissor(0, H - fStatusH, std::max(0.0f, W - 356 * sf), fStatusH);
        if (!fBar.empty()) text(10 * sf, ty, fBar.c_str(), nullptr);
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
                fillColor(255, 255, 255, 31);
                fill();
            }
            char t[8];
            std::snprintf(t, sizeof t, "%d%%", int(kZooms[i] * 100 + 0.5f));
            fillColor(on ? Color(255, 255, 255) : hexc(kStatusRGB));
            text(zx + 22 * sf, ty, t, nullptr);
        }
        beginPath();                                       // maniglia in basso a destra
        for (int k = 0; k < 3; ++k) {
            const float d = (4 + 4 * k) * sf;
            moveTo(W - 2 * sf, H - d);
            lineTo(W - d, H - 2 * sf);
        }
        strokeColor(hexc(kStatusRGB));
        strokeWidth(1);
        stroke();
    }

    // "vol N%" sopra MAIN VOLUME quando cambia il volume d'uscita (tasti + / -)
    void drawVolume()
    {
        if (nowMs() >= fVolShowUntil) return;
        char t[32];
        std::snprintf(t, sizeof t, "vol %d%%", int(std::lround(fVolume * 100)));
        dejavu(std::max(6.0f, std::round(11 * fS)));
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fillColor(col(mcl::kC_LABEL));
        text(px(mcp::kVolX), py(mcp::kVolY), t, nullptr);
    }

    // ------------------------------------------------------------ scheda del firmware
    // Come la Syntakt (drawOverlay): una scheda al centro, sopra il pannello, finche' il
    // motore non gira. Qui in piu': la barra della percentuale nel primo avvio.
    struct Card { float x, y, w, h; };

    Card card() const
    {
        const float sf = float(getScaleFactor());
        const float W = float(getWidth()), H = getHeight() - fStatusH;
        const float w = std::min(W - 24 * sf, 520 * sf), h = std::min(H - 24 * sf, 250 * sf);
        return {(W - w) / 2, (H - h) / 2, w, h};
    }

    // il plugin parte come "nessun firmware" finche' non ha cercato nella cartella: la
    // scheda compare quando lo dice davvero (statusText "No firmware: ...")
    bool cardShown() const
    {
        if (fPhase == CyclesPlugin::kRunning) return false;
        if (fPhase == CyclesPlugin::kNoFirmware) return fSt.compare(0, 11, "No firmware") == 0;
        return true;
    }

    bool wantsButtons() const { return fPhase == CyclesPlugin::kNoFirmware || fPhase == CyclesPlugin::kError; }

    Card button(int i) const
    {
        const float sf = float(getScaleFactor());
        const Card c = card();
        const float bw = 168 * sf, bh = 30 * sf, gap = 12 * sf;
        const float x0 = c.x + (c.w - (2 * bw + gap)) / 2;
        return {x0 + i * (bw + gap), c.y + c.h - bh - 18 * sf, bw, bh};
    }

    int cardHit(float x, float y) const
    {
        if (!cardShown() || !wantsButtons()) return -1;
        for (int i = 0; i < 2; ++i) {
            const Card b = button(i);
            if (x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.h) return i;
        }
        return -1;
    }

    void cardClick(float x, float y)
    {
        const int b = cardHit(x, y);
        if (b == 0) openFolder(fFolder);
        else if (b == 1) pickFirmware();
    }

    void drawCard()
    {
        if (!cardShown()) return;
        const float sf = float(getScaleFactor());
        const Card c = card();
        beginPath();
        rect(0, 0, float(getWidth()), getHeight() - fStatusH);
        fillColor(0, 0, 0, 120);
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
        switch (fPhase) {
        case CyclesPlugin::kNoFirmware:
            title = "Model:Cycles firmware not found";
            body = "This plugin does not include the Elektron firmware. Put your Model:Cycles firmware file (.syx) in the folder below and the engine starts by itself.";
            break;
        case CyclesPlugin::kBooting:
            title = "First start";
            body = "Preparing the machine from the firmware (this happens once, then it is cached). " +
                   std::to_string(fBootSecs) + " s elapsed.";
            break;
        case CyclesPlugin::kBusy:
            title = "Model:Cycles already open";
            body = "There is only one machine, like the hardware: this instance stays silent and starts by itself when you close the other one.";
            break;
        default:
            title = "The engine cannot start";
            body = fSt;
            break;
        }
        const float tx = c.x + 22 * sf, tw = c.w - 44 * sf;
        dejavu(16 * sf);
        textAlign(ALIGN_LEFT | ALIGN_TOP);
        fillColor(hexc(cAccent));
        text(tx, c.y + 18 * sf, title, nullptr);
        dejavu(11.5f * sf);
        fillColor(hexc(cFg));
        textBox(tx, c.y + 48 * sf, tw, body.c_str(), nullptr);
        float y = c.y + 48 * sf + 16 * sf * (1 + float(body.size()) * 6.5f * sf / std::max(1.0f, tw));
        if (fPhase == CyclesPlugin::kBooting) {                      // la barra del primo avvio
            const float bh = 6 * sf;
            beginPath(); rect(tx, y + 4 * sf, tw, bh); fillColor(hexc(cRow)); fill();
            beginPath(); rect(tx, y + 4 * sf, tw * float(std::max(0, std::min(100, fPct))) / 100.0f, bh);
            fillColor(hexc(cAccent)); fill();
            dejavu(10 * sf);
            textAlign(ALIGN_LEFT | ALIGN_TOP);
            fillColor(hexc(cDim));
            text(tx, y + 14 * sf, (std::to_string(fPct) + "%").c_str(), nullptr);
        }
        if (wantsButtons()) {
            dejavu(10 * sf);
            fillColor(hexc(cDim));
            text(tx, y, "Firmware folder:", nullptr);
            dejavu(11 * sf);
            fillColor(hexc(cAccent));
            textBox(tx, y + 14 * sf, tw, fFolder.c_str(), nullptr);
            y += 14 * sf + 16 * sf * (1 + float(fFolder.size()) * 6.5f * sf / std::max(1.0f, tw));
            if (!fNote.empty()) {
                dejavu(10 * sf);
                fillColor(hexc(cErr));
                textBox(tx, y + 4 * sf, tw, fNote.c_str(), nullptr);
            }
            static const char* labels[2] = {"Open folder", "Choose firmware..."};
            for (int i = 0; i < 2; ++i) {
                const Card b = button(i);
                beginPath();
                rect(b.x, b.y, b.w, b.h);
                fillColor(i == 1 ? hexc(cAccent) : hexc(cRow));
                fill();
                beginPath();
                rect(b.x + 0.5f, b.y + 0.5f, b.w - 1, b.h - 1);
                strokeColor(hexc(cLine));
                strokeWidth(1);
                stroke();
                dejavu(11.5f * sf);
                textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
                fillColor(i == 1 ? hexc(cBg) : hexc(cFg));
                text(b.x + b.w / 2, b.y + b.h / 2, labels[i], nullptr);
                textAlign(ALIGN_LEFT | ALIGN_TOP);
            }
        }
    }

    void pickFirmware()
    {
#if defined(_WIN32)
        auto pick = fPick;
        std::thread([pick] {
            char buf[MAX_PATH * 4] = "";
            OPENFILENAMEA o{};
            o.lStructSize = sizeof o;
            o.lpstrFilter = "Elektron firmware (*.syx)\0*.syx\0All files\0*.*\0";
            o.lpstrFile = buf;
            o.nMaxFile = DWORD(sizeof buf);
            o.lpstrTitle = "Choose the Model:Cycles firmware (.syx)";
            o.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
            if (!GetOpenFileNameA(&o)) return;
            std::lock_guard<std::mutex> g(pick->m);
            pick->path = buf;
        }).detach();
#else
        FileBrowserOptions opt;
        opt.title = "Choose the Model:Cycles firmware (.syx)";
        fPickingFw = true;
        if (!openFileBrowser(opt)) fPickingFw = false;
#endif
    }

    void installFirmware(const std::string& path)
    {
        const std::string err = fPlugin ? fPlugin->installFirmware(path) : std::string("no plugin");
        fHostNote = err.empty() ? "Firmware copied to the folder: the engine is starting" : err;
        fNoteUntil = nowMs() + (err.empty() ? 5000 : 9000);
        fDirty = true;
        repaint();
    }

    // ------------------------------------------------------------ pannello: ingresso
    void send(int kind, int code, int arg = 0)
    {
        if (fPlugin) fPlugin->panel(kind, code, arg);
    }

    static int keyCode(unsigned k)
    {
        for (const auto& m : mcl::kKeyMap) if (m.key == k) return m.code;
        return 0;
    }

    static int padKey(unsigned k)
    {
        for (const auto& m : mcl::kPadKeys) if (m.key == k) return m.code;
        return 0;
    }

    // mcgui.py _hit: il controllo sotto il punto (coordinate della foto); i tasti con
    // 2 px di margine, le manopole un po' piu' larghe del pomello
    Hit hitAt(float qx, float qy) const
    {
        Hit r;
        auto inKey = [&](const mcp::Key& k) {
            return std::fabs(qx - k.x) <= k.w / 2 + 2 && std::fabs(qy - k.y) <= k.h / 2 + 2;
        };
        for (const auto& k : mcp::kKeys)
            if (inKey(k)) { r.kind = 0; r.value = k.code; return r; }
        for (const auto& k : mcp::kPads)
            if (inKey(k)) { r.kind = 2; r.value = k.code; return r; }
        for (const auto& n : mcp::kKnobs) {
            const float dx = qx - n.x, dy = qy - n.y, rr = n.r + 5;
            if (dx * dx + dy * dy <= rr * rr) { r.kind = 1; r.value = n.enc; return r; }
        }
        return r;
    }

    void press(int code)
    {
        if (!fHeld.insert(code).second) return;
        send(0, code);
        fDirty = true;
    }

    void release(int code)
    {
        if (!fHeld.erase(code)) return;
        send(1, code);
        fDirty = true;
    }

    // la combinazione e' fatta: FUNC/TRACK/RETRIG si rilasciano fra `ms`
    void closeChord(int ms)
    {
        if (fChord.empty()) return;
        fChordDue = nowMs() + ms;
    }

    void endChord()
    {
        fChordDue = 0;
        const std::set<int> c = fChord;
        fChord.clear();
        for (int code : c) release(code);
    }

    bool mouseDown(float qx, float qy, float winY)
    {
        fDrag = Drag();
        const Hit h = hitAt(qx, qy);
        if (h.kind < 0) return false;
        const int v = h.value;
        if (h.kind == 0) {
            if (fLatched.count(v)) {
                fLatched.erase(v);
                release(v);
            } else if (fChord.count(v)) {
                endChord();
            } else if (isLatching(v) && fChord.empty()) {
                fChord.insert(v);
                press(v);
            } else {
                press(v);
                fDrag.kind = 0;
                fDrag.code = v;
            }
        } else if (h.kind == 1) {
            fDrag.kind = 1;
            fDrag.code = v;
            fDrag.y = winY;
        } else {
            if (v < 1 || v > int(sizeof mcp::kPads / sizeof mcp::kPads[0])) return true;
            const mcp::Key& pd = mcp::kPads[v - 1];
            const double vel = std::max(mcl::kPadVelLo, std::min(mcl::kPadVelHi,
                mcl::kPadVelHi - double((qy - (pd.y - pd.h / 2)) / pd.h) * mcl::kPadVelSpan));
            send(4, v, int(vel));
            fPadsDown.insert(v);
            fDrag.kind = 2;
            fDrag.code = v;
        }
        fDirty = true;
        repaint();
        return true;
    }

    bool mouseUp()
    {
        if (fDrag.kind < 0) return false;
        const Drag d = fDrag;
        fDrag = Drag();
        if (d.kind == 0) {
            release(d.code);
        } else if (d.kind == 1) {
            // un clic su LEVEL/DATA senza trascinare: lo preme
            if (d.code == mcl::kDataEnc && !d.moved && !fLatched.count(mcl::kDataPush)) {
                press(mcl::kDataPush);
                fPushRelease.push_back({nowMs() + mcl::kDataPushMs, mcl::kDataPush});
            }
        } else {
            send(4, d.code, 0);
            fPadsDown.erase(d.code);
        }
        closeChord(mcl::kAutoReleaseMs);
        fDirty = true;
        repaint();
        return true;
    }

    // clic destro (o Ctrl-clic): un tasto, o LEVEL/DATA, resta premuto fino al prossimo
    void latch(float qx, float qy)
    {
        const Hit h = hitAt(qx, qy);
        int code;
        if (h.kind == 0) code = h.value;
        else if (h.kind == 1 && h.value == mcl::kDataEnc) code = mcl::kDataPush;
        else return;
        if (fLatched.count(code)) {
            fLatched.erase(code);
            release(code);
        } else {
            fLatched.insert(code);
            press(code);
        }
        fDirty = true;
        repaint();
    }

    // l'encoder gira (le manopole della foto non hanno tacca: nulla da ridisegnare)
    void turn(int enc, int steps)
    {
        send(3, enc, steps);
    }

    void setVolume(double v)
    {
        fVolume = std::max(0.0, std::min(1.0, std::round(v * 100) / 100));
        if (fPlugin) fPlugin->setVolumeFromUI(fVolume);
        editParameter(CyclesPlugin::kParamVolume, true);
        setParameterValue(CyclesPlugin::kParamVolume, float(fVolume));
        editParameter(CyclesPlugin::kParamVolume, false);
        fVolShowUntil = nowMs() + 1500;
        fDirty = true;
        repaint();
    }

    // ------------------------------------------------------------ stato
    CyclesPlugin* fPlugin = nullptr;
    std::unique_ptr<SeqView> fSeq;
    ViewMenu fView;                                        // Classic / Elektron Emu
    void applyView()
    {
        fSeqMode = fView.mode != ViewMenu::kClassic;
        fSeq->setStudio(fView.mode == ViewMenu::kEmu);
    }
    bool fSeqMode = true;                                  // Elektron Emu (not the panel photo)

    // la foto (caricata al primo disegno) e i LED accesi
    std::string fPhotoPath;
    NanoImage fPhoto;
    bool fLit[64] = {};
    std::set<int> fPadsDown;

    // schermo e LED
    NanoImage fScreenImg;
    std::vector<std::vector<std::pair<int, float>>> fWy, fWx;
    std::vector<uint8_t> fScreenPx;
    int fScreenW = 0, fScreenH = 0;
    bool fScreenDirty = true, fDirty = true;
    uint8_t fScreen[128 * 64] = {};
    uint8_t fLeds[8] = {};
    uint64_t fVersion = ~0ull;

    // misura
    float fS = 0, fOffX = 0, fOffY = 0;
    bool fGrip = false;
    float fGripX = 0, fGripW = 0;
    bool fHand = false;
    std::string fHostNote, fBar;
    double fNoteUntil = 0, fVolShowUntil = 0, fBootT0 = 0;
    float fStatusH = 22;
    int fBootSecs = 0;

    // ingresso
    std::set<int> fHeld, fLatched, fChord;
    std::set<unsigned> fHeldPads;
    Drag fDrag;
    double fChordDue = 0;
    std::vector<std::pair<double, int>> fPushRelease;
    double fWheelAcc = 0, fWheelT = 0;
    double fVolume = 0.25;

    // stato del motore e firmware
    CyclesPlugin::Phase fPhase = CyclesPlugin::kNoFirmware;
    std::string fSt, fNote, fFolder;
    int fPct = 0;
    bool fPickingFw = false;
    std::shared_ptr<PickResult> fPick = std::make_shared<PickResult>();

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CyclesUI)
};

UI* createUI()
{
    return new CyclesUI();
}

END_NAMESPACE_DISTRHO
