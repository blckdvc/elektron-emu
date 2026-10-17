// PhotoLayout.hpp -- dove sono i controlli sulla foto del Model:Cycles
// (cycles_panel.png, 797x532: la foto frontale ritagliata da
// vst/scripts/make-panel-photo.py). Coordinate in pixel della foto: centro,
// larghezza e altezza del tasto (bordo nero compreso) o centro e raggio della
// manopola. Misurate sulla foto (profili del bordo nero dei tasti, area delle
// manopole); i codici sono quelli del motore (core/layout/mc_panel.json: keys,
// knobs, pads), i LED vengono da panel_layout.hpp (kKeyLeds, kKnobLeds, ...).
#pragma once

namespace mcp {

constexpr float kPhotoW = 797, kPhotoH = 532;

// lo schermo OLED 128x64: l'interno della cornice nera (la foto ha "A07 TR1",
// coperto di nero nella versione spenta)
static const float kScreenX = 111.5f, kScreenY = 133, kScreenW = 100, kScreenH = 50;

struct Key { int code; float x, y, w, h; };
struct Knob { int enc; float x, y, r; };

// tasti: codice = chiave di "keys" in mc_panel.json
static const Key kKeys[] = {
    {1, 64.5f, 357, 52, 34},     // FUNC
    {4, 187.5f, 357, 52, 34},    // RETRIG
    {3, 64.5f, 416, 52, 34},     // PATTERN
    {2, 187.5f, 416, 52, 34},    // TRACK
    {12, 55.5f, 239, 34, 34},    // BACK / PAD MENU
    {13, 126.5f, 239, 34, 34},   // SETTINGS / TEMP SAVE
    {14, 197, 239, 34, 34},      // TEMPO / TAP BPM
    {9, 55.5f, 298, 34, 34},     // REC / COPY
    {10, 126.5f, 298, 34, 34},   // PLAY / CLEAR
    {11, 197, 298, 34, 34},      // STOP / PASTE
    {5, 268, 144.5f, 34, 34},    // MACHINES / PRESET MENU
    {6, 268, 205.5f, 34, 34},    // PUNCH / QUANTIZE
    {7, 268, 266.5f, 34, 34},    // GATE / CLICK
    {8, 268, 327.5f, 34, 34},    // LFO / LFO SETUP
    {15, 731.5f, 416, 52, 34},   // PAGE / FILL / SCALE
    {16, 55.5f, 475.5f, 34, 34},  // trig 1..16
    {17, 101, 475.5f, 34, 34},
    {18, 146.5f, 475.5f, 34, 34},
    {19, 192.5f, 475.5f, 34, 34},
    {20, 238, 475.5f, 34, 34},
    {21, 284, 475.5f, 34, 34},
    {22, 329.5f, 475.5f, 34, 34},
    {23, 375.5f, 475.5f, 34, 34},
    {24, 421, 475.5f, 34, 34},
    {25, 466.5f, 475.5f, 34, 34},
    {26, 512.5f, 475.5f, 34, 34},
    {27, 558, 475.5f, 34, 34},
    {28, 603.5f, 475.5f, 34, 34},
    {29, 649.5f, 475.5f, 34, 34},
    {30, 695, 475.5f, 34, 34},
    {31, 740.5f, 475.5f, 34, 34},
};

// pad T1..T6 (codice 1..6, con la velocity dall'altezza del clic)
static const Key kPads[] = {
    {1, 268, 406.5f, 53, 52}, {2, 345.5f, 406.5f, 52, 52}, {3, 422.5f, 406.5f, 52, 52},
    {4, 499.5f, 406.5f, 52, 52}, {5, 577, 406.5f, 52, 52}, {6, 654, 406.5f, 52, 52},
};

// manopole: enc = chiave di "knobs" in mc_panel.json
static const Knob kKnobs[] = {
    {3, 61.5f, 143, 21},         // LEVEL / DATA (blu)
    {12, 367, 145, 20}, {0, 460, 145, 20}, {1, 552, 145, 20}, {2, 645, 145, 20},      // PITCH DECAY COLOR SHAPE
    {13, 367, 237, 20}, {4, 460, 237, 20}, {5, 552, 237, 20}, {6, 645, 237, 20},      // SWEEP CONTOUR DELAY/REV SEND
    {14, 367, 329, 20}, {8, 460, 329, 20}, {9, 552, 329, 20}, {10, 645, 329, 20},     // LFO SPEED VOL+DIST SWING CHANCE
    {15, 732, 144, 20},          // MAIN VOLUME (bianca)
    {7, 731, 236, 20},           // REVERB SIZE
    {11, 731, 327.5f, 19},       // DELAY TIME
};

// i quadratini LED accanto alle 12 manopole dei parametri (enc -> centro), lato 9
struct Dot { int enc; float x, y; };
static const Dot kKnobDots[] = {
    {12, 330, 137}, {0, 422, 137}, {1, 514.5f, 137}, {2, 606.5f, 137},
    {13, 330, 228.5f}, {4, 422, 228.5f}, {5, 514.5f, 228.5f}, {6, 606.5f, 228.5f},
    {14, 330, 320.5f}, {8, 422, 320.5f}, {9, 514.5f, 320.5f}, {10, 606.5f, 320.5f},
};
static const float kDotSize = 9;

// i 4 LED di pagina (1:4 .. 4:4) sopra PAGE, lato 8
static const float kPageLedXY[4][2] = {{710, 381.5f}, {724, 381.5f}, {738.5f, 381.5f}, {752.5f, 381.5f}};
static const float kPageLedSize = 8;

// "vol N%" sopra MAIN VOLUME
static const float kVolX = 732, kVolY = 116;

} // namespace mcp
