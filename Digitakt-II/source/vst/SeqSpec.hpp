// SeqSpec.hpp -- Digitakt II: the keys of the Elektron Emu view (SeqView.hpp) and how the
// LEDs of the machine read as levels. Shared by the UI and tests/seqtest.cpp: it is
// included after SeqView.hpp and panel_layout.hpp, inside the namespace of the caller.
#pragma once

// the keys of the Digitakt II for the sequencer view (SeqView.hpp): codes of
// digitakt2_panel.json, read on the panel photo
inline SeqSpec seqSpec()
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
    {                                                              // FUNC + key, flat
        static const char* const kFn[] = {"SETTINGS", "QUANTIZE", "MACHINE", "SETUP", "EUCLID", "DELAY", "COMPRESSOR", "PERFORM KIT", "PRESETS", "TAP TEMPO", "KEYBOARD", "PAGE SETUP", "SONG", "SAMPLES", "COPY", "CLEAR", "PASTE"};
        static const int kFk[] = {-8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 18, 22, 24, 50, 19, 20, 21};
        s.nFuncs = int(sizeof kFk / sizeof kFk[0]);
        for (int i = 0; i < s.nFuncs; ++i) { s.funcNames[i] = kFn[i]; s.funcKeys[i] = kFk[i]; }
    }
    s.memo = "digitakt2";
    return s;
}

// the LEDs (dt_engine_leds / copyScreen, 50 values): high nibble lit, low nibble dim
inline float seqLedValue(uint8_t v)
{
    const int hi = v >> 4, lo = v & 0x0F;
    if (hi > 0) return std::min(1.0f, hi / 7.0f);
    return lo <= 1 ? 0.0f : 0.15f * lo / 15.0f + 0.05f;
}

inline float seqKeyLevel(const uint8_t* leds, int code)
{
    for (const auto& m : dtl::kLedToControl) if (m.code == code) return seqLedValue(leds[m.led]);
    return 0.0f;
}

inline float seqPageLevel(const uint8_t* leds, int i)
{
    if (i < 0 || i >= int(sizeof dtl::kPageLeds / sizeof dtl::kPageLeds[0])) return 0.0f;
    const uint8_t v = leds[dtl::kPageLeds[i]];
    return (v >> 4) ? 1.0f : ((v & 0x0F) > 1 ? 0.3f : 0.0f);
}
