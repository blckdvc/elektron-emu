// SeqSpec.hpp -- Syntakt: the keys of the Elektron Emu view (SeqView.hpp) and how the
// LEDs of the machine read as levels. Shared by the UI and tests/seqtest.cpp: it is
// included after SeqView.hpp and panel_layout.hpp, inside the namespace of the caller.
#pragma once

// the keys of the Syntakt for the sequencer view (SeqView.hpp): codes of the
// panel layout JSON, read on the panel photo
inline SeqSpec seqSpec()
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

// the LEDs (dt_engine_leds / copyScreen, 50 values): the page LEDs have their own levels
inline float seqLedValue(uint8_t value, int led)
{
    for (int l : stl::kPageLeds)
        if (l == led) {
            for (const auto& p : stl::kPageLevels) if (p.value == value) return float(p.level);
            return 0.0f;
        }
    const int lo = value & 0x0F;
    if (lo < stl::kLitMin) return 0.0f;
    return std::min(1.0f, 0.55f + 0.45f * float(lo - stl::kLitMin) / float(stl::kLitMax - stl::kLitMin));
}

inline float seqKeyLevel(const uint8_t* leds, int code)
{
    for (const auto& m : stl::kLedToControl) if (m.code == code) return seqLedValue(leds[m.led], m.led);
    return 0.0f;
}

inline float seqPageLevel(const uint8_t* leds, int i)
{
    if (i < 0 || i >= int(sizeof stl::kPageLeds / sizeof stl::kPageLeds[0])) return 0.0f;
    return seqLedValue(leds[stl::kPageLeds[i]], stl::kPageLeds[i]);
}
