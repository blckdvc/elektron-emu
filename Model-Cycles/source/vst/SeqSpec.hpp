// SeqSpec.hpp -- Model:Cycles: the keys of the Elektron Emu view (SeqView.hpp) and how the
// LEDs of the machine read as levels. Shared by the UI and tests/seqtest.cpp: it is
// included after SeqView.hpp and panel_layout.hpp, inside the namespace of the caller.
#pragma once

// the keys of the Model:Cycles for the sequencer view (SeqView.hpp): codes of the
// panel layout JSON, read on the panel photo
inline SeqSpec seqSpec()
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
    {                                                              // FUNC + key, flat
        static const char* const kFn[] = {"SOUNDS", "LFO", "PTN TEMPO", "COPY", "CLEAR", "PASTE"};
        static const int kFk[] = {5, 8, 14, 9, 10, 11};
        s.nFuncs = int(sizeof kFk / sizeof kFk[0]);
        for (int i = 0; i < s.nFuncs; ++i) { s.funcNames[i] = kFn[i]; s.funcKeys[i] = kFk[i]; }
    }
    s.memo = "cycles";
    return s;
}

// the LEDs (dt_engine_leds / copyScreen, 8 bytes, 1 = off): led_bits says which bit is which LED
inline void seqLit(const uint8_t leds[8], bool lit[64])
{
    const int nbits = int(sizeof mcl::kLedBits / sizeof mcl::kLedBits[0]);
    for (int id = 0; id < 64; ++id) {
        const int bit = id < nbits ? mcl::kLedBits[id] : -1;
        lit[id] = bit >= 0 && bit / 8 < 8 && ((leds[bit / 8] >> (bit % 8)) & 1) == 0;
    }
}

inline float seqKeyLevel(const bool* lit, int code)
{
    for (const auto& k : mcl::kKeyLeds) if (k.code == code) return k.led >= 0 && k.led < 64 && lit[k.led] ? 1.0f : 0.0f;
    return 0.0f;
}

inline float seqPageLevel(const bool* lit, int i)
{
    if (i < 0 || i >= int(sizeof mcl::kPageLedIds / sizeof mcl::kPageLedIds[0])) return 0.0f;
    const int l = mcl::kPageLedIds[i];
    return l >= 0 && l < 64 && lit[l] ? 1.0f : 0.0f;
}
