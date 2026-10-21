// SeqSpec.hpp -- Analog Rytm MKII: the keys of the Elektron Emu view (SeqView.hpp) and how the
// LEDs of the machine read as levels. Shared by the UI and tests/seqtest.cpp: it is
// included after SeqView.hpp and panel_layout.hpp, inside the namespace of the caller.
#pragma once

// the keys of the Analog Rytm MKII for the sequencer view (SeqView.hpp): codes of the
// panel layout JSON, read on the panel photo
inline SeqSpec seqSpec()
{
    static const char* const kNames[12] = {"BD", "SD", "RS", "CP", "BT", "LT", "MT", "HT", "CH", "OH", "CY", "CB"};
    static const char* const kPages[7] = {"TRIG", "SRC", "SMPL", "FLTR", "AMP", "LFO", "FX TRACK"};
    static const char* const kEnc[8] = {"A", "B", "C", "D", "E", "F", "G", "H"};
    SeqSpec s;
    s.title = "Analog Rytm MKII";
    s.tracks = 12;
    for (int i = 0; i < 12; ++i) s.trackNames[i] = kNames[i];
    for (int i = 0; i < 8; ++i) { s.trig[i] = 25 + i; s.trig[8 + i] = 17 + i; }
    s.nPageKeys = 7;
    for (int i = 0; i < 6; ++i) { s.pageKeys[i] = 43 + i; s.pageNames[i] = kPages[i]; }
    s.pageKeys[6] = 7; s.pageNames[6] = kPages[6];                // FX: delay, reverb, dist, comp on the pages
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
    s.knobY1 = 55;                                                 // the status bar below: the level
    s.levelZone[0] = 0; s.levelZone[1] = 55; s.levelZone[2] = 64; s.levelZone[3] = 64;
    {                                                              // FUNC + key, flat
        static const char* const kFn[] = {"SETTINGS", "SAMPLING", "TEMPO", "PTN: SEQUENTIAL", "PTN: DIR. START", "PTN: DIR. JUMP", "PTN: TEMP JUMP", "CHAIN", "MIDI", "TRIG MUTES", "ACCENT", "SWING", "SLIDE", "COPY", "CLEAR", "PASTE"};
        static const int kFk[] = {-53, -52, -37, 1, 2, 3, 4, 6, 7, 9, 10, 11, 12, 13, 14, 15};
        s.nFuncs = int(sizeof kFk / sizeof kFk[0]);
        for (int i = 0; i < s.nFuncs; ++i) { s.funcNames[i] = kFn[i]; s.funcKeys[i] = kFk[i]; }
    }
    s.memo = "rytm";
    return s;
}

// the LEDs (dt_engine_leds / copyScreen, 128 values): colour in the high nibble, level in the low one
inline float seqKeyLevel(const uint8_t* lit, int code)
{
    for (const auto& m : arl::kLedToControl) if (m.code == code) return float(lit[m.led] & 0x0F) / 15.0f;
    return 0.0f;
}

inline float seqPageLevel(const uint8_t* lit, int i)
{
    if (i < 0 || i >= int(sizeof arl::kPageLeds / sizeof arl::kPageLeds[0])) return 0.0f;
    return float(lit[arl::kPageLeds[i]] & 0x0F) / 15.0f;
}
