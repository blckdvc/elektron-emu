// StarterKit.hpp -- i suoni di partenza di una macchina nata dal primo avvio.
//
// Perche': il primo avvio dal solo .syx (boot freddo + FORMAT +DRIVE) lascia il
// progetto di fabbrica (A01 WAXSAFETY) con le tracce che puntano agli slot dei
// sample di fabbrica, che sul +Drive appena formattato non ci sono: gli slot sono
// vuoti e il firmware rende le tracce mute (FINDINGS i187/i188). Una macchina cosi'
// non suona nessuna nota, come un Digitakt II vero col +Drive cancellato.
// I sample di fabbrica sono di Elektron e non si distribuiscono: qui si sintetizzano
// 16 suoni nostri (8 percussioni, 8 tonali), si scrivono in /incoming e si caricano
// sulle tracce 1-16 dal menu del firmware, come farebbe un utente (manuale 13.6.1,
// FINDINGS i179-i180): TRK+TRIG n, FUNC+SAMPLING, +DRIVE, incoming, il file, RIGHT,
// LOAD TO TRK, NO fino alla pagina principale. Nessuna scrittura diretta in RAM.
// Lo fa il plugin una volta, sullo stato del boot freddo, prima di metterlo in cache.
#pragma once

#include "dtcore.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace dtkit {

// versione del kit: entra nel nome della cache del primo avvio
static constexpr int kVersion = 1;
static constexpr double kRate = 48000.0;

struct Sound { const char* name; std::vector<float> pcm; };

inline uint32_t rnd(uint32_t& s) { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
inline float noise(uint32_t& s) { return float(rnd(s) & 0xffff) / 32768.0f - 1.0f; }

inline std::vector<float> make(int which)
{
    const double pi = 3.14159265358979323846;
    auto len = [](double sec) { return size_t(sec * kRate); };
    std::vector<float> v;
    uint32_t seed = 0x9e3779b9u + uint32_t(which) * 7919u;
    double ph = 0, ph2 = 0, lp = 0, hpPrev = 0, hp = 0;
    auto note = [](double midi) { return 440.0 * std::pow(2.0, (midi - 69.0) / 12.0); };
    switch (which) {
    case 0: // kick: sinusoide da 150 a 45 Hz
        v.resize(len(0.6));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate, f = 45 + 105 * std::exp(-t * 28);
            ph += 2 * pi * f / kRate;
            v[i] = float(std::sin(ph) * std::exp(-t * 6.0) + 0.3 * std::exp(-t * 300) * noise(seed));
        }
        break;
    case 1: // snare: tono a 185 Hz e rumore
        v.resize(len(0.35));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate;
            ph += 2 * pi * 185 / kRate;
            const double n = noise(seed);
            hp = 0.6 * (hp + n - hpPrev); hpPrev = n;
            v[i] = float(0.5 * std::sin(ph) * std::exp(-t * 25) + 0.7 * hp * std::exp(-t * 14));
        }
        break;
    case 2: // clap: tre colpi di rumore e coda
        v.resize(len(0.4));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate;
            const double n = noise(seed);
            lp += 0.35 * (n - lp);
            double env = std::exp(-t * 12) * 0.8;
            for (double b : {0.0, 0.011, 0.022})
                if (t >= b && t < b + 0.01) env = std::exp(-(t - b) * 180);
            v[i] = float((n - lp) * env);
        }
        break;
    case 3: // charleston chiuso
    case 4: // charleston aperto
        v.resize(len(which == 3 ? 0.09 : 0.5));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate;
            const double n = noise(seed);
            hp = 0.3 * (hp + n - hpPrev); hpPrev = n;
            v[i] = float(0.9 * hp * std::exp(-t * (which == 3 ? 45 : 7)));
        }
        break;
    case 5: // tom
        v.resize(len(0.5));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate, f = 95 + 60 * std::exp(-t * 18);
            ph += 2 * pi * f / kRate;
            v[i] = float(0.9 * std::sin(ph) * std::exp(-t * 8));
        }
        break;
    case 6: // rim
        v.resize(len(0.08));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate;
            v[i] = float((0.6 * std::sin(2 * pi * 820 * t) + 0.4 * std::sin(2 * pi * 1650 * t)) * std::exp(-t * 70));
        }
        break;
    case 7: // campanaccio: due quadre
        v.resize(len(0.4));
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate;
            const double s = (std::fmod(t * 540, 1.0) < 0.5 ? 1 : -1) + (std::fmod(t * 800, 1.0) < 0.5 ? 1 : -1);
            lp += 0.25 * (s - lp);
            v[i] = float(0.35 * lp * std::exp(-t * 9));
        }
        break;
    default: { // tonali, Do 3 (MIDI 48) o Do 4: si suonano cromatici dal MIDI
        v.resize(len(which == 14 ? 2.0 : 1.2));
        const double f = note(which == 8 || which == 12 ? 36 : 48);
        for (size_t i = 0; i < v.size(); ++i) {
            const double t = i / kRate;
            double s = 0, env = std::exp(-t * 3);
            const double saw = 2 * std::fmod(t * f, 1.0) - 1;
            switch (which) {
            case 8:  lp += 0.08 * (saw - lp); s = lp * 1.4; env = std::exp(-t * 4); break;         // basso
            case 9:  s = std::fmod(t * f, 1.0) < 0.5 ? 0.6 : -0.6; break;                          // quadra
            case 10: s = saw * 0.7; break;                                                          // dente di sega
            case 11: s = std::sin(2 * pi * f * t + 2.5 * std::exp(-t * 6) * std::sin(2 * pi * f * 3.5 * t)) * 0.8; break; // campana FM
            case 12: s = std::sin(2 * pi * f * t) * 0.9; env = std::exp(-t * 2.5); break;           // basso sinusoidale
            case 13: lp += (0.02 + 0.3 * std::exp(-t * 20)) * (saw - lp); s = lp * 1.3; env = std::exp(-t * 6); break; // pizzicato
            case 14: {                                                                              // accordo (Do minore)
                for (double m : {48.0, 51.0, 55.0}) s += 0.3 * (2 * std::fmod(t * note(m), 1.0) - 1);
                lp += 0.05 * (s - lp); s = lp * 1.2;
                env = std::min(1.0, t * 8) * std::exp(-t * 1.2);
                break;
            }
            default: s = std::sin(2 * pi * f * t) * 0.8 + 0.2 * std::sin(4 * pi * f * t); break;    // seno
            }
            v[i] = float(s * env);
        }
    }
    }
    // niente click alla fine
    const size_t fade = std::min<size_t>(v.size(), 240);
    for (size_t i = 0; i < fade; ++i) v[v.size() - 1 - i] *= float(i) / float(fade);
    return v;
}

inline std::vector<Sound> sounds()
{
    static const char* names[16] = {"01 Kick", "02 Snare", "03 Clap", "04 Closed Hat", "05 Open Hat", "06 Tom",
                                    "07 Rim", "08 Cowbell", "09 Bass", "10 Square", "11 Saw", "12 Bell",
                                    "13 Sine Bass", "14 Pluck", "15 Chord", "16 Sine"};
    std::vector<Sound> out;
    for (int i = 0; i < 16; ++i) out.push_back({names[i], make(i)});
    return out;
}

inline std::vector<uint8_t> wav16(const std::vector<float>& pcm)
{
    std::vector<uint8_t> w(44 + pcm.size() * 2);
    auto u32 = [&](size_t o, uint32_t x) { for (int i = 0; i < 4; ++i) w[o + i] = uint8_t(x >> (8 * i)); };
    auto u16 = [&](size_t o, uint16_t x) { w[o] = uint8_t(x); w[o + 1] = uint8_t(x >> 8); };
    std::memcpy(&w[0], "RIFF", 4); u32(4, uint32_t(w.size() - 8)); std::memcpy(&w[8], "WAVEfmt ", 8);
    u32(16, 16); u16(20, 1); u16(22, 1); u32(24, uint32_t(kRate)); u32(28, uint32_t(kRate) * 2); u16(32, 2); u16(34, 16);
    std::memcpy(&w[36], "data", 4); u32(40, uint32_t(pcm.size() * 2));
    for (size_t i = 0; i < pcm.size(); ++i) {
        const float c = std::max(-1.0f, std::min(1.0f, pcm[i] * 0.9f));
        u16(44 + 2 * i, uint16_t(int16_t(std::lrint(c * 32767.0f))));
    }
    return w;
}

// Carica il kit sullo stato `state` (dal boot freddo) e restituisce il nuovo stato in
// `out`. `cancel` != 0 interrompe. 0 = ok.
inline int apply(const uint8_t* syx, size_t syxLen, const std::vector<uint8_t>& state, std::vector<uint8_t>& out,
                 const std::atomic<int>* cancel, std::string& err)
{
    char e[256] = "";
    const uint32_t frames = 512;
    dt_rt* rt = dt_rt_new_mem(syx, syxLen, state.data(), state.size(), kRate, frames, e, sizeof e);
    if (!rt) { err = e; return -1; }
    std::atomic<bool> run{true};
    std::atomic<uint64_t> blocks{0};
    std::thread th([&] {                       // l'host: il tempo del motore avanza
        std::vector<float> buf(size_t(frames) * 17);
        float* o[17];
        for (int i = 0; i < 17; ++i) o[i] = buf.data() + size_t(i) * frames;
        while (run.load()) { dt_rt_render(rt, frames, o, 17, 1); blocks.fetch_add(1); }
    });
    auto wait = [&](uint64_t n) {             // n blocchi da 512 campioni del motore
        const uint64_t until = blocks.load() + n;
        while (blocks.load() < until && !(cancel && cancel->load()))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };
    auto key = [&](int code, uint64_t after = 24) {
        dt_rt_panel(rt, DT_PANEL_PRESS, code, 0); wait(4);
        dt_rt_panel(rt, DT_PANEL_RELEASE, code, 0); wait(after);
    };
    auto chord = [&](int hold, int code) {     // hold + code (FUNC+SAMPLING, TRK+TRIG)
        dt_rt_panel(rt, DT_PANEL_PRESS, hold, 0); wait(8);
        key(code, 8);
        dt_rt_panel(rt, DT_PANEL_RELEASE, hold, 0); wait(30);
    };
    enum { TRIG1 = 25, YES = 10, NO = 12, UP = 11, DOWN = 14, RIGHT = 15, FUNC = 17, TRK = 16, SAMPLING = 50 };
    int r = 0;
    wait(100);
    const std::vector<Sound> kit = sounds();
    for (const Sound& s : kit) {
        if (cancel && cancel->load()) break;
        uint8_t* blob = nullptr;
        size_t bl = 0;
        const std::vector<uint8_t> w = wav16(s.pcm);
        if (dt_wav2dt(w.data(), w.size(), &blob, &bl, e, sizeof e) != 0
            || dt_drive_put(rt, (std::string("/incoming/") + s.name).c_str(), blob, bl, nullptr, nullptr, e, sizeof e) != 0) {
            std::free(blob);
            err = std::string(s.name) + ": " + e;
            r = -1;
            break;
        }
        std::free(blob);
    }
    wait(50);
    for (int t = 0; t < 16 && r == 0 && !(cancel && cancel->load()); ++t) {
        for (int i = 0; i < 4; ++i) key(NO, 12);                 // pagina principale
        chord(TRK, TRIG1 + t);                                   // traccia t+1
        chord(FUNC, SAMPLING);                                   // SAMPLES
        key(YES);                                                // +DRIVE
        for (int i = 0; i < 3; ++i) key(DOWN, 12);               // incoming (l'ultima cartella)
        key(YES);
        for (int i = 0; i < 17; ++i) key(UP, 10);                // in cima alla lista
        for (int i = 0; i < t; ++i) key(DOWN, 10);               // il file t+1 (in ordine di nome)
        key(YES);                                                // spunta
        key(RIGHT);
        key(DOWN);                                               // LOAD TO TRK
        key(YES, 60);
    }
    for (int i = 0; i < 4; ++i) key(NO, 12);
    chord(TRK, TRIG1);                                           // di nuovo la traccia 1
    wait(50);
    run = false;
    th.join();
    if (r == 0 && cancel && cancel->load()) { err = "interrupted"; r = -1; }
    if (r == 0) {
        uint8_t* buf = nullptr;
        size_t len = 0;
        if (dt_rt_save(rt, &buf, &len) == 0 && buf) out.assign(buf, buf + len);
        else { err = "save failed"; r = -1; }
        std::free(buf);
    }
    dt_rt_free(rt);
    return r;
}

} // namespace dtkit
