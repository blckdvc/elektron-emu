// SeqMem.hpp -- Model:Cycles: the pattern that is playing, read in the memory of the
// emulated OS for the "Elektron Emu" view. The engine thread calls seqmem::read
// between two slices of emulation (dt_rt_set_seq_reader), so the memory is still; the
// view gets the result with dt_rt_seq. The addresses are those of Model:Cycles OS 1.13 (the only OS
// the plug-in runs), found by placing trigs and comparing the RAM.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

struct SeqData {
    uint8_t valid = 0, pattern = 0, tracks = 0, playing = 0;    // pattern: 0 = A01, 16 = B01...
    uint16_t mutes = 0, reserved2 = 0;                           // bit t = track t+1 muted
    uint64_t trigs[16] = {};                                     // bit s = a trig on step s+1
    uint8_t step[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};   // play position, 0xff none
    uint64_t trigs2[16] = {};                                    // bit s = a trig on step s+65 (128-step patterns)
};

namespace seqmem {

inline uint32_t be16(const uint8_t* p) { return uint32_t(p[0]) << 8 | p[1]; }

// Model:Cycles OS 1.13: per pattern 6 tracks of 0x2d2 bytes starting with 64 big-endian words, one per
// step; bit 0 of the word is "trig on". Patterns are 0x3a00 apart from the first one.
static const uint32_t kPattern = 0x406f9e29;     // u8
static const uint32_t kMutes = 0x406f9e2b;       // u8
static const uint32_t kBase = 0x404e9e24, kPatStride = 0x3a00, kTrackStride = 0x2d2;
static const int kTracks = 6;

static const uint32_t kStep = 0x40a7886c;       // u8 per track, the step that plays

inline size_t read(dt_machine* m, uint8_t* out, size_t cap)
{
    SeqData d;
    if (cap < sizeof d) return 0;
    uint8_t p = 0, mu[2] = {0, 0};
    if (dt_machine_mem_read(m, kPattern, &p, 1) || p >= 128) return 0;
    if (dt_machine_mem_read(m, kMutes, mu, 1)) return 0;
    d.pattern = p;
    d.mutes = uint16_t(mu[0]);
    d.tracks = kTracks;
    static thread_local uint8_t buf[kTrackStride * kTracks];
    if (dt_machine_mem_read(m, kBase + p * kPatStride, buf, sizeof buf)) return 0;
    for (int t = 0; t < kTracks; ++t)
        for (int s = 0; s < 64; ++s)
            if (be16(buf + t * kTrackStride + 2 * s) & 1) d.trigs[t] |= uint64_t(1) << s;
    uint8_t st[16];
    if (!dt_machine_mem_read(m, kStep, st, kTracks))
        for (int t = 0; t < kTracks; ++t) d.step[t] = st[t];
    d.valid = 1;
    std::memcpy(out, &d, sizeof d);
    return sizeof d;
}

// Page loop (dt_rt_set_seq_hook, every 2 slices on the engine thread): while it plays,
// a track that is about to leave page `page` gets the first step of the page as its
// next step. A track that never reached the page (shorter than it) is left alone.
// page < 0: off.
static const uint32_t kNext[] = {0x40a78860};   // u8 per track, the next step
static const int kLoopTracks = 6;

inline void loop(dt_machine* m, int page)
{
    static thread_local uint8_t maxSeen[16], last[16];
    static thread_local int pattern = -1, still = 1000;
    uint8_t p = 0, cur[16], nxt[16];
    if (dt_machine_mem_read(m, kPattern, &p, 1) || dt_machine_mem_read(m, kStep, cur, kLoopTracks) ||
        dt_machine_mem_read(m, kNext[0], nxt, kLoopTracks))
        return;
    if (p != pattern) { pattern = p; std::memset(maxSeen, 0, sizeof maxSeen); }
    if (std::memcmp(cur, last, kLoopTracks)) { still = 0; std::memcpy(last, cur, kLoopTracks); }
    else if (still < 1000) ++still;
    for (int t = 0; t < kLoopTracks; ++t) if (cur[t] > maxSeen[t]) maxSeen[t] = cur[t];
    if (page < 0 || still > 400) return;                         // off, or stopped
    const int lo = page * 16, hi = lo + 15;
    bool dirty = false;
    for (int t = 0; t < kLoopTracks; ++t) {
        if (maxSeen[t] < lo || (nxt[t] >= lo && nxt[t] <= hi)) continue;
        nxt[t] = uint8_t(lo);
        dirty = true;
    }
    if (dirty)
        for (uint32_t a : kNext) dt_machine_mem_write(m, a, nxt, kLoopTracks);
}

} // namespace seqmem
