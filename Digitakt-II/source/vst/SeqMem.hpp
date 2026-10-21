// SeqMem.hpp -- Digitakt II: the pattern that is playing, read in the memory of the
// emulated OS for the "Elektron Emu" view. The engine thread calls seqmem::read
// between two slices of emulation (dt_rt_set_seq_reader), so the memory is still; the
// view gets the result with dt_rt_seq. The addresses are those of Digitakt II OS 1.16 (the only OS
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

// Digitakt II OS 1.16: each step of each track has a 16-bit word, bit 0 = trig on (an
// erased trig keeps its other bits and its records in the pool after the words, so only
// this bit says if the trig is there); 128 steps, tracks 0x4A0 apart, patterns 0x15C00
// apart from A01 (tests/seqtest.cpp checks writing and erasing).
static const uint32_t kPattern = 0x41765ca3;     // u8: pattern; then u16 mutes
static const uint32_t kTrigs = 0x4099d89c, kPatStride = 0x15c00, kTrackStride = 0x4a0;

static const uint32_t kStep = 0x43165000;       // u8 per track, the step that plays (it counts on past the length on this OS)

inline size_t read(dt_machine* m, uint8_t* out, size_t cap)
{
    SeqData d;
    if (cap < sizeof d) return 0;
    uint8_t h[3];
    if (dt_machine_mem_read(m, kPattern, h, sizeof h) || h[0] >= 128) return 0;
    d.pattern = h[0];
    d.mutes = uint16_t(be16(h + 1));
    d.tracks = 16;
    static thread_local uint8_t buf[kTrackStride * 16];            // per engine thread
    if (dt_machine_mem_read(m, kTrigs + d.pattern * kPatStride, buf, sizeof buf)) return 0;
    for (int t = 0; t < 16; ++t)
        for (int s = 0; s < 128; ++s) {
            if (!(buf[t * kTrackStride + 2 * s + 1] & 1)) continue;
            if (s < 64) d.trigs[t] |= uint64_t(1) << s;
            else d.trigs2[t] |= uint64_t(1) << (s - 64);
        }
    uint8_t st[16];
    if (!dt_machine_mem_read(m, kStep, st, 16))
        for (int t = 0; t < 16; ++t) d.step[t] = st[t];
    d.valid = 1;
    std::memcpy(out, &d, sizeof d);
    return sizeof d;
}

// Page loop (dt_rt_set_seq_hook, every 2 slices on the engine thread): while it plays,
// a track that is about to leave page `page` gets the first step of the page as its
// next step. A track that never reached the page (shorter than it) is left alone.
// page < 0: off.
static const uint32_t kNext[] = {0x43164fe0, 0x43165144};   // u8 per track, the next step
static const int kLoopTracks = 16;

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
