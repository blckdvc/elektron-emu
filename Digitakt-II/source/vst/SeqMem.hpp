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
    uint8_t valid = 0, pattern = 0, tracks = 0, reserved = 0;   // pattern: 0 = A01, 16 = B01...
    uint16_t mutes = 0, reserved2 = 0;                           // bit t = track t+1 muted
    uint64_t trigs[16] = {};                                     // bit s = a trig on step s+1
};

namespace seqmem {

inline uint32_t be16(const uint8_t* p) { return uint32_t(p[0]) << 8 | p[1]; }

// Digitakt II OS 1.16: each pattern has a pool of 6-byte records [track][step][..][..][..][0],
// four per trig (track 0xff = free); the pools are 0x15C00 apart from A01.
static const uint32_t kPattern = 0x41765ca3;     // u8: pattern; then u16 mutes
static const uint32_t kPool = 0x409a229c, kPoolStride = 0x15c00, kSlots = 6560;

inline size_t read(dt_machine* m, uint8_t* out, size_t cap)
{
    SeqData d;
    if (cap < sizeof d) return 0;
    uint8_t h[3];
    if (dt_machine_mem_read(m, kPattern, h, sizeof h) || h[0] >= 128) return 0;
    d.pattern = h[0];
    d.mutes = uint16_t(be16(h + 1));
    d.tracks = 16;
    static thread_local uint8_t pool[kSlots * 6];                 // per engine thread
    if (dt_machine_mem_read(m, kPool + d.pattern * kPoolStride, pool, sizeof pool)) return 0;
    for (uint32_t k = 0; k < kSlots; ++k) {
        const uint8_t* e = pool + 6 * k;
        if (e[0] < 16 && e[1] < 64) d.trigs[e[0]] |= uint64_t(1) << e[1];
    }
    d.valid = 1;
    std::memcpy(out, &d, sizeof d);
    return sizeof d;
}

} // namespace seqmem
