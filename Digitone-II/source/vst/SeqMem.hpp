// SeqMem.hpp -- Digitone II: the pattern that is playing, read in the memory of the
// emulated OS for the "Elektron Emu" view (see Digitakt-II/source/vst/SeqMem.hpp).
// The addresses of Digitone II OS 1.11 are not known yet: read returns nothing and the
// view shows the selected track from the trig LEDs, as the Modern view.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

struct SeqData {
    uint8_t valid = 0, pattern = 0, tracks = 0, playing = 0;    // pattern: 0 = A01, 16 = B01...
    uint16_t mutes = 0, reserved2 = 0;                           // bit t = track t+1 muted
    uint64_t trigs[16] = {};                                     // bit s = a trig on step s+1
    uint8_t step[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};   // play position (modulo the length), 0xff none
};

namespace seqmem {

inline size_t read(dt_machine* m, uint8_t* out, size_t cap)
{
    (void)m; (void)out; (void)cap;
    return 0;
}

} // namespace seqmem
