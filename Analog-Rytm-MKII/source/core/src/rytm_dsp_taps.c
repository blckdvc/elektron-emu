/* Il motore delle voci (native/rytm_dsp.c, lo stesso testo del Python) con le
 * voci una per una per le uscite separate del plugin: ogni voce, dopo il suo
 * VCA e il suo pan (x * gl, x * gr), prima degli effetti e del mix. Il mix
 * resta quello del Python campione per campione (verificato:
 * tests/test_arcore.py con questa unita'). Compilato con gli stessi flag
 * (-O2 -ffast-math). */
#define RYTM_VOICE_FIELDS float tap[2];
#define RYTM_DSP_FIELDS float vout[8][32][2];
#define RYTM_VOICE_OUT(v, a, b) ((v)->tap[0] = (a), (v)->tap[1] = (b))
#define RYTM_VOICE_BEGIN(d, i) do { int k_; for (k_ = 0; k_ < 8; k_++) (d)->v[k_].tap[0] = (d)->v[k_].tap[1] = 0; } while (0)
#define RYTM_VOICE_END(d, i) do { int k_; for (k_ = 0; k_ < 8; k_++) { (d)->vout[k_][i][0] = (d)->v[k_].tap[0]; (d)->vout[k_][i][1] = (d)->v[k_].tap[1]; } } while (0)
#include "native/rytm_dsp.c"

/* -> le voci dell'ultimo blocco: 8 x 32 frame stereo (L, R) */
const float *rytm_voices(dsp_t *d)
{
    return &d->vout[0][0][0];
}
