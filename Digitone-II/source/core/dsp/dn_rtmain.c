/* Involucro del dispatcher generato (dsp_main.c di tools/dsp_gensrc.py, quello con RT_MAIN)
 * per avere piu' istanze del DSP nello stesso processo senza toccare rt.h.
 *
 * rt.h ha `static Seg segtab[65536];` globale, riempita da rt_init_seg con i puntatori alla
 * memoria di UNA istanza. Qui `segtab` diventa `(*dn_segp)`: la dichiarazione si legge
 * `static Seg (*dn_segp)[65536];` e gli usi (`&segtab[k]`, `st->seg = segtab`) restano
 * corretti. dn__init_seg punta dn_segp sulla tabella dell'istanza, sotto un mutex, e chiama
 * rt_init_seg: dopo, il codice generato legge solo st->seg.
 *
 * Gli altri stati globali di dsp_main.c / rt.h / periph.h (docs/DSP-EMULATOR.md):
 *   n_dispatch, rt_idle_cyc       contatori di diagnostica (scritture concorrenti innocue)
 *   live                          il thread dal vivo del Python (rt_live_*), il plugin non
 *                                 lo usa; rt_push_frame prende live.mu (un mutex globale,
 *                                 per frame) e rt_pull_audio legge live.clock/started (0)
 *   prof_pc, prof_n               profiler a campionamento, non usato
 * e nel sorgente dei nuclei (non e' un RT_MAIN): k_real, scritto e mai letto con KSPACE,
 * e i contatori k_calls/k_iters/k_instrs, diagnostica.
 *
 * Compilato con gli stessi flag dei sorgenti generati e -I <cartella dei sorgenti>.
 */
#define segtab (*dn_segp)
#include "dsp_main.c"
#undef segtab

/* il blocco sulla tabella (una sola scrittura per istanza, a creazione): uno spinlock con operazioni
   atomiche al posto di pthread_mutex, cosi' la libreria costruita sul computer dell'utente (Windows
   compreso) non dipende da libwinpthread */
/* tabella di ripiego: chi chiama rt_init_seg direttamente (bridge.Engine su una dylib fatta
   con questi oggetti, tools/dsp_export.py --lib) ha il comportamento di prima, una tabella
   sola. Definizione che completa la dichiarazione provvisoria venuta da rt.h. */
static Seg dn_seg_default[65536];
static Seg (*dn_segp)[65536] = &dn_seg_default;
static volatile int dn_seg_mu;

size_t dn__seg_bytes(void) { return sizeof(Seg) * 65536; }
size_t dn__st_size(void) { return sizeof(St); }
size_t dn__periph_size(void) { return sizeof(Periph); }

/* la tabella dei segmenti dell'istanza `st` in `tab` (dn__seg_bytes byte) */
void dn__init_seg(St *st, void *tab) {
    while (__atomic_exchange_n(&dn_seg_mu, 1, __ATOMIC_ACQUIRE)) {}
    dn_segp = (Seg (*)[65536])tab;
    rt_init_seg(st);
    dn_segp = &dn_seg_default;
    __atomic_store_n(&dn_seg_mu, 0, __ATOMIC_RELEASE);
}

/* pagine sparse della Periph: quante, base e puntatore della k-esima */
int dn__periph_nsparse(void *per) { return ((Periph *)per)->nsparse; }
uint32_t dn__periph_sparse_base(void *per, int k) { return ((Periph *)per)->sparse_base[k]; }
uint8_t **dn__periph_sparse_slot(void *per, int k) { return &((Periph *)per)->sparse[k]; }
