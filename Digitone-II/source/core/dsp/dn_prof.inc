/* contatori di profilo (solo -DDNI_PROF): quali istruzioni girano e quanto */
#ifdef DNI_PROF
uint64_t dni_prof_mem[2][64], dni_prof_mv[2], dni_prof_cmpc[2];
uint64_t dni_prof_op[OP_COUNT], dni_prof_comp[4096], dni_prof_simd[2], dni_prof_idle;
static const char *const op_names[OP_COUNT] = {"UNDEC","BAD","NOP","1A","CMP","EXEC","EXECS","3","3C","4","5MOVE","5SWAP","6NOMEM","6MEM","7A","7B","7D","BR","BR9A","BR9B","RET","LOOPI","LOOPU","LOOPT","14A","14D","MEM15","16","17","18A","MODIFY","BITREV","20A","IDLE","25D","25P","RFRAME","HALT"};
uint64_t dni_h[12][8192];
uint64_t dni_prof_ticks[OPS_TOTAL], dni_prof_cnt[OPS_TOTAL];
static const char *const spec_names[] = {
#include "dn_spec_names.inc"
};
void dni_prof_dump(void)
{
    static const char *hn[12] = {"6NOMEM op6|c31<<6", "BR c31|call|db|ci|abs|comp", "17 ureg", "MEM15 ureg|size<<7|d<<11", "OP_3 !c31|ureg>=16<<1|size!=4<<2|sgn<<3|comp<<4", "CMP cond", "5MOVE srcg|dstg<<3|c31<<6|comp<<7", "OP_4 (come OP_3)", "OP_3 classe di calcolo", "CMP classe di calcolo", "7A c31|kind<<1|comp<<3", "MODIFY kind"};
    for (int t = 0; t < 12; t++) {
        fprintf(stderr, "  HIST %s\n", hn[t]);
        for (int n = 0; n < 12; n++) {
            int b = -1; for (int k = 0; k < 8192; k++) if (dni_h[t][k] && (b < 0 || dni_h[t][k] > dni_h[t][b])) b = k;
            if (b < 0) break;
            fprintf(stderr, "      %04x: %llu\n", b, (unsigned long long)dni_h[t][b]);
            dni_h[t][b] = 0;
        }
    }
    {   /* tempo per gestore (cntvct, 24 MHz): i piu' costosi */
        uint64_t tt = 0; for (int i = 0; i < OPS_TOTAL; i++) tt += dni_prof_ticks[i];
        for (int n = 0; n < 25; n++) {
            int b = -1; for (int i = 0; i < OPS_TOTAL; i++) if (dni_prof_ticks[i] && (b < 0 || dni_prof_ticks[i] > dni_prof_ticks[b])) b = i;
            if (b < 0) break;
            fprintf(stderr, "  TEMPO %-18s %5.1f%%  %10llu istr  %6.2f tick/istr\n", b < OP_COUNT ? op_names[b] : spec_names[b - OP_COUNT], 100.0 * dni_prof_ticks[b] / tt, (unsigned long long)dni_prof_cnt[b], (double)dni_prof_ticks[b] / dni_prof_cnt[b]);
            dni_prof_ticks[b] = 0;
        }
    }
    uint64_t tot = 0;
    for (int i = 0; i < OP_COUNT; i++) tot += dni_prof_op[i];
    fprintf(stderr, "istruzioni eseguite %llu (simd %llu, nosimd %llu)\n", (unsigned long long)tot, (unsigned long long)dni_prof_simd[1], (unsigned long long)dni_prof_simd[0]);
    for (int i = 0; i < OP_COUNT; i++) if (dni_prof_op[i]) fprintf(stderr, "  %-8s %12llu %5.1f%%\n", op_names[i], (unsigned long long)dni_prof_op[i], 100.0 * dni_prof_op[i] / tot);
    for (int o = 0; o < 2; o++) for (int k = 0; k < 64; k++) if (dni_prof_mem[o][k]) fprintf(stderr, "  mem op%s c31=%d u=%d d=%d size4=%d sgn=%d: %llu\n", o ? "3" : "4", k & 1, (k >> 1) & 1, (k >> 2) & 1, (k >> 3) & 1, (k >> 4) & 1, (unsigned long long)dni_prof_mem[o][k]);
    fprintf(stderr, "  5move c31: %llu other %llu; cmp c31 %llu other %llu\n", (unsigned long long)dni_prof_mv[1], (unsigned long long)dni_prof_mv[0], (unsigned long long)dni_prof_cmpc[1], (unsigned long long)dni_prof_cmpc[0]);
    /* compute: chiave = cu(2 bit)<<8 | op>>? : mostra i piu' frequenti */
    for (int n = 0; n < 40; n++) {
        int best = -1;
        for (int k = 0; k < 4096; k++) if (dni_prof_comp[k] && (best < 0 || dni_prof_comp[k] > dni_prof_comp[best])) best = k;
        if (best < 0) break;
        fprintf(stderr, "  compute-key %03x: %12llu %5.1f%%\n", best, (unsigned long long)dni_prof_comp[best], 100.0 * dni_prof_comp[best] / tot);
        dni_prof_comp[best] = 0;
    }
}
#endif
