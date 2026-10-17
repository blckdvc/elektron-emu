/* I gestori generici dell'interprete (una operazione = un gestore), condivisi dall'esecutore a
   goto calcolato (dn_exec.inc: etichette) e da quello a blocchi (dn_jit.c: funzioni chiamate dal
   codice emesso). OP_PLAIN: nessun effetto sul flusso. OP_FLOW: puo' cambiare `nxt`. Il corpo vede
   st, in, a[], pc, nxt. */
    OP_PLAIN(L_NOP, (void)0)
    OP_PLAIN(L_1A, op_1a(st, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], (uint32_t)a[8], in->live))
    OP_PLAIN(L_CMP, cmp_(st, (uint32_t)a[0], pes(st, a[1]), in->live))
    OP_PLAIN(L_EXEC, execute(st, (uint32_t)a[0], -1, in->live))
    OP_PLAIN(L_EXECS, execute_short(st, (uint32_t)a[0], -1, in->live))
    OP_PLAIN(L_3, op_3(st, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], (uint32_t)a[8], in->live, -1))
    OP_PLAIN(L_3C, mem_move(st, a[0], a[1], get_dag(st, KM, a[2]), 1, a[3], 4, 0, -1, -1))
    OP_PLAIN(L_4, op_4(st, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], (uint32_t)a[8], in->live, -1))
    OP_PLAIN(L_5MOVE, op_5move(st, a[0], a[1], a[2], (uint32_t)a[3], in->live))
    OP_PLAIN(L_5SWAP, op_5swap(st, a[0], a[1], a[2], (uint32_t)a[3], in->live))
    OP_PLAIN(L_6NOMEM, op_6nomem(st, a[0], (uint32_t)a[1], (uint32_t)a[2], in->live))
    OP_PLAIN(L_6MEM, op_6mem(st, a[0], a[1], a[2], a[3], a[4], (uint32_t)a[5], (uint32_t)a[6], in->live, -1))
    OP_PLAIN(L_7A, op_7a(st, a[0], a[1], a[2], a[3], a[4], (uint32_t)a[5], in->live))
    OP_PLAIN(L_7B, op_7b(st, a[0], a[1], a[2], a[3]))
    OP_PLAIN(L_7D, op_7d(st, a[0], a[1], a[2], a[3], a[4], (uint32_t)a[5], in->live))
    OP_FLOW(L_BR, {
        int ok = bcond(st, a[0]);
        uint32_t tg = a[4] ? ((get_dag(st, KI, a[2]) + get_dag(st, KM, a[3])) & 0xFFFFFF) : (uint32_t)a[1];
        if (a[10]) branch_compute(st, a[0], a[11], (uint32_t)a[12], in->live);
        if (ok) branch_common(st, pc, &nxt, tg, a[5], a[6], a[7], a[8], (uint32_t)a[9]);
    })
    OP_FLOW(L_RET, {
        int ok = bcond(st, a[0]);
        if (a[3]) branch_compute(st, a[0], a[4], (uint32_t)a[5], in->live);
        if (ok) do_return(st, pc, &nxt, a[1], a[2]);
    })
    OP_PLAIN(L_LOOPI, st->lcntr = (uint32_t)a[0]; do_loop(st, (uint32_t)a[1], (uint32_t)a[2], 1, (uint32_t)a[0], 0))
    OP_PLAIN(L_LOOPU, st->lcntr = get_ureg(st, a[0]); do_loop(st, (uint32_t)a[1], (uint32_t)a[2], 1, st->lcntr, 0))
    OP_PLAIN(L_LOOPT, do_loop(st, (uint32_t)a[0], (uint32_t)a[1], 0, 0, a[2]))
    OP_PLAIN(L_14A, op_14a(st, a[0], a[1], a[2], (uint32_t)a[3], a[4]))
    OP_PLAIN(L_14D, op_14d(st, a[0], a[1], (uint32_t)a[2], a[3], a[4], a[5]))
    OP_PLAIN(L_MEM15, mem_move(st, a[0], a[1], (uint32_t)a[2], 0, a[3], a[4], 0, -1, -1))
    OP_PLAIN(L_16, op_16(st, a[0], a[1], a[2], (uint32_t)a[3], -1))
    OP_PLAIN(L_17, op_17(st, a[0], (uint32_t)a[1]))
    OP_PLAIN(L_18A, op_18a(st, a[0], a[1], (uint32_t)a[2]))
    OP_PLAIN(L_MODIFY, modify(st, a[0], a[1], (uint32_t)a[2], a[3]))
    OP_PLAIN(L_BITREV, op_19bitrev(st, a[0], a[1], (uint32_t)a[2]))
    OP_PLAIN(L_20A, op_20a(st, a[0], a[1], a[2], a[3], a[4], a[5]))
    OP_PLAIN(L_IDLE, st->idle = 1; st->deadline = st->cycles)
    OP_FLOW(L_25D, op_25direct(st, pc, &nxt, (uint32_t)a[0], (uint32_t)a[1]))
    OP_FLOW(L_25P, op_25pcrel(st, pc, &nxt, (uint32_t)a[0], (uint32_t)a[1]))
    OP_PLAIN(L_RFRAME, op_rframe(st))
