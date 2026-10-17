/* GENERATO da tools/dtcore_profiles.py: non modificare a mano. */
typedef struct {
    const char *main_sha256;
    uint32_t flash_read;
    uint32_t pend_call;
    uint32_t completion_sem;
    uint32_t task_create;
    uint32_t set_pixel;
    uint32_t get_pixel;
    uint32_t px_copy;
    uint32_t depack_copy;
    uint32_t uart8_tx_wait;
    uint32_t uart8_tx_state;
    uint32_t uart8_ring_ptr;
    uint32_t ssi0_dma_force_rte;
    uint32_t sf_mulsf3;
    uint32_t sf_subsf3;
    uint32_t sf_addsf3;
    uint32_t sf_divsf3;
    uint32_t sf_abssf2;
    uint32_t sf_fixsfsi;
    uint32_t sf_cmpsf2;
    uint32_t sem_pend;
    uint32_t pend_b;
    uint32_t intro_done;
    uint32_t frame_sem;
    uint32_t ctx_switch;
    uint32_t display_sem;
    uint32_t queue_recv;
    uint32_t intro_park;
    uint32_t display_wait;
    uint32_t pump_wait;
    uint32_t tick_pend;
    uint32_t sleep_pend;
    uint32_t sd_status;
    uint32_t sd_cmd_sem;
    uint32_t sd_data_sem;
    uint32_t sd_dma_sem;
    uint32_t mainloop;
    uint32_t job_pump;
    uint32_t panel_diff;
    uint32_t fb_front;
    uint32_t intro_pit3_isr;
    uint32_t current_tcb;
    uint32_t entry;
    uint32_t task_start;
    uint32_t call_sites[32];
    uint32_t task_create_sites[32];
} dt_profile;

static const dt_profile DT_PROFILES[] = {
    {"57bb4dfa8df07d846adc72fdb4fb0d3cd3c5680c524bf498338460207e008e7d",
      0x40136cea, 0x401362f4, 0x44e6662c, 0x400012c8, 0x4011221c, 0x401122e8, 0x400d0a56,
      0x401382a4, 0x4000221c, 0x40964d74, 0x40964d84, 0x4002d36a, 0x40182d7c, 0x40182a94,
      0x40182a9a, 0x40182ebe, 0x40181cac, 0x4018360c, 0x401833ac, 0x4000141a, 0x400013a6,
      0x400d1934, 0x43149548, 0x40000410, 0x44e460d8, 0x40001946, 0x400d1960, 0x4013369e,
      0x400f0988, 0x40002a70, 0x401362fa, 0x44e3feac, 0x44e3fec8, 0x44e3fec0, 0x44e3feb8,
      0x40033d00, 0x400f0958, 0x4013390e, 0x402b6058, 0x400d0668, 0x47db3f64, 0x400004e8,
      0x40001314,
      {0x4012d442, 0x4012d4be, 0x4012dd06, 0x4012de0c, 0x4012df3a, 0x4012e120, 0x40136520, 0x401365fa, 0x4013673c, 0x40136844, 0x401369a6, 0x40136ae6, 0x40136c10, 0x40136d12, 0x40136e3a, 0x40136f7c, 0x401370c2, 0x40137126, 0x40137244, 0x401372f4, 0x4013750c, 0x40137616, 0x4013772e, 0x401377e6, 0x401378f4, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
      {0x40001148, 0x40002c22, 0x40033280, 0x400cc66a, 0x400cc798, 0x400ccd04, 0x400d133a, 0x400f0844, 0x40120788, 0x40120b88, 0x4012d1d4, 0x401335ba, 0x40134f3c, 0x40134fc4, 0x40135058, 0x4015397a, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000}},
};
#define DT_NPROFILES (sizeof DT_PROFILES / sizeof DT_PROFILES[0])
