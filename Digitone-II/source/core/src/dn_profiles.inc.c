/* GENERATO da tools/dncore_profiles.py: non modificare a mano. */
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
    {"57b06a7960b7c3dc9803bde6b31896b89a2fbdcc404a932ff503f3856d4d7a61",
      0x401350ce, 0x401346d8, 0x4463ec4c, 0x400012c8, 0x40113b90, 0x40113c5c, 0x400d2f2e,
      0x40136688, 0x4000221c, 0x4058d5bc, 0x4058d5cc, 0x400253ce, 0x40178624, 0x4017833c,
      0x40178342, 0x40178766, 0x40177554, 0x40178eb4, 0x40178c54, 0x4000141a, 0x400013a6,
      0x400d3e0c, 0x42c41614, 0x40000410, 0x4461e6e8, 0x40001946, 0x400d3e38, 0x40131a82,
      0x400f2364, 0x40002a70, 0x401346de, 0x4461b6bc, 0x4461b6d8, 0x4461b6d0, 0x4461b6c8,
      0x4002f178, 0x400f2334, 0x40131cf2, 0x402a0b88, 0x400d2b40, 0x4664acdc, 0x400004e8,
      0x40001314,
      {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
      {0x40001148, 0x40002c22, 0x4002e902, 0x400cea9e, 0x400cebcc, 0x400cf108, 0x400d3812, 0x400f2220, 0x40122130, 0x40122530, 0x4012b892, 0x4013199e, 0x40133320, 0x401333a8, 0x4013343c, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000}},
};
#define DT_NPROFILES (sizeof DT_PROFILES / sizeof DT_PROFILES[0])
