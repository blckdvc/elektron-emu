/* GENERATO da tools/arcore_profiles.py: non modificare a mano. */
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
    uint32_t sd_status;
    uint32_t sd_cmd_sem;
    uint32_t sd_data_sem;
    uint32_t sd_dma_sem;
    uint32_t fb_front;
    uint32_t fb_back;
    uint32_t intro_pit3_isr;
    uint32_t intro_done;
    uint32_t frame_sem;
    uint32_t ctx_switch;
    uint32_t panel_diff;
    uint32_t mainloop;
    uint32_t job_pump;
    uint32_t display_sem;
    uint32_t current_tcb;
} dt_profile;

static const dt_profile DT_PROFILES[] = {
    {"28d9ef40c895e03f17dfce096543109adaf708947392db72818fc2a412d685d6",
      0x401362b6, 0x40098ca4, 0x4193bfd4, 0x40000de4, 0x4006fa08, 0x4006fad4, 0x00000000,
      0x40099450, 0x400019f4, 0x413da6a0, 0x413da6b0, 0x4011d286, 0x4015c364, 0x4015c07c,
      0x4015c082, 0x4015c4a6, 0x00000000, 0x00000000, 0x00000000, 0x4191b0bc, 0x4191b0d8,
      0x4191b0d0, 0x4191b0c8, 0x40287808, 0x4028780c, 0x00000000, 0x00000000, 0x00000000,
      0x40000410, 0x400959f2, 0x00000000, 0x400330c4, 0x00000000, 0x42f67924},
};
#define DT_NPROFILES (sizeof DT_PROFILES / sizeof DT_PROFILES[0])
