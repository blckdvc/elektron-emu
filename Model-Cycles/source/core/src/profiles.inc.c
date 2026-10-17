/* GENERATO da tools/mccore_profiles.py: non modificare a mano. */
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
} dt_profile;

static const dt_profile DT_PROFILES[] = {
    {"cc99d4f0175d34d1e91d046e6ec85a5e8ab58ab9edbb3c24406acd48cb99ee98",
      0x4008fe46, 0x4008f450, 0x40fde7d4, 0x400019ec, 0x400701b8, 0x40070284, 0x00000000,
      0x4009138c, 0x00000000, 0x00000000, 0x40058a08, 0x400c52f8, 0x400c5010, 0x400c5016,
      0x400c543a, 0x00000000, 0x00000000, 0x00000000, 0x40fdd5c0, 0x40fdd5dc, 0x40fdd5d4,
      0x40fdd5cc, 0x401492f0, 0x401492f4, 0x00000000, 0x00000000, 0x00000000, 0x40000410},
};
#define DT_NPROFILES (sizeof DT_PROFILES / sizeof DT_PROFILES[0])
