/* GENERATO da tools/stcore_profiles.py: non modificare a mano. */
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
    uint32_t intro_pit3_isr;
    uint32_t sd_status;
    uint32_t sd_cmd_sem;
    uint32_t sd_data_sem;
    uint32_t sd_dma_sem;
    uint32_t mainloop;
    uint32_t job_pump;
    uint32_t panel_diff;
    uint32_t fb_front;
    uint32_t fb_back;
    uint32_t current_tcb;
    uint32_t entry;
} st_profile;

static const st_profile ST_PROFILES[] = {
    {"8cf2b4b4ea80d58953da46ca453328454fb3fca5cb5f844a2f7bad4b72241f53",
     0x40116bb6, 0x401161c0, 0x444da2b4, 0x40001640, 0x400f7140, 0x400f720c, 0x400865a0, 0x40118170, 0x40002560, 0x405d27b8, 0x405d27c8, 0x400a2034, 0x4015e624, 0x4015e33c, 0x4015e342, 0x4015e766, 0x00000000, 0x4015eef4, 0x4015ec54, 0x40001792, 0x4000171e, 0x00000000, 0x00000000, 0x40000410, 0x00000000, 0x444b972c, 0x444b9748, 0x444b9740, 0x444b9738, 0x4000b2d2, 0x400bda70, 0x40113756, 0x402d6dc0, 0x402d6dc4, 0x454e39a8, 0x400004e8},
};
#define ST_NPROFILES (sizeof ST_PROFILES / sizeof ST_PROFILES[0])
