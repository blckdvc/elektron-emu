/* Rinomina i simboli globali di rt.h/periph.h perche' non si scontrino con quelli del DSP
   ricompilato (libdndsp.a, nelle build di sviluppo). Incluso prima di rt/rt.h. */
#ifndef DN_INTERP_NAMES_H
#define DN_INTERP_NAMES_H
#define mem_move_gen dni_mem_move_gen
#define op_16_gen dni_op_16_gen
#define periph_read dni_periph_read
#define periph_write dni_periph_write
#define rt_block_set dni_rt_block_set
#define rt_cgu_state dni_rt_cgu_state
#define rt_dma_config dni_rt_dma_config
#define rt_dma_set dni_rt_dma_set
#define rt_event_add dni_rt_event_add
#define rt_exec dni_rt_exec
#define rt_frames_queued dni_rt_frames_queued
#define rt_idle_cycles dni_rt_idle_cycles
#define rt_init_seg dni_rt_init_seg
#define rt_offset dni_rt_offset
#define rt_out_channels dni_rt_out_channels
#define rt_periph_new dni_rt_periph_new
#define rt_periph_size dni_rt_periph_size
#define rt_pull_audio dni_rt_pull_audio
#define rt_push_frame dni_rt_push_frame
#define rt_reply dni_rt_reply
#define rt_run dni_rt_run
#define rt_sec_set dni_rt_sec_set
#define rt_sec_state dni_rt_sec_state
#define rt_sizeof dni_rt_sizeof
#define rt_sparse_load dni_rt_sparse_load
#define rt_spi_lock dni_rt_spi_lock
#define rt_spi_state dni_rt_spi_state
#define rt_stat dni_rt_stat

#endif
