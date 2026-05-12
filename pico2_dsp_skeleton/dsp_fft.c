/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file does an FFT on the left channel and prints the highest
 * bin.  It uses Core 1 to perform the FFT and printing, so Core 0
 * can maintain strict real-time audio passthrough.
 */

#include "dsp_common.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <string.h>

// FFT size must be a power of 2 between 64 and 4096 inclusive
// Also, I think FFT_SIZE must be evenly divisible by BLOCK_SIZE
#define FFT_SIZE 4096
#if ((FFT_SIZE > 4096) || (FFT_SIZE < 64))
#error "CMSIS_DSP FFT size out of range"
#endif
#define MAG_SIZE (FFT_SIZE / 2)
#define FFT_MAX_MAG ((float32_t)FFT_SIZE / 4.0f)
#define LN_TO_DB_CONV 8.685889638f

static float32_t fft_input_mirrored_history[2 * FFT_SIZE] = {0};
static __not_in_flash() float32_t hann_window[FFT_SIZE];
static float32_t fft_work_buffer[FFT_SIZE];
static float32_t fft_complex_output[FFT_SIZE];
static float32_t output_mag[MAG_SIZE];
static float32_t output_db[MAG_SIZE];

static arm_rfft_fast_instance_f32 fft_inst;

static float32_t fft_snapshot_buf[FFT_SIZE];
static float32_t temp_block[BLOCK_SIZE];
static volatile int snapshot_write_idx = 0;

static void core1_main(void);

void init_dsp(void)
{
  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
  for (int i = 0; i < FFT_SIZE; i++) {
    float32_t angle = 2.0f * PI_F * (float32_t)i / (float32_t)(FFT_SIZE - 1);
    hann_window[i] = 0.5f * (1.0f - arm_cos_f32(angle));
  }

  // Launch Core 1
  multicore_launch_core1(core1_main);
}

static void convert_to_db(float32_t *mag_input, float32_t *db_output,
                          uint32_t count)
{
  float32_t scale = 1.0f / FFT_MAX_MAG;
  arm_scale_f32(mag_input, scale, db_output, count);

  for (int i = 0; i < count; i++)
    if (db_output[i] < 1e-12f)
      db_output[i] = 1e-12f;

  arm_vlog_f32(db_output, db_output, count);
  arm_scale_f32(db_output, LN_TO_DB_CONV, db_output, count);
}

static void process_sliding_fft(float32_t *snapshot_in,
                                float32_t *output_mag_out)
{
  arm_mult_f32(snapshot_in, hann_window, fft_work_buffer, FFT_SIZE);
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, fft_complex_output, 0);
  arm_cmplx_mag_f32(fft_complex_output, output_mag_out, MAG_SIZE);
}

static void core1_main(void)
{
  uint32_t cur_time, next_time = 0, max_idx;
  float32_t max_mag;

  // Send initial ready token to Core 0
  multicore_fifo_push_blocking(1);

  while (1) {
    // Await indication from core 0 that fft_snapshot_buf is ready
    (void) multicore_fifo_pop_blocking();

    gpio_xor_mask(1u << PIN_DEBUG_SW1); // Toggle SW1 GPIO

    process_sliding_fft(fft_snapshot_buf, output_mag);
    convert_to_db(output_mag, output_db, MAG_SIZE);

    cur_time = to_ms_since_boot(get_absolute_time());
    if (cur_time >= next_time) {
      next_time = 1000 + cur_time;
      arm_max_f32(&output_db[1], MAG_SIZE-1, &max_mag, &max_idx);
      max_idx += 1;
      printf("%f : %f\n", ((float) max_idx)*SAMPLE_RATE/FFT_SIZE, max_mag);
    }
    gpio_xor_mask(1u << PIN_DEBUG_SW1); // Toggle SW1 GPIO

    // Send ready token back to Core 0 to request the next snapshot
    multicore_fifo_push_blocking(1);
  }
}

void process_buf_dsp(q31_t *buf) {
  static uint32_t hist_write_idx = 0;

  // Extract left channel
  buf_left_to_float(buf, temp_block);

  // Core 0 maintains a circular sample buffer using a technique that
  // use more memory but avoids large memory moves.  Core 1 uses the
  // Pico HW FIFO to request data to perform an FFT.  When core 0
  // gets this request, it copies from the circular "mirrored"
  // buffer to a snapshot buffer that core 1 will consume.  Core 0
  // must do this within the realtime deadline dicated by BLOCK_SIZE
  // (~1.3 ms for BLOCK_SIZE 64).  Core 1 does not operate real time.
  // It can take a long time to process the FFT, even taking time to
  // print results.  When it is done, it signals core 0 for the next
  // buffer and just blocks when core 0 signals it.  This architecture
  // implies that not all input samples get FFT'ed.

  memcpy(&fft_input_mirrored_history[hist_write_idx], temp_block,
         BLOCK_SIZE * sizeof(float32_t));
  memcpy(&fft_input_mirrored_history[hist_write_idx + FFT_SIZE], temp_block,
         BLOCK_SIZE * sizeof(float32_t));
  hist_write_idx = (hist_write_idx + BLOCK_SIZE) % FFT_SIZE;

  // Check if Core 1 sent a ready token.
  // Core 1 is a non-realtime process, so only feed it when it's ready.
  if (multicore_fifo_rvalid()) {
    // Pop the token to clear the FIFO
    (void)multicore_fifo_pop_blocking();

    // Copy into snapshot buffer
    float32_t *contiguous_window = &fft_input_mirrored_history[hist_write_idx];
    memcpy(fft_snapshot_buf, contiguous_window, FFT_SIZE * sizeof(float32_t));

    // Tell Core 1 to start doing an FFT
    multicore_fifo_push_blocking(1);
  }
}
