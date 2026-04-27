/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file does an FFT on the left channel and prints bins with
 * output levels greater than a threshold hard-coded below.
 */

#include "dsp_common.h"
#include <stdio.h>
#include <string.h>

#define THRESHOLD -30.0f

#define FFT_SIZE 1024
#define MAG_SIZE (FFT_SIZE / 2)
#define FFT_MAX_MAG 256.0f
#define LN_TO_DB_CONV 8.685889638f

static float32_t fft_input_mirrored_history[2*FFT_SIZE] = {0};
static __not_in_flash() float32_t hann_window[FFT_SIZE];
static float32_t fft_work_buffer[FFT_SIZE];
static float32_t fft_complex_output[FFT_SIZE];
static float32_t output_mag[MAG_SIZE];
static float32_t output_db[MAG_SIZE];

static arm_rfft_fast_instance_f32 fft_inst;

void init_dsp(void) {
    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        float32_t angle = 2.0f * PI_F * (float32_t)i / (float32_t)(FFT_SIZE - 1);
        hann_window[i] = 0.5f * (1.0f - arm_cos_f32(angle));
    }
}

static void convert_to_db(float32_t *mag_input, float32_t *db_output, uint32_t count) {
    float32_t scale = 1.0f / FFT_MAX_MAG;
    arm_scale_f32(mag_input, scale, db_output, count);

    for(int i=0; i<count; i++) if(db_output[i] < 1e-12f) db_output[i] = 1e-12f;

    arm_vlog_f32(db_output, db_output, count);
    arm_scale_f32(db_output, LN_TO_DB_CONV, db_output, count);
}

static void process_sliding_fft(float32_t *float_in, float32_t *output_mag_out)
{
  static uint32_t write_idx = 0;
  float32_t *contiguous_window;

  memcpy(&fft_input_mirrored_history[write_idx], float_in,
         BLOCK_SIZE * sizeof(float32_t));
  memcpy(&fft_input_mirrored_history[write_idx + FFT_SIZE],
         float_in, BLOCK_SIZE * sizeof(float32_t));
  write_idx = (write_idx + BLOCK_SIZE) % FFT_SIZE;

  contiguous_window = &fft_input_mirrored_history[write_idx];

  arm_mult_f32(contiguous_window, hann_window, fft_work_buffer, FFT_SIZE);
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, fft_complex_output, 0);
  arm_cmplx_mag_f32(fft_complex_output, output_mag_out, MAG_SIZE);
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  static int pr_limit = 0;

  buf_left_to_float(buf, float_in);
  process_sliding_fft(float_in, output_mag);
  convert_to_db(output_mag, output_db, MAG_SIZE);

  if (pr_limit >= 350) {
    pr_limit = 0;
    printf("*************************************\n");
    for (int i = 1; i < MAG_SIZE; i++)
      if (output_db[i] >= THRESHOLD)
        printf("%f : %f\n", i*SAMPLE_RATE/FFT_SIZE, output_db[i]);
  } else {
    pr_limit += 1;
  }
}
