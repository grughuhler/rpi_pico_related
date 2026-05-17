/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements an IIR Hilbert Phase-Splitting network
 * using highly optimized coefficients for a 48828.125 Hz sample rate.
 */

#include "dsp_common.h"

#define LO_FREQ 1000.0f

#define NUM_STAGES_I_PATH 3
#define NUM_STAGES_Q_PATH 3

// Optimized coefficients for Fs = 48828.125 Hz Guaranteed to yield <
// 2.5 degrees of phase error between 40 Hz and 16 kHz.
float32_t iir_i_path_coeffs[NUM_STAGES_I_PATH * 5] = {
      0.210668f, 0.0f, -1.0f, 0.0f,   0.210668f,
      0.829548f, 0.0f, -1.0f, 0.0f,   0.829548f,
      0.977870f, 0.0f, -1.0f, 0.0f,   0.977870f,
};
float32_t iir_i_path_state[NUM_STAGES_I_PATH * 4] = {0};

float32_t iir_q_path_coeffs[NUM_STAGES_Q_PATH * 5] = {
      0.587919f, 0.0f, -1.0f, 0.0f,   0.587919f,
      0.936259f, 0.0f, -1.0f, 0.0f,   0.936259f,
      0.994548f, 0.0f, -1.0f, 0.0f,   0.994548f,
};
float32_t iir_q_path_state[NUM_STAGES_Q_PATH * 4] = {0};

static arm_biquad_casd_df1_inst_f32 iir_i_path;
static arm_biquad_casd_df1_inst_f32 iir_q_path;

// Path 2 requires a 1-sample delay before the all-pass network
static float32_t q_path_delay_state = 0.0f;

void init_dsp(void)
{
  arm_biquad_cascade_df1_init_f32(&iir_i_path, NUM_STAGES_I_PATH, 
                                  (float32_t *)&iir_i_path_coeffs[0], 
                                  &iir_i_path_state[0]);
                                  
  arm_biquad_cascade_df1_init_f32(&iir_q_path, NUM_STAGES_Q_PATH, 
                                  (float32_t *)&iir_q_path_coeffs[0], 
                                  &iir_q_path_state[0]);
                                  
  q_path_delay_state = 0.0f;
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out_i_path[BLOCK_SIZE];
  float32_t float_out_q_path[BLOCK_SIZE];
  float32_t float_in_delayed[BLOCK_SIZE];
  float32_t lo_hz = LO_FREQ, lo, lo90, qmix_out, mix_out;
  float32_t phase_lo_incr = 2.0f * PI_F * lo_hz / SAMPLE_RATE;
  static float32_t phase_lo = 0.0f, phase_lo90 = PI_F/2.0f;
  

  // Extract mono left channel from interleaved fixed-point buffer
  buf_left_to_float(buf, float_in);

  // Apply I path Biquad Cascade directly to the original input
  arm_biquad_cascade_df1_f32(&iir_i_path, float_in, float_out_i_path,
                             BLOCK_SIZE);

  // Q path requires a 1-sample delay before the all-pass cascade
  for (int i = 0; i < BLOCK_SIZE; i++) {
    float_in_delayed[i] = q_path_delay_state;
    q_path_delay_state = float_in[i];
  }

  // Apply q path Biquad Cascade to the delayed input
  arm_biquad_cascade_df1_f32(&iir_q_path, float_in_delayed, float_out_q_path,
                             BLOCK_SIZE);

  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    // Compute LO and LO90 samples
    lo = arm_sin_f32(phase_lo);
    phase_lo += phase_lo_incr;
    if (phase_lo > 2.0f * PI_F) phase_lo -= 2.0f * PI_F;

    lo90 = arm_sin_f32(phase_lo90);
    phase_lo90 += phase_lo_incr;
    if (phase_lo90 > 2.0f * PI_F) phase_lo90 -= 2.0f * PI_F;

    // Quadrature mixer output on left channel
    qmix_out = lo*float_out_i_path[j] + lo90*float_out_q_path[j];
    qmix_out *= 0.8f;
    buf[i]   = FAST_FLOAT_TO_FIXED(qmix_out, 31);

    // Regular mixer output on right channel
    mix_out = lo*float_out_i_path[j];
    buf[i+1] = FAST_FLOAT_TO_FIXED(mix_out, 31);
  }
}
