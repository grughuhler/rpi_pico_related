/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements the Hilbert transform using an IIR Phase
 * Splitting network with coefficients optimized for a 48828.125 Hz
 * sample rate.
 */

#include "dsp_common.h"
#define NUM_STAGES_PATH1 5
#define NUM_STAGES_PATH2 5

// Optimized coefficients for Fs = 48828.125 Hz should yield < 0.24
// degrees of phase error between 40 Hz and 16 kHz.
float32_t iir_path1_coeffs[NUM_STAGES_PATH1 * 5] = {
  0.091128f, 0.0f, -1.0f, 0.0f,   0.091128f,
  0.539201f, 0.0f, -1.0f, 0.0f,   0.539201f,
  0.839705f, 0.0f, -1.0f, 0.0f,   0.839705f,
  0.952009f, 0.0f, -1.0f, 0.0f,   0.952009f,
  0.988099f, 0.0f, -1.0f, 0.0f,   0.988099f,
};
float32_t iir_path1_state[NUM_STAGES_PATH1 * 4] = {0};

float32_t iir_path2_coeffs[NUM_STAGES_PATH2 * 5] = {
  0.307112f, 0.0f, -1.0f, 0.0f,   0.307112f,
  0.720435f, 0.0f, -1.0f, 0.0f,   0.720435f,
  0.911190f, 0.0f, -1.0f, 0.0f,   0.911190f,
  0.974934f, 0.0f, -1.0f, 0.0f,   0.974934f,
  0.996503f, 0.0f, -1.0f, 0.0f,   0.996503f,
};
float32_t iir_path2_state[NUM_STAGES_PATH2 * 4] = {0};

static arm_biquad_casd_df1_inst_f32 iir_path1;
static arm_biquad_casd_df1_inst_f32 iir_path2;

// Path 2 requires a 1-sample delay before the all-pass network
static float32_t path2_delay_state = 0.0f;

void init_dsp(void)
{
  arm_biquad_cascade_df1_init_f32(&iir_path1, NUM_STAGES_PATH1, 
                                  (float32_t *)&iir_path1_coeffs[0], 
                                  &iir_path1_state[0]);
                                  
  arm_biquad_cascade_df1_init_f32(&iir_path2, NUM_STAGES_PATH2, 
                                  (float32_t *)&iir_path2_coeffs[0], 
                                  &iir_path2_state[0]);
                                  
  path2_delay_state = 0.0f;
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out_path1[BLOCK_SIZE];
  float32_t float_out_path2[BLOCK_SIZE];
  float32_t float_in_delayed[BLOCK_SIZE];

  // Extract mono left channel from interleaved fixed-point buffer
  buf_left_to_float(buf, float_in);

  // Apply Path 1 Biquad Cascade directly to the original input
  arm_biquad_cascade_df1_f32(&iir_path1, float_in, float_out_path1,
                             BLOCK_SIZE);

  // Path 2 requires a 1-sample delay before the all-pass cascade
  for (int i = 0; i < BLOCK_SIZE; i++) {
    float_in_delayed[i] = path2_delay_state;
    path2_delay_state = float_in[i];
  }

  // Apply Path 2 Biquad Cascade to the delayed input
  arm_biquad_cascade_df1_f32(&iir_path2, float_in_delayed, float_out_path2,
                             BLOCK_SIZE);

#define HILBERT_OUT
#ifdef HILBERT_OUT
  // Re-interleave the two paths back into the audio buffer.
  // The output of Path 1 (Left) and Path 2 (Right) will have
  // a 90-degree phase difference.
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i]   = FAST_FLOAT_TO_FIXED(float_out_path1[j], 31);
    buf[i+1] = FAST_FLOAT_TO_FIXED(float_out_path2[j], 31);
  }
#else
  // Output envelope on left channel, raw input signal on right.
  // Hint: Use non-carrier-suppressed AM as input signal
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i+1] = FAST_FLOAT_TO_FIXED(float_out_path1[j], 31);
    float32_t sout, sin = float_out_path1[j]*float_out_path1[j] +
      float_out_path2[j]*float_out_path2[j];
    arm_sqrt_f32(sin, &sout);
    buf[i] = FAST_FLOAT_TO_FIXED(sout, 31);
  }
#endif
}
