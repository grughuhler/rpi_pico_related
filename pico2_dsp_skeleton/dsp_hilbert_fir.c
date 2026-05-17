/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements a Hilbert transform using coefficients from
 * hilbert_coeffs.h which can be generated using gen_hilbert.py.
 */

#include "dsp_common.h"
#include "hilbert_coeffs.h"

static arm_fir_instance_f32 fir_left;

// Group delay of a linear phase FIR filter of length N is (N - 1) / 2
#define DELAY_SAMPLES ((NUM_TAPS - 1) / 2)
static float32_t delay_buffer[DELAY_SAMPLES];
static uint32_t delay_idx = 0;

void init_dsp(void)
{
  /* Tell FIR sizes and coefficients */
  arm_fir_init_f32(&fir_left, NUM_TAPS, (float32_t *)&hilbert_coeffs[0],
                   &hilbert_state[0], BLOCK_SIZE);
                   
  for(int i = 0; i < DELAY_SAMPLES; i++) {
    delay_buffer[i] = 0.0f;
  }
  delay_idx = 0;
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in_left[BLOCK_SIZE];
  float32_t float_out_left[BLOCK_SIZE];
  float32_t float_out_right[BLOCK_SIZE];

  // Extract the left channel
  buf_left_to_float(buf, float_in_left);

  // Perform Hilbert transform (FIR filter) on the left channel
  arm_fir_f32(&fir_left, float_in_left, float_out_left, BLOCK_SIZE);

  // The output of the FIR filter has a group delay of DELAY_SAMPLES.
  // We need to delay the original left channel by the same amount
  // to output it on the right channel in perfect quadrature.
  for (int i = 0; i < BLOCK_SIZE; i++) {
    float_out_right[i] = delay_buffer[delay_idx];
    delay_buffer[delay_idx] = float_in_left[i];
    delay_idx++;
    if (delay_idx >= DELAY_SAMPLES) {
      delay_idx = 0;
    }
  }

  // Write outputs back to the interleaved fixed-point buffer.
  // buf[i] is left channel (transformed), buf[i+1] is right channel (delayed).
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i] = FAST_FLOAT_TO_FIXED(float_out_left[j], 31);
    buf[i+1] = FAST_FLOAT_TO_FIXED(float_out_right[j], 31);
  }
}
