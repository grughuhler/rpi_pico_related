/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements a Hilbert transform using an FIR with
 * coefficients from hilbert_coeffs.h which can be generated using
 * gen_hilbert_fir.py.
 */

#include "dsp_common.h"
#include "hilbert_coeffs.h"

static arm_fir_instance_f32 fir_quadrature;

// Group delay of a linear phase FIR filter of length N is (N - 1) / 2
#define DELAY_SAMPLES ((NUM_TAPS - 1) / 2)
static float32_t delay_buffer[DELAY_SAMPLES];
static uint32_t delay_idx = 0;

void init_dsp(void)
{
  /* Tell FIR sizes and coefficients */
  arm_fir_init_f32(&fir_quadrature, NUM_TAPS, (float32_t *)&hilbert_coeffs[0],
                   &hilbert_state[0], BLOCK_SIZE);
                   
  for(int i = 0; i < DELAY_SAMPLES; i++) {
    delay_buffer[i] = 0.0f;
  }
  delay_idx = 0;
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in_left[BLOCK_SIZE];
  float32_t float_out_quadrature[BLOCK_SIZE];
  float32_t float_out_in_phase[BLOCK_SIZE];

  // Extract the left channel to float32
  buf_left_to_float(buf, float_in_left);

  // Perform Hilbert transform (FIR filter) on the quadrature channel
  arm_fir_f32(&fir_quadrature, float_in_left, float_out_quadrature,
	      BLOCK_SIZE);

  // The output of the FIR filter has a group delay of DELAY_SAMPLES.
  // We need to delay the original quadrature channel by the same amount
  // to output it on the in_phase channel in perfect quadrature.
  for (int i = 0; i < BLOCK_SIZE; i++) {
    float_out_in_phase[i] = delay_buffer[delay_idx];
    delay_buffer[delay_idx] = float_in_left[i];
    delay_idx++;
    if (delay_idx >= DELAY_SAMPLES) {
      delay_idx = 0;
    }
  }

#define HILBERT_OUT
#ifdef HILBERT_OUT
  // Write outputs back to the interleaved fixed-point buffer. buf[i]
  // is quadrature channel (transformed), buf[i+1] is in_phase channel
  // (appropriately delayed).
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i] = FAST_FLOAT_TO_FIXED(float_out_quadrature[j], 31);
    buf[i+1] = FAST_FLOAT_TO_FIXED(float_out_in_phase[j], 31);
  }
#else
  // Output envelope on left channel, raw input signal on left.
  // Hint: Use non-carrier-suppressed AM as input signal
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i+1] = FAST_FLOAT_TO_FIXED(float_out_quadrature[j], 31);
    float32_t sout, sin = float_out_quadrature[j]*float_out_quadrature[j] +
      float_out_in_phase[j]*float_out_in_phase[j];
    arm_sqrt_f32(sin, &sout);
    buf[i] = FAST_FLOAT_TO_FIXED(sout, 31);
  }
#endif
}
