/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements an FIR filter using coefficients from fir_coeffs.h
 * which can be generated using gen_fir_firwin.py or gen_fir_firwin2.py.
 */

#include "dsp_common.h"
#include "fir_coeffs.h"

static arm_fir_instance_f32 fir_left;

void init_dsp(void)
{
  /* Tell FIR sizes and coefficients */
  arm_fir_init_f32(&fir_left, NUM_TAPS, (float32_t *)&fir_coeffs[0],
                   &fir_state[0], BLOCK_SIZE);
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out[BLOCK_SIZE];

  buf_left_to_float(buf, float_in);

  arm_fir_f32(&fir_left, float_in, float_out, BLOCK_SIZE);

  float_to_buf_left(float_out, buf);
}
