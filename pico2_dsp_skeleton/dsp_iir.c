/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements an IIR filter using coefficients from iir_coeffs.h
 * which can be generated using gen_iir.py.
 */

#include "dsp_common.h"
#include "iir_coeffs.h"

static arm_biquad_cascade_df2T_instance_f32 S2;

void init_dsp(void)
{
  arm_biquad_cascade_df2T_init_f32(&S2, NUM_STAGES, iir_coeffs,
                                   iir_state);
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out[BLOCK_SIZE];

  buf_left_to_float(buf, float_in);

  arm_biquad_cascade_df2T_f32(&S2, float_in, float_out, BLOCK_SIZE);

  float_to_buf_left(float_out, buf);
}
