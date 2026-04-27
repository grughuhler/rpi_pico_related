/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

#include "dsp_common.h"

void buf_left_to_float(q31_t *buf, float32_t *float_in)
{
  // De-interleave buf[i] (left channel) and convert to float
  // buf[i] is left, buf[i+1] is right.
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    float_in[j] = FAST_FIXED_TO_FLOAT(buf[i], 31);
  }
}

void float_to_buf_left(float32_t *float_out, q31_t *buf)
{
  // Convert back and place it in buf[i]
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i] = FAST_FLOAT_TO_FIXED(float_out[j], 31);
  }
}
