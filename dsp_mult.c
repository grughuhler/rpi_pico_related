/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file multiples the signals on the left and right channels and
 * outputs the result on the left channel.  If the two input signals are
 * sine waves with freqeuncies F1 and F1, the output will consist of the
 * sum of cosine waves with frequencies F1+F2 and F1-F2.
 */

#include "dsp_common.h"

void init_dsp(void) {}

void process_buf_dsp(q31_t *buf) {
  for (int i = 0; i < SAMPLES_PER_BUFFER; i += 2) {
    float32_t sample1 = FAST_FIXED_TO_FLOAT(buf[i], 31);
    float32_t sample2 = FAST_FIXED_TO_FLOAT(buf[i + 1], 31);

    float32_t result = sample1 * sample2;
    buf[i] = FAST_FLOAT_TO_FIXED(result, 31);
  }
}
