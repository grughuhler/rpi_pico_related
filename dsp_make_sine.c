/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This program runs on a Raspberry Pi Pico2 and serves as a DSP software
 * test framework.  It works with audio frequency stereo data from a
 * PCM1808 DAC, processes it using a triple-buffering scheme, and sends
 * the result to a PCM5102A DAC.  Audio levels are roughly line-level.
 * You could connect the PCM5102A output to a powered speaker.
 */

/* This file generates sine waves on both the left and right channels.
 * The frequencies are hard-coded below.
 */

#include "dsp_common.h"

void init_dsp(void)
{
}

void process_buf_dsp(q31_t *buf)
{
  float32_t hz_left = 240.0f;
  float32_t  hz_right = 360.0f;
  static float32_t phase_left = 0.0f, phase_right = 0.0f;
  float32_t phase_increment_left = 2.0f * PI_F * hz_left / SAMPLE_RATE;
  float32_t phase_increment_right = 2.0f * PI_F * hz_right / SAMPLE_RATE;

  for (int i = 0; i < SAMPLES_PER_BUFFER; i += 2) {
    float sample = arm_sin_f32(phase_left);
    // The "30" makes the sine wave go from -0.5 to 0.5.
    buf[i] = FAST_FLOAT_TO_FIXED(sample, 30);   // Left channel
    sample = arm_sin_f32(phase_right);
    buf[i+1] = FAST_FLOAT_TO_FIXED(sample, 30); // right channel
    phase_left += phase_increment_left;
    if (phase_left > 2.0f * PI_F) {
      phase_left -= 2.0f * PI_F;
    }
    phase_right += phase_increment_right;
    if (phase_right > 2.0f * PI_F) {
      phase_right -= 2.0f * PI_F;
    }
  }
}
