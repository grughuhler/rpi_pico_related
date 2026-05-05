#include "dsp_common.h"
#include <math.h>

/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements a tone detector using a Goertzel detector.
 * The Pico2's LED will light when the tone is detected.  The frequency
 * of the tone to detect is hard-coded below.
 */

#define FREQ_TO_DETECT 2000.0f

typedef struct {
  float32_t coeff;
  float32_t v1, v2;
  float32_t total_energy;
  uint32_t samples_processed;
  uint32_t samples_needed;
  float32_t confidence_score;
  bool ready;
  float32_t dc_prev_in;
  float32_t dc_prev_out;
} GoertzelDetector;

static GoertzelDetector detector;

static void __attribute__ ((noinline)) init_goertzel(GoertzelDetector *S, float32_t target_f) {
  double omega = (2.0 * PI_D * (double) target_f) / (double) SAMPLE_RATE;
  S->coeff = 2.0 * cos(omega);
    
  float32_t cycles = (target_f <= 400.0) ? 4.0f : 8.0f;
  uint32_t ideal_samples = (uint32_t)(cycles * SAMPLE_RATE / target_f);

  if (ideal_samples < BLOCK_SIZE) ideal_samples = BLOCK_SIZE;

  S->samples_needed = ((ideal_samples + BLOCK_SIZE - 1) / BLOCK_SIZE) * BLOCK_SIZE;
  S->v1 = 0; S->v2 = 0; S->total_energy = 0; S->samples_processed = 0;
  S->dc_prev_in = 0; S->dc_prev_out = 0;
  S->ready = false;
}

void init_dsp(void)
{
  init_goertzel(&detector, FREQ_TO_DETECT);
}

static void goertzel_update(GoertzelDetector *S, float32_t *pSrc) {
  float32_t v0;
  float32_t v1 = S->v1;
  float32_t v2 = S->v2;
  float32_t energy_sum = 0.0f;
  const float32_t alpha = 0.99f; // DC Blocker coefficient

  for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
    float32_t clean_sample = pSrc[i] - S->dc_prev_in + (alpha * S->dc_prev_out);
    S->dc_prev_in = pSrc[i];
    S->dc_prev_out = clean_sample;

    v0 = clean_sample + (S->coeff * v1) - v2;
    v2 = v1;
    v1 = v0;
    energy_sum += clean_sample * clean_sample;
  }

  S->v1 = v1; S->v2 = v2;
  S->total_energy += energy_sum;
  S->samples_processed += BLOCK_SIZE;

  if (S->samples_processed >= S->samples_needed) {
    float32_t pwr = (v1 * v1) + (v2 * v2) - (S->coeff * v1 * v2);
    float32_t n = (float32_t)S->samples_processed;

    if (S->total_energy > 1e-9f) {
      S->confidence_score = (2.0f * pwr) / (S->total_energy * n);
    } else {
      S->confidence_score = 0.0f;
    }

    S->ready = true;
    S->v1 = 0; S->v2 = 0; S->total_energy = 0; S->samples_processed = 0;
  } else {
    S->ready = false;
  }

  if (detector.ready) {
    if (detector.confidence_score > 0.9f) {
      gpio_put(PICO_DEFAULT_LED_PIN, 1);
    } else {
      gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
  }
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];

  buf_left_to_float(buf, float_in);
  goertzel_update(&detector, float_in);
}
