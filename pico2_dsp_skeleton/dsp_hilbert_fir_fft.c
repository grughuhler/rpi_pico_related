/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements a Hilbert transform using coefficients from
 * hilbert_fft_coeffs.h but optimized using FFT overlap-save convolution.
 * Generate the coefficients file using gen_hilbert_fir.py but the number
 * of taps must be FFT_SIZE - BLOCK_SIZE + 1.
 */

#include "dsp_common.h"
#include "hilbert_fft_coeffs.h"
#include <string.h>

// Determine FFT_SIZE based on BLOCK_SIZE to accommodate NUM_TAPS.
// FFT_SIZE must be a power of two.
#if BLOCK_SIZE == 32
#define FFT_SIZE 512
#elif BLOCK_SIZE == 64
#define FFT_SIZE 1024
#elif BLOCK_SIZE == 192
#define FFT_SIZE 2048
#elif BLOCK_SIZE == 384
#define FFT_SIZE 4096
#else
#error "Unsupported BLOCK_SIZE. Add a corresponding FFT_SIZE."
#endif

// Ensure that the FFT_SIZE is large enough for the overlap-save method
#if (NUM_TAPS + BLOCK_SIZE - 1 > FFT_SIZE)
#error "NUM_TAPS is too large for the current BLOCK_SIZE and FFT_SIZE"
#endif

#define CMPLX_BINS (FFT_SIZE / 2)

// Group delay of a linear phase FIR filter of length N is (N - 1) / 2
#define DELAY_SAMPLES ((NUM_TAPS - 1) / 2)
static float32_t delay_buffer[DELAY_SAMPLES];
static uint32_t delay_idx = 0;

static arm_rfft_fast_instance_f32 fft_inst;

// filter_H stores the frequency domain representation of the FIR filter.
static float32_t filter_H[FFT_SIZE];

// Overlap-Save history buffer.
// Length is FFT_SIZE. It stores the past (FFT_SIZE - BLOCK_SIZE)
// samples and the current BLOCK_SIZE samples.
static float32_t history_buffer[FFT_SIZE] = {0};

// Working buffers for FFT processing
static float32_t fft_work_buffer[FFT_SIZE];
static float32_t ifft_output[FFT_SIZE];

void init_dsp(void)
{
  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);

  memset(fft_work_buffer, 0, sizeof(fft_work_buffer));
  // The coefficients in the header are time-reversed for arm_fir_f32.
  // We must un-reverse them here for correct FFT convolution phase!
  for (int i = 0; i < NUM_TAPS; i++) {
    fft_work_buffer[i] = hilbert_coeffs[NUM_TAPS - 1 - i];
  }

  // Compute the forward FFT of the zero-padded impulse response
  // to get the frequency domain coefficients (H[k])
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, filter_H, 0);

  for (int i = 0; i < DELAY_SAMPLES; i++) {
    delay_buffer[i] = 0.0f;
  }
  delay_idx = 0;
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in_left[BLOCK_SIZE];
  float32_t float_out_in_phase[BLOCK_SIZE];
  float32_t float_out_quadrature[BLOCK_SIZE];

  // Extract the left channel
  buf_left_to_float(buf, float_in_left);

  // Overlap-Save: Shift history buffer left by BLOCK_SIZE
  // Move the last (FFT_SIZE - BLOCK_SIZE) samples to the beginning.
  memmove(history_buffer, &history_buffer[BLOCK_SIZE],
          (FFT_SIZE - BLOCK_SIZE) * sizeof(float32_t));

  // Append new samples to the end of the history buffer
  memcpy(&history_buffer[FFT_SIZE - BLOCK_SIZE], float_in_left,
         BLOCK_SIZE * sizeof(float32_t));

  // Compute Forward FFT of the history buffer
  // Must copy history_buffer to a temporary buffer before the FFT.
  memcpy(ifft_output, history_buffer, FFT_SIZE * sizeof(float32_t));
  arm_rfft_fast_f32(&fft_inst, ifft_output, fft_work_buffer, 0);

  // Complex Multiplication in frequency domain
  fft_work_buffer[0] = fft_work_buffer[0] * filter_H[0]; // DC bin (real)
  fft_work_buffer[1] = fft_work_buffer[1] * filter_H[1]; // Nyquist bin (real)
  // Remaining bins (complex)
  arm_cmplx_mult_cmplx_f32(&fft_work_buffer[2], &filter_H[2],
                           &fft_work_buffer[2], CMPLX_BINS - 1);

  // Compute Inverse FFT
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, ifft_output, 1);

  // Extract the valid output samples
  // In Overlap-Save, the valid samples are the LAST BLOCK_SIZE samples,
  // starting at index (FFT_SIZE - BLOCK_SIZE).
  memcpy(float_out_quadrature, &ifft_output[FFT_SIZE - BLOCK_SIZE],
         BLOCK_SIZE * sizeof(float32_t));

  // The output of the FIR filter has a group delay of DELAY_SAMPLES.
  // We need to delay the original input channel by the same amount
  // to output it on the in_phase channel.
  for (int i = 0; i < BLOCK_SIZE; i++) {
    float_out_in_phase[i] = delay_buffer[delay_idx];
    delay_buffer[delay_idx] = float_in_left[i];
    delay_idx++;
    if (delay_idx >= DELAY_SAMPLES) {
      delay_idx = 0;
    }
  }

  // Write outputs back to the interleaved fixed-point buffer.  buf[i]
  // is quadrature channel (transformed), buf[i+1] is in_phase channel
  // (delayed).
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i] = FAST_FLOAT_TO_FIXED(float_out_quadrature[j], 31);
    buf[i + 1] = FAST_FLOAT_TO_FIXED(float_out_in_phase[j], 31);
  }
}
