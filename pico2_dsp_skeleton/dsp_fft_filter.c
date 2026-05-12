/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements filters using an FFT followed by an
 * inverse FFT.  Specifically, it uses the "overlap-save" method.
 * The result is the same as with an FIR, but for large numbers of
 * taps the FFT runs faster.  With BLOCK_SIZE 64, there is not
 * too much benefit on the Pico2.  A 961 FIR tap filter works which is
 * a bit better than what dsp_fir.c can manage.
 *
 * The FFT must be a power of two (like 512 or 1024).
 * The method initializes by doing an FFT on a set of FIR coefficients.
 * The number of FIR coefficients must be FFT_SIZE - BLOCK_SIZE + 1 so
 * for example with the normal BLOCK_SIZE of 64, 449 FIR taps makes
 * FFT_SIZE 512 becuase 512 - 64 + 1 = 449.
 *
 * To define the filter, you make FIR coefficients using the usual
 * python scripts. For example, you can create coefficents via:
 *
 *   gen_fir_firwin.py --btype lp --fc 4000 --taps 961\
 *      --window blackman --file fft_filter_coeffs.h
 *
 * and then "make" to build the Pico2 software.
 *
 * For BLOCK_SIZE 64, I have tested with 449 FIR taps (FFT_SIZE 512) and
 * 961 FIR taps (FFT_SIZE 1024).  A larger FFT cannot complete within
 * the required 1.3 milliseconds.  With BLOCK_SIZE changed to 256, a
 * filter with 1793 FIR taps (FFT_SIZE 2048) worked, but almost all testing
 * was with BLOCK_SIZE 64.
 */

#include "dsp_common.h"
#include "fft_filter_coeffs.h"
#include <string.h>

/* FFT_SIZE must be a power of two so choose NUM_TAPS accordingly */
#define FFT_SIZE (NUM_TAPS + BLOCK_SIZE - 1)
#define CMPLX_BINS (FFT_SIZE / 2)

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
  memcpy(fft_work_buffer, fir_coeffs, NUM_TAPS * sizeof(float32_t));

  // Compute the forward FFT of the zero-padded impulse response
  // to get the frequency domain coefficients (H[k])
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, filter_H, 0);
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out[BLOCK_SIZE];

  buf_left_to_float(buf, float_in);

  // Overlap-Save: Shift history buffer left by BLOCK_SIZE
  // Move the last (FFT_SIZE - BLOCK_SIZE) samples to the beginning.
  memmove(history_buffer, &history_buffer[BLOCK_SIZE], 
	  (FFT_SIZE - BLOCK_SIZE) * sizeof(float32_t));

  // Append new samples to the end of the history buffer
  memcpy(&history_buffer[FFT_SIZE - BLOCK_SIZE], float_in, 
	 BLOCK_SIZE * sizeof(float32_t));

  // Compute Forward FFT of the history buffer
  // Must copy history_buffer to a temporary buffer before the FFT.
  memcpy(ifft_output, history_buffer, FFT_SIZE * sizeof(float32_t));
  arm_rfft_fast_f32(&fft_inst, ifft_output, fft_work_buffer, 0);

  // Complex Multiplication in frequency domain
  fft_work_buffer[0] = fft_work_buffer[0] * filter_H[0];  // DC bin (real)
  fft_work_buffer[1] = fft_work_buffer[1] * filter_H[1]; // Nyquist bin )real)
  // Remaining bins (complex)
  arm_cmplx_mult_cmplx_f32(&fft_work_buffer[2], &filter_H[2],
			   &fft_work_buffer[2], CMPLX_BINS - 1);

  // Compute Inverse FFT
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, ifft_output, 1);

  // Extract the valid output samples
  // In Overlap-Save, the first (NUM_TAPS - 1) samples of the IFFT output 
  // are corrupted by circular aliasing. The valid samples are the LAST 
  // BLOCK_SIZE samples, starting at index (FFT_SIZE - BLOCK_SIZE).
  memcpy(float_out, &ifft_output[FFT_SIZE - BLOCK_SIZE], 
	 BLOCK_SIZE * sizeof(float32_t));

  float_to_buf_left(float_out, buf);
}
