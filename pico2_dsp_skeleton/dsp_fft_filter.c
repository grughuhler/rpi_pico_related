/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements filters using an FFT followed by an
 * inverse FFT.  Specifically, it uses the "overlap-save" method.
 * To define the filter, you make FIR coefficients.  For a size
 * 512 FFT, we need 449 taps.  For example, you can create
 * coefficents via:
 *
 *   gen_fir_firwin.py --btype lp --fc 4000 --taps 449\
 *      --window blackman --file fft_filter_coeffs.h
 *
 * Before compiling.
 *
 * You can try changing the FFT size.  I was able to run with a
 * 1024 point FFT.  The tap count must be FFT_SIZE-63.
 */

#include "dsp_common.h"
#include "fft_filter_coeffs.h"
#include <string.h>

#define FFT_SIZE 512
#define CMPLX_BINS (FFT_SIZE / 2)

static arm_rfft_fast_instance_f32 fft_inst;

// filter_H stores the frequency domain representation of the FIR filter.
static float32_t filter_H[FFT_SIZE];

// Overlap-Save history buffer.
// Length is FFT_SIZE. It stores the past (FFT_SIZE - BLOCK_SIZE) samples
// and the current BLOCK_SIZE samples.
static float32_t history_buffer[FFT_SIZE] = {0};

// Working buffers for FFT processing
static float32_t fft_work_buffer[FFT_SIZE];
static float32_t ifft_output[FFT_SIZE];

void init_dsp(void) {
    // Initialize the RFFT instance
    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);

    // Prepare the filter impulse response in the time domain
    // Zero out the work buffer first
    memset(fft_work_buffer, 0, sizeof(fft_work_buffer));

    // Copy the time-domain coefficients into the work buffer.
    // NUM_TAPS is defined in fft_filter_coeffs.h (should be 449)
    memcpy(fft_work_buffer, fir_coeffs, NUM_TAPS * sizeof(float32_t));

    // Compute the forward FFT of the zero-padded impulse response
    // to get the frequency domain coefficients (H[k])
    arm_rfft_fast_f32(&fft_inst, fft_work_buffer, filter_H, 0);
}

void process_buf_dsp(q31_t *buf) {
    float32_t float_in[BLOCK_SIZE];
    float32_t float_out[BLOCK_SIZE];

    // 1. Convert fixed-point input to float
    buf_left_to_float(buf, float_in);

    // 2. Overlap-Save: Shift history buffer left by BLOCK_SIZE
    // We move the last (FFT_SIZE - BLOCK_SIZE) samples to the beginning.
    memmove(history_buffer, 
            &history_buffer[BLOCK_SIZE], 
            (FFT_SIZE - BLOCK_SIZE) * sizeof(float32_t));

    // 3. Append new samples to the end of the history buffer
    memcpy(&history_buffer[FFT_SIZE - BLOCK_SIZE], 
           float_in, 
           BLOCK_SIZE * sizeof(float32_t));

    // 4. Compute Forward FFT of the history buffer
    // IMPORTANT: arm_rfft_fast_f32 modifies the input buffer in-place!
    // We must copy history_buffer to a temporary buffer before the FFT.
    memcpy(ifft_output, history_buffer, FFT_SIZE * sizeof(float32_t));
    arm_rfft_fast_f32(&fft_inst, ifft_output, fft_work_buffer, 0);

    // 5. Complex Multiplication in frequency domain
    // DC bin (real only)
    fft_work_buffer[0] = fft_work_buffer[0] * filter_H[0];
    // Nyquist bin (real only)
    fft_work_buffer[1] = fft_work_buffer[1] * filter_H[1];
    // Remaining bins (complex)
    arm_cmplx_mult_cmplx_f32(&fft_work_buffer[2], &filter_H[2], &fft_work_buffer[2], CMPLX_BINS - 1);

    // 6. Compute Inverse FFT
    arm_rfft_fast_f32(&fft_inst, fft_work_buffer, ifft_output, 1);

    // 7. Extract the valid output samples
    // In Overlap-Save, the first (NUM_TAPS - 1) samples of the IFFT output 
    // are corrupted by circular aliasing. The valid samples are the LAST 
    // BLOCK_SIZE samples, starting at index (FFT_SIZE - BLOCK_SIZE).
    memcpy(float_out, 
           &ifft_output[FFT_SIZE - BLOCK_SIZE], 
           BLOCK_SIZE * sizeof(float32_t));

    // 8. Convert float output back to fixed-point
    float_to_buf_left(float_out, buf);
}
