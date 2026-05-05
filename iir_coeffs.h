/*
 * CMSIS-DSP Biquad Coefficients for arm_biquad_cascade_df2T_f32
 * Generated for RP2350 / Pico 2
 * Prototype: cheby1, Shape: lowpass
 * Order: 6, Total Biquad Stages: 3
 * Cutoff(s): [4000.0] Hz, FS: 48828.125 Hz
 */

#ifndef IIR_COEFFS_H
#define IIR_COEFFS_H

#include "arm_math.h"

#define NUM_STAGES 3

float32_t iir_coeffs[15] = {
      0.0000145309f,   0.0000290618f,   0.0000145309f,   1.7533887433f,  -0.7839467479f, // Stage 1
      1.0000000000f,   2.0000000000f,   1.0000000000f,   1.7044601636f,  -0.8414544511f, // Stage 2
      1.0000000000f,   2.0000000000f,   1.0000000000f,   1.6913061776f,  -0.9405616098f // Stage 3
};

float32_t iir_state[2 * NUM_STAGES] = {0};

#endif // IIR_COEFFS_H