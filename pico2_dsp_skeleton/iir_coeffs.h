/*
 * CMSIS-DSP Biquad Coefficients for arm_biquad_cascade_df2T_f32
 * Generated for RP2350 / Pico 2
 * Prototype: ellip, Shape: lowpass
 * Order: 12, Total Biquad Stages: 6
 * Cutoff(s): [4000.0] Hz, FS: 48828.125 Hz
 */

#ifndef IIR_COEFFS_H
#define IIR_COEFFS_H

#include "arm_math.h"

#define NUM_STAGES 6

float32_t iir_coeffs[30] = {
      0.0131300106f,  -0.0075546896f,   0.0131300106f,   1.6724946379f,  -0.7300549408f, // Stage 1
      1.0000000000f,  -1.6125856879f,   1.0000000000f,   1.7131728791f,  -0.8913037947f, // Stage 2
      1.0000000000f,  -1.7155633159f,   1.0000000000f,   1.7333037972f,  -0.9710996755f, // Stage 3
      1.0000000000f,  -1.7350820978f,   1.0000000000f,   1.7388776586f,  -0.9931786761f, // Stage 4
      1.0000000000f,  -1.7393347499f,   1.0000000000f,   1.7402256125f,  -0.9984519653f, // Stage 5
      1.0000000000f,  -1.7402464149f,   1.0000000000f,   1.7406241779f,  -0.9997241248f // Stage 6
};

float32_t iir_state[2 * NUM_STAGES] = {0};

#endif // IIR_COEFFS_H