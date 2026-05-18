/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements a Granular / Delay-Line Pitch Shifter.
 * It uses a dual read-pointer architecture with a triangle window
 * crossfade to naturally pitch shift speech.
 * 
 * Includes an IIR Anti-Aliasing filter and Cubic Interpolation
 * for maximum audio quality.
 */

#include "dsp_common.h"

// -------------------------------------------------------------
// Change pitch ratio here.
// 1.0f = No change
// 1.25f = Pitch UP (e.g. higher voice)
// 0.8f  = Pitch DOWN (e.g. lower voice)
// -------------------------------------------------------------
float32_t PITCH_SHIFT_RATIO = 1.1f;

// The circular delay buffer size (must be power of 2 for fast masking)
#define DELAY_BUFFER_SIZE 4096 
#define DELAY_BUFFER_MASK 4095

// The length of the sliding "window" / maximum delay.
#define WINDOW_LENGTH 2000.0f

// Minimum delay to prevent cubic interpolation from reading ahead of the write pointer
#define SAFE_ZONE 4.0f

static float32_t delay_buffer[DELAY_BUFFER_SIZE];
static int write_ptr = 0;

// The delays of our two read pointers.
static float32_t delay1 = SAFE_ZONE;
static float32_t delay2 = SAFE_ZONE + (WINDOW_LENGTH / 2.0f);

// 4th Order Butterworth Low-Pass Filter (12 kHz Cutoff)
// Prevents high-frequency aliasing when pitching UP.
float32_t aa_filter_coeffs[2 * 5] = {
  0.089065f, 0.178129f, 0.089065f, 0.027696f, -0.039743f,
  1.000000f, 2.000000f, 1.000000f, 0.038534f, -0.446605f,
};
float32_t aa_filter_state[2 * 4] = {0};
static arm_biquad_casd_df1_inst_f32 aa_filter;


void init_dsp(void)
{
  for (int i = 0; i < DELAY_BUFFER_SIZE; i++) {
    delay_buffer[i] = 0.0f;
  }
  write_ptr = 0;
  delay1 = SAFE_ZONE;
  delay2 = SAFE_ZONE + (WINDOW_LENGTH / 2.0f);
    
  arm_biquad_cascade_df1_init_f32(&aa_filter, 2, aa_filter_coeffs,
				  aa_filter_state);
}

// Function to read from the delay buffer with Catmull-Rom Cubic Interpolation
static inline float32_t read_delay_interp(float32_t delay)
{
  // Read position is relative to current write pointer
  float32_t read_pos = (float32_t)write_ptr - delay;
    
  // Wrap safely
  if (read_pos < 0.0f) {
    read_pos += (float32_t)DELAY_BUFFER_SIZE;
  }
    
  int idx1 = ((int)read_pos) & DELAY_BUFFER_MASK;
  int idx0 = (idx1 - 1) & DELAY_BUFFER_MASK;
  int idx2 = (idx1 + 1) & DELAY_BUFFER_MASK;
  int idx3 = (idx1 + 2) & DELAY_BUFFER_MASK;
    
  float32_t x = read_pos - (int)read_pos;
    
  float32_t y0 = delay_buffer[idx0];
  float32_t y1 = delay_buffer[idx1];
  float32_t y2 = delay_buffer[idx2];
  float32_t y3 = delay_buffer[idx3];
    
  // Catmull-Rom coefficients
  float32_t c0 = y1;
  float32_t c1 = 0.5f * (y2 - y0);
  float32_t c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
  float32_t c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    
  return ((c3 * x + c2) * x + c1) * x + c0;
}

// Function to calculate the triangle window weight based on delay
static inline float32_t get_window_weight(float32_t delay)
{
  // Normalize delay so SAFE_ZONE is 0.0 and SAFE_ZONE+WINDOW_LENGTH is 1.0
  float32_t norm = (delay - SAFE_ZONE) / WINDOW_LENGTH;
  if (norm < 0.5f) {
    return norm * 2.0f;
  } else {
    return (1.0f - norm) * 2.0f;
  }
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_filtered[BLOCK_SIZE];
    
  // Extract mono left channel from interleaved fixed-point buffer
  buf_left_to_float(buf, float_in);
    
  // Apply 12 kHz Anti-Aliasing Low-Pass Filter
  arm_biquad_cascade_df1_f32(&aa_filter, float_in, float_filtered, BLOCK_SIZE);
    
  // The rate at which the delay changes (read speed vs write speed)
  float32_t delay_rate = 1.0f - PITCH_SHIFT_RATIO;
    
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    // 1. Write incoming filtered sample to circular buffer
    delay_buffer[write_ptr] = float_filtered[j];
        
    // 2. Read from the two delay taps using cubic interpolation
    float32_t val1 = read_delay_interp(delay1);
    float32_t val2 = read_delay_interp(delay2);
        
    // 3. Calculate crossfade weights (triangle window)
    float32_t w1 = get_window_weight(delay1);
    float32_t w2 = get_window_weight(delay2);
        
    // 4. Mix the two overlapping windows
    float32_t out = (val1 * w1) + (val2 * w2);
        
    // 5. Output mixed audio to both left and right channels
    buf[i]   = FAST_FLOAT_TO_FIXED(out, 31);
    //    buf[i+1] = buf[i];
        
    // 6. Advance write pointer circularly
    write_ptr = (write_ptr + 1) & DELAY_BUFFER_MASK;
        
    // 7. Advance read pointer delays
    delay1 += delay_rate;
    delay2 += delay_rate;
        
    // 8. Safely wrap delays if they reach the ends of the window bounds
    if (delay1 < SAFE_ZONE) delay1 += WINDOW_LENGTH;
    else if (delay1 >= (SAFE_ZONE + WINDOW_LENGTH)) delay1 -= WINDOW_LENGTH;
    
    if (delay2 < SAFE_ZONE) delay2 += WINDOW_LENGTH;
    else if (delay2 >= (SAFE_ZONE + WINDOW_LENGTH)) delay2 -= WINDOW_LENGTH;
  }
}
