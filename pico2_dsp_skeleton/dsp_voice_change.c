/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file implements a WSOLA (Waveform Similarity Overlap-Add)
 * Pitch Shifter.  It uses a dynamic crossfade triggered by a smart
 * cross-correlation search to align waveforms and eliminate phase
 * cancellation artifacts.
 * 
 * Includes an IIR Anti-Aliasing filter and Cubic Interpolation
 * for maximum audio quality.
 */

#include "dsp_common.h"
#include "sinc_table.h"

// -------------------------------------------------------------
// Change pitch ratio here.
// 1.0f = No change
// 1.25f = Pitch UP (e.g. higher voice)
// 0.8f  = Pitch DOWN (e.g. lower voice)
// -------------------------------------------------------------
float32_t PITCH_SHIFT_RATIO = 1.3f;

// The circular delay buffer size (must be power of 2 for fast masking)
#define DELAY_BUFFER_SIZE 4096 
#define DELAY_BUFFER_MASK 4095

// WSOLA Parameters
#define WINDOW_LENGTH 1000.0f
#define SAFE_ZONE 420.0f
#define CROSSFADE_LEN 400
#define TEMPLATE_SIZE 800
#define SEARCH_RANGE 400
#define DECIMATION 4
#define SEARCH_TRIGGER_SAMPLES 400

static float32_t delay_buffer[DELAY_BUFFER_SIZE];
static int write_ptr = 0;

static float32_t main_delay = SAFE_ZONE + (WINDOW_LENGTH / 2.0f);
static float32_t xfade_delay = 0.0f;
typedef enum {
    STATE_IDLE,
    STATE_EXTRACT_COARSE,
    STATE_SEARCH_COARSE,
    STATE_EXTRACT_FINE,
    STATE_SEARCH_FINE,
    STATE_WAIT_CROSSFADE,
    STATE_CROSSFADE
} WSOLA_State;

static WSOLA_State wsola_state = STATE_IDLE;
static int crossfade_progress = 0;

// Background Search State Variables
static float32_t search_target_delay = 0.0f;
static float32_t search_active_delay = 0.0f;

#define COARSE_TEMPLATE_SIZE (TEMPLATE_SIZE / DECIMATION)
#define COARSE_SEARCH_RANGE (SEARCH_RANGE / DECIMATION)
static float32_t coarse_template[COARSE_TEMPLATE_SIZE];
static float32_t coarse_search[COARSE_TEMPLATE_SIZE + 2 * COARSE_SEARCH_RANGE];

static float32_t max_coarse_corr = -1e9f;
static int best_coarse_offset_idx = 0;
static int current_coarse_offset = 0;

static int center_fine_offset = 0;
static float32_t max_fine_corr = -1e9f;
static int best_fine_offset = 0;
static int current_fine_offset = 0;
static int fine_search_start = 0;
static int fine_search_end = 0;
static float32_t fine_template_buf[TEMPLATE_SIZE];

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
  
  main_delay = SAFE_ZONE + (WINDOW_LENGTH / 2.0f);
  xfade_delay = 0.0f;
  wsola_state = STATE_IDLE;
  crossfade_progress = 0;
    
  arm_biquad_cascade_df1_init_f32(&aa_filter, 2, aa_filter_coeffs,
                                  aa_filter_state);
}

// Function to read from the delay buffer with 16-point Sinc Interpolation
static inline float32_t read_delay_interp(float32_t delay)
{
  // Read position is relative to current write pointer
  float32_t read_pos = (float32_t)write_ptr - delay;
    
  // Wrap safely
  if (read_pos < 0.0f) {
    read_pos += (float32_t)DELAY_BUFFER_SIZE;
  }
    
  int int_pos = (int)read_pos;
  float32_t frac = read_pos - (float32_t)int_pos;
  
  // Phase lookup for sinc table (64 phases)
  int phase = (int)(frac * 64.0f);
  if (phase > 63) phase = 63;
  if (phase < 0) phase = 0;
  
  const float32_t *coeffs = sinc_table[phase];
  float32_t out = 0.0f;
  
  // 16-point convolution
  // Center of sinc is between index 7 and 8
  int start_idx = int_pos - 7;
  
  for (int i = 0; i < 16; i++) {
    int idx = (start_idx + i) & DELAY_BUFFER_MASK;
    out += delay_buffer[idx] * coeffs[i];
  }
  
  return out;
}

void process_buf_dsp(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_filtered[BLOCK_SIZE];
  static uint32_t which= 0, which_cnt = 0;
  uint32_t orig_sample;
    
  // Extract mono left channel from interleaved fixed-point buffer
  buf_left_to_float(buf, float_in);
    
  // Apply 12 kHz Anti-Aliasing Low-Pass Filter
  arm_biquad_cascade_df1_f32(&aa_filter, float_in, float_filtered, BLOCK_SIZE);
    
  // The rate at which the delay changes (read speed vs write speed)
  float32_t delay_rate = 1.0f - PITCH_SHIFT_RATIO;
    
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    // Write incoming filtered sample to circular buffer
    delay_buffer[write_ptr] = float_filtered[j];
        
    float32_t out;
    
    orig_sample = buf[i];  // Save in case we want to output it

    // ---------------------------------------------------------
    // Incremental Background WSOLA Search State Machine
    // ---------------------------------------------------------
    if (wsola_state == STATE_IDLE && delay_rate != 0.0f) {
      float32_t margin = CROSSFADE_LEN * fabsf(delay_rate);
      if (margin > WINDOW_LENGTH / 2.0f) margin = WINDOW_LENGTH / 2.0f;
      
      // Trigger search 400 samples before the crossfade starts
      float32_t early_margin = margin +
        (SEARCH_TRIGGER_SAMPLES * fabsf(delay_rate));
      
      if (delay_rate < 0.0f && main_delay <= SAFE_ZONE + early_margin) {
        search_target_delay = SAFE_ZONE + WINDOW_LENGTH;
        search_active_delay = main_delay;
        wsola_state = STATE_EXTRACT_COARSE;
      } else if (delay_rate > 0.0f &&
                 main_delay >= SAFE_ZONE + WINDOW_LENGTH - early_margin) {
        search_target_delay = SAFE_ZONE;
        search_active_delay = main_delay;
        wsola_state = STATE_EXTRACT_COARSE;
      }
    } else if (wsola_state == STATE_EXTRACT_COARSE) {
      int start_idx_active = write_ptr - (int)search_active_delay;
      for (int k = 0; k < COARSE_TEMPLATE_SIZE; k++) {
        coarse_template[k] =
          delay_buffer[(start_idx_active - (k * DECIMATION)) & DELAY_BUFFER_MASK];
      }
      
      float32_t base_delay = search_target_delay - SEARCH_RANGE;
      int start_idx_search = write_ptr - (int)base_delay;
      for (int k = 0; k < COARSE_TEMPLATE_SIZE + 2 * COARSE_SEARCH_RANGE;
           k++) {
        coarse_search[k] =
          delay_buffer[(start_idx_search - (k * DECIMATION)) & DELAY_BUFFER_MASK];
      }
      
      max_coarse_corr = -1e9f;
      current_coarse_offset = 0;
      wsola_state = STATE_SEARCH_COARSE;
    } else if (wsola_state == STATE_SEARCH_COARSE) {
      // Compute ONE coarse offset per sample
      float32_t corr = 0.0f;
      float32_t *pT = coarse_template;
      float32_t *pS = &coarse_search[current_coarse_offset];
      for (int k = 0; k < COARSE_TEMPLATE_SIZE; k++) {
        corr += pT[k] * pS[k];
      }
      
      if (corr > max_coarse_corr) {
        max_coarse_corr = corr;
        best_coarse_offset_idx = current_coarse_offset;
      }
      
      current_coarse_offset++;
      if (current_coarse_offset > 2 * COARSE_SEARCH_RANGE) {
        wsola_state = STATE_EXTRACT_FINE;
      }
    } else if (wsola_state == STATE_EXTRACT_FINE) {
      center_fine_offset = best_coarse_offset_idx * DECIMATION;
      fine_search_start = center_fine_offset - (DECIMATION - 1);
      if (fine_search_start < 0) fine_search_start = 0;
      
      fine_search_end = center_fine_offset + (DECIMATION - 1);
      if (fine_search_end > 2 * SEARCH_RANGE)
        fine_search_end = 2 * SEARCH_RANGE;
      
      int start_idx_active = write_ptr - (int)search_active_delay;
      for (int k = 0; k < TEMPLATE_SIZE; k++) {
        fine_template_buf[k] =
          delay_buffer[(start_idx_active - k) & DELAY_BUFFER_MASK];
      }
      
      max_fine_corr = -1e9f;
      current_fine_offset = fine_search_start;
      wsola_state = STATE_SEARCH_FINE;
    } else if (wsola_state == STATE_SEARCH_FINE) {
      // Compute ONE fine offset per sample
      float32_t corr = 0.0f;
      float32_t base_delay = search_target_delay - SEARCH_RANGE;
      int start_idx_search = write_ptr - (int)base_delay;
      int fine_start_idx_search = start_idx_search - current_fine_offset;
      
      for (int k = 0; k < TEMPLATE_SIZE; k++) {
        float32_t s_val =
          delay_buffer[(fine_start_idx_search - k) & DELAY_BUFFER_MASK];
        corr += fine_template_buf[k] * s_val;
      }
      
      if (corr > max_fine_corr) {
        max_fine_corr = corr;
        best_fine_offset = current_fine_offset;
      }
      
      current_fine_offset++;
      if (current_fine_offset > fine_search_end) {
        float32_t best_delay = base_delay + (float32_t)best_fine_offset;
        if (best_delay < 8.0f) best_delay = 8.0f;
        
        // Synchronize phase: target delay shifts by the same amount
        // the active delay shifted
        xfade_delay = best_delay + (main_delay - search_active_delay);
        wsola_state = STATE_WAIT_CROSSFADE;
      }
    }
    
    // Wait until pointers hit exactly the crossfade boundary
    if (wsola_state == STATE_WAIT_CROSSFADE) {
      float32_t margin = CROSSFADE_LEN * fabsf(delay_rate);
      if (margin > WINDOW_LENGTH / 2.0f) margin = WINDOW_LENGTH / 2.0f;
      
      if (delay_rate < 0.0f && main_delay <= SAFE_ZONE + margin) {
        crossfade_progress = 0;
        wsola_state = STATE_CROSSFADE;
      } else if (delay_rate >
                 0.0f && main_delay >= SAFE_ZONE + WINDOW_LENGTH - margin) {
        crossfade_progress = 0;
        wsola_state = STATE_CROSSFADE;
      }
    }
    
    // ---------------------------------------------------------
    // Audio Mix Logic
    // ---------------------------------------------------------
    if (wsola_state == STATE_CROSSFADE) {
      float32_t val_main = read_delay_interp(main_delay);
      float32_t val_xfade = read_delay_interp(xfade_delay);
            
      float32_t w_xfade =
        (float32_t)crossfade_progress / (float32_t)(CROSSFADE_LEN - 1);
      
      // Hanning window fade-in for new pointer
      float32_t w2 = 0.5f * (1.0f - cosf(PI_F * w_xfade)); 
      float32_t w1 = 1.0f - w2;
            
      out = (val_main * w1) + (val_xfade * w2);
            
      crossfade_progress++;
      if (crossfade_progress >= CROSSFADE_LEN) {
        main_delay = xfade_delay;
        wsola_state = STATE_IDLE;
      }
    } else {
      out = read_delay_interp(main_delay);
    }
        
#define NORMAL_OUT
#ifdef NORMAL_OUT
    // Output on left channel, right channel unchanged
    buf[i] = FAST_FLOAT_TO_FIXED(out, 31);
#else
    // Alternate between outputting shifted signal on left and outputting
    // unchanged signal on left.  Switch every so often.  This is to
    // facilitate hearing the change the shift makes.
    if (which)
      buf[i] = FAST_FLOAT_TO_FIXED(out, 31);
    else
      buf[i] = orig_sample;
#endif
        
    // Advance write pointer circularly
    write_ptr = (write_ptr + 1) & DELAY_BUFFER_MASK;
        
    // Advance read pointer delays
    main_delay += delay_rate;
    if (wsola_state == STATE_WAIT_CROSSFADE ||
        wsola_state == STATE_CROSSFADE) {
      xfade_delay += delay_rate;
    }
  }

  which_cnt += 1;
  if (which_cnt >= 2000) {
    which_cnt = 0;
    which = 1 - which;
  }
}
