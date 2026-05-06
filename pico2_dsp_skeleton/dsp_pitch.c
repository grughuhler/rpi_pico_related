/* Copyright 2026 Grug Huhler
 *
 * License: SPDK BSD-2-Clause
 * Much of the code was created by Google Antigravity using Grug's
 * specification, fixes, and testing.
 */

/* This file performs pitch detection on the left channel.  It uses
 * Core 1 to perform the intensive computation and printing, so Core 0
 * can maintain strict real-time audio passthrough.  The method is based
 * on what is described in a paper "A Smarter Way To Find Pitch" by
 * Phillip McLeod.  Code is partly from AI.
 */

#include "dsp_common.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <string.h>

#define WINDOW_SIZE 704
#define MAX_TAU 704
#define BUFFER_SIZE (WINDOW_SIZE + MAX_TAU) // 1408

// Synchronous copy allows history size to perfectly match the buffer size.
#define CIRCULAR_HISTORY_SIZE BUFFER_SIZE // 1408

#define TAU_MIN 8   // ~6103 Hz max frequency
#define TAU_MAX MAX_TAU // ~69.3 Hz min frequency

static float32_t mirrored_history[2 * CIRCULAR_HISTORY_SIZE] = {0};
static float32_t diff[MAX_TAU];
static float32_t lpf_window[BUFFER_SIZE];  // Core 1 local buffer


static void core1_main(void);

void init_dsp(void)
{
  // Launch Core 1
  multicore_launch_core1(core1_main);
}

static void compute_nsdf(float32_t *window)
{
  float32_t E0 = 0.0f;
  arm_dot_prod_f32(window, window, WINDOW_SIZE, &E0);
    
  // Noise Gate: If energy is too low, return silence.
  // 0.01f represents roughly -46 dBFS RMS for a 704-sample window.
  if (E0 < 0.01f) {
    memset(diff, 0, sizeof(diff));
    return;
  }

  float32_t E_tau = E0;

  for (int tau = 0; tau < TAU_MAX; tau++) {
    // Sliding Window Energy O(1)
    if (tau > 0) {
      float32_t drop = window[tau - 1];
      float32_t add = window[tau + WINDOW_SIZE - 1];
      E_tau = E_tau - (drop * drop) + (add * add);
    }
        
    // Single Autocorrelation Pass O(N)
    float32_t acf = 0.0f;
    arm_dot_prod_f32(window, window + tau, WINDOW_SIZE, &acf);
        
    // Algebraic reconstruction of the YIN difference
    float32_t diff_sum = E0 + E_tau - (2.0f * acf);
        
    float32_t denom = E0 + E_tau;
    if (denom > 0.0f) {
      diff[tau] = 1.0f - (diff_sum / denom);
    } else {
      diff[tau] = 0.0f;
    }
  }
}

static int mpm_peak_pick(void)
{
  float32_t max_peak = 0.0f;
    
  // Find the highest local maximum in the NSDF
  for (int tau = TAU_MIN; tau < TAU_MAX - 1; tau++) {
    if (diff[tau] > 0.0f && diff[tau] > diff[tau - 1] &&
        diff[tau] > diff[tau + 1]) {
      if (diff[tau] > max_peak) {
        max_peak = diff[tau];
      }
    }
  }
    
  // If the signal has no strong periodic peaks at all.
  // A clean tone easily exceeds 0.90. A threshold of 0.70 cleanly rejects 
  // transient half-buffers (like when the signal is suddenly turned off) 
  // preventing spurious high-frequency garbage from bleeding into the output.
  if (max_peak < 0.70f) {
    return -1;
  }
    
  // Set the relative threshold. McLeod's recommended k ranges from
  // 0.8 to 1.0.
  float32_t threshold = max_peak * 0.93f;
    
  // Scan from the beginning and pick the first local maximum that
  // crosses the threshold (not paying attention to positive slope
  // zero crossings.   
  for (int tau = TAU_MIN; tau < TAU_MAX - 1; tau++) {
    if (diff[tau] > threshold && diff[tau] > diff[tau - 1]
        && diff[tau] > diff[tau + 1]) {
      return tau;
    }
  }
    
  return -1; // No pitch found
}

static float32_t parabolic_interpolation(int tau_estimate)
{
  if (tau_estimate <= 0 || tau_estimate >= TAU_MAX - 1) {
    return (float32_t)tau_estimate;
  }
    
  // Parabolic interpolation MUST be performed on the array where the
  // local minimum was actually found (diff) to guarantee mathematical
  // stability (denominator > 0).
  float32_t s0 = diff[tau_estimate - 1];
  float32_t s1 = diff[tau_estimate];
  float32_t s2 = diff[tau_estimate + 1];
    
  // Parabolic interpolation: delta = 0.5 * (s0 - s2) / (s0 - 2*s1 + s2)
  float32_t denominator = s0 - 2.0f * s1 + s2;
  if (denominator == 0.0f) {
    return (float32_t)tau_estimate;
  }

  float32_t delta = 0.5f * (s0 - s2) / denominator;

  return (float32_t)tau_estimate + delta;
}

#define MEDIAN_FILTER_LEN 11

static float32_t apply_median_filter(float32_t new_val)
{
  static float32_t history[MEDIAN_FILTER_LEN] = {0};
  static int idx = 0;
    
  history[idx] = new_val;
  idx = (idx + 1) % MEDIAN_FILTER_LEN;
    
  float32_t sorted[MEDIAN_FILTER_LEN];
  memcpy(sorted, history, sizeof(sorted));
    
  // Insertion sort for MEDIAN_FILTER_LEN elements
  for (int i = 1; i < MEDIAN_FILTER_LEN; i++) {
    float32_t key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j = j - 1;
    }
    sorted[j + 1] = key;
  }
    
  return sorted[MEDIAN_FILTER_LEN/2];
}

static const char* note_names[] =
  {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

static void freq_to_note(float32_t freq, char *out_str, size_t out_len)
{
  if (freq <= 0.0f) {
    snprintf(out_str, out_len, "None");
    return;
  }
    
  // MIDI note number: A4 (440 Hz) is 69.
  float32_t pitch_midi = 12.0f * log2f(freq / 440.0f) + 69.0f;
  int note_num = (int)roundf(pitch_midi);
    
  if (note_num < 0 || note_num > 127) {
    snprintf(out_str, out_len, "Out of Range");
    return;
  }
    
  int note_index = note_num % 12;
  int octave = (note_num / 12) - 1;
    
  // Calculate cents deviation
  float32_t cents = (pitch_midi - (float32_t)note_num) * 100.0f;
    
  snprintf(out_str, out_len, "%s%d (%+.1f cents)", note_names[note_index],
           octave, cents);
}

static void core1_main(void)
{
  float32_t detected_freq = 0.0f;

  // Send initial ready token to Core 0
  multicore_fifo_push_blocking(1);
    
  while (1) {
    // Block until Core 0 fills lpf_window and triggers us
    (void)multicore_fifo_pop_blocking();
        
    gpio_xor_mask(1u << PIN_DEBUG_SW1); // Toggle SW1 GPIO
        
    // Remove DC offset (zero-mean) to prevent artificial correlation of
    // a flatline which occurs when the signal is abruptly shut off and
    // capacitors slowly discharge.
    float32_t mean_val = 0.0f;
    arm_mean_f32(lpf_window, BUFFER_SIZE, &mean_val);
    arm_offset_f32(lpf_window, -mean_val, lpf_window, BUFFER_SIZE);
        
    // Compute MPM on the local raw buffer
    compute_nsdf(lpf_window);
    int tau_estimate = mpm_peak_pick();
        
    if (tau_estimate != -1) {
      float32_t exact_tau = parabolic_interpolation(tau_estimate);
      detected_freq = SAMPLE_RATE / exact_tau;
    } else {
      detected_freq = 0.0f;
    }
        
    // Apply MEDIAN_FILTER_LEN-frame median filter to completely obliterate instantaneous
    // octave/subharmonic glitches
    detected_freq = apply_median_filter(detected_freq);

    // Print frequency only if it changes by more than 2 cents, or if
    // pitch state toggles
    static float32_t last_printed_freq = -1.0f;
    bool should_print = false;
        
    if (detected_freq == 0.0f) {
      if (last_printed_freq != 0.0f) {
        should_print = true;
      }
    } else {
      if (last_printed_freq <= 0.0f) {
        should_print = true;
      } else {
        float32_t cents_diff = 1200.0f *
          fabsf(log2f(detected_freq / last_printed_freq));
 
        if (cents_diff >= 2.0f) {
          should_print = true;
        }
      }
    }

    gpio_xor_mask(1u << PIN_DEBUG_SW1); // Toggle SW1 GPIO
        
    if (should_print) {
      last_printed_freq = detected_freq;
      if (detected_freq > 0.0f) {
        char note_str[32];
        freq_to_note(detected_freq, note_str, sizeof(note_str));
        printf("Pitch: %.1f Hz [%s]\n", detected_freq, note_str);
      } else {
        printf("Pitch: None\n");
      }
    }

    // Send ready token back to Core 0 to request the next snapshot
    multicore_fifo_push_blocking(1);
  }
}

void process_buf_dsp(q31_t *buf)
{
  static uint32_t hist_write_idx = 0;
  float32_t float_in[BLOCK_SIZE];
    
  buf_left_to_float(buf, float_in);
    
  memcpy(&mirrored_history[hist_write_idx], float_in,
         BLOCK_SIZE * sizeof(float32_t));
  memcpy(&mirrored_history[hist_write_idx + CIRCULAR_HISTORY_SIZE],
         float_in, BLOCK_SIZE * sizeof(float32_t));
    
  hist_write_idx = (hist_write_idx + BLOCK_SIZE) % CIRCULAR_HISTORY_SIZE;
    
  // Check if Core 1 sent a ready token.
  // Core 1 is a non-realtime process, so only feed it when it's ready.
  if (multicore_fifo_rvalid()) {
    // Pop the token to clear the FIFO
    (void)multicore_fifo_pop_blocking();

    // Copy the current contiguous window into Core 1's local buffer
    // The history is mirrored so just read straight from hist_write_idx
    float32_t *contiguous_window = &mirrored_history[hist_write_idx];
    memcpy(lpf_window, contiguous_window, BUFFER_SIZE * sizeof(float32_t));

    // Send a go-ahead ping to wake up Core 1
    multicore_fifo_push_blocking(1);
  }
}
