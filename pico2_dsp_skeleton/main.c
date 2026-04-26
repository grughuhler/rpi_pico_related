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

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "i2s.pio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#define ARM_MATH_CM33
#include "arm_math.h"

// Use the vcvt instruction to convert between fixed point and float32.
// bits must be a constant.
// bits = 31 means fixed is Q31
// bits = 30 means fixed is Q30 and so on.

#define FAST_FIXED_TO_FLOAT(x, bits) ({ \
    float32_t __result; \
    __asm__ ("vcvt.f32.s32 %0, %1, #" #bits : "=t" (__result) : "t" (x)); \
    __result; \
})

#define FAST_FLOAT_TO_FIXED(x, bits) ({ \
    int32_t __result; \
    __asm__ ("vcvt.s32.f32 %0, %1, #" #bits : "=t" (__result) : "t" (x)); \
    __result; \
})

// Hardware Pin Definitions
#define PIN_SCK 10
#define PIN_BCK 11
#define PIN_LRCK 12
#define PIN_DOUT 13
#define PIN_DIN 14

// Debug Pins for Timing Analysis.  These are just GPIO outs.
#define PIN_DEBUG_RX 2
#define PIN_DEBUG_TX 3
#define PIN_DEBUG_SW 4

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PI 25
#endif

#define PI_F 3.1415926536f
#define PI_D 3.1415926536
#define MAX_INT_F 2147483647.0f
#define SAMPLE_RATE 48828.125f
#define SAMPLES_PER_BUFFER 128
/* That's 64 stereo samples per buffer.  Samples for the two
 * channels are interleaved. */
#define NUM_BUFFERS 3

#define BLOCK_SIZE (SAMPLES_PER_BUFFER / 2)

// 3 Cyclic buffers. int32_t matches the native 32-bit 2's complement format
// from the I2S hardware.
int32_t audio_buffers[NUM_BUFFERS][SAMPLES_PER_BUFFER];

volatile int dma_tx_ch0, dma_tx_ch1;
volatile int dma_rx_ch0, dma_rx_ch1;

// Control indices for buffer rotating
volatile int rx_fill_idx = 1;
volatile int tx_drain_idx = 2;
volatile int process_index = -1;

/* define exactly one USE_XX to select DSP function.
 *   USE_FIR implies calling do_filter
 *   USE_IIR implies calling do_filter
 *   USE_MAKE_SINE implies calling make_sine
 *   USE_MULT implies calling do_mult_nult
 *   USE_DETECT implies calling Goertzel tone detector.  When tone is
                detected LED lights.
 *   USE_FFT imples calling do_fft.  Frequency bins above a threshold
 *           are printed occasionally.
 *   USE_NONE implies calling nothing, samples all passed unchanged
 *
 */

/****************** Select buffer processing **********************/

#define USE_FIR

#if defined(USE_FIR)

#include "fir_coeffs.h"
static arm_fir_instance_f32 fir_left;
void init_fir(void)
{
  /* Tell FIR sizes and coefficients */
  arm_fir_init_f32(&fir_left, NUM_TAPS, (float32_t *)&fir_coeffs[0],
                   &fir_state[0], BLOCK_SIZE);
}

#elif defined(USE_IIR)

#include "iir_coeffs.h"
static arm_biquad_cascade_df2T_instance_f32 S2;
void init_biquad2(void)
{
  arm_biquad_cascade_df2T_init_f32(&S2, NUM_STAGES, iir_coeffs,
                                   iir_state);
}

#elif defined(USE_NONE)
#elif defined(USE_MAKE_SINE)
#elif defined(USE_MULT)
#elif defined(USE_DETECT)
/* Goertzel - call goertzel_update in a loop.  Whenever .ready is
 *  true, check confidence_scope.  A value > ~0.9 indicates presence
 *  of tone.  But experiment with this threshold in your application.
/* --- Goertzel Object Structure --- */
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

GoertzelDetector detector;

/**
 * Initializes the goertzel detector. 
 *   target_f - frequency of tone to detect.  Works worse for
 *              small frequencies.  So, test carefully if
 *              target_f is less than ~100 Hz.
 */

void __attribute__ ((noinline)) init_goertzel(GoertzelDetector *S, float32_t target_f) {
  // omega must be a double and must call cos() and not a float32
  // version of it becuase values returned for small target_f are
  // so close to one.  Could use float if target_f will always be
  // greater than ~500 Hz.
  double omega = (2.0 * PI_D * (double) target_f) / (double) SAMPLE_RATE;
  S->coeff = 2.0 * cos(omega);
    
  // Increasing cycles sharpens the filter but adds execution time.
  // Integer values from 4.0 to 8.0 look reasonable, higher OK for
  // higher frequences.  Experiment with this in your application.
  float32_t cycles = (target_f <= 400.0) ? 4.0f : 8.0f;
  uint32_t ideal_samples = (uint32_t)(cycles * SAMPLE_RATE / target_f);
  if (ideal_samples < 256) ideal_samples = 256;

  S->samples_needed = ((ideal_samples + BLOCK_SIZE - 1) / BLOCK_SIZE) * BLOCK_SIZE;
  S->v1 = 0; S->v2 = 0; S->total_energy = 0; S->samples_processed = 0;
  S->dc_prev_in = 0; S->dc_prev_out = 0;
  S->ready = false;
}
#elif defined(USE_FFT)
/************************* FFT ***************************************/
#define FFT_SIZE 1024
#define MAG_SIZE (FFT_SIZE / 2)

// Buffers that must retain state
static float32_t fft_input_mirrored_history[2*FFT_SIZE] = {0};
static __not_in_flash() float32_t hann_window[FFT_SIZE];

// Scratchpad buffers (reused every call, but data doesn't need to persist)
static float32_t fft_work_buffer[FFT_SIZE];
static float32_t fft_complex_output[FFT_SIZE];
static float32_t output_mag[MAG_SIZE];
static float32_t output_db[MAG_SIZE];

// CMSIS-DSP Instance
static arm_rfft_fast_instance_f32 fft_inst;

void init_sliding_fft() {
    // Initialize the CMSIS-DSP RFFT structure
    // This sets up the twiddle factor pointers for the RP2350
    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    
    // Pre-calculate Hanning window: 0.5 * (1 - cos(2*pi*i / (N-1)))
    // Using arm_cos_f32 is faster than standard math.h cos on the M33
    for (int i = 0; i < FFT_SIZE; i++) {
        float32_t angle = 2.0f * PI * (float32_t)i / (float32_t)(FFT_SIZE - 1);
        hann_window[i] = 0.5f * (1.0f - arm_cos_f32(angle));
    }
}

#else
#error you must define a USE_XX
#endif

void __isr dma_handler() {
  uint32_t ints = dma_hw->ints0;

  // ----- RX Channels Check -----
  if (ints & (1u << dma_rx_ch0)) {
    dma_hw->ints0 = 1u << dma_rx_ch0; // clear interrupt
    gpio_xor_mask(1u << PIN_DEBUG_RX); // Toggle RX debug pin
    int completed_buf = rx_fill_idx;
    process_index = completed_buf;
    rx_fill_idx = (rx_fill_idx + 1) % NUM_BUFFERS;

    // Point the inactive channel to the next-next buffer
    dma_channel_set_write_addr(
        dma_rx_ch0, audio_buffers[(completed_buf + 2) % NUM_BUFFERS], false);
  }
  if (ints & (1u << dma_rx_ch1)) {
    dma_hw->ints0 = 1u << dma_rx_ch1;
    gpio_xor_mask(1u << PIN_DEBUG_RX); // Toggle RX debug pin
    int completed_buf = rx_fill_idx;
    process_index = completed_buf;
    rx_fill_idx = (rx_fill_idx + 1) % NUM_BUFFERS;

    dma_channel_set_write_addr(
        dma_rx_ch1, audio_buffers[(completed_buf + 2) % NUM_BUFFERS], false);
  }

  // ----- TX Channels Check -----
  if (ints & (1u << dma_tx_ch0)) {
    dma_hw->ints0 = 1u << dma_tx_ch0;
    gpio_xor_mask(1u << PIN_DEBUG_TX); // Toggle TX debug pin
    int completed_buf = tx_drain_idx;
    tx_drain_idx = (tx_drain_idx + 1) % NUM_BUFFERS;

    // Update inactive TX channel address so it's ready when the current one
    // finishes
    dma_channel_set_read_addr(
        dma_tx_ch0, audio_buffers[(completed_buf + 2) % NUM_BUFFERS], false);
  }
  if (ints & (1u << dma_tx_ch1)) {
    dma_hw->ints0 = 1u << dma_tx_ch1;
    gpio_xor_mask(1u << PIN_DEBUG_TX); // Toggle TX debug pin
    int completed_buf = tx_drain_idx;
    tx_drain_idx = (tx_drain_idx + 1) % NUM_BUFFERS;

    dma_channel_set_read_addr(
        dma_tx_ch1, audio_buffers[(completed_buf + 2) % NUM_BUFFERS], false);
  }
}


#if defined(USE_MAKE_SINE)
/* This is a test function you can use to ignore the input data
 * and just output a generated sine wave on both the left and
 * right channels.
 */

void make_sine(int32_t *buf, double hz_left, double hz_right)
{
  static float phase_left = 0.0f, phase_right = 0.0f;
  float phase_increment_left = 2.0f * PI_F * hz_left / SAMPLE_RATE;
  float phase_increment_right = 2.0f * PI_F * hz_right / SAMPLE_RATE;

  for (int i = 0; i < SAMPLES_PER_BUFFER; i += 2) {
    float sample = arm_sin_f32(phase_left);
    // The "30" makes the sine wave go from -0.5 to 0.5.
    buf[i] = FAST_FLOAT_TO_FIXED(sample, 30);;   // Left channel
    sample = arm_sin_f32(phase_right);
    buf[i+1] = FAST_FLOAT_TO_FIXED(sample, 30);; // right channel
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

#defined (USE_MULT)
/* This function sets the output of the left channel to be
 * the product of the signals from the left and right channels.
 * You can see a frequency shift
 */

void __attribute__ ((noinline)) do_mult(int32_t *buf)
{
  for (int i = 0; i < SAMPLES_PER_BUFFER; i += 2) {
    float sample1 = ((float) buf[i]) * (1.0f / 2147483648.0f);
    float sample2 = ((float) buf[i+1]) * (1.0f / 2147483648.0f);

    buf[i] = (int32_t)(sample2 * sample1 * 4000000000.0f);
  }
}
#endif

void buf_left_to_float(q31_t *buf, float32_t *float_in)
{
  // De-interleave buf[i] (left channel) and convert to float
  // buf[i] is left, buf[i+1] is right.
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    float_in[j] = FAST_FIXED_TO_FLOAT(buf[i], 31);
  }
}

void float_to_buf_left(float32_t *float_out, q31_t *buf)
{
  // Convert back and place it in buf[i]
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i] = FAST_FLOAT_TO_FIXED(float_out[j], 31);
  }
}

#if defined(USE_FIR) || defined(USE_IIR)
/* This function passes the left channel data through a filter (from
 * Arm's CMSIS_DSP library.  Right channel data is just passed
 * through.
 */

void __attribute__ ((noinline)) do_filter(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out[BLOCK_SIZE];

  buf_left_to_float(buf, float_in);

#if defined(USE_FIR)
  arm_fir_f32(&fir_left, float_in, float_out, BLOCK_SIZE);
#elif defined(USE_IIR)
  arm_biquad_cascade_df2T_f32(&S2, float_in, float_out, BLOCK_SIZE);
#endif

  float_to_buf_left(float_out, buf);
}
#endif

#if defined(USE_DETECT)

/* Detects tones using a Goertzel method.
 * Call this function in the main block processing loop.
 * When .ready ** .confidence_score is high enough, a tone
 * is present, but the confidence_score can bounce around your
 * chosen threshold if the tone is near the detection boundary.
 * Some extra logic to ensure the detection is stable may be a
 * good idea.
 */

void goertzel_update(GoertzelDetector *S, float32_t *pSrc) {
  float32_t v0;
  float32_t v1 = S->v1;
  float32_t v2 = S->v2;
  float32_t energy_sum = 0.0f;
  const float32_t alpha = 0.99f; // DC Blocker coefficient

  for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
    // 1. Simple DC Blocker (High Pass Filter)
    // This is CRITICAL for 60Hz detection
    float32_t clean_sample = pSrc[i] - S->dc_prev_in + (alpha * S->dc_prev_out);
    S->dc_prev_in = pSrc[i];
    S->dc_prev_out = clean_sample;

    // 2. Goertzel Feedback
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
      // Updated normalization to match your observed 0.9+ peaks
      S->confidence_score = (2.0f * pwr) / (S->total_energy * n);
    } else {
      S->confidence_score = 0.0f;
    }

    S->ready = true;
    S->v1 = 0; S->v2 = 0; S->total_energy = 0; S->samples_processed = 0;
  } else {
    S->ready = false;
  }
}

// Covert samples and call Geortzel detector
void do_detect(GoertzelDetector *detector, q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  static float32_t c = 0.0f;

  buf_left_to_float(buf, float_in);
  goertzel_update(detector, float_in);
#if 0
  if (detector->ready) {
    c = detector->confidence_score;
    if (c > 2.0) c = 2.0;
  }
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    buf[i] = FAST_FLOAT_TO_FIXED(c, 31);
  }
#endif
}

#endif

#if defined(USE_FFT)
/*************************** FF **************************************/

// Max expected magnitude for a 1024 FFT with Hanning window is ~256
#define FFT_MAX_MAG 256.0f

#include "arm_math.h"

// Pre-calculate the conversion constant: 20 / ln(10)
#define LN_TO_DB_CONV 8.685889638f
#define FFT_MAX_MAG 256.0f

void convert_to_db(float32_t *mag_input, float32_t *db_output, uint32_t count) {
    // 1. Normalize the magnitudes first
    // CMSIS-DSP vector scale: db_output = mag_input * (1/FFT_MAX_MAG)
    float32_t scale = 1.0f / FFT_MAX_MAG;
    arm_scale_f32(mag_input, scale, db_output, count);

    // 2. Add epsilon to avoid log(0) - CMSIS-DSP doesn't have a 'max with constant' 
    // but we can just use a quick loop or trust our signal floor.
    for(int i=0; i<count; i++) if(db_output[i] < 1e-12f) db_output[i] = 1e-12f;

    // 3. Vector Natural Log (ln)
    // This is the heavy lifter. It processes the whole array.
    arm_vlog_f32(db_output, db_output, count);

    // 4. Convert ln to dB (log10 * 20)
    // db_output = db_output * 8.6858896
    arm_scale_f32(db_output, LN_TO_DB_CONV, db_output, count);
}

void process_sliding_fft(float32_t *input_64, float32_t *output_mag)
{
  static uint32_t write_idx = 0;
  float32_t *contiguous_window;

  // Double-write the new samples to maintain the mirror
  // Write to the current head
  memcpy(&fft_input_mirrored_history[write_idx], input_64,
         BLOCK_SIZE * sizeof(float32_t));
  // Mirror the write to the second half
  memcpy(&fft_input_mirrored_history[write_idx + FFT_SIZE],
         input_64, BLOCK_SIZE * sizeof(float32_t));
  // Advance the write index circularly (within the first 1024)
  write_idx = (write_idx + BLOCK_SIZE) % FFT_SIZE;

  // Point to the start of the current 1024-sample window.
  // The start of the oldest data is the CURRENT write_idx.
  // Because we mirrored, the data from write_idx to (write_idx + 1023)
  // is contiguous!
  contiguous_window = &fft_input_mirrored_history[write_idx];

  // Window: Apply Hanning window to the history, store in scratchpad
  // This prevents the history from being "modified" by the window
  arm_mult_f32(contiguous_window, hann_window, fft_work_buffer, FFT_SIZE);

  // Transform: Time Domain -> Frequency Domain (Packed Complex)
  arm_rfft_fast_f32(&fft_inst, fft_work_buffer, fft_complex_output, 0);

  // Magnitude: Convert Complex [Re, Im] to Real [Magnitude]
  arm_cmplx_mag_f32(fft_complex_output, output_mag, MAG_SIZE);

  return;
}

void do_fft(q31_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  static int pr_limit = 0;

  buf_left_to_float(buf, float_in);
  process_sliding_fft(float_in, output_mag);
  convert_to_db(output_mag, output_db, MAG_SIZE);

  if (pr_limit >= 350) {
    pr_limit = 0;
        printf("*************************************\n");
        for (int i = 1; i < MAG_SIZE; i++)
          if (output_db[i] >= -30.0f)
            printf("%f : %f\n", i*SAMPLE_RATE/FFT_SIZE, output_db[i]);
  } else {
    pr_limit += 1;
  }
}


#endif

int main()
{
  // 1. Explicitly strictly set the system clock to 150 MHz
  set_sys_clock_khz(150000, true);

  // 2. Initialize stdio (redirected to USB through CMake setting)
  stdio_init_all();

  // 3. Status LED Setup (Flash sequence: ON for 1/4s, OFF for 1/4s, ON for
  // 1/4s, OFF for 1/4s).  This is to show program started OK.
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  for (int i = 0; i < 4; i++) {
    gpio_put(PICO_DEFAULT_LED_PIN, i % 2 == 0);
    sleep_ms(250);
  }

  // Initialize debug pins for timing observation
  gpio_init(PIN_DEBUG_RX);
  gpio_set_dir(PIN_DEBUG_RX, GPIO_OUT);
  gpio_init(PIN_DEBUG_TX);
  gpio_set_dir(PIN_DEBUG_TX, GPIO_OUT);
  gpio_init(PIN_DEBUG_SW);
  gpio_set_dir(PIN_DEBUG_SW, GPIO_OUT);

  printf("Pico 2 Audio DSP Framework Starting...\n");

  // Initialize all cyclic buffers to audio zero value
  for (int i = 0; i < NUM_BUFFERS; i++) {
    for (int j = 0; j < SAMPLES_PER_BUFFER; j++) {
      audio_buffers[i][j] = 0;
    }
  }

  // 4. Load PIO program
  PIO pio = pio0;
  uint sm = pio_claim_unused_sm(pio, true);
  uint offset = pio_add_program(pio, &i2s_program);
  i2s_program_init(pio, sm, offset, PIN_DOUT, PIN_DIN, PIN_SCK);

  // 5. Claim DMA Channels
  dma_rx_ch0 = dma_claim_unused_channel(true);
  dma_rx_ch1 = dma_claim_unused_channel(true);
  dma_tx_ch0 = dma_claim_unused_channel(true);
  dma_tx_ch1 = dma_claim_unused_channel(true);

  // --- RX PING DMA (Writes to buffer 1 initially) ---
  dma_channel_config rx_c0 = dma_channel_get_default_config(dma_rx_ch0);
  channel_config_set_transfer_data_size(&rx_c0, DMA_SIZE_32);
  channel_config_set_read_increment(&rx_c0, false);
  channel_config_set_write_increment(&rx_c0, true);
  channel_config_set_dreq(&rx_c0, pio_get_dreq(pio, sm, false)); // RX DREQ
  channel_config_set_chain_to(&rx_c0, dma_rx_ch1);

  dma_channel_configure(dma_rx_ch0, &rx_c0,
                        audio_buffers[1],   // dest (buffer 1)
                        &pio->rxf[sm],      // src
                        SAMPLES_PER_BUFFER, // transfer count
                        false               // Don't start
  );

  // --- RX PONG DMA (Starts aiming at buffer 2) ---
  dma_channel_config rx_c1 = dma_channel_get_default_config(dma_rx_ch1);
  channel_config_set_transfer_data_size(&rx_c1, DMA_SIZE_32);
  channel_config_set_read_increment(&rx_c1, false);
  channel_config_set_write_increment(&rx_c1, true);
  channel_config_set_dreq(&rx_c1, pio_get_dreq(pio, sm, false));
  channel_config_set_chain_to(&rx_c1, dma_rx_ch0);

  dma_channel_configure(dma_rx_ch1, &rx_c1,
                        audio_buffers[2], // dest (buffer 2 initially)
                        &pio->rxf[sm],    // src
                        SAMPLES_PER_BUFFER, false);

  // --- TX PING DMA (Reads from buffer 2 initially, since software will
  // process buffer 0) ---
  dma_channel_config tx_c0 = dma_channel_get_default_config(dma_tx_ch0);
  channel_config_set_transfer_data_size(&tx_c0, DMA_SIZE_32);
  channel_config_set_read_increment(&tx_c0, true);
  channel_config_set_write_increment(&tx_c0, false);
  channel_config_set_dreq(&tx_c0, pio_get_dreq(pio, sm, true)); // TX DREQ
  channel_config_set_chain_to(&tx_c0, dma_tx_ch1);

  dma_channel_configure(dma_tx_ch0, &tx_c0,
                        &pio->txf[sm],    // dest
                        audio_buffers[2], // src
                        SAMPLES_PER_BUFFER, false);

  // --- TX PONG DMA (Starts aiming at buffer 0) ---
  dma_channel_config tx_c1 = dma_channel_get_default_config(dma_tx_ch1);
  channel_config_set_transfer_data_size(&tx_c1, DMA_SIZE_32);
  channel_config_set_read_increment(&tx_c1, true);
  channel_config_set_write_increment(&tx_c1, false);
  channel_config_set_dreq(&tx_c1, pio_get_dreq(pio, sm, true));
  channel_config_set_chain_to(&tx_c1, dma_tx_ch0);

  dma_channel_configure(dma_tx_ch1, &tx_c1,
                        &pio->txf[sm],    // dest
                        audio_buffers[0], // src
                        SAMPLES_PER_BUFFER, false);

  // 6. Install global IRQ handler for all 4 DMA channels on the DMA_0 channel
  dma_channel_set_irq0_enabled(dma_rx_ch0, true);
  dma_channel_set_irq0_enabled(dma_rx_ch1, true);
  dma_channel_set_irq0_enabled(dma_tx_ch0, true);
  dma_channel_set_irq0_enabled(dma_tx_ch1, true);

  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  // Pre-fill the PIO TX FIFO with 4 dummy words (silence)
  // This throttles the TX DMA so it doesn't instantly jump 5 words ahead!
  // This makes the DMAs better aligned in time-- but still not perfect so
  // software does not have quite 1 DMA time period to finish processing
  // its buffer.
  for (int i = 0; i < 4; i++) {
    pio_sm_put(pio, sm, 0);
  }

#if defined(USE_FIR)
  init_fir();
#elif defined(USE_IIR)
  init_biquad2();
#elif defined (USE_DETECT)
  init_goertzel(&detector, 1000.0f);
#elif defined(USE_FFT)
  init_sliding_fft();
#endif

  // 7. Start the DMAs simultaneously
  uint32_t start_mask = (1u << dma_rx_ch0) | (1u << dma_tx_ch0);
  dma_start_channel_mask(start_mask);

  // 8. Start the PIO
  pio_sm_set_enabled(pio, sm, true);

  // Tell the system we are manually processing Buffer 0 first for cycle 0.
  process_index = 0;
  int last_processed = -1;

  // 9. DSP Real-time loop
  while (true) {
    if (process_index != last_processed) {
      last_processed = process_index;

      gpio_xor_mask(1u << PIN_DEBUG_SW); // Toggle SW GPIO
      if (last_processed >= 0 && last_processed < NUM_BUFFERS) {
        int32_t *buf = audio_buffers[last_processed];

        /* Process buffer buf in place here.  If you do nothing,
         * samples will be passed through unchanged.  You have the
         * time of one DMA (approx 1.3ms) to process the buffer.
         */
#if defined(USE_FIR) || defined(USE_IIR)
        do_filter(buf);
#elif defined(USE_MAKE_SINE)
        make_sine(buf, 240, 360);
#elif defined (USE_MULT)
        do_mult(buf);
#elif defined(USE_DETECT)
        do_detect(&detector, buf);
        if (detector.ready) {
          //printf("%f\n", detector.confidence_score);
          if (detector.confidence_score > 0.9f) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
          } else {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
          }
        }
#elif defined(USE_FFT)
        do_fft(buf);
#endif        
        gpio_xor_mask(1u << PIN_DEBUG_SW); // Toggle SW GPIO
      }
    }

    // Relinquish power while we rigorously wait for the hardware deadline.
    // Grug says...  AI added this.  No idea if it really does anything
    // useful.
    tight_loop_contents();
  }

  // Function never ends */
  return 0;
}
