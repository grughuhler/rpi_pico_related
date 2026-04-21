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
#define PICO_DEFAULT_LED_PI  25
#endif

#define PI_F 3.1415926536f
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

/* define USE_XX
 *   USE_FIR implies calling arm_fir_f32
 *   USE_IIR implies calling arm_biquad_cascade_df2T_f32
 *   USE_MAKE_SINE implies calling make_sine
 *   USE_MULT implies calling process_buffer_nult
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

/* This is a test function you can use to ignore the input data
 * and just output a generated sine wave.
 */

void make_sine(int32_t *buf, double hz)
{
  static float phase = 0.0f;
  float phase_increment = 2.0f * PI_F * hz / SAMPLE_RATE;

  for (int i = 0; i < SAMPLES_PER_BUFFER; i += 2) {
    float sample = arm_sin_f32(phase) * (MAX_INT_F / 2.0f);
    buf[i] = (int32_t)sample;       // Left channel
    buf[i+1] = (int32_t)sample;     // Right channel
    phase += phase_increment;
    if (phase > 2.0f * PI_F) {
      phase -= 2.0f * PI_F;
    }
  }
}

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

/* This function passes the left channel data through a filter (from
 * Arm's CMSIS_DSP library.  Right channel data is just passed
 * through.
 */

void __attribute__ ((noinline)) do_filter(int32_t *buf)
{
  float32_t float_in[BLOCK_SIZE];
  float32_t float_out[BLOCK_SIZE];

  // De-interleave buf[i] (left channel) and convert to float
  // buf[i] is left, buf[i+1] is right.
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    float_in[j] = ((float)buf[i]) * (1.0f / 2147483648.0f);
  }

#if defined(USE_FIR)
  arm_fir_f32(&fir_left, float_in, float_out, BLOCK_SIZE);
#elif defined(USE_IIR)
  arm_biquad_cascade_df2T_f32(&S2, float_in, float_out, BLOCK_SIZE);
#endif
  // Convert back and place it in buf[i]
  for (int i = 0, j = 0; i < SAMPLES_PER_BUFFER; i += 2, j++) {
    float sample = float_out[j] * 2147483647.0f;
    // Simple clipping to prevent overflow
    if (sample > 2147483647.0f) sample = 2147483647.0f;
    if (sample < -2147483648.0f) sample = -2147483648.0f;
    buf[i] = (int32_t)sample;
    // buf[i+1] remains unaltered, passing the signal straight through
  }
}

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

#if defined(USE_FIR)
  init_fir();
#elif defined(USE_IIR)
  init_biquad2();
#endif

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
        make_sine(buf, 290);
#elif defined (USE_MULT)
        do_mult(buf);
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
