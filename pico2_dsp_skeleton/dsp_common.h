#ifndef DSP_COMMON_H
#define DSP_COMMON_H

#include "pico/stdlib.h"
#include <stdint.h>

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
#define PICO_DEFAULT_LED_PIN 25
#endif

#define PI_F 3.1415926536f
#define PI_D 3.1415926536
#define MAX_INT_F 2147483647.0f
#define SAMPLE_RATE 48828.125f

#define NUM_BUFFERS 3
#define SAMPLES_PER_BUFFER 128
#define BLOCK_SIZE (SAMPLES_PER_BUFFER / 2)

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

// Conversion functions
void buf_left_to_float(q31_t *buf, float32_t *float_in);
void float_to_buf_left(float32_t *float_out, q31_t *buf);

// Interface implemented by each DSP module
void init_dsp(void);
void process_buf_dsp(q31_t *buf);

#endif // DSP_COMMON_H
