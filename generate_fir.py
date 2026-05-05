import math
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Generate FIR coefficients for CMSIS-DSP.")
    parser.add_argument('--pass_hz', type=float, default=5000.0, help='Passband frequency in Hz (default: 5000)')
    parser.add_argument('--stop_hz', type=float, default=7000.0, help='Stopband frequency in Hz (default: 7000)')
    parser.add_argument('--attenuation', type=float, default=35.0, help='Stopband attenuation in dB (default: 35.0)')
    parser.add_argument('--fs', type=float, default=48828.125, help='Sampling rate in Hz (default: 48828.125)')
    
    args = parser.parse_args()
    
    fs = args.fs
    pass_hz = args.pass_hz
    stop_hz = args.stop_hz
    A = args.attenuation
    
    if pass_hz >= stop_hz:
        print("Error: stop_hz must be greater than pass_hz for a low-pass filter.")
        sys.exit(1)
        
    delta_f = stop_hz - pass_hz
    delta_omega = 2 * math.pi * delta_f / fs
    # Standard rule for calculating filter taps via windowing methods
    N = int(math.ceil((A - 7.95) / (2.285 * delta_omega)))
    if N % 2 == 0:
        N += 1

    cutoff_hz = (pass_hz + stop_hz) / 2.0
    fc = cutoff_hz / fs

    coeffs = []
    M = N - 1
    for i in range(N):
        if i == M / 2.0:
            val = 2.0 * fc
        else:
            n = i - M / 2.0
            val = math.sin(2.0 * math.pi * fc * n) / (math.pi * n)
        
        # Hamming window
        window = 0.54 - 0.46 * math.cos(2.0 * math.pi * i / M)
        coeffs.append(val * window)

    # Normalize coefficients for unity gain at DC
    gain = sum(coeffs)
    coeffs = [c / gain for c in coeffs]

    header_content = f"""#ifndef FIR_COEFFS_H
#define FIR_COEFFS_H

#include "arm_math.h"

// Generated FIR Filter Coefficients
// Sample Rate: {fs} Hz
// Passband: {pass_hz} Hz
// Stopband: {stop_hz} Hz
// Number of Taps (N): {N}
// Target Attenuation: {A} dB

#define NUM_TAPS {N}

const float32_t fir_coeffs[NUM_TAPS] = {{
"""

    for i, c in enumerate(coeffs):
        header_content += f"    {c}f,"
        if (i + 1) % 4 == 0:
            header_content += "\n"

    header_content += "\n};\n\n#endif // FIR_COEFFS_H\n"

    with open('fir_coeffs.h', 'w') as f:
        f.write(header_content)

    print(f"Successfully generated fir_coeffs.h with {N} taps.")
    print(f"Filter specs: Cutoff={pass_hz} Hz, Stopband={stop_hz} Hz, Attenuation={A} dB")

if __name__ == "__main__":
    main()
