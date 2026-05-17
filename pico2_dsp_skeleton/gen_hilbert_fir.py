#!/bin/python3

import argparse
import numpy as np
from scipy import signal
import sys

def main():
    parser = argparse.ArgumentParser(
        description='CMSIS-DSP Hilbert Coefficient Generator for RP2350/Pico 2',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Example Invocations:
  python gen_hilbert.py --taps 101 --window blackman
        """
    )
    
    parser.add_argument('--fs', type=float, default=48828.125, 
                        help='Sample rate in Hz (default: 48828.125)')
    parser.add_argument('--taps', type=int, default=101, 
                        help='Number of filter taps (must be odd) (default: 101)')
    parser.add_argument('--window', type=str, default='blackman',
                        help='Window function to apply (default: blackman)')
    parser.add_argument('--file', type=str, default='hilbert_coeffs.h',
                        help='Output header file name (default: hilbert_coeffs.h)')

    args = parser.parse_args()

    if args.taps % 2 == 0:
        print("Error: --taps must be an odd integer for an integer group delay.")
        sys.exit(1)

    try:
        # Generate Hilbert Taps using the windowed ideal impulse response method
        # This is far more robust than remez for wideband Hilbert transformers
        M = (args.taps - 1) // 2
        n = np.arange(-M, M + 1)
        taps = np.zeros(args.taps)
        for i, val in enumerate(n):
            if val != 0 and val % 2 != 0:
                taps[i] = 2.0 / (np.pi * val)
        
        # Apply window
        win = signal.get_window(args.window, args.taps)
        taps *= win
        
        # NOTE: arm_fir_f32 expects coefficients in time-reversed order!
        # Since it's an anti-symmetric filter, reversing it flips the sign.
        # We time-reverse it here to ensure the exact correct phase shift.
        taps = taps[::-1]

    except Exception as e:
        print(f"Error generating Hilbert filter: {e}")
        sys.exit(1)

    output = []
    output.append(f"/*")
    output.append(f" * CMSIS-DSP Hilbert Coefficients for arm_fir_f32")
    output.append(f" * Generated for RP2350 / Pico 2")
    output.append(f" * Method: Windowed Ideal Impulse Response")
    output.append(f" * Window: {args.window}")
    output.append(f" * Taps: {args.taps}")
    output.append(f" * FS: {args.fs} Hz")
    output.append(f" */\n")
    
    output.append(f"#ifndef HILBERT_COEFFS_H")
    output.append(f"#define HILBERT_COEFFS_H\n")
    output.append(f"#include \"arm_math.h\"\n")
    output.append(f"#define NUM_TAPS {args.taps}\n")
    
    output.append(f"float32_t hilbert_coeffs[NUM_TAPS] = {{")

    for i in range(0, len(taps), 4):
        chunk = taps[i:i+4]
        line = ", ".join([f"{val:14.10f}f" for val in chunk])
        comma = "," if i + 4 < len(taps) else ""
        output.append(f"    {line}{comma}")

    output.append("};\n")
    
    output.append(f"float32_t hilbert_state[NUM_TAPS + BLOCK_SIZE - 1] = {{0}};\n")
    output.append(f"#endif // HILBERT_COEFFS_H")

    try:
        with open(args.file, 'w') as f:
            f.write("\n".join(output))
        print(f"Successfully wrote Hilbert coefficients to {args.file}")
    except IOError as e:
        print(f"Error writing to file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
