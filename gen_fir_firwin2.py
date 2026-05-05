#!/bin/python3

import argparse
import numpy as np
from scipy import signal
import sys

def main():
    parser = argparse.ArgumentParser(
        description='CMSIS-DSP FIR Generator (firwin2) for RP2350/Pico 2',
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    parser.add_argument('--fs', type=float, default=48828.125, help='Sample rate (Hz)')
    parser.add_argument('--window', default='hamming', help='Window type')
    parser.add_argument('--taps', type=int, default=101, help='Number of taps')
    parser.add_argument('--points', type=float, nargs='+', required=True,
                        help='Sequence of freq gain pairs (f1 g1 f2 g2 ...)')
    parser.add_argument('--file', type=str, default='fir_coeffs.h', help='Output file')

    args = parser.parse_args()

    if len(args.points) % 2 != 0:
        print("Error: --points must be pairs of frequency and gain.")
        sys.exit(1)

    freqs = list(args.points[0::2])
    gains = list(args.points[1::2])

    # Fix the Nyquist/Epsilon issue
    nyquist = args.fs / 2.0
    eps = 1e-5 

    if freqs[0] > eps:
        freqs.insert(0, 0.0)
        gains.insert(0, gains[0])
    else:
        freqs[0] = 0.0
    
    if freqs[-1] > nyquist + eps:
        print(f"Error: Frequency {freqs[-1]} exceeds Nyquist ({nyquist}).")
        sys.exit(1)
        
    if freqs[-1] < nyquist - eps:
        freqs.append(nyquist)
        gains.append(gains[-1])
    else:
        freqs[-1] = nyquist # Snap to exact Nyquist

    try:
        taps = signal.firwin2(numtaps=args.taps, freq=freqs, gain=gains, 
                             window=args.window, fs=args.fs)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Building the output with proper C formatting
    out = []
    out.append("/*")
    out.append(" * CMSIS-DSP FIR Coefficients for arm_fir_f32")
    out.append(" * Method: firwin2 (Frequency Sampling)")
    out.append(f" * Taps: {args.taps}, Window: {args.window}")
    out.append(" * Defined Points (Hz, Gain):")
    for f, g in zip(freqs, gains):
        out.append(f" * {f:8.2f} Hz -> {g:6.3f}")
    out.append(" */\n")
    
    out.append("#ifndef FIR_COEFFS_H")
    out.append("#define FIR_COEFFS_H\n")
    out.append("#include \"arm_math.h\"\n")
    out.append(f"#define NUM_TAPS {args.taps}\n")
    
    out.append(f"float32_t fir_coeffs[NUM_TAPS] = {{")
    for i in range(0, len(taps), 4):
        chunk = taps[i:i+4]
        line = ", ".join([f"{val:14.10f}f" for val in chunk])
        comma = "," if i + 4 < len(taps) else ""
        out.append(f"    {line}{comma}")
    out.append("};\n")
    
    out.append("// BLOCK_SIZE must be defined in your main project")
    out.append(f"float32_t fir_state[NUM_TAPS + BLOCK_SIZE - 1] = {{0}};\n")
    out.append("#endif // FIR_COEFFS_H")

    try:
        with open(args.file, 'w') as f:
            f.write("\n".join(out))
        print(f"Successfully wrote {args.file}")
    except IOError as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
