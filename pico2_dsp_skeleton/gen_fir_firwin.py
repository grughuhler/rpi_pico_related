#!/bin/python3

import argparse
import numpy as np
from scipy import signal
import sys

def main():
    parser = argparse.ArgumentParser(
        description='CMSIS-DSP FIR Coefficient Generator for RP2350/Pico 2',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Example Invocations:
  python gen_fir.py --btype lp --fc 2000 --taps 101
  python gen_fir.py --btype bp --fc 1000 3000 --window blackman
  python gen_fir.py --btype hp --fc 5000 --taps 151 --file highpass_fir.h
        """
    )
    
    # Fundamental Settings
    parser.add_argument('--fs', type=float, default=48828.125, 
                        help='Sample rate in Hz (default: 48828.125)')
    parser.add_argument('--btype', choices=['lp', 'hp', 'bp', 'bs'], 
                        default='lp', help='Filter shape: lp, hp, bp, bs')
    parser.add_argument('--window', default='hamming',
                        help='Window type: hamming, hann, blackman, nuttall, etc. (default: hamming)')
    
    # Filter Parameters
    parser.add_argument('--taps', type=int, default=101, 
                        help='Number of filter taps (must be odd for some filter types) (default: 101)')
    parser.add_argument('--fc', type=float, nargs='+', 
                        help='Cutoff frequency in Hz. Single value for lp/hp, two values for bp/bs.')

    # Output Settings
    parser.add_argument('--file', type=str, default='fir_coeffs.h',
                        help='Output header file name (default: fir_coeffs.h)')

    args = parser.parse_args()

    # Handle conditional defaults for FC
    if args.fc is None:
        args.fc = [4000.0] if args.btype in ['lp', 'hp'] else [4000.0, 8000.0]
    
    # Validation
    if args.btype in ['lp', 'hp'] and len(args.fc) != 1:
        print(f"Error: --btype {args.btype} requires one frequency.")
        sys.exit(1)
    elif args.btype in ['bp', 'bs'] and len(args.fc) != 2:
        print(f"Error: --btype {args.btype} requires two frequencies.")
        sys.exit(1)

    # Map shorthands to firwin pass_zero logic
    # pass_zero=True means 0Hz is in the passband (Lowpass or Bandstop)
    pass_zero_map = {'lp': True, 'hp': False, 'bp': False, 'bs': True}
    
    try:
        # Generate FIR Taps
        taps = signal.firwin(
            numtaps=args.taps,
            cutoff=args.fc,
            pass_zero=pass_zero_map[args.btype],
            window=args.window,
            fs=args.fs
        )
    except ValueError as e:
        print(f"Error generating FIR filter: {e}")
        sys.exit(1)

    # Generate the string content
    output = []
    output.append(f"/*")
    output.append(f" * CMSIS-DSP FIR Coefficients for arm_fir_f32")
    output.append(f" * Generated for RP2350 / Pico 2")
    output.append(f" * Method: Windowed FIR (Window: {args.window})")
    output.append(f" * Taps (Length): {args.taps} (Order: {args.taps - 1})")
    output.append(f" * Shape: {args.btype}, Cutoff(s): {args.fc} Hz, FS: {args.fs} Hz")
    output.append(f" */\n")
    
    output.append(f"#ifndef FIR_COEFFS_H")
    output.append(f"#define FIR_COEFFS_H\n")
    output.append(f"#include \"arm_math.h\"\n")
    output.append(f"#define NUM_TAPS {args.taps}\n")
    
    # Note: arm_fir_init_f32 expects coefficients in normal order.
    # Some older versions or specialized FIR functions expect time-reversed.
    output.append(f"float32_t fir_coeffs[NUM_TAPS] = {{")

    for i in range(0, len(taps), 4):
        chunk = taps[i:i+4]
        line = ", ".join([f"{val:14.10f}f" for val in chunk])
        comma = "," if i + 4 < len(taps) else ""
        output.append(f"    {line}{comma}")

    output.append("};\n")
    
    # State buffer for arm_fir_f32 is (numTaps + blockSize - 1)
    output.append(f"float32_t fir_state[NUM_TAPS + BLOCK_SIZE - 1] = {{0}};\n")
    output.append(f"#endif // FIR_COEFFS_H")

    # Write to file
    try:
        with open(args.file, 'w') as f:
            f.write("\n".join(output))
        print(f"Successfully wrote FIR coefficients to {args.file}")
    except IOError as e:
        print(f"Error writing to file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
