#!/bin/python3

import argparse
import numpy as np
from scipy import signal
import sys
import os

def main():
    parser = argparse.ArgumentParser(
        description='CMSIS-DSP Biquad Coefficient Generator for RP2350/Pico 2',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Example Invocations:
  python gen_filter.py --btype lp --fc 2000
  python gen_filter.py --btype bp --fc 1000 3000 --file my_filter.h
        """
    )
    
    # Fundamental Settings
    parser.add_argument('--fs', type=float, default=48828.125, 
                        help='Sample rate in Hz (default: 48828.125)')
    parser.add_argument('--ftype', choices=['butter', 'cheby1', 'cheby2', 'ellip', 'bessel'], 
                        default='cheby1', help='Filter prototype math (default: cheby1)')
    parser.add_argument('--btype', choices=['lp', 'hp', 'bp', 'bs'], 
                        default='lp', help='Filter shape: lp (lowpass), hp (highpass), bp (bandpass), bs (bandstop)')
    
    # Filter Parameters
    parser.add_argument('--order', type=int, default=6, 
                        help='Filter order (number of poles) (default: 6)')
    parser.add_argument('--fc', type=float, nargs='+', 
                        help='Cutoff frequency in Hz. Single value for lp/hp, two values for bp/bs.')
    parser.add_argument('--rp', type=float, default=1.0, 
                        help='Passband ripple in dB (default: 1.0)')
    parser.add_argument('--rs', type=float, default=40.0, 
                        help='Stopband attenuation in dB (default: 40.0)')

    # Output Settings
    parser.add_argument('--file', type=str, default='iir_coeffs.h',
                        help='Output header file name (default: iir_coeffs.h)')

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

    btype_map = {'lp': 'lowpass', 'hp': 'highpass', 'bp': 'bandpass', 'bs': 'bandstop'}
    
    try:
        sos = signal.iirfilter(
            N=args.order, Wn=args.fc, rp=args.rp, rs=args.rs,
            btype=btype_map[args.btype], ftype=args.ftype,
            analog=False, fs=args.fs, output='sos'
        )
    except ValueError as e:
        print(f"Error generating filter: {e}")
        sys.exit(1)

    num_stages = len(sos)
    
    # Generate the string content
    output = []
    output.append(f"/*")
    output.append(f" * CMSIS-DSP Biquad Coefficients for arm_biquad_cascade_df2T_f32")
    output.append(f" * Generated for RP2350 / Pico 2")
    output.append(f" * Prototype: {args.ftype}, Shape: {btype_map[args.btype]}")
    output.append(f" * Order: {args.order}, Total Biquad Stages: {num_stages}")
    output.append(f" * Cutoff(s): {args.fc} Hz, FS: {args.fs} Hz")
    output.append(f" */\n")
    
    output.append(f"#ifndef IIR_COEFFS_H")
    output.append(f"#define IIR_COEFFS_H\n")
    output.append(f"#include \"arm_math.h\"\n")
    output.append(f"#define NUM_STAGES {num_stages}\n")
    output.append(f"float32_t iir_coeffs[{num_stages * 5}] = {{")

    for i, stage in enumerate(sos):
        b0, b1, b2, a0, a1, a2 = stage
        b0 /= a0; b1 /= a0; b2 /= a0
        na1 = -(a1 / a0); na2 = -(a2 / a0)

        line = f"    {b0:14.10f}f, {b1:14.10f}f, {b2:14.10f}f, {na1:14.10f}f, {na2:14.10f}f"
        comma = "," if i < num_stages - 1 else ""
        output.append(f"{line}{comma} // Stage {i+1}")

    output.append("};\n")
    output.append(f"float32_t iir_state[2 * NUM_STAGES] = {{0}};\n")
    output.append(f"#endif // IIR_COEFFS_H")

    # Write to file
    try:
        with open(args.file, 'w') as f:
            f.write("\n".join(output))
        print(f"Successfully wrote coefficients to {args.file}")
    except IOError as e:
        print(f"Error writing to file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
