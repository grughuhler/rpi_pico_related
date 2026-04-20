import argparse
import numpy as np
from scipy import signal
import sys

def main():
    parser = argparse.ArgumentParser(
        description='CMSIS-DSP Biquad Coefficient Generator for RP2350/Pico 2',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Example Invocations:
  python gen_filter.py --btype lp --fc 2000
  python gen_filter.py --ftype butter --btype hp --order 8
  python gen_filter.py --btype bp --fc 1000 3000
  python gen_filter.py --ftype ellip --rp 0.1 --rs 80
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
                        help='Cutoff frequency in Hz. Single value for lp/hp, two values for bp/bs.\n'
                             'Defaults: 4000 for lp/hp, [4000, 8000] for bp/bs.')
    parser.add_argument('--rp', type=float, default=1.0, 
                        help='Passband ripple in dB (used by cheby1 and ellip) (default: 1.0)')
    parser.add_argument('--rs', type=float, default=40.0, 
                        help='Stopband attenuation in dB (used by cheby2 and ellip) (default: 40.0)')

    args = parser.parse_args()

    # Handle conditional defaults for FC
    if args.fc is None:
        if args.btype in ['lp', 'hp']:
            args.fc = [4000.0]
        else:
            args.fc = [4000.0, 8000.0]
    
    # Validation for frequency count
    if args.btype in ['lp', 'hp'] and len(args.fc) != 1:
        print(f"Error: --btype {args.btype} requires exactly one frequency for --fc.")
        sys.exit(1)
    elif args.btype in ['bp', 'bs'] and len(args.fc) != 2:
        print(f"Error: --btype {args.btype} requires two frequencies for --fc (e.g., --fc 4000 8000).")
        sys.exit(1)

    # Map shorthands to scipy strings
    btype_map = {'lp': 'lowpass', 'hp': 'highpass', 'bp': 'bandpass', 'bs': 'bandstop'}
    scipy_btype = btype_map[args.btype]

    # Generate the filter
    try:
        sos = signal.iirfilter(
            N=args.order,
            Wn=args.fc,
            rp=args.rp,
            rs=args.rs,
            btype=scipy_btype,
            ftype=args.ftype,
            analog=False,
            fs=args.fs,
            output='sos'
        )
    except ValueError as e:
        print(f"Error generating filter: {e}")
        sys.exit(1)

    # Print Metadata Header
    num_stages = len(sos)
    print(f"/*")
    print(f" * CMSIS-DSP Biquad Coefficients for arm_biquad_cascade_df2T_f32")
    print(f" * Prototype: {args.ftype}, Shape: {scipy_btype}")
    print(f" * Order: {args.order}, Total Biquad Stages: {num_stages}")
    print(f" * Cutoff(s): {args.fc} Hz, FS: {args.fs} Hz")
    if args.ftype in ['cheby1', 'ellip']: print(f" * Passband Ripple: {args.rp} dB")
    if args.ftype in ['cheby2', 'ellip']: print(f" * Stopband Attenuation: {args.rs} dB")
    print(f" */")

    # Output for C
    print(f"#define NUM_STAGES {num_stages}")
    print(f"float32_t coeffs[{num_stages * 5}] = {{")

    for i, stage in enumerate(sos):
        b0, b1, b2, a0, a1, a2 = stage
        
        # Normalize and negate feedback coefficients
        b0 /= a0
        b1 /= a0
        b2 /= a0
        na1 = -(a1 / a0)
        na2 = -(a2 / a0)

        line = f"    {b0:14.10f}f, {b1:14.10f}f, {b2:14.10f}f, {na1:14.10f}f, {na2:14.10f}f"
        comma = "," if i < num_stages - 1 else ""
        print(f"{line}{comma} // Stage {i+1}")

    print("};")
    print(f"\n// Declare state: float32_t state[2 * NUM_STAGES] = {{0}};")

if __name__ == "__main__":
    main()
