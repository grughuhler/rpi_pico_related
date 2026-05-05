import math
import sys

def generate_fir(fc_hz, fs_hz, taps, output_file):
    fc = fc_hz / fs_hz
    M = taps
    alpha = (M - 1) / 2.0
    
    coeffs = []
    sum_c = 0.0
    
    for n in range(M):
        if n == alpha:
            h_d = 2 * fc
        else:
            x = 2 * math.pi * fc * (n - alpha)
            h_d = math.sin(x) / (math.pi * (n - alpha))
            
        # Blackman window
        w = 0.42 - 0.5 * math.cos(2 * math.pi * n / (M - 1)) + 0.08 * math.cos(4 * math.pi * n / (M - 1))
        
        c = h_d * w
        coeffs.append(c)
        sum_c += c
        
    # Normalize
    coeffs = [c / sum_c for c in coeffs]
    
    with open(output_file, 'w') as f:
        f.write("/* Generated pure-python FIR coeffs */\n")
        f.write("#ifndef FFT_FILTER_COEFFS_H\n")
        f.write("#define FFT_FILTER_COEFFS_H\n\n")
        f.write("#include \"arm_math.h\"\n\n")
        f.write(f"#define NUM_TAPS {M}\n\n")
        f.write("float32_t fir_coeffs[NUM_TAPS] = {\n")
        
        for i in range(0, M, 4):
            chunk = coeffs[i:i+4]
            line = ", ".join([f"{val:14.10f}f" for val in chunk])
            comma = "," if i + 4 < M else ""
            f.write(f"    {line}{comma}\n")
            
        f.write("};\n\n")
        f.write("#endif\n")

if __name__ == "__main__":
    generate_fir(4000.0, 48828.125, 449, "fft_filter_coeffs.h")
    print("Done")
