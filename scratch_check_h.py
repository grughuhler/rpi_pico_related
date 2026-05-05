import math

def read_coeffs():
    coeffs = []
    with open("fft_filter_coeffs.h", "r") as f:
        for line in f:
            if "f" in line and "," in line or "f" in line:
                parts = line.strip().split(",")
                for p in parts:
                    p = p.strip()
                    if p.endswith("f") and not p.startswith("#"):
                        try:
                            coeffs.append(float(p[:-1]))
                        except ValueError:
                            pass
    return coeffs

coeffs = read_coeffs()
M = len(coeffs)

# We want to check the magnitude response of these coeffs
# Let's compute DFT magnitude at various frequencies

N = 1000
max_stopband = -200.0
for i in range(N):
    freq = 25000 * i / N
    # normalized freq
    w = 2 * math.pi * freq / 48828.125
    
    real_part = 0.0
    imag_part = 0.0
    for n in range(M):
        real_part += coeffs[n] * math.cos(w * n)
        imag_part += coeffs[n] * math.sin(w * n)
        
    mag = math.sqrt(real_part**2 + imag_part**2)
    mag_db = 20 * math.log10(mag) if mag > 1e-10 else -200
    
    if freq > 5000:
        max_stopband = max(max_stopband, mag_db)

print(f"Max stopband magnitude (>5000 Hz): {max_stopband:.2f} dB")
