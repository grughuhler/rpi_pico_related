import numpy as np
import sys

num_phases = 64
num_points = 16

print("/* Auto-generated 16-point Sinc Table with Blackman-Harris Window */")
print("#ifndef SINC_TABLE_H")
print("#define SINC_TABLE_H")
print("#include \"arm_math.h\"")
print("")
print(f"const float32_t sinc_table[{num_phases}][{num_points}] = {{")

for phase in range(num_phases):
    frac = phase / float(num_phases)
    
    row = []
    for i in range(num_points):
        # Center is between index 7 and 8
        x = i - (num_points // 2 - 1) - frac
        
        if x == 0.0:
            sinc_val = 1.0
        else:
            sinc_val = np.sin(np.pi * x) / (np.pi * x)
            
        # Moving window tracking the fractional offset
        n = x + (num_points / 2)
        if n < 0 or n >= num_points:
            w = 0.0
        else:
            w = (0.35875 - 0.48829 * np.cos(2 * np.pi * n / num_points) + 
                 0.14128 * np.cos(4 * np.pi * n / num_points) - 
                 0.01168 * np.cos(6 * np.pi * n / num_points))
                 
        row.append(sinc_val * w)
        
    print("  { " + ", ".join([f"{v:.6f}f" for v in row]) + " },")

print("};")
print("#endif")
