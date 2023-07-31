def cycle(x):
    return round(((-5e-4 * x + 0.2098) * x - 31.43) * x + 1675)

fs = 20000 # Hz
with open('../crossing.X/lampcycles.inc','w') as f:
    f.write("; LED PFM cycles (V <= ~3.3 V)\n")
    for k in range(79, 149):
        c = min(max(cycle(k), 1), 255)
        fc = fs / c
        v = 1.024 / (k / 256)
        f.write(f"    dw          {c} ; {v:.2f} V, {fc:.1f} Hz\n")
