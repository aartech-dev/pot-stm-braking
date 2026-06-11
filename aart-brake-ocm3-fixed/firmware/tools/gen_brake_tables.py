#!/usr/bin/env python3
"""Generate the brake DAC tables for brake_module.c.

Non-linear (geometric-ish) resistance set: finer steps where small
differences matter most (low ohms), coarser at the gentle end.

Higher DAC = higher Q2 gate voltage = lower Rds(on) = lower resistance.
OHM_LABELS and SOFT_ENDPOINTS are ESTIMATES. Measure effective
resistance on the bench, adjust, re-run, and paste the output over
brake_ohms_label[], brake_soft_dac_by_ohm[] and brake_tables[][].
"""
GAMMA = 1.8
HARD  = 3095
# Non-linear set. index 0..4.
OHM_LABELS     = [2,    3,    4,    6,    8]      # nominal full-scale Ω
SOFT_ENDPOINTS = [2300, 2200, 2100, 1950, 1860]  # DAC at pot CCW

def gen(soft):
    return [max(soft, min(HARD, round(soft + (HARD-soft)*((i/255.0)**GAMMA))))
            for i in range(256)]

print("static const uint16_t brake_ohms_label[BRAKE_OHMS_COUNT] = {")
print("    " + ", ".join(str(o) for o in OHM_LABELS) + ",")
print("};\n")
print("static const uint16_t brake_soft_dac_by_ohm[BRAKE_OHMS_COUNT] = {")
print("    " + ", ".join(str(e) for e in SOFT_ENDPOINTS) + ",")
print("};\n")
print("static const uint16_t brake_tables[BRAKE_OHMS_COUNT][256] = {")
for k,(o,soft) in enumerate(zip(OHM_LABELS, SOFT_ENDPOINTS)):
    t = gen(soft)
    print(f"  /* index {k}: ~{o} ohm full scale (soft={soft}) */")
    print("  {")
    for r in range(0,256,16):
        print("    " + ", ".join(f"{v:4d}" for v in t[r:r+16]) + ",")
    print("  },")
print("};")
