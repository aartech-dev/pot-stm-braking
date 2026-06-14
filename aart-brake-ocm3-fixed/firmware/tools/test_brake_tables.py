#!/usr/bin/env python3
"""
AART brake-table inspector.

Shows the brake and anti-brake DAC values (and rough physical estimates)
at any pot setting, for any profile. Reproduces exactly what the firmware
does at runtime: pot -> 8-bit index -> two table lookups.

Usage examples:
    python3 test_brake_tables.py                 # interactive-ish demo
    python3 test_brake_tables.py --ohms 3 --pot 40
    python3 test_brake_tables.py --ohms 3 --pot 40 --raw 1638
    python3 test_brake_tables.py --sweep --ohms 3

--pot is a percentage 0..100 (0 = full CCW, 100 = full CW).
--raw overrides with a literal 12-bit ADC value 0..4095.
"""

import argparse
from gen_brake_tables import (
    DEFAULT_PROFILES, make_brake_table, make_ka_table,
    DAC_KA_OFF, DAC_BRAKE_HARD, DAC_MAX, TABLE_LEN,
)

# ---- Rough physical interpretation (ESTIMATES - calibrate on bench) -------
# DAC code -> gate voltage is linear: V = code/4095 * 3.3
VREF = 3.3

# Anchor points for a hand-wavy DAC->ohms read-out, purely illustrative.
# Brake: higher DAC = lower resistance. Anchored to the design estimates.
_BRAKE_ANCHORS = [(1860, 8.0), (1950, 6.0), (2100, 4.0),
                  (2200, 3.0), (2300, 2.0), (3095, 0.05)]


def dac_to_volts(code):
    return code / DAC_MAX * VREF


def brake_dac_to_ohms_est(code):
    """Piecewise-linear interpolation over the anchor estimates. Illustrative
    only - the real curve is FET/temp/current dependent."""
    a = _BRAKE_ANCHORS
    if code <= a[0][0]:
        return a[0][1]
    if code >= a[-1][0]:
        return a[-1][1]
    for (c0, o0), (c1, o1) in zip(a, a[1:]):
        if c0 <= code <= c1:
            f = (code - c0) / (c1 - c0)
            return round(o0 + (o1 - o0) * f, 2)
    return a[-1][1]


def ka_dac_to_inject_volts_est(code):
    """Anti-brake injection estimate. ka_max(2480)~2.0V down to ka_off=0V."""
    if code >= DAC_KA_OFF:
        return 0.0
    # 2480 -> ~2.0V injection, linear toward 0 at DAC_KA_OFF
    span = DAC_KA_OFF - 2480
    if span <= 0:
        return 0.0
    frac = (DAC_KA_OFF - code) / span
    return round(2.0 * frac, 2)


def get_profile(ohms):
    for p in DEFAULT_PROFILES:
        if p.label_ohms == ohms:
            return p
    # nearest
    return min(DEFAULT_PROFILES, key=lambda p: abs(p.label_ohms - ohms))


def report(profile, raw):
    idx = (raw >> 4) & 0xFF
    brake_tbl = make_brake_table(profile)
    ka_tbl = make_ka_table(profile)
    bdac = brake_tbl[idx]
    kdac = ka_tbl[idx]

    pot_pct = raw / DAC_MAX * 100
    print(f"  Profile          : ~{profile.label_ohms} ohm full scale "
          f"(brake_soft={profile.brake_soft}, gamma={profile.brake_gamma})")
    print(f"  Pot raw / index  : {raw:4d} / {idx:3d}   ({pot_pct:5.1f}% CW)")
    print(f"  -- BRAKE  (Q2) --")
    print(f"    DAC code       : {bdac:4d}   ({dac_to_volts(bdac):.2f} V gate)")
    print(f"    est. resistance: ~{brake_dac_to_ohms_est(bdac)} ohm")
    print(f"  -- ANTI-BRAKE (Q1) --")
    if kdac >= DAC_KA_OFF:
        print(f"    DAC code       : {kdac:4d}   (Q1 OFF - no injection)")
    else:
        print(f"    DAC code       : {kdac:4d}   ({dac_to_volts(kdac):.2f} V gate)")
        print(f"    est. injection : ~{ka_dac_to_inject_volts_est(kdac)} V forward")
    print()


def sweep(profile, step_pct=10):
    brake_tbl = make_brake_table(profile)
    ka_tbl = make_ka_table(profile)
    print(f"  Sweep for ~{profile.label_ohms} ohm profile "
          f"(brake_soft={profile.brake_soft}):")
    print(f"  {'pot%':>5} {'idx':>4} {'brakeDAC':>9} {'~ohm':>6} "
          f"{'kaDAC':>6} {'~inj V':>7}")
    print("  " + "-" * 44)
    for pct in range(0, 101, step_pct):
        raw = round(pct / 100 * DAC_MAX)
        idx = (raw >> 4) & 0xFF
        b = brake_tbl[idx]
        k = ka_tbl[idx]
        inj = "off" if k >= DAC_KA_OFF else f"{ka_dac_to_inject_volts_est(k)}"
        print(f"  {pct:5d} {idx:4d} {b:9d} {brake_dac_to_ohms_est(b):6} "
              f"{k:6d} {inj:>7}")
    print()


def main():
    ap = argparse.ArgumentParser(description="Inspect brake/anti-brake tables")
    ap.add_argument("--ohms", type=int, default=3,
                    help="profile to use (nearest of 2,3,4,6,8). default 3")
    ap.add_argument("--pot", type=float, default=None,
                    help="pot position 0..100 %% (0=CCW, 100=CW)")
    ap.add_argument("--raw", type=int, default=None,
                    help="raw 12-bit ADC 0..4095 (overrides --pot)")
    ap.add_argument("--sweep", action="store_true",
                    help="print a full sweep table for the profile")
    args = ap.parse_args()

    profile = get_profile(args.ohms)

    if args.sweep:
        sweep(profile)
        return

    if args.raw is not None:
        raw = max(0, min(DAC_MAX, args.raw))
    elif args.pot is not None:
        raw = round(max(0.0, min(100.0, args.pot)) / 100 * DAC_MAX)
    else:
        # default demo: three representative pot positions
        print("No --pot/--raw given; showing CCW, mid, CW for the profile.\n")
        for pct in (0, 50, 100):
            report(profile, round(pct / 100 * DAC_MAX))
        return

    report(profile, raw)


if __name__ == "__main__":
    main()
