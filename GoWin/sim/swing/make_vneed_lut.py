#!/usr/bin/env python3
"""Generate the v_need LUT used by src/swing_up.v.

v_need(e) = sqrt(2*K*(1-cos(pi*|e|/2048)) + 2*margin)
K         = (g/l) * (T * 2048/pi)^2
T         = 5 ms control period, |e| in 12-bit angle counts.

The default table in swing_up.v is for l = 0.30 m and margin = 0.
If your pendulum length is different, run this script and paste the generated
case items into swing_up.v (or keep the default and tune VNEED_OFFSET).
"""
import argparse
import math

G = 9.81
T = 0.005
CPR = 2048.0 / math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=0.30, help="pendulum length in m")
    ap.add_argument("--margin", type=float, default=0.0,
                    help="extra energy (counts^2, positive = a bit more energy)")
    args = ap.parse_args()

    K = (G / args.length) * (T * CPR) ** 2
    vals = []
    for idx in range(33):
        e = idx * 64
        v2 = 2.0 * K * (1.0 - math.cos(e / CPR)) + 2.0 * args.margin
        vals.append(int(round(math.sqrt(v2))) if v2 > 0 else 0)
    # clamp to 8 bit
    vals = [max(0, min(255, v)) for v in vals]

    print(f"// generated: l={args.length:.3f} m, margin={args.margin:g}, K={K:.1f}")
    for idx, v in enumerate(vals):
        if idx < 32:
            print(f"                6'd{idx}: vneed_lut = 8'd{v};")
        else:
            print(f"                default: vneed_lut = 8'd{v};")
    print()
    print("// table:", vals)


if __name__ == "__main__":
    main()
