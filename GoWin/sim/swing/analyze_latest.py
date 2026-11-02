import csv

p = r"D:\DeepSeek\pendulum\run.csv"
rows = list(csv.DictReader(open(p)))
for seg in sorted(set(r["segment"] for r in rows), key=int):
    sub = [r for r in rows if r["segment"] == seg]
    t0 = float(sub[0]["t_s"])
    t1 = float(sub[-1]["t_s"])
    print("seg", seg, "n", len(sub), "t", t0, "->", t1, "dt", round(t1 - t0, 2))
    for idx in [0, 1, len(sub) // 2, len(sub) - 2, len(sub) - 1]:
        r = sub[idx]
        arm = float(r["arm_deg"])
        le = float(r["loc_error"])
        tgt = arm + le * 360 / 1320
        print(
            f"  t={float(r['t_s']):8.2f} arm={arm:8.2f} tgt={tgt:8.2f} "
            f"locerr={le:6.1f} loc_cmd={r['loc_cmd']:>4} angle={r['angle_raw']:>4} "
            f"run={r['run']} inwin={r['in_window']}"
        )
