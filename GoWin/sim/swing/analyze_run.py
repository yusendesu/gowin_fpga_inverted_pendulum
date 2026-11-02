import csv

p = r"D:\DeepSeek\pendulum\run.csv"
rows = list(csv.DictReader(open(p)))
print("t arm_deg angle_raw angle_target angle_err loc_err loc_cmd angle_pid loc_pid motor duty run")
for r in rows:
    t = float(r["t_s"])
    if 23.0 <= t <= 33.0:
        # print every ~0.1 s
        if abs(t * 10 - round(t * 10)) < 0.011:
            print(
                f"{t:5.1f} {float(r['arm_deg']):8.2f} {r['angle_raw']:>4} "
                f"{r['angle_target']:>4} {r['angle_error']:>5} {r['loc_error']:>5} "
                f"{r['loc_cmd']:>4} {r['angle_pid']:>4} {r['loc_pid']:>4} "
                f"{r['motor']:>4} {r['duty']:>3} {r['run']}"
            )
