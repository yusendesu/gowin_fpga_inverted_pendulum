import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PAT = re.compile(
    r"t=\s*([\d.]+)ms\s+Angle=\s*(-?\d+)\s+err=\s*(-?\d+)\s+err_int=\s*(-?\d+)\s+"
    r"out=\s*(-?\d+)\s+run=(\d)\s+motor=\s*(-?\d+)\s+duty=\s*(\d+)")


def load(path):
    rows = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = PAT.search(line)
        if m:
            rows.append([float(x) for x in m.groups()])
    return rows


orig = load("sim/run_orig.log")
fix = load("sim/run_fix.log")


def col(rows, i):
    return [r[i] for r in rows]


fig, ax = plt.subplots(3, 1, figsize=(9.0, 7.2), sharex=True,
                       gridspec_kw={"height_ratios": [1.0, 1.4, 1.3]})

for rows, lab, c in ((orig, "original RTL", "#d62728"), (fix, "fixed RTL", "#1f77b4")):
    ax[0].plot(col(rows, 0), col(rows, 1), color=c, lw=1.6, label=lab)
    ax[1].plot(col(rows, 0), [v / 65536.0 for v in col(rows, 3)], color=c, lw=1.6, label=lab)
    ax[2].step(col(rows, 0), col(rows, 6), where="post", color=c, lw=1.6, label=lab)

ax[0].set_ylabel("Angle (ADC counts)")
ax[0].set_title("Reproduction: angle near balance (-6 counts) after 100 ms with the pole down")
ax[1].set_ylabel("integral err_int (counts)")
ax[1].set_yscale("symlog", linthresh=200)
ax[1].axhline(32767, color="gray", ls="--", lw=0.8, label="16.16 overflow")
ax[1].axhline(-32768, color="gray", ls="--", lw=0.8)

ax[1].annotate("wrap-around", xy=(85, -32128), xytext=(55, -8000),
               fontsize=8, color="#d62728",
               arrowprops=dict(arrowstyle="->", color="#d62728", lw=0.8))
ax[2].set_ylabel("motor command (signed, % duty)")
ax[2].set_xlabel("time (ms)")
ax[2].set_ylim(-110, 110)
ax[2].axhline(100, color="gray", ls=":", lw=0.8)
ax[2].axhline(-100, color="gray", ls=":", lw=0.8)

for a in ax:
    a.axvline(100, color="k", ls=":", lw=0.9)
    a.axvline(130, color="k", ls=":", lw=0.9)
    a.grid(True, alpha=0.3)
ax[0].annotate("pole lifted\nto balance", xy=(100, 2096), xytext=(58, 1600),
               fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.8))
ax[2].annotate("start -> full speed", xy=(130, -100), xytext=(140, -55),
               fontsize=8, color="#d62728",
               arrowprops=dict(arrowstyle="->", color="#d62728", lw=0.8))
ax[2].annotate("2~3 %", xy=(175, -3), xytext=(150, 40), fontsize=8, color="#1f77b4",
               arrowprops=dict(arrowstyle="->", color="#1f77b4", lw=0.8))

ax[0].legend(loc="lower right", fontsize=8, framealpha=0.9)
fig.tight_layout()
fig.savefig("sim/compare.png", dpi=160)
print("saved sim/compare.png")
