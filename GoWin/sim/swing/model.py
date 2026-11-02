#!/usr/bin/env python3
"""Python model of the pendulum + the new auto swing-up / balance handover.

This is a quick design aid, not a replacement for the RTL simulation.
Plant: pendulum on a horizontally accelerated pivot (cart approximation of
the Furuta arm near the pendulum).  The sign convention is chosen so that the
existing balance PID (out = Kp*(target-actual) + Kd*d(error)) is stable.
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------- plant ----------------
G = 9.81
L = 0.30                 # m, pendulum length
A_FULL = 40.0            # m/s^2 pivot acceleration at 100% duty
FRICTION = 0.6           # 1/s, viscous damping at the pendulum pivot
DT = 1e-4
T_END = 6.0

# ---------------- sensor / control ----------------
CENTER = 2053
COUNTS_PER_RAD = 2048.0 / math.pi
CTRL_T = 0.005           # 5 ms
ANGLE_KP, ANGLE_KI, ANGLE_KD = 0.30, 0.01, 0.40
ANGLE_INT_MAX = 500

# ---------------- swing-up ----------------
SWING_DUTY = 80
KICK_DUTY = 35
KICK_MS = 100
CAPTURE_ANGLE = 300
CAPTURE_VEL = 8
CAPTURE_TICKS = 3
TIMEOUT_MS = 4000
VEL_DEADBAND = 2
ENERGY_MARGIN = 0.0
K_ENERGY = (G / L) * (CTRL_T * COUNTS_PER_RAD) ** 2
POLARITY = +1


def signed12(x):
    x &= 0xFFF
    return x - 0x1000 if x >= 0x800 else x


def wrap_counts(theta):
    return int(round((theta * COUNTS_PER_RAD + CENTER) % 4096))


class Plant:
    def __init__(self, theta0=math.pi, omega0=0.0):
        self.theta = theta0
        self.omega = omega0

    def step(self, u, dt=DT):
        # theta measured from upright; u in [-1, 1]; positive u -> positive
        # angular acceleration at theta=0 (matches a stable balance loop).
        alpha = (G / L) * math.sin(self.theta) + (A_FULL / L) * u * math.cos(self.theta)
        alpha -= FRICTION * self.omega
        self.omega += alpha * dt
        self.theta += self.omega * dt
        # keep in [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi


def run(polarity=POLARITY, swing_duty=SWING_DUTY, kick_duty=KICK_DUTY,
        kick_ms=KICK_MS, capture_angle=CAPTURE_ANGLE, capture_vel=CAPTURE_VEL):
    plant = Plant()
    n = int(T_END / DT)
    ctrl_every = int(CTRL_T / DT)

    # controller state
    state = 0            # 0=idle/hang confirm, 1=swing, 2=balance
    hang_ms = 0
    armed = True
    swing_ms = 0
    kick_ms_cnt = 0
    e_prev = None
    v_filt = 0.0
    cap_cnt = 0
    int_err = 0.0
    err_prev = 0.0
    run_prev = False
    balance_started = False

    log = {k: [] for k in ("t", "theta", "theta_cmd", "angle", "e", "v",
                           "drive", "state", "pwm")}
    for k in ("t5", "e5", "v5", "drive5", "state5", "theta5"):
        log[k] = []
    drive = 0
    angle = wrap_counts(plant.theta)

    for i in range(n):
        t = i * DT
        # --- plant ---
        plant.step(drive / 100.0)
        # --- 1 ms sensor/state tick ---
        if i % int(0.001 / DT) == 0:
            angle = wrap_counts(plant.theta)
            if state == 0:
                hang_now = (angle <= 80) or (angle >= 4096 - 80)
                if hang_now:
                    hang_ms += 1
                else:
                    hang_ms = 0
                if hang_ms >= 500:
                    state = 1
                    swing_ms = 0
                    kick_ms_cnt = 0
                    e_prev = None
                    v_filt = 0.0
                    cap_cnt = 0
            elif state == 2:
                # fall protection uses raw angle (12 bit) in +/-500 window
                if not (CENTER - 500 < angle < CENTER + 500):
                    state = 0
                    hang_ms = 0
                    balance_started = False
                    int_err = 0.0
                    err_prev = 0.0
        # --- 5 ms control tick ---
        if i > 0 and i % ctrl_every == 0:
            log["t5"].append(t)
            log["e5"].append(signed12(angle - CENTER))
            log["v5"].append(v_filt)
            log["drive5"].append(drive)
            log["state5"].append(state)
            log["theta5"].append(math.degrees(plant.theta))
            if state == 1:
                swing_ms += 5
                e = signed12(angle - CENTER)
                if e_prev is None:
                    e_prev = e
                v_raw = signed12(e - e_prev)
                e_prev = e
                v_filt = v_filt + (v_raw - v_filt) * 0.5
                e_abs = abs(e)
                if kick_ms_cnt < kick_ms:
                    kick_ms_cnt += 5
                    # kick, but never kick when already at the top
                    if e_abs > capture_angle:
                        drive = polarity * kick_duty
                    else:
                        drive = 0
                else:
                    cos_neg = e_abs > 1024
                    # speed needed to just reach upright from the current angle
                    e_rad = e_abs / COUNTS_PER_RAD
                    v_need = math.sqrt(max(0.0, 2 * K_ENERGY * (1 - math.cos(e_rad))
                                           + 2 * ENERGY_MARGIN))
                    pump_ok = abs(v_filt) < v_need
                    if VEL_DEADBAND < abs(v_filt) and pump_ok and e_abs > capture_angle:
                        # u = +sign(v * cos(e)); cos(e)<0 when the rod is
                        # below the horizontal (|e| > 1024 counts).
                        prod_neg = (v_filt < 0) ^ cos_neg
                        drive = -polarity * swing_duty if prod_neg else polarity * swing_duty
                    else:
                        drive = 0
                # capture detection
                if e_abs < capture_angle and abs(v_filt) <= capture_vel:
                    cap_cnt += 1
                else:
                    cap_cnt = 0
                if cap_cnt >= CAPTURE_TICKS:
                    state = 2
                    balance_started = True
                    print(f"captured at t={t:.3f}s e={e} v={v_filt:.1f} swing_ms={swing_ms}")
            elif state == 2:
                e = signed12(angle - CENTER)
                err = 0 - e  # target - actual in counts
                if not balance_started:
                    balance_started = True
                    err_prev = err
                    int_err = 0.0
                int_err += err
                int_err = max(-ANGLE_INT_MAX, min(ANGLE_INT_MAX, int_err))
                out = ANGLE_KP * err + ANGLE_KI * int_err + ANGLE_KD * (err - err_prev)
                err_prev = err
                out = max(-100, min(100, out))
                # 8 bit signed truncation like RTL
                drive = int(out)
                if swing_ms and t > 0 and not log["t"]:
                    pass
        # log every 10 ms
        if i % int(0.010 / DT) == 0:
            log["t"].append(t)
            log["theta"].append(math.degrees(plant.theta))
            log["theta_cmd"].append(drive / 100.0)
            log["angle"].append(angle)
            log["e"].append(signed12(angle - CENTER))
            log["v"].append(v_filt)
            log["drive"].append(drive)
            log["state"].append(state)
            log["pwm"].append(0)
    return log


def main():
    log = run()
    t = np.array(log["t"])
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    ax[0].plot(t, log["theta"], label="pendulum angle from upright (deg)")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].axhline(180, color="r", lw=0.5, ls="--")
    ax[0].axhline(-180, color="r", lw=0.5, ls="--")
    ax[0].set_ylabel("deg")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, log["drive"], label="motor command %")
    ax[1].set_ylabel("%")
    ax[1].legend(loc="upper right")
    ax[2].plot(t, log["state"], label="state 0=idle 1=swing 2=balance")
    ax[2].set_ylabel("state")
    ax[2].set_xlabel("time (s)")
    ax[2].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig("sim/swing/model.png", dpi=130)
    print("saved sim/swing/model.png")


if __name__ == "__main__":
    main()
