#!/usr/bin/env python3
"""
analyse_hardware_log.py -- Analyse ArduPilot dataflash (.BIN) log from hardware.

Focuses on the yaw / tail-rotor subsystem to diagnose why SERVO4 winds up
even with near-zero yaw rate.  Extracts and plots:

  Panel 1 -- Yaw rate: RATE.YDes vs RATE.Y (deg/s) and PIDR error
  Panel 2 -- Yaw PID terms: P, I, D, FF from PIDR
  Panel 3 -- SERVO4 PWM from RCOU.C4 (us) + HRSC throttle
  Panel 4 -- Raw gyro Z from IMU (deg/s, both IMUs if present)

Usage
-----
    python simulation/analysis/analyse_hardware_log.py <log.BIN>
    python simulation/analysis/analyse_hardware_log.py  # auto-finds latest in simulation/logs/hardware/
"""
from __future__ import annotations

import math
import os
import sys

_SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)


def _find_latest_log() -> str:
    hw_dir = os.path.join(_SIM_DIR, "logs", "hardware")
    if not os.path.isdir(hw_dir):
        raise FileNotFoundError(f"No hardware log dir: {hw_dir}")
    bins = [f for f in os.listdir(hw_dir) if f.upper().endswith(".BIN")]
    if not bins:
        raise FileNotFoundError(f"No .BIN files in {hw_dir}")
    return os.path.join(hw_dir, max(bins))


def _load_log(path: str) -> dict[str, list]:
    """Parse the .BIN log and return arrays for each channel of interest."""
    from pymavlink import DFReader

    data: dict[str, list] = {
        # time base (s) for each message type
        "rate_t": [], "rate_ydes": [], "rate_y": [], "rate_yout": [],
        "pidr_t": [], "pidr_err": [], "pidr_p": [], "pidr_i": [],
        "pidr_d": [], "pidr_ff": [],
        "rcou_t": [], "rcou_c4": [],
        "rcin_t": [], "rcin_c1": [], "rcin_c2": [], "rcin_c3": [], "rcin_c4": [],
        "hrsc_t": [], "hrsc_throt": [],
        "imu0_t": [], "imu0_gz": [],
        "imu1_t": [], "imu1_gz": [],
        "att_t":  [], "att_yaw":  [], "att_desyaw": [],
    }

    log = DFReader.DFReader_binary(path)
    while True:
        msg = log.recv_msg()
        if msg is None:
            break
        t = msg.get_type()
        us = getattr(msg, "TimeUS", None)
        if us is None:
            continue
        ts = us * 1e-6

        if t == "RATE":
            data["rate_t"].append(ts)
            data["rate_ydes"].append(math.degrees(msg.YDes))
            data["rate_y"].append(math.degrees(msg.Y))
            data["rate_yout"].append(msg.YOut)
        elif t == "PIDR":
            data["pidr_t"].append(ts)
            data["pidr_err"].append(math.degrees(msg.Err))
            data["pidr_p"].append(msg.P)
            data["pidr_i"].append(msg.I)
            data["pidr_d"].append(msg.D)
            data["pidr_ff"].append(msg.FF)
        elif t == "RCIN":
            data["rcin_t"].append(ts)
            data["rcin_c1"].append(getattr(msg, "C1", 0))
            data["rcin_c2"].append(getattr(msg, "C2", 0))
            data["rcin_c3"].append(getattr(msg, "C3", 0))
            data["rcin_c4"].append(getattr(msg, "C4", 0))
        elif t == "RCOU":
            data["rcou_t"].append(ts)
            data["rcou_c4"].append(msg.C4)
        elif t == "HRSC":
            data["hrsc_t"].append(ts)
            data["hrsc_throt"].append(msg.Throt)
        elif t == "IMU":
            gz_deg = math.degrees(msg.GyrZ)
            if msg.I == 0:
                data["imu0_t"].append(ts)
                data["imu0_gz"].append(gz_deg)
            elif msg.I == 1:
                data["imu1_t"].append(ts)
                data["imu1_gz"].append(gz_deg)
        elif t == "ATT":
            data["att_t"].append(ts)
            data["att_yaw"].append(msg.Yaw)
            data["att_desyaw"].append(msg.DesYaw)

    return data


def _print_summary(data: dict[str, list]) -> None:
    """Print a text summary of the yaw PID behaviour."""
    sep = "-" * 60

    if data["pidr_t"]:
        i_vals  = data["pidr_i"]
        p_vals  = data["pidr_p"]
        err_vals = data["pidr_err"]
        t0 = data["pidr_t"][0]
        t1 = data["pidr_t"][-1]
        print(f"\n{sep}")
        print("YLAW RATE PID  (PIDR)")
        print(sep)
        print(f"  Duration        : {t1 - t0:.1f} s  ({len(i_vals)} samples)")
        print(f"  Error  mean/max : {sum(err_vals)/len(err_vals):+.4f} / {max(abs(e) for e in err_vals):.4f}  deg/s")
        print(f"  P term range    : {min(p_vals):.4f} .. {max(p_vals):.4f}")
        print(f"  I term range    : {min(i_vals):.4f} .. {max(i_vals):.4f}")
        i_rate = (i_vals[-1] - i_vals[0]) / max(t1 - t0, 1e-3)
        print(f"  I wind-up rate  : {i_rate:+.4f} /s  (I went {i_vals[0]:.4f} -> {i_vals[-1]:.4f})")

    if data["rcou_t"]:
        c4 = data["rcou_c4"]
        t0 = data["rcou_t"][0]
        t1 = data["rcou_t"][-1]
        print(f"\n{sep}")
        print("SERVO4 OUTPUT  (RCOU.C4)")
        print(sep)
        print(f"  Duration        : {t1 - t0:.1f} s  ({len(c4)} samples)")
        print(f"  PWM range       : {min(c4)} .. {max(c4)} us")
        ramp = (c4[-1] - c4[0]) / max(t1 - t0, 1e-3)
        print(f"  Ramp rate       : {ramp:+.1f} us/s")

    if data["rcin_t"]:
        c4 = data["rcin_c4"]
        c3 = data["rcin_c3"]
        print(f"\n{sep}")
        print("RC INPUT  (RCIN)")
        print(sep)
        print(f"  Samples         : {len(c4)}")
        print(f"  CH4 (yaw)  range: {min(c4)} .. {max(c4)} us  (neutral=1500)")
        print(f"  CH4 mean        : {sum(c4)/len(c4):.0f} us")
        print(f"  CH3 (coll) range: {min(c3)} .. {max(c3)} us")
        yaw_cmd_mean = (sum(c4)/len(c4) - 1500.0) / 500.0
        print(f"  CH4 normalized  : {yaw_cmd_mean:+.3f}  (-1=full left, 0=centre, +1=full right)")
        if abs(yaw_cmd_mean) > 0.05:
            print(f"  [DIAGNOSIS] CH4 is NOT centred -- yaw rate setpoint is non-zero!")
            print(f"              This is likely the root cause of SERVO4 wind-up.")
            print(f"              In ACRO mode yaw rate cmd = CH4 deviation * ACRO_YAW_RATE.")
        else:
            print(f"  [OK] CH4 is centred -- yaw rate setpoint is near zero.")

    if data["imu0_t"]:
        gz = data["imu0_gz"]
        mean_gz = sum(gz) / len(gz)
        print(f"\n{sep}")
        print("IMU0 GYRO Z  (raw, deg/s)")
        print(sep)
        print(f"  Samples         : {len(gz)}")
        print(f"  Mean            : {mean_gz:+.4f} deg/s")
        print(f"  Std dev         : {(sum((g - mean_gz)**2 for g in gz)/len(gz))**0.5:.4f} deg/s")
        print(f"  Range           : {min(gz):.4f} .. {max(gz):.4f} deg/s")
        print()
        if abs(mean_gz) > 0.05:
            print("  [DIAGNOSIS] Gyro Z mean is non-zero -> gyro bias not compensated.")
            print("              The yaw PID integrates this as persistent error.")
            print("              Fix: allow EKF time to learn gyro bias before arming,")
            print("              or set INS_GYROFFS_Z to compensate the bias.")
        else:
            print("  [DIAGNOSIS] Gyro Z mean is near zero -- bias is not the primary cause.")
            print("              Check H_YAW_TRIM feedforward and PIDR.FF term.")
    print(sep)


def _plot(data: dict[str, list], path: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("  matplotlib not available -- skipping plot")
        return

    fig = plt.figure(figsize=(14, 13))
    fig.suptitle(f"Hardware log: {os.path.basename(path)}", fontsize=12)
    gs = gridspec.GridSpec(5, 1, hspace=0.50)

    # -- Panel 1: yaw rate ----------------------------------------------------
    ax1 = fig.add_subplot(gs[0])
    if data["rate_t"]:
        t0 = data["rate_t"][0]
        t  = [x - t0 for x in data["rate_t"]]
        ax1.plot(t, data["rate_ydes"], "b--", lw=0.8, label="YDes (setpoint)")
        ax1.plot(t, data["rate_y"],    "r",   lw=1.0, label="Y (actual)")
    if data["pidr_t"]:
        if not data["rate_t"]:
            t0 = data["pidr_t"][0]
        t = [x - t0 for x in data["pidr_t"]]
        ax1.plot(t, data["pidr_err"], "g", lw=0.8, alpha=0.7, label="PIDR Err")
    ax1.set_ylabel("deg/s")
    ax1.set_title("Yaw rate: setpoint vs actual vs error")
    ax1.legend(fontsize=8)
    ax1.axhline(0, color="k", lw=0.5)
    ax1.grid(True, alpha=0.3)

    # -- Panel 2: PID terms ---------------------------------------------------
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    if data["pidr_t"]:
        t = [x - t0 for x in data["pidr_t"]]
        ax2.plot(t, data["pidr_p"],  label="P",  lw=0.9)
        ax2.plot(t, data["pidr_i"],  label="I",  lw=1.2)
        ax2.plot(t, data["pidr_d"],  label="D",  lw=0.7, alpha=0.7)
        ax2.plot(t, data["pidr_ff"], label="FF", lw=0.8, ls="--")
    ax2.set_ylabel("controller output")
    ax2.set_title("Yaw PID terms (PIDR)")
    ax2.legend(fontsize=8, ncol=4)
    ax2.axhline(0, color="k", lw=0.5)
    ax2.grid(True, alpha=0.3)

    # -- Panel 3: RC inputs (CH4 yaw stick) ------------------------------------
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    if data["rcin_t"]:
        t = [x - t0 for x in data["rcin_t"]]
        ax3.plot(t, data["rcin_c4"], "m", lw=1.0, label="CH4 yaw (us)")
        ax3.plot(t, data["rcin_c3"], "c--", lw=0.8, alpha=0.7, label="CH3 coll (us)")
    ax3.axhline(1500, color="k", lw=0.8, ls="--", label="neutral 1500")
    ax3.axhline(1000, color="k", lw=0.4, ls=":")
    ax3.axhline(2000, color="k", lw=0.4, ls=":")
    ax3.set_ylabel("RC input (us)")
    ax3.set_title("RC inputs: CH4 yaw stick + CH3 collective")
    ax3.legend(fontsize=8, ncol=3)
    ax3.grid(True, alpha=0.3)

    # -- Panel 4: SERVO4 PWM + RSC throttle ------------------------------------
    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    if data["rcou_t"]:
        t = [x - t0 for x in data["rcou_t"]]
        ax4.plot(t, data["rcou_c4"], "b", lw=1.2, label="SERVO4 PWM (us)")
    ax4b = ax4.twinx()
    if data["hrsc_t"]:
        t = [x - t0 for x in data["hrsc_t"]]
        ax4b.plot(t, data["hrsc_throt"], "r--", lw=0.8, label="RSC throttle %")
        ax4b.set_ylabel("RSC throttle %", color="r")
    ax4.set_ylabel("PWM (us)")
    ax4.set_title("SERVO4 output + RSC throttle")
    ax4.axhline(800,  color="k", lw=0.5, ls=":")
    ax4.axhline(2000, color="k", lw=0.5, ls=":")
    ax4.legend(loc="upper left",  fontsize=8)
    ax4b.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)

    # -- Panel 5: raw gyro Z ---------------------------------------------------
    ax4 = fig.add_subplot(gs[4], sharex=ax1)
    if data["imu0_t"]:
        t = [x - t0 for x in data["imu0_t"]]
        ax4.plot(t, data["imu0_gz"], lw=0.6, alpha=0.8, label="IMU0 GyrZ")
        mean0 = sum(data["imu0_gz"]) / len(data["imu0_gz"])
        ax4.axhline(mean0, color="C0", ls="--", lw=1.0, label=f"IMU0 mean={mean0:+.3f}")
    if data["imu1_t"]:
        t = [x - t0 for x in data["imu1_t"]]
        ax4.plot(t, data["imu1_gz"], lw=0.6, alpha=0.8, label="IMU1 GyrZ")
        mean1 = sum(data["imu1_gz"]) / len(data["imu1_gz"])
        ax4.axhline(mean1, color="C1", ls="--", lw=1.0, label=f"IMU1 mean={mean1:+.3f}")
    ax4.axhline(0, color="k", lw=0.5)
    ax4.set_ylabel("deg/s")
    ax4.set_xlabel("time (s)")
    ax4.set_title("Raw gyro Z (both IMUs)")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    out = path.replace(".BIN", "_analysis.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"  Plot saved -> {out}")


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else _find_latest_log()
    print(f"Analysing: {path}")
    data = _load_log(path)
    _print_summary(data)
    _plot(data, path)


if __name__ == "__main__":
    main()
