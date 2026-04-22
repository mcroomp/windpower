"""
flight_log.py -- Unified flight log data structure.

Loads all per-test log files into a single FlightLog for analysis.
Used by analyse_run.py to produce bucketed flight reports.
"""

from __future__ import annotations

import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

_SIM_DIR = Path(__file__).resolve().parents[1]
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from telemetry_csv import read_csv
from mavlink_log import iter_messages as _iter_mavlink  # noqa: F401 (re-exported for callers)
from ekf_flags import (
    MAV_MODE_ARMED, ARDU_MODES,
    decode_flags, flag_diff, has_warn,
)


# ---------------------------------------------------------------------------
# Time alignment
# ---------------------------------------------------------------------------

def to_sim(t_wall: float, t0_wall: float, t0_boot_s: float) -> float:
    """Convert MAVLink wall time to sim time (boot seconds)."""
    return (t_wall - t0_wall) + t0_boot_s


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class FlightEvent:
    """Single timestamped flight event extracted from any log source."""
    t_sim: float
    category: str  # "EKF" | "GPS" | "ARM" | "MODE" | "PHASE" | "CRASH" | "RAWES" | "INFO"
    severity: str  # "OK" | "WARN" | "ERROR" | "INFO"
    text: str


@dataclass
class Bucket:
    """Aggregated data for one time window of a flight."""
    t_start: float
    t_end: float
    bucket_s: float
    phase: str
    is_kinematic: bool
    n_rows: int
    # Physics aggregates (from telemetry CSV)
    alt_mean: float = 0.0
    alt_min: float = 0.0
    alt_max: float = 0.0
    orbit_r_mean: float = 0.0
    tether_tension_mean: float = 0.0
    tether_tension_max: float = 0.0
    tether_slack_frac: float = 0.0
    collective_rad_mean: float = 0.0
    aero_T_mean: float = 0.0
    vel_z_mean: float = 0.0
    heading_gap_deg_mean: float = 0.0
    heading_gap_deg_max: float = 0.0
    omega_rotor_mean: float = 0.0
    # Actual attitude (from ATTITUDE MAVLink)
    att_roll_deg: Optional[float] = None
    att_pitch_deg: Optional[float] = None
    att_yaw_deg: Optional[float] = None
    att_rollspeed_peak: Optional[float] = None   # deg/s
    att_pitchspeed_peak: Optional[float] = None  # deg/s
    att_yawspeed_peak: Optional[float] = None    # deg/s
    # Target attitude (from ATTITUDE_TARGET MAVLink)
    tgt_roll_deg: Optional[float] = None
    tgt_pitch_deg: Optional[float] = None
    tgt_thrust: Optional[float] = None
    att_err_deg: Optional[float] = None  # mean geodesic angle between target and actual
    # EKF health (from EKF_STATUS_REPORT)
    ekf_flags: Optional[int] = None
    ekf_pos_var_peak: Optional[float] = None
    ekf_vel_var_peak: Optional[float] = None
    # Dataflash ATT: actual vs desired attitude (from dataflash.BIN ATT log)
    df_roll_deg: Optional[float] = None
    df_pitch_deg: Optional[float] = None
    df_yaw_deg: Optional[float] = None
    df_des_roll_deg: Optional[float] = None
    df_des_pitch_deg: Optional[float] = None
    df_des_yaw_deg: Optional[float] = None
    df_roll_err_deg: Optional[float] = None   # actual - desired (signed)
    df_pitch_err_deg: Optional[float] = None
    # Dataflash SWSH: swashplate servo mix outputs
    df_swsh_col: Optional[float] = None    # collective (%)
    df_swsh_pcyc: Optional[float] = None   # pitch cyclic (%)
    df_swsh_rcyc: Optional[float] = None   # roll cyclic (%)
    # Dataflash PIDR: roll rate PID breakdown (mean per bucket)
    df_pidr_tar: Optional[float] = None    # target roll rate (deg/s)
    df_pidr_act: Optional[float] = None    # actual roll rate (deg/s)
    df_pidr_p:   Optional[float] = None    # P term
    df_pidr_ff:  Optional[float] = None    # FF term
    df_pidr_i:   Optional[float] = None    # I term
    df_pidr_d:   Optional[float] = None    # D term
    # Dataflash PIDP: pitch rate PID breakdown (mean per bucket)
    df_pidp_tar: Optional[float] = None    # target pitch rate (deg/s)
    df_pidp_act: Optional[float] = None    # actual pitch rate (deg/s)
    df_pidp_p:   Optional[float] = None    # P term
    df_pidp_ff:  Optional[float] = None    # FF term
    df_pidp_i:   Optional[float] = None    # I term
    df_pidp_d:   Optional[float] = None    # D term
    # Events occurring in this window
    events: list = field(default_factory=list)


@dataclass
class FlightLog:
    """All per-test log data in one structure."""
    log_dir: Path
    tel_rows: list = field(default_factory=list)  # list[TelRow]
    t0_wall: Optional[float] = None       # wall time of first SYSTEM_TIME msg
    t0_boot_s: Optional[float] = None    # boot seconds of first SYSTEM_TIME msg
    # Run metadata
    run_id_mediator: Optional[int] = None
    run_id_pytest: Optional[int] = None
    sensor_mode: Optional[str] = None
    damp_T: Optional[float] = None
    k_ang: Optional[float] = None
    kin_launch_pos: Optional[tuple] = None  # (x, y, z) NED [m]
    kin_vel: Optional[tuple] = None         # (vx, vy, vz) [m/s]
    sitl_crash_error: Optional[str] = None
    sitl_crash_stack: list = field(default_factory=list)
    # All events sorted by t_sim
    events: list = field(default_factory=list)  # list[FlightEvent]
    # Pre-processed MAVLink data (private)
    _att_rows: list = field(default_factory=list, repr=False)  # {t_sim, roll, pitch, yaw, ...}
    _tgt_rows: list = field(default_factory=list, repr=False)  # {t_sim, roll, pitch, q, thrust}
    _ekf_rows: list = field(default_factory=list, repr=False)  # {t_sim, flags, pos_var, vel_var}
    _mavlink_records: list = field(default_factory=list, repr=False)
    # Pre-processed dataflash BIN data (private); time in boot seconds (TimeUS/1e6)
    _df_att_rows:  list = field(default_factory=list, repr=False)  # ATT:  {t_s, roll, des_roll, pitch, des_pitch, yaw, des_yaw}
    _df_swsh_rows: list = field(default_factory=list, repr=False)  # SWSH: {t_s, col, pcyc, rcyc}
    _df_pidr_rows: list = field(default_factory=list, repr=False)  # PIDR: {t_s, tar, act, err, p, i, d, ff}
    _df_pidp_rows: list = field(default_factory=list, repr=False)  # PIDP: {t_s, tar, act, err, p, i, d, ff}
    _df_mode_rows: list = field(default_factory=list, repr=False)  # MODE: {t_s, mode, mode_name, reason}

    def to_sim_mav(self, t_wall: float) -> float:
        if self.t0_wall is None or self.t0_boot_s is None:
            return 0.0
        return to_sim(t_wall, self.t0_wall, self.t0_boot_s)

    @classmethod
    def load(cls, log_dir: "Path | str") -> "FlightLog":
        """Load all log files from log_dir into a unified FlightLog."""
        fl = cls(log_dir=Path(log_dir))
        fl._load_telemetry()
        fl._load_mavlink()
        fl._load_dataflash()
        fl._load_mediator()
        fl._load_arducopter()
        fl._load_gcs_log()
        fl._load_tel_events()
        fl.events.sort(key=lambda e: e.t_sim)
        return fl

    def _load_telemetry(self) -> None:
        path = self.log_dir / "telemetry.csv"
        if path.exists():
            self.tel_rows = read_csv(path)

    def _load_mavlink(self) -> None:
        path = self.log_dir / "mavlink.jsonl"
        if not path.exists():
            return
        records = []
        with path.open(encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        self._mavlink_records = records

        # Locate SYSTEM_TIME anchor for wall→sim conversion
        for d in records:
            if d.get("mavpackettype") == "SYSTEM_TIME":
                self.t0_wall = d["_t_wall"]
                self.t0_boot_s = d.get("time_boot_ms", 0) / 1000.0
                break
        if self.t0_wall is None or self.t0_boot_s is None:
            return  # cannot time-align MAVLink data

        t0_wall, t0_boot_s = self.t0_wall, self.t0_boot_s
        prev_ekf: Optional[int] = None
        prev_arm: Optional[bool] = None
        prev_mode: Optional[int] = None

        for d in records:
            mt = d.get("mavpackettype", "")
            ts = to_sim(d["_t_wall"], t0_wall, t0_boot_s)

            if mt == "ATTITUDE":
                self._att_rows.append({
                    "t_sim":      ts,
                    "roll":       math.degrees(d.get("roll",  0.0)),
                    "pitch":      math.degrees(d.get("pitch", 0.0)),
                    "yaw":        math.degrees(d.get("yaw",   0.0)),
                    "rollspeed":  abs(math.degrees(d.get("rollspeed",  0.0))),
                    "pitchspeed": abs(math.degrees(d.get("pitchspeed", 0.0))),
                    "yawspeed":   abs(math.degrees(d.get("yawspeed",   0.0))),
                })

            elif mt == "ATTITUDE_TARGET":
                q = d.get("q")
                if q and len(q) == 4:
                    r, p, y = _quat_to_euler(q[0], q[1], q[2], q[3])
                    self._tgt_rows.append({
                        "t_sim":  ts,
                        "roll":   math.degrees(r),
                        "pitch":  math.degrees(p),
                        "yaw":    math.degrees(y),
                        "q":      q,
                        "thrust": d.get("thrust", 0.0),
                    })

            elif mt == "EKF_STATUS_REPORT":
                flags = int(d.get("flags", 0))
                self._ekf_rows.append({
                    "t_sim":   ts,
                    "flags":   flags,
                    "pos_var": d.get("pos_horiz_variance", 0.0),
                    "vel_var": d.get("velocity_variance",  0.0),
                })
                if flags != prev_ekf:
                    sev  = "WARN" if has_warn(flags) else "OK"
                    diff = (flag_diff(prev_ekf, flags) if prev_ekf is not None
                            else f"init [{decode_flags(flags)}]")
                    self.events.append(FlightEvent(ts, "EKF", sev,
                                                   f"flags 0x{flags:04x}: {diff}"))
                    prev_ekf = flags

            elif mt == "STATUSTEXT":
                text = d.get("text", "").rstrip("\x00").strip()
                if text:
                    cat, sev = _categorize_statustext(text)
                    self.events.append(FlightEvent(ts, cat, sev, text))

            elif mt == "HEARTBEAT":
                armed = bool(d.get("base_mode", 0) & MAV_MODE_ARMED)
                mode  = int(d.get("custom_mode", 0))
                if armed != prev_arm:
                    self.events.append(FlightEvent(
                        ts, "ARM", "INFO",
                        "ARMED" if armed else "DISARMED",
                    ))
                    prev_arm = armed
                if mode != prev_mode:
                    mode_name = ARDU_MODES.get(mode, str(mode))
                    self.events.append(FlightEvent(
                        ts, "MODE", "INFO", f"mode -> {mode_name}({mode})",
                    ))
                    prev_mode = mode

    def _load_dataflash(self) -> None:
        """Load ATT and SWSH messages from dataflash.BIN (ArduPilot binary log).

        Uses pymavlink DFReader.  Silently skips if the file is absent or
        pymavlink is unavailable.  Time axis is TimeUS/1e6 (boot seconds).
        """
        path = self.log_dir / "dataflash.BIN"
        if not path.exists():
            return
        try:
            from pymavlink import DFReader
        except ImportError:
            return

        log = DFReader.DFReader_binary(str(path))
        msg = log.recv_msg()
        while msg is not None:
            mt = msg.get_type()
            time_us = getattr(msg, "TimeUS", None)
            if time_us is None:
                msg = log.recv_msg()
                continue
            t_s = time_us / 1e6
            if mt == "ATT":
                self._df_att_rows.append({
                    "t_s":       t_s,
                    "roll":      msg.Roll,
                    "des_roll":  msg.DesRoll,
                    "pitch":     msg.Pitch,
                    "des_pitch": msg.DesPitch,
                    "yaw":       msg.Yaw,
                    "des_yaw":   msg.DesYaw,
                })
            elif mt == "SWSH":
                self._df_swsh_rows.append({
                    "t_s":  t_s,
                    "col":  msg.Col,
                    "pcyc": msg.PCyc,
                    "rcyc": msg.RCyc,
                })
            elif mt == "PIDR":
                self._df_pidr_rows.append({
                    "t_s": t_s,
                    "tar": getattr(msg, "Tar", 0.0),
                    "act": getattr(msg, "Act", 0.0),
                    "err": getattr(msg, "Err", 0.0),
                    "p":   getattr(msg, "P",   0.0),
                    "i":   getattr(msg, "I",   0.0),
                    "d":   getattr(msg, "D",   0.0),
                    "ff":  getattr(msg, "FF",  0.0),
                })
            elif mt == "PIDP":
                self._df_pidp_rows.append({
                    "t_s": t_s,
                    "tar": getattr(msg, "Tar", 0.0),
                    "act": getattr(msg, "Act", 0.0),
                    "err": getattr(msg, "Err", 0.0),
                    "p":   getattr(msg, "P",   0.0),
                    "i":   getattr(msg, "I",   0.0),
                    "d":   getattr(msg, "D",   0.0),
                    "ff":  getattr(msg, "FF",  0.0),
                })
            elif mt == "MODE":
                mode_num  = int(getattr(msg, "Mode", -1))
                mode_name = ARDU_MODES.get(mode_num, str(mode_num))
                reason    = int(getattr(msg, "Rsn", -1))
                self._df_mode_rows.append({
                    "t_s":       t_s,
                    "mode":      mode_num,
                    "mode_name": mode_name,
                    "reason":    reason,
                })
            msg = log.recv_msg()

    def _load_mediator(self) -> None:
        path = self.log_dir / "mediator.log"
        if not path.exists():
            return

        RE_RUN_ID = re.compile(r"RUN_ID=(\d+)")
        RE_SENSOR = re.compile(r"Sensor mode:\s+(\S+)")
        RE_DAMP   = re.compile(r"Startup damping:.*?T=([\d.]+)s.*?k_ang=([\d.]+)")
        RE_KIN    = re.compile(
            r"Kinematic startup:.*?T=([\d.]+)s\s+"
            r"launch_pos=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]\s+"
            r"vel=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
        )

        for line in path.read_text(errors="replace").splitlines():
            if m := RE_RUN_ID.search(line):
                self.run_id_mediator = int(m.group(1))
            if m := RE_SENSOR.search(line):
                self.sensor_mode = m.group(1)
            if m := RE_DAMP.search(line):
                self.damp_T = float(m.group(1))
                self.k_ang  = float(m.group(2))
            if m := RE_KIN.search(line):
                self.kin_launch_pos = (
                    float(m.group(2)), float(m.group(3)), float(m.group(4)),
                )
                self.kin_vel = (
                    float(m.group(5)), float(m.group(6)), float(m.group(7)),
                )

        # Fallback: events.jsonl for run_id
        if self.run_id_mediator is None:
            ev_path = path.parent / "events.jsonl"
            if ev_path.exists():
                for line in ev_path.read_text(errors="replace").splitlines():
                    try:
                        ev = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if ev.get("event") == "startup" and "run_id" in ev:
                        self.run_id_mediator = int(ev["run_id"])
                        break

    def _load_arducopter(self) -> None:
        path = self.log_dir / "arducopter.log"
        if not path.exists():
            return
        lines = path.read_text(errors="replace").splitlines()
        for i, ln in enumerate(lines):
            if "ERROR:" in ln and any(k in ln.lower()
                                      for k in ("exception", "abort", "signal")):
                self.sitl_crash_error = ln.strip()
                in_dump = False
                for j in range(i + 1, len(lines)):
                    if "begin dumpstack" in lines[j]:
                        in_dump = True
                        continue
                    if "end dumpstack" in lines[j]:
                        break
                    if in_dump and lines[j].strip().startswith("#"):
                        self.sitl_crash_stack.append(lines[j].strip())
                t_crash = self.tel_rows[-1].t_sim if self.tel_rows else 0.0
                self.events.append(FlightEvent(
                    t_crash, "CRASH", "ERROR", self.sitl_crash_error,
                ))
                break

    def _load_gcs_log(self) -> None:
        """Extract pytest RUN_ID from gcs.log."""
        path = self.log_dir / "gcs.log"
        if not path.exists():
            return
        RE_RUNID = re.compile(r"RUN_ID=(\d+)")
        for line in path.read_text(errors="replace").splitlines():
            if m := RE_RUNID.search(line):
                self.run_id_pytest = int(m.group(1))
                break

    def _load_tel_events(self) -> None:
        """Add FlightEvents from telemetry CSV note column."""
        for row in self.tel_rows:
            if row.note:
                self.events.append(FlightEvent(row.t_sim, "PHASE", "INFO", row.note))

    def buckets(self, bucket_s: float = 1.0) -> list:
        """Aggregate all data into parameterized time buckets.

        Returns list[Bucket] covering the full telemetry time range, or
        the dataflash ATT time range when no telemetry CSV is present.
        Each bucket contains physics aggregates, MAVLink attitude/EKF data,
        dataflash ATT/SWSH data, and FlightEvents in that window.

        Args:
            bucket_s: bucket width in simulation seconds (default 1.0).
                      Use larger values (e.g. 5.0, 10.0) for a high-level
                      overview; use 1.0 for fine-grained debugging.
        """
        if self.tel_rows:
            t_start = self.tel_rows[0].t_sim
            t_end   = self.tel_rows[-1].t_sim
        elif self._df_att_rows:
            t_start = self._df_att_rows[0]["t_s"]
            t_end   = self._df_att_rows[-1]["t_s"]
        else:
            return []
        n_bkts  = max(1, math.ceil((t_end - t_start) / bucket_s))

        result: list[Bucket] = []
        for bi in range(n_bkts):
            bs = t_start + bi * bucket_s
            be = t_start + (bi + 1) * bucket_s

            brows = [r for r in self.tel_rows if bs <= r.t_sim < be]

            if brows:
                is_kin = any(r.damp_alpha > 0 for r in brows)
                if is_kin:
                    phase = "kinematic"
                else:
                    phases: dict = {}
                    for r in brows:
                        p = r.phase if r.phase else "steady"
                        phases[p] = phases.get(p, 0) + 1
                    phase = max(phases, key=phases.__getitem__)
                n = len(brows)
                alts  = [-r.pos_z           for r in brows]
                orbits = [math.hypot(r.pos_x, r.pos_y) for r in brows]
                tens   = [r.tether_tension  for r in brows]
                slacks = [r.tether_slack    for r in brows]
                cols   = [r.collective_rad  for r in brows]
                aeros  = [r.aero_T          for r in brows]
                velzs  = [r.vel_z           for r in brows]
                hdgaps = [abs(r.heading_gap_deg) for r in brows]
                omegas = [r.omega_rotor     for r in brows]
                b = Bucket(
                    t_start=bs, t_end=be, bucket_s=bucket_s,
                    phase=phase, is_kinematic=is_kin, n_rows=n,
                    alt_mean=sum(alts) / n, alt_min=min(alts), alt_max=max(alts),
                    orbit_r_mean=sum(orbits) / n,
                    tether_tension_mean=sum(tens) / n, tether_tension_max=max(tens),
                    tether_slack_frac=sum(1 for s in slacks if s > 0) / n,
                    collective_rad_mean=sum(cols) / n,
                    aero_T_mean=sum(aeros) / n,
                    vel_z_mean=sum(velzs) / n,
                    heading_gap_deg_mean=sum(hdgaps) / n, heading_gap_deg_max=max(hdgaps),
                    omega_rotor_mean=sum(omegas) / n,
                )
            else:
                b = Bucket(
                    t_start=bs, t_end=be, bucket_s=bucket_s,
                    phase="", is_kinematic=False, n_rows=0,
                )

            # ATTITUDE rows in window
            att_in = [r for r in self._att_rows if bs <= r["t_sim"] < be]
            if att_in:
                n_a = len(att_in)
                b.att_roll_deg       = sum(r["roll"]  for r in att_in) / n_a
                b.att_pitch_deg      = sum(r["pitch"] for r in att_in) / n_a
                b.att_yaw_deg        = sum(r["yaw"]   for r in att_in) / n_a
                b.att_rollspeed_peak  = max(r["rollspeed"]  for r in att_in)
                b.att_pitchspeed_peak = max(r["pitchspeed"] for r in att_in)
                b.att_yawspeed_peak   = max(r["yawspeed"]   for r in att_in)

            # ATTITUDE_TARGET rows in window
            tgt_in = [r for r in self._tgt_rows if bs <= r["t_sim"] < be]
            if tgt_in:
                n_t = len(tgt_in)
                b.tgt_roll_deg  = sum(r["roll"]   for r in tgt_in) / n_t
                b.tgt_pitch_deg = sum(r["pitch"]  for r in tgt_in) / n_t
                b.tgt_thrust    = sum(r["thrust"] for r in tgt_in) / n_t
                if att_in:
                    errs = []
                    for tr in tgt_in:
                        # Find nearest ATTITUDE message by time
                        ar = min(att_in, key=lambda a: abs(a["t_sim"] - tr["t_sim"]))
                        qa = _euler_to_quat(
                            math.radians(ar["roll"]),
                            math.radians(ar["pitch"]),
                            math.radians(ar["yaw"]),
                        )
                        tq = tr["q"]
                        errs.append(_quat_angle_deg(qa, (tq[0], tq[1], tq[2], tq[3])))
                    b.att_err_deg = sum(errs) / len(errs)

            # EKF_STATUS_REPORT rows in window
            ekf_in = [r for r in self._ekf_rows if bs <= r["t_sim"] < be]
            if ekf_in:
                b.ekf_flags        = ekf_in[-1]["flags"]
                b.ekf_pos_var_peak = max(r["pos_var"] for r in ekf_in)
                b.ekf_vel_var_peak = max(r["vel_var"] for r in ekf_in)

            # Dataflash ATT rows in window (boot-second time axis)
            df_att_in = [r for r in self._df_att_rows if bs <= r["t_s"] < be]
            if df_att_in:
                n_da = len(df_att_in)
                b.df_roll_deg      = sum(r["roll"]      for r in df_att_in) / n_da
                b.df_pitch_deg     = sum(r["pitch"]     for r in df_att_in) / n_da
                b.df_yaw_deg       = sum(r["yaw"]       for r in df_att_in) / n_da
                b.df_des_roll_deg  = sum(r["des_roll"]  for r in df_att_in) / n_da
                b.df_des_pitch_deg = sum(r["des_pitch"] for r in df_att_in) / n_da
                b.df_des_yaw_deg   = sum(r["des_yaw"]   for r in df_att_in) / n_da
                b.df_roll_err_deg  = b.df_roll_deg - b.df_des_roll_deg
                b.df_pitch_err_deg = b.df_pitch_deg - b.df_des_pitch_deg

            # Dataflash SWSH rows in window
            df_swsh_in = [r for r in self._df_swsh_rows if bs <= r["t_s"] < be]
            if df_swsh_in:
                n_ds = len(df_swsh_in)
                b.df_swsh_col  = sum(r["col"]  for r in df_swsh_in) / n_ds
                b.df_swsh_pcyc = sum(r["pcyc"] for r in df_swsh_in) / n_ds
                b.df_swsh_rcyc = sum(r["rcyc"] for r in df_swsh_in) / n_ds

            # Dataflash PIDR/PIDP rows in window
            df_pidr_in = [r for r in self._df_pidr_rows if bs <= r["t_s"] < be]
            if df_pidr_in:
                n = len(df_pidr_in)
                b.df_pidr_tar = sum(r["tar"] for r in df_pidr_in) / n
                b.df_pidr_act = sum(r["act"] for r in df_pidr_in) / n
                b.df_pidr_p   = sum(r["p"]   for r in df_pidr_in) / n
                b.df_pidr_ff  = sum(r["ff"]  for r in df_pidr_in) / n
                b.df_pidr_i   = sum(r["i"]   for r in df_pidr_in) / n
                b.df_pidr_d   = sum(r["d"]   for r in df_pidr_in) / n

            df_pidp_in = [r for r in self._df_pidp_rows if bs <= r["t_s"] < be]
            if df_pidp_in:
                n = len(df_pidp_in)
                b.df_pidp_tar = sum(r["tar"] for r in df_pidp_in) / n
                b.df_pidp_act = sum(r["act"] for r in df_pidp_in) / n
                b.df_pidp_p   = sum(r["p"]   for r in df_pidp_in) / n
                b.df_pidp_ff  = sum(r["ff"]  for r in df_pidp_in) / n
                b.df_pidp_i   = sum(r["i"]   for r in df_pidp_in) / n
                b.df_pidp_d   = sum(r["d"]   for r in df_pidp_in) / n

            # Events in window
            b.events = [e for e in self.events if bs <= e.t_sim < be]

            result.append(b)

        return result


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _categorize_statustext(text: str) -> tuple:
    """Return (category, severity) for a STATUSTEXT string."""
    tl = text.lower()
    if "rawes" in tl:
        cat = "RAWES"
    elif any(k in tl for k in ("gps", "origin set", "relposned", "fix")):
        cat = "GPS"
    elif any(k in tl for k in ("ekf", "yaw", "compass", "ahrs", "tilt align")):
        cat = "EKF"
    elif any(k in tl for k in ("armed", "disarm")):
        cat = "ARM"
    else:
        cat = "INFO"

    if any(k in tl for k in ("emergency", "abort", "fail", "crash")):
        sev = "ERROR"
    elif any(k in tl for k in ("glitch", "reset", "error", "lost")):
        sev = "WARN"
    elif any(k in tl for k in ("using gps", "origin set", "aligned",
                                "healthy", "initialised")):
        sev = "OK"
    else:
        sev = "INFO"
    return cat, sev


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple:
    """ZYX Euler angles (rad) to unit quaternion [w, x, y, z]."""
    cr, sr = math.cos(roll  * 0.5), math.sin(roll  * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw   * 0.5), math.sin(yaw   * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _quat_to_euler(w: float, x: float, y: float, z: float) -> tuple:
    """Unit quaternion [w, x, y, z] to ZYX Euler (roll, pitch, yaw) in radians."""
    roll  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp    = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _quat_angle_deg(q1: tuple, q2: tuple) -> float:
    """Geodesic angle in degrees between two unit quaternions."""
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    return math.degrees(2.0 * math.acos(min(1.0, dot)))
