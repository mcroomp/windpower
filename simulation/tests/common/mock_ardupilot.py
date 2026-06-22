"""MockArdupilot adapter used by simtests and diagnostics.

Provides a common interface for driving either:
- Lua-backed control code (RawesLua harness), or
- Python-equivalent local AP behavior.
"""

from __future__ import annotations

import os
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from arduloop import GuidedAttitudeController, HeliParams
from controller import (AccelVibrationDamper,
                        compute_bz_altitude_hold, compute_rate_cmd,
                        compute_rate_cmd_sqrt, slerp_body_z)
from landing_planner import LandingCommand
from physics_core import HubObservation
from pumping_planner import TensionCommand
from telemetry_csv import TelRow, write_csv


def _tel_every_from_env(dt: float, default_hz: float = 20.0) -> int:
    hz = float(os.environ.get("RAWES_TEL_HZ", default_hz))
    return max(1, round(1.0 / (hz * dt)))


def _feed_obs(sim, obs, accel_body: "np.ndarray | None" = None) -> None:
    """Write HubObservation-like state into a RawesLua mock."""
    sim.R = obs.R
    sim.pos_ned = obs.pos.tolist()
    sim.vel_ned = obs.vel.tolist()
    sim.gyro = obs.gyro.tolist()
    if accel_body is not None:
        sim.accel = accel_body.tolist()


class _MockArdupilotBase:
    """Common adapter utilities shared by lua/python backends."""

    COL_MIN = -0.28
    COL_MAX = 0.10

    def __init__(self, *, wind: "np.ndarray", dt: float, initial_col_rad: float = 0.0) -> None:
        self.col_rad = float(initial_col_rad)
        self.roll_sp = 0.0
        self.pitch_sp = 0.0
        self._wind = wind
        self._tel_every = _tel_every_from_env(dt)
        self.tel_fn: "Callable[..., dict] | None" = None
        self._telemetry: list = []
        self._log_step = 0
        self._guided_ctrl: "GuidedAttitudeController | None" = None

    def enable_guided(self, heli_params: "HeliParams | None" = None) -> None:
        self._guided_ctrl = GuidedAttitudeController(heli_params or HeliParams())

    def step_physics(self, runner, dt: float, *, rest_length: "float | None" = None) -> dict:
        if self._guided_ctrl is None:
            raise RuntimeError("Call enable_guided() before step_physics()")
        obs = runner.observe()
        q_body = Rotation.from_matrix(obs.R).as_quat()
        gyro = obs.gyro
        col_norm = (self.col_rad - self.COL_MIN) / (self.COL_MAX - self.COL_MIN) * 2.0 - 1.0
        heli_out = self._guided_ctrl.update(
            q_body,
            gyro,
            dt,
            collective_norm=col_norm,
            sim_time=runner.t_sim,
            pos_z_up_m=float(-obs.pos[2]),
            vel_z_up_mps=float(-obs.vel[2]),
        )
        if heli_out.collective_norm_cmd is not None and abs(float(self._guided_ctrl.climbrate_ms)) > 1e-6:
            c_norm = float(np.clip(heli_out.collective_norm_cmd, -1.0, 1.0))
            self.col_rad = self.COL_MIN + 0.5 * (c_norm + 1.0) * (self.COL_MAX - self.COL_MIN)
        return runner.step_guided(dt, self.col_rad, heli_out, rest_length=rest_length)

    def log(self, runner, sr: dict) -> None:
        if self.tel_fn is None or self._tel_every is None:
            return
        if self._log_step % self._tel_every == 0:
            self._telemetry.append(
                TelRow.from_physics(runner, sr, self.col_rad, self._wind, **self.tel_fn(runner, sr))
            )
        self._log_step += 1

    def write_telemetry(self, path) -> None:
        if self._telemetry:
            write_csv(self._telemetry, path)

    @property
    def telemetry(self) -> list:
        return self._telemetry


class _LuaBackend(_MockArdupilotBase):
    def __init__(self, sim, *, initial_col_rad: float = 0.0, wind: "np.ndarray", dt: float) -> None:
        super().__init__(wind=wind, dt=dt, initial_col_rad=initial_col_rad)
        self._sim = sim
        hz = round(1.0 / dt)
        self.enable_guided(HeliParams(loop_rate_hz=hz))
        assert self._guided_ctrl is not None
        self._ctrl = self._guided_ctrl

    def tick(self, t_sim: float, runner, *, inject=None, accel_ned: "np.ndarray | None" = None) -> None:
        self._sim._mock.millis_val = int(t_sim * 1000)
        obs = runner.observe()
        accel_body = obs.R.T @ np.asarray(accel_ned, dtype=float) if accel_ned is not None else None
        _feed_obs(self._sim, obs, accel_body=accel_body)
        if inject is not None:
            inject(self._sim, runner)
        self._sim._update_fn()

        gt_throttle = self._sim._mock.guided_throttle
        if gt_throttle is not None:
            self.col_rad = self.COL_MIN + float(gt_throttle) * (self.COL_MAX - self.COL_MIN)
        else:
            ch3 = self._sim.ch_out[3]
            if ch3 is not None:
                self.col_rad = self.COL_MIN + (ch3 - 1000) / 1000.0 * (self.COL_MAX - self.COL_MIN)

        gt_rate = self._sim._mock.guided_rate_target
        if gt_rate is not None:
            throttle = (
                float(gt_throttle)
                if gt_throttle is not None
                else max(0.0, min(1.0, (self.col_rad - self.COL_MIN) / (self.COL_MAX - self.COL_MIN)))
            )
            self._ctrl.set_target_rate_and_throttle(
                float(gt_rate.roll_rate),
                float(gt_rate.pitch_rate),
                float(gt_rate.yaw_rate),
                throttle,
                sim_time=t_sim,
            )

        gt = self._sim._mock.guided_target
        if gt is not None:
            throttle = (
                float(gt_throttle)
                if gt_throttle is not None
                else max(0.0, min(1.0, (self.col_rad - self.COL_MIN) / (self.COL_MAX - self.COL_MIN)))
            )
            self._ctrl.set_target_angle_and_rate_and_throttle(
                float(gt.roll_deg),
                float(gt.pitch_deg),
                float(gt.yaw_deg),
                0.0,
                0.0,
                0.0,
                throttle,
                sim_time=t_sim,
            )

    def step(self, runner, dt: float, *, rest_length: "float | None" = None) -> dict:
        obs = runner.observe()
        if not self._ctrl._target_set:
            self._ctrl.set_target_rotation(obs.R, sim_time=runner.t_sim)
        return self.step_physics(runner, dt, rest_length=rest_length)


class _PumpingPythonMode:
    """Python-equivalent pumping behavior for MockArdupilot."""

    CMD_TIMEOUT_S: float = 0.5
    FEASIBILITY_WINDOW_S: float = 1.0

    COL_MIN_RAD: float = -0.28
    COL_MAX_RAD: float = 0.10

    K_VIB:       float = 0.008
    VIB_HP_HZ:   float = 1.5
    VIB_VEL_TAU: float = 0.5
    VIB_COL_MAX: float = 0.04

    KP_ALT: float = 0.010
    KI_ALT: float = 0.001
    KD_VZ:  float = 0.040
    RATE_ACCEL_MAX_RADSS: float = 4.0

    def __init__(
        self,
        ic_pos         : np.ndarray,
        mass_kg        : float,
        slew_rate_rad_s: float,
        warm_coll_rad  : float,
        tension_ic     : float,
        cmd_timeout_s  : float = CMD_TIMEOUT_S,
        coll_min_rad   : float = COL_MIN_RAD,
        coll_max_rad   : float = COL_MAX_RAD,
        kp_outer       : float = 2.5,
        kp_alt         : float = KP_ALT,
        ki_alt         : float = KI_ALT,
        kd_vz          : float = KD_VZ,
        rate_accel_max_radss: float = RATE_ACCEL_MAX_RADSS,
        az_ref_rad     : "float | None" = None,
        events         = None,
        k_vib          : float = K_VIB,
        vib_hp_hz      : float = VIB_HP_HZ,
        vib_vel_tau_s  : float = VIB_VEL_TAU,
        vib_col_max    : float = VIB_COL_MAX,
    ) -> None:
        self._mass_kg    = float(mass_kg)
        self._timeout    = float(cmd_timeout_s)
        self._slew       = float(slew_rate_rad_s)
        self._kp_outer   = float(kp_outer)
        self._events     = events
        self._az_ref_rad = None if az_ref_rad is None else float(az_ref_rad)

        tlen     = float(np.linalg.norm(ic_pos))
        self._el = float(np.arcsin(max(-1.0, min(1.0, float(-ic_pos[2]) / max(tlen, 0.1)))))

        self._col_min = float(coll_min_rad)
        self._col_max = float(coll_max_rad)
        self._tension_for_bz = float(tension_ic)
        self._kp_alt = float(kp_alt)
        self._ki_alt = float(ki_alt)
        self._kd_vz = float(kd_vz)
        self._rate_accel_max = float(rate_accel_max_radss)

        self._col_trim    = float(warm_coll_rad)
        self._C_held      = float(warm_coll_rad)
        self._alt_i       = 0.0
        self._target_alt  = float(-ic_pos[2])
        self._cmd_age     = 0.0
        self._comms_ok    = True
        self._t_sim       = 0.0
        self._pos_ned     = np.asarray(ic_pos, dtype=float)

        self._vib_damper = (
            AccelVibrationDamper(
                k_vib=float(k_vib),
                hp_freq_hz=float(vib_hp_hz),
                vel_tau_s=float(vib_vel_tau_s),
                col_damp_max=float(vib_col_max),
            ) if k_vib != 0.0 else None
        )
        self._last_vib_corr = 0.0
        self._bz_goal = None
        self._last_roll_sp = 0.0
        self._last_pitch_sp = 0.0
        self._tension_ic = float(tension_ic)

    def receive_command(self, cmd: TensionCommand, dt_cmd: float) -> None:  # noqa: ARG002
        new_alt = float(cmd.alt_m)

        if self._events is not None:
            pos_ned = self._pos_ned
            tlen    = float(np.linalg.norm(pos_ned))
            hub_alt = float(-pos_ned[2])
            phase   = cmd.phase

            if new_alt > tlen - 1e-3:
                self._events.record(
                    "ap_impossible_alt", self._t_sim, phase, hub_alt,
                    target_alt=new_alt, tlen=tlen,
                )
            else:
                cmd_el   = float(np.arcsin(max(-1.0, min(1.0, new_alt / max(tlen, 0.1)))))
                el_gap   = abs(cmd_el - self._el)
                max_slew = self._slew * self.FEASIBILITY_WINDOW_S
                if el_gap > max_slew:
                    hub_el = float(np.arcsin(max(-1.0, min(1.0, hub_alt / max(tlen, 0.1)))))
                    self._events.record(
                        "ap_unreachable_alt", self._t_sim, phase, hub_alt,
                        target_alt=new_alt, ap_el_deg=float(np.degrees(self._el)),
                        cmd_el_deg=float(np.degrees(cmd_el)),
                        el_gap_deg=float(np.degrees(el_gap)),
                        max_slew_deg=float(np.degrees(max_slew)),
                        hub_el_deg=float(np.degrees(hub_el)),
                    )

        self._target_alt = new_alt
        self._cmd_age = 0.0
        self._comms_ok = True

        self._tension_for_bz = float(cmd.tension_target_n)

    def step(
        self,
        obs      : HubObservation,
        dt       : float,
        *,
        accel_ned: "np.ndarray | None" = None,
    ) -> "tuple[float, float, float]":
        self._pos_ned  = obs.pos
        self._cmd_age += dt
        self._t_sim   += dt
        if self._comms_ok and self._cmd_age > self._timeout:
            self._comms_ok = False

        tlen      = float(np.linalg.norm(obs.pos))
        target_el = float(np.arcsin(max(-1.0, min(1.0, self._target_alt / max(tlen, 0.1)))))
        delta     = float(np.clip(target_el - self._el, -self._slew * dt, self._slew * dt))
        self._el += delta

        bz_goal = compute_bz_altitude_hold(
            obs.pos, self._el, self._tension_for_bz, self._mass_kg,
            az_ref_rad=self._az_ref_rad,
        )
        R       = obs.R
        bz_now  = R[:, 2]
        rate_sp = compute_rate_cmd_sqrt(
            bz_now, bz_goal, R,
            kp=self._kp_outer,
            accel_max=self._rate_accel_max,
            dt=dt,
            kd=0.0,
        )
        self._bz_goal = bz_goal
        self._last_roll_sp = float(rate_sp[0])
        self._last_pitch_sp = float(rate_sp[1])

        alt_m = float(-obs.pos[2])
        vz_up = float(-obs.vel[2])
        alt_err = self._target_alt - alt_m
        self._alt_i = float(np.clip(
            self._alt_i + self._ki_alt * alt_err * dt,
            self._col_min - self._col_trim,
            self._col_max - self._col_trim,
        ))
        col_out = float(np.clip(
            self._col_trim + self._kp_alt * alt_err - self._kd_vz * vz_up + self._alt_i,
            self._col_min,
            self._col_max,
        ))
        self._last_vib_corr = 0.0
        if accel_ned is not None and self._vib_damper is not None:
            accel_body_z = float((R.T @ np.asarray(accel_ned, dtype=float))[2])
            self._last_vib_corr = self._vib_damper.step(accel_body_z, dt)
            col_out = float(np.clip(col_out + self._last_vib_corr, self._col_min, self._col_max))
        self._C_held = col_out

        return col_out, float(rate_sp[0]), float(rate_sp[1])

    @property
    def comms_ok(self) -> bool:
        return self._comms_ok

    @property
    def tension_feedforward_n(self) -> float:
        return self._tension_for_bz

    @property
    def elevation_rad(self) -> float:
        return self._el

    def log_fields(self) -> dict:
        bz = self._bz_goal
        return dict(
            tension_feedforward_n        = self._tension_for_bz,
            tension_ic_n                 = self._tension_ic,
            elevation_rad                = self.elevation_rad,
            comms_ok                     = self.comms_ok,
            collective_from_alt_ctrl     = self._C_held,
            gnd_alt_cmd_m                = self._target_alt,
            alt_pid_integral             = self._alt_i,
            vib_corr                     = self._last_vib_corr,
            roll_sp_rads                 = self._last_roll_sp,
            pitch_sp_rads                = self._last_pitch_sp,
            body_z_eq                    = bz.tolist() if bz is not None else [0.0, 0.0, 0.0],
        )


class _LandingPythonMode:
    """Python-equivalent landing behavior for MockArdupilot."""

    COL_MIN_RAD: float = -0.28
    COL_MAX_RAD: float = 0.10
    KP_VZ: float = 0.05
    KI_VZ: float = 0.005

    def __init__(
        self,
        ic_body_z:       np.ndarray,
        slew_rate_rad_s: float,
        warm_coll_rad:   float,
        kp_vz:           float,
        ki_vz:           float,
        col_min_rad:     float,
        col_max_rad:     float,
        kp_outer:        float,
    ) -> None:
        bz = np.asarray(ic_body_z, dtype=float)
        self._bz_current = bz / np.linalg.norm(bz)
        self._bz_target  = self._bz_current.copy()
        self._slew       = float(slew_rate_rad_s)
        self._C_held     = float(warm_coll_rad)
        self._col_i      = float(warm_coll_rad)
        self._kp_vz      = float(kp_vz)
        self._ki_vz      = float(ki_vz)
        self._col_min    = float(col_min_rad)
        self._col_max    = float(col_max_rad)
        self._kp_outer   = float(kp_outer)
        self._vz_sp      = 0.0
        self._phase      = "reel_in"

    def receive_command(self, cmd: LandingCommand, dt_cmd: float) -> None:  # noqa: ARG002
        prev_phase = self._phase
        self._phase = cmd.phase
        self._bz_target = np.asarray(cmd.body_z_target, dtype=float).copy()
        self._vz_sp = float(cmd.vz_setpoint_ms)
        if cmd.phase != prev_phase:
            self._col_i = float(np.clip(cmd.col_cruise_rad, self._col_min, self._col_max))

    def step(
        self,
        obs: HubObservation,
        dt:  float,
        *,
        accel_ned: "np.ndarray | None" = None,  # noqa: ARG002
    ) -> "tuple[float, float, float]":
        R      = obs.R
        bz_now = R[:, 2]
        self._bz_current = slerp_body_z(self._bz_current, self._bz_target, self._slew, dt)
        rate_sp = compute_rate_cmd(bz_now, self._bz_current, R, kp=self._kp_outer, kd=0.0)

        vz_err = float(obs.vel[2]) - self._vz_sp
        self._col_i = float(np.clip(
            self._col_i + self._ki_vz * vz_err * dt,
            self._col_min,
            self._col_max,
        ))
        self._C_held = float(np.clip(
            self._col_i + self._kp_vz * vz_err,
            self._col_min,
            self._col_max,
        ))

        return self._C_held, float(rate_sp[0]), float(rate_sp[1])

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def bz_current(self) -> np.ndarray:
        return self._bz_current.copy()

    @property
    def elevation_rad(self) -> float:
        return float(np.arcsin(np.clip(-self._bz_current[2], -1.0, 1.0)))

    def log_fields(self) -> dict:
        return dict(
            elevation_rad = self.elevation_rad,
            body_z_eq     = self.bz_current,
        )


class _PythonBackend(_MockArdupilotBase):
    AP_HZ: float = 50.0

    def __init__(self, mode, *, wind: "np.ndarray", dt: float) -> None:
        super().__init__(wind=wind, dt=dt, initial_col_rad=float(mode._C_held))
        self._mode = mode
        self._t_last = None

    @property
    def ap(self):
        return self._mode

    @property
    def comms_ok(self) -> bool:
        return bool(getattr(self._mode, "comms_ok", True))

    @property
    def elevation_rad(self) -> float:
        return float(getattr(self._mode, "elevation_rad"))

    def receive_command(self, cmd, dt_cmd: float) -> None:
        self._mode.receive_command(cmd, dt_cmd)

    def controller_step(
        self,
        obs: HubObservation,
        dt: float,
        *,
        accel_ned: "np.ndarray | None" = None,
    ) -> "tuple[float, float, float]":
        col, roll_sp, pitch_sp = self._mode.step(obs, dt, accel_ned=accel_ned)
        self.col_rad = col
        self.roll_sp = roll_sp
        self.pitch_sp = pitch_sp
        return col, roll_sp, pitch_sp

    def log_fields(self) -> dict:
        return self._mode.log_fields()

    def tick(self, t_sim: float, runner, *, inject=None, accel_ned: "np.ndarray | None" = None) -> None:
        dt = 1.0 / self.AP_HZ if self._t_last is None else t_sim - self._t_last
        self._t_last = t_sim

        if inject is not None:
            inject(self, runner)

        obs = runner.observe()
        self.controller_step(obs, dt, accel_ned=accel_ned)

        if self._guided_ctrl is not None:
            throttle = max(0.0, min(1.0, (self.col_rad - self.COL_MIN) / (self.COL_MAX - self.COL_MIN)))
            self._guided_ctrl.set_target_rate_and_throttle(
                float(np.degrees(self.roll_sp)),
                float(np.degrees(self.pitch_sp)),
                0.0,
                throttle,
                sim_time=t_sim,
            )


class MockArdupilot:
    """Public test adapter entrypoint for lua/python AP backends."""

    AP_HZ: float = _PythonBackend.AP_HZ

    PUMPING_CONSTANTS = {
        "CMD_TIMEOUT_S": _PumpingPythonMode.CMD_TIMEOUT_S,
        "COL_MIN_RAD": _PumpingPythonMode.COL_MIN_RAD,
        "COL_MAX_RAD": _PumpingPythonMode.COL_MAX_RAD,
        "K_VIB": _PumpingPythonMode.K_VIB,
        "VIB_HP_HZ": _PumpingPythonMode.VIB_HP_HZ,
        "VIB_VEL_TAU": _PumpingPythonMode.VIB_VEL_TAU,
        "VIB_COL_MAX": _PumpingPythonMode.VIB_COL_MAX,
        "KP_ALT": _PumpingPythonMode.KP_ALT,
        "KI_ALT": _PumpingPythonMode.KI_ALT,
        "KD_VZ": _PumpingPythonMode.KD_VZ,
        "RATE_ACCEL_MAX_RADSS": _PumpingPythonMode.RATE_ACCEL_MAX_RADSS,
    }
    LANDING_CONSTANTS = {
        "COL_MIN_RAD": _LandingPythonMode.COL_MIN_RAD,
        "COL_MAX_RAD": _LandingPythonMode.COL_MAX_RAD,
        "KP_VZ": _LandingPythonMode.KP_VZ,
        "KI_VZ": _LandingPythonMode.KI_VZ,
    }

    @classmethod
    def for_lua(cls, sim, *, initial_col_rad: float = 0.0, wind: "np.ndarray", dt: float):
        return _LuaBackend(sim, initial_col_rad=initial_col_rad, wind=wind, dt=dt)

    @classmethod
    def for_python(cls, *, mode: str, wind: "np.ndarray", dt: float, **kwargs):
        if mode in ("pumping", "tension"):
            backend_mode = _PumpingPythonMode(**kwargs)
        elif mode == "landing":
            backend_mode = _LandingPythonMode(**kwargs)
        else:
            raise ValueError(f"Unsupported MockArdupilot Python mode: {mode!r}")
        return _PythonBackend(backend_mode, wind=wind, dt=dt)

    @classmethod
    def python_constants(cls, mode: str) -> dict:
        if mode in ("pumping", "tension"):
            return dict(cls.PUMPING_CONSTANTS)
        if mode == "landing":
            return dict(cls.LANDING_CONSTANTS)
        raise ValueError(f"Unsupported MockArdupilot Python mode: {mode!r}")

    @classmethod
    def for_pumping(
        cls,
        *,
        ic_pos: np.ndarray,
        mass_kg: float,
        slew_rate_rad_s: float,
        warm_coll_rad: float,
        tension_ic: float,
        wind: "np.ndarray",
        dt: float,
        **kwargs,
    ):
        """Construct a Python-equivalent pumping backend."""
        return cls.for_python(
            mode="pumping",
            ic_pos=ic_pos,
            mass_kg=mass_kg,
            slew_rate_rad_s=slew_rate_rad_s,
            warm_coll_rad=warm_coll_rad,
            tension_ic=tension_ic,
            wind=wind,
            dt=dt,
            **kwargs,
        )

    @classmethod
    def for_landing(
        cls,
        *,
        ic_body_z: np.ndarray,
        slew_rate_rad_s: float,
        warm_coll_rad: float,
        wind: "np.ndarray",
        dt: float,
        kp_vz: float = _LandingPythonMode.KP_VZ,
        ki_vz: float = _LandingPythonMode.KI_VZ,
        col_min_rad: float = _LandingPythonMode.COL_MIN_RAD,
        col_max_rad: float = _LandingPythonMode.COL_MAX_RAD,
        kp_outer: float = 2.5,
    ):
        return cls.for_python(
            mode="landing",
            ic_body_z=ic_body_z,
            slew_rate_rad_s=slew_rate_rad_s,
            warm_coll_rad=warm_coll_rad,
            kp_vz=kp_vz,
            ki_vz=ki_vz,
            col_min_rad=col_min_rad,
            col_max_rad=col_max_rad,
            kp_outer=kp_outer,
            wind=wind,
            dt=dt,
        )
