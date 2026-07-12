"""MockArdupilot adapter used by simtests and diagnostics.

Provides a common interface for driving either:
- Lua-backed control code (RawesLua harness), or
- Python-equivalent local AP behavior.
"""

from __future__ import annotations

import math
import os
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from arduloop import GuidedAttitudeController, HeliParams
from controller import (AZ_REF_TAU_S, compute_bz_altitude_hold,
                        compute_rate_cmd, compute_rate_cmd_sqrt,
                        compute_crosswind_rate_cmd,
                        apply_crosswind_rate_to_body_rates,
                        slerp_body_z, update_plane_azimuth)
from landing_planner import LandingCommand
from physics_core import HubObservation
from pumping_planner import TensionCommand
from param_defaults import get_ap_param, load_ap_params, load_rawes_lua_constants, load_collective_phys_range
from telemetry_csv import TelRow, write_csv


def _load_rawes_pumping_constants() -> dict[str, float]:
    lua_constants = load_rawes_lua_constants((
        "RATE_ACCEL_MAX_RADSS", "THRUST_SLEW_MAX", "THRUST_CRUISE",
        "POST_RELEASE_BLEND_S", "POST_RELEASE_RECOVERY_S",
    ))
    params = load_ap_params()
    return {
        "KP_ALT": get_ap_param("RAWES_KP_ALT", params=params),
        "KI_ALT": get_ap_param("RAWES_KI_ALT", params=params),
        "KD_VZ": get_ap_param("RAWES_KD_VZ", params=params),
        "KP_EL": get_ap_param("RAWES_KP_EL", params=params),
        "TRP_S": params.get("RAWES_TRP", 2.0),  # tension ramp τ; default from Lua param:add_param
        "RATE_ACCEL_MAX_RADSS": lua_constants["RATE_ACCEL_MAX_RADSS"],
        "THRUST_SLEW_MAX": lua_constants["THRUST_SLEW_MAX"],
        "THRUST_CRUISE": lua_constants["THRUST_CRUISE"],
        "POST_RELEASE_BLEND_S": lua_constants["POST_RELEASE_BLEND_S"],
        "POST_RELEASE_RECOVERY_S": lua_constants["POST_RELEASE_RECOVERY_S"],
    }


_RAWES_PUMPING_CONSTANTS = _load_rawes_pumping_constants()


def _bz_ned_to_roll_pitch_deg(bz_ned: np.ndarray, yaw_rad: float) -> tuple[float, float]:
    """Convert desired body-z (NED) to roll/pitch at fixed yaw.

    Mirrors rawes.lua:bz_ned_to_roll_pitch() so Python and Lua angle paths use
    the same geometry.
    """
    bz = np.asarray(bz_ned, dtype=float)
    n = float(np.linalg.norm(bz))
    if n <= 1e-9:
        return 0.0, 0.0
    bz = bz / n

    cy = float(np.cos(yaw_rad))
    sy = float(np.sin(yaw_rad))
    bz_fwd = cy * float(bz[0]) + sy * float(bz[1])
    bz_right = -sy * float(bz[0]) + cy * float(bz[1])
    bz_down = float(bz[2])

    pitch_deg = float(np.degrees(np.arctan2(bz_fwd, bz_down)))
    roll_deg = float(np.degrees(np.arcsin(np.clip(-bz_right, -1.0, 1.0))))
    return roll_deg, pitch_deg


def _tel_every_from_env(dt: float, default_hz: float = 20.0) -> int:
    hz = float(os.environ.get("RAWES_TEL_HZ", default_hz))
    return max(1, round(1.0 / (hz * dt)))


def rawes_sitl_heli_params(loop_rate_hz: float) -> HeliParams:
    """HeliParams loaded from ArduPilot .parm files with requested loop rate."""
    return HeliParams(loop_rate_hz=loop_rate_hz)


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

    _COL_RANGE: "tuple[float, float] | None" = None

    @classmethod
    def _get_col_range(cls) -> "tuple[float, float]":
        if cls._COL_RANGE is None:
            cls._COL_RANGE = load_collective_phys_range()
        return cls._COL_RANGE

    @property
    def COL_MIN(self) -> float:
        return self._get_col_range()[0]

    @property
    def COL_MAX(self) -> float:
        return self._get_col_range()[1]

    def __init__(self, *, wind: "np.ndarray", dt: float, initial_thrust: float = 0.263) -> None:
        col_min, col_max = self._get_col_range()
        self._last_thrust = float(initial_thrust)
        self.col_rad = col_min + self._last_thrust * (col_max - col_min)
        self.roll_sp = 0.0
        self.pitch_sp = 0.0
        self._wind = wind
        self._tel_every = _tel_every_from_env(dt)
        self.tel_fn: "Callable[..., dict] | None" = None
        self._telemetry: list = []
        self._log_step = 0
        self._guided_ctrl: "GuidedAttitudeController | None" = None

    def enable_guided(self, heli_params: "HeliParams | None" = None) -> None:
        hp = heli_params or HeliParams()
        self._guided_ctrl = GuidedAttitudeController(hp)

    def step_physics(self, runner, dt: float, *, rest_length: "float | None" = None) -> dict:
        if self._guided_ctrl is None:
            raise RuntimeError("Call enable_guided() before step_physics()")
        obs = runner.observe()
        q_body = Rotation.from_matrix(obs.R).as_quat()
        gyro = obs.gyro
        col_norm = self._last_thrust * 2.0 - 1.0
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
            self._last_thrust = 0.5 * (c_norm + 1.0)
            self.col_rad = self.COL_MIN + self._last_thrust * (self.COL_MAX - self.COL_MIN)
        return runner.step_guided(dt, self.col_rad, heli_out, rest_length=rest_length)

    def log(self, runner, sr: dict) -> None:
        if self.tel_fn is None or self._tel_every is None:
            return
        if self._log_step % self._tel_every == 0:
            # Extract current body attitude (actual) from runner
            obs = runner.observe()
            R = obs.R
            roll_now  = math.atan2(float(R[2, 1]), float(R[2, 2]))
            pitch_now = -math.asin(float(np.clip(R[2, 0], -1.0, 1.0)))
            yaw_now   = math.atan2(float(R[1, 0]), float(R[0, 0]))

            # Extract target attitude and rates from guided controller
            mav_att_target_roll_deg = float("nan")
            mav_att_target_pitch_deg = float("nan")
            mav_att_target_yaw_deg = float("nan")
            mav_att_target_roll_rate_rads = float("nan")
            mav_att_target_pitch_rate_rads = float("nan")
            mav_att_target_yaw_rate_rads = float("nan")

            if self._guided_ctrl is not None:
                target_euler = self._guided_ctrl.attitude_target_euler_deg
                mav_att_target_roll_deg = float(target_euler[0])
                mav_att_target_pitch_deg = float(target_euler[1])
                mav_att_target_yaw_deg = float(target_euler[2])
                
                rate_tgt = self._guided_ctrl.last_rate_target_rads
                mav_att_target_roll_rate_rads = float(rate_tgt[0])
                mav_att_target_pitch_rate_rads = float(rate_tgt[1])
                mav_att_target_yaw_rate_rads = float(rate_tgt[2])

            rate_terms = {
                "rate_roll_p_contrib": float("nan"),
                "rate_roll_i_contrib": float("nan"),
                "rate_roll_d_contrib": float("nan"),
                "rate_roll_ff_contrib": float("nan"),
                "rate_pitch_p_contrib": float("nan"),
                "rate_pitch_i_contrib": float("nan"),
                "rate_pitch_d_contrib": float("nan"),
                "rate_pitch_ff_contrib": float("nan"),
                "rate_yaw_p_contrib": float("nan"),
                "rate_yaw_i_contrib": float("nan"),
                "rate_yaw_d_contrib": float("nan"),
                "rate_yaw_ff_contrib": float("nan"),
            }
            if self._guided_ctrl is not None:
                _rate_ctrl = getattr(self._guided_ctrl, "_rate_ctrl", None)
                if _rate_ctrl is not None:
                    _pid_roll = getattr(_rate_ctrl, "pid_roll", None)
                    _pid_pitch = getattr(_rate_ctrl, "pid_pitch", None)
                    _pid_yaw = getattr(_rate_ctrl, "pid_yaw", None)
                    if _pid_roll is not None:
                        rate_terms["rate_roll_p_contrib"] = float(_pid_roll.debug.P)
                        rate_terms["rate_roll_i_contrib"] = float(_pid_roll.debug.I)
                        rate_terms["rate_roll_d_contrib"] = float(_pid_roll.debug.D)
                        rate_terms["rate_roll_ff_contrib"] = float(_pid_roll.debug.FF + _pid_roll.debug.DFF)
                    if _pid_pitch is not None:
                        rate_terms["rate_pitch_p_contrib"] = float(_pid_pitch.debug.P)
                        rate_terms["rate_pitch_i_contrib"] = float(_pid_pitch.debug.I)
                        rate_terms["rate_pitch_d_contrib"] = float(_pid_pitch.debug.D)
                        rate_terms["rate_pitch_ff_contrib"] = float(_pid_pitch.debug.FF + _pid_pitch.debug.DFF)
                    if _pid_yaw is not None:
                        rate_terms["rate_yaw_p_contrib"] = float(_pid_yaw.debug.P)
                        rate_terms["rate_yaw_i_contrib"] = float(_pid_yaw.debug.I)
                        rate_terms["rate_yaw_d_contrib"] = float(_pid_yaw.debug.D)
                        rate_terms["rate_yaw_ff_contrib"] = float(_pid_yaw.debug.FF + _pid_yaw.debug.DFF)

            extra_kwargs = self.tel_fn(runner, sr) if self.tel_fn else {}
            extra_kwargs.update(
                mav_att_roll_deg=math.degrees(roll_now),
                mav_att_pitch_deg=math.degrees(pitch_now),
                mav_att_yaw_deg=math.degrees(yaw_now),
                mav_att_roll_rate_rads=float(obs.gyro[0]),
                mav_att_pitch_rate_rads=float(obs.gyro[1]),
                mav_att_yaw_rate_rads=float(obs.gyro[2]),
                mav_att_target_roll_deg=mav_att_target_roll_deg,
                mav_att_target_pitch_deg=mav_att_target_pitch_deg,
                mav_att_target_yaw_deg=mav_att_target_yaw_deg,
                mav_att_target_roll_rate_rads=mav_att_target_roll_rate_rads,
                mav_att_target_pitch_rate_rads=mav_att_target_pitch_rate_rads,
                mav_att_target_yaw_rate_rads=mav_att_target_yaw_rate_rads,
                roll_sp_rads=mav_att_target_roll_rate_rads,
                pitch_sp_rads=mav_att_target_pitch_rate_rads,
                yaw_sp_rads=mav_att_target_yaw_rate_rads,
            )
            extra_kwargs.update(rate_terms)

            self._telemetry.append(
                TelRow.from_physics(runner, sr, self.col_rad, self._wind, **extra_kwargs)
            )
        self._log_step += 1

    def write_telemetry(self, path) -> None:
        if self._telemetry:
            write_csv(self._telemetry, path)

    @property
    def telemetry(self) -> list:
        return self._telemetry


class _LuaBackend(_MockArdupilotBase):
    def __init__(self, sim, *, initial_thrust: float = 0.263, wind: "np.ndarray", dt: float) -> None:
        super().__init__(wind=wind, dt=dt, initial_thrust=initial_thrust)
        self._sim = sim
        hz = round(1.0 / dt)
        self.enable_guided(rawes_sitl_heli_params(hz))
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
            self._last_thrust = float(gt_throttle)
            self.col_rad = self.COL_MIN + self._last_thrust * (self.COL_MAX - self.COL_MIN)
        else:
            ch3 = self._sim.ch_out[3]
            if ch3 is not None:
                self._last_thrust = max(0.0, min(1.0, (ch3 - 1000) / 1000.0))
                self.col_rad = self.COL_MIN + self._last_thrust * (self.COL_MAX - self.COL_MIN)

        gt_rate = self._sim._mock.guided_rate_target
        if gt_rate is not None:
            throttle = float(gt_throttle) if gt_throttle is not None else self._last_thrust
            self._ctrl.set_target_rate_and_throttle(
                float(gt_rate.roll_rate),
                float(gt_rate.pitch_rate),
                float(gt_rate.yaw_rate),
                throttle,
                sim_time=t_sim,
            )

        gt = self._sim._mock.guided_target
        if gt is not None:
            throttle = float(gt_throttle) if gt_throttle is not None else self._last_thrust
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


def _blend_bz(bz_now: "np.ndarray", bz_goal: "np.ndarray", alpha: float) -> "np.ndarray":
    """Normalised linear blend from bz_now toward bz_goal. Mirrors rawes.lua blend_bz()."""
    a = max(0.0, min(1.0, alpha))
    v = (1.0 - a) * np.asarray(bz_now, dtype=float) + a * np.asarray(bz_goal, dtype=float)
    n = float(np.linalg.norm(v))
    return np.asarray(bz_now, dtype=float) if n < 1e-6 else v / n


class _PumpingPythonMode:
    """Python-equivalent of rawes.lua do_steady_loop_inner().

    Variable names mirror the Lua originals so the two can be diffed line-by-line.
    """

    CMD_TIMEOUT_S: float = 0.5
    FEASIBILITY_WINDOW_S: float = 1.0

    # Mirror rawes.lua constants (loaded from Lua source to stay in sync).
    KP_ALT:                float = _RAWES_PUMPING_CONSTANTS["KP_ALT"]
    KI_ALT:                float = _RAWES_PUMPING_CONSTANTS["KI_ALT"]
    KD_VZ:                 float = _RAWES_PUMPING_CONSTANTS["KD_VZ"]
    KP_EL:                 float = _RAWES_PUMPING_CONSTANTS["KP_EL"]
    THRUST_CRUISE:         float = _RAWES_PUMPING_CONSTANTS["THRUST_CRUISE"]
    THRUST_SLEW_MAX:       float = _RAWES_PUMPING_CONSTANTS["THRUST_SLEW_MAX"]
    POST_RELEASE_BLEND_S:     float = _RAWES_PUMPING_CONSTANTS["POST_RELEASE_BLEND_S"]
    POST_RELEASE_RECOVERY_S:  float = _RAWES_PUMPING_CONSTANTS["POST_RELEASE_RECOVERY_S"]
    TRP_S:                 float = _RAWES_PUMPING_CONSTANTS["TRP_S"]  # tension ramp τ [s]
    VZ_GATE_RATE_RADS:     float = 1.0
    VZ_GATE_MIN:           float = 0.0
    RATE_ACCEL_MAX_RADSS:  float = _RAWES_PUMPING_CONSTANTS["RATE_ACCEL_MAX_RADSS"]

    def __init__(
        self,
        ic_pos              : np.ndarray,
        mass_kg             : float,
        slew_rate_rad_s     : float,
        warm_thrust         : float,
        tension_ic          : float,
        cmd_timeout_s       : float = CMD_TIMEOUT_S,
        kp_outer            : float = KP_EL,
        kp_alt              : float = KP_ALT,
        ki_alt              : float = KI_ALT,
        kd_vz               : float = KD_VZ,
        rate_accel_max_radss: float = 0.0,
        az_ref_tau_s        : float = AZ_REF_TAU_S,
        cw_rate_kp          : float = 0.0,
        cw_rate_kd          : float = 0.0,
        cw_rate_max         : float = 0.6,
        events              = None,
    ) -> None:
        self._mass_kg        = float(mass_kg)
        self._timeout        = float(cmd_timeout_s)
        self._bz_slew        = float(slew_rate_rad_s)  # mirrors rawes.lua _bz_slew
        self._kp_outer       = float(kp_outer)
        self._events         = events
        self._az_tau         = float(az_ref_tau_s)

        # Mirror rawes.lua: _az_ref seeded from initial position
        self._az_ref         = float(np.arctan2(ic_pos[1], ic_pos[0]))

        tlen                 = float(np.linalg.norm(ic_pos))
        self._el_rad         = float(np.arcsin(max(-1.0, min(1.0, float(-ic_pos[2]) / max(tlen, 0.1)))))  # mirrors _el_rad

        self._tension_cmd_n  = float(tension_ic)   # step target (mirrors rawes.lua _tension_cmd_n)
        self._tension_for_bz = float(tension_ic)   # ramped output (mirrors rawes.lua _tension_n)
        self._kp_alt         = float(kp_alt)
        self._ki_alt         = float(ki_alt)
        self._kd_vz          = float(kd_vz)
        self._rate_accel_max = float(rate_accel_max_radss) if rate_accel_max_radss else _RAWES_PUMPING_CONSTANTS["RATE_ACCEL_MAX_RADSS"]
        self._cw_rate_kp     = float(cw_rate_kp)
        self._cw_rate_kd     = float(cw_rate_kd)
        self._cw_rate_max    = float(cw_rate_max)

        # Mirror rawes.lua: _ic_thrust (from RAWES_THR), _thrust_trim, _last_thrust, _alt_i
        self._ic_thrust      = float(warm_thrust)   # mirrors _ic_thrust; updated from RAWES_THR NVF in Lua
        self._thrust_trim    = float(warm_thrust)   # mirrors _thrust_trim; reset at capture
        self._last_thrust    = float(warm_thrust)   # mirrors _last_thrust; slew state
        self._thrust_held    = float(warm_thrust)   # last output for telemetry
        self._alt_i          = 0.0

        # Mirror rawes.lua: _target_alt, _tension_cmd_n, _cmd_age
        self._target_alt     = float(-ic_pos[2])
        self._cmd_age        = 0.0
        self._comms_ok       = True
        self._t_sim          = 0.0
        self._pos_ned        = np.asarray(ic_pos, dtype=float)
        self._pos_design     = np.asarray(ic_pos, dtype=float)

        # Mirror rawes.lua: _capture_ms (None = not-yet-captured → recovery_alpha stays 1.0)
        self._capture_t: "float | None" = None

        self._bz_goal        = None
        self._last_roll_sp   = 0.0
        self._last_pitch_sp  = 0.0
        self._tension_ic     = float(tension_ic)

    def _ic_thrust_or_default(self) -> float:
        """Return IC thrust if set, else THRUST_CRUISE. Mirrors rawes.lua ic_thrust_or_default()."""
        return self._ic_thrust  # always set from warm_thrust at init

    def notify_captured(self, t_sim: float) -> None:
        """Signal steady-state capture. Mirrors rawes.lua do_steady_enter capture block.

        Resets altitude integrator, seeds _thrust_trim/_last_thrust from IC,
        and starts the recovery ramp timer (_capture_t).
        """
        thr_ff             = max(0.0, min(1.0, self._ic_thrust_or_default()))
        self._thrust_trim  = thr_ff
        self._last_thrust  = thr_ff
        self._alt_i        = 0.0
        self._capture_t    = float(t_sim)
        self._tension_for_bz = self._tension_cmd_n  # seed ramp at capture (no startup transient)

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
                el_gap   = abs(cmd_el - self._el_rad)
                max_slew = self._bz_slew * self.FEASIBILITY_WINDOW_S
                if el_gap > max_slew:
                    hub_el = float(np.arcsin(max(-1.0, min(1.0, hub_alt / max(tlen, 0.1)))))
                    self._events.record(
                        "ap_unreachable_alt", self._t_sim, phase, hub_alt,
                        target_alt=new_alt, ap_el_deg=float(np.degrees(self._el_rad)),
                        cmd_el_deg=float(np.degrees(cmd_el)),
                        el_gap_deg=float(np.degrees(el_gap)),
                        max_slew_deg=float(np.degrees(max_slew)),
                        hub_el_deg=float(np.degrees(hub_el)),
                    )

        self._target_alt     = new_alt
        self._cmd_age        = 0.0
        self._comms_ok       = True
        self._tension_cmd_n  = float(cmd.tension_target_n)  # step; ramp applied in step()

    def step(
        self,
        obs      : HubObservation,
        dt       : float,
        *,
        accel_ned: "np.ndarray | None" = None,  # noqa: ARG002
    ) -> "tuple[float, float, float]":
        self._pos_ned  = obs.pos
        self._cmd_age += dt
        self._t_sim   += dt
        if self._comms_ok and self._cmd_age > self._timeout:
            self._comms_ok = False

        tlen      = float(np.linalg.norm(obs.pos))

        # Elevation slew: mirrors rawes.lua _bz_slew * dt clamped step
        target_el = float(np.arcsin(max(-1.0, min(1.0, -obs.pos[2] / max(tlen, 0.1)))))
        max_step  = self._bz_slew * dt
        el_step   = float(np.clip(target_el - self._el_rad, -max_step, max_step))
        self._el_rad += el_step

        self._az_ref = update_plane_azimuth(self._az_ref, obs.pos, self._az_tau, dt)

        # Tension ramp: mirrors rawes.lua _apply_tension_ramp(dt)
        if self.TRP_S > 0.0:
            self._tension_for_bz += (dt / self.TRP_S) * (self._tension_cmd_n - self._tension_for_bz)
        else:
            self._tension_for_bz = self._tension_cmd_n

        # body-z altitude-hold target: mirrors rawes.lua bz_altitude_hold()
        bz_goal = compute_bz_altitude_hold(
            obs.pos, self._el_rad, self._tension_for_bz, self._mass_kg,
            az_ref_rad=self._az_ref,
        )
        R      = obs.R
        bz_now = R[:, 2]

        # recovery_alpha + bz_goal blend after capture: mirrors rawes.lua
        recovery_alpha = 1.0
        if self._capture_t is not None:
            t_rel_s    = self._t_sim - self._capture_t
            blend      = min(1.0, t_rel_s / self.POST_RELEASE_BLEND_S)
            recovery_alpha = max(0.0, min(1.0, t_rel_s / self.POST_RELEASE_RECOVERY_S))
            if blend < 1.0:
                bz_goal = _blend_bz(bz_now, bz_goal, blend)

        # body-z rate command: mirrors rawes.lua compute_rate_cmd_sqrt() → send_guided_angle_rate_throttle()
        rate_sp = compute_rate_cmd_sqrt(
            bz_now, bz_goal, R,
            kp=self._kp_outer,
            accel_max=self._rate_accel_max,
            dt=dt,
            kd=0.0,
        )

        # Crosswind rate damping: mirrors rawes.lua rate_roll_cw / rate_pitch_cw block
        if self._cw_rate_kp > 0.0 or self._cw_rate_kd > 0.0:
            omega_east_cmd  = compute_crosswind_rate_cmd(
                obs.pos, obs.vel, self._pos_design,
                kp=self._cw_rate_kp, kd=self._cw_rate_kd,
                rate_max=self._cw_rate_max,
            )
            omega_body_corr = apply_crosswind_rate_to_body_rates(omega_east_cmd, R)
            rate_sp = np.asarray(rate_sp, dtype=float).copy()
            rate_sp[0] += float(omega_body_corr[0])
            rate_sp[1] += float(omega_body_corr[1])

        self._bz_goal      = bz_goal
        self._last_roll_sp = float(rate_sp[0])
        self._last_pitch_sp = float(rate_sp[1])

        # ── Altitude PID: mirrors rawes.lua do_steady_loop_inner() ──────────
        alt_m   = float(-obs.pos[2])
        vz_up   = float(-obs.vel[2])
        alt_err = self._target_alt - alt_m

        # Integral with anti-windup (mirrors rawes.lua i_min / i_max clamping)
        i_min = 0.0 - self._thrust_trim
        i_max = 1.0 - self._thrust_trim
        self._alt_i = max(i_min, min(i_max, self._alt_i + self._ki_alt * alt_err * dt))

        # Rate-stability gate (mirrors rawes.lua vz_gate)
        rate_mag = float(np.linalg.norm(obs.gyro))
        vz_gate  = max(self.VZ_GATE_MIN, min(1.0, 1.0 - rate_mag / self.VZ_GATE_RATE_RADS))

        alt_p     = self._kp_alt * alt_err
        alt_d     = -self._kd_vz * vz_gate * vz_up
        thrust_pid = max(0.0, min(1.0, self._thrust_trim + alt_p + alt_d + self._alt_i))

        # Recovery blend (mirrors rawes.lua ic_thrust + recovery_alpha * (thrust_pid - ic_thrust))
        ic_thrust_now = self._ic_thrust_or_default()
        thrust_cmd    = ic_thrust_now + recovery_alpha * (thrust_pid - ic_thrust_now)
        thrust_cmd    = max(0.0, min(1.0, thrust_cmd))

        # Slew rate limit (mirrors rawes.lua THRUST_SLEW_MAX per step)
        thrust_delta     = max(-self.THRUST_SLEW_MAX, min(self.THRUST_SLEW_MAX, thrust_cmd - self._last_thrust))
        self._last_thrust += thrust_delta
        self._thrust_held  = self._last_thrust

        return self._last_thrust, float(rate_sp[0]), float(rate_sp[1])

    @property
    def comms_ok(self) -> bool:
        return self._comms_ok

    @property
    def tension_feedforward_n(self) -> float:
        return self._tension_for_bz

    @property
    def elevation_rad(self) -> float:
        return self._el_rad

    def log_fields(self) -> dict:
        bz = self._bz_goal
        return dict(
            tension_feedforward_n = self._tension_for_bz,
            tension_ic_n          = self._tension_ic,
            elevation_rad         = self.elevation_rad,
            comms_ok              = self.comms_ok,
            thrust_from_alt_ctrl  = self._thrust_held,
            gnd_alt_cmd_m         = self._target_alt,
            alt_pid_integral      = self._alt_i,
            roll_sp_rads          = self._last_roll_sp,
            pitch_sp_rads         = self._last_pitch_sp,
            body_z_eq             = bz.tolist() if bz is not None else [0.0, 0.0, 0.0],
        )

    @property
    def body_z_target(self) -> np.ndarray | None:
        return None if self._bz_goal is None else np.asarray(self._bz_goal, dtype=float).copy()


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

    @property
    def body_z_target(self) -> np.ndarray:
        return self.bz_current


class _PythonBackend(_MockArdupilotBase):
    AP_HZ: float = 50.0

    def __init__(self, mode, *, wind: "np.ndarray", dt: float) -> None:
        super().__init__(wind=wind, dt=dt, initial_thrust=float(mode._thrust_held))
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
        thrust, roll_sp, pitch_sp = self._mode.step(obs, dt, accel_ned=accel_ned)
        self.col_rad = self.COL_MIN + float(thrust) * (self.COL_MAX - self.COL_MIN)
        self.roll_sp = roll_sp
        self.pitch_sp = pitch_sp
        return thrust, roll_sp, pitch_sp

    def log_fields(self) -> dict:
        return self._mode.log_fields()

    def tick(self, t_sim: float, runner, *, inject=None, accel_ned: "np.ndarray | None" = None) -> None:
        dt = 1.0 / self.AP_HZ if self._t_last is None else t_sim - self._t_last
        self._t_last = t_sim

        if inject is not None:
            inject(self, runner)

        obs = runner.observe()
        thrust, roll_sp, pitch_sp = self.controller_step(obs, dt, accel_ned=accel_ned)

        if self._guided_ctrl is not None:
            bz_target = getattr(self._mode, "body_z_target", None)
            if bz_target is not None:
                yaw_rad = float(np.arctan2(obs.R[1, 0], obs.R[0, 0]))
                roll_deg, pitch_deg = _bz_ned_to_roll_pitch_deg(np.asarray(bz_target, dtype=float), yaw_rad)
                self._guided_ctrl.set_target_angle_and_rate_and_throttle(
                    roll_deg,
                    pitch_deg,
                    float(np.degrees(yaw_rad)),
                    0.0,
                    0.0,
                    0.0,
                    float(thrust),
                    sim_time=t_sim,
                )
            else:
                self._guided_ctrl.set_target_rate_and_throttle(
                    float(np.degrees(roll_sp)),
                    float(np.degrees(pitch_sp)),
                    0.0,
                    float(thrust),
                    sim_time=t_sim,
                )


class MockArdupilot:
    """Public test adapter entrypoint for lua/python AP backends."""

    AP_HZ: float = _PythonBackend.AP_HZ

    PUMPING_CONSTANTS = {
        "CMD_TIMEOUT_S": _PumpingPythonMode.CMD_TIMEOUT_S,
        "KP_ALT": _PumpingPythonMode.KP_ALT,
        "KI_ALT": _PumpingPythonMode.KI_ALT,
        "KD_VZ": _PumpingPythonMode.KD_VZ,
        "KP_EL": _PumpingPythonMode.KP_EL,
        "RATE_ACCEL_MAX_RADSS": _RAWES_PUMPING_CONSTANTS["RATE_ACCEL_MAX_RADSS"],
    }
    LANDING_CONSTANTS = {
        "COL_MIN_RAD": _LandingPythonMode.COL_MIN_RAD,
        "COL_MAX_RAD": _LandingPythonMode.COL_MAX_RAD,
        "KP_VZ": _LandingPythonMode.KP_VZ,
        "KI_VZ": _LandingPythonMode.KI_VZ,
    }

    @classmethod
    def for_lua(cls, sim, *, initial_thrust: float = 0.263, wind: "np.ndarray", dt: float):
        return _LuaBackend(sim, initial_thrust=initial_thrust, wind=wind, dt=dt)

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
        warm_thrust: float,
        tension_ic: float,
        wind: "np.ndarray",
        dt: float,
        **kwargs,
    ):
        return cls.for_python(
            mode="pumping",
            ic_pos=ic_pos,
            mass_kg=mass_kg,
            slew_rate_rad_s=slew_rate_rad_s,
            warm_thrust=warm_thrust,
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
        kp_outer: float = _PumpingPythonMode.KP_EL,
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
