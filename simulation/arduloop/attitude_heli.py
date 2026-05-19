"""
Heli rate-loop wrapper, equivalent to the rate-control portion of
`AC_AttitudeControl_Heli`.

Per call to :meth:`HeliRateController.update`:

    1. Run roll, pitch, yaw PIDs with their own filters/notches.
    2. Apply pirouette compensation (rotate I-terms of roll & pitch by the
       yaw-rate integral over dt) — ArduPilot `ATC_PIRO_COMP`.
    3. Rotate the (roll_out, pitch_out) pair by `H_SW_H3_PHANG` via
       :class:`SwashH3`.

It does **not** synthesise attitude — feed it body-frame rate targets from
your outer attitude / position loop.

The optional hover-roll-trim (`ATC_HOVR_ROL_TRM`) is implemented as a
collective-scaled additive bias on the roll cyclic, exactly as in
`AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .params import HeliParams
from .pid import AC_PID
from .swash import SwashH3


@dataclass
class HeliRateOutput:
    roll_cyclic:  float = 0.0   # [-1, 1]
    pitch_cyclic: float = 0.0
    yaw_cmd:      float = 0.0   # tail-rotor / yaw axis (no swash mixing)


class HeliRateController:
    def __init__(self, params: HeliParams):
        self.params = params
        fs = params.loop_rate_hz
        self.pid_roll  = AC_PID(params.roll,  fs)
        self.pid_pitch = AC_PID(params.pitch, fs)
        self.pid_yaw   = AC_PID(params.yaw,   fs)
        self.swash = SwashH3(params.H_SW_H3_PHANG)
        self.out = HeliRateOutput()

    def reload_params(self) -> None:
        self.pid_roll.reload_params()
        self.pid_pitch.reload_params()
        self.pid_yaw.reload_params()
        self.swash.set_phase(self.params.H_SW_H3_PHANG)

    def reset(self) -> None:
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()

    def update(self,
               rate_target_rads: tuple[float, float, float],
               gyro_rate_rads:   tuple[float, float, float],
               dt: float,
               collective_norm: float = 0.0,
               saturated: tuple[bool, bool, bool] = (False, False, False),
               ) -> HeliRateOutput:
        """Run one tick of the rate loop.

        Parameters
        ----------
        rate_target_rads : (roll, pitch, yaw) target body rates, rad/s.
        gyro_rate_rads   : measured body rates (already gyro-filtered), rad/s.
        dt               : tick duration, seconds.
        collective_norm  : current collective in [-1, 1]; used by hover-roll-trim.
        saturated        : per-axis output saturation flags for anti-windup.
        """
        tr, tp, ty = rate_target_rads
        gr, gp, gy = gyro_rate_rads
        sr, sp, sy = saturated
        p = self.params

        roll_out  = self.pid_roll.update_all(tr,  gr, dt, limit=sr)
        pitch_out = self.pid_pitch.update_all(tp, gp, dt, limit=sp)
        yaw_out   = self.pid_yaw.update_all(ty,  gy, dt, limit=sy)

        # ------------------------------------------------------------------
        # PIRO_COMP — rotate roll & pitch I-terms by yaw rate (AP `_piro_comp`)
        # ------------------------------------------------------------------
        if p.PIRO_COMP_enabled and dt > 0.0:
            piro_roll_i  = self.pid_roll.get_i()
            piro_pitch_i = self.pid_pitch.get_i()
            yaw_dtheta = gy * dt
            c = math.cos(yaw_dtheta)
            s = math.sin(yaw_dtheta)
            self.pid_roll.set_i( piro_roll_i  * c - piro_pitch_i * s)
            self.pid_pitch.set_i(piro_roll_i  * s + piro_pitch_i * c)

        # ------------------------------------------------------------------
        # Hover roll trim — `ATC_HOVR_ROL_TRM` in centi-degrees, scaled by
        # collective. Converted to a small additive cyclic offset.
        # ------------------------------------------------------------------
        if p.HOVR_ROL_TRM_cd != 0.0:
            trim_rad = math.radians(p.HOVR_ROL_TRM_cd * 0.01)
            roll_out += trim_rad * max(0.0, collective_norm)

        # ------------------------------------------------------------------
        # Swash phase rotation — `H_SW_H3_PHANG`
        # ------------------------------------------------------------------
        roll_mix, pitch_mix = self.swash.mix(roll_out, pitch_out)

        # Output clipping
        lim = p.output_limit
        self.out.roll_cyclic  = max(-lim, min(lim, roll_mix))
        self.out.pitch_cyclic = max(-lim, min(lim, pitch_mix))
        self.out.yaw_cmd      = max(-lim, min(lim, yaw_out))
        return self.out
