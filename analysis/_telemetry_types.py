"""Read-only contracts for mixed text/numeric analysis telemetry records."""

from typing import Literal, Protocol, overload

type _TextColumn = Literal["phase", "note"]
type _NumericColumn = Literal[
    "t_sim", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
    "sens_vel_n", "sens_vel_e", "sens_vel_d", "rpy_yaw", "v_horiz_ms",
    "tether_rest_length", "tether_tension", "tether_slack", "winch_speed_ms",
    "collective_rad", "thrust_from_alt_ctrl", "alt_pid_integral", "omega_rotor",
    "bz_eq_x", "bz_eq_y", "bz_eq_z", "r02", "r12", "r22", "gnd_alt_cmd_m",
    "aero_T", "aero_v_i", "aero_v_axial", "aero_v_inplane", "tether_length",
    "tether_extension",
    "tension_feedforward_n",
]


class _TelemetryRow(Protocol):
    @overload
    def __getitem__(self, key: _TextColumn, /) -> str: ...
    @overload
    def __getitem__(self, key: _NumericColumn, /) -> float: ...
    @overload
    def get(self, key: _TextColumn, default: str, /) -> str: ...
    @overload
    def get(self, key: _NumericColumn, default: float, /) -> float: ...
    def __contains__(self, key: object, /) -> bool: ...
