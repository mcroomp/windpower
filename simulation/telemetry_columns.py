"""
telemetry_columns.py -- Canonical telemetry CSV schema and column metadata.

This file owns the master ordered telemetry CSV field list.
Any schema questions should start here.
"""

from __future__ import annotations


# ---------------------------------------------------------------------------
# Column groups -- grouped by primary producer/source
# Frame comments are included for fields with explicit frame semantics.
# ---------------------------------------------------------------------------

COLUMN_GROUPS: tuple[tuple[str, tuple[str, ...]], ...] = (
    # Core loop bookkeeping written by mediator main loop.
    (
        "mediator_loop",
        (
            "t_sim",
            "sitl_time",
            "frame_count",
            "phase",
            "note",
        ),
    ),
    # Startup damping blend state from PhysicsCore kinematic/release logic.
    ("startup_damping", ("damp_alpha",)),
    # Hub state + kinematics from PhysicsCore / RigidBodyDynamics integration.
    (
        "physics_state",
        (
            "pos_x",         # NED world
            "pos_y",         # NED world
            "pos_z",         # NED world
            "vel_x",         # NED world
            "vel_y",         # NED world
            "vel_z",         # NED world
            "omega_x",       # NED/world angular-velocity vector components
            "omega_y",       # NED/world angular-velocity vector components
            "omega_z",       # NED/world angular-velocity vector components
            "accel_x",       # NED world
            "accel_y",       # NED world
            "accel_z",       # NED world
            "omega_rotor",
        ),
    ),
    # Tether force/length outputs from tether model inside PhysicsCore.
    (
        "tether_model",
        (
            "tether_length",
            "tether_extension",
            "tether_tension",
            "tether_rest_length",
            "tether_slack",
            "tether_fx",     # NED world force
            "tether_fy",     # NED world force
            "tether_fz",     # NED world force
            "tether_mx",     # body/orbital moment components used by dynamics
            "tether_my",     # body/orbital moment components used by dynamics
            "tether_mz",     # body/orbital moment components used by dynamics
        ),
    ),
    # Rotor/aero inputs and outputs from rotor model evaluation in PhysicsCore.
    (
        "aero_model",
        (
            "collective_rad",
            "tilt_lon",      # body cyclic (longitudinal)
            "tilt_lat",      # body cyclic (lateral)
            "aero_fx",       # NED world force
            "aero_fy",       # NED world force
            "aero_fz",       # NED world force
            "aero_mx",       # body/orbital moment components used by dynamics
            "aero_my",       # body/orbital moment components used by dynamics
            "aero_mz",       # body/orbital moment components used by dynamics
            "aero_T",
            "aero_v_axial",  # rotor/body-axis decomposition scalar
            "aero_v_inplane",# rotor/body-axis decomposition scalar
            "aero_v_i",
            "aero_Q_spin",
        ),
    ),
    # Outer-loop control diagnostics from planner/Lua command pipeline via mediator.
    (
        "ap_outer_loop",
        (
            "collective_norm",
            "tension_feedforward_n",
            "tension_ic_n",
            "collective_from_alt_ctrl",
            "gnd_alt_cmd_m",
            "winch_speed_ms",
            "elevation_rad",
            "el_correction_rad",
            "coll_saturated",
            "comms_ok",
            "alt_pid_integral",
            "roll_sp_rads",  # body roll-rate setpoint
            "pitch_sp_rads", # body pitch-rate setpoint
            "roll_rate_err_rads",   # body roll-rate error
            "pitch_rate_err_rads",  # body pitch-rate error
        ),
    ),
    # Net force/moment assembled in PhysicsCore before state integration.
    (
        "net_dynamics",
        (
            "F_x",           # NED world net force
            "F_y",           # NED world net force
            "F_z",           # NED world net force
            "M_x",           # body/orbital net moment component
            "M_y",           # body/orbital net moment component
            "M_z",           # body/orbital net moment component
        ),
    ),
    # Euler attitude solution derived from hub rotation matrix.
    (
        "attitude_solution",
        (
            "rpy_roll",      # body attitude expressed in NED Euler
            "rpy_pitch",     # body attitude expressed in NED Euler
            "rpy_yaw",       # body attitude expressed in NED Euler
        ),
    ),
    # Synthetic IMU/GPS sensor outputs produced by sensor model.
    (
        "sensor_model",
        (
            "orb_yaw_rad",   # yaw in NED
            "v_horiz_ms",
            "sens_vel_n",    # NED sensor velocity
            "sens_vel_e",    # NED sensor velocity
            "sens_vel_d",    # NED sensor velocity
            "sens_accel_x",  # body-frame accelerometer
            "sens_accel_y",  # body-frame accelerometer
            "sens_accel_z",  # body-frame accelerometer
            "sens_gyro_x",   # body-frame gyro
            "sens_gyro_y",   # body-frame gyro
            "sens_gyro_z",   # body-frame gyro
            "vel_heading_deg", # heading derived from NED velocity
            "heading_gap_deg", # heading gap in NED heading convention
        ),
    ),
    # Asynchronous MAVLink captures (ATTITUDE/TARGET/SERVO) sampled by mediator.
    (
        "mavlink_async",
        (
            "mav_time_boot_ms",
            "mav_time_usec",
            "mav_att_roll_deg",   # MAVLink ATTITUDE roll (NED Euler)
            "mav_att_pitch_deg",  # MAVLink ATTITUDE pitch (NED Euler)
            "mav_att_yaw_deg",    # MAVLink ATTITUDE yaw (NED Euler)
            "mav_att_target_roll_deg",       # MAVLink ATTITUDE_TARGET roll (NED Euler)
            "mav_att_target_pitch_deg",      # MAVLink ATTITUDE_TARGET pitch (NED Euler)
            "mav_att_target_yaw_deg",        # MAVLink ATTITUDE_TARGET yaw (NED Euler)
            "mav_att_target_roll_rate_rads", # MAVLink ATTITUDE_TARGET body rate
            "mav_att_target_pitch_rate_rads",# MAVLink ATTITUDE_TARGET body rate
            "mav_att_target_yaw_rate_rads",  # MAVLink ATTITUDE_TARGET body rate
            "mav_servo1_us",
            "mav_servo2_us",
            "mav_servo3_us",
            "mav_servo4_us",
            # Lua NAMED_VALUE_FLOAT diagnostics (latest async snapshot)
            "mav_nvf_yaw_i",
            "mav_nvf_yaw_out",
            "mav_nvf_yff_trim",
            "mav_nvf_yff_u",
            "mav_nvf_yff_gz",
            "mav_nvf_yff_a",
        ),
    ),
    # Direct servo values decoded from SITL servo packet in mediator.
    (
        "mediator_servo_capture",
        (
            "servo_s1_us",
            "servo_s2_us",
            "servo_s3_us",
            "servo4_us",
        ),
    ),
    # Anti-rotation torque model internals from mediator_torque path.
    ("torque_model", ("q_bearing_nm", "q_motor_nm", "throttle")),
    # Ambient wind vector from simulation environment config.
    (
        "environment",
        (
            "wind_x",        # NED world wind
            "wind_y",        # NED world wind
            "wind_z",        # NED world wind
        ),
    ),
    # body_z controller targets/errors from controller/mediator diagnostics.
    (
        "ap_bodyz_controller",
        (
            "bz_eq_x",       # NED world vector
            "bz_eq_y",       # NED world vector
            "bz_eq_z",       # NED world vector
            "body_z_err_deg",
            "body_z_target_az_rad", # NED azimuth convention
            "body_z_az_gap_deg",    # NED azimuth convention
        ),
    ),
    # Derived orbital/downwind kinematics computed in telemetry assembly.
    (
        "derived_kinematics",
        (
            "vel_radial_mps",      # NED-horizontal radial component
            "orbit_radius_m",      # NED-horizontal geometric radius
            "orbit_azimuth_rad",   # NED azimuth convention
            "orbit_az_rate_rads",  # NED azimuth rate
            "vel_tangential_mps",  # NED-horizontal tangential component
            "pos_downwind_m",      # wind-aligned horizontal frame
            "pos_crosswind_m",     # wind-aligned horizontal frame
            "vel_downwind_mps",    # wind-aligned horizontal frame
            "vel_crosswind_mps",   # wind-aligned horizontal frame
        ),
    ),
    # Flattened body->NED rotation matrix copied from hub attitude state.
    (
        "rotation_matrix",
        (
            "r00",           # R body->NED
            "r01",           # R body->NED
            "r02",           # R body->NED
            "r10",           # R body->NED
            "r11",           # R body->NED
            "r12",           # R body->NED
            "r20",           # R body->NED
            "r21",           # R body->NED
            "r22",           # R body->NED
        ),
    ),
)

COLUMN_SPECS: tuple[tuple[str, str], ...] = tuple(
    (name, source)
    for source, names in COLUMN_GROUPS
    for name in names
)

_COLUMN_NAMES = [name for name, _source in COLUMN_SPECS]
if len(set(_COLUMN_NAMES)) != len(_COLUMN_NAMES):
    raise ValueError("Duplicate telemetry column in COLUMN_GROUPS")

# Keep this as a list for DictWriter fieldnames and stable key-order checks.
COLUMNS: list[str] = [name for name, _source in COLUMN_SPECS]
COLUMN_SOURCES: dict[str, str] = dict(COLUMN_SPECS)

ASYNC_MAV_COLUMNS: tuple[str, ...] = (
    "mav_time_boot_ms",
    "mav_time_usec",
    "mav_att_roll_deg",
    "mav_att_pitch_deg",
    "mav_att_yaw_deg",
    "mav_att_target_roll_deg",
    "mav_att_target_pitch_deg",
    "mav_att_target_yaw_deg",
    "mav_att_target_roll_rate_rads",
    "mav_att_target_pitch_rate_rads",
    "mav_att_target_yaw_rate_rads",
    "mav_servo1_us",
    "mav_servo2_us",
    "mav_servo3_us",
    "mav_servo4_us",
    "mav_nvf_yaw_i",
    "mav_nvf_yaw_out",
    "mav_nvf_yff_trim",
    "mav_nvf_yff_u",
    "mav_nvf_yff_gz",
    "mav_nvf_yff_a",
)
