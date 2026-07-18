"""debug_guided_step.py -- step-by-step validation of GuidedAttitudeController.

One-off diagnostic, not a unit test.

Run:
    .venv\Scripts\python.exe tests/oneoff/debug_guided_step.py

Build up from the simplest possible scenario:
  Stage 1 - controller at equilibrium with zero gyro -> must produce zero tilt
  Stage 2 - small pitch error -> must produce corrective tilt with correct sign
  Stage 3 - use actual IC state from steady_state_starting.json, verify small output
  Stage 4 - closed-loop 1s sim from IC, check tilt and angle stay bounded
"""
from __future__ import annotations
import sys, math, pathlib
import numpy as np
from scipy.spatial.transform import Rotation


from arduloop import GuidedAttitudeController, HeliParams, RateAxisParams
from arduloop.guided import GuidedAttitudeParams

DT = 2.5e-3   # 400 Hz

# ── Build heli params matching test_steady_flight.py ──────────────────────────
rp = RateAxisParams(P=0.67, I=0.15, D=0.02, IMAX=0.30, FLTT=40.0, FLTE=0.0, FLTD=40.0)
hp = HeliParams()
hp.roll = rp
hp.pitch = rp


def make_ctrl() -> GuidedAttitudeController:
    return GuidedAttitudeController(hp)


def q_from_R(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_quat()


# ── Stage 1: equilibrium -- target == body, gyro == 0 ─────────────────────────
print("=" * 60)
print("STAGE 1: body at equilibrium, target == body, zero gyro")
print("Expected: tilt_lon ~ 0, tilt_lat ~ 0")
print()

# Use a realistic tilted body: elevation 65 deg, yaw = 2.14 rad (from IC)
R_eq = Rotation.from_euler('ZYX', [2.14, -0.76, -0.71]).as_matrix()
q_eq = q_from_R(R_eq)
gyro_zero = np.zeros(3)

ctrl = make_ctrl()
# Set target == body (perfect equilibrium)
ctrl.set_target_rotation(R_eq, sim_time=0.0)

out = ctrl.update(q_eq, gyro_zero, DT, sim_time=0.0)
tlon = -out.pitch_cyclic   # sign convention from step_guided
tlat =  out.roll_cyclic
print(f"  tilt_lon = {tlon:+.6f}  (expect ~0)")
print(f"  tilt_lat = {tlat:+.6f}  (expect ~0)")
assert abs(tlon) < 0.01, f"FAIL: tlon={tlon:.4f} too large"
assert abs(tlat) < 0.01, f"FAIL: tlat={tlat:.4f} too large"
print("  [PASS]")


# ── Stage 2: small pitch error -- body needs to pitch DOWN ─────────────────────
print()
print("=" * 60)
print("STAGE 2: body pitched 5 deg nose-UP from target")
print("Expected: tilt_lon > 0 (nose-down correction)")
print()

# Body has 5 deg more nose-up (negative pitch in ZYX) than target
R_target = R_eq.copy()
R_body   = Rotation.from_euler('ZYX', [2.14, -0.76 - math.radians(5), -0.71]).as_matrix()
q_body_s2 = q_from_R(R_body)

ctrl2 = make_ctrl()
ctrl2.set_target_rotation(R_target, sim_time=0.0)
out2 = ctrl2.update(q_body_s2, gyro_zero, DT, sim_time=0.0)
tlon2 = -out2.pitch_cyclic
tlat2 =  out2.roll_cyclic
print(f"  tilt_lon = {tlon2:+.6f}  (expect > 0)")
print(f"  tilt_lat = {tlat2:+.6f}  (expect ~0)")
if tlon2 > 0.001:
    print("  [PASS] sign correct")
else:
    print("  [FAIL] wrong sign or too small")


# ── Stage 3: actual IC state from steady_state_starting.json ──────────────────
print()
print("=" * 60)
print("STAGE 3: actual IC state -- check initial tilt from guided controller")
print()

import json
ic_path = pathlib.Path(__file__).resolve().parents[2] / "steady_state_starting.json"
with open(ic_path) as f:
    ic_data = json.load(f)

pos_ic  = np.array(ic_data["pos"])
vel_ic  = np.array(ic_data["vel"])
R0_list = ic_data["R0"]
R_ic    = np.array(R0_list)
q_ic    = q_from_R(R_ic)

from simulation.controller import compute_bz_altitude_hold, damp_bz_eq_lateral
tlen    = float(np.linalg.norm(pos_ic))
el_rad  = math.asin(max(-1.0, min(1.0, -pos_ic[2] / max(tlen, 0.1))))
tension_ic = float(ic_data.get("tension_ic", 300.0))

bz1 = compute_bz_altitude_hold(pos_ic, el_rad, tension_ic, 2.0)
bz_goal = damp_bz_eq_lateral(bz1, pos_ic, vel_ic, np.zeros(3), tension_ic, 50.0)

yaw_rad = float(np.arctan2(R_ic[1, 0], R_ic[0, 0]))
print(f"  IC pos: {pos_ic}")
print(f"  tether length: {tlen:.2f} m, elevation: {math.degrees(el_rad):.1f} deg")
print(f"  bz_goal:  {np.round(bz_goal, 4)}")
print(f"  bz_now:   {np.round(R_ic[:, 2], 4)}")

angle_err = math.degrees(math.acos(float(np.clip(np.dot(bz_goal, R_ic[:, 2]), -1, 1))))
print(f"  bz angle error: {angle_err:.3f} deg")

# Build R_target from bz_goal + current yaw
cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
bz = bz_goal / float(np.linalg.norm(bz_goal))
bz_fwd   =  cy * bz[0] + sy * bz[1]
bz_right = -sy * bz[0] + cy * bz[1]
bz_down  =  bz[2]
pitch_t  = float(np.arctan2(bz_fwd, bz_down))
roll_t   = float(np.arcsin(np.clip(-bz_right, -1.0, 1.0)))
R_target_s3 = Rotation.from_euler('ZYX', [yaw_rad, pitch_t, roll_t]).as_matrix()

# Verify R_target[:,2] matches bz_goal
bz_check = R_target_s3[:, 2]
check_err = math.degrees(math.acos(float(np.clip(np.dot(bz_goal, bz_check), -1, 1))))
print(f"  R_target[:,2] vs bz_goal: {check_err:.4f} deg (expect <0.01)")

# Full 3D angle between R_target and R_ic
q_target_s3 = q_from_R(R_target_s3)
err_3d = float(np.degrees((Rotation.from_quat(q_ic).inv() * Rotation.from_quat(q_target_s3)).magnitude()))
print(f"  Full 3D orientation error (R_target vs R_ic): {err_3d:.3f} deg")

ctrl3 = make_ctrl()
ctrl3.set_target_rotation(R_target_s3, sim_time=0.0)
out3 = ctrl3.update(q_ic, gyro_zero, DT, sim_time=0.0)
tlon3 = -out3.pitch_cyclic
tlat3 =  out3.roll_cyclic
print(f"  Initial tilt_lon = {tlon3:+.6f}")
print(f"  Initial tilt_lat = {tlat3:+.6f}")
print(f"  _ang_vel_target: {np.round(ctrl3._ang_vel_target, 5)}")

if abs(tlon3) < 0.05 and abs(tlat3) < 0.05:
    print("  [PASS] initial tilt bounded < 0.05")
else:
    print("  [WARN] large initial tilt -- this may cause divergence")


# ── Stage 4: 1s closed-loop from IC with locked orientation (no physics) ───────
print()
print("=" * 60)
print("STAGE 4: 1s closed-loop with constant orientation (no physics)")
print("  Controller should damp to zero tilt as error is corrected.")
print()

ctrl4 = make_ctrl()
ctrl4.set_target_rotation(R_target_s3, sim_time=0.0)
gyro_sim = np.zeros(3)
tlon_hist, tlat_hist = [], []

for step in range(400):   # 1 s
    t = step * DT
    if step % 8 == 0:     # 50 Hz target update
        ctrl4.set_target_rotation(R_target_s3, sim_time=t)
    out4 = ctrl4.update(q_ic, gyro_sim, DT, sim_time=t)
    tlon_hist.append(-out4.pitch_cyclic)
    tlat_hist.append(out4.roll_cyclic)

tlon_arr = np.array(tlon_hist)
tlat_arr = np.array(tlat_hist)
print(f"  tlon: t=0  {tlon_arr[0]:+.5f}  t=0.1s {tlon_arr[40]:+.5f}  t=0.5s {tlon_arr[200]:+.5f}  t=1s {tlon_arr[-1]:+.5f}")
print(f"  tlat: t=0  {tlat_arr[0]:+.5f}  t=0.1s {tlat_arr[40]:+.5f}  t=0.5s {tlat_arr[200]:+.5f}  t=1s {tlat_arr[-1]:+.5f}")
tlon_max = float(np.max(np.abs(tlon_arr)))
tlat_max = float(np.max(np.abs(tlat_arr)))
print(f"  max |tlon| = {tlon_max:.5f},  max |tlat| = {tlat_max:.5f}")

if max(tlon_max, tlat_max) < 0.1:
    print("  [PASS] tilt stays bounded")
elif max(tlon_max, tlat_max) < 0.3:
    print("  [WARN] moderate tilt, may cause instability in real physics")
else:
    print("  [FAIL] large tilt even with frozen orientation")


# ── Stage 5: same closed-loop but with non-zero gyro matching ang_vel_target ──
print()
print("=" * 60)
print("STAGE 5: same as stage 4 but gyro tracks ang_vel_target (ideal rate tracking)")
print("  Rate PIDs see zero error -> tilt should be purely from outer P")
print()

ctrl5 = make_ctrl()
ctrl5.set_target_rotation(R_target_s3, sim_time=0.0)
tlon_hist5, tlat_hist5 = [], []

for step in range(400):
    t = step * DT
    if step % 8 == 0:
        ctrl5.set_target_rotation(R_target_s3, sim_time=t)
    # Feed back ang_vel_target as gyro (ideal rate tracking)
    gyro_ff = ctrl5._ang_vel_target.copy()
    out5 = ctrl5.update(q_ic, gyro_ff, DT, sim_time=t)
    tlon_hist5.append(-out5.pitch_cyclic)
    tlat_hist5.append(out5.roll_cyclic)

tlon_arr5 = np.array(tlon_hist5)
tlat_arr5 = np.array(tlat_hist5)
print(f"  tlon max = {np.max(np.abs(tlon_arr5)):.5f},  tlat max = {np.max(np.abs(tlat_arr5)):.5f}")
if max(np.max(np.abs(tlon_arr5)), np.max(np.abs(tlat_arr5))) < 0.05:
    print("  [PASS]")
else:
    print("  [WARN] larger than expected even with ideal rate tracking")

print()
print("Done.")
