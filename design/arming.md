# Arming, Disarming, and Hardware Safe-Off

This is the primary owner document for RAWES arm/disarm behavior. It covers
hardware and SITL startup sequences, safe-off requirements, traditional-heli
ArduPilot behavior that affects those sequences, and the evidence required
before changing them.

`HARDWARE_STARTUP.md` is the chronological hardware-session log. Record ports,
commands, measurements, and final state there after a hardware session, but
update this document whenever a finding changes the current arm/disarm model or
procedure.

The ArduPilot source facts below are verified against Copter 4.7.1,
`dbe792162d06cab66c3475fd5556bf7a120f119e`.

## Safety invariants

Every normal disarm, forced disarm, failed startup, calibration exit, and test
cleanup must converge through `calibrate.hw._set_safe_off_state()` to:

- disarmed, confirmed from heartbeat;
- ArduPilot ACRO;
- `RAWES_MODE=0`;
- `H_FLYBAR_MODE=1`;
- `H_SV_MAN=0`;
- `SERVO9_FUNCTION=0`;
- `H_YAW_TRIM=0`;
- neutral swash output from Lua's disarmed mode-0 RC overrides.

`SERVO9_FUNCTION` must not be restored after cleanup. A later passive run takes
ownership explicitly only after its attitude handoff is safe.

Disarmed does not by itself mean that the swash is neutral or that an output is
unassigned. Verify the complete invariant. Do not use historical port or baud
values: the first hardware operation in a conversation must run
`python -m calibrate` without connection arguments and reuse the detected
native Pixhawk USB connection only for that session.

## Ground-side arm and disarm

Normal arming uses `MAV_CMD_COMPONENT_ARM_DISARM` with the ordinary ArduPilot
pre-arm checks. `calibrate arm --force` is an explicit operator choice that
sends the ArduPilot force-arm magic value `21196`; it is appropriate for a
secured bench when a known check such as compass alignment cannot pass.

Force arm bypasses pre-arm checks. It does not bypass traditional-heli spool,
interlock, or runup state. Treat command acknowledgement, heartbeat armed
state, interlock assertion, and `Runup Complete` as distinct facts.

Disarm behavior is:

1. Set `RAWES_MODE=0`.
2. Request normal MAVLink disarm.
3. Confirm disarmed heartbeat.
4. If normal disarm is rejected, issue the bounded force-disarm fallback.
5. Apply and verify the complete safe-off invariant.

Cleanup must run after partial startup failures as well as successful runs.

## Traditional-heli facts relevant to startup

### Interlock and runup

Lua owns the channel-8 motor-interlock override. After arming it currently
holds interlock low for 500 ms, then asserts it. This provides at least one
ACRO ground-idle controller interval before runup.

`H_RSC_RAMP_TIME` and `H_RSC_RUNUP_TIME` are timeout inputs, not proof that
runup completed. Startup must wait for ArduPilot's `Runup Complete` status.

### Flybar mode

`H_FLYBAR_MODE=1` is the required RAWES configuration. In ACRO it enables
mechanical-flybar-style roll/pitch passthrough:

- pilot roll and pitch go directly to the swash;
- the heli controller zeros roll/pitch attitude error and refreshes Euler
  targets from AHRS;
- it does not reconstruct the internal quaternion attitude target.

Leaving ACRO clears flybar passthrough. Do not temporarily switch to
`H_FLYBAR_MODE=0` during an armed startup; that changes the flight-control
configuration rather than fixing the handoff producer.

### Manual swash mode

`H_SV_MAN` is a setup mechanism, not a flight output-inhibit API:

| Value | ArduPilot behavior |
|---:|---|
| 0 | Automated flight control |
| 1 | Pilot-input passthrough |
| 2 | Maximum collective |
| 3 | Zero cyclic and zero-thrust collective |
| 4 | Minimum collective |
| 5 | Oscillation test |

ArduPilot documents nonzero `H_SV_MAN` values as setup-only and requires zero
for flight. `H_SV_MAN=3` cannot precondition GUIDED_NOGPS while disarmed
because GUIDED exits before running its attitude controller. Do not use an
armed `H_SV_MAN` transition as the passive handoff unless a future change
provides and validates a supported output-gating contract.

### Landed state and the GUIDED_NOGPS transient

ArduPilot unconditionally sets `ap.land_complete=true` while disarmed and on
disarm. Copter's `EXTENDED_SYS_STATE.landed_state` is an exact public
projection:

- `MAV_LANDED_STATE_ON_GROUND` when `ap.land_complete` is true;
- landing/takeoff states for the corresponding modes;
- `MAV_LANDED_STATE_IN_AIR` otherwise.

In GUIDED angle control, an armed vehicle with positive commanded thrust and
`land_complete=true` enters the takeoff branch. That branch calls
`zero_throttle_and_relax_ac()`. Despite its name, the helper commands level
roll and pitch, so a stationary RAWES airframe near inverted can receive an
attitude target roughly 180 degrees from its actual attitude. Once heli spool
state reaches `THROTTLE_UNLIMITED`, GUIDED clears `land_complete`; subsequent
zero-rate commands preserve the newly installed level target.

Disarmed GUIDED_NOGPS settling cannot consume this branch because
`ModeGuided::angle_control_run()` returns immediately while disarmed.

Armed ACRO provides a stock-firmware route to clear the flag before the mode
switch. With spool state `THROTTLE_UNLIMITED`, traditional-heli land detection
clears `land_complete` when collective exceeds:

```text
zero_thrust_pct + 0.5 * (H_COL_HOVER - zero_thrust_pct)
```

where `zero_thrust_pct` maps `H_COL_ZERO_THRST` from the configured
`H_COL_ANG_MIN..H_COL_ANG_MAX` range into normalized collective. ACRO also
clears the flag when collective is above its lower limit. A stationary bench
can reassert landed state after approximately one second if all landing
criteria become true again.

## Passive bench startup

The intended stock-firmware sequence is:

1. Verify `H_FLYBAR_MODE=1` and `H_SV_MAN=0`.
2. Keep `SERVO9_FUNCTION=0`; select `RAWES_MODE=0` and ACRO.
3. Arm, allow Lua's 500 ms ground-idle interval, assert interlock, and wait for
   ArduPilot's explicit `Runup Complete`.
4. Capture the current attitude, then briefly use RAWES ACRO-manual mode with
   zero roll, zero pitch, and the passive collective. Lua owns these continuous
   RC overrides; calibration owns the sequence and telemetry gates.
5. Require `EXTENDED_SYS_STATE.landed_state=MAV_LANDED_STATE_IN_AIR`.
6. Immediately turn off the yaw motor output, enter GUIDED_NOGPS, and have
   calibration stream the captured quaternion, zero body rates, and normalized
   thrust through `SET_ATTITUDE_TARGET`. `GUID_OPTIONS` bit 3 must be set.
7. Require active heartbeat plus a quiet attitude interval while calibration
   continues that target stream.
8. Send the passive thrust and relative offsets, send `RAWES_PEN=1`, then set
   `RAWES_MODE=3`. Calibration continues streaming until Lua confirms both
   anchor capture and passive mode entry.
9. Lua replaces the temporary runup collective with neutral fallback RC
   collective before its first steady target call. Restore yaw-motor ownership
   only after Lua acknowledges steady-state control.

Lua does not poll `vehicle:get_mode()` during ACRO staging. Calibration verifies
the actual ArduPilot mode from heartbeats. Once mode 3 is active, Lua keeps
neutral roll, pitch, collective, and yaw RC overrides refreshed even in
GUIDED_NOGPS, so an unexpected return to ACRO never exposes released inputs.

As of 2026-10-05, this sequence is implemented for SITL validation but is not yet
approved for connected hardware. Approval requires the passive bench startup
SITL test to prove:

- landed state clears before the mode change;
- no level target is introduced;
- attitude-target error stays within the test gate;
- the first and subsequent swash steps satisfy the smoothness limits;
- yaw PID terms participate with the correct opposing sign;
- motor correction is smooth and stable rather than relying on yaw trim.

## ACRO manual startup

ACRO manual operation requires:

- `H_FLYBAR_MODE=1`;
- `H_SV_MAN=0`;
- `IM_ACRO_COL_EXP=0`;
- - `RAWES_MODE=2`;
- normalized roll, pitch, and collective seeded before use.

Roll and pitch are always ordered `roll, pitch`. Calibration owns mode and
landed-state validation; Lua applies each received RC value immediately and
refreshes the short-lived overrides. Entering passive replaces the temporary
runup collective with neutral collective rather than releasing RC1-RC3. Exit
through the common safe-off path.

## SITL scripting-thread starvation

The passive torque regression can expose a lockstep-SITL artifact that is
separate from the landed-state GUIDED handoff. ArduPilot 4.7.1 marks every
generated `AP_Vehicle` Lua binding `scheduler-semaphore`. The main scheduler
holds that semaphore for the complete flight-control loop and releases it only
while waiting for the next INS sample.

`SCR_DEBUG_OPTS=8`, Lua phase timing, and Linux `/proc` thread tracing proved
that the scripting thread can wake once per 2.5 ms frame yet repeatedly lose
that short semaphore-acquisition window. The long calls were exclusively
`vehicle:get_mode()` and `vehicle:set_target_*()`:

- a reproduced `vehicle:get_mode()` wait lasted 3.307 s;
- the scripting thread slept in `futex_wait_queue` rather than consuming CPU
  or waiting runnable;
- the main loop remained healthy at 400 Hz and lockstep stayed near 1.0x;
- after 3.110 s without a Lua refresh, RC4 and RC8 overrides expired together;
- output 8 dropped from 2000 to 1000, heli runup restarted, and swash spread
  reached 175 us;
- both overrides recovered immediately after the vehicle binding returned.

This evidence identifies scheduler-semaphore starvation as the cause of that
SITL output interruption. It does not replace the separately verified hardware
theory: entering armed GUIDED with landed state set installs a level target.
Do not hide the SITL failure by increasing or disabling `RC_OVERRIDE_TIME`;
that weakens a safety timeout without fixing the blocked scripting thread.

The passive path avoids the startup exposure by having calibration stream
`SET_ATTITUDE_TARGET` until Lua activation. After activation, fixed passive hold
does not poll `vehicle:get_mode()` and refreshes its one
`vehicle:set_target_angle_and_rate_and_throttle()` call at 20 Hz instead of
making two scheduler-locked vehicle calls at 100 Hz. RC fallback still refreshes
at 100 Hz before that call. The end-to-end regression passed with this
mitigation on 2026-10-05. This reduces exposure without weakening either
`RC_OVERRIDE_TIME` or GUIDED's target timeout; it does not change the underlying
SITL mutex fairness.

## Evidence and tests

Use LinkHub as the sole MAVLink owner and evidence source. Its `.lhc` journal
contains RX, TX, host ingest time, and simulation clock in one global order.
Use `linkhub query`; do not create duplicate MAVLink JSONL logs.

For sparse live reads, request only the needed message types. LinkHub advances
the cursor across irrelevant records in bounded scans. `collapse=true` may be
used for consumers that only need current recognized state; calibration CSV and
smoothness analysis must consume uncollapsed batches so no telemetry samples
are lost.

The end-to-end regression owner is
`tests/sitl/torque/test_passive_bench_startup_torque_sitl.py`. Every SITL test
under `tests/sitl/**` must run through `bash test.sh stack`; use
`bash test.sh stack -n 1 -k test_passive_bench_startup_torque_sitl` for this
single test. A torque fixture that replaces the normal heli parm chain must
explicitly boot the hardware `H_FLYBAR_MODE`, `IM_ACRO_COL_EXP`, and collective
geometry; ArduPilot defaults do not represent the RAWES hardware handoff.

Before any hardware retest:

1. pass focused unit and Lua tests;
2. pass the end-to-end passive bench startup SITL test;
3. inspect the LinkHub journal and recovered DataFlash;
4. document the expected actuator envelope;
5. physically disconnect the rotor and secure the swash mechanism;
6. auto-detect the native USB connection;
7. verify safe-off before and after the run.

## Documentation update rule

Update this document in the same change whenever investigation or source
inspection changes any critical fact about:

- ArduPilot arming, disarming, land detection, interlock, runup, or mode entry;
- `H_FLYBAR_MODE`, `H_SV_MAN`, swash output ownership, or motor assignment;
- force-arm or force-disarm behavior;
- passive, ACRO-manual, or cleanup sequencing;
- required telemetry gates, tests, or safe-off state.

Do not leave a changed arm/disarm rule only in a test, code comment, pull
request discussion, or hardware-session log.
