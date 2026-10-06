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
- `SERVO9_FUNCTION=36`;
- `H_YAW_TRIM=0`;
- output 9 observed off;
- neutral swash output from Lua's disarmed mode-0 RC overrides.

`calibrate.hw.verify_safe_off()` is the shared read-only acceptance check used
after production cleanup and by the SITL passive acceptance test. It returns
the heartbeat, parameter, and output-9 results as one typed report; do not
duplicate those assertions in individual startup tests.

The DDFP mapping remains configured across boot and cleanup. ArduPilot forces
DDFP to minimum output in `SHUT_DOWN`, `GROUND_IDLE`, and `SPOOLING_DOWN`.
Do not unassign or restore `SERVO9_FUNCTION` at runtime: Copter 4.7.1 does not
rebuild the live `SRV_Channels` function map for those parameter writes.

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

Lua owns the channel-8 motor-interlock override. It actively refreshes the low
PWM override (1000 us) while disarmed and for 500 ms after arming, then asserts
the high override (2000 us). Using a zero override here is incorrect because
ArduPilot interprets it as releasing the override, which can expose a high
physical/default input and reject arming with `Motor Interlock Enabled`. The
500 ms low interval provides at least one ACRO ground-idle controller interval
before runup.

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

Independently of landed state, `ModeGuided::angle_control_start()` initialises
the GUIDED angle target to level roll/pitch at the current yaw, and Copter
ignores `SET_ATTITUDE_TARGET` until the vehicle is already in a guided mode.
The interval between the mode change and the first ground attitude target
therefore commands level. Away from level attitude this produces a swash
transient; clearing landed state alone does not remove it.

Armed ACRO provides a stock-firmware route to clear the flag before the mode
switch. With spool state `THROTTLE_UNLIMITED`, traditional-heli land detection
clears `land_complete` when collective exceeds:

```text
zero_thrust_pct + 0.5 * (H_COL_HOVER - zero_thrust_pct)
```

where `zero_thrust_pct` maps `H_COL_ZERO_THRST` from the configured
`H_COL_ANG_MIN..H_COL_ANG_MAX` range into normalized collective. ACRO also
clears the flag when collective is above its lower limit.

## Passive bench startup

Programmatic callers use `calibrate.run.run_passive()` with
`PassiveRunOptions`. The interactive `run passive` command and the SITL
acceptance test both enter the same production orchestration; tests inject only
their artifact directory rather than monkeypatching calibration globals.

The intended stock-firmware sequence is:

1. Verify `H_FLYBAR_MODE=1` and `H_SV_MAN=0`.
2. Verify the boot-established `SERVO9_FUNCTION=36` mapping, select
   `RAWES_MODE=0`, clear `H_YAW_TRIM`, and select ACRO.
3. Arm, allow Lua's 500 ms ground-idle interval, assert interlock, and wait for
   ArduPilot's explicit `Runup Complete`.
4. Briefly use RAWES ACRO-manual mode with
   zero roll, zero pitch, and the passive collective. Lua owns these continuous
   RC overrides; calibration owns the sequence and telemetry gates.
5. Require `EXTENDED_SYS_STATE.landed_state=MAV_LANDED_STATE_IN_AIR`.
6. Send the Lua-handled `ENTER_GUIDED` command (COMMAND_LONG 31010). In one
   scripting tick Lua captures the current quaternion, switches to
   GUIDED_NOGPS, and installs that attitude with zero body rates and the IC
   thrust; it holds that target (re-sent on change or as a 1 s keepalive) until passive is enabled. Ground
   never sends `DO_SET_MODE` for this transition, so ArduPilot's level entry
   target is replaced before the next attitude-controller run.
   `GUID_OPTIONS` bit 3 must be set.
7. Require active heartbeat plus a quiet attitude interval while Lua continues
   the guided-entry hold.
8. Send the passive thrust and relative offsets, set `RAWES_MODE=3`, then send
   `ENTER_PASSIVE` (COMMAND_LONG 31011, param1 = yaw-trim seed). Lua captures
   the anchor, enables passive hold, and returns the COMMAND_ACK.
9. Lua replaces the temporary runup collective with neutral fallback RC
   collective before its first steady target call.

When the yaw motor or ESC is disconnected, physical yaw movement cannot close
the held-heading loop. A stationary body can still have substantial heading
error and therefore a persistent Motor4 demand. The yaw-trim observer reads
that applied-function demand and can drive `H_YAW_TRIM` to its clamp even
though measured yaw rate is near zero. Disconnected runs may verify command
generation, but must not be used to tune the observer. Never reconnect the
motor while armed or with accumulated trim; canonical safe-off clears
`H_YAW_TRIM` before reconnection.

The LinkHub UI stationary bench route must send `ENTER_PASSIVE` with
param1 = 0. A supplied seed makes Lua hold that trim during passive instead
of entering the adaptive observer fallback. This prevents a small persistent
AP yaw correction from being absorbed into trim while the actuator is
disconnected. The AP yaw PID remains active; zero trim does not promise an
exactly zero transient Motor4 command.

Lua must continue sampling and publishing the applied Motor4 readback
(`YFF_U`) while holding a passive seed. Returning before that readback makes
the ground display retain a pre-passive value and can falsely imply an active
motor command. Ground displays must mark `YFF_U` inactive while disarmed.

Lua does not poll `vehicle:get_mode()` during ACRO staging. Calibration verifies
the actual ArduPilot mode from heartbeats. Once mode 3 is active, Lua keeps
neutral roll, pitch, collective, and yaw RC overrides refreshed even in
GUIDED_NOGPS, so an unexpected return to ACRO never exposes released inputs.

On Copter 4.7.1, a runtime `SERVO9_FUNCTION` write changes the stored parameter
but does not rebuild `SRV_Channels`' function-to-physical-channel map. That map
is populated by `SRV_Channels::update_aux_servo_function()` during
`Copter::init_rc_out()`. Keep function 36 configured at boot instead of trying
to transfer ownership during the handoff.

Approval of a connected yaw motor still requires the passive bench startup test
to prove:

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
- `RAWES_MODE=2`;
- normalized roll, pitch, and collective seeded before use.

Roll and pitch are always ordered `roll, pitch`. Calibration owns mode and
landed-state validation; Lua applies each received RC value immediately and
refreshes the short-lived overrides. Entering passive replaces the temporary
runup collective with neutral collective rather than releasing RC1-RC3. Exit
through the common safe-off path.

## SITL scripting-thread starvation

SITL-only lockstep scheduling behavior (not a hardware arm/disarm fact) is documented in [sitl_testing.md](sitl_testing.md#sitl-scripting-thread-starvation).

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
