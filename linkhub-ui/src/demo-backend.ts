import { DIAG_ARRAY_ID, DIAG_ARRAY_NAME, DIAG_KEYS } from "./diag-array";
import { demoPose } from "./demo-flight";
import {
  Attitude, AttitudeQuaternion, AttitudeTarget, BatteryStatus, CommandAck, CommandLong,
  DebugFloatArray, EkfStatusReport, EscTelemetry1To4, EscTelemetry9To12, ExtendedSysState,
  Heartbeat, LocalPositionNed, MagCalProgress, MagCalReport, MagCalStatus, MavAutopilot,
  MavBatteryFunction, MavBatteryType, MavCmd, MavLandedState, MavModeFlag, MavParamType,
  MavResult, MavSeverity, MavState, MavType, MavVtolState, PidTuning, PidTuningAxis,
  RcChannels, Rpm, ServoOutputRaw, Statustext, Vibration,
  type CommandLongFields, type HeartbeatFields, type MavlinkMessage,
} from "./generated/protocol";
import { enumName, mavEnum } from "./mav";
import { TELEMETRY_PROFILES } from "./telemetry-rates";
import type { JsonObject, LinkHubStatus, MessageRecord, ParameterResult } from "./types";
import { MOTOR_TO_ROTOR_GEAR_RATIO } from "./vehicle-motion";

const DEFAULT_RATES: Record<string, number> = {
  ...TELEMETRY_PROFILES.wifi.rates,
  HEARTBEAT: 1, EXTENDED_SYS_STATE: 2, DEBUG_FLOAT_ARRAY: 20,
  ESC_TELEMETRY_9_TO_12: 10,
};
const MESSAGE_IDS: Record<number, string> = {
  0: "HEARTBEAT", 30: "ATTITUDE", 31: "ATTITUDE_QUATERNION",
  32: "LOCAL_POSITION_NED", 36: "SERVO_OUTPUT_RAW", 83: "ATTITUDE_TARGET",
  191: "MAG_CAL_PROGRESS", 192: "MAG_CAL_REPORT", 226: "RPM",
};
const MAX_JOURNAL_RECORDS = 60_000;

class DemoRequestError extends Error {}

function object(value: unknown): JsonObject {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    throw new DemoRequestError("Expected a JSON object");
  }
  return value as JsonObject;
}

function finite(value: unknown): number {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new DemoRequestError("Expected a finite number");
  }
  return value;
}

// Mirror LinkHub's records.rs state keys for the messages this stand-in emits.
function snapshotKey(record: MessageRecord): string | null {
  const keyedField = {
    NAMED_VALUE_FLOAT: "name", NAMED_VALUE_INT: "name", DEBUG_VECT: "name",
    DEBUG_FLOAT_ARRAY: "array_id", PID_TUNING: "axis", BATTERY_STATUS: "id",
  }[record.message];
  const discriminator = keyedField ? record.fields[keyedField] : undefined;
  if (keyedField ? discriminator === undefined : !Object.hasOwn(DEFAULT_RATES, record.message)) {
    return null;
  }
  return JSON.stringify([
    record.direction, record.system_id, record.component_id, record.message, discriminator,
  ]);
}

function sleep(ms: number, signal?: AbortSignal | null): Promise<void> {
  signal?.throwIfAborted();
  return new Promise((resolve, reject) => {
    const cleanup = () => signal?.removeEventListener("abort", abort);
    const timer = globalThis.setTimeout(() => { cleanup(); resolve(); }, ms);
    const abort = () => {
      globalThis.clearTimeout(timer);
      cleanup();
      reject(signal?.reason ?? new DOMException("Aborted", "AbortError"));
    };
    signal?.addEventListener("abort", abort, { once: true });
  });
}

/** In-memory LinkHub HTTP boundary; never forwards requests to a network. */
export class DemoBackend {
  private readonly parameters = new Map<string, ParameterResult>();
  private readonly rates = new Map<string, number>(Object.entries(DEFAULT_RATES));
  private readonly lastSample = new Map<string, number>();
  private journal: MessageRecord[] = [];
  private readonly started: number;
  private updated = 0;
  private generation = 1;
  private sequence = 0;
  private armed = true;
  private mode = 20;
  private flightOffset = 0;
  private flightStarted = 0;
  private thrust = 0.342;
  private readonly offsets = { roll: 0, pitch: 0, yaw: 0 };
  private calibration: {
    start: number; mask: number; autosave: boolean; reported: boolean; lastProgress: number;
  } | null = null;
  private calibrationSaved = false;
  private rxCount = 0;
  private txCount = 0;

  constructor(
    defaults: ReadonlyMap<string, number>,
    private readonly clock: () => number = () => performance.now(),
  ) {
    this.started = clock();
    const values = new Map([
      ["GUID_OPTIONS", 8], ["H_RSC_RAMP_TIME", 1], ["H_RSC_RUNUP_TIME", 1],
      ["H_YAW_TRIM", 0], ["H_SV_MAN", 0],
      ["ARMING_SKIPCHK", 0], ["BRD_SAFETY_DEFLT", 0], ["SCR_ENABLE", 1],
      ["FS_THR_ENABLE", 0], ["FS_GCS_ENABLE", 0],
      ...defaults,
      ["RAWES_MODE", 3], ["RAWES_TEL_HZ", 20],
    ]);
    for (const [name, value] of values) {
      this.parameters.set(name, {
        name, value, type: mavEnum(Number.isInteger(value) ? MavParamType.INT32 : MavParamType.REAL32),
      });
    }
    this.text("OFFLINE DEMO: synthetic vehicle only. No hardware or network connection.");
    this.text("Try status, stop, run passive, config check, compass calibrate, and the Yaw/Wobble panels.");
    this.emitFrame(0);
  }

  private now(): number {
    return Math.max(0, this.clock() - this.started);
  }

  private get cursor(): string {
    return `v1:${this.sequence}`;
  }

  private get token(): string {
    return `demo:${this.generation}`;
  }

  private value(name: string): number {
    const parameter = this.parameters.get(name);
    if (!parameter) {
      throw new Error(`Unknown demo parameter ${name}`);
    }
    return parameter.value;
  }

  private append(message: string, fields: JsonObject, time = this.now(), direction: "rx" | "tx" = "rx"): void {
    this.sequence += 1;
    if (direction === "rx") this.rxCount += 1;
    else this.txCount += 1;
    this.journal.push({
      message, fields, direction, system_id: direction === "rx" ? 1 : 255, component_id: 1,
      cursor: this.cursor, received_time: new Date(time).toISOString(),
      received_time_ns: Math.round(time * 1_000_000),
      sim_clock: { epoch: this.generation, time_boot_ms: Math.round(time), quality: "synthetic" },
    });
  }

  private text(text: string): void {
    this.appendMessage(new Statustext({ severity: mavEnum(MavSeverity.INFO), text }));
  }

  private appendMessage<TFields extends object>(
    message: MavlinkMessage<TFields>,
    time = this.now(),
    direction: "rx" | "tx" = "rx",
  ): void {
    this.append(message.message, Object.fromEntries(Object.entries(message.fields)), time, direction);
  }

  private heartbeat(): HeartbeatFields {
    return {
      mavtype: mavEnum(MavType.HELICOPTER), autopilot: mavEnum(MavAutopilot.ARDUPILOTMEGA),
      base_mode: [MavModeFlag.CUSTOM_MODE_ENABLED, ...(this.armed ? [MavModeFlag.SAFETY_ARMED] : [])].join(" | "),
      custom_mode: this.mode, system_status: mavEnum(this.armed ? MavState.ACTIVE : MavState.STANDBY),
      mavlink_version: 3,
    };
  }

  private fields(time: number): Record<string, JsonObject> {
    const flightTime = this.flightOffset + (this.armed ? (time - this.flightStarted) / 1_000 : 0);
    const pose = demoPose(
      Math.max(0, flightTime), this.offsets.roll, this.offsets.pitch, this.offsets.yaw,
      this.value("RAWES_MODE") === 2,
    );
    const boot = Math.round(time);
    const rpm = this.armed ? pose.rotorRpm * this.thrust / 0.342 : 0;
    const throttle = this.armed ? Math.min(0.7, 0.3 + 0.03 * Math.sin(time / 1_000)) : 0;
    const col = this.armed ? this.thrust : 0.5;
    const collective = (this.value("H_COL_MIN") - 1000) / 1000
      + col * (this.value("H_COL_MAX") - this.value("H_COL_MIN")) / 1000;
    const roll = this.armed ? pose.roll * 0.5 : 0;
    const pitch = this.armed ? pose.pitch * 0.5 : 0;
    const pwm = (scaled: number) => Math.round(1000 + Math.max(0, Math.min(1, scaled)) * 1000);
    const diag: Record<string, number> = {
      YFF_T: 0.3, YFF_U: throttle, YFF_GZ: pose.yawspeed,
      OL_RSP: pose.roll, OL_PSP: pose.pitch, OL_YSP: pose.yawspeed + 0.01,
      OL_RER: 0.025, OL_PER: 0.018, OL_YER: 0.035,
      OL_AP: 0.02, OL_AI: 0.01, OL_AD: 0.001, OL_COL: col, OL_TEN: 80,
      ANCH_N: 0, ANCH_E: 0, ANCH_D: 0,
    };
    const frames = [
      new Heartbeat(this.heartbeat()),
      new LocalPositionNed({
        time_boot_ms: boot, x: pose.north, y: pose.east, z: pose.down,
        vx: this.armed ? pose.vn : 0, vy: this.armed ? pose.ve : 0, vz: this.armed ? pose.vd : 0,
      }),
      new AttitudeQuaternion({
        time_boot_ms: boot, q1: pose.attitude.w, q2: pose.attitude.x, q3: pose.attitude.y, q4: pose.attitude.z,
        rollspeed: this.armed ? pose.rollspeed : 0,
        pitchspeed: this.armed ? pose.pitchspeed : 0,
        yawspeed: this.armed ? pose.yawspeed : 0,
      }),
      new Attitude({
        time_boot_ms: boot, roll: pose.roll, pitch: pose.pitch, yaw: pose.yaw,
        rollspeed: pose.rollspeed, pitchspeed: pose.pitchspeed, yawspeed: pose.yawspeed,
      }),
      new AttitudeTarget({
        time_boot_ms: boot, type_mask: "ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE",
        q: [pose.target.w, pose.target.x, pose.target.y, pose.target.z],
        body_roll_rate: 0, body_pitch_rate: 0, body_yaw_rate: pose.yawspeed + 0.01, thrust: this.thrust,
      }),
      new Rpm({ rpm1: rpm * MOTOR_TO_ROTOR_GEAR_RATIO, rpm2: 0 }),
      new ServoOutputRaw({
        time_usec: boot * 1000, port: 0,
        servo1_raw: pwm(collective + Math.sqrt(3) / 2 * 0.45 * roll + 0.225 * pitch),
        servo2_raw: pwm(collective - Math.sqrt(3) / 2 * 0.45 * roll + 0.225 * pitch),
        servo3_raw: pwm(collective - 0.45 * pitch),
        servo4_raw: 0, servo5_raw: 0, servo6_raw: 0, servo7_raw: 0, servo8_raw: 0,
        servo9_raw: this.armed ? Math.round(this.value("SERVO9_MIN")
          + throttle * (this.value("SERVO9_MAX") - this.value("SERVO9_MIN"))) : 0,
      }),
      new DebugFloatArray({
        time_usec: boot * 1000, array_id: DIAG_ARRAY_ID, name: DIAG_ARRAY_NAME,
        data: [(1 << DIAG_KEYS.length) - 1, ...DIAG_KEYS.map((key) => diag[key] ?? 0)],
      }),
      new PidTuning({
        axis: mavEnum(PidTuningAxis.YAW), desired: pose.yawspeed + 0.01, achieved: pose.yawspeed,
        FF: 0.01, P: 0.002 * Math.sin(time / 200), I: 0.005, D: 0.001 * Math.cos(time / 200),
      }),
      new ExtendedSysState({
        vtol_state: mavEnum(MavVtolState.UNDEFINED),
        landed_state: mavEnum(this.armed ? MavLandedState.IN_AIR : MavLandedState.ON_GROUND),
      }),
      new EkfStatusReport({
        flags: "EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT | EKF_POS_HORIZ_ABS | EKF_POS_VERT_ABS",
        velocity_variance: 0.01, pos_horiz_variance: 0.02, pos_vert_variance: 0.02,
        compass_variance: 0.01, terrain_alt_variance: 0,
      }),
      new BatteryStatus({
        id: 0, voltages: [24000, ...Array<number>(9).fill(65535)],
        current_battery: 120, battery_remaining: 94,
        current_consumed: 200, energy_consumed: 17280, temperature: 2200,
        battery_function: mavEnum(MavBatteryFunction.ALL), mavtype: mavEnum(MavBatteryType.LIPO),
      }),
      new RcChannels({
        time_boot_ms: boot, chancount: 9, rssi: 255,
        chan1_raw: 1500, chan2_raw: 1500, chan3_raw: 1500,
        chan4_raw: 1500, chan5_raw: 1500, chan6_raw: 1500,
        chan7_raw: 1500, chan8_raw: 1500, chan9_raw: 1500,
        chan10_raw: 65535, chan11_raw: 65535, chan12_raw: 65535,
        chan13_raw: 65535, chan14_raw: 65535, chan15_raw: 65535,
        chan16_raw: 65535, chan17_raw: 65535, chan18_raw: 65535,
      }),
      new Vibration({
        time_usec: boot * 1000, vibration_x: 0.12, vibration_y: 0.1, vibration_z: 0.08,
        clipping_0: 0, clipping_1: 0, clipping_2: 0,
      }),
      new EscTelemetry1To4({
        voltage: [0, 0, 0, 0], current: [0, 0, 0, 0], totalcurrent: [0, 0, 0, 0],
        rpm: [0, 0, 0, 0], count: [0, 0, 0, 0], temperature: [22, 22, 22, 22],
      }),
      new EscTelemetry9To12({
        voltage: [2400, 0, 0, 0], current: [this.armed ? 120 : 0, 0, 0, 0],
        totalcurrent: [200, 0, 0, 0], rpm: [Math.round(rpm * MOTOR_TO_ROTOR_GEAR_RATIO), 0, 0, 0],
        count: [1, 0, 0, 0], temperature: [22, 0, 0, 0],
      }),
    ];
    return Object.fromEntries(frames.map(({ message, fields }) => [message, { ...fields }]));
  }

  private emitFrame(time: number): void {
    const fields = this.fields(time);
    for (const [message, hz] of this.rates) {
      if (hz > 0 && fields[message]) {
        this.append(message, fields[message], time);
        this.lastSample.set(message, time);
      }
    }
  }

  private advance(): void {
    const now = this.now();
    const pending: { message: string; time: number }[] = [];
    for (const [message, hz] of this.rates) {
      if (hz <= 0) continue;
      const period = 1000 / hz;
      const last = Math.max(this.lastSample.get(message) ?? this.updated, now - 1000);
      for (let time = last + period; time <= now; time += period) {
        pending.push({ message, time });
      }
    }
    pending.sort((a, b) => a.time - b.time);
    let cachedTime = -1;
    let cached: Record<string, JsonObject> = {};
    for (const { message, time } of pending) {
      if (time !== cachedTime) {
        cachedTime = time;
        cached = this.fields(time);
      }
      const fields = cached[message];
      if (fields) this.append(message, fields, time);
      this.lastSample.set(message, time);
    }
    if (this.calibration && now >= this.calibration.start && !this.calibration.reported
      && now - this.calibration.lastProgress >= 500) {
      this.calibration.lastProgress = now;
      const percent = Math.min(100, Math.floor((now - this.calibration.start) / 60));
      for (let id = 0; id < 3; id += 1) {
        if (!(this.calibration.mask & (1 << id))) continue;
        this.appendMessage(new MagCalProgress({
          compass_id: id, cal_mask: this.calibration.mask, attempt: 1,
          cal_status: mavEnum(MagCalStatus.RUNNING_STEP_ONE), completion_pct: percent,
          direction_x: 0, direction_y: 0, direction_z: 1,
          completion_mask: Array<number>(10).fill(percent === 100 ? 255 : 0),
        }));
        if (percent === 100) {
          this.appendMessage(new MagCalReport({
            compass_id: id, cal_mask: this.calibration.mask, cal_status: mavEnum(MagCalStatus.SUCCESS),
            autosaved: this.calibration.autosave ? 1 : 0, fitness: 0.12,
            ofs_x: 1.2, ofs_y: -0.7, ofs_z: 0.4,
            diag_x: 1, diag_y: 1, diag_z: 1,
            offdiag_x: 0, offdiag_y: 0, offdiag_z: 0, orientation_confidence: 0.99,
          }));
        }
      }
      if (percent === 100) {
        this.calibration.reported = true;
        this.calibrationSaved = this.calibration.autosave;
      }
    }
    if (this.journal.length > MAX_JOURNAL_RECORDS) {
      this.journal.splice(0, this.journal.length - MAX_JOURNAL_RECORDS);
    }
    this.updated = now;
  }

  private status(): LinkHubStatus {
    const { base_mode, custom_mode, system_status } = this.heartbeat();
    const rx: Record<string, number> = {};
    const tx: Record<string, number> = {};
    for (const record of this.journal) {
      if (record.received_time_ns / 1e6 < this.now() - 3000) continue;
      const rates = record.direction === "rx" ? rx : tx;
      rates[record.message] = (rates[record.message] ?? 0) + 64 / 3;
    }
    const total = (rates: Record<string, number>) => Object.values(rates).reduce((a, b) => a + b, 0);
    return {
      connected: true, ready: true, connection: "offline-demo", port: null, baud: 1500000,
      clock_epoch: this.generation, target_system: 1, target_component: 1,
      base_mode, custom_mode, system_status,
      latest_time_boot_ms: Math.round(this.now()), received_messages: this.rxCount,
      transmitted_messages: this.txCount, received_bytes: this.rxCount * 64, transmitted_bytes: this.txCount * 64,
      rx_bps: total(rx), tx_bps: total(tx), rx_bps_by_message: rx, tx_bps_by_message: tx,
      cursor: this.cursor, generation: this.token,
    };
  }

  private setParameter(name: string, value: unknown, type: unknown): ParameterResult {
    if (!this.parameters.has(name)) throw new DemoRequestError(`Unknown demo parameter ${name}`);
    const number = finite(value);
    const paramType = Object.values(MavParamType).find((entry) => entry === enumName(type));
    if (paramType === undefined) {
      throw new DemoRequestError("Unknown MAV_PARAM_TYPE");
    }
    const parameter: ParameterResult = {
      name, value: number, type: mavEnum(paramType),
    };
    this.parameters.set(name, parameter);
    if (name === "RAWES_TEL_HZ") {
      this.rates.set("DEBUG_FLOAT_ARRAY", Math.max(0, Math.min(300, number)));
    }
    return parameter;
  }

  private command(body: JsonObject): JsonObject {
    const command = enumName(body.command);
    if (command === null) throw new DemoRequestError("Command must be a MAVLink enum value");
    const wireCommand: CommandLongFields["command"] = { type: command };
    const params = Array.isArray(body.params) ? body.params.map(finite) : [];
    if (params.length > 7) throw new DemoRequestError("COMMAND_LONG accepts at most seven parameters");
    const after = this.cursor;
    this.appendMessage(new CommandLong({
      command: wireCommand, target_system: body.target_system === undefined ? 1 : finite(body.target_system),
      target_component: body.target_component === undefined ? 1 : finite(body.target_component),
      confirmation: 0, param1: params[0] ?? 0, param2: params[1] ?? 0, param3: params[2] ?? 0,
      param4: params[3] ?? 0, param5: params[4] ?? 0, param6: params[5] ?? 0, param7: params[6] ?? 0,
    }), this.now(), "tx");
    let result = MavResult.ACCEPTED;
    switch (command) {
      case MavCmd.COMPONENT_ARM_DISARM: {
        const armed = params[0] === 1;
        if (this.armed && !armed) this.flightOffset += (this.now() - this.flightStarted) / 1000;
        if (!this.armed && armed) this.flightStarted = this.now();
        this.armed = armed;
        this.text(armed ? "DEMO: heli runup complete" : "DEMO: disarmed");
        break;
      }
      case MavCmd.DO_SET_MODE:
        this.mode = params[1] ?? 1;
        break;
      case MavCmd.USER_1:
        this.mode = 20;
        break;
      case MavCmd.USER_2:
        this.parameters.set("RAWES_MODE", { name: "RAWES_MODE", value: 3, type: mavEnum(MavParamType.INT32) });
        break;
      case MavCmd.PREFLIGHT_REBOOT_SHUTDOWN:
        if (this.armed) { result = MavResult.DENIED; break; }
        this.generation += 1;
        this.mode = this.value("INITIAL_MODE");
        this.parameters.set("RAWES_MODE", { name: "RAWES_MODE", value: 0, type: mavEnum(MavParamType.INT32) });
        this.calibration = null;
        this.text("DEMO: reboot complete; synthetic vehicle reconnected");
        break;
      case MavCmd.SET_MESSAGE_INTERVAL: {
        const name = MESSAGE_IDS[params[0] ?? -1];
        if (!name) { result = MavResult.UNSUPPORTED; break; }
        const interval = params[1] ?? 0;
        this.rates.set(name, interval < 0 ? 0 : interval === 0 ? (DEFAULT_RATES[name] ?? 2) : Math.min(300, 1e6 / interval));
        break;
      }
      case MavCmd.DO_START_MAG_CAL:
        if (this.armed) { result = MavResult.DENIED; break; }
        this.calibration = {
          start: this.now() + (params[3] ?? 0) * 1000,
          mask: (params[0] ?? 0) || 1, autosave: params[2] === 1, reported: false, lastProgress: -Infinity,
        };
        this.calibrationSaved = false;
        break;
      case MavCmd.DO_ACCEPT_MAG_CAL:
        if (!this.calibration?.reported || this.calibrationSaved) { result = MavResult.DENIED; break; }
        this.calibrationSaved = true;
        break;
      case MavCmd.DO_CANCEL_MAG_CAL:
        this.calibration = null;
        break;
      default:
        result = MavResult.UNSUPPORTED;
    }
    this.appendMessage(new CommandAck({ command: wireCommand, result: mavEnum(result) }));
    this.emitFrame(this.now());
    return { command: body.command, result: mavEnum(result), status: result === MavResult.ACCEPTED ? "accepted" : "rejected", progress: 100, after_cursor: after };
  }

  readonly fetch: typeof fetch = async (input, init) => {
    const signal = init?.signal ?? (input instanceof Request ? input.signal : undefined);
    signal?.throwIfAborted();
    const url = new URL(input instanceof Request ? input.url : String(input), "https://offline-demo.invalid");
    const method = init?.method ?? "GET";
    this.advance();
    const json = (value: unknown, status = 200) => new Response(JSON.stringify(value), {
      status, headers: { "Content-Type": "application/json" },
    });
    try {
      const body = init?.body === undefined ? {} : object(JSON.parse(String(init.body)));
      const path = url.pathname;
      if (method === "GET" && path === "/v1/status") {
        return json({ service: "LinkHub offline demo", api_version: 1, run_id: this.token, cursor: this.cursor });
      }
      if (method === "GET" && path === "/v1/mavlink/status") return json(this.status());
      if (method === "POST" && path === "/v1/journal/flush") return json({ cursor: this.cursor });
      if (method === "GET" && path === "/v1/mavlink/messages") {
        const expected = url.searchParams.get("expected_generation");
        if (expected && expected !== this.token) {
          return json({ error: "generation_changed", message: "Demo vehicle rebooted", expected_generation: expected, current_generation: this.token, cursor: this.cursor }, 409);
        }
        const match = /^v1:(\d+)$/.exec(url.searchParams.get("after") ?? "");
        if (!match) throw new DemoRequestError("Invalid journal cursor");
        const after = Number(match[1]);
        if (after >= this.sequence && Number(url.searchParams.get("wait_ms")) > 0) {
          await sleep(Math.min(50, Number(url.searchParams.get("wait_ms"))), signal);
          this.advance();
        }
        const names = url.searchParams.get("messages")?.split(",");
        const direction = url.searchParams.get("direction");
        const lag = url.searchParams.get("max_lag_ms");
        let records = this.journal.filter((record) =>
          Number(record.cursor.slice(3)) > after
          && (!direction || record.direction === direction)
          && (!names || names.includes(record.message))
          && (lag === null || record.received_time_ns / 1e6 >= this.now() - Number(lag)));
        if (url.searchParams.get("collapse") === "true") {
          const latest = new Map<string, MessageRecord>();
          for (const record of records) {
            const key = snapshotKey(record);
            if (key !== null) latest.set(key, record);
          }
          records = records.filter((record) => {
            const key = snapshotKey(record);
            return key === null || latest.get(key) === record;
          });
        }
        const limit = Number(url.searchParams.get("limit") ?? 1000);
        if (!Number.isInteger(limit) || limit < 1) throw new DemoRequestError("Journal limit must be a positive integer");
        const limited = records.length > limit;
        records = records.slice(0, limit);
        return json({ records, next_cursor: limited ? records.at(-1)?.cursor : this.cursor });
      }
      if (path === "/v1/mavlink/parameters") {
        if (method === "GET") return json({ parameters: [...this.parameters.values()] });
        if (method === "PUT" && Array.isArray(body.parameters)) {
          const parameters = body.parameters.map((entry) => {
            const parameter = object(entry);
            return this.setParameter(String(parameter.name), parameter.value, parameter.type);
          });
          return json({ parameters });
        }
      }
      if (path.startsWith("/v1/mavlink/parameters/")) {
        const name = decodeURIComponent(path.slice("/v1/mavlink/parameters/".length));
        if (!this.parameters.has(name)) return json({ message: `Unknown demo parameter ${name}` }, 404);
        if (method === "GET") return json(this.parameters.get(name));
        if (method === "PUT") return json(this.setParameter(name, body.value, body.type));
      }
      if (method === "PUT" && path === "/v1/mavlink/message-rates") {
        for (const [name, hz] of Object.entries(body)) {
          if (!(name in this.fields(this.now())) && name !== "MAG_CAL_PROGRESS" && name !== "MAG_CAL_REPORT") {
            throw new DemoRequestError(`Demo does not synthesize ${name}`);
          }
          if (hz !== null && (finite(hz) < 0 || finite(hz) > 300)) {
            throw new DemoRequestError("Demo message rates must be within 0..300 Hz");
          }
          this.rates.set(name, hz === null ? (DEFAULT_RATES[name] ?? 2) : finite(hz));
          this.lastSample.set(name, this.now());
        }
        return json(Object.fromEntries(this.rates));
      }
      if (method === "POST" && path === "/v1/mavlink/commands") return json(this.command(body));
      if (method === "POST" && path === "/v1/mavlink/messages") {
        if (typeof body.message !== "string") throw new DemoRequestError("Message name is required");
        const fields = object(body.fields);
        const after = this.cursor;
        this.append(body.message, fields, this.now(), "tx");
        if (body.message === "NAMED_VALUE_FLOAT") {
          const value = finite(fields.value);
          switch (fields.name) {
            case "RAWES_THR": this.thrust = Math.max(0, Math.min(1, value)); break;
            case "RAWES_ROFF": this.offsets.roll = value; break;
            case "RAWES_POFF": this.offsets.pitch = value; break;
            case "RAWES_YOFF": this.offsets.yaw = value; break;
          }
        }
        return json({ after_cursor: after });
      }
      return json({ message: `Unsupported demo endpoint: ${method} ${path}` }, 404);
    } catch (error) {
      if (signal?.aborted) throw error;
      if (!(error instanceof DemoRequestError) && !(error instanceof SyntaxError)) throw error;
      return json({ message: error.message }, 400);
    }
  };
}
