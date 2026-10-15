import { eulerFromQuaternion } from "./attitude";
import { diagValue, isDiagRecord } from "./diag-array";
import { MavModeFlag } from "./generated/protocol";
import { enumName, hasFlag, heartbeatState } from "./mav";
import { copterModeName } from "./modes";
import type { MessageRecord } from "./types";
import {
  dejitter,
  estimateLag,
  lowerBound,
  twistAboutBodyZ,
  upperBound,
  type Quat,
} from "./yaw-math";

/** Longest window the chart can show, plus margin for scrolling back. */
export const WINDOW_MAX_S = 65;

/** Messages the yaw panel needs, requested from the journal without collapsing. */
export const YAW_MESSAGES = [
  "HEARTBEAT",
  "STATUSTEXT",
  "ATTITUDE_QUATERNION",
  "ATTITUDE_TARGET",
  "PID_TUNING",
  "SERVO_OUTPUT_RAW",
  "RPM",
  "ESC_TELEMETRY_1_TO_4",
  "ESC_TELEMETRY_5_TO_8",
  "ESC_TELEMETRY_9_TO_12",
  "DEBUG_FLOAT_ARRAY",
] as const;

// How fast the vehicle and PC clocks may drift apart, in seconds per second.
const CLOCK_DRIFT = 1e-4;
const TARGET_STALE_S = 1;
const SEVERE_SEVERITIES = new Set([
  "MAV_SEVERITY_EMERGENCY",
  "MAV_SEVERITY_ALERT",
  "MAV_SEVERITY_CRITICAL",
  "MAV_SEVERITY_ERROR",
  "MAV_SEVERITY_WARNING",
]);

/**
 * Parallel columns sharing one time axis. Stamped sets carry times derived
 * from the vehicle clock. Arrival sets only know when a frame reached LinkHub,
 * so their times are de-batched on read.
 */
export class Samples<K extends string> {
  private readonly raw: number[] = [];
  private readonly columns: Record<K, number[]>;
  private smoothed: number[] | null = null;

  constructor(
    private readonly names: readonly K[],
    private readonly arrivalTimed: boolean,
  ) {
    this.columns = Object.fromEntries(names.map((name) => [name, [] as number[]])) as Record<K, number[]>;
  }

  get length(): number {
    return this.raw.length;
  }

  get t(): readonly number[] {
    if (!this.arrivalTimed) {
      return this.raw;
    }
    this.smoothed ??= dejitter(this.raw);
    return this.smoothed;
  }

  v(name: K): readonly number[] {
    return this.columns[name];
  }

  push(time: number, row: Record<K, number>): void {
    const last = this.raw[this.raw.length - 1];
    this.raw.push(last !== undefined && time < last ? last : time);
    for (const name of this.names) {
      this.columns[name].push(row[name]);
    }
    this.smoothed = null;
  }

  /** Drops samples that arrived before `cutoff`. */
  pruneBefore(cutoff: number): void {
    const drop = lowerBound(this.raw, cutoff);
    if (drop === 0) {
      return;
    }
    this.raw.splice(0, drop);
    for (const name of this.names) {
      this.columns[name].splice(0, drop);
    }
    this.smoothed = null;
  }

  clear(): void {
    this.raw.length = 0;
    for (const name of this.names) {
      this.columns[name].length = 0;
    }
    this.smoothed = null;
  }
}

/** Maps vehicle boot time onto the PC clock by tracking the smallest transit delay. */
class BootClock {
  private offset: number | null = null;
  private lastBoot = 0;
  private lastRecv = 0;

  map(boot: number, recv: number): number {
    const candidate = recv - boot;
    if (this.offset === null || boot < this.lastBoot - 1) {
      this.offset = candidate;
    } else {
      this.offset = Math.min(candidate, this.offset + CLOCK_DRIFT * Math.max(0, recv - this.lastRecv));
    }
    this.lastBoot = boot;
    this.lastRecv = recv;
    return boot + this.offset;
  }

  reset(): void {
    this.offset = null;
  }
}

export interface YawEvent {
  t: number;
  kind: "armed" | "disarmed" | "mode" | "text";
  label: string;
}

export interface YawModelOptions {
  /** SERVO9 output range used to turn the PWM readback into a throttle. */
  servoMinUs: number;
  servoMaxUs: number;
  /** Lua observer plant gain d(yaw rate)/d(throttle) [rad/s per unit]. */
  yawGain: number;
}

const BENCH_SLOPE_RPM_PER_US = 0.504;

/** The gain `rawes.lua` uses for its yaw trim observer (YFF_A). */
export function yawGainFromSlope(slopeRpmPerUs: number, spanUs = 1000): number {
  const slope = slopeRpmPerUs > 0 ? slopeRpmPerUs : BENCH_SLOPE_RPM_PER_US;
  return slope * spanUs * (2 * Math.PI) / 60;
}

function numeric(value: unknown): number {
  return typeof value === "number" && Number.isFinite(value) ? value : Number.NaN;
}

function toDegrees(radians: number): number {
  return radians * 180 / Math.PI;
}

function wrapDegrees(angle: number): number {
  return angle - 360 * Math.round(angle / 360);
}

export class YawModel {
  /** ATTITUDE_QUATERNION: unwrapped heading, heading the target implies, error and gyro rate. */
  readonly att = new Samples(["hdg", "hdgT", "hdgErr", "gyro"] as const, false);
  /** ATTITUDE_TARGET body yaw rate. */
  readonly tgt = new Samples(["rate"] as const, false);
  /** SERVO_OUTPUT_RAW output 9 as a 0..1 throttle. */
  readonly servo = new Samples(["u"] as const, false);
  /** Lua diagnostics: trim, observer target throttle and commanded yaw rate. */
  readonly diag = new Samples(["trim", "ueq", "rateLua"] as const, false);
  /** Yaw rate PID: rad/s, controller terms and the command they sum to with the trim. */
  readonly pid = new Samples(
    ["rateDes", "rateAct", "rateErr", "ff", "p", "i", "d", "trim", "cmd"] as const,
    true,
  );
  readonly rpm = new Samples(["rpm"] as const, true);
  readonly esc = new Samples(["amps"] as const, true);
  readonly events: YawEvent[] = [];

  armed = false;
  mode: string | null = null;
  /** Newest receive time seen, in seconds since the first record. */
  latestT = 0;
  /** performance.now() when the newest batch was ingested, to extrapolate the live edge. */
  ingestedAt = 0;
  options: YawModelOptions = {
    servoMinUs: 1000,
    servoMaxUs: 2000,
    yawGain: yawGainFromSlope(0),
  };

  private epoch: number | null = null;
  private readonly clock = new BootClock();
  private lastHeading: number | null = null;
  private target: { q: Quat; recv: number } | null = null;
  private trim: number | null = null;
  private lastText = "";
  private readonly armedTransitions: { t: number; armed: boolean }[] = [];

  private get sets(): Samples<string>[] {
    return [this.att, this.tgt, this.servo, this.diag, this.pid, this.rpm, this.esc] as Samples<string>[];
  }

  configure(servoMinUs: number, servoMaxUs: number, slopeRpmPerUs: number): void {
    const span = servoMaxUs - servoMinUs;
    this.options = {
      servoMinUs,
      servoMaxUs,
      yawGain: yawGainFromSlope(slopeRpmPerUs, span > 0 ? span : 1000),
    };
  }

  /** Forgets everything; used when LinkHub reports a new connection generation. */
  reset(): void {
    for (const set of this.sets) {
      set.clear();
    }
    this.events.length = 0;
    this.armedTransitions.length = 0;
    this.armed = false;
    this.mode = null;
    this.latestT = 0;
    this.epoch = null;
    this.clock.reset();
    this.lastHeading = null;
    this.target = null;
    this.trim = null;
    this.lastText = "";
  }

  isArmedAt(t: number): boolean {
    const index = upperBound(this.armedTransitions.map((entry) => entry.t), t) - 1;
    return index >= 0 && (this.armedTransitions[index]?.armed ?? false);
  }

  ingest(records: readonly MessageRecord[]): void {
    for (const record of records) {
      if (record.direction === "rx") {
        this.apply(record);
      }
    }
    if (records.length > 0) {
      this.ingestedAt = performance.now();
    }
    const cutoff = this.latestT - WINDOW_MAX_S;
    for (const set of this.sets) {
      set.pruneBefore(cutoff);
    }
    const keep = this.events.findIndex((event) => event.t >= cutoff);
    this.events.splice(0, keep === -1 ? this.events.length : keep);
  }

  private apply(record: MessageRecord): void {
    this.epoch ??= record.received_time_ns / 1e9;
    const recv = record.received_time_ns / 1e9 - this.epoch;
    this.latestT = Math.max(this.latestT, recv);
    const fields = record.fields;
    switch (record.message) {
      case "ATTITUDE_QUATERNION":
        this.applyAttitude(fields, recv);
        break;
      case "ATTITUDE_TARGET":
        this.applyTarget(fields, recv);
        break;
      case "PID_TUNING":
        this.applyPid(fields, recv);
        break;
      case "SERVO_OUTPUT_RAW":
        this.applyServo(fields, recv);
        break;
      case "RPM": {
        const rpm = numeric(fields.rpm1);
        this.rpm.push(recv, { rpm: rpm >= 0 ? rpm : Number.NaN });
        break;
      }
      case "ESC_TELEMETRY_9_TO_12": {
        const current = Array.isArray(fields.current) ? numeric(fields.current[0]) : Number.NaN;
        this.esc.push(recv, { amps: current / 100 });
        break;
      }
      case "HEARTBEAT":
        this.applyHeartbeat(fields, recv);
        break;
      case "STATUSTEXT":
        this.applyText(fields, recv);
        break;
      case "DEBUG_FLOAT_ARRAY":
        if (isDiagRecord(record)) {
          this.applyDiag(record, recv);
        }
        break;
      default:
        break;
    }
  }

  private applyAttitude(fields: MessageRecord["fields"], recv: number): void {
    const q: Quat = [numeric(fields.q1), numeric(fields.q2), numeric(fields.q3), numeric(fields.q4)];
    const boot = numeric(fields.time_boot_ms);
    if (!q.every(Number.isFinite) || !Number.isFinite(boot)) {
      return;
    }
    const euler = eulerFromQuaternion(q[0], q[1], q[2], q[3]);
    if (!euler) {
      return;
    }
    const yaw = toDegrees(euler.yaw);
    const heading = this.lastHeading === null
      ? yaw
      : this.lastHeading + wrapDegrees(yaw - this.lastHeading);
    this.lastHeading = heading;
    const fresh = this.target !== null && recv - this.target.recv < TARGET_STALE_S;
    const error = fresh && this.target ? toDegrees(twistAboutBodyZ(q, this.target.q)) : Number.NaN;
    this.att.push(this.clock.map(boot / 1000, recv), {
      hdg: heading,
      hdgT: heading - error,
      hdgErr: error,
      gyro: numeric(fields.yawspeed),
    });
  }

  private applyTarget(fields: MessageRecord["fields"], recv: number): void {
    const raw = fields.q;
    const boot = numeric(fields.time_boot_ms);
    if (!Array.isArray(raw) || raw.length !== 4 || !Number.isFinite(boot)) {
      return;
    }
    const q: Quat = [numeric(raw[0]), numeric(raw[1]), numeric(raw[2]), numeric(raw[3])];
    if (q.every(Number.isFinite)) {
      this.target = { q, recv };
    }
    this.tgt.push(this.clock.map(boot / 1000, recv), { rate: numeric(fields.body_yaw_rate) });
  }

  private applyPid(fields: MessageRecord["fields"], recv: number): void {
    if (enumName(fields.axis) !== "PID_TUNING_YAW") {
      return;
    }
    const [desired, achieved, ff, p, i, d] = [
      fields.desired, fields.achieved, fields.FF, fields.P, fields.I, fields.D,
    ].map(numeric) as [number, number, number, number, number, number];
    const trim = this.trim ?? 0;
    this.pid.push(recv, {
      rateDes: desired,
      rateAct: achieved,
      rateErr: desired - achieved,
      ff,
      p,
      i,
      d,
      trim,
      cmd: trim + ff + p + i + d,
    });
  }

  private applyServo(fields: MessageRecord["fields"], recv: number): void {
    const usec = numeric(fields.time_usec);
    const raw = numeric(fields.servo9_raw);
    if (!Number.isFinite(usec)) {
      return;
    }
    const { servoMinUs, servoMaxUs } = this.options;
    // 0 us means the output is disabled (ArduPilot holds DDFP off while disarmed).
    const u = Number.isFinite(raw) && raw >= 500
      ? (raw - servoMinUs) / (servoMaxUs - servoMinUs)
      : Number.NaN;
    this.servo.push(this.clock.map(usec / 1e6, recv), { u });
  }

  private applyDiag(record: MessageRecord, recv: number): void {
    const usec = numeric(record.fields.time_usec);
    if (!Number.isFinite(usec)) {
      return;
    }
    const trim = diagValue(record, "YFF_T");
    const applied = diagValue(record, "YFF_U");
    const gyro = diagValue(record, "YFF_GZ");
    this.trim = trim ?? null;
    this.diag.push(this.clock.map(usec / 1e6, recv), {
      trim: trim ?? Number.NaN,
      ueq: applied !== undefined && gyro !== undefined
        ? applied - gyro / this.options.yawGain
        : Number.NaN,
      rateLua: diagValue(record, "OL_YSP") ?? Number.NaN,
    });
  }

  private applyHeartbeat(fields: MessageRecord["fields"], recv: number): void {
    const state = heartbeatState(fields);
    if (!state) {
      return;
    }
    const armed = hasFlag(state.base_mode, MavModeFlag.SAFETY_ARMED);
    const mode = copterModeName(state.custom_mode);
    if (this.armedTransitions.length === 0) {
      this.armedTransitions.push({ t: Number.NEGATIVE_INFINITY, armed });
    } else if (armed !== this.armed) {
      this.armedTransitions.push({ t: recv, armed });
      this.events.push({ t: recv, kind: armed ? "armed" : "disarmed", label: armed ? "armed" : "disarmed" });
    }
    if (this.mode !== null && mode !== this.mode) {
      this.events.push({ t: recv, kind: "mode", label: `mode ${mode}` });
    }
    this.armed = armed;
    this.mode = mode;
  }

  private applyText(fields: MessageRecord["fields"], recv: number): void {
    const text = typeof fields.text === "string" ? fields.text.trim() : "";
    const severe = SEVERE_SEVERITIES.has(enumName(fields.severity) ?? "");
    if (text === "" || text === this.lastText || !(severe || text.startsWith("RAWES"))) {
      return;
    }
    this.lastText = text;
    this.events.push({ t: recv, kind: "text", label: text });
  }
}

export interface YawMetrics {
  armed: boolean;
  mode: string | null;
  rateErrRms: number | null;
  hdgErrRms: number | null;
  iShare: number | null;
  satPercent: number | null;
  lagMs: number | null;
}

function rms(values: number[]): number | null {
  return values.length === 0
    ? null
    : Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length);
}

/** True where the yaw command sits at a limit of the 0..1 DDFP range. */
export function isSaturated(command: number): boolean {
  return command <= 0 || command >= 1;
}

export function computeMetrics(model: YawModel, now: number, windowS = 10): YawMetrics {
  const start = now - windowS;
  const pidTimes = model.pid.t;
  const rateErrors: number[] = [];
  let iSum = 0;
  let totalSum = 0;
  let armedCount = 0;
  let saturated = 0;
  for (let index = lowerBound(pidTimes, start); index < pidTimes.length; index += 1) {
    const t = pidTimes[index] as number;
    if (t > now || !model.isArmedAt(t)) {
      continue;
    }
    const error = model.pid.v("rateErr")[index] as number;
    const [ff, p, i, d, command] = (["ff", "p", "i", "d", "cmd"] as const)
      .map((name) => model.pid.v(name)[index] as number) as [number, number, number, number, number];
    if (![error, ff, p, i, d, command].every(Number.isFinite)) {
      continue;
    }
    armedCount += 1;
    rateErrors.push(error);
    iSum += Math.abs(i);
    totalSum += Math.abs(ff) + Math.abs(p) + Math.abs(i) + Math.abs(d);
    if (isSaturated(command)) {
      saturated += 1;
    }
  }
  const headingErrors: number[] = [];
  const attTimes = model.att.t;
  for (let index = lowerBound(attTimes, start); index < attTimes.length; index += 1) {
    const error = model.att.v("hdgErr")[index] as number;
    if ((attTimes[index] as number) <= now && Number.isFinite(error)) {
      headingErrors.push(error);
    }
  }
  let lagMs: number | null = null;
  if (model.armed) {
    const lag = estimateLag(
      { t: model.servo.t, v: model.servo.v("u") },
      { t: model.rpm.t, v: model.rpm.v("rpm") },
      start,
      now,
    );
    lagMs = lag !== null && lag.correlation >= 0.5 ? lag.lagS * 1000 : null;
  }
  return {
    armed: model.armed,
    mode: model.mode,
    rateErrRms: rms(rateErrors),
    hdgErrRms: rms(headingErrors),
    iShare: totalSum > 1e-9 ? iSum / totalSum : null,
    satPercent: armedCount > 0 ? (100 * saturated) / armedCount : null,
    lagMs,
  };
}
