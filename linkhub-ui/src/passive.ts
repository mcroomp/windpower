import type { LinkHubApi } from "./api";
import {
  MavLandedState,
  MavState,
  NamedValueFloat,
  SetAttitudeTarget,
} from "./generated/protocol";
import type { TelemetryStore } from "./telemetry";
import type { LinkHubStatus, MessageRecord, Quaternion } from "./types";

const ARMED_FLAG = 128;
const MODE_ACRO = 1;
const MODE_GUIDED_NOGPS = 20;
const MODE_PASSIVE = 3;
const ANGLE_STEP_DEG = 5;
const ANGLE_LIMIT_DEG = 30;
const THRUST_STEP = 0.05;
const DEFAULT_THRUST = 0.342;

export interface PassiveOptions {
  force: boolean;
  thrust: number;
  durationSeconds: number | null;
  rollDegrees: number;
  pitchDegrees: number;
  yawDegrees: number;
}

export const PASSIVE_TELEMETRY_RATES = Object.freeze({
  ATTITUDE: 25,
  ATTITUDE_QUATERNION: 25,
  ATTITUDE_TARGET: 25,
  EXTENDED_SYS_STATE: 25,
  SERVO_OUTPUT_RAW: 25,
  LOCAL_POSITION_NED: 10,
  BATTERY_STATUS: 2,
  EKF_STATUS_REPORT: 2,
});

export type PassivePhase = "idle" | "starting" | "running" | "stopping" | "failed";

export function parseRunPassive(tokens: string[]): PassiveOptions {
  const options: PassiveOptions = {
    force: false,
    thrust: DEFAULT_THRUST,
    durationSeconds: null,
    rollDegrees: 0,
    pitchDegrees: 0,
    yawDegrees: 0,
  };
  for (let index = 0; index < tokens.length; index += 1) {
    const token = tokens[index];
    if (token === "--force") {
      options.force = true;
      continue;
    }
    const value = tokens[index + 1];
    if (value === undefined) {
      throw new Error(`${token} requires a value`);
    }
    if (token === "--duration") {
      options.durationSeconds = positiveNumber(value, token);
    } else if (token === "--roll") {
      options.rollDegrees = boundedNumber(value, token, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
    } else if (token === "--pitch") {
      options.pitchDegrees = boundedNumber(value, token, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
    } else if (token === "--yaw") {
      options.yawDegrees = boundedNumber(value, token, -180, 180);
    } else if (token === "--trim") {
      const match = /^thr=(.+)$/.exec(value);
      if (!match?.[1]) {
        throw new Error("Only --trim thr=<0..1> is supported");
      }
      options.thrust = boundedNumber(match[1], "--trim thr", 0, 1);
    } else {
      throw new Error(`Unknown run passive option: ${token}`);
    }
    index += 1;
  }
  return options;
}

function positiveNumber(value: string, option: string): number {
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || parsed <= 0) {
    throw new Error(`${option} must be greater than zero`);
  }
  return parsed;
}

function boundedNumber(value: string, option: string, minimum: number, maximum: number): number {
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || parsed < minimum || parsed > maximum) {
    throw new Error(`${option} must be within ${minimum}..${maximum}`);
  }
  return parsed;
}

function isArmed(status: LinkHubStatus | null): boolean {
  return Boolean(status && (status.base_mode & ARMED_FLAG));
}

function recordNumber(record: MessageRecord | undefined, field: string): number | null {
  const value = record?.fields[field];
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function recordEnum(value: unknown, expectedName: string, expectedValue: number): boolean {
  return value === expectedName || Number(value) === expectedValue;
}

export class PassiveController {
  private currentPhase: PassivePhase = "idle";
  private operation: AbortController | null = null;
  private cleanup: Promise<void> | null = null;
  private durationTimer: number | null = null;
  private thrust = DEFAULT_THRUST;
  private rollDegrees = 0;
  private pitchDegrees = 0;
  private yawDegrees = 0;

  constructor(
    private readonly api: LinkHubApi,
    private readonly telemetry: TelemetryStore,
    private readonly write: (message: string, kind?: "info" | "error" | "fc") => void,
    private readonly phaseChanged: (phase: PassivePhase) => void,
  ) {
    telemetry.onGeneration((generation, previous) => {
      this.write(`Link generation changed: ${previous ?? "none"} -> ${generation}`, "error");
      this.resetLocalState();
      this.setPhase("idle");
    });
  }

  get phase(): PassivePhase {
    return this.currentPhase;
  }

  async reconcile(): Promise<void> {
    const status = await this.api.status();
    const rawesMode = await this.api.getParameter("RAWES_MODE");
    if (
      isArmed(status)
      && status.custom_mode === MODE_GUIDED_NOGPS
      && Math.round(rawesMode.value) === MODE_PASSIVE
    ) {
      this.restoreTargets();
      this.setPhase("running");
      this.write("Attached to active passive run from observed vehicle state.");
    }
  }

  async start(options: PassiveOptions): Promise<void> {
    if (this.currentPhase !== "idle" && this.currentPhase !== "failed") {
      throw new Error(`Cannot start passive while ${this.currentPhase}`);
    }
    await this.reconcile();
    if (this.isRunning()) {
      return;
    }
    this.operation = new AbortController();
    const signal = this.operation.signal;
    this.thrust = options.thrust;
    this.rollDegrees = options.rollDegrees;
    this.pitchDegrees = options.pitchDegrees;
    this.yawDegrees = options.yawDegrees;
    this.setPhase("starting");
    try {
      await this.configureTelemetry();
      await this.ensureGuidedThrustOption();
      await this.ensureTailSetup();
      await this.setParameter("H_SV_MAN", 0);
      await this.setParameter("RAWES_MODE", 0);
      await this.setModeAndConfirm(MODE_ACRO, signal);

      const runupAfter = this.telemetry.checkpoint();
      if (!isArmed(this.telemetry.status)) {
        this.write(`Sending ${options.force ? "force-" : ""}arm command…`);
        const armAfter = this.telemetry.checkpoint();
        try {
          await this.api.setArmed(true, options.force);
        } catch (error) {
          throw await this.armFailure(error, armAfter, options.force, signal);
        }
        await this.waitForArmed(true, signal, 15_000);
      }
      this.write("Vehicle armed; waiting for heli runup.");
      await this.waitForRunup(runupAfter, signal);

      const initialQuaternion = await this.captureQuaternion(signal);
      await this.sendNamedValue("RAWES_THR", this.thrust);
      await this.sendNamedValue("RAWES_RLL", 0);
      await this.sendNamedValue("RAWES_PIT", 0);
      await this.sendNamedValue("RAWES_COL", this.thrust);
      await this.setParameter("RAWES_MODE", 2);
      await this.waitForInAir(signal);

      await this.setModeAndConfirm(MODE_GUIDED_NOGPS, signal);
      await this.waitForSettledAttitude(initialQuaternion, signal);

      await this.sendTargets();
      const passiveAfter = this.telemetry.checkpoint();
      await this.sendNamedValue("RAWES_PEN", 1);
      await this.setParameter("RAWES_MODE", MODE_PASSIVE);
      await this.waitForPassiveAcknowledgement(initialQuaternion, passiveAfter, signal);

      this.setPhase("running");
      this.write("Passive run active. Keyboard controls are enabled.");
      if (options.durationSeconds !== null) {
        this.durationTimer = window.setTimeout(
          () => void this.stop(),
          options.durationSeconds * 1_000,
        );
      }
    } catch (error) {
      if (!(error instanceof DOMException && error.name === "AbortError")) {
        this.setPhase("failed");
      }
      await this.safeOff();
      throw error;
    }
  }

  async stop(): Promise<void> {
    if (this.currentPhase === "idle" || this.currentPhase === "stopping") {
      return this.cleanup ?? Promise.resolve();
    }
    this.operation?.abort();
    this.setPhase("stopping");
    await this.safeOff();
  }

  async adjust(axis: "roll" | "pitch" | "yaw" | "collective", direction: -1 | 1): Promise<void> {
    if (this.currentPhase !== "running") {
      return;
    }
    if (axis === "collective") {
      this.thrust = Math.max(0, Math.min(1, this.thrust + direction * THRUST_STEP));
      await this.sendNamedValue("RAWES_THR", this.thrust);
    } else if (axis === "yaw") {
      this.yawDegrees = ((this.yawDegrees + direction * ANGLE_STEP_DEG + 180) % 360) - 180;
      await this.sendOffsets();
    } else {
      const next = Math.max(
        -ANGLE_LIMIT_DEG,
        Math.min(
          ANGLE_LIMIT_DEG,
          (axis === "roll" ? this.rollDegrees : this.pitchDegrees)
            + direction * ANGLE_STEP_DEG,
        ),
      );
      if (axis === "roll") {
        this.rollDegrees = next;
      } else {
        this.pitchDegrees = next;
      }
      await this.sendOffsets();
    }
    this.write(
      `target roll=${this.rollDegrees.toFixed(0)}° pitch=${this.pitchDegrees.toFixed(0)}° `
      + `yaw=${this.yawDegrees.toFixed(0)}° thrust=${this.thrust.toFixed(3)}`,
    );
  }

  async resetTarget(): Promise<void> {
    if (this.currentPhase !== "running") {
      return;
    }
    this.rollDegrees = 0;
    this.pitchDegrees = 0;
    this.yawDegrees = 0;
    await this.sendOffsets();
    this.write("Passive offsets reset to the onboard attitude anchor.");
  }

  private async configureTelemetry(): Promise<void> {
    this.write("Configuring passive telemetry streams…");
    try {
      await this.api.setMessageRates(PASSIVE_TELEMETRY_RATES);
    } catch (error) {
      const detail = error instanceof Error ? error.message : String(error);
      throw new Error(
        `Passive start could not configure required telemetry streams: ${detail}. `
        + "The vehicle was left in the canonical safe-off state.",
        { cause: error },
      );
    }
  }

  private async armFailure(
    error: unknown,
    after: number,
    forced: boolean,
    signal: AbortSignal,
  ): Promise<Error> {
    const deadline = performance.now() + 500;
    let reason: string | null = null;
    while (performance.now() < deadline) {
      signal.throwIfAborted();
      const messages = this.telemetry.recordsAfter(
        after,
        (record) => record.direction === "rx" && record.message === "STATUSTEXT",
      );
      reason = messages
        .map((record) => String(record.fields.text ?? "").trim())
        .filter((text) => text.startsWith("PreArm:") || text.startsWith("Arm:"))
        .at(-1) ?? null;
      if (reason) {
        break;
      }
      await delay(50, signal);
    }
    const detail = error instanceof Error ? error.message : String(error);
    const controllerReason = reason ? ` Flight controller: ${reason}.` : "";
    const guidance = forced
      ? " Check the flight-controller messages and resolve the arm failure before retrying."
      : " Resolve the pre-arm condition before retrying; use --force only when intentionally bypassing ArduPilot arm checks.";
    return new Error(`${detail}.${controllerReason}${guidance}`, { cause: error });
  }

  private async ensureGuidedThrustOption(): Promise<void> {
    const options = await this.api.getParameter("GUID_OPTIONS");
    const required = Math.round(options.value) | 8;
    if (required !== Math.round(options.value)) {
      await this.setParameter("GUID_OPTIONS", required);
    }
  }

  private async ensureTailSetup(): Promise<void> {
    const [tail, motorFunction] = await Promise.all([
      this.api.getParameter("H_TAIL_TYPE"),
      this.api.getParameter("SERVO9_FUNCTION"),
    ]);
    if (Math.round(tail.value) !== 3) {
      throw new Error(`Passive requires H_TAIL_TYPE=3; found ${tail.value}`);
    }
    if (Math.round(motorFunction.value) !== 36) {
      throw new Error(`Passive requires SERVO9_FUNCTION=36 at boot; found ${motorFunction.value}`);
    }
  }

  private async waitForRunup(after: number, signal: AbortSignal): Promise<void> {
    const [ramp, runup] = await Promise.all([
      this.api.getParameter("H_RSC_RAMP_TIME", signal),
      this.api.getParameter("H_RSC_RUNUP_TIME", signal),
    ]);
    const timeoutMs = (Math.max(ramp.value, runup.value) + 5.5) * 1_000;
    await this.waitForText("runup complete", timeoutMs, signal, after);
    this.write("ArduPilot reports heli runup complete.", "fc");
  }

  private async captureQuaternion(signal: AbortSignal): Promise<Quaternion> {
    const existing = this.quaternion();
    if (existing) {
      return existing;
    }
    const record = await this.telemetry.waitFor(
      (candidate) => candidate.direction === "rx"
        && candidate.message === "ATTITUDE_QUATERNION",
      3_000,
      signal,
    );
    return this.quaternion(record) ?? Promise.reject(new Error("Invalid attitude quaternion"));
  }

  private quaternion(
    record = this.telemetry.get("ATTITUDE_QUATERNION"),
  ): Quaternion | null {
    const q1 = recordNumber(record, "q1");
    const q2 = recordNumber(record, "q2");
    const q3 = recordNumber(record, "q3");
    const q4 = recordNumber(record, "q4");
    if (q1 === null || q2 === null || q3 === null || q4 === null) {
      return null;
    }
    return [q1, q2, q3, q4];
  }

  private async waitForInAir(signal: AbortSignal): Promise<void> {
    const current = this.telemetry.get("EXTENDED_SYS_STATE");
    if (recordEnum(current?.fields.landed_state, "IN_AIR", MavLandedState.IN_AIR)) {
      return;
    }
    await this.telemetry.waitFor(
      (record) => record.message === "EXTENDED_SYS_STATE"
        && recordEnum(record.fields.landed_state, "IN_AIR", MavLandedState.IN_AIR),
      3_000,
      signal,
    );
  }

  private async waitForSettledAttitude(
    target: Quaternion,
    signal: AbortSignal,
  ): Promise<void> {
    this.write("Waiting for GUIDED ACTIVE and a quiet attitude interval…");
    let quietSince: number | null = null;
    let latestAttitudeAt: number | null = null;
    let active = this.telemetry.status?.system_status === MavState.ACTIVE;
    const unsubscribe = this.telemetry.onRecord((record) => {
      if (record.direction !== "rx") {
        return;
      }
      if (record.message === "HEARTBEAT") {
        active = recordEnum(record.fields.system_status, "ACTIVE", MavState.ACTIVE);
        if (!active) {
          quietSince = null;
        }
      }
      if (record.message === "ATTITUDE" || record.message === "ATTITUDE_QUATERNION") {
        latestAttitudeAt = performance.now();
        const rates = ["rollspeed", "pitchspeed", "yawspeed"]
          .map((field) => Math.abs(Number(record.fields[field] ?? Infinity)));
        if (active && Math.max(...rates) <= 0.05) {
          quietSince ??= performance.now();
        } else {
          quietSince = null;
        }
      }
      if (
        record.message === "STATUSTEXT"
        && String(record.fields.text ?? "").toLowerCase().includes("yaw alignment complete")
      ) {
        quietSince = null;
      }
    });
    const deadline = performance.now() + 15_000;
    try {
      while (performance.now() < deadline) {
        signal.throwIfAborted();
        await this.sendAttitudeTarget(target);
        const now = performance.now();
        if (
          active
          && quietSince !== null
          && latestAttitudeAt !== null
          && now - quietSince >= 3_000
          && now - latestAttitudeAt <= 500
        ) {
          return;
        }
        await delay(50, signal);
      }
      throw new Error("Timed out waiting for GUIDED/EKF attitude to settle");
    } finally {
      unsubscribe();
    }
  }

  private async waitForPassiveAcknowledgement(
    target: Quaternion,
    after: number,
    signal: AbortSignal,
  ): Promise<void> {
    let modeSeen = false;
    let captureSeen = false;
    for (const record of this.telemetry.recordsAfter(
      after,
      (candidate) => candidate.direction === "rx" && candidate.message === "STATUSTEXT",
    )) {
      const text = String(record.fields.text ?? "");
      modeSeen ||= text.includes("RAWES: mode 3 (passive) entered");
      captureSeen ||= text.includes("RAWES: passive active");
    }
    const unsubscribe = this.telemetry.onRecord((record) => {
      if (record.direction !== "rx" || record.message !== "STATUSTEXT") {
        return;
      }
      const text = String(record.fields.text ?? "");
      this.write(text, "fc");
      modeSeen ||= text.includes("RAWES: mode 3 (passive) entered");
      captureSeen ||= text.includes("RAWES: passive active");
    });
    const deadline = performance.now() + 2_000;
    try {
      while (performance.now() < deadline) {
        signal.throwIfAborted();
        if (modeSeen && captureSeen) {
          return;
        }
        await this.sendAttitudeTarget(target);
        await delay(50, signal);
      }
      throw new Error("Lua did not confirm passive steady-state control");
    } finally {
      unsubscribe();
    }
  }

  private async setModeAndConfirm(mode: number, signal: AbortSignal): Promise<void> {
    if (this.telemetry.status?.custom_mode !== mode) {
      await this.api.setMode(mode);
    }
    const deadline = performance.now() + 10_000;
    while (performance.now() < deadline) {
      signal.throwIfAborted();
      if (this.telemetry.status?.custom_mode === mode) {
        return;
      }
      await delay(50, signal);
    }
    throw new Error(`Flight mode ${mode} was not confirmed`);
  }

  private async waitForArmed(
    expected: boolean,
    signal: AbortSignal,
    timeoutMs: number,
  ): Promise<void> {
    const deadline = performance.now() + timeoutMs;
    while (performance.now() < deadline) {
      signal.throwIfAborted();
      if (isArmed(this.telemetry.status) === expected) {
        return;
      }
      await delay(50, signal);
    }
    throw new Error(`Vehicle was not confirmed ${expected ? "armed" : "disarmed"}`);
  }

  private async waitForText(
    text: string,
    timeoutMs: number,
    signal: AbortSignal,
    after = 0,
  ): Promise<void> {
    const expected = text.toLowerCase();
    const found = this.telemetry.recordsAfter(
      after,
      (record) => record.direction === "rx"
        && record.message === "STATUSTEXT"
        && String(record.fields.text ?? "").toLowerCase().includes(expected),
    );
    if (found.length > 0) {
      return;
    }
    await this.telemetry.waitFor(
      (record) => record.direction === "rx"
        && record.message === "STATUSTEXT"
        && String(record.fields.text ?? "").toLowerCase().includes(expected),
      timeoutMs,
      signal,
    );
  }

  private async sendTargets(): Promise<void> {
    await this.sendNamedValue("RAWES_THR", this.thrust);
    await this.sendOffsets();
  }

  private async sendOffsets(): Promise<void> {
    await Promise.all([
      this.sendNamedValue("RAWES_ROFF", radians(this.rollDegrees)),
      this.sendNamedValue("RAWES_POFF", radians(this.pitchDegrees)),
      this.sendNamedValue("RAWES_YOFF", radians(this.yawDegrees)),
    ]);
  }

  private async sendNamedValue(name: string, value: number): Promise<void> {
    await this.api.sendMessage(new NamedValueFloat({
      name,
      value,
      time_boot_ms: 0,
    }));
  }

  private async sendAttitudeTarget(quaternion: Quaternion): Promise<void> {
    const status = this.telemetry.status;
    await this.api.sendMessage(new SetAttitudeTarget({
      target_system: status?.target_system ?? 0,
      target_component: status?.target_component ?? 0,
      type_mask: 0,
      q: [...quaternion],
      body_roll_rate: 0,
      body_pitch_rate: 0,
      body_yaw_rate: 0,
      thrust: this.thrust,
      time_boot_ms: 0,
    }));
  }

  private async setParameter(name: string, value: number): Promise<void> {
    const result = await this.api.setParameter(name, value);
    if (Math.abs(result.value - value) > 1e-4) {
      throw new Error(`${name} read back as ${result.value}, expected ${value}`);
    }
  }

  private restoreTargets(): void {
    const value = (name: string): number | null =>
      recordNumber(this.telemetry.get("NAMED_VALUE_FLOAT", "tx", name), "value");
    this.thrust = value("RAWES_THR") ?? DEFAULT_THRUST;
    this.rollDegrees = degrees(value("RAWES_ROFF") ?? 0);
    this.pitchDegrees = degrees(value("RAWES_POFF") ?? 0);
    this.yawDegrees = degrees(value("RAWES_YOFF") ?? 0);
  }

  private async safeOff(): Promise<void> {
    if (this.cleanup) {
      return this.cleanup;
    }
    this.cleanup = this.applySafeOff().finally(() => {
      this.cleanup = null;
      this.resetLocalState();
      this.setPhase("idle");
    });
    return this.cleanup;
  }

  private async applySafeOff(): Promise<void> {
    this.write("Applying canonical safe-off state…");
    const attempt = async (label: string, action: () => Promise<unknown>) => {
      try {
        await action();
        this.write(`${label}: OK`);
      } catch (error) {
        this.write(`${label}: ${error instanceof Error ? error.message : String(error)}`, "error");
      }
    };
    await attempt("RAWES_MODE=0", () => this.setParameter("RAWES_MODE", 0));
    const status = await this.api.status().catch(() => null);
    if (isArmed(status)) {
      await attempt("force disarm", () => this.api.setArmed(false, true));
    }
    await attempt("H_YAW_TRIM=0", () => this.setParameter("H_YAW_TRIM", 0));
    await attempt("H_FLYBAR_MODE=1", () => this.setParameter("H_FLYBAR_MODE", 1));
    await attempt("H_SV_MAN=0", () => this.setParameter("H_SV_MAN", 0));
    await attempt("flight mode ACRO", () => this.api.setMode(MODE_ACRO));
    await attempt("SERVO9_FUNCTION verification", async () => {
      const result = await this.api.getParameter("SERVO9_FUNCTION");
      if (Math.round(result.value) !== 36) {
        throw new Error(`expected 36, found ${result.value}`);
      }
    });
  }

  private resetLocalState(): void {
    if (this.durationTimer !== null) {
      window.clearTimeout(this.durationTimer);
      this.durationTimer = null;
    }
    this.operation = null;
    this.thrust = DEFAULT_THRUST;
    this.rollDegrees = 0;
    this.pitchDegrees = 0;
    this.yawDegrees = 0;
  }

  private setPhase(phase: PassivePhase): void {
    this.currentPhase = phase;
    this.phaseChanged(phase);
  }

  private isRunning(): boolean {
    return this.currentPhase === "running";
  }
}

function radians(value: number): number {
  return value * Math.PI / 180;
}

function degrees(value: number): number {
  return value * 180 / Math.PI;
}

function delay(milliseconds: number, signal: AbortSignal): Promise<void> {
  return new Promise((resolve, reject) => {
    const complete = () => {
      signal.removeEventListener("abort", abort);
      resolve();
    };
    const timeout = window.setTimeout(complete, milliseconds);
    const abort = () => {
      window.clearTimeout(timeout);
      signal.removeEventListener("abort", abort);
      reject(signal.reason ?? new DOMException("Aborted", "AbortError"));
    };
    signal.addEventListener("abort", abort, { once: true });
  });
}
