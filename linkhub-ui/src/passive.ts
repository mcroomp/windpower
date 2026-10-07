import { formatMavResult, type LinkHubApi } from "./api";
import {
  MavCmd,
  MavLandedState,
  MavResult,
  MavState,
  NamedValueFloat,
} from "./generated/protocol";
import { enumIs, isArmed, isVehicleHeartbeat } from "./mav";
import type { TelemetryStore } from "./telemetry";
import { DISPLAY_TELEMETRY_RATES } from "./telemetry-rates";
import type { MessageRecord } from "./types";

const MODE_ACRO = 1;
const MODE_GUIDED_NOGPS = 20;
const MODE_PASSIVE = 3;
const ANGLE_STEP_DEG = 5;
const ANGLE_LIMIT_DEG = 30;
const THRUST_STEP = 0.05;
const DEFAULT_THRUST = 0.342;
// rawes.lua COMMAND_LONG IDs (MAV_CMD_USER_1/2); Lua owns the acknowledgement.
export const CMD_ENTER_GUIDED = MavCmd.USER_1;
export const CMD_ENTER_PASSIVE = MavCmd.USER_2;

export interface PassiveOptions {
  force: boolean;
  thrust: number;
  durationSeconds: number | null;
  rollDegrees: number;
  pitchDegrees: number;
  yawDegrees: number;
}

export const PASSIVE_TELEMETRY_RATES = Object.freeze({
  ...DISPLAY_TELEMETRY_RATES,
  EXTENDED_SYS_STATE: 25,
  EKF_STATUS_REPORT: 2,
});
export const PASSIVE_YAW_TRIM_SEED = 0;

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

function recordNumber(record: MessageRecord | undefined, field: string): number | null {
  const value = record?.fields[field];
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

export class PassiveController {
  private currentPhase: PassivePhase = "idle";
  private operation: AbortController | null = null;
  private cleanup: Promise<void> | null = null;
  private durationTimer: ReturnType<typeof globalThis.setTimeout> | null = null;
  private thrust = DEFAULT_THRUST;
  private rollDegrees = 0;
  private pitchDegrees = 0;
  private yawDegrees = 0;
  private recapturing = false;
  private targetUpdate = Promise.resolve();

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

      await this.sendNamedValue("RAWES_THR", this.thrust);
      await this.sendNamedValue("RAWES_RLL", 0);
      await this.sendNamedValue("RAWES_PIT", 0);
      await this.sendNamedValue("RAWES_COL", this.thrust);
      await this.setParameter("RAWES_MODE", 2);
      await this.waitForInAir(signal);

      this.write("Lua capturing attitude and entering GUIDED_NOGPS…");
      await this.luaCommand(CMD_ENTER_GUIDED, [], "ENTER_GUIDED", signal);
      await this.waitForMode(MODE_GUIDED_NOGPS, signal);
      await this.waitForSettledAttitude(signal);

      await this.sendTargets();
      await this.setParameter("RAWES_MODE", MODE_PASSIVE);
      await this.luaCommand(
        CMD_ENTER_PASSIVE,
        [PASSIVE_YAW_TRIM_SEED],
        "ENTER_PASSIVE",
        signal,
      );

      this.setPhase("running");
      this.write("Passive run active. Keyboard controls are enabled.");
      if (options.durationSeconds !== null) {
        this.durationTimer = globalThis.setTimeout(
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
    if (this.currentPhase === "stopping") {
      return this.cleanup ?? Promise.resolve();
    }
    this.operation?.abort();
    this.setPhase("stopping");
    await this.safeOff();
  }

  async adjust(axis: "roll" | "pitch" | "yaw" | "collective", direction: -1 | 1): Promise<void> {
    if (this.currentPhase !== "running" || this.recapturing) {
      return;
    }
    if (axis === "collective") {
      this.thrust = Math.max(0, Math.min(1, this.thrust + direction * THRUST_STEP));
      await this.sendNamedValue("RAWES_THR", this.thrust);
    } else if (axis === "yaw") {
      this.yawDegrees = ((this.yawDegrees + direction * ANGLE_STEP_DEG + 180) % 360) - 180;
      const offsets = this.currentOffsets();
      await this.queueTargetUpdate(() => this.sendOffsets(offsets));
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
      const offsets = this.currentOffsets();
      await this.queueTargetUpdate(() => this.sendOffsets(offsets));
    }
    this.write(
      `target roll=${this.rollDegrees.toFixed(0)}° pitch=${this.pitchDegrees.toFixed(0)}° `
      + `yaw=${this.yawDegrees.toFixed(0)}° thrust=${this.thrust.toFixed(3)}`,
    );
  }

  async recaptureTarget(): Promise<void> {
    if (this.currentPhase !== "running" || this.recapturing) {
      return;
    }
    this.recapturing = true;
    try {
      this.rollDegrees = 0;
      this.pitchDegrees = 0;
      this.yawDegrees = 0;
      const signal = this.operation?.signal ?? new AbortController().signal;
      await this.queueTargetUpdate(async () => {
        await this.sendOffsets(this.currentOffsets());
        await this.luaCommand(
          CMD_ENTER_PASSIVE,
          [PASSIVE_YAW_TRIM_SEED],
          "ENTER_PASSIVE recapture",
          signal,
        );
      });
      this.write("Passive direction recaptured; keyboard attitude offsets cleared.");
    } finally {
      this.recapturing = false;
    }
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

  private async waitForInAir(signal: AbortSignal): Promise<void> {
    const current = this.telemetry.get("EXTENDED_SYS_STATE");
    if (enumIs(current?.fields.landed_state, MavLandedState.IN_AIR)) {
      return;
    }
    await this.telemetry.waitFor(
      (record) => record.message === "EXTENDED_SYS_STATE"
        && enumIs(record.fields.landed_state, MavLandedState.IN_AIR),
      3_000,
      signal,
    );
  }

  private async waitForSettledAttitude(signal: AbortSignal): Promise<void> {
    this.write("Waiting for GUIDED ACTIVE and a quiet attitude interval…");
    let quietSince: number | null = null;
    let latestAttitudeAt: number | null = null;
    let active = enumIs(this.telemetry.status?.system_status, MavState.ACTIVE);
    const unsubscribe = this.telemetry.onRecord((record) => {
      if (record.direction !== "rx") {
        return;
      }
      if (record.message === "HEARTBEAT" && isVehicleHeartbeat(record.fields)) {
        active = enumIs(record.fields.system_status, MavState.ACTIVE);
        if (!active) {
          quietSince = null;
        }
      }
      if (record.message === "ATTITUDE_QUATERNION") {
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

  private async luaCommand(
    command: MavCmd,
    params: number[],
    label: string,
    signal: AbortSignal,
  ): Promise<void> {
    const after = this.telemetry.checkpoint();
    const result = await this.api.command(command, params);
    if (enumIs(result.result, MavResult.ACCEPTED)) {
      this.write(`Lua accepted ${label}.`);
      return;
    }
    await delay(300, signal);
    const reason = this.telemetry.recordsAfter(
      after,
      (record) => record.direction === "rx" && record.message === "STATUSTEXT",
    )
      .map((record) => String(record.fields.text ?? ""))
      .filter((text) => text.startsWith("RAWES cmd"))
      .at(-1);
    throw new Error(
      `${label} rejected with ${formatMavResult(result.result)}`
      + (reason ? `: ${reason}` : ""),
    );
  }

  private async waitForMode(mode: number, signal: AbortSignal): Promise<void> {
    const deadline = performance.now() + 5_000;
    while (performance.now() < deadline) {
      signal.throwIfAborted();
      if (this.telemetry.status?.custom_mode === mode) {
        return;
      }
      await delay(50, signal);
    }
    throw new Error(`Flight mode ${mode} was not confirmed`);
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

  private currentOffsets(): readonly [number, number, number] {
    return [this.rollDegrees, this.pitchDegrees, this.yawDegrees];
  }

  private async sendOffsets(
    offsets: readonly [number, number, number] = this.currentOffsets(),
  ): Promise<void> {
    const [roll, pitch, yaw] = offsets;
    await Promise.all([
      this.sendNamedValue("RAWES_ROFF", radians(roll)),
      this.sendNamedValue("RAWES_POFF", radians(pitch)),
      this.sendNamedValue("RAWES_YOFF", radians(yaw)),
    ]);
  }

  private queueTargetUpdate(operation: () => Promise<void>): Promise<void> {
    const result = this.targetUpdate.then(operation);
    this.targetUpdate = result.catch(() => undefined);
    return result;
  }

  private async sendNamedValue(name: string, value: number): Promise<void> {
    await this.api.sendMessage(new NamedValueFloat({
      name,
      value,
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
      globalThis.clearTimeout(this.durationTimer);
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
    const timeout = globalThis.setTimeout(complete, milliseconds);
    const abort = () => {
      globalThis.clearTimeout(timeout);
      signal.removeEventListener("abort", abort);
      reject(signal.reason ?? new DOMException("Aborted", "AbortError"));
    };
    signal.addEventListener("abort", abort, { once: true });
  });
}
