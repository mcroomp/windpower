import type { LinkHubApi } from "./api";
import { isArmed } from "./mav";
import type { PassiveController } from "./passive";
import type { TelemetryStore } from "./telemetry";
import type { ExperimentInfo, MessageRecord, MotorStatus, SavedExperiment } from "./types";
import { MOTOR_TO_ROTOR_GEAR_RATIO } from "./vehicle-motion";

export type ExperimentStage = "idle" | "running" | "stopping-motor" | "spin-down" | "disarming" | "saving";

export interface ExperimentTiming {
  pollMs: number;
  motorStopTimeoutMs: number;
  spinDownTimeoutMs: number;
  /** Rotor RPM older than this is treated as unavailable. */
  rpmFreshMs: number;
  stoppedRotorRpm: number;
  now: () => number;
  sleep: (ms: number) => Promise<void>;
}

const DEFAULT_TIMING: ExperimentTiming = {
  pollMs: 250,
  motorStopTimeoutMs: 30_000,
  spinDownTimeoutMs: 60_000,
  rpmFreshMs: 2_000,
  stoppedRotorRpm: 1,
  now: () => performance.now(),
  sleep: (ms) => new Promise((resolve) => globalThis.setTimeout(resolve, ms)),
};

type ExperimentApi = Pick<
  LinkHubApi,
  "experiment" | "startExperiment" | "stopExperiment" | "motorStatus" | "stopMotor" | "status"
>;

export function rotorRpm(record: MessageRecord): number | null {
  if (record.direction !== "rx") return null;
  if (record.message === "NAMED_VALUE_FLOAT" && record.fields.name === "SIM_RPM") {
    const value = record.fields.value;
    return typeof value === "number" && Number.isFinite(value) ? Math.abs(value) : null;
  }
  if (record.message === "RPM") {
    const value = record.fields.rpm1;
    return typeof value === "number" && Number.isFinite(value) && value >= 0
      ? value / MOTOR_TO_ROTOR_GEAR_RATIO
      : null;
  }
  return null;
}

export function formatElapsed(ms: number): string {
  const total = Math.max(0, Math.floor(ms / 1_000));
  const hours = Math.floor(total / 3_600);
  const minutes = Math.floor((total % 3_600) / 60);
  const seconds = total % 60;
  const pad = (value: number) => String(value).padStart(2, "0");
  return hours > 0 ? `${hours}:${pad(minutes)}:${pad(seconds)}` : `${pad(minutes)}:${pad(seconds)}`;
}

function motorSpinning(status: MotorStatus): boolean {
  return status.available && (status.running || status.stopping || status.speed_percent > 0);
}

function message(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/**
 * Owns the experiment lifecycle shared by the browser and CLI. Arm and motor
 * actions require an active experiment; stopping ramps the bench motor down,
 * waits for the rotor to stop, applies safe-off, then has LinkHub save the
 * experiment's journal window.
 */
export class ExperimentController {
  private current: ExperimentInfo | null = null;
  private currentStage: ExperimentStage = "idle";
  private stopping: Promise<SavedExperiment> | null = null;
  private rpm: { value: number; at: number } | null = null;
  private readonly listeners = new Set<() => void>();
  private readonly timing: ExperimentTiming;

  constructor(
    private readonly api: ExperimentApi,
    telemetry: Pick<TelemetryStore, "onRecord">,
    private readonly passive: Pick<PassiveController, "phase" | "stop">,
    private readonly write: (message: string, kind?: "info" | "error") => void,
    timing: Partial<ExperimentTiming> = {},
  ) {
    this.timing = { ...DEFAULT_TIMING, ...timing };
    telemetry.onRecord((record) => {
      const value = rotorRpm(record);
      if (value !== null) this.rpm = { value, at: this.timing.now() };
    });
  }

  get active(): ExperimentInfo | null {
    return this.current;
  }

  /** True while an experiment is active and not being stopped. */
  get running(): boolean {
    return this.current !== null && this.currentStage === "running";
  }

  get stage(): ExperimentStage {
    return this.currentStage;
  }

  onChange(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  async refresh(): Promise<ExperimentInfo | null> {
    const active = await this.api.experiment();
    if (!this.stopping) {
      this.setState(active, active ? "running" : "idle");
    }
    return active;
  }

  async start(name: string, description: string): Promise<ExperimentInfo> {
    if (this.stopping) throw new Error("The previous experiment is still stopping");
    const active = await this.api.startExperiment(name, description);
    this.setState(active, "running");
    this.write(`Experiment started: ${active.name}`);
    return active;
  }

  /** Throws unless an experiment is active on the server. */
  async requireActive(action: string): Promise<void> {
    const active = this.stopping ? null : await this.refresh();
    if (!active) throw new Error(`${action} refused: start an experiment first`);
  }

  stop(): Promise<SavedExperiment> {
    this.stopping ??= this.shutdown().finally(() => {
      this.stopping = null;
    });
    return this.stopping;
  }

  private async shutdown(): Promise<SavedExperiment> {
    if (!(await this.api.experiment())) {
      this.setState(null, "idle");
      throw new Error("No experiment is running");
    }
    try {
      await this.stopMotor();
      await this.waitForSpinDown();
      await this.disarm();
      this.setStage("saving");
      const saved = await this.api.stopExperiment();
      this.setState(null, "idle");
      this.write(`Experiment saved: ${saved.record_count} records -> ${saved.path}`);
      return saved;
    } catch (error) {
      const active = await this.api.experiment().catch(() => this.current);
      this.setState(active, active ? "running" : "idle");
      this.write(`Experiment stop failed (press Stop to retry): ${message(error)}`, "error");
      throw error;
    }
  }

  private async stopMotor(): Promise<void> {
    this.setStage("stopping-motor");
    const status = await this.api.motorStatus();
    if (!motorSpinning(status)) return;
    this.write("Experiment stop: ramping the bench motor down");
    let latest = await this.api.stopMotor();
    const deadline = this.timing.now() + this.timing.motorStopTimeoutMs;
    while (motorSpinning(latest)) {
      if (this.timing.now() >= deadline) {
        throw new Error("Bench motor did not report stopped; not disarming");
      }
      await this.timing.sleep(this.timing.pollMs);
      latest = await this.api.motorStatus();
    }
  }

  private async waitForSpinDown(): Promise<void> {
    this.setStage("spin-down");
    const deadline = this.timing.now() + this.timing.spinDownTimeoutMs;
    let announced = false;
    for (;;) {
      const now = this.timing.now();
      const rpm = this.rpm && now - this.rpm.at <= this.timing.rpmFreshMs ? this.rpm.value : null;
      if (rpm === null) {
        this.write("Experiment stop: rotor RPM unavailable; continuing after motor stop", "error");
        return;
      }
      if (rpm <= this.timing.stoppedRotorRpm) return;
      if (now >= deadline) {
        throw new Error(`Rotor still turning at ${rpm.toFixed(1)} RPM; not disarming`);
      }
      if (!announced) {
        this.write(`Experiment stop: waiting for rotor to stop (${rpm.toFixed(1)} RPM)`);
        announced = true;
      }
      await this.timing.sleep(this.timing.pollMs);
    }
  }

  private async disarm(): Promise<void> {
    this.setStage("disarming");
    if (this.passive.phase === "idle" && !isArmed(await this.api.status())) return;
    this.write("Experiment stop: applying safe-off");
    await this.passive.stop();
  }

  private setStage(stage: ExperimentStage): void {
    this.setState(this.current, stage);
  }

  private setState(active: ExperimentInfo | null, stage: ExperimentStage): void {
    if (this.current?.id === active?.id && this.currentStage === stage) return;
    this.current = active;
    this.currentStage = stage;
    for (const listener of this.listeners) listener();
  }
}
