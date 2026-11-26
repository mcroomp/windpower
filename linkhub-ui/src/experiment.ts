import type { LinkHubApi } from "./api";
import { recordingProblem } from "./camera-recording";
import { isArmed } from "./mav";
import type { PassiveController } from "./passive";
import type { TelemetryStore } from "./telemetry";
import type { CameraPhone, CameraStatus, ExperimentInfo, MessageRecord, MotorStatus, SavedExperiment } from "./types";
import { MOTOR_TO_ROTOR_GEAR_RATIO } from "./vehicle-motion";

export type ExperimentStage = "idle" | "starting" | "running"
  | "stopping-motor" | "spin-down" | "disarming" | "saving";

export interface VideoCheck {
  state: "pending" | "ready" | "failed";
  detail: string;
}

type CameraBaseline = CameraStatus | { error: string };

export interface ExperimentTiming {
  pollMs: number;
  motorStopTimeoutMs: number;
  spinDownTimeoutMs: number;
  recordingTimeoutMs: number;
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
  recordingTimeoutMs: 15_000,
  rpmFreshMs: 2_000,
  stoppedRotorRpm: 1,
  now: () => performance.now(),
  sleep: (ms) => new Promise((resolve) => globalThis.setTimeout(resolve, ms)),
};

type ExperimentApi = Pick<
  LinkHubApi,
  "experiment" | "startExperiment" | "stopExperiment" | "markExperiment" | "motorStatus" | "stopMotor"
  | "status" | "cameraStatus" | "onboardLogEvents"
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

/** Null once LinkHub has saved the start-of-experiment parameters and scripts. */
function snapshotProblem(active: ExperimentInfo): string | null {
  switch (active.snapshot.state) {
    case "complete": return null;
    case "capturing": return "capturing parameter and script snapshot";
    case "failed": return `parameter and script snapshot failed: ${active.snapshot.error ?? "unknown error"}`;
  }
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
  private starting = false;
  private readonly expectedCameras = new Map<string, CameraPhone>();
  private cameraBaseline: Promise<CameraBaseline> | null = null;
  private video: VideoCheck = { state: "pending", detail: "Checking connected cameras…" };
  private videoDeadline = 0;
  private videoWasReady = false;
  private generation = 0;
  private refreshing: Promise<ExperimentInfo | null> | null = null;
  private readonly pendingLogs = new Map<string, { name: string; cursor: string }>();
  private logRefresh: Promise<void> | null = null;
  private logRefreshError: string | null = null;
  private rpm: { value: number; at: number } | null = null;
  private readonly listeners = new Set<() => void>();
  private readonly timing: ExperimentTiming;

  constructor(
    private readonly api: ExperimentApi,
    private readonly telemetry: Pick<TelemetryStore, "onRecord" | "profile">,
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

  get controlsReady(): boolean {
    return this.motorLeaseExperimentId !== null && this.video.state === "ready";
  }

  get motorLeaseExperimentId(): string | null {
    return this.running && !this.stopping && this.current?.snapshot.state === "complete"
      ? this.current.id : null;
  }

  get videoCheck(): VideoCheck {
    return this.video;
  }

  refreshOnboardLogs(): Promise<void> {
    if (this.logRefresh) return this.logRefresh;
    if (!this.pendingLogs.size) return Promise.resolve();
    this.logRefresh = this.readOnboardLogEvents().finally(() => { this.logRefresh = null; });
    return this.logRefresh;
  }

  private async readOnboardLogEvents(): Promise<void> {
    const results = await Promise.allSettled([...this.pendingLogs].map(async ([id, pending]) => {
      const batch = await this.api.onboardLogEvents(pending.cursor);
      pending.cursor = batch.next_cursor;
      for (const { data: { fields } } of batch.records) {
        const summary = fields.onboard_logs;
        if (fields.experiment_id !== id || !summary || typeof summary !== "object" || Array.isArray(summary)
          || !("state" in summary)) continue;
        const state = summary.state;
        const count = "logs" in summary && Array.isArray(summary.logs) ? summary.logs.length : 0;
        const error = "error" in summary && typeof summary.error === "string" ? `: ${summary.error}` : "";
        let detail: string;
        switch (state) {
          case "downloading": detail = "Downloading onboard logs (.BIN)…"; break;
          case "progress":
            if (!("id" in summary) || typeof summary.id !== "number"
              || !("percent" in summary) || typeof summary.percent !== "number"
              || !("received_bytes" in summary) || typeof summary.received_bytes !== "number"
              || !("total_bytes" in summary) || typeof summary.total_bytes !== "number") {
              this.write(`${pending.name}: Invalid onboard log progress report`, "error");
              continue;
            }
            detail = `Downloading onboard-${summary.id}.BIN: ${summary.percent}% `
              + `(${summary.received_bytes.toLocaleString()} / ${summary.total_bytes.toLocaleString()} bytes)`;
            break;
          case "complete": detail = count ? `Onboard log downloads complete (${count} .BIN ${count === 1 ? "file" : "files"})`
            : "No new onboard logs to download"; break;
          case "failed": detail = `Onboard log download failed${error}`; break;
          case "skipped": detail = `Onboard log download skipped${error}`; break;
          default: continue;
        }
        this.write(`${pending.name}: ${detail}`, state === "failed" ? "error" : "info");
        if (state !== "downloading" && state !== "progress") {
          this.pendingLogs.delete(id);
          break;
        }
      }
    }));
    const failure = results.find((result) => result.status === "rejected");
    const detail = failure ? message(failure.reason) : null;
    if (detail && detail !== this.logRefreshError) {
      this.write(`Onboard log status unavailable; retrying: ${detail}`, "error");
    }
    this.logRefreshError = detail;
  }

  get stage(): ExperimentStage {
    return this.currentStage;
  }

  onChange(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  refresh(): Promise<ExperimentInfo | null> {
    if (this.starting || this.stopping) return Promise.resolve(this.current);
    if (this.refreshing) return this.refreshing;
    const task = this.refreshStatus(this.generation).finally(() => {
      if (this.refreshing === task) this.refreshing = null;
    });
    this.refreshing = task;
    return task;
  }

  private async refreshStatus(generation: number): Promise<ExperimentInfo | null> {
    const active = await this.api.experiment();
    if (generation !== this.generation) return this.current;
    this.setState(active, active ? "running" : "idle");
    if (!active) return null;
    try {
      const [camera, baseline] = await Promise.all([this.api.cameraStatus(), this.cameraBaseline]);
      if (generation !== this.generation) return this.current;
      if (baseline && "error" in baseline) {
        this.setVideoCheck({ state: "failed", detail: `Initial camera check unavailable: ${baseline.error}; restart to verify` });
        return active;
      }
      this.cameraBaseline = null;
      for (const phone of baseline?.phones ?? []) this.expectedCameras.set(phone.stream, phone);
      const problem = this.checkRecording(active, camera);
      if (problem) {
        const failed = this.videoWasReady || this.timing.now() >= this.videoDeadline;
        this.setVideoCheck({ state: failed ? "failed" : "pending", detail: problem });
      } else {
        this.videoWasReady = true;
        this.setVideoCheck({
          state: "ready",
          detail: this.expectedCameras.size ? "Recordings verified" : "No cameras connected",
        });
      }
    } catch (error) {
      if (generation !== this.generation) return this.current;
      this.setVideoCheck({ state: "failed", detail: `Camera status unavailable: ${message(error)}` });
    }
    return active;
  }

  async start(name: string, description: string): Promise<ExperimentInfo> {
    if (this.stopping) throw new Error("The previous experiment is still stopping");
    if (this.starting || this.current) throw new Error("An experiment is already active or starting");
    this.starting = true;
    this.invalidateRefresh();
    this.setState(null, "starting");
    const baseline = this.api.cameraStatus().catch((error: unknown) => ({ error: message(error) }));
    let active: ExperimentInfo;
    try {
      active = await this.api.startExperiment(name, description, this.telemetry.profile);
    } catch (error) {
      this.setState(null, "idle");
      this.write(`Experiment start failed: ${message(error)}`, "error");
      throw error;
    } finally {
      this.starting = false;
    }
    this.cameraBaseline = baseline;
    this.write(`Experiment started: ${active.name}`);
    this.setState(active, "running");
    void this.refresh().catch((error: unknown) => {
      this.write(`Experiment status unavailable: ${message(error)}`, "error");
    });
    return active;
  }

  /** Arm and motor actions still require verified experiment checks. */
  async requireActive(action: string): Promise<void> {
    if (this.starting) throw new Error(`${action} refused: experiment is starting`);
    const generation = this.generation;
    const active = this.stopping ? null : await this.refresh();
    if (!active || generation !== this.generation || !this.running || this.stopping) {
      throw new Error(`${action} refused: start an experiment first`);
    }
    const problem = snapshotProblem(active) ?? (this.video.state === "ready" ? null : this.video.detail);
    if (problem) throw new Error(`${action} refused: ${problem}`);
  }

  private checkRecording(active: ExperimentInfo, camera: CameraStatus): string | null {
    for (const phone of camera.phones) this.expectedCameras.set(phone.stream, phone);
    if (camera.experiment_id !== active.id) return "Camera status does not match the active experiment";
    return [...this.expectedCameras.values()]
      .map((phone) => recordingProblem(phone, camera))
      .filter((problem) => problem !== null)
      .join("; ") || null;
  }

  private setVideoCheck(video: VideoCheck): void {
    if (this.video.state === video.state && this.video.detail === video.detail) return;
    this.video = video;
    this.reportVideo();
    for (const listener of this.listeners) listener();
  }

  /** Journals a labelled segment boundary in the active experiment. */
  async mark(label: string): Promise<void> {
    if (!this.current) throw new Error("mark refused: start an experiment first");
    const mark = await this.api.markExperiment(label);
    this.write(`Mark at ${formatElapsed(mark.elapsed_s * 1_000)}: ${mark.label}`);
  }

  stop(): Promise<SavedExperiment> {
    if (this.starting) return Promise.reject(new Error("The experiment is still starting"));
    if (!this.stopping) {
      this.invalidateRefresh();
      this.stopping = Promise.resolve().then(() => this.shutdown()).finally(() => {
        this.stopping = null;
        for (const listener of this.listeners) listener();
      });
      this.setStage("stopping-motor");
    }
    return this.stopping;
  }

  private async shutdown(): Promise<SavedExperiment> {
    try {
      const active = await this.api.experiment();
      if (!active) {
        this.setState(null, "idle");
        throw new Error("No experiment is running");
      }
      this.setState(active, "stopping-motor");
      await this.stopMotor();
      await this.waitForSpinDown();
      await this.disarm();
      this.setStage("saving");
      const saved = await this.api.stopExperiment();
      this.pendingLogs.set(saved.experiment.id, { name: saved.experiment.name, cursor: saved.end_cursor });
      this.setState(null, "idle");
      this.write(
        `Experiment saved: ${saved.record_count} records -> ${saved.path}. `
        + "Checking for new onboard logs (.BIN). Phone recordings finish uploading to the same directory.",
      );
      return saved;
    } catch (error) {
      const active = this.current && await this.api.experiment().catch(() => this.current);
      this.setState(active, active ? "running" : "idle");
      this.write(`Experiment stop failed (press Stop to retry): ${message(error)}`, "error");
      throw error;
    }
  }

  private async stopMotor(): Promise<void> {
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

  private invalidateRefresh(): void {
    this.generation++;
    this.refreshing = null;
  }

  private reportSnapshot(active: ExperimentInfo): void {
    const snapshot = active.snapshot;
    switch (snapshot.state) {
      case "capturing": this.write("Downloading parameters and scripts…"); break;
      case "complete": {
        const scripts = snapshot.scripts.map(({ name }) => name).join(", ");
        this.write(`Parameter/script snapshot saved (${snapshot.parameter_count} parameters; ${scripts || "no scripts"})`);
        break;
      }
      case "failed": this.write(`Parameter/script snapshot failed: ${snapshot.error ?? "unknown error"}`, "error"); break;
    }
  }

  private reportVideo(): void {
    if (this.video.state === "failed") {
      this.write(`Video check failed; arming and motor starts locked: ${this.video.detail}`, "error");
    } else {
      this.write(this.video.state === "pending" ? `Waiting for cameras: ${this.video.detail}` : `Video: ${this.video.detail}`);
    }
  }

  private setState(active: ExperimentInfo | null, stage: ExperimentStage): void {
    const changedExperiment = this.current?.id !== active?.id;
    const snapshotChanged = this.current?.snapshot.state !== active?.snapshot.state
      || this.current?.snapshot.error !== active?.snapshot.error;
    const changed = changedExperiment || this.currentStage !== stage
      || snapshotChanged;
    if (changedExperiment) {
      this.expectedCameras.clear();
      this.video = { state: "pending", detail: "Checking connected cameras…" };
      this.videoWasReady = false;
      this.videoDeadline = this.timing.now() + this.timing.recordingTimeoutMs;
      if (this.current || !active) this.cameraBaseline = null;
    }
    this.current = active;
    this.currentStage = stage;
    if (active && stage === "running") {
      if (changedExperiment || snapshotChanged) this.reportSnapshot(active);
      if (changedExperiment) this.reportVideo();
    }
    if (changed) for (const listener of this.listeners) listener();
  }
}
