import { formatMavResult, type LinkHubApi } from "./api";
import { MagCalStatus, MavCmd, MavResult } from "./generated/protocol";
import { enumIs, enumName, isArmed } from "./mav";
import { cursorSequence, type TelemetryStore } from "./telemetry";
import type { JsonObject, MessageRecord } from "./types";

export interface CompassCalibrationOptions {
  /** Bitmask of compasses to calibrate; 0 means all. */
  mask: number;
  retryOnFailure: boolean;
  /** Save offsets without waiting for `compass accept`. */
  autosave: boolean;
  delaySeconds: number;
}

export type CompassCommand =
  | { action: "calibrate"; options: CompassCalibrationOptions }
  | { action: "cancel" }
  | { action: "accept" };

export const COMPASS_USAGE =
  "Usage: compass calibrate [--mask N] [--retry] [--autosave] [--delay S] | compass cancel | compass accept";

const CALIBRATION_TIMEOUT_MS = 15 * 60_000;
const NO_PROGRESS_WARNING_MS = 10_000;
const PROGRESS_STEP_PERCENT = 5;
// ArduPilot only streams these in its EXTRA3 group, so they must be requested.
const MAG_CAL_STREAM_HZ = 2;
const FINAL_STATUSES = new Set<string>([
  MagCalStatus.SUCCESS,
  MagCalStatus.FAILED,
  MagCalStatus.BAD_ORIENTATION,
  MagCalStatus.BAD_RADIUS,
]);

export function parseCompassCommand(tokens: string[]): CompassCommand {
  const [action, ...rest] = tokens;
  if ((action === "cancel" || action === "accept") && rest.length === 0) {
    return { action };
  }
  if (action !== "calibrate") {
    throw new Error(COMPASS_USAGE);
  }
  const options: CompassCalibrationOptions = {
    mask: 0,
    retryOnFailure: false,
    autosave: false,
    delaySeconds: 0,
  };
  for (let index = 0; index < rest.length; index += 1) {
    const token = rest[index];
    if (token === "--retry") {
      options.retryOnFailure = true;
    } else if (token === "--autosave") {
      options.autosave = true;
    } else if (token === "--mask" || token === "--delay") {
      const raw = rest[index + 1];
      if (raw === undefined) {
        throw new Error(`${token} requires a value`);
      }
      const value = Number(raw);
      index += 1;
      if (token === "--mask") {
        if (!Number.isInteger(value) || value < 0 || value > 7) {
          throw new Error("--mask must be an integer bitmask 0..7 (0 = all compasses)");
        }
        options.mask = value;
      } else {
        if (!Number.isFinite(value) || value < 0 || value > 60) {
          throw new Error("--delay must be within 0..60 seconds");
        }
        options.delaySeconds = value;
      }
    } else {
      throw new Error(`Unknown compass calibrate option: ${token}\n${COMPASS_USAGE}`);
    }
  }
  return { action: "calibrate", options };
}

function statusLabel(value: unknown): string {
  return enumName(value)?.replace(/^MAG_CAL_/, "") ?? "UNKNOWN";
}

function numberField(fields: JsonObject, name: string): number | null {
  const value = fields[name];
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function triple(fields: JsonObject, prefix: string): string {
  return ["x", "y", "z"]
    .map((axis) => (numberField(fields, `${prefix}_${axis}`) ?? NaN).toFixed(2))
    .join(", ");
}

export function formatMagCalProgress(fields: JsonObject): string {
  const attempt = numberField(fields, "attempt");
  const percent = numberField(fields, "completion_pct");
  return `compass ${numberField(fields, "compass_id") ?? "?"}  ${statusLabel(fields.cal_status)}`
    + `  ${percent === null ? "?" : percent}%`
    + (attempt && attempt > 1 ? `  (attempt ${attempt})` : "");
}

export function formatMagCalReport(fields: JsonObject): string[] {
  const fitness = numberField(fields, "fitness");
  const lines = [
    `compass ${numberField(fields, "compass_id") ?? "?"}  ${statusLabel(fields.cal_status)}`
    + `  fitness ${fitness === null ? "n/a" : fitness.toFixed(2)}`
    + `  saved ${numberField(fields, "autosaved") ? "yes" : "no"}`,
    `  offsets  ${triple(fields, "ofs")}`,
    `  diag     ${triple(fields, "diag")}`,
    `  offdiag  ${triple(fields, "offdiag")}`,
  ];
  const confidence = numberField(fields, "orientation_confidence");
  if (confidence !== null && confidence > 0) {
    lines.push(`  orientation confidence ${confidence.toFixed(2)}`);
  }
  return lines;
}

function isFinalStatus(value: unknown): boolean {
  const name = enumName(value);
  return name !== null && FINAL_STATUSES.has(name);
}

type Writer = (message: string, kind?: "info" | "error") => void;

/** Starts, streams, cancels and accepts ArduPilot magnetometer calibration. */
export class CompassCalibration {
  private cancelActive: (() => void) | null = null;

  constructor(
    private readonly api: LinkHubApi,
    private readonly telemetry: Pick<TelemetryStore, "onRecord" | "onGeneration">,
    private readonly write: Writer,
  ) {}

  get active(): boolean {
    return this.cancelActive !== null;
  }

  /** Runs one calibration to completion, streaming progress; resolves when it ends. */
  async start(options: CompassCalibrationOptions): Promise<void> {
    if (this.active) {
      throw new Error("A compass calibration is already running (compass cancel to stop it)");
    }
    if (isArmed(await this.api.status())) {
      throw new Error("compass calibrate refused: vehicle is ARMED");
    }

    let expected = options.mask;
    let finished = 0;
    let anyFailure = false;
    let needsAccept = false;
    let sawProgress = false;
    let cancelled = false;
    let startSequence: number | null = null;
    const buffered: MessageRecord[] = [];
    const printed = new Map<string, string>();
    let settle: (error?: Error) => void = () => {};
    const done = new Promise<void>((resolve, reject) => {
      settle = (error) => (error ? reject(error) : resolve());
    });

    let ended = false;
    const finalize = (error?: Error) => {
      if (ended) {
        return;
      }
      ended = true;
      this.cancelActive = null;
      disposeRecords();
      disposeGeneration();
      globalThis.clearTimeout(timeout);
      globalThis.clearTimeout(noProgressWarning);
      // Back to the vehicle's own stream rates (interval 0 = default).
      void this.api.setMessageRates({ MAG_CAL_PROGRESS: null, MAG_CAL_REPORT: null })
        .catch(() => undefined);
      settle(error);
    };

    const handle = (record: MessageRecord) => {
      // Ignore anything the vehicle sent before it accepted this start command,
      // such as repeated reports from an earlier calibration.
      if (startSequence === null || cursorSequence(record.cursor) <= startSequence) {
        return;
      }
      const compassId = numberField(record.fields, "compass_id");
      if (compassId === null) {
        return;
      }
      if (options.mask !== 0 && (options.mask & (1 << compassId)) === 0) {
        return;
      }
      if (options.mask === 0) {
        expected |= numberField(record.fields, "cal_mask") ?? 0;
      }
      if (record.message === "MAG_CAL_PROGRESS") {
        sawProgress = true;
        const percent = numberField(record.fields, "completion_pct") ?? 0;
        const key = `${statusLabel(record.fields.cal_status)}:${String(record.fields.attempt)}:`
          + `${Math.floor(percent / PROGRESS_STEP_PERCENT)}`;
        if (printed.get(`progress:${compassId}`) !== key) {
          printed.set(`progress:${compassId}`, key);
          this.write(formatMagCalProgress(record.fields));
        }
        return;
      }
      // MAG_CAL_REPORT repeats at the stream rate while a result is held.
      const status = record.fields.cal_status;
      const success = enumIs(status, MagCalStatus.SUCCESS);
      const saved = Boolean(numberField(record.fields, "autosaved"));
      const key = `${statusLabel(status)}:${saved}`;
      if (printed.get(`report:${compassId}`) !== key) {
        printed.set(`report:${compassId}`, key);
        this.write(formatMagCalReport(record.fields).join("\n"));
        if (!success && isFinalStatus(status) && options.retryOnFailure) {
          this.write(`compass ${compassId} failed; the vehicle is retrying`, "error");
        }
      }
      if (!isFinalStatus(status) || (!success && options.retryOnFailure)) {
        return;
      }
      // With autosave the vehicle saves just after reporting success.
      if (success && options.autosave && !saved) {
        return;
      }
      finished |= 1 << compassId;
      anyFailure ||= !success;
      needsAccept ||= success && !saved;
      if (expected !== 0 && (finished & expected) === expected) {
        finalize();
      }
    };

    const disposeRecords = this.telemetry.onRecord((record) => {
      if (record.direction !== "rx"
        || (record.message !== "MAG_CAL_PROGRESS" && record.message !== "MAG_CAL_REPORT")) {
        return;
      }
      if (startSequence === null) {
        buffered.push(record);
      } else {
        handle(record);
      }
    });
    const disposeGeneration = this.telemetry.onGeneration(() => {
      finalize(new Error("Link generation changed during compass calibration"));
    });
    const timeout = globalThis.setTimeout(
      () => finalize(new Error("Compass calibration timed out after 15 minutes")),
      CALIBRATION_TIMEOUT_MS,
    );
    const noProgressWarning = globalThis.setTimeout(() => {
      if (this.active && !sawProgress) {
        this.write(
          "No MAG_CAL_PROGRESS received yet; waiting (is the vehicle waiting out --delay?).",
          "error",
        );
      }
    }, NO_PROGRESS_WARNING_MS);
    this.cancelActive = () => {
      cancelled = true;
      this.write("Compass calibration cancelled.");
      finalize();
    };

    try {
      await this.api.setMessageRates({
        MAG_CAL_PROGRESS: MAG_CAL_STREAM_HZ,
        MAG_CAL_REPORT: MAG_CAL_STREAM_HZ,
      });
      const result = await this.api.command(MavCmd.DO_START_MAG_CAL, [
        options.mask,
        options.retryOnFailure ? 1 : 0,
        options.autosave ? 1 : 0,
        options.delaySeconds,
        0,
      ]);
      if (!enumIs(result.result, MavResult.ACCEPTED)) {
        throw new Error(
          `Compass calibration rejected with ${formatMavResult(result.result)}. `
          + "The vehicle must be disarmed, and a compass must be healthy with its "
          + "COMPASS_USE/USE2/USE3 parameter enabled to be calibrated.",
        );
      }
      startSequence = cursorSequence(result.after_cursor);
    } catch (error) {
      cancelled = true;
      finalize();
      throw error;
    }
    for (const record of buffered.splice(0)) {
      handle(record);
    }

    if (this.active) {
      this.write(
        `Compass calibration started (${options.mask === 0 ? "all compasses" : `mask ${options.mask}`}`
        + `, ${options.autosave ? "autosave" : "manual accept"}${options.retryOnFailure ? ", retry" : ""}).`,
      );
      this.write(
        "Rotate the vehicle slowly through every orientation, away from metal, magnets and "
        + "motor current. Esc or 'compass cancel' stops.",
      );
    }
    await done;
    if (cancelled || finished === 0) {
      return;
    }
    if (anyFailure) {
      this.write("Compass calibration finished with a failure; see the reports above.", "error");
    } else if (needsAccept) {
      this.write("Calibration succeeded. Run 'compass accept' to save the offsets, then reboot to apply.");
    } else {
      this.write("Calibration succeeded and was saved; reboot to apply.");
    }
  }

  /** Tells the vehicle to cancel and stops the local calibration if one is running. */
  async cancel(): Promise<void> {
    const cancelLocal = this.cancelActive;
    const result = await this.api.command(MavCmd.DO_CANCEL_MAG_CAL, [0]);
    if (!enumIs(result.result, MavResult.ACCEPTED)) {
      throw new Error(`Cancel rejected with ${formatMavResult(result.result)}`);
    }
    if (cancelLocal) {
      cancelLocal();
    } else {
      this.write("Compass calibration cancelled.");
    }
  }

  async accept(): Promise<void> {
    const result = await this.api.command(MavCmd.DO_ACCEPT_MAG_CAL, [0]);
    if (!enumIs(result.result, MavResult.ACCEPTED)) {
      throw new Error(
        `Accept rejected with ${formatMavResult(result.result)} `
        + "(there is no successful, unsaved calibration to accept)",
      );
    }
    this.write("Compass calibration accepted and saved; reboot to apply.");
  }
}
