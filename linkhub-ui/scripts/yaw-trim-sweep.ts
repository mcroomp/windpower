// Yaw-motor trim sweep through LinkHub (RAWES_MODE=0, armed in ACRO).
//
// Steps H_YAW_TRIM through a ladder, holds each step, and records output 9,
// motor RPM, ESC telemetry and the hub body-z rate. By default the yaw rate
// PID is zeroed (ATC_RAT_YAW_P/I = 0) so output 9 equals the trim; the
// original gains are restored on exit. ESC, q or Ctrl-C aborts. Every exit
// converges on the canonical safe-off state (design/arming.md).
//
// Run from linkhub-ui/: npm exec -- vite-node --script scripts/yaw-trim-sweep.ts [options]

import { mkdirSync, createWriteStream, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { LinkHubApi } from "../src/api";
import { enumName, isArmed, isVehicleHeartbeat } from "../src/mav";
import { PassiveController } from "../src/passive";
import { TelemetryStore } from "../src/telemetry";
import type { JsonObject as Json, MessageRecord } from "../src/types";

interface Options {
  url: string;
  start: number;
  stop: number;
  step: number;
  holdS: number;
  down: boolean;
  closedLoop: boolean;
  force: boolean;
  maxRate: number;
  check: boolean;
}

const USAGE = `Usage: npm exec -- vite-node --script scripts/yaw-trim-sweep.ts [options]
  --url URL          LinkHub base URL (default http://127.0.0.1:8999)
  --start X          first trim (default 0)
  --stop X           last trim (default 0.20, hard cap 0.5)
  --step X           trim step (default 0.01)
  --hold S           seconds per step (default 5)
  --no-down          skip the descending pass (hysteresis check)
  --closed-loop      keep the yaw rate PID active (default: P=I=0, open loop)
  --force            force-arm (bypass pre-arm checks on the secured bench)
  --max-rate R       abort if |hub body-z rate| > R rad/s for 0.5 s (default 10)
  --check            verify link, params and telemetry only; never arms`;

export function parseOptions(argv: string[]): Options {
  const options: Options = {
    url: "http://127.0.0.1:8999",
    start: 0,
    stop: 0.2,
    step: 0.01,
    holdS: 5,
    down: true,
    closedLoop: false,
    force: false,
    maxRate: 10,
    check: false,
  };
  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i];
    const value = (): string => {
      const next = argv[++i];
      if (next === undefined) throw new Error(`${arg} needs a value`);
      return next;
    };
    const number = (): number => {
      const parsed = Number(value());
      if (!Number.isFinite(parsed)) throw new Error(`${arg} needs a number`);
      return parsed;
    };
    switch (arg) {
      case "--url": options.url = value().replace(/\/$/, ""); break;
      case "--start": options.start = number(); break;
      case "--stop": options.stop = number(); break;
      case "--step": options.step = number(); break;
      case "--hold": options.holdS = number(); break;
      case "--no-down": options.down = false; break;
      case "--closed-loop": options.closedLoop = true; break;
      case "--force": options.force = true; break;
      case "--max-rate": options.maxRate = number(); break;
      case "--check": options.check = true; break;
      case "-h": case "--help": console.log(USAGE); process.exit(0);
      default: throw new Error(`Unknown option ${arg}\n${USAGE}`);
    }
  }
  if (options.start < 0 || options.stop > 0.5 || options.start > options.stop) {
    throw new Error("Trim range must satisfy 0 <= start <= stop <= 0.5");
  }
  if (options.step <= 0 || options.holdS < 1) {
    throw new Error("--step must be > 0 and --hold >= 1 s");
  }
  if (options.maxRate <= 0) {
    throw new Error("--max-rate must be > 0");
  }
  if (Math.ceil((options.stop - options.start) / options.step) > 10_000) {
    throw new Error("Trim ladder must have at most 10001 ascending steps");
  }
  return options;
}

export function trimLadder(options: Options): number[] {
  const up: number[] = [];
  const count = Math.floor((options.stop - options.start) / options.step);
  for (let k = 0; k <= count; k++) {
    up.push(Math.min(options.stop, options.start + k * options.step));
  }
  if (up.at(-1) !== options.stop) {
    up.push(options.stop);
  }
  return options.down ? [...up, ...up.slice(0, -1).reverse()] : up;
}

// ---------------------------------------------------------------------------
// LinkHub HTTP client
// ---------------------------------------------------------------------------

function statusText(fields: Json): string {
  const text = fields.text;
  if (Array.isArray(text)) {
    return String.fromCharCode(...text.filter((c): c is number => typeof c === "number" && c > 0));
  }
  return String(text ?? "");
}

// ---------------------------------------------------------------------------
// Telemetry recorder
// ---------------------------------------------------------------------------

const STREAMS = [
  "SERVO_OUTPUT_RAW", "RPM", "ATTITUDE_QUATERNION", "ESC_TELEMETRY_9_TO_12",
  "HEARTBEAT", "STATUSTEXT",
];

const CSV_HEADER = "t_s,trim_cmd,step,servo9_us,rpm1,esc_rpm9,esc_cur9_a,gz_rad_s,armed";

interface Sample {
  t: number;
  step: number;
  trim: number;
  servo9: number;
  rpm1: number;
  escRpm: number;
  gz: number;
}

class Recorder {
  readonly hub: LinkHubApi;
  readonly csv: ReturnType<typeof createWriteStream>;
  readonly t0 = performance.now();
  readonly samples: Sample[] = [];
  readonly texts: { t: number; text: string }[] = [];
  cursor: string;
  armed = false;
  rpm1 = NaN;
  escRpm = NaN;
  escCurrent = NaN;
  gz = NaN;
  gzSeenAt = -Infinity;
  servoSeenAt = -Infinity;
  heartbeatSeenAt = -Infinity;
  step = -1;
  trim = 0;
  running = true;
  failure: Error | null = null;
  private loop: Promise<void> | null = null;

  constructor(
    hub: LinkHubApi,
    cursor: string,
    csvPath: string,
    private readonly generation: string,
    private readonly targetSystem: number,
  ) {
    this.hub = hub;
    this.cursor = cursor;
    this.csv = createWriteStream(csvPath);
    this.csv.on("error", (error) => {
      this.failure = error;
    });
    this.csv.write(`${CSV_HEADER}\n`);
  }

  now(): number {
    return (performance.now() - this.t0) / 1000;
  }

  start(): void {
    this.loop = (async () => {
      while (this.running) {
        try {
          const batch = await this.hub.queryMessages({
            after: this.cursor, waitMs: 250, messages: STREAMS, direction: "rx", limit: 5000,
            expectedGeneration: this.generation,
          });
          this.cursor = batch.next_cursor;
          for (const record of batch.records) this.ingest(record);
        } catch (error) {
          if (this.running) {
            this.failure = error instanceof Error ? error : new Error(String(error));
            await sleep(200);
          }
        }
      }
    })();
  }

  async close(): Promise<void> {
    this.running = false;
    await this.loop;
    await new Promise<void>((resolve) => this.csv.end(resolve));
  }

  private ingest(record: MessageRecord): void {
    if (record.system_id !== this.targetSystem || record.component_id !== 1) return;
    const f = record.fields;
    const t = this.now();
    switch (record.message) {
      case "HEARTBEAT":
        if (record.component_id === 1 && isVehicleHeartbeat(f)) {
          this.armed = isArmed({ base_mode: String(f.base_mode) });
          this.heartbeatSeenAt = t;
        }
        break;
      case "STATUSTEXT": {
        const text = statusText(f);
        this.texts.push({ t, text });
        console.log(`  [fc] ${text}`);
        break;
      }
      case "RPM":
        this.rpm1 = Number(f.rpm1);
        break;
      case "ESC_TELEMETRY_9_TO_12": {
        const rpm = f.rpm as number[] | undefined;
        const current = f.current as number[] | undefined;
        this.escRpm = Number(rpm?.[0] ?? NaN);
        this.escCurrent = Number(current?.[0] ?? NaN) / 100;
        break;
      }
      case "ATTITUDE_QUATERNION":
        this.gz = Number(f.yawspeed);
        if (Number.isFinite(this.gz)) this.gzSeenAt = t;
        break;
      case "SERVO_OUTPUT_RAW": {
        const servo9 = Number(f.servo9_raw);
        if (Number.isFinite(servo9)) this.servoSeenAt = t;
        this.csv.write([
          t.toFixed(3), this.trim.toFixed(4), this.step, servo9,
          fmt(this.rpm1), fmt(this.escRpm), fmt(this.escCurrent), fmt(this.gz),
          this.armed ? 1 : 0,
        ].join(",") + "\n");
        if (this.step >= 0) {
          this.samples.push({
            t, step: this.step, trim: this.trim, servo9,
            rpm1: this.rpm1, escRpm: this.escRpm, gz: this.gz,
          });
        }
        break;
      }
    }
  }
}

function fmt(value: number): string {
  return Number.isFinite(value) ? String(Math.round(value * 1e4) / 1e4) : "";
}

// ---------------------------------------------------------------------------
// Abort handling
// ---------------------------------------------------------------------------

const abort = new AbortController();

function requestAbort(reason: string): void {
  if (!abort.signal.aborted) {
    console.log(`\n  ABORT: ${reason}`);
    abort.abort(new Error(reason));
  }
}

function installAbortKeys(): () => void {
  const onSigint = () => requestAbort("SIGINT");
  process.on("SIGINT", onSigint);
  if (!process.stdin.isTTY) {
    console.log("  [WARN] stdin is not a TTY; only Ctrl-C/SIGINT can abort.");
    return () => process.off("SIGINT", onSigint);
  }
  process.stdin.setRawMode(true);
  process.stdin.resume();
  const onData = (data: Buffer) => {
    for (const byte of data) {
      if (byte === 0x1b || byte === 0x03 || byte === 0x71 || byte === 0x51) {
        requestAbort(byte === 0x1b ? "ESC pressed" : "operator stop");
      }
    }
  };
  process.stdin.on("data", onData);
  return () => {
    process.off("SIGINT", onSigint);
    process.stdin.off("data", onData);
    process.stdin.setRawMode(false);
    process.stdin.pause();
  };
}

function sleep(ms: number, signal?: AbortSignal): Promise<void> {
  return new Promise((resolve, reject) => {
    if (signal?.aborted) return reject(signal.reason);
    const timer = setTimeout(() => {
      signal?.removeEventListener("abort", onAbort);
      resolve();
    }, ms);
    const onAbort = () => {
      clearTimeout(timer);
      reject(signal?.reason);
    };
    signal?.addEventListener("abort", onAbort, { once: true });
  });
}

export async function waitFor(
  predicate: () => boolean, timeoutMs: number, what: string, signal: AbortSignal,
): Promise<void> {
  const deadline = performance.now() + timeoutMs;
  while (true) {
    signal.throwIfAborted();
    if (predicate()) return;
    if (performance.now() > deadline) throw new Error(`Timed out waiting for ${what}`);
    await sleep(50, signal);
  }
}

// ---------------------------------------------------------------------------
// Sweep
// ---------------------------------------------------------------------------

interface StepSummary {
  step: number;
  trim: number;
  n: number;
  servo9_us: number;
  rpm1: number;
  rpm1_std: number;
  esc_rpm9: number;
  gz_rad_s: number;
  gz_std: number;
}

function mean(values: number[]): number {
  const finite = values.filter(Number.isFinite);
  return finite.length ? finite.reduce((a, b) => a + b, 0) / finite.length : NaN;
}

function std(values: number[]): number {
  const finite = values.filter(Number.isFinite);
  const m = mean(finite);
  return finite.length > 1
    ? Math.sqrt(finite.reduce((a, b) => a + (b - m) ** 2, 0) / (finite.length - 1))
    : NaN;
}

function summarize(samples: Sample[], step: number, trim: number, windowStart: number): StepSummary {
  const window = samples.filter((s) => s.step === step && s.t >= windowStart);
  return {
    step, trim, n: window.length,
    servo9_us: mean(window.map((s) => s.servo9)),
    rpm1: mean(window.map((s) => s.rpm1)),
    rpm1_std: std(window.map((s) => s.rpm1)),
    esc_rpm9: mean(window.map((s) => s.escRpm)),
    gz_rad_s: mean(window.map((s) => s.gz)),
    gz_std: std(window.map((s) => s.gz)),
  };
}

function printStep(s: StepSummary): void {
  const f = (v: number, d: number, w: number) => (Number.isFinite(v) ? v.toFixed(d) : "-").padStart(w);
  console.log(
    `  ${String(s.step).padStart(3)}  trim ${s.trim.toFixed(3)}  servo9 ${f(s.servo9_us, 0, 5)} us`
    + `  rpm1 ${f(s.rpm1, 0, 6)} ±${f(s.rpm1_std, 0, 4)}  esc ${f(s.esc_rpm9, 0, 6)}`
    + `  gz ${f(s.gz_rad_s, 3, 7)} ±${f(s.gz_std, 3, 5)} rad/s  (n=${s.n})`,
  );
}

export async function safeOff(hub: LinkHubApi, restore: Map<string, number>): Promise<void> {
  const errors: Error[] = [];
  const attempt = async (label: string, action: () => Promise<unknown>) => {
    try {
      await action();
      console.log(`    ${label}: OK`);
    } catch (error) {
      errors.push(new Error(label, { cause: error }));
      console.log(`    ${label}: FAILED ${error instanceof Error ? error.message : String(error)}`);
    }
  };
  const controller = new PassiveController(hub, new TelemetryStore(hub), (message, kind) => {
    console.log(`    ${message}`);
    if (kind === "error") errors.push(new Error(message));
  }, () => {});
  await attempt("canonical safe-off", () => controller.stop());
  for (const [name, value] of restore) {
    await attempt(`${name}=${value}`, () => hub.setParameter(name, value, false));
  }
  await attempt("disarmed + output 9 off", async () => {
    const status = await hub.status();
    if (!status.ready) throw new Error("LinkHub not ready to verify safe-off");
    if (isArmed(status)) throw new Error("still armed");
    if (status.custom_mode !== 1) throw new Error("ACRO mode not confirmed");
    const servoMin = await hub.getParameter("SERVO9_MIN");
    const batch = await hub.queryMessages({
      after: status.cursor, waitMs: 1500, messages: ["SERVO_OUTPUT_RAW"], direction: "rx",
      expectedGeneration: status.generation,
    });
    const servo9 = Number(batch.records.at(-1)?.fields.servo9_raw ?? NaN);
    if (!Number.isFinite(servo9) || servo9 > servoMin.value) {
      throw new Error(`fresh servo9_raw=${servo9}, expected <= SERVO9_MIN=${servoMin.value}`);
    }
  });
  if (errors.length > 0) throw new AggregateError(errors, "Safe-off verification failed");
}

async function main(): Promise<void> {
  const options = parseOptions(process.argv.slice(2));
  const hub = new LinkHubApi(options.url);
  const ladder = trimLadder(options);

  const status = await hub.status();
  if (!status.ready) throw new Error("LinkHub not ready");
  if (isArmed(status)) throw new Error("Vehicle is already armed; refusing to start");
  const [
    { value: tail }, { value: motorFunction }, { value: ratP },
    { value: ratI }, { value: ramp }, { value: runup },
  ] = await Promise.all([
    hub.getParameter("H_TAIL_TYPE"), hub.getParameter("SERVO9_FUNCTION"),
    hub.getParameter("ATC_RAT_YAW_P"), hub.getParameter("ATC_RAT_YAW_I"),
    hub.getParameter("H_RSC_RAMP_TIME"), hub.getParameter("H_RSC_RUNUP_TIME"),
  ]);
  if (Math.round(tail) !== 3) throw new Error(`H_TAIL_TYPE=${tail}, expected 3 (DDFP)`);
  if (Math.round(motorFunction) !== 36) throw new Error(`SERVO9_FUNCTION=${motorFunction}, expected 36 at boot`);

  const repoRoot = fileURLToPath(new URL("../../", import.meta.url));
  const stamp = new Date().toISOString().replace(/[-:]/g, "").replace(/\..*/, "").replace("T", "_");
  const outDir = path.join(repoRoot, "simulation", "logs", "calibrate", `yaw_trim_sweep_${stamp}`);
  mkdirSync(outDir, { recursive: true });

  console.log(`  LinkHub ${options.url}  connection ${String(status.connection)}`);
  console.log(`  ATC_RAT_YAW_P=${ratP.toFixed(4)} I=${ratI.toFixed(4)}  `
    + `mode: ${options.closedLoop ? "closed loop (yaw PID active)" : "open loop (P=I=0 during sweep)"}`);
  console.log(`  Ladder (${ladder.length} steps x ${options.holdS}s): ${ladder.join(" ")}`);
  console.log(`  Logs: ${outDir}`);

  const startCursor = String(status.cursor);
  const recorder = new Recorder(
    hub, startCursor, path.join(outDir, "samples.csv"), status.generation, status.target_system,
  );
  recorder.start();

  if (options.check) {
    await sleep(2000);
    await recorder.close();
    const rows = recorder.samples.length;
    console.log(`  Check: armed=${recorder.armed} rpm1=${fmt(recorder.rpm1)} gz=${fmt(recorder.gz)}`
      + ` servo stream ${recorder.servoSeenAt > 0 ? "OK" : "MISSING"} attitude stream `
      + `${recorder.gzSeenAt > 0 ? "OK" : "MISSING"} (step rows ${rows})`);
    if (recorder.failure) throw recorder.failure;
    if (recorder.servoSeenAt < 0 || recorder.gzSeenAt < 0 || recorder.heartbeatSeenAt < 0) {
      throw new Error("Required servo, attitude or heartbeat telemetry is missing");
    }
    return;
  }

  const restore = new Map<string, number>();
  const summaries: StepSummary[] = [];
  const removeKeys = installAbortKeys();
  console.log("  Press ESC, q or Ctrl-C at any time to stop.");
  let error: unknown = null;
  try {
    await hub.setParameter("H_FLYBAR_MODE", 1, true);
    await hub.setParameter("H_SV_MAN", 0, true);
    await hub.setParameter("RAWES_MODE", 0, true);
    await hub.setParameter("H_YAW_TRIM", 0, false);
    await hub.setModeAndWait(1);
    if (!options.closedLoop) {
      restore.set("ATC_RAT_YAW_P", Math.round(ratP * 1e6) / 1e6);
      restore.set("ATC_RAT_YAW_I", Math.round(ratI * 1e6) / 1e6);
      await hub.setParameter("ATC_RAT_YAW_P", 0, false);
      await hub.setParameter("ATC_RAT_YAW_I", 0, false);
    }
    await waitFor(() => recorder.servoSeenAt > 0 && recorder.gzSeenAt > 0, 5000,
      "SERVO_OUTPUT_RAW + ATTITUDE_QUATERNION streams", abort.signal);

    const textMark = recorder.texts.length;
    console.log(`  ${options.force ? "Force-arming" : "Arming"} in ACRO (RAWES_MODE=0)…`);
    const armResult = await hub.command(
      "MAV_CMD_COMPONENT_ARM_DISARM", [1, options.force ? 21196 : 0], 15_000);
    if (enumName(armResult.result) !== "MAV_RESULT_ACCEPTED") {
      await sleep(500);
      const reason = recorder.texts.slice(textMark).map((x) => x.text)
        .filter((x) => x.startsWith("PreArm") || x.startsWith("Arm")).at(-1);
      throw new Error(`Arm rejected (${enumName(armResult.result)})${reason ? `: ${reason}` : ""}`);
    }
    await waitFor(() => recorder.armed, 5000, "armed heartbeat", abort.signal);
    await waitFor(
      () => recorder.texts.slice(textMark).some((x) => x.text.toLowerCase().includes("runup complete")),
      (Math.max(ramp, runup) + 5.5) * 1000, "heli runup complete", abort.signal);
    console.log("  Runup complete; starting sweep.");

    let overRateSince: number | null = null;
    for (let step = 0; step < ladder.length; step++) {
      abort.signal.throwIfAborted();
      const trim = ladder[step] as number;
      await hub.setParameter("H_YAW_TRIM", trim, false);
      recorder.trim = trim;
      recorder.step = step;
      const stepStart = recorder.now();
      const stepEnd = stepStart + options.holdS;
      while (recorder.now() < stepEnd) {
        await sleep(50, abort.signal);
        if (!recorder.armed) throw new Error("Vehicle disarmed during sweep");
        if (recorder.failure) throw recorder.failure;
        const now = recorder.now();
        if (now - recorder.gzSeenAt > 1.5) throw new Error("Attitude telemetry stalled");
        if (now - recorder.heartbeatSeenAt > 3) throw new Error("Heartbeat telemetry stalled");
        if (now - recorder.servoSeenAt > 1.5) throw new Error("Servo telemetry stalled");
        if (!Number.isFinite(recorder.gz)) throw new Error("Invalid attitude rate telemetry");
        if (Math.abs(recorder.gz) > options.maxRate) {
          overRateSince ??= now;
          if (now - overRateSince > 0.5) {
            throw new Error(`|gz|=${Math.abs(recorder.gz).toFixed(2)} rad/s > --max-rate ${options.maxRate}`);
          }
        } else {
          overRateSince = null;
        }
      }
      const summary = summarize(recorder.samples, step, trim, stepStart + options.holdS * 0.4);
      summaries.push(summary);
      printStep(summary);
    }
    console.log("  Sweep complete.");
  } catch (caught) {
    error = caught;
    console.log(`  Stopped: ${caught instanceof Error ? caught.message : String(caught)}`);
  } finally {
    recorder.step = -1;
    try {
      await safeOff(hub, restore);
    } catch (cleanupError) {
      console.error(cleanupError);
      error = error === null ? cleanupError : new AggregateError([error, cleanupError]);
    } finally {
      removeKeys();
      await recorder.close();
    }
    const endStatus = await hub.status();
    writeFileSync(path.join(outDir, "summary.json"), JSON.stringify({
      options, ladder, start_cursor: startCursor, end_cursor: endStatus.cursor,
      original_gains: { ATC_RAT_YAW_P: ratP, ATC_RAT_YAW_I: ratI },
      aborted: error !== null,
      error: error instanceof Error ? error.message : error === null ? null : String(error),
      steps: summaries,
    }, null, 2));
    console.log(`  Wrote ${path.join(outDir, "summary.json")} and samples.csv`);
  }
  if (error !== null) process.exitCode = 1;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  main().catch((error) => {
    console.error(`  ERROR: ${error instanceof Error ? error.message : String(error)}`);
    process.exitCode = 1;
  });
}
