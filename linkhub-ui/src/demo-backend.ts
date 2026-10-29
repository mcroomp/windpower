import recordingData from "./demo/pumping-cycle.json";
import { MavParamType } from "./generated/protocol";
import { heartbeatState, mavEnum } from "./mav";
import { TELEMETRY_PROFILES } from "./telemetry-rates";
import type { JsonObject, LinkHubStatus, MessageRecord, ParameterResult } from "./types";

const MAX_JOURNAL_RECORDS = 60_000;
const READ_ONLY = "Recorded SITL replay is read-only. Vehicle commands, calibration, and parameter changes are disabled.";
const GENERATION = "sitl-replay:1";

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

function parseRecording() {
  const raw = object(recordingData);
  const duration = finite(raw.duration);
  if (raw.version !== 1 || duration <= 0 || !Array.isArray(raw.records) || raw.records.length === 0) {
    throw new Error("Invalid SITL recording");
  }
  let previous = 0;
  const records = raw.records.map((entry: unknown) => {
    if (!Array.isArray(entry) || entry.length !== 3 || typeof entry[1] !== "string") {
      throw new Error("Invalid SITL frame");
    }
    const time = finite(entry[0]);
    if (time < previous || time >= duration) throw new Error("Unordered SITL recording");
    previous = time;
    return { time, message: entry[1], fields: object(entry[2]) };
  });
  const parameterTypes = object(raw.parameterTypes);
  const parameters = Object.entries(object(raw.parameters)).map(([name, value]) => {
    const wireType = parameterTypes[name];
    const type = wireType === undefined
      ? Number.isInteger(value) ? MavParamType.INT32 : MavParamType.REAL32
      : Object.values(MavParamType).find((type) => type === object(wireType).type);
    if (type === undefined) throw new Error(`Invalid recorded parameter type: ${name}`);
    return { name, value: finite(value), type: mavEnum(type) };
  });
  const heartbeat = records.find((record) => record.message === "HEARTBEAT");
  const state = heartbeat && heartbeatState(heartbeat.fields);
  if (!state) throw new Error("SITL recording has no valid vehicle heartbeat");
  return { duration, records, parameters, state };
}

export const pumpingCycle = parseRecording();

// Match LinkHub's snapshot discriminators; event messages must not be collapsed.
function snapshotKey(record: MessageRecord): string | null {
  const keyedField = {
    NAMED_VALUE_FLOAT: "name", NAMED_VALUE_INT: "name", DEBUG_VECT: "name",
    DEBUG_FLOAT_ARRAY: "array_id", PID_TUNING: "axis", BATTERY_STATUS: "id",
  }[record.message];
  if (record.message === "STATUSTEXT") return null;
  return JSON.stringify([
    record.direction, record.system_id, record.component_id, record.message,
    keyedField ? record.fields[keyedField] : null,
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

/** Browser-local HTTP boundary serving recorded data, never vehicle operations. */
export class DemoBackend {
  private readonly parameters = new Map<string, ParameterResult>(
    pumpingCycle.parameters.map((parameter) => [parameter.name, parameter]),
  );
  private readonly rates = new Map<string, number | null>(Object.entries(TELEMETRY_PROFILES.wifi.rates));
  private readonly lastSample = new Map<string, number>();
  private readonly started: number;
  private readonly journal: MessageRecord[] = [];
  private sequence = 0;
  private loop = 0;
  private frame = 0;
  private state = pumpingCycle.state;

  constructor(private readonly clock: () => number = () => performance.now()) {
    this.started = clock();
    this.parameters.set("RAWES_TEL_HZ", {
      name: "RAWES_TEL_HZ", value: 20, type: mavEnum(MavParamType.REAL32),
    });
    this.advance();
  }

  get playback(): { time: number; phase: string } {
    const time = this.now() % pumpingCycle.duration;
    let phase = "";
    for (const candidate of recordingData.phases) {
      if (candidate.time <= time) phase = candidate.name;
    }
    return { time, phase };
  }

  private now(): number {
    return Math.max(0, (this.clock() - this.started) / 1000);
  }

  private get cursor(): string {
    return `v1:${this.sequence}`;
  }

  private advance(): void {
    const now = this.now();
    // Bound catch-up after a suspended tab without changing playback's time base.
    const earliestLoop = Math.max(0, Math.floor(now / pumpingCycle.duration) - 1);
    if (this.loop < earliestLoop) {
      this.loop = earliestLoop;
      this.frame = 0;
    }
    for (;;) {
      const frame = pumpingCycle.records[this.frame];
      if (!frame) {
        this.loop += 1;
        this.frame = 0;
        continue;
      }
      const time = this.loop * pumpingCycle.duration + frame.time;
      if (time > now) break;
      this.frame += 1;
      const fields = { ...frame.fields };
      // Each loop gets a monotonic boot clock; all other recorded values are intact.
      if ("time_boot_ms" in fields) fields.time_boot_ms = Math.round(time * 1000);
      if ("time_usec" in fields) fields.time_usec = Math.round(time * 1e6);
      if (frame.message === "HEARTBEAT") {
        const state = heartbeatState(fields);
        if (!state) throw new Error("Invalid replay heartbeat");
        this.state = state;
      }
      const key = snapshotKey({
        message: frame.message, fields, direction: "rx", system_id: 1, component_id: 1,
        cursor: "", received_time: "", received_time_ns: 0,
      }) ?? frame.message;
      const rate = this.rates.get(frame.message);
      // Rates only thin local delivery, never upsample or alter the source.
      if (frame.time !== 0 && rate !== null && rate !== undefined
        && (rate === 0 || time - (this.lastSample.get(key) ?? -Infinity) < 1 / rate - 1e-6)) continue;
      this.lastSample.set(key, time);
      this.sequence += 1;
      this.journal.push({
        message: frame.message, fields, direction: "rx", system_id: 1, component_id: 1,
        cursor: this.cursor, received_time: new Date(time * 1000).toISOString(),
        received_time_ns: Math.round(time * 1e9),
        sim_clock: { epoch: 1, time_boot_ms: Math.round(time * 1000), quality: "recorded-replay" },
      });
    }
    if (this.journal.length > MAX_JOURNAL_RECORDS) {
      this.journal.splice(0, this.journal.length - MAX_JOURNAL_RECORDS);
    }
  }

  private status(): LinkHubStatus {
    const rx: Record<string, number> = {};
    const cutoff = this.now() - 3;
    for (let index = this.journal.length - 1; index >= 0; index -= 1) {
      const record = this.journal[index]!;
      if (record.received_time_ns / 1e9 < cutoff) break;
      rx[record.message] = (rx[record.message] ?? 0) + 64 / 3;
    }
    return {
      ...this.state, connected: true, ready: true, connection: "recorded-sitl-replay",
      port: null, baud: null, clock_epoch: 1, target_system: 1, target_component: 1,
      latest_time_boot_ms: Math.round(this.now() * 1000),
      received_messages: this.sequence, transmitted_messages: 0,
      received_bytes: this.sequence * 64, transmitted_bytes: 0,
      rx_bps: Object.values(rx).reduce((sum, value) => sum + value, 0), tx_bps: 0,
      rx_bps_by_message: rx, tx_bps_by_message: {},
      cursor: this.cursor, generation: GENERATION,
    };
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
      if (method === "GET" && path === "/v1/motor") return json({ available: false });
      if (path.startsWith("/v1/motor")) return json({ message: READ_ONLY }, 403);
      if (method === "GET" && path === "/v1/status") {
        return json({ service: "LinkHub recorded SITL replay", api_version: 1, run_id: GENERATION, cursor: this.cursor });
      }
      if (method === "GET" && path === "/v1/mavlink/status") return json(this.status());
      if (method === "POST" && path === "/v1/journal/flush") return json({ cursor: this.cursor });
      if (method === "GET" && path === "/v1/mavlink/messages") {
        const expected = url.searchParams.get("expected_generation");
        if (expected && expected !== GENERATION) {
          return json({ error: "generation_changed", message: "Recording generation changed", expected_generation: expected, current_generation: GENERATION, cursor: this.cursor }, 409);
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
        const cutoff = lag === null ? -Infinity : this.now() * 1000 - Number(lag);
        // Sequences are contiguous within the retained journal, including after trimming.
        const start = Math.max(0, after - (this.sequence - this.journal.length));
        let records = this.journal.slice(start).filter((record) =>
          (!direction || record.direction === direction)
          && (!names || names.includes(record.message))
          && record.received_time_ns / 1e6 >= cutoff);
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
      if (path === "/v1/mavlink/parameters" && method === "GET") {
        return json({ parameters: [...this.parameters.values()] });
      }
      if (path.startsWith("/v1/mavlink/parameters/")) {
        const name = decodeURIComponent(path.slice("/v1/mavlink/parameters/".length));
        const parameter = this.parameters.get(name);
        if (!parameter) return json({ message: `Parameter not present in recording: ${name}` }, 404);
        if (method === "GET") return json(parameter);
        // TelemetryStore uses this parameter to configure display diagnostics.
        if (method === "PUT" && name === "RAWES_TEL_HZ") {
          const rate = finite(body.value);
          if (rate < 0 || rate > 300) throw new DemoRequestError("Display rate must be within 0..300 Hz");
          this.rates.set("DEBUG_FLOAT_ARRAY", rate);
          const display = { ...parameter, value: rate };
          this.parameters.set(name, display);
          return json(display);
        }
      }
      if (method === "PUT" && path === "/v1/mavlink/message-rates") {
        for (const value of Object.values(body)) {
          if (value !== null && (finite(value) < 0 || finite(value) > 300)) {
            throw new DemoRequestError("Display rates must be within 0..300 Hz");
          }
        }
        for (const [name, rate] of Object.entries(body)) this.rates.set(name, rate === null ? null : finite(rate));
        return json(Object.fromEntries(this.rates));
      }
      if (method !== "GET" && path.startsWith("/v1/mavlink/")) return json({ message: READ_ONLY }, 403);
      return json({ message: `Unsupported replay endpoint: ${method} ${path}` }, 404);
    } catch (error) {
      if (signal?.aborted) throw error;
      if (!(error instanceof DemoRequestError) && !(error instanceof SyntaxError)) throw error;
      return json({ message: error.message }, 400);
    }
  };
}
