import type { LinkHubApi } from "./api";
import { heartbeatState } from "./mav";
import { DISPLAY_TELEMETRY_RATES } from "./telemetry-rates";
import type { LinkHubStatus, MessageRecord } from "./types";

type Listener = (record: MessageRecord) => void;
type GenerationListener = (generation: string, previous: string | null) => void;
type ConnectionListener = (connected: boolean, error?: Error) => void;
type StatusListener = (status: LinkHubStatus) => void;

const MAX_TELEMETRY_LAG_MS = 1_000;

function cursorSequence(cursor: string): number {
  const value = Number(cursor.split(":", 2)[1]);
  return Number.isFinite(value) ? value : 0;
}

function stateKey(record: MessageRecord): string {
  const keyed = record.fields.name;
  return `${record.direction}:${record.message}:${typeof keyed === "string" ? keyed : ""}`;
}

export class TelemetryStore {
  private readonly latest = new Map<string, MessageRecord>();
  private readonly recent: MessageRecord[] = [];
  private readonly listeners = new Set<Listener>();
  private readonly generationListeners = new Set<GenerationListener>();
  private readonly connectionListeners = new Set<ConnectionListener>();
  private readonly statusListeners = new Set<StatusListener>();
  private controller: AbortController | null = null;
  private cursor = "v1:0";
  private currentStatus: LinkHubStatus | null = null;
  private running: Promise<void> | null = null;
  private configuredGeneration: string | null = null;

  constructor(private readonly api: LinkHubApi) {}

  get status(): LinkHubStatus | null {
    return this.currentStatus;
  }

  onStatus(listener: StatusListener): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }

  async start(): Promise<void> {
    if (this.running) {
      return;
    }
    this.controller = new AbortController();
    try {
      const status = await this.api.status(this.controller.signal);
      await this.configureDisplayTelemetry(status);
      this.currentStatus = status;
      for (const listener of this.statusListeners) {
        listener(status);
      }
      const initialTail = status.cursor;
      while (cursorSequence(this.cursor) < cursorSequence(initialTail)) {
        const batch = await this.api.readMessages(
          this.cursor,
          0,
          true,
          this.controller.signal,
          MAX_TELEMETRY_LAG_MS,
        );
        this.apply(batch.records);
        if (batch.next_cursor === this.cursor) {
          break;
        }
        this.cursor = batch.next_cursor;
      }
      this.running = this.poll(this.controller.signal);
    } catch (error) {
      this.controller.abort();
      this.controller = null;
      this.currentStatus = null;
      throw error;
    }
  }

  stop(): void {
    this.controller?.abort();
    this.controller = null;
    this.running = null;
  }

  get(message: string, direction: "rx" | "tx" = "rx", name = ""): MessageRecord | undefined {
    return this.latest.get(`${direction}:${message}:${name}`);
  }

  checkpoint(): number {
    return cursorSequence(this.cursor);
  }

  recordsAfter(sequence: number, predicate: (record: MessageRecord) => boolean): MessageRecord[] {
    return this.recent.filter(
      (record) => cursorSequence(record.cursor) > sequence && predicate(record),
    );
  }

  onRecord(listener: Listener): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  onGeneration(listener: GenerationListener): () => void {
    this.generationListeners.add(listener);
    return () => this.generationListeners.delete(listener);
  }

  onConnection(listener: ConnectionListener): () => void {
    this.connectionListeners.add(listener);
    return () => this.connectionListeners.delete(listener);
  }

  waitFor(
    predicate: (record: MessageRecord) => boolean,
    timeoutMs: number,
    signal?: AbortSignal,
  ): Promise<MessageRecord> {
    return new Promise((resolve, reject) => {
      const timeout = globalThis.setTimeout(() => {
        cleanup();
        reject(new Error(`Timed out after ${(timeoutMs / 1000).toFixed(1)} s`));
      }, timeoutMs);
      const abort = () => {
        cleanup();
        reject(signal?.reason ?? new DOMException("Aborted", "AbortError"));
      };
      const listener = (record: MessageRecord) => {
        if (predicate(record)) {
          cleanup();
          resolve(record);
        }
      };
      const cleanup = () => {
        globalThis.clearTimeout(timeout);
        this.listeners.delete(listener);
        signal?.removeEventListener("abort", abort);
      };
      this.listeners.add(listener);
      signal?.addEventListener("abort", abort, { once: true });
    });
  }

  private apply(records: MessageRecord[]): void {
    for (const record of records) {
      this.latest.set(stateKey(record), record);
      this.recent.push(record);
      if (this.recent.length > 500) {
        this.recent.splice(0, this.recent.length - 500);
      }
      for (const listener of this.listeners) {
        listener(record);
      }
      if (record.direction === "rx" && record.message === "HEARTBEAT") {
        const heartbeat = heartbeatState(record.fields);
        if (this.currentStatus && heartbeat) {
          this.currentStatus = { ...this.currentStatus, ...heartbeat };
        }
      }
    }
  }

  private async poll(signal: AbortSignal): Promise<void> {
    let lastStatusCheck = performance.now();
    let connected = true;
    while (!signal.aborted) {
      try {
        const batch = await this.api.readMessages(
          this.cursor,
          1_000,
          true,
          signal,
          MAX_TELEMETRY_LAG_MS,
        );
        if (!connected) {
          connected = true;
          for (const listener of this.connectionListeners) {
            listener(true);
          }
        }
        this.cursor = batch.next_cursor;
        this.apply(batch.records);
        if (performance.now() - lastStatusCheck >= 1_000) {
          lastStatusCheck = performance.now();
          const status = await this.api.status(signal);
          if (this.handleGenerationChange(status)) {
            connected = false;
            this.cursor = status.cursor;
          }
          await this.configureDisplayTelemetry(status);
          this.currentStatus = status;
          for (const listener of this.statusListeners) {
            listener(status);
          }
        }
      } catch (error) {
        if (signal.aborted) {
          return;
        }
        connected = false;
        const failure = error instanceof Error ? error : new Error(String(error));
        for (const listener of this.connectionListeners) {
          listener(false, failure);
        }
        await new Promise((resolve) => globalThis.setTimeout(resolve, 500));
      }
    }
  }

  private async configureDisplayTelemetry(status: LinkHubStatus): Promise<void> {
    if (!status.connected || !status.ready
      || status.generation === this.configuredGeneration) {
      return;
    }
    await this.api.setMessageRates(DISPLAY_TELEMETRY_RATES);
    this.configuredGeneration = status.generation;
  }

  private handleGenerationChange(status: LinkHubStatus): boolean {
    const previous = this.currentStatus?.generation ?? null;
    if (previous === null || status.generation === previous) {
      return false;
    }
    this.latest.clear();
    this.recent.length = 0;
    for (const listener of this.connectionListeners) {
      listener(false);
    }
    for (const listener of this.generationListeners) {
      listener(status.generation, previous);
    }
    return true;
  }
}
