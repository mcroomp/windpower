import type { LinkHubApi } from "./api";
import { sleep } from "./sleep";
import { YAW_MESSAGES, type YawModel } from "./yaw-model";

const POLL_PAUSE_MS = 50;
const RETRY_MS = 500;

/**
 * Reads every yaw-related frame from the LinkHub journal into a YawModel.
 * TelemetryStore reads collapsed (latest frame per message), which would drop
 * almost every sample of a 300 Hz stream, so the chart has its own reader. It
 * only runs while the panel is open and starts at the live end of the journal.
 */
export class YawRecorder {
  private controller: AbortController | null = null;

  constructor(
    private readonly api: LinkHubApi,
    private readonly model: YawModel,
    private readonly onError: (error: Error) => void = () => {},
  ) {}

  get running(): boolean {
    return this.controller !== null;
  }

  start(): void {
    if (this.controller) {
      return;
    }
    this.controller = new AbortController();
    void this.run(this.controller.signal);
  }

  stop(): void {
    this.controller?.abort();
    this.controller = null;
  }

  private async run(signal: AbortSignal): Promise<void> {
    let cursor: string | null = null;
    let failing = false;
    while (!signal.aborted) {
      try {
        cursor ??= await this.api.currentCursor(signal);
        const batch = await this.api.queryMessages({
          after: cursor,
          waitMs: 500,
          limit: 5_000,
          direction: "rx",
          messages: YAW_MESSAGES,
          collapse: false,
        }, signal);
        this.model.ingest(batch.records);
        cursor = batch.next_cursor;
        failing = false;
        await sleep(POLL_PAUSE_MS, signal);
      } catch (error) {
        if (signal.aborted) {
          return;
        }
        cursor = null;
        if (!failing) {
          failing = true;
          this.onError(error instanceof Error ? error : new Error(String(error)));
        }
        await sleep(RETRY_MS, signal);
      }
    }
  }
}
