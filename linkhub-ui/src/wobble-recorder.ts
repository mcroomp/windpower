import type { LinkHubApi } from "./api";
import { sleep } from "./sleep";
import { WOBBLE_MESSAGES, type WobbleModel } from "./wobble-model";

const POLL_PAUSE_MS = 50;
const RETRY_MS = 500;

export class WobbleRecorder {
  private controller: AbortController | null = null;

  constructor(
    private readonly api: LinkHubApi,
    private readonly model: WobbleModel,
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
          messages: WOBBLE_MESSAGES,
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
