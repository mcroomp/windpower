import type { LinkHubApi } from "./api";
import type { CsvStatus } from "./types";
import { CSV_INPUTS, CsvModel, csvValues } from "./csv-model";
import { drawCsvChart } from "./csv-chart";

const WINDOW_SECONDS = 5 * 60;

function cursorSequence(cursor: string): bigint {
  if (!/^v1:\d+$/.test(cursor)) throw new Error(`Invalid journal cursor: ${cursor}`);
  return BigInt(cursor.slice(3));
}

export class CsvPanel {
  private readonly controller = new AbortController();
  private readonly state: HTMLElement;
  private readonly rows: HTMLElement;
  private readonly canvas: HTMLCanvasElement;
  private readonly timing: HTMLElement;
  private readonly observer: ResizeObserver;
  private readonly model = new CsvModel();
  private cursor = "v1:0";
  private runId: string | undefined;
  private source: string | undefined;
  private totalRows = 0;
  private timer: ReturnType<typeof setTimeout> | undefined;
  private received = false;
  private disposed = false;
  private lastError: string | null = null;

  constructor(
    private readonly button: HTMLButtonElement,
    private readonly root: HTMLElement,
    private readonly api: Pick<LinkHubApi, "csvStatus" | "csvSamples" | "serviceStatus">,
    private readonly onError: (message: string) => void,
  ) {
    root.innerHTML = `
      <div class="csv-head"><h2>CSVs</h2><span data-csv="state" role="status"></span></div>
      <canvas class="csv-chart" data-csv="chart" aria-label="Last five minutes of raw A1, A2, B1 and B2 ADC counts with trailing one-second averages"></canvas>
      <div data-csv="timing" role="status"></div>
      <div class="csv-rows" data-csv="rows"></div>
    `;
    this.state = this.element('[data-csv="state"]');
    this.rows = this.element('[data-csv="rows"]');
    const canvas = root.querySelector<HTMLCanvasElement>('[data-csv="chart"]');
    if (!canvas) throw new Error("Missing CSV chart");
    this.canvas = canvas;
    this.timing = this.element('[data-csv="timing"]');
    this.observer = new ResizeObserver(() => this.draw());
    this.observer.observe(this.canvas);
    button.hidden = true;
    root.hidden = true;
    button.addEventListener("click", this.toggle);
    void this.refresh();
  }

  dispose(): void {
    this.disposed = true;
    clearTimeout(this.timer);
    this.controller.abort();
    this.observer.disconnect();
    this.button.removeEventListener("click", this.toggle);
  }

  private readonly toggle = (): void => {
    this.root.hidden = !this.root.hidden;
    this.button.setAttribute("aria-pressed", String(!this.root.hidden));
    if (!this.root.hidden) this.draw();
  };

  private async refresh(): Promise<void> {
    try {
      if (this.model.samples.some((sample) => (sample.time ?? sample.receivedTime ?? -Infinity) < this.cutoff())) {
        this.draw();
      }
      const status = await this.api.csvStatus(this.controller.signal);
      if (this.disposed) return;
      this.render(status);
      if (status.available && status.latest) {
        const service = await this.api.serviceStatus(
          AbortSignal.any([this.controller.signal, AbortSignal.timeout(3_000)]),
        );
        if (this.disposed) return;
        const tail = cursorSequence(service.cursor);
        if (this.runId !== service.run_id || this.source !== status.latest.source
          || status.rows_received < this.totalRows || cursorSequence(this.cursor) > tail) {
          this.model.clear();
          this.cursor = "v1:0";
          this.draw();
        }
        this.runId = service.run_id;
        this.source = status.latest.source;
        this.totalRows = status.rows_received;
        // Bound each poll to a fixed journal snapshot and a finite number of pages.
        for (let page = 0; page < 32 && cursorSequence(this.cursor) < tail; page++) {
          const batch = await this.api.csvSamples(this.cursor, this.source, 1_000, this.controller.signal);
          if (this.disposed) return;
          const next = cursorSequence(batch.next_cursor);
          if (next <= cursorSequence(this.cursor)) throw new Error("Journal cursor did not advance");
          this.model.append(batch.records.filter((record) => cursorSequence(record.cursor) <= tail), this.cutoff());
          this.cursor = next > tail ? service.cursor : batch.next_cursor;
          this.draw();
        }
      }
      this.lastError = null;
    } catch (error) {
      if (this.disposed) return;
      const message = `CSV data unavailable: ${error instanceof Error ? error.message : String(error)}`;
      this.state.textContent = message;
      this.state.classList.toggle("error", true);
      if (message !== this.lastError) this.onError(message);
      this.lastError = message;
    } finally {
      if (!this.disposed) this.timer = setTimeout(() => void this.refresh(), 1_000);
    }
  }

  private render(status: CsvStatus): void {
    if (!status.available || !status.latest) {
      this.button.hidden = true;
      this.root.hidden = true;
      this.button.setAttribute("aria-pressed", "false");
      this.rows.replaceChildren();
      this.model.clear();
      this.cursor = "v1:0";
      this.source = undefined;
      this.draw();
      this.received = false;
      return;
    }
    this.button.hidden = false;
    if (!this.received) {
      this.root.hidden = false;
      this.button.setAttribute("aria-pressed", "true");
      this.received = true;
    }
    const time = (ns: number) => new Date(ns / 1e6).toLocaleTimeString();
    this.state.classList.toggle("error", false);
    this.state.textContent = `${status.latest.source} | ${status.latest.filename} | Rows received: ${status.rows_received} | last received ${time(status.latest.received_time_ns)}`;
    const table = document.createElement("table");
    const head = document.createElement("thead");
    const heading = document.createElement("tr");
    for (const name of ["Field", "Latest value"]) {
      const cell = document.createElement("th");
      cell.textContent = name;
      heading.append(cell);
    }
    head.append(heading);
    const body = document.createElement("tbody");
    const latest = csvValues(null);
    for (const reading of [...status.recent, status.latest]) {
      if (reading.source !== status.latest.source || reading.filename !== status.latest.filename) continue;
      const values = csvValues(reading.values);
      for (const input of CSV_INPUTS) {
        if (values[input] !== null) latest[input] = values[input];
      }
    }
    const fields = CSV_INPUTS.filter((field) => latest[field] !== null);
    for (const field of fields) {
      const tr = document.createElement("tr");
      for (const value of [field, String(latest[field])]) {
        const cell = document.createElement("td");
        cell.textContent = value;
        tr.append(cell);
      }
      body.append(tr);
    }
    table.append(head, body);
    this.rows.replaceChildren(table);
  }

  private draw(): void {
    if (this.disposed) return;
    const cutoff = this.cutoff();
    this.model.prune(cutoff);
    const visible = this.model.samples.filter((sample) => (sample.time ?? sample.receivedTime ?? -Infinity) >= cutoff);
    const unavailable = visible.filter((sample) => sample.time === null).length;
    this.timing.textContent = !visible.length
      ? "No CSV samples in the last 5 minutes"
      : `${visible.length} retained samples in the last 5 minutes | Estimated acquisition timing; separate curves per file/clock epoch`
        + (unavailable ? ` | Timing unavailable for ${unavailable} samples (not plotted or averaged)` : "");
    drawCsvChart(this.canvas, this.model, cutoff);
  }

  private cutoff(): number { return Date.now() / 1_000 - WINDOW_SECONDS; }

  private element(selector: string): HTMLElement {
    const element = this.root.querySelector<HTMLElement>(selector);
    if (!element) throw new Error(`Missing CSV panel element ${selector}`);
    return element;
  }
}
