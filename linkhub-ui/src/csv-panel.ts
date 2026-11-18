import type { LinkHubApi } from "./api";
import type { CsvStatus } from "./types";

export class CsvPanel {
  private readonly controller = new AbortController();
  private readonly state: HTMLElement;
  private readonly rows: HTMLElement;
  private timer: ReturnType<typeof setTimeout> | undefined;
  private received = false;
  private disposed = false;
  private lastError: string | null = null;

  constructor(
    private readonly button: HTMLButtonElement,
    private readonly root: HTMLElement,
    private readonly api: Pick<LinkHubApi, "csvStatus">,
    private readonly onError: (message: string) => void,
  ) {
    root.innerHTML = `
      <div class="csv-head"><h2>CSVs</h2><span data-csv="state" role="status"></span></div>
      <div class="csv-rows" data-csv="rows"></div>
    `;
    this.state = this.element('[data-csv="state"]');
    this.rows = this.element('[data-csv="rows"]');
    button.hidden = true;
    root.hidden = true;
    button.addEventListener("click", this.toggle);
    void this.refresh();
  }

  dispose(): void {
    this.disposed = true;
    clearTimeout(this.timer);
    this.controller.abort();
    this.button.removeEventListener("click", this.toggle);
  }

  private readonly toggle = (): void => {
    this.root.hidden = !this.root.hidden;
    this.button.setAttribute("aria-pressed", String(!this.root.hidden));
  };

  private async refresh(): Promise<void> {
    try {
      const status = await this.api.csvStatus(this.controller.signal);
      if (this.disposed) return;
      this.lastError = null;
      this.render(status);
    } catch (error) {
      if (this.disposed) return;
      const message = `CSV status unavailable: ${error instanceof Error ? error.message : String(error)}`;
      this.state.textContent = message;
      this.state.classList.toggle("error", true);
      if (message !== this.lastError) this.onError(message);
      this.lastError = message;
    } finally {
      if (!this.disposed) this.timer = setTimeout(() => void this.refresh(), 1_000);
    }
  }

  private render(status: CsvStatus): void {
    if (!status.available || !status.latest || status.recent.length === 0) {
      this.button.hidden = true;
      this.root.hidden = true;
      this.button.setAttribute("aria-pressed", "false");
      this.rows.replaceChildren();
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
    this.state.textContent = `${status.latest.source} | ${status.latest.filename} | last received ${time(status.latest.received_time_ns)}`;
    const table = document.createElement("table");
    const head = document.createElement("thead");
    const heading = document.createElement("tr");
    for (const name of ["File", "Received", ...status.columns]) {
      const cell = document.createElement("th");
      cell.textContent = name;
      heading.append(cell);
    }
    head.append(heading);
    const body = document.createElement("tbody");
    for (const row of status.recent) {
      const tr = document.createElement("tr");
      for (const value of [
        row.filename, time(row.received_time_ns),
        ...status.columns.map((field) => String(row.values[field])),
      ]) {
        const cell = document.createElement("td");
        cell.textContent = value;
        tr.append(cell);
      }
      body.append(tr);
    }
    table.append(head, body);
    this.rows.replaceChildren(table);
  }

  private element(selector: string): HTMLElement {
    const element = this.root.querySelector<HTMLElement>(selector);
    if (!element) throw new Error(`Missing CSV panel element ${selector}`);
    return element;
  }
}
