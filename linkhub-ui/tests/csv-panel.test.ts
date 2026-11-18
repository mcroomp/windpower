import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { CsvPanel } from "../src/csv-panel";
import type { CsvReading, CsvStatus } from "../src/types";

class Element extends EventTarget {
  innerHTML = "";
  textContent = "";
  hidden = false;
  children: Element[] = [];
  attributes = new Map<string, string>();
  classList = { toggle: vi.fn() };
  append(...children: Element[]): void { this.children.push(...children); }
  replaceChildren(...children: Element[]): void { this.children = children; }
  setAttribute(name: string, value: string): void { this.attributes.set(name, value); }
}

const empty: CsvStatus = { available: true, columns: [], latest: null, recent: [] };
const reading: CsvReading = {
  source: "bench.temperature", filename: "bench.csv", received_time_ns: 1_000_000_000,
  sequence: 8, source_sequence: 42, values: { temperature_c: -12.5, sensor: "<img src=x>" },
};
const populated: CsvStatus = {
  available: true, columns: ["temperature_c", "sensor"], latest: reading, recent: [reading],
};

function setup(initial: CsvStatus = empty) {
  const state = new Element();
  const rows = new Element();
  const root = Object.assign(new Element(), {
    querySelector: (selector: string) => selector === '[data-csv="state"]' ? state : rows,
  });
  const button = new Element();
  vi.stubGlobal("document", {
    createElement: (tag: string) => tag === "section" ? root : tag === "button" ? button : new Element(),
  });
  const api = { csvStatus: vi.fn(async (_signal?: AbortSignal) => initial) };
  const onError = vi.fn();
  const panel = new CsvPanel(
    document.createElement("button"), document.createElement("section"), api, onError,
  );
  return { panel, api, onError, root, button, state, rows };
}

beforeEach(() => vi.useFakeTimers());
afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe("CSV pane", () => {
  it("appears on received data, preserves configured field order and renders text safely", async () => {
    const { panel, root, button, rows, state, api } = setup();
    await vi.advanceTimersByTimeAsync(0);
    expect(button.hidden).toBe(true);
    expect(root.hidden).toBe(true);
    api.csvStatus.mockResolvedValue(populated);
    await vi.advanceTimersByTimeAsync(1000);
    expect(button.hidden).toBe(false);
    expect(root.hidden).toBe(false);
    expect(button.attributes.get("aria-pressed")).toBe("true");
    const table = rows.children[0]!;
    expect(table.children[0]!.children[0]!.children.map((cell) => cell.textContent))
      .toEqual(["File", "Received", "temperature_c", "sensor"]);
    const cells = table.children[1]!.children[0]!.children;
    expect(cells[0]!.textContent).toBe("bench.csv");
    expect(cells[2]!.textContent).toBe("-12.5");
    expect(cells[3]!.textContent).toBe("<img src=x>");
    expect(cells[3]!.innerHTML).toBe("");
    expect(state.textContent).toContain("last received");
    button.dispatchEvent(new Event("click"));
    await vi.advanceTimersByTimeAsync(1000);
    expect(root.hidden).toBe(true);
    expect(button.attributes.get("aria-pressed")).toBe("false");
    panel.dispose();
  });

  it("retains old rows with an explicit error on fetch failure and recovers", async () => {
    const { panel, rows, state, api, onError } = setup(populated);
    await vi.advanceTimersByTimeAsync(0);
    const table = rows.children[0];
    api.csvStatus.mockRejectedValue(new Error("network down"));
    await vi.advanceTimersByTimeAsync(2000);
    expect(state.textContent).toBe("CSV status unavailable: network down");
    expect(rows.children[0]).toBe(table);
    expect(onError).toHaveBeenCalledTimes(1);
    api.csvStatus.mockResolvedValue(populated);
    await vi.advanceTimersByTimeAsync(1000);
    expect(state.textContent).toContain("bench.temperature");
    expect(state.classList.toggle).toHaveBeenLastCalledWith("error", false);
    api.csvStatus.mockResolvedValue({ ...empty, available: false });
    await vi.advanceTimersByTimeAsync(1000);
    expect(rows.children).toEqual([]);
    panel.dispose();
  });

  it("serializes polling and ignores a pending result after disposal", async () => {
    const { panel, api, root, button } = setup();
    await vi.advanceTimersByTimeAsync(0);
    let finish!: (value: CsvStatus) => void;
    api.csvStatus.mockImplementationOnce(() => new Promise((resolve) => { finish = resolve; }));
    await vi.advanceTimersByTimeAsync(5000);
    expect(api.csvStatus).toHaveBeenCalledTimes(2);
    const signal = api.csvStatus.mock.lastCall?.[0];
    panel.dispose();
    expect(signal?.aborted).toBe(true);
    finish(populated);
    await vi.advanceTimersByTimeAsync(5000);
    expect(api.csvStatus).toHaveBeenCalledTimes(2);
    expect(root.hidden).toBe(true);
    expect(button.hidden).toBe(true);
  });
});
