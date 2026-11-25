import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { CsvPanel } from "../src/csv-panel";
import { CSV_INPUTS, CsvModel, csvSample, csvSeries, type CsvInput, type CsvSample } from "../src/csv-model";
import type { CsvReading, CsvStatus, DiagnosticBatch, DiagnosticRecord } from "../src/types";

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

const empty: CsvStatus = { available: true, columns: [], rows_received: 0, latest: null, recent: [] };
const reading: CsvReading = {
  source: "bench.temperature", filename: "bench.csv", received_time_ns: 1_000_000_000,
  sample_time_ns: 1_000_000_000, time_quality: "estimated", clock_epoch: 0,
  sequence: 8, source_sequence: 42, values: { input: "B2", raw: 40, gain: 32 },
};
const populated: CsvStatus = {
  available: true, columns: ["input", "raw", "gain"], rows_received: 10,
  latest: reading, recent: [
    { ...reading, values: { input: "A1", raw: -12.5, gain: 64 } },
    { ...reading, values: { input: "A2", raw: -20, gain: 32 } },
    { ...reading, values: { input: "B1", raw: 30, gain: 64 } },
    reading,
  ],
};

function setup(initial: CsvStatus = empty, canvasWidth = 800) {
  const state = new Element();
  const rows = new Element();
  const timing = new Element();
  const context = {
    strokeStyle: "", lineWidth: 1,
    setTransform: vi.fn(), clearRect: vi.fn(), fillText: vi.fn(), setLineDash: vi.fn(),
    beginPath: vi.fn(), moveTo: vi.fn(), lineTo: vi.fn(), stroke: vi.fn(), fillRect: vi.fn(),
    measureText: vi.fn((text: string) => ({ width: text.length * 10 })),
  };
  const canvas = Object.assign(new Element(), { clientWidth: canvasWidth, getContext: () => context });
  const elements = new Map([
    ['[data-csv="state"]', state], ['[data-csv="rows"]', rows],
    ['[data-csv="chart"]', canvas], ['[data-csv="timing"]', timing],
  ]);
  const root = Object.assign(new Element(), {
    querySelector: (selector: string) => elements.get(selector),
  });
  const button = new Element();
  vi.stubGlobal("document", {
    createElement: (tag: string) => tag === "section" ? root : tag === "button" ? button : new Element(),
  });
  vi.stubGlobal("window", { devicePixelRatio: 2 });
  vi.stubGlobal("ResizeObserver", class {
    observe = vi.fn();
    disconnect = vi.fn();
  });
  const api = {
    csvStatus: vi.fn(async (_signal?: AbortSignal) => initial),
    serviceStatus: vi.fn(async (_signal?: AbortSignal) => ({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:0",
    })),
    csvSamples: vi.fn(async (_after: string, _source: string, _limit?: number, _signal?: AbortSignal): Promise<DiagnosticBatch> =>
      ({ records: [], next_cursor: "v1:0" })),
  };
  const onError = vi.fn();
  const panel = new CsvPanel(
    document.createElement("button"), document.createElement("section"), api, onError,
  );
  return { panel, api, onError, root, button, state, rows, timing, context };
}

beforeEach(() => {
  vi.useFakeTimers();
  vi.setSystemTime(0);
});
afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe("CSV pane", () => {
  it("plots only the last five minutes and ages out samples without new uploads", async () => {
    vi.setSystemTime(600_000);
    const { panel, api, timing, context, state } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:3",
    });
    api.csvSamples.mockResolvedValueOnce({
      records: [event(1, 100), event(2, 299.5), event(3, 300)], next_cursor: "v1:3",
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(timing.textContent).toContain("1 retained samples in the last 5 minutes");
    expect(context.fillRect).toHaveBeenCalledTimes(2);
    await vi.advanceTimersByTimeAsync(1000);
    expect(timing.textContent).toBe("No CSV samples in the last 5 minutes");
    expect(api.csvSamples).toHaveBeenCalledTimes(1);
    expect(state.textContent).toContain("Rows received: 10");
    panel.dispose();
  });

  it("appears on received data and combines alternating rows into four latest input values", async () => {
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
      .toEqual(["Field", "Latest value"]);
    expect(table.children[1]!.children.map((row) => row.children.map((cell) => cell.textContent)))
      .toEqual([["A1", "-12.5"], ["A2", "-20"], ["B1", "30"], ["B2", "40"]]);
    expect(table.children[1]!.children[0]!.children[1]!.innerHTML).toBe("");
    expect(state.textContent).toContain("Rows received: 10");
    expect(state.textContent).toContain("last received");
    button.dispatchEvent(new Event("click"));
    await vi.advanceTimersByTimeAsync(1000);
    expect(root.hidden).toBe(true);
    expect(button.attributes.get("aria-pressed")).toBe("false");
    panel.dispose();
  });

  it("updates latest values and total count rather than showing the recent-row history", async () => {
    const { panel, api, rows, state } = setup(populated);
    await vi.advanceTimersByTimeAsync(0);
    api.csvStatus.mockResolvedValue({
      ...populated, rows_received: 300,
      latest: { ...reading, values: { input: "B2", raw: 12, gain: 32 } },
    });
    await vi.advanceTimersByTimeAsync(1000);
    const fields = rows.children[0]!.children[1]!.children;
    expect(fields.map((row) => row.children.map((cell) => cell.textContent)))
      .toEqual([["A1", "-12.5"], ["A2", "-20"], ["B1", "30"], ["B2", "12"]]);
    expect(state.textContent).toContain("Rows received: 300");
    panel.dispose();
  });

  it("retains old rows with an explicit error on fetch failure and recovers", async () => {
    const { panel, rows, state, api, onError } = setup(populated);
    await vi.advanceTimersByTimeAsync(0);
    const table = rows.children[0];
    api.csvStatus.mockRejectedValue(new Error("network down"));
    await vi.advanceTimersByTimeAsync(2000);
    expect(state.textContent).toBe("CSV data unavailable: network down");
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

  it("consumes every journal page including empty advancing pages and redraws live before later pages finish", async () => {
    const { panel, api, timing, context } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:12",
    });
    let finish!: (batch: DiagnosticBatch) => void;
    api.csvSamples
      .mockResolvedValueOnce({ records: [event(1, 0), event(2, 0.1)], next_cursor: "v1:2" })
      .mockResolvedValueOnce({ records: [], next_cursor: "v1:7" })
      .mockImplementationOnce(() => new Promise((resolve) => { finish = resolve; }));
    await vi.advanceTimersByTimeAsync(0);
    expect(timing.textContent).toContain("2 retained samples");
    expect(context.lineTo).toHaveBeenCalled();
    expect(api.csvSamples.mock.calls.map(([after]) => after)).toEqual(["v1:0", "v1:2", "v1:7"]);
    finish({ records: Array.from({ length: 5 }, (_, i) => event(i + 8, i / 10 + 1)), next_cursor: "v1:12" });
    await vi.advanceTimersByTimeAsync(0);
    expect(timing.textContent).toContain("7 retained samples");
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.csvSamples).toHaveBeenCalledTimes(3);
    expect(api.csvSamples).toHaveBeenCalledWith("v1:0", reading.source, 1000, expect.any(AbortSignal));
    panel.dispose();
  });

  it("bounds reads to a snapshot and retains new arrivals for the next poll", async () => {
    const { panel, api, timing } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:2",
    });
    api.csvSamples.mockResolvedValueOnce({
      records: [event(1, 0), event(2, 0.1), event(3, 0.2)], next_cursor: "v1:3",
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(timing.textContent).toContain("2 retained samples");
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:3",
    });
    api.csvSamples.mockResolvedValueOnce({ records: [event(3, 0.2)], next_cursor: "v1:3" });
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.csvSamples.mock.lastCall?.[0]).toBe("v1:2");
    expect(timing.textContent).toContain("3 retained samples");
    panel.dispose();
  });

  it("bounds empty advancing backlog work and resumes the cursor next poll", async () => {
    const { panel, api } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:40",
    });
    api.csvSamples.mockImplementation(async (after) => ({
      records: [], next_cursor: `v1:${Number(after.slice(3)) + 1}`,
    }));
    await vi.advanceTimersByTimeAsync(0);
    expect(api.csvSamples).toHaveBeenCalledTimes(32);
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.csvSamples).toHaveBeenCalledTimes(40);
    expect(api.csvSamples.mock.calls[32]?.[0]).toBe("v1:32");
    panel.dispose();
  });

  it("reports unavailable legacy timing, preserves graph on journal errors, and resets on restart", async () => {
    const { panel, api, timing, state, context } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:2",
    });
    api.csvSamples.mockResolvedValueOnce({
      records: [event(1, 0), { ...event(2, 1), data: {
        source: reading.source, event: "csv.sample",
        fields: { filename: "bench.csv", values: { input: "A1", raw: 999 }, device_time: "12:00:00" },
      } }], next_cursor: "v1:2",
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(timing.textContent).toContain("Timing unavailable for 1 samples");
    const draws = context.clearRect.mock.calls.length;
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:3",
    });
    api.csvSamples.mockRejectedValueOnce(new Error("journal down"));
    await vi.advanceTimersByTimeAsync(1000);
    expect(state.textContent).toContain("CSV data unavailable: journal down");
    expect(context.clearRect).toHaveBeenCalledTimes(draws);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-2", cursor: "v1:1",
    });
    api.csvSamples.mockResolvedValueOnce({ records: [event(1, 2)], next_cursor: "v1:1" });
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.csvSamples.mock.lastCall?.[0]).toBe("v1:0");
    expect(timing.textContent).toContain("1 retained samples");
    expect(timing.textContent).not.toContain("Timing unavailable");
    panel.dispose();
  });

  it("plots four raw input traces and means while retaining filename, receive time, and row count", async () => {
    const { panel, api, context, rows, state } = setup({
      ...populated, columns: ["rownum", "device_time", "input", "raw", "gain", "temperature"],
      latest: { ...reading, values: { input: "B2", raw: 4, gain: 32, temperature: 999999 } },
    });
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:4",
    });
    api.csvSamples.mockResolvedValueOnce({
      records: CSV_INPUTS.map((input, index) =>
        event(index + 1, 1, { values: { input, raw: index + 1, temperature: 999999 } })),
      next_cursor: "v1:4",
    });
    await vi.advanceTimersByTimeAsync(0);
    const labels = context.fillText.mock.calls.map(([label]) => label);
    expect(labels).toEqual(expect.arrayContaining([
      ...CSV_INPUTS.flatMap((input) => [`${input} raw`, `${input} 1 s mean`]),
      "Estimated acquisition elapsed (s) · raw ADC counts",
    ]));
    expect(labels).not.toContain("temperature");
    expect(rows.children[0]!.children[1]!.children.map((row) => row.children[0]!.textContent))
      .toEqual(["A1", "A2", "B1", "B2"]);
    expect(state.textContent).toContain("bench.csv");
    expect(state.textContent).toContain("last received");
    expect(state.textContent).toContain("Rows received: 10");
    panel.dispose();
  });

  it.each([320, 850])("lays out measured legend labels without overlap at %i px", async (width) => {
    const { panel, api, context } = setup(populated, width);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:1",
    });
    api.csvSamples.mockResolvedValueOnce({ records: [event(1, 0)], next_cursor: "v1:1" });
    await vi.advanceTimersByTimeAsync(0);
    const names = CSV_INPUTS.flatMap((input) => [`${input} raw`, `${input} 1 s mean`]);
    const labels = context.fillText.mock.calls.filter(([text]) => names.includes(text)).slice(-8);
    expect(labels).toHaveLength(8);
    for (const [index, [text, x, y]] of labels.entries()) {
      const labelWidth = context.measureText(text).width;
      expect(x).toBeGreaterThanOrEqual(77);
      expect(x + labelWidth).toBeLessThanOrEqual(width - 10);
      const previous = labels[index - 1];
      if (previous && previous[2] === y) {
        expect(x).toBeGreaterThanOrEqual(previous[1] + context.measureText(previous[0]).width + 18);
      }
    }
    if (width === 320) expect(labels[7]![2]).toBeGreaterThan(labels[0]![2]);
    panel.dispose();
  });

  it("connects each raw input across rows acquired for the other three inputs", async () => {
    const { panel, api, context } = setup(populated);
    let segments = 0;
    const rawStrokes: number[] = [];
    context.beginPath.mockImplementation(() => { segments = 0; });
    context.lineTo.mockImplementation(() => { segments++; });
    context.stroke.mockImplementation(() => {
      if (context.lineWidth === 1 && ["#78b7ff", "#f0c36b", "#71ddb4", "#c792ea"].includes(context.strokeStyle)) {
        rawStrokes.push(segments);
      }
    });
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:8",
    });
    api.csvSamples.mockResolvedValueOnce({
      records: Array.from({ length: 8 }, (_, index) =>
        event(index + 1, index / 10, { values: { input: CSV_INPUTS[index % 4], raw: index } })),
      next_cursor: "v1:8",
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(rawStrokes.slice(-4)).toEqual([1, 1, 1, 1]);
    panel.dispose();
  });

  it("omits missing sensors and all unrelated mapped fields from the latest table", async () => {
    const { panel, rows } = setup({
      ...populated, columns: ["device_time", "input", "raw", "gain"],
      latest: { ...reading, values: { input: "A1", raw: 3, gain: 64 } },
      recent: [
        { ...reading, filename: "old.csv", values: { input: "A2", raw: 99 } },
        { ...reading, values: { input: "B1", raw: "<img src=x>" } },
        { ...reading, values: { input: "temperature", raw: 1000 } },
      ],
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(rows.children[0]!.children[1]!.children.map((row) => row.children.map((cell) => cell.textContent)))
      .toEqual([["A1", "3"]]);
    panel.dispose();
  });

  it("deduplicates journal errors and resumes without dropping previously plotted batches", async () => {
    const { panel, api, timing, onError } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:2",
    });
    api.csvSamples
      .mockResolvedValueOnce({ records: [event(1, 0)], next_cursor: "v1:1" })
      .mockRejectedValue(new Error("journal down"));
    await vi.advanceTimersByTimeAsync(1000);
    expect(onError).toHaveBeenCalledTimes(1);
    expect(timing.textContent).toContain("1 retained samples");
    api.csvSamples.mockResolvedValueOnce({ records: [event(2, 0.5)], next_cursor: "v1:2" });
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.csvSamples.mock.lastCall?.[0]).toBe("v1:1");
    expect(timing.textContent).toContain("2 retained samples");
    panel.dispose();
  });

  it("does not loop on a stalled cursor and cancels pending journal reads on disposal", async () => {
    const { panel, api, state, timing } = setup(populated);
    api.serviceStatus.mockResolvedValue({
      service: "linkhub", api_version: 1, run_id: "run-1", cursor: "v1:1",
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(api.csvSamples).toHaveBeenCalledTimes(1);
    expect(state.textContent).toContain("Journal cursor did not advance");
    let finish!: (batch: DiagnosticBatch) => void;
    api.csvSamples.mockImplementationOnce(() => new Promise((resolve) => { finish = resolve; }));
    await vi.advanceTimersByTimeAsync(1000);
    const signal = api.csvSamples.mock.lastCall?.[3];
    panel.dispose();
    expect(signal?.aborted).toBe(true);
    finish({ records: [event(1, 0)], next_cursor: "v1:1" });
    await vi.advanceTimersByTimeAsync(1000);
    expect(timing.textContent).not.toContain("1 retained samples");
    expect(api.csvSamples).toHaveBeenCalledTimes(2);
  });
});

function event(sequence: number, time: number, overrides: Record<string, unknown> = {}): DiagnosticRecord {
  return {
    cursor: `v1:${sequence}`, ingest_time_ns: time * 1e9, data: {
      source: reading.source, event: "csv.sample", fields: {
        filename: "bench.csv", sample_time_ns: time * 1e9, clock_epoch: 0,
        time_quality: "estimated", values: { input: "A1", raw: sequence, gain: 64 },
        ...overrides,
      },
    },
  };
}

function sample(time: number, raw: number | null, input: CsvInput = "A1"): CsvSample {
  return {
    filename: "bench.csv", epoch: 0, time, reset: false,
    A1: null, A2: null, B1: null, B2: null, [input]: raw,
  };
}

describe("CSV one-second averaging", () => {
  it("demultiplexes labeled signed raw counts without copying values into other inputs", () => {
    const model = new CsvModel();
    const raw = [-8388608, 0, 8388607, -42];
    model.append(CSV_INPUTS.map((input, index) =>
      event(index + 1, index / 10, { values: { input, raw: raw[index], gain: index % 2 ? 32 : 64 } })));
    expect(model.samples.map((sample) => CSV_INPUTS.map((input) => sample[input]))).toEqual([
      [-8388608, null, null, null], [null, 0, null, null],
      [null, null, 8388607, null], [null, null, null, -42],
    ]);
    expect(csvSeries(model.samples)[0]!.at(-1)!.means)
      .toEqual({ A1: -8388608, A2: 0, B1: 8388607, B2: -42 });
  });

  it("uses irregular acquisition spacing, inclusive window edges, and independent alternating inputs", () => {
    const points = csvSeries([
      sample(0, 2), sample(0.2, 12, "B1"), sample(0.2, 4),
      sample(1, 9), sample(1, 18, "B1"), sample(1.01, 11),
    ])[0]!;
    expect(points.map((point) => point.means.A1)).toEqual([2, 3, 3, 5, 5, 8]);
    expect(points.map((point) => point.means.B1)).toEqual([null, 12, 12, 15, 15, 15]);
    expect(points.map((point) => point.means.A2)).toEqual([null, null, null, null, null, null]);
    expect(points.map((point) => point.means.B2)).toEqual([null, null, null, null, null, null]);
  });

  it("sorts out-of-order rows in one epoch and includes equal timestamps", () => {
    const points = csvSeries([sample(0.5, 8), sample(0, 2), sample(0.5, 4)])[0]!;
    expect(points.map((point) => [point.time, point.means.A1])).toEqual([[0, 2], [0.5, 14 / 3], [0.5, 14 / 3]]);
  });

  it("neither connects nor averages across filenames, clock resets or unavailable timing", () => {
    const groups = csvSeries([
      sample(0, 100), { ...sample(0.1, 2), epoch: 1, reset: true },
      sample(0.2, 3), { ...sample(0.3, 4), filename: "other.csv" },
      { ...sample(0.4, 99), time: null }, { ...sample(0.5, 5), filename: "other.csv" },
    ]);
    expect(groups).toHaveLength(5);
    expect(groups.map((group) => group[0]!.means.A1)).toEqual([100, 2, 3, 4, 5]);
  });

  it("requires explicit valid acquisition metadata instead of receipt or raw device time", () => {
    expect(csvSample(event(1, 0, { sample_time_ns: null }))?.time).toBeNull();
    expect(csvSample(event(1, 0, { time_quality: "unavailable" }))?.time).toBeNull();
    expect(csvSample(event(1, 0, { clock_epoch: null }))?.time).toBeNull();
    expect(csvSample(event(1, 0, { values: { input: "A1", raw: NaN } })))
      .toMatchObject({ A1: null, A2: null, B1: null, B2: null });
    expect(csvSample(event(1, 0, { values: { input: "B2", raw: "10" } }))?.B2).toBeNull();
    expect(csvSample(event(1, 0, { values: { input: "A", raw: 10 } })))
      .toMatchObject({ A1: null, A2: null, B1: null, B2: null });
  });

  it("retains at most 10,000 samples without relying on the five-row status preview", () => {
    const model = new CsvModel();
    model.append(Array.from({ length: 10_005 }, (_, i) => event(i + 1, i / 10)));
    expect(model.samples).toHaveLength(10_000);
    expect(model.samples[0]?.A1).toBe(6);
    model.clear();
    expect(model.samples).toEqual([]);
  });
});
