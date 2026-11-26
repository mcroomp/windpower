import type { DiagnosticRecord } from "./types";

export const CSV_INPUTS = ["A1", "A2", "B1", "B2"] as const;
export type CsvInput = typeof CSV_INPUTS[number];
type CsvValues = Record<CsvInput, number | null>;

export interface CsvSample extends CsvValues {
  filename: string;
  epoch: number | null;
  time: number | null;
  receivedTime?: number;
  reset: boolean;
}

export interface CsvPoint extends CsvValues {
  time: number;
  means: CsvValues;
}

const numeric = (value: unknown): number | null =>
  typeof value === "number" && Number.isFinite(value) ? value : null;

export function csvValues(values: unknown): CsvValues {
  const result: CsvValues = { A1: null, A2: null, B1: null, B2: null };
  if (typeof values !== "object" || values === null || !("input" in values) || !("raw" in values)) {
    return result;
  }
  const input = CSV_INPUTS.find((input) => input === values.input);
  if (input) result[input] = numeric(values.raw);
  return result;
}

export function csvSample(record: DiagnosticRecord): CsvSample | null {
  const fields = record.data.fields;
  if (typeof fields.filename !== "string" || typeof fields.values !== "object"
    || fields.values === null || Array.isArray(fields.values)) return null;
  const quality = fields.time_quality;
  const timed = quality === "estimated" || quality === "clock_reset" || quality === "out_of_order";
  const ns = timed ? numeric(fields.sample_time_ns) : null;
  const epoch = timed ? numeric(fields.clock_epoch) : null;
  return {
    filename: fields.filename, epoch, time: ns === null || epoch === null ? null : ns / 1e9,
    receivedTime: record.ingest_time_ns === undefined ? undefined : record.ingest_time_ns / 1e9,
    reset: quality === "clock_reset",
    ...csvValues(fields.values),
  };
}

/** Inclusive [t - 1, t] arithmetic means, independently for each finite field. */
export function csvSeries(samples: readonly CsvSample[]): CsvPoint[][] {
  const groups: CsvSample[][] = [];
  let previous: CsvSample | undefined;
  for (const sample of samples) {
    if (sample.time === null) {
      previous = undefined;
      continue;
    }
    if (!previous || sample.reset || previous.filename !== sample.filename || previous.epoch !== sample.epoch) {
      groups.push([]);
    }
    groups.at(-1)!.push(sample);
    previous = sample;
  }
  return groups.map((group) => {
    const sorted = group.slice().sort((a, b) => a.time! - b.time!);
    const points: CsvPoint[] = [];
    let start = 0;
    let end = 0;
    const sums = { A1: 0, A2: 0, B1: 0, B2: 0 };
    const counts = { A1: 0, A2: 0, B1: 0, B2: 0 };
    for (const sample of sorted) {
      const time = sample.time!;
      // Include simultaneous acquisitions regardless of their journal arrival order.
      while (end < sorted.length && sorted[end]!.time! <= time) {
        const added = sorted[end++]!;
        for (const input of CSV_INPUTS) {
          if (added[input] !== null) { sums[input] += added[input]; counts[input]++; }
        }
      }
      while (sorted[start]!.time! < time - 1) {
        const removed = sorted[start++]!;
        for (const input of CSV_INPUTS) {
          if (removed[input] !== null) { sums[input] -= removed[input]; counts[input]--; }
        }
      }
      const means = csvValues(null);
      for (const input of CSV_INPUTS) {
        means[input] = counts[input] ? sums[input] / counts[input] : null;
      }
      points.push({
        time, A1: sample.A1, A2: sample.A2, B1: sample.B1, B2: sample.B2, means,
      });
    }
    return points;
  });
}

export class CsvModel {
  readonly samples: CsvSample[] = [];

  append(records: readonly DiagnosticRecord[], cutoff = -Infinity): void {
    for (const record of records) {
      const sample = csvSample(record);
      if (sample) this.samples.push(sample);
    }
    this.prune(cutoff);
    if (this.samples.length > 10_000) this.samples.splice(0, this.samples.length - 10_000);
  }

  prune(cutoff: number): boolean {
    const length = this.samples.length;
    let kept = 0;
    for (const sample of this.samples) {
      if ((sample.time ?? sample.receivedTime ?? -Infinity) >= cutoff - 1) this.samples[kept++] = sample;
    }
    this.samples.length = kept;
    return kept !== length;
  }

  clear(): void { this.samples.length = 0; }
}
