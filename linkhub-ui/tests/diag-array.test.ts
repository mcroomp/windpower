import { describe, expect, it } from "vitest";
import { DIAG_ARRAY_ID, DIAG_KEYS, diagValue, isDiagRecord } from "../src/diag-array";
import type { MessageRecord } from "../src/types";

function record(arrayId: number, data: (number | null)[], direction: "rx" | "tx" = "rx"): MessageRecord {
  return {
    received_time: "t", received_time_ns: 1, direction, system_id: 1, component_id: 1,
    message: "DEBUG_FLOAT_ARRAY", cursor: "v1:1",
    fields: { time_usec: 0, array_id: arrayId, name: "RAWES_DIAG", data },
  };
}

function diagData(values: Partial<Record<(typeof DIAG_KEYS)[number], number>>): number[] {
  const data = new Array<number>(58).fill(0);
  DIAG_KEYS.forEach((key, index) => {
    const value = values[key];
    if (value !== undefined) {
      data[0] = (data[0] ?? 0) + (1 << index);
      data[index + 1] = value;
    }
  });
  return data;
}

describe("diagnostic DEBUG_FLOAT_ARRAY", () => {
  it("reads only the keys the validity mask marks as set", () => {
    const rec = record(DIAG_ARRAY_ID, diagData({ YFF_U: 0.25, ANCH_D: -3 }));
    expect(diagValue(rec, "YFF_U")).toBe(0.25);
    expect(diagValue(rec, "ANCH_D")).toBe(-3);
    expect(diagValue(rec, "YFF_T")).toBeUndefined();
  });

  it("distinguishes a real zero from an unset key", () => {
    const rec = record(DIAG_ARRAY_ID, diagData({ YFF_T: 0 }));
    expect(diagValue(rec, "YFF_T")).toBe(0);
    expect(diagValue(rec, "YFF_U")).toBeUndefined();
  });

  it("ignores other arrays and missing records", () => {
    expect(diagValue(record(DIAG_ARRAY_ID + 1, diagData({ YFF_U: 1 })), "YFF_U")).toBeUndefined();
    expect(diagValue(undefined, "YFF_U")).toBeUndefined();
    expect(isDiagRecord(record(DIAG_ARRAY_ID, []))).toBe(true);
    expect(isDiagRecord(record(DIAG_ARRAY_ID, [], "tx"))).toBe(false);
    expect(isDiagRecord(record(DIAG_ARRAY_ID + 1, []))).toBe(false);
  });
});
