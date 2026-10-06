import { describe, expect, it } from "vitest";
import { compareConfig, configTargets, parseParm } from "../src/config";
import { MavParamType } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";

describe("parseParm", () => {
  it("parses names and values, skipping comments and bad lines", () => {
    const params = parseParm("# header\nA_B 1\nC 2.5 # note\n\nBAD\nD notanumber\nE,3\n");
    expect([...params]).toEqual([["A_B", 1], ["C", 2.5], ["E", 3]]);
  });
});

describe("configTargets", () => {
  it("includes RAWES overrides and excludes sensor calibration values", () => {
    const targets = configTargets(false);
    expect(targets.get("FS_CRASH_CHECK")).toBe(0);
    expect(targets.get("ACRO_TRAINER")).toBe(0);
    expect([...targets.keys()].some((name) => name.startsWith("INS_ACCOFFS_"))).toBe(false);
  });

  it("adds the ArduPilot base file with --all", () => {
    expect(configTargets(true).size).toBeGreaterThan(configTargets(false).size);
  });
});

describe("compareConfig", () => {
  it("classifies ok, diff and missing parameters", () => {
    const rows = compareConfig(
      new Map([["A", 1], ["B", 0], ["C", 5]]),
      new Map([
        ["A", { name: "A", value: 1.00001, type: mavEnum(MavParamType.REAL32) }],
        ["B", { name: "B", value: 1, type: mavEnum(MavParamType.INT8) }],
      ]),
    );
    expect(rows.map((row) => [row.name, row.status])).toEqual([
      ["A", "ok"],
      ["B", "diff"],
      ["C", "missing"],
    ]);
    expect(rows[1]?.type).toEqual({ type: "MAV_PARAM_TYPE_INT8" });
  });
});
