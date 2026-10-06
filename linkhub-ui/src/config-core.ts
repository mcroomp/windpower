import type { ParameterResult } from "./types";

const CALIBRATION_PARAM_PREFIXES = [
  "INS_ACCOFFS_",
  "INS_ACCSCAL_",
  "INS_ACC2OFFS_",
  "INS_ACC2SCAL_",
  "INS_ACC3OFFS_",
  "INS_ACC3SCAL_",
  "INS_GYROFFS_",
  "INS_GYR2OFFS_",
  "INS_GYR3OFFS_",
  "COMPASS_OFS",
  "COMPASS_DIA",
  "COMPASS_ODI",
  "COMPASS_MOT",
  "BARO1_GND_PRESS",
  "BARO2_GND_PRESS",
  "BARO3_GND_PRESS",
  "BARO1_GND_TEMP",
  "BARO2_GND_TEMP",
  "BARO3_GND_TEMP",
  "GND_ABS_PRESS",
  "GND_TEMP",
  "AHRS_TRIM_",
];

const TOLERANCE = 1e-4;

export interface ConfigProvider {
  sources(all: boolean): string[];
  targets(all: boolean): Map<string, number>;
}

export function parseParm(text: string): Map<string, number> {
  const params = new Map<string, number>();
  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.split("#", 1)[0]?.trim() ?? "";
    const [name, value] = line.split(/[\s,]+/);
    if (!name || value === undefined) {
      continue;
    }
    const parsed = Number(value);
    if (Number.isFinite(parsed)) {
      params.set(name, parsed);
    }
  }
  return params;
}

export function mergeConfigTargets(files: string[]): Map<string, number> {
  const merged = new Map<string, number>();
  for (const text of files) {
    for (const [name, value] of parseParm(text)) {
      merged.set(name, value);
    }
  }
  for (const name of merged.keys()) {
    if (CALIBRATION_PARAM_PREFIXES.some((prefix) => name.startsWith(prefix))) {
      merged.delete(name);
    }
  }
  return merged;
}

export interface ConfigRow {
  name: string;
  expected: number;
  actual?: number;
  type?: ParameterResult["type"];
  status: "ok" | "diff" | "missing";
}

export function compareConfig(
  targets: Map<string, number>,
  current: Map<string, ParameterResult>,
): ConfigRow[] {
  return [...targets.keys()].sort().map((name) => {
    const expected = targets.get(name) as number;
    const record = current.get(name);
    if (!record) {
      return { name, expected, status: "missing" };
    }
    return {
      name,
      expected,
      actual: record.value,
      type: record.type,
      status: Math.abs(record.value - expected) < TOLERANCE ? "ok" : "diff",
    };
  });
}

export function matchesTarget(actual: number | undefined, expected: number): boolean {
  return actual !== undefined && Math.abs(actual - expected) < TOLERANCE;
}
