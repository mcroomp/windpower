import type { MessageRecord } from "./types";

/**
 * Schema of rawes.lua's diagnostic DEBUG_FLOAT_ARRAY. Owned by
 * groundstation/rawes_diag.py (which documents the layout); keep DIAG_KEYS in
 * sync with it and with `_diag_nvf_keys` in scripts/rawes.lua.
 * tests/unit/test_rawes_diag.py checks all three.
 */
export const DIAG_ARRAY_ID = 1;
export const DIAG_ARRAY_NAME = "RAWES_DIAG";

export const DIAG_KEYS = [
  "YFF_T", "YFF_U", "YFF_GZ",
  "OL_RSP", "OL_PSP", "OL_YSP",
  "OL_RER", "OL_PER", "OL_YER",
  "OL_AP", "OL_AI", "OL_AD", "OL_COL",
  "OL_TEN",
  "ANCH_N", "ANCH_E", "ANCH_D",
] as const;

export type DiagKey = (typeof DIAG_KEYS)[number];

export function isDiagRecord(record: MessageRecord): boolean {
  return record.direction === "rx"
    && record.message === "DEBUG_FLOAT_ARRAY"
    && record.fields.array_id === DIAG_ARRAY_ID;
}

/** Value of `key` in a diagnostic array record, or undefined if it was not set. */
export function diagValue(
  record: MessageRecord | undefined,
  key: DiagKey,
): number | undefined {
  if (!record || record.fields.array_id !== DIAG_ARRAY_ID
    || !Array.isArray(record.fields.data)) {
    return undefined;
  }
  const data = record.fields.data as unknown[];
  const index = DIAG_KEYS.indexOf(key);
  const mask = Number(data[0] ?? 0);
  if (!Number.isInteger(mask) || (mask & (1 << index)) === 0) {
    return undefined;
  }
  const value = Number(data[index + 1] ?? 0);
  return Number.isFinite(value) ? value : undefined;
}
