const COPTER_MODES: Record<number, string> = {
  0: "STABILIZE",
  1: "ACRO",
  2: "ALT_HOLD",
  3: "AUTO",
  4: "GUIDED",
  5: "LOITER",
  6: "RTL",
  9: "LAND",
  16: "POSHOLD",
  20: "GUIDED_NOGPS",
};

export function copterModeName(mode: number): string {
  return COPTER_MODES[mode] ?? `MODE_${mode}`;
}

export function formatCopterMode(mode: number | undefined): string {
  return mode === undefined ? "n/a" : `${copterModeName(mode)} (${mode})`;
}
