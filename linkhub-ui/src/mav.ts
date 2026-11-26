import { MavAutopilot, MavModeFlag, type MavEnum } from "./generated/protocol";
import type { JsonObject, LinkHubStatus } from "./types";

const FLAG_SEPARATOR = " | ";

function validMillivolts(value: unknown): value is number {
  return typeof value === "number" && Number.isInteger(value) && value > 0 && value < 65_535;
}

export function reportedBatteryVolts(battery?: JsonObject, system?: JsonObject): number | null {
  let millivolts = 0;
  for (const cells of [battery?.voltages, battery?.voltages_ext]) {
    if (!Array.isArray(cells)) continue;
    for (const cell of cells) {
      if (validMillivolts(cell)) millivolts += cell;
    }
  }
  if (millivolts > 0) return millivolts / 1_000;
  const fallback = system?.voltage_battery;
  return validMillivolts(fallback) ? fallback / 1_000 : null;
}

/** Wraps a dialect name in LinkHub's enumeration wire shape. */
export function mavEnum<TName extends string>(name: TName): MavEnum<TName> {
  return { type: name };
}

/** Full dialect name of a wire enumeration, or null when the value is not one. */
export function enumName(value: unknown): string | null {
  if (typeof value !== "object" || value === null) {
    return null;
  }
  const name = (value as { type?: unknown }).type;
  return typeof name === "string" ? name : null;
}

export function enumIs(value: unknown, expected: string): boolean {
  return enumName(value) === expected;
}

/** Splits a wire bitmask into its member names; anything but a non-empty string has none. */
export function parseFlags(value: unknown): ReadonlySet<string> {
  if (typeof value !== "string" || value === "") {
    return new Set();
  }
  return new Set(value.split(FLAG_SEPARATOR).map((name) => name.trim()));
}

export function hasFlag(value: unknown, flag: string): boolean {
  return parseFlags(value).has(flag);
}

export function isArmed(status: Pick<LinkHubStatus, "base_mode"> | null | undefined): boolean {
  return status !== null && status !== undefined
    && hasFlag(status.base_mode, MavModeFlag.SAFETY_ARMED);
}

/**
 * True for a heartbeat that describes the vehicle. Radios (a DroneBridge ESP32),
 * GCSs and companion computers heartbeat with autopilot INVALID and would
 * otherwise overwrite the vehicle's mode and armed state.
 */
export function isVehicleHeartbeat(fields: JsonObject): boolean {
  return !enumIs(fields.autopilot, MavAutopilot.INVALID);
}

/** Mode fields of a HEARTBEAT record, or null when the record is malformed. */
export function heartbeatState(
  fields: JsonObject,
): Pick<LinkHubStatus, "base_mode" | "custom_mode" | "system_status"> | null {
  const { base_mode: baseMode, custom_mode: customMode, system_status: systemStatus } = fields;
  if (
    !isVehicleHeartbeat(fields)
    || typeof baseMode !== "string"
    || typeof customMode !== "number"
    || enumName(systemStatus) === null
  ) {
    return null;
  }
  return {
    base_mode: baseMode,
    custom_mode: customMode,
    system_status: systemStatus as LinkHubStatus["system_status"],
  };
}
