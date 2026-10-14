import profileConfig from "./telemetry-profiles.json";

export type TelemetryProfile = keyof typeof profileConfig.profiles;

export const TELEMETRY_PROFILES = profileConfig.profiles;

export function isTelemetryProfile(value: string): value is TelemetryProfile {
  return Object.hasOwn(profileConfig.profiles, value);
}

if (!isTelemetryProfile(profileConfig.default)) {
  throw new Error(`Unknown default telemetry profile ${profileConfig.default}`);
}

export const DEFAULT_TELEMETRY_PROFILE = profileConfig.default;

// Rates and vehicle parameters persist until changed, so every profile must set
// the same keys; otherwise switching profiles would leave stale values behind.
function profileKeys(kind: "rates" | "params"): string[] {
  const sets = Object.entries(profileConfig.profiles).map(([name, profile]) => ({
    name,
    keys: Object.keys(profile[kind]).sort(),
  }));
  const [first, ...rest] = sets;
  for (const other of rest) {
    if (first && other.keys.join() !== first.keys.join()) {
      throw new Error(
        `Telemetry profiles ${first.name} and ${other.name} must set the same ${kind}`,
      );
    }
  }
  return first?.keys ?? [];
}

profileKeys("rates");

/** Vehicle parameters owned by the telemetry profiles (excluded from config checks). */
export const TELEMETRY_PROFILE_PARAM_NAMES: readonly string[] = profileKeys("params");

export const TELEMETRY_PROFILE_OPTIONS: readonly {
  name: TelemetryProfile;
  label: string;
}[] = Object.entries(profileConfig.profiles).map(([name, profile]) => ({
  name: name as TelemetryProfile,
  label: profile.label,
}));

export function selectTelemetryProfile(baud: number | null | undefined): TelemetryProfile {
  const profiles = TELEMETRY_PROFILE_OPTIONS
    .map(({ name }) => ({ name, minbaud: TELEMETRY_PROFILES[name].minbaud }))
    .sort((left, right) => left.minbaud - right.minbaud);
  const selected = baud === null || baud === undefined
    ? profiles.at(-1)
    : profiles.filter((profile) => profile.minbaud <= baud).at(-1) ?? profiles[0];
  if (!selected) {
    throw new Error("At least one telemetry profile is required");
  }
  return selected.name;
}

export const DISPLAY_TELEMETRY_RATES =
  TELEMETRY_PROFILES[DEFAULT_TELEMETRY_PROFILE].rates;
