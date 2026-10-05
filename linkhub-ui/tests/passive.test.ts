import { describe, expect, it } from "vitest";

import { PASSIVE_TELEMETRY_RATES, parseRunPassive } from "../src/passive";

describe("parseRunPassive", () => {
  it("uses production passive defaults", () => {
    expect(parseRunPassive([])).toEqual({
      force: false,
      thrust: 0.342,
      durationSeconds: null,
      rollDegrees: 0,
      pitchDegrees: 0,
      yawDegrees: 0,
    });
  });

  it("parses supported passive options", () => {
    expect(parseRunPassive([
      "--force",
      "--duration", "30",
      "--trim", "thr=0.5",
      "--roll", "5",
      "--pitch", "-10",
      "--yaw", "15",
    ])).toEqual({
      force: true,
      thrust: 0.5,
      durationSeconds: 30,
      rollDegrees: 5,
      pitchDegrees: -10,
      yawDegrees: 15,
    });
  });

  describe("passive telemetry", () => {
    it("does not request a rate for event-driven STATUSTEXT messages", () => {
      expect(PASSIVE_TELEMETRY_RATES).not.toHaveProperty("STATUSTEXT");
    });
  });

  it("rejects unsafe or unsupported inputs", () => {
    expect(() => parseRunPassive(["--trim", "thr=1.1"])).toThrow("0..1");
    expect(() => parseRunPassive(["--roll", "31"])).toThrow("-30..30");
    expect(() => parseRunPassive(["--osc", "all"])).toThrow("Unknown");
  });
});
