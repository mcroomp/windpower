import { describe, expect, it } from "vitest";

import { MavModeFlag, MavState } from "../src/generated/protocol";
import {
  enumIs,
  enumName,
  hasFlag,
  heartbeatState,
  isArmed,
  isVehicleHeartbeat,
  mavEnum,
  parseFlags,
} from "../src/mav";

describe("enumName", () => {
  it("reads the full dialect name from the wire shape", () => {
    expect(enumName({ type: "MAV_STATE_ACTIVE" })).toBe("MAV_STATE_ACTIVE");
  });

  it("tolerates names outside the generated enums", () => {
    expect(enumName({ type: "MAV_STATE_FLIGHT_TERMINATION" })).toBe("MAV_STATE_FLIGHT_TERMINATION");
  });

  it("rejects numbers and malformed values", () => {
    expect(enumName(4)).toBeNull();
    expect(enumName(null)).toBeNull();
    expect(enumName(undefined)).toBeNull();
    expect(enumName("MAV_STATE_ACTIVE")).toBeNull();
    expect(enumName({ type: 4 })).toBeNull();
  });
});

describe("enumIs", () => {
  it("compares the dialect name", () => {
    expect(enumIs(mavEnum(MavState.ACTIVE), MavState.ACTIVE)).toBe(true);
    expect(enumIs(mavEnum(MavState.STANDBY), MavState.ACTIVE)).toBe(false);
    expect(enumIs(undefined, MavState.ACTIVE)).toBe(false);
  });
});

describe("parseFlags", () => {
  it("splits a bitmask into member names", () => {
    expect([...parseFlags("MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED")])
      .toEqual(["MAV_MODE_FLAG_SAFETY_ARMED", "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"]);
  });

  it("treats the empty string as no flags", () => {
    expect(parseFlags("").size).toBe(0);
  });

  it("treats non-strings as no flags", () => {
    expect(parseFlags(128).size).toBe(0);
    expect(parseFlags(undefined).size).toBe(0);
  });

  it("tolerates unlisted flag names", () => {
    expect(hasFlag("SOME_FUTURE_FLAG | MAV_MODE_FLAG_SAFETY_ARMED", MavModeFlag.SAFETY_ARMED))
      .toBe(true);
    expect(hasFlag("SOME_FUTURE_FLAG", MavModeFlag.SAFETY_ARMED)).toBe(false);
  });
});

describe("isArmed", () => {
  it("reads the safety-armed flag", () => {
    expect(isArmed({ base_mode: "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED" }))
      .toBe(true);
    expect(isArmed({ base_mode: "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED" })).toBe(false);
    expect(isArmed({ base_mode: "" })).toBe(false);
    expect(isArmed(null)).toBe(false);
  });
});

describe("heartbeatState", () => {
  it("extracts typed mode state", () => {
    expect(heartbeatState({
      mavtype: { type: "MAV_TYPE_HELICOPTER" },
      base_mode: "MAV_MODE_FLAG_SAFETY_ARMED",
      custom_mode: 20,
      system_status: { type: "MAV_STATE_ACTIVE" },
    })).toEqual({
      base_mode: "MAV_MODE_FLAG_SAFETY_ARMED",
      custom_mode: 20,
      system_status: { type: "MAV_STATE_ACTIVE" },
    });
  });

  it("rejects the old numeric shapes", () => {
    expect(heartbeatState({ base_mode: 128, custom_mode: 20, system_status: 4 })).toBeNull();
    expect(heartbeatState({
      base_mode: "",
      custom_mode: 20,
      system_status: 4,
    })).toBeNull();
  });

  it("ignores heartbeats from radios and other non-autopilot components", () => {
    const radio = {
      mavtype: { type: "MAV_TYPE_ONBOARD_CONTROLLER" },
      autopilot: { type: "MAV_AUTOPILOT_INVALID" },
      base_mode: "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
      custom_mode: 0,
      system_status: { type: "MAV_STATE_ACTIVE" },
    };
    expect(isVehicleHeartbeat(radio)).toBe(false);
    expect(heartbeatState(radio)).toBeNull();
    expect(isVehicleHeartbeat({
      ...radio,
      autopilot: { type: "MAV_AUTOPILOT_ARDUPILOTMEGA" },
    })).toBe(true);
  });
});
