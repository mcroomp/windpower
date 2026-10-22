import { afterEach, beforeEach, describe, expect, it, vi, type MockInstance } from "vitest";
import { LinkHubApi } from "../src/api";
import { MavModeFlag, MavParamType, MavState } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";
import { TelemetryStore } from "../src/telemetry";
import {
  DEFAULT_TELEMETRY_PROFILE,
  DISPLAY_TELEMETRY_RATES,
  selectTelemetryProfile,
  TELEMETRY_PROFILE_OPTIONS,
  TELEMETRY_PROFILE_PARAM_NAMES,
  TELEMETRY_PROFILES,
} from "../src/telemetry-rates";
import type { LinkHubStatus } from "../src/types";

it("requests measured RPM telemetry at 100 Hz over Wi-Fi", () => {
  expect(DISPLAY_TELEMETRY_RATES.RPM).toBe(100);
});

it("sets the same rates and parameters in every profile so switching leaves nothing stale", () => {
  const [first, ...rest] = Object.values(TELEMETRY_PROFILES);
  for (const profile of rest) {
    expect(Object.keys(profile.rates).sort()).toEqual(Object.keys(first!.rates).sort());
    expect(Object.keys(profile.params).sort()).toEqual(Object.keys(first!.params).sort());
  }
  expect(TELEMETRY_PROFILE_PARAM_NAMES).toEqual(["RAWES_TEL_HZ"]);
});

it("carries yaw tracking diagnostics in every profile", () => {
  expect(TELEMETRY_PROFILES.wifi.rates).toMatchObject({
    PID_TUNING: 200, EKF_STATUS_REPORT: 2, VIBRATION: 2, ESC_TELEMETRY_1_TO_4: 10,
  });
  expect(TELEMETRY_PROFILES.wifi.params).toEqual({ RAWES_TEL_HZ: 20 });
  expect(TELEMETRY_PROFILES.radio.rates).toMatchObject({
    PID_TUNING: 4, EKF_STATUS_REPORT: 2, VIBRATION: 2, ESC_TELEMETRY_1_TO_4: 1,
  });
  expect(TELEMETRY_PROFILES.radio.params).toEqual({ RAWES_TEL_HZ: 2 });
});

it("loads selectable Wi-Fi, BLE and radio profiles from JSON", () => {
  expect(DEFAULT_TELEMETRY_PROFILE).toBe("wifi");
  expect(TELEMETRY_PROFILE_OPTIONS).toEqual([
    { name: "wifi", label: "Wi-Fi" },
    { name: "ble", label: "BLE" },
    { name: "radio", label: "Radio" },
  ]);
});

it("selects the largest profile supported by the detected baud", () => {
  expect(selectTelemetryProfile("tcp:127.0.0.1:5760", null)).toBe("wifi");
  expect(selectTelemetryProfile("auto", 1_500_000)).toBe("wifi");
  expect(selectTelemetryProfile("auto", 2_000_000)).toBe("wifi");
  expect(selectTelemetryProfile("auto", 921_600)).toBe("radio");
  expect(selectTelemetryProfile("auto", 57600)).toBe("radio");
  expect(selectTelemetryProfile("auto", 38400)).toBe("radio");
});

it("selects the BLE profile for a DroneBridge BLE connection", () => {
  expect(selectTelemetryProfile("ble:DroneBridge", null)).toBe("ble");
  expect(TELEMETRY_PROFILES.ble.rates.ATTITUDE_QUATERNION).toBe(50);
});

it("uses reduced rates for radio telemetry", () => {
  expect(TELEMETRY_PROFILES.radio.rates).toEqual({
    ATTITUDE: 0,
    ATTITUDE_QUATERNION: 10,
    ATTITUDE_TARGET: 10,
    SERVO_OUTPUT_RAW: 10,
    PID_TUNING: 4,
    LOCAL_POSITION_NED: 5,
    BATTERY_STATUS: 1,
    RC_CHANNELS: 1,
    RPM: 5,
    EKF_STATUS_REPORT: 2,
    VIBRATION: 2,
    ESC_TELEMETRY_1_TO_4: 1,
  });
});

function status(generation: string, ready = true, baud: number | null = 1_500_000): LinkHubStatus {
  return {
    connected: ready, ready, generation, cursor: "v1:0", connection: "serial:test",
    port: ready ? "COM1" : null, baud: ready ? baud : null,
    clock_epoch: 1, target_system: 1, target_component: 1, base_mode: "",
    custom_mode: 0, system_status: mavEnum(MavState.STANDBY), latest_time_boot_ms: 0,
    received_messages: 0, transmitted_messages: 0,
    received_bytes: 0, transmitted_bytes: 0, rx_bps: null, tx_bps: null,
    rx_bps_by_message: {}, tx_bps_by_message: {},
  };
}

let setParameter: MockInstance<LinkHubApi["setParameter"]>;

beforeEach(() => {
  setParameter = vi.spyOn(LinkHubApi.prototype, "setParameter").mockResolvedValue({
    name: "RAWES_TEL_HZ", value: 0, type: mavEnum(MavParamType.REAL32),
  });
});

afterEach(() => vi.restoreAllMocks());

describe("live display telemetry", () => {
  it("applies profile parameters as floats after the rates", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    vi.spyOn(api, "readMessages").mockImplementation(() => new Promise(() => {}));
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    expect(setParameter).toHaveBeenCalledExactlyOnceWith("RAWES_TEL_HZ", 20, false);
    telemetry.stop();
  });

  it("warns about an unapplied parameter without blocking the rates", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    vi.spyOn(api, "readMessages").mockImplementation(() => new Promise(() => {}));
    setParameter.mockRejectedValue(new Error("parameter not found"));
    const telemetry = new TelemetryStore(api);
    const warnings: string[] = [];
    telemetry.onWarning((message) => warnings.push(message));
    await telemetry.start();
    expect(rates).toHaveBeenCalledTimes(1);
    expect(telemetry.status).not.toBeNull();
    expect(warnings).toEqual([
      "Telemetry profile wifi: RAWES_TEL_HZ not applied: parameter not found",
    ]);
    telemetry.stop();
  });

  it("restores the lower diagnostic rate when switching to radio", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    vi.spyOn(api, "readMessages").mockImplementation(() => new Promise(() => {}));
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    await telemetry.setProfile("radio");
    expect(setParameter.mock.calls).toEqual([
      ["RAWES_TEL_HZ", 20, false],
      ["RAWES_TEL_HZ", 2, false],
    ]);
    telemetry.stop();
  });

  it("requests streams at startup without starting a passive run", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    vi.spyOn(api, "readMessages").mockImplementation(() => new Promise(() => {}));
    const telemetry = new TelemetryStore(api);
    const onStatus = vi.fn();
    telemetry.onStatus(onStatus);
    await telemetry.start();
    expect(rates).toHaveBeenCalledExactlyOnceWith(DISPLAY_TELEMETRY_RATES);
    expect(onStatus).toHaveBeenCalledExactlyOnceWith(status("first"));
    telemetry.stop();
  });

  it("selects the radio profile for a lower-baud serial connection", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first", true, 57600));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    vi.spyOn(api, "readMessages").mockImplementation(() => new Promise(() => {}));
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    expect(telemetry.profile).toBe("radio");
    expect(rates).toHaveBeenCalledExactlyOnceWith(TELEMETRY_PROFILES.radio.rates);
    telemetry.stop();
  });

  it("reports stream configuration failures rather than showing success", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    vi.spyOn(api, "setMessageRates").mockRejectedValue(new Error("stream request rejected"));
    const telemetry = new TelemetryStore(api);
    await expect(telemetry.start()).rejects.toThrow("stream request rejected");
    expect(telemetry.status).toBeNull();
    telemetry.stop();
  });

  it("applies a manual profile immediately and auto-selects on a new generation", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status")
      .mockResolvedValueOnce(status("first"))
      .mockResolvedValueOnce(status("second"));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    let reads = 0;
    vi.spyOn(performance, "now").mockImplementation(() => reads * 1_100);
    vi.spyOn(api, "readMessages").mockImplementation(async () => {
      reads += 1;
      if (reads > 1) {
        return new Promise(() => {});
      }
      return { records: [], next_cursor: "v1:0" };
    });
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    await telemetry.setProfile("radio");
    await vi.waitFor(() => expect(rates).toHaveBeenCalledTimes(3));
    expect(telemetry.profile).toBe("wifi");
    expect(rates.mock.calls).toEqual([
      [TELEMETRY_PROFILES.wifi.rates],
      [TELEMETRY_PROFILES.radio.rates],
      [TELEMETRY_PROFILES.wifi.rates],
    ]);
    telemetry.stop();
  });

  it("configures once per ready generation and defers while disconnected", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status")
      .mockResolvedValueOnce(status("first"))
      .mockResolvedValueOnce(status("first"))
      .mockResolvedValueOnce(status("second", false))
      .mockResolvedValueOnce(status("second"));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    let reads = 0;
    vi.spyOn(performance, "now").mockImplementation(() => reads * 1_100);
    vi.spyOn(api, "readMessages").mockImplementation(async () => {
      reads += 1;
      if (reads > 3) {
        return new Promise(() => {});
      }
      return { records: [], next_cursor: "v1:0" };
    });
    const telemetry = new TelemetryStore(api);
    const snapshots: LinkHubStatus[] = [];
    const unsubscribe = telemetry.onStatus((snapshot) => snapshots.push(snapshot));
    await telemetry.start();
    await vi.waitFor(() => expect(rates).toHaveBeenCalledTimes(2));
    expect(rates.mock.calls).toEqual([
      [DISPLAY_TELEMETRY_RATES], [DISPLAY_TELEMETRY_RATES],
    ]);
    expect(snapshots.map((snapshot) => snapshot.generation)).toEqual([
      "first", "first", "second", "second",
    ]);
    unsubscribe();
    telemetry.stop();
  });

  it("folds typed HEARTBEAT mode fields into the live status", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue({ ...status("first"), cursor: "v1:1" });
    vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    let delivered = false;
    vi.spyOn(api, "readMessages").mockImplementation(async () => {
      if (delivered) {
        return new Promise(() => {});
      }
      delivered = true;
      return {
        records: [{
          received_time: "t", received_time_ns: 1, direction: "rx", system_id: 1,
          component_id: 1, message: "HEARTBEAT", cursor: "v1:1",
          fields: {
            mavtype: { type: "MAV_TYPE_HELICOPTER" },
            autopilot: { type: "MAV_AUTOPILOT_ARDUPILOTMEGA" },
            base_mode: `${MavModeFlag.SAFETY_ARMED} | ${MavModeFlag.CUSTOM_MODE_ENABLED}`,
            custom_mode: 20,
            system_status: { type: "MAV_STATE_FLIGHT_TERMINATION" },
          },
        }],
        next_cursor: "v1:1",
      };
    });
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    expect(telemetry.status).toMatchObject({
      base_mode: "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
      custom_mode: 20,
      system_status: { type: "MAV_STATE_FLIGHT_TERMINATION" },
    });
    telemetry.stop();
  });
});
