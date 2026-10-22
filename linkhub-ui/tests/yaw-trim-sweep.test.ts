import { afterEach, describe, expect, it, vi } from "vitest";
import { LinkHubApi } from "../src/api";
import { parseOptions, safeOff, trimLadder, waitFor } from "../scripts/yaw-trim-sweep";

afterEach(() => {
  vi.restoreAllMocks();
});

describe("yaw trim sweep", () => {
  it("never exceeds the requested trim or hard cap for non-divisible steps", () => {
    const options = parseOptions(["--stop", "0.5", "--step", "0.3"]);
    expect(trimLadder(options)).toEqual([0, 0.3, 0.5, 0.3, 0]);
    expect(trimLadder(parseOptions(["--stop", "0.1", "--step", "0.3"])))
      .toEqual([0, 0.1, 0]);
  });

  it("supports a single step and an ascending-only sweep", () => {
    expect(trimLadder(parseOptions(["--start", "0.1", "--stop", "0.1"]))).toEqual([0.1]);
    expect(trimLadder(parseOptions(["--stop", "0.1", "--step", "0.05", "--no-down"])))
      .toEqual([0, 0.05, 0.1]);
  });

  it.each([
    ["--max-rate", "0"],
    ["--max-rate", "-1"],
    ["--stop", "0.6"],
    ["--start", "-0.1"],
    ["--step", "0"],
    ["--step", "0.000000001"],
    ["--hold", "0.5"],
    ["--stop", "NaN"],
  ])("rejects invalid options %s %s", (...args) => {
    expect(() => parseOptions(args)).toThrow();
  });

  it("honors cancellation even if the awaited telemetry condition is already true", async () => {
    const controller = new AbortController();
    controller.abort(new Error("operator stop"));
    await expect(waitFor(() => true, 1000, "telemetry", controller.signal))
      .rejects.toThrow("operator stop");
  });

  function mockSafeOff() {
    const api = new LinkHubApi();
    vi.spyOn(console, "log").mockImplementation(() => {});
    vi.spyOn(api, "status").mockResolvedValue({
      connected: true, ready: true, base_mode: "", custom_mode: 1,
      cursor: "v1:10", generation: "run-1",
      connection: "tcp:127.0.0.1:5760", port: null, baud: null, clock_epoch: 0,
      target_system: 1, target_component: 1, system_status: { type: "MAV_STATE_STANDBY" },
      latest_time_boot_ms: 1, received_messages: 1, transmitted_messages: 0,
      received_bytes: 1, transmitted_bytes: 0, rx_bps: null, tx_bps: null,
      rx_bps_by_message: {}, tx_bps_by_message: {},
    });
    const set = vi.spyOn(api, "setParameter").mockImplementation(async (name, value) => ({
      name, value, type: { type: "MAV_PARAM_TYPE_REAL32" },
    }));
    vi.spyOn(api, "getParameter").mockImplementation(async (name) => ({
      name, value: name === "SERVO9_MIN" ? 1000 : 36,
      type: { type: "MAV_PARAM_TYPE_INT32" },
    }));
    vi.spyOn(api, "setMode").mockResolvedValue();
    const messages = vi.spyOn(api, "queryMessages").mockResolvedValue({
      next_cursor: "v1:11",
      records: [{
        cursor: "v1:11", received_time: "", received_time_ns: 1,
        direction: "rx", system_id: 1, component_id: 1,
        message: "SERVO_OUTPUT_RAW", fields: { servo9_raw: 1000 },
      }],
    });
    return { api, set, messages };
  }

  it("uses canonical cleanup, restores gains and verifies fresh output telemetry", async () => {
    const { api, set, messages } = mockSafeOff();
    await safeOff(api, new Map([["ATC_RAT_YAW_P", 0.18]]));
    expect(set).toHaveBeenCalledWith("RAWES_MODE", 0, true, undefined);
    expect(set).toHaveBeenCalledWith("H_YAW_TRIM", 0, true, undefined);
    expect(set).toHaveBeenCalledWith("ATC_RAT_YAW_P", 0.18, false);
    expect(messages).toHaveBeenCalledWith(expect.objectContaining({
      after: "v1:10", expectedGeneration: "run-1", direction: "rx",
    }));
  });

  it("fails cleanup instead of accepting missing fresh servo telemetry", async () => {
    const { api, messages } = mockSafeOff();
    messages.mockResolvedValue({ records: [], next_cursor: "v1:10" });
    await expect(safeOff(api, new Map())).rejects.toThrow("Safe-off verification failed");
  });

  it("reports cleanup errors while still attempting gain restoration", async () => {
    const { api, set } = mockSafeOff();
    set.mockRejectedValueOnce(new Error("parameter write failed"));
    await expect(safeOff(api, new Map([["ATC_RAT_YAW_I", 0.02]])))
      .rejects.toThrow("Safe-off verification failed");
    expect(set).toHaveBeenCalledWith("ATC_RAT_YAW_I", 0.02, false);
  });
});
