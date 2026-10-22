import { afterEach, describe, expect, it, vi } from "vitest";

import { LinkHubApi } from "../src/api";
import { CommandRunner } from "../src/commands";
import { MavModeFlag, MavParamType, MavState } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";
import { parseRunCollective, PassiveController } from "../src/passive";
import { TelemetryStore } from "../src/telemetry";
import type { LinkHubStatus } from "../src/types";

const controllers: PassiveController[] = [];

afterEach(async () => {
  for (const controller of controllers.splice(0)) {
    if (controller.phase !== "idle") await controller.stop();
  }
  vi.useRealTimers();
});

function setup() {
  vi.useFakeTimers();
  const api = new LinkHubApi();
  const status: LinkHubStatus = {
    connected: true, ready: true, connection: "test", port: null, baud: null,
    clock_epoch: 0, target_system: 1, target_component: 1, base_mode: "",
    custom_mode: 1, system_status: mavEnum(MavState.STANDBY),
    latest_time_boot_ms: 0, received_messages: 0, transmitted_messages: 0,
    received_bytes: 0, transmitted_bytes: 0, rx_bps: null, tx_bps: null,
    rx_bps_by_message: {}, tx_bps_by_message: {}, cursor: "v1:0", generation: "test",
  };
  const params = new Map([
    ["RAWES_MODE", 0], ["H_TAIL_TYPE", 3], ["SERVO9_FUNCTION", 36],
    ["H_FLYBAR_MODE", 1], ["H_SV_MAN", 0], ["IM_ACRO_COL_EXP", 0],
  ]);
  vi.spyOn(api, "status").mockImplementation(async () => ({ ...status }));
  vi.spyOn(api, "getParameter").mockImplementation(async (name) => {
    const value = params.get(name);
    if (value === undefined) throw new Error(`Unexpected parameter ${name}`);
    return { name, value, type: mavEnum(MavParamType.REAL32) };
  });
  const setParameter = vi.spyOn(api, "setParameter").mockImplementation(async (name, value) => {
    params.set(name, value);
    return { name, value, type: mavEnum(MavParamType.REAL32) };
  });
  const mode = vi.spyOn(api, "setModeAndWait").mockResolvedValue();
  vi.spyOn(api, "setMode").mockResolvedValue();
  const arm = vi.spyOn(api, "setArmed").mockImplementation(async () => {
    status.base_mode = "";
  });
  const send = vi.spyOn(api, "sendMessage").mockResolvedValue("v1:1");
  const output: string[] = [];
  const telemetry = new TelemetryStore(api);
  const controller = new PassiveController(api, telemetry, (line) => output.push(line), () => {});
  controllers.push(controller);
  const runner = new CommandRunner(
    api, telemetry, controller, (line) => output.push(line),
    { sources: () => [], targets: () => new Map() },
  );
  return { api, status, params, setParameter, mode, arm, send, output, controller, runner };
}

describe("run collective", () => {
  it("parses bounded collective and duration without armed/passive options", () => {
    expect(parseRunCollective([])).toEqual({ thrust: 0.5, durationSeconds: null });
    expect(parseRunCollective(["--duration", "5", "--trim", "thr=0.25"]))
      .toEqual({ thrust: 0.25, durationSeconds: 5 });
    for (const tokens of [
      ["--force"], ["--roll", "5"], ["--trim", "thr=1.1"],
      ["--trim", "thr=NaN"], ["--duration", "0"], ["--duration"],
      ["--trim", "thr="],
    ]) {
      expect(() => parseRunCollective(tokens)).toThrow();
    }
  });

  it("starts via the shared command runner and only sends collective adjustments", async () => {
    const { runner, controller, params, mode, arm, send, output } = setup();
    await runner.execute("run collective --trim thr=0.6");
    expect(controller.phase).toBe("collective");
    expect(params.get("RAWES_MODE")).toBe(6);
    expect(mode).toHaveBeenCalledExactlyOnceWith(1);
    expect(arm).not.toHaveBeenCalled();
    expect(output.join("\n")).toContain("SERVOS LIVE");
    await controller.adjust("roll", 1);
    await controller.adjust("pitch", -1);
    await controller.adjust("yaw", 1);
    await controller.recaptureTarget();
    expect(send).toHaveBeenCalledTimes(1);
    await controller.adjust("collective", 1);
    expect(send.mock.calls.map(([message]) => message.fields)).toEqual([
      { name: "RAWES_COL", value: 0.6, time_boot_ms: 0 },
      { name: "RAWES_COL", value: 0.65, time_boot_ms: 0 },
    ]);
    await expect(runner.execute("arm")).rejects.toThrow("a run is active");
    await expect(runner.execute("run passive")).rejects.toThrow("Cannot start");
    await runner.execute("stop");
    expect(params.get("RAWES_MODE")).toBe(0);
    expect(controller.phase).toBe("idle");
    const sentAtStop = send.mock.calls.length;
    await vi.advanceTimersByTimeAsync(5_000);
    expect(send).toHaveBeenCalledTimes(sentAtStop);
  });

  it("rejects an armed or disconnected vehicle without mutating it", async () => {
    const { runner, status, setParameter, arm } = setup();
    status.base_mode = MavModeFlag.SAFETY_ARMED;
    await expect(runner.execute("run collective")).rejects.toThrow("confirmed disarmed");
    status.base_mode = "";
    status.connected = false;
    await expect(runner.execute("run collective")).rejects.toThrow("connected");
    expect(setParameter).not.toHaveBeenCalled();
    expect(arm).not.toHaveBeenCalled();
  });

  it("refreshes the unchanged collective and stops at its duration", async () => {
    const { runner, send, controller, params } = setup();
    await runner.execute("run collective --duration 2");
    await vi.advanceTimersByTimeAsync(1_500);
    expect(send).toHaveBeenCalledTimes(4);
    await vi.advanceTimersByTimeAsync(500);
    expect(controller.phase).toBe("idle");
    expect(params.get("RAWES_MODE")).toBe(0);
  });

  it("clamps keyboard collective at both endpoints", async () => {
    const { runner, controller, send } = setup();
    await runner.execute("run collective --trim thr=1");
    await controller.adjust("collective", 1);
    expect(send.mock.calls.at(-1)?.[0].fields).toMatchObject({ value: 1 });
    await runner.execute("stop");
    await runner.execute("run collective --trim thr=0");
    await controller.adjust("collective", -1);
    expect(send.mock.calls.at(-1)?.[0].fields).toMatchObject({ value: 0 });
  });

  it("stops and reports an unexpected arm without sending another collective", async () => {
    const { runner, controller, status, send, arm, params, output } = setup();
    await runner.execute("run collective");
    status.base_mode = MavModeFlag.SAFETY_ARMED;
    await vi.advanceTimersByTimeAsync(500);
    expect(send).toHaveBeenCalledTimes(1);
    expect(arm).toHaveBeenCalledWith(false, true);
    expect(controller.phase).toBe("idle");
    expect(params.get("RAWES_MODE")).toBe(0);
    expect(output.join("\n")).toContain("lost disarmed");
  });

  it("does not reactivate setup after Lua has stopped it", async () => {
    const { runner, controller, params, send } = setup();
    await runner.execute("run collective");
    params.set("RAWES_MODE", 0);
    await vi.advanceTimersByTimeAsync(500);
    expect(controller.phase).toBe("idle");
    expect(send).toHaveBeenCalledTimes(1);
  });

  it("cleans up configuration failures without arming or sending a setpoint", async () => {
    const { runner, params, send, arm, controller } = setup();
    params.set("IM_ACRO_COL_EXP", 0.3);
    await expect(runner.execute("run collective")).rejects.toThrow("IM_ACRO_COL_EXP=0");
    expect(controller.phase).toBe("idle");
    expect(params.get("RAWES_MODE")).toBe(0);
    expect(send).not.toHaveBeenCalled();
    expect(arm).not.toHaveBeenCalled();
  });

  it("reports keepalive failures and attempts safe-off", async () => {
    const { runner, api, params, output, controller } = setup();
    await runner.execute("run collective");
    vi.mocked(api.status).mockRejectedValueOnce(new Error("link failed"));
    await vi.advanceTimersByTimeAsync(500);
    expect(controller.phase).toBe("idle");
    expect(params.get("RAWES_MODE")).toBe(0);
    expect(output.join("\n")).toContain("link failed");
  });

  it("cleans up a failed keyboard adjustment", async () => {
    const { runner, controller, send, params } = setup();
    await runner.execute("run collective");
    send.mockRejectedValueOnce(new Error("adjust failed"));
    await expect(controller.adjust("collective", 1)).rejects.toThrow("adjust failed");
    expect(controller.phase).toBe("idle");
    expect(params.get("RAWES_MODE")).toBe(0);
  });

  it("does not enter setup after a stop during startup", async () => {
    const { runner, mode, controller, send, params } = setup();
    let releaseMode = () => {};
    mode.mockImplementationOnce(() => new Promise<void>((resolve) => { releaseMode = resolve; }));
    const starting = runner.execute("run collective");
    await vi.waitFor(() => expect(mode).toHaveBeenCalledOnce());
    await runner.execute("stop");
    releaseMode();
    await expect(starting).rejects.toThrow("abort");
    expect(controller.phase).toBe("idle");
    expect(params.get("RAWES_MODE")).toBe(0);
    expect(send).not.toHaveBeenCalled();
  });
});
