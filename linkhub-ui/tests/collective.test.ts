import { afterEach, describe, expect, it, vi } from "vitest";

import { LinkHubApi } from "../src/api";
import { CommandRunner } from "../src/commands";
import { MavModeFlag, MavParamType, MavState } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";
import { parseRunCollective, PassiveController } from "../src/passive";
import { TelemetryStore } from "../src/telemetry";
import type { LinkHubStatus, MessageRecord } from "../src/types";

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
    ["H_COL_MIN", 1000], ["H_COL_MAX", 2000], ["H_CYC_MAX", 1000],
    ["H_SW_TYPE", 3], ["H_SW_COL_DIR", 1], ["H_SW_LIN_SVO", 0],
  ]);
  for (let channel = 1; channel <= 3; channel++) {
    for (const [suffix, value] of [
      ["FUNCTION", 32 + channel], ["MIN", 1000], ["MAX", 2000], ["TRIM", 1517], ["REVERSED", 1],
    ] as const) params.set(`SERVO${channel}_${suffix}`, value);
  }
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
  const setParameters = vi.spyOn(api, "setParameters").mockImplementation(async (parameters) => {
    for (const parameter of parameters) params.set(parameter.name, parameter.value);
    return new Map(parameters.map((parameter) => [parameter.name, parameter]));
  });
  const mode = vi.spyOn(api, "setModeAndWait").mockResolvedValue();
  vi.spyOn(api, "setMode").mockResolvedValue();
  const arm = vi.spyOn(api, "setArmed").mockImplementation(async () => {
    status.base_mode = "";
  });
  const send = vi.spyOn(api, "sendMessage").mockResolvedValue("v1:1");
  const output: string[] = [];
  const telemetry = new TelemetryStore(api);
  vi.spyOn(telemetry, "status", "get").mockImplementation(() => status);
  const listeners: ((record: MessageRecord) => void)[] = [];
  vi.spyOn(telemetry, "onRecord").mockImplementation((listener) => {
    listeners.push(listener);
    return () => {};
  });
  const feedServos = () => {
    for (const listener of listeners) listener({
      message: "SERVO_OUTPUT_RAW", direction: "rx", system_id: 1, component_id: 1,
      cursor: "v1:1", received_time: "", received_time_ns: 0,
      fields: { port: 0, servo1_raw: 1500, servo2_raw: 1500, servo3_raw: 1500, servo9_raw: 1000 },
    });
  };
  const controller = new PassiveController(api, telemetry, (line) => output.push(line), () => {});
  controllers.push(controller);
  const runner = new CommandRunner(
    api, telemetry, controller, (line) => output.push(line),
    { sources: () => [], targets: () => new Map() },
  );
  return { api, status, params, setParameter, setParameters, mode, arm, send, output, controller, runner, feedServos };
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

  describe("swash calibration", () => {
    it("reads actual mixer and servo calibration without writing or enabling motion", async () => {
      const { controller, setParameter, send } = setup();
      const geometry = await controller.readSwashGeometry();
      expect(geometry).toEqual({
        type: 3, collectiveReversed: true, linear: false,
        servos: Array.from({ length: 3 }, () => ({ min: 1000, max: 2000, trim: 1517, reversed: true })),
      });
      expect(setParameter).not.toHaveBeenCalled();
      expect(send).not.toHaveBeenCalled();
      expect(controller.phase).toBe("idle");
    });

    it.each([
      ["H_SW_COL_DIR", 2, "must be 0 or 1"],
      ["SERVO2_REVERSED", NaN, "not finite"],
      ["SERVO3_FUNCTION", 36, "SERVO3_FUNCTION=35"],
    ])("rejects invalid envelope dependency %s", async (name, value, message) => {
      const { controller, params, setParameter } = setup();
      params.set(name, value);
      await expect(controller.readSwashGeometry()).rejects.toThrow(message);
      expect(setParameter).not.toHaveBeenCalled();
    });

    it("reports missing envelope calibration instead of using neutral trim defaults", async () => {
      const { controller, params } = setup();
      params.delete("SERVO1_TRIM");
      await expect(controller.readSwashGeometry()).rejects.toThrow("SERVO1_TRIM");
    });

    it("uses collective mode 6 for all three normalized axes, without arming", async () => {
      const { controller, feedServos, params, send, arm, runner } = setup();
      feedServos();
      await controller.startSwash();
      expect(controller.phase).toBe("swash");
      expect(params.get("RAWES_MODE")).toBe(6);
      const moving = controller.setSwashTarget({ roll: 0.6, pitch: -0.8, collective: 0.7 });
      await vi.advanceTimersByTimeAsync(100);
      await moving;
      expect(send.mock.calls.slice(-3).map(([message]) => message.fields)).toEqual([
        { name: "RAWES_RLL", value: 0.6, time_boot_ms: 0 },
        { name: "RAWES_PIT", value: -0.8, time_boot_ms: 0 },
        { name: "RAWES_COL", value: 0.7, time_boot_ms: 0 },
      ]);
      expect(arm).not.toHaveBeenCalled();
      await expect(runner.execute("arm")).rejects.toThrow("run is active");
    });

    it("refuses movement without fresh servo outputs and stops when they stall", async () => {
      const { controller, feedServos, params, output } = setup();
      await expect(controller.startSwash()).rejects.toThrow("fresh");
      feedServos();
      await controller.startSwash();
      await vi.advanceTimersByTimeAsync(2000);
      expect(controller.phase).toBe("idle");
      expect(params.get("RAWES_MODE")).toBe(0);
      expect(output.join("\n")).toContain("telemetry is stale");
    });

    it("rejects invalid targets before transmitting", async () => {
      const { controller, feedServos, send } = setup();
      feedServos();
      await controller.startSwash();
      send.mockClear();
      await expect(controller.setSwashTarget({ roll: 1, pitch: 1, collective: 0.5 })).rejects.toThrow("unit cyclic circle");
      await expect(controller.setSwashTarget({ roll: 0, pitch: 0, collective: NaN })).rejects.toThrow("finite");
      expect(send).not.toHaveBeenCalled();
    });

    it("coalesces dragging and prevents remaining axis writes after Stop", async () => {
      const { controller, feedServos, send } = setup();
      feedServos();
      await controller.startSwash();
      send.mockClear();
      let release = () => {};
      send.mockImplementationOnce(() => new Promise<string>((resolve) => { release = () => resolve("v1:2"); }));
      const moving = controller.setSwashTarget({ roll: 0.5, pitch: 0, collective: 0.5 });
      const rejected = expect(moving).rejects.toThrow();
      await vi.advanceTimersByTimeAsync(100);
      await controller.stop();
      release();
      await rejected;
      expect(send).toHaveBeenCalledTimes(1);
      expect(controller.phase).toBe("idle");
    });

    it("rate-limits dragging to 10 Hz and coalesces to the latest target", async () => {
      const { controller, feedServos, send } = setup();
      feedServos();
      await controller.startSwash();
      send.mockClear();
      const first = controller.setSwashTarget({ roll: 0.1, pitch: 0, collective: 0.5 });
      const second = controller.setSwashTarget({ roll: 0.2, pitch: 0, collective: 0.6 });
      const third = controller.setSwashTarget({ roll: 0.3, pitch: 0, collective: 0.7 });
      expect(send).not.toHaveBeenCalled();
      await vi.advanceTimersByTimeAsync(99);
      expect(send).not.toHaveBeenCalled();
      await vi.advanceTimersByTimeAsync(1);
      await Promise.all([first, second, third]);
      expect(send).toHaveBeenCalledTimes(3);
      expect(send.mock.calls.map(([message]) => message.fields)).toEqual([
        { name: "RAWES_RLL", value: 0.3, time_boot_ms: 0 },
        { name: "RAWES_PIT", value: 0, time_boot_ms: 0 },
        { name: "RAWES_COL", value: 0.7, time_boot_ms: 0 },
      ]);
    });

    it("reports sweep transport failures even when cleanup aborts the sweep", async () => {
      const { controller, feedServos, send } = setup();
      feedServos();
      await controller.startSwash();
      send.mockRejectedValueOnce(new Error("sweep transport failed"));
      const failure = expect(controller.startSwashSweep(0.25, 4)).rejects.toThrow("sweep transport failed");
      await vi.advanceTimersByTimeAsync(100);
      await failure;
      expect(controller.phase).toBe("idle");
    });

    it("writes all six settings in one typed batch without single-parameter writes", async () => {
      const { controller, setParameter, setParameters, params } = setup();
      params.set("H_COL_MAX", 1300);
      const expected = { min: 1400, max: 1600, cyclic: 1500 };
      expect(await controller.applySwashLimits(expected)).toEqual(expected);
      expect(setParameters).toHaveBeenCalledOnce();
      expect(setParameters.mock.calls[0]?.[0]).toEqual([
        ["H_COL_MIN", 1400], ["H_COL_MAX", 1600], ["H_CYC_MAX", 1500],
        ["SERVO1_TRIM", 1500], ["SERVO2_TRIM", 1500], ["SERVO3_TRIM", 1500],
      ].map(([name, value]) => ({ name, value, type: mavEnum(MavParamType.INT32) })));
      expect(setParameters.mock.calls[0]?.slice(1)).toEqual([15_000, 2, expect.any(AbortSignal)]);
      expect(setParameter).not.toHaveBeenCalled();
      expect(params.get("H_COL_MIN")).toBe(1400);
      expect(controller.phase).toBe("idle");
    });

    it("centers all three trims at the rounded-down midpoint of the new limits", async () => {
      const { controller, params, setParameters } = setup();
      const limits = { min: 1200, max: 1557, cyclic: 1000 };
      expect(await controller.applySwashLimits(limits)).toEqual(limits);
      expect(setParameters.mock.calls[0]?.[0].map(({ name, value }) => [name, value])).toEqual([
        ["H_COL_MIN", 1200], ["H_COL_MAX", 1557], ["H_CYC_MAX", 1000],
        ["SERVO1_TRIM", 1378], ["SERVO2_TRIM", 1378], ["SERVO3_TRIM", 1378],
      ]);
      for (let channel = 1; channel <= 3; channel++) {
        expect(params.get(`SERVO${channel}_TRIM`)).toBe(1378);
      }
    });

    it.each(["H_COL_MIN", "H_COL_MAX", "H_CYC_MAX", "SERVO1_TRIM", "SERVO2_TRIM", "SERVO3_TRIM"])(
      "rejects inconsistent final readback for %s", async (changed) => {
        const { controller, api, params, setParameters } = setup();
        vi.mocked(api.getParameter).mockImplementation(async (name) => {
          const value = params.get(name);
          if (value === undefined) throw new Error(`Unexpected parameter ${name}`);
          return {
            name, value: value + (name === changed && setParameters.mock.calls.length === 1 ? 1 : 0),
            type: mavEnum(MavParamType.REAL32),
          };
        });
        await expect(controller.applySwashLimits({ min: 1200, max: 1557, cyclic: 1000 }))
          .rejects.toThrow("partially applied");
        expect(controller.phase).toBe("idle");
      },
    );

    it("rejects a midpoint outside a servo's calibrated endpoints before writing", async () => {
      const { controller, params, setParameter, setParameters } = setup();
      params.set("SERVO2_MIN", 1400);
      await expect(controller.applySwashLimits({ min: 1200, max: 1557, cyclic: 1000 }))
        .rejects.toThrow("trim calibration");
      expect(setParameter).not.toHaveBeenCalled();
      expect(setParameters).not.toHaveBeenCalled();
    });

    it("does not claim success if the vehicle becomes armed after the batch", async () => {
      const { controller, api, status, setParameters } = setup();
      vi.mocked(api.status).mockImplementation(async () => ({
        ...status, base_mode: setParameters.mock.calls.length > 0 ? MavModeFlag.SAFETY_ARMED : "",
      }));
      await expect(controller.applySwashLimits({ min: 1200, max: 1557, cyclic: 1000 }))
        .rejects.toThrow("partially applied");
      expect(setParameters).toHaveBeenCalledOnce();
    });

    it("reports a failed trim write without claiming the settings are consistent", async () => {
      const { controller, params, setParameters } = setup();
      setParameters.mockImplementation(async (parameters) => {
        for (const { name, value } of parameters) {
          if (name === "SERVO2_TRIM") throw new Error("Trim ACK lost");
          params.set(name, value);
        }
        throw new Error("Expected failed batch");
      });
      await expect(controller.applySwashLimits({ min: 1200, max: 1557, cyclic: 1000 }))
        .rejects.toThrow("Limits and trims may be partially applied");
      expect(params.get("SERVO1_TRIM")).toBe(1378);
      expect(params.get("SERVO2_TRIM")).toBe(1517);
      expect(params.get("SERVO3_TRIM")).toBe(1517);
    });

    it("refuses armed limit edits and active-run edits without writing", async () => {
      const { controller, status, setParameter, setParameters, feedServos } = setup();
      const limits = { min: 1100, max: 1900, cyclic: 1200 };
      status.base_mode = MavModeFlag.SAFETY_ARMED;
      await expect(controller.applySwashLimits(limits)).rejects.toThrow("disarmed");
      expect(setParameter).not.toHaveBeenCalled();
      expect(setParameters).not.toHaveBeenCalled();
      status.base_mode = "";
      feedServos();
      await controller.startSwash();
      setParameter.mockClear();
      await expect(controller.applySwashLimits(limits)).rejects.toThrow("Stop");
      expect(setParameter).not.toHaveBeenCalled();
      expect(setParameters).not.toHaveBeenCalled();
    });

    it("reports uncertain or partial writes instead of claiming success", async () => {
      const { controller, setParameters } = setup();
      setParameters.mockRejectedValueOnce(new Error("ACK lost"));
      await expect(controller.applySwashLimits({ min: 1100, max: 1900, cyclic: 1200 }))
        .rejects.toThrow("partially applied");
      expect(setParameters).toHaveBeenCalledOnce();
      expect(controller.phase).toBe("idle");
    });

    it.each(["missing", "mismatched", "non-finite"])("rejects a %s batch result", async (failure) => {
      const { controller, setParameters } = setup();
      setParameters.mockImplementation(async (parameters) => new Map(
        parameters
          .filter(({ name }) => failure !== "missing" || name !== "SERVO2_TRIM")
          .map((parameter) => [parameter.name, {
            ...parameter,
            value: parameter.name === "SERVO2_TRIM"
              ? failure === "non-finite" ? NaN : parameter.value + 1
              : parameter.value,
          }]),
      ));
      await expect(controller.applySwashLimits({ min: 1200, max: 1557, cyclic: 1000 }))
        .rejects.toThrow("SERVO2_TRIM batch readback");
      expect(controller.phase).toBe("idle");
    });

    it("sweeps one bounded circle at fixed collective and finishes at center", async () => {
      const { controller, feedServos, send } = setup();
      feedServos();
      await controller.startSwash();
      const timer = setInterval(feedServos, 250);
      const sweep = controller.startSwashSweep(1, 4);
      await vi.advanceTimersByTimeAsync(4200);
      await sweep;
      clearInterval(timer);
      expect(controller.swashPosition).toEqual({ roll: 0, pitch: 0, collective: 0.5 });
      expect(controller.swashSweeping).toBe(false);
      const named = send.mock.calls.flatMap(([message]) => {
        if (!("name" in message.fields) || !("value" in message.fields)) return [];
        return [{ name: message.fields.name, value: message.fields.value }];
      });
      for (const name of ["RAWES_RLL", "RAWES_PIT"]) {
        const values = named.filter((message) => message.name === name).map((message) => Number(message.value));
        expect(Math.max(...values)).toBeGreaterThan(0.8);
        expect(Math.min(...values)).toBeLessThan(-0.8);
        expect(values.every((value) => Math.abs(value) <= 1)).toBe(true);
      }
      expect(named.filter((message) => message.name === "RAWES_COL")
        .every((message) => message.value === 0.5)).toBe(true);
    });

    it("cancels sweeping on manual movement and on Stop", async () => {
      const { controller, feedServos } = setup();
      feedServos();
      await controller.startSwash();
      const timer = setInterval(feedServos, 250);
      const sweep = controller.startSwashSweep(0.25, 4);
      await vi.advanceTimersByTimeAsync(500);
      const manual = controller.setSwashTarget({ roll: 0.1, pitch: 0, collective: 0.7 });
      await vi.advanceTimersByTimeAsync(100);
      await manual;
      await sweep;
      await vi.advanceTimersByTimeAsync(1000);
      expect(controller.swashPosition).toEqual({ roll: 0.1, pitch: 0, collective: 0.7 });
      const centering = controller.setSwashTarget({ roll: 0, pitch: 0, collective: 0.7 });
      await vi.advanceTimersByTimeAsync(100);
      await centering;
      const next = controller.startSwashSweep(0.25, 4);
      await vi.advanceTimersByTimeAsync(300);
      await controller.stop();
      await next;
      clearInterval(timer);
      expect(controller.phase).toBe("idle");
      expect(controller.swashSweeping).toBe(false);
    });
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
