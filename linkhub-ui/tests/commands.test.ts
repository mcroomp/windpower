import { describe, expect, it, vi } from "vitest";

import { LinkHubApi } from "../src/api";
import { CommandRunner, tokenizeCommand } from "../src/commands";
import {
  MavCmd,
  MavModeFlag,
  MavParamType,
  MavResult,
} from "../src/generated/protocol";
import { mavEnum } from "../src/mav";
import { PassiveController } from "../src/passive";
import { TelemetryStore } from "../src/telemetry";
import type { LinkHubStatus } from "../src/types";

function sharedRunner() {
  const output: string[] = [];
  const api = new LinkHubApi();
  const telemetry = new TelemetryStore(api);
  const passive = new PassiveController(api, telemetry, () => undefined, () => undefined);
  const runner = new CommandRunner(
    api,
    telemetry,
    passive,
    (message) => output.push(message),
    { sources: () => [], targets: () => new Map() },
  );
  return { api, passive, runner, output };
}

describe("shared command runner", () => {
  it("tokenizes quoted browser terminal commands for both runtimes", () => {
    expect(tokenizeCommand('run passive --trim "thr=0.4" --duration 5')).toEqual([
      "run",
      "passive",
      "--trim",
      "thr=0.4",
      "--duration",
      "5",
    ]);
  });

  describe("core LinkHub commands", () => {
    it("reads filtered journal batches as JSON", async () => {
      const { api, runner, output } = sharedRunner();
      const query = vi.spyOn(api, "queryMessages").mockResolvedValue({
        records: [],
        next_cursor: "v1:9",
      });

      await runner.executeTokens([
        "journal", "read", "v1:4",
        "--messages", "HEARTBEAT,STATUSTEXT",
        "--direction", "rx",
        "--wait", "1.5",
        "--limit", "25",
        "--collapse",
        "--expected-generation", "v1:run:2",
      ]);

      expect(query).toHaveBeenCalledExactlyOnceWith({
        after: "v1:4",
        messages: ["HEARTBEAT", "STATUSTEXT"],
        direction: "rx",
        waitMs: 1_500,
        limit: 25,
        collapse: true,
        expectedGeneration: "v1:run:2",
      });
      expect(JSON.parse(output.join("\n"))).toMatchObject({ next_cursor: "v1:9" });
    });

    it("sends raw messages and commands", async () => {
      const { api, runner, output } = sharedRunner();
      const sendMessage = vi.spyOn(api, "sendRawMessage").mockResolvedValue("v1:10");
      await runner.executeTokens([
        "message", "send", "NAMED_VALUE_FLOAT", "{\"name\":\"TEST\",\"value\":1}",
      ]);
      expect(sendMessage).toHaveBeenCalledExactlyOnceWith(
        "NAMED_VALUE_FLOAT",
        { name: "TEST", value: 1 },
      );
      expect(output).toContain("v1:10");

      const sendCommand = vi.spyOn(api, "command").mockResolvedValue({
        command: mavEnum(MavCmd.USER_1),
        result: mavEnum(MavResult.ACCEPTED),
        progress: 0,
        status: "",
        after_cursor: "v1:11",
      });
      await runner.executeTokens([
        "command", "send", "MAV_CMD_USER_1", "1", "2.5", "--timeout", "4",
      ]);
      expect(sendCommand).toHaveBeenCalledExactlyOnceWith(
        "MAV_CMD_USER_1",
        [1, 2.5],
        4_000,
      );
    });

    it("uses confirmed arm and canonical safe-off disarm", async () => {
      const { api, passive, runner } = sharedRunner();
      const arm = vi.spyOn(api, "setArmedAndWait").mockResolvedValue();
      const stop = vi.spyOn(passive, "stop").mockResolvedValue();

      await runner.executeTokens(["arm", "--force"]);
      expect(arm).toHaveBeenCalledExactlyOnceWith(true, true);

      await runner.executeTokens(["disarm"]);
      expect(stop).toHaveBeenCalledExactlyOnceWith();
    });

    it("sets typed parameters and message rates", async () => {
      const { api, runner } = sharedRunner();
      const setParameter = vi.spyOn(api, "setParameter").mockResolvedValue({
        name: "RAWES_MODE",
        value: 0,
        type: mavEnum(MavParamType.UINT8),
      });
      await runner.executeTokens([
        "param", "set", "RAWES_MODE", "0", "--type", "MAV_PARAM_TYPE_UINT8",
      ]);
      expect(setParameter).toHaveBeenCalledExactlyOnceWith(
        "RAWES_MODE", 0, true, undefined, 3_000, "MAV_PARAM_TYPE_UINT8",
      );

      const setRates = vi.spyOn(api, "setMessageRates").mockResolvedValue({
        configured: [],
      });
      await runner.executeTokens([
        "rates", "set", "ATTITUDE=0", "RPM=5", "BATTERY_STATUS=default",
      ]);
      expect(setRates).toHaveBeenCalledExactlyOnceWith({
        ATTITUDE: 0,
        RPM: 5,
        BATTERY_STATUS: null,
      });
    });
  });

  it("renders help without a browser DOM or LinkHub connection", async () => {
    const output: string[] = [];
    const api = new LinkHubApi();
    const telemetry = new TelemetryStore(api);
    const passive = new PassiveController(api, telemetry, () => undefined, () => undefined);
    const runner = new CommandRunner(
      api,
      telemetry,
      passive,
      (message) => output.push(message),
      {
        sources: () => [],
        targets: () => new Map(),
      },
    );

    await runner.executeTokens(["help"]);

    expect(output).toContain("status");
    expect(output).toContain("stop                     apply canonical safe-off");
    expect(output).toContain("reboot                   reboot the autopilot (disarmed only)");
  });

  describe("reboot", () => {
    function makeRunner(armed: boolean, result: MavResult = MavResult.ACCEPTED) {
      const output: string[] = [];
      const api = new LinkHubApi();
      vi.spyOn(api, "status").mockResolvedValue({
        base_mode: armed ? MavModeFlag.SAFETY_ARMED : "",
      } as LinkHubStatus);
      const command = vi.spyOn(api, "command").mockResolvedValue({
        command: mavEnum(MavCmd.PREFLIGHT_REBOOT_SHUTDOWN), result: mavEnum(result),
        progress: 0, status: "", after_cursor: "v1:1",
      });
      const telemetry = new TelemetryStore(api);
      const passive = new PassiveController(api, telemetry, () => undefined, () => undefined);
      const runner = new CommandRunner(
        api, telemetry, passive, (message) => output.push(message),
        { sources: () => [], targets: () => new Map() },
      );
      return { runner, command, output };
    }

    it("sends a normal autopilot reboot and reminds the operator to restore safe-off", async () => {
      const { runner, command, output } = makeRunner(false);
      await runner.executeTokens(["reboot"]);
      expect(command).toHaveBeenCalledExactlyOnceWith(MavCmd.PREFLIGHT_REBOOT_SHUTDOWN, [1]);
      expect(output.join("\n")).toContain("Reboot accepted");
      expect(output.join("\n")).toContain("run 'stop'");
    });

    it("refuses while armed without sending anything", async () => {
      const { runner, command } = makeRunner(true);
      await expect(runner.executeTokens(["reboot"])).rejects.toThrow("ARMED");
      expect(command).not.toHaveBeenCalled();
    });

    it("reports a rejected reboot and ignores extra arguments as unknown", async () => {
      const { runner } = makeRunner(false, MavResult.FAILED);
      await expect(runner.executeTokens(["reboot"])).rejects.toThrow("Reboot rejected");
      await expect(runner.executeTokens(["reboot", "now"])).rejects.toThrow("Unknown command");
    });
  });
});
