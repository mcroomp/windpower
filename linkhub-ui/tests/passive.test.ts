import { describe, expect, it } from "vitest";

import luaSource from "../../scripts/rawes.lua?raw";
import {
  CMD_ENTER_GUIDED,
  CMD_ENTER_PASSIVE,
  PassiveController,
  PASSIVE_TELEMETRY_RATES,
  PASSIVE_YAW_TRIM_SEED,
  parseRunPassive,
} from "../src/passive";
import { MavResult } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";

describe("Lua command IDs", () => {
  it("match rawes.lua", () => {
    // MAV_CMD_USER_1 / MAV_CMD_USER_2 in the MAVLink dialect.
    const numericIds = { [CMD_ENTER_GUIDED]: 31010, [CMD_ENTER_PASSIVE]: 31011 };
    expect(luaSource).toMatch(
      new RegExp(`MAV_CMD_RAWES_ENTER_GUIDED = ${numericIds[CMD_ENTER_GUIDED]}\\b`),
    );
    expect(luaSource).toMatch(
      new RegExp(`MAV_CMD_RAWES_ENTER_PASSIVE = ${numericIds[CMD_ENTER_PASSIVE]}\\b`),
    );
  });
});

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

    describe("passive recapture", () => {
      it("clears keyboard offsets before recapturing the onboard direction", async () => {
        const messages: Array<{ name: string; value: number }> = [];
        const commands: Array<{ command: string; params: number[] }> = [];
        const writes: string[] = [];
        const api = {
          async sendMessage(message: { name: string; value: number }) {
            messages.push({ name: message.name, value: message.value });
          },
          async command(command: string, params: number[]) {
            commands.push({ command, params });
            return { result: mavEnum(MavResult.ACCEPTED) };
          },
        };
        const telemetry = {
          checkpoint: () => 0,
          recordsAfter: () => [],
        };
        const controller = new PassiveController(
          api as never,
          telemetry as never,
          (message) => writes.push(message),
          () => undefined,
        );
        (controller as unknown as { currentPhase: string }).currentPhase = "running";

        await controller.adjust("roll", 1);
        await controller.adjust("pitch", -1);
        await controller.adjust("yaw", 1);
        await controller.recaptureTarget();

        expect(messages.slice(-3)).toEqual([
          { name: "RAWES_ROFF", value: 0 },
          { name: "RAWES_POFF", value: 0 },
          { name: "RAWES_YOFF", value: 0 },
        ]);
        expect(commands).toEqual([
          { command: CMD_ENTER_PASSIVE, params: [PASSIVE_YAW_TRIM_SEED] },
        ]);
        expect(writes.at(-1)).toContain("recaptured");
        expect(writes.at(-1)).toContain("offsets cleared");
      });
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

    it("holds zero adaptive yaw trim during a bench passive run", () => {
      expect(PASSIVE_YAW_TRIM_SEED).toBe(0);
    });
  });

  it("rejects unsafe or unsupported inputs", () => {
    expect(() => parseRunPassive(["--trim", "thr=1.1"])).toThrow("0..1");
    expect(() => parseRunPassive(["--roll", "31"])).toThrow("-30..30");
    expect(() => parseRunPassive(["--osc", "all"])).toThrow("Unknown");
  });
});
