import { describe, expect, it, vi } from "vitest";

import { ExperimentController, formatElapsed } from "../src/experiment";
import { MavModeFlag } from "../src/generated/protocol";
import type { PassivePhase } from "../src/passive";
import type { ExperimentInfo, LinkHubStatus, MessageRecord, MotorStatus } from "../src/types";

const INFO: ExperimentInfo = {
  id: "e1", name: "Spin", description: "", run_id: "r1", started_at_ns: 0, start_cursor: "v1:1",
};

function motor(speed: number, stopping = false): MotorStatus {
  return {
    available: true, connected: true, device: "BLDC", device_id: "m1",
    running: speed > 0 && !stopping, stopping, speed_percent: speed, target_speed_percent: 0,
    direction: "forward", command_expires_in_ms: null, error: null,
  };
}

function setup(rotorRpmAfterSleeps: number[]) {
  const calls: string[] = [];
  let clock = 0;
  let sleeps = 0;
  let active: ExperimentInfo | null = INFO;
  const motorSequence = [motor(40), motor(20, true), motor(0)];
  let listener: (record: MessageRecord) => void = () => undefined;
  const feedRpm = (value: number) => listener({
    message: "NAMED_VALUE_FLOAT", direction: "rx", system_id: 1, component_id: 1,
    cursor: "v1:2", received_time: "", received_time_ns: 0, fields: { name: "SIM_RPM", value },
  });
  const api = {
    experiment: vi.fn(async () => active),
    startExperiment: vi.fn(),
    stopExperiment: vi.fn(async () => {
      calls.push("save");
      const experiment = active!;
      active = null;
      return { experiment, stopped_at_ns: 1, end_cursor: "v1:9", record_count: 5, path: "x.json" };
    }),
    motorStatus: vi.fn(async () => {
      calls.push("motor-status");
      return motorSequence.shift() ?? motor(0);
    }),
    stopMotor: vi.fn(async () => {
      calls.push("motor-stop");
      return motorSequence.shift() ?? motor(0);
    }),
    status: vi.fn(async () => ({ base_mode: MavModeFlag.SAFETY_ARMED }) as LinkHubStatus),
  };
  const passive = {
    phase: "running" as PassivePhase,
    stop: vi.fn(async () => { calls.push("safe-off"); }),
  };
  const controller = new ExperimentController(
    api,
    { onRecord: (next) => { listener = next; return () => undefined; } },
    passive,
    () => undefined,
    {
      pollMs: 100,
      motorStopTimeoutMs: 1_000,
      spinDownTimeoutMs: 1_000,
      now: () => clock,
      sleep: async (ms) => {
        clock += ms;
        const rpm = rotorRpmAfterSleeps[sleeps++];
        if (rpm !== undefined) feedRpm(rpm);
      },
    },
  );
  return { api, passive, controller, calls, feedRpm };
}

describe("experiment controller", () => {
  it("ramps the motor down, waits for the rotor to stop, then safe-offs and saves", async () => {
    // The motor poll consumes the first sleep; the rotor reports stopped after the fourth.
    const { controller, calls, feedRpm } = setup([90, 90, 40, 0.5]);
    await controller.refresh();
    feedRpm(120);
    const saved = await controller.stop();
    expect(saved.record_count).toBe(5);
    expect(calls).toEqual(["motor-status", "motor-stop", "motor-status", "safe-off", "save"]);
    expect(controller.active).toBeNull();
    expect(controller.stage).toBe("idle");
  });

  it("does not disarm or end the experiment while the rotor keeps turning", async () => {
    const { controller, api, passive, feedRpm } = setup(Array(20).fill(50));
    await controller.refresh();
    feedRpm(50);
    await expect(controller.stop()).rejects.toThrow("Rotor still turning");
    expect(passive.stop).not.toHaveBeenCalled();
    expect(api.stopExperiment).not.toHaveBeenCalled();
    expect(controller.running).toBe(true);
  });

  it("refuses gated actions without an active experiment", async () => {
    const { controller, api } = setup([]);
    api.experiment.mockResolvedValue(null);
    await expect(controller.requireActive("arm")).rejects.toThrow("arm refused: start an experiment first");
  });

  it("formats elapsed time for the stop button", () => {
    expect(formatElapsed(65_400)).toBe("01:05");
    expect(formatElapsed(3_725_000)).toBe("1:02:05");
  });
});
