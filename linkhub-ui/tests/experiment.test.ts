import { describe, expect, it, vi } from "vitest";

import { ExperimentController, formatElapsed } from "../src/experiment";
import { MavModeFlag } from "../src/generated/protocol";
import type { PassivePhase } from "../src/passive";
import type { CameraStatus, DiagnosticBatch, ExperimentInfo, LinkHubStatus, MessageRecord, MotorStatus } from "../src/types";

const INFO: ExperimentInfo = {
  id: "e1", name: "Spin", description: "", run_id: "r1", started_at_ns: 0, start_cursor: "v1:1", directory: "x",
  telemetry_profile: "wifi",
  snapshot: {
    state: "complete", error: null, parameter_count: 900, completed_at_ns: 1,
    scripts: [{ name: "rawes.lua", bytes: 10, sha256: "ab" }],
  },
};

function withSnapshot(state: ExperimentInfo["snapshot"]["state"], error: string | null = null): ExperimentInfo {
  return { ...INFO, snapshot: { ...INFO.snapshot, state, error } };
}

function cameraStatus(devices: string[] = [], chunks = 0): CameraStatus {
  return {
    url: null, reason: null, qr_svg: null, experiment_id: INFO.id,
    phones: devices.map((stream) => ({
      stream, label: stream, recording: INFO.id, last_seen_ns: Date.now() * 1e6,
    })),
    videos: devices.map((stream) => ({
      stream: `${stream}-abcd`, file: `video-${stream}-abcd.webm`,
      bytes: chunks * 1_000, chunks, updated_at_ns: Date.now() * 1e6, finished: false,
    })),
  };
}

function motor(speed: number, stopping = false): Extract<MotorStatus, { available: true }> {
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
    startExperiment: vi.fn(async () => { active = INFO; return active; }),
    cameraStatus: vi.fn(async (): Promise<CameraStatus> => cameraStatus()),
    onboardLogEvents: vi.fn(async (after: string): Promise<DiagnosticBatch> => ({ records: [], next_cursor: after })),
    stopExperiment: vi.fn(async () => {
      calls.push("save");
      const experiment = active!;
      active = null;
      return { experiment, stopped_at_ns: 1, end_cursor: "v1:9", record_count: 5, path: "x/mavlog.json" };
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
    markExperiment: vi.fn(async (label: string) => ({ label, cursor: "v1:5", elapsed_s: 75 })),
  };
  const passive = {
    phase: "running" as PassivePhase,
    stop: vi.fn(async () => { calls.push("safe-off"); }),
  };
  const write = vi.fn();
  const controller = new ExperimentController(
    api,
    { profile: "wifi", onRecord: (next) => { listener = next; return () => undefined; } },
    passive,
    write,
    {
      pollMs: 100,
      motorStopTimeoutMs: 1_000,
      spinDownTimeoutMs: 1_000,
      recordingTimeoutMs: 1_000,
      now: () => clock,
      sleep: async (ms) => {
        clock += ms;
        const rpm = rotorRpmAfterSleeps[sleeps++];
        if (rpm !== undefined) feedRpm(rpm);
      },
    },
  );
  return { api, passive, controller, calls, feedRpm, write, advance: (ms: number) => { clock += ms; } };
}

describe("experiment controller", () => {
  it("reports onboard download start and completion after saving, ignoring other experiments", async () => {
    const { controller, api, write } = setup([]);
    await controller.stop();
    expect(controller.active).toBeNull();
    expect(write).toHaveBeenCalledWith(expect.stringContaining("Checking for new onboard logs (.BIN)"));
    const event = (sequence: number, experiment_id: string, state: string) => ({
      cursor: `v1:${sequence}`,
      data: {
        source: "linkhub", event: "experiment.onboard_logs",
        fields: { experiment_id, onboard_logs: {
          state, id: 5, percent: 50, received_bytes: 500, total_bytes: 1000,
          logs: [{ file: "onboard-5.BIN" }],
        } },
      },
    });
    api.onboardLogEvents.mockResolvedValueOnce({
      records: [event(10, "other", "complete"), event(11, INFO.id, "downloading")], next_cursor: "v1:11",
    }).mockResolvedValueOnce({
      records: [event(12, INFO.id, "progress"), event(13, INFO.id, "complete")], next_cursor: "v1:13",
    });
    await controller.refreshOnboardLogs();
    expect(api.onboardLogEvents).toHaveBeenNthCalledWith(1, "v1:9");
    expect(write).toHaveBeenCalledWith("Spin: Downloading onboard logs (.BIN)…", "info");
    await controller.refreshOnboardLogs();
    expect(api.onboardLogEvents).toHaveBeenNthCalledWith(2, "v1:11");
    expect(write).toHaveBeenCalledWith(`Spin: Downloading onboard-5.BIN: 50% (500 / ${(1000).toLocaleString()} bytes)`, "info");
    expect(write).toHaveBeenCalledWith("Spin: Onboard log downloads complete (1 .BIN file)", "info");
    expect(write).not.toHaveBeenCalledWith(expect.stringContaining("complete"), "error");
    await controller.refreshOnboardLogs();
    expect(api.onboardLogEvents).toHaveBeenCalledTimes(2);
  });

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

  it("saves without hardware commands when no motor or vehicle has connected", async () => {
    const { controller, api, passive, calls } = setup([]);
    passive.phase = "idle";
    api.motorStatus.mockResolvedValue({
      ...motor(0), connected: false, device: null, device_id: null,
    });
    api.status.mockResolvedValue({
      connected: false, ready: false, base_mode: "", received_messages: 0,
    } as LinkHubStatus);
    await controller.refresh();

    await controller.stop();

    expect(api.stopMotor).not.toHaveBeenCalled();
    expect(passive.stop).not.toHaveBeenCalled();
    expect(calls).toEqual(["save"]);
    expect(controller.active).toBeNull();
    expect(controller.stage).toBe("idle");
  });

  it("refuses gated actions without an active experiment", async () => {
    const { controller, api } = setup([]);
    api.experiment.mockResolvedValue(null);
    await expect(controller.requireActive("arm")).rejects.toThrow("arm refused: start an experiment first");
  });

  it("allows starting without cameras and reports that no video is being recorded", async () => {
    const { api, controller, write } = setup([]);
    await controller.start("Spin", "");
    expect(api.startExperiment).toHaveBeenCalledExactlyOnceWith("Spin", "", "wifi");
    expect(controller.running).toBe(true);
    await controller.refresh();
    expect(controller.videoCheck).toEqual({ state: "ready", detail: "No cameras connected" });
    expect(write).toHaveBeenCalledWith(expect.stringContaining("Experiment started: Spin"));
  });

  it("keeps actions locked until every phone has at least two saved chunks", async () => {
    const { api, controller } = setup([]);
    api.cameraStatus
      .mockResolvedValueOnce(cameraStatus(["one", "two"]))
      .mockResolvedValueOnce(cameraStatus(["one", "two"], 1))
      .mockResolvedValueOnce({
        ...cameraStatus(["one", "two"], 2),
        videos: cameraStatus(["one"], 2).videos,
      })
      .mockResolvedValue(cameraStatus(["one", "two"], 2));
    await controller.start("Spin", "");
    expect(controller.running).toBe(true);
    await controller.refresh();
    expect(controller.controlsReady).toBe(false);
    await expect(controller.requireActive("arm")).rejects.toThrow("two");
    await expect(controller.requireActive("arm")).resolves.toBeUndefined();
    expect(controller.controlsReady).toBe(true);
  });

  it("times out on heartbeat-only recording claims, keeps actions locked, and can still stop", async () => {
    const { api, controller, advance } = setup([]);
    api.cameraStatus.mockResolvedValue(cameraStatus(["one"]));
    await controller.start("Spin", "");
    await controller.refresh();
    expect(controller.videoCheck.state).toBe("pending");
    advance(1_000);
    await controller.refresh();
    expect(controller.active).toEqual(INFO);
    expect(controller.videoCheck).toMatchObject({ state: "failed", detail: expect.stringContaining("one") });
    expect(controller.running).toBe(true);
    await expect(controller.requireActive("Motor turn on")).rejects.toThrow("Motor turn on refused: one");
    await controller.stop();
    expect(api.stopExperiment).toHaveBeenCalledOnce();
    expect(controller.stage).toBe("idle");
  });

  it("does not mistake a disconnected expected phone for a camera-free experiment", async () => {
    const { api, controller } = setup([]);
    api.cameraStatus
      .mockResolvedValueOnce(cameraStatus(["one"]))
      .mockResolvedValue(cameraStatus());
    await controller.start("Spin", "");
    await expect(controller.requireActive("arm")).rejects.toThrow("one: disconnected");
  });

  it.each(["stale", "finished", "empty", "wrong-phone", "wrong-experiment"] as const)(
    "rejects %s video even when the phone claims it is recording",
    async (problem) => {
      const { api, controller } = setup([]);
      const camera = cameraStatus(["one"], 2);
      const video = camera.videos[0]!;
      if (problem === "stale") video.updated_at_ns = (Date.now() - 11_000) * 1e6;
      if (problem === "finished") video.finished = true;
      if (problem === "empty") video.bytes = 0;
      if (problem === "wrong-phone") video.stream = "someone-else-abcd";
      if (problem === "wrong-experiment") camera.experiment_id = "old";
      api.cameraStatus.mockResolvedValue(camera);
      await expect(controller.requireActive("arm")).rejects.toThrow("arm refused:");
      expect(controller.controlsReady).toBe(false);
    },
  );

  it("rechecks recording after reload, locks on a stall, and recovers when data resumes", async () => {
    const { api, controller } = setup([]);
    api.cameraStatus.mockResolvedValue(cameraStatus(["one"], 2));
    await controller.refresh();
    expect(controller.running).toBe(true);
    const stalled = cameraStatus(["one"], 2);
    stalled.videos[0]!.updated_at_ns = (Date.now() - 11_000) * 1e6;
    api.cameraStatus.mockResolvedValue(stalled);
    await controller.refresh();
    expect(controller.videoCheck.state).toBe("failed");
    expect(controller.motorLeaseExperimentId).toBe(INFO.id);
    await expect(controller.requireActive("arm")).rejects.toThrow("arm refused:");
    api.cameraStatus.mockResolvedValue(cameraStatus(["one"], 3));
    await controller.refresh();
    expect(controller.controlsReady).toBe(true);
  });

  it("fails closed when camera status cannot be checked", async () => {
    const { api, controller } = setup([]);
    await controller.refresh();
    api.cameraStatus.mockRejectedValue(new Error("network down"));
    await expect(controller.requireActive("arm")).rejects.toThrow("network down");
    expect(controller.running).toBe(true);
    expect(controller.controlsReady).toBe(false);
  });

  it("starts even if camera status is unavailable", async () => {
    const { api, controller } = setup([]);
    api.cameraStatus.mockRejectedValueOnce(new Error("network down"));
    await expect(controller.start("Spin", "")).resolves.toEqual(INFO);
    await controller.refresh();
    expect(api.startExperiment).toHaveBeenCalledOnce();
    expect(controller.videoCheck).toMatchObject({ state: "failed", detail: expect.stringContaining("network down") });
    await expect(controller.requireActive("arm")).rejects.toThrow("Initial camera check unavailable");
  });

  it("keeps actions locked until the parameter and script snapshot is saved", async () => {
    const { api, controller } = setup([]);
    api.startExperiment.mockResolvedValue(withSnapshot("capturing"));
    api.experiment.mockResolvedValue(withSnapshot("capturing"));
    await controller.start("Spin", "");
    expect(controller.running).toBe(true);
    await expect(controller.requireActive("arm")).rejects.toThrow("capturing parameter and script snapshot");
    api.experiment.mockResolvedValue(INFO);
    await expect(controller.requireActive("arm")).resolves.toBeUndefined();
    expect(controller.active?.snapshot.state).toBe("complete");
  });

  it("refuses to run when the snapshot fails and can still stop", async () => {
    const { api, controller } = setup([]);
    api.experiment.mockResolvedValue(withSnapshot("failed", "download @PARAM/param.pck: timeout"));
    await controller.start("Spin", "");
    await expect(controller.requireActive("arm")).rejects.toThrow("arm refused: parameter and script snapshot failed");
    expect(controller.running).toBe(true);
    expect(controller.active?.snapshot.error).toContain("timeout");
    await controller.stop();
    expect(api.stopExperiment).toHaveBeenCalledOnce();
  });

  it("starts and stops without waiting on video or a pending snapshot", async () => {
    const { api, controller, write } = setup([]);
    let resolveCamera!: (status: CameraStatus) => void;
    const pendingCamera = new Promise<CameraStatus>((resolve) => { resolveCamera = resolve; });
    api.cameraStatus.mockReturnValue(pendingCamera);
    api.startExperiment.mockResolvedValue(withSnapshot("capturing"));
    api.experiment.mockResolvedValueOnce(withSnapshot("capturing"));
    await expect(controller.start("Spin", "")).resolves.toEqual(withSnapshot("capturing"));
    expect(write).toHaveBeenCalledWith("Downloading parameters and scripts…");
    expect(write).toHaveBeenCalledWith(expect.stringContaining("Waiting for cameras:"));
    expect(controller.running).toBe(true);
    expect(controller.controlsReady).toBe(false);
    await controller.mark("before checks");
    expect(api.markExperiment).toHaveBeenCalledExactlyOnceWith("before checks");
    const pendingStatus = controller.refresh();
    const stop = controller.stop();
    expect(controller.running).toBe(false);
    await stop;
    resolveCamera(cameraStatus(["one"], 2));
    await pendingStatus;
    expect(controller.active).toBeNull();
    expect(controller.stage).toBe("idle");
  });

  it("marks segments only in an active experiment and reports the elapsed time", async () => {
    const { api, controller, write } = setup([]);
    await expect(controller.mark("high")).rejects.toThrow("start an experiment first");
    expect(api.markExperiment).not.toHaveBeenCalled();
    await controller.refresh();
    await controller.mark("collective high");
    expect(api.markExperiment).toHaveBeenCalledExactlyOnceWith("collective high");
    expect(write).toHaveBeenCalledWith("Mark at 01:15: collective high");
  });

  it("formats elapsed time for the stop button", () => {
    expect(formatElapsed(65_400)).toBe("01:05");
    expect(formatElapsed(3_725_000)).toBe("1:02:05");
  });
});
