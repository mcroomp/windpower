import { afterEach, describe, expect, it, vi } from "vitest";
import { ApiError, LinkHubApi, LinkHubGenerationChanged } from "../src/api";
import { CompassCalibration } from "../src/compass-cal";
import { CommandRunner } from "../src/commands";
import { browserConfigProvider, configTargets } from "../src/config";
import { DemoBackend } from "../src/demo-backend";
import { demoPose, ORBIT_PERIOD, ORBIT_RADIUS } from "../src/demo-flight";
import { DIAG_ARRAY_NAME, diagValue } from "../src/diag-array";
import { MavCmd, MavParamType, MavResult, MavType, PidTuningAxis } from "../src/generated/protocol";
import { enumIs, isArmed, mavEnum } from "../src/mav";
import { PassiveController, parseRunPassive } from "../src/passive";
import { TELEMETRY_PROFILES } from "../src/telemetry-rates";
import { TelemetryStore } from "../src/telemetry";
import { VehicleMotion } from "../src/vehicle-motion";
import { WobbleModel } from "../src/wobble-model";
import { WobbleRecorder } from "../src/wobble-recorder";
import { YawModel } from "../src/yaw-model";
import { YawRecorder } from "../src/yaw-recorder";

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

function fixture() {
  let time = 0;
  const backend = new DemoBackend(configTargets(true), () => time);
  const api = new LinkHubApi("", backend.fetch);
  return { backend, api, advance: (milliseconds: number) => { time += milliseconds; } };
}

describe("full UI offline backend", () => {
  it("runs the real HTTP client without ever calling global fetch", async () => {
    const network = vi.fn(() => { throw new Error("Network access forbidden"); });
    vi.stubGlobal("fetch", network);
    const { api } = fixture();
    expect((await api.serviceStatus()).service).toContain("offline demo");
    expect(isArmed(await api.status())).toBe(true);
    expect((await api.status()).custom_mode).toBe(20);
    const messages = await api.readMessages("v1:0", 0, true);
    const heartbeat = messages.records.find((record) => record.message === "HEARTBEAT");
    expect(enumIs(heartbeat?.fields.mavtype, MavType.HELICOPTER)).toBe(true);
    expect(heartbeat?.fields.type).toBeUndefined();
    const motion = new VehicleMotion();
    for (const record of messages.records) motion.accept(record, 0);
    expect(motion.position.toArray()).toEqual([0, 4.5, -ORBIT_RADIUS]);
    expect(motion.targetVisible).toBe(true);
    expect(network).not.toHaveBeenCalled();
  });

  it("generates a normalized circular flight with accurate NED velocities and quiet capture", () => {
    for (let time = 0; time < 60; time += 0.2) {
      const pose = demoPose(time);
      expect(Math.hypot(pose.north, pose.east)).toBeCloseTo(ORBIT_RADIUS, 10);
      expect(pose.attitude.length()).toBeCloseTo(1, 10);
      const next = demoPose(time + 0.0001);
      expect((next.north - pose.north) / 0.0001).toBeCloseTo(pose.vn, 4);
      expect((next.east - pose.east) / 0.0001).toBeCloseTo(pose.ve, 4);
      expect((next.down - pose.down) / 0.0001).toBeCloseTo(pose.vd, 4);
    }
    expect(demoPose(ORBIT_PERIOD / 4).east).toBeCloseTo(ORBIT_RADIUS);
    expect(demoPose(5, 0, 0, 0, true)).toMatchObject({ rollspeed: 0, pitchspeed: 0, yawspeed: 0 });
  });

  it("reads, writes and lists typed parameters and rejects unknown ones", async () => {
    const { api } = fixture();
    const value = await api.setParameter("RAWES_YAW_SLP", 0.55, false, undefined, 3000, MavParamType.REAL32);
    expect(value.value).toBe(0.55);
    expect(enumIs(value.type, MavParamType.REAL32)).toBe(true);
    expect((await api.getParameter("RAWES_YAW_SLP")).value).toBe(0.55);
    expect((await api.listParameters()).size).toBeGreaterThan(100);
    expect((await api.setParameters([value])).get(value.name)).toEqual(value);
    await expect(api.getParameter("NOT_A_PARAMETER")).rejects.toBeInstanceOf(ApiError);
    await expect(api.setParameter("NOT_A_PARAMETER", 1)).rejects.toThrow("Unknown demo parameter");
  });

  it("honors telemetry profiles, collapse, filtering, limits, and lag without losing cursor progress", async () => {
    const { api, advance } = fixture();
    await api.setMessageRates(TELEMETRY_PROFILES.radio.rates);
    const before = await api.currentCursor();
    advance(1000);
    const all = await api.queryMessages({ after: before, messages: ["ATTITUDE_QUATERNION"], direction: "rx" });
    expect(all.records).toHaveLength(10);
    const collapsed = await api.queryMessages({ after: before, messages: ["ATTITUDE_QUATERNION"], collapse: true });
    expect(collapsed.records).toHaveLength(1);
    const first = await api.queryMessages({ after: before, messages: ["ATTITUDE_QUATERNION"], limit: 3 });
    const remaining = await api.queryMessages({ after: first.next_cursor, messages: ["ATTITUDE_QUATERNION"] });
    expect(first.records.length + remaining.records.length).toBe(10);
    const fresh = await api.queryMessages({ after: before, messages: ["ATTITUDE_QUATERNION"], maxLagMs: 100 });
    expect(fresh.records).toHaveLength(2);
    await expect(api.setMessageRates({ NOT_A_MESSAGE: 1 })).rejects.toThrow("does not synthesize");
  });

  it("journal messages drive real yaw and wobble analysis with finite samples", async () => {
    const { api, advance } = fixture();
    const yaw = new YawModel();
    const wobble = new WobbleModel();
    let cursor = 0;
    for (let index = 0; index < 80; index += 1) {
      advance(100);
      const batch = await api.queryMessages({ after: `v1:${cursor}`, limit: 5000 });
      yaw.ingest(batch.records);
      wobble.ingest(batch.records);
      cursor = Number(batch.next_cursor.slice(3));
    }
    expect(yaw.att.length).toBeGreaterThan(1000);
    expect(yaw.pid.length).toBeGreaterThan(1000);
    expect(yaw.rpm.length).toBeGreaterThan(100);
    expect(yaw.diag.length).toBeGreaterThan(100);
    expect(yaw.att.v("gyro").every(Number.isFinite)).toBe(true);
    expect(yaw.pid.v("cmd").every(Number.isFinite)).toBe(true);
    expect(wobble.spectrum("roll")).not.toBeNull();
  });

  it("preserves events and command transactions when collapsing telemetry", async () => {
    const { api } = fixture();
    const initial = await api.readMessages("v1:0", 0, true);
    expect(initial.records.filter((record) => record.message === "STATUSTEXT")).toHaveLength(2);
    const after = await api.currentCursor();
    await api.command(MavCmd.DO_SET_MODE, [1, 1], { targetSystem: 1, targetComponent: 1 });
    await api.command(MavCmd.DO_SET_MODE, [1, 20]);
    const batch = await api.readMessages(after, 0, true);
    expect(batch.records.filter((record) => record.message === "COMMAND_LONG")).toHaveLength(2);
    expect(batch.records.filter((record) => record.message === "COMMAND_ACK")).toHaveLength(2);
    const command = batch.records.find((record) => record.message === "COMMAND_LONG");
    expect(command?.fields).toMatchObject({
      target_system: 1, target_component: 1, confirmation: 0,
      param1: 1, param2: 1, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0,
    });
    expect(command?.fields.params).toBeUndefined();
    const heartbeats = batch.records.filter((record) => record.message === "HEARTBEAT");
    expect(heartbeats).toHaveLength(1);
    expect(heartbeats[0]?.fields.custom_mode).toBe((await api.status()).custom_mode);
  });

  it("collapses state by the production per-message keys rather than just its name", async () => {
    const { api } = fixture();
    const after = await api.currentCursor();
    for (const [axis, desired] of [
      [PidTuningAxis.ROLL, 1], [PidTuningAxis.YAW, 2], [PidTuningAxis.ROLL, 3],
    ] as const) {
      await api.sendRawMessage("PID_TUNING", {
        axis: mavEnum(axis), desired, achieved: 0, FF: 0, P: 0, I: 0, D: 0,
      });
    }
    for (const id of [1, 2]) {
      await api.sendRawMessage("DEBUG_FLOAT_ARRAY", { array_id: id, name: "same-name", time_usec: 0, data: [id] });
    }
    const batch = await api.queryMessages({ after, collapse: true, direction: "tx" });
    expect(batch.records.filter((record) => record.message === "PID_TUNING").map((record) => record.fields.desired)).toEqual([2, 3]);
    expect(batch.records.filter((record) => record.message === "DEBUG_FLOAT_ARRAY")).toHaveLength(2);
  });

  it("feeds the unmodified production chart readers over the injected HTTP transport", async () => {
    vi.useFakeTimers({ toFake: ["setTimeout", "clearTimeout", "Date", "performance"] });
    const network = vi.fn(() => { throw new Error("Network access forbidden"); });
    vi.stubGlobal("fetch", network);
    const backend = new DemoBackend(configTargets(true));
    const api = new LinkHubApi("", backend.fetch);
    const yaw = new YawModel();
    const wobble = new WobbleModel();
    const errors: string[] = [];
    const yawRecorder = new YawRecorder(api, yaw, (error) => errors.push(error.message));
    const wobbleRecorder = new WobbleRecorder(api, wobble, (error) => errors.push(error.message));
    try {
      yawRecorder.start();
      wobbleRecorder.start();
      await vi.advanceTimersByTimeAsync(2000);
      expect(yaw.att.length).toBeGreaterThan(300);
      expect(yaw.pid.length).toBeGreaterThan(200);
      expect(wobble.spectrum("roll")).not.toBeNull();
      expect(errors).toEqual([]);
      expect(network).not.toHaveBeenCalled();
    } finally {
      yawRecorder.stop();
      wobbleRecorder.stop();
    }
  });

  it("changes fake attitude and thrust through production named-value messages", async () => {
    const { api, advance } = fixture();
    await api.sendRawMessage("NAMED_VALUE_FLOAT", { name: "RAWES_ROFF", value: 0.2 });
    await api.sendRawMessage("NAMED_VALUE_FLOAT", { name: "RAWES_THR", value: 0.5 });
    advance(100);
    const batch = await api.readMessages("v1:0", 0, true);
    const target = batch.records.find((record) => record.message === "DEBUG_FLOAT_ARRAY");
    expect(diagValue(target, "OL_RSP")).toBeGreaterThan(0.29);
    expect(diagValue(target, "OL_COL")).toBe(0.5);
    expect(target?.fields.name).toBe(DIAG_ARRAY_NAME);
    expect(batch.records.some((record) => record.direction === "tx")).toBe(true);
  });

  it("runs real terminal status and config check/apply without missing parameters", async () => {
    const { api } = fixture();
    const telemetry = new TelemetryStore(api);
    const messages: string[] = [];
    const errors: string[] = [];
    const write = (message: string, kind?: string) => {
      messages.push(message);
      if (kind === "error") errors.push(message);
    };
    const passive = new PassiveController(api, telemetry, write, () => {});
    const runner = new CommandRunner(api, telemetry, passive, write, browserConfigProvider);
    await runner.execute("status");
    expect(errors).toEqual([]);
    await runner.execute("config check --all");
    expect(messages.at(-1)).toContain("0 differ, 0 missing");
    await api.setArmed(false);
    await api.setParameter("H_COL_MIN", 1200);
    await runner.execute("config apply");
    expect((await api.getParameter("H_COL_MIN")).value).toBe(configTargets(false).get("H_COL_MIN"));
    expect(errors).toEqual([]);
  });

  it("confirms mode and arm changes, gates reboot, and protects journal generations", async () => {
    const { api } = fixture();
    expect(enumIs((await api.command(MavCmd.PREFLIGHT_REBOOT_SHUTDOWN, [1])).result, MavResult.DENIED)).toBe(true);
    await api.setArmedAndWait(false);
    await api.setModeAndWait(1);
    const status = await api.status();
    expect(isArmed(status)).toBe(false);
    await api.command(MavCmd.PREFLIGHT_REBOOT_SHUTDOWN, [1]);
    expect((await api.status()).generation).not.toBe(status.generation);
    await expect(api.queryMessages({ after: status.cursor, expectedGeneration: status.generation })).rejects.toBeInstanceOf(LinkHubGenerationChanged);
    expect(enumIs((await api.command(MavCmd.NAV_TAKEOFF)).result, MavResult.UNSUPPORTED)).toBe(true);
  });

  it("aborts long-polls instead of leaving a retry loop running", async () => {
    const { api } = fixture();
    const controller = new AbortController();
    const pending = api.readMessages(await api.currentCursor(), 1000, false, controller.signal);
    controller.abort();
    await expect(pending).rejects.toMatchObject({ name: "AbortError" });
  });

  it("runs the actual passive controller through stop, start, adjustments and safe-off", async () => {
    vi.useFakeTimers({ toFake: ["setTimeout", "clearTimeout", "Date", "performance"] });
    const backend = new DemoBackend(configTargets(true));
    const api = new LinkHubApi("", backend.fetch);
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    const errors: string[] = [];
    const passive = new PassiveController(api, telemetry, (text, kind) => {
      if (kind === "error") errors.push(text);
    }, () => {});
    try {
      await passive.reconcile();
      expect(passive.phase).toBe("running");
      await passive.stop();
      let completed = false;
      const start = passive.start(parseRunPassive([])).then(
        () => { completed = true; },
        (error: unknown) => { completed = true; errors.push(String(error)); },
      );
      for (let step = 0; step < 120 && !completed; step += 1) {
        await vi.advanceTimersByTimeAsync(100);
      }
      expect(errors).toEqual([]);
      expect(passive.phase).toBe("running");
      await start;
      await passive.adjust("roll", 1);
      await passive.adjust("collective", 1);
      await passive.recaptureTarget();
      await passive.stop();
      expect(passive.phase).toBe("idle");
      expect(isArmed(await api.status())).toBe(false);
      expect((await api.getParameter("SERVO9_FUNCTION")).value).toBe(36);
      expect((await api.getParameter("RAWES_MODE")).value).toBe(0);
      expect(errors).toEqual([]);
    } finally {
      await passive.stop();
      telemetry.stop();
    }
  });

  it("streams a complete compass calibration through the real controller", async () => {
    vi.useFakeTimers({ toFake: ["setTimeout", "clearTimeout", "Date", "performance"] });
    const backend = new DemoBackend(configTargets(true));
    const api = new LinkHubApi("", backend.fetch);
    await api.setArmed(false);
    const telemetry = new TelemetryStore(api);
    await telemetry.start();
    const messages: string[] = [];
    const compass = new CompassCalibration(api, telemetry, (message) => messages.push(message));
    try {
      let completed = false;
      let failure: unknown;
      const done = compass.start({ mask: 1, retryOnFailure: false, autosave: false, delaySeconds: 0 }).then(
        () => { completed = true; },
        (error: unknown) => { completed = true; failure = error; },
      );
      for (let step = 0; step < 100 && !completed; step += 1) {
        await vi.advanceTimersByTimeAsync(100);
      }
      expect(failure).toBeUndefined();
      expect(completed).toBe(true);
      await done;
      expect(messages.some((message) => message.includes("SUCCESS"))).toBe(true);
      await compass.accept();
      expect(messages.at(-1)).toContain("accepted and saved");
      const next = compass.start({ mask: 1, retryOnFailure: false, autosave: false, delaySeconds: 0 });
      await vi.advanceTimersByTimeAsync(100);
      await compass.cancel();
      await next;
      expect(compass.active).toBe(false);
    } finally {
      telemetry.stop();
    }
  });
});
