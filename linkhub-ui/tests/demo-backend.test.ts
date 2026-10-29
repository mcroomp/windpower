import { afterEach, describe, expect, it, vi } from "vitest";
import { Vector3 } from "three";
import { ApiError, LinkHubApi, LinkHubGenerationChanged } from "../src/api";
import { CommandRunner } from "../src/commands";
import { browserConfigProvider } from "../src/config";
import { DemoBackend, pumpingCycle } from "../src/demo-backend";
import recording from "../src/demo/pumping-cycle.json";
import { diagValue } from "../src/diag-array";
import { MavCmd, MavParamType, MavType } from "../src/generated/protocol";
import { enumIs, isArmed } from "../src/mav";
import { PassiveController } from "../src/passive";
import { TelemetryStore } from "../src/telemetry";
import { TELEMETRY_PROFILES } from "../src/telemetry-rates";
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
  const backend = new DemoBackend(() => time);
  const api = new LinkHubApi("", backend.fetch);
  return { backend, api, advance: (milliseconds: number) => { time += milliseconds; } };
}

describe("recorded SITL demo", () => {
  it("disables Bluetooth motor access in the offline replay", async () => {
    const { api } = fixture();
    expect(await api.motorStatus()).toEqual({ available: false });
    for (const action of [
      () => api.motorDevices(),
      () => api.connectMotor("motor-1"),
      () => api.runMotor("motor-1", 10, "forward", 2000),
      () => api.stopMotor(),
    ]) {
      await expect(action()).rejects.toMatchObject({ status: 403, message: expect.stringContaining("read-only") });
    }
  });

  it("points the rendered tether downward throughout the recorded pumping cycle", () => {
    const motion = new VehicleMotion();
    let now = 0;
    for (const frame of pumpingCycle.records) {
      if (frame.message !== "ATTITUDE_QUATERNION") continue;
      now += 2000;
      motion.accept({
        message: frame.message, fields: frame.fields, direction: "rx",
        system_id: 1, component_id: 1, cursor: "v1:1",
        received_time: "", received_time_ns: 0,
      }, now);
      const tether = new Vector3(0, -1, 0).applyQuaternion(motion.attitude);
      expect(tether.y).toBeLessThan(-0.4);
    }
  });

  it("contains exactly one complete measured pumping cycle, with honest channel provenance", () => {
    expect(recording.source.test).toBe("test_pumping_cycle_lua_sitl");
    expect(recording.source.cycle).toBe(2);
    expect(recording.source.startSimSeconds).toBeGreaterThan(recording.source.kinematicExit);
    expect(pumpingCycle.duration).toBeCloseTo(49.4, 6);
    expect(recording.phases.map((phase) => phase.name)).toEqual(["reel-out", "reel-in"]);
    expect(recording.phases[1]?.time).toBeCloseTo(24.6, 6);
    expect(pumpingCycle.records.length).toBeGreaterThan(3500);
    for (const frame of pumpingCycle.records) {
      expect(frame.time).toBeGreaterThanOrEqual(0);
      expect(frame.time).toBeLessThan(pumpingCycle.duration);
      expect(["ATTITUDE_TARGET", "PID_TUNING", "RPM"]).not.toContain(frame.message);
      if (frame.message === "ATTITUDE_QUATERNION") {
        const q = ["q1", "q2", "q3", "q4"].map((key) => Number(frame.fields[key]));
        expect(Math.hypot(...q)).toBeCloseTo(1, 8);
      }
      if (frame.message === "DEBUG_FLOAT_ARRAY") {
        expect(frame.fields.data).toHaveLength(18);
      }
    }
  });

  it("runs the production HTTP client and motion path without network access", async () => {
    const network = vi.fn(() => { throw new Error("Network access forbidden"); });
    vi.stubGlobal("fetch", network);
    const { api } = fixture();
    expect((await api.serviceStatus()).service).toContain("recorded SITL");
    expect(isArmed(await api.status())).toBe(true);
    expect((await api.status()).custom_mode).toBe(20);
    expect((await api.getParameter("RAWES_MODE")).value).toBe(1);
    expect(enumIs((await api.getParameter("RAWES_YAW_SLP")).type, MavParamType.REAL32)).toBe(true);
    expect((await api.listParameters()).size).toBeGreaterThan(1400);
    const batch = await api.readMessages("v1:0", 0, true);
    const heartbeat = batch.records.find((record) => record.message === "HEARTBEAT");
    expect(enumIs(heartbeat?.fields.mavtype, MavType.HELICOPTER)).toBe(true);
    const motion = new VehicleMotion();
    for (const record of batch.records) motion.accept(record, 0);
    motion.update(2, 2000);
    const position = pumpingCycle.records.find((record) => record.message === "LOCAL_POSITION_NED")?.fields;
    expect(motion.position.toArray()).toEqual([position?.y, -Number(position?.z), -Number(position?.x)]);
    expect(motion.rotorRpm).toBeGreaterThan(240);
    expect(motion.rpm).toBe(0);
    expect(motion.targetVisible).toBe(false);
    expect(network).not.toHaveBeenCalled();
  });

  it("loops the same samples with monotonic journal and boot clocks", async () => {
    const { api, backend, advance } = fixture();
    const initial = await api.readMessages("v1:0", 0, false);
    expect(backend.playback.phase).toBe("reel-out");
    advance(25_000);
    await api.status();
    expect(backend.playback.phase).toBe("reel-in");
    const before = await api.currentCursor();
    advance(pumpingCycle.duration * 1000 - 25_000 + 1);
    const batch = await api.queryMessages({ after: before, limit: 10_000 });
    const replayStart = batch.records.filter((record) =>
      Math.abs(record.received_time_ns / 1e9 - pumpingCycle.duration) < 1e-6);
    expect(backend.playback.phase).toBe("reel-out");
    expect(replayStart.map((record) => [record.message, record.fields.x, record.fields.q1]))
      .toEqual(initial.records.map((record) => [record.message, record.fields.x, record.fields.q1]));
    let previous = 0;
    for (const record of batch.records) {
      expect(record.received_time_ns).toBeGreaterThanOrEqual(previous);
      expect(record.sim_clock?.time_boot_ms).toBe(Math.round(record.received_time_ns / 1e6));
      if ("time_boot_ms" in record.fields) expect(record.fields.time_boot_ms).toBe(record.sim_clock?.time_boot_ms);
      previous = record.received_time_ns;
    }
    expect((await api.status()).generation).toBe("sitl-replay:1");
  });

  it("supports local profiles, journal filters, collapse and finite pagination", async () => {
    const { api, advance } = fixture();
    await api.setMessageRates({ ...TELEMETRY_PROFILES.radio.rates, ATTITUDE_QUATERNION: 2 });
    await api.setParameter("RAWES_TEL_HZ", 1);
    const before = await api.currentCursor();
    advance(4000);
    const options = { after: before, messages: ["ATTITUDE_QUATERNION"] };
    const all = await api.queryMessages(options);
    expect(all.records.length).toBeGreaterThanOrEqual(6);
    expect(all.records.length).toBeLessThanOrEqual(8);
    expect((await api.queryMessages({ ...options, collapse: true })).records).toHaveLength(1);
    const first = await api.queryMessages({ ...options, limit: 2 });
    const rest = await api.queryMessages({ ...options, after: first.next_cursor });
    expect(first.records.length + rest.records.length).toBe(all.records.length);
    expect((await api.queryMessages({ ...options, maxLagMs: 1000 })).records.length).toBeLessThan(all.records.length);
    expect((await api.queryMessages({ after: before, direction: "tx" })).records).toEqual([]);
    await expect(api.queryMessages({ after: before, expectedGeneration: "wrong" }))
      .rejects.toBeInstanceOf(LinkHubGenerationChanged);
  });

  it("preserves recorded events when collapsing and exposes recorded diagnostics", async () => {
    const { api, advance } = fixture();
    advance(10_000);
    const all = await api.queryMessages({ after: "v1:0", limit: 5000 });
    const collapsed = await api.queryMessages({ after: "v1:0", collapse: true });
    expect(collapsed.records.filter((record) => record.message === "STATUSTEXT"))
      .toEqual(all.records.filter((record) => record.message === "STATUSTEXT"));
    expect(diagValue(collapsed.records.find((record) => record.message === "DEBUG_FLOAT_ARRAY"), "OL_TEN"))
      .toBeGreaterThan(0);
  });

  it("rejects every vehicle-changing boundary without altering the recording", async () => {
    const { api } = fixture();
    const parameter = await api.getParameter("H_COL_MIN");
    const requests = [
      api.setParameter("H_COL_MIN", 1200),
      api.setParameters([{ ...parameter, value: 1200 }]),
      api.command(MavCmd.COMPONENT_ARM_DISARM, [0]),
      api.command(MavCmd.PREFLIGHT_REBOOT_SHUTDOWN, [1]),
      api.command(MavCmd.DO_START_MAG_CAL, [1]),
      api.sendRawMessage("NAMED_VALUE_FLOAT", { name: "RAWES_THR", value: 0.5 }),
    ];
    await Promise.all(requests.map((request) =>
      expect(request).rejects.toMatchObject({ status: 403, message: expect.stringContaining("read-only") })));
    expect(await api.getParameter("H_COL_MIN")).toEqual(parameter);
    expect(isArmed(await api.status())).toBe(true);
    expect((await api.status()).transmitted_messages).toBe(0);
    await expect(api.getParameter("MISSING")).rejects.toBeInstanceOf(ApiError);
    await expect(api.setMessageRates({ ATTITUDE: -1 })).rejects.toThrow("0..300");
  });

  it("keeps production terminal inspection and passive reconciliation working", async () => {
    const { api } = fixture();
    const telemetry = new TelemetryStore(api);
    const messages: string[] = [];
    const write = (message: string) => messages.push(message);
    const passive = new PassiveController(api, telemetry, write, () => {});
    const runner = new CommandRunner(api, telemetry, passive, write, browserConfigProvider);
    await passive.reconcile();
    expect(passive.phase).toBe("idle");
    await runner.execute("status");
    expect(messages.join("\n")).toContain("GUIDED_NOGPS");
    await runner.execute("config check");
    expect(messages.join("\n")).toContain("differ");
    await expect(runner.execute("param set H_COL_MIN 1200")).rejects.toThrow("read-only");
  });

  it("drives the real telemetry store and chart readers across a loop", async () => {
    vi.useFakeTimers({ toFake: ["setTimeout", "clearTimeout", "Date", "performance"] });
    const api = new LinkHubApi("", new DemoBackend().fetch);
    const telemetry = new TelemetryStore(api);
    const yaw = new YawModel();
    const wobble = new WobbleModel();
    const errors: string[] = [];
    telemetry.onWarning((message) => errors.push(message));
    const yawReader = new YawRecorder(api, yaw, (error) => errors.push(error.message));
    const wobbleReader = new WobbleRecorder(api, wobble, (error) => errors.push(error.message));
    try {
      await telemetry.start();
      await telemetry.setProfile("radio");
      yawReader.start();
      wobbleReader.start();
      await vi.advanceTimersByTimeAsync(60_000);
      expect(errors).toEqual([]);
      expect(yaw.att.length).toBeGreaterThan(400);
      expect(yaw.pid.length).toBe(0);
      expect(yaw.rpm.length).toBe(0);
      expect(yaw.diag.length).toBeGreaterThan(80);
      expect(yaw.att.v("gyro").every(Number.isFinite)).toBe(true);
      expect(yaw.att.t.at(-1)).toBeGreaterThan(59);
      expect(wobble.spectrum("roll")).not.toBeNull();
      expect(telemetry.get("NAMED_VALUE_FLOAT", "rx", "SIM_RPM")).toBeDefined();
      expect(telemetry.status?.connected).toBe(true);
    } finally {
      telemetry.stop();
      yawReader.stop();
      wobbleReader.stop();
    }
  });

  it("bounds catch-up after suspension and cancels pending reads", async () => {
    const { api, advance } = fixture();
    advance(86_400_000);
    const batch = await api.queryMessages({ after: "v1:0", limit: 60_000 });
    expect(batch.records.length).toBeLessThan(8000);
    expect(batch.records.at(-1)?.sim_clock?.time_boot_ms).toBeGreaterThan(86_399_000);
    const controller = new AbortController();
    const pending = api.readMessages(await api.currentCursor(), 1000, false, controller.signal);
    controller.abort();
    await expect(pending).rejects.toMatchObject({ name: "AbortError" });
  });
});
