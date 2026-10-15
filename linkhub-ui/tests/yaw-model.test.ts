import { describe, expect, it } from "vitest";
import { DIAG_KEYS } from "../src/diag-array";
import type { MessageRecord } from "../src/types";
import { computeMetrics, isSaturated, yawGainFromSlope, YawModel } from "../src/yaw-model";

let sequence = 0;
function record(
  message: string,
  fields: Record<string, unknown>,
  recvS: number,
): MessageRecord {
  sequence += 1;
  return {
    received_time: "t", received_time_ns: 1_000_000_000_000 + recvS * 1e9, direction: "rx",
    system_id: 1, component_id: 1, message, cursor: `v1:${sequence}`, fields,
  };
}

const armedHeartbeat = (recvS: number, armed = true, customMode = 20) => record("HEARTBEAT", {
  mavtype: { type: "MAV_TYPE_HELICOPTER" },
  autopilot: { type: "MAV_AUTOPILOT_ARDUPILOTMEGA" },
  base_mode: armed
    ? "MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"
    : "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
  custom_mode: customMode,
  system_status: { type: "MAV_STATE_ACTIVE" },
}, recvS);

const yawQ = (angle: number) => [Math.cos(angle / 2), 0, 0, Math.sin(angle / 2)];

const attitude = (bootMs: number, yaw: number, recvS: number, yawspeed = 0) => record(
  "ATTITUDE_QUATERNION",
  { time_boot_ms: bootMs, q1: yawQ(yaw)[0], q2: 0, q3: 0, q4: yawQ(yaw)[3], yawspeed },
  recvS,
);

const target = (bootMs: number, yaw: number, recvS: number, rate = 0) => record(
  "ATTITUDE_TARGET",
  { time_boot_ms: bootMs, q: yawQ(yaw), body_yaw_rate: rate, body_roll_rate: 0, body_pitch_rate: 0, thrust: 0 },
  recvS,
);

const pid = (recvS: number, values: Partial<Record<"desired" | "achieved" | "FF" | "P" | "I" | "D", number>>, axis = "PID_TUNING_YAW") =>
  record("PID_TUNING", {
    desired: 0, achieved: 0, FF: 0, P: 0, I: 0, D: 0, axis: { type: axis }, ...values,
  }, recvS);

function diag(recvS: number, usec: number, values: Partial<Record<(typeof DIAG_KEYS)[number], number>>) {
  const data = new Array<number>(DIAG_KEYS.length + 1).fill(0);
  DIAG_KEYS.forEach((key, index) => {
    const value = values[key];
    if (value !== undefined) {
      data[0] = (data[0] as number) + (1 << index);
      data[index + 1] = value;
    }
  });
  return record("DEBUG_FLOAT_ARRAY", { time_usec: usec, array_id: 1, name: "RAWES_DIAG", data }, recvS);
}

describe("YawModel", () => {
  it("builds heading, the heading the target implies and the quaternion error", () => {
    const model = new YawModel();
    model.ingest([
      target(100, 0.1, 0.1),
      attitude(110, 0.3, 0.11),
      attitude(120, 0.31, 0.12),
    ]);
    const error = model.att.v("hdgErr");
    expect(error[0]).toBeCloseTo(11.459, 2);
    expect(model.att.v("hdg")[0]).toBeCloseTo(17.188, 2);
    expect(model.att.v("hdgT")[0]).toBeCloseTo(17.188 - 11.459, 2);
  });

  it("has no target error until ATTITUDE_TARGET arrives, and again when it goes stale", () => {
    const model = new YawModel();
    model.ingest([attitude(100, 0.3, 0.1), target(110, 0.1, 0.11), attitude(120, 0.3, 0.12), attitude(2000, 0.3, 2.5)]);
    const error = model.att.v("hdgErr");
    expect(error[0]).toBeNaN();
    expect(error[1]).not.toBeNaN();
    expect(error[2]).toBeNaN();
  });

  it("keeps heading continuous across the +/-180 degree wrap", () => {
    const model = new YawModel();
    model.ingest([attitude(100, 3.1, 0.1), attitude(110, -3.1, 0.11), attitude(120, -3.0, 0.12)]);
    const heading = model.att.v("hdg");
    expect(heading[1]).toBeGreaterThan(heading[0] as number);
    expect(heading[2]).toBeGreaterThan(heading[1] as number);
  });

  it("keeps only the yaw axis of PID_TUNING and sums trim with the terms", () => {
    const model = new YawModel();
    model.ingest([
      diag(0.1, 100_000, { YFF_T: 0.25 }),
      pid(0.2, { desired: 0.5, achieved: 0.4, P: 0.05, I: 0.1, D: -0.01, FF: 0.02 }),
      pid(0.3, { desired: 1 }, "PID_TUNING_ROLL"),
    ]);
    expect(model.pid.length).toBe(1);
    expect(model.pid.v("rateErr")[0]).toBeCloseTo(0.1, 9);
    expect(model.pid.v("trim")[0]).toBe(0.25);
    expect(model.pid.v("cmd")[0]).toBeCloseTo(0.25 + 0.05 + 0.1 - 0.01 + 0.02, 9);
  });

  it("treats the trim as zero once the Lua array stops reporting it", () => {
    const model = new YawModel();
    model.ingest([diag(0.1, 100_000, { YFF_T: 0.25 }), diag(0.6, 600_000, {}), pid(0.7, { P: 0.1 })]);
    expect(model.diag.v("trim")[1]).toBeNaN();
    expect(model.pid.v("trim")[0]).toBe(0);
  });

  it("derives the observer target throttle from the Lua gain", () => {
    const model = new YawModel();
    const gain = yawGainFromSlope(0);
    model.ingest([diag(0.1, 100_000, { YFF_T: 0.3, YFF_U: 0.5, YFF_GZ: 0.2, OL_YSP: 0.7 })]);
    expect(model.diag.v("ueq")[0]).toBeCloseTo(0.5 - 0.2 / gain, 9);
    expect(model.diag.v("rateLua")[0]).toBeCloseTo(0.7, 6);
    expect(gain).toBeCloseTo(52.78, 1);
  });

  it("turns the SERVO9 readback into a throttle and ignores a disabled output", () => {
    const model = new YawModel();
    model.ingest([
      record("SERVO_OUTPUT_RAW", { time_usec: 100_000, servo9_raw: 1500 }, 0.1),
      record("SERVO_OUTPUT_RAW", { time_usec: 110_000, servo9_raw: 0 }, 0.11),
    ]);
    expect(model.servo.v("u")[0]).toBeCloseTo(0.5, 9);
    expect(model.servo.v("u")[1]).toBeNaN();
  });

  it("aligns boot-stamped streams to each other regardless of batching", () => {
    const model = new YawModel();
    // Same boot instant delivered 80 ms apart in arrival time.
    model.ingest([
      attitude(1000, 0, 5.0),
      record("SERVO_OUTPUT_RAW", { time_usec: 1_000_000, servo9_raw: 1500 }, 5.08),
    ]);
    expect(model.servo.t[0]).toBeCloseTo(model.att.t[0] as number, 4);
  });

  it("records arm, mode and important text events and ignores repeats", () => {
    const model = new YawModel();
    model.ingest([
      armedHeartbeat(1, false, 1),
      armedHeartbeat(2, true, 1),
      armedHeartbeat(3, true, 20),
      record("STATUSTEXT", { severity: { type: "MAV_SEVERITY_INFO" }, text: "EKF3 IMU0 initialised" }, 3.5),
      record("STATUSTEXT", { severity: { type: "MAV_SEVERITY_INFO" }, text: "RAWES: mode 3" }, 4),
      record("STATUSTEXT", { severity: { type: "MAV_SEVERITY_INFO" }, text: "RAWES: mode 3" }, 4.5),
      record("STATUSTEXT", { severity: { type: "MAV_SEVERITY_WARNING" }, text: "Bad thing" }, 5),
    ]);
    expect(model.events.map((event) => [event.kind, event.label])).toEqual([
      ["armed", "armed"],
      ["mode", "mode GUIDED_NOGPS"],
      ["text", "RAWES: mode 3"],
      ["text", "Bad thing"],
    ]);
    expect(model.isArmedAt(0.5)).toBe(false);
    expect(model.isArmedAt(2.5)).toBe(true);
    expect(model.armed).toBe(true);
  });

  it("forgets old samples and can be reset", () => {
    const model = new YawModel();
    model.ingest([pid(0, { P: 1 }), pid(100, { P: 2 })]);
    expect(model.pid.length).toBe(1);
    model.reset();
    expect(model.pid.length).toBe(0);
    expect(model.latestT).toBe(0);
  });
});

describe("computeMetrics", () => {
  function armedModel(): YawModel {
    const model = new YawModel();
    const records: MessageRecord[] = [armedHeartbeat(0)];
    for (let index = 0; index < 200; index += 1) {
      const t = 1 + index * 0.01;
      records.push(pid(t, { desired: 0.1, achieved: index % 2 ? 0.08 : 0.12, I: 0.3, P: 0.1 }));
    }
    model.ingest(records);
    return model;
  }

  it("computes rate-error RMS, I share and no saturation inside the limits", () => {
    const model = armedModel();
    const metrics = computeMetrics(model, 3);
    expect(metrics.armed).toBe(true);
    expect(metrics.rateErrRms).toBeCloseTo(0.02, 6);
    expect(metrics.iShare).toBeCloseTo(0.75, 6);
    expect(metrics.satPercent).toBe(0);
  });

  it("counts saturation only while armed", () => {
    const model = new YawModel();
    const records: MessageRecord[] = [armedHeartbeat(0, false)];
    for (let index = 0; index < 10; index += 1) {
      records.push(pid(1 + index * 0.01, { P: -1 }));
    }
    model.ingest(records);
    expect(computeMetrics(model, 2).satPercent).toBeNull();
    model.ingest([armedHeartbeat(3, true)]);
    model.ingest([pid(3.6, { P: -1 }), pid(3.61, { P: 0.2 })]);
    expect(computeMetrics(model, 4).satPercent).toBeCloseTo(50, 6);
  });

  it("returns nothing when no data exists", () => {
    const metrics = computeMetrics(new YawModel(), 5);
    expect(metrics).toMatchObject({
      rateErrRms: null, hdgErrRms: null, iShare: null, satPercent: null, lagMs: null,
    });
  });

  it("defines saturation as either limit of the 0..1 range", () => {
    expect([isSaturated(-0.2), isSaturated(0), isSaturated(0.4), isSaturated(1), isSaturated(1.2)])
      .toEqual([true, true, false, true, true]);
  });
});
