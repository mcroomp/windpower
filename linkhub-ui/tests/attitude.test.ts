import { describe, expect, it } from "vitest";
import { eulerFromAttitudeQuaternion, eulerFromQuaternion } from "../src/attitude";
import type { MessageRecord } from "../src/types";

const deg = (radians: number) => radians * 180 / Math.PI;

function quaternionFromEuler(roll: number, pitch: number, yaw: number): number[] {
  const [cr, sr] = [Math.cos(roll / 2), Math.sin(roll / 2)];
  const [cp, sp] = [Math.cos(pitch / 2), Math.sin(pitch / 2)];
  const [cy, sy] = [Math.cos(yaw / 2), Math.sin(yaw / 2)];
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

describe("Euler angles from ATTITUDE_QUATERNION", () => {
  it("round-trips ZYX roll, pitch and yaw", () => {
    const [w, x, y, z] = quaternionFromEuler(0.3, -0.5, 2.0);
    const euler = eulerFromQuaternion(w!, x!, y!, z!)!;
    expect(euler.roll).toBeCloseTo(0.3, 9);
    expect(euler.pitch).toBeCloseTo(-0.5, 9);
    expect(euler.yaw).toBeCloseTo(2.0, 9);
  });

  it("matches known orientations and tolerates a non-unit quaternion", () => {
    expect(deg(eulerFromQuaternion(2, 0, 0, 0)!.yaw)).toBeCloseTo(0, 9);
    const yaw90 = eulerFromQuaternion(Math.SQRT1_2, 0, 0, Math.SQRT1_2)!;
    expect(deg(yaw90.yaw)).toBeCloseTo(90, 9);
    const roll90 = eulerFromQuaternion(Math.SQRT1_2, Math.SQRT1_2, 0, 0)!;
    expect(deg(roll90.roll)).toBeCloseTo(90, 9);
  });

  it("clamps pitch at the gimbal-lock pole", () => {
    const [w, x, y, z] = quaternionFromEuler(0, Math.PI / 2, 0);
    expect(deg(eulerFromQuaternion(w!, x!, y!, z!)!.pitch)).toBeCloseTo(90, 6);
  });

  it("rejects zero or non-numeric quaternions", () => {
    expect(eulerFromQuaternion(0, 0, 0, 0)).toBeNull();
    const record = (fields: Record<string, unknown>) => ({
      received_time: "t", received_time_ns: 1, direction: "rx", system_id: 1,
      component_id: 1, message: "ATTITUDE_QUATERNION", cursor: "v1:1", fields,
    }) as MessageRecord;
    expect(eulerFromAttitudeQuaternion(undefined)).toBeNull();
    expect(eulerFromAttitudeQuaternion(record({ q1: 1, q2: 0, q3: 0 }))).toBeNull();
    expect(eulerFromAttitudeQuaternion(record({ q1: 1, q2: 0, q3: 0, q4: 0 }))!.roll).toBe(0);
  });
});
