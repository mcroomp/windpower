import * as THREE from "three";
import { describe, expect, it } from "vitest";
import type { MessageRecord } from "../src/types";
import { VehicleMotion } from "../src/vehicle-motion";

function record(message: string, fields: MessageRecord["fields"]): MessageRecord {
  return {
    message, fields, direction: "rx", system_id: 1, component_id: 1,
    cursor: "v1:1", received_time: "", received_time_ns: 0,
  };
}

function pose(q: THREE.Quaternion): MessageRecord {
  return record("ATTITUDE_QUATERNION", { q1: q.w, q2: q.x, q3: q.y, q4: q.z });
}

describe("VehicleMotion", () => {
  it("keeps the capture arrow hidden without a valid received target", () => {
    const motion = new VehicleMotion();
    expect(motion.targetVisible).toBe(false);
    motion.accept(pose(new THREE.Quaternion()), 0);
    for (const q of [[0, 0, 0, 0], [1, 0, 0], [NaN, 0, 0, 0], [1, 0, 0, Infinity]]) {
      motion.accept(record("ATTITUDE_TARGET", { q }), 0);
      expect(motion.targetVisible).toBe(false);
    }
    motion.accept({
      ...record("ATTITUDE_TARGET", { q: [1, 0, 0, 0] }), direction: "tx",
    }, 0);
    expect(motion.targetVisible).toBe(false);
    motion.accept(record("ATTITUDE_TARGET", { q: [1, 0, 0, 0] }), 0);
    expect(motion.targetVisible).toBe(true);
    motion.reset();
    expect(motion.targetVisible).toBe(false);
  });

  it("places the target arrow in front of the axle and points up the target rotor axis", () => {
    const motion = new VehicleMotion();
    motion.accept(pose(new THREE.Quaternion()), 0);
    motion.accept(record("ATTITUDE_TARGET", { q: [1, 0, 0, 0] }), 0);
    const origin = new THREE.Vector3();
    const direction = new THREE.Vector3();
    motion.targetArrowPose(origin, direction);
    expect(origin.distanceTo(new THREE.Vector3(0, 5.6, -0.75))).toBeLessThan(1e-10);
    expect(direction.distanceTo(new THREE.Vector3(0, 1, 0))).toBeLessThan(1e-10);

    motion.accept(pose(new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(1, 0, 0), Math.PI / 2,
    )), 2_000);
    motion.accept(record("LOCAL_POSITION_NED", { x: 2, y: 3, z: -4 }), 2_000);
    motion.targetArrowPose(origin, direction);
    expect(origin.distanceTo(new THREE.Vector3(3.6, 4, -2.75))).toBeLessThan(1e-10);
    expect(direction.distanceTo(new THREE.Vector3(0, 1, 0))).toBeLessThan(1e-10);

    motion.accept(record("ATTITUDE_TARGET", {
      q: [Math.SQRT1_2, Math.SQRT1_2, 0, 0],
    }), 2_000);
    motion.targetArrowPose(origin, direction);
    expect(direction.distanceTo(new THREE.Vector3(1, 0, 0))).toBeLessThan(1e-10);
  });

  it("reports target upper-axis direction error in the current hub frame", () => {
    const motion = new VehicleMotion();
    motion.accept(pose(new THREE.Quaternion()), 0);
    motion.accept(record("ATTITUDE_TARGET", { q: [1, 0, 0, 0] }), 0);
    expect(motion.targetDirectionError()).toEqual({
      forwardDegrees: 0,
      rightDegrees: 0,
      totalDegrees: 0,
    });

    const angle = 10 * Math.PI / 180;
    motion.accept(record("ATTITUDE_TARGET", {
      q: [Math.cos(angle / 2), Math.sin(angle / 2), 0, 0],
    }), 100);
    motion.update(1, 1_100);
    const error = motion.targetDirectionError();
    expect(error).not.toBeNull();
    expect(error?.forwardDegrees).toBeCloseTo(0, 8);
    expect(error?.rightDegrees).toBeCloseTo(10, 8);
    expect(error?.totalDegrees).toBeCloseTo(10, 8);
  });

  it("maps NED position and snaps the first observation without flying from the origin", () => {
    const motion = new VehicleMotion();
    motion.accept(record("LOCAL_POSITION_NED", { x: 2, y: 3, z: -4 }), 0);
    expect(motion.position.toArray()).toEqual([3, 4, -2]);
  });

  it("lerps position without overshooting and holds the last position on silence", () => {
    const motion = new VehicleMotion();
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 0, z: 0 }), 0);
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 10, z: 0 }), 100);
    motion.update(0.08, 180);
    expect(motion.position.x).toBeCloseTo(10 * (1 - Math.exp(-1)), 10);
    motion.update(1, 1_180);
    expect(motion.position.x).toBeLessThanOrEqual(10);
    expect(motion.position.x).toBeCloseTo(10, 4);
  });

  it.each([30, 60, 144])("has the same smoothing response at %i fps", (fps) => {
    const motion = new VehicleMotion();
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 0, z: 0 }), 0);
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 10, z: 0 }), 100);
    motion.accept(pose(new THREE.Quaternion()), 0);
    motion.accept(pose(new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1), Math.PI / 2,
    )), 100);
    const destination = new VehicleMotion();
    destination.accept(pose(new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1), Math.PI / 2,
    )), 0);
    for (let frame = 1; frame <= fps; frame += 1) {
      motion.update(1 / fps, 100 + frame / fps * 1_000);
    }
    expect(motion.position.x).toBeCloseTo(10 * (1 - Math.exp(-1 / 0.08)), 9);
    expect(motion.attitude.angleTo(destination.attitude))
      .toBeCloseTo(Math.PI / 2 * Math.exp(-1 / 0.08), 6);
  });

  it("preserves the existing FRD/model rotation convention", () => {
    const raw = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.4, -0.7, 1.2));
    const motion = new VehicleMotion();
    motion.accept(pose(raw), 0);
    const expected = new THREE.Matrix4().set(
      0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1,
    ).multiply(new THREE.Matrix4().makeRotationFromQuaternion(raw)).multiply(
      new THREE.Matrix4().set(
        1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1,
      ),
    );
    expect(motion.attitude.angleTo(new THREE.Quaternion().setFromRotationMatrix(expected)))
      .toBeLessThan(1e-7);
  });

  it("uses the short quaternion arc across the yaw wrap", () => {
    const motion = new VehicleMotion();
    const axis = new THREE.Vector3(0, 0, 1);
    motion.accept(pose(new THREE.Quaternion().setFromAxisAngle(axis, 179 * Math.PI / 180)), 0);
    const start = motion.attitude.clone();
    motion.accept(pose(new THREE.Quaternion().setFromAxisAngle(axis, -179 * Math.PI / 180)), 100);
    motion.update(0.08, 180);
    expect(start.angleTo(motion.attitude)).toBeLessThan(2 * Math.PI / 180);
    expect(start.angleTo(motion.attitude)).toBeGreaterThan(0);
  });

  it("normalizes incoming quaternions and rejects zero or invalid samples", () => {
    const motion = new VehicleMotion();
    motion.accept(record("ATTITUDE_QUATERNION", { q1: 2, q2: 0, q3: 0, q4: 0 }), 0);
    const previous = motion.attitude.clone();
    for (const q1 of [0, NaN, Infinity]) {
      motion.accept(record("ATTITUDE_QUATERNION", { q1, q2: 0, q3: 0, q4: 0 }), 100);
      motion.update(0.1, 200);
      expect(motion.attitude.angleTo(previous)).toBeLessThan(1e-7);
    }
    expect(motion.attitude.length()).toBeCloseTo(1);
    motion.accept(record("LOCAL_POSITION_NED", { x: 3, y: NaN, z: -4 }), 200);
    expect(motion.position.toArray()).toEqual([0, 5, 0]);
  });

  it("ignores transmitted records", () => {
    const motion = new VehicleMotion();
    motion.accept({
      ...record("LOCAL_POSITION_NED", { x: 1, y: 2, z: 3 }), direction: "tx",
    }, 0);
    expect(motion.position.toArray()).toEqual([0, 5, 0]);
  });

  it("snaps observations after a long gap instead of interpolating across it", () => {
    const motion = new VehicleMotion();
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 0, z: 0 }), 0);
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 100, z: 0 }), 2_000);
    expect(motion.position.x).toBe(100);
  });

  it("resets interpolation, rotor speed, and target visibility across generations", () => {
    const motion = new VehicleMotion();
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 10, z: 0 }), 0);
    motion.accept(record("ATTITUDE_TARGET", { q: [1, 0, 0, 0] }), 0);
    motion.accept(record("RPM", { rpm1: 1_000, rpm2: -1 }), 0);
    motion.update(0.1, 100);
    expect(motion.rpm).toBeGreaterThan(0);
    expect(new THREE.Vector3(0, 0, 1).applyQuaternion(motion.target).toArray())
      .toEqual([0, -1, 0]);
    motion.reset();
    expect(motion.rpm).toBe(0);
    expect(motion.targetVisible).toBe(false);
    motion.accept(record("LOCAL_POSITION_NED", { x: 0, y: 100, z: 0 }), 200);
    expect(motion.position.x).toBe(100);
  });

  it("holds the last measured RPM through telemetry gaps", () => {
    const motion = new VehicleMotion();
    motion.accept(record("RPM", { rpm1: 1_000, rpm2: -1 }), 0);
    motion.update(0.1, 100);
    motion.update(1, 2_000);
    expect(motion.rpm).toBeCloseTo(1_000);
  });

  it("accepts measured RPM1 and rejects invalid RPM samples", () => {
    const motion = new VehicleMotion();
    motion.accept(record("RPM", { rpm1: 600, rpm2: -1 }), 0);
    motion.update(0.08, 80);
    expect(motion.rpm).toBeCloseTo(600 * (1 - Math.exp(-1)));

    motion.accept(record("RPM", { rpm1: -1, rpm2: -1 }), 100);
    motion.accept(record("RPM", { rpm1: NaN, rpm2: -1 }), 100);
    motion.update(0.08, 180);
    expect(motion.rpm).toBeGreaterThan(0);
  });
});
