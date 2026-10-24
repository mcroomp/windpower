import * as THREE from "three";

export const ORBIT_PERIOD = 30;
export const ORBIT_RADIUS = 2.4;

export function demoPose(
  time: number,
  rollOffset = 0,
  pitchOffset = 0,
  yawOffset = 0,
  settled = false,
) {
  if (!Number.isFinite(time) || time < 0) {
    throw new RangeError("Demo time must be finite and nonnegative");
  }
  const phase = time * 2 * Math.PI / ORBIT_PERIOD;
  const angularSpeed = 2 * Math.PI / ORBIT_PERIOD;
  const wobble = 2 * Math.PI * 1.2 * time;
  const roll = rollOffset + (settled ? 0 : 0.1 + 0.04 * Math.sin(phase) + 0.012 * Math.sin(wobble));
  const pitch = pitchOffset + (settled ? 0 : 0.14 * Math.sin(2 * phase) + 0.008 * Math.sin(wobble));
  const yaw = yawOffset + (settled ? 0 : phase + Math.PI / 2);
  const rollRate = settled ? 0 : 0.04 * angularSpeed * Math.cos(phase)
    + 0.012 * 2 * Math.PI * 1.2 * Math.cos(wobble);
  const pitchRate = settled ? 0 : 0.28 * angularSpeed * Math.cos(2 * phase)
    + 0.008 * 2 * Math.PI * 1.2 * Math.cos(wobble);
  const yawRate = settled ? 0 : angularSpeed;
  // ArduPilot body-to-NED attitude uses ZYX, not Three.js's default XYZ.
  const attitude = new THREE.Quaternion().setFromEuler(new THREE.Euler(roll, pitch, yaw, "ZYX"));
  const target = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(roll + 0.025, pitch + 0.018, yaw + 0.035, "ZYX"),
  );
  return {
    north: ORBIT_RADIUS * Math.cos(phase),
    east: ORBIT_RADIUS * Math.sin(phase),
    down: -4.5 - 0.4 * Math.sin(2 * phase),
    vn: -ORBIT_RADIUS * angularSpeed * Math.sin(phase),
    ve: ORBIT_RADIUS * angularSpeed * Math.cos(phase),
    vd: -0.8 * angularSpeed * Math.cos(2 * phase),
    roll, pitch, yaw, attitude, target,
    rollspeed: rollRate - yawRate * Math.sin(pitch),
    pitchspeed: pitchRate * Math.cos(roll) + yawRate * Math.sin(roll) * Math.cos(pitch),
    yawspeed: -pitchRate * Math.sin(roll) + yawRate * Math.cos(roll) * Math.cos(pitch),
    rotorRpm: 42 + 4 * Math.sin(phase),
  };
}
