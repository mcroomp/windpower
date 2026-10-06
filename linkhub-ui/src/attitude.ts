import type { MessageRecord } from "./types";

export interface EulerAngles {
  /** Radians, ArduPilot's ZYX (yaw-pitch-roll) convention. */
  roll: number;
  pitch: number;
  yaw: number;
}

export function eulerFromQuaternion(
  w: number,
  x: number,
  y: number,
  z: number,
): EulerAngles | null {
  const length = Math.hypot(w, x, y, z);
  if (!Number.isFinite(length) || length < 1e-8) {
    return null;
  }
  [w, x, y, z] = [w / length, x / length, y / length, z / length];
  const sinPitch = 2 * (w * y - z * x);
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.abs(sinPitch) >= 1
      ? Math.sign(sinPitch) * Math.PI / 2
      : Math.asin(sinPitch),
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  };
}

/** Euler angles of an ATTITUDE_QUATERNION record (q1..q4 = w, x, y, z). */
export function eulerFromAttitudeQuaternion(
  record: MessageRecord | undefined,
): EulerAngles | null {
  if (!record) {
    return null;
  }
  const [w, x, y, z] = ["q1", "q2", "q3", "q4"].map((name) => record.fields[name]);
  return [w, x, y, z].every((value) => typeof value === "number")
    ? eulerFromQuaternion(w as number, x as number, y as number, z as number)
    : null;
}
