import * as THREE from "three";
import type { MessageRecord } from "./types";

const WORLD_FROM_NED = new THREE.Quaternion().setFromRotationMatrix(
  new THREE.Matrix4().set(
    0, 1, 0, 0,
    0, 0, -1, 0,
    -1, 0, 0, 0,
    0, 0, 0, 1,
  ),
);
const BODY_FROM_MODEL = new THREE.Quaternion().setFromRotationMatrix(
  new THREE.Matrix4().set(
    1, 0, 0, 0,
    0, 0, -1, 0,
    0, 1, 0, 0,
    0, 0, 0, 1,
  ),
);

export function numberField(record: MessageRecord, name: string): number | null {
  const value = record.fields[name];
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function readQuaternion(record: MessageRecord, output: THREE.Quaternion): boolean {
  const q = record.fields.q;
  const components = record.message === "ATTITUDE_TARGET"
    ? (Array.isArray(q) && q.length === 4 ? q : [])
    : ["q1", "q2", "q3", "q4"].map((name) => numberField(record, name));
  const [w, x, y, z] = components;
  if (typeof w !== "number" || typeof x !== "number"
    || typeof y !== "number" || typeof z !== "number"
    || ![w, x, y, z].every(Number.isFinite)) {
    return false;
  }
  const length = Math.hypot(w, x, y, z);
  if (length < 1e-8) {
    return false;
  }
  output.set(x / length, y / length, z / length, w / length);
  return true;
}

// A time-based blend gives the same response at 30, 60, and 144 Hz.
const SMOOTHING_SECONDS = 0.08;
const STALE_AFTER_MS = 1_000;
export const MOTOR_TO_ROTOR_GEAR_RATIO = 10;
export interface TargetDirectionError {
  forwardDegrees: number;
  rightDegrees: number;
  totalDegrees: number;
}

export class VehicleMotion {
  readonly position = new THREE.Vector3(0, 5, 0);
  readonly attitude = new THREE.Quaternion();
  readonly target = new THREE.Quaternion();
  targetVisible = false;
  rpm = 0;

  private readonly desiredPosition = new THREE.Vector3();
  private readonly desiredAttitude = new THREE.Quaternion();
  private readonly desiredTarget = new THREE.Quaternion();
  private readonly actualBodyToNed = new THREE.Quaternion();
  private readonly targetBodyToNed = new THREE.Quaternion();
  private hasPosition = false;
  private hasAttitude = false;
  private positionTime = -Infinity;
  private attitudeTime = -Infinity;
  private targetTime = -Infinity;
  private desiredRpm = 0;

  reset(): void {
    this.hasPosition = false;
    this.hasAttitude = false;
    this.targetVisible = false;
    this.desiredRpm = 0;
    this.rpm = 0;
  }

  targetArrowPose(origin: THREE.Vector3, direction: THREE.Vector3): void {
    origin.set(0.75, -0.6, 0).applyQuaternion(this.attitude).add(this.position);
    // Model -Y maps to FRD -Z, the upper rotor-axis direction.
    direction.set(0, 0, -1).applyQuaternion(this.target);
  }

  targetDirectionError(): TargetDirectionError | null {
    if (!this.hasAttitude || !this.targetVisible) {
      return null;
    }
    const targetAxis = new THREE.Vector3(0, 0, -1)
      .applyQuaternion(this.targetBodyToNed)
      .applyQuaternion(this.actualBodyToNed.clone().invert());
    return {
      forwardDegrees: Math.atan2(targetAxis.x, -targetAxis.z) * 180 / Math.PI,
      rightDegrees: Math.atan2(targetAxis.y, -targetAxis.z) * 180 / Math.PI,
      totalDegrees: Math.acos(THREE.MathUtils.clamp(-targetAxis.z, -1, 1)) * 180 / Math.PI,
    };
  }

  accept(record: MessageRecord, now: number): void {
    if (record.direction !== "rx") {
      return;
    }
    switch (record.message) {
      case "ATTITUDE_QUATERNION":
        if (readQuaternion(record, this.actualBodyToNed)) {
          this.desiredAttitude.copy(this.actualBodyToNed)
            .premultiply(WORLD_FROM_NED)
            .multiply(BODY_FROM_MODEL);
          if (!this.hasAttitude || now - this.attitudeTime > STALE_AFTER_MS) {
            this.attitude.copy(this.desiredAttitude);
          }
          this.hasAttitude = true;
          this.attitudeTime = now;
        }
        break;
      case "LOCAL_POSITION_NED": {
        const north = numberField(record, "x");
        const east = numberField(record, "y");
        const down = numberField(record, "z");
        if (north !== null && east !== null && down !== null) {
          this.desiredPosition.set(east, -down, -north);
          if (!this.hasPosition || now - this.positionTime > STALE_AFTER_MS) {
            this.position.copy(this.desiredPosition);
          }
          this.hasPosition = true;
          this.positionTime = now;
        }
        break;
      }
      case "ATTITUDE_TARGET":
        if (readQuaternion(record, this.targetBodyToNed)) {
          this.desiredTarget.copy(this.targetBodyToNed).premultiply(WORLD_FROM_NED);
          if (!this.targetVisible || now - this.targetTime > STALE_AFTER_MS) {
            this.target.copy(this.desiredTarget);
          }
          this.targetVisible = true;
          this.targetTime = now;
        }
        break;
      case "RPM": {
        const rpm = numberField(record, "rpm1");
        if (rpm !== null && rpm >= 0) {
          this.desiredRpm = rpm;
        }
        break;
      }
    }
  }

  update(elapsed: number, _now: number): void {
    const blend = -Math.expm1(-Math.max(0, elapsed) / SMOOTHING_SECONDS);
    if (this.hasPosition) {
      this.position.lerp(this.desiredPosition, blend);
    }
    if (this.hasAttitude) {
      this.attitude.slerp(this.desiredAttitude, blend);
    }
    if (this.targetVisible) {
      this.target.slerp(this.desiredTarget, blend);
    }
    this.rpm += (this.desiredRpm - this.rpm) * blend;
    if (this.rpm < 0.01) {
      this.rpm = 0;
    }
  }
}
