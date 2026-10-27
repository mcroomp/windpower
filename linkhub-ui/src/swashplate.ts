const CYCLIC_GAIN = 0.45;
const ROLL_FACTOR = Math.cos(-60 * Math.PI / 180 + Math.PI / 2) * CYCLIC_GAIN;
const PITCH_FACTOR_CH3 = -CYCLIC_GAIN;

export interface SwashLimits {
  min: number;
  max: number;
  cyclic: number;
}

export function swashNeutralTrim(limits: SwashLimits): number {
  validateSwashLimits(limits);
  return Math.floor((limits.min + limits.max) / 2);
}

export interface SwashTarget {
  roll: number;
  pitch: number;
  collective: number;
}

export interface SwashServo {
  min: number;
  max: number;
  trim: number;
  reversed: boolean;
}

export interface SwashGeometry {
  type: number;
  collectiveReversed: boolean;
  linear: boolean;
  servos: readonly [SwashServo, SwashServo, SwashServo];
}

interface SwashServoRange {
  min: number;
  max: number;
  clipped: boolean;
}

export function swashServoEnvelope(
  limits: SwashLimits,
  geometry: SwashGeometry,
): readonly [SwashServoRange, SwashServoRange, SwashServoRange] {
  validateSwashLimits(limits);
  if (geometry.type !== 3) throw new Error("Calculated envelope requires H_SW_TYPE=3 (H3-120).");
  const collective = [limits.min, limits.max].map((value) => {
    const scaled = (value - 1000) / 1000;
    return 2 * (geometry.collectiveReversed ? 1 - scaled : scaled) - 1;
  });
  // AP caps the cyclic vector at H_CYC_MAX/4500 before its 0.45 swash gain.
  const cyclic = 2 * CYCLIC_GAIN * limits.cyclic / 4500;
  const low = Math.min(...collective) - cyclic;
  const high = Math.max(...collective) + cyclic;
  const linearize = (value: number) => geometry.linear
    ? Math.asin(0.766044 * Math.max(-1, Math.min(1, value))) * 1.145916
    : value;
  const calculate = (servo: SwashServo): SwashServoRange => {
    if (![servo.min, servo.max, servo.trim].every(Number.isFinite)
      || servo.min >= servo.max || servo.trim < servo.min || servo.trim > servo.max) {
      throw new Error("Invalid servo endpoint or trim calibration.");
    }
    const outputs = [low, high].map((value) => servo.trim
      + (servo.reversed ? -1 : 1) * 500 * linearize(value));
    const rawMin = Math.min(...outputs);
    const rawMax = Math.max(...outputs);
    return {
      min: Math.max(servo.min, Math.min(servo.max, rawMin)),
      max: Math.max(servo.min, Math.min(servo.max, rawMax)),
      clipped: rawMin < servo.min - 1e-6 || rawMax > servo.max + 1e-6
        || (geometry.linear && (low < -1 || high > 1)),
    };
  };
  return [calculate(geometry.servos[0]), calculate(geometry.servos[1]), calculate(geometry.servos[2])];
}

export function validateSwashLimits(limits: SwashLimits): void {
  if (![limits.min, limits.max, limits.cyclic].every(Number.isInteger)
    || limits.min < 800 || limits.max > 2200 || limits.min >= limits.max
    || limits.cyclic < 0 || limits.cyclic > 4500) {
    throw new Error("Require integer collective endpoints 800 <= min < max <= 2200 us and cyclic 0..4500 centidegrees.");
  }
}

export function validateSwashTarget(target: SwashTarget): void {
  if (![target.roll, target.pitch, target.collective].every(Number.isFinite)
    || Math.hypot(target.roll, target.pitch) > 1 + 1e-9
    || target.collective < 0 || target.collective > 1) {
    throw new Error("Swash target must be finite, inside the unit cyclic circle, with collective 0..1.");
  }
}

export function swashSweepTarget(progress: number, radius: number): { roll: number; pitch: number } {
  if (!Number.isFinite(progress) || !Number.isFinite(radius) || radius <= 0 || radius > 1) {
    throw new Error("Sweep requires finite progress and a radius above 0 and at most 100%.");
  }
  const t = Math.max(0, Math.min(1, progress));
  if (t === 0 || t === 1) return { roll: 0, pitch: 0 };
  const envelope = Math.min(1, t * 6, (1 - t) * 6) * radius;
  const angle = 2 * Math.PI * Math.max(0, Math.min(1, (t - 1 / 6) * 1.5));
  return { roll: envelope * Math.cos(angle), pitch: envelope * Math.sin(angle) };
}

export interface SwashplateState {
  roll: number;
  pitch: number;
  collective: number;
}

export interface SwashControlPositions {
  cyclicLeftRight: number;
  cyclicUpDown: number;
  collectiveUpDown: number;
}

const SWASH_TRIM_US = 1517;
const SWASH_SCALE_US = 500;
const SWASH_DISPLAY_TRAVEL = 0.7;

function clampUnit(value: number): number {
  return Math.max(-1, Math.min(1, value));
}

export function swashControlPositions(
  servo1Us: number,
  servo2Us: number,
  servo3Us: number,
): SwashControlPositions | null {
  const pwm = [servo1Us, servo2Us, servo3Us];
  if (!pwm.every(Number.isFinite)) {
    return null;
  }
  const [height1, height2, height3] = pwm.map(
    (value) => (value - SWASH_TRIM_US) / SWASH_SCALE_US * SWASH_DISPLAY_TRAVEL,
  );
  if (height1 === undefined || height2 === undefined || height3 === undefined) {
    return null;
  }
  const collectiveHeight = (height1 + height2 + height3) / 3;
  const tiltLongitudinal = (height1 - height2) / (Math.sqrt(3) / 2);
  const tiltLateral = 2 * (collectiveHeight - height3);
  return {
    cyclicLeftRight: clampUnit(tiltLateral / SWASH_DISPLAY_TRAVEL),
    cyclicUpDown: clampUnit(tiltLongitudinal / SWASH_DISPLAY_TRAVEL),
    collectiveUpDown: clampUnit(
      (collectiveHeight / 4) / (SWASH_DISPLAY_TRAVEL / 4),
    ),
  };
}

export function decodeH3Swashplate(
  servo1Us: number,
  servo2Us: number,
  servo3Us: number,
  hColMin: number,
  hColMax: number,
): SwashplateState | null {
  const values = [servo1Us, servo2Us, servo3Us, hColMin, hColMax];
  if (!values.every(Number.isFinite) || hColMax <= hColMin) {
    return null;
  }

  const output = [servo1Us, servo2Us, servo3Us]
    .map((pwm) => (((pwm - 1500) / 500) + 1) / 2);
  const [output1, output2, output3] = output;
  if (output1 === undefined || output2 === undefined || output3 === undefined) {
    return null;
  }

  const collectiveScaled = (output1 + output2 + output3) / 3;
  const roll = (output1 - output2) / (2 * ROLL_FACTOR);
  const pitch = (collectiveScaled - output3) / -PITCH_FACTOR_CH3;
  const collectiveScale = (hColMax - hColMin) * 0.001;
  const collectiveOffset = (hColMin - 1000) * 0.001;
  const collective = (collectiveScaled - collectiveOffset) / collectiveScale;
  return { roll, pitch, collective };
}
