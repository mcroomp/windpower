const CYCLIC_GAIN = 0.45;
const ROLL_FACTOR = Math.cos(-60 * Math.PI / 180 + Math.PI / 2) * CYCLIC_GAIN;
const PITCH_FACTOR_CH3 = -CYCLIC_GAIN;

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
