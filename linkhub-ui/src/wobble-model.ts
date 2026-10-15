import type { MessageRecord } from "./types";

export const WOBBLE_MESSAGES = ["ATTITUDE_QUATERNION"] as const;
export const WOBBLE_WINDOW_S = 8;

const MAX_SAMPLES = 2048;
const MIN_SAMPLES = 32;
const MIN_COMPONENT_HZ = 0.25;

export type WobbleAxis = "roll" | "pitch" | "yaw";

export interface FrequencyComponent {
  frequencyHz: number;
  amplitude: number;
}

export interface WobbleSpectrum {
  frequenciesHz: number[];
  amplitudes: number[];
  real: number[];
  imaginary: number[];
  components: FrequencyComponent[];
  sampleRateHz: number;
  durationS: number;
}

export interface DominantWobbleMode {
  frequencyHz: number;
  amplitude: number;
  axis: Record<WobbleAxis, number>;
  contributions: Record<WobbleAxis, number>;
  linearity: number;
}

interface AxisSamples {
  times: number[];
  values: number[];
}

function numeric(value: unknown): number {
  return typeof value === "number" && Number.isFinite(value) ? value : Number.NaN;
}

function lowerBound(values: readonly number[], target: number): number {
  let low = 0;
  let high = values.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if ((values[middle] as number) < target) {
      low = middle + 1;
    } else {
      high = middle;
    }
  }
  return low;
}

function median(values: number[]): number {
  if (values.length === 0) {
    return Number.NaN;
  }
  values.sort((left, right) => left - right);
  const middle = Math.floor(values.length / 2);
  return values.length % 2 === 0
    ? ((values[middle - 1] as number) + (values[middle] as number)) / 2
    : values[middle] as number;
}

function powerOfTwoAtMost(value: number): number {
  return 2 ** Math.floor(Math.log2(Math.max(1, value)));
}

function fft(real: number[], imaginary: number[]): void {
  const size = real.length;
  for (let index = 1, reversed = 0; index < size; index += 1) {
    let bit = size >>> 1;
    for (; reversed & bit; bit >>>= 1) {
      reversed ^= bit;
    }
    reversed ^= bit;
    if (index < reversed) {
      [real[index], real[reversed]] = [real[reversed] as number, real[index] as number];
      [imaginary[index], imaginary[reversed]] = [
        imaginary[reversed] as number,
        imaginary[index] as number,
      ];
    }
  }
  for (let length = 2; length <= size; length <<= 1) {
    const angle = -2 * Math.PI / length;
    const stepReal = Math.cos(angle);
    const stepImaginary = Math.sin(angle);
    for (let offset = 0; offset < size; offset += length) {
      let twiddleReal = 1;
      let twiddleImaginary = 0;
      for (let index = 0; index < length / 2; index += 1) {
        const even = offset + index;
        const odd = even + length / 2;
        const oddReal = (real[odd] as number) * twiddleReal
          - (imaginary[odd] as number) * twiddleImaginary;
        const oddImaginary = (real[odd] as number) * twiddleImaginary
          + (imaginary[odd] as number) * twiddleReal;
        const evenReal = real[even] as number;
        const evenImaginary = imaginary[even] as number;
        real[even] = evenReal + oddReal;
        imaginary[even] = evenImaginary + oddImaginary;
        real[odd] = evenReal - oddReal;
        imaginary[odd] = evenImaginary - oddImaginary;
        const nextReal = twiddleReal * stepReal - twiddleImaginary * stepImaginary;
        twiddleImaginary = twiddleReal * stepImaginary + twiddleImaginary * stepReal;
        twiddleReal = nextReal;
      }
    }
  }
}

function interpolate(times: readonly number[], values: readonly number[], target: number): number {
  const upper = lowerBound(times, target);
  if (upper <= 0) {
    return values[0] as number;
  }
  if (upper >= times.length) {
    return values.at(-1) as number;
  }
  const t0 = times[upper - 1] as number;
  const t1 = times[upper] as number;
  const fraction = t1 === t0 ? 0 : (target - t0) / (t1 - t0);
  return (values[upper - 1] as number) * (1 - fraction) + (values[upper] as number) * fraction;
}

export function analyzeWobble(
  sourceTimes: readonly number[],
  sourceValues: readonly number[],
  windowS = WOBBLE_WINDOW_S,
): WobbleSpectrum | null {
  if (sourceTimes.length !== sourceValues.length || sourceTimes.length < MIN_SAMPLES) {
    return null;
  }
  const end = sourceTimes.at(-1) as number;
  let startIndex = lowerBound(sourceTimes, end - windowS);
  let times = sourceTimes.slice(startIndex);
  let values = sourceValues.slice(startIndex);
  const intervals = times.slice(1).map((time, index) => time - (times[index] as number))
    .filter((interval) => interval > 0 && Number.isFinite(interval));
  const typicalInterval = median(intervals);
  if (!Number.isFinite(typicalInterval) || typicalInterval <= 0) {
    return null;
  }
  for (let index = times.length - 1; index > 0; index -= 1) {
    if ((times[index] as number) - (times[index - 1] as number) > typicalInterval * 4) {
      startIndex = index;
      times = times.slice(startIndex);
      values = values.slice(startIndex);
      break;
    }
  }
  const size = Math.min(MAX_SAMPLES, powerOfTwoAtMost(times.length));
  if (size < MIN_SAMPLES) {
    return null;
  }
  const start = times[0] as number;
  const durationS = (times.at(-1) as number) - start;
  if (durationS <= 0) {
    return null;
  }
  const sampleRateHz = (size - 1) / durationS;
  const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
  const real = new Array<number>(size);
  const imaginary = new Array<number>(size).fill(0);
  let windowSum = 0;
  for (let index = 0; index < size; index += 1) {
    const time = start + index * durationS / (size - 1);
    const weight = 0.5 - 0.5 * Math.cos(2 * Math.PI * index / (size - 1));
    real[index] = (interpolate(times, values, time) - mean) * weight;
    windowSum += weight;
  }
  fft(real, imaginary);
  const frequenciesHz: number[] = [];
  const amplitudes: number[] = [];
  const normalizedReal: number[] = [];
  const normalizedImaginary: number[] = [];
  for (let bin = 1; bin <= size / 2; bin += 1) {
    frequenciesHz.push(bin * sampleRateHz / size);
    const scale = 2 / windowSum;
    const realPart = (real[bin] as number) * scale;
    const imaginaryPart = (imaginary[bin] as number) * scale;
    normalizedReal.push(realPart);
    normalizedImaginary.push(imaginaryPart);
    amplitudes.push(Math.hypot(realPart, imaginaryPart));
  }
  const candidates: FrequencyComponent[] = [];
  for (let index = 1; index < amplitudes.length - 1; index += 1) {
    const amplitude = amplitudes[index] as number;
    const frequencyHz = frequenciesHz[index] as number;
    if (
      frequencyHz >= MIN_COMPONENT_HZ
      && amplitude > (amplitudes[index - 1] as number)
      && amplitude >= (amplitudes[index + 1] as number)
    ) {
      candidates.push({ frequencyHz, amplitude });
    }
  }
  candidates.sort((left, right) => right.amplitude - left.amplitude);
  return {
    frequenciesHz,
    amplitudes,
    real: normalizedReal,
    imaginary: normalizedImaginary,
    components: candidates.slice(0, 5),
    sampleRateHz,
    durationS,
  };
}

export function analyzeDominantMode(
  spectra: Readonly<Record<WobbleAxis, WobbleSpectrum | null>>,
  maxFrequencyHz = Number.POSITIVE_INFINITY,
): DominantWobbleMode | null {
  const roll = spectra.roll;
  const pitch = spectra.pitch;
  const yaw = spectra.yaw;
  if (!roll || !pitch || !yaw) {
    return null;
  }
  const count = Math.min(
    roll.frequenciesHz.length,
    pitch.frequenciesHz.length,
    yaw.frequenciesHz.length,
  );
  let bestIndex = -1;
  let bestPower = 0;
  for (let index = 0; index < count; index += 1) {
    const frequency = roll.frequenciesHz[index] as number;
    if (frequency < MIN_COMPONENT_HZ || frequency > maxFrequencyHz) {
      continue;
    }
    const power = (roll.amplitudes[index] as number) ** 2
      + (pitch.amplitudes[index] as number) ** 2
      + (yaw.amplitudes[index] as number) ** 2;
    if (power > bestPower) {
      bestPower = power;
      bestIndex = index;
    }
  }
  if (bestIndex < 0 || bestPower <= 1e-12) {
    return null;
  }
  const real = [
    roll.real[bestIndex] as number,
    pitch.real[bestIndex] as number,
    yaw.real[bestIndex] as number,
  ];
  const imaginary = [
    roll.imaginary[bestIndex] as number,
    pitch.imaginary[bestIndex] as number,
    yaw.imaginary[bestIndex] as number,
  ];
  const covariance = Array.from({ length: 3 }, (_, row) =>
    Array.from({ length: 3 }, (_, column) =>
      (real[row] as number) * (real[column] as number)
      + (imaginary[row] as number) * (imaginary[column] as number)
    )
  );
  const diagonal = covariance.map((row, index) => row[index] as number);
  const initialAxis = diagonal.indexOf(Math.max(...diagonal));
  let vector = [0, 0, 0];
  vector[initialAxis] = 1;
  for (let iteration = 0; iteration < 16; iteration += 1) {
    const next = covariance.map((row) =>
      row.reduce((sum, value, index) => sum + value * (vector[index] as number), 0)
    );
    const norm = Math.hypot(...next);
    if (norm <= 1e-12) {
      return null;
    }
    vector = next.map((value) => value / norm);
  }
  const largestComponent = vector.reduce(
    (best, value, index) => Math.abs(value) > Math.abs(vector[best] as number) ? index : best,
    0,
  );
  if ((vector[largestComponent] as number) < 0) {
    vector = vector.map((value) => -value);
  }
  const projected = covariance.map((row) =>
    row.reduce((sum, value, index) => sum + value * (vector[index] as number), 0)
  );
  const principalPower = vector.reduce(
    (sum, value, index) => sum + value * (projected[index] as number),
    0,
  );
  const totalPower = diagonal.reduce((sum, value) => sum + value, 0);
  const secondaryPower = Math.max(0, totalPower - principalPower);
  const linearity = totalPower <= 0
    ? 0
    : Math.max(0, Math.min(1, (principalPower - secondaryPower) / totalPower));
  const squared = vector.map((value) => value ** 2);
  return {
    frequencyHz: roll.frequenciesHz[bestIndex] as number,
    amplitude: Math.sqrt(bestPower),
    axis: {
      roll: vector[0] as number,
      pitch: vector[1] as number,
      yaw: vector[2] as number,
    },
    contributions: {
      roll: squared[0] as number,
      pitch: squared[1] as number,
      yaw: squared[2] as number,
    },
    linearity,
  };
}

export class WobbleModel {
  private readonly samples: Record<WobbleAxis, AxisSamples> = {
    roll: { times: [], values: [] },
    pitch: { times: [], values: [] },
    yaw: { times: [], values: [] },
  };
  private lastBootS = 0;
  ingestedAt = 0;

  ingest(records: readonly MessageRecord[]): void {
    let accepted = false;
    for (const record of records) {
      if (record.direction !== "rx" || record.message !== "ATTITUDE_QUATERNION") {
        continue;
      }
      const time = numeric(record.fields.time_boot_ms) / 1000;
      const values: Record<WobbleAxis, number> = {
        roll: numeric(record.fields.rollspeed),
        pitch: numeric(record.fields.pitchspeed),
        yaw: numeric(record.fields.yawspeed),
      };
      if (!Number.isFinite(time) || Object.values(values).some((value) => !Number.isFinite(value))) {
        continue;
      }
      if (time < this.lastBootS) {
        this.reset();
      }
      this.lastBootS = time;
      for (const axis of Object.keys(this.samples) as WobbleAxis[]) {
        this.samples[axis].times.push(time);
        this.samples[axis].values.push(values[axis]);
      }
      accepted = true;
    }
    if (!accepted) {
      return;
    }
    this.ingestedAt = performance.now();
    const cutoff = this.lastBootS - WOBBLE_WINDOW_S - 1;
    for (const sample of Object.values(this.samples)) {
      const drop = lowerBound(sample.times, cutoff);
      sample.times.splice(0, drop);
      sample.values.splice(0, drop);
    }
  }

  spectrum(axis: WobbleAxis): WobbleSpectrum | null {
    const sample = this.samples[axis];
    return analyzeWobble(sample.times, sample.values);
  }

  dominantMode(maxFrequencyHz = Number.POSITIVE_INFINITY): DominantWobbleMode | null {
    return analyzeDominantMode({
      roll: this.spectrum("roll"),
      pitch: this.spectrum("pitch"),
      yaw: this.spectrum("yaw"),
    }, maxFrequencyHz);
  }

  reset(): void {
    for (const sample of Object.values(this.samples)) {
      sample.times.length = 0;
      sample.values.length = 0;
    }
    this.lastBootS = 0;
    this.ingestedAt = 0;
  }
}
