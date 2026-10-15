export type Quat = readonly [number, number, number, number];

/** Wraps an angle in radians into [-pi, pi]. */
export function wrapPi(angle: number): number {
  return angle - 2 * Math.PI * Math.round(angle / (2 * Math.PI));
}

function normalize(q: Quat): Quat {
  const length = Math.hypot(q[0], q[1], q[2], q[3]);
  return length < 1e-12 ? [1, 0, 0, 0] : [q[0] / length, q[1] / length, q[2] / length, q[3] / length];
}

function conjugate(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}

function multiply(a: Quat, b: Quat): Quat {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

/**
 * Rotation about the body z axis (the rotor axis) that takes `target` to
 * `actual`, in radians (-pi, pi]. Unlike subtracting Euler yaw angles it does
 * not wrap at +/-180 degrees and stays defined when the rotor axis is horizontal.
 * Quaternions are [w, x, y, z], body to world.
 */
export function twistAboutBodyZ(actual: Quat, target: Quat): number {
  const error = multiply(conjugate(normalize(target)), normalize(actual));
  return wrapPi(2 * Math.atan2(error[3], error[0]));
}

/**
 * Removes receive-time batching from a stream that was emitted at a steady
 * rate. Each frame reaches LinkHub no earlier than it was emitted, so the
 * emission times lie on a line `c + period * index` that stays below every
 * arrival. The tightest such line (smallest total gap to the arrivals) follows
 * the frames that were delivered with the least delay, which gives the period
 * and phase without assuming a requested rate.
 *
 * A link stall delays frames but does not lose them, so the frame index stays
 * valid across it and the delayed burst is spread back over the stall. Only a
 * long arrival gap (a pause or reconnect) starts a new segment. Each segment is
 * fitted in blocks of about two seconds so a slightly wrong period or a few
 * lost frames cannot drift far.
 */
export function dejitter(arrivals: readonly number[]): number[] {
  const count = arrivals.length;
  const result = arrivals.slice();
  if (count < 3) {
    return result;
  }
  const meanPeriod = ((arrivals[count - 1] as number) - (arrivals[0] as number)) / (count - 1);
  const splitAbove = Math.max(HARD_GAP_S, 8 * meanPeriod);
  const blockFrames = Math.max(
    MIN_BLOCK_FRAMES,
    meanPeriod > 0 ? Math.round(BLOCK_S / meanPeriod) : MIN_BLOCK_FRAMES,
  );
  let segmentStart = 0;
  for (let index = 1; index <= count; index += 1) {
    if (index < count && (arrivals[index] as number) - (arrivals[index - 1] as number) <= splitAbove) {
      continue;
    }
    let start = segmentStart;
    while (start < index) {
      // Fold a short tail into the previous block rather than fitting it alone.
      const end = index - start < blockFrames * 1.5 ? index : start + blockFrames;
      dejitterSegment(arrivals, start, end, result);
      start = end;
    }
    segmentStart = index;
  }
  return result;
}

const HARD_GAP_S = 1;
const BLOCK_S = 2;
const MIN_BLOCK_FRAMES = 20;

function dejitterSegment(
  arrivals: readonly number[],
  start: number,
  end: number,
  result: number[],
): void {
  const count = end - start;
  if (count < 2) {
    return;
  }
  // Lower convex hull of (index, arrival).
  const hull: number[] = [];
  for (let index = 0; index < count; index += 1) {
    while (hull.length >= 2) {
      const a = hull[hull.length - 2] as number;
      const b = hull[hull.length - 1] as number;
      const cross = ((arrivals[start + b] as number) - (arrivals[start + a] as number)) * (index - a)
        - ((arrivals[start + index] as number) - (arrivals[start + a] as number)) * (b - a);
      if (cross >= 0) {
        hull.pop();
      } else {
        break;
      }
    }
    hull.push(index);
  }
  let totalArrival = 0;
  let totalIndex = 0;
  for (let index = 0; index < count; index += 1) {
    totalArrival += arrivals[start + index] as number;
    totalIndex += index;
  }
  let best: { period: number; offset: number } | null = null;
  let bestGap = Number.POSITIVE_INFINITY;
  for (let edge = 1; edge < hull.length; edge += 1) {
    const from = hull[edge - 1] as number;
    const to = hull[edge] as number;
    const period = ((arrivals[start + to] as number) - (arrivals[start + from] as number)) / (to - from);
    if (!(period > 0)) {
      continue;
    }
    const offset = (arrivals[start + from] as number) - period * from;
    const gap = totalArrival - count * offset - period * totalIndex;
    if (gap < bestGap) {
      bestGap = gap;
      best = { period, offset };
    }
  }
  if (best === null) {
    return;
  }
  for (let index = 0; index < count; index += 1) {
    result[start + index] = Math.min(arrivals[start + index] as number, best.offset + best.period * index);
  }
}

/** First index whose value is >= x in an ascending array. */
export function lowerBound(values: readonly number[], x: number): number {
  let low = 0;
  let high = values.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if ((values[middle] as number) < x) {
      low = middle + 1;
    } else {
      high = middle;
    }
  }
  return low;
}

/** First index whose value is > x in an ascending array. */
export function upperBound(values: readonly number[], x: number): number {
  let low = 0;
  let high = values.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if ((values[middle] as number) <= x) {
      low = middle + 1;
    } else {
      high = middle;
    }
  }
  return low;
}

export interface ColumnBin {
  /** Sample range [start, end) inside the pixel column. */
  start: number;
  end: number;
}

/** Splits the samples in [t0, t1) into `columns` equal time bins, one per pixel column. */
export function columnBins(
  times: readonly number[],
  t0: number,
  t1: number,
  columns: number,
): ColumnBin[] {
  const bins: ColumnBin[] = [];
  const width = (t1 - t0) / columns;
  let index = lowerBound(times, t0);
  for (let column = 0; column < columns; column += 1) {
    const edge = t0 + (column + 1) * width;
    const start = index;
    while (index < times.length && (times[index] as number) < edge) {
      index += 1;
    }
    bins.push({ start, end: index });
  }
  return bins;
}

/** Linearly interpolated value at x, or NaN outside the data or next to a gap. */
export function interpolateAt(
  times: readonly number[],
  values: readonly number[],
  x: number,
): number {
  const upper = lowerBound(times, x);
  if (upper === 0 || upper >= times.length) {
    return upper < times.length && times[upper] === x ? (values[upper] as number) : Number.NaN;
  }
  const t1 = times[upper] as number;
  const t0 = times[upper - 1] as number;
  const v1 = values[upper] as number;
  const v0 = values[upper - 1] as number;
  if (t1 === t0) {
    return v1;
  }
  return v0 + (v1 - v0) * ((x - t0) / (t1 - t0));
}

export interface TimedSeries {
  t: readonly number[];
  v: readonly number[];
}

export interface LagEstimate {
  lagS: number;
  correlation: number;
}

/**
 * Delay of `response` behind `command` over [t0, t1], found by maximising the
 * Pearson correlation of both resampled on a regular grid. Null when either
 * signal is flat or has too few samples to say anything.
 */
export function estimateLag(
  command: TimedSeries,
  response: TimedSeries,
  t0: number,
  t1: number,
  stepS = 0.01,
  maxLagS = 0.5,
): LagEstimate | null {
  const lagSteps = Math.round(maxLagS / stepS);
  const gridCount = Math.floor((t1 - t0) / stepS);
  const usable = gridCount - lagSteps;
  if (usable < 50) {
    return null;
  }
  const commandGrid: number[] = [];
  const responseGrid: number[] = [];
  for (let index = 0; index < gridCount; index += 1) {
    const x = t0 + index * stepS;
    commandGrid.push(interpolateAt(command.t, command.v, x));
    responseGrid.push(interpolateAt(response.t, response.v, x));
  }
  let best: LagEstimate | null = null;
  for (let lag = 0; lag <= lagSteps; lag += 1) {
    let count = 0;
    let sumA = 0;
    let sumB = 0;
    let sumAA = 0;
    let sumBB = 0;
    let sumAB = 0;
    for (let index = 0; index < usable; index += 1) {
      const a = commandGrid[index] as number;
      const b = responseGrid[index + lag] as number;
      if (!Number.isFinite(a) || !Number.isFinite(b)) {
        continue;
      }
      count += 1;
      sumA += a;
      sumB += b;
      sumAA += a * a;
      sumBB += b * b;
      sumAB += a * b;
    }
    if (count < 50) {
      continue;
    }
    const varianceA = sumAA - (sumA * sumA) / count;
    const varianceB = sumBB - (sumB * sumB) / count;
    if (varianceA < 1e-9 || varianceB < 1e-9) {
      continue;
    }
    const correlation = (sumAB - (sumA * sumB) / count) / Math.sqrt(varianceA * varianceB);
    if (best === null || correlation > best.correlation) {
      best = { lagS: lag * stepS, correlation };
    }
  }
  return best;
}
