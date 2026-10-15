import { describe, expect, it } from "vitest";
import {
  columnBins,
  dejitter,
  estimateLag,
  interpolateAt,
  lowerBound,
  twistAboutBodyZ,
  upperBound,
  wrapPi,
  type Quat,
} from "../src/yaw-math";

function yawQuat(angle: number): Quat {
  return [Math.cos(angle / 2), 0, 0, Math.sin(angle / 2)];
}

describe("wrapPi", () => {
  it("wraps into [-pi, pi]", () => {
    expect(wrapPi(2.5 * Math.PI)).toBeCloseTo(Math.PI / 2, 9);
    expect(wrapPi(-3 * Math.PI / 2)).toBeCloseTo(Math.PI / 2, 9);
    expect(wrapPi(0.3)).toBeCloseTo(0.3, 12);
  });
});

describe("twistAboutBodyZ", () => {
  it("is the yaw difference without wrapping at 180 degrees", () => {
    const error = twistAboutBodyZ(yawQuat(-3.1), yawQuat(3.1));
    expect(error).toBeCloseTo(2 * Math.PI - 6.2, 9);
  });

  it("is signed positive when the vehicle is clockwise of the target", () => {
    expect(twistAboutBodyZ(yawQuat(0.3), yawQuat(0.1))).toBeCloseTo(0.2, 9);
    expect(twistAboutBodyZ(yawQuat(0.1), yawQuat(0.3))).toBeCloseTo(-0.2, 9);
  });

  it("treats q and -q as the same attitude", () => {
    const q = yawQuat(0.4);
    const negated: Quat = [-q[0], -q[1], -q[2], -q[3]];
    expect(twistAboutBodyZ(negated, yawQuat(0.1))).toBeCloseTo(0.3, 9);
  });

  it("ignores a tilt that both attitudes share, even with the rotor axis horizontal", () => {
    const tilt = (yaw: number): Quat => {
      // 90 degrees of pitch (rotor axis horizontal) then yaw about body z.
      const pitch: Quat = [Math.SQRT1_2, 0, Math.SQRT1_2, 0];
      const z = yawQuat(yaw);
      return [
        pitch[0] * z[0] - pitch[1] * z[1] - pitch[2] * z[2] - pitch[3] * z[3],
        pitch[0] * z[1] + pitch[1] * z[0] + pitch[2] * z[3] - pitch[3] * z[2],
        pitch[0] * z[2] - pitch[1] * z[3] + pitch[2] * z[0] + pitch[3] * z[1],
        pitch[0] * z[3] + pitch[1] * z[2] - pitch[2] * z[1] + pitch[3] * z[0],
      ];
    };
    expect(twistAboutBodyZ(tilt(0.5), tilt(0.2))).toBeCloseTo(0.3, 9);
  });
});

describe("dejitter", () => {
  it("recovers the emission times of a stream delivered in bursts", () => {
    // 100 Hz stream delivered in bursts of five: each frame arrives when the
    // burst's last frame was emitted.
    const emitted = Array.from({ length: 200 }, (_, index) => 10 + index * 0.01);
    const arrivals = emitted.map((_, index) => emitted[Math.min(199, index - (index % 5) + 4)] as number);
    const times = dejitter(arrivals);
    times.forEach((time, index) => expect(time).toBeCloseTo(emitted[index] as number, 9));
  });

  it("is never later than the arrival and is monotonic", () => {
    const arrivals = [0, 0.012, 0.019, 0.041, 0.043, 0.052, 0.07, 0.081];
    const times = dejitter(arrivals);
    times.forEach((time, index) => expect(time).toBeLessThanOrEqual((arrivals[index] as number) + 1e-12));
    for (let index = 1; index < times.length; index += 1) {
      expect(times[index] as number).toBeGreaterThan(times[index - 1] as number);
    }
  });

  it("fits each side of a long pause separately", () => {
    const first = Array.from({ length: 50 }, (_, index) => index * 0.01);
    const second = Array.from({ length: 50 }, (_, index) => 5 + index * 0.01);
    const times = dejitter([...first, ...second]);
    times.forEach((time, index) => expect(time).toBeCloseTo([...first, ...second][index] as number, 9));
  });

  it("spreads frames delayed by a link stall back over the stall", () => {
    // 100 Hz for 8 s; frames emitted during a 200 ms stall are delivered together at its end.
    const emitted = Array.from({ length: 800 }, (_, index) => 20 + index * 0.01);
    const arrivals = emitted.map((time) => (time >= 22 && time < 22.2 ? 22.2 : time));
    const times = dejitter(arrivals);
    times.forEach((time, index) => expect(Math.abs(time - (emitted[index] as number))).toBeLessThan(0.002));
    // The raw arrivals leave a hole there; the recovered times do not.
    const largestGap = Math.max(...times.slice(1).map((time, index) => time - (times[index] as number)));
    expect(largestGap).toBeLessThan(0.02);
  });

  it("leaves a steady stream alone and tolerates tiny inputs", () => {
    dejitter([0, 0.1, 0.2, 0.3]).forEach((time, index) => expect(time).toBeCloseTo(index * 0.1, 9));
    expect(dejitter([])).toEqual([]);
    expect(dejitter([4])).toEqual([4]);
  });
});

describe("search helpers", () => {
  const values = [1, 2, 2, 4];
  it("finds bounds", () => {
    expect(lowerBound(values, 2)).toBe(1);
    expect(upperBound(values, 2)).toBe(3);
    expect(lowerBound(values, 5)).toBe(4);
    expect(upperBound(values, 0)).toBe(0);
  });

  it("bins samples by time", () => {
    const bins = columnBins([0.1, 0.2, 0.6, 0.9], 0, 1, 2);
    expect(bins).toEqual([{ start: 0, end: 2 }, { start: 2, end: 4 }]);
  });

  it("interpolates and refuses to extrapolate", () => {
    expect(interpolateAt([0, 1], [0, 10], 0.25)).toBeCloseTo(2.5, 9);
    expect(interpolateAt([0, 1], [0, 10], 2)).toBeNaN();
    expect(interpolateAt([0, 1], [0, Number.NaN], 0.5)).toBeNaN();
  });
});

describe("estimateLag", () => {
  const dt = 0.01;
  const times = Array.from({ length: 1200 }, (_, index) => index * dt);
  const command = times.map((t) => Math.sin(2 * Math.PI * 0.7 * t) + 0.5 * Math.sin(2 * Math.PI * 2.3 * t));

  it("recovers a known delay", () => {
    const delay = 0.12;
    const response = times.map((t) =>
      Math.sin(2 * Math.PI * 0.7 * (t - delay)) + 0.5 * Math.sin(2 * Math.PI * 2.3 * (t - delay)));
    const lag = estimateLag({ t: times, v: command }, { t: times, v: response }, 0, 10);
    expect(lag?.lagS).toBeCloseTo(delay, 2);
    expect(lag?.correlation).toBeGreaterThan(0.95);
  });

  it("reports nothing for a flat signal or too little data", () => {
    const flat = times.map(() => 3);
    expect(estimateLag({ t: times, v: command }, { t: times, v: flat }, 0, 10)).toBeNull();
    expect(estimateLag({ t: times, v: command }, { t: times, v: command }, 0, 0.3)).toBeNull();
  });
});
