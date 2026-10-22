import { describe, expect, it } from "vitest";

import { decodeH3Swashplate, swashControlPositions, swashServoEnvelope, swashSweepTarget, validateSwashLimits, type SwashGeometry } from "../src/swashplate";

describe("calculated H3-120 servo envelope", () => {
  function geometry(): SwashGeometry {
    const servo = { min: 1000, max: 2000, trim: 1500, reversed: false };
    return {
      type: 3, collectiveReversed: false, linear: false,
      servos: [{ ...servo }, { ...servo }, { ...servo }],
    };
  }

  it("includes full circular cyclic travel at both collective endpoints", () => {
    const ranges = swashServoEnvelope({ min: 1342, max: 1657, cyclic: 1000 }, geometry());
    for (const range of ranges) {
      expect(range.min).toBeCloseTo(1242, 8);
      expect(range.max).toBeCloseTo(1757, 8);
      expect(range.clipped).toBe(false);
    }
  });

  it("applies collective direction and each servo's trim and reversal", () => {
    const config = geometry();
    config.collectiveReversed = true;
    config.servos[0].trim = 1517;
    config.servos[0].reversed = true;
    config.servos[2].trim = 1520;
    const ranges = swashServoEnvelope({ min: 1342, max: 1657, cyclic: 1000 }, config);
    for (const [range, min, max] of [
      [ranges[0], 1259, 1774], [ranges[1], 1243, 1758], [ranges[2], 1263, 1778],
    ] as const) {
      expect(range.min).toBeCloseTo(min, 8);
      expect(range.max).toBeCloseTo(max, 8);
    }
  });

  it("shows clipping rather than predicting motion beyond servo endpoints", () => {
    for (const linear of [false, true]) {
      const config = geometry();
      config.linear = linear;
      for (const range of swashServoEnvelope({ min: 1000, max: 2000, cyclic: 4500 }, config)) {
        expect(range.min).toBeCloseTo(1000, 2);
        expect(range.max).toBeCloseTo(2000, 2);
        expect(range.clipped).toBe(true);
      }
    }
  });

  it("uses ArduPilot's nonlinear servo throw correction when enabled", () => {
    const config = geometry();
    config.linear = true;
    const [range] = swashServoEnvelope({ min: 1400, max: 1600, cyclic: 0 }, config);
    expect(range.min).toBeGreaterThan(1410);
    expect(range.min).toBeLessThan(1420);
    expect(range.max).toBeCloseTo(3000 - range.min, 8);
    expect(range.clipped).toBe(false);
  });

  it("rejects unsupported or invalid calibration instead of supplying defaults", () => {
    const config = geometry();
    config.type = 0;
    expect(() => swashServoEnvelope({ min: 1000, max: 2000, cyclic: 1000 }, config)).toThrow("H3-120");
    config.type = 3;
    config.servos[1].trim = NaN;
    expect(() => swashServoEnvelope({ min: 1000, max: 2000, cyclic: 1000 }, config)).toThrow("trim");
    expect(() => swashServoEnvelope({ min: NaN, max: 2000, cyclic: 1000 }, geometry())).toThrow();
  });
});

describe("swash calibration bounds", () => {
  it("rejects inverted, non-finite, fractional, or out-of-range limits", () => {
    for (const limits of [
      { min: 1600, max: 1500, cyclic: 1000 },
      { min: 799, max: 2000, cyclic: 1000 },
      { min: 1000, max: 2201, cyclic: 1000 },
      { min: 1000.5, max: 2000, cyclic: 1000 },
      { min: 1000, max: 2000, cyclic: NaN },
      { min: 1000, max: 2000, cyclic: 4501 },
    ]) expect(() => validateSwashLimits(limits)).toThrow();
  });

  it("ramps in and out and stays inside the selected cyclic radius", () => {
    expect(swashSweepTarget(0, 0.5)).toEqual({ roll: 0, pitch: 0 });
    expect(swashSweepTarget(1, 0.5)).toEqual({ roll: 0, pitch: 0 });
    for (let index = 0; index <= 100; index++) {
      const point = swashSweepTarget(index / 100, 0.5);
      expect(Math.hypot(point.roll, point.pitch)).toBeLessThanOrEqual(0.5 + 1e-10);
    }
    expect(() => swashSweepTarget(0.5, 1.01)).toThrow();
  });
});

describe("decodeH3Swashplate", () => {
  it("decodes neutral outputs with default collective limits", () => {
    expect(decodeH3Swashplate(1500, 1500, 1500, 1000, 2000)).toEqual({
      roll: 0,
      pitch: 0,
      collective: 0.5,
    });

    describe("swashControlPositions", () => {
      it("matches the neutral Python passive-live widget position", () => {
        expect(swashControlPositions(1517, 1517, 1517)).toEqual({
          cyclicLeftRight: 0,
          cyclicUpDown: 0,
          collectiveUpDown: 0,
        });
      });

      it("matches Python widget cyclic and collective coordinates", () => {
        const positions = swashControlPositions(
          1773.6860279185587,
          1860.3139720814413,
          1667,
        );
        expect(positions?.cyclicLeftRight).toBeCloseTo(0.4, 10);
        expect(positions?.cyclicUpDown).toBeCloseTo(-0.2, 10);
        expect(positions?.collectiveUpDown).toBeCloseTo(0.5, 10);
      });
    });
  });

  it("matches the Python H3-120 inverse mixer", () => {
    const decoded = decodeH3Swashplate(
      1610.884572681199,
      1299.115427318801,
      1590,
      1000,
      2000,
    );
    expect(decoded?.roll).toBeCloseTo(0.4, 10);
    expect(decoded?.pitch).toBeCloseTo(-0.2, 10);
    expect(decoded?.collective).toBeCloseTo(0.5, 10);
  });

  it("uses the configured hardware collective limits", () => {
    const decoded = decodeH3Swashplate(1518, 1518, 1518, 1342, 1657);
    expect(decoded?.collective).toBeCloseTo(0.55873, 4);
  });
});
