import { describe, expect, it } from "vitest";

import { decodeH3Swashplate, swashControlPositions } from "../src/swashplate";

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
