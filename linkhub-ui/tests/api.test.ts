import { describe, expect, it } from "vitest";

import { formatMavResult } from "../src/api";

describe("formatMavResult", () => {
  it("names known MAVLink command results", () => {
    expect(formatMavResult(4)).toBe("MAV_RESULT_FAILED (4)");
  });

  it("preserves unknown result codes", () => {
    expect(formatMavResult(99)).toBe("unknown MAV_RESULT (99)");
  });
});
