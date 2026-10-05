import { describe, expect, it } from "vitest";
import { formatCopterMode } from "../src/modes";

describe("formatCopterMode", () => {
  it("shows the ArduCopter mode name with its number", () => {
    expect(formatCopterMode(20)).toBe("GUIDED_NOGPS (20)");
    expect(formatCopterMode(1)).toBe("ACRO (1)");
  });

  it("falls back for unknown or missing modes", () => {
    expect(formatCopterMode(99)).toBe("MODE_99 (99)");
    expect(formatCopterMode(undefined)).toBe("n/a");
  });
});
