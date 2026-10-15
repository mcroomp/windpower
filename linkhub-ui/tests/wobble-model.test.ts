import { describe, expect, it } from "vitest";
import type { MessageRecord } from "../src/types";
import { analyzeDominantMode, analyzeWobble, WobbleModel } from "../src/wobble-model";

function record(timeS: number, roll: number, pitch: number, yaw: number): MessageRecord {
  return {
    received_time: "t",
    received_time_ns: 1e12 + timeS * 1e9,
    direction: "rx",
    system_id: 1,
    component_id: 1,
    message: "ATTITUDE_QUATERNION",
    cursor: `v1:${timeS}`,
    fields: {
      time_boot_ms: timeS * 1000,
      rollspeed: roll,
      pitchspeed: pitch,
      yawspeed: yaw,
    },
  };
}

describe("analyzeWobble", () => {
  it("identifies the strongest component frequencies and amplitudes", () => {
    const sampleRate = 256;
    const times = Array.from({ length: sampleRate * 8 }, (_, index) => index / sampleRate);
    const values = times.map((time) =>
      0.8 * Math.sin(2 * Math.PI * 12 * time) + 0.25 * Math.sin(2 * Math.PI * 37 * time)
    );
    const spectrum = analyzeWobble(times, values);
    expect(spectrum).not.toBeNull();
    expect(spectrum?.components[0]?.frequencyHz).toBeCloseTo(12, 1);
    expect(spectrum?.components[0]?.amplitude).toBeCloseTo(0.8, 1);
    expect(spectrum?.components[1]?.frequencyHz).toBeCloseTo(37, 1);
    expect(spectrum?.components[1]?.amplitude).toBeCloseTo(0.25, 1);
  });

  it("returns no spectrum until enough samples are available", () => {
    expect(analyzeWobble([0, 0.1], [1, 2])).toBeNull();
  });
});

describe("WobbleModel", () => {
  it("keeps independent roll, pitch, and yaw spectra", () => {
    const sampleRate = 128;
    const records = Array.from({ length: sampleRate * 8 }, (_, index) => {
      const time = index / sampleRate;
      return record(
        time,
        Math.sin(2 * Math.PI * 7 * time),
        Math.sin(2 * Math.PI * 13 * time),
        Math.sin(2 * Math.PI * 19 * time),
      );
    });
    const model = new WobbleModel();
    model.ingest(records);
    expect(model.spectrum("roll")?.components[0]?.frequencyHz).toBeCloseTo(7, 1);
    expect(model.spectrum("pitch")?.components[0]?.frequencyHz).toBeCloseTo(13, 1);
    expect(model.spectrum("yaw")?.components[0]?.frequencyHz).toBeCloseTo(19, 1);
    model.reset();
    expect(model.spectrum("roll")).toBeNull();
  });

  it("finds a fixed principal body axis at the dominant frequency", () => {
    const sampleRate = 256;
    const times = Array.from({ length: sampleRate * 8 }, (_, index) => index / sampleRate);
    const wave = times.map((time) => Math.sin(2 * Math.PI * 12 * time));
    const mode = analyzeDominantMode({
      roll: analyzeWobble(times, wave.map((value) => value * 0.8)),
      pitch: analyzeWobble(times, wave.map((value) => value * 0.4)),
      yaw: analyzeWobble(times, wave.map((value) => value * 0.2)),
    });
    expect(mode?.frequencyHz).toBeCloseTo(12, 1);
    expect(mode?.linearity).toBeCloseTo(1, 3);
    expect(mode?.contributions.roll).toBeCloseTo(0.8 ** 2 / 0.84, 2);
    expect(mode?.contributions.pitch).toBeCloseTo(0.4 ** 2 / 0.84, 2);
    expect(mode?.contributions.yaw).toBeCloseTo(0.2 ** 2 / 0.84, 2);
  });

  it("recognizes circular motion as having no reliable fixed axis", () => {
    const sampleRate = 256;
    const times = Array.from({ length: sampleRate * 8 }, (_, index) => index / sampleRate);
    const mode = analyzeDominantMode({
      roll: analyzeWobble(times, times.map((time) => Math.cos(2 * Math.PI * 12 * time))),
      pitch: analyzeWobble(times, times.map((time) => Math.sin(2 * Math.PI * 12 * time))),
      yaw: analyzeWobble(times, times.map(() => 0)),
    });
    expect(mode?.frequencyHz).toBeCloseTo(12, 1);
    expect(mode?.linearity).toBeLessThan(0.02);
  });
});
