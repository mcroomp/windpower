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

  it.each([185, 300, 522])("preserves frequencies, amplitudes and resolution with %i ms gaps", (gapMs) => {
    const allTimes = Array.from({ length: 2048 }, (_, index) => index / 256);
    const times = allTimes.filter((time) =>
      !(time > 1 && time < 1 + gapMs / 1000)
      && !(time > 4 && time < 4 + gapMs / 1000)
    );
    const values = times.map((time) =>
      0.8 * Math.sin(2 * Math.PI * 12 * time) + 0.25 * Math.sin(2 * Math.PI * 97 * time)
    );
    const spectrum = analyzeWobble(times, values);
    expect(spectrum?.durationS).toBeCloseTo(2047 / 256);
    expect(spectrum?.frequenciesHz).toHaveLength(1024);
    expect(spectrum?.coverage).toBeCloseTo(times.length / allTimes.length, 2);
    expect(spectrum?.components[0]?.frequencyHz).toBeCloseTo(12, 1);
    expect(spectrum?.components[0]?.amplitude).toBeCloseTo(0.8, 1);
    expect(spectrum?.components[1]?.frequencyHz).toBeCloseTo(97, 1);
    expect(spectrum?.components[1]?.amplitude).toBeCloseTo(0.25, 1);
  });

  it("does not fabricate a ramp across a missing interval", () => {
    const times = [
      ...Array.from({ length: 256 }, (_, index) => index / 256),
      ...Array.from({ length: 256 }, (_, index) => 7 + index / 256),
    ];
    const spectrum = analyzeWobble(times, times.map(() => 5));
    expect(spectrum?.coverage).toBeCloseTo(0.25, 2);
    expect(spectrum?.amplitudes.every((amplitude) => amplitude === 0)).toBe(true);
  });
});

describe("WobbleModel", () => {
  it("distinguishes initial collection from gap recovery", () => {
    const model = new WobbleModel();
    expect(model.snapshot()).toEqual({
      spectra: { roll: null, pitch: null, yaw: null },
      state: "collecting",
    });
    model.ingest([record(1, 0, 0, 0)]);
    expect(model.snapshot().state).toBe("collecting");
  });

  it.each([185, 300, 522])("keeps analyzing immediately after a %i ms gap", (gapMs) => {
    const model = new WobbleModel();
    const records = Array.from({ length: 512 }, (_, index) => {
      const time = index * 0.003;
      const wave = Math.sin(2 * Math.PI * 12 * time);
      return record(time, wave, wave * 0.5, wave * 0.25);
    });
    model.ingest(records);
    const before = model.snapshot();
    expect(before.state).toBe("live");
    expect(before.spectra.roll?.durationS).toBeCloseTo(1.533);
    const gapEnd = 1.533 + gapMs / 1000;
    model.ingest([record(gapEnd, 1, 0.5, 0.25)]);
    const resumed = model.snapshot();
    expect(resumed.state).toBe("live");
    expect(resumed.spectra).not.toBe(before.spectra);
    for (const spectrum of Object.values(resumed.spectra)) {
      expect(spectrum?.durationS).toBeCloseTo(gapEnd);
      expect(spectrum?.coverage).toBeLessThan(1);
      expect(spectrum?.coverage).toBeGreaterThan(0.7);
    }
  });

  it("holds the last analysis only when too few samples remain in the window", () => {
    const model = new WobbleModel();
    model.ingest(Array.from({ length: 64 }, (_, index) => record(index * 0.003, 1, 2, 3)));
    const before = model.snapshot();
    expect(before.state).toBe("live");
    model.ingest(Array.from({ length: 31 }, (_, index) => record(10 + index * 0.003, 1, 2, 3)));
    expect(model.snapshot()).toEqual({ spectra: before.spectra, state: "rebuilding" });
    model.ingest([record(10 + 31 * 0.003, 1, 2, 3)]);
    expect(model.snapshot().state).toBe("live");
  });

  it.each(["reset", "reboot"])("clears held spectra on %s", (action) => {
    const model = new WobbleModel();
    model.ingest(Array.from({ length: 64 }, (_, index) => record(10 + index * 0.003, 1, 2, 3)));
    expect(model.snapshot().state).toBe("live");
    if (action === "reset") {
      model.reset();
    } else {
      model.ingest([record(0, 1, 2, 3)]);
    }
    expect(model.snapshot()).toEqual({
      spectra: { roll: null, pitch: null, yaw: null },
      state: "collecting",
    });
  });

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
