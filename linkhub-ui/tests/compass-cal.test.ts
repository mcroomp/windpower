import { afterEach, describe, expect, it, vi } from "vitest";
import { LinkHubApi } from "../src/api";
import {
  CompassCalibration,
  formatMagCalProgress,
  formatMagCalReport,
  parseCompassCommand,
  type CompassCalibrationOptions,
} from "../src/compass-cal";
import { MagCalStatus, MavCmd, MavModeFlag, MavResult } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";
import type { LinkHubStatus, MessageRecord } from "../src/types";

const START_SEQUENCE = 10;
const DEFAULTS: CompassCalibrationOptions = {
  mask: 0, retryOnFailure: false, autosave: false, delaySeconds: 0,
};

class FakeTelemetry {
  private readonly records = new Set<(record: MessageRecord) => void>();
  private readonly generations = new Set<(generation: string, previous: string | null) => void>();
  onRecord(listener: (record: MessageRecord) => void): () => boolean {
    this.records.add(listener);
    return () => this.records.delete(listener);
  }
  onGeneration(listener: (generation: string, previous: string | null) => void): () => boolean {
    this.generations.add(listener);
    return () => this.generations.delete(listener);
  }
  emit(record: MessageRecord): void {
    this.records.forEach((listener) => listener(record));
  }
  generationChanged(): void {
    this.generations.forEach((listener) => listener("second", "first"));
  }
}

function record(sequence: number, message: string, fields: Record<string, unknown>): MessageRecord {
  return {
    received_time: "t", received_time_ns: sequence, direction: "rx", system_id: 1,
    component_id: 1, message, cursor: `v1:${sequence}`, fields,
  };
}

const progress = (sequence: number, compass: number, pct: number, mask = 1) =>
  record(sequence, "MAG_CAL_PROGRESS", {
    compass_id: compass, cal_mask: mask, attempt: 1, completion_pct: pct,
    cal_status: mavEnum(MagCalStatus.RUNNING_STEP_ONE),
  });

const report = (
  sequence: number, compass: number, status: MagCalStatus, saved = 0, mask = 1,
) => record(sequence, "MAG_CAL_REPORT", {
  compass_id: compass, cal_mask: mask, cal_status: mavEnum(status), autosaved: saved,
  fitness: 4.5, ofs_x: 1, ofs_y: -2, ofs_z: 3, diag_x: 1, diag_y: 1, diag_z: 1,
  offdiag_x: 0, offdiag_y: 0, offdiag_z: 0,
});

function setup(armed = false, result: MavResult = MavResult.ACCEPTED) {
  const api = new LinkHubApi();
  vi.spyOn(api, "status").mockResolvedValue({
    base_mode: armed ? MavModeFlag.SAFETY_ARMED : "",
  } as LinkHubStatus);
  const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
  const command = vi.spyOn(api, "command").mockResolvedValue({
    command: mavEnum(MavCmd.DO_START_MAG_CAL), result: mavEnum(result), progress: 0,
    status: "", after_cursor: `v1:${START_SEQUENCE}`,
  });
  const telemetry = new FakeTelemetry();
  const output: string[] = [];
  const compass = new CompassCalibration(api, telemetry, (message) => output.push(message));
  return { api, rates, command, telemetry, output, compass };
}

afterEach(() => vi.restoreAllMocks());

describe("compass command parsing", () => {
  it("parses calibrate options and the accept/cancel actions", () => {
    expect(parseCompassCommand(["calibrate"])).toEqual({ action: "calibrate", options: DEFAULTS });
    expect(parseCompassCommand(["calibrate", "--mask", "3", "--retry", "--autosave", "--delay", "5"]))
      .toEqual({
        action: "calibrate",
        options: { mask: 3, retryOnFailure: true, autosave: true, delaySeconds: 5 },
      });
    expect(parseCompassCommand(["accept"])).toEqual({ action: "accept" });
    expect(parseCompassCommand(["cancel"])).toEqual({ action: "cancel" });
  });

  it("rejects bad input", () => {
    expect(() => parseCompassCommand([])).toThrow("Usage");
    expect(() => parseCompassCommand(["calibrate", "--mask", "9"])).toThrow("--mask");
    expect(() => parseCompassCommand(["calibrate", "--delay", "99"])).toThrow("--delay");
    expect(() => parseCompassCommand(["calibrate", "--delay"])).toThrow("requires a value");
    expect(() => parseCompassCommand(["calibrate", "--bogus"])).toThrow("Unknown");
    expect(() => parseCompassCommand(["accept", "extra"])).toThrow("Usage");
  });
});

describe("compass message formatting", () => {
  it("formats progress and reports", () => {
    expect(formatMagCalProgress(progress(1, 2, 35).fields)).toBe("compass 2  RUNNING_STEP_ONE  35%");
    const lines = formatMagCalReport(report(1, 0, MagCalStatus.SUCCESS).fields);
    expect(lines[0]).toBe("compass 0  SUCCESS  fitness 4.50  saved no");
    expect(lines[1]).toBe("  offsets  1.00, -2.00, 3.00");
  });
});

describe("compass calibration", () => {
  it("requests the streams, sends DO_START_MAG_CAL and streams progress to completion", async () => {
    const { rates, command, telemetry, output, compass } = setup();
    const finished = compass.start({ ...DEFAULTS, retryOnFailure: true, delaySeconds: 3 });
    await vi.waitFor(() => expect(output.join("\n")).toContain("started"));

    expect(rates).toHaveBeenCalledWith({ MAG_CAL_PROGRESS: 2, MAG_CAL_REPORT: 2 });
    expect(command).toHaveBeenCalledWith(MavCmd.DO_START_MAG_CAL, [0, 1, 0, 3, 0]);
    expect(compass.active).toBe(true);

    telemetry.emit(report(START_SEQUENCE - 1, 0, MagCalStatus.SUCCESS));
    telemetry.emit(progress(11, 0, 4));
    telemetry.emit(progress(12, 0, 6));
    telemetry.emit(progress(13, 0, 7));
    telemetry.emit(progress(14, 0, 55));
    telemetry.emit(report(15, 0, MagCalStatus.SUCCESS));
    telemetry.emit(report(16, 0, MagCalStatus.SUCCESS));
    await finished;

    expect(compass.active).toBe(false);
    const text = output.join("\n");
    expect(text.match(/RUNNING_STEP_ONE/g)).toHaveLength(3);
    expect(text.match(/fitness/g)).toHaveLength(1);
    expect(text).toContain("Run 'compass accept'");
    expect(rates).toHaveBeenLastCalledWith({ MAG_CAL_PROGRESS: null, MAG_CAL_REPORT: null });
  });

  it("handles records that arrive before the command acknowledgement", async () => {
    const { api, telemetry, output, compass } = setup();
    let release: (value: Awaited<ReturnType<typeof api.command>>) => void = () => {};
    vi.spyOn(api, "command").mockReturnValue(new Promise((resolve) => { release = resolve; }));
    const finished = compass.start(DEFAULTS);
    await vi.waitFor(() => expect(api.command).toHaveBeenCalled());
    telemetry.emit(progress(5, 0, 50));
    telemetry.emit(report(12, 0, MagCalStatus.SUCCESS, 1));
    release({
      command: mavEnum(MavCmd.DO_START_MAG_CAL), result: mavEnum(MavResult.ACCEPTED),
      progress: 0, status: "", after_cursor: `v1:${START_SEQUENCE}`,
    });
    await finished;
    expect(output.join("\n")).not.toContain("50%");
    expect(output.join("\n")).toContain("saved; reboot to apply");
  });

  it("waits for the vehicle to save when autosave is requested", async () => {
    const { telemetry, output, compass } = setup();
    let ended = false;
    const finished = compass.start({ ...DEFAULTS, autosave: true }).then(() => { ended = true; });
    await vi.waitFor(() => expect(output.join("\n")).toContain("started"));
    telemetry.emit(report(11, 0, MagCalStatus.SUCCESS, 0));
    await Promise.resolve();
    expect(ended).toBe(false);
    telemetry.emit(report(12, 0, MagCalStatus.SUCCESS, 1));
    await finished;
    expect(output.join("\n")).toContain("saved; reboot to apply");
    expect(output.join("\n")).not.toContain("Run 'compass accept'");
  });

  it("waits for every compass the vehicle reports in cal_mask", async () => {
    const { telemetry, output, compass } = setup();
    let ended = false;
    const finished = compass.start(DEFAULTS).then(() => { ended = true; });
    await vi.waitFor(() => expect(output.join("\n")).toContain("started"));
    telemetry.emit(progress(11, 1, 20, 3));
    telemetry.emit(report(12, 0, MagCalStatus.SUCCESS, 0, 3));
    await Promise.resolve();
    expect(ended).toBe(false);
    telemetry.emit(report(13, 1, MagCalStatus.FAILED, 0, 3));
    await finished;
    expect(output.join("\n")).toContain("finished with a failure");
  });

  it("only follows the requested compasses and keeps waiting through retries", async () => {
    const { telemetry, output, compass } = setup();
    let ended = false;
    const finished = compass.start({ ...DEFAULTS, mask: 2, retryOnFailure: true })
      .then(() => { ended = true; });
    await vi.waitFor(() => expect(output.join("\n")).toContain("started"));
    telemetry.emit(report(11, 0, MagCalStatus.SUCCESS, 0, 3));
    telemetry.emit(report(12, 1, MagCalStatus.FAILED, 0, 3));
    await Promise.resolve();
    expect(ended).toBe(false);
    expect(output.join("\n")).toContain("retrying");
    telemetry.emit(report(13, 1, MagCalStatus.SUCCESS, 0, 3));
    await finished;
    expect(output.join("\n")).not.toContain("compass 0");
  });

  it("refuses to start while armed", async () => {
    const { command, compass } = setup(true);
    await expect(compass.start(DEFAULTS)).rejects.toThrow("ARMED");
    expect(command).not.toHaveBeenCalled();
  });

  it("explains a rejected start and restores the stream rates", async () => {
    const { rates, compass } = setup(false, MavResult.FAILED);
    await expect(compass.start(DEFAULTS)).rejects.toThrow("COMPASS_USE");
    expect(compass.active).toBe(false);
    expect(rates).toHaveBeenLastCalledWith({ MAG_CAL_PROGRESS: null, MAG_CAL_REPORT: null });
  });

  it("aborts when the link generation changes", async () => {
    const { telemetry, output, compass } = setup();
    const finished = compass.start(DEFAULTS);
    await vi.waitFor(() => expect(output.join("\n")).toContain("started"));
    telemetry.generationChanged();
    await expect(finished).rejects.toThrow("generation changed");
    expect(compass.active).toBe(false);
  });

  it("cancels an active calibration and sends DO_CANCEL_MAG_CAL", async () => {
    const { command, telemetry, output, compass } = setup();
    const finished = compass.start(DEFAULTS);
    await vi.waitFor(() => expect(output.join("\n")).toContain("started"));
    telemetry.emit(progress(11, 0, 10));
    await compass.cancel();
    await finished;
    expect(command).toHaveBeenLastCalledWith(MavCmd.DO_CANCEL_MAG_CAL, [0]);
    expect(output.join("\n")).toContain("cancelled");
    expect(output.join("\n")).not.toContain("succeeded");
    expect(compass.active).toBe(false);
  });

  it("accepts a finished calibration and reports a refusal", async () => {
    const { command, output, compass } = setup();
    await compass.accept();
    expect(command).toHaveBeenCalledWith(MavCmd.DO_ACCEPT_MAG_CAL, [0]);
    expect(output.join("\n")).toContain("accepted");
    command.mockResolvedValue({
      command: mavEnum(MavCmd.DO_ACCEPT_MAG_CAL), result: mavEnum(MavResult.FAILED),
      progress: 0, status: "", after_cursor: "v1:1",
    });
    await expect(compass.accept()).rejects.toThrow("no successful, unsaved calibration");
  });
});
