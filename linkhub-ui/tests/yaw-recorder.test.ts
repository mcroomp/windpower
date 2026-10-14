import { afterEach, describe, expect, it, vi } from "vitest";
import { LinkHubApi } from "../src/api";
import type { MessageBatch, MessageRecord } from "../src/types";
import { YAW_MESSAGES, YawModel } from "../src/yaw-model";
import { YawRecorder } from "../src/yaw-recorder";

function pidRecord(sequence: number): MessageRecord {
  return {
    received_time: "t", received_time_ns: 1e12 + sequence * 1e7, direction: "rx", system_id: 1,
    component_id: 1, message: "PID_TUNING", cursor: `v1:${sequence}`,
    fields: {
      desired: 0, achieved: 0, FF: 0, P: 0.1, I: 0, D: 0, axis: { type: "PID_TUNING_YAW" },
    },
  };
}

afterEach(() => vi.restoreAllMocks());

describe("YawRecorder", () => {
  it("reads unfiltered yaw frames from the live end of the journal", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "currentCursor").mockResolvedValue("v1:100");
    const query = vi.spyOn(api, "queryMessages");
    let calls = 0;
    query.mockImplementation(async () => {
      calls += 1;
      if (calls === 1) {
        return { records: [pidRecord(101), pidRecord(102)], next_cursor: "v1:102" } satisfies MessageBatch;
      }
      return new Promise(() => {});
    });
    const model = new YawModel();
    const recorder = new YawRecorder(api, model);
    recorder.start();
    await vi.waitFor(() => expect(model.pid.length).toBe(2));
    expect(query.mock.calls[0]?.[0]).toMatchObject({
      after: "v1:100", collapse: false, direction: "rx", messages: YAW_MESSAGES,
    });
    await vi.waitFor(() => expect(query.mock.calls[1]?.[0]).toMatchObject({ after: "v1:102" }));
    recorder.stop();
    expect(recorder.running).toBe(false);
  });

  it("reports a failure once, then resumes from a fresh cursor", async () => {
    const api = new LinkHubApi();
    const cursor = vi.spyOn(api, "currentCursor").mockResolvedValue("v1:1");
    let calls = 0;
    vi.spyOn(api, "queryMessages").mockImplementation(async () => {
      calls += 1;
      if (calls <= 2) {
        throw new Error("link down");
      }
      return calls === 3
        ? { records: [pidRecord(2)], next_cursor: "v1:2" }
        : new Promise(() => {});
    });
    const model = new YawModel();
    const errors: string[] = [];
    const recorder = new YawRecorder(api, model, (error) => errors.push(error.message));
    recorder.start();
    await vi.waitFor(() => expect(model.pid.length).toBe(1), { timeout: 3_000 });
    expect(errors).toEqual(["link down"]);
    expect(cursor.mock.calls.length).toBeGreaterThanOrEqual(3);
    recorder.stop();
  });

  it("starts only once", () => {
    const api = new LinkHubApi();
    const current = vi.spyOn(api, "currentCursor").mockImplementation(() => new Promise(() => {}));
    const recorder = new YawRecorder(api, new YawModel());
    recorder.start();
    recorder.start();
    expect(current).toHaveBeenCalledTimes(1);
    recorder.stop();
  });
});
