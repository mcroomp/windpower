import { afterEach, describe, expect, it, vi } from "vitest";
import { LinkHubApi } from "../src/api";
import type { MessageBatch, MessageRecord } from "../src/types";
import { WOBBLE_MESSAGES, WobbleModel } from "../src/wobble-model";
import { WobbleRecorder } from "../src/wobble-recorder";

function attitudeRecord(sequence: number): MessageRecord {
  return {
    received_time: "t",
    received_time_ns: 1e12 + sequence * 1e7,
    direction: "rx",
    system_id: 1,
    component_id: 1,
    message: "ATTITUDE_QUATERNION",
    cursor: `v1:${sequence}`,
    fields: {
      time_boot_ms: sequence * 10,
      rollspeed: 0,
      pitchspeed: 0,
      yawspeed: 0,
    },
  };
}

afterEach(() => vi.restoreAllMocks());

describe("WobbleRecorder", () => {
  it("reads uncollapsed attitude frames only while running", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "currentCursor").mockResolvedValue("v1:100");
    const query = vi.spyOn(api, "queryMessages");
    query.mockResolvedValueOnce({
      records: [attitudeRecord(101)],
      next_cursor: "v1:101",
    } satisfies MessageBatch).mockImplementation(() => new Promise(() => {}));
    const model = new WobbleModel();
    const ingest = vi.spyOn(model, "ingest");
    const recorder = new WobbleRecorder(api, model);
    recorder.start();
    await vi.waitFor(() => expect(ingest).toHaveBeenCalledTimes(1));
    expect(query.mock.calls[0]?.[0]).toMatchObject({
      after: "v1:100",
      collapse: false,
      direction: "rx",
      messages: WOBBLE_MESSAGES,
    });
    recorder.stop();
    expect(recorder.running).toBe(false);
  });
});
