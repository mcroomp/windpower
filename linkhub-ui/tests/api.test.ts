import { afterEach, describe, expect, it, vi } from "vitest";

import { formatMavResult, LinkHubApi } from "../src/api";
import { MavCmd, MavParamType, MavResult } from "../src/generated/protocol";
import { mavEnum } from "../src/mav";

afterEach(() => {
  vi.unstubAllGlobals();
});

function stubFetch(response: unknown) {
  const fetchMock = vi.fn().mockResolvedValue({ ok: true, json: async () => response });
  vi.stubGlobal("fetch", fetchMock);
  return fetchMock;
}

function requestBody(fetchMock: ReturnType<typeof stubFetch>): unknown {
  return JSON.parse((fetchMock.mock.calls[0]?.[1] as { body: string }).body);
}

const ACCEPTED = { command: mavEnum(MavCmd.DO_SET_MODE), result: mavEnum(MavResult.ACCEPTED) };

describe("formatMavResult", () => {
  it("names known MAVLink command results", () => {
    expect(formatMavResult(mavEnum(MavResult.FAILED))).toBe("MAV_RESULT_FAILED");
  });

  it("preserves names outside the generated enum", () => {
    expect(formatMavResult({ type: "MAV_RESULT_FUTURE" })).toBe("MAV_RESULT_FUTURE");
  });

  it("resolves API paths against a Node CLI base URL", async () => {
    const fetchMock = stubFetch({ service: "linkhub" });

    await new LinkHubApi("http://127.0.0.1:8999").serviceStatus();

    expect(fetchMock).toHaveBeenCalledWith(
      "http://127.0.0.1:8999/v1/status",
      expect.objectContaining({ method: "GET" }),
    );
  });
});

describe("typed wire contract", () => {
  it("sends commands with a typed command enumeration", async () => {
    const fetchMock = stubFetch({ ...ACCEPTED, progress: 0, status: "ok", after_cursor: "v1:1" });

    await new LinkHubApi().command(MavCmd.USER_1, [], 5_000);

    expect(requestBody(fetchMock)).toEqual({
      command: { type: "MAV_CMD_USER_1" },
      params: [],
      timeout_ms: 5_000,
    });
  });

  it("sets mode through MAV_CMD_DO_SET_MODE", async () => {
    const fetchMock = stubFetch(ACCEPTED);

    await new LinkHubApi().setMode(20);

    expect(requestBody(fetchMock)).toMatchObject({
      command: { type: "MAV_CMD_DO_SET_MODE" },
      params: [1, 20],
    });
  });

  it("arms and force-disarms through MAV_CMD_COMPONENT_ARM_DISARM", async () => {
    const fetchMock = stubFetch(ACCEPTED);

    await new LinkHubApi().setArmed(false, true);

    expect(requestBody(fetchMock)).toMatchObject({
      command: { type: "MAV_CMD_COMPONENT_ARM_DISARM" },
      params: [0, 21196],
    });
  });

  it("reports a rejected command by dialect name", async () => {
    stubFetch({ ...ACCEPTED, result: mavEnum(MavResult.DENIED) });

    await expect(new LinkHubApi().setArmed(true)).rejects.toThrow(
      "Arm rejected with MAV_RESULT_DENIED",
    );
  });

  it("treats an unlisted result name as rejection", async () => {
    stubFetch({ ...ACCEPTED, result: { type: "MAV_RESULT_FUTURE" } });

    await expect(new LinkHubApi().setMode(1)).rejects.toThrow("MAV_RESULT_FUTURE");
  });

  it("sends parameter types as enumerations", async () => {
    const fetchMock = stubFetch({
      name: "H_SV_MAN",
      value: 0,
      type: mavEnum(MavParamType.INT32),
    });

    await new LinkHubApi().setParameter("H_SV_MAN", 0);
    expect(requestBody(fetchMock)).toEqual({
      value: 0,
      type: { type: "MAV_PARAM_TYPE_INT32" },
      timeout_ms: 3_000,
    });

    fetchMock.mockClear();
    await new LinkHubApi().setParameter("RAWES_YAW_SLP", 0.5);
    expect(requestBody(fetchMock)).toMatchObject({ type: { type: "MAV_PARAM_TYPE_REAL32" } });
  });
});
