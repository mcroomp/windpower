import { afterEach, describe, expect, it, vi } from "vitest";

import {
  formatMavResult,
  LinkHubApi,
  LinkHubGenerationChanged,
} from "../src/api";
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
  return JSON.parse((fetchMock.mock.lastCall?.[1] as { body: string }).body);
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
  it("uses the motor discovery, exact selection, leased command and stop endpoints", async () => {
    const fetchMock = stubFetch({ available: true });
    const api = new LinkHubApi();
    await api.motorStatus();
    await api.motorDevices();
    await api.connectMotor("motor-2");
    expect(requestBody(fetchMock)).toEqual({ device_id: "motor-2" });
    await api.runMotor("motor-2", 15, "forward", 2000);
    expect(requestBody(fetchMock)).toEqual({
      device_id: "motor-2", speed_percent: 15, direction: "forward", timeout_ms: 2000,
    });
    await api.stopMotor();
    expect(fetchMock.mock.calls.map(([url, init]) => [url, init.method])).toEqual([
      ["/v1/motor", "GET"],
      ["/v1/motor/devices", "GET"],
      ["/v1/motor/connect", "POST"],
      ["/v1/motor", "PUT"],
      ["/v1/motor/stop", "POST"],
    ]);
  });

  it("reads selected parameters in one batch operation with timeout and retries", async () => {
    const records = [
      { name: "SERVO9_MIN", value: 1000, type: mavEnum(MavParamType.INT16), index: 1, count: 2, after_cursor: "v1:7" },
      { name: "RAWES_YAW_SLP", value: 0.5, type: mavEnum(MavParamType.REAL32), index: 2, count: 2, after_cursor: "v1:8" },
    ];
    const fetchMock = stubFetch({ parameters: records });
    const result = await new LinkHubApi().getParameters(["SERVO9_MIN", "RAWES_YAW_SLP"], 6000, 1);
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith(
      "/v1/mavlink/parameters", expect.objectContaining({ method: "POST" }),
    );
    expect(requestBody(fetchMock)).toEqual({
      names: ["SERVO9_MIN", "RAWES_YAW_SLP"], timeout_ms: 6000, retries: 1,
    });
    expect([...result.values()]).toEqual(records);
  });

  it("reads filtered journal messages with generation protection", async () => {
    const fetchMock = stubFetch({ records: [], next_cursor: "v1:8" });

    await new LinkHubApi().queryMessages({
      after: "v1:3",
      waitMs: 1_500,
      limit: 25,
      direction: "rx",
      messages: ["HEARTBEAT", "STATUSTEXT"],
      collapse: true,
      maxLagMs: 500,
      expectedGeneration: "v1:run:2",
    });

    const url = String(fetchMock.mock.calls[0]?.[0]);
    expect(url).toContain("/v1/mavlink/messages?");
    expect(Object.fromEntries(new URL(url, "http://test").searchParams)).toEqual({
      after: "v1:3",
      wait_ms: "1500",
      limit: "25",
      collapse: "true",
      direction: "rx",
      messages: "HEARTBEAT,STATUSTEXT",
      max_lag_ms: "500",
      expected_generation: "v1:run:2",
    });
  });

  it("surfaces generation changes with recovery details", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      status: 409,
      json: async () => ({
        error: "generation_changed",
        message: "generation changed",
        expected_generation: "v1:old:1",
        current_generation: "v1:new:2",
        cursor: "v1:19",
      }),
    }));

    const failure = await new LinkHubApi().queryMessages({
      after: "v1:4",
      expectedGeneration: "v1:old:1",
    }).catch((error: unknown) => error);

    expect(failure).toBeInstanceOf(LinkHubGenerationChanged);
    expect(failure).toMatchObject({
      baseline: "v1:old:1",
      current: "v1:new:2",
      cursor: "v1:19",
    });
  });

  it("flushes the journal and sends decoded messages by name", async () => {
    const fetchMock = stubFetch({ cursor: "v1:7" });
    await expect(new LinkHubApi().flushJournal()).resolves.toBe("v1:7");
    expect(fetchMock).toHaveBeenLastCalledWith(
      "/v1/journal/flush",
      expect.objectContaining({ method: "POST", body: "{}" }),
    );

    fetchMock.mockResolvedValue({
      ok: true,
      json: async () => ({ after_cursor: "v1:8" }),
    });
    await expect(
      new LinkHubApi().sendRawMessage("NAMED_VALUE_FLOAT", { name: "TEST", value: 1 }),
    ).resolves.toBe("v1:8");
    expect(requestBody(fetchMock)).toEqual({
      message: "NAMED_VALUE_FLOAT",
      fields: { name: "TEST", value: 1 },
    });
  });

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

    fetchMock.mockClear();
    await new LinkHubApi().setParameter(
      " rawes_mode ",
      0,
      true,
      undefined,
      5_000,
      MavParamType.INT8,
    );
    expect(fetchMock.mock.calls[0]?.[0]).toBe(
      "/v1/mavlink/parameters/RAWES_MODE",
    );
    expect(requestBody(fetchMock)).toEqual({
      value: 0,
      type: { type: "MAV_PARAM_TYPE_INT8" },
      timeout_ms: 5_000,
    });
  });

  it.each([
    [-128, MavParamType.INT8],
    [32767, MavParamType.INT16],
    [16_777_218, MavParamType.INT32],
    [1.25, MavParamType.REAL32],
  ])("preserves parameter value %s across all four operations", async (value, paramType) => {
    const record = { name: "TEST", value, type: mavEnum(paramType) };
    const fetchMock = stubFetch(record);
    const api = new LinkHubApi();

    await expect(api.getParameter("TEST")).resolves.toEqual(record);
    await expect(api.setParameter("TEST", value, Number.isInteger(value), undefined, 3_000, paramType))
      .resolves.toEqual(record);
    expect(requestBody(fetchMock)).toMatchObject({ value, type: record.type });

    fetchMock.mockResolvedValue({ ok: true, json: async () => ({ parameters: [record] }) });
    await expect(api.listParameters()).resolves.toEqual(new Map([["TEST", record]]));
    await expect(api.setParameters([record])).resolves.toEqual(new Map([["TEST", record]]));
    expect(requestBody(fetchMock)).toMatchObject({ parameters: [record] });
  });

  it("forwards the abort signal and retry budget for a batch parameter write", async () => {
    const record = { name: "SERVO1_TRIM", value: 1378, type: mavEnum(MavParamType.INT32) };
    const fetchMock = stubFetch({ parameters: [record] });
    const operation = new AbortController();
    await expect(new LinkHubApi().setParameters([record], 15_000, 2, operation.signal))
      .resolves.toEqual(new Map([[record.name, record]]));
    expect(fetchMock).toHaveBeenCalledWith(
      "/v1/mavlink/parameters",
      expect.objectContaining({ method: "PUT", signal: operation.signal }),
    );
    expect(requestBody(fetchMock)).toEqual({ parameters: [record], timeout_ms: 15_000, retries: 2 });
  });

  it("surfaces rejected lossy integer writes", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      status: 400,
      json: async () => ({
        error: "invalid_operation",
        message: "integer parameter value cannot be represented exactly",
      }),
    }));
    await expect(new LinkHubApi().setParameter("TEST", 16_777_217))
      .rejects.toThrow("cannot be represented exactly");
  });
});
