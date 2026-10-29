import type {
  CommandResult,
  JsonObject,
  LinkHubStatus,
  MessageBatch,
  MotorDevice,
  MotorStatus,
  ParameterResult,
  ServiceStatus,
} from "./types";
import {
  MavCmd,
  MavParamType,
  MavResult,
  type MavEnum,
  type MavlinkMessage,
} from "./generated/protocol";
import { enumIs, enumName, isArmed, mavEnum } from "./mav";

export class ApiError extends Error {
  constructor(
    public readonly status: number,
    message: string,
    public readonly payload: JsonObject | null = null,
  ) {
    super(message);
  }
}

export class LinkHubGenerationChanged extends ApiError {
  constructor(
    public readonly baseline: string,
    public readonly current: string | null,
    public readonly cursor: string | null,
    payload: JsonObject,
  ) {
    super(
      409,
      `LinkHub generation changed (${JSON.stringify(baseline)} -> ${JSON.stringify(current)})`,
      payload,
    );
  }
}

export interface MessageReadOptions {
  after: string;
  waitMs?: number;
  limit?: number;
  direction?: "rx" | "tx";
  messages?: readonly string[];
  collapse?: boolean;
  maxLagMs?: number;
  expectedGeneration?: string;
}

export interface CommandOptions {
  timeoutMs?: number;
  targetSystem?: number;
  targetComponent?: number;
}

export function formatMavResult(result: MavEnum<MavResult>): string {
  return enumName(result) ?? "unknown MAV_RESULT";
}

export class LinkHubApi {
  constructor(
    private readonly baseUrl = "",
    private readonly transport: typeof fetch = (...args) => globalThis.fetch(...args),
  ) {}

  private async request<T>(
    method: string,
    path: string,
    body?: unknown,
    signal?: AbortSignal,
  ): Promise<T> {
    const url = this.baseUrl ? new URL(path, this.baseUrl).toString() : path;
    const response = await this.transport(url, {
      method,
      body: body === undefined ? undefined : JSON.stringify(body),
      headers: body === undefined ? undefined : { "Content-Type": "application/json" },
      signal,
    });
    const value = await response.json().catch(() => null) as JsonObject | null;
    if (!response.ok) {
      const message = typeof value?.message === "string"
        ? value.message
        : `${method} ${path} failed with HTTP ${response.status}`;
      if (
        response.status === 409
        && value?.error === "generation_changed"
        && typeof value.expected_generation === "string"
      ) {
        throw new LinkHubGenerationChanged(
          value.expected_generation,
          typeof value.current_generation === "string" ? value.current_generation : null,
          typeof value.cursor === "string" ? value.cursor : null,
          value,
        );
      }
      throw new ApiError(response.status, message, value);
    }
    return value as T;
  }

  serviceStatus(signal?: AbortSignal): Promise<ServiceStatus> {
    return this.request("GET", "/v1/status", undefined, signal);
  }

  status(signal?: AbortSignal): Promise<LinkHubStatus> {
    return this.request("GET", "/v1/mavlink/status", undefined, signal);
  }

  motorStatus(): Promise<MotorStatus> {
    return this.request("GET", "/v1/motor", undefined, AbortSignal.timeout(3_000));
  }

  motorDevices(): Promise<{ devices: MotorDevice[] }> {
    return this.request("GET", "/v1/motor/devices", undefined, AbortSignal.timeout(30_000));
  }

  connectMotor(deviceId: string): Promise<MotorStatus> {
    return this.request("POST", "/v1/motor/connect", { device_id: deviceId }, AbortSignal.timeout(30_000));
  }

  runMotor(deviceId: string, speedPercent: number, direction: "forward" | "reverse", timeoutMs: number): Promise<MotorStatus> {
    return this.request("PUT", "/v1/motor", {
      device_id: deviceId, speed_percent: speedPercent, direction, timeout_ms: timeoutMs,
    }, AbortSignal.timeout(3_000));
  }

  stopMotor(): Promise<MotorStatus> {
    return this.request("POST", "/v1/motor/stop", {}, AbortSignal.timeout(15_000));
  }

  async currentCursor(signal?: AbortSignal): Promise<string> {
    return (await this.status(signal)).cursor;
  }

  async flushJournal(signal?: AbortSignal): Promise<string> {
    const result = await this.request<{ cursor: string }>(
      "POST",
      "/v1/journal/flush",
      {},
      signal,
    );
    return result.cursor;
  }

  queryMessages(options: MessageReadOptions, signal?: AbortSignal): Promise<MessageBatch> {
    const query = new URLSearchParams({
      after: options.after,
      wait_ms: String(options.waitMs ?? 0),
      limit: String(options.limit ?? 1_000),
      collapse: String(options.collapse ?? false),
    });
    if (options.direction !== undefined) {
      query.set("direction", options.direction);
    }
    if (options.messages !== undefined && options.messages.length > 0) {
      query.set("messages", options.messages.join(","));
    }
    if (options.maxLagMs !== undefined) {
      query.set("max_lag_ms", String(options.maxLagMs));
    }
    if (options.expectedGeneration !== undefined) {
      query.set("expected_generation", options.expectedGeneration);
    }
    return this.request("GET", `/v1/mavlink/messages?${query}`, undefined, signal);
  }

  readMessages(
    cursor: string,
    waitMs: number,
    collapse: boolean,
    signal?: AbortSignal,
    maxLagMs?: number,
  ): Promise<MessageBatch> {
    return this.queryMessages({
      after: cursor,
      waitMs,
      collapse,
      maxLagMs,
    }, signal);
  }

  getParameter(
    name: string,
    signal?: AbortSignal,
    timeoutMs = 3_000,
  ): Promise<ParameterResult> {
    return this.request(
      "GET",
      `/v1/mavlink/parameters/${encodeURIComponent(name.trim().toUpperCase())}?timeout_ms=${timeoutMs}`,
      undefined,
      signal,
    );
  }

  setParameter(
    name: string,
    value: number,
    integer = Number.isInteger(value),
    signal?: AbortSignal,
    timeoutMs = 3_000,
    paramType?: MavParamType,
  ): Promise<ParameterResult> {
    return this.request(
      "PUT",
      `/v1/mavlink/parameters/${encodeURIComponent(name.trim().toUpperCase())}`,
      {
        value,
        type: mavEnum(paramType ?? (integer ? MavParamType.INT32 : MavParamType.REAL32)),
        timeout_ms: timeoutMs,
      },
      signal,
    );
  }

  setMessageRates(rates: Record<string, number | null>): Promise<JsonObject> {
    return this.request("PUT", "/v1/mavlink/message-rates", rates);
  }

  async getParameters(
    names: string[],
    timeoutMs = 15_000,
    retries = 2,
  ): Promise<Map<string, ParameterResult>> {
    const result = await this.request<{ parameters: ParameterResult[] }>(
      "POST",
      "/v1/mavlink/parameters",
      { names, timeout_ms: timeoutMs, retries },
    );
    return new Map(result.parameters.map((parameter) => [parameter.name, parameter]));
  }

  async listParameters(timeoutMs = 30_000): Promise<Map<string, ParameterResult>> {
    const result = await this.request<{ parameters: ParameterResult[] }>(
      "GET",
      `/v1/mavlink/parameters?timeout_ms=${timeoutMs}`,
    );
    return new Map(result.parameters.map((parameter) => [parameter.name, parameter]));
  }

  async setParameters(
    parameters: ParameterResult[],
    timeoutMs = 15_000,
    retries = 2,
    signal?: AbortSignal,
  ): Promise<Map<string, ParameterResult>> {
    const result = await this.request<{ parameters: ParameterResult[] }>(
      "PUT",
      "/v1/mavlink/parameters",
      { parameters, timeout_ms: timeoutMs, retries },
      signal,
    );
    return new Map(result.parameters.map((parameter) => [parameter.name, parameter]));
  }

  command(
    command: MavCmd | string,
    params: number[] = [],
    options: number | CommandOptions = 10_000,
  ): Promise<CommandResult> {
    const resolved = typeof options === "number" ? { timeoutMs: options } : options;
    const body: JsonObject = {
      command: mavEnum(command),
      params,
      timeout_ms: resolved.timeoutMs ?? 10_000,
    };
    if (resolved.targetSystem !== undefined) {
      body.target_system = resolved.targetSystem;
    }
    if (resolved.targetComponent !== undefined) {
      body.target_component = resolved.targetComponent;
    }
    return this.request("POST", "/v1/mavlink/commands", body);
  }

  async sendRawMessage(
    message: string,
    fields: JsonObject,
    sourceSystem?: number,
    sourceComponent?: number,
  ): Promise<string> {
    const body: JsonObject = { message, fields };
    if (sourceSystem !== undefined) {
      body.source_system = sourceSystem;
    }
    if (sourceComponent !== undefined) {
      body.source_component = sourceComponent;
    }
    const result = await this.request<{ after_cursor: string }>(
      "POST",
      "/v1/mavlink/messages",
      body,
    );
    return result.after_cursor;
  }

  async sendMessage<TFields extends object>(
    message: MavlinkMessage<TFields>,
  ): Promise<string> {
    return this.sendRawMessage(message.message, message.fields as JsonObject);
  }

  async setMode(mode: number): Promise<void> {
    const result = await this.command(MavCmd.DO_SET_MODE, [1, mode]);
    if (!enumIs(result.result, MavResult.ACCEPTED)) {
      throw new Error(`Mode ${mode} rejected with ${formatMavResult(result.result)}`);
    }
  }

  async setArmed(armed: boolean, force = false): Promise<void> {
    const result = await this.command(MavCmd.COMPONENT_ARM_DISARM, [
      armed ? 1 : 0,
      force ? 21196 : 0,
    ], 15_000);
    if (!enumIs(result.result, MavResult.ACCEPTED)) {
      throw new Error(
        `${armed ? "Arm" : "Disarm"} rejected with ${formatMavResult(result.result)}`,
      );
    }
  }

  async setModeAndWait(mode: number, timeoutMs = 10_000): Promise<void> {
    await this.setMode(mode);
    await this.waitForStatus(
      (status) => status.custom_mode === mode,
      timeoutMs,
      `Mode ${mode} was not confirmed by heartbeat`,
    );
  }

  async setArmedAndWait(
    armed: boolean,
    force = false,
    timeoutMs = 10_000,
  ): Promise<void> {
    await this.setArmed(armed, force);
    await this.waitForStatus(
      (status) => isArmed(status) === armed,
      timeoutMs,
      `Vehicle was not confirmed ${armed ? "armed" : "disarmed"} by heartbeat`,
    );
  }

  setMessageInterval(messageId: number, intervalUs: number): Promise<CommandResult> {
    return this.command(MavCmd.SET_MESSAGE_INTERVAL, [messageId, intervalUs]);
  }

  private async waitForStatus(
    predicate: (status: LinkHubStatus) => boolean,
    timeoutMs: number,
    timeoutMessage: string,
  ): Promise<void> {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      if (predicate(await this.status())) {
        return;
      }
      await new Promise((resolve) => globalThis.setTimeout(resolve, 50));
    }
    throw new Error(timeoutMessage);
  }
}
