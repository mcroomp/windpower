import type {
  CommandResult,
  JsonObject,
  LinkHubStatus,
  MessageBatch,
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
import { enumIs, enumName, mavEnum } from "./mav";

export class ApiError extends Error {
  constructor(
    public readonly status: number,
    message: string,
  ) {
    super(message);
  }
}

export function formatMavResult(result: MavEnum<MavResult>): string {
  return enumName(result) ?? "unknown MAV_RESULT";
}

export class LinkHubApi {
  constructor(private readonly baseUrl = "") {}

  private async request<T>(
    method: string,
    path: string,
    body?: unknown,
    signal?: AbortSignal,
  ): Promise<T> {
    const url = this.baseUrl ? new URL(path, this.baseUrl).toString() : path;
    const response = await fetch(url, {
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
      throw new ApiError(response.status, message);
    }
    return value as T;
  }

  serviceStatus(signal?: AbortSignal): Promise<ServiceStatus> {
    return this.request("GET", "/v1/status", undefined, signal);
  }

  status(signal?: AbortSignal): Promise<LinkHubStatus> {
    return this.request("GET", "/v1/mavlink/status", undefined, signal);
  }

  readMessages(
    cursor: string,
    waitMs: number,
    collapse: boolean,
    signal?: AbortSignal,
    maxLagMs?: number,
  ): Promise<MessageBatch> {
    const query = new URLSearchParams({
      after: cursor,
      wait_ms: String(waitMs),
      limit: "1000",
      collapse: String(collapse),
    });
    if (maxLagMs !== undefined) {
      query.set("max_lag_ms", String(maxLagMs));
    }
    return this.request("GET", `/v1/mavlink/messages?${query}`, undefined, signal);
  }

  getParameter(name: string, signal?: AbortSignal): Promise<ParameterResult> {
    return this.request(
      "GET",
      `/v1/mavlink/parameters/${encodeURIComponent(name)}?timeout_ms=3000`,
      undefined,
      signal,
    );
  }

  setParameter(
    name: string,
    value: number,
    integer = Number.isInteger(value),
    signal?: AbortSignal,
  ): Promise<ParameterResult> {
    return this.request(
      "PUT",
      `/v1/mavlink/parameters/${encodeURIComponent(name)}`,
      {
        value,
        type: mavEnum(integer ? MavParamType.INT32 : MavParamType.REAL32),
        timeout_ms: 3000,
      },
      signal,
    );
  }

  setMessageRates(rates: Record<string, number | null>): Promise<JsonObject> {
    return this.request("PUT", "/v1/mavlink/message-rates", rates);
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
  ): Promise<Map<string, ParameterResult>> {
    const result = await this.request<{ parameters: ParameterResult[] }>(
      "PUT",
      "/v1/mavlink/parameters",
      { parameters, timeout_ms: timeoutMs, retries: 2 },
    );
    return new Map(result.parameters.map((parameter) => [parameter.name, parameter]));
  }

  command(command: MavCmd, params: number[], timeoutMs = 10_000): Promise<CommandResult> {
    return this.request("POST", "/v1/mavlink/commands", {
      command: mavEnum(command),
      params,
      timeout_ms: timeoutMs,
    });
  }

  async sendMessage<TFields extends object>(
    message: MavlinkMessage<TFields>,
  ): Promise<string> {
    const result = await this.request<{ after_cursor: string }>(
      "POST",
      "/v1/mavlink/messages",
      {
        message: message.message,
        fields: message.fields,
      },
    );
    return result.after_cursor;
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
}
