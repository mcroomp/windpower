import type {
  MavCmd,
  MavEnum,
  MavFlags,
  MavParamType,
  MavResult,
  MavState,
} from "./generated/protocol";

export type JsonObject = Record<string, unknown>;

export interface LinkHubStatus {
  connected: boolean;
  ready: boolean;
  connection: string;
  port: string | null;
  baud: number | null;
  clock_epoch: number;
  target_system: number;
  target_component: number;
  base_mode: MavFlags;
  custom_mode: number;
  system_status: MavEnum<MavState>;
  latest_time_boot_ms: number;
  received_messages: number;
  transmitted_messages: number;
  received_bytes: number;
  transmitted_bytes: number;
  rx_bps: number | null;
  tx_bps: number | null;
  rx_bps_by_message: Record<string, number>;
  tx_bps_by_message: Record<string, number>;
  error?: string | null;
  cursor: string;
  generation: string;
}

export interface ServiceStatus {
  service: string;
  api_version: number;
  run_id: string;
  cursor: string;
}

export interface MessageRecord {
  received_time: string;
  received_time_ns: number;
  direction: "rx" | "tx";
  system_id: number;
  component_id: number;
  message: string;
  fields: JsonObject;
  cursor: string;
  sim_clock?: SimClock;
}

export interface MessageBatch {
  records: MessageRecord[];
  next_cursor: string;
  next_clock?: SimClock;
}

export interface SimClock {
  epoch: number;
  time_boot_ms: number | null;
  quality: string | null;
}

export interface ParameterResult {
  name: string;
  value: number;
  type: MavEnum<MavParamType>;
}

export interface CommandResult {
  command: MavEnum<MavCmd>;
  result: MavEnum<MavResult>;
  progress: number;
  status: string;
  after_cursor: string;
}

export type Quaternion = readonly [number, number, number, number];
