import type {
  MavCmd,
  MavEnum,
  MavFlags,
  MavParamType,
  MavResult,
  MavState,
} from "./generated/protocol";

export type JsonObject = Record<string, unknown>;

export interface CsvReading {
  source: string;
  filename: string;
  received_time_ns: number;
  sequence: number;
  source_sequence: number;
  sample_time_ns: number | null;
  time_quality: "estimated" | "clock_reset" | "out_of_order" | "unavailable";
  clock_epoch: number;
  values: Record<string, string | number>;
}

export interface DiagnosticRecord {
  cursor: string;
  ingest_time_ns?: number;
  data: {
    source: string;
    event: string;
    fields: JsonObject;
  };
}

export interface DiagnosticBatch {
  records: DiagnosticRecord[];
  next_cursor: string;
}

export interface CsvStatus {
  available: boolean;
  columns: string[];
  rows_received: number;
  latest: CsvReading | null;
  recent: CsvReading[];
}

export interface MotorDevice {
  id: string;
  name: string;
}

export type MotorStatus = { available: false } | {
  available: true;
  connected: boolean;
  device: string | null;
  device_id: string | null;
  running: boolean;
  speed_percent: number;
  target_speed_percent: number;
  stopping: boolean;
  direction: "forward" | "reverse";
  command_expires_in_ms: number | null;
  error: string | null;
};

export interface ExperimentScript {
  name: string;
  bytes: number;
  sha256: string;
}

/** Parameters, Lua scripts and firmware captured when the experiment started. */
export interface ExperimentSnapshot {
  state: "capturing" | "complete" | "failed";
  error: string | null;
  parameter_count: number | null;
  scripts: ExperimentScript[];
  completed_at_ns: number | null;
}

export interface ExperimentInfo {
  id: string;
  name: string;
  description: string;
  run_id: string;
  started_at_ns: number;
  start_cursor: string;
  directory: string;
  /** The UI telemetry profile when the experiment started. */
  telemetry_profile: string | null;
  snapshot: ExperimentSnapshot;
}

export interface SavedExperiment {
  experiment: ExperimentInfo;
  stopped_at_ns: number;
  end_cursor: string;
  record_count: number;
  path: string;
}

export interface CameraVideo {
  stream: string;
  file: string;
  bytes: number;
  chunks: number;
  updated_at_ns: number;
  finished: boolean;
}

export interface CameraPhone {
  stream: string;
  label: string;
  /** Experiment the phone reports it is recording, if any. */
  recording: string | null;
  last_seen_ns: number;
}

export interface CameraStatus {
  /** HTTPS camera-page URL for phones; null when unavailable (see `reason`). */
  url: string | null;
  reason: string | null;
  qr_svg: string | null;
  /** Phones that sent a heartbeat recently. */
  phones: CameraPhone[];
  experiment_id: string | null;
  /** Recordings received for the running experiment. */
  videos: CameraVideo[];
}

export interface LinkHubStatus {
  connected: boolean;
  ready: boolean;
  connection: string;
  port: string | null;
  baud: number | null;
  /** The serial port is the flight controller's own USB; its baud is nominal. */
  native_usb: boolean;
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
