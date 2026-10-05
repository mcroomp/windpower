// Generated from LinkHub's Rust protocol descriptor. Do not edit.

export interface MavlinkMessage<TFields extends object> {
  readonly message: string;
  readonly fields: TFields;
}

export enum MavSeverity {
  EMERGENCY = 0,
  ALERT = 1,
  CRITICAL = 2,
  ERROR = 3,
  WARNING = 4,
  NOTICE = 5,
  INFO = 6,
  DEBUG = 7,
}

export enum MavType {
  HELICOPTER = 4,
  GCS = 6,
}

export enum MavAutopilot {
  ARDUPILOTMEGA = 3,
  INVALID = 8,
}

export enum MavState {
  STANDBY = 3,
  ACTIVE = 4,
}

export enum MavVtolState {
  UNDEFINED = 0,
  TRANSITION_TO_FW = 1,
  TRANSITION_TO_MC = 2,
  MC = 3,
  FW = 4,
}

export enum MavLandedState {
  UNDEFINED = 0,
  ON_GROUND = 1,
  IN_AIR = 2,
  TAKEOFF = 3,
  LANDING = 4,
}

export enum MavResult {
  ACCEPTED = 0,
  TEMPORARILY_REJECTED = 1,
  DENIED = 2,
  UNSUPPORTED = 3,
  FAILED = 4,
}

export enum PidAxis {
  ROLL = 1,
  PITCH = 2,
  YAW = 3,
  ACCEL_Z = 4,
}

export enum MavParamType {
  INT8 = 2,
  INT16 = 4,
  INT32 = 6,
  REAL32 = 9,
}

export interface StatusTextFields {
  readonly text: string;
  readonly severity?: MavSeverity;
}

export class StatusText implements MavlinkMessage<StatusTextFields> {
  readonly message = "STATUSTEXT";
  constructor(readonly fields: StatusTextFields) {}
}

export interface AttitudeFields {
  readonly roll: number;
  readonly pitch: number;
  readonly yaw: number;
  readonly rollspeed: number;
  readonly pitchspeed: number;
  readonly yawspeed: number;
  readonly time_boot_ms?: number;
}

export class Attitude implements MavlinkMessage<AttitudeFields> {
  readonly message = "ATTITUDE";
  constructor(readonly fields: AttitudeFields) {}
}

export interface AttitudeQuaternionFields {
  readonly q1: number;
  readonly q2: number;
  readonly q3: number;
  readonly q4: number;
  readonly rollspeed?: number;
  readonly pitchspeed?: number;
  readonly yawspeed?: number;
  readonly time_boot_ms?: number;
}

export class AttitudeQuaternion implements MavlinkMessage<AttitudeQuaternionFields> {
  readonly message = "ATTITUDE_QUATERNION";
  constructor(readonly fields: AttitudeQuaternionFields) {}
}

export interface LocalPositionNedFields {
  readonly x: number;
  readonly y: number;
  readonly z: number;
  readonly vx?: number;
  readonly vy?: number;
  readonly vz?: number;
  readonly time_boot_ms?: number;
}

export class LocalPositionNed implements MavlinkMessage<LocalPositionNedFields> {
  readonly message = "LOCAL_POSITION_NED";
  constructor(readonly fields: LocalPositionNedFields) {}
}

export interface GlobalPositionIntFields {
  readonly lat: number;
  readonly lon: number;
  readonly alt: number;
  readonly relative_alt: number;
  readonly vx?: number;
  readonly vy?: number;
  readonly vz?: number;
  readonly hdg?: number;
  readonly time_boot_ms?: number;
}

export class GlobalPositionInt implements MavlinkMessage<GlobalPositionIntFields> {
  readonly message = "GLOBAL_POSITION_INT";
  constructor(readonly fields: GlobalPositionIntFields) {}
}

export interface EkfStatusReportFields {
  readonly flags: number;
  readonly velocity_variance?: number;
  readonly pos_horiz_variance?: number;
  readonly pos_vert_variance?: number;
  readonly compass_variance?: number;
  readonly terrain_alt_variance?: number;
}

export class EkfStatusReport implements MavlinkMessage<EkfStatusReportFields> {
  readonly message = "EKF_STATUS_REPORT";
  constructor(readonly fields: EkfStatusReportFields) {}
}

export interface BatteryStatusFields {
  readonly current_battery?: number;
  readonly battery_remaining?: number;
  readonly voltages?: readonly number[];
}

export class BatteryStatus implements MavlinkMessage<BatteryStatusFields> {
  readonly message = "BATTERY_STATUS";
  constructor(readonly fields: BatteryStatusFields) {}
}

export interface SysStatusFields {
  readonly onboard_control_sensors_present?: number;
  readonly onboard_control_sensors_enabled?: number;
  readonly onboard_control_sensors_health?: number;
  readonly load?: number;
  readonly voltage_battery?: number;
  readonly current_battery?: number;
  readonly battery_remaining?: number;
}

export class SysStatus implements MavlinkMessage<SysStatusFields> {
  readonly message = "SYS_STATUS";
  constructor(readonly fields: SysStatusFields) {}
}

export interface HeartbeatFields {
  readonly type: MavType;
  readonly autopilot: MavAutopilot;
  readonly base_mode: number;
  readonly custom_mode: number;
  readonly system_status: MavState;
  readonly mavlink_version?: number;
}

export class Heartbeat implements MavlinkMessage<HeartbeatFields> {
  readonly message = "HEARTBEAT";
  constructor(readonly fields: HeartbeatFields) {}
}

export interface ExtendedSysStateFields {
  readonly vtol_state: MavVtolState;
  readonly landed_state: MavLandedState;
}

export class ExtendedSysState implements MavlinkMessage<ExtendedSysStateFields> {
  readonly message = "EXTENDED_SYS_STATE";
  constructor(readonly fields: ExtendedSysStateFields) {}
}

export interface NamedValueFloatFields {
  readonly name: string;
  readonly value: number;
  readonly time_boot_ms?: number;
}

export class NamedValueFloat implements MavlinkMessage<NamedValueFloatFields> {
  readonly message = "NAMED_VALUE_FLOAT";
  constructor(readonly fields: NamedValueFloatFields) {}
}

export interface NamedValueIntFields {
  readonly name: string;
  readonly value: number;
  readonly time_boot_ms?: number;
}

export class NamedValueInt implements MavlinkMessage<NamedValueIntFields> {
  readonly message = "NAMED_VALUE_INT";
  constructor(readonly fields: NamedValueIntFields) {}
}

export interface CommandAckFields {
  readonly command: number;
  readonly result: MavResult;
}

export class CommandAck implements MavlinkMessage<CommandAckFields> {
  readonly message = "COMMAND_ACK";
  constructor(readonly fields: CommandAckFields) {}
}

export interface CommandLongFields {
  readonly target_system: number;
  readonly target_component: number;
  readonly command: number;
  readonly confirmation?: number;
  readonly param1?: number;
  readonly param2?: number;
  readonly param3?: number;
  readonly param4?: number;
  readonly param5?: number;
  readonly param6?: number;
  readonly param7?: number;
}

export class CommandLong implements MavlinkMessage<CommandLongFields> {
  readonly message = "COMMAND_LONG";
  constructor(readonly fields: CommandLongFields) {}
}

export interface SetAttitudeTargetFields {
  readonly target_system?: number;
  readonly target_component?: number;
  readonly type_mask?: number;
  readonly q?: readonly number[];
  readonly body_roll_rate?: number;
  readonly body_pitch_rate?: number;
  readonly body_yaw_rate?: number;
  readonly thrust?: number;
  readonly time_boot_ms?: number;
}

export class SetAttitudeTarget implements MavlinkMessage<SetAttitudeTargetFields> {
  readonly message = "SET_ATTITUDE_TARGET";
  constructor(readonly fields: SetAttitudeTargetFields) {}
}

export interface RcChannelsFields {
  readonly chan1_raw?: number | null;
  readonly chan2_raw?: number | null;
  readonly chan3_raw?: number | null;
  readonly chan4_raw?: number | null;
}

export class RcChannels implements MavlinkMessage<RcChannelsFields> {
  readonly message = "RC_CHANNELS";
  constructor(readonly fields: RcChannelsFields) {}
}

export interface ServoOutputRawFields {
  readonly servo1_raw?: number;
  readonly servo2_raw?: number;
  readonly servo3_raw?: number;
  readonly servo4_raw?: number;
  readonly servo5_raw?: number;
  readonly servo6_raw?: number;
  readonly servo7_raw?: number;
  readonly servo8_raw?: number;
  readonly servo9_raw?: number;
  readonly servo10_raw?: number;
  readonly servo11_raw?: number;
  readonly servo12_raw?: number;
  readonly servo13_raw?: number;
  readonly servo14_raw?: number;
  readonly servo15_raw?: number;
  readonly servo16_raw?: number;
  readonly port?: number;
  readonly time_usec?: number;
}

export class ServoOutputRaw implements MavlinkMessage<ServoOutputRawFields> {
  readonly message = "SERVO_OUTPUT_RAW";
  constructor(readonly fields: ServoOutputRawFields) {}
}

export interface PidTuningFields {
  readonly axis?: PidAxis;
  readonly desired?: number | null;
  readonly achieved?: number | null;
  readonly FF?: number | null;
  readonly P?: number | null;
  readonly I?: number | null;
  readonly D?: number | null;
  readonly PDmod?: number | null;
  readonly SRate?: number | null;
}

export class PidTuning implements MavlinkMessage<PidTuningFields> {
  readonly message = "PID_TUNING";
  constructor(readonly fields: PidTuningFields) {}
}

export interface ParamSetFields {
  readonly target_system: number;
  readonly target_component: number;
  readonly param_id: string;
  readonly param_value: number;
  readonly param_type: MavParamType;
}

export class ParamSet implements MavlinkMessage<ParamSetFields> {
  readonly message = "PARAM_SET";
  constructor(readonly fields: ParamSetFields) {}
}

export interface ParamValueFields {
  readonly param_id: string;
  readonly param_value: number;
  readonly param_type?: MavParamType;
  readonly param_count?: number;
  readonly param_index?: number;
}

export class ParamValue implements MavlinkMessage<ParamValueFields> {
  readonly message = "PARAM_VALUE";
  constructor(readonly fields: ParamValueFields) {}
}

export interface ParamRequestReadFields {
  readonly target_system: number;
  readonly target_component: number;
  readonly param_id: string;
  readonly param_index?: number;
}

export class ParamRequestRead implements MavlinkMessage<ParamRequestReadFields> {
  readonly message = "PARAM_REQUEST_READ";
  constructor(readonly fields: ParamRequestReadFields) {}
}

export interface RequestDataStreamFields {
  readonly target_system: number;
  readonly target_component: number;
  readonly req_stream_id: number;
  readonly req_message_rate: number;
  readonly start_stop?: number;
}

export class RequestDataStream implements MavlinkMessage<RequestDataStreamFields> {
  readonly message = "REQUEST_DATA_STREAM";
  constructor(readonly fields: RequestDataStreamFields) {}
}

export const MESSAGE_CLASSES = {
  "STATUSTEXT": StatusText,
  "ATTITUDE": Attitude,
  "ATTITUDE_QUATERNION": AttitudeQuaternion,
  "LOCAL_POSITION_NED": LocalPositionNed,
  "GLOBAL_POSITION_INT": GlobalPositionInt,
  "EKF_STATUS_REPORT": EkfStatusReport,
  "BATTERY_STATUS": BatteryStatus,
  "SYS_STATUS": SysStatus,
  "HEARTBEAT": Heartbeat,
  "EXTENDED_SYS_STATE": ExtendedSysState,
  "NAMED_VALUE_FLOAT": NamedValueFloat,
  "NAMED_VALUE_INT": NamedValueInt,
  "COMMAND_ACK": CommandAck,
  "SET_ATTITUDE_TARGET": SetAttitudeTarget,
  "ATTITUDE_TARGET": SetAttitudeTarget,
  "RC_CHANNELS": RcChannels,
  "SERVO_OUTPUT_RAW": ServoOutputRaw,
  "PID_TUNING": PidTuning,
  "PARAM_VALUE": ParamValue,
} as const;
