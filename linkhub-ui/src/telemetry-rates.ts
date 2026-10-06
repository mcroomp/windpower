export type TelemetryProfile = "usb" | "radio";

// Rate 0 disables a message on the vehicle. ATTITUDE is redundant with
// ATTITUDE_QUATERNION (same body rates; the UI derives Euler angles from the
// quaternion), so both profiles switch it off instead of streaming it.
export const TELEMETRY_PROFILES: Readonly<
  Record<TelemetryProfile, Readonly<Record<string, number>>>
> = Object.freeze({
  usb: Object.freeze({
    ATTITUDE: 0,
    ATTITUDE_QUATERNION: 25,
    ATTITUDE_TARGET: 25,
    SERVO_OUTPUT_RAW: 25,
    LOCAL_POSITION_NED: 10,
    BATTERY_STATUS: 2,
    RC_CHANNELS: 1,
    RPM: 5,
  }),
  radio: Object.freeze({
    ATTITUDE: 0,
    ATTITUDE_QUATERNION: 10,
    ATTITUDE_TARGET: 10,
    SERVO_OUTPUT_RAW: 10,
    LOCAL_POSITION_NED: 5,
    BATTERY_STATUS: 1,
    RC_CHANNELS: 1,
    RPM: 5,
  }),
});

export const DISPLAY_TELEMETRY_RATES = TELEMETRY_PROFILES.usb;
