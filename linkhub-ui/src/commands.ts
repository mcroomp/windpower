import {
  formatMavResult,
  type LinkHubApi,
  type MessageReadOptions,
} from "./api";
import { CompassCalibration, parseCompassCommand } from "./compass-cal";
import {
  compareConfig,
  matchesTarget,
  type ConfigProvider,
} from "./config-core";
import { diagValue, DIAG_ARRAY_NAME } from "./diag-array";
import { MavCmd, MavParamType, MavResult } from "./generated/protocol";
import { formatCopterMode } from "./modes";
import { enumIs, isArmed, parseFlags } from "./mav";
import { parseRunPassive, type PassiveController } from "./passive";
import { decodeH3Swashplate } from "./swashplate";
import type { TelemetryStore } from "./telemetry";
import type { ParameterResult } from "./types";

const STATUS_PARAMETERS = [
  "FRAME_CLASS",
  "INITIAL_MODE",
  "H_TAIL_TYPE",
  "SCR_ENABLE",
  "RAWES_MODE",
  "RAWES_YAW_SLP",
  "ARMING_SKIPCHK",
  "BRD_SAFETY_DEFLT",
  "ACRO_TRAINER",
  "FS_THR_ENABLE",
  "FS_GCS_ENABLE",
];

export type CommandOutputKind = "info" | "error" | "fc" | "command";
export type CommandWriter = (message: string, kind?: CommandOutputKind) => void;

export function tokenizeCommand(command: string): string[] {
  return command.match(/(?:[^\s"]+|"[^"]*")+/g)?.map((value) =>
    value.startsWith("\"") ? value.slice(1, -1) : value
  ) ?? [];
}

export class CommandRunner {
  private readonly compass: CompassCalibration;

  constructor(
    private readonly api: LinkHubApi,
    private readonly telemetry: TelemetryStore,
    private readonly passive: PassiveController,
    private readonly write: CommandWriter,
    private readonly configProvider: ConfigProvider,
  ) {
    this.compass = new CompassCalibration(api, telemetry, write);
  }

  get compassActive(): boolean {
    return this.compass.active;
  }

  cancelCompass(): Promise<void> {
    return this.compass.cancel();
  }

  execute(command: string): Promise<void> {
    return this.executeTokens(tokenizeCommand(command));
  }

  async executeTokens(tokens: string[]): Promise<void> {
    if (tokens[0] === "status" && tokens.length === 1) {
      await this.status();
    } else if (tokens[0] === "hub" && tokens[1] === "status" && tokens.length === 2) {
      this.writeJson(await this.api.serviceStatus());
    } else if (tokens[0] === "journal") {
      await this.journal(tokens.slice(1));
    } else if (tokens[0] === "message") {
      await this.message(tokens.slice(1));
    } else if (tokens[0] === "command") {
      await this.command(tokens.slice(1));
    } else if (tokens[0] === "param") {
      await this.parameter(tokens.slice(1));
    } else if (tokens[0] === "mode") {
      await this.mode(tokens.slice(1));
    } else if (tokens[0] === "arm") {
      await this.arm(tokens.slice(1));
    } else if (tokens[0] === "disarm" && tokens.length === 1) {
      await this.passive.stop();
    } else if (tokens[0] === "rates") {
      await this.rates(tokens.slice(1));
    } else if (tokens[0] === "run" && tokens[1] === "passive") {
      if (this.compass.active) {
        throw new Error("run passive refused: a compass calibration is active");
      }
      await this.passive.start(parseRunPassive(tokens.slice(2)));
    } else if (tokens[0] === "stop" && tokens.length === 1) {
      await this.passive.stop();
    } else if (tokens[0] === "reboot" && tokens.length === 1) {
      await this.reboot();
    } else if (tokens[0] === "compass") {
      await this.compassCommand(tokens.slice(1));
    } else if (tokens[0] === "config") {
      await this.config(tokens.slice(1));
    } else if (tokens[0] === "help" && tokens.length === 1) {
      this.help();
    } else {
      throw new Error(`Unknown command: ${tokens.join(" ")}`);
    }
  }

  private writeJson(value: unknown): void {
    this.write(JSON.stringify(value, null, 2));
  }

  private async journal(args: string[]): Promise<void> {
    if (args[0] === "cursor" && args.length === 1) {
      this.write(await this.api.currentCursor());
      return;
    }
    if (args[0] === "flush" && args.length === 1) {
      this.write(await this.api.flushJournal());
      return;
    }
    if (args[0] !== "read" || args.length < 2) {
      throw new Error(
        "Usage: journal cursor | journal flush | journal read AFTER "
        + "[--messages A,B] [--direction rx|tx] [--wait S] [--limit N] "
        + "[--collapse] [--expected-generation TOKEN]",
      );
    }
    const options: MessageReadOptions = { after: args[1] as string };
    for (let index = 2; index < args.length; index += 1) {
      const option = args[index];
      if (option === "--collapse") {
        options.collapse = true;
        continue;
      }
      const value = args[index + 1];
      if (value === undefined) {
        throw new Error(`${option} requires a value`);
      }
      if (option === "--messages") {
        options.messages = value.split(",").filter(Boolean);
      } else if (option === "--direction" && (value === "rx" || value === "tx")) {
        options.direction = value;
      } else if (option === "--wait") {
        options.waitMs = parseNonNegativeNumber(value, "--wait") * 1_000;
      } else if (option === "--limit") {
        options.limit = parsePositiveInteger(value, "--limit");
      } else if (option === "--expected-generation") {
        options.expectedGeneration = value;
      } else {
        throw new Error(`Unknown journal read option: ${option}`);
      }
      index += 1;
    }
    this.writeJson(await this.api.queryMessages(options));
  }

  private async message(args: string[]): Promise<void> {
    if (args[0] !== "send" || args.length !== 3) {
      throw new Error("Usage: message send NAME COMPACT_JSON_FIELDS");
    }
    const fields = parseJsonObject(args[2] as string, "message fields");
    this.write(await this.api.sendRawMessage(args[1] as string, fields));
  }

  private async command(args: string[]): Promise<void> {
    if (args[0] !== "send" || args.length < 2) {
      throw new Error("Usage: command send MAV_CMD_NAME [PARAM ...] [--timeout S]");
    }
    const params: number[] = [];
    let timeoutMs = 10_000;
    for (let index = 2; index < args.length; index += 1) {
      const value = args[index] as string;
      if (value === "--timeout") {
        const timeout = args[index + 1];
        if (timeout === undefined) {
          throw new Error("--timeout requires a value");
        }
        timeoutMs = parsePositiveNumber(timeout, "--timeout") * 1_000;
        index += 1;
      } else {
        params.push(parseFiniteNumber(value, "command parameter"));
      }
    }
    this.writeJson(await this.api.command(args[1] as string, params, timeoutMs));
  }

  private async parameter(args: string[]): Promise<void> {
    if (args[0] === "get" && args.length === 2) {
      this.writeJson(await this.api.getParameter(args[1] as string));
      return;
    }
    if (args[0] === "list" && args.length === 1) {
      this.writeJson(Object.fromEntries(await this.api.listParameters()));
      return;
    }
    if (args[0] !== "set" || (args.length !== 3 && args.length !== 5)) {
      throw new Error(
        "Usage: param get NAME | param list | "
        + "param set NAME VALUE [--type MAV_PARAM_TYPE_NAME]",
      );
    }
    const value = parseFiniteNumber(args[2] as string, "parameter value");
    let paramType: MavParamType | undefined;
    if (args.length === 5) {
      if (args[3] !== "--type" || !Object.values(MavParamType).includes(args[4] as MavParamType)) {
        throw new Error("--type must be a full MAV_PARAM_TYPE_* name");
      }
      paramType = args[4] as MavParamType;
    }
    this.writeJson(
      await this.api.setParameter(
        args[1] as string,
        value,
        Number.isInteger(value),
        undefined,
        3_000,
        paramType,
      ),
    );
  }

  private async mode(args: string[]): Promise<void> {
    if (args[0] !== "set" || args.length !== 2) {
      throw new Error("Usage: mode set CUSTOM_MODE");
    }
    const mode = parseNonNegativeInteger(args[1] as string, "custom mode");
    await this.api.setModeAndWait(mode);
    this.write(`Mode ${mode} confirmed`);
  }

  private async arm(args: string[]): Promise<void> {
    if (args.length > 1 || (args.length === 1 && args[0] !== "--force")) {
      throw new Error("Usage: arm [--force]");
    }
    if (this.compass.active) {
      throw new Error("arm refused: a compass calibration is active");
    }
    if (this.passive.phase !== "idle") {
      throw new Error("arm refused: a passive run is active");
    }
    const force = args[0] === "--force";
    await this.api.setArmedAndWait(true, force);
    this.write(`Vehicle armed${force ? " (pre-arm checks bypassed)" : ""}`);
  }

  private async rates(args: string[]): Promise<void> {
    if (args[0] !== "set" || args.length < 2) {
      throw new Error("Usage: rates set MESSAGE=HZ|default [MESSAGE=HZ|default ...]");
    }
    const rates: Record<string, number | null> = {};
    for (const assignment of args.slice(1)) {
      const separator = assignment.indexOf("=");
      if (separator <= 0 || separator === assignment.length - 1) {
        throw new Error(`Invalid rate assignment: ${assignment}`);
      }
      const name = assignment.slice(0, separator).toUpperCase();
      const value = assignment.slice(separator + 1);
      rates[name] = value === "default"
        ? null
        : parseNonNegativeNumber(value, `rate for ${name}`);
    }
    this.writeJson(await this.api.setMessageRates(rates));
  }

  private async status(): Promise<void> {
    const status = await this.api.status();
    const armed = isArmed(status);
    this.write("VEHICLE");
    this.write(`  connected  ${status.connected ? "yes" : "no"}`);
    this.write(`  generation ${status.generation}`);
    this.write(`  armed     ${armed ? "YES" : "no"}`);
    this.write(`  mode      ${formatCopterMode(status.custom_mode)}`);
    this.write(`  RX/TX     ${status.received_messages} / ${status.transmitted_messages}`);

    const battery = this.telemetry.get("BATTERY_STATUS");
    if (battery) {
      const voltages = Array.isArray(battery.fields.voltages)
        ? battery.fields.voltages.filter((value) => Number(value) !== 65535).map(Number)
        : [];
      const volts = voltages.reduce((sum, value) => sum + value, 0) / 1_000;
      const current = Number(battery.fields.current_battery ?? -1) / 100;
      this.write(`  battery   ${volts.toFixed(2)} V  ${current >= 0 ? `${current.toFixed(2)} A` : ""}`);
    }

    const ekf = this.telemetry.get("EKF_STATUS_REPORT");
    if (ekf) {
      const flags = [...parseFlags(ekf.fields.flags)];
      this.write(`  EKF flags ${flags.length > 0 ? flags.join(" | ") : "none"}`);
    }

    const servos = this.telemetry.get("SERVO_OUTPUT_RAW");
    if (servos) {
      this.write(
        `  servos    ${[1, 2, 3]
          .map((channel) => `S${channel}=${String(servos.fields[`servo${channel}_raw`] ?? "n/a")}`)
          .join("  ")}`,
      );
      const limits = await Promise.allSettled([
        this.api.getParameter("H_COL_MIN"),
        this.api.getParameter("H_COL_MAX"),
      ]);
      if (limits[0].status === "fulfilled" && limits[1].status === "fulfilled") {
        const swash = decodeH3Swashplate(
          Number(servos.fields.servo1_raw),
          Number(servos.fields.servo2_raw),
          Number(servos.fields.servo3_raw),
          limits[0].value.value,
          limits[1].value.value,
        );
        if (swash) {
          const signed = (value: number) =>
            `${value >= 0 ? "+" : ""}${(value * 100).toFixed(1)}%`;
          this.write(
            `  swash     roll=${signed(swash.roll)}  pitch=${signed(swash.pitch)}  `
            + `collective=${(swash.collective * 100).toFixed(1)}%`,
          );
        }
      } else {
        this.write("  swash     unavailable (H_COL_MIN/MAX read failed)", "error");
      }
    }
    const motorCommand = diagValue(
      this.telemetry.get("DEBUG_FLOAT_ARRAY", "rx", DIAG_ARRAY_NAME),
      "YFF_U",
    );
    this.write(
      `  motor     DShot/S9=${String(servos?.fields.servo9_raw ?? "n/a")}  YFF_U=${motorCommand !== undefined
        ? `${(motorCommand * 100).toFixed(1)}%`
        : "unavailable"}`,
    );

    this.write("KEY PARAMETERS");
    const values = await Promise.allSettled(
      STATUS_PARAMETERS.map((name) => this.api.getParameter(name)),
    );
    values.forEach((result, index) => {
      const name = STATUS_PARAMETERS[index] ?? "unknown";
      if (result.status === "fulfilled") {
        this.write(`  ${name.padEnd(20)} ${result.value.value}`);
      } else {
        this.write(`  ${name.padEnd(20)} unavailable`, "error");
      }
    });
  }

  private async config(args: string[]): Promise<void> {
    const [sub, ...options] = args;
    if ((sub !== "check" && sub !== "apply") || options.some((option) => option !== "--all")) {
      throw new Error("Usage: config check [--all]  OR  config apply [--all]");
    }
    const apply = sub === "apply";
    const all = options.includes("--all");
    if (apply) {
      const status = await this.api.status();
      if (isArmed(status)) {
        throw new Error("config apply refused: vehicle is ARMED");
      }
      if (this.passive.phase !== "idle") {
        throw new Error("config apply refused: a passive run is active");
      }
    }

    const targets = this.configProvider.targets(all);
    this.write(`CONFIG ${apply ? "APPLY" : "CHECK"}  (${all ? "all shared + hardware defaults" : "RAWES common + hardware overrides"})`);
    this.configProvider.sources(all).forEach((source) => this.write(`  source ${source}`));
    this.write(`  reading ${targets.size} target parameters from vehicle…`);
    const rows = compareConfig(targets, await this.api.listParameters());
    const diffs = rows.filter((row) => row.status === "diff");
    const missing = rows.filter((row) => row.status === "missing");

    let verified = new Map<string, ParameterResult>();
    if (apply && diffs.length > 0) {
      verified = await this.api.setParameters(diffs.map((row) => ({
        name: row.name,
        value: row.expected,
        type: row.type as ParameterResult["type"],
      })));
    }

    const format = (value: number | undefined) =>
      value === undefined ? "n/a" : String(Number(value.toPrecision(6)));
    let failed = missing.length;
    for (const row of diffs) {
      const line = `  ${row.name.padEnd(22)} expected ${format(row.expected).padStart(10)}  actual ${format(row.actual).padStart(10)}`;
      if (!apply) {
        this.write(`${line}  DIFF`, "error");
      } else if (matchesTarget(verified.get(row.name)?.value, row.expected)) {
        this.write(`${line}  -> SET`);
      } else {
        this.write(`${line}  FAIL verification mismatch`, "error");
        failed += 1;
      }
    }
    for (const row of missing) {
      this.write(`  ${row.name.padEnd(22)} expected ${format(row.expected).padStart(10)}  not found on vehicle`, "error");
    }
    const okCount = rows.length - diffs.length - missing.length;
    this.write(`  ${okCount} OK, ${diffs.length} differ, ${missing.length} missing`);
    if (!apply && diffs.length > 0) {
      this.write("  run 'config apply' to write the differences");
    } else if (apply && diffs.length > 0 && failed === 0) {
      this.write("  applied; reboot if any changed parameter is boot-time only");
    }
  }

  private async reboot(): Promise<void> {
    if (isArmed(await this.api.status())) {
      throw new Error("reboot refused: vehicle is ARMED (disarm first)");
    }
    if (this.passive.phase !== "idle") {
      throw new Error("reboot refused: a passive run is active");
    }
    if (this.compass.active) {
      throw new Error("reboot refused: a compass calibration is active");
    }
    this.write("Sending reboot command …");
    // param1 = 1 reboots the autopilot (3 would hold it in the bootloader).
    const result = await this.api.command(MavCmd.PREFLIGHT_REBOOT_SHUTDOWN, [1]);
    if (!enumIs(result.result, MavResult.ACCEPTED)) {
      throw new Error(`Reboot rejected with ${formatMavResult(result.result)}`);
    }
    this.write("Reboot accepted; the link reconnects in a few seconds.");
    this.write(
      "The vehicle boots in its INITIAL_MODE (ACRO once 'config apply' has set it): "
      + "run 'stop' to verify the canonical safe-off state before any armed test.",
    );
  }

  private async compassCommand(args: string[]): Promise<void> {
    const command = parseCompassCommand(args);
    if (command.action === "cancel") {
      await this.compass.cancel();
    } else if (command.action === "accept") {
      await this.compass.accept();
    } else {
      if (this.passive.phase !== "idle") {
        throw new Error("compass calibrate refused: a passive run is active");
      }
      await this.compass.start(command.options);
    }
  }

  private help(): void {
    this.write("status");
    this.write("hub status                print LinkHub service status as JSON");
    this.write("journal cursor|flush");
    this.write("journal read AFTER [--messages A,B] [--direction rx|tx] [--wait S]");
    this.write("message send NAME COMPACT_JSON_FIELDS");
    this.write("command send MAV_CMD_NAME [PARAM ...] [--timeout S]");
    this.write("param get NAME | param list | param set NAME VALUE [--type TYPE]");
    this.write("mode set CUSTOM_MODE");
    this.write("arm [--force]            arm and confirm heartbeat state");
    this.write("disarm                   apply canonical safe-off");
    this.write("rates set MESSAGE=HZ|default [...]");
    this.write("config check [--all]   compare vehicle params with repo defaults");
    this.write("config apply [--all]   write differing params (disarmed only)");
    this.write("run passive [--force] [--duration S] [--trim thr=0.342]");
    this.write("            [--roll DEG] [--pitch DEG] [--yaw DEG]");
    this.write("stop                     apply canonical safe-off");
    this.write("reboot                   reboot the autopilot (disarmed only)");
    this.write("compass calibrate [--mask N] [--retry] [--autosave] [--delay S]");
    this.write("                         disarmed only; streams progress (Esc cancels)");
    this.write("compass accept|cancel    save a finished calibration / abort");
  }
}

function parseJsonObject(value: string, label: string): Record<string, unknown> {
  let parsed: unknown;
  try {
    parsed = JSON.parse(value);
  } catch (error) {
    throw new Error(`${label} must be valid JSON`, { cause: error });
  }
  if (typeof parsed !== "object" || parsed === null || Array.isArray(parsed)) {
    throw new Error(`${label} must be a JSON object`);
  }
  return parsed as Record<string, unknown>;
}

function parseFiniteNumber(value: string, label: string): number {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    throw new Error(`${label} must be a finite number`);
  }
  return parsed;
}

function parsePositiveNumber(value: string, label: string): number {
  const parsed = parseFiniteNumber(value, label);
  if (parsed <= 0) {
    throw new Error(`${label} must be greater than zero`);
  }
  return parsed;
}

function parseNonNegativeNumber(value: string, label: string): number {
  const parsed = parseFiniteNumber(value, label);
  if (parsed < 0) {
    throw new Error(`${label} must not be negative`);
  }
  return parsed;
}

function parsePositiveInteger(value: string, label: string): number {
  const parsed = parsePositiveNumber(value, label);
  if (!Number.isInteger(parsed)) {
    throw new Error(`${label} must be an integer`);
  }
  return parsed;
}

function parseNonNegativeInteger(value: string, label: string): number {
  const parsed = parseNonNegativeNumber(value, label);
  if (!Number.isInteger(parsed)) {
    throw new Error(`${label} must be an integer`);
  }
  return parsed;
}
