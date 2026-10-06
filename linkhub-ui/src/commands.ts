import type { LinkHubApi } from "./api";
import {
  compareConfig,
  matchesTarget,
  type ConfigProvider,
} from "./config-core";
import { diagValue, DIAG_ARRAY_NAME } from "./diag-array";
import { formatCopterMode } from "./modes";
import { isArmed, parseFlags } from "./mav";
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
  constructor(
    private readonly api: LinkHubApi,
    private readonly telemetry: TelemetryStore,
    private readonly passive: PassiveController,
    private readonly write: CommandWriter,
    private readonly configProvider: ConfigProvider,
  ) {}

  execute(command: string): Promise<void> {
    return this.executeTokens(tokenizeCommand(command));
  }

  async executeTokens(tokens: string[]): Promise<void> {
    if (tokens[0] === "status" && tokens.length === 1) {
      await this.status();
    } else if (tokens[0] === "run" && tokens[1] === "passive") {
      await this.passive.start(parseRunPassive(tokens.slice(2)));
    } else if (tokens[0] === "stop" && tokens.length === 1) {
      await this.passive.stop();
    } else if (tokens[0] === "config") {
      await this.config(tokens.slice(1));
    } else if (tokens[0] === "help" && tokens.length === 1) {
      this.help();
    } else {
      throw new Error(`Unknown command: ${tokens.join(" ")}`);
    }
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

  private help(): void {
    this.write("status");
    this.write("config check [--all]   compare vehicle params with repo defaults");
    this.write("config apply [--all]   write differing params (disarmed only)");
    this.write("run passive [--force] [--duration S] [--trim thr=0.342]");
    this.write("            [--roll DEG] [--pitch DEG] [--yaw DEG]");
    this.write("stop                     apply canonical safe-off");
  }
}
