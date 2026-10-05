import type { LinkHubApi } from "./api";
import { parseRunPassive, type PassiveController } from "./passive";
import type { TelemetryStore } from "./telemetry";

const MODES: Record<number, string> = {
  0: "STABILIZE",
  1: "ACRO",
  2: "ALT_HOLD",
  3: "AUTO",
  4: "GUIDED",
  5: "LOITER",
  6: "RTL",
  9: "LAND",
  16: "POSHOLD",
  20: "GUIDED_NOGPS",
};

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

export class Terminal {
  private history: string[] = [];
  private historyIndex = 0;

  constructor(
    form: HTMLFormElement,
    private readonly input: HTMLInputElement,
    private readonly output: HTMLElement,
    private readonly api: LinkHubApi,
    private readonly telemetry: TelemetryStore,
    private readonly passive: PassiveController,
  ) {
    form.addEventListener("submit", (event) => {
      event.preventDefault();
      const command = input.value.trim();
      input.value = "";
      if (command) {
        this.history.push(command);
        this.historyIndex = this.history.length;
        this.write(`> ${command}`, "command");
        void this.execute(command);
      }
    });
    input.addEventListener("keydown", (event) => this.keyDown(event));
    this.write("RAWES LinkHub UI");
    this.write("Commands: status, run passive [options], stop, help");
    input.focus();
  }

  write(message: string, kind: "info" | "error" | "fc" | "command" = "info"): void {
    const line = document.createElement("div");
    line.className = `terminal-line ${kind}`;
    line.textContent = message;
    this.output.append(line);
    this.output.scrollTop = this.output.scrollHeight;
  }

  private async execute(command: string): Promise<void> {
    const tokens = command.match(/(?:[^\s"]+|"[^"]*")+/g)?.map((value) =>
      value.startsWith("\"") ? value.slice(1, -1) : value
    ) ?? [];
    try {
      if (tokens[0] === "status" && tokens.length === 1) {
        await this.status();
      } else if (tokens[0] === "run" && tokens[1] === "passive") {
        await this.passive.start(parseRunPassive(tokens.slice(2)));
      } else if (tokens[0] === "stop" && tokens.length === 1) {
        await this.passive.stop();
      } else if (tokens[0] === "help") {
        this.help();
      } else {
        throw new Error(`Unknown command: ${command}`);
      }
    } catch (error) {
      if (!(error instanceof DOMException && error.name === "AbortError")) {
        this.write(error instanceof Error ? error.message : String(error), "error");
      }
    }
  }

  private async status(): Promise<void> {
    const status = await this.api.status();
    const armed = Boolean(status.base_mode & 128);
    this.write("VEHICLE");
    this.write(`  connected  ${status.connected ? "yes" : "no"}`);
    this.write(`  generation ${status.generation}`);
    this.write(`  armed     ${armed ? "YES" : "no"}`);
    this.write(`  mode      ${MODES[status.custom_mode] ?? `MODE_${status.custom_mode}`}`);
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
      this.write(`  EKF flags 0x${Number(ekf.fields.flags ?? 0).toString(16).padStart(4, "0")}`);
    }

    const servos = this.telemetry.get("SERVO_OUTPUT_RAW");
    if (servos) {
      this.write(
        `  servos    ${[1, 2, 3, 9]
          .map((channel) => `S${channel}=${String(servos.fields[`servo${channel}_raw`] ?? "n/a")}`)
          .join("  ")}`,
      );
    }

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

  private help(): void {
    this.write("status");
    this.write("run passive [--force] [--duration S] [--trim thr=0.342]");
    this.write("            [--roll DEG] [--pitch DEG] [--yaw DEG]");
    this.write("stop");
    this.write("During passive: arrows=roll/pitch, -/=thrust, ,/.=yaw, Space=reset, Esc=stop");
  }

  private keyDown(event: KeyboardEvent): void {
    if (event.key === "ArrowUp" && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("pitch", 1);
    } else if (event.key === "ArrowDown" && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("pitch", -1);
    } else if (event.key === "ArrowLeft" && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("roll", -1);
    } else if (event.key === "ArrowRight" && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("roll", 1);
    } else if (event.key === "-" && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("collective", -1);
    } else if (event.key === "=" && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("collective", 1);
    } else if ((event.key === "," || event.key === "<") && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("yaw", -1);
    } else if ((event.key === "." || event.key === ">") && this.passive.phase === "running") {
      event.preventDefault();
      void this.passive.adjust("yaw", 1);
    } else if (event.key === " " && this.passive.phase === "running" && this.input.value === "") {
      event.preventDefault();
      void this.passive.resetTarget();
    } else if (event.key === "Escape" && this.passive.phase !== "idle") {
      event.preventDefault();
      void this.passive.stop();
    } else if (event.key === "ArrowUp" && this.passive.phase !== "running") {
      event.preventDefault();
      this.navigateHistory(-1);
    } else if (event.key === "ArrowDown" && this.passive.phase !== "running") {
      event.preventDefault();
      this.navigateHistory(1);
    }
  }

  private navigateHistory(delta: number): void {
    this.historyIndex = Math.max(0, Math.min(this.history.length, this.historyIndex + delta));
    this.input.value = this.history[this.historyIndex] ?? "";
  }
}
