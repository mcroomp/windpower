import type { LinkHubApi } from "./api";
import { CommandRunner } from "./commands";
import { browserConfigProvider } from "./config";
import type { PassiveController } from "./passive";
import type { TelemetryStore } from "./telemetry";

export class Terminal {
  private history: string[] = [];
  private historyIndex = 0;
  private readonly runner: CommandRunner;

  constructor(
    form: HTMLFormElement,
    private readonly input: HTMLInputElement,
    private readonly output: HTMLElement,
    api: LinkHubApi,
    telemetry: TelemetryStore,
    private readonly passive: PassiveController,
  ) {
    this.runner = new CommandRunner(
      api,
      telemetry,
      passive,
      (message, kind) => this.write(message, kind),
      browserConfigProvider,
    );
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
    this.write("Commands: status, config check|apply, run passive [options], stop, help");
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
    try {
      await this.runner.execute(command);
    } catch (error) {
      if (!(error instanceof DOMException && error.name === "AbortError")) {
        this.write(error instanceof Error ? error.message : String(error), "error");
      }
    }
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
      void this.passive.recaptureTarget();
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
