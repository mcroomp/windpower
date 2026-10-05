import "./styles.css";
import { LinkHubApi } from "./api";
import { PassiveController, type PassivePhase } from "./passive";
import { VehicleScene } from "./scene";
import { TelemetryStore } from "./telemetry";
import { Terminal } from "./terminal";

function required<T extends Element>(selector: string): T {
  const element = document.querySelector<T>(selector);
  if (!element) {
    throw new Error(`Missing required element ${selector}`);
  }
  return element;
}

const api = new LinkHubApi();
const telemetry = new TelemetryStore(api);
const output = required<HTMLElement>("#terminal-output");
const operationState = required<HTMLElement>("#operation-state");
const connectionState = required<HTMLElement>("#connection-state");
const overlay = required<HTMLElement>("#telemetry-overlay");
const form = required<HTMLFormElement>("#terminal-form");
const input = required<HTMLInputElement>("#terminal-input");

let terminal: Terminal;
const write = (message: string, kind: "info" | "error" | "fc" = "info") => {
  if (terminal) {
    terminal.write(message, kind);
    return;
  }
  const line = document.createElement("div");
  line.className = `terminal-line ${kind}`;
  line.textContent = message;
  output.append(line);
};
const setPhase = (phase: PassivePhase) => {
  operationState.textContent = phase;
  operationState.className = `badge ${phase}`;
};
const passive = new PassiveController(api, telemetry, write, setPhase);
terminal = new Terminal(form, input, output, api, telemetry, passive);
const scene = new VehicleScene(required<HTMLElement>("#scene"), telemetry);

function updateOverlay(): void {
  const status = telemetry.status;
  const attitude = telemetry.get("ATTITUDE");
  const servos = telemetry.get("SERVO_OUTPUT_RAW");
  const degrees = (value: unknown) => (
    typeof value === "number" ? (value * 180 / Math.PI).toFixed(1) : "n/a"
  );
  const field = (name: string) => String(servos?.fields[name] ?? "n/a");
  overlay.textContent = [
    `${status && (status.base_mode & 128) ? "ARMED" : "disarmed"} · mode ${status?.custom_mode ?? "n/a"}`,
    `roll ${degrees(attitude?.fields.roll)}°  pitch ${degrees(attitude?.fields.pitch)}°  yaw ${degrees(attitude?.fields.yaw)}°`,
    `S1 ${field("servo1_raw")}  S2 ${field("servo2_raw")}  S3 ${field("servo3_raw")}  motor ${field("servo9_raw")} µs`,
  ].join("\n");
}

telemetry.onRecord(updateOverlay);
telemetry.onGeneration(() => {
  connectionState.textContent = "reconnecting";
  connectionState.className = "badge offline";
});
telemetry.onConnection((connected, error) => {
  connectionState.textContent = connected ? "connected" : "offline";
  connectionState.className = `badge ${connected ? "online" : "offline"}`;
  if (error) {
    write(`Telemetry connection lost: ${error.message}`, "error");
  } else if (connected) {
    write("Telemetry connection restored.");
  }
});

while (!telemetry.status) {
  try {
    await telemetry.start();
  } catch (error) {
    connectionState.textContent = "offline";
    connectionState.className = "badge offline";
    write(error instanceof Error ? error.message : String(error), "error");
    await new Promise((resolve) => window.setTimeout(resolve, 1_000));
  }
}
connectionState.textContent = "connected";
connectionState.className = "badge online";
updateOverlay();
await passive.reconcile().catch((error) => {
  write(`Passive reconciliation failed: ${error instanceof Error ? error.message : String(error)}`, "error");
});

window.addEventListener("beforeunload", () => {
  scene.dispose();
  telemetry.stop();
});
