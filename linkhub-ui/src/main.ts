import "./styles.css";
import { LinkHubApi } from "./api";
import { formatCopterMode } from "./modes";
import { PassiveController, type PassivePhase } from "./passive";
import { VehicleScene } from "./scene";
import { decodeH3Swashplate, swashControlPositions } from "./swashplate";
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
const swashControls = required<HTMLElement>("#swash-controls");
const cyclicDot = required<HTMLElement>("#cyclic-dot");
const collectiveDot = required<HTMLElement>("#collective-dot");
let swashLimits: { min: number; max: number } | null = null;

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
  const yawMotor = telemetry.get("NAMED_VALUE_FLOAT", "rx", "YFF_U");
  const degrees = (value: unknown) => (
    typeof value === "number" ? (value * 180 / Math.PI).toFixed(1) : "n/a"
  );
  const field = (name: string) => String(servos?.fields[name] ?? "n/a");
  const armed = Boolean(status && (status.base_mode & 128));
  const motorCommand = !armed
    ? "inactive"
    : typeof yawMotor?.fields.value === "number"
    ? `${(yawMotor.fields.value * 100).toFixed(1)}%`
    : "n/a";
  const swash = swashLimits
    ? decodeH3Swashplate(
      Number(servos?.fields.servo1_raw),
      Number(servos?.fields.servo2_raw),
      Number(servos?.fields.servo3_raw),
      swashLimits.min,
      swashLimits.max,
    )
    : null;
  const signedPercent = (value: number) => `${value >= 0 ? "+" : ""}${(value * 100).toFixed(1)}%`;
  const swashText = swash
    ? `roll ${signedPercent(swash.roll)}  pitch ${signedPercent(swash.pitch)}  collective ${(swash.collective * 100).toFixed(1)}%`
    : "unavailable";
  const controls = swashControlPositions(
    Number(servos?.fields.servo1_raw),
    Number(servos?.fields.servo2_raw),
    Number(servos?.fields.servo3_raw),
  );
  swashControls.classList.toggle("unavailable", controls === null);
  if (controls) {
    cyclicDot.style.left = `${(controls.cyclicLeftRight + 1) * 50}%`;
    cyclicDot.style.top = `${(1 - controls.cyclicUpDown) * 50}%`;
    collectiveDot.style.top = `${(1 - controls.collectiveUpDown) * 50}%`;
  }
  overlay.textContent = [
    `${armed ? "ARMED" : "disarmed"} · ${formatCopterMode(status?.custom_mode)}`,
    `roll ${degrees(attitude?.fields.roll)}°  pitch ${degrees(attitude?.fields.pitch)}°  yaw ${degrees(attitude?.fields.yaw)}°`,
    scene.hasCaptureTarget ? "Capture target: yellow arrow" : "No capture target",
    `swash ${swashText}`,
    `S1 ${field("servo1_raw")}  S2 ${field("servo2_raw")}  S3 ${field("servo3_raw")}  DShot/S9 ${field("servo9_raw")}  YFF_U ${motorCommand}`,
  ].join("\n");
}

let overlayTimer: number | undefined;
function scheduleOverlay(): void {
  if (overlayTimer === undefined) {
    overlayTimer = window.setTimeout(() => {
      overlayTimer = undefined;
      updateOverlay();
    }, 100);
  }
}
telemetry.onRecord((record) => {
  if (record.direction === "rx" && (
    record.message === "HEARTBEAT"
    || record.message === "ATTITUDE"
    || record.message === "ATTITUDE_TARGET"
    || record.message === "SERVO_OUTPUT_RAW"
    || (record.message === "NAMED_VALUE_FLOAT" && record.fields.name === "YFF_U")
  )) {
    scheduleOverlay();
  }
});
telemetry.onGeneration(() => {
  scheduleOverlay();
  connectionState.textContent = "reconnecting";
  connectionState.className = "badge offline";
});
telemetry.onConnection((connected, error) => {
  scheduleOverlay();
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
try {
  const [minimum, maximum] = await Promise.all([
    api.getParameter("H_COL_MIN"),
    api.getParameter("H_COL_MAX"),
  ]);
  swashLimits = { min: minimum.value, max: maximum.value };
} catch (error) {
  write(
    `Swashplate limits unavailable: ${error instanceof Error ? error.message : String(error)}`,
    "error",
  );
}
updateOverlay();
await passive.reconcile().catch((error) => {
  write(`Passive reconciliation failed: ${error instanceof Error ? error.message : String(error)}`, "error");
});

window.addEventListener("beforeunload", () => {
  window.clearTimeout(overlayTimer);
  scene.dispose();
  telemetry.stop();
});
