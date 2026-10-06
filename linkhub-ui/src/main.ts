import "./styles.css";
import { LinkHubApi } from "./api";
import { diagValue, DIAG_ARRAY_NAME, isDiagRecord } from "./diag-array";
import {
  formatKbps,
  formatLinkThroughput,
  messageThroughputRows,
} from "./link-throughput";
import type { LinkHubStatus } from "./types";
import { formatCopterMode } from "./modes";
import { isArmed } from "./mav";
import { PassiveController, type PassivePhase } from "./passive";
import { VehicleScene } from "./scene";
import { decodeH3Swashplate, swashControlPositions } from "./swashplate";
import { TelemetryStore } from "./telemetry";
import type { TelemetryProfile } from "./telemetry-rates";
import { Terminal } from "./terminal";
import { MOTOR_TO_ROTOR_GEAR_RATIO } from "./vehicle-motion";

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
const telemetryProfile = required<HTMLSelectElement>("#telemetry-profile");
const linkThroughput = required<HTMLButtonElement>("#link-throughput");
const linkDetails = required<HTMLElement>("#link-details");
const overlay = required<HTMLElement>("#telemetry-overlay");
const form = required<HTMLFormElement>("#terminal-form");
const input = required<HTMLInputElement>("#terminal-input");
const swashControls = required<HTMLElement>("#swash-controls");
const cyclicDot = required<HTMLElement>("#cyclic-dot");
const collectiveDot = required<HTMLElement>("#collective-dot");
const targetErrorWidget = required<HTMLElement>("#target-error-widget");
const targetErrorDot = required<HTMLElement>("#target-error-dot");
const targetErrorValue = required<HTMLElement>("#target-error-value");
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
  const yawMotor = diagValue(
    telemetry.get("DEBUG_FLOAT_ARRAY", "rx", DIAG_ARRAY_NAME),
    "YFF_U",
  );
  const rpm = telemetry.get("RPM");
  const degrees = (value: unknown) => (
    typeof value === "number" ? (value * 180 / Math.PI).toFixed(1) : "n/a"
  );
  const field = (name: string) => String(servos?.fields[name] ?? "n/a");
  const armed = isArmed(status);
  const motorCommand = !armed
    ? "inactive"
    : yawMotor !== undefined
    ? `${(yawMotor * 100).toFixed(1)}%`
    : "n/a";
  const motorRpm = typeof rpm?.fields.rpm1 === "number" && rpm.fields.rpm1 >= 0
    ? rpm.fields.rpm1
    : null;
  const rpmText = motorRpm === null
    ? "RPM unavailable"
    : `motor ${motorRpm.toFixed(0)} RPM  rotor ${(motorRpm / MOTOR_TO_ROTOR_GEAR_RATIO).toFixed(1)} RPM`;
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
  if (controls) {
    cyclicDot.style.left = `${(controls.cyclicLeftRight + 1) * 50}%`;
    cyclicDot.style.top = `${(1 - controls.cyclicUpDown) * 50}%`;
    collectiveDot.style.top = `${(1 - controls.collectiveUpDown) * 50}%`;
  }
  const targetError = scene.targetDirectionError;
  targetErrorWidget.classList.toggle("unavailable", targetError === null);
  if (targetError) {
    const limitDegrees = 30;
    const right = Math.max(-1, Math.min(1, targetError.rightDegrees / limitDegrees));
    const forward = Math.max(-1, Math.min(1, targetError.forwardDegrees / limitDegrees));
    targetErrorDot.style.left = `${(right + 1) * 50}%`;
    targetErrorDot.style.top = `${(1 - forward) * 50}%`;
    targetErrorValue.textContent = `${targetError.totalDegrees.toFixed(1)}°`;
  } else {
    targetErrorValue.textContent = "n/a";
  }
  swashControls.classList.toggle(
    "unavailable",
    controls === null && targetError === null,
  );
  overlay.textContent = [
    `${armed ? "ARMED" : "disarmed"} · ${formatCopterMode(status?.custom_mode)}`,
    `roll ${degrees(attitude?.fields.roll)}°  pitch ${degrees(attitude?.fields.pitch)}°  yaw ${degrees(attitude?.fields.yaw)}°`,
    scene.hasCaptureTarget ? "Capture target: yellow arrow" : "No capture target",
    `swash ${swashText}`,
    `S1 ${field("servo1_raw")}  S2 ${field("servo2_raw")}  S3 ${field("servo3_raw")}  DShot/S9 ${field("servo9_raw")}  YFF_U ${motorCommand}`,
    rpmText,
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
function renderLinkThroughput(status: LinkHubStatus | null): void {
  linkThroughput.textContent = formatLinkThroughput(status);
  const rows = messageThroughputRows(status);
  const table = document.createElement("table");
  const head = table.createTHead().insertRow();
  for (const label of ["Message", "RX kbps", "TX kbps"]) {
    head.append(Object.assign(document.createElement("th"), { textContent: label }));
  }
  const body = table.createTBody();
  for (const row of rows) {
    const tr = body.insertRow();
    for (const text of [row.message, formatKbps(row.rxBps), formatKbps(row.txBps)]) {
      tr.insertCell().textContent = text;
    }
  }
  if (rows.length === 0) {
    const empty = document.createElement("div");
    empty.className = "empty";
    empty.textContent = status?.connected ? "No traffic in the last 3 s" : "Not connected";
    linkDetails.replaceChildren(empty);
  } else {
    linkDetails.replaceChildren(table);
  }
}

function setLinkDetailsOpen(open: boolean): void {
  linkDetails.hidden = !open;
  linkThroughput.setAttribute("aria-expanded", String(open));
}
linkThroughput.addEventListener("click", () => setLinkDetailsOpen(linkDetails.hidden));
document.addEventListener("click", (event) => {
  if (!linkDetails.hidden && event.target instanceof Node
    && !linkThroughput.contains(event.target) && !linkDetails.contains(event.target)) {
    setLinkDetailsOpen(false);
  }
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !linkDetails.hidden) {
    setLinkDetailsOpen(false);
  }
});
renderLinkThroughput(null);

telemetry.onRecord((record) => {
  if (record.direction === "rx" && (
    record.message === "HEARTBEAT"
    || record.message === "ATTITUDE"
    || record.message === "ATTITUDE_QUATERNION"
    || record.message === "ATTITUDE_TARGET"
    || record.message === "SERVO_OUTPUT_RAW"
    || record.message === "RPM"
    || isDiagRecord(record)
  )) {
    scheduleOverlay();
  }
});
telemetry.onGeneration(() => {
  renderLinkThroughput(null);
  scheduleOverlay();
  connectionState.textContent = "reconnecting";
  connectionState.className = "badge offline";
});
telemetry.onConnection((connected, error) => {
  if (!connected) {
    renderLinkThroughput(null);
  }
  scheduleOverlay();
  connectionState.textContent = connected ? "connected" : "offline";
  connectionState.className = `badge ${connected ? "online" : "offline"}`;
  if (error) {
    write(`Telemetry connection lost: ${error.message}`, "error");
  } else if (connected) {
    write("Telemetry connection restored.");
  }
});
telemetry.onStatus(renderLinkThroughput);
telemetryProfile.addEventListener("change", async () => {
  const profile = telemetryProfile.value as TelemetryProfile;
  telemetryProfile.disabled = true;
  try {
    await telemetry.setProfile(profile);
    write(`Telemetry profile changed to ${profile}.`);
  } catch (error) {
    write(
      `Telemetry profile change failed: ${error instanceof Error ? error.message : String(error)}`,
      "error",
    );
  } finally {
    telemetryProfile.value = telemetry.profile;
    telemetryProfile.disabled = false;
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
