import type { LinkHubApi } from "./api";
import type { MotorStatus } from "./types";

const LEASE_MS = 2_000;
const RENEW_MS = 500;

export class MotorPanel {
  private opened = false;
  private busy = 0;
  private pending = Promise.resolve();
  private timer: ReturnType<typeof setTimeout> | undefined;
  private status: MotorStatus | null = null;
  private ownedDevice: string | null = null;
  private leaseDeadline = 0;
  private revision = 0;
  private pollEpoch = 0;
  private error: string | null = null;
  private readonly device: HTMLSelectElement;
  private readonly scanButton: HTMLButtonElement;
  private readonly connectButton: HTMLButtonElement;
  private readonly speed: HTMLInputElement;
  private readonly speedValue: HTMLOutputElement;
  private readonly onButton: HTMLButtonElement;
  private readonly offButton: HTMLButtonElement;
  private readonly state: HTMLElement;
  private readonly hide = () => {
    if (document.hidden && this.ownedDevice) void this.stop();
  };
  private readonly leave = () => {
    if (this.ownedDevice) void this.stop();
  };
  private readonly escape = (event: KeyboardEvent) => {
    if (this.opened && event.key === "Escape" && this.ownedDevice) void this.stop();
  };

  constructor(
    root: HTMLElement,
    private readonly api: Pick<LinkHubApi, "motorStatus" | "motorDevices" | "connectMotor" | "runMotor" | "stopMotor">,
    private readonly write: (message: string, kind?: "info" | "error") => void,
  ) {
    root.innerHTML = `
      <div class="motor-head"><strong>BLUETOOTH ROTOR MOTOR</strong>
        <span data-role="state" role="status">Not connected</span>
        <button data-role="off" class="motor-off" type="button">Turn off / stop</button></div>
      <p class="motor-warning">External bench drive. Secure the mechanism and keep clear.
        Speed is a percentage, not an RPM setpoint.</p>
      <div class="motor-controls">
        <label>Bluetooth device <select data-role="device" aria-label="Bluetooth motor device">
          <option value="">Scan for devices</option></select></label>
        <button data-role="scan" type="button">Scan</button>
        <button data-role="connect" type="button">Connect</button>
        <label>Speed <input data-role="speed" aria-label="Motor speed (%)"
          type="range" min="0" max="100" step="1" value="10">
          <output data-role="speed-value">10%</output></label>
        <button data-role="on" type="button">Turn on</button>
      </div>
      <small>Uses Bluetooth on the LinkHub server. Connecting leaves the motor off.
        A 2 s lease is renewed every 500 ms while this visible panel owns the motor.
        Hiding the tab or closing the panel stops it; browser loss lets the lease expire.</small>`;
    const element = <T extends HTMLElement>(role: string): T => {
      const found = root.querySelector<T>(`[data-role="${role}"]`);
      if (!found) throw new Error(`Missing motor element ${role}`);
      return found;
    };
    this.device = element("device");
    this.scanButton = element("scan");
    this.connectButton = element("connect");
    this.speed = element("speed");
    this.speedValue = element("speed-value");
    this.onButton = element("on");
    this.offButton = element("off");
    this.state = element("state");
    this.scanButton.addEventListener("click", () => void this.scan());
    this.connectButton.addEventListener("click", () => void this.connect());
    this.device.addEventListener("change", () => this.render());
    this.onButton.addEventListener("click", () => void this.turnOn());
    this.offButton.addEventListener("click", () => void this.stop());
    this.speed.addEventListener("input", () => this.render());
    this.speed.addEventListener("change", () => {
      if (this.ownedDevice) void (Number(this.speed.value) === 0 ? this.stop() : this.renew());
    });
    document.addEventListener("visibilitychange", this.hide);
    window.addEventListener("pagehide", this.leave);
    window.addEventListener("keydown", this.escape);
    this.render();
  }

  get isOpen(): boolean { return this.opened; }

  open(): void {
    if (this.opened) return;
    this.opened = true;
    void this.poll(++this.pollEpoch);
  }

  close(): void {
    this.opened = false;
    this.pollEpoch += 1;
    clearTimeout(this.timer);
    if (this.ownedDevice) void this.stop();
  }

  dispose(): void {
    this.close();
    document.removeEventListener("visibilitychange", this.hide);
    window.removeEventListener("pagehide", this.leave);
    window.removeEventListener("keydown", this.escape);
  }

  scan(): Promise<void> {
    return this.run(async () => {
      const { devices } = await this.api.motorDevices();
      const selected = this.device.value;
      this.device.replaceChildren(new Option(devices.length ? "Select a device" : "No matching devices found", ""));
      for (const device of devices) {
        this.device.add(new Option(`${device.name} (${device.id})`, device.id));
      }
      this.device.value = selected;
      this.update(await this.api.motorStatus());
    });
  }

  connect(): Promise<void> {
    const device = this.device.value;
    return this.run(async () => {
      if (!device) throw new Error("Select a Bluetooth device first");
      this.update(await this.api.connectMotor(device));
    });
  }

  turnOn(): Promise<void> {
    const revision = this.revision;
    return this.run(async () => {
      if (revision !== this.revision) return;
      const status = await this.api.motorStatus();
      this.update(status);
      if (!this.opened || document.hidden) throw new Error("Keep the motor panel visible to turn on");
      if (!status.available || !status.connected || !status.device_id || status.device_id !== this.device.value) {
        throw new Error("Connect the selected Bluetooth device first");
      }
      if (status.running) throw new Error("Motor is already running; stop it before taking control");
      if (Number(this.speed.value) === 0) throw new Error("Choose a speed above 0%");
      if (revision !== this.revision) return;
      this.ownedDevice = status.device_id;
      await this.sendRun(revision, true);
    });
  }

  stop(): Promise<void> {
    this.ownedDevice = null;
    this.revision += 1;
    return this.run(async () => this.update(await this.api.stopMotor()));
  }

  private renew(): Promise<void> {
    return this.run(async () => {
      if (this.ownedDevice) await this.sendRun(this.revision);
    }, false);
  }

  private async sendRun(revision: number, initial = false): Promise<void> {
    const started = performance.now();
    if (revision !== this.revision || !this.ownedDevice) return;
    if (!this.opened || document.hidden || (!initial && started >= this.leaseDeadline)) {
      throw new Error("Motor lease ended; turn on explicitly to restart");
    }
    const speed = Number(this.speed.value);
    if (!Number.isInteger(speed) || speed < 1 || speed > 100) throw new Error("Speed must be 1..100%");
    const direction = this.status?.available ? this.status.direction : "forward";
    const status = await this.api.runMotor(this.ownedDevice, speed, direction, LEASE_MS);
    this.update(status);
    if (revision !== this.revision) return;
    if (!status.available || !status.connected || !status.running || status.device_id !== this.ownedDevice) {
      throw new Error("Motor did not confirm the requested running state");
    }
    this.leaseDeadline = started + LEASE_MS;
  }

  private async poll(epoch: number): Promise<void> {
    if (!this.opened || epoch !== this.pollEpoch) return;
    if (this.busy === 0) {
      await this.run(async () => {
        const status = await this.api.motorStatus();
        this.update(status);
        if (!this.ownedDevice) return;
        if (!this.opened || document.hidden || performance.now() >= this.leaseDeadline
          || !status.available || !status.connected || !status.running || status.device_id !== this.ownedDevice) {
          this.ownedDevice = null;
          this.revision += 1;
          this.update(await this.api.stopMotor());
          return;
        }
        if (Number(this.speed.value) === 0) {
          this.ownedDevice = null;
          this.update(await this.api.stopMotor());
          return;
        }
        await this.sendRun(this.revision);
      }, false);
    }
    if (this.opened && epoch === this.pollEpoch) this.timer = setTimeout(() => void this.poll(epoch), RENEW_MS);
  }

  private run(action: () => Promise<void>, foreground = true): Promise<void> {
    if (foreground) {
      this.busy += 1;
      this.error = null;
      this.render();
    }
    const task = this.pending.then(action).catch(async (error: unknown) => {
      const owned = this.ownedDevice !== null;
      this.ownedDevice = null;
      this.revision += 1;
      const message = error instanceof Error ? error.message : String(error);
      if (this.error !== message) this.write(`Motor: ${message}`, "error");
      this.error = message;
      if (owned) {
        try {
          this.update(await this.api.stopMotor());
        } catch (stopError) {
          this.write(`Motor stop failed; lease must expire: ${stopError instanceof Error ? stopError.message : String(stopError)}`, "error");
        }
      }
    }).finally(() => {
      if (foreground) this.busy -= 1;
      this.render();
    });
    this.pending = task;
    return task;
  }

  private update(status: MotorStatus): void {
    this.status = status;
    if (status.available && status.device_id && !Array.from(this.device.options).some((option) => option.value === status.device_id)) {
      this.device.add(new Option(`${status.device ?? "Motor"} (${status.device_id})`, status.device_id));
    }
    if (!this.device.value && status.available && status.device_id) this.device.value = status.device_id;
  }

  private render(): void {
    const status = this.status;
    const available = status?.available === true;
    const connected = available && status.connected;
    const running = available && status.running;
    const busy = this.busy > 0;
    const speedText = `${this.speed.value}%`;
    if (this.speedValue.textContent !== speedText) this.speedValue.textContent = speedText;
    const stateText = this.error
      ?? (available ? status.error : null)
      ?? (status?.available === false ? "Motor backend disabled in LinkHub" : !status ? "Loading"
        : running ? `ON - ${status.speed_percent}%${this.ownedDevice ? "" : " (another client)"}`
        : connected ? `OFF - ${status.device}` : "Disconnected");
    if (this.state.textContent !== stateText) this.state.textContent = stateText;
    this.state.classList.toggle("error", Boolean(this.error || (available && status.error)));
    this.scanButton.disabled = !available || busy || running;
    this.device.disabled = !available || busy || running;
    this.connectButton.disabled = !available || busy || running || !this.device.value;
    this.onButton.disabled = !connected || busy || running || Boolean(this.error)
      || this.device.value !== (available ? status.device_id : null) || Number(this.speed.value) === 0;
    this.offButton.disabled = !available;
    this.speed.disabled = !connected || busy || (running && !this.ownedDevice);
  }
}
