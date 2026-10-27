import type { PassiveController } from "./passive";
import { isArmed } from "./mav";
import { swashNeutralTrim, swashServoEnvelope, type SwashGeometry, type SwashLimits } from "./swashplate";
import type { TelemetryStore } from "./telemetry";

export class SwashPanel {
  private opened = false;
  private busy = false;
  private ownsOperation = false;
  private loaded = false;
  private error: string | null = null;
  private timer: ReturnType<typeof setInterval> | null = null;
  private readonly unsubscribe: () => void;
  private readonly unsubscribeGeneration: () => void;
  private geometry: SwashGeometry | null = null;
  private geometryError: string | null = null;
  private loadEpoch = 0;
  private readonly envelope: HTMLElement;
  private readonly neutral: HTMLElement;
  private readonly status: HTMLElement;
  private readonly min: HTMLInputElement;
  private readonly max: HTMLInputElement;
  private readonly cyclic: HTMLInputElement;
  private readonly collective: HTMLInputElement;
  private readonly radius: HTMLInputElement;
  private readonly duration: HTMLInputElement;
  private readonly secured: HTMLInputElement;
  private readonly clearance: HTMLInputElement;
  private readonly enable: HTMLButtonElement;
  private readonly stopButton: HTMLButtonElement;
  private readonly apply: HTMLButtonElement;
  private readonly reload: HTMLButtonElement;
  private readonly sweepButton: HTMLButtonElement;
  private readonly sweepOpen: HTMLButtonElement;
  private readonly sweepDialog: HTMLDialogElement;
  private readonly sweepStatus: HTMLElement;
  private readonly sweepServos: HTMLElement;
  private readonly sweepDot: HTMLElement;
  private readonly sweepPosition: HTMLElement;
  private readonly pad: HTMLElement;
  private readonly dot: HTMLElement;
  private readonly position: HTMLElement;
  private readonly servoValues: HTMLElement;
  private readonly motionButtons: HTMLButtonElement[] = [];
  private readonly escape = (event: KeyboardEvent) => {
    if (event.key === "Escape" && this.opened && this.ownsOperation) {
      event.preventDefault();
      void this.run(() => this.sweepDialog.open ? this.closeSweep() : this.stop());
    }
  };

  constructor(
    root: HTMLElement,
    private readonly controller: PassiveController,
    private readonly telemetry: TelemetryStore,
    private readonly write: (message: string, kind?: "info" | "error") => void,
    private readonly limitsChanged: (limits: SwashLimits) => void,
    private readonly compassActive: () => boolean,
  ) {
    root.innerHTML = `
      <div class="swash-head"><strong>SWASH CALIBRATION</strong><span data-role="status" role="status">Read-only</span>
        <button data-role="stop" class="swash-stop" type="button">Stop / safe-off</button></div>
      <p class="swash-warning">Servos live when enabled. Secure rotor and swash; keep clear and start small.
        Expanding limits can bind linkages. Updated Lua required (mode 6).</p>
      <div class="swash-grid">
        <fieldset><legend>Configured limits</legend>
          <label title="H_COL_MIN">Min (us)<input data-role="min" aria-label="Collective minimum (us)" type="number" min="800" max="2199" step="1"></label>
          <label title="H_COL_MAX">Max (us)<input data-role="max" aria-label="Collective maximum (us)" type="number" min="801" max="2200" step="1"></label>
          <label title="H_CYC_MAX">Cyclic (cdeg)<input data-role="cyclic" aria-label="Cyclic maximum (cdeg)" type="number" min="0" max="4500" step="1"></label>
          <div class="swash-actions"><button data-role="reload" type="button">Reload limits</button><button data-role="apply" type="button">Apply limits</button></div>
          <small>Applies H_COL_MIN/MAX, H_CYC_MAX and all three servo trims at the midpoint (rounded down).
            Can move servos even while stopped; writes persist. 100 cdeg = 1 degree.</small>
          <output data-role="neutral" aria-label="Neutral trim consistency"></output>
          <div class="swash-envelope"><strong>Calculated servo min/max (us)</strong>
            <output data-role="envelope" aria-label="Calculated servo minimum and maximum">Reload limits to calculate</output>
            <small>Draft limits + full cyclic, loaded trims/reversal. Estimated, not measured. Clipping is marked.</small></div>
        </fieldset>
        <fieldset><legend>Manual movement</legend>
          <label class="swash-check"><input data-role="secured" type="checkbox">Mechanism secured; area clear</label>
          <div class="swash-actions"><button data-role="enable" type="button">Enable movement</button></div>
          <div data-role="pad" class="swash-pad" aria-label="Cyclic control pad">
            <span class="swash-pad-label">+ pitch</span><span data-role="dot" class="swash-dot"></span>
          </div>
          <div class="swash-presets" data-role="presets"></div>
          <label>Collective <input data-role="collective" type="range" min="0" max="100" value="50" step="1"></label>
          <output data-role="position"></output>
        </fieldset>
        <fieldset><legend>Servo readback</legend>
          <pre data-role="servos" aria-live="off">Waiting for servo telemetry</pre>
          <button data-role="sweep-open" type="button">Automatic cyclic sweep...</button>
          <small>One bounded circle. Enable movement first.</small>
        </fieldset>
      </div>
      <dialog data-role="sweep-dialog" class="swash-sweep-dialog" aria-labelledby="swash-sweep-title">
          <div class="swash-dialog-head"><h2 id="swash-sweep-title">Automatic cyclic sweep</h2>
            <button data-role="sweep-close" class="swash-stop" type="button">Stop / close</button></div>
          <p class="swash-warning">Servos live. Confirm mechanical clearance before sweeping.
            Closing this dialog or Escape stops calibration and applies safe-off.</p>
          <p data-role="sweep-status" role="status"></p>
          <div class="swash-sweep-feedback">
            <div>
              <div class="swash-pad swash-sweep-pad" role="img" aria-label="Commanded cyclic position">
                <span class="swash-pad-label">+ pitch</span><span data-role="sweep-dot" class="swash-dot"></span>
              </div>
              <small>Commanded cyclic</small>
            </div>
            <div><output data-role="sweep-position"></output>
              <pre data-role="sweep-servos">Waiting for servo telemetry</pre></div>
          </div>
          <label>Radius (%)<input data-role="radius" aria-label="Sweep radius (% of cyclic limit)" type="number" min="1" max="100" step="1" value="25"></label>
          <label>Duration (s)<input data-role="duration" aria-label="One-circle duration (s)" type="number" min="4" max="60" step="1" value="12"></label>
          <label class="swash-check"><input data-role="clearance" type="checkbox">Clearance checked at this collective and sweep radius</label>
          <button data-role="sweep" type="button">Sweep one circle</button>
          <small>Ramps from center, circles, then returns to center. Collective stays fixed. Stop / close neutralizes all axes.</small>
      </dialog>`;
    const element = <T extends HTMLElement>(role: string): T => {
      const found = root.querySelector<T>(`[data-role="${role}"]`);
      if (!found) throw new Error(`Missing swash element ${role}`);
      return found;
    };
    this.status = element("status");
    this.envelope = element("envelope");
    this.neutral = element("neutral");
    this.min = element("min");
    this.max = element("max");
    this.cyclic = element("cyclic");
    this.collective = element("collective");
    this.radius = element("radius");
    this.duration = element("duration");
    this.secured = element("secured");
    this.clearance = element("clearance");
    this.enable = element("enable");
    this.stopButton = element("stop");
    this.apply = element("apply");
    this.reload = element("reload");
    this.sweepButton = element("sweep");
    this.sweepOpen = element("sweep-open");
    this.sweepDialog = element("sweep-dialog");
    this.sweepStatus = element("sweep-status");
    this.sweepServos = element("sweep-servos");
    this.sweepDot = element("sweep-dot");
    this.sweepPosition = element("sweep-position");
    this.pad = element("pad");
    this.dot = element("dot");
    this.position = element("position");
    this.servoValues = element("servos");
    const presets = element("presets");
    for (const [label, roll, pitch] of [
      ["+ pitch limit", 0, 1], ["- roll limit", -1, 0], ["Center cyclic", 0, 0],
      ["+ roll limit", 1, 0], ["- pitch limit", 0, -1],
    ] as const) {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = label.replace(" limit", "").replace(" cyclic", "");
      button.setAttribute("aria-label", label);
      button.title = label;
      button.addEventListener("click", () => void this.run(() => this.move(roll, pitch)));
      this.motionButtons.push(button);
      presets.append(button);
    }
    this.enable.addEventListener("click", () => void this.run(async () => {
      if (this.compassActive()) throw new Error("Stop compass calibration before swash calibration.");
      if (!this.secured.checked || this.controller.phase !== "idle") throw new Error("Confirm secured hardware and stop other runs first.");
      this.ownsOperation = true;
      this.clearance.checked = false;
      await this.controller.startSwash();
    }, true));
    this.stopButton.addEventListener("click", () => void this.run(() => this.stop()));
    this.reload.addEventListener("click", () => void this.run(() => this.load(), true));
    this.apply.addEventListener("click", () => void this.run(async () => {
      if (this.compassActive()) throw new Error("Stop compass calibration before editing swash limits.");
      if (!this.secured.checked) throw new Error("Confirm secured hardware before applying limits and trims.");
      this.ownsOperation = true;
      const limits = { min: this.min.valueAsNumber, max: this.max.valueAsNumber, cyclic: this.cyclic.valueAsNumber };
      this.loaded = false;
      this.geometry = null;
      this.geometryError = "Settings changed or partially applied; reload limits to verify.";
      this.clearance.checked = false;
      await this.controller.applySwashLimits(limits);
      await this.load();
      this.write(`Swash limits and S1/S2/S3 midpoint trims (${swashNeutralTrim(limits)} us) applied and verified.`);
    }, true));
    this.collective.addEventListener("input", () => {
      this.clearance.checked = false;
      const target = { ...this.controller.swashPosition, collective: this.collective.valueAsNumber / 100 };
      void this.run(() => this.controller.setSwashTarget(target));
    });
    this.sweepButton.addEventListener("click", () => void this.run(async () => {
      if (!this.clearance.checked) throw new Error("Confirm sweep clearance first.");
      await this.controller.startSwashSweep(this.radius.valueAsNumber / 100, this.duration.valueAsNumber);
    }));
    this.sweepOpen.addEventListener("click", () => {
      this.clearance.checked = false;
      this.sweepDialog.showModal();
      this.render();
    });
    element("sweep-close").addEventListener("click", () => void this.run(() => this.closeSweep()));
    this.sweepDialog.addEventListener("cancel", (event) => {
      event.preventDefault();
      void this.run(() => this.closeSweep());
    });
    for (const input of [this.radius, this.duration]) {
      input.addEventListener("input", () => { this.clearance.checked = false; });
    }
    this.secured.addEventListener("change", () => {
      if (!this.secured.checked && this.ownsOperation) void this.run(() => this.stop());
      this.render();
    });
    this.clearance.addEventListener("change", () => this.render());
    this.pad.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 || this.controller.phase !== "swash") return;
      this.pad.setPointerCapture(event.pointerId);
      this.movePointer(event);
    });
    this.pad.addEventListener("pointermove", (event) => {
      if (this.pad.hasPointerCapture(event.pointerId)) this.movePointer(event);
    });
    this.unsubscribe = telemetry.onRecord((record) => {
      const status = telemetry.status;
      if (record.message === "SERVO_OUTPUT_RAW" && record.direction === "rx" && record.fields.port === 0 && status
        && record.system_id === status.target_system && record.component_id === status.target_component) {
        this.servoValues.textContent = [1, 2, 3, 9].map((channel) => {
          const value = record.fields[`servo${channel}_raw`];
          return `S${channel}  ${typeof value === "number" && Number.isFinite(value) ? `${value} us` : "unavailable"}`;
        }).join("\n");
        this.sweepServos.textContent = this.servoValues.textContent;
      }
    });
    this.unsubscribeGeneration = telemetry.onGeneration(() => {
      this.loadEpoch++;
      this.geometry = null;
      this.loaded = false;
      this.geometryError = "Connection changed; reload limits.";
      this.secured.checked = false;
      this.clearance.checked = false;
      this.render();
    });
    document.addEventListener("keydown", this.escape);
    this.render();
  }

  get isOpen(): boolean { return this.opened; }

  open(): void {
    this.opened = true;
    this.timer = setInterval(() => this.render(), 100);
    void this.run(() => this.load(), true);
  }

  async close(): Promise<void> {
    this.opened = false;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    if (this.sweepDialog.open) this.sweepDialog.close();
    if (this.ownsOperation) await this.run(() => this.stop());
    this.secured.checked = false;
    this.clearance.checked = false;
  }

  dispose(): void {
    void this.close();
    this.unsubscribe();
    this.unsubscribeGeneration();
    document.removeEventListener("keydown", this.escape);
  }

  private async stop(): Promise<void> {
    await this.controller.stop();
    this.ownsOperation = false;
    this.clearance.checked = false;
  }

  private async closeSweep(): Promise<void> {
    if (this.ownsOperation) await this.stop();
    this.sweepDialog.close();
    this.clearance.checked = false;
  }

  private async load(): Promise<void> {
    const epoch = ++this.loadEpoch;
    this.loaded = false;
    this.geometry = null;
    this.geometryError = null;
    try {
      const limits = await this.controller.readSwashLimits();
      if (epoch !== this.loadEpoch) throw new Error("Connection changed; reload limits.");
      this.showLimits(limits);
      const geometry = await this.controller.readSwashGeometry();
      if (epoch !== this.loadEpoch) throw new Error("Connection changed; reload limits.");
      this.geometry = geometry;
      this.loaded = true;
    } catch (error) {
      if (epoch === this.loadEpoch) this.loaded = false;
      if (epoch === this.loadEpoch) this.geometryError = error instanceof Error ? error.message : String(error);
      throw error;
    }
  }

  private showLimits(limits: SwashLimits): void {
    this.min.value = String(limits.min);
    this.max.value = String(limits.max);
    this.cyclic.value = String(limits.cyclic);
    this.limitsChanged(limits);
  }

  private movePointer(event: PointerEvent): void {
    const bounds = this.pad.getBoundingClientRect();
    let roll = (event.clientX - bounds.left) / bounds.width * 2 - 1;
    let pitch = 1 - (event.clientY - bounds.top) / bounds.height * 2;
    const magnitude = Math.max(1, Math.hypot(roll, pitch));
    roll /= magnitude;
    pitch /= magnitude;
    void this.run(() => this.move(roll, pitch));
  }

  private async move(roll: number, pitch: number): Promise<void> {
    await this.controller.setSwashTarget({ roll, pitch, collective: this.collective.valueAsNumber / 100 });
  }

  private async run(action: () => Promise<unknown>, busy = false): Promise<void> {
    this.error = null;
    if (busy) this.busy = true;
    this.render();
    try {
      await action();
    } catch (error) {
      this.error = error instanceof Error ? error.message : String(error);
      this.write(`Swash calibration: ${this.error}`, "error");
    } finally {
      if (busy) this.busy = false;
      this.render();
    }
  }

  private render(): void {
    const phase = this.controller.phase;
    if (phase === "idle") this.ownsOperation = false;
    const active = phase === "swash";
    const status = this.telemetry.status;
    const ready = !!status?.connected && status.ready && !isArmed(status) && !this.compassActive();
    this.status.textContent = this.error ?? (active
      ? `SERVOS LIVE${this.controller.swashSweeping ? " - sweeping" : ""}`
      : phase === "idle" ? "Read-only - movement disabled" : `Controller: ${phase}`)
      + (this.controller.swashServoFresh ? "" : " - servo telemetry stale");
    this.status.classList.toggle("error", this.error !== null);
    this.enable.disabled = this.busy || phase !== "idle" || !ready || !this.loaded
      || !this.secured.checked || !this.controller.swashServoFresh;
    this.stopButton.disabled = !this.ownsOperation && !active;
    this.apply.disabled = this.busy || phase !== "idle" || !ready || !this.loaded || !this.secured.checked;
    this.reload.disabled = this.busy || phase !== "idle" || !ready;
    for (const input of [this.min, this.max, this.cyclic]) input.disabled = this.busy || phase !== "idle";
    this.collective.disabled = !active;
    this.pad.setAttribute("aria-disabled", String(!active));
    for (const button of this.motionButtons) button.disabled = !active;
    this.sweepButton.disabled = !active || !this.clearance.checked || this.controller.swashSweeping;
    this.sweepOpen.disabled = !active;
    this.radius.disabled = this.duration.disabled = this.controller.swashSweeping;
    const target = this.controller.swashPosition;
    if (active) this.collective.value = String(target.collective * 100);
    else if (phase === "idle") this.collective.value = "50";
    this.position.textContent = `Roll ${(target.roll * 100).toFixed(0)}% / pitch ${(target.pitch * 100).toFixed(0)}% / collective ${(target.collective * 100).toFixed(0)}%`;
    this.sweepPosition.textContent = this.position.textContent;
    for (const dot of [this.dot, this.sweepDot]) {
      dot.style.left = `${(target.roll + 1) * 50}%`;
      dot.style.top = `${(1 - target.pitch) * 50}%`;
    }
    this.servoValues.classList.toggle("stale", !this.controller.swashServoFresh);
    this.servoValues.title = this.controller.swashServoFresh ? "Live servo PWM" : "STALE - no valid swash output in the last 2 s";
    this.sweepServos.classList.toggle("stale", !this.controller.swashServoFresh);
    this.sweepStatus.textContent = this.error ?? (this.controller.swashSweeping
      ? `Sweeping at fixed collective ${(target.collective * 100).toFixed(0)}%`
      : active ? `Center cyclic before starting. Collective ${(target.collective * 100).toFixed(0)}%` : "Stopped - movement disabled");
    try {
      const trim = swashNeutralTrim({
        min: this.min.valueAsNumber, max: this.max.valueAsNumber, cyclic: this.cyclic.valueAsNumber,
      });
      const servos = this.geometry?.servos;
      this.neutral.textContent = `Apply neutral: ${trim} us for S1/S2/S3. ` + (servos
        ? `Loaded trims: ${servos.map((servo) => servo.trim).join(" / ")} us`
          + (servos.every((servo) => servo.trim === trim) ? " (consistent)." : " (differ from draft midpoint).")
        : "Reload to verify loaded trims.");
    } catch (error) {
      this.neutral.textContent = `Neutral unavailable: ${error instanceof Error ? error.message : String(error)}`;
    }
    if (!this.geometry) {
      this.envelope.textContent = this.geometryError ? `Unavailable: ${this.geometryError}` : "Loading mixer calibration...";
    } else {
      try {
        const range = swashServoEnvelope({
          min: this.min.valueAsNumber, max: this.max.valueAsNumber, cyclic: this.cyclic.valueAsNumber,
        }, this.geometry);
        this.envelope.textContent = range.map((servo, index) =>
          `S${index + 1} ${servo.min.toFixed(1)}..${servo.max.toFixed(1)}${servo.clipped ? " (clipped)" : ""}`,
        ).join("\n");
      } catch (error) {
        this.envelope.textContent = `Unavailable: ${error instanceof Error ? error.message : String(error)}`;
      }
    }
  }
}
