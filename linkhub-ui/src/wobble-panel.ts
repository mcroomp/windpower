import {
  analyzeDominantMode,
  type DominantWobbleMode,
  type WobbleAxis,
  type WobbleModel,
  type WobbleSpectrum,
  WOBBLE_WINDOW_S,
} from "./wobble-model";

const AXES: readonly WobbleAxis[] = ["roll", "pitch", "yaw"];
const AXIS_COLORS: Record<WobbleAxis, string> = {
  roll: "#78b7ff",
  pitch: "#f0c36b",
  yaw: "#c792ea",
};
const REDRAW_MS = 250;

export class WobblePanel {
  private readonly canvases = new Map<WobbleAxis, HTMLCanvasElement>();
  private readonly componentLists = new Map<WobbleAxis, HTMLElement>();
  private readonly modeCanvas: HTMLCanvasElement;
  private readonly modeDetails: HTMLElement;
  private readonly status: HTMLElement;
  private readonly maxFrequency: HTMLSelectElement;
  private readonly maxAmplitude: HTMLSelectElement;
  private readonly observer: ResizeObserver;
  private frame = 0;
  private lastDraw = 0;
  private opened = false;

  constructor(private readonly root: HTMLElement, private readonly model: WobbleModel) {
    root.replaceChildren();
    const head = document.createElement("div");
    head.className = "wobble-head";
    this.status = document.createElement("div");
    this.status.className = "wobble-status";
    this.status.textContent = "Collecting samples…";
    const controls = document.createElement("label");
    controls.className = "wobble-controls";
    controls.append("Frequency range ");
    this.maxFrequency = document.createElement("select");
    this.maxFrequency.setAttribute("aria-label", "Maximum wobble frequency");
    for (const frequency of [20, 50, 100, 150]) {
      this.maxFrequency.add(new Option(`${frequency} Hz`, String(frequency), frequency === 100, frequency === 100));
    }
    controls.append(this.maxFrequency);
    controls.append("Amplitude ");
    this.maxAmplitude = document.createElement("select");
    this.maxAmplitude.setAttribute("aria-label", "Maximum wobble amplitude");
    for (const amplitude of [0.02, 0.05, 0.1, 0.25, 0.5, 1, 2]) {
      const selected = amplitude === 0.25;
      this.maxAmplitude.add(new Option(
        `${amplitude} rad/s`,
        String(amplitude),
        selected,
        selected,
      ));
    }
    controls.append(this.maxAmplitude);
    head.append(this.status, controls);

    const charts = document.createElement("div");
    charts.className = "wobble-charts";
    for (const axis of AXES) {
      const card = document.createElement("section");
      card.className = `wobble-axis wobble-${axis}`;
      const title = document.createElement("h3");
      title.textContent = `${axis.toUpperCase()} RATE`;
      const canvas = document.createElement("canvas");
      canvas.setAttribute("aria-label", `${axis} wobble frequency spectrum`);
      const components = document.createElement("div");
      components.className = "wobble-components";
      components.textContent = "Waiting for data";
      card.append(title, canvas, components);
      charts.append(card);
      this.canvases.set(axis, canvas);
      this.componentLists.set(axis, components);
    }
    const modeCard = document.createElement("section");
    modeCard.className = "wobble-axis wobble-mode";
    const modeTitle = document.createElement("h3");
    modeTitle.textContent = "DOMINANT MODE";
    this.modeCanvas = document.createElement("canvas");
    this.modeCanvas.setAttribute("aria-label", "Dominant wobble body axis");
    this.modeDetails = document.createElement("div");
    this.modeDetails.className = "wobble-mode-details";
    this.modeDetails.textContent = "Waiting for data";
    modeCard.append(modeTitle, this.modeCanvas, this.modeDetails);
    charts.append(modeCard);
    const note = document.createElement("div");
    note.className = "wobble-note";
    note.textContent = `Body-rate spectra use the latest ${WOBBLE_WINDOW_S} s of Pixhawk attitude telemetry. `
      + "All axes share the selected absolute amplitude scale; clipped peaks are marked at the top edge.";
    root.append(head, charts, note);
    this.maxFrequency.addEventListener("change", () => this.draw());
    this.maxAmplitude.addEventListener("change", () => this.draw());
    this.observer = new ResizeObserver(() => this.draw());
    this.observer.observe(charts);
  }

  get isOpen(): boolean {
    return this.opened;
  }

  open(): void {
    if (this.opened) {
      return;
    }
    this.opened = true;
    this.frame = requestAnimationFrame((time) => this.tick(time));
  }

  close(): void {
    this.opened = false;
    cancelAnimationFrame(this.frame);
  }

  dispose(): void {
    this.close();
    this.observer.disconnect();
    this.root.replaceChildren();
  }

  private tick(time: number): void {
    if (!this.opened) {
      return;
    }
    this.frame = requestAnimationFrame((next) => this.tick(next));
    if (time - this.lastDraw < REDRAW_MS) {
      return;
    }
    this.lastDraw = time;
    this.draw();
  }

  private draw(): void {
    if (!this.opened) {
      return;
    }
    const { spectra, state } = this.model.snapshot();
    const first = spectra.roll;
    const stale = this.model.ingestedAt === 0 || performance.now() - this.model.ingestedAt > 2_000;
    this.status.textContent = stale
      ? `No recent attitude telemetry${first ? " - showing last valid spectrum" : ""}`
      : state === "rebuilding"
      ? "Rebuilding after telemetry gap - showing last valid spectrum"
      : first
      ? `${first.durationS.toFixed(1)} s window · ${(first.coverage * 100).toFixed(0)}% coverage`
        + ` · ${first.sampleRateHz.toFixed(0)} Hz grid`
      : "Collecting samples…";
    const maxHz = Number(this.maxFrequency.value);
    const maxAmplitude = Number(this.maxAmplitude.value);
    for (const axis of AXES) {
      const spectrum = spectra[axis];
      this.drawSpectrum(
        this.canvases.get(axis) as HTMLCanvasElement,
        spectrum,
        axis,
        maxHz,
        maxAmplitude,
      );
      const components = this.componentLists.get(axis) as HTMLElement;
      const peaks = spectrum?.components.filter((component) => component.frequencyHz <= maxHz).slice(0, 3) ?? [];
      components.replaceChildren();
      if (peaks.length === 0) {
        components.textContent = spectrum ? "No peaks in range" : "Waiting for data";
        continue;
      }
      for (const [index, peak] of peaks.entries()) {
        const row = document.createElement("div");
        row.innerHTML = `<span>#${index + 1}</span><strong>${peak.frequencyHz.toFixed(2)} Hz</strong>`
          + `<span>${peak.amplitude.toFixed(3)} rad/s</span>`;
        components.append(row);
      }
    }
    const mode = analyzeDominantMode(spectra, maxHz);
    this.drawMode(mode);
  }

  private drawMode(mode: DominantWobbleMode | null): void {
    const canvas = this.modeCanvas;
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const center = { x: width / 2, y: height / 2 };
    const length = Math.min(width, height) * 0.32;
    const project = (roll: number, pitch: number, yaw: number) => ({
      x: center.x + length * (pitch * 0.86 - roll * 0.48),
      y: center.y + length * (yaw * 0.9 + pitch * 0.3 + roll * 0.3),
    });
    ctx.font = "9px ui-monospace, 'Cascadia Code', monospace";
    ctx.lineWidth = 1;
    const basis: [string, WobbleAxis, number, number, number][] = [
      ["ROLL X", "roll", 1, 0, 0],
      ["PITCH Y", "pitch", 0, 1, 0],
      ["YAW Z", "yaw", 0, 0, 1],
    ];
    for (const [label, axis, roll, pitch, yaw] of basis) {
      const point = project(roll, pitch, yaw);
      ctx.strokeStyle = "rgba(117, 145, 184, 0.45)";
      ctx.beginPath();
      ctx.moveTo(center.x, center.y);
      ctx.lineTo(point.x, point.y);
      ctx.stroke();
      ctx.fillStyle = AXIS_COLORS[axis];
      ctx.textAlign = point.x >= center.x ? "left" : "right";
      ctx.fillText(label, point.x + (point.x >= center.x ? 3 : -3), point.y);
    }
    if (!mode || mode.linearity < 0.5) {
      ctx.fillStyle = "#8fa2bc";
      ctx.textAlign = "center";
      ctx.fillText(mode ? "no single fixed axis" : "collecting", center.x, center.y - 8);
    } else {
      const positive = project(mode.axis.roll, mode.axis.pitch, mode.axis.yaw);
      const negative = project(-mode.axis.roll, -mode.axis.pitch, -mode.axis.yaw);
      ctx.strokeStyle = "#71ddb4";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(negative.x, negative.y);
      ctx.lineTo(positive.x, positive.y);
      ctx.stroke();
      ctx.fillStyle = "#71ddb4";
      ctx.beginPath();
      ctx.arc(positive.x, positive.y, 4, 0, 2 * Math.PI);
      ctx.fill();
    }
    if (!mode) {
      this.modeDetails.textContent = "Waiting for data";
      return;
    }
    const classification = mode.linearity >= 0.8
      ? "fixed-axis"
      : mode.linearity >= 0.5
      ? "elliptical"
      : "circular / multi-axis";
    const percent = (value: number) => `${Math.round(value * 100)}%`;
    this.modeDetails.innerHTML = [
      `<div><span>frequency</span><strong>${mode.frequencyHz.toFixed(2)} Hz</strong></div>`,
      `<div><span>magnitude</span><strong>${mode.amplitude.toFixed(3)} rad/s</strong></div>`,
      `<div><span>linearity</span><strong>${percent(mode.linearity)} · ${classification}</strong></div>`,
      `<div><span>axis share</span><strong>R ${percent(mode.contributions.roll)} · P ${percent(mode.contributions.pitch)} · Y ${percent(mode.contributions.yaw)}</strong></div>`,
    ].join("");
  }

  private drawSpectrum(
    canvas: HTMLCanvasElement,
    spectrum: WobbleSpectrum | null,
    axis: WobbleAxis,
    maxHz: number,
    maxAmplitude: number,
  ): void {
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const margin = { left: 42, right: 8, top: 8, bottom: 22 };
    const plotWidth = Math.max(1, width - margin.left - margin.right);
    const plotHeight = Math.max(1, height - margin.top - margin.bottom);
    ctx.font = "10px ui-monospace, 'Cascadia Code', monospace";
    ctx.strokeStyle = "rgba(70, 90, 121, 0.35)";
    ctx.fillStyle = "#8fa2bc";
    ctx.textAlign = "center";
    for (const fraction of [0, 0.25, 0.5, 0.75, 1]) {
      const x = margin.left + fraction * plotWidth;
      ctx.beginPath();
      ctx.moveTo(x, margin.top);
      ctx.lineTo(x, margin.top + plotHeight);
      ctx.stroke();
      ctx.fillText(String(Math.round(fraction * maxHz)), x, height - 6);
    }
    if (!spectrum) {
      ctx.fillText("collecting", margin.left + plotWidth / 2, margin.top + plotHeight / 2);
      return;
    }
    const visible = spectrum.frequenciesHz.map((frequency, index) => ({
      frequency,
      amplitude: spectrum.amplitudes[index] as number,
    })).filter((point) => point.frequency <= maxHz);
    ctx.textAlign = "right";
    ctx.fillText(maxAmplitude.toFixed(maxAmplitude < 0.1 ? 2 : 1), margin.left - 5, margin.top + 8);
    ctx.fillText("0", margin.left - 5, margin.top + plotHeight);
    ctx.strokeStyle = AXIS_COLORS[axis];
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    visible.forEach((point, index) => {
      const x = margin.left + point.frequency / maxHz * plotWidth;
      const y = margin.top + plotHeight
        - Math.min(1, point.amplitude / maxAmplitude) * plotHeight;
      if (index === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
    const clipped = visible.filter((point) => point.amplitude > maxAmplitude);
    if (clipped.length > 0) {
      ctx.fillStyle = "#f18a94";
      for (const point of clipped) {
        const x = margin.left + point.frequency / maxHz * plotWidth;
        ctx.beginPath();
        ctx.moveTo(x - 3, margin.top + 1);
        ctx.lineTo(x + 3, margin.top + 1);
        ctx.lineTo(x, margin.top + 6);
        ctx.closePath();
        ctx.fill();
      }
    }
  }
}
