import { columnBins, lowerBound, upperBound } from "./yaw-math";
import {
  computeMetrics,
  isSaturated,
  WINDOW_MAX_S,
  type Samples,
  type YawEvent,
  type YawMetrics,
  type YawModel,
} from "./yaw-model";

const COLORS = {
  setpoint: "#78b7ff",
  measured: "#ffca28",
  error: "rgba(255, 89, 100, 0.24)",
  trim: "#3fbf8f",
  ff: "#4dd0e1",
  p: "#78b7ff",
  i: "#f0c36b",
  d: "#c792ea",
  command: "#e8edf5",
  observer: "#ff8fa3",
  limit: "rgba(241, 138, 148, 0.8)",
  muted: "#5c7aa6",
  grid: "rgba(70, 90, 121, 0.35)",
  text: "#8fa2bc",
  strong: "#cbd8e9",
};
const EVENT_COLORS: Record<YawEvent["kind"], string> = {
  armed: "#71ddb4",
  disarmed: "#f18a94",
  mode: "#78b7ff",
  text: "#f0c36b",
};
const MARGIN = { left: 56, right: 10, axis: 22, gap: 8, title: 15 };
const FONT = "10px ui-monospace, 'Cascadia Code', monospace";
const GAP_BREAK_S = 0.3;
const REDRAW_MS = 33;
const METRICS_MS = 250;
const HOVER_TOLERANCE_S = 0.5;

interface Rect { x: number; y: number; w: number; h: number }
interface View { t0: number; t1: number }
interface Range { min: number; max: number }
interface LineStyle { color: string; width?: number; dash?: number[]; step?: boolean; alpha?: number }
interface LegendItem { label: string; color: string; dash?: number[] }
interface Resampled<K extends string> { t: number[]; cols: Record<K, number[]> }

function finite(value: number): boolean {
  return Number.isFinite(value);
}

function fixed(value: number, digits: number): string {
  return finite(value) ? value.toFixed(digits) : "—";
}

function wrapDegrees(angle: number): number {
  return angle - 360 * Math.round(angle / 360);
}

function niceStep(span: number, count: number): number {
  const raw = Math.max(span, 1e-12) / count;
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const normalised = raw / magnitude;
  return (normalised < 1.5 ? 1 : normalised < 3 ? 2 : normalised < 7 ? 5 : 10) * magnitude;
}

function ticksFor(range: Range, count = 3): { values: number[]; step: number } {
  const step = niceStep(range.max - range.min, count);
  const values: number[] = [];
  for (let value = Math.ceil(range.min / step) * step; value <= range.max + step * 1e-9; value += step) {
    values.push(Math.abs(value) < step * 1e-9 ? 0 : value);
  }
  return { values, step };
}

function decimals(step: number): number {
  return Math.max(0, Math.min(4, -Math.floor(Math.log10(step) + 1e-9)));
}

interface Column { t: readonly number[]; v: readonly number[] }

function visibleRange(
  columns: Column[],
  view: View,
  include: number[],
  minSpan: number,
): Range | null {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (const column of columns) {
    const hi = Math.min(column.t.length, upperBound(column.t, view.t1) + 1);
    for (let index = Math.max(0, lowerBound(column.t, view.t0) - 1); index < hi; index += 1) {
      const value = column.v[index] as number;
      if (finite(value)) {
        min = Math.min(min, value);
        max = Math.max(max, value);
      }
    }
  }
  if (!finite(min)) {
    return null;
  }
  for (const value of include) {
    min = Math.min(min, value);
    max = Math.max(max, value);
  }
  if (max - min < minSpan) {
    const middle = (max + min) / 2;
    min = middle - minSpan / 2;
    max = middle + minSpan / 2;
  }
  const pad = (max - min) * 0.08;
  return { min: min - pad, max: max + pad };
}

/** Samples inside the view, or per-pixel-column means when there are far more than pixels. */
function resample<K extends string>(
  set: Samples<K>,
  names: readonly K[],
  view: View,
  width: number,
): Resampled<K> {
  const times = set.t;
  const lo = Math.max(0, lowerBound(times, view.t0) - 1);
  const hi = Math.min(times.length, upperBound(times, view.t1) + 1);
  const columns = Object.fromEntries(names.map((name) => [name, [] as number[]])) as Record<K, number[]>;
  const result: Resampled<K> = { t: [], cols: columns };
  if (hi - lo <= width * 2) {
    for (let index = lo; index < hi; index += 1) {
      result.t.push(times[index] as number);
      for (const name of names) {
        columns[name].push(set.v(name)[index] as number);
      }
    }
    return result;
  }
  const count = Math.max(1, Math.floor(width));
  const bins = columnBins(times, view.t0, view.t1, count);
  bins.forEach((bin, column) => {
    if (bin.end <= bin.start) {
      return;
    }
    result.t.push(view.t0 + ((column + 0.5) * (view.t1 - view.t0)) / count);
    for (const name of names) {
      let sum = 0;
      let used = 0;
      for (let index = bin.start; index < bin.end; index += 1) {
        const value = set.v(name)[index] as number;
        if (finite(value)) {
          sum += value;
          used += 1;
        }
      }
      columns[name].push(used > 0 ? sum / used : Number.NaN);
    }
  });
  return result;
}

/** Index ranges [start, end) separated by gaps longer than GAP_BREAK_S. */
function runs(times: readonly number[]): [number, number][] {
  const result: [number, number][] = [];
  let start = 0;
  for (let index = 1; index <= times.length; index += 1) {
    if (index === times.length || (times[index] as number) - (times[index - 1] as number) > GAP_BREAK_S) {
      if (index > start) {
        result.push([start, index]);
      }
      start = index;
    }
  }
  return result;
}

function valueAt<K extends string>(set: Samples<K>, name: K, t: number): number {
  const times = set.t;
  const upper = lowerBound(times, t);
  let best = -1;
  let bestDistance = HOVER_TOLERANCE_S;
  for (const candidate of [upper - 1, upper]) {
    if (candidate < 0 || candidate >= times.length) {
      continue;
    }
    const distance = Math.abs((times[candidate] as number) - t);
    if (distance <= bestDistance) {
      bestDistance = distance;
      best = candidate;
    }
  }
  return best < 0 ? Number.NaN : (set.v(name)[best] as number);
}

function lastFinite<K extends string>(set: Samples<K>, name: K, maxAgeS: number, now: number): number {
  const values = set.v(name);
  const times = set.t;
  for (let index = values.length - 1; index >= 0; index -= 1) {
    if (now - (times[index] as number) > maxAgeS) {
      break;
    }
    const value = values[index] as number;
    if (finite(value)) {
      return value;
    }
  }
  return Number.NaN;
}

export class YawPanel {
  private readonly root: HTMLElement;
  private readonly canvas: HTMLCanvasElement;
  private readonly dial: HTMLCanvasElement;
  private readonly wrap: HTMLElement;
  private readonly tip: HTMLElement;
  private readonly pauseButton: HTMLButtonElement;
  private readonly windowSelect: HTMLSelectElement;
  private readonly metricsEl: HTMLElement;
  private readonly observer: ResizeObserver;
  private frame = 0;
  private lastDraw = 0;
  private lastMetrics = 0;
  private opened = false;
  private paused = false;
  private viewEnd = 0;
  private windowS = 30;
  private hover: { x: number; t: number } | null = null;
  private drag: { x: number; viewEnd: number } | null = null;
  private view: View = { t0: 0, t1: 1 };
  private plotLeft = MARGIN.left;
  private plotWidth = 1;
  private dirty = true;

  constructor(container: HTMLElement, private readonly model: YawModel) {
    this.root = container;
    container.replaceChildren();
    const head = document.createElement("div");
    head.className = "yaw-head";
    this.metricsEl = document.createElement("div");
    this.metricsEl.className = "yaw-readout";
    this.dial = document.createElement("canvas");
    this.dial.className = "yaw-dial";
    this.dial.setAttribute("aria-hidden", "true");
    const controls = document.createElement("div");
    controls.className = "yaw-controls";
    this.pauseButton = document.createElement("button");
    this.pauseButton.type = "button";
    this.pauseButton.textContent = "Pause";
    this.pauseButton.addEventListener("click", () => this.setPaused(!this.paused));
    this.windowSelect = document.createElement("select");
    this.windowSelect.setAttribute("aria-label", "Yaw chart time window");
    for (const seconds of [10, 30, 60]) {
      this.windowSelect.add(new Option(`${seconds} s`, String(seconds), seconds === 30, seconds === 30));
    }
    this.windowSelect.addEventListener("change", () => {
      this.windowS = Number(this.windowSelect.value);
      this.dirty = true;
    });
    controls.append(this.pauseButton, this.windowSelect);
    head.append(this.metricsEl, this.dial, controls);

    this.wrap = document.createElement("div");
    this.wrap.className = "yaw-chart-wrap";
    this.canvas = document.createElement("canvas");
    this.canvas.className = "yaw-chart";
    this.tip = document.createElement("div");
    this.tip.className = "yaw-tip";
    this.tip.hidden = true;
    this.wrap.append(this.canvas, this.tip);

    const note = document.createElement("div");
    note.className = "yaw-note";
    note.textContent = "Dashed = setpoint, solid = measured. PID, RPM and ESC frames carry no vehicle "
      + "timestamp, so their alignment with attitude is approximate (about 50 ms). Drag while paused to scroll.";
    container.append(head, this.wrap, note);

    this.canvas.addEventListener("mousemove", (event) => this.onMove(event));
    this.canvas.addEventListener("mouseleave", () => {
      this.hover = null;
      this.tip.hidden = true;
      this.dirty = true;
    });
    this.canvas.addEventListener("mousedown", (event) => {
      if (this.paused) {
        this.drag = { x: event.clientX, viewEnd: this.viewEnd };
      }
    });
    window.addEventListener("mouseup", () => {
      this.drag = null;
    });
    this.observer = new ResizeObserver(() => {
      this.dirty = true;
    });
    this.observer.observe(this.wrap);
  }

  get isOpen(): boolean {
    return this.opened;
  }

  open(): void {
    if (this.opened) {
      return;
    }
    this.opened = true;
    this.dirty = true;
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

  private liveNow(): number {
    const extra = Math.min(1, Math.max(0, (performance.now() - this.model.ingestedAt) / 1000));
    return this.model.latestT + (this.model.ingestedAt > 0 ? extra : 0);
  }

  private setPaused(paused: boolean): void {
    this.paused = paused;
    this.viewEnd = this.liveNow();
    this.pauseButton.textContent = paused ? "Resume" : "Pause";
    this.pauseButton.setAttribute("aria-pressed", String(paused));
    this.dirty = true;
  }

  private onMove(event: MouseEvent): void {
    const bounds = this.canvas.getBoundingClientRect();
    const x = event.clientX - bounds.left;
    if (this.drag) {
      const shift = ((event.clientX - this.drag.x) / Math.max(1, this.plotWidth)) * this.windowS;
      this.viewEnd = Math.min(
        this.liveNow(),
        Math.max(this.model.latestT - WINDOW_MAX_S + this.windowS, this.drag.viewEnd - shift),
      );
    }
    if (x >= this.plotLeft && x <= this.plotLeft + this.plotWidth) {
      const t = this.view.t0 + ((x - this.plotLeft) / this.plotWidth) * (this.view.t1 - this.view.t0);
      this.hover = { x, t };
      this.showTip(event.clientX - bounds.left, event.clientY - bounds.top, t);
    } else {
      this.hover = null;
      this.tip.hidden = true;
    }
    this.dirty = true;
  }

  private showTip(x: number, y: number, t: number): void {
    const m = this.model;
    const nearby = m.events.filter((event) => Math.abs(event.t - t) < 0.4).map((event) => `▲ ${event.label}`);
    const lines = [
      `t ${fixed(t - this.view.t1, 2)} s`,
      `heading  ${fixed(valueAt(m.att, "hdgT", t), 1)}° → ${fixed(valueAt(m.att, "hdg", t), 1)}°  err ${fixed(valueAt(m.att, "hdgErr", t), 2)}°`,
      `rate     ${fixed(valueAt(m.pid, "rateDes", t), 3)} → ${fixed(valueAt(m.pid, "rateAct", t), 3)} rad/s`,
      `lua rate ${fixed(valueAt(m.diag, "rateLua", t), 3)}  att tgt ${fixed(valueAt(m.tgt, "rate", t), 3)}`,
      `trim ${fixed(valueAt(m.pid, "trim", t), 3)}  FF ${fixed(valueAt(m.pid, "ff", t), 3)}  P ${fixed(valueAt(m.pid, "p", t), 3)}  I ${fixed(valueAt(m.pid, "i", t), 3)}  D ${fixed(valueAt(m.pid, "d", t), 3)}`,
      `command ${fixed(valueAt(m.pid, "cmd", t), 3)}  applied ${fixed(valueAt(m.servo, "u", t), 3)}  observer ${fixed(valueAt(m.diag, "ueq", t), 3)}`,
      `motor ${fixed(valueAt(m.rpm, "rpm", t), 0)} RPM  ESC ${fixed(valueAt(m.esc, "amps", t), 2)} A`,
      ...nearby,
    ];
    this.tip.textContent = lines.join("\n");
    this.tip.hidden = false;
    const flip = x > this.wrap.clientWidth * 0.55;
    this.tip.style.left = flip ? "" : `${x + 14}px`;
    this.tip.style.right = flip ? `${this.wrap.clientWidth - x + 14}px` : "";
    this.tip.style.top = `${Math.max(4, Math.min(y, this.wrap.clientHeight - 130))}px`;
  }

  private tick(time: number): void {
    if (!this.opened) {
      return;
    }
    this.frame = requestAnimationFrame((next) => this.tick(next));
    const live = !this.paused;
    if (time - this.lastDraw < REDRAW_MS || (!live && !this.dirty)) {
      return;
    }
    this.lastDraw = time;
    this.dirty = false;
    const now = this.paused ? this.viewEnd : this.liveNow();
    this.view = { t0: now - this.windowS, t1: now };
    this.draw(now);
    if (time - this.lastMetrics >= METRICS_MS) {
      this.lastMetrics = time;
      this.renderMetrics(now);
    }
  }

  private renderMetrics(now: number): void {
    const metrics = computeMetrics(this.model, now);
    const stale = this.model.ingestedAt === 0 || performance.now() - this.model.ingestedAt > 2000;
    const item = (label: string, value: string) =>
      `<span class="metric"><span class="metric-label">${label}</span><span class="metric-value">${value}</span></span>`;
    const state = stale
      ? "no recent data"
      : `${metrics.armed ? "ARMED" : "disarmed"} · ${metrics.mode ?? "n/a"}`;
    this.metricsEl.innerHTML = [
      `<span class="metric state ${metrics.armed && !stale ? "armed" : ""}">${state}</span>`,
      item("rate err RMS", metrics.rateErrRms === null ? "—" : `${metrics.rateErrRms.toFixed(3)} rad/s`),
      item("heading err RMS", metrics.hdgErrRms === null ? "—" : `${metrics.hdgErrRms.toFixed(2)}°`),
      item("I share", metrics.iShare === null ? "—" : `${(metrics.iShare * 100).toFixed(0)}%`),
      item("saturated", metrics.satPercent === null ? "—" : `${metrics.satPercent.toFixed(1)}%`),
      item("cmd→RPM lag", metrics.lagMs === null ? "—" : `${metrics.lagMs.toFixed(0)} ms`),
    ].join("");
    this.drawDial(metrics, now);
  }

  private drawDial(metrics: YawMetrics, now: number): void {
    const size = 84;
    const dpr = window.devicePixelRatio || 1;
    if (this.dial.width !== size * dpr) {
      this.dial.width = size * dpr;
      this.dial.height = size * dpr;
      this.dial.style.width = `${size}px`;
      this.dial.style.height = `${size}px`;
    }
    const ctx = this.dial.getContext("2d");
    if (!ctx) {
      return;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size, size);
    const centre = size / 2;
    const radius = 30;
    const m = this.model;
    const error = lastFinite(m.att, "hdgErr", 1, now);
    const rate = lastFinite(m.pid, "rateAct", 1, now);
    const throttle = lastFinite(m.servo, "u", 1, now);
    const command = lastFinite(m.pid, "cmd", 1, now);
    const arc = (r: number, from: number, to: number, color: string, width: number) => {
      ctx.beginPath();
      ctx.arc(centre, centre, r, from, to, to < from);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineCap = "round";
      ctx.stroke();
    };
    const top = -Math.PI / 2;
    // Motor throttle: outer ring filled clockwise from the bottom-left.
    const ringStart = Math.PI * 0.75;
    arc(radius + 6, ringStart, ringStart + Math.PI * 1.5, "rgba(70, 90, 121, 0.45)", 3);
    if (finite(throttle)) {
      const saturated = finite(command) && metrics.armed && isSaturated(command);
      arc(radius + 6, ringStart, ringStart + Math.PI * 1.5 * Math.min(1, Math.max(0, throttle)),
        saturated ? "#ff5964" : COLORS.measured, 3);
    }
    arc(radius, 0, Math.PI * 2, "#aeb9c9", 1.5);
    // Target tick at the top.
    ctx.strokeStyle = COLORS.setpoint;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centre, centre - radius - 4);
    ctx.lineTo(centre, centre - radius + 6);
    ctx.stroke();
    if (finite(error)) {
      // Error arc and needle: +/-30 degrees spans +/-90 degrees of dial.
      const angle = top + (Math.max(-30, Math.min(30, error)) / 30) * (Math.PI / 2);
      arc(radius - 6, top, angle, "rgba(255, 89, 100, 0.85)", 4);
      ctx.strokeStyle = COLORS.measured;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centre, centre);
      ctx.lineTo(centre + Math.cos(angle) * (radius - 3), centre + Math.sin(angle) * (radius - 3));
      ctx.stroke();
    }
    if (finite(rate)) {
      // Yaw rate arrow inside the ring: 1 rad/s sweeps 60 degrees.
      const sweep = Math.max(-2, Math.min(2, rate)) * (Math.PI / 3);
      if (Math.abs(sweep) > 0.03) {
        arc(radius - 14, top, top + sweep, COLORS.setpoint, 2);
      }
    }
    ctx.fillStyle = COLORS.strong;
    ctx.font = FONT;
    ctx.textAlign = "center";
    ctx.fillText(finite(error) ? `${error.toFixed(1)}°` : "—", centre, centre + 20);
  }

  private draw(now: number): void {
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(1, this.wrap.clientWidth);
    const height = Math.max(1, this.wrap.clientHeight);
    if (this.canvas.width !== Math.round(width * dpr) || this.canvas.height !== Math.round(height * dpr)) {
      this.canvas.width = Math.round(width * dpr);
      this.canvas.height = Math.round(height * dpr);
      this.canvas.style.width = `${width}px`;
      this.canvas.style.height = `${height}px`;
    }
    const ctx = this.canvas.getContext("2d");
    if (!ctx) {
      return;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.font = FONT;
    this.plotLeft = MARGIN.left;
    this.plotWidth = Math.max(1, width - MARGIN.left - MARGIN.right);
    const panelHeight = (height - MARGIN.axis - MARGIN.gap * 3) / 4;
    const rect = (index: number): Rect => ({
      x: this.plotLeft,
      y: index * (panelHeight + MARGIN.gap),
      w: this.plotWidth,
      h: Math.max(20, panelHeight),
    });
    this.drawHeading(ctx, rect(0));
    this.drawRate(ctx, rect(1));
    this.drawCommand(ctx, rect(2));
    this.drawSpeed(ctx, rect(3));
    this.drawEvents(ctx, height);
    this.drawAxis(ctx, height, now);
    if (this.hover) {
      ctx.strokeStyle = "rgba(232, 237, 245, 0.55)";
      ctx.lineWidth = 1;
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.moveTo(this.hover.x, 0);
      ctx.lineTo(this.hover.x, height - MARGIN.axis);
      ctx.stroke();
    }
  }

  private xOf(rect: Rect, t: number): number {
    return rect.x + ((t - this.view.t0) / (this.view.t1 - this.view.t0)) * rect.w;
  }

  private frameAndAxes(
    ctx: CanvasRenderingContext2D,
    rect: Rect,
    title: string,
    unit: string,
    range: Range | null,
    legend: LegendItem[],
    formatTick: (value: number) => string = (value) => String(value),
  ): ((value: number) => number) {
    ctx.fillStyle = "rgba(255, 255, 255, 0.025)";
    ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
    ctx.strokeStyle = "#27334a";
    ctx.lineWidth = 1;
    ctx.setLineDash([]);
    ctx.strokeRect(rect.x + 0.5, rect.y + 0.5, rect.w - 1, rect.h - 1);
    ctx.textAlign = "left";
    ctx.fillStyle = COLORS.strong;
    const heading = `${title}  ${unit}`;
    ctx.fillText(heading, rect.x + 6, rect.y + MARGIN.title - 3);
    this.drawLegend(ctx, rect, ctx.measureText(heading).width, legend);
    if (range === null) {
      ctx.textAlign = "center";
      ctx.fillStyle = COLORS.muted;
      ctx.fillText("no data", rect.x + rect.w / 2, rect.y + rect.h / 2 + 4);
      return () => rect.y + rect.h / 2;
    }
    const yOf = (value: number) => rect.y + rect.h - ((value - range.min) / (range.max - range.min)) * rect.h;
    const { values, step } = ticksFor(range);
    ctx.textAlign = "right";
    for (const value of values) {
      const y = yOf(value);
      if (y < rect.y + MARGIN.title || y > rect.y + rect.h - 2) {
        continue;
      }
      ctx.strokeStyle = COLORS.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(rect.x, Math.round(y) + 0.5);
      ctx.lineTo(rect.x + rect.w, Math.round(y) + 0.5);
      ctx.stroke();
      ctx.fillStyle = COLORS.text;
      ctx.fillText(formatTick(Number(value.toFixed(decimals(step) + 1))), rect.x - 6, y + 3);
    }
    return yOf;
  }

  /** Legend after the title; wraps to a second row, with a backing so lines under it stay legible. */
  private drawLegend(
    ctx: CanvasRenderingContext2D,
    rect: Rect,
    titleWidth: number,
    legend: LegendItem[],
  ): void {
    const rowHeight = 12;
    const limit = rect.x + rect.w - 6;
    let x = rect.x + 6 + titleWidth + 16;
    let row = 0;
    ctx.textAlign = "left";
    for (const item of legend) {
      const width = ctx.measureText(item.label).width + 14 + 4 + 12;
      if (x + width > limit) {
        row += 1;
        x = rect.x + 6;
      }
      if (row > 1) {
        return;
      }
      const baseline = rect.y + MARGIN.title - 3 + row * rowHeight;
      ctx.fillStyle = "rgba(8, 13, 22, 0.72)";
      ctx.fillRect(x - 2, baseline - 10, width - 6, rowHeight);
      ctx.strokeStyle = item.color;
      ctx.lineWidth = 2;
      ctx.setLineDash(item.dash ?? []);
      ctx.beginPath();
      ctx.moveTo(x, baseline - 3);
      ctx.lineTo(x + 14, baseline - 3);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = COLORS.text;
      ctx.fillText(item.label, x + 18, baseline);
      x += width;
    }
  }

  private line(
    ctx: CanvasRenderingContext2D,
    rect: Rect,
    yOf: (value: number) => number,
    column: Column,
    style: LineStyle,
  ): void {
    const { t, v } = column;
    const lo = Math.max(0, lowerBound(t, this.view.t0) - 1);
    const hi = Math.min(t.length, upperBound(t, this.view.t1) + 1);
    if (hi <= lo) {
      return;
    }
    ctx.save();
    ctx.beginPath();
    ctx.rect(rect.x, rect.y, rect.w, rect.h);
    ctx.clip();
    ctx.strokeStyle = style.color;
    ctx.lineWidth = style.width ?? 1.4;
    ctx.lineJoin = "round";
    ctx.setLineDash(style.dash ?? []);
    ctx.globalAlpha = style.alpha ?? 1;
    const dense = hi - lo > rect.w * 2;
    if (dense) {
      this.denseLine(ctx, rect, yOf, column);
    } else {
      ctx.beginPath();
      let previousT = Number.NEGATIVE_INFINITY;
      let previousY = 0;
      let pen = false;
      for (let index = lo; index < hi; index += 1) {
        const value = v[index] as number;
        const time = t[index] as number;
        if (!finite(value)) {
          pen = false;
          continue;
        }
        const x = this.xOf(rect, time);
        const y = yOf(value);
        if (!pen || time - previousT > GAP_BREAK_S * (style.step ? 4 : 1)) {
          ctx.moveTo(x, y);
        } else if (style.step) {
          ctx.lineTo(x, previousY);
          ctx.lineTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
        pen = true;
        previousT = time;
        previousY = y;
      }
      if (style.step && pen) {
        ctx.lineTo(rect.x + rect.w, previousY);
      }
      ctx.stroke();
    }
    ctx.restore();
  }

  private denseLine(
    ctx: CanvasRenderingContext2D,
    rect: Rect,
    yOf: (value: number) => number,
    column: Column,
  ): void {
    const bins = columnBins(column.t, this.view.t0, this.view.t1, Math.max(1, Math.floor(rect.w)));
    const baseAlpha = ctx.globalAlpha;
    const means: ([number, number] | null)[] = [];
    ctx.beginPath();
    bins.forEach((bin, index) => {
      let low = Number.POSITIVE_INFINITY;
      let high = Number.NEGATIVE_INFINITY;
      let sum = 0;
      let used = 0;
      for (let sample = bin.start; sample < bin.end; sample += 1) {
        const value = column.v[sample] as number;
        if (finite(value)) {
          low = Math.min(low, value);
          high = Math.max(high, value);
          sum += value;
          used += 1;
        }
      }
      if (used === 0) {
        means.push(null);
        return;
      }
      const x = rect.x + index + 0.5;
      ctx.moveTo(x, yOf(low));
      ctx.lineTo(x, yOf(high));
      means.push([x, yOf(sum / used)]);
    });
    ctx.globalAlpha = baseAlpha * 0.35;
    ctx.stroke();
    ctx.globalAlpha = baseAlpha;
    ctx.beginPath();
    let pen = false;
    for (const point of means) {
      if (point === null) {
        pen = false;
      } else if (pen) {
        ctx.lineTo(point[0], point[1]);
      } else {
        ctx.moveTo(point[0], point[1]);
        pen = true;
      }
    }
    ctx.stroke();
  }

  private fillBetween(
    ctx: CanvasRenderingContext2D,
    rect: Rect,
    yOf: (value: number) => number,
    times: readonly number[],
    upper: readonly number[],
    lower: readonly number[],
    color: string,
  ): void {
    ctx.save();
    ctx.beginPath();
    ctx.rect(rect.x, rect.y, rect.w, rect.h);
    ctx.clip();
    ctx.fillStyle = color;
    for (const [start, end] of runs(times)) {
      ctx.beginPath();
      let started = false;
      for (let index = start; index < end; index += 1) {
        const value = upper[index] as number;
        if (finite(value)) {
          const x = this.xOf(rect, times[index] as number);
          if (started) {
            ctx.lineTo(x, yOf(value));
          } else {
            ctx.moveTo(x, yOf(value));
            started = true;
          }
        }
      }
      for (let index = end - 1; index >= start; index -= 1) {
        const value = lower[index] as number;
        if (finite(value) && started) {
          ctx.lineTo(this.xOf(rect, times[index] as number), yOf(value));
        }
      }
      ctx.closePath();
      ctx.fill();
    }
    ctx.restore();
  }

  private drawHeading(ctx: CanvasRenderingContext2D, rect: Rect): void {
    const att = this.model.att;
    const columns: Column[] = [
      { t: att.t, v: att.v("hdg") },
      { t: att.t, v: att.v("hdgT") },
    ];
    const range = visibleRange(columns, this.view, [], 2);
    const yOf = this.frameAndAxes(ctx, rect, "HEADING", "deg", range, [
      { label: "target", color: COLORS.setpoint, dash: [4, 3] },
      { label: "actual", color: COLORS.measured },
      { label: "error", color: "rgba(255, 89, 100, 0.7)" },
    ], (value) => String(Math.round(wrapDegrees(value) * 10) / 10));
    if (range === null) {
      return;
    }
    const resampled = resample(att, ["hdg", "hdgT"] as const, this.view, rect.w);
    this.fillBetween(ctx, rect, yOf, resampled.t, resampled.cols.hdg, resampled.cols.hdgT, COLORS.error);
    this.line(ctx, rect, yOf, columns[1] as Column, { color: COLORS.setpoint, dash: [4, 3], width: 1.3 });
    this.line(ctx, rect, yOf, columns[0] as Column, { color: COLORS.measured, width: 1.5 });
  }

  private drawRate(ctx: CanvasRenderingContext2D, rect: Rect): void {
    const m = this.model;
    const desired = { t: m.pid.t, v: m.pid.v("rateDes") };
    const achieved = { t: m.pid.t, v: m.pid.v("rateAct") };
    const target = { t: m.tgt.t, v: m.tgt.v("rate") };
    const lua = { t: m.diag.t, v: m.diag.v("rateLua") };
    const range = visibleRange([desired, achieved, target, lua], this.view, [0], 0.1);
    const yOf = this.frameAndAxes(ctx, rect, "YAW RATE", "rad/s", range, [
      { label: "desired", color: COLORS.setpoint, dash: [4, 3] },
      { label: "achieved", color: COLORS.measured },
      { label: "attitude target", color: COLORS.muted, dash: [1, 3] },
      { label: "Lua", color: COLORS.d },
    ]);
    if (range === null) {
      return;
    }
    const resampled = resample(m.pid, ["rateDes", "rateAct"] as const, this.view, rect.w);
    this.fillBetween(ctx, rect, yOf, resampled.t, resampled.cols.rateDes, resampled.cols.rateAct, COLORS.error);
    this.line(ctx, rect, yOf, target, { color: COLORS.muted, dash: [1, 3], width: 1.2 });
    this.line(ctx, rect, yOf, lua, { color: COLORS.d, step: true, width: 1.2 });
    this.line(ctx, rect, yOf, desired, { color: COLORS.setpoint, dash: [4, 3], width: 1.3 });
    this.line(ctx, rect, yOf, achieved, { color: COLORS.measured, width: 1.5 });
  }

  private drawCommand(ctx: CanvasRenderingContext2D, rect: Rect): void {
    const m = this.model;
    const command = { t: m.pid.t, v: m.pid.v("cmd") };
    const applied = { t: m.servo.t, v: m.servo.v("u") };
    const observer = { t: m.diag.t, v: m.diag.v("ueq") };
    const range = visibleRange([command, applied, observer], this.view, [0, 1], 0.2);
    const yOf = this.frameAndAxes(ctx, rect, "MOTOR COMMAND", "0–1 throttle", range, [
      { label: "trim", color: COLORS.trim },
      { label: "FF", color: COLORS.ff },
      { label: "P", color: COLORS.p },
      { label: "I", color: COLORS.i },
      { label: "D", color: COLORS.d },
      { label: "command", color: COLORS.command },
      { label: "applied", color: COLORS.measured },
      { label: "observer", color: COLORS.observer, dash: [4, 3] },
    ], (value) => String(Math.round(value * 100) / 100));
    if (range === null) {
      if (!m.armed) {
        ctx.fillStyle = COLORS.muted;
        ctx.textAlign = "center";
        ctx.fillText("disarmed: trim and PID inactive", rect.x + rect.w / 2, rect.y + rect.h / 2 + 18);
      }
      return;
    }
    const names = ["trim", "ff", "p", "i", "d"] as const;
    const bands: Record<(typeof names)[number], string> = {
      trim: COLORS.trim, ff: COLORS.ff, p: COLORS.p, i: COLORS.i, d: COLORS.d,
    };
    const stack = resample(m.pid, names, this.view, rect.w);
    const sum = (index: number, count: number) => {
      let total = 0;
      for (let band = 0; band < count; band += 1) {
        const value = stack.cols[names[band] as (typeof names)[number]][index] as number;
        total += finite(value) ? value : 0;
      }
      return total;
    };
    names.forEach((name, band) => {
      const top = stack.t.map((_, index) => sum(index, band + 1));
      const bottom = stack.t.map((_, index) => sum(index, band));
      this.fillBetween(ctx, rect, yOf, stack.t, top, bottom, `${bands[name]}55`);
    });
    // Anything outside 0..1 is clipped by ArduPilot's DDFP output.
    ctx.save();
    ctx.beginPath();
    ctx.rect(rect.x, rect.y, rect.w, rect.h);
    ctx.clip();
    ctx.fillStyle = "rgba(241, 138, 148, 0.08)";
    ctx.fillRect(rect.x, yOf(0), rect.w, rect.y + rect.h - yOf(0));
    ctx.fillRect(rect.x, rect.y, rect.w, Math.max(0, yOf(1) - rect.y));
    ctx.strokeStyle = COLORS.limit;
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 3]);
    for (const limit of [0, 1]) {
      ctx.beginPath();
      ctx.moveTo(rect.x, Math.round(yOf(limit)) + 0.5);
      ctx.lineTo(rect.x + rect.w, Math.round(yOf(limit)) + 0.5);
      ctx.stroke();
    }
    ctx.restore();
    this.line(ctx, rect, yOf, command, { color: COLORS.command, width: 1, alpha: 0.9 });
    this.line(ctx, rect, yOf, observer, { color: COLORS.observer, dash: [4, 3], step: true, width: 1.2 });
    this.line(ctx, rect, yOf, applied, { color: COLORS.measured, width: 1.6 });
    this.drawSaturation(ctx, rect);
  }

  private drawSaturation(ctx: CanvasRenderingContext2D, rect: Rect): void {
    const pid = this.model.pid;
    const times = pid.t;
    const commands = pid.v("cmd");
    const marked = new Set<number>();
    for (let index = Math.max(0, lowerBound(times, this.view.t0)); index < times.length; index += 1) {
      const t = times[index] as number;
      if (t > this.view.t1) {
        break;
      }
      const value = commands[index] as number;
      if (finite(value) && isSaturated(value) && this.model.isArmedAt(t)) {
        marked.add(Math.floor(this.xOf(rect, t)));
      }
    }
    ctx.fillStyle = "#ff5964";
    for (const x of marked) {
      ctx.fillRect(x, rect.y + rect.h - 4, 1, 4);
    }
  }

  private drawSpeed(ctx: CanvasRenderingContext2D, rect: Rect): void {
    const m = this.model;
    const rpm = { t: m.rpm.t, v: m.rpm.v("rpm") };
    const applied = { t: m.servo.t, v: m.servo.v("u") };
    const autoRange = visibleRange([rpm], this.view, [0], 20);
    const range = autoRange === null
      ? null
      : { min: 0, max: Math.max(20, autoRange.max) };
    const yOf = this.frameAndAxes(ctx, rect, "MOTOR SPEED", "RPM", range, [
      { label: "measured RPM", color: COLORS.setpoint },
      { label: "applied throttle (right axis)", color: COLORS.measured, dash: [4, 3] },
    ], (value) => String(Math.round(value)));
    if (range === null) {
      return;
    }
    // The throttle shares the panel on its own 0..1 scale.
    const throttleY = (value: number) => rect.y + rect.h - value * (rect.h - MARGIN.title) - 2;
    this.line(ctx, rect, throttleY, applied, { color: COLORS.measured, dash: [4, 3], width: 1.2, alpha: 0.9 });
    this.line(ctx, rect, yOf, rpm, { color: COLORS.setpoint, width: 1.6 });
    ctx.fillStyle = COLORS.text;
    ctx.textAlign = "left";
    ctx.fillText("1", rect.x + rect.w + 2, throttleY(1) + 3);
    ctx.fillText("0", rect.x + rect.w + 2, throttleY(0) + 3);
  }

  private drawEvents(ctx: CanvasRenderingContext2D, height: number): void {
    const bottom = height - MARGIN.axis;
    for (const event of this.model.events) {
      if (event.t < this.view.t0 || event.t > this.view.t1) {
        continue;
      }
      const x = this.plotLeft + ((event.t - this.view.t0) / (this.view.t1 - this.view.t0)) * this.plotWidth;
      const color = EVENT_COLORS[event.kind];
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.55;
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0);
      ctx.lineTo(x + 0.5, bottom);
      ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.setLineDash([]);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(x - 4, bottom + 7);
      ctx.lineTo(x + 4, bottom + 7);
      ctx.lineTo(x, bottom + 1);
      ctx.closePath();
      ctx.fill();
    }
  }

  private drawAxis(ctx: CanvasRenderingContext2D, height: number, now: number): void {
    const step = niceStep(this.windowS, 6);
    ctx.fillStyle = COLORS.text;
    ctx.textAlign = "center";
    for (let age = 0; age <= this.windowS; age += step) {
      const x = this.plotLeft + this.plotWidth - (age / this.windowS) * this.plotWidth;
      ctx.fillText(age === 0 ? (this.paused ? `${fixed(now, 0)} s` : "now") : `−${age.toFixed(age < 1 ? 1 : 0)} s`, x, height - 4);
    }
  }
}
