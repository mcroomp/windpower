import { CSV_INPUTS, csvSeries, type CsvModel } from "./csv-model";

const COLORS = ["#78b7ff", "#f0c36b", "#71ddb4", "#c792ea"];
const LINES = CSV_INPUTS.flatMap((input, index) => [
  { input, label: `${input} raw`, color: COLORS[index]!, smooth: false },
  { input, label: `${input} 1 s mean`, color: COLORS[index]!, smooth: true },
]);

export function drawCsvChart(canvas: HTMLCanvasElement, model: CsvModel, cutoff: number): void {
  const width = Math.max(320, canvas.clientWidth);
  const height = 240;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.font = "10px ui-monospace, 'Cascadia Code', monospace";
  ctx.textAlign = "left";
  let legendX = 56, legendY = 15;
  for (const line of LINES) {
    const itemWidth = 21 + ctx.measureText(line.label).width + 18;
    if (legendX > 56 && legendX + itemWidth > width - 10) {
      legendX = 56;
      legendY += 20;
    }
    ctx.strokeStyle = line.color;
    ctx.lineWidth = line.smooth ? 2 : 1;
    ctx.setLineDash(line.smooth ? [5, 3] : []);
    ctx.beginPath(); ctx.moveTo(legendX, legendY - 3); ctx.lineTo(legendX + 15, legendY - 3); ctx.stroke();
    ctx.fillStyle = line.color;
    ctx.fillText(line.label, legendX + 21, legendY);
    legendX += itemWidth;
  }
  const top = legendY + 18;
  ctx.setLineDash([]);
  const groups = csvSeries(model.samples).map((group) => group.filter((point) => point.time >= cutoff));
  const points = groups.flat();
  let t0 = Infinity, t1 = -Infinity, min = Infinity, max = -Infinity;
  for (const point of points) {
    t0 = Math.min(t0, point.time);
    t1 = Math.max(t1, point.time);
    for (const line of LINES) {
      const value = line.smooth ? point.means[line.input] : point[line.input];
      if (value !== null) { min = Math.min(min, value); max = Math.max(max, value); }
    }
  }
  ctx.fillStyle = "#8fa2bc";
  if (!points.length || !Number.isFinite(min)) {
    ctx.fillText("No timed A1/A2/B1/B2 samples to plot", 56, top + 30);
    return;
  }
  const pad = Math.max((max - min) * 0.08, Math.abs(max) * 0.01, 0.01);
  min -= pad; max += pad;
  const span = Math.max(t1 - t0, 0.001);
  const left = 56, plotWidth = width - 66, plotHeight = height - top - 40;
  const x = (time: number) => left + (time - t0) / span * plotWidth;
  const y = (value: number) => top + (max - value) / (max - min) * plotHeight;
  for (let tick = 0; tick <= 4; tick++) {
    const fraction = tick / 4;
    const py = top + fraction * plotHeight;
    ctx.strokeStyle = "rgba(70, 90, 121, 0.35)";
    ctx.beginPath(); ctx.moveTo(left, py); ctx.lineTo(width - 10, py); ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText((max - fraction * (max - min)).toPrecision(4), left - 6, py + 3);
    ctx.textAlign = tick === 0 ? "left" : tick === 4 ? "right" : "center";
    ctx.fillText(((t1 - t0) * fraction).toFixed(2), left + fraction * plotWidth, height - 23);
  }
  ctx.textAlign = "center";
  ctx.fillText("Estimated acquisition elapsed (s) · raw ADC counts", left + plotWidth / 2, height - 6);
  for (const line of LINES) {
    ctx.strokeStyle = line.color;
    ctx.fillStyle = line.color;
    ctx.lineWidth = line.smooth ? 2 : 1;
    ctx.setLineDash(line.smooth ? [5, 3] : []);
    for (const group of groups) {
      ctx.beginPath();
      let connected = false;
      for (const point of group) {
        const value = line.smooth ? point.means[line.input] : point[line.input];
        if (value === null) {
          if (line.smooth) connected = false;
          continue;
        }
        if (connected) ctx.lineTo(x(point.time), y(value));
        else ctx.moveTo(x(point.time), y(value));
        connected = true;
        ctx.fillRect(x(point.time) - 1, y(value) - 1, 2, 2);
      }
      ctx.stroke();
    }
  }
  ctx.setLineDash([]);
}
