import type { LinkHubApi } from "./api";
import type { ExperimentController } from "./experiment";
import type { ExperimentVideo } from "./types";

const REFRESH_MS = 2_000;
/** A recording without a new chunk for this long is shown as stalled. */
const STALLED_MS = 10_000;

function describeVideo(video: ExperimentVideo, nowMs: number): string {
  const size = `${(video.bytes / 1e6).toFixed(1)} MB in ${video.chunks} chunks`;
  if (video.finished) return `${video.file}: ${size}, finished`;
  const idle = nowMs - video.updated_at_ns / 1e6;
  return `${video.file}: ${size}, ${idle > STALLED_MS ? `no data for ${Math.round(idle / 1_000)} s` : "recording"}`;
}

/**
 * Header button and dialog that show the running experiment's phone camera
 * QR code and the phone recordings LinkHub has received.
 */
export class ExperimentCameraPanel {
  private readonly poll: ReturnType<typeof setInterval>;
  private readonly unsubscribe: () => void;
  private readonly qr: HTMLElement;
  private readonly link: HTMLAnchorElement;
  private readonly reason: HTMLElement;
  private readonly videos: HTMLUListElement;

  constructor(
    private readonly button: HTMLButtonElement,
    private readonly dialog: HTMLDialogElement,
    private readonly experiment: ExperimentController,
    private readonly api: Pick<LinkHubApi, "experimentCamera">,
  ) {
    this.qr = this.element("#camera-qr");
    this.link = this.element("#camera-url");
    this.reason = this.element("#camera-reason");
    this.videos = this.element("#camera-videos");
    this.element<HTMLButtonElement>("#camera-close").addEventListener("click", () => dialog.close());
    button.addEventListener("click", () => this.open());
    this.unsubscribe = experiment.onChange(() => this.render());
    this.poll = setInterval(() => {
      if (dialog.open) void this.refresh();
    }, REFRESH_MS);
    this.render();
  }

  open(): void {
    if (!this.experiment.active) return;
    if (!this.dialog.open) this.dialog.showModal();
    void this.refresh();
  }

  dispose(): void {
    clearInterval(this.poll);
    this.unsubscribe();
  }

  private render(): void {
    this.button.hidden = this.experiment.active === null;
    if (!this.experiment.active && this.dialog.open) this.dialog.close();
  }

  private async refresh(): Promise<void> {
    try {
      const camera = await this.api.experimentCamera();
      if (camera.qr_svg) this.qr.innerHTML = camera.qr_svg;
      else this.qr.replaceChildren();
      this.qr.hidden = !camera.qr_svg;
      this.link.hidden = !camera.url;
      if (camera.url) {
        this.link.href = camera.url;
        this.link.textContent = camera.url;
      }
      this.reason.hidden = !camera.reason;
      this.reason.textContent = camera.reason ?? "";
      const now = Date.now();
      const items = camera.videos.map((video) => {
        const item = document.createElement("li");
        item.textContent = describeVideo(video, now);
        return item;
      });
      if (items.length === 0) {
        const item = document.createElement("li");
        item.textContent = "No phone is recording yet.";
        items.push(item);
      }
      this.videos.replaceChildren(...items);
    } catch (error) {
      this.reason.hidden = false;
      this.reason.textContent = `Camera link unavailable: ${error instanceof Error ? error.message : String(error)}`;
    }
  }

  private element<T extends Element = HTMLElement>(selector: string): T {
    const found = this.dialog.querySelector<T>(selector);
    if (!found) throw new Error(`Missing camera element ${selector}`);
    return found;
  }
}
