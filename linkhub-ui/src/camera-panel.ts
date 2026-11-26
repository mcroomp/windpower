import type { LinkHubApi } from "./api";
import { recordingProblem, RECORDING_STALLED_MS } from "./camera-recording";
import type { CameraPhone, CameraStatus, CameraVideo } from "./types";

const POLL_MS = 3_000;

function describeVideo(video: CameraVideo, nowMs: number): string {
  const size = `${(video.bytes / 1e6).toFixed(1)} MB in ${video.chunks} chunks`;
  if (video.finished) return `${video.file}: ${size}, finished`;
  const idle = nowMs - video.updated_at_ns / 1e6;
  return `${video.file}: ${size}, ${idle > RECORDING_STALLED_MS ? `no data for ${Math.round(idle / 1_000)} s` : "recording"}`;
}

function describePhone(phone: CameraPhone, camera: CameraStatus): string {
  if (!camera.experiment_id) return `${phone.label}: ready`;
  return recordingProblem(phone, camera) ?? `${phone.label}: recording verified`;
}

function listItems(lines: string[], empty: string): HTMLLIElement[] {
  return (lines.length ? lines : [empty]).map((line) => {
    const item = document.createElement("li");
    item.textContent = line;
    return item;
  });
}

/**
 * Header button and dialog for phone cameras. Phones connect once by scanning
 * the QR code and then record every experiment automatically; the button
 * shows how many are connected.
 */
export class CameraPanel {
  private readonly poll: ReturnType<typeof setInterval>;
  private readonly qr: HTMLElement;
  private readonly link: HTMLAnchorElement;
  private readonly reason: HTMLElement;
  private readonly phones: HTMLUListElement;
  private readonly videos: HTMLUListElement;
  private qrSvg: string | null = null;

  constructor(
    private readonly button: HTMLButtonElement,
    private readonly dialog: HTMLDialogElement,
    private readonly api: Pick<LinkHubApi, "cameraStatus">,
  ) {
    this.qr = this.element("#camera-qr");
    this.link = this.element("#camera-url");
    this.reason = this.element("#camera-reason");
    this.phones = this.element("#camera-phones");
    this.videos = this.element("#camera-videos");
    this.element<HTMLButtonElement>("#camera-close").addEventListener("click", () => dialog.close());
    button.addEventListener("click", () => {
      if (!dialog.open) dialog.showModal();
      void this.refresh();
    });
    this.poll = setInterval(() => void this.refresh(), POLL_MS);
    void this.refresh();
  }

  dispose(): void {
    clearInterval(this.poll);
  }

  private async refresh(): Promise<void> {
    let camera: CameraStatus;
    try {
      camera = await this.api.cameraStatus();
    } catch (error) {
      this.button.textContent = "📷 Cameras ?";
      this.button.classList.toggle("recording", false);
      this.reason.hidden = false;
      this.reason.textContent = `Camera status unavailable: ${error instanceof Error ? error.message : String(error)}`;
      return;
    }
    const recording = camera.experiment_id
      ? camera.phones.filter((phone) => recordingProblem(phone, camera) === null)
      : [];
    const verified = camera.experiment_id && camera.phones.length > 0
      ? ` · ${recording.length}/${camera.phones.length} verified`
      : "";
    this.button.textContent = `📷 ${camera.phones.length} camera${camera.phones.length === 1 ? "" : "s"}${verified}`;
    this.button.classList.toggle("recording", recording.length > 0 && recording.length === camera.phones.length);
    this.button.title = camera.phones.length
      ? camera.phones.map((phone) => describePhone(phone, camera)).join("\n")
      : "No phone camera connected; click to show the QR code";
    if (!this.dialog.open) return;

    if (camera.qr_svg !== this.qrSvg) {
      this.qrSvg = camera.qr_svg;
      if (camera.qr_svg) this.qr.innerHTML = camera.qr_svg;
      else this.qr.replaceChildren();
    }
    this.qr.hidden = !camera.qr_svg;
    this.link.hidden = !camera.url;
    if (camera.url) {
      this.link.href = camera.url;
      this.link.textContent = camera.url;
    }
    this.reason.hidden = !camera.reason;
    this.reason.textContent = camera.reason ?? "";
    this.phones.replaceChildren(...listItems(
      camera.phones.map((phone) => describePhone(phone, camera)),
      "No phone connected.",
    ));
    const now = Date.now();
    this.videos.hidden = camera.experiment_id === null;
    this.videos.replaceChildren(...listItems(
      camera.videos.map((video) => describeVideo(video, now)),
      "No recording received for this experiment yet.",
    ));
  }

  private element<T extends Element = HTMLElement>(selector: string): T {
    const found = this.dialog.querySelector<T>(selector);
    if (!found) throw new Error(`Missing camera element ${selector}`);
    return found;
  }
}
