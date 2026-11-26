import type { CameraPhone, CameraStatus } from "./types";

export const RECORDING_STALLED_MS = 10_000;

/** Heartbeats alone do not prove that video is reaching the experiment file. */
export function recordingProblem(phone: CameraPhone, camera: CameraStatus, nowMs = Date.now()): string | null {
  const connected = camera.phones.find((candidate) => candidate.stream === phone.stream);
  if (!connected) return `${phone.label}: disconnected`;
  if (!camera.experiment_id || connected.recording !== camera.experiment_id) {
    return `${phone.label}: waiting for recording`;
  }
  const receiving = camera.videos.some((video) =>
    video.stream.startsWith(`${phone.stream}-`)
    && !video.finished
    && video.chunks >= 2
    && video.bytes > 0
    && nowMs - video.updated_at_ns / 1e6 <= RECORDING_STALLED_MS,
  );
  return receiving ? null : `${phone.label}: waiting for live video saved to disk`;
}
