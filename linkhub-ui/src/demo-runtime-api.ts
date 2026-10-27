import { LinkHubApi } from "./api";
import { DemoBackend } from "./demo-backend";
import recording from "./demo/pumping-cycle.json";
import type { VehicleSceneOptions } from "./scene";

export const sceneOptions: VehicleSceneOptions = {
  followVehicle: true,
  groundHeight: recording.groundHeight,
};

export function createLinkHubApi(): LinkHubApi {
  document.title = "RAWES LinkHub / Recorded SITL pumping cycle";
  const backend = new DemoBackend();
  const heading = document.querySelector("h1");
  if (heading) {
    const update = () => {
      const { time, phase } = backend.playback;
      heading.textContent = `${phase} / ${time.toFixed(1)} of ${recording.duration.toFixed(1)} s`;
    };
    update();
    window.setInterval(update, 250);
  }
  const label = document.querySelector(".scene-panel .eyebrow");
  if (label) {
    label.textContent = "RAWES / RECORDED SITL / READ-ONLY";
  }
  return new LinkHubApi("", backend.fetch);
}
