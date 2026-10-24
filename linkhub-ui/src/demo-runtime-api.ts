import { LinkHubApi } from "./api";
import { configTargets } from "./config";
import { DemoBackend } from "./demo-backend";

export function createLinkHubApi(): LinkHubApi {
  document.title = "RAWES LinkHub / Offline demo";
  const heading = document.querySelector("h1");
  if (heading) {
    heading.textContent = "Demo vehicle state";
  }
  const label = document.querySelector(".scene-panel .eyebrow");
  if (label) {
    label.textContent = "RAWES / OFFLINE DEMO";
  }
  const backend = new DemoBackend(configTargets(true));
  return new LinkHubApi("", backend.fetch);
}
