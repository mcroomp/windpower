import { LinkHubApi } from "./api";
import type { VehicleSceneOptions } from "./scene";

export const sceneOptions: VehicleSceneOptions = {};

export function createLinkHubApi(): LinkHubApi {
  return new LinkHubApi();
}
