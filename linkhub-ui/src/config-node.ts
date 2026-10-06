import { readFileSync } from "node:fs";
import path from "node:path";
import { pathToFileURL } from "node:url";

import {
  mergeConfigTargets,
  type ConfigProvider,
} from "./config-core";

const SOURCES = [
  "tests/sitl/copter-heli.parm",
  "tests/sitl/rawes_common_defaults.parm",
  "hardware/rawes_hardware_defaults.parm",
];
const repoRoot = process.env.RAWES_REPO_ROOT
  ? pathToFileURL(`${path.resolve(process.env.RAWES_REPO_ROOT)}${path.sep}`)
  : new URL("../../", import.meta.url);

function sources(all: boolean): string[] {
  return all ? [...SOURCES] : SOURCES.slice(1);
}

export const nodeConfigProvider: ConfigProvider = {
  sources,
  targets(all: boolean): Map<string, number> {
    return mergeConfigTargets(
      sources(all).map((path) => readFileSync(new URL(path, repoRoot), "utf8")),
    );
  },
};
