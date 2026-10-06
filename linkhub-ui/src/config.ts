import apBaseParm from "../../tests/sitl/copter-heli.parm?raw";
import rawesCommonParm from "../../tests/sitl/rawes_common_defaults.parm?raw";
import rawesHardwareParm from "../../hardware/rawes_hardware_defaults.parm?raw";
import {
  compareConfig,
  matchesTarget,
  mergeConfigTargets,
  parseParm,
  type ConfigProvider,
  type ConfigRow,
} from "./config-core";

export {
  compareConfig,
  matchesTarget,
  parseParm,
  type ConfigProvider,
  type ConfigRow,
};

export function configSources(all: boolean): string[] {
  return [
    ...(all ? ["tests/sitl/copter-heli.parm"] : []),
    "tests/sitl/rawes_common_defaults.parm",
    "hardware/rawes_hardware_defaults.parm",
  ];
}

/** Later files override earlier ones; mirrors `calibrate config` targets. */
export function configTargets(all: boolean): Map<string, number> {
  const files = all
    ? [apBaseParm, rawesCommonParm, rawesHardwareParm]
    : [rawesCommonParm, rawesHardwareParm];
  return mergeConfigTargets(files);
}

export const browserConfigProvider: ConfigProvider = {
  sources: configSources,
  targets: configTargets,
};
