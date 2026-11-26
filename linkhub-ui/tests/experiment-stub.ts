import { vi } from "vitest";
import type { ExperimentInfo } from "../src/types";

/** CommandRunner experiment dependency with an experiment already running. */
export function activeExperiment(active = true) {
  return {
    requireActive: vi.fn(async (action: string) => {
      if (!active) throw new Error(`${action} refused: start an experiment first`);
    }),
    refresh: vi.fn<() => Promise<ExperimentInfo | null>>(async () => null),
    videoCheck: { state: "ready" as const, detail: "No cameras connected" },
    start: vi.fn(),
    stop: vi.fn(),
    mark: vi.fn(async () => undefined),
  };
}
