import { vi } from "vitest";

/** CommandRunner experiment dependency with an experiment already running. */
export function activeExperiment(active = true) {
  return {
    requireActive: vi.fn(async (action: string) => {
      if (!active) throw new Error(`${action} refused: start an experiment first`);
    }),
    refresh: vi.fn(async () => null),
    start: vi.fn(),
    stop: vi.fn(),
  };
}
