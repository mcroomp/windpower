import { describe, expect, it } from "vitest";

import { LinkHubApi } from "../src/api";
import { CommandRunner, tokenizeCommand } from "../src/commands";
import { PassiveController } from "../src/passive";
import { TelemetryStore } from "../src/telemetry";

describe("shared command runner", () => {
  it("tokenizes quoted browser terminal commands for both runtimes", () => {
    expect(tokenizeCommand('run passive --trim "thr=0.4" --duration 5')).toEqual([
      "run",
      "passive",
      "--trim",
      "thr=0.4",
      "--duration",
      "5",
    ]);
  });

  it("renders help without a browser DOM or LinkHub connection", async () => {
    const output: string[] = [];
    const api = new LinkHubApi();
    const telemetry = new TelemetryStore(api);
    const passive = new PassiveController(api, telemetry, () => undefined, () => undefined);
    const runner = new CommandRunner(
      api,
      telemetry,
      passive,
      (message) => output.push(message),
      {
        sources: () => [],
        targets: () => new Map(),
      },
    );

    await runner.executeTokens(["help"]);

    expect(output).toContain("status");
    expect(output).toContain("stop                     apply canonical safe-off");
  });
});
