import { LinkHubApi } from "./api";
import { CommandRunner, type CommandOutputKind } from "./commands";
import { nodeConfigProvider } from "./config-node";
import { PassiveController, type PassivePhase } from "./passive";
import { TelemetryStore } from "./telemetry";

interface CliOptions {
  server: string;
  command: string[];
}

function parseArgs(args: string[]): CliOptions {
  let server = "http://127.0.0.1:8999";
  const command: string[] = [];
  for (let index = 0; index < args.length; index += 1) {
    const token = args[index];
    if (token === "--server") {
      const value = args[index + 1];
      if (!value) {
        throw new Error("--server requires a URL");
      }
      server = new URL(value).toString();
      index += 1;
    } else {
      command.push(token as string);
    }
  }
  return { server, command };
}

function write(message: string, kind: CommandOutputKind = "info"): void {
  const prefix = kind === "fc" ? "[FC] " : kind === "error" ? "[ERROR] " : "";
  const stream = kind === "error" ? process.stderr : process.stdout;
  stream.write(`${prefix}${message}\n`);
}

async function main(): Promise<void> {
  const options = parseArgs(process.argv.slice(2));
  const command = options.command.length > 0 ? options.command : ["help"];
  const api = new LinkHubApi(options.server);
  const telemetry = new TelemetryStore(api);
  let phase: PassivePhase = "idle";
  let finishRun: (() => void) | null = null;
  const passive = new PassiveController(
    api,
    telemetry,
    write,
    (nextPhase) => {
      phase = nextPhase;
      write(`Passive phase: ${nextPhase}`);
      if (nextPhase === "idle") {
        finishRun?.();
      }
    },
  );
  const runner = new CommandRunner(api, telemetry, passive, write, nodeConfigProvider);
  const needsTelemetry = command[0] === "status"
    || (command[0] === "run" && command[1] === "passive");

  const stop = () => {
    void passive.stop().catch((error: unknown) => {
      write(error instanceof Error ? error.message : String(error), "error");
    });
  };
  process.once("SIGINT", stop);
  process.once("SIGTERM", stop);

  try {
    if (needsTelemetry) {
      await telemetry.start();
    }
    await runner.executeTokens(command);
    if (command[0] === "run" && command[1] === "passive" && phase !== "idle") {
      await new Promise<void>((resolve) => {
        finishRun = resolve;
        if (phase === "idle") {
          resolve();
        }
      });
    }
  } finally {
    telemetry.stop();
    process.removeListener("SIGINT", stop);
    process.removeListener("SIGTERM", stop);
  }
}

main().catch((error: unknown) => {
  if (!(error instanceof DOMException && error.name === "AbortError")) {
    write(error instanceof Error ? error.message : String(error), "error");
    process.exitCode = 1;
  }
});
