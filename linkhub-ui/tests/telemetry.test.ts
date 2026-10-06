import { afterEach, describe, expect, it, vi } from "vitest";
import { LinkHubApi } from "../src/api";
import { TelemetryStore } from "../src/telemetry";
import { DISPLAY_TELEMETRY_RATES } from "../src/telemetry-rates";
import type { LinkHubStatus } from "../src/types";

function status(generation: string, ready = true): LinkHubStatus {
  return {
    connected: ready, ready, generation, cursor: "v1:0", connection: "serial:test",
    clock_epoch: 1, target_system: 1, target_component: 1, base_mode: 0,
    custom_mode: 0, system_status: 3, latest_time_boot_ms: 0,
    received_messages: 0, transmitted_messages: 0,
    received_bytes: 0, transmitted_bytes: 0, rx_bps: null, tx_bps: null,
  };
}

afterEach(() => vi.restoreAllMocks());

describe("live display telemetry", () => {
  it("requests streams at startup without starting a passive run", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    vi.spyOn(api, "readMessages").mockImplementation(() => new Promise(() => {}));
    const telemetry = new TelemetryStore(api);
    const onStatus = vi.fn();
    telemetry.onStatus(onStatus);
    await telemetry.start();
    expect(rates).toHaveBeenCalledExactlyOnceWith(DISPLAY_TELEMETRY_RATES);
    expect(onStatus).toHaveBeenCalledExactlyOnceWith(status("first"));
    telemetry.stop();
  });

  it("reports stream configuration failures rather than showing success", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status").mockResolvedValue(status("first"));
    vi.spyOn(api, "setMessageRates").mockRejectedValue(new Error("stream request rejected"));
    const telemetry = new TelemetryStore(api);
    await expect(telemetry.start()).rejects.toThrow("stream request rejected");
    expect(telemetry.status).toBeNull();
    telemetry.stop();
  });

  it("configures once per ready generation and defers while disconnected", async () => {
    const api = new LinkHubApi();
    vi.spyOn(api, "status")
      .mockResolvedValueOnce(status("first"))
      .mockResolvedValueOnce(status("first"))
      .mockResolvedValueOnce(status("second", false))
      .mockResolvedValueOnce(status("second"));
    const rates = vi.spyOn(api, "setMessageRates").mockResolvedValue({});
    let reads = 0;
    vi.spyOn(performance, "now").mockImplementation(() => reads * 1_100);
    vi.spyOn(api, "readMessages").mockImplementation(async () => {
      reads += 1;
      if (reads > 3) {
        return new Promise(() => {});
      }
      return { records: [], next_cursor: "v1:0" };
    });
    const telemetry = new TelemetryStore(api);
    const snapshots: LinkHubStatus[] = [];
    const unsubscribe = telemetry.onStatus((snapshot) => snapshots.push(snapshot));
    await telemetry.start();
    await vi.waitFor(() => expect(rates).toHaveBeenCalledTimes(2));
    expect(rates.mock.calls).toEqual([
      [DISPLAY_TELEMETRY_RATES], [DISPLAY_TELEMETRY_RATES],
    ]);
    expect(snapshots.map((snapshot) => snapshot.generation)).toEqual([
      "first", "first", "second", "second",
    ]);
    unsubscribe();
    telemetry.stop();
  });
});
