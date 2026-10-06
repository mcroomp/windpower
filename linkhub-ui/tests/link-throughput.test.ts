import { describe, expect, it } from "vitest";
import { MavState } from "../src/generated/protocol";
import {
  formatKbps,
  formatLinkThroughput,
  messageThroughputRows,
} from "../src/link-throughput";
import { mavEnum } from "../src/mav";
import type { LinkHubStatus } from "../src/types";

const status: LinkHubStatus = {
  connected: true, ready: true, generation: "first", cursor: "v1:0", connection: "test",
  clock_epoch: 1, target_system: 1, target_component: 1, base_mode: "",
  custom_mode: 0, system_status: mavEnum(MavState.STANDBY), latest_time_boot_ms: 0,
  received_messages: 0, transmitted_messages: 0,
  received_bytes: 0, transmitted_bytes: 0, rx_bps: null, tx_bps: null,
  rx_bps_by_message: {}, tx_bps_by_message: {},
};

describe("per-message throughput", () => {
  it("merges RX and TX, sorts busiest first, and drops idle or invalid rows", () => {
    const rows = messageThroughputRows({
      ...status,
      rx_bps_by_message: { ATTITUDE: 4_000, HEARTBEAT: 296, RPM: 0, BAD: Number.NaN },
      tx_bps_by_message: { HEARTBEAT: 168, REQUEST_DATA_STREAM: 8_000 },
    });
    expect(rows).toEqual([
      { message: "REQUEST_DATA_STREAM", rxBps: 0, txBps: 8_000 },
      { message: "ATTITUDE", rxBps: 4_000, txBps: 0 },
      { message: "HEARTBEAT", rxBps: 296, txBps: 168 },
    ]);
  });

  it("shows nothing while disconnected", () => {
    expect(messageThroughputRows({
      ...status, connected: false, rx_bps_by_message: { ATTITUDE: 4_000 },
    })).toEqual([]);
    expect(messageThroughputRows(null)).toEqual([]);
  });

  it("formats kbps with two decimals", () => {
    expect(formatKbps(4_000)).toBe("4.00");
    expect(formatKbps(296)).toBe("0.30");
    expect(formatKbps(0)).toBe("\u2014");
  });
});

describe("LinkHub-owned throughput display", () => {
  it("formats decimal bits per second without estimating client traffic", () => {
    expect(formatLinkThroughput({ ...status, rx_bps: 18_400, tx_bps: 1_200 }))
      .toBe("RX 18.4 kbps \u00b7 TX 1.2 kbps");
    expect(formatLinkThroughput({ ...status, rx_bps: 2_500_000, tx_bps: 168 }))
      .toBe("RX 2.5 Mbps \u00b7 TX 168 bps");
  });

  it("distinguishes idle rates from unavailable samples", () => {
    expect(formatLinkThroughput({ ...status, rx_bps: 0, tx_bps: 0 }))
      .toBe("RX 0 bps \u00b7 TX 0 bps");
    expect(formatLinkThroughput(status)).toBe("RX \u2014 \u00b7 TX \u2014");
  });

  it("does not show old rates while disconnected", () => {
    expect(formatLinkThroughput({ ...status, connected: false, rx_bps: 18_400, tx_bps: 1_200 }))
      .toBe("RX \u2014 \u00b7 TX \u2014");
    expect(formatLinkThroughput(null)).toBe("RX \u2014 \u00b7 TX \u2014");
  });

  it("rejects invalid measurements", () => {
    expect(formatLinkThroughput({ ...status, rx_bps: Number.NaN, tx_bps: -1 }))
      .toBe("RX \u2014 \u00b7 TX \u2014");
  });
});
