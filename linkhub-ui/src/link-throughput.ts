import type { LinkHubStatus } from "./types";

function formatRate(bps: number | null): string {
  if (bps === null || !Number.isFinite(bps) || bps < 0) {
    return "\u2014";
  }
  if (bps >= 1_000_000) {
    return `${(bps / 1_000_000).toFixed(1)} Mbps`;
  }
  if (bps >= 1_000) {
    return `${(bps / 1_000).toFixed(1)} kbps`;
  }
  return `${Math.round(bps)} bps`;
}

export function formatLinkThroughput(status: LinkHubStatus | null): string {
  if (!status?.connected) {
    return "RX \u2014 \u00b7 TX \u2014";
  }
  return `RX ${formatRate(status.rx_bps)} \u00b7 TX ${formatRate(status.tx_bps)}`;
}

export interface MessageThroughputRow {
  message: string;
  rxBps: number;
  txBps: number;
}

function validRate(bps: number | undefined): number {
  return bps !== undefined && Number.isFinite(bps) && bps > 0 ? bps : 0;
}

/** Per-message rows, busiest first; empty while disconnected. */
export function messageThroughputRows(status: LinkHubStatus | null): MessageThroughputRow[] {
  if (!status?.connected) {
    return [];
  }
  const names = new Set([
    ...Object.keys(status.rx_bps_by_message),
    ...Object.keys(status.tx_bps_by_message),
  ]);
  return [...names]
    .map((message) => ({
      message,
      rxBps: validRate(status.rx_bps_by_message[message]),
      txBps: validRate(status.tx_bps_by_message[message]),
    }))
    .filter((row) => row.rxBps > 0 || row.txBps > 0)
    .sort((a, b) => (b.rxBps + b.txBps) - (a.rxBps + a.txBps)
      || a.message.localeCompare(b.message));
}

export function formatKbps(bps: number): string {
  return bps > 0 ? (bps / 1_000).toFixed(2) : "\u2014";
}
