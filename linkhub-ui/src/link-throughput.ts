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
