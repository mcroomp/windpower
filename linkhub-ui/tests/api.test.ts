import { afterEach, describe, expect, it, vi } from "vitest";

import { formatMavResult, LinkHubApi } from "../src/api";

afterEach(() => {
  vi.unstubAllGlobals();
});

describe("formatMavResult", () => {
  it("names known MAVLink command results", () => {
    expect(formatMavResult(4)).toBe("MAV_RESULT_FAILED (4)");
  });

  it("preserves unknown result codes", () => {
    expect(formatMavResult(99)).toBe("unknown MAV_RESULT (99)");
  });

  it("resolves API paths against a Node CLI base URL", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      json: async () => ({ service: "linkhub" }),
    });
    vi.stubGlobal("fetch", fetchMock);

    await new LinkHubApi("http://127.0.0.1:8999").serviceStatus();

    expect(fetchMock).toHaveBeenCalledWith(
      "http://127.0.0.1:8999/v1/status",
      expect.objectContaining({ method: "GET" }),
    );
  });
});
