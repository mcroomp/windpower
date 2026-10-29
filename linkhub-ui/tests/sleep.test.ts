import { getEventListeners } from "node:events";
import { afterEach, describe, expect, it, vi } from "vitest";
import { sleep } from "../src/sleep";

afterEach(() => vi.useRealTimers());

describe("abortable sleep", () => {
  it("releases the abort listener after every completed polling delay", async () => {
    vi.useFakeTimers();
    const controller = new AbortController();
    for (let poll = 0; poll < 100; poll += 1) {
      const waiting = sleep(50, controller.signal);
      expect(getEventListeners(controller.signal, "abort")).toHaveLength(1);
      vi.advanceTimersByTime(49);
      expect(getEventListeners(controller.signal, "abort")).toHaveLength(1);
      vi.advanceTimersByTime(1);
      await waiting;
      expect(getEventListeners(controller.signal, "abort")).toHaveLength(0);
      expect(vi.getTimerCount()).toBe(0);
    }
  });

  it("resolves and releases its timer when aborted", async () => {
    vi.useFakeTimers();
    const controller = new AbortController();
    const waiting = sleep(500, controller.signal);
    controller.abort();
    await waiting;
    expect(getEventListeners(controller.signal, "abort")).toHaveLength(0);
    expect(vi.getTimerCount()).toBe(0);
  });

  it("does not schedule a delay after cancellation", async () => {
    vi.useFakeTimers();
    const controller = new AbortController();
    controller.abort();
    await sleep(500, controller.signal);
    expect(getEventListeners(controller.signal, "abort")).toHaveLength(0);
    expect(vi.getTimerCount()).toBe(0);
  });
});
