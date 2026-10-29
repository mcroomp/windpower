import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { MotorPanel } from "../src/motor-panel";
import type { MotorStatus } from "../src/types";

class FakeOption {
  constructor(public text: string, public value: string) {}
}

class Element extends EventTarget {
  innerHTML = "";
  textContent = "";
  private disabledValue = false;
  disabledChanges: boolean[] = [];
  get disabled(): boolean { return this.disabledValue; }
  set disabled(value: boolean) {
    if (value !== this.disabledValue) this.disabledChanges.push(value);
    this.disabledValue = value;
  }
  value = "";
  options: FakeOption[] = [];
  classList = { toggle: vi.fn() };
  add(option: FakeOption): void { this.options.push(option); }
  replaceChildren(...options: FakeOption[]): void {
    this.options = options;
    this.value = options[0]?.value ?? "";
  }
}

function setup(initial: MotorStatus = connected()) {
  const elements = new Map<string, Element>();
  const root = {
    innerHTML: "",
    querySelector(selector: string) {
      const role = /"([^"]+)"/.exec(selector)?.[1] ?? "";
      if (!elements.has(role)) elements.set(role, new Element());
      return elements.get(role);
    },
  };
  const doc = Object.assign(new EventTarget(), { hidden: false, createElement: () => root });
  vi.stubGlobal("document", doc);
  vi.stubGlobal("window", new EventTarget());
  vi.stubGlobal("Option", FakeOption);
  let status = initial;
  const api = {
    motorStatus: vi.fn(async () => status),
    motorDevices: vi.fn(async () => ({ devices: [
      { id: "motor-1", name: "BLDC A" }, { id: "motor-2", name: "BLDC B" },
    ] })),
    connectMotor: vi.fn(async (id: string) => {
      status = { ...connected(), device_id: id, device: id };
      return status;
    }),
    runMotor: vi.fn(async (_id: string, speed: number): Promise<MotorStatus> => {
      status = { ...connected(), running: true, speed_percent: speed, target_speed_percent: speed, command_expires_in_ms: 2000 };
      return status;
    }),
    stopMotor: vi.fn(async () => { status = connected(); return status; }),
  };
  const write = vi.fn();
  const panel = new MotorPanel(document.createElement("section"), api, write);
  const speed = elements.get("speed")!;
  speed.value = "10";
  panel.open();
  return { panel, api, elements, doc, write, speed, setStatus: (value: MotorStatus) => { status = value; } };
}

function connected(): Extract<MotorStatus, { available: true }> {
  return {
    available: true, connected: true, device: "BLDC A", device_id: "motor-1",
    running: false, speed_percent: 0, direction: "forward",
    target_speed_percent: 0, stopping: false,
    command_expires_in_ms: null, error: null,
  };
}

beforeEach(() => {
  vi.useFakeTimers();
  vi.spyOn(performance, "now").mockImplementation(() => Date.now());
});
afterEach(() => {
  vi.useRealTimers();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

describe("Bluetooth motor panel", () => {
  it("shows applied versus target commands and a server-owned stop", async () => {
    const { panel, elements, api, setStatus } = setup({
      ...connected(), running: true, speed_percent: 30, target_speed_percent: 80,
    });
    await vi.advanceTimersByTimeAsync(0);
    expect(elements.get("state")!.textContent).toContain("30% -> 80%");
    setStatus({ ...connected(), running: true, stopping: true, speed_percent: 20 });
    await vi.advanceTimersByTimeAsync(500);
    expect(elements.get("state")!.textContent).toBe("STOPPING - 20%");
    expect(elements.get("on")!.disabled).toBe(true);
    expect(api.runMotor).not.toHaveBeenCalled();
    panel.dispose();
  });

  it("keeps controls stable while a background status refresh is pending", async () => {
    const { panel, api, elements } = setup();
    await vi.advanceTimersByTimeAsync(0);
    const before = [...elements.values()].map((element) => [...element.disabledChanges]);
    let finish: (status: MotorStatus) => void = () => { throw new Error("No pending refresh"); };
    api.motorStatus.mockImplementationOnce(() => new Promise<MotorStatus>((resolve) => { finish = resolve; }));
    await vi.advanceTimersByTimeAsync(500);
    expect(elements.get("device")!.disabled).toBe(false);
    expect(elements.get("on")!.disabled).toBe(false);
    expect(elements.get("speed")!.disabled).toBe(false);
    expect([...elements.values()].map((element) => element.disabledChanges)).toEqual(before);
    finish(connected());
    await vi.advanceTimersByTimeAsync(0);
    expect([...elements.values()].map((element) => element.disabledChanges)).toEqual(before);
    expect(api.runMotor).not.toHaveBeenCalled();
    panel.dispose();
  });

  it("keeps the speed slider stable during background lease renewal", async () => {
    const { panel, api, speed } = setup();
    await panel.turnOn();
    const before = [...speed.disabledChanges];
    let finish: (status: MotorStatus) => void = () => { throw new Error("No pending renewal"); };
    api.runMotor.mockImplementationOnce(() => new Promise<MotorStatus>((resolve) => { finish = resolve; }));
    await vi.advanceTimersByTimeAsync(500);
    expect(speed.disabled).toBe(false);
    expect(speed.disabledChanges).toEqual(before);
    finish({ ...connected(), running: true, speed_percent: 10, command_expires_in_ms: 2000 });
    await vi.advanceTimersByTimeAsync(0);
    expect(speed.disabledChanges).toEqual(before);
    panel.dispose();
  });

  it("scans and connects the explicitly selected device without starting it", async () => {
    const { panel, api, elements } = setup();
    await panel.scan();
    expect(elements.get("device")!.options.map((option) => option.value)).toEqual(["", "motor-1", "motor-2"]);
    elements.get("device")!.value = "motor-2";
    await panel.connect();
    expect(api.connectMotor).toHaveBeenCalledWith("motor-2");
    expect(api.runMotor).not.toHaveBeenCalled();
    panel.dispose();
  });

  it("renews a two-second lease every 500 ms and stops when closed", async () => {
    const { panel, api, speed } = setup();
    await panel.turnOn();
    expect(api.runMotor).toHaveBeenLastCalledWith("motor-1", 10, "forward", 2000);
    await vi.advanceTimersByTimeAsync(500);
    expect(api.runMotor).toHaveBeenCalledTimes(2);
    speed.value = "25";
    speed.dispatchEvent(new Event("change"));
    await vi.advanceTimersByTimeAsync(0);
    expect(api.runMotor).toHaveBeenLastCalledWith("motor-1", 25, "forward", 2000);
    panel.close();
    await vi.advanceTimersByTimeAsync(0);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    const calls = api.runMotor.mock.calls.length;
    await vi.advanceTimersByTimeAsync(3000);
    expect(api.runMotor).toHaveBeenCalledTimes(calls);
    panel.dispose();
  });

  it("stops on zero speed and does not adopt another client's running motor", async () => {
    const { panel, api, speed, setStatus } = setup();
    await panel.turnOn();
    speed.value = "0";
    speed.dispatchEvent(new Event("change"));
    await vi.advanceTimersByTimeAsync(0);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    setStatus({ ...connected(), running: true, speed_percent: 50 });
    const calls = api.runMotor.mock.calls.length;
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.runMotor).toHaveBeenCalledTimes(calls);
    panel.dispose();
  });

  it("stops when the tab is hidden and never restarts when shown again", async () => {
    const { panel, api, doc } = setup();
    await panel.turnOn();
    doc.hidden = true;
    doc.dispatchEvent(new Event("visibilitychange"));
    await vi.advanceTimersByTimeAsync(0);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    doc.hidden = false;
    doc.dispatchEvent(new Event("visibilitychange"));
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.runMotor).toHaveBeenCalledTimes(1);
    panel.dispose();
  });

  it("does not revive an expired lease after a suspended event loop", async () => {
    const { panel, api } = setup();
    await panel.turnOn();
    vi.setSystemTime(Date.now() + 3000);
    await vi.advanceTimersByTimeAsync(500);
    expect(api.runMotor).toHaveBeenCalledTimes(1);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    panel.dispose();
  });

  it("revokes renewal and reports network failures explicitly", async () => {
    const { panel, api, write } = setup();
    await panel.turnOn();
    api.motorStatus.mockRejectedValueOnce(new Error("network lost"));
    await vi.advanceTimersByTimeAsync(500);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    expect(write).toHaveBeenCalledWith("Motor: network lost", "error");
    await vi.advanceTimersByTimeAsync(1000);
    expect(api.runMotor).toHaveBeenCalledTimes(1);
    panel.dispose();
  });

  it("queues off after an in-flight on command and does not renew it", async () => {
    const { panel, api } = setup();
    let finish: (status: MotorStatus) => void = () => { throw new Error("No pending command"); };
    api.runMotor.mockImplementationOnce(() => new Promise<MotorStatus>((resolve) => { finish = resolve; }));
    const starting = panel.turnOn();
    await vi.advanceTimersByTimeAsync(0);
    const stopping = panel.stop();
    finish({ ...connected(), running: true, speed_percent: 10 });
    await Promise.all([starting, stopping]);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    await vi.advanceTimersByTimeAsync(1500);
    expect(api.runMotor).toHaveBeenCalledTimes(1);
    panel.dispose();
  });

  it("renders a disabled backend without attempting Bluetooth access", async () => {
    const { panel, api, elements } = setup({ available: false });
    await vi.advanceTimersByTimeAsync(0);
    expect(elements.get("scan")!.disabled).toBe(true);
    expect(elements.get("on")!.disabled).toBe(true);
    expect(elements.get("state")!.textContent).toContain("disabled");
    expect(api.motorDevices).not.toHaveBeenCalled();
    panel.dispose();
  });

  it("refuses to take over a motor already running for another client", async () => {
    const { panel, api, write } = setup({ ...connected(), running: true, speed_percent: 50 });
    await panel.turnOn();
    expect(api.runMotor).not.toHaveBeenCalled();
    expect(write).toHaveBeenCalledWith("Motor: Motor is already running; stop it before taking control", "error");
    panel.dispose();
  });

  it("stops renewal if the connected device changes", async () => {
    const { panel, api, setStatus } = setup();
    await panel.turnOn();
    setStatus({ ...connected(), device_id: "motor-2", running: true });
    await vi.advanceTimersByTimeAsync(500);
    expect(api.runMotor).toHaveBeenCalledTimes(1);
    expect(api.stopMotor).toHaveBeenCalledTimes(1);
    panel.dispose();
  });
});
