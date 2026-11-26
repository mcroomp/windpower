import { formatElapsed, type ExperimentController, type ExperimentStage } from "./experiment";

const REFRESH_MS = 2_000;

const STAGE_LABELS: Record<Exclude<ExperimentStage, "idle" | "running">, string> = {
  starting: "Starting experiment…",
  "stopping-motor": "Stopping motor…",
  "spin-down": "Rotor spin-down…",
  disarming: "Safe-off…",
  saving: "Saving log…",
};

export class ExperimentButton {
  private readonly tick: ReturnType<typeof setInterval>;
  private readonly poll: ReturnType<typeof setInterval>;
  private readonly unsubscribe: () => void;

  constructor(
    private readonly button: HTMLButtonElement,
    private readonly dialog: HTMLDialogElement,
    private readonly experiment: ExperimentController,
    private readonly write: (message: string, kind?: "info" | "error") => void,
  ) {
    const form = this.element<HTMLFormElement>("#experiment-form");
    const name = this.element<HTMLInputElement>("#experiment-name");
    const description = this.element<HTMLTextAreaElement>("#experiment-description");
    const error = this.element<HTMLElement>("#experiment-error");
    const confirm = this.element<HTMLButtonElement>("#experiment-confirm");
    this.element<HTMLButtonElement>("#experiment-cancel").addEventListener("click", () => dialog.close());

    button.addEventListener("click", () => {
      if (experiment.running) {
        void experiment.stop().catch(() => undefined);
      } else if (experiment.stage === "idle") {
        error.hidden = true;
        dialog.showModal();
        name.focus();
      }
    });
    form.addEventListener("submit", (event) => {
      event.preventDefault();
      confirm.disabled = true;
      error.hidden = true;
      experiment.start(name.value, description.value).then(() => {
        name.value = "";
        description.value = "";
        dialog.close();
      }, (failure: unknown) => {
        error.textContent = failure instanceof Error ? failure.message : String(failure);
        error.hidden = false;
      }).finally(() => {
        confirm.disabled = false;
      });
    });

    this.unsubscribe = experiment.onChange(() => this.render());
    this.tick = setInterval(() => this.render(), 1_000);
    this.poll = setInterval(() => {
      void this.refresh();
      void this.experiment.refreshOnboardLogs();
    }, REFRESH_MS);
    this.render();
    void this.refresh();
  }

  dispose(): void {
    clearInterval(this.tick);
    clearInterval(this.poll);
    this.unsubscribe();
  }

  private async refresh(): Promise<void> {
    try {
      await this.experiment.refresh();
    } catch (error) {
      this.write(`Experiment status unavailable: ${error instanceof Error ? error.message : String(error)}`, "error");
    }
  }

  private render(): void {
    const active = this.experiment.active;
    const stage = this.experiment.stage;
    let text: string;
    let state: "start" | "stop" | "stopping";
    if (active && stage === "running") {
      text = `■ Stop  ${formatElapsed(Date.now() - active.started_at_ns / 1e6)}`;
      state = "stop";
    } else if (stage !== "idle" && stage !== "running") {
      text = STAGE_LABELS[stage];
      state = "stopping";
    } else {
      text = "▶ Start Experiment";
      state = "start";
    }
    if (this.button.textContent !== text) this.button.textContent = text;
    this.button.className = `experiment-button ${state}`;
    this.button.disabled = state === "stopping";
    this.button.title = active
      ? `${active.name}${active.description ? `\n${active.description}` : ""}\nStop: motor ramp-down, rotor spin-down, safe-off, save log`
      : "Arm and motor actions are disabled until an experiment is started";
  }

  private element<T extends Element>(selector: string): T {
    const found = this.dialog.querySelector<T>(selector);
    if (!found) throw new Error(`Missing experiment element ${selector}`);
    return found;
  }
}
