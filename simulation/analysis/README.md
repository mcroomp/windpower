# analysis/

Standalone analysis and utility scripts. Not part of the simulation runtime.
These are not imported by `mediator.py` or any test fixture.

| Script | Purpose | Status |
|--------|---------|--------|
| `generate_flight_report.py` | Offline multi-panel plot from mediator telemetry CSV | Active |
| `redraw_flight_report.py` | Regenerate `flight_report.png` from saved `flight_data.json` | Active |
| `merge_logs.py` | Merge mediator/SITL/GCS logs in timestamp order | Active |
| `analyse_30s_dip.py` | Diagnostic for 30 s guided flight altitude dip | Historical |
| `plot_30s_mechanism.py` | Annotated diagram of the 30 s dip mechanism | Historical |
| `build.py` | Docker image build progress monitor | Active |
