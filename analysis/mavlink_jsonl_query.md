# `mavlink_jsonl_query.py` -- MAVLink JSONL log query tool

Canonical usage doc for `analysis/mavlink_jsonl_query.py`, the standard,
first-line tool for diagnosing any problematic run that produced a
`*.mavlink.jsonl` log (calibrate `run`, SITL stack tests). Prefer this tool
over writing one-off jsonl-parsing code or manually grepping the raw file. If
a diagnosis need doesn't fit an existing subcommand, prefer extending this
script (new subcommand/filter) over a standalone script -- keep this doc in
sync with any command/flag changes.

Referenced from `AGENTS.md` (Documentation Ownership table + "MAVLink Log
Diagnosis" section) and `design/calibration.md`.

## Log format

Logs are written by `MavlinkLogWriter` (`groundstation/mavlink_log.py`) from
both `groundstation/gcs.py` (`RawesGCS.start_mavlog`) and the bench
calibration tool (`calibrate/run.py`). Every line is one JSON object:

```json
{"_t_wall": <float>, "_dir": "rx"|"tx", "mavpackettype": "<TYPE>", ...fields...}
```

`_t_wall` is wall-clock seconds; `_dir` is `"rx"` (from the FC) or `"tx"` (to
the FC). The tool adds a derived `t_rel` field (seconds since the first
message in the log) to every message before filtering/printing.

## Subcommands

```
mavlink_jsonl_query.py types      <log.jsonl>
mavlink_jsonl_query.py show       <log.jsonl> [filters] [--fields a,b,c] [--json] [--limit N]
mavlink_jsonl_query.py count      <log.jsonl> [filters] [--by FIELD]
mavlink_jsonl_query.py stats      <log.jsonl> --type T --field F [filters]
mavlink_jsonl_query.py armed      <log.jsonl>
mavlink_jsonl_query.py statustext <log.jsonl> [--since S] [--until S] [--min-severity N]
mavlink_jsonl_query.py nvf        <log.jsonl> [--name RAWES_ARM] [--dir rx|tx]
mavlink_jsonl_query.py param      <log.jsonl> [--id RAWES_MODE]
```

Run `--help` on any subcommand for the exact flags; this doc gives the intent
and gotchas for each.

- **`types`** -- list every `mavpackettype` seen, split by direction, with
  count and first/last `t_rel`. Always the right first command on an unknown
  log: confirms whether a message type is present at all before chasing a
  decode bug. A message type with count 0 (i.e. absent from this list) means
  it was never requested/streamed from the FC -- not a decode/registration
  bug in `groundstation/gcs.py`.
- **`show`** -- generic filtered dump of raw message fields, one line per
  message. Use `--fields` to restrict columns, `--json` to emit raw JSON
  lines (for piping into `jq`/Python), `--limit` to cap output. This is the
  fallback for any message type without a dedicated subcommand; it does NOT
  do any special-casing (e.g. no STATUSTEXT reassembly -- use the
  `statustext` subcommand for that).
- **`count`** -- count matched messages, optionally grouped by a field
  (`--by mavpackettype`, `--by mode`, etc).
- **`stats`** -- min/max/mean/median/stdev for one numeric field, e.g.
  `--type ATTITUDE --field yaw`.
- **`armed`** -- timeline of HEARTBEAT armed-state/mode transitions (rx only,
  excludes GCS self-heartbeats). Good for confirming arm/disarm timing and
  flight-mode transitions without wading through every HEARTBEAT.
- **`statustext`** -- dump `gcs:send_text()`/STATUSTEXT messages in order,
  **reassembled** from ArduPilot's wire-level chunking. See "STATUSTEXT
  reassembly" below -- this is the one subcommand where NOT using the tool
  (i.e. grepping the raw jsonl) actively produces misleading output.
- **`nvf`** -- dump the `NAMED_VALUE_FLOAT` stream, i.e. the ground<->Lua
  `RAWES_*` interface. Filter with `--name RAWES_ARM` to follow one channel,
  `--dir tx` to see only what the ground sent.
- **`param`** -- dump `PARAM_SET`/`PARAM_VALUE` events, filter with `--id`.

## Common filters (`show`/`count`/`stats`)

- `-t/--type TYPE` (repeatable; OR) -- e.g. `-t ATTITUDE -t AHRS2`
- `--dir {rx,tx}`
- `--since SEC` / `--until SEC` -- `t_rel` bounds
- `--eq FIELD=VALUE` (repeatable; AND) -- e.g. `--eq mavpackettype=STATUSTEXT`
- `--contains TEXT` -- substring match on the raw JSON line (case-insensitive)

## STATUSTEXT reassembly (agent-critical)

ArduPilot's `gcs:send_text()`/STATUSTEXT wire format caps the `text` field at
50 bytes. Any Lua `gcs:send_text()` call longer than that is split across
multiple STATUSTEXT messages that share a common non-zero `id` field, ordered
by `chunk_seq` (0, 1, 2, ...). `id == 0` is the ArduPilot sentinel for a
standalone, already-short single-chunk message -- those must never be merged
with each other even if several appear back-to-back.

`mavlink_jsonl_query.py statustext` reassembles chunks sharing the same
non-zero `id` into a single logical line before printing. Do not use `show
-t STATUSTEXT` or grep the raw jsonl directly when reading Lua diagnostic
text -- both will print each ~50-byte fragment as its own line, sometimes
splitting a message mid-word (e.g. `"RAWES YIC capture: r=89.9 p=20.0 y=-175.3"`
can arrive on the wire as `"RAWES YIC capture: r=89.9 p=20.0 y=-17"` +
`"5.3"`), which is hard to read for both humans and AI and can look like log
corruption when it's just unreassembled chunking.

## Example: diagnosing a "target never updates" bug

```
mavlink_jsonl_query.py types      simulation/logs/calibrate/run_passive_....mavlink.jsonl
mavlink_jsonl_query.py statustext simulation/logs/calibrate/run_passive_....mavlink.jsonl
mavlink_jsonl_query.py show -t ATTITUDE_TARGET --fields q simulation/logs/calibrate/run_passive_....mavlink.jsonl
```

`types` confirms the message is actually present (vs. a missing stream
request); `statustext` surfaces Lua's own diagnostic text (e.g. `RAWES YIC
capture: ...`, `RAWES guided cmd: ...`) to compare what Lua *thinks* it
captured/sent against what the FC is actually reporting.
