#!/usr/bin/env python3
"""
mavlink_jsonl_query.py -- Standard CLI tool for querying *.mavlink.jsonl logs.

These logs are written by MavlinkLogWriter (groundstation/mavlink_log.py) from
both groundstation/gcs.py (RawesGCS.start_mavlog) and the bench calibration tool
(calibrate/run.py). Every line is one JSON object:

    {"_t_wall": <float>, "_dir": "rx"|"tx", "mavpackettype": "<TYPE>", ...fields...}

This is the standard, first-line tool for diagnosing any problematic run that
has a `*.mavlink.jsonl` log (calibrate `run`, SITL stack tests) -- prefer it
over writing one-off jsonl parsing code or manually grepping the raw file, and
prefer extending this script (new subcommand or filter) over either of those.
See mavlink_jsonl_query.md in this directory for the canonical usage doc
(subcommand intent, gotchas, examples); keep that file in sync with any
command/flag changes.

Usage summary (see `--help` on each subcommand for full options):
    mavlink_jsonl_query.py types      <log.jsonl>
    mavlink_jsonl_query.py show       <log.jsonl> [filters] [--fields a,b,c] [--json]
    mavlink_jsonl_query.py count      <log.jsonl> [filters] [--by FIELD]
    mavlink_jsonl_query.py stats      <log.jsonl> --type T --field F [filters]
    mavlink_jsonl_query.py armed      <log.jsonl>
    mavlink_jsonl_query.py statustext <log.jsonl> [--min-severity N]
    mavlink_jsonl_query.py nvf        <log.jsonl> [--name RAWES_ARM]
    mavlink_jsonl_query.py param      <log.jsonl> [--id RAWES_MODE]

Common filters (available on show/count/stats):
    -t/--type TYPE      (repeatable; OR)      e.g. -t ATTITUDE -t AHRS2
    --dir {rx,tx}
    --since SEC          messages with t_rel >= SEC (t_rel = _t_wall - log start)
    --until SEC           messages with t_rel <= SEC
    --eq FIELD=VALUE     (repeatable; AND)     e.g. --eq mavpackettype=STATUSTEXT
    --contains TEXT       substring match on the raw JSON line (case-insensitive)
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any, Iterable, Iterator

import simulation as _simulation_pkg
_SIM_DIR = Path(_simulation_pkg.__file__).resolve().parent  # simulation/

from groundstation.mavlink_log import iter_messages  # noqa: E402
from groundstation.ekf_flags import MAV_MODE_ARMED, ARDU_MODES, MAV_STATE  # noqa: E402

MAV_SEVERITY = {
    0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
    4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG",
}


# ---------------------------------------------------------------------------
# Loading + generic filtering
# ---------------------------------------------------------------------------

def load(path: "str | Path") -> list[dict]:
    """Load all messages from *path*, sorted by _t_wall, with a t_rel field
    (seconds since the first message) added to every dict."""
    msgs = list(iter_messages(path))
    msgs.sort(key=lambda m: m.get("_t_wall", 0.0))
    if not msgs:
        return msgs
    t0 = msgs[0]["_t_wall"]
    for m in msgs:
        m["t_rel"] = m.get("_t_wall", t0) - t0
    return msgs


def _coerce(value: str) -> Any:
    """Best-effort str -> number for --eq comparisons."""
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def apply_filters(
    msgs: Iterable[dict],
    types: "list[str] | None" = None,
    direction: "str | None" = None,
    since: "float | None" = None,
    until: "float | None" = None,
    eq: "list[str] | None" = None,
    contains: "str | None" = None,
) -> Iterator[dict]:
    type_set = set(types) if types else None
    eq_pairs: list[tuple[str, Any]] = []
    for spec in eq or []:
        if "=" not in spec:
            raise ValueError(f"--eq expects FIELD=VALUE, got {spec!r}")
        k, v = spec.split("=", 1)
        eq_pairs.append((k, _coerce(v)))
    contains_lower = contains.lower() if contains else None

    for m in msgs:
        if type_set is not None and m.get("mavpackettype") not in type_set:
            continue
        if direction is not None and m.get("_dir") != direction:
            continue
        if since is not None and m.get("t_rel", 0.0) < since:
            continue
        if until is not None and m.get("t_rel", 0.0) > until:
            continue
        if eq_pairs and not all(m.get(k) == v or str(m.get(k)) == str(v) for k, v in eq_pairs):
            continue
        if contains_lower is not None and contains_lower not in json.dumps(m).lower():
            continue
        yield m


def _add_common_filter_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("-t", "--type", action="append", dest="types", metavar="TYPE",
                    help="mavpackettype to include (repeatable; OR)")
    p.add_argument("--dir", choices=["rx", "tx"], help="restrict to received or sent messages")
    p.add_argument("--since", type=float, metavar="SEC", help="only messages with t_rel >= SEC")
    p.add_argument("--until", type=float, metavar="SEC", help="only messages with t_rel <= SEC")
    p.add_argument("--eq", action="append", metavar="FIELD=VALUE",
                    help="only messages where FIELD == VALUE (repeatable; AND)")
    p.add_argument("--contains", metavar="TEXT",
                    help="only messages whose raw JSON line contains TEXT (case-insensitive)")


def _filtered(args: argparse.Namespace, path: str) -> list[dict]:
    msgs = load(path)
    return list(apply_filters(
        msgs,
        types=args.types, direction=args.dir,
        since=args.since, until=args.until,
        eq=args.eq, contains=args.contains,
    ))


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------

def cmd_types(args: argparse.Namespace) -> None:
    msgs = load(args.log)
    counts: dict[tuple[str, str], list[float]] = {}
    for m in msgs:
        key = (m.get("mavpackettype", "?"), m.get("_dir", "?"))
        counts.setdefault(key, []).append(m["t_rel"])
    print(f"{'TYPE':<26} {'DIR':<4} {'COUNT':>7} {'FIRST_T':>10} {'LAST_T':>10}")
    for (mtype, d), times in sorted(counts.items()):
        print(f"{mtype:<26} {d:<4} {len(times):>7} {times[0]:>10.3f} {times[-1]:>10.3f}")
    print(f"\nTotal messages: {len(msgs)}   duration: {msgs[-1]['t_rel']:.3f}s" if msgs else "empty log")


def cmd_show(args: argparse.Namespace) -> None:
    rows = _filtered(args, args.log)
    if args.limit:
        rows = rows[: args.limit]
    if args.json:
        for m in rows:
            print(json.dumps(m))
        return
    fields = [f.strip() for f in args.fields.split(",")] if args.fields else None
    for m in rows:
        if fields:
            vals = " ".join(f"{f}={m.get(f)!r}" for f in fields)
        else:
            skip = {"_t_wall", "t_rel", "_dir", "mavpackettype"}
            vals = " ".join(f"{k}={v!r}" for k, v in m.items() if k not in skip)
        print(f"[{m['t_rel']:>9.3f}s] {m.get('_dir'):<3} {m.get('mavpackettype'):<24} {vals}")
    print(f"\n{len(rows)} message(s)", file=sys.stderr)


def cmd_count(args: argparse.Namespace) -> None:
    rows = _filtered(args, args.log)
    if not args.by:
        print(len(rows))
        return
    buckets: dict[Any, int] = {}
    for m in rows:
        buckets[m.get(args.by)] = buckets.get(m.get(args.by), 0) + 1
    for key, n in sorted(buckets.items(), key=lambda kv: -kv[1]):
        print(f"{key!r:<30} {n}")
    print(f"\nTotal: {len(rows)}", file=sys.stderr)


def cmd_stats(args: argparse.Namespace) -> None:
    rows = _filtered(args, args.log)
    values = []
    for m in rows:
        v = m.get(args.field)
        if isinstance(v, (int, float)):
            values.append(float(v))
    if not values:
        print(f"No numeric values for field {args.field!r} in {len(rows)} matched message(s)")
        return
    print(f"field={args.field}  n={len(values)}")
    print(f"  min={min(values):.6g}  max={max(values):.6g}  mean={statistics.fmean(values):.6g}")
    print(f"  median={statistics.median(values):.6g}", end="")
    if len(values) > 1:
        print(f"  stdev={statistics.stdev(values):.6g}")
    else:
        print()
    print(f"  first={values[0]:.6g}  last={values[-1]:.6g}")


def cmd_armed(args: argparse.Namespace) -> None:
    """Timeline of HEARTBEAT armed/mode transitions (rx only, autopilot heartbeats)."""
    msgs = load(args.log)
    hbs = [
        m for m in apply_filters(msgs, types=["HEARTBEAT"], direction="rx")
        if m.get("autopilot", 0) != 8  # exclude MAV_AUTOPILOT_INVALID (GCS heartbeats logged as rx if any)
    ]
    last_armed = None
    last_mode = None
    if not hbs:
        print("No HEARTBEAT (rx) messages found")
        return
    for m in hbs:
        armed = bool(m.get("base_mode", 0) & MAV_MODE_ARMED)
        mode_num = m.get("custom_mode", -1)
        mode = ARDU_MODES.get(mode_num, f"mode{mode_num}")
        state = MAV_STATE.get(m.get("system_status"), m.get("system_status"))
        if armed != last_armed or mode != last_mode:
            print(f"[{m['t_rel']:>9.3f}s] armed={armed!s:<5} mode={mode:<10} system_status={state}")
            last_armed = armed
            last_mode = mode


def _reassemble_statustext(rows: Iterable[dict]) -> Iterator[dict]:
    """Reassemble multi-chunk STATUSTEXT messages into single logical lines.

    ArduPilot's gcs:send_text()/STATUSTEXT wire format caps `text` at 50 bytes;
    longer messages are split across multiple STATUSTEXT messages that share a
    common non-zero `id`, ordered by `chunk_seq` (0, 1, 2, ...). `id == 0` is
    the ArduPilot sentinel for a standalone, already-short single-chunk
    message -- those must NEVER be merged with each other even if several
    appear back-to-back. Without this reassembly, `show`/raw dumps print each
    ~50-char fragment as its own line/timestamp, which is hard for both humans
    and AI to read (a single log line like "RAWES YIC capture: r=89.9 p=20.0
    y=-175.3" can be split mid-word across two lines).
    """
    pending_id: "int | None" = None
    pending_chunks: dict[int, str] = {}
    pending_meta: "dict | None" = None

    def _flush() -> "dict | None":
        if pending_meta is None:
            return None
        out = dict(pending_meta)
        out["text"] = "".join(pending_chunks[k] for k in sorted(pending_chunks))
        return out

    for m in rows:
        mid = m.get("id", 0)
        text = m.get("text", "")
        if isinstance(text, (bytes, bytearray)):
            text = text.decode("utf-8", errors="replace")
        text = text.split("\x00", 1)[0]
        if mid == 0:
            flushed = _flush()
            if flushed is not None:
                yield flushed
            pending_id, pending_chunks, pending_meta = None, {}, None
            out = dict(m)
            out["text"] = text
            yield out
            continue
        if mid != pending_id:
            flushed = _flush()
            if flushed is not None:
                yield flushed
            pending_id = mid
            pending_chunks = {}
            pending_meta = dict(m)
        pending_chunks[m.get("chunk_seq", 0)] = text
    flushed = _flush()
    if flushed is not None:
        yield flushed


def cmd_statustext(args: argparse.Namespace) -> None:
    msgs = load(args.log)
    rows = apply_filters(msgs, types=["STATUSTEXT"], direction="rx",
                          since=args.since, until=args.until)
    for m in _reassemble_statustext(rows):
        sev = m.get("severity", -1)
        if args.min_severity is not None and sev > args.min_severity:
            # NOTE: lower severity number == more severe (MAVLink convention)
            continue
        print(f"[{m['t_rel']:>9.3f}s] {MAV_SEVERITY.get(sev, sev):<9} {m['text']}")


def cmd_nvf(args: argparse.Namespace) -> None:
    msgs = load(args.log)
    for m in apply_filters(msgs, types=["NAMED_VALUE_FLOAT"], direction=args.dir,
                            since=args.since, until=args.until):
        name = str(m.get("name", "")).split("\x00", 1)[0]
        if args.name and name != args.name:
            continue
        print(f"[{m['t_rel']:>9.3f}s] {name:<12} = {m.get('value')}")


def cmd_param(args: argparse.Namespace) -> None:
    msgs = load(args.log)
    for m in apply_filters(msgs, types=["PARAM_SET", "PARAM_VALUE"],
                            since=args.since, until=args.until):
        pid = str(m.get("param_id", "")).split("\x00", 1)[0]
        if args.id and pid != args.id:
            continue
        print(f"[{m['t_rel']:>9.3f}s] {m.get('_dir'):<3} {m.get('mavpackettype'):<12} "
              f"{pid:<16} = {m.get('param_value')}")


# ---------------------------------------------------------------------------
# CLI wiring
# ---------------------------------------------------------------------------

def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(
        description="Query *.mavlink.jsonl logs (see mavlink_jsonl_query.md for docs).")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("types", help="list message types with counts and time range")
    p.add_argument("log")
    p.set_defaults(func=cmd_types)

    p = sub.add_parser("show", help="print filtered messages")
    p.add_argument("log")
    _add_common_filter_args(p)
    p.add_argument("--fields", help="comma-separated field names to print (default: all)")
    p.add_argument("--limit", type=int, help="max rows to print")
    p.add_argument("--json", action="store_true", help="print raw JSON lines instead of a table")
    p.set_defaults(func=cmd_show)

    p = sub.add_parser("count", help="count filtered messages, optionally grouped by a field")
    p.add_argument("log")
    _add_common_filter_args(p)
    p.add_argument("--by", metavar="FIELD", help="group counts by this field (e.g. mavpackettype)")
    p.set_defaults(func=cmd_count)

    p = sub.add_parser("stats", help="numeric min/max/mean/median/stdev for one field")
    p.add_argument("log")
    _add_common_filter_args(p)
    p.add_argument("--field", required=True, help="field name to aggregate")
    p.set_defaults(func=cmd_stats)

    p = sub.add_parser("armed", help="timeline of HEARTBEAT armed-state/mode transitions")
    p.add_argument("log")
    p.set_defaults(func=cmd_armed)

    p = sub.add_parser("statustext", help="dump STATUSTEXT messages in order")
    p.add_argument("log")
    p.add_argument("--since", type=float)
    p.add_argument("--until", type=float)
    p.add_argument("--min-severity", type=int, metavar="N",
                    help="only show severity <= N (0=EMERGENCY..7=DEBUG; lower is worse)")
    p.set_defaults(func=cmd_statustext)

    p = sub.add_parser("nvf", help="dump NAMED_VALUE_FLOAT stream (ground<->Lua RAWES_* interface)")
    p.add_argument("log")
    p.add_argument("--name", help="only this NVF name, e.g. RAWES_ARM")
    p.add_argument("--dir", choices=["rx", "tx"])
    p.add_argument("--since", type=float)
    p.add_argument("--until", type=float)
    p.set_defaults(func=cmd_nvf)

    p = sub.add_parser("param", help="dump PARAM_SET/PARAM_VALUE events")
    p.add_argument("log")
    p.add_argument("--id", help="only this param_id, e.g. RAWES_MODE")
    p.add_argument("--since", type=float)
    p.add_argument("--until", type=float)
    p.set_defaults(func=cmd_param)

    args = ap.parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
