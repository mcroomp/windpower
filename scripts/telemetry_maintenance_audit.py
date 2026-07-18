"""Telemetry schema maintenance audit.

Scans the repository for references to every telemetry CSV column defined in
simulation/telemetry_columns.py and writes AI-friendly reports with file/line hits.

Outputs:
  - telemetry_audit_report.json   (machine-readable)
  - telemetry_audit_report.md     (human-readable)

By default, output goes to simulation/logs/telemetry_maintenance/.
"""

from __future__ import annotations

import argparse
import ast
import importlib.util
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import simulation


EXCLUDED_DIRS = {
    ".git",
    ".venv",
    "__pycache__",
    "node_modules",
    "dist",
    "build",
    "simulation/logs",
}

CODE_SUFFIXES = {
    ".py",
    ".json",
    ".yaml",
    ".yml",
    ".toml",
    ".ini",
    ".cfg",
    ".sh",
    ".cmd",
}

DOC_SUFFIXES = {".md", ".txt"}


@dataclass
class RefHit:
    path: str
    line: int
    kind: str
    snippet: str


def _is_excluded(path: Path, repo_root: Path) -> bool:
    rel = path.relative_to(repo_root).as_posix()
    parts = rel.split("/")
    for i in range(1, len(parts) + 1):
        joined = "/".join(parts[:i])
        if joined in EXCLUDED_DIRS or parts[i - 1] in EXCLUDED_DIRS:
            return True
    return False


def _iter_text_files(repo_root: Path, *, include_docs: bool) -> Iterable[Path]:
    suffixes = set(CODE_SUFFIXES)
    if include_docs:
        suffixes |= DOC_SUFFIXES
    for p in repo_root.rglob("*"):
        if not p.is_file():
            continue
        if _is_excluded(p, repo_root):
            continue
        if p.suffix.lower() in suffixes:
            yield p


def _load_columns(schema_path: Path) -> list[str]:
    src = schema_path.read_text(encoding="utf-8")
    tree = ast.parse(src, filename=str(schema_path))
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == "COLUMNS":
            value = node.value
            if isinstance(value, (ast.List, ast.Tuple)):
                out: list[str] = []
                for elt in value.elts:
                    if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                        out.append(elt.value)
                if out:
                    return out
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "COLUMNS":
                    value = node.value
                    if isinstance(value, (ast.List, ast.Tuple)):
                        out = []
                        for elt in value.elts:
                            if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                                out.append(elt.value)
                        if out:
                            return out
    # Fallback for computed schemas (e.g. COLUMNS derived from COLUMN_SPECS).
    spec = importlib.util.spec_from_file_location("telemetry_schema_audit", schema_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not load telemetry schema module for schema extraction")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    column_specs = getattr(mod, "COLUMN_SPECS", None)
    if column_specs:
        out = [name for name, _source in column_specs]
        if out:
            return out
    columns = getattr(mod, "COLUMNS", None)
    if isinstance(columns, list) and columns:
        return columns
    raise RuntimeError("Could not parse COLUMNS/COLUMN_SPECS from telemetry schema module")


def _classify_kind(rel_path: str, line_text: str, column: str) -> str:
    t = line_text.strip()
    if rel_path in {"simulation/telemetry_csv.py", "simulation/telemetry_columns.py"}:
        if "COLUMNS" in t or f'"{column}"' in t or f"'{column}'" in t:
            return "schema"
        if "from_physics" in t or "from_tel" in t or "to_dict" in t:
            return "schema-mapping"
        return "schema-support"
    if "simulation/mediator.py" in rel_path and f'fields["{column}"]' in t:
        return "producer-mediator"
    if rel_path.startswith("analysis/"):
        return "consumer-analysis"
    if rel_path.startswith("tests/"):
        return "consumer-test"
    if "telemetry" in t.lower() or "csv" in t.lower():
        return "consumer-generic"
    return "reference"


def _find_references(repo_root: Path, columns: list[str], *, include_docs: bool) -> dict[str, list[RefHit]]:
    refs: dict[str, list[RefHit]] = {c: [] for c in columns}
    regexes = {
        c: re.compile(rf"(?<![A-Za-z0-9_]){re.escape(c)}(?![A-Za-z0-9_])")
        for c in columns
    }
    files = list(_iter_text_files(repo_root, include_docs=include_docs))
    for p in files:
        rel = p.relative_to(repo_root).as_posix()
        try:
            text = p.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            continue
        lines = text.splitlines()
        for i, line in enumerate(lines, start=1):
            for col, rx in regexes.items():
                if not rx.search(line):
                    continue
                refs[col].append(
                    RefHit(
                        path=rel,
                        line=i,
                        kind=_classify_kind(rel, line, col),
                        snippet=line.strip()[:240],
                    )
                )
    return refs


def _status_for_hits(hits: list[RefHit]) -> str:
    if not hits:
        return "unreferenced"
    non_schema = [h for h in hits if h.kind not in {"schema", "schema-mapping", "schema-support"}]
    if not non_schema:
        return "schema-only"
    return "referenced"


def _build_report(columns: list[str], refs: dict[str, list[RefHit]]) -> dict:
    per_column = []
    counts = {"referenced": 0, "schema-only": 0, "unreferenced": 0}
    for col in columns:
        hits = refs[col]
        status = _status_for_hits(hits)
        counts[status] += 1
        per_column.append(
            {
                "column": col,
                "status": status,
                "reference_count": len(hits),
                "references": [
                    {
                        "path": h.path,
                        "line": h.line,
                        "kind": h.kind,
                        "snippet": h.snippet,
                    }
                    for h in hits
                ],
            }
        )
    return {
        "summary": {
            "total_columns": len(columns),
            "referenced": counts["referenced"],
            "schema_only": counts["schema-only"],
            "unreferenced": counts["unreferenced"],
        },
        "columns": per_column,
    }


def _write_markdown(report: dict, out_md: Path) -> None:
    lines: list[str] = []
    s = report["summary"]
    lines.append("# Telemetry Schema Audit")
    lines.append("")
    lines.append(f"- total_columns: {s['total_columns']}")
    lines.append(f"- referenced: {s['referenced']}")
    lines.append(f"- schema_only: {s['schema_only']}")
    lines.append(f"- unreferenced: {s['unreferenced']}")
    lines.append("")

    for item in report["columns"]:
        lines.append(f"## {item['column']}")
        lines.append(f"- status: {item['status']}")
        lines.append(f"- reference_count: {item['reference_count']}")
        if not item["references"]:
            lines.append("- references: none")
            lines.append("")
            continue
        lines.append("- references:")
        for ref in item["references"]:
            lines.append(
                f"  - {ref['path']}:{ref['line']} [{ref['kind']}] :: {ref['snippet']}"
            )
        lines.append("")

    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit telemetry CSV column usage across the codebase")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Repository root (default: inferred from script location)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(simulation.__file__).resolve().parent / "logs" / "telemetry_maintenance",
        help="Output directory",
    )
    parser.add_argument(
        "--include-docs",
        action="store_true",
        help="Include markdown/text files in the reference scan",
    )
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    telemetry_schema_path = repo_root / "simulation" / "telemetry_columns.py"
    columns = _load_columns(telemetry_schema_path)
    refs = _find_references(repo_root, columns, include_docs=args.include_docs)
    report = _build_report(columns, refs)

    out_json = out_dir / "telemetry_audit_report.json"
    out_md = out_dir / "telemetry_audit_report.md"
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    _write_markdown(report, out_md)

    print(f"report_json={out_json}")
    print(f"report_md={out_md}")
    print(
        "summary "
        f"total={report['summary']['total_columns']} "
        f"referenced={report['summary']['referenced']} "
        f"schema_only={report['summary']['schema_only']} "
        f"unreferenced={report['summary']['unreferenced']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
