#!/usr/bin/env python3
"""Check Modelica source using the Rumoca Python binding."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", nargs="?", default="src/FixedWingOuterLoop.mo", type=Path)
    args = parser.parse_args()

    try:
        import rumoca as rm
    except ImportError as exc:
        raise SystemExit("error: Python module 'rumoca' is required") from exc

    if not args.model.is_file():
        raise SystemExit(f"error: Modelica file not found: {args.model}")

    source = args.model.read_text(encoding="utf-8")
    formatted = rm.format(source, filename=str(args.model))
    if formatted != source:
        print(f"error: Modelica formatting differs: {args.model}", file=sys.stderr)
        return 1

    diagnostics = rm.validate(str(args.model))
    failures = [diag for diag in diagnostics if diag.level in {"error", "warning"}]
    for diag in failures:
        location = args.model
        if diag.line is not None and diag.column is not None:
            location = Path(f"{args.model}:{diag.line}:{diag.column}")
        print(f"{location}: {diag.level}: {diag.message}", file=sys.stderr)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
