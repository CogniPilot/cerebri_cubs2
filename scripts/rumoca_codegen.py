#!/usr/bin/env python3
"""Generate Rumoca code through the Python binding."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model_file", type=Path)
    parser.add_argument("--model", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        import rumoca as rm
    except ImportError as exc:
        raise SystemExit(
            "error: Python module 'rumoca' is required for Modelica code generation"
        ) from exc

    args.output.mkdir(parents=True, exist_ok=True)

    if hasattr(rm, "load"):
        model = rm.load(str(args.model_file), model=args.model)
        generated = model.codegen(args.target).save_all(str(args.output))
    else:
        generated = rm.Session().codegen_file(
            str(args.model_file),
            args.model,
            args.target,
            str(args.output),
        )

    for path in generated:
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
