#!/usr/bin/env python3
"""Run a Rumoca scenario through the Python binding."""

from __future__ import annotations

import argparse
import inspect
from pathlib import Path
import sys
import tomllib
from typing import Any, Callable


SCENARIO_RUNNER_UNAVAILABLE_EXIT = 77


def call_with_supported_kwargs(fn: Callable[..., Any], scenario: Path) -> Any:
    attempts = [
        ((), {"config": str(scenario)}),
        ((), {"path": str(scenario)}),
        ((), {"scenario": str(scenario)}),
        ((str(scenario),), {}),
    ]
    for args, kwargs in attempts:
        try:
            return fn(*args, **kwargs)
        except TypeError:
            continue
    signature = "unknown"
    try:
        signature = str(inspect.signature(fn))
    except (TypeError, ValueError):
        pass
    raise TypeError(f"could not call {fn!r} with scenario path; signature {signature}")


def find_scenario_runner(rm: Any) -> Callable[[Path], Any]:
    names = (
        "run_scenario",
        "run_scenario_file",
        "simulate_scenario",
        "simulate_scenario_file",
        "run",
        "simulate",
    )
    for name in names:
        candidate = getattr(rm, name, None)
        if callable(candidate):
            return lambda scenario, candidate=candidate: call_with_supported_kwargs(candidate, scenario)

    scenario_type = getattr(rm, "Scenario", None)
    if scenario_type is not None:
        for constructor in ("from_file", "from_path", "load"):
            build = getattr(scenario_type, constructor, None)
            if not callable(build):
                continue
            scenario_obj = build

            def run_scenario(scenario: Path, build: Callable[..., Any] = scenario_obj) -> Any:
                obj = call_with_supported_kwargs(build, scenario)
                for method_name in ("run", "simulate"):
                    method = getattr(obj, method_name, None)
                    if callable(method):
                        return method()
                raise AttributeError("Rumoca Scenario object has no run/simulate method")

            return run_scenario

    session_type = getattr(rm, "Session", None)
    if session_type is not None:
        for name in ("run_scenario", "run_scenario_file", "simulate_scenario", "simulate_scenario_file"):
            candidate = getattr(session_type, name, None)
            if callable(candidate):
                return lambda scenario, candidate=candidate: call_with_supported_kwargs(candidate, scenario)

        try:
            session = session_type()
        except TypeError:
            session = None
        if session is not None:
            for name in ("run_scenario", "run_scenario_file", "simulate_scenario", "simulate_scenario_file"):
                candidate = getattr(session, name, None)
                if callable(candidate):
                    return lambda scenario, candidate=candidate: call_with_supported_kwargs(candidate, scenario)

    raise AttributeError(
        "the installed Rumoca Python binding does not expose a scenario runner; "
        "expected a run_scenario/simulate_scenario function, Session method, or Scenario class"
    )


def scenario_data(scenario: Path) -> dict[str, Any]:
    return tomllib.loads(scenario.read_text(encoding="utf-8"))


def requires_interactive_runner(data: dict[str, Any]) -> bool:
    sim = data.get("sim", {})
    return sim.get("mode") == "lockstep" or "lockstep" in data or data.get("transport") is not None


def missing_runner_message(data: dict[str, Any]) -> str:
    if requires_interactive_runner(data):
        return (
            "this scenario uses interactive transport/lockstep settings and "
            "requires Rumoca's Python scenario runner"
        )
    return (
        "the installed Rumoca Python binding does not expose a scenario runner; "
        "falling back to batch simulation is possible only for non-interactive scenarios"
    )


def run_batch_fallback(rm: Any, scenario: Path) -> int:
    data = scenario_data(scenario)
    sim = data.get("sim", {})
    if requires_interactive_runner(data):
        print(f"error: {missing_runner_message(data)}", file=sys.stderr)
        return SCENARIO_RUNNER_UNAVAILABLE_EXIT

    _session, model, config = rm.Session.from_scenario(str(scenario))
    t_end = float(sim.get("t_end", 1.0))
    result = model.simulate(t=(0.0, t_end), config=config)
    output = sim.get("output")
    if output:
        import csv

        output_path = Path(output)
        if not output_path.is_absolute():
            output_path = scenario.parent / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        names = ["time", *result.names]
        with output_path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(names)
            for idx, time_s in enumerate(result.time):
                writer.writerow([float(time_s), *[float(result[name][idx]) for name in result.names]])
        print(f"wrote {output_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True, type=Path)
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="only verify that the installed binding can run this scenario",
    )
    args = parser.parse_args()
    data = scenario_data(args.config)

    try:
        import rumoca as rm
    except ImportError as exc:
        raise SystemExit("error: Python module 'rumoca' is required") from exc

    try:
        runner = find_scenario_runner(rm)
    except AttributeError:
        if args.check_only:
            if requires_interactive_runner(data):
                print(f"error: {missing_runner_message(data)}", file=sys.stderr)
                return SCENARIO_RUNNER_UNAVAILABLE_EXIT
            return 0
        return run_batch_fallback(rm, args.config)

    if args.check_only:
        return 0
    runner(args.config)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
