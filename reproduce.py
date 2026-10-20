#!/usr/bin/env python3
"""Run predefined experiment matrices from the Transit-MP paper."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterable, List, Tuple

from src.transit_mp import SimulationCase, run_case


DEFAULT_SEEDS = [1, 3, 6]


def cases_for(experiment: str) -> List[Tuple[str, float, int]]:
    cases: List[Tuple[str, float, int]] = []
    if experiment in {"fully-cv", "all"}:
        # mTransit-MP equals Transit-MP at 100% CV penetration.
        for method in ["occ", "eocc", "transit"]:
            for seed in DEFAULT_SEEDS:
                cases.append((method, 1.0, seed))
    if experiment in {"penetration", "all"}:
        for p in [0.1, 0.2, 0.3, 0.5, 0.7]:
            for method in ["occ", "eocc", "transit", "mtransit"]:
                for seed in DEFAULT_SEEDS:
                    cases.append((method, p, seed))
    if experiment == "sparse-cv":
        for method in ["eocc", "transit", "mtransit"]:
            for seed in DEFAULT_SEEDS:
                cases.append((method, 0.1, seed))
    return cases


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", choices=["fully-cv", "penetration", "sparse-cv", "all"], required=True)
    parser.add_argument("--dry-run", action="store_true", help="Print cases without starting SUMO")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    output_root = root / "outputs" / f"paper_{args.experiment}"
    cases = cases_for(args.experiment)

    print(f"Experiment: {args.experiment}; {len(cases)} simulation cases")
    for method, p, seed in cases:
        print(f"  {method:9s} p={p:.1f} seed={seed}")
    if args.dry_run:
        return

    rows = []
    for method, p, seed in cases:
        run_dir = output_root / method / f"p{int(round(100*p)):03d}_seed{seed}"
        case = SimulationCase(method=method, penetration=p, seed=seed, gui=args.gui)
        metrics = run_case(case, root, run_dir, overwrite=args.overwrite)
        rows.append(metrics)

        output_root.mkdir(parents=True, exist_ok=True)
        fieldnames = list(rows[0].keys())
        with (output_root / "aggregate_metrics.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    print(f"Aggregate results: {(output_root / 'aggregate_metrics.csv').resolve()}")


if __name__ == "__main__":
    main()
