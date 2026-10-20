#!/usr/bin/env python3
"""Run one Transit-MP paper simulation case."""

from __future__ import annotations

import argparse
from pathlib import Path

from src.transit_mp import METHODS, SimulationCase, run_case


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--method", choices=sorted(METHODS), required=True,
                   help="Controller: transit, mtransit, occ, or eocc")
    p.add_argument("--penetration", type=float, required=True,
                   help="Private-car CV penetration rate in (0,1]")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--decision-interval", type=int, default=10)
    p.add_argument("--yellow-time", type=int, default=3)
    p.add_argument("--startup-lost-time", type=int, default=1)
    p.add_argument("--max-time", type=int, default=13000)
    p.add_argument("--history-window-seconds", type=int, default=900)
    p.add_argument("--private-cv-occupancy", type=int, default=2)
    p.add_argument("--min-transit-occupancy", type=int, default=20)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--save-fcd", action="store_true")
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--output", type=Path, default=None,
                   help="Output directory. Default: outputs/<method>/pXX_seedY")
    return p


def main() -> None:
    args = build_parser().parse_args()
    root = Path(__file__).resolve().parent
    case = SimulationCase(
        method=args.method,
        penetration=args.penetration,
        seed=args.seed,
        decision_interval=args.decision_interval,
        yellow_time=args.yellow_time,
        startup_lost_time=args.startup_lost_time,
        max_time=args.max_time,
        private_cv_occupancy=args.private_cv_occupancy,
        min_transit_occupancy=args.min_transit_occupancy,
        history_window_seconds=args.history_window_seconds,
        gui=args.gui,
        save_fcd=args.save_fcd,
    )
    output = args.output or (
        root / "outputs" / args.method / f"p{int(round(100 * args.penetration)):03d}_seed{args.seed}"
    )
    metrics = run_case(case, root, output, overwrite=args.overwrite)
    print(f"Finished {metrics['paper_method']} p={args.penetration:g} seed={args.seed}")
    print(f"Results: {output.resolve()}")
    print(f"Average vehicle delay: {metrics['vehicle_delay_s']:.3f} s/veh")
    print(f"Passenger delay: {metrics['passenger_delay_s']:.3f} s")
    print(f"Max spillover: {metrics['max_spillover_count']:.1f} veh")


if __name__ == "__main__":
    main()
