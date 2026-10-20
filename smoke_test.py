#!/usr/bin/env python3
"""Run a short end-to-end smoke test for all four paper controllers.

This test is intentionally not a numerical reproduction check.  It verifies
that OCC-MP, eOCC-MP, Transit-MP, and mTransit-MP can each execute for 1000 s
on the supplied Amsterdam scenario at 10% private-car CV penetration.
"""

from __future__ import annotations

import json
from pathlib import Path

from src.transit_mp import METHODS, SimulationCase, run_case


METHOD_ORDER = ["occ", "eocc", "transit", "mtransit"]
PENETRATION = 0.10
SEED = 1
MAX_TIME = 1000


def main() -> None:
    root = Path(__file__).resolve().parent
    output_root = root / "outputs" / "smoke_test_1000s_p010"
    output_root.mkdir(parents=True, exist_ok=True)

    rows = []
    failures = []

    print("Transit-MP reproducibility smoke test")
    print(f"  penetration = {PENETRATION:.2f}")
    print(f"  seed        = {SEED}")
    print(f"  horizon     = {MAX_TIME} s")
    print()

    for method in METHOD_ORDER:
        paper_name = METHODS[method]["paper_name"]
        run_dir = output_root / method
        print(f"[{paper_name}] starting ...")

        case = SimulationCase(
            method=method,
            penetration=PENETRATION,
            seed=SEED,
            max_time=MAX_TIME,
        )

        try:
            metrics = run_case(case, root, run_dir, overwrite=True)
            steps = int(metrics["simulated_steps"])
            expected_files = [
                run_dir / "run_config.json",
                run_dir / "tripinfo.xml",
                run_dir / "summary.xml",
                run_dir / "signal_states.csv",
                run_dir / "metrics.json",
            ]
            missing = [p.name for p in expected_files if not p.exists()]

            # For this supplied 3-h demand scenario, vehicles are present beyond
            # 1000 s.  Reaching exactly MAX_TIME therefore verifies that the
            # controller/SUMO loop survived the requested smoke-test horizon.
            ok = steps == MAX_TIME and not missing
            status = "PASS" if ok else "FAIL"
            print(f"[{paper_name}] {status}: simulated_steps={steps}")
            if missing:
                print(f"  missing outputs: {', '.join(missing)}")

            row = {
                "method": method,
                "paper_method": paper_name,
                "penetration": PENETRATION,
                "seed": SEED,
                "requested_steps": MAX_TIME,
                "simulated_steps": steps,
                "status": status,
                "missing_outputs": missing,
            }
            rows.append(row)
            if not ok:
                failures.append(row)
        except Exception as exc:  # keep testing remaining controllers
            row = {
                "method": method,
                "paper_method": paper_name,
                "penetration": PENETRATION,
                "seed": SEED,
                "requested_steps": MAX_TIME,
                "simulated_steps": None,
                "status": "ERROR",
                "error": f"{type(exc).__name__}: {exc}",
            }
            rows.append(row)
            failures.append(row)
            print(f"[{paper_name}] ERROR: {type(exc).__name__}: {exc}")

    report = {
        "purpose": "execution-only smoke test; numerical equality across SUMO versions is not required",
        "penetration": PENETRATION,
        "seed": SEED,
        "max_time_s": MAX_TIME,
        "runs": rows,
    }
    report_path = output_root / "smoke_test_report.json"
    report_path.write_text(json.dumps(report, indent=2, allow_nan=True), encoding="utf-8")

    print()
    print(f"Report: {report_path.resolve()}")
    if failures:
        print(f"Smoke test FAILED: {len(failures)} of {len(METHOD_ORDER)} controller(s) did not pass.")
        raise SystemExit(1)
    print("Smoke test PASSED: all four controllers completed 1000 s.")


if __name__ == "__main__":
    main()
