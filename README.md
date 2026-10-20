# Transit-MP / mTransit-MP reproducibility code

Version **1.0.0**

This repository contains the cleaned reproducibility code associated with the following paper:

> **Tan, C., Liu, H., Sun, D., Rinaldi, M., & van Lint, H. (2027).** Transit-MP: Transit-prioritized max-pressure control in sparse connected vehicle environments. *Transportation Research Part B: Methodological*, *215*, 103622.  
> [https://doi.org/10.1016/j.trb.2026.103622](https://doi.org/10.1016/j.trb.2026.103622)

The package retains the Amsterdam SUMO scenario and the four signal-control methods used in the paper comparison:

| CLI name | Paper method | Station-aware transit treatment | Sparse-CV historical fallback |
|---|---|---:|---:|
| `occ` | OCC-MP | No | No |
| `eocc` | eOCC-MP | Yes | No |
| `transit` | Transit-MP | Yes | No |
| `mtransit` | mTransit-MP | Yes | Yes |

The code has been cleaned from the original research working directory while preserving the core controller implementation, Amsterdam movement/phase mappings, transit-station logic, and experiment inputs. Development-only scripts, unrelated experiments, duplicate controller versions, generated outputs, warning logs, and local plotting utilities have been removed.

## 1. SUMO version and reproducibility note

The experiments reported in the paper were conducted using **Eclipse SUMO 1.19.0**. The supplied SUMO configuration and network-related files also retain their original SUMO 1.19.0 generation metadata.

We recommend **SUMO 1.19.0** when the goal is to reproduce the original simulation environment as closely as possible.

The code may also run with newer SUMO releases, but users should be aware of two compatibility issues:

1. **TraCI API compatibility.** Some functions used by the original implementation may be deprecated or may become unavailable in newer SUMO versions. For example, newer SUMO releases may warn that `traci.vehicle.getNextStops()` is deprecated. Such warnings do not affect execution as long as the function remains supported, but future releases may require equivalent API updates.
2. **Numerical reproducibility.** SUMO updates can change microscopic simulation behavior, vehicle dynamics, routing details, or API behavior. Therefore, numerical results may differ across SUMO versions even when the Transit-MP controller implementation is unchanged. Exact equality with the numbers reported in the paper is not required for validating the implementation.

For this release, functional validation consists of confirming that all four controllers can execute normally under the same scenario and settings. A 1000-s smoke test at 10% private-car CV penetration and seed 1 was completed successfully for **OCC-MP, eOCC-MP, Transit-MP, and mTransit-MP**.

## 2. Amsterdam scenario and paper-default settings

The supplied scenario contains three signalized intersections in Amsterdam, 7 tram flows, 8 bus flows, and 31 transit stops. Private traffic contains connected (`veh_CV`) and non-connected (`veh_car`) vehicles. Transit vehicles are treated as connected vehicles in the supplied experiments.

Paper-default controller settings are:

- decision interval: **10 s**
- yellow transition: **3 s**
- green start-up lost time used in the switching penalty: **1 s**
- total switching loss used in phase-pressure comparison: **4 s**
- private-CV occupancy used by the implementation: **2 persons/vehicle**
- minimum bus/tram occupancy used by the implementation: **20 persons/vehicle**
- demand period: **0–10800 s (3 h)**
- simulation continues until the network clears or **13000 s**, whichever comes first

For mTransit-MP, `data/link_volumes_15min_darkFree.csv` contains the historical movement volumes used by the zero-CV-observation fallback. The supplied file contains 12 15-minute slices and 62 controlled movements per slice.

## 3. Requirements

Install the Python dependencies:

```bash
pip install -r requirements.txt
```

SUMO itself is an external dependency and is therefore not installed through `requirements.txt`.

Set `SUMO_HOME` so Python can import the SUMO tools.

Linux/macOS example:

```bash
export SUMO_HOME=/path/to/sumo
```

Windows PowerShell example:

```powershell
$env:SUMO_HOME="C:\Program Files (x86)\Eclipse\Sumo"
```

Validate the static inputs before running simulations:

```bash
python tools/validate_inputs.py
```

Expected validation summary:

```text
Input validation passed.
  Transit stops: 31
  Bus lines/flows: 8
  Tram lines/flows: 7
  Historical flow data: 744 rows, 12 x 15-min slices, 62 movements/slice
```

## 4. Quick smoke test

Run:

```bash
python smoke_test.py
```

The smoke test executes the four controllers with common settings:

```text
CV penetration = 0.10
seed           = 1
horizon        = 1000 s
```

A run passes when the controller reaches the requested 1000-s horizon and writes the expected output files. The test is intended to verify executable controller logic rather than numerical agreement across different SUMO versions.

Outputs are written to:

```text
outputs/smoke_test_1000s_p010/
```

including `smoke_test_report.json`.

## 5. Run one simulation

### OCC-MP

```bash
python run.py --method occ --penetration 0.1 --seed 1
```

### eOCC-MP

```bash
python run.py --method eocc --penetration 0.1 --seed 1
```

### Transit-MP

```bash
python run.py --method transit --penetration 0.1 --seed 1
```

### mTransit-MP

```bash
python run.py --method mtransit --penetration 0.1 --seed 1
```

Use `--gui` to launch `sumo-gui`; otherwise the command-line SUMO binary is used.

Each run is stored under a method- and scenario-specific directory, for example:

```text
outputs/
└── mtransit/
    └── p010_seed1/
        ├── input/              # exact runtime copy of the scenario
        ├── run_config.json
        ├── tripinfo.xml
        ├── summary.xml
        ├── signal_states.csv
        └── metrics.json
```

The source scenario under `scenario/amsterdam/` is never modified. CV penetration is applied only to the runtime copy written to the output directory.

## 6. Reproduce the main experiment matrices

Inspect the planned cases without launching SUMO:

```bash
python reproduce.py --experiment penetration --dry-run
```

Available presets are:

```bash
python reproduce.py --experiment fully-cv
python reproduce.py --experiment penetration
python reproduce.py --experiment sparse-cv
python reproduce.py --experiment all
```

Default replication seeds are `1, 3, 6`, matching the final supplied mTransit-MP working script. The penetration preset evaluates `0.1, 0.2, 0.3, 0.5, 0.7` for all four methods. At 100% CV penetration, mTransit-MP is operationally equivalent to Transit-MP in the supplied setting and is not run separately in the `fully-cv` preset.

Batch runs write `aggregate_metrics.csv`, including average vehicle delay, passenger delay, maximum in-network vehicle count, maximum spillover count, maximum unserved count, and method-specific metadata.

## 7. Method implementation

The common implementation is in `src/transit_mp.py`.

**OCC-MP.** The upstream queue-related pressure is weighted by average upstream occupancy. Transit-station dwell is not excluded.

**eOCC-MP.** The same occupancy-weighted queue-based pressure is used, while transit vehicles contribute only when they are eligible according to the station-aware rule.

**Transit-MP.** The upstream state uses occupancy-weighted normalized link travel time, while the downstream state uses vehicle-based normalized link travel time. Transit vehicles still dwelling before the relevant station are excluded from the effective state.

**mTransit-MP.** Transit-MP is used whenever real-time CV observations are available. When a grouped private-traffic movement has zero effective CV observation, the historical movement volume and current movement vehicle count are used to construct the fallback state.

The cleaned implementation retains the movement groups, saturation-flow values, phase definitions, link mappings, and station logic of the supplied final research code.

## 8. Repository structure

```text
.
├── README.md
├── CITATION.cff
├── LICENSE
├── VERSION
├── requirements.txt
├── run.py
├── reproduce.py
├── smoke_test.py
├── src/
│   └── transit_mp.py
├── scenario/
│   └── amsterdam/
│       ├── 1_1_osm.net.xml
│       ├── 1_osm.bus.trips.xml
│       ├── 1_1_osm.tram.trips.xml
│       ├── 1_osm.car.trips.xml
│       ├── 1_osm_stops.add.xml
│       ├── 1_osm.sumocfg
│       └── osm.view.xml
├── data/
│   ├── Edge_length_speed.txt
│   ├── For_correcting_LTT.txt
│   ├── In_out_pairs.txt
│   ├── Signal_phase.txt
│   └── link_volumes_15min_darkFree.csv
├── tools/
│   └── validate_inputs.py
└── outputs/
```

## 9. Citation

If you use this code, please cite:

> **Tan, C., Liu, H., Sun, D., Rinaldi, M., & van Lint, H. (2027).** Transit-MP: Transit-prioritized max-pressure control in sparse connected vehicle environments. *Transportation Research Part B: Methodological*, *215*, 103622.  
> [https://doi.org/10.1016/j.trb.2026.103622](https://doi.org/10.1016/j.trb.2026.103622)

Machine-readable citation metadata are provided in `CITATION.cff`. GitHub can use this file to expose a **Cite this repository** entry on the repository page.

## 10. License

This repository is licensed under the **GNU General Public License v3.0 (GPL-3.0)**. See `LICENSE` for the full license text.

The copyright remains with the copyright holders. GPL-3.0 permits use, modification, and redistribution subject to its copyleft terms; distributed derivative works must comply with the GPL-3.0 requirements.

Copyright © 2026 Chaopeng Tan and contributors.
