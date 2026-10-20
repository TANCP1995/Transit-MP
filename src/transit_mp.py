"""Core simulation code for the Transit-MP paper.

This module is a cleaned version of the authors' research scripts.  It keeps
four controllers used in the paper:

* OCC-MP
* eOCC-MP
* Transit-MP
* mTransit-MP

The controller equations and Amsterdam movement/phase mappings are preserved
from the supplied final research code.  Experiment configuration and file I/O
have been separated from the controller logic so that a fresh checkout does
not modify the source demand files.
"""

from __future__ import annotations

import copy
import csv
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple

import numpy as np
import pandas as pd


TLS_IDS = ("TL_ZZ", "TL_IM", "TL_WL")
NODE_NAMES = ("ZZ", "IM", "WL")
MATRIX_SIZES = {"ZZ": 16, "IM": 22, "WL": 24}

# Stops immediately relevant to the station-aware pressure calculation.
TARGET_STOPS = {
    "3299908041",
    "434507140",
    "434507138",
    "2674521970",
    "bs_0",
    "4621136512",
    "434524954",
    "3303180663",
}

# Source links on which a transit vehicle is counted directly.
TRANSIT_SOURCE_EDGES = {
    "770446896#0",
    "754704157",
    "-770426572#0",
    "770426567",
    "770426566#0",
    "325023862#0",
    "770434440",
    "7045269",
    "-814779818",
    "122385065#0",
}

# mTransit-MP fallback groups for private-traffic movements.
FALLBACK_GROUPS = {
    "ZZ": np.array([[0, 1, 3], [5, 6, 7], [8, 9, 10], [13, 14, 15]], dtype=int),
    "IM": np.array([[0, 1, 2], [6, 8, 9], [11, 12, 13], [17, 19, 20]], dtype=int),
    "WL": np.array([[0, 1, 2], [6, 7, 8], [12, 13, 14], [18, 19, 20]], dtype=int),
}

# Downstream movement classification used in the final supplied code.
DOWNSTREAM_CLASSIFICATION = {
    "ZZ": {
        "left": {3, 7, 10, 15},
        "right": {0, 5, 8, 13},
    },
    "IM": {
        "left": {2, 6, 11, 12, 15, 18, 19, 23, 24},
        "right": {0, 3, 4, 7, 8, 13, 16, 20, 21, 25},
    },
    "WL": {
        "left": {2, 6, 9, 12, 15, 18, 21, 24},
        "right": {0, 3, 7, 10, 13, 16, 19, 22},
    },
}

METHODS = {
    "transit": {"paper_name": "Transit-MP", "pressure_index": 0, "station_aware": True, "fallback": False},
    "mtransit": {"paper_name": "mTransit-MP", "pressure_index": 0, "station_aware": True, "fallback": True},
    "occ": {"paper_name": "OCC-MP", "pressure_index": 1, "station_aware": False, "fallback": False},
    "eocc": {"paper_name": "eOCC-MP", "pressure_index": 2, "station_aware": True, "fallback": False},
}


@dataclass(frozen=True)
class SimulationCase:
    method: str
    penetration: float
    seed: int
    decision_interval: int = 10
    yellow_time: int = 3
    startup_lost_time: int = 1
    max_time: int = 13000
    private_cv_occupancy: int = 2
    min_transit_occupancy: int = 20
    history_window_seconds: int = 900
    gui: bool = False
    save_fcd: bool = False

    @property
    def lost_time(self) -> int:
        return self.yellow_time + self.startup_lost_time

    def validate(self) -> None:
        if self.method not in METHODS:
            raise ValueError(f"Unknown method {self.method!r}; choose from {sorted(METHODS)}")
        if not (0.0 < self.penetration <= 1.0):
            raise ValueError("penetration must be in (0, 1]")
        if self.decision_interval <= self.yellow_time:
            raise ValueError("decision_interval must exceed yellow_time")
        if self.lost_time >= self.decision_interval:
            raise ValueError("yellow_time + startup_lost_time must be smaller than decision_interval")
        if self.history_window_seconds <= 0:
            raise ValueError("history_window_seconds must be positive")


@dataclass
class StaticData:
    link_start_edge: Dict[str, str]
    edge_info: Dict[str, Tuple[float, float]]
    in_out: Dict[str, Tuple[str, List[int]]]
    phases: Dict[str, List[str]]
    historical_volumes: pd.DataFrame


class TravelTimeTracker:
    """Track the latest entry time of each vehicle into each physical edge."""

    def __init__(self) -> None:
        self.current_edge: Dict[str, str] = {}
        self.entry_time: Dict[Tuple[str, str], int] = {}

    def update(self, vehicle_id: str, edge_id: str, time_s: int) -> None:
        if self.current_edge.get(vehicle_id) != edge_id:
            self.current_edge[vehicle_id] = edge_id
            self.entry_time[(vehicle_id, edge_id)] = time_s

    def link_travel_time(self, vehicle_id: str, start_edge: str, time_s: int) -> float:
        enter = self.entry_time.get((vehicle_id, start_edge))
        return 0.0 if enter is None else float(time_s - enter)


def _load_two_col(path: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    with path.open("r", encoding="utf-8") as f:
        for row in csv.reader(f, delimiter=" "):
            row = [x for x in row if x != ""]
            if row:
                out[row[0]] = row[1]
    return out


def load_static_data(data_dir: Path) -> StaticData:
    link_start_edge = _load_two_col(data_dir / "For_correcting_LTT.txt")

    edge_info: Dict[str, Tuple[float, float]] = {}
    with (data_dir / "Edge_length_speed.txt").open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            edge, length, speed = line.split()
            edge_info[edge] = (float(length), float(speed))

    in_out: Dict[str, Tuple[str, List[int]]] = {}
    with (data_dir / "In_out_pairs.txt").open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            edge, node, streams = line.split()
            in_out[edge] = (node, [int(x) for x in streams.split("_")])

    phases: Dict[str, List[str]] = {node: [] for node in NODE_NAMES}
    with (data_dir / "Signal_phase.txt").open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            fields = line.split()
            phases[fields[0]].append(fields[1])

    historical = pd.read_csv(data_dir / "link_volumes_15min_darkFree.csv")
    historical["sliceIndex"] = historical["sliceIndex"].astype(int)
    historical["linkIndex"] = historical["linkIndex"].astype(int)
    historical["volumeVeh"] = historical["volumeVeh"].astype(float)

    return StaticData(
        link_start_edge=link_start_edge,
        edge_info=edge_info,
        in_out=in_out,
        phases=phases,
        historical_volumes=historical,
    )


def import_sumo_tools():
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        raise RuntimeError(
            "SUMO_HOME is not set. Install SUMO and set SUMO_HOME to its installation directory."
        )
    tools = str(Path(sumo_home) / "tools")
    if tools not in sys.path:
        sys.path.append(tools)
    try:
        from sumolib import checkBinary  # type: ignore
        import traci  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Could not import SUMO Python tools from $SUMO_HOME/tools. Check your SUMO installation."
        ) from exc
    return checkBinary, traci


def set_penetration_rate(route_file: Path, penetration: float) -> None:
    """Set the private-car CV share in a runtime copy of the route file."""
    tree = ET.parse(route_file)
    root = tree.getroot()
    target = None
    for elem in root.iter("vTypeDistribution"):
        if elem.get("id") == "mixedtype":
            target = elem
            break
    if target is None:
        raise ValueError(f"No vTypeDistribution id='mixedtype' in {route_file}")
    target.set("vTypes", "veh_car veh_CV")
    target.set("probabilities", f"{1.0 - penetration:.12g} {penetration:.12g}")
    tree.write(route_file, encoding="UTF-8", xml_declaration=True)


def prepare_runtime_inputs(scenario_dir: Path, run_dir: Path, penetration: float) -> Path:
    input_dir = run_dir / "input"
    if input_dir.exists():
        shutil.rmtree(input_dir)
    input_dir.mkdir(parents=True, exist_ok=True)

    required = [
        "1_1_osm.net.xml",
        "1_osm.bus.trips.xml",
        "1_1_osm.tram.trips.xml",
        "1_osm.car.trips.xml",
        "1_osm_stops.add.xml",
        "1_osm.sumocfg",
        "osm.view.xml",
    ]
    for name in required:
        src = scenario_dir / name
        if not src.exists():
            raise FileNotFoundError(src)
        shutil.copy2(src, input_dir / name)

    set_penetration_rate(input_dir / "1_osm.car.trips.xml", penetration)
    return input_dir / "1_osm.sumocfg"


def _station_eligible(traci, vehicle_id: str, current_edge: str, next_tls, decision_interval: int) -> int:
    """Return beta_v for a transit vehicle in the supplied Amsterdam scenario."""
    depart_index = 0
    next_stops = traci.vehicle.getNextStops(vehicle_id)
    previous_stops = traci.vehicle.getStops(vehicle_id, limit=-1)

    if next_stops:
        next_stop = next_stops[0]
        stop_id = next_stop[2]
        dwell_or_until = next_stop[4]
        condition1 = stop_id in TARGET_STOPS and (dwell_or_until <= 1 or dwell_or_until > 10000)
    else:
        condition1 = False

    if previous_stops:
        last_stop = previous_stops[0]
        try:
            last_stop_id = last_stop.stoppingPlaceID
            last_lane = last_stop.lane
        except AttributeError:
            # Fallback for tuple-like StopData in other SUMO releases.
            last_stop_id = getattr(last_stop, "stoppingPlaceID", "")
            last_lane = getattr(last_stop, "lane", "")
        # Preserve the original research-script condition exactly.  When no
        # downstream TLS exists, next_tls[2] is -1 and the distance test is
        # therefore true; the stop/edge conditions still determine beta_v.
        near_tls = next_tls[2] <= traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle_id)) * decision_interval
        condition2 = last_stop_id in TARGET_STOPS and current_edge == last_lane[:-2] and near_tls
    else:
        condition2 = False

    condition3 = current_edge in TRANSIT_SOURCE_EDGES
    if condition1 or condition2 or condition3:
        depart_index = 1
    return depart_index


def extract_cv_information(
    traci,
    cv_ids: Sequence[str],
    static: StaticData,
    tracker: TravelTimeTracker,
    time_s: int,
    case: SimulationCase,
) -> List[List[object]]:
    current_edge_position: List[List[object]] = []

    for vehicle_id in cv_ids:
        current_edge = traci.vehicle.getRoadID(vehicle_id)
        if current_edge not in static.edge_info:
            continue

        edge_length, edge_speed = static.edge_info[current_edge]
        edge_ett = edge_length / edge_speed
        start_edge = static.link_start_edge.get(current_edge, current_edge)

        vtype = traci.vehicle.getTypeID(vehicle_id)
        current_position = traci.vehicle.getLanePosition(vehicle_id)
        occupancy = traci.vehicle.getPersonNumber(vehicle_id)
        if vtype in {"tram_tram", "bus_bus"} and occupancy < case.min_transit_occupancy:
            occupancy = case.min_transit_occupancy
        if vtype == "veh_CV":
            occupancy = case.private_cv_occupancy

        tls_list = traci.vehicle.getNextTLS(vehicle_id)
        next_tls = tls_list[0] if tls_list else (-1, -1, -1, -1)

        tracker.update(vehicle_id, current_edge, time_s)
        ltt = tracker.link_travel_time(vehicle_id, start_edge, time_s)

        depart_index = 1
        if METHODS[case.method]["station_aware"] and vtype in {"bus_bus", "tram_tram"}:
            depart_index = _station_eligible(traci, vehicle_id, current_edge, next_tls, case.decision_interval)

        current_edge_position.append(
            [
                vehicle_id, current_edge, current_position, ltt, occupancy,
                next_tls[0], int(next_tls[1]), next_tls[2], next_tls[3],
                edge_length, edge_ett, depart_index,
            ]
        )

    return current_edge_position


def initialize_weights() -> Dict[str, np.ndarray]:
    weights = {
        "ZZ": np.zeros((16, 21), dtype=float),
        "IM": np.zeros((22, 21), dtype=float),
        "WL": np.zeros((24, 21), dtype=float),
    }
    for mat in weights.values():
        mat[:, 20] = 0.5
    weights["ZZ"][[4, 11, 12], 20] = 0.4
    weights["IM"][[3, 4, 5, 10, 14, 15, 16, 18, 21], 20] = 0.4
    weights["WL"][[3, 4, 5, 9, 10, 11, 15, 16, 17, 21, 22, 23], 20] = 0.4
    return weights


def update_incoming(vehinfo: Sequence[object], weights: MutableMapping[str, np.ndarray]) -> None:
    tls_id = str(vehinfo[5])
    node = {"TL_ZZ": "ZZ", "TL_IM": "IM", "TL_WL": "WL"}.get(tls_id)
    if node is None:
        return
    idx = int(vehinfo[6])
    mat = weights[node]
    if idx < 0 or idx >= len(mat):
        return

    ltt = float(vehinfo[3])
    occupancy = float(vehinfo[4])
    edge_length = float(vehinfo[9])
    edge_ett = float(vehinfo[10])
    beta = float(vehinfo[11])

    # Transit-MP upstream state: occupancy-weighted normalized link travel time.
    mat[idx, 0] += ltt * occupancy / edge_ett * beta

    # OCC-MP / eOCC-MP queue-related upstream states.
    mat[idx, 5] += 1.0 / np.sqrt(edge_length) * beta
    mat[idx, 10] += 1.0 / np.sqrt(edge_length) * beta

    # Average upstream occupancy used by OCC-MP/eOCC-MP.
    mat[idx, 15] += occupancy * beta
    mat[idx, 19] += beta


def update_outgoing(
    vehinfo: Sequence[object],
    static: StaticData,
    weights: MutableMapping[str, np.ndarray],
) -> None:
    current_edge = str(vehinfo[1])
    mapped = static.in_out.get(current_edge)
    if mapped is None:
        return
    out_node, out_streams = mapped
    mat = weights[out_node]
    classification = DOWNSTREAM_CLASSIFICATION[out_node]
    movement_idx = int(vehinfo[6])

    if movement_idx < 0:
        bundles = {"A": (1, 2, 3), "B": (6, 7, 8), "C": (11, 12, 13), "D": (16, 17, 18)}
    elif movement_idx in classification["left"]:
        bundles = {"A": (1,), "B": (6,), "C": (11,), "D": (16,)}
    elif movement_idx in classification["right"]:
        bundles = {"A": (3,), "B": (8,), "C": (13,), "D": (18,)}
    else:
        bundles = {"A": (2,), "B": (7,), "C": (12,), "D": (17,)}

    ltt = float(vehinfo[3])
    occupancy = float(vehinfo[4])
    edge_length = float(vehinfo[9])
    edge_ett = float(vehinfo[10])
    beta = float(vehinfo[11])

    # The final Transit-MP code uses a vehicle-based (not occupancy-weighted)
    # downstream state.  OCC/eOCC downstream terms are queue-count based.
    d_a = ltt / edge_ett * beta
    d_b = 1.0 / np.sqrt(edge_length) * beta
    d_c = 1.0 / np.sqrt(edge_length) * beta
    d_d = occupancy * beta

    for stream in out_streams:
        for col in bundles["A"]:
            mat[stream, col] += d_a
        for col in bundles["B"]:
            mat[stream, col] += d_b
        for col in bundles["C"]:
            mat[stream, col] += d_c
        for col in bundles["D"]:
            mat[stream, col] += d_d


def stream_pressure(weight: np.ndarray) -> np.ndarray:
    # Turning ratios are estimated from the observed downstream movement counts.
    total = weight[:, 11] + weight[:, 12] + weight[:, 13]
    for i in range(len(weight)):
        if total[i] <= 0:
            continue
        left = weight[i, 11] / total[i]
        straight = weight[i, 12] / total[i]
        right = weight[i, 13] / total[i]
        weight[i, 1] *= left
        weight[i, 2] *= straight
        weight[i, 3] *= right
        weight[i, 6] *= left
        weight[i, 7] *= straight
        weight[i, 8] *= right
        weight[i, 11] *= left
        weight[i, 12] *= straight
        weight[i, 13] *= right

    observed = weight[:, 19] != 0
    # col 4: Transit-MP, col 9: OCC-MP, col 14: eOCC-MP
    weight[:, 4] = (weight[:, 0] - (weight[:, 1] + weight[:, 2] + weight[:, 3])) * weight[:, 20]
    weight[observed, 9] = (
        (weight[observed, 5] - (weight[observed, 6] + weight[observed, 7] + weight[observed, 8]))
        * weight[observed, 20]
        * weight[observed, 15]
        / weight[observed, 19]
    )
    weight[observed, 14] = (
        (weight[observed, 10] - (weight[observed, 11] + weight[observed, 12] + weight[observed, 13]))
        * weight[observed, 20]
        * weight[observed, 15]
        / weight[observed, 19]
    )
    return weight


def count_zero_cv_groups(weights: Mapping[str, np.ndarray]) -> int:
    n = 0
    for node, groups in FALLBACK_GROUPS.items():
        for movements in groups:
            if float(weights[node][movements, 0].sum()) == 0.0:
                n += 1
    return n


def _get_from_edge(traci, tls_id: str, movement_idx: int) -> str:
    # This preserves the mapping used in the supplied final research script.
    links_nested = traci.trafficlight.getControlledLinks(tls_id)
    flat_links = [link for group in links_nested for link in group]
    in_lane = flat_links[movement_idx][0]
    return traci.lane.getEdgeID(in_lane)


def estimate_fallback_states(
    traci,
    static: StaticData,
    time_s: int,
    penetration: float,
    history_window_seconds: int,
) -> Dict[str, np.ndarray]:
    estimates = {
        "ZZ": np.zeros((16, 5), dtype=float),
        "IM": np.zeros((22, 5), dtype=float),
        "WL": np.zeros((24, 5), dtype=float),
    }

    for vehicle_id in traci.vehicle.getIDList():
        tls_info = traci.vehicle.getNextTLS(vehicle_id)
        if not tls_info:
            continue
        tls_id, movement_idx = tls_info[0][0], int(tls_info[0][1])
        node = {"TL_ZZ": "ZZ", "TL_IM": "IM", "TL_WL": "WL"}.get(tls_id)
        if node is not None and 0 <= movement_idx < len(estimates[node]):
            estimates[node][movement_idx, 2] += 1.0  # Q_io in the implementation

    slice_index = time_s // history_window_seconds + 1
    rows = static.historical_volumes[static.historical_volumes["sliceIndex"] == slice_index]
    for row in rows.itertuples(index=False):
        tls_id = str(row.tlID)
        movement_idx = int(row.linkIndex)
        node = {"TL_ZZ": "ZZ", "TL_IM": "IM", "TL_WL": "WL"}.get(tls_id)
        if node is None or not (0 <= movement_idx < len(estimates[node])):
            continue

        from_edge = _get_from_edge(traci, tls_id, movement_idx)
        if from_edge not in static.edge_info:
            continue
        edge_length, edge_speed = static.edge_info[from_edge]
        edge_ett = edge_length / edge_speed
        arrival_rate = float(row.volumeVeh) / history_window_seconds
        q = estimates[node][movement_idx, 2]
        estimates[node][movement_idx, 3] = arrival_rate

        if q <= 0:
            estimated_tt = 0.0
        elif arrival_rate <= 0:
            # Limiting behavior of the original expression as lambda -> 0+.
            estimated_tt = np.inf
        else:
            estimated_tt = penetration * (q + 0.5 * q * q / (arrival_rate * edge_ett))
        estimates[node][movement_idx, 4] = estimated_tt

    return estimates


def apply_fallback(weights: MutableMapping[str, np.ndarray], estimates: Mapping[str, np.ndarray]) -> None:
    for node, groups in FALLBACK_GROUPS.items():
        for movements in groups:
            if float(weights[node][movements, 0].sum()) == 0.0:
                weights[node][movements, 0] = estimates[node][movements, 4]


def compute_phase_pressures(phases: Mapping[str, Sequence[str]], weights: Mapping[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """Return [Transit, OCC, eOCC] pressure for every feasible phase."""
    out: Dict[str, np.ndarray] = {}
    for node in NODE_NAMES:
        values = np.zeros((len(phases[node]), 3), dtype=float)
        mat = weights[node]
        for phase_idx, state in enumerate(phases[node]):
            active = [i for i, char in enumerate(state) if char != "r"]
            if active:
                values[phase_idx, 0] = mat[active, 4].sum()
                values[phase_idx, 1] = mat[active, 9].sum()
                values[phase_idx, 2] = mat[active, 14].sum()
        out[node] = values
    return out


def choose_signal_plan(
    phases: Mapping[str, Sequence[str]],
    phase_pressures: Mapping[str, np.ndarray],
    last_phase: Sequence[int],
    case: SimulationCase,
) -> Tuple[List[str], List[int]]:
    plans: List[str] = []
    phase_indices: List[int] = []
    method_idx = int(METHODS[case.method]["pressure_index"])
    switch_factor = (case.decision_interval - case.lost_time) / case.decision_interval

    for node_i, node in enumerate(NODE_NAMES):
        adjusted = phase_pressures[node].copy() * switch_factor
        if last_phase[node_i] >= 0:
            adjusted[last_phase[node_i], :] /= switch_factor

        selected_column = adjusted[:, method_idx]
        if np.max(selected_column) == 0:
            max_phase = (last_phase[node_i] + 1) % len(selected_column)
        else:
            max_phase = int(np.argmax(selected_column))
        plans.append(phases[node][max_phase])
        phase_indices.append(max_phase)

    return plans, phase_indices


def parse_metrics(tripinfo_xml: Path, summary_xml: Path) -> Dict[str, float]:
    delays: Dict[str, List[float]] = {"bus_bus": [], "tram_tram": [], "veh_CV": [], "veh_car": []}
    stop_counts: Dict[str, List[float]] = {"bus_bus": [], "tram_tram": [], "veh_CV": [], "veh_car": []}
    all_delays: List[float] = []
    passenger_delays: List[float] = []

    root = ET.parse(tripinfo_xml).getroot()
    for trip in root.findall("tripinfo"):
        vtype = trip.get("vType", "")
        delay = float(trip.get("timeLoss", "nan"))
        stops = float(trip.get("waitingCount", "0"))
        if np.isfinite(delay):
            all_delays.append(delay)
            if vtype in delays:
                delays[vtype].append(delay)
                stop_counts[vtype].append(stops)

    for person in root.findall("personinfo"):
        for ride in person.findall("ride"):
            val = float(ride.get("timeLoss", "nan"))
            if np.isfinite(val):
                passenger_delays.append(val)

    running: List[float] = []
    halting: List[float] = []
    waiting: List[float] = []
    summary_root = ET.parse(summary_xml).getroot()
    for step in summary_root.findall("step"):
        running.append(float(step.get("running", 0)))
        halting.append(float(step.get("halting", 0)))
        waiting.append(float(step.get("waiting", 0)))

    def mean_or_nan(values: Sequence[float]) -> float:
        return float(np.mean(values)) if values else float("nan")

    def sum_or_zero(values: Sequence[float]) -> float:
        return float(np.sum(values)) if values else 0.0

    max_running = max(running, default=0.0)
    max_waiting = max(waiting, default=0.0)
    max_halting = max(halting, default=0.0)
    max_unserved = max((r + w for r, w in zip(running, waiting)), default=0.0)

    return {
        "bus_delay_s": mean_or_nan(delays["bus_bus"]),
        "tram_delay_s": mean_or_nan(delays["tram_tram"]),
        "cv_delay_s": mean_or_nan(delays["veh_CV"]),
        "nv_delay_s": mean_or_nan(delays["veh_car"]),
        "vehicle_delay_s": mean_or_nan(all_delays),
        "passenger_delay_s": mean_or_nan(passenger_delays),
        "bus_stop_count": sum_or_zero(stop_counts["bus_bus"]),
        "tram_stop_count": sum_or_zero(stop_counts["tram_tram"]),
        "cv_stop_count": sum_or_zero(stop_counts["veh_CV"]),
        "nv_stop_count": sum_or_zero(stop_counts["veh_car"]),
        "max_vehicle_count": float(max_running),
        "max_queueing_count": float(max_halting),
        "max_spillover_count": float(max_waiting),
        "max_unserved_count": float(max_unserved),
    }


def run_case(
    case: SimulationCase,
    project_root: Path,
    output_dir: Path,
    overwrite: bool = False,
) -> Dict[str, object]:
    case.validate()
    project_root = project_root.resolve()
    scenario_dir = project_root / "scenario" / "amsterdam"
    data_dir = project_root / "data"
    output_dir = output_dir.resolve()

    if output_dir.exists() and any(output_dir.iterdir()) and not overwrite:
        metrics_file = output_dir / "metrics.json"
        if metrics_file.exists():
            raise FileExistsError(
                f"{output_dir} already contains results. Use --overwrite or choose another output directory."
            )
        raise FileExistsError(f"{output_dir} is not empty. Use --overwrite or choose another directory.")
    if output_dir.exists() and overwrite:
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    static = load_static_data(data_dir)
    runtime_config = prepare_runtime_inputs(scenario_dir, output_dir, case.penetration)

    check_binary, traci = import_sumo_tools()
    sumo_binary = check_binary("sumo-gui" if case.gui else "sumo")

    tripinfo_xml = output_dir / "tripinfo.xml"
    summary_xml = output_dir / "summary.xml"
    fcd_xml = output_dir / "fcd.xml"

    cmd = [
        sumo_binary,
        "-c", str(runtime_config),
        "--start",
        "--quit-on-end",
        "--tripinfo-output", str(tripinfo_xml),
        "--summary-output", str(summary_xml),
        "--seed", str(case.seed),
    ]
    if case.save_fcd:
        cmd.extend(["--fcd-output.attributes", "all", "--fcd-output", str(fcd_xml)])

    metadata = {
        "case": asdict(case),
        "paper_method": METHODS[case.method]["paper_name"],
        "sumo_command": cmd,
    }
    (output_dir / "run_config.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    tracker = TravelTimeTracker()
    last_phase = [-1, -1, -1]
    last_plan: List[object] = [-1, -1, -1]
    plan_t = 0
    yellow_index = [0, 0, 0]
    sumo_plan: List[str] = []
    sumo_phase: List[int] = []
    signal_state: List[List[object]] = []
    fallback_decisions = 0
    time_s = 0

    traci.start(cmd)
    try:
        while traci.simulation.getMinExpectedNumber() > 0 and time_s < case.max_time:
            traci.simulationStep()

            vehicle_ids = traci.vehicle.getIDList()
            cv_ids = [
                veh for veh in vehicle_ids
                if traci.vehicle.getTypeID(veh) in {"bus_bus", "tram_tram", "veh_CV"}
            ]
            vehicle_info = extract_cv_information(traci, cv_ids, static, tracker, time_s, case)

            if time_s % case.decision_interval == 0:
                weights = initialize_weights()
                for vehinfo in vehicle_info:
                    update_incoming(vehinfo, weights)
                    update_outgoing(vehinfo, static, weights)

                if METHODS[case.method]["fallback"] and count_zero_cv_groups(weights) > 0:
                    estimates = estimate_fallback_states(
                        traci, static, time_s, case.penetration, case.history_window_seconds
                    )
                    apply_fallback(weights, estimates)
                    fallback_decisions += 1

                for node in NODE_NAMES:
                    stream_pressure(weights[node])
                phase_pressures = compute_phase_pressures(static.phases, weights)
                sumo_plan, sumo_phase = choose_signal_plan(static.phases, phase_pressures, last_phase, case)
                plan_t = 0

            if plan_t == 0:
                yellow_index = []
                for i in range(3):
                    changed = last_phase[i] != -1 and sumo_phase[i] != last_phase[i]
                    yellow_index.append(1 if changed else 0)

            for i, tls_id in enumerate(TLS_IDS):
                if plan_t < case.yellow_time and yellow_index[i] == 1:
                    yellow_plan = str(last_plan[i]).replace("G", "y").replace("g", "y")
                    traci.trafficlight.setRedYellowGreenState(tls_id, yellow_plan)
                else:
                    traci.trafficlight.setRedYellowGreenState(tls_id, sumo_plan[i])

            if plan_t == case.decision_interval - 1:
                last_phase = copy.deepcopy(sumo_phase)
                last_plan = copy.deepcopy(sumo_plan)

            signal_state.append(
                [
                    time_s,
                    traci.trafficlight.getRedYellowGreenState("TL_ZZ"),
                    traci.trafficlight.getRedYellowGreenState("TL_IM"),
                    traci.trafficlight.getRedYellowGreenState("TL_WL"),
                ]
            )
            plan_t += 1
            time_s += 1
    finally:
        traci.close()

    pd.DataFrame(signal_state, columns=["time_s", "TL_ZZ", "TL_IM", "TL_WL"]).to_csv(
        output_dir / "signal_states.csv", index=False
    )

    metrics: Dict[str, object] = {
        "method": case.method,
        "paper_method": METHODS[case.method]["paper_name"],
        "penetration": case.penetration,
        "seed": case.seed,
        "simulated_steps": time_s,
        "mtransit_fallback_decisions": fallback_decisions,
    }
    metrics.update(parse_metrics(tripinfo_xml, summary_xml))
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=True), encoding="utf-8")
    return metrics
