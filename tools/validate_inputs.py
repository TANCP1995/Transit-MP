#!/usr/bin/env python3
"""Validate the static Amsterdam scenario and historical fallback data."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pandas as pd


def count_flows(path: Path) -> int:
    return len(ET.parse(path).getroot().findall("flow"))


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    scenario = root / "scenario" / "amsterdam"
    data = root / "data"

    required = [
        scenario / "1_1_osm.net.xml",
        scenario / "1_osm.bus.trips.xml",
        scenario / "1_1_osm.tram.trips.xml",
        scenario / "1_osm.car.trips.xml",
        scenario / "1_osm_stops.add.xml",
        scenario / "1_osm.sumocfg",
        data / "For_correcting_LTT.txt",
        data / "Edge_length_speed.txt",
        data / "In_out_pairs.txt",
        data / "Signal_phase.txt",
        data / "link_volumes_15min_darkFree.csv",
    ]
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        print("Missing required files:")
        for p in missing:
            print("  ", p)
        sys.exit(1)

    stop_count = len(ET.parse(scenario / "1_osm_stops.add.xml").getroot().findall("busStop"))
    bus_flows = count_flows(scenario / "1_osm.bus.trips.xml")
    tram_flows = count_flows(scenario / "1_1_osm.tram.trips.xml")

    volumes = pd.read_csv(data / "link_volumes_15min_darkFree.csv")
    per_slice = volumes.groupby("sliceIndex").size()

    assert stop_count == 31, f"Expected 31 transit stops, found {stop_count}"
    assert bus_flows == 8, f"Expected 8 bus flows, found {bus_flows}"
    assert tram_flows == 7, f"Expected 7 tram flows, found {tram_flows}"
    assert len(volumes) == 744, f"Expected 744 historical-volume rows, found {len(volumes)}"
    assert list(sorted(volumes.sliceIndex.unique())) == list(range(1, 13)), "Expected 12 historical 15-min slices"
    assert (per_slice == 62).all(), "Every historical slice should contain 62 controlled movements"

    print("Input validation passed.")
    print(f"  Transit stops: {stop_count}")
    print(f"  Bus lines/flows: {bus_flows}")
    print(f"  Tram lines/flows: {tram_flows}")
    print(f"  Historical flow data: {len(volumes)} rows, 12 x 15-min slices, 62 movements/slice")


if __name__ == "__main__":
    main()
