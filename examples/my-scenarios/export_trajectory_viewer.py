#!/usr/bin/env python3
"""
Interactive HTML trajectory viewer (replaces trajectory_3d.png).
Reads pos_x/pos_y/path from wifi-hybrid-rssi_log.csv and switch events from
wifi-hybrid-switch_log.csv, downsamples them to a fixed time step, and embeds
the result into trajectory_viewer_template.html to produce a single
self-contained HTML file: a top-down map with playback controls, per-STA
color coding, fading trails, switch-event flashes, and hover tooltips.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/export_trajectory_viewer.py \\
        Waypoint_outputs/patrol/spd2.0/seed6:patrol \\
        -o Waypoint_outputs/patrol/spd2.0/seed6/trajectory_viewer.html

    python3 examples/my-scenarios/export_trajectory_viewer.py \\
        Waypoint_outputs/patrol/spd2.0/seed6:patrol \\
        Waypoint_outputs/transport/spd2.0/seed6:transport \\
        Waypoint_outputs/work/spd2.0/seed6:work \\
        -o trajectory_viewer_comparison.html
"""
import argparse
import csv
import json
import sys
from pathlib import Path

FIELD_SIZE = 400.0
MESH_APS = [[100.0, 100.0], [300.0, 100.0], [300.0, 300.0], [100.0, 300.0]]
BUILDINGS = [
    [0.0, 60.0, 96.0, 104.0],
    [340.0, 400.0, 96.0, 104.0],
    [0.0, 60.0, 296.0, 304.0],
    [340.0, 400.0, 296.0, 304.0],
    [80.0, 140.0, 320.0, 328.0],
    [170.0, 250.0, 300.0, 308.0],
    [255.0, 335.0, 20.0, 28.0],
]


def load_track(rssi_csv):
    per_sta = {}
    with open(rssi_csv) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["time_s"])
                sta = int(row["sta_index"])
                x = float(row["pos_x"])
                y = float(row["pos_y"])
            except (ValueError, KeyError):
                continue
            path = row.get("path", "wifi")
            per_sta.setdefault(sta, []).append((t, x, y, path))
    for sta in per_sta:
        per_sta[sta].sort(key=lambda r: r[0])
    return per_sta


def resample(track, dt, max_t):
    if not track:
        return []
    out = []
    i = 0
    n = len(track)
    t = 0.0
    while t <= max_t + 1e-9:
        while i + 1 < n and track[i + 1][0] <= t:
            i += 1
        row = track[i]
        out.append({"t": round(t, 2), "x": round(row[1], 2), "y": round(row[2], 2), "path": row[3]})
        t += dt
    return out


def load_switches(switch_csv):
    events = []
    if not switch_csv.exists():
        return events
    with open(switch_csv) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["trigger_time_s"])
                sta = int(row["sta_index"])
            except (ValueError, KeyError):
                continue
            events.append({"t": round(t, 2), "sta": sta, "from": row.get("from", ""), "to": row.get("to", "")})
    events.sort(key=lambda e: e["t"])
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", help="run_dir or run_dir:label")
    parser.add_argument("-o", "--output", default="trajectory_viewer.html")
    parser.add_argument("--dt", type=float, default=0.5, help="Resample step in seconds")
    args = parser.parse_args()

    template_path = Path(__file__).parent / "trajectory_viewer_template.html"
    template = template_path.read_text()

    runs_data = []
    max_t_overall = 0.0
    for spec in args.runs:
        run_dir, _, label = spec.partition(":")
        label = label or Path(run_dir).name
        rssi_csv = Path(run_dir) / "wifi-hybrid-rssi_log.csv"
        switch_csv = Path(run_dir) / "wifi-hybrid-switch_log.csv"
        if not rssi_csv.exists():
            print(f"skip (missing): {rssi_csv}", file=sys.stderr)
            continue
        per_sta = load_track(rssi_csv)
        if not per_sta:
            print(f"skip (no position data): {rssi_csv}", file=sys.stderr)
            continue
        max_t = max(row[0] for track in per_sta.values() for row in track)
        max_t_overall = max(max_t_overall, max_t)
        stas = []
        for sta in sorted(per_sta.keys()):
            samples = resample(per_sta[sta], args.dt, max_t)
            stas.append({"id": sta, "samples": samples})
        switches = load_switches(switch_csv)
        runs_data.append({"label": label, "stas": stas, "switches": switches})

    if not runs_data:
        print("No usable run data found.", file=sys.stderr)
        sys.exit(1)

    payload = {
        "fieldSize": FIELD_SIZE,
        "buildings": BUILDINGS,
        "meshAPs": MESH_APS,
        "maxTime": round(max_t_overall, 2),
        "dt": args.dt,
        "runs": runs_data,
    }

    html = template.replace("__TRAJECTORY_DATA__", json.dumps(payload))
    Path(args.output).write_text(html)
    print(f"Wrote {args.output} ({sum(len(r['stas']) for r in runs_data)} STAs, "
          f"{sum(len(r['switches']) for r in runs_data)} switch events, max_t={max_t_overall:.1f}s)")


if __name__ == "__main__":
    main()
