#!/usr/bin/env python3
"""
2D RSSI heatmap (Phase 2 Item 1 deliverable: rssi_heatmap.png).
Bins WiFi RSSI samples from wifi-hybrid-rssi_log.csv (pos_x, pos_y,
avg_rssi_dbm) onto a grid over the 400x400m site to visualize WiFi dead
zones and switching hotspots.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/plot_rssi_heatmap.py \\
        Waypoint_outputs/patrol/spd2.0/seed6 Waypoint_outputs/transport/spd2.0/seed6 \\
        -o rssi_heatmap.png
"""
import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

FIELD_SIZE = 400.0
MESH_APS = [(100.0, 100.0), (300.0, 100.0), (300.0, 300.0), (100.0, 300.0)]
BUILDINGS = [
    ("Residential", (0.0, 60.0, 96.0, 104.0)),
    ("Residential", (340.0, 400.0, 96.0, 104.0)),
    ("Residential", (0.0, 60.0, 296.0, 304.0)),
    ("Residential", (340.0, 400.0, 296.0, 304.0)),
    ("Office", (80.0, 140.0, 320.0, 328.0)),
    ("Office", (170.0, 250.0, 300.0, 308.0)),
    ("Commercial", (255.0, 335.0, 20.0, 28.0)),
]


def read_rssi_points(csv_path):
    xs, ys, rssis = [], [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            if row.get("rat") != "wifi":
                continue
            try:
                x = float(row["pos_x"])
                y = float(row["pos_y"])
                rssi = float(row["avg_rssi_dbm"])
            except (ValueError, KeyError):
                continue
            xs.append(x)
            ys.append(y)
            rssis.append(rssi)
    return xs, ys, rssis


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+")
    parser.add_argument("-o", "--output", default="rssi_heatmap.png")
    parser.add_argument("--cell-size", type=float, default=20.0, help="Grid cell size in meters")
    args = parser.parse_args()

    all_x, all_y, all_rssi = [], [], []
    for run_dir in args.run_dirs:
        csv_path = Path(run_dir) / "wifi-hybrid-rssi_log.csv"
        if not csv_path.exists():
            print(f"skip (missing): {csv_path}", file=sys.stderr)
            continue
        x, y, rssi = read_rssi_points(csv_path)
        all_x.extend(x)
        all_y.extend(y)
        all_rssi.extend(rssi)

    if not all_x:
        print("No WiFi RSSI data found in any input run.", file=sys.stderr)
        sys.exit(1)

    x = np.array(all_x)
    y = np.array(all_y)
    rssi = np.array(all_rssi)

    n_bins = int(FIELD_SIZE / args.cell_size)
    grid_sum = np.zeros((n_bins, n_bins))
    grid_count = np.zeros((n_bins, n_bins))
    xi = np.clip((x / args.cell_size).astype(int), 0, n_bins - 1)
    yi = np.clip((y / args.cell_size).astype(int), 0, n_bins - 1)
    for i, j, r in zip(xi, yi, rssi):
        grid_sum[j, i] += r
        grid_count[j, i] += 1
    with np.errstate(invalid="ignore"):
        grid_avg = np.where(grid_count > 0, grid_sum / grid_count, np.nan)

    fig, axis = plt.subplots(figsize=(9, 8))
    im = axis.imshow(grid_avg, origin="lower", extent=[0, FIELD_SIZE, 0, FIELD_SIZE],
                      cmap="RdYlGn", vmin=-95, vmax=-30)
    fig.colorbar(im, ax=axis, label="Avg WiFi RSSI (dBm)")

    for _, (x0, x1, y0, y1) in BUILDINGS:
        axis.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                          facecolor="dimgray", edgecolor="black", alpha=0.85))
    for ax_x, ax_y in MESH_APS:
        axis.scatter([ax_x], [ax_y], color="blue", marker="^", s=100, edgecolor="white", zorder=5)

    axis.set_xlim(0, FIELD_SIZE)
    axis.set_ylim(0, FIELD_SIZE)
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_title("WiFi RSSI Heatmap (dark blocks = buildings, ▲ = mesh AP)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
