#!/usr/bin/env python3
"""
3D trajectory plot (Phase 2 Item 1 deliverable: trajectory_3d.png).
Reads the pos_x/pos_y/pos_z columns that wifi_hybrid_try_2.cc appends to
wifi-hybrid-rssi_log.csv and plots one line per STA.

Coloring:
  - One run given (typical single-scenario usage): each STA gets its own
    color (tab20), with a start marker (o) and end marker (X) so direction
    of travel is visible on a static image.
  - Multiple runs given (cross-robot-type comparison): each run/label gets
    its own color instead, so robot types are distinguishable at a glance.

Building footprints and mesh AP positions are drawn at ground level for
spatial context, since STA height barely varies (~0-30m) compared to the
400x400m field, and a bare 3D plot with no landmarks is hard to read.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/plot_trajectory_3d.py \\
        Waypoint_outputs/patrol/spd2.0/seed6 -o trajectory_3d.png

    python3 examples/my-scenarios/plot_trajectory_3d.py \\
        Waypoint_outputs/patrol/spd2.0/seed6:patrol \\
        Waypoint_outputs/transport/spd2.0/seed6:transport \\
        Waypoint_outputs/work/spd2.0/seed6:work \\
        -o trajectory_3d_comparison.png
"""
import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

FIELD_SIZE = 400.0
MESH_APS = [(100.0, 100.0), (300.0, 100.0), (300.0, 300.0), (100.0, 300.0)]
BUILDINGS = [
    (0.0, 60.0, 96.0, 104.0),
    (340.0, 400.0, 96.0, 104.0),
    (0.0, 60.0, 296.0, 304.0),
    (340.0, 400.0, 296.0, 304.0),
    (80.0, 140.0, 320.0, 328.0),
    (170.0, 250.0, 300.0, 308.0),
    (255.0, 335.0, 20.0, 28.0),
]


def read_positions(csv_path):
    positions = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["time_s"])
                sta = int(row["sta_index"])
                x = float(row["pos_x"])
                y = float(row["pos_y"])
                z = float(row["pos_z"])
            except (ValueError, KeyError):
                continue
            positions.setdefault(sta, []).append((t, x, y, z))
    for sta in positions:
        positions[sta].sort(key=lambda r: r[0])
    return positions


def draw_site_context(axis):
    """Ground-level building footprints and mesh AP markers, so the field
    layout is recognizable instead of a bare cube of numbers."""
    for x0, x1, y0, y1 in BUILDINGS:
        face = [(x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0)]
        axis.add_collection3d(
            Poly3DCollection([face], facecolor="lightgray", edgecolor="dimgray", alpha=0.6)
        )
    for ax_x, ax_y in MESH_APS:
        axis.scatter([ax_x], [ax_y], [1.5], color="black", marker="^", s=80,
                     depthshade=False, zorder=10)


def plot_by_sta(axis, positions, label):
    colors = plt.get_cmap("tab20")
    sta_indices = sorted(positions.keys())
    for i, sta in enumerate(sta_indices):
        samples = positions[sta]
        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        zs = [s[3] for s in samples]
        color = colors(i % 20)
        axis.plot(xs, ys, zs, color=color, alpha=0.9, linewidth=1.6,
                  label=f"{label} STA {sta}")
        axis.scatter([xs[0]], [ys[0]], [zs[0]], color=color, marker="o", s=50,
                     edgecolor="black", linewidth=0.5, zorder=8)
        axis.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=color, marker="X", s=60,
                     edgecolor="black", linewidth=0.5, zorder=8)


def plot_by_run(axis, positions, label, color):
    first = True
    for sta, samples in positions.items():
        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        zs = [s[3] for s in samples]
        axis.plot(xs, ys, zs, color=color, alpha=0.85, linewidth=1.3,
                  label=label if first else None)
        axis.scatter([xs[0]], [ys[0]], [zs[0]], color=color, marker="o", s=35,
                     edgecolor="black", linewidth=0.4, zorder=8)
        axis.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=color, marker="X", s=45,
                     edgecolor="black", linewidth=0.4, zorder=8)
        first = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+",
                         help="run_dir or run_dir:label, e.g. Waypoint_outputs/patrol/spd2.0/seed6:patrol")
    parser.add_argument("-o", "--output", default="trajectory_3d.png")
    parser.add_argument("--elev", type=float, default=50.0, help="3D view elevation angle")
    parser.add_argument("--azim", type=float, default=-60.0, help="3D view azimuth angle")
    args = parser.parse_args()

    fig = plt.figure(figsize=(11, 9))
    axis = fig.add_subplot(111, projection="3d")
    run_colors = plt.get_cmap("tab10")

    single_run = len(args.runs) == 1
    plotted_any = False

    for idx, spec in enumerate(args.runs):
        run_dir, _, label = spec.partition(":")
        label = label or Path(run_dir).name
        csv_path = Path(run_dir) / "wifi-hybrid-rssi_log.csv"
        if not csv_path.exists():
            print(f"skip (missing): {csv_path}", file=sys.stderr)
            continue
        positions = read_positions(csv_path)
        if not positions:
            print(f"skip (no position data): {csv_path}", file=sys.stderr)
            continue

        if single_run:
            plot_by_sta(axis, positions, label)
        else:
            plot_by_run(axis, positions, label, run_colors(idx % 10))
        plotted_any = True

    if not plotted_any:
        print("No trajectory data found in any input run.", file=sys.stderr)
        sys.exit(1)

    draw_site_context(axis)

    axis.set_xlim(0, FIELD_SIZE)
    axis.set_ylim(0, FIELD_SIZE)
    axis.set_zlim(0, 30)
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_zlabel("Z (m)")
    axis.view_init(elev=args.elev, azim=args.azim)
    title = "STA Trajectories" if single_run else "STA Trajectories by Robot Type"
    axis.set_title(f"{title}  (▲ = mesh AP, ○ = start, ✕ = end, gray = buildings)")
    axis.legend(loc="upper left", fontsize=8, ncol=2 if len(axis.get_legend_handles_labels()[0]) > 8 else 1)
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
