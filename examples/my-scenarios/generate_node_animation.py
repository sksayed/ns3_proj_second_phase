#!/usr/bin/env python3
"""
Node-movement animation (Phase 2 Item 1 deliverable: animation.mp4).
Animates STA positions over time from wifi-hybrid-rssi_log.csv, colored by
current path (WiFi vs cellular), for visual confirmation of movement paths
and switching events.

NOTE: the enhancement plan names this deliverable animation.mp4, produced
"with AI assistance" via the NetAnim GUI. NetAnim requires a GUI + manual
screen capture, which isn't scriptable in this environment. This script
produces the same content (node positions + path coloring over time) as a
.gif via matplotlib's Pillow writer instead, which needs no system
dependencies. If ffmpeg is installed, pass --output animation.mp4 and it
will be used automatically.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/generate_node_animation.py \\
        Waypoint_outputs/patrol/spd2.0/seed6 -o animation.gif
"""
import argparse
import csv
import shutil
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

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


def sample_at(track, t):
    best = track[0]
    for row in track:
        if row[0] <= t:
            best = row
        else:
            break
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("-o", "--output", default="animation.gif")
    parser.add_argument("--dt", type=float, default=1.0, help="Sampling step in seconds")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    rssi_csv = run_dir / "wifi-hybrid-rssi_log.csv"
    if not rssi_csv.exists():
        print(f"Missing {rssi_csv}", file=sys.stderr)
        sys.exit(1)

    per_sta = load_track(rssi_csv)
    if not per_sta:
        print("No position data found.", file=sys.stderr)
        sys.exit(1)

    max_t = max(row[0] for track in per_sta.values() for row in track)
    n_frames = int(max_t // args.dt) + 1

    fig, axis = plt.subplots(figsize=(8, 8))

    def draw_static():
        axis.clear()
        for x0, x1, y0, y1 in BUILDINGS:
            axis.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                              facecolor="lightgray", edgecolor="black"))
        for ax_x, ax_y in MESH_APS:
            axis.scatter([ax_x], [ax_y], color="blue", marker="^", s=100, zorder=5)
        axis.set_xlim(0, FIELD_SIZE)
        axis.set_ylim(0, FIELD_SIZE)
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")

    def update(frame):
        draw_static()
        t = frame * args.dt
        for sta, track in per_sta.items():
            row = sample_at(track, t)
            color = "green" if row[3] == "wifi" else "orange"
            axis.scatter([row[1]], [row[2]], color=color, s=80, zorder=6)
            axis.annotate(str(sta), (row[1], row[2]), fontsize=8, xytext=(3, 3),
                          textcoords="offset points")
        axis.set_title(f"Node positions at t={t:.0f}s (green=WiFi, orange=cellular)")
        return []

    anim = FuncAnimation(fig, update, frames=n_frames, interval=200)

    use_mp4 = args.output.endswith(".mp4")
    if use_mp4 and shutil.which("ffmpeg") is None:
        print("ffmpeg not found; falling back to .gif output.", file=sys.stderr)
        args.output = str(Path(args.output).with_suffix(".gif"))
        use_mp4 = False

    writer = FFMpegWriter(fps=5) if use_mp4 else PillowWriter(fps=5)
    anim.save(args.output, writer=writer)
    print(f"Wrote {args.output} ({n_frames} frames)")


if __name__ == "__main__":
    main()
