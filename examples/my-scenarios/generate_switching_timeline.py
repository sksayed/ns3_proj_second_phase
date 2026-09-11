#!/usr/bin/env python3
"""
Switching-timeline GIF (Phase 2 Item 1 deliverable: switching_timeline.gif).
Cross-references wifi-hybrid-switch_log.csv (event times) against
wifi-hybrid-rssi_log.csv (positions) to plot WHERE switches happened, animated
over 20s windows, so switching-intensive zones/periods are visible at a glance.

Requires ffmpeg for true .mp4; without it (checked at runtime) this writes a
.gif via matplotlib's Pillow writer, which needs no system packages.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/generate_switching_timeline.py \\
        Waypoint_outputs/patrol/spd2.0/seed6 -o switching_timeline.gif
"""
import argparse
import bisect
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

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


def load_positions(rssi_csv):
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
            per_sta.setdefault(sta, []).append((t, x, y))
    for sta in per_sta:
        per_sta[sta].sort(key=lambda r: r[0])
    return per_sta


def nearest_position(per_sta, sta, t):
    samples = per_sta.get(sta)
    if not samples:
        return None
    times = [s[0] for s in samples]
    i = bisect.bisect_left(times, t)
    candidates = []
    if i < len(samples):
        candidates.append(samples[i])
    if i > 0:
        candidates.append(samples[i - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda s: abs(s[0] - t))
    return best[1], best[2]


def load_switch_events(switch_csv, per_sta):
    events = []
    with open(switch_csv) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["trigger_time_s"])
                sta = int(row["sta_index"])
            except (ValueError, KeyError):
                continue
            pos = nearest_position(per_sta, sta, t)
            if pos is None:
                continue
            to_wifi = "wifi" in row.get("to", "").lower()
            events.append((t, sta, pos[0], pos[1], to_wifi))
    events.sort(key=lambda e: e[0])
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("-o", "--output", default="switching_timeline.gif")
    parser.add_argument("--window", type=float, default=20.0, help="Window size in seconds")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    rssi_csv = run_dir / "wifi-hybrid-rssi_log.csv"
    switch_csv = run_dir / "wifi-hybrid-switch_log.csv"
    if not rssi_csv.exists() or not switch_csv.exists():
        print(f"Missing rssi/switch log in {run_dir}", file=sys.stderr)
        sys.exit(1)

    per_sta = load_positions(rssi_csv)
    events = load_switch_events(switch_csv, per_sta)
    if not events:
        print("No switch events with resolvable positions; nothing to animate.", file=sys.stderr)
        sys.exit(1)

    max_t = max(e[0] for e in events)
    n_windows = int(max_t // args.window) + 1

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
        win_start = frame * args.window
        win_end = win_start + args.window
        window_events = [e for e in events if win_start <= e[0] < win_end]
        # e[4] is True when the switch target path is wifi.
        cellular_targets = [(e[2], e[3]) for e in window_events if not e[4]]
        wifi_targets = [(e[2], e[3]) for e in window_events if e[4]]
        if cellular_targets:
            xs, ys = zip(*cellular_targets)
            axis.scatter(xs, ys, color="red", marker="x", s=120, label="WiFi→Cellular")
        if wifi_targets:
            xs, ys = zip(*wifi_targets)
            axis.scatter(xs, ys, color="green", marker="o", s=100, label="Cellular→WiFi")
        axis.set_title(f"Switching events, t=[{win_start:.0f}s, {win_end:.0f}s)  "
                        f"({len(window_events)} events)")
        if window_events:
            axis.legend(loc="upper right")
        return []

    anim = FuncAnimation(fig, update, frames=n_windows, interval=800)
    anim.save(args.output, writer=PillowWriter(fps=1))
    print(f"Wrote {args.output} ({n_windows} windows, {len(events)} total events)")


if __name__ == "__main__":
    main()
