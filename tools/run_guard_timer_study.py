#!/usr/bin/env python3
"""
Guard Timer before/after study (enhancement plan section 4.2.3).

Runs paired scenarios (identical mobility/seed/config, Guard Timer off vs.
on) across enough seeds and mobility patterns to gather a meaningful sample
of genuine Intra-Mesh HO events, since a deliberately-designed boundary-
crossing route was tried and found to mostly fail (the actual -58dBm
coverage of two APs 200m apart doesn't overlap enough for a clean handover
in most cases -- see conversation/report for that finding). This instead
uses the existing organic mobility patterns (gaussmarkov/patrol/transport/
work), which already produce genuine Intra-Mesh HO events at a real, if
modest, rate.

Writes one output directory per (mobility, seed, guard_state) combination,
plus a summary CSV with switch-event counts, so generate_guard_timer_report.py
can compute the false-trigger suppression rate.

Usage (from ns-3.45/):
    python3 tools/run_guard_timer_study.py --dry-run
    python3 tools/run_guard_timer_study.py --parallel 5
"""
import argparse
import csv
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

MOBILITY_TYPES = [
    {"label": "gaussmarkov", "mobilityModel": "gaussmarkov", "robotType": None},
    {"label": "patrol", "mobilityModel": "waypoint", "robotType": "patrol"},
    {"label": "transport", "mobilityModel": "waypoint", "robotType": "transport"},
    {"label": "work", "mobilityModel": "waypoint", "robotType": "work"},
]

DEFAULT_SEEDS = list(range(101, 111))  # 10 seeds, distinct from the project's 7/8/9 convention
RESULTS_ROOT = "Traffic_qos_outputs/guard_timer_study"
SIM_TIME = 90
NUM_STA = 10
CELLULAR_MODE = "lte"
HOTSPOT_BAND = "5g"
PAYLOAD_BYTES = 1048576  # 1MB
FLOW_SCALE = 1.0
RSSI_THRESHOLD = -58
GUARD_TIMER_S = 0.5

SUMMARY_FIELDS = [
    "run_id", "mobility", "seed", "guard_enabled", "status", "elapsed_sec",
    "switch_events", "intra_mesh", "wifi_to_cell", "cell_to_wifi",
    "cellular_mode", "num_sta", "hotspot_band", "condition_tag",
]


def build_command(mobility, seed, guard_enabled, out_dir, cellular_mode, num_sta, hotspot_band):
    parts = [
        "traffic-qos",
        f"--cellularMode={cellular_mode}",
        f"--hotspotBand={hotspot_band}",
        f"--numStaNodes={num_sta}",
        f"--rngSeed={seed}",
        f"--simTime={SIM_TIME}",
        f"--flowScale={FLOW_SCALE}",
        f"--uploadBytes={PAYLOAD_BYTES}",
        f"--downloadBytes={PAYLOAD_BYTES}",
        "--enableSwitching=true",
        f"--rssiThresholdDbm={RSSI_THRESHOLD}",
        f"--guardTimerEnabled={'true' if guard_enabled else 'false'}",
        f"--guardTimerS={GUARD_TIMER_S}",
        f"--mobilityModel={mobility['mobilityModel']}",
    ]
    if mobility["robotType"]:
        parts.append(f"--robotType={mobility['robotType']}")
    parts.append(f"--outputDir={out_dir}")
    run_str = " ".join(parts)
    return f'./ns3 run --no-build "{run_str}"'


def count_events(switch_log: Path):
    counts = {"switch_events": 0, "intra_mesh": 0, "wifi_to_cell": 0, "cell_to_wifi": 0}
    if not switch_log.exists():
        return counts
    with switch_log.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            counts["switch_events"] += 1
            t = row.get("type", "")
            if t in counts:
                counts[t] += 1
    return counts


def run_one(mobility, seed, guard_enabled, cellular_mode, num_sta, hotspot_band, condition_tag, results_root):
    label = mobility["label"]
    guard_tag = "guard_on" if guard_enabled else "guard_off"
    run_id = f"{condition_tag}_{label}_seed{seed}_{guard_tag}" if condition_tag else f"{label}_seed{seed}_{guard_tag}"
    out_dir = f"{results_root}/{run_id}"
    cmd = build_command(mobility, seed, guard_enabled, out_dir, cellular_mode, num_sta, hotspot_band)

    t0 = time.time()
    proc = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    elapsed = time.time() - t0

    Path(out_dir).mkdir(parents=True, exist_ok=True)
    (Path(out_dir) / "run.log").write_text(
        f"command: {cmd}\nexit_code: {proc.returncode}\nelapsed_sec: {elapsed:.3f}\n\n"
        f"--- stdout ---\n{proc.stdout[-100000:]}\n--- stderr ---\n{proc.stderr[-20000:]}\n",
        encoding="utf-8",
    )

    row = {
        "run_id": run_id, "mobility": label, "seed": seed,
        "guard_enabled": guard_enabled, "elapsed_sec": f"{elapsed:.1f}",
        "cellular_mode": cellular_mode, "num_sta": num_sta,
        "hotspot_band": hotspot_band, "condition_tag": condition_tag,
    }
    if proc.returncode != 0:
        row["status"] = "failed"
        row.update({"switch_events": 0, "intra_mesh": 0, "wifi_to_cell": 0, "cell_to_wifi": 0})
    else:
        row["status"] = "ok"
        row.update(count_events(Path(out_dir) / "wifi-hybrid-switch_log.csv"))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seeds", default=",".join(str(s) for s in DEFAULT_SEEDS))
    parser.add_argument("--only", help="Comma-separated mobility labels (gaussmarkov,patrol,transport,work)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--parallel", type=int, default=1)
    parser.add_argument("--summary-csv", default=f"{RESULTS_ROOT}/summary.csv")
    parser.add_argument("--results-root", default=RESULTS_ROOT)
    parser.add_argument("--cellular-mode", default=CELLULAR_MODE)
    parser.add_argument("--num-sta", type=int, default=NUM_STA)
    parser.add_argument("--hotspot-band", default=HOTSPOT_BAND)
    parser.add_argument("--condition-tag", default="",
                         help="Prefix for run_id/output dirs, so different conditions (e.g. nr, sta20) don't collide")
    args = parser.parse_args()

    seeds = [int(s) for s in args.seeds.split(",")]
    only = set(args.only.split(",")) if args.only else None
    mobility_types = [m for m in MOBILITY_TYPES if not only or m["label"] in only]

    jobs = []
    for mobility in mobility_types:
        for seed in seeds:
            for guard_enabled in (False, True):
                jobs.append((mobility, seed, guard_enabled))

    if args.dry_run:
        for i, (mobility, seed, guard_enabled) in enumerate(jobs, 1):
            tag = f"{args.condition_tag}_" if args.condition_tag else ""
            out_dir = f"{args.results_root}/{tag}{mobility['label']}_seed{seed}_{'guard_on' if guard_enabled else 'guard_off'}"
            cmd = build_command(mobility, seed, guard_enabled, out_dir, args.cellular_mode, args.num_sta, args.hotspot_band)
            print(f"[{i}] {cmd}")
        print(f"\nTotal jobs: {len(jobs)}")
        return

    print(f"Running {len(jobs)} jobs with parallelism={args.parallel}...")
    summary_path = Path(args.summary_csv)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", newline="", encoding="utf-8") as sf:
        writer = csv.DictWriter(sf, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        done = 0
        t_start = time.time()
        with ThreadPoolExecutor(max_workers=args.parallel) as executor:
            futures = {
                executor.submit(run_one, m, s, g, args.cellular_mode, args.num_sta, args.hotspot_band,
                                 args.condition_tag, args.results_root): (m, s, g)
                for m, s, g in jobs
            }
            for future in as_completed(futures):
                row = future.result()
                writer.writerow(row)
                sf.flush()
                done += 1
                elapsed_total = time.time() - t_start
                rate = done / elapsed_total if elapsed_total > 0 else 0
                eta_min = (len(jobs) - done) / rate / 60 if rate > 0 else float("inf")
                print(f"[{done}/{len(jobs)}] {row['run_id']}: {row['status']} "
                      f"(switches={row['switch_events']} intra_mesh={row['intra_mesh']}) "
                      f"| elapsed={elapsed_total/60:.1f}min eta={eta_min:.1f}min", flush=True)

    print(f"\nDone. Summary: {summary_path}")


if __name__ == "__main__":
    main()
