#!/usr/bin/env python3
"""
Full mobility matrix runner (Phase 2 Item 1, July revision).

Extends the original patrol/transport/work/gaussmarkov speed-sweep
(run_waypoint_scenarios.py) with the same STA-count / payload / cellular-mode
axes the Phase 1 matrix (SW_unit/run_hybrid_matrix.py) used, per reviewer
feedback that the June deliverable's verification scope (5 STAs fixed, one
cellular mode, no payload sweep) wasn't comparable to Phase 1.

Axes (all crossed): 4 mobility types x 2 cellular modes x 3 STA counts x
3 payloads x 3 speeds x 3 seeds = 648 runs.

CLI flags mirror run_hybrid_matrix.py exactly (--rssiThresholdDbm=-58,
--enableSwitching=1, --requireAssocForWifiReturn=true) so results are
comparable to the existing Phase 1 matrix; rssiHysteresisDb/pdrThreshold
are left at their compiled-in defaults (3.0 / 0.9), which match Phase 1's
"h3 pdr0p90" convention.

Run from ns-3.45/:
    python3 examples/my-scenarios/run_mobility_matrix.py --dry-run
    python3 examples/my-scenarios/run_mobility_matrix.py --only patrol --dry-run
    python3 examples/my-scenarios/run_mobility_matrix.py --parallel 5
    python3 examples/my-scenarios/run_mobility_matrix.py --parallel 5 --skip-existing
"""
import argparse
import csv
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

SCENARIO_DIR = Path(__file__).parent / "waypoint_scenarios"
SCRIPT_DIR = Path(__file__).parent

PAYLOAD_TO_BYTES = {
    "10kb": 10 * 1024,
    "50kb": 50 * 1024,
    "1mb": 1 * 1024 * 1024,
    "2mb": 2 * 1024 * 1024,
}

DEFAULT_STAS = [5, 10, 15]
DEFAULT_PAYLOADS = ["10kb", "50kb", "1mb"]
DEFAULT_CELLULAR_MODES = ["lte", "nr"]
DEFAULT_SPEEDS = [0.5, 2.0, 5.0]
DEFAULT_SEEDS = [7, 8, 9]
RSSI_THRESHOLD_DBM = -58
SIM_TIME = 90


def load_mobility_types():
    """Read robotType/mobilityModel/outputRoot/simTime from the existing
    per-type JSON configs so this script stays in sync with them (they still
    document the movement pattern each type models); the sweep axes below
    (STA count/payload/cellular mode/speed/seed) are supplied by this
    script's own CLI, overriding the single-value fields in those JSONs.
    """
    types = []
    for path in sorted(SCENARIO_DIR.glob("*.json")):
        with open(path) as f:
            cfg = json.load(f)
        types.append(cfg)
    return types


def build_command(mtype, cellular, sta, payload, speed, seed, output_dir):
    robot_type = mtype["robotType"] if mtype["robotType"] != "n/a" else "patrol"
    payload_bytes = PAYLOAD_TO_BYTES[payload]
    run_str = (
        "wifi-hybrid-try-2 "
        f"--mobilityModel={mtype['mobilityModel']} "
        f"--robotType={robot_type} "
        f"--cellularMode={cellular} "
        f"--hotspotBand={mtype.get('hotspotBand', '5g')} "
        f"--meshConfig={mtype.get('meshConfig', 1)} "
        "--enableSwitching=1 "
        "--requireAssocForWifiReturn=true "
        f"--numStaNodes={sta} "
        f"--simTime={SIM_TIME} "
        f"--rssiThresholdDbm={RSSI_THRESHOLD_DBM} "
        f"--rngSeed={seed} "
        f"--uploadBytes={payload_bytes} "
        f"--downloadBytes={payload_bytes} "
        f"--staSpeedMin={speed} --staSpeedMax={speed} "
        f"--outputDir={output_dir}"
    )
    return f'./ns3 run --no-build "{run_str}"'


def generate_visuals(mtype, output_dir):
    label = mtype["robotType"] if mtype["robotType"] != "n/a" else "gaussmarkov"
    jobs = [
        [sys.executable, str(SCRIPT_DIR / "export_trajectory_viewer.py"),
         f"{output_dir}:{label}", "-o", f"{output_dir}/trajectory_viewer.html"],
        [sys.executable, str(SCRIPT_DIR / "plot_rssi_heatmap.py"),
         output_dir, "-o", f"{output_dir}/rssi_heatmap.png"],
        [sys.executable, str(SCRIPT_DIR / "generate_switching_timeline.py"),
         output_dir, "-o", f"{output_dir}/switching_timeline.gif"],
        [sys.executable, str(SCRIPT_DIR / "generate_node_animation.py"),
         output_dir, "-o", f"{output_dir}/animation.gif", "--dt", "2.0"],
        [sys.executable, str(SCRIPT_DIR / "plot_trajectory_3d.py"),
         output_dir, "-o", f"{output_dir}/trajectory_3d.png"],
    ]
    errors = []
    for cmd in jobs:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            last_line = result.stderr.strip().splitlines()[-1] if result.stderr else "unknown error"
            errors.append(f"{Path(cmd[1]).stem}: {last_line}")
    return errors


def count_switch_events(switch_log_path: Path):
    if not switch_log_path.exists():
        return 0
    with switch_log_path.open(newline="") as f:
        return sum(1 for _ in csv.DictReader(f))


def build_jobs(mobility_types, only, stas, payloads, cellular_modes, speeds, seeds):
    jobs = []
    for mtype in mobility_types:
        name = Path(mtype["outputRoot"]).name
        if only and name not in only and mtype["robotType"] not in only:
            continue
        for cellular in cellular_modes:
            for sta in stas:
                for payload in payloads:
                    for speed in speeds:
                        for seed in seeds:
                            out_dir = (
                                f"{mtype['outputRoot']}/"
                                f"{cellular}_sta{sta}_{payload}_spd{speed}_seed{seed}"
                            )
                            jobs.append((mtype, cellular, sta, payload, speed, seed, out_dir))
    return jobs


def run_one(job_id, mtype, cellular, sta, payload, speed, seed, out_dir, skip_visuals, skip_existing):
    rssi_log = Path(out_dir) / "wifi-hybrid-rssi_log.csv"
    switch_log = Path(out_dir) / "wifi-hybrid-switch_log.csv"
    if skip_existing and rssi_log.exists() and switch_log.exists():
        return job_id, out_dir, True, 0.0, ["skipped-existing"]

    cmd = build_command(mtype, cellular, sta, payload, speed, seed, out_dir)
    t0 = time.time()
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    elapsed = time.time() - t0
    if result.returncode != 0:
        return job_id, out_dir, False, elapsed, [result.stderr.strip().splitlines()[-1] if result.stderr else "unknown error"]

    viz_errors = [] if skip_visuals else generate_visuals(mtype, out_dir)
    return job_id, out_dir, True, elapsed, viz_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--only", help="Comma-separated robotType filter (patrol,transport,work,gaussmarkov_baseline)")
    parser.add_argument("--stas", default=",".join(str(s) for s in DEFAULT_STAS))
    parser.add_argument("--payloads", default=",".join(DEFAULT_PAYLOADS))
    parser.add_argument("--cellular-modes", default=",".join(DEFAULT_CELLULAR_MODES))
    parser.add_argument("--speeds", default=",".join(str(s) for s in DEFAULT_SPEEDS))
    parser.add_argument("--seeds", default=",".join(str(s) for s in DEFAULT_SEEDS))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-visuals", action="store_true")
    parser.add_argument("--skip-existing", action="store_true",
                         help="Skip jobs whose output dir already has rssi/switch logs (resume support)")
    parser.add_argument("--parallel", type=int, default=1)
    parser.add_argument("--summary-csv", default="Waypoint_outputs/mobility_matrix_summary.csv")
    args = parser.parse_args()

    only = set(args.only.split(",")) if args.only else None
    stas = [int(s) for s in args.stas.split(",")]
    payloads = args.payloads.split(",")
    cellular_modes = args.cellular_modes.split(",")
    speeds = [float(s) for s in args.speeds.split(",")]
    seeds = [int(s) for s in args.seeds.split(",")]

    mobility_types = load_mobility_types()
    jobs = build_jobs(mobility_types, only, stas, payloads, cellular_modes, speeds, seeds)

    if not jobs:
        print("No jobs matched.", file=sys.stderr)
        sys.exit(1)

    if args.dry_run:
        for i, (mtype, cellular, sta, payload, speed, seed, out_dir) in enumerate(jobs, 1):
            cmd = build_command(mtype, cellular, sta, payload, speed, seed, out_dir)
            print(f"[{i}] {mtype['robotType']} {cellular} sta{sta} {payload} spd{speed} seed{seed} -> {out_dir}")
            print(f"      {cmd}")
        print(f"\nTotal jobs: {len(jobs)}")
        return

    print(f"Running {len(jobs)} simulations with parallelism={args.parallel}...")
    summary_path = Path(args.summary_csv)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["scenario", "robot_type", "mobility_model", "cellular_mode", "sta", "payload",
              "speed", "seed", "status", "elapsed_sec", "switch_events", "errors"]
    with summary_path.open("w", newline="", encoding="utf-8") as sf:
        writer = csv.DictWriter(sf, fieldnames=fields)
        writer.writeheader()

        done = 0
        failed = 0
        t_start = time.time()
        with ThreadPoolExecutor(max_workers=args.parallel) as executor:
            futures = {
                executor.submit(run_one, i, mtype, cellular, sta, payload, speed, seed, out_dir,
                                 args.skip_visuals, args.skip_existing): (i, mtype, cellular, sta, payload, speed, seed, out_dir)
                for i, (mtype, cellular, sta, payload, speed, seed, out_dir) in enumerate(jobs, 1)
            }
            for future in as_completed(futures):
                job_id, mtype, cellular, sta, payload, speed, seed, out_dir = futures[future]
                _, out_dir_ret, ok, elapsed, errors = future.result()
                done += 1
                status = "ok" if ok else "failed"
                if not ok:
                    failed += 1
                switch_events = count_switch_events(Path(out_dir_ret) / "wifi-hybrid-switch_log.csv") if ok else 0
                writer.writerow({
                    "scenario": out_dir_ret,
                    "robot_type": mtype["robotType"],
                    "mobility_model": mtype["mobilityModel"],
                    "cellular_mode": cellular,
                    "sta": sta,
                    "payload": payload,
                    "speed": speed,
                    "seed": seed,
                    "status": status,
                    "elapsed_sec": f"{elapsed:.1f}",
                    "switch_events": switch_events,
                    "errors": "; ".join(errors) if errors else "",
                })
                sf.flush()
                elapsed_total = time.time() - t_start
                rate = done / elapsed_total if elapsed_total > 0 else 0
                eta_sec = (len(jobs) - done) / rate if rate > 0 else float("inf")
                print(f"[{done}/{len(jobs)}] {out_dir_ret}: {status}"
                      f"{' (' + '; '.join(errors) + ')' if errors else ''}"
                      f" | elapsed={elapsed_total/60:.1f}min eta={eta_sec/60:.1f}min", flush=True)

    print(f"\nTotal runs: {len(jobs)}, failed: {failed}")
    print(f"Summary CSV: {summary_path}")


if __name__ == "__main__":
    main()
