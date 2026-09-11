#!/usr/bin/env python3
"""
Speed-sweep runner for the Waypoint + Dwell-Time mobility scenarios
(Phase 2 Item 1). Reads the scenario JSON configs in
examples/my-scenarios/waypoint_scenarios/ and runs wifi-hybrid-try-2 for
every (robotType x speed x seed) combination, plus the Gauss-Markov baseline
at the same speeds/seeds for a like-for-like comparison.

After each run, this also generates the trajectory_viewer.html / rssi_heatmap.png /
switching_timeline.gif / animation.gif visuals into that run's output
directory (pass --skip-visuals to turn that off, e.g. for a quick data-only
sweep before deciding which runs are worth visualizing).

Run from ns-3.45/:
    python3 examples/my-scenarios/run_waypoint_scenarios.py
    python3 examples/my-scenarios/run_waypoint_scenarios.py --only patrol,work
    python3 examples/my-scenarios/run_waypoint_scenarios.py --dry-run
    python3 examples/my-scenarios/run_waypoint_scenarios.py --skip-visuals
    python3 examples/my-scenarios/run_waypoint_scenarios.py --parallel 5
"""
import argparse
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

SCENARIO_DIR = Path(__file__).parent / "waypoint_scenarios"
SCRIPT_DIR = Path(__file__).parent


def load_scenarios():
    scenarios = []
    for path in sorted(SCENARIO_DIR.glob("*.json")):
        with open(path) as f:
            scenarios.append(json.load(f))
    return scenarios


def build_command(scenario, speed, seed, output_dir):
    parts = [
        "./ns3", "run",
        '"wifi-hybrid-try-2'
        f' --mobilityModel={scenario["mobilityModel"]}'
        f' --robotType={scenario["robotType"] if scenario["robotType"] != "n/a" else "patrol"}'
        f' --staSpeedMin={speed} --staSpeedMax={speed}'
        f' --rngSeed={seed}'
        f' --numStaNodes={scenario["numStaNodes"]}'
        f' --simTime={scenario["simTime"]}'
        f' --cellularMode={scenario["cellularMode"]}'
        f' --hotspotBand={scenario["hotspotBand"]}'
        f' --meshConfig={scenario["meshConfig"]}'
        f' --rssiThresholdDbm=-58'
        f' --outputDir={output_dir}"',
    ]
    return " ".join(parts)


def generate_visuals(scenario, output_dir):
    label = scenario["robotType"] if scenario["robotType"] != "n/a" else "gaussmarkov"
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


def run_one(job_id, scenario, speed, seed, skip_visuals):
    output_dir = f"{scenario['outputRoot']}/spd{speed}/seed{seed}"
    cmd = build_command(scenario, speed, seed, output_dir)
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        return job_id, output_dir, False, result.stderr.strip().splitlines()[-1:] or ["unknown error"]
    viz_errors = [] if skip_visuals else generate_visuals(scenario, output_dir)
    return job_id, output_dir, True, viz_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", help="Comma-separated robotType filter (patrol,transport,work,gaussmarkov_baseline)")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without running them")
    parser.add_argument("--skip-visuals", action="store_true",
                         help="Don't generate trajectory/heatmap/timeline/animation files after each run")
    parser.add_argument("--parallel", type=int, default=1,
                         help="Number of simulations to run concurrently (default 1 = sequential)")
    args = parser.parse_args()

    only = set(args.only.split(",")) if args.only else None

    scenarios = load_scenarios()
    if not scenarios:
        print(f"No scenario JSON files found in {SCENARIO_DIR}", file=sys.stderr)
        sys.exit(1)

    jobs = []
    for scenario in scenarios:
        name = Path(scenario["outputRoot"]).name
        if only and name not in only and scenario["robotType"] not in only:
            continue
        for speed in scenario["speedsMps"]:
            for seed in scenario["seeds"]:
                jobs.append((scenario, speed, seed))

    if not jobs:
        print("No scenarios matched --only filter.", file=sys.stderr)
        sys.exit(1)

    if args.dry_run:
        for i, (scenario, speed, seed) in enumerate(jobs, 1):
            output_dir = f"{scenario['outputRoot']}/spd{speed}/seed{seed}"
            cmd = build_command(scenario, speed, seed, output_dir)
            print(f"[{i}] {scenario['robotType']} speed={speed} seed={seed} -> {output_dir}\n      {cmd}")
        print(f"\nTotal runs: {len(jobs)}")
        return

    print(f"Running {len(jobs)} simulations with parallelism={args.parallel}...")
    done = 0
    failed = 0
    with ThreadPoolExecutor(max_workers=args.parallel) as executor:
        futures = {
            executor.submit(run_one, i, scenario, speed, seed, args.skip_visuals): i
            for i, (scenario, speed, seed) in enumerate(jobs, 1)
        }
        for future in as_completed(futures):
            job_id, output_dir, ok, errors = future.result()
            done += 1
            if ok:
                status = "OK"
                if errors:
                    status += f" (visuals: {'; '.join(errors)})"
            else:
                status = f"FAILED ({'; '.join(errors)})"
                failed += 1
            print(f"[{done}/{len(jobs)}] {output_dir}: {status}")

    print(f"\nTotal runs: {len(jobs)}, failed: {failed}")


if __name__ == "__main__":
    main()
