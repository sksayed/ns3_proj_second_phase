#!/usr/bin/env python3
"""
Run the July traffic-qos 192-scenario factorial campaign.

Matrix (seeds 7, 8, 9):
  cellularMode × hotspotBand × numStaNodes × payload × rngSeed
  = 2 × 2 × 4 × 4 × 3 = 192

Features:
  - Writes every run under a single campaign root
  - Continuously updates summary.csv
  - Tracks failures in failed.csv for later retry
  - --retry-failed re-runs only failed scenarios
  - --skip-existing resumes a partial campaign
  - Gathers per-flow QoS metrics into gathered_metrics.csv

Usage (from ns-3.45/):
  # Dry-run: print the 192 commands
  python3 tools/run_traffic_qos_matrix.py --dry-run

  # Full campaign
  python3 tools/run_traffic_qos_matrix.py --sim-time 60

  # Resume / skip completed
  python3 tools/run_traffic_qos_matrix.py --skip-existing

  # Re-run only failures
  python3 tools/run_traffic_qos_matrix.py --retry-failed

  # Quick test of first N scenarios
  python3 tools/run_traffic_qos_matrix.py --limit 2 --sim-time 20
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

_CSV_LOCK = threading.Lock()


# Payload tags map to flowScale applied to Sensor/Video rates in traffic_qos.cc.
# Control rate stays fixed. Baseline (1.0) = Sensor 4 Mbps + Video 5 Mbps.
PAYLOAD_TO_FLOW_SCALE = {
    "10kb": 0.10,
    "50kb": 0.25,
    "1mb": 1.00,
    "2mb": 2.00,
}

PAYLOAD_TO_BYTES = {
    "10kb": 10 * 1024,
    "50kb": 50 * 1024,
    "1mb": 1 * 1024 * 1024,
    "2mb": 2 * 1024 * 1024,
}

SUMMARY_FIELDS = [
    "run_id",
    "status",
    "exit_code",
    "elapsed_sec",
    "scenario_dir",
    "cellularMode",
    "hotspotBand",
    "numStaNodes",
    "payload",
    "flowScale",
    "rngSeed",
    "simTime",
    "switch_events",
    "resolved",
    "timeout",
    "superseded",
    "error",
]

GATHER_FIELDS = [
    "run_id",
    "cellularMode",
    "hotspotBand",
    "numStaNodes",
    "payload",
    "rngSeed",
    "flow",
    "pdr_pct",
    "loss_pct",
    "mean_delay_ms",
    "p99_ms",
    "throughput_mbps",
    "tput_share_pct",
]


@dataclass(frozen=True)
class Scenario:
    cellular_mode: str
    hotspot_band: str
    num_sta: int
    payload: str
    seed: int
    sim_time: int

    @property
    def run_id(self) -> str:
        return (
            f"{self.cellular_mode}_{self.hotspot_band}_"
            f"sta{self.num_sta}_{self.payload}_seed{self.seed}"
        )

    @property
    def flow_scale(self) -> float:
        return PAYLOAD_TO_FLOW_SCALE[self.payload]

    @property
    def payload_bytes(self) -> int:
        return PAYLOAD_TO_BYTES[self.payload]

    def relative_dir(self, results_root: str, campaign_name: str) -> str:
        return f"{results_root}/{campaign_name}/{self.run_id}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Run traffic-qos 192-run factorial matrix with failure tracking."
    )
    p.add_argument(
        "--campaign-name",
        default="Traffic_qos_matrix_192",
        help="Campaign folder name under --results-root (default: %(default)s).",
    )
    p.add_argument(
        "--results-root",
        default="Traffic_qos_outputs",
        help="Root directory for campaign outputs (default: %(default)s).",
    )
    p.add_argument(
        "--sim-time",
        type=int,
        default=60,
        help="Simulation time in seconds (default: %(default)s).",
    )
    p.add_argument(
        "--seeds",
        default="7,8,9",
        help="Comma-separated RNG seeds (default: %(default)s).",
    )
    p.add_argument(
        "--modes",
        default="lte,nr",
        help="Comma-separated cellular modes (default: %(default)s).",
    )
    p.add_argument(
        "--bands",
        default="2g,5g",
        help="Comma-separated hotspot bands (default: %(default)s).",
    )
    p.add_argument(
        "--stas",
        default="5,10,15,20",
        help="Comma-separated STA counts (default: %(default)s).",
    )
    p.add_argument(
        "--payloads",
        default="10kb,50kb,1mb,2mb",
        help="Comma-separated payload tags (default: %(default)s).",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned commands without executing.",
    )
    p.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip scenarios that already have a successful marker.",
    )
    p.add_argument(
        "--retry-failed",
        action="store_true",
        help="Only re-run scenarios listed in failed.csv.",
    )
    p.add_argument(
        "--limit",
        type=int,
        default=0,
        help="If >0, only run the first N scenarios (for smoke tests).",
    )
    p.add_argument(
        "--no-metrics",
        action="store_true",
        help="Skip flow_metrics.py gathering after each successful run.",
    )
    p.add_argument(
        "--parallel",
        type=int,
        default=1,
        help="Number of scenarios to run concurrently (default 1 = sequential).",
    )
    return p.parse_args()


def csv_list(raw: str) -> List[str]:
    return [t.strip() for t in raw.split(",") if t.strip()]


def build_scenarios(args: argparse.Namespace) -> List[Scenario]:
    modes = csv_list(args.modes)
    bands = csv_list(args.bands)
    stas = [int(x) for x in csv_list(args.stas)]
    payloads = csv_list(args.payloads)
    seeds = [int(x) for x in csv_list(args.seeds)]

    for p in payloads:
        if p not in PAYLOAD_TO_FLOW_SCALE:
            raise SystemExit(f"Unknown payload '{p}'. Choose from {list(PAYLOAD_TO_FLOW_SCALE)}")

    scenarios: List[Scenario] = []
    for mode in modes:
        for band in bands:
            for sta in stas:
                for payload in payloads:
                    for seed in seeds:
                        scenarios.append(
                            Scenario(
                                cellular_mode=mode,
                                hotspot_band=band,
                                num_sta=sta,
                                payload=payload,
                                seed=seed,
                                sim_time=args.sim_time,
                            )
                        )
    return scenarios


def success_marker(run_dir: Path) -> Path:
    return run_dir / "RUN_OK"


def is_successful(run_dir: Path) -> bool:
    if success_marker(run_dir).exists():
        return True
    # Fallback: key artifacts present
    return (
        (run_dir / "wifi-hybrid-flowmon_data.xml").exists()
        and (run_dir / "wifi-hybrid-switch_log.csv").exists()
        and (run_dir / "config_test_2.json").exists()
    )


def count_switch_outcomes(switch_log: Path) -> Dict[str, int]:
    out = {"switch_events": 0, "resolved": 0, "timeout": 0, "superseded": 0}
    if not switch_log.exists():
        return out
    with switch_log.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            out["switch_events"] += 1
            status = (row.get("status") or "").strip().lower()
            if status in out:
                out[status] += 1
    return out


def write_csv(path: Path, fields: List[str], rows: List[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow(row)


def append_or_replace_row(path: Path, fields: List[str], row: dict, key: str = "run_id") -> None:
    """Upsert one row into a CSV by run_id. Locked internally so this is
    safe to call concurrently under --parallel."""
    with _CSV_LOCK:
        rows: List[dict] = []
        if path.exists():
            with path.open(newline="", encoding="utf-8") as fh:
                rows = list(csv.DictReader(fh))
        rows = [r for r in rows if r.get(key) != row.get(key)]
        rows.append({f: row.get(f, "") for f in fields})
        # Keep stable order by run_id
        rows.sort(key=lambda r: r.get(key, ""))
        write_csv(path, fields, rows)


def load_failed_run_ids(failed_csv: Path) -> List[str]:
    if not failed_csv.exists():
        return []
    with failed_csv.open(newline="", encoding="utf-8") as fh:
        return [r["run_id"] for r in csv.DictReader(fh) if r.get("run_id")]


def build_ns3_command(sc: Scenario, out_dir: str) -> List[str]:
    run_str = (
        "traffic-qos "
        f"--cellularMode={sc.cellular_mode} "
        f"--hotspotBand={sc.hotspot_band} "
        f"--numStaNodes={sc.num_sta} "
        f"--rngSeed={sc.seed} "
        f"--simTime={sc.sim_time} "
        f"--flowScale={sc.flow_scale} "
        f"--uploadBytes={sc.payload_bytes} "
        f"--downloadBytes={sc.payload_bytes} "
        "--enableSwitching=true "
        "--rssiThresholdDbm=-58 "
        f"--outputDir={out_dir}"
    )
    # --no-build: the binary is built once up front by the caller. Without
    # this, concurrent `./ns3 run` invocations under --parallel race on the
    # build-check step and fail intermittently (seen previously with the
    # mobility matrix runner).
    return ["./ns3", "run", "--no-build", run_str]


def gather_flow_metrics(
    ns3_root: Path,
    run_dir: Path,
    sc: Scenario,
    gather_csv: Path,
) -> None:
    xml = run_dir / "wifi-hybrid-flowmon_data.xml"
    if not xml.exists():
        return
    md = run_dir / "traffic-qos-metrics.md"
    cmd = [
        "python3",
        "examples/my-scenarios/flow_metrics.py",
        str(xml.relative_to(ns3_root)),
        "--sim-time",
        str(sc.sim_time),
        "--md",
        str(md.relative_to(ns3_root)),
    ]
    switch_log = run_dir / "wifi-hybrid-switch_log.csv"
    if switch_log.exists():
        cmd.extend(["--switch-log", str(switch_log.relative_to(ns3_root))])

    proc = subprocess.run(cmd, cwd=ns3_root, text=True, capture_output=True)
    (run_dir / "flow_metrics_run.log").write_text(
        f"exit_code={proc.returncode}\n\n--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}\n",
        encoding="utf-8",
    )
    if proc.returncode != 0:
        return

    # Parse the markdown table lines for Control/Sensor/Video
    if not md.exists():
        return
    parsed_rows: List[dict] = []
    for line in md.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| Control") and not line.startswith("| Sensor") and not line.startswith("| Video"):
            continue
        parts = [p.strip() for p in line.strip("|").split("|")]
        if len(parts) < 9:
            continue
        flow, _proto, _n, pdr, loss, mean_d, p99, tput, share = parts[:9]
        parsed_rows.append(
            {
                "run_id": sc.run_id,
                "cellularMode": sc.cellular_mode,
                "hotspotBand": sc.hotspot_band,
                "numStaNodes": sc.num_sta,
                "payload": sc.payload,
                "rngSeed": sc.seed,
                "flow": flow,
                "pdr_pct": pdr,
                "loss_pct": loss,
                "mean_delay_ms": mean_d,
                "p99_ms": p99,
                "throughput_mbps": tput,
                "tput_share_pct": share,
            }
        )

    # Read-modify-write on the shared gather_csv: must be serialized when
    # called concurrently under --parallel.
    with _CSV_LOCK:
        rows_existing: List[dict] = []
        if gather_csv.exists():
            with gather_csv.open(newline="", encoding="utf-8") as fh:
                rows_existing = [r for r in csv.DictReader(fh) if r.get("run_id") != sc.run_id]
        rows_existing.extend(parsed_rows)
        write_csv(gather_csv, GATHER_FIELDS, rows_existing)


def run_one(
    ns3_root: Path,
    sc: Scenario,
    results_root: str,
    campaign_name: str,
    dry_run: bool,
    collect_metrics: bool,
    gather_csv: Path,
) -> dict:
    rel = sc.relative_dir(results_root, campaign_name)
    run_dir = ns3_root / rel
    cmd = build_ns3_command(sc, rel)

    row = {
        "run_id": sc.run_id,
        "status": "ok",
        "exit_code": 0,
        "elapsed_sec": "0.000",
        "scenario_dir": rel,
        "cellularMode": sc.cellular_mode,
        "hotspotBand": sc.hotspot_band,
        "numStaNodes": sc.num_sta,
        "payload": sc.payload,
        "flowScale": sc.flow_scale,
        "rngSeed": sc.seed,
        "simTime": sc.sim_time,
        "switch_events": 0,
        "resolved": 0,
        "timeout": 0,
        "superseded": 0,
        "error": "",
    }

    if dry_run:
        print("DRY-RUN:", " ".join(cmd))
        row["status"] = "dry_run"
        return row

    run_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    try:
        proc = subprocess.run(cmd, cwd=ns3_root, text=True, capture_output=True)
        elapsed = time.time() - t0
        row["elapsed_sec"] = f"{elapsed:.3f}"
        row["exit_code"] = proc.returncode
        (run_dir / "matrix_run.log").write_text(
            f"command: {' '.join(cmd)}\n"
            f"exit_code: {proc.returncode}\n"
            f"elapsed_sec: {elapsed:.3f}\n\n"
            f"--- stdout ---\n{proc.stdout[-200000:]}\n"
            f"--- stderr ---\n{proc.stderr[-50000:]}\n",
            encoding="utf-8",
        )
        (run_dir / "scenario.json").write_text(
            json.dumps({**asdict(sc), "run_id": sc.run_id, "flow_scale": sc.flow_scale}, indent=2),
            encoding="utf-8",
        )

        outcomes = count_switch_outcomes(run_dir / "wifi-hybrid-switch_log.csv")
        row.update(outcomes)

        if proc.returncode != 0:
            row["status"] = "failed"
            row["error"] = f"ns3 exit {proc.returncode}"
            success_marker(run_dir).unlink(missing_ok=True)
            (run_dir / "RUN_FAILED").write_text(row["error"] + "\n", encoding="utf-8")
            return row

        if not is_successful(run_dir):
            row["status"] = "failed"
            row["error"] = "missing expected output artifacts"
            (run_dir / "RUN_FAILED").write_text(row["error"] + "\n", encoding="utf-8")
            return row

        success_marker(run_dir).write_text("ok\n", encoding="utf-8")
        (run_dir / "RUN_FAILED").unlink(missing_ok=True)

        if collect_metrics:
            gather_flow_metrics(ns3_root, run_dir, sc, gather_csv)

        return row

    except Exception as exc:  # noqa: BLE001
        elapsed = time.time() - t0
        row["elapsed_sec"] = f"{elapsed:.3f}"
        row["status"] = "failed"
        row["exit_code"] = -1
        row["error"] = str(exc)
        (run_dir / "RUN_FAILED").write_text(row["error"] + "\n", encoding="utf-8")
        return row


def rebuild_failed_csv(summary_csv: Path, failed_csv: Path) -> int:
    if not summary_csv.exists():
        write_csv(failed_csv, SUMMARY_FIELDS, [])
        return 0
    with summary_csv.open(newline="", encoding="utf-8") as fh:
        failed = [r for r in csv.DictReader(fh) if r.get("status") == "failed"]
    write_csv(failed_csv, SUMMARY_FIELDS, failed)
    return len(failed)


def write_manifest(campaign_dir: Path, scenarios: List[Scenario], args: argparse.Namespace) -> None:
    manifest = {
        "campaign_name": args.campaign_name,
        "total_planned": len(scenarios),
        "sim_time": args.sim_time,
        "seeds": csv_list(args.seeds),
        "modes": csv_list(args.modes),
        "bands": csv_list(args.bands),
        "stas": csv_list(args.stas),
        "payloads": csv_list(args.payloads),
        "payload_to_flow_scale": PAYLOAD_TO_FLOW_SCALE,
        "formula": "2 modes × 2 bands × 4 STA × 4 payloads × 3 seeds = 192 (with defaults)",
        "run_ids": [s.run_id for s in scenarios],
    }
    (campaign_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()
    ns3_root = Path(__file__).resolve().parent.parent
    if not (ns3_root / "ns3").exists():
        raise SystemExit(f"ns3 launcher not found at {ns3_root}")

    campaign_dir = ns3_root / args.results_root / args.campaign_name
    campaign_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = campaign_dir / "summary.csv"
    failed_csv = campaign_dir / "failed.csv"
    gather_csv = campaign_dir / "gathered_metrics.csv"
    progress_log = campaign_dir / "progress.log"

    scenarios = build_scenarios(args)
    write_manifest(campaign_dir, scenarios, args)

    if args.retry_failed:
        failed_ids = set(load_failed_run_ids(failed_csv))
        if not failed_ids:
            print(f"No failed runs listed in {failed_csv}")
            return 0
        scenarios = [s for s in scenarios if s.run_id in failed_ids]
        print(f"Retrying {len(scenarios)} failed scenario(s) from {failed_csv}")

    if args.limit and args.limit > 0:
        scenarios = scenarios[: args.limit]

    print(f"Campaign root: {campaign_dir}")
    print(f"Planned scenarios this invocation: {len(scenarios)}")
    print(f"Seeds: {csv_list(args.seeds)}")
    print(f"Parallelism: {args.parallel}")

    ok = failed = skipped = 0
    to_run: List[tuple] = []

    for idx, sc in enumerate(scenarios, start=1):
        rel = sc.relative_dir(args.results_root, args.campaign_name)
        run_dir = ns3_root / rel

        if args.skip_existing and is_successful(run_dir) and not args.retry_failed:
            print(f"[{idx}/{len(scenarios)}] SKIP {sc.run_id}")
            outcomes = count_switch_outcomes(run_dir / "wifi-hybrid-switch_log.csv")
            row = {
                "run_id": sc.run_id,
                "status": "skipped",
                "exit_code": 0,
                "elapsed_sec": "0.000",
                "scenario_dir": rel,
                "cellularMode": sc.cellular_mode,
                "hotspotBand": sc.hotspot_band,
                "numStaNodes": sc.num_sta,
                "payload": sc.payload,
                "flowScale": sc.flow_scale,
                "rngSeed": sc.seed,
                "simTime": sc.sim_time,
                "error": "",
                **outcomes,
            }
            append_or_replace_row(summary_csv, SUMMARY_FIELDS, row)
            skipped += 1
            continue

        to_run.append((idx, sc))

    done_count = 0
    t_start = time.time()
    with ThreadPoolExecutor(max_workers=max(1, args.parallel)) as executor:
        futures = {
            executor.submit(
                run_one,
                ns3_root=ns3_root,
                sc=sc,
                results_root=args.results_root,
                campaign_name=args.campaign_name,
                dry_run=args.dry_run,
                collect_metrics=not args.no_metrics and not args.dry_run,
                gather_csv=gather_csv,
            ): (idx, sc)
            for idx, sc in to_run
        }
        for future in as_completed(futures):
            idx, sc = futures[future]
            row = future.result()
            append_or_replace_row(summary_csv, SUMMARY_FIELDS, row)

            with progress_log.open("a", encoding="utf-8") as fh:
                fh.write(
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {row['status']:8}  "
                    f"{sc.run_id}  exit={row['exit_code']}  {row['elapsed_sec']}s  {row.get('error','')}\n"
                )

            if row["status"] == "ok":
                ok += 1
            elif row["status"] == "failed":
                failed += 1
            else:
                skipped += 1

            done_count += 1
            n_failed = rebuild_failed_csv(summary_csv, failed_csv)
            elapsed_total = time.time() - t_start
            rate = done_count / elapsed_total if elapsed_total > 0 else 0
            eta_min = (len(to_run) - done_count) / rate / 60 if rate > 0 else float("inf")
            print(
                f"[{done_count}/{len(to_run)}] {sc.run_id}: {row['status']} ({row['elapsed_sec']}s)  "
                f"switches={row['switch_events']} resolved={row['resolved']} "
                f"timeout={row['timeout']}  | failed.csv={n_failed}  "
                f"| elapsed={elapsed_total/60:.1f}min eta={eta_min:.1f}min",
                flush=True,
            )

    n_failed = rebuild_failed_csv(summary_csv, failed_csv)
    print()
    print("======== CAMPAIGN SUMMARY ========")
    print(f"ok={ok}  failed={failed}  skipped/dry={skipped}  total_this_run={len(scenarios)}")
    print(f"summary:  {summary_csv}")
    print(f"failed:   {failed_csv}  ({n_failed} entries)")
    print(f"metrics:  {gather_csv}")
    print(f"manifest: {campaign_dir / 'manifest.json'}")
    if n_failed:
        print(f"\nRetry failures later with:")
        print(f"  python3 tools/run_traffic_qos_matrix.py --retry-failed --campaign-name {args.campaign_name}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
