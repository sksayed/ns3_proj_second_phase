#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List


@dataclass(frozen=True)
class Scenario:
    phase: str
    mode: str
    band: str
    sta: int
    payload: str
    seed: int
    hysteresis: int
    threshold: int
    pdr_threshold: float
    sim_time: int
    sta_speed_mps: int

    @property
    def output_dir(self) -> str:
        pdr_tag = f"{self.pdr_threshold:.2f}".replace(".", "p")
        return (
            f"{self.phase}_{self.mode}_{self.band}_sta{self.sta}_"
            f"{self.payload}_seed{self.seed}_th{self.threshold}_h{self.hysteresis}_"
            f"pdr{pdr_tag}_spd{self.sta_speed_mps}_t{self.sim_time}"
        )


PAYLOAD_TO_BYTES = {
    "10kb": 10 * 1024,
    "50kb": 50 * 1024,
    "1mb": 1 * 1024 * 1024,
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Run WiFi-hybrid scenario matrix and parse per-run RSSI/network reports.\n"
            "Phases:\n"
            "  screening: sta=[5,10,15], payload=50kb, seed=6, h=[2,3,4], speed=[5,10,15]\n"
            "  full:      sta=[5,10,15], payload=[10kb,50kb,1mb], seed=[6,7,8], "
            "h=[best_hysteresis], speed=[5,10,15]"
        )
    )
    p.add_argument(
        "--phase",
        choices=["screening", "full", "both"],
        default="screening",
        help="Which matrix phase to run (default: %(default)s).",
    )
    p.add_argument(
        "--best-hysteresis",
        type=int,
        default=3,
        help="Hysteresis value used by full phase (default: %(default)s).",
    )
    p.add_argument(
        "--threshold",
        type=int,
        default=-58,
        help="WiFi RSSI threshold in dBm (default: %(default)s).",
    )
    p.add_argument(
        "--pdr-thresholds",
        default="0.9",
        help=(
            "Comma-separated per-STA PDR thresholds for switching trigger, "
            "each in range [0,1] (default: %(default)s)."
        ),
    )
    p.add_argument(
        "--sim-time",
        type=int,
        default=90,
        help="Simulation time in seconds (default: %(default)s).",
    )
    p.add_argument(
        "--output-prefix",
        default="Wifi_hybrid_matrix",
        help="Campaign prefix used for generated output folders (default: %(default)s).",
    )
    p.add_argument(
        "--results-root",
        default="hybrid_test_results",
        help=(
            "Primary root directory for all run outputs and summaries "
            "(default: %(default)s)."
        ),
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without executing.",
    )
    p.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip scenarios whose output folder already has switch/rssi logs.",
    )
    return p.parse_args()


def parse_pdr_thresholds(raw: str) -> List[float]:
    values: List[float] = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        val = float(token)
        if val < 0.0 or val > 1.0:
            raise ValueError(f"Invalid PDR threshold {val}; expected 0..1")
        values.append(round(val, 4))
    if not values:
        raise ValueError("At least one PDR threshold is required")
    return values


def build_screening_scenarios(args: argparse.Namespace) -> List[Scenario]:
    scenarios: List[Scenario] = []
    pdr_thresholds = parse_pdr_thresholds(args.pdr_thresholds)
    for mode in ("lte", "nr"):
        for band in ("2g", "5g"):
            for sta in (5, 10, 15):
                for speed in (5, 10, 15):
                    for h in (2, 3, 4):
                        for pdr in pdr_thresholds:
                            scenarios.append(
                                Scenario(
                                    phase="screening",
                                    mode=mode,
                                    band=band,
                                    sta=sta,
                                    payload="50kb",
                                    seed=6,
                                    hysteresis=h,
                                    threshold=args.threshold,
                                    pdr_threshold=pdr,
                                    sim_time=args.sim_time,
                                    sta_speed_mps=speed,
                                )
                            )
    return scenarios


def build_full_scenarios(args: argparse.Namespace) -> List[Scenario]:
    scenarios: List[Scenario] = []
    pdr_thresholds = parse_pdr_thresholds(args.pdr_thresholds)
    for mode in ("lte", "nr"):
        for band in ("2g", "5g"):
            for sta in (5, 10, 15):
                for speed in (5, 10, 15):
                    for payload in ("10kb", "50kb", "1mb"):
                        for seed in (6, 7, 8):
                            for pdr in pdr_thresholds:
                                scenarios.append(
                                    Scenario(
                                        phase="full",
                                        mode=mode,
                                        band=band,
                                        sta=sta,
                                        payload=payload,
                                        seed=seed,
                                        hysteresis=args.best_hysteresis,
                                        threshold=args.threshold,
                                        pdr_threshold=pdr,
                                        sim_time=args.sim_time,
                                        sta_speed_mps=speed,
                                    )
                                )
    return scenarios


def iter_scenarios(args: argparse.Namespace) -> Iterable[Scenario]:
    if args.phase in ("screening", "both"):
        yield from build_screening_scenarios(args)
    if args.phase in ("full", "both"):
        yield from build_full_scenarios(args)


def count_switch_events(switch_log_path: Path) -> tuple[int, int, int]:
    if not switch_log_path.exists():
        return 0, 0, 0
    total = 0
    wifi_to_cell = 0
    cell_to_wifi = 0
    stas = set()
    with switch_log_path.open(newline="") as f:
        rd = csv.DictReader(f)
        for row in rd:
            total += 1
            stas.add(row.get("sta_index", ""))
            src = (row.get("from") or "").strip().lower()
            dst = (row.get("to") or "").strip().lower()
            if src == "wifi" and dst in ("lte", "nr"):
                wifi_to_cell += 1
            if src in ("lte", "nr") and dst == "wifi":
                cell_to_wifi += 1
    return total, len(stas), wifi_to_cell - cell_to_wifi


def run_one(
    ns3_root: Path,
    sc: Scenario,
    results_root: str,
    output_prefix: str,
    dry_run: bool,
) -> tuple[int, float, str]:
    out_dir = f"{results_root}/{output_prefix}_{sc.output_dir}"
    payload_bytes = PAYLOAD_TO_BYTES[sc.payload]
    run_str = (
        "wifi-hybrid-try-2 "
        f"--cellularMode={sc.mode} "
        f"--hotspotBand={sc.band} "
        "--enableSwitching=1 "
        f"--numStaNodes={sc.sta} "
        f"--simTime={sc.sim_time} "
        f"--rssiThresholdDbm={sc.threshold} "
        f"--rssiHysteresisDb={sc.hysteresis} "
        f"--pdrThreshold={sc.pdr_threshold} "
        f"--rngSeed={sc.seed} "
        f"--uploadBytes={payload_bytes} "
        f"--downloadBytes={payload_bytes} "
        f"--staSpeedMin={sc.sta_speed_mps} "
        f"--staSpeedMax={sc.sta_speed_mps} "
        f"--outputDir={out_dir}"
    )
    cmd = ["./ns3", "run", run_str]

    if dry_run:
        print("DRY-RUN:", " ".join(cmd))
        return 0, 0.0, out_dir

    t0 = time.time()
    proc = subprocess.run(cmd, cwd=ns3_root, text=True, capture_output=True)
    elapsed = time.time() - t0

    # Keep a per-run command log for traceability.
    run_path = ns3_root / out_dir
    run_path.mkdir(parents=True, exist_ok=True)
    (run_path / "matrix_run.log").write_text(
        f"command: {' '.join(cmd)}\n"
        f"exit_code: {proc.returncode}\n"
        f"elapsed_sec: {elapsed:.3f}\n\n"
        f"--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}\n",
        encoding="utf-8",
    )
    return proc.returncode, elapsed, out_dir


def parse_rssi_report(ns3_root: Path, out_dir: str, dry_run: bool) -> int:
    csv_path = f"{out_dir}/wifi-hybrid-rssi_log.csv"
    if dry_run:
        print(f"DRY-RUN: python3 tools/rssi_report.py -i {csv_path}")
        return 0
    return subprocess.run(
        ["python3", "tools/rssi_report.py", "-i", csv_path],
        cwd=ns3_root,
        text=True,
        capture_output=True,
    ).returncode


def main() -> int:
    args = parse_args()
    ns3_root = Path(__file__).resolve().parent.parent
    if not (ns3_root / "ns3").exists():
        raise SystemExit(f"ns3 launcher not found in expected root: {ns3_root}")
    results_root_path = ns3_root / args.results_root
    results_root_path.mkdir(parents=True, exist_ok=True)

    scenarios = list(iter_scenarios(args))
    print(f"Planned scenarios: {len(scenarios)}")

    summary_rows = []
    for idx, sc in enumerate(scenarios, start=1):
        out_dir = f"{args.results_root}/{args.output_prefix}_{sc.output_dir}"
        run_dir = ns3_root / out_dir
        rssi_log = run_dir / "wifi-hybrid-rssi_log.csv"
        switch_log = run_dir / "wifi-hybrid-switch_log.csv"
        if args.skip_existing and rssi_log.exists() and switch_log.exists():
            print(f"[{idx}/{len(scenarios)}] SKIP existing {out_dir}")
            total, unique_stas, directional_balance = count_switch_events(switch_log)
            summary_rows.append(
                {
                    "scenario": out_dir,
                    "status": "skipped",
                    "exit_code": 0,
                    "elapsed_sec": "0.000",
                    "switch_events": total,
                    "unique_switch_stas": unique_stas,
                    "direction_balance_w2c_minus_c2w": directional_balance,
                }
            )
            continue

        print(f"[{idx}/{len(scenarios)}] RUN {out_dir}")
        rc, elapsed, out_dir_real = run_one(
            ns3_root, sc, args.results_root, args.output_prefix, args.dry_run
        )
        parse_rc = parse_rssi_report(ns3_root, out_dir_real, args.dry_run)

        total, unique_stas, directional_balance = count_switch_events(ns3_root / out_dir_real / "wifi-hybrid-switch_log.csv")
        status = "ok" if rc == 0 and parse_rc == 0 else "failed"
        summary_rows.append(
            {
                "scenario": out_dir_real,
                "status": status,
                "exit_code": rc,
                "elapsed_sec": f"{elapsed:.3f}",
                "switch_events": total,
                "unique_switch_stas": unique_stas,
                "direction_balance_w2c_minus_c2w": directional_balance,
                "phase": sc.phase,
                "mode": sc.mode,
                "band": sc.band,
                "sta": sc.sta,
                "payload": sc.payload,
                "seed": sc.seed,
                "threshold": sc.threshold,
                "hysteresis": sc.hysteresis,
                    "pdr_threshold": sc.pdr_threshold,
                "sim_time": sc.sim_time,
                "sta_speed_mps": sc.sta_speed_mps,
            }
        )

    summary_path = results_root_path / f"{args.output_prefix}_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        fields = [
            "scenario",
            "status",
            "exit_code",
            "elapsed_sec",
            "switch_events",
            "unique_switch_stas",
            "direction_balance_w2c_minus_c2w",
            "phase",
            "mode",
            "band",
            "sta",
            "payload",
            "seed",
            "threshold",
            "hysteresis",
            "pdr_threshold",
            "sim_time",
            "sta_speed_mps",
        ]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in summary_rows:
            w.writerow(row)

    ok = sum(1 for r in summary_rows if r.get("status") in ("ok", "skipped"))
    print(f"Finished {len(summary_rows)} scenarios. Non-failed: {ok}")
    print(f"Summary CSV: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

