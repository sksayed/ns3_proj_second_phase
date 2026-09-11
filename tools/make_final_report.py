#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any
from collections import defaultdict

try:
    import matplotlib  # type: ignore[reportMissingImports]
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # type: ignore[reportMissingImports]
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# Project root relative to this script
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS_ROOT = PROJECT_ROOT / "hybrid_test_results"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "analysis_reports"

# Example folder name:
# Wifi_hybrid_matrix_full_lte_2g_sta5_50kb_seed7_th-58_h2_pdr0p90_spd10_t90
DIR_RE = re.compile(
    r"^Wifi_hybrid_matrix_full_"
    r"(?P<mode>[^_]+)_"
    r"(?P<band>[^_]+)_"
    r"sta(?P<sta>\d+)_"
    r"(?P<payload>[^_]+)_"
    r"seed(?P<seed>\d+)_"
    r"th(?P<threshold>-?\d+)_"
    r"h(?P<hysteresis>\d+)_"
    r"pdr(?P<pdr_tag>[^_]+)_"
    r"spd(?P<speed>\d+)_"
    r"t(?P<sim_time>\d+)$"
)


@dataclass
class ScenarioRecord:
    scenario_dir: str
    mode: str
    band: str
    sta: int
    payload: str
    seed: int
    threshold_dbm: int
    hysteresis_db: int
    pdr_trigger_tag: str
    speed_mps: int
    sim_time_s: int
    avg_pdr_pct: float | None
    packet_weighted_pdr_pct: float | None
    avg_delay_ms: float | None
    avg_jitter_ms: float | None
    avg_throughput_mbps: float | None
    active_clients: int | None
    delay_target_compliance: str | None
    switch_events: int | None
    switch_to_cellular: int | None
    switch_to_wifi: int | None
    switch_resolved: int | None
    switch_timeout: int | None
    switch_unresolved: int | None
    avg_resolved_switch_latency_ms: float | None
    sta_throughput_fairness: float | None
    sta_pdr_fairness: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a section-wise final report from existing hybrid metrics."
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=DEFAULT_RESULTS_ROOT,
        help=f"Root directory containing scenario result folders (default: {DEFAULT_RESULTS_ROOT}).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory for markdown and CSV tables (default: {DEFAULT_OUTPUT_DIR}).",
    )
    parser.add_argument(
        "--report-name",
        default="final_hybrid_report",
        help="Base name of output markdown file (without extension).",
    )
    parser.add_argument(
        "--no-charts",
        action="store_true",
        help="Disable PNG chart generation under output_dir/figures.",
    )
    return parser.parse_args()


def safe_float(value: str) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_sta_table(lines: list[str]) -> tuple[list[float], list[float]]:
    """Parse per-STA rows for throughput/PDR fairness."""
    pdr_values: list[float] = []
    throughput_values: list[float] = []
    in_table = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("| STA IP |"):
            in_table = True
            continue
        if in_table and stripped.startswith("| **Average**"):
            break
        if in_table and stripped.startswith("| ---"):
            continue
        if in_table and stripped.startswith("|"):
            cols = [c.strip() for c in stripped.strip("|").split("|")]
            # Expected columns:
            # STA IP, Traffic Types, L4 Protocols, PDR, Avg Delay, Avg Jitter, Throughput, TX, RX, Lost
            if len(cols) < 7:
                continue
            if cols[0].startswith("**Average**"):
                continue
            pdr = safe_float(cols[3])
            thr = safe_float(cols[6])
            if pdr is not None:
                pdr_values.append(pdr)
            if thr is not None:
                throughput_values.append(thr)
    return pdr_values, throughput_values


def parse_switching_event_latencies(lines: list[str]) -> list[float]:
    latencies: list[float] = []
    in_table = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("| STA | From | To |"):
            in_table = True
            continue
        if in_table and stripped.startswith("| ---"):
            continue
        if in_table and stripped.startswith("|"):
            cols = [c.strip() for c in stripped.strip("|").split("|")]
            if len(cols) < 6:
                continue
            recovered = safe_float(cols[4])
            latency = safe_float(cols[5])
            if recovered is not None and latency is not None and recovered >= 0:
                latencies.append(latency)
        elif in_table and stripped and not stripped.startswith("|"):
            break
    return latencies


def jain_fairness(values: list[float]) -> float | None:
    positive = [v for v in values if v is not None and v >= 0]
    n = len(positive)
    if n == 0:
        return None
    denom = n * sum(v * v for v in positive)
    if denom <= 0:
        return None
    return (sum(positive) ** 2) / denom


def parse_markdown_metrics(path: Path) -> dict[str, Any]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()

    def find_line(pred):
        for line in lines:
            if pred(line):
                return line
        return None

    # Average row in FlowMonitor table
    avg_line = find_line(lambda l: l.startswith("| **Average**"))
    avg_pdr_pct = avg_delay_ms = avg_jitter_ms = avg_thr_mbps = None
    if avg_line:
        cols = [c.strip() for c in avg_line.split("|")]
        try:
            avg_pdr_pct = float(cols[4]) if cols[4] else None
            avg_delay_ms = float(cols[5]) if cols[5] else None
            avg_jitter_ms = float(cols[6]) if cols[6] else None
            avg_thr_mbps = float(cols[7]) if cols[7] else None
        except (ValueError, IndexError):
            pass

    pw_line = find_line(lambda l: "Packet-weighted PDR" in l)
    pw_pdr_pct = None
    if pw_line:
        m = re.search(r"\*\*(.*?)\*\*", pw_line)
        if m:
            pw_pdr_pct = safe_float(m.group(1))

    active_clients = None
    delay_compliance = None
    switch_events = None
    to_cellular = None
    to_wifi = None
    resolved = timeout = unresolved = None
    avg_resolved_switch_latency_ms = None

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("- Active clients:"):
            m = re.search(r"\*\*(\d+)\*\*", stripped)
            if m:
                active_clients = int(m.group(1))
        elif stripped.startswith("- Delay target compliance"):
            m = re.search(r"\*\*(.*?)\*\*", stripped)
            if m:
                delay_compliance = m.group(1)
        elif stripped.startswith("- Switch events:"):
            m_total = re.search(r"\*\*(\d+)\*\*", stripped)
            m_to_c = re.search(r"to cellular:\s*(\d+)", stripped)
            m_to_w = re.search(r"to WiFi:\s*(\d+)", stripped)
            if m_total:
                switch_events = int(m_total.group(1))
            if m_to_c:
                to_cellular = int(m_to_c.group(1))
            if m_to_w:
                to_wifi = int(m_to_w.group(1))
        elif stripped.startswith("- Switching outcomes:"):
            m_res = re.search(r"resolved=(\d+)", stripped)
            m_to = re.search(r"timeout=(\d+)", stripped)
            m_un = re.search(r"unresolved=(\d+)", stripped)
            if m_res:
                resolved = int(m_res.group(1))
            if m_to:
                timeout = int(m_to.group(1))
            if m_un:
                unresolved = int(m_un.group(1))
        elif stripped.startswith("- Avg resolved switching latency"):
            m = re.search(r"\*\*(.*?)\*\*", stripped)
            if m and m.group(1).strip().upper() != "N/A":
                avg_resolved_switch_latency_ms = safe_float(m.group(1))

    # Fallback: compute from switching table if summary line is N/A/missing
    if avg_resolved_switch_latency_ms is None:
        latencies = parse_switching_event_latencies(lines)
        if latencies:
            avg_resolved_switch_latency_ms = statistics.fmean(latencies)

    sta_pdr_values, sta_thr_values = parse_sta_table(lines)
    sta_pdr_fairness = jain_fairness(sta_pdr_values)
    sta_throughput_fairness = jain_fairness(sta_thr_values)

    return {
        "avg_pdr_pct": avg_pdr_pct,
        "avg_delay_ms": avg_delay_ms,
        "avg_jitter_ms": avg_jitter_ms,
        "avg_throughput_mbps": avg_thr_mbps,
        "packet_weighted_pdr_pct": pw_pdr_pct,
        "active_clients": active_clients,
        "delay_target_compliance": delay_compliance,
        "switch_events": switch_events,
        "switch_to_cellular": to_cellular,
        "switch_to_wifi": to_wifi,
        "switch_resolved": resolved,
        "switch_timeout": timeout,
        "switch_unresolved": unresolved,
        "avg_resolved_switch_latency_ms": avg_resolved_switch_latency_ms,
        "sta_throughput_fairness": sta_throughput_fairness,
        "sta_pdr_fairness": sta_pdr_fairness,
    }


def extract_records(results_root: Path) -> list[ScenarioRecord]:
    records: list[ScenarioRecord] = []
    for md_path in results_root.rglob("wifi-hybrid-metrics_data.md"):
        scenario_dir = md_path.parent.name
        m = DIR_RE.match(scenario_dir)
        if not m:
            continue
        meta = m.groupdict()
        metrics = parse_markdown_metrics(md_path)

        records.append(
            ScenarioRecord(
                scenario_dir=scenario_dir,
                mode=meta["mode"],
                band=meta["band"],
                sta=int(meta["sta"]),
                payload=meta["payload"],
                seed=int(meta["seed"]),
                threshold_dbm=int(meta["threshold"]),
                hysteresis_db=int(meta["hysteresis"]),
                pdr_trigger_tag=meta["pdr_tag"],
                speed_mps=int(meta["speed"]),
                sim_time_s=int(meta["sim_time"]),
                avg_pdr_pct=metrics["avg_pdr_pct"],
                packet_weighted_pdr_pct=metrics["packet_weighted_pdr_pct"],
                avg_delay_ms=metrics["avg_delay_ms"],
                avg_jitter_ms=metrics["avg_jitter_ms"],
                avg_throughput_mbps=metrics["avg_throughput_mbps"],
                active_clients=metrics["active_clients"],
                delay_target_compliance=metrics["delay_target_compliance"],
                switch_events=metrics["switch_events"],
                switch_to_cellular=metrics["switch_to_cellular"],
                switch_to_wifi=metrics["switch_to_wifi"],
                switch_resolved=metrics["switch_resolved"],
                switch_timeout=metrics["switch_timeout"],
                switch_unresolved=metrics["switch_unresolved"],
                avg_resolved_switch_latency_ms=metrics["avg_resolved_switch_latency_ms"],
                sta_throughput_fairness=metrics["sta_throughput_fairness"],
                sta_pdr_fairness=metrics["sta_pdr_fairness"],
            )
        )
    return records


def fmt(value: float | None, digits: int = 2) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float) and math.isnan(value):
        return "N/A"
    return f"{value:.{digits}f}"


def mean_or_none(values: list[float | None]) -> float | None:
    valid = [v for v in values if v is not None]
    if not valid:
        return None
    return statistics.fmean(valid)


def write_csv(path: Path, headers: list[str], rows: list[list[Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)


def section_switch_summary(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    rows: list[list[Any]] = []
    for mode in sorted({r.mode for r in records}):
        items = [r for r in records if r.mode == mode]
        total_switches = sum(r.switch_events or 0 for r in items)
        total_timeout = sum(r.switch_timeout or 0 for r in items)
        total_resolved = sum(r.switch_resolved or 0 for r in items)
        failure_rate = (total_timeout / total_switches) if total_switches > 0 else None
        success_rate = (total_resolved / total_switches) if total_switches > 0 else None
        rows.append(
            [
                mode,
                len(items),
                total_switches,
                total_resolved,
                total_timeout,
                fmt(failure_rate, 4),
                fmt(success_rate, 4),
            ]
        )
    headers = [
        "mode",
        "scenario_count",
        "total_switches",
        "resolved",
        "timeout",
        "switch_failure_rate",
        "switch_success_rate",
    ]
    return headers, rows


def section_handover_by_condition(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[int, str, int], list[ScenarioRecord]] = {}
    for r in records:
        key = (r.sta, r.payload, r.speed_mps)
        grouped.setdefault(key, []).append(r)

    rows: list[list[Any]] = []
    for key in sorted(grouped.keys()):
        items = grouped[key]
        total_switches = sum(r.switch_events or 0 for r in items)
        resolved = sum(r.switch_resolved or 0 for r in items)
        timeout = sum(r.switch_timeout or 0 for r in items)
        success_rate = (resolved / total_switches) if total_switches > 0 else None
        failure_rate = (timeout / total_switches) if total_switches > 0 else None
        avg_latency = mean_or_none([r.avg_resolved_switch_latency_ms for r in items])
        rows.append(
            [
                key[0],
                key[1],
                key[2],
                len(items),
                total_switches,
                resolved,
                timeout,
                fmt(success_rate, 4),
                fmt(failure_rate, 4),
                fmt(avg_latency, 2),
            ]
        )

    headers = [
        "sta",
        "payload",
        "speed_mps",
        "scenario_count",
        "total_switches",
        "resolved",
        "timeout",
        "handover_success_rate",
        "switch_failure_rate",
        "avg_resolved_switch_latency_ms",
    ]
    return headers, rows


def section_stability(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str, int, str, int], list[float]] = {}
    for r in records:
        clients = r.active_clients if r.active_clients and r.active_clients > 0 else r.sta
        if clients <= 0 or r.sim_time_s <= 0:
            continue
        stability = (r.switch_events or 0) / (r.sim_time_s / 60.0) / clients
        key = (r.mode, r.band, r.sta, r.payload, r.speed_mps)
        grouped.setdefault(key, []).append(stability)

    rows: list[list[Any]] = []
    for key in sorted(grouped.keys()):
        values = grouped[key]
        mean_val = statistics.fmean(values) if values else None
        rows.append(
            [
                key[0],
                key[1],
                key[2],
                key[3],
                key[4],
                len(values),
                fmt(mean_val, 4),
            ]
        )

    headers = [
        "mode",
        "band",
        "sta",
        "payload",
        "speed_mps",
        "seed_count",
        "mean_switches_per_minute_per_sta",
    ]
    return headers, rows


def section_fairness(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str, int, str, int], list[ScenarioRecord]] = {}
    for r in records:
        key = (r.mode, r.band, r.sta, r.payload, r.speed_mps)
        grouped.setdefault(key, []).append(r)

    rows: list[list[Any]] = []
    for key in sorted(grouped.keys()):
        items = grouped[key]
        thr_fair = mean_or_none([r.sta_throughput_fairness for r in items])
        pdr_fair = mean_or_none([r.sta_pdr_fairness for r in items])
        rows.append(
            [
                key[0],
                key[1],
                key[2],
                key[3],
                key[4],
                len(items),
                fmt(thr_fair, 4),
                fmt(pdr_fair, 4),
            ]
        )
    headers = [
        "mode",
        "band",
        "sta",
        "payload",
        "speed_mps",
        "seed_count",
        "jain_throughput_fairness_mean",
        "jain_pdr_fairness_mean",
    ]
    return headers, rows


def section_pareto(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str, int, str, int], list[ScenarioRecord]] = {}
    for r in records:
        key = (r.mode, r.band, r.sta, r.payload, r.speed_mps)
        grouped.setdefault(key, []).append(r)

    points: list[dict[str, Any]] = []
    for key, items in grouped.items():
        pdr = mean_or_none([r.packet_weighted_pdr_pct for r in items])
        delay = mean_or_none([r.avg_delay_ms for r in items])
        if pdr is None or delay is None:
            continue
        points.append(
            {
                "mode": key[0],
                "band": key[1],
                "sta": key[2],
                "payload": key[3],
                "speed_mps": key[4],
                "seed_count": len(items),
                "mean_packet_weighted_pdr_pct": pdr,
                "mean_delay_ms": delay,
            }
        )

    # Pareto frontier: maximize PDR, minimize Delay
    frontier: list[dict[str, Any]] = []
    for p in points:
        dominated = False
        for q in points:
            if q is p:
                continue
            better_or_equal = (
                q["mean_packet_weighted_pdr_pct"] >= p["mean_packet_weighted_pdr_pct"]
                and q["mean_delay_ms"] <= p["mean_delay_ms"]
            )
            strictly_better = (
                q["mean_packet_weighted_pdr_pct"] > p["mean_packet_weighted_pdr_pct"]
                or q["mean_delay_ms"] < p["mean_delay_ms"]
            )
            if better_or_equal and strictly_better:
                dominated = True
                break
        if not dominated:
            frontier.append(p)

    frontier.sort(
        key=lambda x: (-x["mean_packet_weighted_pdr_pct"], x["mean_delay_ms"])
    )

    rows: list[list[Any]] = []
    for p in frontier:
        rows.append(
            [
                p["mode"],
                p["band"],
                p["sta"],
                p["payload"],
                p["speed_mps"],
                p["seed_count"],
                fmt(p["mean_packet_weighted_pdr_pct"], 2),
                fmt(p["mean_delay_ms"], 2),
            ]
        )
    headers = [
        "mode",
        "band",
        "sta",
        "payload",
        "speed_mps",
        "seed_count",
        "mean_packet_weighted_pdr_pct",
        "mean_delay_ms",
    ]
    return headers, rows


def write_master_csv(records: list[ScenarioRecord], path: Path) -> None:
    headers = [
        "scenario_dir",
        "mode",
        "band",
        "sta",
        "payload",
        "seed",
        "threshold_dbm",
        "hysteresis_db",
        "pdr_trigger_tag",
        "speed_mps",
        "sim_time_s",
        "avg_pdr_pct",
        "packet_weighted_pdr_pct",
        "avg_delay_ms",
        "avg_jitter_ms",
        "avg_throughput_mbps",
        "active_clients",
        "delay_target_compliance",
        "switch_events",
        "switch_to_cellular",
        "switch_to_wifi",
        "switch_resolved",
        "switch_timeout",
        "switch_unresolved",
        "avg_resolved_switch_latency_ms",
        "sta_throughput_fairness",
        "sta_pdr_fairness",
    ]
    rows: list[list[Any]] = []
    for r in records:
        rows.append([getattr(r, h) for h in headers])
    write_csv(path, headers, rows)


def render_markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    out = []
    out.append("| " + " | ".join(headers) + " |")
    out.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        out.append("| " + " | ".join(str(v) for v in row) + " |")
    return "\n".join(out)


def _aggregate_switch_rates_by_mode(
    records: list[ScenarioRecord],
) -> tuple[list[str], list[float], list[float]]:
    modes = sorted({r.mode for r in records})
    failure_rates: list[float] = []
    success_rates: list[float] = []
    for mode in modes:
        items = [r for r in records if r.mode == mode]
        total = sum(r.switch_events or 0 for r in items)
        timeout = sum(r.switch_timeout or 0 for r in items)
        resolved = sum(r.switch_resolved or 0 for r in items)
        failure_rates.append((timeout / total) if total else 0.0)
        success_rates.append((resolved / total) if total else 0.0)
    return modes, failure_rates, success_rates


def generate_charts(
    records: list[ScenarioRecord],
    figures_dir: Path,
) -> dict[str, str]:
    """
    Generate one PNG chart per report section.
    Returns: mapping {section title -> relative markdown image path}.
    """
    chart_map: dict[str, str] = {}
    if not MATPLOTLIB_AVAILABLE:
        return chart_map

    figures_dir.mkdir(parents=True, exist_ok=True)

    # 1) Switch reliability summary
    modes, failure_rates, success_rates = _aggregate_switch_rates_by_mode(records)
    if modes:
        plt.figure(figsize=(7, 4))
        x = range(len(modes))
        plt.bar([i - 0.18 for i in x], failure_rates, width=0.36, label="Failure rate")
        plt.bar([i + 0.18 for i in x], success_rates, width=0.36, label="Success rate")
        plt.xticks(list(x), [m.upper() for m in modes])
        plt.ylim(0, 1.0)
        plt.ylabel("Rate")
        plt.title("Switch Failure/Success Rate by Mode")
        plt.legend()
        plt.grid(axis="y", alpha=0.3)
        plt.tight_layout()
        p = figures_dir / "section1_switch_reliability.png"
        plt.savefig(p, dpi=150, bbox_inches="tight")
        plt.close()
        chart_map["1. Switch Reliability Summary"] = f"figures/{p.name}"

    # 2) Handover success by condition: success vs latency scatter
    grouped: dict[tuple[int, str, int], list[ScenarioRecord]] = defaultdict(list)
    for r in records:
        grouped[(r.sta, r.payload, r.speed_mps)].append(r)
    xs: list[float] = []
    ys: list[float] = []
    colors: list[int] = []
    for (sta, payload, speed), items in grouped.items():
        total = sum(i.switch_events or 0 for i in items)
        resolved = sum(i.switch_resolved or 0 for i in items)
        success = (resolved / total) if total > 0 else None
        latency = mean_or_none([i.avg_resolved_switch_latency_ms for i in items])
        if success is None or latency is None:
            continue
        xs.append(success)
        ys.append(latency)
        colors.append(speed)
    if xs:
        plt.figure(figsize=(7, 4))
        sc = plt.scatter(xs, ys, c=colors, cmap="viridis", alpha=0.8)
        plt.xlabel("Handover success rate")
        plt.ylabel("Avg resolved switching latency (ms)")
        plt.title("Handover Success by Condition")
        plt.grid(alpha=0.3)
        cbar = plt.colorbar(sc)
        cbar.set_label("Speed (m/s)")
        plt.tight_layout()
        p = figures_dir / "section2_handover_condition.png"
        plt.savefig(p, dpi=150, bbox_inches="tight")
        plt.close()
        chart_map["2. Handover Success by Condition"] = f"figures/{p.name}"

    # 3) Stability index by STA
    sta_vals: dict[int, list[float]] = defaultdict(list)
    for r in records:
        clients = r.active_clients if r.active_clients and r.active_clients > 0 else r.sta
        if clients <= 0 or r.sim_time_s <= 0:
            continue
        stab = (r.switch_events or 0) / (r.sim_time_s / 60.0) / clients
        sta_vals[r.sta].append(stab)
    if sta_vals:
        x_sta = sorted(sta_vals.keys())
        y_mean = [statistics.fmean(sta_vals[s]) for s in x_sta]
        plt.figure(figsize=(7, 4))
        plt.plot(x_sta, y_mean, marker="o")
        plt.xlabel("STA count")
        plt.ylabel("Mean switches/min/STA")
        plt.title("Stability Index vs STA Count")
        plt.grid(alpha=0.3)
        plt.tight_layout()
        p = figures_dir / "section3_stability.png"
        plt.savefig(p, dpi=150, bbox_inches="tight")
        plt.close()
        chart_map["3. Stability Index"] = f"figures/{p.name}"

    # 4) Fairness scatter: throughput fairness vs pdr fairness
    xf: list[float] = []
    yf: list[float] = []
    mode_color: list[int] = []
    mode_map = {"lte": 0, "nr": 1}
    for r in records:
        if r.sta_throughput_fairness is None or r.sta_pdr_fairness is None:
            continue
        xf.append(r.sta_throughput_fairness)
        yf.append(r.sta_pdr_fairness)
        mode_color.append(mode_map.get(r.mode, 2))
    if xf:
        plt.figure(figsize=(7, 4))
        sc = plt.scatter(xf, yf, c=mode_color, cmap="coolwarm", alpha=0.6)
        plt.xlabel("Jain throughput fairness")
        plt.ylabel("Jain PDR fairness")
        plt.title("Fairness Across STAs")
        plt.grid(alpha=0.3)
        # build simple legend manually
        handles, _ = sc.legend_elements()
        plt.legend(handles, ["LTE", "NR", "Other"][: len(handles)], title="Mode")
        plt.tight_layout()
        p = figures_dir / "section4_fairness.png"
        plt.savefig(p, dpi=150, bbox_inches="tight")
        plt.close()
        chart_map["4. Fairness Across STAs"] = f"figures/{p.name}"

    # 5) Pareto view: all points + frontier highlight
    grouped_p: dict[tuple[str, str, int, str, int], list[ScenarioRecord]] = defaultdict(list)
    for r in records:
        grouped_p[(r.mode, r.band, r.sta, r.payload, r.speed_mps)].append(r)
    points: list[dict[str, float]] = []
    for items in grouped_p.values():
        pdr = mean_or_none([i.packet_weighted_pdr_pct for i in items])
        delay = mean_or_none([i.avg_delay_ms for i in items])
        if pdr is None or delay is None:
            continue
        points.append({"pdr": pdr, "delay": delay})
    if points:
        frontier_idx: list[int] = []
        for i, p in enumerate(points):
            dominated = False
            for j, q in enumerate(points):
                if i == j:
                    continue
                if (q["pdr"] >= p["pdr"] and q["delay"] <= p["delay"]) and (
                    q["pdr"] > p["pdr"] or q["delay"] < p["delay"]
                ):
                    dominated = True
                    break
            if not dominated:
                frontier_idx.append(i)
        plt.figure(figsize=(7, 4))
        plt.scatter([p["delay"] for p in points], [p["pdr"] for p in points], alpha=0.35, label="All configs")
        fx = [points[i]["delay"] for i in frontier_idx]
        fy = [points[i]["pdr"] for i in frontier_idx]
        plt.scatter(fx, fy, color="red", alpha=0.9, label="Pareto frontier")
        plt.xlabel("Mean delay (ms)")
        plt.ylabel("Mean packet-weighted PDR (%)")
        plt.title("Pareto View: Reliability vs Latency")
        plt.grid(alpha=0.3)
        plt.legend()
        plt.tight_layout()
        p = figures_dir / "section5_pareto.png"
        plt.savefig(p, dpi=150, bbox_inches="tight")
        plt.close()
        chart_map["5. Pareto Frontier (Reliability vs Latency)"] = f"figures/{p.name}"

    return chart_map


def write_markdown_report(
    report_path: Path,
    records: list[ScenarioRecord],
    section_tables: list[tuple[str, str, list[str], list[list[Any]]]],
    section_charts: dict[str, str] | None = None,
) -> None:
    lines: list[str] = []
    lines.append("# Hybrid Matrix Final Analysis Report")
    lines.append("")
    lines.append(f"- Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"- Scenarios parsed: {len(records)}")
    lines.append("")

    for title, note, headers, rows in section_tables:
        lines.append(f"## {title}")
        lines.append("")
        if note:
            lines.append(f"- {note}")
            lines.append("")
        if rows:
            lines.append(render_markdown_table(headers, rows))
        else:
            lines.append("_No data available for this section._")
        if section_charts and title in section_charts:
            lines.append("")
            lines.append(f"![{title}]({section_charts[title]})")
        lines.append("")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    if not args.results_root.exists():
        print(f"Results directory not found: {args.results_root}")
        return

    records = extract_records(args.results_root)
    if not records:
        print("No full-matrix metrics files found. Nothing to report.")
        return

    output_dir: Path = args.output_dir
    report_data_dir = output_dir / "report_data"
    output_dir.mkdir(parents=True, exist_ok=True)
    report_data_dir.mkdir(parents=True, exist_ok=True)

    # 1) Master CSV (reuses and extends previous behavior)
    master_csv = output_dir / "wifi_hybrid_final_report.csv"
    write_master_csv(records, master_csv)

    # 2) Section-wise CSVs
    s1_h, s1_r = section_switch_summary(records)
    s2_h, s2_r = section_handover_by_condition(records)
    s3_h, s3_r = section_stability(records)
    s4_h, s4_r = section_fairness(records)
    s5_h, s5_r = section_pareto(records)

    write_csv(report_data_dir / "switch_summary.csv", s1_h, s1_r)
    write_csv(report_data_dir / "handover_success_by_condition.csv", s2_h, s2_r)
    write_csv(report_data_dir / "stability_index.csv", s3_h, s3_r)
    write_csv(report_data_dir / "fairness_summary.csv", s4_h, s4_r)
    write_csv(report_data_dir / "pareto_frontier.csv", s5_h, s5_r)

    # 3) Markdown report
    report_path = output_dir / f"{args.report_name}.md"
    section_tables = [
        (
            "1. Switch Reliability Summary",
            "Switch failure rate = timeout / total switches, aggregated by mode.",
            s1_h,
            s1_r,
        ),
        (
            "2. Handover Success by Condition",
            "Condition key = (STA count, payload, speed).",
            s2_h,
            s2_r,
        ),
        (
            "3. Stability Index",
            "Stability index = switches per minute per STA.",
            s3_h,
            s3_r,
        ),
        (
            "4. Fairness Across STAs",
            "Jain fairness index computed from per-STA throughput and PDR rows.",
            s4_h,
            s4_r,
        ),
        (
            "5. Pareto Frontier (Reliability vs Latency)",
            "Pareto-optimal configurations maximize packet-weighted PDR and minimize delay.",
            s5_h,
            s5_r,
        ),
    ]
    section_charts: dict[str, str] = {}
    if not args.no_charts:
        if MATPLOTLIB_AVAILABLE:
            figures_dir = output_dir / "figures"
            section_charts = generate_charts(records, figures_dir)
        else:
            print("Chart generation skipped: matplotlib is not installed.")

    write_markdown_report(report_path, records, section_tables, section_charts)

    print(f"Wrote master CSV: {master_csv}")
    print(f"Wrote section CSVs under: {report_data_dir}")
    print(f"Wrote markdown report: {report_path}")
    if section_charts:
        print(f"Wrote {len(section_charts)} chart(s) under: {output_dir / 'figures'}")


if __name__ == "__main__":
    main()

