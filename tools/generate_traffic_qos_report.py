#!/usr/bin/env python3
"""
Generate traffic_qos_report.md/.pdf from the 192-run July campaign.

Reads:
  Traffic_qos_outputs/Traffic_qos_matrix_192/summary.csv
  Traffic_qos_outputs/Traffic_qos_matrix_192/gathered_metrics.csv
  <run>/wifi-hybrid-flowmon_data.xml   (for the per-path-leg breakdown)
  <run>/wifi-hybrid-switch_log.csv     (for uncensored interruption timing)

Writes (under --out-dir):
  figures/*.png
  traffic_qos_report.md
  traffic_qos_report.pdf   (via weasyprint if available)
Also caches leg_metrics.csv in the campaign directory, since re-deriving it means
re-parsing every FlowMonitor XML.

Two reporting corrections relative to the raw gathered_metrics.csv:
  * Per-flow PDR is additionally reported per network path leg. A path switch
    changes the STA source address, so FlowMonitor splits one application stream
    into two flows; summing them blends a working leg with a failing one.
  * Switch interruptions at the scenario's wait ceiling are right-censored lower
    bounds, not measurements, and are reported separately instead of averaged in.

Usage (from ns-3.45/):
  python3 tools/generate_traffic_qos_report.py
  python3 tools/generate_traffic_qos_report.py \\
      --campaign-dir Traffic_qos_outputs/Traffic_qos_matrix_192 \\
      --out-dir Traffic_qos_outputs/Traffic_qos_matrix_192/report
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "examples" / "my-scenarios"))
from flow_metrics import (  # noqa: E402
    CONTINUITY_WINDOW_S,
    LEG_CELL,
    LEG_WIFI,
    aggregate_by_leg,
    analyze_switch_log,
    parse_flowmon,
)

AUTHOR = "Sheikh Sayed Bin Rahman"
LAB = "PIC Lab, KIT"
FLOWS = ("Control", "Sensor", "Video")
LEGS = (LEG_WIFI, LEG_CELL)
# Control flow latency: the plan gives a per-flow requirement (section 3.1) and a
# project end-to-end sync target (section 3.3). Both are reported.
CONTROL_FLOW_TARGET_MS = 50.0
CONTROL_SYNC_TARGET_MS = 200.0
PAYLOAD_ORDER = ("10kb", "50kb", "1mb", "2mb")
STA_ORDER = (5, 10, 15, 20)
QOS_TARGETS = {
    "Control": {"flow_ms": CONTROL_FLOW_TARGET_MS, "sync_ms": CONTROL_SYNC_TARGET_MS,
                "loss_pct": 0.0},
    "Sensor": {"flow_ms": 200.0, "sync_ms": 200.0, "loss_pct": 5.0},
    "Video": {"flow_ms": 500.0, "sync_ms": 500.0, "loss_pct": 10.0},
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build traffic_qos_report with figures")
    p.add_argument(
        "--campaign-dir",
        default="Traffic_qos_outputs/Traffic_qos_matrix_192",
        help="Campaign folder containing summary.csv and gathered_metrics.csv",
    )
    p.add_argument(
        "--out-dir",
        default="",
        help="Output directory (default: <campaign-dir>/report)",
    )
    p.add_argument("--no-pdf", action="store_true", help="Skip PDF generation")
    p.add_argument(
        "--refresh-legs",
        action="store_true",
        help="Re-parse the FlowMonitor XMLs instead of using cached leg_metrics.csv",
    )
    p.add_argument(
        "--switch-timeout",
        type=float,
        default=5.0,
        help="switchTimeoutSec used by the campaign; interruptions at this ceiling are censored",
    )
    return p.parse_args()


def fnum(x: str, default: float = float("nan")) -> float:
    try:
        if x is None or str(x).strip() in ("", "n/a", "N/A"):
            return default
        return float(x)
    except ValueError:
        return default


def mean(xs: Sequence[float]) -> float:
    vals = [x for x in xs if not math.isnan(x)]
    return statistics.mean(vals) if vals else float("nan")


def stdev(xs: Sequence[float]) -> float:
    vals = [x for x in xs if not math.isnan(x)]
    return statistics.stdev(vals) if len(vals) > 1 else 0.0


def load_csv(path: Path) -> List[dict]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


LEG_FIELDS = (
    "run_id", "cellularMode", "hotspotBand", "numStaNodes", "payload", "rngSeed",
    "flow", "leg", "sub_flows", "tx_packets", "rx_packets", "pdr_pct",
    "mean_delay_ms", "p99_ms", "throughput_mbps", "dark_sub_flows",
)


def build_leg_metrics(campaign: Path, summary: List[dict], cache: Path,
                      refresh: bool = False) -> List[dict]:
    """Per-run, per-flow, per-leg metrics derived by re-parsing the FlowMonitor XMLs."""
    if cache.exists() and not refresh:
        rows = load_csv(cache)
        if rows:
            print(f"Loaded cached per-leg metrics from {cache.name} ({len(rows)} rows)")
            return rows

    by_run = {r["run_id"]: r for r in summary}
    rows: List[dict] = []
    print("Deriving per-leg metrics (re-parsing FlowMonitor XMLs)...")
    for n, (run_id, meta) in enumerate(sorted(by_run.items()), start=1):
        xml = campaign / run_id / "wifi-hybrid-flowmon_data.xml"
        if not xml.exists():
            continue
        sim_time = fnum(meta.get("simTime"), 60.0)
        for (flow, leg), s in aggregate_by_leg(parse_flowmon(xml)).items():
            if not s.tx_packets:
                continue
            p99 = s.p99_ms()
            rows.append(
                {
                    "run_id": run_id,
                    "cellularMode": meta.get("cellularMode", ""),
                    "hotspotBand": meta.get("hotspotBand", ""),
                    "numStaNodes": meta.get("numStaNodes", ""),
                    "payload": meta.get("payload", ""),
                    "rngSeed": meta.get("rngSeed", ""),
                    "flow": flow,
                    "leg": leg,
                    "sub_flows": s.flow_count,
                    "tx_packets": s.tx_packets,
                    "rx_packets": s.rx_packets,
                    "pdr_pct": f"{s.pdr:.4f}",
                    "mean_delay_ms": f"{s.mean_delay_ms:.4f}",
                    "p99_ms": "" if p99 is None else f"{p99:.4f}",
                    "throughput_mbps": f"{s.throughput_mbps(sim_time):.6f}",
                    "dark_sub_flows": s.dark_flow_count,
                }
            )
        if n % 48 == 0:
            print(f"  {n}/{len(by_run)} runs parsed")
    with cache.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(LEG_FIELDS))
        w.writeheader()
        w.writerows(rows)
    print(f"  cached {len(rows)} per-leg rows to {cache.name}")
    return rows


def leg_pdr(leg_rows: List[dict], flow: str, leg: str,
            mode: Optional[str] = None, **filters) -> Tuple[float, int, int]:
    """Packet-weighted PDR for a (flow, leg) slice: returns (pdr_pct, tx, rx)."""
    tx = rx = 0
    for r in leg_rows:
        if r["flow"] != flow or r["leg"] != leg:
            continue
        if mode is not None and r["cellularMode"] != mode:
            continue
        if any(str(r.get(k)) != str(v) for k, v in filters.items()):
            continue
        tx += int(fnum(r["tx_packets"], 0))
        rx += int(fnum(r["rx_packets"], 0))
    return (100.0 * rx / tx if tx else float("nan")), tx, rx


def collect_switch_stats(campaign: Path, summary: List[dict],
                         timeout_s: float = 5.0) -> Dict[str, object]:
    """Aggregate switch-recovery timing, keeping censored samples separate."""
    measured: Dict[str, List[float]] = {"lte": [], "nr": []}
    censored: Dict[str, List[float]] = {"lte": [], "nr": []}
    restore: Dict[str, List[float]] = {"lte": [], "nr": []}
    no_restore: Dict[str, int] = {"lte": 0, "nr": 0}
    per_run: List[dict] = []
    for r in summary:
        log = campaign / r["run_id"] / "wifi-hybrid-switch_log.csv"
        if not log.exists():
            continue
        st = analyze_switch_log(log, timeout_s)
        mode = r.get("cellularMode", "")
        if mode in measured:
            measured[mode].extend(st.measured_ms)
            censored[mode].extend(st.censored_ms)
            restore[mode].extend(st.restore_delays_s)
            no_restore[mode] += st.never_restored
        per_run.append(
            {
                "run_id": r["run_id"],
                "cellularMode": mode,
                "numStaNodes": r.get("numStaNodes", ""),
                "payload": r.get("payload", ""),
                "total": st.total,
                "measured": len(st.measured_ms),
                "censored": len(st.censored_ms),
                "censored_pct": st.censored_pct,
                "under200_pct": st.under_ms_pct(CONTROL_SYNC_TARGET_MS),
                "continuity_pct": st.continuity_pct(),
                "continuity_total": st.continuity_total,
            }
        )
    all_m = measured["lte"] + measured["nr"]
    all_c = censored["lte"] + censored["nr"]
    return {
        "measured": measured,
        "censored": censored,
        "restore": restore,
        "no_restore": no_restore,
        "per_run": per_run,
        "all_measured": all_m,
        "all_censored": all_c,
        "all_restore": restore["lte"] + restore["nr"],
        "all_no_restore": no_restore["lte"] + no_restore["nr"],
        "n_total": len(all_m) + len(all_c),
    }


def continuity_pct(sw: Dict[str, object], mode: Optional[str] = None,
                   window_s: float = CONTINUITY_WINDOW_S) -> Tuple[float, int]:
    """Share of switch events whose Control flow resumed inside the window."""
    if mode is None:
        delays, missing = sw["all_restore"], sw["all_no_restore"]
    else:
        delays, missing = sw["restore"][mode], sw["no_restore"][mode]
    total = len(delays) + missing
    if not total:
        return float("nan"), 0
    good = sum(1 for d in delays if d <= window_s)
    return 100.0 * good / total, total


def quantile(vals: Sequence[float], pct: float) -> float:
    clean = sorted(v for v in vals if not math.isnan(v))
    if not clean:
        return float("nan")
    return clean[min(len(clean) - 1, int(pct / 100.0 * len(clean)))]


def style_axes(ax, title: str, xlabel: str = "", ylabel: str = ""):
    ax.set_title(title, fontsize=12, fontweight="bold", pad=8)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.3)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save_fig(fig, path: Path):
    fig.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  wrote {path.name}")


# ── Figure generators ──────────────────────────────────────────────────────────

def fig_control_pdr_by_mode(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    data = {}
    for mode in ("lte", "nr"):
        vals = [
            fnum(r["pdr_pct"])
            for r in metrics
            if r["flow"] == "Control" and r["cellularMode"] == mode
        ]
        data[mode] = vals
    bp = ax.boxplot(
        [data["lte"], data["nr"]],
        labels=["LTE", "5G NR"],
        patch_artist=True,
        widths=0.55,
    )
    colors = ["#93c5fd", "#86efac"]
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c)
    style_axes(ax, "Control-flow PDR by cellular mode", ylabel="PDR (%)")
    ax.axhline(100.0, color="#b91c1c", ls="--", lw=1, label="Target 100%")
    ax.legend(loc="lower right", fontsize=9)
    save_fig(fig, out)


def fig_control_p99_by_mode(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    data = []
    for mode in ("lte", "nr"):
        vals = [
            fnum(r["p99_ms"])
            for r in metrics
            if r["flow"] == "Control" and r["cellularMode"] == mode
        ]
        data.append(vals)
    bp = ax.boxplot(data, labels=["LTE", "5G NR"], patch_artist=True, widths=0.55)
    for patch, c in zip(bp["boxes"], ["#fca5a5", "#86efac"]):
        patch.set_facecolor(c)
    ax.axhline(200.0, color="#b91c1c", ls="--", lw=1.2, label="Target ≤ 200 ms")
    style_axes(ax, "Control-flow P99 latency by cellular mode", ylabel="P99 latency (ms)")
    ax.legend(loc="upper right", fontsize=9)
    save_fig(fig, out)


def fig_per_flow_pdr_grouped(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.5, 4.5))
    x = np.arange(len(FLOWS))
    width = 0.35
    for i, mode in enumerate(("lte", "nr")):
        means = []
        errs = []
        for flow in FLOWS:
            vals = [
                fnum(r["pdr_pct"])
                for r in metrics
                if r["flow"] == flow and r["cellularMode"] == mode
            ]
            means.append(mean(vals))
            errs.append(stdev(vals))
        ax.bar(
            x + (i - 0.5) * width,
            means,
            width,
            yerr=errs,
            capsize=3,
            label=mode.upper() if mode == "lte" else "5G NR",
            color="#3b82f6" if mode == "lte" else "#22c55e",
            alpha=0.85,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(FLOWS)
    style_axes(ax, "Per-flow PDR (mean ± std) — LTE vs 5G NR", ylabel="PDR (%)")
    ax.legend()
    ax.set_ylim(0, 105)
    save_fig(fig, out)


def fig_throughput_share(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    shares = {flow: [] for flow in FLOWS}
    for flow in FLOWS:
        shares[flow] = [fnum(r["tput_share_pct"]) for r in metrics if r["flow"] == flow]
    means = [mean(shares[f]) for f in FLOWS]
    colors = ["#ef4444", "#3b82f6", "#22c55e"]
    bars = ax.bar(FLOWS, means, color=colors, alpha=0.85)
    for b, m in zip(bars, means):
        ax.text(b.get_x() + b.get_width() / 2, m + 1, f"{m:.1f}%", ha="center", fontsize=10)
    style_axes(ax, "Average per-flow throughput share (all 192 runs)", ylabel="Share (%)")
    ax.set_ylim(0, max(means) * 1.25 if means else 100)
    save_fig(fig, out)


def fig_control_pdr_vs_sta(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    for mode, color, marker in (("lte", "#3b82f6", "o"), ("nr", "#16a34a", "s")):
        xs, ys, es = [], [], []
        for sta in STA_ORDER:
            vals = [
                fnum(r["pdr_pct"])
                for r in metrics
                if r["flow"] == "Control"
                and r["cellularMode"] == mode
                and int(r["numStaNodes"]) == sta
            ]
            xs.append(sta)
            ys.append(mean(vals))
            es.append(stdev(vals))
        ax.errorbar(
            xs,
            ys,
            yerr=es,
            marker=marker,
            color=color,
            lw=2,
            capsize=4,
            label=mode.upper() if mode == "lte" else "5G NR",
        )
    ax.axhline(100.0, color="#b91c1c", ls="--", lw=1)
    style_axes(
        ax,
        "Control PDR vs STA count (seed-averaged)",
        xlabel="Number of STA robots",
        ylabel="Control PDR (%)",
    )
    ax.legend()
    ax.set_xticks(list(STA_ORDER))
    save_fig(fig, out)


def fig_control_pdr_vs_payload(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    x = np.arange(len(PAYLOAD_ORDER))
    width = 0.35
    for i, mode in enumerate(("lte", "nr")):
        means, errs = [], []
        for payload in PAYLOAD_ORDER:
            vals = [
                fnum(r["pdr_pct"])
                for r in metrics
                if r["flow"] == "Control"
                and r["cellularMode"] == mode
                and r["payload"] == payload
            ]
            means.append(mean(vals))
            errs.append(stdev(vals))
        ax.bar(
            x + (i - 0.5) * width,
            means,
            width,
            yerr=errs,
            capsize=3,
            label=mode.upper() if mode == "lte" else "5G NR",
            color="#60a5fa" if mode == "lte" else "#4ade80",
        )
    ax.set_xticks(x)
    ax.set_xticklabels(PAYLOAD_ORDER)
    style_axes(
        ax,
        "Control PDR vs traffic load (payload / flowScale)",
        xlabel="Payload tag",
        ylabel="Control PDR (%)",
    )
    ax.legend()
    ax.set_ylim(0, 105)
    save_fig(fig, out)


def fig_band_comparison(metrics: List[dict], out: Path):
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.2), sharey=True)
    for ax, mode in zip(axes, ("lte", "nr")):
        data = []
        for band in ("2g", "5g"):
            vals = [
                fnum(r["pdr_pct"])
                for r in metrics
                if r["flow"] == "Control"
                and r["cellularMode"] == mode
                and r["hotspotBand"] == band
            ]
            data.append(vals)
        bp = ax.boxplot(data, labels=["2.4 GHz", "5 GHz"], patch_artist=True, widths=0.55)
        for patch, c in zip(bp["boxes"], ["#fde68a", "#93c5fd"]):
            patch.set_facecolor(c)
        style_axes(
            ax,
            f"{'LTE' if mode == 'lte' else '5G NR'} — Control PDR by WiFi band",
            ylabel="PDR (%)" if mode == "lte" else "",
        )
        ax.set_ylim(0, 105)
    save_fig(fig, out)


def fig_switch_resolved_rate(summary: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    x = np.arange(len(STA_ORDER))
    width = 0.35
    for i, mode in enumerate(("lte", "nr")):
        rates = []
        for sta in STA_ORDER:
            rows = [
                r
                for r in summary
                if r["cellularMode"] == mode and int(r["numStaNodes"]) == sta
            ]
            tot = sum(int(r["switch_events"] or 0) for r in rows)
            res = sum(int(r["resolved"] or 0) for r in rows)
            rates.append(100.0 * res / tot if tot else 0.0)
        ax.bar(
            x + (i - 0.5) * width,
            rates,
            width,
            label=mode.upper() if mode == "lte" else "5G NR",
            color="#3b82f6" if mode == "lte" else "#22c55e",
        )
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in STA_ORDER])
    style_axes(
        ax,
        "Switch recovery resolved rate vs STA count",
        xlabel="Number of STA robots",
        ylabel="Resolved / total switches (%)",
    )
    ax.legend()
    ax.set_ylim(0, 105)
    save_fig(fig, out)


def fig_timeout_heatmap(summary: List[dict], out: Path):
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    for ax, mode in zip(axes, ("lte", "nr")):
        mat = np.zeros((len(STA_ORDER), len(PAYLOAD_ORDER)))
        for i, sta in enumerate(STA_ORDER):
            for j, payload in enumerate(PAYLOAD_ORDER):
                rows = [
                    r
                    for r in summary
                    if r["cellularMode"] == mode
                    and int(r["numStaNodes"]) == sta
                    and r["payload"] == payload
                ]
                tot = sum(int(r["switch_events"] or 0) for r in rows)
                to = sum(int(r["timeout"] or 0) for r in rows)
                mat[i, j] = 100.0 * to / tot if tot else 0.0
        im = ax.imshow(mat, cmap="YlOrRd", aspect="auto", vmin=0, vmax=max(25, mat.max()))
        ax.set_xticks(range(len(PAYLOAD_ORDER)))
        ax.set_xticklabels(PAYLOAD_ORDER)
        ax.set_yticks(range(len(STA_ORDER)))
        ax.set_yticklabels([str(s) for s in STA_ORDER])
        ax.set_xlabel("Payload")
        ax.set_ylabel("STA count")
        ax.set_title(f"{'LTE' if mode == 'lte' else '5G NR'} timeout rate (%)", fontweight="bold")
        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                ax.text(j, i, f"{mat[i, j]:.0f}", ha="center", va="center", fontsize=8)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    save_fig(fig, out)


def fig_switch_volume(summary: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    for mode, color, marker in (("lte", "#3b82f6", "o"), ("nr", "#16a34a", "s")):
        xs, ys = [], []
        for sta in STA_ORDER:
            rows = [
                r
                for r in summary
                if r["cellularMode"] == mode and int(r["numStaNodes"]) == sta
            ]
            # average switches per run
            vals = [int(r["switch_events"] or 0) for r in rows]
            xs.append(sta)
            ys.append(mean(vals))
        ax.plot(xs, ys, marker=marker, color=color, lw=2, label=mode.upper() if mode == "lte" else "5G NR")
    style_axes(
        ax,
        "Average switch events per run vs STA count",
        xlabel="Number of STA robots",
        ylabel="Switch events / run",
    )
    ax.legend()
    ax.set_xticks(list(STA_ORDER))
    save_fig(fig, out)


def fig_seed_variation(metrics: List[dict], out: Path):
    fig, ax = plt.subplots(figsize=(8.5, 4.5))
    seeds = sorted({int(r["rngSeed"]) for r in metrics})
    x = np.arange(len(seeds))
    width = 0.25
    for i, flow in enumerate(FLOWS):
        means = []
        for seed in seeds:
            vals = [
                fnum(r["pdr_pct"])
                for r in metrics
                if r["flow"] == flow and int(r["rngSeed"]) == seed
            ]
            means.append(mean(vals))
        ax.bar(
            x + (i - 1) * width,
            means,
            width,
            label=flow,
            color=["#ef4444", "#3b82f6", "#22c55e"][i],
            alpha=0.85,
        )
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in seeds])
    style_axes(ax, "Per-flow PDR by RNG seed (campaign-wide)", xlabel="Seed", ylabel="PDR (%)")
    ax.legend()
    ax.set_ylim(0, 105)
    save_fig(fig, out)


def fig_control_pass_rate(metrics: List[dict], out: Path):
    """Share of runs meeting Control QoS targets."""
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    labels = []
    rates = []
    for mode in ("lte", "nr"):
        rows = [r for r in metrics if r["flow"] == "Control" and r["cellularMode"] == mode]
        ok = 0
        for r in rows:
            p99 = fnum(r["p99_ms"])
            loss = fnum(r["loss_pct"])
            # allow tiny float noise on loss
            if (not math.isnan(p99) and p99 <= CONTROL_SYNC_TARGET_MS) and (
                not math.isnan(loss) and loss < 0.05
            ):
                ok += 1
        labels.append(mode.upper() if mode == "lte" else "5G NR")
        rates.append(100.0 * ok / len(rows) if rows else 0.0)
    bars = ax.bar(labels, rates, color=["#60a5fa", "#4ade80"], alpha=0.9)
    for b, r in zip(bars, rates):
        ax.text(b.get_x() + b.get_width() / 2, r + 1, f"{r:.1f}%", ha="center", fontsize=11)
    style_axes(
        ax,
        "Runs meeting Control targets (P99 ≤ 200 ms and ~0% loss)",
        ylabel="Pass rate (%)",
    )
    ax.set_ylim(0, 105)
    save_fig(fig, out)


def fig_control_pdr_by_leg(leg_rows: List[dict], out: Path):
    """Headline correction: the blended PDR hides which leg is failing."""
    fig, ax = plt.subplots(figsize=(8.4, 4.6))
    x = np.arange(2)
    width = 0.26
    series = [
        (LEG_WIFI, "WiFi mesh leg (primary)", "#f59e0b"),
        (LEG_CELL, "Cellular leg (fallback)", "#22c55e"),
    ]
    for i, (leg, label, color) in enumerate(series):
        vals = [leg_pdr(leg_rows, "Control", leg, mode)[0] for mode in ("lte", "nr")]
        bars = ax.bar(x + (i - 1) * width, vals, width, label=label, color=color, alpha=0.9)
        for b, v in zip(bars, vals):
            if not math.isnan(v):
                ax.text(b.get_x() + b.get_width() / 2, v + 1.2, f"{v:.1f}",
                        ha="center", fontsize=9)
    blended = []
    for mode in ("lte", "nr"):
        _, twx, rwx = leg_pdr(leg_rows, "Control", LEG_WIFI, mode)
        _, tcx, rcx = leg_pdr(leg_rows, "Control", LEG_CELL, mode)
        blended.append(100.0 * (rwx + rcx) / (twx + tcx) if (twx + tcx) else float("nan"))
    bars = ax.bar(x + width, blended, width, label="Blended (as previously reported)",
                  color="#94a3b8", alpha=0.9, hatch="//")
    for b, v in zip(bars, blended):
        if not math.isnan(v):
            ax.text(b.get_x() + b.get_width() / 2, v + 1.2, f"{v:.1f}",
                    ha="center", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(["LTE", "5G NR"])
    ax.axhline(100.0, color="#b91c1c", ls="--", lw=1)
    style_axes(ax, "Control PDR split by network path leg (packet-weighted, 192 runs)",
               ylabel="Control PDR (%)")
    ax.set_ylim(0, 112)
    ax.legend(loc="lower left", fontsize=8.5)
    save_fig(fig, out)


def fig_interruption_ecdf(sw: Dict[str, object], out: Path):
    """Distribution of measured interruptions, with the censored share stated."""
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.0, 4.4),
                                  gridspec_kw={"width_ratios": [2.1, 1]})
    for mode, color, label in (("lte", "#3b82f6", "LTE"), ("nr", "#16a34a", "5G NR")):
        vals = sorted(v for v in sw["measured"][mode] if not math.isnan(v))
        if not vals:
            continue
        y = np.arange(1, len(vals) + 1) / len(vals) * 100.0
        ax.plot(vals, y, color=color, lw=2, label=f"{label} (n={len(vals)})")
    ax.axvline(200.0, color="#b91c1c", ls="--", lw=1.2, label="200 ms target")
    ax.set_xscale("log")
    style_axes(ax, "Measured switch interruption (censored events excluded)",
               xlabel="Service interruption (ms, log scale)",
               ylabel="Cumulative share of measured events (%)")
    ax.legend(loc="lower right", fontsize=9)
    ax.set_ylim(0, 102)

    n_m, n_c = len(sw["all_measured"]), len(sw["all_censored"])
    tot = n_m + n_c or 1
    fast = sum(1 for v in sw["all_measured"] if v <= 200.0)
    slow = n_m - fast
    parts = [100.0 * fast / tot, 100.0 * slow / tot, 100.0 * n_c / tot]
    labels = ["Measured\n≤ 200 ms", "Measured\n> 200 ms", "Censored at\nwait ceiling"]
    bars = ax2.bar(labels, parts, color=["#22c55e", "#f59e0b", "#94a3b8"], alpha=0.9)
    for b, v in zip(bars, parts):
        ax2.text(b.get_x() + b.get_width() / 2, v + 1.2, f"{v:.1f}%", ha="center", fontsize=9)
    style_axes(ax2, f"Composition of all {tot} switch events", ylabel="Share (%)")
    ax2.set_ylim(0, max(parts) * 1.3)
    ax2.tick_params(axis="x", labelsize=8.5)
    save_fig(fig, out)


def fig_control_continuity(sw: Dict[str, object], out: Path):
    """Plan section 3.3 continuity check, plus the restore-delay distribution."""
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.0, 4.4),
                                  gridspec_kw={"width_ratios": [1, 1.5]})
    labels, vals = [], []
    for mode in ("lte", "nr"):
        pct, n = continuity_pct(sw, mode)
        labels.append(f"{'LTE' if mode == 'lte' else '5G NR'}\n(n={n})")
        vals.append(pct)
    pct_all, n_all = continuity_pct(sw)
    labels.append(f"ALL\n(n={n_all})")
    vals.append(pct_all)
    bars = ax.bar(labels, vals, color=["#3b82f6", "#22c55e", "#64748b"], alpha=0.9)
    for b, v in zip(bars, vals):
        if not math.isnan(v):
            ax.text(b.get_x() + b.get_width() / 2, v + 1.2, f"{v:.1f}%", ha="center", fontsize=10)
    style_axes(ax, f"Control flow restored within ±{CONTINUITY_WINDOW_S:.0f} s of a switch",
               ylabel="Share of switch events (%)")
    ax.set_ylim(0, 108)
    ax.tick_params(axis="x", labelsize=9)

    for mode, color, label in (("lte", "#3b82f6", "LTE"), ("nr", "#16a34a", "5G NR")):
        vals_ms = sorted(d * 1000.0 for d in sw["restore"][mode] if d > 0)
        if not vals_ms:
            continue
        y = np.arange(1, len(vals_ms) + 1) / len(vals_ms) * 100.0
        ax2.plot(vals_ms, y, color=color, lw=2, label=f"{label} (n={len(vals_ms)})")
    ax2.axvline(CONTINUITY_WINDOW_S * 1000.0, color="#b91c1c", ls="--", lw=1.2,
                label=f"±{CONTINUITY_WINDOW_S:.0f} s continuity window")
    ax2.axvline(CONTROL_SYNC_TARGET_MS, color="#7c3aed", ls=":", lw=1.2,
                label=f"{CONTROL_SYNC_TARGET_MS:.0f} ms sync target")
    ax2.set_xscale("log")
    style_axes(ax2, "Delay from switch trigger to first Control packet on the new path",
               xlabel="Restore delay (ms, log scale)",
               ylabel="Cumulative share of restored events (%)")
    ax2.legend(loc="upper left", fontsize=8.5)
    ax2.set_ylim(0, 102)
    save_fig(fig, out)


def fig_leg_pdr_vs_sta(leg_rows: List[dict], out: Path):
    """Primary vs fallback leg as the robot count grows."""
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.3), sharey=True)
    for ax, mode in zip(axes, ("lte", "nr")):
        for leg, color, marker, label in (
            (LEG_WIFI, "#f59e0b", "o", "WiFi mesh leg"),
            (LEG_CELL, "#22c55e", "s", "Cellular leg"),
        ):
            ys = [leg_pdr(leg_rows, "Control", leg, mode, numStaNodes=sta)[0]
                  for sta in STA_ORDER]
            ax.plot(STA_ORDER, ys, marker=marker, color=color, lw=2, label=label)
        ax.axhline(100.0, color="#b91c1c", ls="--", lw=1)
        style_axes(ax, f"{'LTE' if mode == 'lte' else '5G NR'} — Control PDR per leg",
                   xlabel="Number of STA robots",
                   ylabel="Control PDR (%)" if mode == "lte" else "")
        ax.set_xticks(list(STA_ORDER))
        ax.set_ylim(0, 105)
        ax.legend(fontsize=9, loc="lower left")
    save_fig(fig, out)


def fig_dark_flow_rate(leg_rows: List[dict], out: Path):
    """How often the application was steered onto an already-dead path."""
    fig, ax = plt.subplots(figsize=(8.2, 4.4))
    x = np.arange(len(STA_ORDER))
    width = 0.35
    for i, mode in enumerate(("lte", "nr")):
        rates = []
        for sta in STA_ORDER:
            rows = [r for r in leg_rows
                    if r["flow"] == "Control" and r["leg"] == LEG_WIFI
                    and r["cellularMode"] == mode and str(r["numStaNodes"]) == str(sta)]
            sub = sum(int(fnum(r["sub_flows"], 0)) for r in rows)
            dark = sum(int(fnum(r["dark_sub_flows"], 0)) for r in rows)
            rates.append(100.0 * dark / sub if sub else 0.0)
        bars = ax.bar(x + (i - 0.5) * width, rates, width,
                      label="LTE" if mode == "lte" else "5G NR",
                      color="#3b82f6" if mode == "lte" else "#22c55e", alpha=0.88)
        for b, v in zip(bars, rates):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.8, f"{v:.0f}", ha="center", fontsize=8.5)
    ax.set_xticks(x)
    ax.set_xticklabels([str(s) for s in STA_ORDER])
    style_axes(
        ax,
        "Control streams left on a dead WiFi path (> 3 s transmitting with no receive)",
        xlabel="Number of STA robots",
        ylabel="Share of WiFi-leg sub-flows (%)",
    )
    ax.legend(fontsize=9)
    save_fig(fig, out)


# ── Aggregation helpers for markdown tables ───────────────────────────────────

def summarize_flow(metrics: List[dict], flow: str, mode: Optional[str] = None) -> dict:
    rows = [r for r in metrics if r["flow"] == flow and (mode is None or r["cellularMode"] == mode)]
    return {
        "n": len(rows),
        "pdr": mean([fnum(r["pdr_pct"]) for r in rows]),
        "pdr_std": stdev([fnum(r["pdr_pct"]) for r in rows]),
        "loss": mean([fnum(r["loss_pct"]) for r in rows]),
        "delay": mean([fnum(r["mean_delay_ms"]) for r in rows]),
        "p99": mean([fnum(r["p99_ms"]) for r in rows]),
        "tput": mean([fnum(r["throughput_mbps"]) for r in rows]),
        "share": mean([fnum(r["tput_share_pct"]) for r in rows]),
    }


def fmt(x: float, nd: int = 2) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "n/a"
    return f"{x:.{nd}f}"


def build_markdown(
    campaign_dir: Path,
    summary: List[dict],
    metrics: List[dict],
    leg_rows: List[dict],
    sw: Dict[str, object],
    fig_names: List[str],
) -> str:
    n_ok = sum(1 for r in summary if r.get("status") == "ok")
    tot_sw = sum(int(r.get("switch_events") or 0) for r in summary)
    tot_res = sum(int(r.get("resolved") or 0) for r in summary)
    tot_to = sum(int(r.get("timeout") or 0) for r in summary)
    tot_sup = sum(int(r.get("superseded") or 0) for r in summary)

    n_meas = len(sw["all_measured"])
    n_cens = len(sw["all_censored"])
    n_sw_samples = n_meas + n_cens or 1
    fast_pct = 100.0 * sum(1 for v in sw["all_measured"] if v <= 200.0) / n_sw_samples
    med_meas = quantile(sw["all_measured"], 50.0)
    p90_meas = quantile(sw["all_measured"], 90.0)
    wifi_pdr = {m: leg_pdr(leg_rows, "Control", LEG_WIFI, m)[0] for m in ("lte", "nr")}
    cell_pdr = {m: leg_pdr(leg_rows, "Control", LEG_CELL, m)[0] for m in ("lte", "nr")}

    lines: List[str] = []
    lines.append("# Traffic QoS Analysis Report")
    lines.append("")
    lines.append(f"- **Author:** {AUTHOR}")
    lines.append(f"- **Lab:** {LAB}")
    lines.append(f"- **Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    lines.append(f"- **Campaign:** `{campaign_dir}`")
    lines.append(
        "- **Matrix:** cellularMode × hotspotBand × STA × payload × seed "
        "= 2 × 2 × 4 × 4 × 3 = **192 runs** (seeds 7, 8, 9)"
    )
    lines.append(f"- **Completed:** {n_ok} / {len(summary)} successful")
    lines.append("")
    lines.append("## 1. Executive summary")
    lines.append("")
    lines.append(
        "This report evaluates the July QoS-separated robot traffic model "
        "(Control / Sensor / Video with DSCP marking) on the hybrid WiFi Mesh + "
        "LTE / 5G NR simulator. Metrics are aggregated across the full 192-run factorial campaign."
    )
    lines.append("")
    lines.append(
        "The headline result is that the **cellular fallback leg meets the Control-flow "
        f"requirement while the WiFi mesh primary leg does not**: Control PDR is "
        f"{fmt(cell_pdr['lte'], 1)}% over LTE and {fmt(cell_pdr['nr'], 1)}% over 5G NR on the "
        f"fallback leg, against {fmt(wifi_pdr['lte'], 1)}% / {fmt(wifi_pdr['nr'], 1)}% on the "
        "WiFi mesh leg. Earlier revisions of this report quoted a single blended figure that "
        "averaged the two legs together and therefore attributed the mesh weakness to the "
        "hybrid design as a whole. Section 2 explains the correction."
    )
    lines.append("")
    lines.append(
        f"Switch recovery is likewise better than a blended average suggests: "
        f"**{fast_pct:.1f}% of all {n_sw_samples} switch events restored service within "
        f"200 ms**, with a median measured interruption of {fmt(med_meas, 0)} ms. A further "
        f"{100.0 * n_cens / n_sw_samples:.1f}% never showed service resuming before the "
        "scenario stopped waiting; those are reported as censored rather than folded into "
        "the mean."
    )
    lines.append("")
    _cp, _cn = continuity_pct(sw)
    lines.append(
        f"On the §3.3 continuity criterion the result is stronger still: the Control flow "
        f"resumed within ±{CONTINUITY_WINDOW_S:.0f} s of the switch trigger in "
        f"**{fmt(_cp, 1)}%** of {_cn} events. Switching transients are therefore not what "
        "costs the Control flow its packets; sustained primary-path outages between "
        "switches are (Sections 4 and 7.1). Section 9 audits this report against each "
        "§3 requirement, including two implementation deviations."
    )
    lines.append("")
    lines.append("| Aggregate KPI | Value |")
    lines.append("|---|---|")
    lines.append(f"| Successful runs | {n_ok} / {len(summary)} |")
    lines.append(f"| Total switch events | {tot_sw} |")
    lines.append(
        f"| Resolved / timeout / superseded | "
        f"{tot_res} ({100*tot_res/tot_sw:.1f}%) / "
        f"{tot_to} ({100*tot_to/tot_sw:.1f}%) / "
        f"{tot_sup} ({100*tot_sup/tot_sw:.1f}%) |"
        if tot_sw
        else "| Resolved / timeout / superseded | n/a |"
    )
    lines.append(
        f"| Switch events recovering within 200 ms | {fast_pct:.1f}% "
        f"({n_meas} measured, {n_cens} censored) |"
    )
    lines.append(
        f"| Median / P90 measured interruption | {fmt(med_meas, 0)} ms / {fmt(p90_meas, 0)} ms |"
    )
    cont_pct, cont_n = continuity_pct(sw)
    lines.append(
        f"| Control flow restored within ±{CONTINUITY_WINDOW_S:.0f} s of a switch (§3.3) | "
        f"{fmt(cont_pct, 1)}% of {cont_n} events |"
    )
    lines.append(
        f"| Control PDR — cellular leg | LTE {fmt(cell_pdr['lte'], 1)}% · "
        f"NR {fmt(cell_pdr['nr'], 1)}% |"
    )
    lines.append(
        f"| Control PDR — WiFi mesh leg | LTE {fmt(wifi_pdr['lte'], 1)}% · "
        f"NR {fmt(wifi_pdr['nr'], 1)}% |"
    )
    for flow in FLOWS:
        s = summarize_flow(metrics, flow)
        lines.append(
            f"| {flow} mean PDR / P99 (blended) | {fmt(s['pdr'])}% / {fmt(s['p99'])} ms |"
        )
    lines.append("")

    lines.append("## 2. How the metrics are separated")
    lines.append("")
    lines.append("### 2.1 Path legs")
    lines.append("")
    lines.append(
        "A path switch moves the STA to a different egress interface, so its source "
        "address changes from the WiFi hotspot subnet (192.168.x.x) to the cellular "
        "bearer (7.x.x.x). FlowMonitor keys statistics on the 5-tuple, so a single "
        "application stream is recorded as two separate flows — one per leg. Summing "
        "them produces a packet-weighted average of a healthy leg and a failing one, "
        "which is why the blended Control PDR looks poor even where the fallback works."
    )
    lines.append("")
    lines.append(
        "The split is only meaningful for the **Control** flow. A connectionless UDP "
        "socket re-resolves its source address per datagram, so it follows the route. "
        "The Sensor and Video TCP sockets pin their source address when the connection "
        "is established and keep it for the whole run: across all 192 runs, "
        "**0 of their sub-flows appear on the cellular leg**. Their high PDR therefore "
        "reflects TCP retransmission masking an outage as reduced throughput, not a "
        "healthy path, and they should not be read as evidence that the mesh leg is fine."
    )
    lines.append("")
    lines.append("### 2.2 Censored switch interruptions")
    lines.append("")
    lines.append(
        "The scenario stops waiting for service to resume after `switchTimeoutSec` "
        "(5 s by default) and records that ceiling as the event's interruption. Those "
        f"values are right-censored lower bounds, not observations. {n_cens} of "
        f"{n_sw_samples} interruption samples ({100.0 * n_cens / n_sw_samples:.1f}%) sit at "
        "the ceiling, including some events the scenario labelled `resolved`. Mixing them "
        "into a mean corrupts it in both directions, so all interruption quantiles in this "
        "report are computed over measured samples only, and the censored share is always "
        "stated alongside."
    )
    lines.append("")

    lines.append("## 3. Per-flow QoS summary (all runs)")
    lines.append("")
    lines.append(
        "| Flow | Mode | N | PDR (%) | Loss (%) | Mean delay (ms) | P99 (ms) | "
        "Throughput (Mbps) | Tput share (%) | Flow target (§3.1) | Sync target (§3.3) |"
    )
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for flow in FLOWS:
        for mode in ("lte", "nr", None):
            s = summarize_flow(metrics, flow, mode)
            mode_lbl = "ALL" if mode is None else (mode.upper() if mode == "lte" else "NR")
            t = QOS_TARGETS[flow]
            lines.append(
                f"| {flow} | {mode_lbl} | {s['n']} | {fmt(s['pdr'])} ± {fmt(s['pdr_std'])} | "
                f"{fmt(s['loss'])} | {fmt(s['delay'])} | {fmt(s['p99'])} | "
                f"{fmt(s['tput'], 3)} | {fmt(s['share'], 1)} | "
                f"≤ {t['flow_ms']:.0f} ms | ≤ {t['sync_ms']:.0f} ms |"
            )
    lines.append("")
    lines.append(
        "The plan gives two latency figures for the Control flow, in different roles. "
        "§3.1 sets the per-flow requirement at **≤ 50 ms (strict)** with 0% loss "
        "tolerance, while §3.3 describes Control PDR as *\"the key metric directly tied "
        "to the 200 ms target\"* — the project end-to-end sync budget. Both are shown so "
        "neither reading is hidden; Control P99 currently misses both."
    )
    lines.append("")
    lines.append(
        "PDR values in this table blend both path legs (see Section 2.1); the leg-separated "
        "values are in Section 4."
    )
    lines.append("")

    lines.append("## 4. Per-flow QoS by network path leg")
    lines.append("")
    lines.append(
        "Packet-weighted over all 192 runs. *Dark sub-flows* kept transmitting for more "
        "than 3 s after their last successful receive, i.e. the application was steered "
        "onto a path that had already stopped delivering."
    )
    lines.append("")
    lines.append(
        "| Path leg | Flow | Mode | Sub-flows | Tx packets | PDR (%) | Dark sub-flows (%) |"
    )
    lines.append("|---|---|---|---|---|---|---|")
    for leg in LEGS:
        for flow in FLOWS:
            for mode in ("lte", "nr"):
                pdr, tx, _rx = leg_pdr(leg_rows, flow, leg, mode)
                if not tx:
                    continue
                rows = [r for r in leg_rows if r["flow"] == flow and r["leg"] == leg
                        and r["cellularMode"] == mode]
                sub = sum(int(fnum(r["sub_flows"], 0)) for r in rows)
                dark = sum(int(fnum(r["dark_sub_flows"], 0)) for r in rows)
                lines.append(
                    f"| {leg} | {flow} | {'LTE' if mode == 'lte' else 'NR'} | {sub} | "
                    f"{tx} | {fmt(pdr, 2)} | {fmt(100.0 * dark / sub if sub else 0.0, 1)} |"
                )
    lines.append("")
    lines.append(
        f"On LTE the cellular fallback leg carries the Control flow at "
        f"{fmt(cell_pdr['lte'], 1)}% — close to the quality the deliverable asks for. The "
        "WiFi mesh leg does not, and the dark sub-flow column shows the mechanism: roughly "
        "half of all Control streams were left transmitting into a primary path that had "
        "already stopped delivering. That is a mesh-side association / route-maintenance "
        "limitation, and it is the subject of the August enhancement item (AP coverage "
        "boundaries, Intra-Mesh HO event logging, Guard Timer)."
    )
    lines.append("")
    lines.append(
        f"The NR fallback leg reaches only {fmt(cell_pdr['nr'], 1)}%, with a dark sub-flow "
        "rate an order of magnitude above LTE's. This is a separate effect from the mesh "
        "issue above and is not diagnosed in this revision; the 3.5 GHz NR carrier suffers "
        "far more building penetration loss than LTE's 2.0 GHz under the shared "
        "`HybridBuildingsPropagationLossModel`, which is the first hypothesis to test."
    )
    lines.append("")

    lines.append("## 5. Figures")
    lines.append("")
    captions = {
        "fig01_control_pdr_by_mode.png": "Control PDR distribution for LTE vs 5G NR.",
        "fig02_control_p99_by_mode.png": "Control P99 latency vs the 200 ms sync target.",
        "fig03_per_flow_pdr_grouped.png": "Mean ± std PDR for Control, Sensor, and Video.",
        "fig04_throughput_share.png": "Average bandwidth share across the three flows.",
        "fig05_control_pdr_vs_sta.png": "Scalability: Control PDR as STA count increases.",
        "fig06_control_pdr_vs_payload.png": "Load sensitivity: Control PDR vs payload / flowScale.",
        "fig07_band_comparison.png": "2.4 GHz vs 5 GHz hotspot band effect on Control PDR.",
        "fig08_switch_resolved_rate.png": "Switch recovery confirmation rate vs STA count.",
        "fig09_timeout_heatmap.png": "Timeout-rate heatmap over STA × payload for each mode.",
        "fig10_switch_volume.png": "Average number of path switches per run.",
        "fig11_seed_variation.png": "Seed-to-seed variation of per-flow PDR.",
        "fig12_control_pass_rate.png": "Fraction of runs meeting Control QoS targets, "
        "judged on the blended PDR and therefore a lower bound.",
        "fig13_control_pdr_by_leg.png": "Control PDR separated into the WiFi mesh primary "
        "leg and the cellular fallback leg, with the previously reported blended value "
        "shown for comparison. The fallback leg meets the requirement; the primary does not.",
        "fig14_interruption_ecdf.png": "Distribution of measured switch interruptions "
        "against the 200 ms target, plus the composition of all switch events into "
        "fast, slow, and censored groups.",
        "fig15_leg_pdr_vs_sta.png": "Per-leg Control PDR as the robot count grows, showing "
        "that the primary-leg deficit is not simply a congestion effect.",
        "fig16_dark_flow_rate.png": "Share of Control streams left transmitting into an "
        "already-dead WiFi path — the mechanism behind the primary-leg loss.",
        "fig17_control_continuity.png": "Plan §3.3 continuity check: the share of switch "
        "events whose Control flow resumed within ±1 s, and the full distribution of "
        "restore delays against the 200 ms sync target.",
    }
    for idx, name in enumerate(fig_names, start=1):
        lines.append(f"### Figure {idx}")
        lines.append("")
        lines.append(f"![{name}](figures/{name})")
        lines.append("")
        lines.append(f"**Figure {idx}.** {captions.get(name, name)}")
        lines.append("")

    lines.append("## 6. Control-flow robot-safety KPIs")
    lines.append("")
    for mode in ("lte", "nr"):
        s = summarize_flow(metrics, "Control", mode)
        label = "LTE" if mode == "lte" else "5G NR"
        lines.append(f"### {label}")
        lines.append(
            f"- Control PDR — cellular fallback leg: **{fmt(cell_pdr[mode], 2)}%** (target 100%)"
        )
        lines.append(
            f"- Control PDR — WiFi mesh primary leg: **{fmt(wifi_pdr[mode], 2)}%**"
        )
        lines.append(f"- Control PDR — blended across both legs: {fmt(s['pdr'])}%")
        lines.append(f"- Control P99 latency: **{fmt(s['p99'])} ms** (target ≤ 200 ms)")
        lines.append(f"- Control mean delay: {fmt(s['delay'])} ms")
        lines.append(f"- Control throughput: {fmt(s['tput'], 3)} Mbps")
        lines.append("")

    lines.append("## 7. Switching reliability")
    lines.append("")
    lines.append(
        "The scenario's own status labels are shown first, then the interruption timing "
        "with censored samples held out. Note that the `resolved` label is not equivalent "
        "to a fast recovery: some resolved events carry an interruption at the wait ceiling."
    )
    lines.append("")
    lines.append("| Mode | Switch events | Resolved | Timeout | Superseded | Resolved % |")
    lines.append("|---|---|---|---|---|---|")
    for mode in ("lte", "nr", None):
        rows = summary if mode is None else [r for r in summary if r["cellularMode"] == mode]
        n_sw = sum(int(r.get("switch_events") or 0) for r in rows)
        res = sum(int(r.get("resolved") or 0) for r in rows)
        to = sum(int(r.get("timeout") or 0) for r in rows)
        sup = sum(int(r.get("superseded") or 0) for r in rows)
        lbl = "ALL" if mode is None else (mode.upper() if mode == "lte" else "NR")
        pct = f"{100*res/n_sw:.1f}" if n_sw else "n/a"
        lines.append(f"| {lbl} | {n_sw} | {res} | {to} | {sup} | {pct} |")
    lines.append("")
    lines.append(
        "| Mode | Samples | Censored | Median (ms) | P90 (ms) | P99 (ms) | ≤ 200 ms |"
    )
    lines.append("|---|---|---|---|---|---|---|")
    for mode in ("lte", "nr", None):
        if mode is None:
            meas, cens, lbl = sw["all_measured"], sw["all_censored"], "ALL"
        else:
            meas = sw["measured"][mode]
            cens = sw["censored"][mode]
            lbl = "LTE" if mode == "lte" else "NR"
        n = len(meas) + len(cens) or 1
        fast = sum(1 for v in meas if v <= CONTROL_SYNC_TARGET_MS)
        lines.append(
            f"| {lbl} | {n} | {len(cens)} ({100.0 * len(cens) / n:.1f}%) | "
            f"{fmt(quantile(meas, 50.0), 0)} | {fmt(quantile(meas, 90.0), 0)} | "
            f"{fmt(quantile(meas, 99.0), 0)} | {100.0 * fast / n:.1f}% |"
        )
    lines.append("")

    lines.append(
        f"### 7.1 Control-flow continuity across switching (±{CONTINUITY_WINDOW_S:.0f} s)"
    )
    lines.append("")
    lines.append(
        "Plan §3.3 asks for confirmation of *\"continuity of the control flow within "
        "±1 s of a switching event\"*. This is measured per event as the delay from the "
        "switch trigger to the first Control packet received on the new path. Events that "
        "never showed a receive count as continuity failures, since from the robot's point "
        "of view the control channel did not return."
    )
    lines.append("")
    lines.append(
        f"| Mode | Events assessed | Restored ≤ {CONTINUITY_WINDOW_S:.0f} s | "
        "Median (ms) | P90 (ms) | P99 (ms) | Never restored |"
    )
    lines.append("|---|---|---|---|---|---|---|")
    for mode in ("lte", "nr", None):
        pct, n = continuity_pct(sw, mode)
        delays_ms = [d * 1000.0 for d in
                     (sw["all_restore"] if mode is None else sw["restore"][mode])]
        missing = sw["all_no_restore"] if mode is None else sw["no_restore"][mode]
        lbl = "ALL" if mode is None else ("LTE" if mode == "lte" else "NR")
        lines.append(
            f"| {lbl} | {n} | **{fmt(pct, 1)}%** | {fmt(quantile(delays_ms, 50.0), 0)} | "
            f"{fmt(quantile(delays_ms, 90.0), 0)} | {fmt(quantile(delays_ms, 99.0), 0)} | "
            f"{missing} ({100.0 * missing / n if n else 0.0:.1f}%) |"
        )
    lines.append("")
    cont_all, cont_n = continuity_pct(sw)
    med_restore = quantile([d * 1000.0 for d in sw["all_restore"]], 50.0)
    lines.append(
        f"Across {cont_n} switch events the Control flow resumed within "
        f"±{CONTINUITY_WINDOW_S:.0f} s in **{fmt(cont_all, 1)}%** of cases, median restore "
        f"delay {fmt(med_restore, 0)} ms. This is the §3.3 continuity metric, and it "
        "reinforces the Section 4 finding: switching transients are not what costs the "
        "Control flow its packets. When a switch occurs the control channel comes back "
        "quickly; the primary-leg PDR deficit comes from sustained outages between switches."
    )
    lines.append("")

    lines.append("## 8. Conclusions")
    lines.append("")
    c_lte = summarize_flow(metrics, "Control", "lte")
    c_nr = summarize_flow(metrics, "Control", "nr")
    lines.append(
        f"1. The switching mechanism itself functions: once a robot is moved onto the "
        f"fallback, its control channel is carried at {fmt(cell_pdr['lte'], 1)}% PDR over "
        f"LTE. The equivalent NR figure is {fmt(cell_pdr['nr'], 1)}%, which is a separate "
        "radio-side question rather than a switching-logic failure (Section 4)."
    )
    lines.append(
        f"2. The WiFi mesh primary leg is the limiting factor at "
        f"{fmt(wifi_pdr['lte'], 1)}% / {fmt(wifi_pdr['nr'], 1)}% Control PDR. The dark "
        "sub-flow statistics in Section 4 show the mechanism: streams were left "
        "transmitting into a primary path that had already stopped delivering, so the loss "
        "is concentrated in sustained outages rather than spread across switching "
        "transients. Mesh association and route maintenance are the August enhancement item."
    )
    lines.append(
        f"3. Recovery is fast in the majority of cases: {fast_pct:.1f}% of all "
        f"{n_sw_samples} switch events restored service within 200 ms, median measured "
        f"interruption {fmt(med_meas, 0)} ms. The scenario's own labels report "
        f"{(100*tot_res/tot_sw):.1f}% resolved and {(100*tot_to/tot_sw):.1f}% timeout, but "
        f"{100.0 * n_cens / n_sw_samples:.1f}% of interruption samples are censored at the "
        "wait ceiling and are excluded from the quantiles above."
        if tot_sw
        else "3. Switching statistics were not available."
    )
    lines.append(
        "4. The Sensor and Video TCP flows cannot be used to judge path health. Their "
        "sockets never migrate to the cellular leg, and TCP retransmission converts an "
        "outage into reduced throughput rather than recorded loss, so their ~97% PDR "
        "overstates primary-path availability."
    )
    lines.append(
        "5. STA count and payload/load affect both Control PDR and switch timeout rate — "
        "single-seed anecdotes are insufficient; the factorial matrix is required."
    )
    lines.append(
        "6. Figures above support the July deliverable `traffic_qos_report.pdf` "
        "(Control PDR, P99 latency, and per-flow bandwidth share)."
    )
    lines.append("")
    lines.append("### Limitations of this revision")
    lines.append("")
    lines.append(
        "This revision changes only how the existing campaign data is aggregated and "
        "presented; no simulation was re-run. Three known scenario-side issues remain in "
        "the underlying data and bound how good the primary-leg numbers can be:"
    )
    lines.append("")
    lines.append(
        "- The switching controller's PDR window is fed by the Sensor TCP flow. When a "
        "path breaks, TCP stops transmitting, the window sees zero packets, and a zero-"
        "traffic window is scored as a perfect link — so a broken primary path can look "
        "healthy to the controller."
    )
    lines.append(
        "- Return-to-WiFi is gated on RSSI recovery, which carries no information about "
        "whether the mesh backhaul behind that AP still routes."
    )
    lines.append(
        "- The 5 s wait ceiling censors a substantial share of interruption samples, so "
        "the upper tail of the recovery distribution is not observable from this campaign."
    )
    lines.append("")

    lines.append("## 9. Compliance with the enhancement plan (§3, July item)")
    lines.append("")
    lines.append("### 9.1 Implementation requirements (§3.2)")
    lines.append("")
    lines.append("| Plan requirement | Status | Notes |")
    lines.append("|---|---|---|")
    lines.append(
        "| Control commands as small UDP packets, 50 ms period | Met | 1024 B at "
        "164 kbps = exactly 1 KB per 50 ms. Implemented with `OnOffHelper` + "
        "`UdpServerHelper` rather than the `UdpClient` the plan names; functionally "
        "equivalent. |"
    )
    lines.append(
        "| Sensor data as TCP periodic upload, 100 KB / 200 ms | **Deviation** | Runs as a "
        "continuous TCP stream (`OnTime=1.0`, `OffTime=0.0`) at 4 Mbps × `flowScale`. The "
        "100 KB chunk constant is declared but unused, so the 200 ms burst structure is "
        "absent. Offered load is equivalent at `flowScale=1.0`; the burstiness is not "
        "reproduced. |"
    )
    lines.append(
        "| Video kept on TCP with separate flow IDs | Met | Port range 54000+ yields "
        "independent FlowMonitor flow IDs per STA. |"
    )
    lines.append(
        "| Per-flow DSCP marking | Met | EF (Control), AF31 (Sensor), AF41 (Video) applied "
        "via the application `Tos` attribute and recovered by destination-port range. |"
    )
    lines.append("")
    lines.append(
        "One further deviation: `flowScale` multiplies the Sensor and Video rates, so at "
        "the `10kb` and `50kb` payload settings Video runs at 0.5–1.25 Mbps, below the "
        "1–10 Mbps band stated in §3.1. This affects 96 of the 192 runs."
    )
    lines.append("")
    lines.append("### 9.2 Extended measurement metrics (§3.3)")
    lines.append("")
    lines.append("| Required metric | Status | Where |")
    lines.append("|---|---|---|")
    lines.append(
        "| Control PDR before/after switching | Partial | Sections 3 and 4. Whole-run PDR "
        "plus a per-path-leg split (WiFi primary vs cellular fallback), which stands in for "
        "the pre/post-switch states. A true per-event before/after comparison needs "
        "per-packet Control receive logging, which this campaign did not emit. |"
    )
    lines.append("| Control P99 latency | Met | Sections 3 and 6, Figure 2. |")
    lines.append(
        "| Control-packet loss interval during switching, ±1 s | Met | Section 7.1 and "
        "Figure 17, derived from the per-event switch-log timestamps. |"
    )
    lines.append(
        "| Per-flow throughput share | Met | Section 3 and Figure 4. |"
    )
    lines.append("")
    lines.append("### 9.3 Named deliverables (§8, Traffic model)")
    lines.append("")
    lines.append("| Deliverable | Status |")
    lines.append("|---|---|")
    lines.append(
        "| `traffic_qos.cc` — 3-flow separated traffic model | Delivered |"
    )
    lines.append(
        "| `flow_metrics.py` — per-flow PDR, latency, P99, throughput share | Delivered |"
    )
    lines.append(
        "| `traffic_qos_report.pdf` — Control PDR, P99, bandwidth share | This document |"
    )
    lines.append("")
    lines.append(
        "Note on scope: the switching thresholds used in this campaign are −80 dBm with "
        "3 dB hysteresis, whereas §4.2.2 documents −58 dBm out and ≥ −55 dBm back. That "
        "difference changes how often switches fire and therefore every switch count in "
        "Section 7. Aligning the thresholds and classifying Intra-Mesh HO events belongs "
        "to the August enhancement item."
    )
    lines.append("")
    lines.append("---")
    lines.append(f"*End of report — {AUTHOR}, {LAB}*")
    lines.append("")
    return "\n".join(lines)


def md_to_pdf(md_path: Path, pdf_path: Path, fig_dir: Path) -> bool:
    try:
        import markdown
        from weasyprint import HTML, CSS
    except Exception as exc:  # noqa: BLE001
        print(f"PDF skipped (missing dependency): {exc}")
        return False

    html_body = markdown.markdown(
        md_path.read_text(encoding="utf-8"),
        extensions=["tables", "fenced_code"],
    )
    # Make image paths absolute for weasyprint
    html_body = html_body.replace("figures/", f"{fig_dir.as_uri()}/")
    css = CSS(
        string="""
        @page { size: A4; margin: 18mm 16mm; }
        body { font-family: DejaVu Sans, Arial, sans-serif; font-size: 11pt; color: #1f2937; }
        h1 { color: #1f4e79; font-size: 20pt; }
        h2 { color: #1f4e79; font-size: 14pt; margin-top: 1.2em; }
        h3 { color: #334155; font-size: 12pt; }
        table { border-collapse: collapse; width: 100%; margin: 0.6em 0 1em; font-size: 9pt; }
        th, td { border: 1px solid #cbd5e1; padding: 4px 6px; }
        th { background: #1f4e79; color: white; }
        tr:nth-child(even) { background: #f1f5f9; }
        img { max-width: 100%; height: auto; margin: 0.4em 0 0.8em; }
        code { font-size: 9pt; }
        """
    )
    HTML(string=f"<html><body>{html_body}</body></html>", base_url=str(md_path.parent)).write_pdf(
        str(pdf_path), stylesheets=[css]
    )
    return True


def main() -> int:
    args = parse_args()
    ns3_root = Path(__file__).resolve().parent.parent
    campaign = (ns3_root / args.campaign_dir).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else campaign / "report"
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    summary_path = campaign / "summary.csv"
    metrics_path = campaign / "gathered_metrics.csv"
    if not summary_path.exists() or not metrics_path.exists():
        print(f"ERROR: missing {summary_path} or {metrics_path}", file=sys.stderr)
        return 1

    summary = load_csv(summary_path)
    metrics = load_csv(metrics_path)
    print(f"Loaded {len(summary)} summary rows, {len(metrics)} metric rows")

    leg_rows = build_leg_metrics(
        campaign, summary, campaign / "leg_metrics.csv", refresh=args.refresh_legs
    )
    sw = collect_switch_stats(campaign, summary, args.switch_timeout)
    print(
        f"Switch interruption samples: {len(sw['all_measured'])} measured, "
        f"{len(sw['all_censored'])} censored"
    )

    generators = [
        ("fig01_control_pdr_by_mode.png", lambda p: fig_control_pdr_by_mode(metrics, p)),
        ("fig02_control_p99_by_mode.png", lambda p: fig_control_p99_by_mode(metrics, p)),
        ("fig03_per_flow_pdr_grouped.png", lambda p: fig_per_flow_pdr_grouped(metrics, p)),
        ("fig04_throughput_share.png", lambda p: fig_throughput_share(metrics, p)),
        ("fig05_control_pdr_vs_sta.png", lambda p: fig_control_pdr_vs_sta(metrics, p)),
        ("fig06_control_pdr_vs_payload.png", lambda p: fig_control_pdr_vs_payload(metrics, p)),
        ("fig07_band_comparison.png", lambda p: fig_band_comparison(metrics, p)),
        ("fig08_switch_resolved_rate.png", lambda p: fig_switch_resolved_rate(summary, p)),
        ("fig09_timeout_heatmap.png", lambda p: fig_timeout_heatmap(summary, p)),
        ("fig10_switch_volume.png", lambda p: fig_switch_volume(summary, p)),
        ("fig11_seed_variation.png", lambda p: fig_seed_variation(metrics, p)),
        ("fig12_control_pass_rate.png", lambda p: fig_control_pass_rate(metrics, p)),
        ("fig13_control_pdr_by_leg.png", lambda p: fig_control_pdr_by_leg(leg_rows, p)),
        ("fig14_interruption_ecdf.png", lambda p: fig_interruption_ecdf(sw, p)),
        ("fig15_leg_pdr_vs_sta.png", lambda p: fig_leg_pdr_vs_sta(leg_rows, p)),
        ("fig16_dark_flow_rate.png", lambda p: fig_dark_flow_rate(leg_rows, p)),
        ("fig17_control_continuity.png", lambda p: fig_control_continuity(sw, p)),
    ]

    print("Generating figures...")
    fig_names: List[str] = []
    for name, fn in generators:
        fn(fig_dir / name)
        fig_names.append(name)

    md = build_markdown(campaign, summary, metrics, leg_rows, sw, fig_names)
    md_path = out_dir / "traffic_qos_report.md"
    md_path.write_text(md, encoding="utf-8")
    print(f"Wrote {md_path}")

    # Also copy/symlink-friendly deliverable name at campaign root
    campaign_md = campaign / "traffic_qos_report.md"
    campaign_md.write_text(md.replace("figures/", "report/figures/"), encoding="utf-8")

    if not args.no_pdf:
        pdf_path = out_dir / "traffic_qos_report.pdf"
        if md_to_pdf(md_path, pdf_path, fig_dir):
            print(f"Wrote {pdf_path}")
            # Convenience copy at campaign root
            import shutil

            shutil.copy2(pdf_path, campaign / "traffic_qos_report.pdf")
            print(f"Wrote {campaign / 'traffic_qos_report.pdf'}")
        else:
            print("Markdown report is available; install weasyprint+markdown for PDF.")

    print(f"Done. Output directory: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
