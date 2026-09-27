#!/usr/bin/env python3
"""
September Weeks 3–4 statistical reinforcement for the traffic-qos 640-run matrix.

Implements the plan (§5.2 / §7 / §8):
  * Bootstrap-resampled (1,000) 95% confidence intervals on key KPIs
  * Mann–Whitney U tests: WiFi+LTE vs WiFi+5G NR (PDR, delay, throughput)
  * Kruskal–Wallis tests: STA-count trends (5 / 10 / 15 / 20)
  * Enhancement_Final_Report.pdf integrating all four enhancement items

Reads (from --campaign-dir):
  gathered_metrics.csv, summary.csv, optional per-run wifi-hybrid-switch_log.csv

Writes (under --out-dir, default <campaign>/stats_analysis):
  bootstrap_ci.csv
  mann_whitney.csv
  kruskal_wallis.csv
  switch_bootstrap_ci.csv
  figures/*.png
  stats_analysis.md
  Enhancement_Final_Report.md
  Enhancement_Final_Report.pdf

Usage (from ns-3.45/):
  python3 tools/stats_analysis.py
  python3 tools/stats_analysis.py \\
      --campaign-dir Traffic_qos_outputs/Traffic_qos_matrix_sep_seeds7to16 \\
      --out-dir Traffic_qos_outputs/Traffic_qos_matrix_sep_seeds7to16/stats_analysis
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

AUTHOR = "Sheikh Sayed Bin Rahman"
LAB = "PIC Lab, KIT"
FLOWS = ("Control", "Sensor", "Video")
KPIS = (
    ("pdr_pct", "PDR (%)", True),          # higher better
    ("mean_delay_ms", "Mean delay (ms)", False),
    ("throughput_mbps", "Throughput (Mbps)", True),
    ("p99_ms", "P99 delay (ms)", False),
)
STA_ORDER = (5, 10, 15, 20)
N_BOOT = 1000
ALPHA = 0.05
CI_WIDTH_TARGET_FRAC = 0.10  # plan: CI half-width within ±10% of mean
RNG_SEED = 20260927


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Bootstrap CIs + Mann–Whitney + Kruskal–Wallis for traffic-qos matrix"
    )
    p.add_argument(
        "--campaign-dir",
        default="Traffic_qos_outputs/Traffic_qos_matrix_sep_seeds7to16",
        help="Campaign folder with gathered_metrics.csv and summary.csv",
    )
    p.add_argument(
        "--out-dir",
        default="",
        help="Output directory (default: <campaign-dir>/stats_analysis)",
    )
    p.add_argument("--n-boot", type=int, default=N_BOOT, help="Bootstrap iterations")
    p.add_argument("--alpha", type=float, default=ALPHA, help="Significance / CI alpha")
    p.add_argument(
        "--skip-switch-logs",
        action="store_true",
        help="Skip per-run switch_log parsing (faster; omits interruption CIs)",
    )
    p.add_argument("--no-pdf", action="store_true", help="Skip PDF generation")
    return p.parse_args()


def fnum(x: object, default: float = float("nan")) -> float:
    try:
        if x is None or x == "":
            return default
        return float(x)
    except (TypeError, ValueError):
        return default


def load_csv(path: Path) -> List[dict]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows: Sequence[dict], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def bootstrap_ci(
    values: Sequence[float],
    n_boot: int,
    alpha: float,
    rng: np.random.Generator,
) -> Tuple[float, float, float, int, float]:
    """Return (mean, lo, hi, n, half_width_frac). Percentile bootstrap on the mean."""
    arr = np.asarray([v for v in values if v is not None and not math.isnan(v)], dtype=float)
    n = int(arr.size)
    if n == 0:
        return float("nan"), float("nan"), float("nan"), 0, float("nan")
    if n == 1:
        m = float(arr[0])
        return m, m, m, 1, 0.0
    means = np.empty(n_boot, dtype=float)
    for i in range(n_boot):
        sample = rng.choice(arr, size=n, replace=True)
        means[i] = float(sample.mean())
    lo, hi = np.percentile(means, [100.0 * alpha / 2.0, 100.0 * (1.0 - alpha / 2.0)])
    m = float(arr.mean())
    half = 0.5 * (float(hi) - float(lo))
    frac = abs(half / m) if abs(m) > 1e-12 else float("nan")
    return m, float(lo), float(hi), n, frac


def fmt(x: float, nd: int = 3) -> str:
    if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
        return "n/a"
    return f"{x:.{nd}f}"


def sig_label(p: float, alpha: float) -> str:
    if p is None or (isinstance(p, float) and math.isnan(p)):
        return "n/a"
    return "yes" if p < alpha else "no"


# ── Data extraction ───────────────────────────────────────────────────────────

def metric_values(
    metrics: Sequence[dict],
    *,
    flow: Optional[str] = None,
    mode: Optional[str] = None,
    sta: Optional[int] = None,
    band: Optional[str] = None,
    payload: Optional[str] = None,
    field: str = "pdr_pct",
) -> List[float]:
    out: List[float] = []
    for r in metrics:
        if flow is not None and r.get("flow") != flow:
            continue
        if mode is not None and r.get("cellularMode") != mode:
            continue
        if band is not None and r.get("hotspotBand") != band:
            continue
        if payload is not None and r.get("payload") != payload:
            continue
        if sta is not None and int(float(r.get("numStaNodes") or 0)) != sta:
            continue
        v = fnum(r.get(field))
        if not math.isnan(v):
            out.append(v)
    return out


def load_switch_interruptions(campaign: Path, summary: Sequence[dict]) -> List[dict]:
    """One row per resolved switch event with service_interruption_ms."""
    rows: List[dict] = []
    for s in summary:
        run_id = s.get("run_id") or ""
        scenario_dir = s.get("scenario_dir") or ""
        if not run_id:
            continue
        # scenario_dir may be relative to ns-3.45
        path = campaign / run_id / "wifi-hybrid-switch_log.csv"
        if not path.exists() and scenario_dir:
            # scenario_dir may be relative to ns-3.45/
            alt = Path(scenario_dir)
            if not alt.is_absolute():
                alt = campaign.parent.parent / scenario_dir
            path = alt / "wifi-hybrid-switch_log.csv"
        if not path.exists():
            continue
        try:
            with path.open(newline="", encoding="utf-8") as f:
                for ev in csv.DictReader(f):
                    if (ev.get("status") or "").strip().lower() != "resolved":
                        continue
                    ms = fnum(ev.get("service_interruption_ms"))
                    if math.isnan(ms):
                        continue
                    rows.append(
                        {
                            "run_id": run_id,
                            "cellularMode": s.get("cellularMode") or "",
                            "hotspotBand": s.get("hotspotBand") or "",
                            "numStaNodes": s.get("numStaNodes") or "",
                            "payload": s.get("payload") or "",
                            "rngSeed": s.get("rngSeed") or "",
                            "service_interruption_ms": ms,
                            "type": ev.get("type") or "",
                        }
                    )
        except OSError:
            continue
    return rows


# ── Analyses ──────────────────────────────────────────────────────────────────

def run_bootstrap(
    metrics: Sequence[dict],
    n_boot: int,
    alpha: float,
    rng: np.random.Generator,
) -> List[dict]:
    rows: List[dict] = []

    def emit(scope: str, flow: Optional[str], mode: Optional[str], sta: Optional[int], field: str, label: str, higher: bool) -> None:
        vals = metric_values(metrics, flow=flow, mode=mode, sta=sta, field=field)
        mean, lo, hi, n, frac = bootstrap_ci(vals, n_boot, alpha, rng)
        rows.append(
            {
                "scope": scope,
                "flow": flow or "ALL",
                "cellularMode": mode or "ALL",
                "numStaNodes": sta if sta is not None else "ALL",
                "kpi": field,
                "kpi_label": label,
                "higher_better": "yes" if higher else "no",
                "n": n,
                "mean": f"{mean:.6f}" if not math.isnan(mean) else "",
                "ci_lo": f"{lo:.6f}" if not math.isnan(lo) else "",
                "ci_hi": f"{hi:.6f}" if not math.isnan(hi) else "",
                "ci_halfwidth_frac": f"{frac:.6f}" if not math.isnan(frac) else "",
                "ci_within_10pct_mean": (
                    "yes" if (not math.isnan(frac) and frac <= CI_WIDTH_TARGET_FRAC) else "no"
                ),
            }
        )

    for field, label, higher in KPIS:
        # overall
        emit("overall", None, None, None, field, label, higher)
        # by mode
        for mode in ("lte", "nr"):
            emit("by_mode", None, mode, None, field, label, higher)
        # by flow × mode
        for flow in FLOWS:
            for mode in ("lte", "nr"):
                emit("by_flow_mode", flow, mode, None, field, label, higher)
        # by STA
        for sta in STA_ORDER:
            emit("by_sta", None, None, sta, field, label, higher)
        # by flow × STA
        for flow in FLOWS:
            for sta in STA_ORDER:
                emit("by_flow_sta", flow, None, sta, field, label, higher)
        # by flow × mode × STA
        for flow in FLOWS:
            for mode in ("lte", "nr"):
                for sta in STA_ORDER:
                    emit("by_flow_mode_sta", flow, mode, sta, field, label, higher)
    return rows


def run_mann_whitney(metrics: Sequence[dict], alpha: float) -> List[dict]:
    rows: List[dict] = []
    # Overall per flow + KPI, and per flow × STA
    comparisons: List[Tuple[str, Optional[str], Optional[int]]] = [
        ("by_flow", flow, None) for flow in FLOWS
    ] + [
        ("by_flow_sta", flow, sta) for flow in FLOWS for sta in STA_ORDER
    ]

    for scope, flow, sta in comparisons:
        for field, label, higher in KPIS:
            a = metric_values(metrics, flow=flow, mode="lte", sta=sta, field=field)
            b = metric_values(metrics, flow=flow, mode="nr", sta=sta, field=field)
            if len(a) < 2 or len(b) < 2:
                rows.append(
                    {
                        "scope": scope,
                        "flow": flow,
                        "numStaNodes": sta if sta is not None else "ALL",
                        "kpi": field,
                        "kpi_label": label,
                        "n_lte": len(a),
                        "n_nr": len(b),
                        "median_lte": "",
                        "median_nr": "",
                        "mean_lte": "",
                        "mean_nr": "",
                        "U": "",
                        "p_value": "",
                        "significant_p_lt_0.05": "n/a",
                        "higher_median_mode": "n/a",
                        "note": "insufficient samples",
                    }
                )
                continue
            # SciPy 1.11+: method='auto'; alternative='two-sided'
            res = stats.mannwhitneyu(a, b, alternative="two-sided", method="auto")
            med_a = float(np.median(a))
            med_b = float(np.median(b))
            if med_a == med_b:
                winner = "tie"
            elif higher:
                winner = "lte" if med_a > med_b else "nr"
            else:
                winner = "lte" if med_a < med_b else "nr"
            rows.append(
                {
                    "scope": scope,
                    "flow": flow,
                    "numStaNodes": sta if sta is not None else "ALL",
                    "kpi": field,
                    "kpi_label": label,
                    "n_lte": len(a),
                    "n_nr": len(b),
                    "median_lte": f"{med_a:.6f}",
                    "median_nr": f"{med_b:.6f}",
                    "mean_lte": f"{float(np.mean(a)):.6f}",
                    "mean_nr": f"{float(np.mean(b)):.6f}",
                    "U": f"{float(res.statistic):.4f}",
                    "p_value": f"{float(res.pvalue):.6g}",
                    "significant_p_lt_0.05": sig_label(float(res.pvalue), alpha),
                    "higher_median_mode": winner,
                    "note": "",
                }
            )
    return rows


def run_kruskal(metrics: Sequence[dict], alpha: float) -> List[dict]:
    rows: List[dict] = []
    scopes = [
        ("by_flow", None),
        ("by_flow_mode_lte", "lte"),
        ("by_flow_mode_nr", "nr"),
    ]
    for scope, mode in scopes:
        for flow in FLOWS:
            for field, label, _higher in KPIS:
                groups = [
                    metric_values(metrics, flow=flow, mode=mode, sta=sta, field=field)
                    for sta in STA_ORDER
                ]
                sizes = [len(g) for g in groups]
                if any(n < 2 for n in sizes):
                    rows.append(
                        {
                            "scope": scope,
                            "flow": flow,
                            "cellularMode": mode or "ALL",
                            "kpi": field,
                            "kpi_label": label,
                            "n_sta5": sizes[0],
                            "n_sta10": sizes[1],
                            "n_sta15": sizes[2],
                            "n_sta20": sizes[3],
                            "H": "",
                            "p_value": "",
                            "significant_p_lt_0.05": "n/a",
                            "note": "insufficient samples",
                        }
                    )
                    continue
                res = stats.kruskal(*groups)
                # Directional hint: Spearman of STA vs mean KPI
                means = [float(np.mean(g)) for g in groups]
                rho, _ = stats.spearmanr(STA_ORDER, means)
                rows.append(
                    {
                        "scope": scope,
                        "flow": flow,
                        "cellularMode": mode or "ALL",
                        "kpi": field,
                        "kpi_label": label,
                        "n_sta5": sizes[0],
                        "n_sta10": sizes[1],
                        "n_sta15": sizes[2],
                        "n_sta20": sizes[3],
                        "mean_sta5": f"{means[0]:.6f}",
                        "mean_sta10": f"{means[1]:.6f}",
                        "mean_sta15": f"{means[2]:.6f}",
                        "mean_sta20": f"{means[3]:.6f}",
                        "H": f"{float(res.statistic):.4f}",
                        "p_value": f"{float(res.pvalue):.6g}",
                        "significant_p_lt_0.05": sig_label(float(res.pvalue), alpha),
                        "spearman_rho_sta_vs_mean": f"{float(rho):.4f}",
                        "note": "",
                    }
                )
    return rows


def run_switch_bootstrap(
    switch_rows: Sequence[dict],
    n_boot: int,
    alpha: float,
    rng: np.random.Generator,
) -> List[dict]:
    rows: List[dict] = []
    if not switch_rows:
        return rows

    def vals(mode: Optional[str] = None, sta: Optional[int] = None) -> List[float]:
        out = []
        for r in switch_rows:
            if mode is not None and r.get("cellularMode") != mode:
                continue
            if sta is not None and int(float(r.get("numStaNodes") or 0)) != sta:
                continue
            out.append(float(r["service_interruption_ms"]))
        return out

    targets = [("overall", None, None)]
    for mode in ("lte", "nr"):
        targets.append((f"mode_{mode}", mode, None))
    for sta in STA_ORDER:
        targets.append((f"sta_{sta}", None, sta))

    for scope, mode, sta in targets:
        v = vals(mode, sta)
        mean, lo, hi, n, frac = bootstrap_ci(v, n_boot, alpha, rng)
        within_200 = sum(1 for x in v if x <= 200.0)
        rows.append(
            {
                "scope": scope,
                "cellularMode": mode or "ALL",
                "numStaNodes": sta if sta is not None else "ALL",
                "kpi": "service_interruption_ms",
                "n_resolved": n,
                "mean_ms": f"{mean:.6f}" if not math.isnan(mean) else "",
                "ci_lo_ms": f"{lo:.6f}" if not math.isnan(lo) else "",
                "ci_hi_ms": f"{hi:.6f}" if not math.isnan(hi) else "",
                "ci_halfwidth_frac": f"{frac:.6f}" if not math.isnan(frac) else "",
                "pct_le_200ms": f"{(100.0 * within_200 / n):.3f}" if n else "",
            }
        )
    return rows


# ── Figures ───────────────────────────────────────────────────────────────────

def style_axes(ax, title: str, xlabel: str = "", ylabel: str = "") -> None:
    ax.set_title(title, fontsize=11, fontweight="bold", color="#1f4e79")
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.25)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def fig_ci_by_mode(metrics: Sequence[dict], boot_rows: Sequence[dict], out: Path, rng_unused=None) -> None:
    # Control PDR / delay / tput: LTE vs NR with error bars from bootstrap table
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.8))
    kpis = [("pdr_pct", "PDR (%)"), ("mean_delay_ms", "Mean delay (ms)"), ("throughput_mbps", "Throughput (Mbps)")]
    lookup = {
        (r["kpi"], r["cellularMode"]): r
        for r in boot_rows
        if r["scope"] == "by_flow_mode" and r["flow"] == "Control"
    }
    for ax, (kpi, ylab) in zip(axes, kpis):
        means, yerr_lo, yerr_hi, labels, colors = [], [], [], [], []
        for mode, color, lab in (("lte", "#3b82f6", "LTE"), ("nr", "#22c55e", "5G NR")):
            r = lookup.get((kpi, mode))
            if not r or not r["mean"]:
                continue
            m = float(r["mean"])
            lo = float(r["ci_lo"])
            hi = float(r["ci_hi"])
            means.append(m)
            yerr_lo.append(m - lo)
            yerr_hi.append(hi - m)
            labels.append(lab)
            colors.append(color)
        x = np.arange(len(means))
        ax.bar(x, means, color=colors, alpha=0.88, yerr=[yerr_lo, yerr_hi], capsize=4, ecolor="#334155")
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        style_axes(ax, f"Control {ylab.split('(')[0].strip()}", ylabel=ylab)
    fig.suptitle("Control-flow KPIs — mean ± bootstrap 95% CI (10 seeds)", fontsize=12, color="#0f172a")
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)


def fig_sta_trend(metrics: Sequence[dict], out: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0))
    for ax, (field, ylab) in zip(
        axes, (("pdr_pct", "Control PDR (%)"), ("mean_delay_ms", "Control mean delay (ms)"))
    ):
        for mode, color, lab in (("lte", "#3b82f6", "LTE"), ("nr", "#22c55e", "5G NR")):
            ys, yerr = [], []
            for sta in STA_ORDER:
                vals = metric_values(metrics, flow="Control", mode=mode, sta=sta, field=field)
                if not vals:
                    ys.append(float("nan"))
                    yerr.append(0.0)
                    continue
                m = float(np.mean(vals))
                se = float(np.std(vals, ddof=1) / math.sqrt(len(vals))) if len(vals) > 1 else 0.0
                ys.append(m)
                yerr.append(1.96 * se)
            ax.errorbar(STA_ORDER, ys, yerr=yerr, marker="o", color=color, label=lab, capsize=3)
        style_axes(ax, ylab, xlabel="STA count", ylabel=ylab)
        ax.legend(fontsize=9)
    fig.suptitle("STA-count trends (mean ± approx. 95% SE) — Kruskal–Wallis input", fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)


def fig_pvalue_heatmap(mw_rows: Sequence[dict], out: Path) -> None:
    # Control flow: STA × KPI p-values
    kpis = ["pdr_pct", "mean_delay_ms", "throughput_mbps"]
    labels = ["PDR", "Delay", "Thr"]
    mat = np.full((len(STA_ORDER), len(kpis)), np.nan)
    for i, sta in enumerate(STA_ORDER):
        for j, kpi in enumerate(kpis):
            for r in mw_rows:
                if (
                    r["scope"] == "by_flow_sta"
                    and r["flow"] == "Control"
                    and str(r["numStaNodes"]) == str(sta)
                    and r["kpi"] == kpi
                    and r["p_value"]
                ):
                    mat[i, j] = float(r["p_value"])
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    im = ax.imshow(np.log10(np.clip(mat, 1e-16, 1.0)), cmap="RdYlGn_r", aspect="auto")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels)
    ax.set_yticks(range(len(STA_ORDER)))
    ax.set_yticklabels([str(s) for s in STA_ORDER])
    ax.set_ylabel("STA count")
    ax.set_title("Mann–Whitney p-values (Control, LTE vs NR)\n(log10 scale; green = larger p)")
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            if not math.isnan(mat[i, j]):
                ax.text(j, i, f"{mat[i, j]:.3g}", ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=ax, fraction=0.046, label="log10(p)")
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)


# ── Markdown / PDF ────────────────────────────────────────────────────────────

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
    html_body = html_body.replace("figures/", f"{fig_dir.as_uri()}/")
    css = CSS(
        string="""
        @page { size: A4; margin: 16mm 14mm; }
        body { font-family: DejaVu Sans, Arial, sans-serif; font-size: 10.5pt; color: #1f2937; }
        h1 { color: #1f4e79; font-size: 18pt; }
        h2 { color: #1f4e79; font-size: 13pt; margin-top: 1.1em; }
        h3 { color: #334155; font-size: 11.5pt; }
        table { border-collapse: collapse; width: 100%; margin: 0.5em 0 0.9em; font-size: 8.5pt; }
        th, td { border: 1px solid #cbd5e1; padding: 3px 5px; }
        th { background: #1f4e79; color: white; }
        tr:nth-child(even) { background: #f1f5f9; }
        img { max-width: 100%; height: auto; margin: 0.3em 0 0.7em; }
        code { font-size: 8.5pt; }
        """
    )
    HTML(string=f"<html><body>{html_body}</body></html>", base_url=str(md_path.parent)).write_pdf(
        str(pdf_path), stylesheets=[css]
    )
    return True


def build_stats_markdown(
    campaign: Path,
    summary: Sequence[dict],
    metrics: Sequence[dict],
    boot: Sequence[dict],
    mw: Sequence[dict],
    kw: Sequence[dict],
    sw_boot: Sequence[dict],
    fig_names: Sequence[str],
    n_boot: int,
    alpha: float,
) -> str:
    n_runs = len({r.get("run_id") for r in metrics})
    n_ok = sum(1 for r in summary if r.get("status") in ("ok", "skipped"))
    lines: List[str] = []
    lines += [
        "# Statistical Analysis Report (September Weeks 3–4)",
        "",
        f"- **Author:** {AUTHOR}",
        f"- **Lab:** {LAB}",
        f"- **Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        f"- **Campaign:** `{campaign}`",
        f"- **Runs with metrics:** {n_runs} (summary rows: {len(summary)}, status ok/skipped: {n_ok})",
        f"- **Bootstrap:** {n_boot} resamples, {(1 - alpha) * 100:.0f}% percentile CI",
        f"- **Significance:** p < {alpha:g}",
        "",
        "## 1. Method",
        "",
        "This script implements §5.2 of the NS-3 Simulation Enhancement Plan:",
        "",
        "1. **Bootstrap 95% CIs** on the sample mean (1,000 resamples) for PDR, mean delay, "
        "P99 delay, and throughput, plus resolved switch interruption time.",
        "2. **Mann–Whitney U** (two-sided) comparing WiFi+LTE vs WiFi+5G NR per flow "
        "(and per flow × STA).",
        "3. **Kruskal–Wallis** across STA counts {5, 10, 15, 20}, with a Spearman ρ hint "
        "for monotonic direction.",
        "",
        "Unit of observation = one matrix cell (one seed × factor combination) per flow. "
        "With 10 seeds the LTE and NR arms each contribute 320 Control-flow observations "
        "in the overall mode comparison.",
        "",
        "## 2. Figures",
        "",
    ]
    for name in fig_names:
        lines.append(f"![{name}](figures/{name})")
        lines.append("")

    # Bootstrap highlight table — Control by mode
    lines += [
        "## 3. Bootstrap 95% confidence intervals (Control flow)",
        "",
        "| Mode | KPI | n | Mean | 95% CI | Half-width / mean | ≤10% target |",
        "|---|---|---:|---:|---|---:|---|",
    ]
    for r in boot:
        if r["scope"] != "by_flow_mode" or r["flow"] != "Control":
            continue
        if r["kpi"] not in ("pdr_pct", "mean_delay_ms", "throughput_mbps", "p99_ms"):
            continue
        lines.append(
            f"| {r['cellularMode']} | {r['kpi_label']} | {r['n']} | {fmt(fnum(r['mean']))} | "
            f"[{fmt(fnum(r['ci_lo']))}, {fmt(fnum(r['ci_hi']))}] | "
            f"{fmt(fnum(r['ci_halfwidth_frac']), 3)} | {r['ci_within_10pct_mean']} |"
        )
    lines.append("")

    if sw_boot:
        lines += [
            "### Switch interruption (resolved only)",
            "",
            "| Scope | Mode | STA | n | Mean (ms) | 95% CI (ms) | % ≤ 200 ms |",
            "|---|---|---|---:|---:|---|---:|",
        ]
        for r in sw_boot:
            lines.append(
                f"| {r['scope']} | {r['cellularMode']} | {r['numStaNodes']} | {r['n_resolved']} | "
                f"{fmt(fnum(r['mean_ms']), 2)} | "
                f"[{fmt(fnum(r['ci_lo_ms']), 2)}, {fmt(fnum(r['ci_hi_ms']), 2)}] | "
                f"{fmt(fnum(r['pct_le_200ms']), 1)} |"
            )
        lines.append("")

    # Mann-Whitney
    lines += [
        "## 4. Mann–Whitney U — LTE vs 5G NR",
        "",
        "p < 0.05 is marked **significant**. `higher_median_mode` is the arm with the "
        "better median (higher PDR/throughput, lower delay).",
        "",
        "### 4.1 Overall by flow",
        "",
        "| Flow | KPI | n_LTE | n_NR | Median LTE | Median NR | U | p | Sig? | Better median |",
        "|---|---|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for r in mw:
        if r["scope"] != "by_flow":
            continue
        lines.append(
            f"| {r['flow']} | {r['kpi_label']} | {r['n_lte']} | {r['n_nr']} | "
            f"{fmt(fnum(r['median_lte']))} | {fmt(fnum(r['median_nr']))} | "
            f"{fmt(fnum(r['U']), 1)} | {r['p_value'] or 'n/a'} | "
            f"**{r['significant_p_lt_0.05']}** | {r['higher_median_mode']} |"
        )
    lines += [
        "",
        "### 4.2 Control flow by STA count",
        "",
        "| STA | KPI | p | Sig? | Better median | Mean LTE | Mean NR |",
        "|---:|---|---:|---|---|---:|---:|",
    ]
    for r in mw:
        if r["scope"] != "by_flow_sta" or r["flow"] != "Control":
            continue
        lines.append(
            f"| {r['numStaNodes']} | {r['kpi_label']} | {r['p_value'] or 'n/a'} | "
            f"**{r['significant_p_lt_0.05']}** | {r['higher_median_mode']} | "
            f"{fmt(fnum(r['mean_lte']))} | {fmt(fnum(r['mean_nr']))} |"
        )
    lines.append("")

    # Kruskal
    lines += [
        "## 5. Kruskal–Wallis — STA-count trend",
        "",
        "| Scope | Flow | Mode | KPI | H | p | Sig? | ρ (STA vs mean) | Means 5/10/15/20 |",
        "|---|---|---|---|---:|---:|---|---:|---|",
    ]
    for r in kw:
        means = "/".join(
            fmt(fnum(r.get(k)), 2)
            for k in ("mean_sta5", "mean_sta10", "mean_sta15", "mean_sta20")
        )
        lines.append(
            f"| {r['scope']} | {r['flow']} | {r['cellularMode']} | {r['kpi_label']} | "
            f"{fmt(fnum(r['H']), 2)} | {r['p_value'] or 'n/a'} | "
            f"**{r['significant_p_lt_0.05']}** | "
            f"{fmt(fnum(r.get('spearman_rho_sta_vs_mean')), 3)} | {means} |"
        )
    lines += [
        "",
        "## 6. Interpretation notes",
        "",
        "- Bootstrap CIs quantify uncertainty after expanding from 3 → 10 seeds "
        "(640 matrix cells). The plan target is CI half-width ≤ 10% of the mean for key KPIs.",
        "- Mann–Whitney does **not** assume normality; it is appropriate for PDR and "
        "switching latency distributions.",
        "- Kruskal–Wallis tests whether the four STA levels share one distribution; a "
        "significant result supports a STA-load effect without requiring linearity.",
        "",
        "CSV artifacts: `bootstrap_ci.csv`, `mann_whitney.csv`, `kruskal_wallis.csv`, "
        "`switch_bootstrap_ci.csv`.",
        "",
        f"---",
        f"*End of stats analysis — {AUTHOR}, {LAB}*",
        "",
    ]
    return "\n".join(lines)


def build_final_report(
    campaign: Path,
    stats_md_body: str,
    boot: Sequence[dict],
    mw: Sequence[dict],
    kw: Sequence[dict],
    n_runs: int,
) -> str:
    """Integrate all four enhancement items + statistical results."""
    # Pull a few headline stats
    def boot_row(mode: str, kpi: str) -> Optional[dict]:
        for r in boot:
            if r["scope"] == "by_flow_mode" and r["flow"] == "Control" and r["cellularMode"] == mode and r["kpi"] == kpi:
                return r
        return None

    def mw_row(flow: str, kpi: str) -> Optional[dict]:
        for r in mw:
            if r["scope"] == "by_flow" and r["flow"] == flow and r["kpi"] == kpi:
                return r
        return None

    lte_pdr = boot_row("lte", "pdr_pct")
    nr_pdr = boot_row("nr", "pdr_pct")
    lte_d = boot_row("lte", "mean_delay_ms")
    nr_d = boot_row("nr", "mean_delay_ms")
    mw_pdr = mw_row("Control", "pdr_pct")
    mw_delay = mw_row("Control", "mean_delay_ms")

    # Count significant MW tests
    n_sig = sum(1 for r in mw if r.get("significant_p_lt_0.05") == "yes")
    n_mw = sum(1 for r in mw if r.get("p_value"))
    n_kw_sig = sum(1 for r in kw if r.get("significant_p_lt_0.05") == "yes")
    n_kw = sum(1 for r in kw if r.get("p_value"))

    lines: List[str] = [
        "# Enhancement Final Report",
        "",
        "**Phase 1 NS-3 simulation enhancement — completion report**",
        "",
        f"- **Author:** {AUTHOR}",
        f"- **Lab:** {LAB}",
        f"- **Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "- **Plan:** NS-3 Simulation Enhancement Plan (four-item accuracy upgrade)",
        f"- **September campaign:** `{campaign.name}` — **{n_runs}** seeded scenarios",
        "",
        "## 1. Purpose",
        "",
        "This report closes the four enhancement items that bridge Phase 1 hybrid "
        "simulation results toward field-validation readiness: realistic mobility, "
        "robot-oriented traffic QoS, WiFi-mesh internal handover / Guard Timer, and "
        "statistical reinforcement of the WiFi+LTE vs WiFi+5G comparison.",
        "",
        "## 2. Status of the four enhancement items",
        "",
        "| # | Item | Target | Primary deliverables | Status |",
        "|---|---|---|---|---|",
        "| 1 | Realistic mobility | Jun 2026 | `waypoint_mobility` scenarios, "
        "`Waypoint_outputs/mobility_comparison_report.pdf` | **Complete** |",
        "| 2 | Traffic model (Control/Sensor/Video) | Jul 2026 | `traffic_qos.cc`, "
        "`flow_metrics.py`, July traffic QoS report / presentations | **Complete** |",
        "| 3 | WiFi mesh internal HO + Guard Timer | Aug 2026 | Intra-mesh logging, "
        "`guard_timer_report.pdf` | **Complete** |",
        "| 4 | Statistical reinforcement (10 seeds) | Sep 2026 | 640-run matrix, "
        "`stats_analysis.py`, this report | **Complete** |",
        "",
        "### 2.1 Mobility (Item 1)",
        "",
        "Gauss-Markov random walks were supplemented with waypoint + dwell patterns "
        "that better match patrol / transport / work robots on a construction site. "
        "Comparison outputs (trajectory plots, RSSI heatmaps, switching timelines, "
        "and the mobility comparison PDF) live under `Waypoint_outputs/`.",
        "",
        "### 2.2 Traffic QoS (Item 2)",
        "",
        "Generic HTTP/video mixes were replaced by three DSCP-marked robot flows "
        "(Control / Sensor / Video) with per-flow FlowMonitor metrics. The September "
        f"matrix reuses that traffic model across **{n_runs}** cells "
        "(2 modes × 2 bands × 4 STA × 4 payloads × 10 seeds).",
        "",
        "### 2.3 WiFi internal handover / Guard Timer (Item 3)",
        "",
        "Intra-mesh HO events are classified in the switch log (`type` field: "
        "`intra_mesh` / `wifi_to_cell` / `cell_to_wifi`). Guard Timer experiments and "
        "the review PDF are under `Traffic_qos_outputs/guard_timer_study/` and "
        "`august_deliverable/`.",
        "",
        "### 2.4 Statistical reinforcement (Item 4) — headline results",
        "",
        "After expanding from 3 to **10 RNG seeds**, Bootstrap CIs, Mann–Whitney U, "
        "and Kruskal–Wallis tests were applied to the September campaign.",
        "",
        "| Control KPI | LTE mean [95% CI] | 5G NR mean [95% CI] | MW p (overall) | Significant? |",
        "|---|---|---|---:|---|",
    ]

    def ci_cell(r: Optional[dict]) -> str:
        if not r or not r.get("mean"):
            return "n/a"
        return f"{fmt(fnum(r['mean']))} [{fmt(fnum(r['ci_lo']))}, {fmt(fnum(r['ci_hi']))}]"

    lines.append(
        f"| PDR (%) | {ci_cell(lte_pdr)} | {ci_cell(nr_pdr)} | "
        f"{(mw_pdr or {}).get('p_value', 'n/a')} | "
        f"**{(mw_pdr or {}).get('significant_p_lt_0.05', 'n/a')}** |"
    )
    lines.append(
        f"| Mean delay (ms) | {ci_cell(lte_d)} | {ci_cell(nr_d)} | "
        f"{(mw_delay or {}).get('p_value', 'n/a')} | "
        f"**{(mw_delay or {}).get('significant_p_lt_0.05', 'n/a')}** |"
    )
    lines += [
        "",
        f"- Mann–Whitney tests with p < 0.05: **{n_sig} / {n_mw}**",
        f"- Kruskal–Wallis tests with p < 0.05 (STA trend): **{n_kw_sig} / {n_kw}**",
        "",
        "Detailed tables and figures are in Sections 4–6 below (same content as "
        "`stats_analysis.md`).",
        "",
        "## 3. Cross-item conclusions",
        "",
        "1. **Mobility realism** reduces the mismatch between simulated RSSI traces and "
        "path-constrained robots before field trials.",
        "2. **Per-flow QoS** makes Control latency/loss requirements directly measurable "
        "instead of hiding them in a blended PDR.",
        "3. **Guard Timer / intra-mesh classification** separates mesh-internal roaming "
        "from cellular failover so the hybrid controller is not blamed for AP handovers.",
        "4. **Ten-seed statistics** replace “appears different” language with Bootstrap "
        "intervals and non-parametric tests at α = 0.05 for the LTE vs 5G NR claim.",
        "",
        "Together these four items satisfy the September Weeks 3–4 deliverable set "
        "(`stats_analysis.py` + `Enhancement_Final_Report.pdf`) on top of the 640-run "
        "matrix completed in Weeks 1–2.",
        "",
        "---",
        "",
        "# Appendix — Full statistical analysis",
        "",
    ]
    # Append stats body without its own H1 title
    appendix = stats_md_body
    if appendix.startswith("# "):
        appendix = "\n".join(appendix.splitlines()[1:]).lstrip()
    lines.append(appendix)
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    ns3_root = Path(__file__).resolve().parent.parent
    campaign = (ns3_root / args.campaign_dir).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else campaign / "stats_analysis"
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

    rng = np.random.default_rng(RNG_SEED)

    print("Running bootstrap CIs…")
    boot = run_bootstrap(metrics, args.n_boot, args.alpha, rng)
    write_csv(
        out_dir / "bootstrap_ci.csv",
        boot,
        [
            "scope", "flow", "cellularMode", "numStaNodes", "kpi", "kpi_label",
            "higher_better", "n", "mean", "ci_lo", "ci_hi", "ci_halfwidth_frac",
            "ci_within_10pct_mean",
        ],
    )

    print("Running Mann–Whitney U…")
    mw = run_mann_whitney(metrics, args.alpha)
    write_csv(
        out_dir / "mann_whitney.csv",
        mw,
        [
            "scope", "flow", "numStaNodes", "kpi", "kpi_label", "n_lte", "n_nr",
            "median_lte", "median_nr", "mean_lte", "mean_nr", "U", "p_value",
            "significant_p_lt_0.05", "higher_median_mode", "note",
        ],
    )

    print("Running Kruskal–Wallis…")
    kw = run_kruskal(metrics, args.alpha)
    write_csv(
        out_dir / "kruskal_wallis.csv",
        kw,
        [
            "scope", "flow", "cellularMode", "kpi", "kpi_label",
            "n_sta5", "n_sta10", "n_sta15", "n_sta20",
            "mean_sta5", "mean_sta10", "mean_sta15", "mean_sta20",
            "H", "p_value", "significant_p_lt_0.05", "spearman_rho_sta_vs_mean", "note",
        ],
    )

    sw_boot: List[dict] = []
    if not args.skip_switch_logs:
        print("Parsing switch logs for interruption CIs…")
        sw_rows = load_switch_interruptions(campaign, summary)
        print(f"  resolved interruption samples: {len(sw_rows)}")
        sw_boot = run_switch_bootstrap(sw_rows, args.n_boot, args.alpha, rng)
        write_csv(
            out_dir / "switch_bootstrap_ci.csv",
            sw_boot,
            [
                "scope", "cellularMode", "numStaNodes", "kpi", "n_resolved",
                "mean_ms", "ci_lo_ms", "ci_hi_ms", "ci_halfwidth_frac", "pct_le_200ms",
            ],
        )

    print("Writing figures…")
    fig_names = [
        "fig01_control_bootstrap_ci.png",
        "fig02_sta_trends.png",
        "fig03_mw_pvalue_heatmap.png",
    ]
    fig_ci_by_mode(metrics, boot, fig_dir / fig_names[0])
    fig_sta_trend(metrics, fig_dir / fig_names[1])
    fig_pvalue_heatmap(mw, fig_dir / fig_names[2])

    stats_md = build_stats_markdown(
        campaign, summary, metrics, boot, mw, kw, sw_boot, fig_names, args.n_boot, args.alpha
    )
    stats_path = out_dir / "stats_analysis.md"
    stats_path.write_text(stats_md, encoding="utf-8")
    print(f"Wrote {stats_path}")

    n_runs = len({r.get("run_id") for r in metrics})
    final_md = build_final_report(campaign, stats_md, boot, mw, kw, n_runs)
    final_path = out_dir / "Enhancement_Final_Report.md"
    final_path.write_text(final_md, encoding="utf-8")
    # Also place a copy at campaign root for the plan's named deliverable path
    campaign_copy = campaign / "Enhancement_Final_Report.md"
    campaign_copy.write_text(final_md, encoding="utf-8")
    print(f"Wrote {final_path}")

    if not args.no_pdf:
        pdf_path = out_dir / "Enhancement_Final_Report.pdf"
        ok = md_to_pdf(final_path, pdf_path, fig_dir)
        if ok:
            # campaign-root copy
            import shutil

            shutil.copy2(pdf_path, campaign / "Enhancement_Final_Report.pdf")
            print(f"Wrote {pdf_path}")
            print(f"Copied {(campaign / 'Enhancement_Final_Report.pdf')}")
        else:
            print("Markdown written; PDF not produced.")

    # Quick console summary
    n_sig = sum(1 for r in mw if r.get("significant_p_lt_0.05") == "yes")
    print(
        f"Done. Mann–Whitney significant: {n_sig}/{sum(1 for r in mw if r.get('p_value'))}; "
        f"outputs in {out_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
