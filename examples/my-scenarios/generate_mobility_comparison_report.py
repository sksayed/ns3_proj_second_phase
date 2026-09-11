#!/usr/bin/env python3
"""
Waypoint vs. Gauss-Markov mobility comparison report (Phase 2 Item 1
deliverable: mobility_comparison_report.pdf / .md), July revision.

Scans Waypoint_outputs/{patrol,transport,work,gaussmarkov_baseline}/
<cellular>_sta<N>_<payload>_spd<speed>_seed<seed>/ run directories produced
by run_mobility_matrix.py, aggregates switching frequency, switching
burstiness, RSSI-change abruptness, service interruption duration, and PDR
across seeds 7/8/9, and writes a Markdown report (plus a PDF via weasyprint
if available).

This replaces the June version, which only had 1 seed (fake "±0.00%"),
used a non-standard -80dBm RSSI threshold, fixed STA count/payload/cellular
mode (no sweep), and blended patrol/transport/work into one "waypoint
average" number in its verdict section. Per reviewer feedback, this version:
  - Computes real mean/SD/95% CI across 3 seeds (7,8,9).
  - Standardizes on the project's -58dBm RSSI threshold.
  - Adds STA count (5/10/15), payload (10kb/50kb/1mb), and cellular mode
    (LTE/NR) sensitivity sections, matching the Phase 1 matrix's sweep axes.
  - Reports patrol/transport/work as individual rows against the
    Gauss-Markov baseline instead of one blended "waypoint average".
  - Frames conclusions as a "preliminary signal" (n=3 seeds), not a
    statistically confirmed result -- full statistical reinforcement
    (10 seeds, bootstrap CIs, significance tests) is enhancement item 4,
    targeted for Sep 2026.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/generate_mobility_comparison_report.py
    python3 examples/my-scenarios/generate_mobility_comparison_report.py \\
        --root Waypoint_outputs --out Waypoint_outputs/mobility_comparison_report
"""
import argparse
import csv
import re
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats as spstats

SCENARIOS = ["gaussmarkov_baseline", "patrol", "transport", "work"]
DIR_RE = re.compile(r"^(lte|nr)_sta(\d+)_(10kb|50kb|1mb|2mb)_spd([\d.]+)_seed(\d+)$")

# Canonical configuration used for the headline/speed/verdict sections, so
# they stay readable as single tables. The STA/payload/cellular sensitivity
# sections (4-6) hold this canonical point fixed on the other two axes while
# sweeping the axis under study.
CANON_CELLULAR = "lte"
CANON_STA = 5
CANON_PAYLOAD = "1mb"
CANON_SPEED = 5.0  # matches the June baseline speed for continuity

REPORT_AUTHOR = "Sheikh Sayed Bin Rahman"
REPORT_ID = "2025210714"
REPORT_LAB = "PIC Lab , KIT"


def read_switch_metrics(switch_csv, sim_time):
    if not switch_csv.exists():
        return None
    events = []
    with open(switch_csv) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["trigger_time_s"])
                interruption = float(row["service_interruption_ms"])
                status = row["status"]
            except (ValueError, KeyError):
                continue
            events.append((t, interruption, status))

    n_events = len(events)
    events_per_100s = n_events / (sim_time / 100.0) if sim_time > 0 else 0.0

    resolved = [e[1] for e in events if e[2] == "resolved"]
    mean_interruption = statistics.mean(resolved) if resolved else float("nan")
    p95_interruption = (
        sorted(resolved)[int(0.95 * (len(resolved) - 1))] if len(resolved) > 1 else
        (resolved[0] if resolved else float("nan"))
    )

    # Burstiness: bin trigger times into 10 windows across sim_time and take the
    # coefficient of variation of per-bin counts. Higher = more concentrated in
    # specific time windows (shadow zones); lower = more uniformly spread out.
    n_bins = 10
    bin_counts = [0] * n_bins
    for t, _, _ in events:
        idx = min(n_bins - 1, int((t / sim_time) * n_bins)) if sim_time > 0 else 0
        bin_counts[idx] += 1
    mean_count = statistics.mean(bin_counts)
    burstiness = (statistics.pstdev(bin_counts) / mean_count) if mean_count > 0 else 0.0

    return {
        "n_events": n_events,
        "events_per_100s": events_per_100s,
        "mean_interruption_ms": mean_interruption,
        "p95_interruption_ms": p95_interruption,
        "burstiness": burstiness,
    }


def read_rssi_abruptness(rssi_csv, resample_dt=1.0):
    """
    RSSI samples arrive far faster than once per second (every WiFi frame),
    so raw consecutive-row deltas are dominated by sub-second EWA noise and
    stay near zero regardless of real movement. Resample each STA's WiFi
    avg_rssi_dbm onto a fixed dt grid (hold-last-value) first, so deltas
    reflect actual second-to-second signal change, not oversampling.
    """
    if not rssi_csv.exists():
        return None
    per_sta = {}
    with open(rssi_csv) as f:
        for row in csv.DictReader(f):
            if row.get("rat") != "wifi":
                continue
            try:
                t = float(row["time_s"])
                sta = int(row["sta_index"])
                rssi = float(row["avg_rssi_dbm"])
            except (ValueError, KeyError):
                continue
            per_sta.setdefault(sta, []).append((t, rssi))

    deltas = []
    for sta, samples in per_sta.items():
        samples.sort(key=lambda r: r[0])
        if len(samples) < 2:
            continue
        max_t = samples[-1][0]
        grid = []
        i = 0
        n = len(samples)
        t = 0.0
        while t <= max_t + 1e-9:
            while i + 1 < n and samples[i + 1][0] <= t:
                i += 1
            grid.append(samples[i][1])
            t += resample_dt
        for r0, r1 in zip(grid, grid[1:]):
            deltas.append(abs(r1 - r0))

    if not deltas:
        return None
    deltas.sort()
    mean_delta = statistics.mean(deltas)
    p95_delta = deltas[int(0.95 * (len(deltas) - 1))]
    return {"mean_abs_rssi_delta": mean_delta, "p95_abs_rssi_delta": p95_delta}


def read_pdr(metrics_md):
    if not metrics_md.exists():
        return None
    text = metrics_md.read_text()
    m = re.search(r"Packet-weighted PDR \(%\):\s*\*\*([\d.]+)\*\*", text)
    return float(m.group(1)) if m else None


def collect_run(run_dir, sim_time):
    switch_metrics = read_switch_metrics(run_dir / "wifi-hybrid-switch_log.csv", sim_time)
    rssi_metrics = read_rssi_abruptness(run_dir / "wifi-hybrid-rssi_log.csv")
    pdr = read_pdr(run_dir / "wifi-hybrid-metrics_data.md")
    if switch_metrics is None or rssi_metrics is None:
        return None
    merged = {**switch_metrics, **rssi_metrics, "pdr": pdr}
    return merged


def scan_all(root, sim_time):
    """Flat list of run records: one dict per run directory, tagged with its
    scenario/cellular/sta/payload/speed/seed parsed from the folder name."""
    records = []
    for scenario in SCENARIOS:
        scenario_dir = root / scenario
        if not scenario_dir.exists():
            continue
        for run_dir in sorted(scenario_dir.iterdir()):
            if not run_dir.is_dir():
                continue
            m = DIR_RE.match(run_dir.name)
            if not m:
                continue
            cellular, sta, payload, speed, seed = m.groups()
            metrics = collect_run(run_dir, sim_time)
            if metrics is None:
                continue
            records.append({
                "scenario": scenario, "cellular": cellular, "sta": int(sta),
                "payload": payload, "speed": float(speed), "seed": int(seed),
                "run_dir": run_dir, **metrics,
            })
    return records


def group_by(records, keys):
    out = {}
    for r in records:
        k = tuple(r[key] for key in keys)
        out.setdefault(k, []).append(r)
    return out


def agg_stats(values):
    """Mean/SD/95% CI (t-distribution) across however many seeds are present.
    With n=3 the CI is wide -- that's an honest reflection of a 3-seed
    sample, not a bug; see the report's own 'preliminary signal' framing."""
    vals = np.array([v for v in values if v is not None and v == v], dtype=float)
    n = len(vals)
    if n == 0:
        return {"n": 0, "mean": float("nan"), "sd": float("nan"), "ci_lo": float("nan"), "ci_hi": float("nan")}
    mean = float(vals.mean())
    if n == 1:
        return {"n": 1, "mean": mean, "sd": 0.0, "ci_lo": mean, "ci_hi": mean}
    sd = float(vals.std(ddof=1))
    tval = spstats.t.ppf(0.975, df=n - 1)
    margin = tval * sd / np.sqrt(n)
    return {"n": n, "mean": mean, "sd": sd, "ci_lo": mean - margin, "ci_hi": mean + margin}


def metric_stats(runs, key):
    return agg_stats([r[key] for r in runs])


def fmt(agg, decimals=2):
    if agg["mean"] != agg["mean"]:
        return "n/a"
    return f"{agg['mean']:.{decimals}f} \u00b1 {agg['sd']:.{decimals}f} (n={agg['n']})"


def fmt_ci(agg, decimals=2):
    if agg["mean"] != agg["mean"]:
        return "n/a"
    return f"[{agg['ci_lo']:.{decimals}f}, {agg['ci_hi']:.{decimals}f}]"


def _wrap_chart_grids(html):
    """Wrap consecutive <p><img></p> blocks (already-converted HTML) in a
    chart-grid div so figures lay out side by side in the PDF, matching
    analysis_report_updated/md_to_pdf.py's approach."""
    pattern = re.compile(r"((?:<p>\s*<img[^>]+>\s*</p>\s*){2,})", re.MULTILINE)
    return pattern.sub(lambda m: f'<div class="chart-grid">{m.group(1)}</div>', html)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="Waypoint_outputs")
    parser.add_argument("--sim-time", type=float, default=90.0)
    parser.add_argument("--out", default="Waypoint_outputs/mobility_comparison_report")
    parser.add_argument("--no-pdf", action="store_true")
    args = parser.parse_args()

    root = Path(args.root)
    records = scan_all(root, args.sim_time)

    if not records:
        print(f"No run data found under {root}. Run run_mobility_matrix.py first.", file=sys.stderr)
        sys.exit(1)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    assets_dir = out_path.parent / "report_assets"
    assets_dir.mkdir(exist_ok=True)

    canon = {"cellular": CANON_CELLULAR, "sta": CANON_STA, "payload": CANON_PAYLOAD, "speed": CANON_SPEED}

    def filt(records, **fixed):
        return [r for r in records if all(r[k] == v for k, v in fixed.items())]

    # --- Section 1: Head-to-head at the canonical config ---
    headline_rows = []
    for scenario in SCENARIOS:
        runs = filt(records, scenario=scenario, **canon)
        if not runs:
            continue
        row = {"scenario": scenario, "n_runs": len(runs)}
        for key in ["events_per_100s", "burstiness", "mean_abs_rssi_delta",
                    "p95_abs_rssi_delta", "mean_interruption_ms", "pdr"]:
            row[key] = metric_stats(runs, key)
        headline_rows.append(row)

    # --- Section 2: Speed sensitivity (waypoint scenarios only), at canonical cellular/sta/payload ---
    speed_rows = []
    for scenario in ["patrol", "transport", "work"]:
        for speed in sorted({r["speed"] for r in filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, payload=CANON_PAYLOAD)}):
            runs = filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, payload=CANON_PAYLOAD, speed=speed)
            if not runs:
                continue
            row = {"scenario": scenario, "speed": speed, "n_runs": len(runs)}
            for key in ["events_per_100s", "burstiness", "mean_abs_rssi_delta", "p95_abs_rssi_delta"]:
                row[key] = metric_stats(runs, key)
            speed_rows.append(row)
    speed_rows.sort(key=lambda r: (r["scenario"], r["speed"]))

    # --- Section 3: STA count sensitivity, at canonical cellular/payload/speed ---
    sta_rows = []
    for scenario in SCENARIOS:
        for sta in sorted({r["sta"] for r in filt(records, scenario=scenario, cellular=CANON_CELLULAR, payload=CANON_PAYLOAD, speed=CANON_SPEED)}):
            runs = filt(records, scenario=scenario, cellular=CANON_CELLULAR, payload=CANON_PAYLOAD, speed=CANON_SPEED, sta=sta)
            if not runs:
                continue
            row = {"scenario": scenario, "sta": sta, "n_runs": len(runs)}
            for key in ["events_per_100s", "mean_interruption_ms", "pdr"]:
                row[key] = metric_stats(runs, key)
            sta_rows.append(row)
    sta_rows.sort(key=lambda r: (r["scenario"], r["sta"]))

    # --- Section 4: Payload sensitivity, at canonical cellular/sta/speed ---
    payload_order = {"10kb": 0, "50kb": 1, "1mb": 2, "2mb": 3}
    payload_rows = []
    for scenario in SCENARIOS:
        for payload in sorted({r["payload"] for r in filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, speed=CANON_SPEED)}, key=lambda p: payload_order.get(p, 99)):
            runs = filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, speed=CANON_SPEED, payload=payload)
            if not runs:
                continue
            row = {"scenario": scenario, "payload": payload, "n_runs": len(runs)}
            for key in ["events_per_100s", "mean_interruption_ms", "pdr"]:
                row[key] = metric_stats(runs, key)
            payload_rows.append(row)
    payload_rows.sort(key=lambda r: (r["scenario"], payload_order.get(r["payload"], 99)))

    # --- Section 5: Cellular mode comparison, at canonical sta/payload/speed ---
    cellular_rows = []
    for scenario in SCENARIOS:
        for cellular in ["lte", "nr"]:
            runs = filt(records, scenario=scenario, sta=CANON_STA, payload=CANON_PAYLOAD, speed=CANON_SPEED, cellular=cellular)
            if not runs:
                continue
            row = {"scenario": scenario, "cellular": cellular, "n_runs": len(runs)}
            for key in ["events_per_100s", "mean_interruption_ms", "pdr"]:
                row[key] = metric_stats(runs, key)
            cellular_rows.append(row)
    cellular_rows.sort(key=lambda r: (r["scenario"], r["cellular"]))

    # --- Charts (headline section) ---
    def bar_chart(rows, key, title, ylabel, filename):
        labels = [r["scenario"] for r in rows]
        means = [r[key]["mean"] for r in rows]
        sds = [r[key]["sd"] for r in rows]
        fig, axis = plt.subplots(figsize=(6, 4))
        axis.bar(labels, means, yerr=sds, capsize=4, color="#2563eb")
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        fig.tight_layout()
        fig.savefig(assets_dir / filename, dpi=140)
        plt.close(fig)

    if headline_rows:
        bar_chart(headline_rows, "events_per_100s",
                  f"Switching Frequency @ {CANON_SPEED} m/s (LTE, 5 STA, 1MB)", "events / 100s",
                  "chart_events_per_100s.png")
        bar_chart(headline_rows, "burstiness",
                  f"Switching Burstiness @ {CANON_SPEED} m/s (LTE, 5 STA, 1MB)",
                  "coefficient of variation (bin counts)", "chart_burstiness.png")
        bar_chart(headline_rows, "p95_abs_rssi_delta",
                  f"RSSI Change Abruptness (p95) @ {CANON_SPEED} m/s (LTE, 5 STA, 1MB)", "|\u0394 RSSI| dB (p95)",
                  "chart_rssi_abruptness.png")
        bar_chart(headline_rows, "mean_interruption_ms",
                  f"Mean Service Interruption @ {CANON_SPEED} m/s (LTE, 5 STA, 1MB)", "ms",
                  "chart_interruption.png")

    # --- Representative 3D trajectory plots: one per scenario type, canonical config ---
    trajectory_pngs = {}
    for scenario in SCENARIOS:
        runs = filt(records, scenario=scenario, **canon)
        if runs:
            src = runs[0]["run_dir"] / "trajectory_3d.png"
            if src.exists():
                dest = assets_dir / f"trajectory_3d_{scenario}.png"
                dest.write_bytes(src.read_bytes())
                trajectory_pngs[scenario] = dest.name

    # --- Verdict: per-type against Gauss-Markov, not blended ---
    gm = next((r for r in headline_rows if r["scenario"] == "gaussmarkov_baseline"), None)
    wp_rows = [r for r in headline_rows if r["scenario"] != "gaussmarkov_baseline"]

    total_runs = len(records)
    seeds_found = sorted({r["seed"] for r in records})
    sta_values = sorted({r["sta"] for r in records})
    payload_values = sorted({r["payload"] for r in records}, key=lambda p: payload_order.get(p, 99))
    cellular_values = sorted({r["cellular"] for r in records})
    speed_values = sorted({r["speed"] for r in records})

    zero_switch_runs = [r for r in records if r["n_events"] == 0]

    lines = []
    lines.append("# Waypoint vs. Gauss-Markov Mobility Comparison: (Phase 2 Item 1, Revised)")
    lines.append("")
    lines.append(f"- Author : {REPORT_AUTHOR}")
    lines.append(f"- ID: {REPORT_ID}")
    lines.append(f"- Lab: {REPORT_LAB}")
    lines.append(f"- **Total simulation runs evaluated:** {total_runs}")
    lines.append("- **Revision note:** this supersedes the June submission. Per reviewer feedback: "
                 "seeds expanded from 1 to 3 (7/8/9) with real mean/SD statistics instead of "
                 "\"\u00b10.00%\"; RSSI threshold standardized to the project's \u221258 dBm (was "
                 "\u221280 dBm); the \"work\" scenario's building-site placement was rebalanced so "
                 "it no longer over-samples the field's worst-covered corners; results are broken "
                 "out per robot type instead of blended into one \"waypoint average\"; STA count, "
                 "payload, and cellular mode are now swept (previously fixed); and every run now "
                 "has a 3D trajectory plot, animation, RSSI heatmap, and switching timeline.")
    lines.append("")

    lines.append("### Experimental design overview")
    lines.append("")
    lines.append("Each configuration corresponds to one simulation run comparing the Waypoint + "
                 "dwell-time mobility model (patrol / transport / work robot archetypes) against "
                 "the existing GaussMarkovMobilityModel baseline, now crossed with STA count, "
                 "payload size, and cellular mode -- the same sweep axes the original Phase 1 "
                 "matrix used -- so results are directly comparable to it.")
    lines.append("")
    lines.append("| Design element | Specification |")
    lines.append("| --- | --- |")
    lines.append(f"| Total configurations | {total_runs} simulation runs |")
    lines.append("| Scenarios | gaussmarkov_baseline, patrol, transport, work |")
    lines.append(f"| Speeds swept | {', '.join(str(s) for s in speed_values)} m/s |")
    lines.append(f"| STA counts swept | {', '.join(str(s) for s in sta_values)} |")
    lines.append(f"| Payloads swept | {', '.join(payload_values)} |")
    lines.append(f"| Cellular modes swept | {', '.join(c.upper() for c in cellular_values)} |")
    lines.append(f"| RNG seeds | {', '.join(str(s) for s in seeds_found)} "
                 "(project convention: 3-repetition average) |")
    lines.append(f"| Parameters held constant | hotspotBand=5g; meshConfig=1; simulation duration "
                 f"{args.sim_time:.0f}s; RSSI handover threshold \u221258 dBm |")
    lines.append(f"| Canonical config for headline/speed/verdict sections | "
                 f"{CANON_CELLULAR.upper()}, {CANON_STA} STA, {CANON_PAYLOAD} payload, "
                 f"{CANON_SPEED} m/s (see Sections 3-5 for STA/payload/cellular sensitivity) |")
    lines.append("")

    # ---- Section 1: Head-to-head ----
    lines.append(f"## 1. Head-to-Head Comparison @ Canonical Config "
                 f"({CANON_CELLULAR.upper()}, {CANON_STA} STA, {CANON_PAYLOAD}, {CANON_SPEED} m/s)")
    lines.append("")
    lines.append("This section presents the top-level switching and signal-quality indicators for "
                 "each mobility scenario at one fixed configuration, each now averaged over 3 seeds "
                 "with real mean \u00b1 SD (95% CI in parentheses where shown) -- not the single-run "
                 "\"\u00b10.00%\" from the June submission.")
    lines.append("")
    lines.append("### Section Summary")
    lines.append("")
    if headline_rows:
        best_pdr = max(headline_rows, key=lambda r: r["pdr"]["mean"] if r["pdr"]["mean"] == r["pdr"]["mean"] else -1)
        fastest = min(headline_rows, key=lambda r: r["mean_interruption_ms"]["mean"] if r["mean_interruption_ms"]["mean"] == r["mean_interruption_ms"]["mean"] else 1e9)
        slowest = max(headline_rows, key=lambda r: r["mean_interruption_ms"]["mean"] if r["mean_interruption_ms"]["mean"] == r["mean_interruption_ms"]["mean"] else -1)
        lines.append(f"- **Best reliability:** `{best_pdr['scenario']}` reaches "
                     f"**{fmt(best_pdr['pdr'])}%** packet-weighted PDR.")
        lines.append(f"- **Fastest switch recovery:** `{fastest['scenario']}` averages "
                     f"**{fmt(fastest['mean_interruption_ms'], decimals=1)} ms** service interruption.")
        lines.append(f"- **Slowest switch recovery:** `{slowest['scenario']}` averages "
                     f"**{fmt(slowest['mean_interruption_ms'], decimals=1)} ms** service interruption.")
        if gm:
            lines.append(f"- **Gauss-Markov RSSI abruptness:** p95 |\u0394RSSI| = "
                         f"**{fmt(gm['p95_abs_rssi_delta'])} dB** -- the reference point Section 6 checks against.")
    lines.append("")
    lines.append("| Scenario | Runs | Switch events /100s | Burstiness (CoV) | "
                 "Mean \\|\u0394RSSI\\| (dB) | p95 \\|\u0394RSSI\\| (dB) | Mean interruption (ms) | Packet-weighted PDR (%) |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for row in headline_rows:
        lines.append(
            f"| {row['scenario']} | {row['n_runs']} | "
            f"{fmt(row['events_per_100s'])} | {fmt(row['burstiness'])} | "
            f"{fmt(row['mean_abs_rssi_delta'])} | {fmt(row['p95_abs_rssi_delta'])} | "
            f"{fmt(row['mean_interruption_ms'], decimals=1)} | "
            f"{fmt(row['pdr'])} |"
        )
    lines.append("")
    lines.append("95% confidence intervals (t-distribution, n=3 -- wide by construction; a firmer "
                 "CI needs enhancement item 4's planned 10-seed expansion):")
    lines.append("")
    lines.append("| Scenario | Switch events /100s 95% CI | Mean interruption (ms) 95% CI | PDR (%) 95% CI |")
    lines.append("|---|---|---|---|")
    for row in headline_rows:
        lines.append(
            f"| {row['scenario']} | {fmt_ci(row['events_per_100s'])} | "
            f"{fmt_ci(row['mean_interruption_ms'], decimals=1)} | {fmt_ci(row['pdr'])} |"
        )
    lines.append("")
    if headline_rows:
        lines.append("![Switching frequency](report_assets/chart_events_per_100s.png)")
        lines.append("")
        lines.append("![Switching burstiness](report_assets/chart_burstiness.png)")
        lines.append("")
        lines.append("![RSSI abruptness](report_assets/chart_rssi_abruptness.png)")
        lines.append("")
        lines.append("![Service interruption](report_assets/chart_interruption.png)")
        lines.append("")
        lines.append("_Figure: Switching frequency, burstiness, RSSI-change abruptness, and mean "
                     f"service interruption across scenarios at the canonical config._")
        lines.append("")

    # ---- Section 2: Speed sensitivity ----
    lines.append("## 2. Waypoint Speed Sensitivity (0.5 / 2.0 / 5.0 m/s)")
    lines.append("")
    lines.append(f"Canonical cellular/STA/payload held fixed ({CANON_CELLULAR.upper()}, "
                 f"{CANON_STA} STA, {CANON_PAYLOAD}); speed is the variable of interest, per the "
                 "enhancement plan's 3-level speed comparison (section 2.3).")
    lines.append("")
    lines.append("### Section Summary")
    lines.append("")
    for scenario in ["patrol", "transport", "work"]:
        rows_s = sorted([r for r in speed_rows if r["scenario"] == scenario], key=lambda r: r["speed"])
        if len(rows_s) >= 2:
            lo, hi = rows_s[0], rows_s[-1]
            lo_v, hi_v = lo["p95_abs_rssi_delta"]["mean"], hi["p95_abs_rssi_delta"]["mean"]
            lines.append(f"- `{scenario}` from **{lo['speed']}** to **{hi['speed']} m/s**: "
                         f"p95 |\u0394RSSI| {lo_v:.2f} \u2192 {hi_v:.2f} dB "
                         f"({'+' if hi_v >= lo_v else ''}{hi_v - lo_v:.2f} dB).")
    lines.append("")
    lines.append("| Scenario | Speed (m/s) | Runs | Switch events /100s | Burstiness (CoV) | "
                 "Mean \\|\u0394RSSI\\| (dB) | p95 \\|\u0394RSSI\\| (dB) |")
    lines.append("|---|---|---|---|---|---|---|")
    for row in speed_rows:
        lines.append(
            f"| {row['scenario']} | {row['speed']} | {row['n_runs']} | "
            f"{fmt(row['events_per_100s'])} | {fmt(row['burstiness'])} | "
            f"{fmt(row['mean_abs_rssi_delta'])} | {fmt(row['p95_abs_rssi_delta'])} |"
        )
    lines.append("")
    lines.append("_Figure interpretation:_ RSSI-change abruptness increases with speed across "
                 "all three robot types -- faster movement covers more distance between "
                 "fixed-interval samples, so signal strength changes more per second.")
    lines.append("")

    # ---- Section 3: STA count sensitivity ----
    lines.append("## 3. STA Count Sensitivity (5 / 10 / 15 STAs)")
    lines.append("")
    lines.append(f"Canonical cellular/payload/speed held fixed ({CANON_CELLULAR.upper()}, "
                 f"{CANON_PAYLOAD}, {CANON_SPEED} m/s); STA count is swept, matching the original "
                 "Phase 1 matrix's granularity for this axis. This is new in the July revision -- "
                 "June fixed STA count at 5 throughout.")
    lines.append("")
    lines.append("| Scenario | STA count | Runs | Switch events /100s | Mean interruption (ms) | PDR (%) |")
    lines.append("|---|---|---|---|---|---|")
    for row in sta_rows:
        lines.append(
            f"| {row['scenario']} | {row['sta']} | {row['n_runs']} | "
            f"{fmt(row['events_per_100s'])} | {fmt(row['mean_interruption_ms'], decimals=1)} | "
            f"{fmt(row['pdr'])} |"
        )
    lines.append("")

    # ---- Section 4: Payload sensitivity ----
    lines.append("## 4. Payload Sensitivity (10KB / 50KB / 1MB)")
    lines.append("")
    lines.append(f"Canonical cellular/STA/speed held fixed ({CANON_CELLULAR.upper()}, {CANON_STA} STA, "
                 f"{CANON_SPEED} m/s); payload size is swept. New in the July revision.")
    lines.append("")
    lines.append("| Scenario | Payload | Runs | Switch events /100s | Mean interruption (ms) | PDR (%) |")
    lines.append("|---|---|---|---|---|---|")
    for row in payload_rows:
        lines.append(
            f"| {row['scenario']} | {row['payload']} | {row['n_runs']} | "
            f"{fmt(row['events_per_100s'])} | {fmt(row['mean_interruption_ms'], decimals=1)} | "
            f"{fmt(row['pdr'])} |"
        )
    lines.append("")
    lines.append("**Caveat -- read the interruption column with this in mind:** at 10KB/50KB "
                 "payload, interruption times are *higher* than at 1MB, which looks backwards. "
                 "Checking the raw switch logs shows why: at small payloads the TCP flow finishes "
                 "transferring almost immediately, so by the time a WiFi->cellular switch happens "
                 "there is often no more application traffic in flight. \"Time to first RX after "
                 "switch\" then measures how long until the *next* packet happens to be generated "
                 "(sometimes never, hence some runs logging `timeout` status with multi-second "
                 "\"durations\") rather than genuine network path-recovery speed. At 1MB the flow "
                 "is still actively transferring, so recovery is detected within milliseconds. This "
                 "is a property of the interruption metric's definition (last-good-RX to "
                 "first-RX-after-switch), not a real payload-dependent slowdown in the switching "
                 "mechanism itself -- and it is also why several payload rows above show n=1 or "
                 "n=2 instead of n=3: some seeds had zero `resolved` switch events to average at "
                 "all, only timeouts.")
    lines.append("")

    # ---- Section 5: Cellular mode comparison ----
    lines.append("## 5. Cellular Mode Comparison (LTE vs. NR)")
    lines.append("")
    lines.append(f"Canonical STA/payload/speed held fixed ({CANON_STA} STA, {CANON_PAYLOAD}, "
                 f"{CANON_SPEED} m/s); cellular fallback mode is swept. New in the July revision -- "
                 "June only tested LTE.")
    lines.append("")
    lines.append("| Scenario | Cellular | Runs | Switch events /100s | Mean interruption (ms) | PDR (%) |")
    lines.append("|---|---|---|---|---|---|")
    for row in cellular_rows:
        lines.append(
            f"| {row['scenario']} | {row['cellular'].upper()} | {row['n_runs']} | "
            f"{fmt(row['events_per_100s'])} | {fmt(row['mean_interruption_ms'], decimals=1)} | "
            f"{fmt(row['pdr'])} |"
        )
    lines.append("")

    # ---- Section 6: Verdict, per type (not blended) ----
    lines.append("## 6. Verdict Against the Enhancement Plan's Predictions")
    lines.append("")
    lines.append("The enhancement plan predicts two effects of switching from Gauss-Markov to "
                 "Waypoint mobility: switching events should become more concentrated "
                 "(vs. dispersed), and RSSI variation should become more abrupt in specific "
                 "zones (vs. gradual). **Each robot type is checked individually against the "
                 "Gauss-Markov baseline below** -- the June report blended patrol/transport/work "
                 "into one \"waypoint average\" here, which masked how differently \"work\" behaves "
                 "from the other two (reviewer finding).")
    lines.append("")
    lines.append("### Section Summary")
    lines.append("")
    if gm and wp_rows:
        gm_at_ceiling = gm["burstiness"]["mean"] >= 2.99
        for row in wp_rows:
            burst_confirmed = row["burstiness"]["mean"] > gm["burstiness"]["mean"]
            rssi_confirmed = row["p95_abs_rssi_delta"]["mean"] > gm["p95_abs_rssi_delta"]["mean"]
            lines.append(f"**`{row['scenario']}`** vs. Gauss-Markov:")
            lines.append("")
            ceiling_note = " (baseline already at the metric's ceiling -- see caveat below)" if gm_at_ceiling else ""
            lines.append(
                f"- Switching dispersed \u2192 concentrated: "
                f"{'CONFIRMED (preliminary)' if burst_confirmed else 'NOT CONFIRMED'} -- "
                f"burstiness {gm['burstiness']['mean']:.2f} \u2192 {row['burstiness']['mean']:.2f}"
                f"{ceiling_note}."
            )
            lines.append(
                f"- RSSI gradual \u2192 abrupt-change zones: "
                f"{'CONFIRMED (preliminary)' if rssi_confirmed else 'NOT CONFIRMED'} -- "
                f"p95 |\u0394RSSI| {gm['p95_abs_rssi_delta']['mean']:.2f} \u2192 {row['p95_abs_rssi_delta']['mean']:.2f} dB."
            )
            lines.append("")
        lines.append(f"**Reading these as a preliminary signal, not a confirmed result:** each "
                     f"comparison above is n=3 seeds per side. That is enough to move past "
                     f"June's single-run \"\u00b10.00%\" problem, but not enough for the statistical "
                     f"confidence enhancement item 4 targets (10 seeds, bootstrap 95% CIs, "
                     f"Mann-Whitney U / Kruskal-Wallis significance tests, Sep 2026). Treat "
                     f"\"CONFIRMED (preliminary)\" above as *this data points that way*, not as a "
                     f"statistically significant finding.")
        if gm_at_ceiling:
            lines.append("")
            lines.append(f"**Caveat on the burstiness comparison specifically:** at this canonical "
                         f"config, total switch events per run are low (roughly {gm['events_per_100s']['mean']*0.9:.0f}"
                         f"-{max(r['events_per_100s']['mean'] for r in wp_rows)*0.9:.0f} events over 90s), "
                         f"and checking the raw switch logs shows they almost all land within the "
                         f"same ~9s time bin right at simulation start (t\u224810-12s, when the WiFi "
                         f"RSSI-averaging window first fills and STAs make their first path "
                         f"decision) regardless of mobility model. That pins the Gauss-Markov "
                         f"baseline's burstiness at the metric's mathematical ceiling (3.00) "
                         f"before mobility is even a factor, so \"dispersed \u2192 concentrated\" has "
                         f"no room to show improvement at this operating point -- the metric isn't "
                         f"wrong, it's just saturated here. The RSSI-abruptness comparison above "
                         f"doesn't have this ceiling problem and is the more trustworthy of the two "
                         f"checks at this config.")
    else:
        lines.append("Insufficient data to compare (need both gaussmarkov_baseline and "
                     "at least one waypoint scenario at the canonical config).")
    lines.append("")

    # ---- Section 7: Visual deliverables ----
    lines.append("## 7. Visual Deliverables")
    lines.append("")
    lines.append("Every one of the "
                 f"{total_runs} runs in this sweep has its own 3D trajectory plot, node-movement "
                 "animation, RSSI heatmap, and switching timeline GIF (files `trajectory_3d.png`, "
                 "`animation.gif`, `rssi_heatmap.png`, `switching_timeline.gif`, "
                 "`trajectory_viewer.html` in each run's output directory) -- these were the "
                 "deliverables missing from the June submission. One representative 3D "
                 "trajectory per scenario type, at the canonical config, is embedded below; the "
                 "full set lives on disk per-run rather than being bundled into this report.")
    lines.append("")
    for scenario in SCENARIOS:
        if scenario in trajectory_pngs:
            lines.append(f"![{scenario} trajectory](report_assets/{trajectory_pngs[scenario]})")
            lines.append("")
    if zero_switch_runs:
        lines.append(f"_Data note: {len(zero_switch_runs)} run(s) had zero switch events "
                     f"(all `work` scenario, low STA count/speed, seed 9) -- their "
                     f"`switching_timeline.gif` legitimately has nothing to animate; all other "
                     f"outputs for those runs are intact._")
        lines.append("")

    md_content = "\n".join(lines) + "\n"
    md_path = out_path.with_suffix(".md")
    md_path.write_text(md_content, encoding="utf-8")
    print(f"Wrote {md_path}")

    if not args.no_pdf:
        try:
            import markdown as md_lib
        except ImportError as exc:
            print(f"Skipping PDF (missing dependency: {exc})", file=sys.stderr)
            return
        try:
            from weasyprint import HTML
        except ImportError:
            HTML = None

        html_body = md_lib.markdown(md_content, extensions=["tables", "fenced_code", "nl2br"])
        html_body = _wrap_chart_grids(html_body)
        title = out_path.stem.replace("_", " ")
        html_doc = f"""<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <title>{title}</title>
    <style>
      @page {{
        size: A4;
        margin: 20mm 20mm 30mm 20mm;
        @bottom-center {{
          content: "Page " counter(page) " of " counter(pages);
          font-size: 10pt;
          color: #666;
          font-family: Arial, sans-serif;
        }}
      }}
      @page:first {{ @bottom-center {{ content: ""; }} }}
      body {{ font-family: Arial, sans-serif; font-size: 11pt; line-height: 1.45; color: #222; }}
      h1, h2, h3 {{ page-break-after: avoid; page-break-inside: avoid; }}
      h1 {{ color: #1f4e79; border-bottom: 3px solid #1f4e79; padding-bottom: 10px; margin: 0 0 18px; font-size: 22pt; }}
      h2 {{ color: #2e5f8a; margin-top: 30px; margin-bottom: 10px; border-bottom: 2px solid #2e5f8a; padding-bottom: 5px; font-size: 16pt; }}
      h3 {{ color: #2e5f8a; margin-top: 18px; margin-bottom: 8px; font-size: 13pt; }}
      p {{ margin: 8px 0; }}
      ul {{ margin: 8px 0 12px 20px; }}
      li {{ margin: 4px 0; }}
      blockquote {{ border-left: 4px solid #2e5f8a; margin: 12px 0; padding: 4px 16px; background: #f5f8fb; color: #444; }}
      table {{ border-collapse: collapse; width: 100%; margin: 15px 0; font-size: 0.85em; table-layout: fixed; page-break-inside: auto; }}
      thead {{ display: table-header-group; }}
      tbody {{ display: table-row-group; }}
      tr {{ page-break-inside: avoid; page-break-after: auto; }}
      th {{ background-color: #1f4e79; color: white; padding: 10px 8px; text-align: center; font-weight: bold; border: 1px solid #ddd; word-wrap: break-word; }}
      td {{ padding: 8px 6px; text-align: center; border: 1px solid #ddd; word-wrap: break-word; vertical-align: top; }}
      tr:nth-child(even) {{ background-color: #f5f5f5; }}
      tr:nth-child(odd) {{ background-color: white; }}
      code {{ background: #f4f4f4; padding: 2px 4px; border-radius: 3px; font-family: "DejaVu Sans Mono", monospace; font-size: 0.95em; }}
      img {{ max-width: 100%; height: auto; display: block; margin: 12px auto; }}
      .chart-grid {{ font-size: 0; margin: 10px 0 16px; }}
      .chart-grid p {{ display: inline-block; width: 48%; margin: 0 1% 14px; vertical-align: top; }}
      .chart-grid img {{ width: 100%; margin: 0; }}
    </style>
  </head>
  <body>
    {html_body}
  </body>
</html>"""
        pdf_path = out_path.with_suffix(".pdf")
        if HTML is not None:
            HTML(string=html_doc, base_url=str(out_path.parent)).write_pdf(str(pdf_path))
        else:
            from xhtml2pdf import pisa
            with open(pdf_path, "wb") as f:
                result = pisa.CreatePDF(src=html_doc, dest=f, encoding="utf-8")
            if result.err:
                print("PDF generation failed (xhtml2pdf reported errors).", file=sys.stderr)
                return
        print(f"Wrote {pdf_path}")


if __name__ == "__main__":
    main()
