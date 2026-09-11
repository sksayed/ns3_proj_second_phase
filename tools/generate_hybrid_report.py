#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import html
import math
import re
import statistics
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import matplotlib  # type: ignore[reportMissingImports]

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # type: ignore[reportMissingImports]
    from matplotlib.patches import Patch  # type: ignore[reportMissingImports]

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS_ROOT = PROJECT_ROOT / "hybrid_test_results_updated"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "analysis_report_updated"

REPORT_TITLE = "Hybrid Comparison Report Phase - 1"
REPORT_AUTHOR_LINE = "Author : Sheikh Sayed Bin Rahman"
REPORT_ID_LINE = "ID: 2025210714"
REPORT_LAB_LINE = "Lab: PIC Lab , KIT"

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

PAYLOAD_ORDER = {"10kb": 0, "50kb": 1, "1mb": 2, "2mb": 3}
MAX_SEED_STABILITY_ROWS_IN_REPORT = 10


@dataclass
class SwitchEvent:
    sta: int
    src: str
    dst: str
    last_ok_s: float | None
    recovered_s: float | None
    latency_ms: float | None
    rssi_dbm: float | None


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
    # NOTE: packet-weighted PDR is intentionally not computed anymore.
    # Kept for backward compatibility with older report schemas.
    packet_weighted_pdr_pct: float | None
    avg_delay_ms: float | None
    avg_jitter_ms: float | None
    avg_throughput_mbps: float | None
    active_clients: int | None
    delay_target_ratio_pct: float | None
    delay_target_hits: int | None
    delay_target_total: int | None
    switch_events: int
    switch_to_cellular: int
    switch_to_wifi: int
    switch_resolved: int
    switch_timeout: int
    switch_unresolved: int
    avg_resolved_switch_latency_ms: float | None
    median_switch_latency_ms: float | None
    max_switch_latency_ms: float | None
    switch_latency_target_ok_ratio_pct: float | None
    mean_wifi_rssi_dbm: float | None
    min_wifi_rssi_dbm: float | None
    max_wifi_rssi_dbm: float | None
    mean_cellular_rsrp_dbm: float | None
    min_cellular_rsrp_dbm: float | None
    max_cellular_rsrp_dbm: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a new hybrid comparison report from per-scenario markdown outputs."
    )
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--report-name", default="hybrid_comparison_report")
    parser.add_argument("--no-charts", action="store_true")
    parser.add_argument("--html", action="store_true", help="Also emit a simple HTML report.")
    return parser.parse_args()


def safe_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def fmt(value: float | None, digits: int = 2) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "N/A"
    return f"{value:.{digits}f}"


def mean_or_none(values: list[float | None]) -> float | None:
    valid = [v for v in values if v is not None and not math.isnan(v)]
    if not valid:
        return None
    return statistics.fmean(valid)


def median_or_none(values: list[float | None]) -> float | None:
    valid = [v for v in values if v is not None and not math.isnan(v)]
    if not valid:
        return None
    return statistics.median(valid)


def std_or_none(values: list[float | None]) -> float | None:
    valid = [v for v in values if v is not None and not math.isnan(v)]
    if len(valid) < 2:
        return None
    return statistics.stdev(valid)


def fmt_delta(value: float | None, digits: int = 2) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "N/A"
    sign = "+" if value > 0 else ""
    return f"{sign}{value:.{digits}f}"


def mean_attr(records: list[ScenarioRecord], attr: str) -> float | None:
    return mean_or_none([getattr(record, attr) for record in records])


def valid_attr_values(records: list[ScenarioRecord], attr: str) -> list[float]:
    values = [getattr(record, attr) for record in records]
    return [value for value in values if value is not None and not math.isnan(value)]


def aggregate_switch_success_rate(records: list[ScenarioRecord]) -> float | None:
    total_switches = sum(record.switch_events for record in records)
    total_resolved = sum(record.switch_resolved for record in records)
    return 100.0 * total_resolved / total_switches if total_switches else None


def ci95_bounds(values: list[float]) -> tuple[float | None, float | None]:
    if len(values) < 2:
        return None, None
    mean_value = statistics.fmean(values)
    margin = 1.96 * statistics.stdev(values) / math.sqrt(len(values))
    return mean_value - margin, mean_value + margin


def fmt_ci95(values: list[float], digits: int = 2) -> str:
    low, high = ci95_bounds(values)
    if low is None or high is None:
        return "N/A"
    return f"[{low:.{digits}f}, {high:.{digits}f}]"


def group_records(
    records: list[ScenarioRecord], key_fn: Any
) -> dict[Any, list[ScenarioRecord]]:
    grouped: dict[Any, list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[key_fn(record)].append(record)
    return grouped


def mode_display(mode: str) -> str:
    return {"lte": "WiFi+LTE", "nr": "WiFi+5G"}.get(mode, mode.upper())


def mode_chart_label(mode: str) -> str:
    """Short axis/legend labels for plots (nr → 5G, not NR)."""
    return {"lte": "LTE", "nr": "5G"}.get(mode, mode.upper())


def band_display(band: str) -> str:
    return {"2g": "2.4 GHz", "5g": "5 GHz"}.get(band, band)


def pick_better_mode_label(lte_value: float, nr_value: float, *, prefer_lower: bool) -> str:
    """Return the display name of the mode with the better KPI (ties → combined label)."""
    if prefer_lower:
        if lte_value < nr_value:
            return mode_display("lte")
        if nr_value < lte_value:
            return mode_display("nr")
    else:
        if lte_value > nr_value:
            return mode_display("lte")
        if nr_value > lte_value:
            return mode_display("nr")
    return f"{mode_display('lte')} / {mode_display('nr')} (tie)"


def report_metadata_header_lines(records: list[ScenarioRecord]) -> list[str]:
    """Title block: author/lab and how len(records) factorizes over matrix dimensions."""
    lines = [
        f"# {REPORT_TITLE}",
        "",
        f"- {REPORT_AUTHOR_LINE}",
        f"- {REPORT_ID_LINE}",
        f"- {REPORT_LAB_LINE}",
    ]
    n = len(records)
    lines.append(f"- **Scenarios parsed:** {n}")
    if n == 0:
        lines.append("")
        return lines

    dim_specs: list[tuple[str, str, set[Any]]] = [
        ("cellular mode (LTE vs 5G)", "cellular mode", {r.mode for r in records}),
        ("WiFi band (2.4 GHz vs 5 GHz)", "WiFi band", {r.band for r in records}),
        ("STA count", "STA count", {r.sta for r in records}),
        ("payload size", "payload size", {r.payload for r in records}),
        ("RNG seed", "RNG seed", {r.seed for r in records}),
        ("RSSI threshold (dBm)", "RSSI threshold", {r.threshold_dbm for r in records}),
        ("hysteresis (dB)", "hysteresis", {r.hysteresis_db for r in records}),
        ("PDR trigger tag", "PDR trigger tag", {r.pdr_trigger_tag for r in records}),
        ("STA speed (m/s)", "STA speed", {r.speed_mps for r in records}),
        ("simulation time (s)", "simulation time", {r.sim_time_s for r in records}),
    ]
    varying = [(long_l, short_l, len(values)) for long_l, short_l, values in dim_specs if len(values) > 1]
    product = math.prod(c for _, _, c in varying) if varying else 1
    product_str = " × ".join(str(c) for _, _, c in varying)
    phrase_map = {
        "cellular mode": "cellular mode",
        "WiFi band": "WiFi band",
        "STA count": "STA count",
        "payload size": "payload size",
        "RNG seed": "RNG seed",
        "RSSI threshold": "RSSI threshold",
        "hysteresis": "hysteresis",
        "PDR trigger tag": "PDR trigger tag",
        "STA speed": "STA speed",
        "simulation time": "simulation time",
    }
    dim_phrases = [phrase_map.get(short_l, short_l.lower()) for _, short_l, _ in varying]
    if len(dim_phrases) <= 2:
        combo = " and ".join(dim_phrases)
    else:
        combo = ", ".join(dim_phrases[:-1]) + f", and {dim_phrases[-1]}"
    lines.append(
        f"- **How this count arises:** {n} = **{product_str}** "
        f"(one scenario per combination of {combo})."
    )
    sta_sorted = ", ".join(str(s) for s in sorted({r.sta for r in records}))
    payloads_sorted = ", ".join(
        sorted({r.payload for r in records}, key=lambda p: PAYLOAD_ORDER.get(p, 99))
    )
    seeds_sorted = ", ".join(str(s) for s in sorted({r.seed for r in records}))
    lines.append(
        f"- **Levels in this dataset:** STA ∈ {{{sta_sorted}}}; payloads {{{payloads_sorted}}}; seeds {{{seeds_sorted}}}."
    )
    fixed_bits: list[str] = []
    th = {r.threshold_dbm for r in records}
    if len(th) == 1:
        fixed_bits.append(f"RSSI threshold {next(iter(th))} dBm")
    hy = {r.hysteresis_db for r in records}
    if len(hy) == 1:
        fixed_bits.append(f"hysteresis {next(iter(hy))} dB")
    pt = {r.pdr_trigger_tag for r in records}
    if len(pt) == 1:
        fixed_bits.append(f"PDR tag `{next(iter(pt))}`")
    sp = {r.speed_mps for r in records}
    if len(sp) == 1:
        fixed_bits.append(f"STA speed {next(iter(sp))} m/s")
    st = {r.sim_time_s for r in records}
    if len(st) == 1:
        fixed_bits.append(f"simulation time {next(iter(st))} s")
    if fixed_bits:
        lines.append("- **Held constant across all runs:** " + ", ".join(fixed_bits) + ".")
    if product != n:
        lines.append(
            f"- _Note: multiplying the varying dimensions gives **{product}**, not {n}; "
            "the results tree may omit some factorial cells or contain extras._"
        )
    lines.append("")
    return lines


def _md_bold_to_html(fragment: str) -> str:
    """Turn **...** spans into <strong> (simple; no nesting)."""
    parts = fragment.split("**")
    out: list[str] = []
    for i, p in enumerate(parts):
        seg = html.escape(p)
        if i % 2 == 1:
            out.append(f"<strong>{seg}</strong>")
        else:
            out.append(seg)
    return "".join(out)


def report_metadata_header_html(records: list[ScenarioRecord]) -> list[str]:
    """Same content as markdown header, as HTML fragments (no outer <body>)."""
    chunks: list[str] = [f"<h1>{html.escape(REPORT_TITLE)}</h1>"]
    for line in report_metadata_header_lines(records)[2:]:  # skip repeated title + blank
        s = line.strip()
        if not s:
            continue
        if s.startswith("- "):
            inner = s[2:].strip()
            chunks.append(f"<p>{_md_bold_to_html(inner)}</p>")
    return chunks


def payload_sort_key(payload: str) -> int:
    return PAYLOAD_ORDER.get(payload, 99)


def render_subsection(title: str, headers: list[str], rows: list[list[Any]]) -> list[str]:
    lines = [f"### {title}", ""]
    if rows:
        lines.append(render_markdown_table(headers, rows))
    else:
        lines.append("_No data available for this subsection._")
    lines.append("")
    return lines


def chart_caption(title: str, note: str) -> str:
    explicit = {
        "1. Hybrid KPI Summary": "Overall KPI levels and switching outcomes aggregated across all parsed scenarios.",
        "2. Mode Comparison: WiFi+LTE vs WiFi+5G": "Aggregate WiFi+LTE versus WiFi+5G comparison across reliability, latency, throughput, and radio quality.",
        "3. Scalability by STA Count": "KPI trends as STA count increases from 5 to 30 clients.",
        "4. Payload Impact": "Reliability, throughput, delay, and switching changes as payload increases from 10kb to 2mb.",
        "5. Link Quality Analysis (WiFi RSSI + LTE/5G RSRP)": "Relationship between WiFi RSSI / LTE-5G RSRP conditions and observed performance.",
        "6. Switching Behavior and Recovery": "Switch totals, recovery success, and latency behavior by mode and band.",
        "7. Statistical Stability Across Seeds": "Seed-to-seed variability of delivery, delay, throughput, and switch latency.",
        "8. Key Findings and Best Configuration": "Best-performing scenarios positioned by reliability and delay characteristics.",
    }
    return explicit.get(title, note)


def build_section_summary(title: str, records: list[ScenarioRecord]) -> list[str]:
    lines: list[str] = []
    if title.startswith("1. Hybrid KPI Summary"):
        best_pdr = max(records, key=lambda r: r.avg_pdr_pct or float("-inf"))
        best_delay = min(records, key=lambda r: r.avg_delay_ms or float("inf"))
        best_thr = max(records, key=lambda r: r.avg_throughput_mbps or float("-inf"))
        total_switches = sum(r.switch_events for r in records)
        total_resolved = sum(r.switch_resolved for r in records)
        lines.extend(
            [
                "### Section Summary",
                "",
                f"- **Reliability peak:** `{best_pdr.scenario_dir}` reaches **{fmt(best_pdr.avg_pdr_pct)}%** average PDR.",
                f"- **Lowest delay:** `{best_delay.scenario_dir}` records **{fmt(best_delay.avg_delay_ms)} ms** average delay.",
                f"- **Throughput peak:** `{best_thr.scenario_dir}` reaches **{fmt(best_thr.avg_throughput_mbps)} Mbps** average throughput.",
                f"- **Switching baseline:** **{total_resolved}/{total_switches}** aggregate switch events were resolved across the full dataset.",
                "",
            ]
        )
    elif title.startswith("2. Mode Comparison"):
        grouped = group_records(records, lambda r: r.mode)
        lte = grouped.get("lte", [])
        nr = grouped.get("nr", [])
        if lte and nr:
            lte_pdr = mean_attr(lte, "avg_pdr_pct")
            nr_pdr = mean_attr(nr, "avg_pdr_pct")
            lte_delay = mean_attr(lte, "avg_delay_ms")
            nr_delay = mean_attr(nr, "avg_delay_ms")
            lte_thr = mean_attr(lte, "avg_throughput_mbps")
            nr_thr = mean_attr(nr, "avg_throughput_mbps")
            lte_switch = aggregate_switch_success_rate(lte)
            nr_switch = aggregate_switch_success_rate(nr)
            lines.extend(
                [
                    "### Section Summary",
                    "",
                    f"- **Reliability:** `{mode_display('lte')}` averages **{fmt(lte_pdr)}%** PDR vs **{fmt(nr_pdr)}%** for `{mode_display('nr')}`.",
                    f"- **Latency:** `{mode_display('lte')}` averages **{fmt(lte_delay)} ms** vs **{fmt(nr_delay)} ms** for `{mode_display('nr')}`.",
                    f"- **Throughput:** `{mode_display('nr')}` averages **{fmt(nr_thr)} Mbps** vs **{fmt(lte_thr)} Mbps** for `{mode_display('lte')}`.",
                    f"- **Switch success:** `{mode_display('lte')}` averages **{fmt(lte_switch)}%** vs **{fmt(nr_switch)}%** for `{mode_display('nr')}`.",
                    "",
                ]
            )
    elif title.startswith("3. Scalability"):
        grouped = group_records(records, lambda r: (r.mode, r.sta))
        sta_values = sorted({r.sta for r in records})
        if sta_values:
            min_sta, max_sta = sta_values[0], sta_values[-1]
            lines.extend(["### Section Summary", ""])
            for mode in ("lte", "nr"):
                low = grouped.get((mode, min_sta), [])
                high = grouped.get((mode, max_sta), [])
                if not low or not high:
                    continue
                pdr_change = (mean_attr(high, "avg_pdr_pct") or 0.0) - (
                    mean_attr(low, "avg_pdr_pct") or 0.0
                )
                delay_change = (mean_attr(high, "avg_delay_ms") or 0.0) - (
                    mean_attr(low, "avg_delay_ms") or 0.0
                )
                thr_change = (mean_attr(high, "avg_throughput_mbps") or 0.0) - (
                    mean_attr(low, "avg_throughput_mbps") or 0.0
                )
                lines.append(
                    f"- `{mode_display(mode)}` from **{min_sta}** to **{max_sta}** STAs: PDR {fmt_delta(pdr_change)} pp, throughput {fmt_delta(thr_change)} Mbps, delay {fmt_delta(delay_change)} ms."
                )
            lines.append("")
    elif title.startswith("4. Payload Impact"):
        grouped = group_records(records, lambda r: (r.mode, r.payload))
        payloads = sorted({r.payload for r in records}, key=payload_sort_key)
        if payloads:
            first_payload, last_payload = payloads[0], payloads[-1]
            lines.extend(["### Section Summary", ""])
            for mode in ("lte", "nr"):
                low = grouped.get((mode, first_payload), [])
                high = grouped.get((mode, last_payload), [])
                if not low or not high:
                    continue
                pdr_change = (mean_attr(high, "avg_pdr_pct") or 0.0) - (
                    mean_attr(low, "avg_pdr_pct") or 0.0
                )
                delay_change = (mean_attr(high, "avg_delay_ms") or 0.0) - (
                    mean_attr(low, "avg_delay_ms") or 0.0
                )
                thr_change = (mean_attr(high, "avg_throughput_mbps") or 0.0) - (
                    mean_attr(low, "avg_throughput_mbps") or 0.0
                )
                lines.append(
                    f"- `{mode_display(mode)}` from **{first_payload}** to **{last_payload}**: PDR {fmt_delta(pdr_change)} pp, throughput {fmt_delta(thr_change)} Mbps, delay {fmt_delta(delay_change)} ms."
                )
            lines.append("")
    elif title.startswith("5. Link Quality"):
        grouped = group_records(records, lambda r: (r.mode, r.band))
        lines.extend(["### Section Summary", ""])
        for mode in ("lte", "nr"):
            for metric_attr, metric_label in (
                ("avg_pdr_pct", "PDR"),
                ("avg_resolved_switch_latency_ms", "switch latency"),
            ):
                band_means: list[tuple[str, float | None]] = []
                for band in ("2g", "5g"):
                    items = grouped.get((mode, band), [])
                    if items:
                        band_means.append((band, mean_attr(items, metric_attr)))
                if len(band_means) == 2 and all(value is not None for _, value in band_means):
                    better = max(band_means, key=lambda item: item[1] or float("-inf"))
                    if metric_attr == "avg_resolved_switch_latency_ms":
                        better = min(band_means, key=lambda item: item[1] or float("inf"))
                    lines.append(
                        f"- `{mode_display(mode)}`: better `{metric_label}` is on **{band_display(better[0])}** ({fmt(better[1])}{' ms' if 'latency' in metric_attr else '%'})."
                    )
        lines.append("")
    elif title.startswith("6. Switching Behavior"):
        grouped = group_records(records, lambda r: (r.mode, r.band))
        rows = []
        for key, items in grouped.items():
            success = aggregate_switch_success_rate(items)
            latency = mean_attr(items, "avg_resolved_switch_latency_ms")
            rows.append((key[0], key[1], success, latency))
        if rows:
            best_success = max(rows, key=lambda row: row[2] if row[2] is not None else float("-inf"))
            best_latency = min(rows, key=lambda row: row[3] if row[3] is not None else float("inf"))
            lines.extend(
                [
                    "### Section Summary",
                    "",
                    f"- Best switch success rate appears in `{mode_display(best_success[0])}` on **{band_display(best_success[1])}** at **{fmt(best_success[2])}%**.",
                    f"- Lowest resolved switch latency appears in `{mode_display(best_latency[0])}` on **{band_display(best_latency[1])}** at **{fmt(best_latency[3])} ms**.",
                    "- **Interpretation:** `latency_ms` is treated here as effective handover downtime between the last healthy pre-switch point and recovery on the target link.",
                    "- **Application context:** interactive voice/video often targets sub-200 ms disruption, so the current 200 ms compliance rates indicate that most switches are too slow for seamless real-time experience.",
                    "",
                ]
            )
    elif title.startswith("7. Statistical Stability"):
        grouped = group_records(records, lambda r: (r.mode, r.band, r.sta, r.payload, r.speed_mps))
        variability_rows = []
        for key, items in grouped.items():
            variability_rows.append((key, std_or_none([r.avg_pdr_pct for r in items]), std_or_none([r.avg_delay_ms for r in items])))
        valid = [row for row in variability_rows if row[1] is not None]
        if valid:
            most_stable = min(valid, key=lambda row: row[1] if row[1] is not None else float("inf"))
            least_stable = max(valid, key=lambda row: row[1] if row[1] is not None else float("-inf"))
            lines.extend(
                [
                    "### Section Summary",
                    "",
                    f"- Most stable PDR cluster: `{mode_display(most_stable[0][0])}` / `{band_display(most_stable[0][1])}` / `{most_stable[0][2]} STA` / `{most_stable[0][3]}` / `{most_stable[0][4]} mps` with **{fmt(most_stable[1], 3)}** stddev.",
                    f"- Least stable PDR cluster: `{mode_display(least_stable[0][0])}` / `{band_display(least_stable[0][1])}` / `{least_stable[0][2]} STA` / `{least_stable[0][3]}` / `{least_stable[0][4]} mps` with **{fmt(least_stable[1], 3)}** stddev.",
                    "- **Statistical caution:** this report does not run formal hypothesis tests; small LTE/5G gaps should be interpreted alongside the observed seed-to-seed variability.",
                    "",
                ]
            )
    elif title.startswith("8. Key Findings"):
        headers, rows = section_key_findings(records)
        if rows:
            outlier_note = ""
            try:
                best_recovery_latency = float(rows[2][10])
            except (TypeError, ValueError):
                best_recovery_latency = None  # type: ignore[assignment]
            if best_recovery_latency is not None and best_recovery_latency < 1.0:
                outlier_note = (
                    f"- **Outlier note:** the best measured switch-recovery latency (**{rows[2][10]} ms**) is far below the section-level averages and should be treated as an exceptional case rather than typical behavior."
                )
            lines.extend(
                [
                    "### Section Summary",
                    "",
                    f"- Best reliability scenario: `{rows[0][-1]}` with **{rows[0][7]}%** Mean PDR.",
                    f"- Lowest delay scenario: `{rows[1][-1]}` with **{rows[1][8]} ms** average delay.",
                    f"- Best balanced scenario: `{rows[3][-1]}` with Mean PDR **{rows[3][7]}%** and delay **{rows[3][8]} ms**.",
                    "- **On `avg_switch_latency_ms`:** this is the mean **resolved handover** downtime (timeouts excluded). The **lowest_delay** row is picked only by low **traffic** delay (`avg_delay_ms`); that scenario may have **no** resolved handovers (e.g. all switch attempts time out), so this column can read **N/A** there. Use the **best_switch_recovery** row for the best measured handover latency.",
                    *([outlier_note] if outlier_note else []),
                    "",
                ]
            )
    return lines


def build_metric_comparison_table(
    grouped: dict[Any, list[ScenarioRecord]],
    dimension_values: list[Any],
    key_builder: Any,
    metric_attr: str,
    dimension_label: str,
    metric_label: str,
    digits: int = 2,
    delta_label: str = "delta_5g_minus_lte",
) -> list[str]:
    rows: list[list[Any]] = []
    for value in dimension_values:
        lte_items = grouped.get(key_builder("lte", value), [])
        nr_items = grouped.get(key_builder("nr", value), [])
        lte_value = mean_attr(lte_items, metric_attr) if lte_items else None
        nr_value = mean_attr(nr_items, metric_attr) if nr_items else None
        delta = (nr_value - lte_value) if nr_value is not None and lte_value is not None else None
        rows.append([value, fmt(lte_value, digits), fmt(nr_value, digits), fmt_delta(delta, digits)])
    return render_subsection(
        metric_label,
        [dimension_label, "wifi_lte", "wifi_5g", delta_label],
        rows,
    )


def build_section_breakdowns(title: str, records: list[ScenarioRecord]) -> list[str]:
    lines: list[str] = []
    if title.startswith("2. Mode Comparison"):
        grouped = group_records(records, lambda r: r.mode)
        lte = grouped.get("lte", [])
        nr = grouped.get("nr", [])
        if lte and nr:
            metrics = [
                ("PDR (%)", "avg_pdr_pct", False),
                ("Average delay (ms), lower is better", "avg_delay_ms", True),
                ("Average Throughput (Mbps)", "avg_throughput_mbps", False),
                ("Switch Success Rate (%)", None, False),
                ("Avg switch latency (ms), lower is better", "avg_resolved_switch_latency_ms", True),
            ]
            rows: list[list[Any]] = []
            for label, attr, lower_is_better in metrics:
                if attr is None:
                    lte_samples = [100.0 * r.switch_resolved / r.switch_events for r in lte if r.switch_events]
                    nr_samples = [100.0 * r.switch_resolved / r.switch_events for r in nr if r.switch_events]
                    lte_value = aggregate_switch_success_rate(lte)
                    nr_value = aggregate_switch_success_rate(nr)
                else:
                    lte_samples = valid_attr_values(lte, attr)
                    nr_samples = valid_attr_values(nr, attr)
                    lte_value = mean_attr(lte, attr)
                    nr_value = mean_attr(nr, attr)
                if lte_value is None or nr_value is None:
                    better = "N/A"
                    delta = None
                else:
                    better = pick_better_mode_label(lte_value, nr_value, prefer_lower=lower_is_better)
                    delta = nr_value - lte_value
                rows.append([label, better, fmt(lte_value), fmt_ci95(lte_samples), fmt(nr_value), fmt_ci95(nr_samples), fmt_delta(delta)])
            lines.extend(
                render_subsection(
                    "Focused Metric Comparison",
                    [
                        "metric",
                        "better_mode",
                        "wifi_lte",
                        "wifi_lte_95ci",
                        "wifi_5g",
                        "wifi_5g_95ci",
                        "delta_5g_minus_lte",
                    ],
                    rows,
                )
            )
            lines.extend(
                [
                    "- **Better mode:** for *Average delay* and *Avg switch latency*, the **smaller** mean wins; for PDR, throughput, and switch success, the **larger** mean wins. `delta_5g_minus_lte` is always (WiFi+5G mean − WiFi+LTE mean).",
                    "- Confidence intervals are approximate 95% ranges computed from scenario-level observations; overlapping intervals suggest that small LTE/5G gaps may not be practically meaningful without deeper testing.",
                    "",
                ]
            )
    elif title.startswith("3. Scalability"):
        grouped = group_records(records, lambda r: (r.mode, r.sta))
        sta_values = sorted({r.sta for r in records})
        lines.extend(
            build_metric_comparison_table(
                grouped,
                sta_values,
                lambda mode, value: (mode, value),
                "avg_pdr_pct",
                "sta",
                "Packet Delivery Comparison by STA Count",
                delta_label="delta_5g_minus_lte_pp",
            )
        )
        lines.extend(
            build_metric_comparison_table(
                grouped,
                sta_values,
                lambda mode, value: (mode, value),
                "avg_throughput_mbps",
                "sta",
                "Throughput Comparison by STA Count",
            )
        )
        lines.extend(
            build_metric_comparison_table(
                grouped,
                sta_values,
                lambda mode, value: (mode, value),
                "avg_delay_ms",
                "sta",
                "Delay Comparison by STA Count",
            )
        )
    elif title.startswith("4. Payload Impact"):
        grouped = group_records(records, lambda r: (r.mode, r.payload))
        payloads = sorted({r.payload for r in records}, key=payload_sort_key)
        lines.extend(
            build_metric_comparison_table(
                grouped,
                payloads,
                lambda mode, value: (mode, value),
                "avg_pdr_pct",
                "payload",
                "Packet Delivery Comparison by Payload",
                delta_label="delta_5g_minus_lte_pp",
            )
        )
        lines.extend(
            build_metric_comparison_table(
                grouped,
                payloads,
                lambda mode, value: (mode, value),
                "avg_throughput_mbps",
                "payload",
                "Throughput Comparison by Payload",
            )
        )
        lines.extend(
            build_metric_comparison_table(
                grouped,
                payloads,
                lambda mode, value: (mode, value),
                "avg_delay_ms",
                "payload",
                "Delay Comparison by Payload",
            )
        )
    return lines


def parse_switch_event_table(lines: list[str]) -> list[SwitchEvent]:
    events: list[SwitchEvent] = []
    in_table = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("| STA | From | To |"):
            in_table = True
            continue
        if not in_table:
            continue
        if stripped.startswith("| ---"):
            continue
        if not stripped.startswith("|"):
            break
        cols = [c.strip() for c in stripped.strip("|").split("|")]
        if len(cols) < 8:
            continue
        events.append(
            SwitchEvent(
                sta=int(cols[0]) if cols[0].isdigit() else -1,
                src=cols[1].lower(),
                dst=cols[2].lower(),
                last_ok_s=safe_float(cols[3]),
                recovered_s=safe_float(cols[4]),
                latency_ms=safe_float(cols[5]),
                rssi_dbm=safe_float(cols[6]),
            )
        )
    return events


def parse_metrics_markdown(path: Path) -> dict[str, Any]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    avg_pdr = avg_delay = avg_jitter = avg_thr = None
    packet_weighted_pdr = None
    active_clients = None
    delay_target_ratio_pct = None
    delay_target_hits = None
    delay_target_total = None
    switch_events = switch_to_cellular = switch_to_wifi = 0
    switch_resolved = switch_timeout = switch_unresolved = 0
    avg_switch_latency = None

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("| **Average**"):
            cols = [c.strip() for c in stripped.split("|")]
            try:
                avg_pdr = safe_float(cols[4])
                avg_delay = safe_float(cols[5])
                avg_jitter = safe_float(cols[6])
                avg_thr = safe_float(cols[7])
            except IndexError:
                pass
        elif stripped.startswith("- Active clients:"):
            m = re.search(r"\*\*(\d+)\*\*", stripped)
            if m:
                active_clients = int(m.group(1))
        elif stripped.startswith("- Delay target compliance"):
            m = re.search(r"\*\*(\d+)\/(\d+) \(([\d.]+)%\)\*\*", stripped)
            if m:
                delay_target_hits = int(m.group(1))
                delay_target_total = int(m.group(2))
                delay_target_ratio_pct = safe_float(m.group(3))
        elif stripped.startswith("- Switch events:"):
            m_total = re.search(r"\*\*(\d+)\*\*", stripped)
            m_to_cell = re.search(r"to cellular:\s*(\d+)", stripped)
            m_to_wifi = re.search(r"to WiFi:\s*(\d+)", stripped)
            if m_total:
                switch_events = int(m_total.group(1))
            if m_to_cell:
                switch_to_cellular = int(m_to_cell.group(1))
            if m_to_wifi:
                switch_to_wifi = int(m_to_wifi.group(1))
        elif stripped.startswith("- Switching outcomes:"):
            m = re.search(r"resolved=(\d+)", stripped)
            if m:
                switch_resolved = int(m.group(1))
            m = re.search(r"timeout=(\d+)", stripped)
            if m:
                switch_timeout = int(m.group(1))
            m = re.search(r"unresolved=(\d+)", stripped)
            if m:
                switch_unresolved = int(m.group(1))
        elif stripped.startswith("- Avg resolved switching latency"):
            m = re.search(r"\*\*(.*?)\*\*", stripped)
            if m and m.group(1).strip().upper() != "N/A":
                avg_switch_latency = safe_float(m.group(1))

    switch_records = parse_switch_event_table(lines)
    # Latency stats should exclude timeouts. In the markdown switch table, timeouts
    # are represented with "Recovered (s) = -1.000000".
    resolved_switch_latencies = [
        ev.latency_ms
        for ev in switch_records
        if ev.latency_ms is not None and ev.recovered_s is not None and ev.recovered_s >= 0.0
    ]
    if avg_switch_latency is None:
        avg_switch_latency = mean_or_none(resolved_switch_latencies)

    under_200 = [v for v in resolved_switch_latencies if v is not None and v <= 200.0]
    switch_latency_target_ok_ratio_pct = (
        100.0 * len(under_200) / len(resolved_switch_latencies) if resolved_switch_latencies else None
    )

    return {
        "avg_pdr_pct": avg_pdr,
        # Weighted PDR is no longer calculated; keep field as None for compatibility.
        "packet_weighted_pdr_pct": None,
        "avg_delay_ms": avg_delay,
        "avg_jitter_ms": avg_jitter,
        "avg_throughput_mbps": avg_thr,
        "active_clients": active_clients,
        "delay_target_ratio_pct": delay_target_ratio_pct,
        "delay_target_hits": delay_target_hits,
        "delay_target_total": delay_target_total,
        "switch_events": switch_events,
        "switch_to_cellular": switch_to_cellular,
        "switch_to_wifi": switch_to_wifi,
        "switch_resolved": switch_resolved,
        "switch_timeout": switch_timeout,
        "switch_unresolved": switch_unresolved,
        "avg_resolved_switch_latency_ms": avg_switch_latency,
        "median_switch_latency_ms": median_or_none(resolved_switch_latencies),
        "max_switch_latency_ms": max(resolved_switch_latencies) if resolved_switch_latencies else None,
        "switch_latency_target_ok_ratio_pct": switch_latency_target_ok_ratio_pct,
    }


def parse_metric_summary_table(lines: list[str], metric_name: str) -> list[tuple[int, float, float | None, float | None]]:
    rows: list[tuple[int, float, float | None, float | None]] = []
    current_metric = None
    in_table = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("## "):
            current_metric = stripped[3:].strip()
            in_table = False
            continue
        if current_metric != metric_name:
            continue
        if stripped.startswith("| STA | sta_index | node_id | samples |"):
            in_table = True
            continue
        if not in_table:
            continue
        if stripped.startswith("| ---"):
            continue
        if not stripped.startswith("|"):
            break
        cols = [c.strip() for c in stripped.strip("|").split("|")]
        if len(cols) < 7:
            continue
        sta_index = int(cols[1])
        max_match = re.search(r"(-?[\d.]+)", cols[4])
        min_match = re.search(r"(-?[\d.]+)", cols[5])
        mean_value = safe_float(cols[6])
        rows.append(
            (
                sta_index,
                mean_value if mean_value is not None else math.nan,
                safe_float(max_match.group(1)) if max_match else None,
                safe_float(min_match.group(1)) if min_match else None,
            )
        )
    return rows


def parse_link_quality_reports(scenario_dir: Path) -> dict[str, float | None]:
    wifi_path = scenario_dir / "wifi-hybrid-rssi_log_wifi_report.md"
    combined_path = scenario_dir / "wifi-hybrid-rssi_log_report.md"
    nr_path = scenario_dir / "wifi-hybrid-rssi_log_nr_report.md"

    wifi_mean = wifi_min = wifi_max = None
    if wifi_path.exists():
        wifi_lines = wifi_path.read_text(encoding="utf-8", errors="ignore").splitlines()
        wifi_rows = parse_metric_summary_table(wifi_lines, "wifi_rssi")
        if wifi_rows:
            wifi_mean = mean_or_none([r[1] for r in wifi_rows])
            wifi_max = max((r[2] for r in wifi_rows if r[2] is not None), default=None)
            wifi_min = min((r[3] for r in wifi_rows if r[3] is not None), default=None)

    cellular_metric = "nr_rsrp" if nr_path.exists() else "lte_rsrp"
    source_path = nr_path if nr_path.exists() else combined_path
    cell_mean = cell_min = cell_max = None
    if source_path.exists():
        cell_lines = source_path.read_text(encoding="utf-8", errors="ignore").splitlines()
        cell_rows = parse_metric_summary_table(cell_lines, cellular_metric)
        if cell_rows:
            cell_mean = mean_or_none([r[1] for r in cell_rows])
            cell_max = max((r[2] for r in cell_rows if r[2] is not None), default=None)
            cell_min = min((r[3] for r in cell_rows if r[3] is not None), default=None)

    return {
        "mean_wifi_rssi_dbm": wifi_mean,
        "min_wifi_rssi_dbm": wifi_min,
        "max_wifi_rssi_dbm": wifi_max,
        "mean_cellular_rsrp_dbm": cell_mean,
        "min_cellular_rsrp_dbm": cell_min,
        "max_cellular_rsrp_dbm": cell_max,
    }


def extract_records(results_root: Path) -> list[ScenarioRecord]:
    records: list[ScenarioRecord] = []
    for md_path in results_root.rglob("wifi-hybrid-metrics_data.md"):
        scenario_dir = md_path.parent.name
        match = DIR_RE.match(scenario_dir)
        if not match:
            continue
        meta = match.groupdict()
        metrics = parse_metrics_markdown(md_path)
        radio = parse_link_quality_reports(md_path.parent)
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
                packet_weighted_pdr_pct=None,
                avg_delay_ms=metrics["avg_delay_ms"],
                avg_jitter_ms=metrics["avg_jitter_ms"],
                avg_throughput_mbps=metrics["avg_throughput_mbps"],
                active_clients=metrics["active_clients"],
                delay_target_ratio_pct=metrics["delay_target_ratio_pct"],
                delay_target_hits=metrics["delay_target_hits"],
                delay_target_total=metrics["delay_target_total"],
                switch_events=metrics["switch_events"],
                switch_to_cellular=metrics["switch_to_cellular"],
                switch_to_wifi=metrics["switch_to_wifi"],
                switch_resolved=metrics["switch_resolved"],
                switch_timeout=metrics["switch_timeout"],
                switch_unresolved=metrics["switch_unresolved"],
                avg_resolved_switch_latency_ms=metrics["avg_resolved_switch_latency_ms"],
                median_switch_latency_ms=metrics["median_switch_latency_ms"],
                max_switch_latency_ms=metrics["max_switch_latency_ms"],
                switch_latency_target_ok_ratio_pct=metrics["switch_latency_target_ok_ratio_pct"],
                mean_wifi_rssi_dbm=radio["mean_wifi_rssi_dbm"],
                min_wifi_rssi_dbm=radio["min_wifi_rssi_dbm"],
                max_wifi_rssi_dbm=radio["max_wifi_rssi_dbm"],
                mean_cellular_rsrp_dbm=radio["mean_cellular_rsrp_dbm"],
                min_cellular_rsrp_dbm=radio["min_cellular_rsrp_dbm"],
                max_cellular_rsrp_dbm=radio["max_cellular_rsrp_dbm"],
            )
        )
    return records


def write_csv(path: Path, headers: list[str], rows: list[list[Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)


def render_markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    lines.extend("| " + " | ".join(str(v) for v in row) + " |" for row in rows)
    return "\n".join(lines)


def section_hybrid_kpi_summary(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    total_switches = sum(r.switch_events for r in records)
    total_resolved = sum(r.switch_resolved for r in records)
    total_timeout = sum(r.switch_timeout for r in records)
    rows = [
        [
            "overall",
            len(records),
            fmt(mean_or_none([r.avg_pdr_pct for r in records])),
            fmt(mean_or_none([r.avg_delay_ms for r in records])),
            fmt(mean_or_none([r.avg_jitter_ms for r in records])),
            fmt(mean_or_none([r.avg_throughput_mbps for r in records])),
            total_switches,
            total_resolved,
            total_timeout,
            fmt(100.0 * total_resolved / total_switches if total_switches else None),
            fmt(mean_or_none([r.avg_resolved_switch_latency_ms for r in records])),
            fmt(mean_or_none([r.delay_target_ratio_pct for r in records])),
        ]
    ]
    headers = [
        "scope",
        "scenario_count",
        "avg_pdr_pct",
        "avg_delay_ms",
        "avg_jitter_ms",
        "avg_throughput_mbps",
        "switch_events",
        "resolved",
        "timeout",
        "switch_success_rate_pct",
        "avg_switch_latency_ms",
        "delay_target_compliance_pct",
    ]
    return headers, rows


def section_mode_comparison(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[str, list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[record.mode].append(record)
    rows: list[list[Any]] = []
    for mode in sorted(grouped):
        items = grouped[mode]
        total_switches = sum(r.switch_events for r in items)
        total_resolved = sum(r.switch_resolved for r in items)
        rows.append(
            [
                mode_display(mode),
                len(items),
                fmt(mean_or_none([r.avg_pdr_pct for r in items])),
                fmt(mean_or_none([r.avg_delay_ms for r in items])),
                fmt(mean_or_none([r.avg_throughput_mbps for r in items])),
                total_switches,
                fmt(100.0 * total_resolved / total_switches if total_switches else None),
                fmt(mean_or_none([r.avg_resolved_switch_latency_ms for r in items])),
                fmt(mean_or_none([r.mean_wifi_rssi_dbm for r in items])),
                fmt(mean_or_none([r.mean_cellular_rsrp_dbm for r in items])),
            ]
        )
    headers = [
        "mode",
        "scenario_count",
        "mean_pdr_pct",
        "mean_delay_ms",
        "mean_throughput_mbps",
        "total_switch_events",
        "switch_success_rate_pct",
        "mean_switch_latency_ms",
        "mean_wifi_rssi_dbm",
        "mean_cellular_rsrp_dbm",
    ]
    return headers, rows


def section_scalability(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, int], list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[(record.mode, record.sta)].append(record)
    rows: list[list[Any]] = []
    for key in sorted(grouped):
        items = grouped[key]
        total_switches = sum(r.switch_events for r in items)
        rows.append(
            [
                mode_display(key[0]),
                key[1],
                len(items),
                fmt(mean_or_none([r.avg_pdr_pct for r in items])),
                fmt(mean_or_none([r.avg_delay_ms for r in items])),
                fmt(mean_or_none([r.avg_throughput_mbps for r in items])),
                fmt(mean_or_none([r.avg_resolved_switch_latency_ms for r in items])),
                fmt(total_switches / len(items) if items else None),
            ]
        )
    headers = [
        "mode",
        "sta",
        "scenario_count",
        "mean_pdr_pct",
        "mean_delay_ms",
        "mean_throughput_mbps",
        "mean_switch_latency_ms",
        "mean_switch_events_per_scenario",
    ]
    return headers, rows


def section_payload_impact(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str], list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[(record.mode, record.payload)].append(record)
    rows: list[list[Any]] = []
    for key in sorted(grouped, key=lambda x: (x[0], PAYLOAD_ORDER.get(x[1], 99))):
        items = grouped[key]
        rows.append(
            [
                mode_display(key[0]),
                key[1],
                len(items),
                fmt(mean_or_none([r.avg_pdr_pct for r in items])),
                fmt(mean_or_none([r.avg_delay_ms for r in items])),
                fmt(mean_or_none([r.avg_throughput_mbps for r in items])),
                fmt(mean_or_none([r.avg_resolved_switch_latency_ms for r in items])),
                fmt(mean_or_none([float(r.switch_events) for r in items])),
            ]
        )
    headers = [
        "mode",
        "payload",
        "scenario_count",
        "mean_pdr_pct",
        "mean_delay_ms",
        "mean_throughput_mbps",
        "mean_switch_latency_ms",
        "mean_switch_events",
    ]
    return headers, rows


def section_link_quality(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str], list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[(record.mode, record.band)].append(record)
    rows: list[list[Any]] = []
    for key in sorted(grouped):
        items = grouped[key]
        rows.append(
            [
                mode_display(key[0]),
                band_display(key[1]),
                len(items),
                fmt(mean_or_none([r.mean_wifi_rssi_dbm for r in items])),
                fmt(mean_or_none([r.min_wifi_rssi_dbm for r in items])),
                fmt(mean_or_none([r.mean_cellular_rsrp_dbm for r in items])),
                fmt(mean_or_none([r.min_cellular_rsrp_dbm for r in items])),
                fmt(mean_or_none([r.avg_pdr_pct for r in items])),
                fmt(mean_or_none([r.avg_resolved_switch_latency_ms for r in items])),
            ]
        )
    headers = [
        "mode",
        "band",
        "scenario_count",
        "mean_wifi_rssi_dbm",
        "mean_worst_wifi_rssi_dbm",
        "mean_cellular_rsrp_dbm",
        "mean_worst_cellular_rsrp_dbm",
        "mean_pdr_pct",
        "mean_switch_latency_ms",
    ]
    return headers, rows


def section_switching_behavior(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str], list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[(record.mode, record.band)].append(record)
    rows: list[list[Any]] = []
    for key in sorted(grouped):
        items = grouped[key]
        total_switches = sum(r.switch_events for r in items)
        total_resolved = sum(r.switch_resolved for r in items)
        total_timeout = sum(r.switch_timeout for r in items)
        total_to_cell = sum(r.switch_to_cellular for r in items)
        total_to_wifi = sum(r.switch_to_wifi for r in items)
        rows.append(
            [
                mode_display(key[0]),
                band_display(key[1]),
                len(items),
                total_switches,
                total_to_cell,
                total_to_wifi,
                total_resolved,
                total_timeout,
                fmt(100.0 * total_resolved / total_switches if total_switches else None),
                fmt(mean_or_none([r.avg_resolved_switch_latency_ms for r in items])),
                fmt(median_or_none([r.median_switch_latency_ms for r in items])),
                fmt(mean_or_none([r.max_switch_latency_ms for r in items])),
                fmt(mean_or_none([r.switch_latency_target_ok_ratio_pct for r in items])),
            ]
        )
    headers = [
        "mode",
        "band",
        "scenario_count",
        "total_switches",
        "wifi_to_cellular",
        "cellular_to_wifi",
        "resolved",
        "timeout",
        "switch_success_rate_pct",
        "mean_switch_latency_ms",
        "median_switch_latency_ms",
        "mean_max_switch_latency_ms",
        "switch_latency_le_200ms_pct",
    ]
    return headers, rows


def section_seed_stability(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    grouped: dict[tuple[str, str, int, str, int], list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        grouped[(record.mode, record.band, record.sta, record.payload, record.speed_mps)].append(record)
    rows: list[list[Any]] = []
    for key in sorted(grouped, key=lambda x: (x[0], x[1], x[2], PAYLOAD_ORDER.get(x[3], 99), x[4])):
        items = grouped[key]
        rows.append(
            [
                mode_display(key[0]),
                band_display(key[1]),
                key[2],
                key[3],
                key[4],
                len(items),
                fmt(std_or_none([r.avg_pdr_pct for r in items]), 3),
                fmt(std_or_none([r.avg_delay_ms for r in items]), 3),
                fmt(std_or_none([r.avg_throughput_mbps for r in items]), 3),
                fmt(std_or_none([r.avg_resolved_switch_latency_ms for r in items]), 3),
            ]
        )
    headers = [
        "mode",
        "band",
        "sta",
        "payload",
        "speed_mps",
        "seed_count",
        "pdr_stddev",
        "delay_stddev_ms",
        "throughput_stddev_mbps",
        "switch_latency_stddev_ms",
    ]
    return headers, rows


def scenario_score(record: ScenarioRecord, records: list[ScenarioRecord]) -> float | None:
    pdr_values = [r.avg_pdr_pct for r in records if r.avg_pdr_pct is not None]
    delay_values = [r.avg_delay_ms for r in records if r.avg_delay_ms is not None]
    success_values = [
        100.0 * r.switch_resolved / r.switch_events
        for r in records
        if r.switch_events > 0
    ]
    if (
        record.avg_pdr_pct is None
        or record.avg_delay_ms is None
        or not pdr_values
        or not delay_values
    ):
        return None
    pdr_min, pdr_max = min(pdr_values), max(pdr_values)
    delay_min, delay_max = min(delay_values), max(delay_values)
    pdr_score = 1.0 if pdr_max == pdr_min else (record.avg_pdr_pct - pdr_min) / (pdr_max - pdr_min)
    delay_score = 1.0 if delay_max == delay_min else (delay_max - record.avg_delay_ms) / (delay_max - delay_min)
    success_rate = (100.0 * record.switch_resolved / record.switch_events) if record.switch_events > 0 else 100.0
    success_min = min(success_values) if success_values else 0.0
    success_max = max(success_values) if success_values else 100.0
    success_score = 1.0 if success_max == success_min else (success_rate - success_min) / (success_max - success_min)
    return 0.45 * pdr_score + 0.35 * delay_score + 0.20 * success_score


def section_key_findings(records: list[ScenarioRecord]) -> tuple[list[str], list[list[Any]]]:
    candidates = [r for r in records if r.avg_pdr_pct is not None and r.avg_delay_ms is not None]
    if not candidates:
        return ["objective", "scenario_dir"], []
    best_pdr = max(candidates, key=lambda r: r.avg_pdr_pct or float("-inf"))
    best_delay = min(candidates, key=lambda r: r.avg_delay_ms or float("inf"))
    best_recovery = min(
        [r for r in candidates if r.avg_resolved_switch_latency_ms is not None],
        key=lambda r: r.avg_resolved_switch_latency_ms or float("inf"),
        default=best_delay,
    )
    balanced = max(candidates, key=lambda r: scenario_score(r, candidates) or float("-inf"))
    winners = [
        ("best_reliability", best_pdr),
        ("lowest_delay", best_delay),
        ("best_switch_recovery", best_recovery),
        ("best_balanced", balanced),
    ]
    rows = []
    for label, record in winners:
        rows.append(
            [
                label,
                mode_display(record.mode),
                band_display(record.band),
                record.sta,
                record.payload,
                record.speed_mps,
                record.seed,
                fmt(record.avg_pdr_pct),
                fmt(record.avg_delay_ms),
                fmt(record.avg_throughput_mbps),
                fmt(record.avg_resolved_switch_latency_ms),
                record.scenario_dir,
            ]
        )
    headers = [
        "objective",
        "mode",
        "band",
        "sta",
        "payload",
        "speed_mps",
        "seed",
        "avg_pdr_pct",
        "avg_delay_ms",
        "avg_throughput_mbps",
        "avg_switch_latency_ms",
        "scenario_dir",
    ]
    return headers, rows


def generate_charts(records: list[ScenarioRecord], figures_dir: Path) -> dict[str, str]:
    if not MATPLOTLIB_AVAILABLE:
        return {}
    figures_dir.mkdir(parents=True, exist_ok=True)
    chart_map: dict[str, str] = {}

    # 1. KPI summary
    overall = {
        "PDR": mean_or_none([r.avg_pdr_pct for r in records]) or 0.0,
        "Delay": mean_or_none([r.avg_delay_ms for r in records]) or 0.0,
        "Thr": mean_or_none([r.avg_throughput_mbps for r in records]) or 0.0,
        "SwLat": mean_or_none([r.avg_resolved_switch_latency_ms for r in records]) or 0.0,
    }
    plt.figure(figsize=(7, 4))
    plt.bar(list(overall.keys()), list(overall.values()), color=["#3b82f6", "#ef4444", "#10b981", "#f59e0b"])
    plt.title("Hybrid KPI Summary")
    plt.ylabel("Value")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    p = figures_dir / "section1_hybrid_kpi_summary.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["1. Hybrid KPI Summary"] = f"figures/{p.name}"

    # 2. mode comparison (dual y-axis: PDR % and delay ms must not share one scale)
    modes = sorted({r.mode for r in records})
    pdr_vals = [mean_or_none([r.avg_pdr_pct for r in records if r.mode == m]) or 0.0 for m in modes]
    delay_vals = [mean_or_none([r.avg_delay_ms for r in records if r.mode == m]) or 0.0 for m in modes]
    x = list(range(len(modes)))
    w = 0.36
    fig, ax_pdr = plt.subplots(figsize=(7, 4))
    ax_del = ax_pdr.twinx()
    ax_pdr.bar([i - w / 2 for i in x], pdr_vals, width=w, label="Mean PDR (%)", color="#3b82f6", alpha=0.85)
    ax_del.bar([i + w / 2 for i in x], delay_vals, width=w, label="Mean delay (ms)", color="#ef4444", alpha=0.85)
    ax_pdr.set_xticks(x)
    ax_pdr.set_xticklabels([mode_chart_label(m) for m in modes])
    ax_pdr.set_ylabel("Mean PDR (%)")
    ax_del.set_ylabel("Mean delay (ms)")
    ax_pdr.grid(axis="y", alpha=0.3)
    h1, l1 = ax_pdr.get_legend_handles_labels()
    h2, l2 = ax_del.get_legend_handles_labels()
    ax_pdr.legend(h1 + h2, l1 + l2, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    fig.tight_layout()
    p = figures_dir / "section2_mode_comparison.png"
    fig.savefig(p, dpi=150, bbox_inches="tight")
    plt.close(fig)
    chart_map["2. Mode Comparison: WiFi+LTE vs WiFi+5G"] = f"figures/{p.name}"

    # 3. scalability
    plt.figure(figsize=(7, 4))
    for mode in modes:
        xs = sorted({r.sta for r in records if r.mode == mode})
        ys = [mean_or_none([r.avg_delay_ms for r in records if r.mode == mode and r.sta == sta]) or 0.0 for sta in xs]
        plt.plot(xs, ys, marker="o", label=f"{mode_chart_label(mode)} delay")
    plt.xlabel("STA count")
    plt.ylabel("Mean delay (ms)")
    plt.title("Scalability by STA Count")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    p = figures_dir / "section3_scalability.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["3. Scalability by STA Count"] = f"figures/{p.name}"

    # 4. payload impact
    plt.figure(figsize=(7, 4))
    payloads = sorted({r.payload for r in records}, key=lambda v: PAYLOAD_ORDER.get(v, 99))
    for mode in modes:
        ys = [mean_or_none([r.avg_pdr_pct for r in records if r.mode == mode and r.payload == payload]) or 0.0 for payload in payloads]
        plt.plot(payloads, ys, marker="o", label=f"{mode_chart_label(mode)} PDR")
    plt.xlabel("Payload")
    plt.ylabel("Mean PDR (%)")
    plt.title("Payload Impact")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    p = figures_dir / "section4_payload_impact.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["4. Payload Impact"] = f"figures/{p.name}"

    # 5. link quality
    plt.figure(figsize=(7, 4))
    xs = [r.mean_wifi_rssi_dbm for r in records if r.mean_wifi_rssi_dbm is not None and r.avg_pdr_pct is not None]
    ys = [r.avg_pdr_pct for r in records if r.mean_wifi_rssi_dbm is not None and r.avg_pdr_pct is not None]
    colors = ["#3b82f6" if r.mode == "lte" else "#10b981" for r in records if r.mean_wifi_rssi_dbm is not None and r.avg_pdr_pct is not None]
    plt.scatter(xs, ys, c=colors, alpha=0.55)
    plt.xlabel("Mean WiFi RSSI (dBm)")
    plt.ylabel("Mean PDR (%)")
    plt.title("Link Quality Analysis")
    plt.grid(alpha=0.3)
    legend_modes = sorted({r.mode for r in records if r.mean_wifi_rssi_dbm is not None and r.avg_pdr_pct is not None})
    legend_handles = [
        Patch(facecolor="#3b82f6" if m == "lte" else "#10b981", label=mode_chart_label(m)) for m in legend_modes
    ]
    if legend_handles:
        plt.legend(handles=legend_handles)
    plt.tight_layout()
    p = figures_dir / "section5_link_quality.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["5. Link Quality Analysis (WiFi RSSI + LTE/5G RSRP)"] = f"figures/{p.name}"

    # 6. switching
    plt.figure(figsize=(7, 4))
    success = []
    labels = []
    for mode in modes:
        items = [r for r in records if r.mode == mode]
        total_switches = sum(r.switch_events for r in items)
        total_resolved = sum(r.switch_resolved for r in items)
        labels.append(mode_chart_label(mode))
        success.append(100.0 * total_resolved / total_switches if total_switches else 0.0)
    plt.bar(labels, success, color=["#2563eb", "#059669"][: len(labels)])
    plt.ylim(0, 100)
    plt.ylabel("Switch success rate (%)")
    plt.title("Switching Behavior and Recovery")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    p = figures_dir / "section6_switching_behavior.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["6. Switching Behavior and Recovery"] = f"figures/{p.name}"

    # 7. seed stability
    grouped: dict[tuple[str, str], list[float]] = defaultdict(list)
    config_groups: dict[tuple[str, str, int, str, int], list[ScenarioRecord]] = defaultdict(list)
    for record in records:
        config_groups[(record.mode, record.band, record.sta, record.payload, record.speed_mps)].append(record)
    for (mode, band, _sta, _payload, _speed), items in config_groups.items():
        stdv = std_or_none([r.avg_delay_ms for r in items])
        if stdv is not None:
            grouped[(mode, band)].append(stdv)
    labels = []
    values = []
    for key in sorted(grouped):
        labels.append(f"{mode_chart_label(key[0])}-{band_display(key[1])}")
        values.append(statistics.fmean(grouped[key]))
    plt.figure(figsize=(8, 4))
    plt.bar(labels, values, color="#8b5cf6")
    plt.ylabel("Mean delay stddev across seeds (ms)")
    plt.title("Statistical Stability Across Seeds")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    p = figures_dir / "section7_seed_stability.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["7. Statistical Stability Across Seeds"] = f"figures/{p.name}"

    # 8. key findings / best config
    pts = [r for r in records if r.avg_pdr_pct is not None and r.avg_delay_ms is not None]
    plt.figure(figsize=(7, 4))
    for mode in modes:
        subset = [r for r in pts if r.mode == mode]
        plt.scatter(
            [r.avg_delay_ms for r in subset],
            [r.avg_pdr_pct for r in subset],
            alpha=0.55,
            label=mode_chart_label(mode),
        )
    plt.xlabel("Mean delay (ms)")
    plt.ylabel("Mean PDR (%)")
    plt.title("Key Findings and Best Configuration")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    p = figures_dir / "section8_best_configuration.png"
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    chart_map["8. Key Findings and Best Configuration"] = f"figures/{p.name}"

    return chart_map


def write_markdown_report(
    report_path: Path,
    records: list[ScenarioRecord],
    sections: list[tuple[str, str, list[str], list[list[Any]]]],
    chart_map: dict[str, str] | None = None,
) -> None:
    lines: list[str] = []
    lines.extend(report_metadata_header_lines(records))
    for title, note, headers, rows in sections:
        lines.append(f"## {title}")
        lines.append("")
        if note:
            lines.append(f"- {note}")
            lines.append("")
        lines.extend(build_section_summary(title, records))
        display_rows = rows
        if title.startswith("7. Statistical Stability Across Seeds"):
            lines.append(
                f"- Showing the first **{min(len(rows), MAX_SEED_STABILITY_ROWS_IN_REPORT)}** grouped rows in the report for readability; full stability data remains in the CSV export."
            )
            lines.append("")
            display_rows = rows[:MAX_SEED_STABILITY_ROWS_IN_REPORT]
        if display_rows:
            lines.append(render_markdown_table(headers, display_rows))
        else:
            lines.append("_No data available for this section._")
        if chart_map and title in chart_map:
            lines.append("")
            lines.append(f"![{title}]({chart_map[title]})")
            lines.append("")
            lines.append(f"_Figure: {chart_caption(title, note)}_")
        extra_lines = build_section_breakdowns(title, records)
        if extra_lines:
            lines.append("")
            lines.extend(extra_lines)
        lines.append("")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def write_html_report(
    html_path: Path,
    records: list[ScenarioRecord],
    sections: list[tuple[str, str, list[str], list[list[Any]]]],
    chart_map: dict[str, str] | None = None,
) -> None:
    parts = [
        f"<html><head><meta charset='utf-8'><title>{html.escape(REPORT_TITLE)}</title>",
        "<style>body{font-family:Arial,sans-serif;max-width:1200px;margin:32px auto;padding:0 16px;line-height:1.4}table{border-collapse:collapse;margin:16px 0;width:100%}th,td{border:1px solid #ccc;padding:6px 8px;font-size:13px}th{background:#f3f4f6}img{max-width:100%;margin:16px 0}code{background:#f3f4f6;padding:2px 4px}</style>",
        "</head><body>",
        *report_metadata_header_html(records),
    ]
    for title, note, headers, rows in sections:
        parts.append(f"<h2>{html.escape(title)}</h2>")
        if note:
            parts.append(f"<p>{html.escape(note)}</p>")
        if rows:
            parts.append("<table><thead><tr>" + "".join(f"<th>{html.escape(h)}</th>" for h in headers) + "</tr></thead><tbody>")
            for row in rows:
                parts.append("<tr>" + "".join(f"<td>{html.escape(str(v))}</td>" for v in row) + "</tr>")
            parts.append("</tbody></table>")
        if chart_map and title in chart_map:
            parts.append(f"<img src='{html.escape(chart_map[title])}' alt='{html.escape(title)}'>")
    parts.append("</body></html>")
    html_path.write_text("".join(parts), encoding="utf-8")


def write_master_csv(records: list[ScenarioRecord], path: Path) -> None:
    headers = list(ScenarioRecord.__dataclass_fields__.keys())
    rows: list[list[Any]] = []
    for record in records:
        row = [getattr(record, field) for field in headers]
        if "mode" in headers:
            row[headers.index("mode")] = mode_display(record.mode)
        if "band" in headers:
            row[headers.index("band")] = band_display(record.band)
        rows.append(row)
    write_csv(path, headers, rows)


def main() -> None:
    args = parse_args()
    records = extract_records(args.results_root)
    if not records:
        raise SystemExit(f"No scenario markdown files found under {args.results_root}")

    output_dir = args.output_dir
    report_data_dir = output_dir / "report_data" / args.report_name
    output_dir.mkdir(parents=True, exist_ok=True)
    report_data_dir.mkdir(parents=True, exist_ok=True)

    s1 = section_hybrid_kpi_summary(records)
    s2 = section_mode_comparison(records)
    s3 = section_scalability(records)
    s4 = section_payload_impact(records)
    s5 = section_link_quality(records)
    s6 = section_switching_behavior(records)
    s7 = section_seed_stability(records)
    s8 = section_key_findings(records)

    sections = [
        ("1. Hybrid KPI Summary", "Overall KPI snapshot across all parsed hybrid scenarios.", *s1),
        ("2. Mode Comparison: WiFi+LTE vs WiFi+5G", "Architecture-level comparison aggregated over the full matrix.", *s2),
        ("3. Scalability by STA Count", "How performance and switching behavior evolve as the client count increases.", *s3),
        ("4. Payload Impact", "Effect of larger payload sizes on reliability, latency, throughput, and switching.", *s4),
        ("5. Link Quality Analysis (WiFi RSSI + LTE/5G RSRP)", "Relationship between radio conditions and observed hybrid KPIs.", *s5),
        ("6. Switching Behavior and Recovery", "Switch direction, recovery success, latency, and 200 ms compliance.", *s6),
        ("7. Statistical Stability Across Seeds", "Variability of KPIs across repeated runs with different RNG seeds.", *s7),
        ("8. Key Findings and Best Configuration", "Best configurations for reliability, latency, switching, and balanced performance.", *s8),
    ]

    write_master_csv(records, output_dir / f"{args.report_name}_master.csv")
    for idx, (_title, _note, headers, rows) in enumerate(sections, start=1):
        write_csv(report_data_dir / f"section{idx}.csv", headers, rows)

    chart_map = {}
    if not args.no_charts and MATPLOTLIB_AVAILABLE:
        chart_map = generate_charts(records, output_dir / "figures")

    report_path = output_dir / f"{args.report_name}.md"
    write_markdown_report(report_path, records, sections, chart_map)
    if args.html:
        write_html_report(output_dir / f"{args.report_name}.html", records, sections, chart_map)

    print(f"Wrote report: {report_path}")
    print(f"Wrote master CSV: {output_dir / f'{args.report_name}_master.csv'}")
    print(f"Wrote section CSVs under: {report_data_dir}")
    if chart_map:
        print(f"Wrote figures under: {output_dir / 'figures'}")


if __name__ == "__main__":
    main()
