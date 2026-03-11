#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class Extremum:
    value: float
    time_s: float
    freq_mhz: float
    wifi_avg_rssi_dbm_at_time: float


@dataclass
class MetricAgg:
    metric: str  # e.g. wifi_rssi, lte_rsrp, nr_sinr
    sta_index: int
    node_id: int

    n: int = 0
    sum_value: float = 0.0

    max_v: Optional[Extremum] = None
    min_v: Optional[Extremum] = None

    # histogram bins labeled by "bin top" in dB (step size 3).
    # Each bin covers (top-3, top] in dBm.
    hist: Dict[int, int] = field(default_factory=dict)
    observed_max: float = float("-inf")
    observed_min: float = float("inf")

    def add(
        self,
        time_s: float,
        freq_mhz: float,
        value: float,
        wifi_avg_rssi_dbm_at_time: float,
        bin_step_db: int,
    ) -> None:
        self.n += 1
        self.sum_value += value

        self.observed_max = max(self.observed_max, value)
        self.observed_min = min(self.observed_min, value)

        if self.max_v is None or value > self.max_v.value:
            self.max_v = Extremum(value, time_s, freq_mhz, wifi_avg_rssi_dbm_at_time)
        if self.min_v is None or value < self.min_v.value:
            self.min_v = Extremum(value, time_s, freq_mhz, wifi_avg_rssi_dbm_at_time)

        bin_top = int(bin_step_db * math.ceil(value / bin_step_db))
        self.hist[bin_top] = self.hist.get(bin_top, 0) + 1

    def mean(self) -> float:
        return self.sum_value / self.n if self.n else float("nan")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Parse wifi-hybrid RSSI log CSV and emit per-STA report "
            "(max/min/mean + 3 dB histogram)."
        )
    )
    p.add_argument(
        "-i",
        "--input",
        default="Wifi_hybrid_outputs_nr_kpi/wifi-hybrid-rssi_log.csv",
        help="Path to input CSV (default: %(default)s, relative to ns-3.45/).",
    )
    p.add_argument(
        "-o",
        "--output",
        default="",
        help=(
            "Path to output markdown report. "
            "Default: alongside input with suffix '_report.md'."
        ),
    )
    p.add_argument(
        "--output-csv",
        default="",
        help=(
            "Optional path to output CSV summary. "
            "Default: alongside input with suffix '_report.csv'."
        ),
    )
    p.add_argument(
        "--bin-step-db",
        type=int,
        default=3,
        help="Histogram step size in dB (default: %(default)s).",
    )
    p.add_argument(
        "--use-column",
        choices=["inst_rssi_dbm", "avg_rssi_dbm"],
        default="inst_rssi_dbm",
        help=(
            "Which WiFi RSSI column to summarize (default: %(default)s). "
            "Cellular metrics (RSRP/SINR) are always parsed when present."
        ),
    )
    return p.parse_args()


def _resolve_path(ns345_root: str, maybe_rel: str) -> str:
    if os.path.isabs(maybe_rel):
        return maybe_rel
    return os.path.join(ns345_root, maybe_rel)


def _default_outputs(input_path: str) -> Tuple[str, str]:
    base, ext = os.path.splitext(input_path)
    if ext.lower() != ".csv":
        base = input_path
    return f"{base}_report.md", f"{base}_report.csv"


def _format_float(x: float, nd: int = 4) -> str:
    if math.isnan(x):
        return "nan"
    return f"{x:.{nd}f}"


def _to_float(s: str) -> float:
    try:
        if s is None:
            return float("nan")
        s2 = str(s).strip()
        if s2 == "" or s2.lower() == "nan":
            return float("nan")
        return float(s2)
    except Exception:
        return float("nan")


def _iter_metrics_from_row(row: Dict[str, str], wifi_column: str) -> Iterable[Tuple[str, float]]:
    """
    Yield (metric_name, value) pairs from a row.

    - WiFi: metric 'wifi_rssi' using wifi_column.
    - Cellular (extended format): metric '<rat>_rsrp' from rsrp_dbm, '<rat>_sinr' from sinr_db.
    """
    rat = (row.get("rat") or "").strip().lower()
    if not rat:
        rat = "wifi"

    # WiFi RSSI always available in legacy files.
    wifi_v = _to_float(row.get(wifi_column, "nan"))
    if not math.isnan(wifi_v):
        yield ("wifi_rssi", wifi_v)

    # Extended fields (may not exist in legacy CSV).
    rsrp = _to_float(row.get("rsrp_dbm", "nan"))
    if not math.isnan(rsrp):
        yield (f"{rat}_rsrp", rsrp)

    sinr = _to_float(row.get("sinr_db", "nan"))
    if not math.isnan(sinr):
        yield (f"{rat}_sinr", sinr)


def _write_csv_summary(path: str, aggs: List[MetricAgg]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "metric",
                "sta_index",
                "node_id",
                "sta_label",
                "samples",
                "max_db",
                "max_time_s",
                "min_db",
                "min_time_s",
                "mean_db",
            ]
        )
        for agg in aggs:
            sta_label = f"STA{agg.sta_index + 1}"
            w.writerow(
                [
                    agg.metric,
                    agg.sta_index,
                    agg.node_id,
                    sta_label,
                    agg.n,
                    _format_float(agg.max_v.value if agg.max_v else float("nan")),
                    _format_float(agg.max_v.time_s if agg.max_v else float("nan")),
                    _format_float(agg.min_v.value if agg.min_v else float("nan")),
                    _format_float(agg.min_v.time_s if agg.min_v else float("nan")),
                    _format_float(agg.mean()),
                ]
            )


def _write_markdown_report(
    path: str,
    input_path: str,
    wifi_column: str,
    bin_step_db: int,
    aggs: List[MetricAgg],
    mapping_mismatches: List[Tuple[int, int]],
    title: str,
) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as out:
        out.write(f"# {title}\n\n")
        out.write(f"- Input: `{input_path}`\n")
        out.write(f"- WiFi column summarized: `{wifi_column}`\n")
        out.write(f"- Histogram bin step: {bin_step_db} dB (bins are `(top-step, top]`)\n\n")

        if mapping_mismatches:
            uniq = sorted(set(mapping_mismatches))[:10]
            out.write("## STA identity check\n\n")
            out.write(
                "Observed some rows where `node_id != sta_index + 6` "
                f"(showing up to 10 unique mismatches): {uniq}\n\n"
            )
        else:
            out.write("## STA identity check\n\n")
            out.write("All rows match `node_id == sta_index + 6`.\n\n")

        metrics = sorted({a.metric for a in aggs})
        for metric in metrics:
            subset = [a for a in aggs if a.metric == metric]
            out.write(f"## {metric}\n\n")
            out.write(
                "| STA | sta_index | node_id | samples | max (dB) @ time(s) | min (dB) @ time(s) | mean (dB) |\n"
            )
            out.write("| --- | ---: | ---: | ---: | --- | --- | --- |\n")
            for agg in subset:
                sta_label = f"STA{agg.sta_index + 1}"
                max_s = (
                    f"{_format_float(agg.max_v.value)} @ {_format_float(agg.max_v.time_s)}"
                    if agg.max_v
                    else "nan"
                )
                min_s = (
                    f"{_format_float(agg.min_v.value)} @ {_format_float(agg.min_v.time_s)}"
                    if agg.min_v
                    else "nan"
                )
                out.write(
                    f"| {sta_label} | {agg.sta_index} | {agg.node_id} | {agg.n} | {max_s} | {min_s} | {_format_float(agg.mean())} |\n"
                )

            out.write("\n### Per-STA histograms (highest to lowest)\n\n")
            for agg in subset:
                sta_label = f"STA{agg.sta_index + 1}"
                out.write(
                    f"#### {sta_label} (sta_index={agg.sta_index}, node_id={agg.node_id})\n\n"
                )
                out.write(
                    f"- samples: {agg.n}\n"
                    f"- max: {_format_float(agg.max_v.value)} at {_format_float(agg.max_v.time_s)} s\n"
                    f"- min: {_format_float(agg.min_v.value)} at {_format_float(agg.min_v.time_s)} s\n"
                    f"- mean: {_format_float(agg.mean())}\n"
                )
                if agg.metric == "wifi_rssi" and agg.max_v is not None:
                    out.write(
                        f"- wifi avg_rssi_dbm at max time: {_format_float(agg.max_v.wifi_avg_rssi_dbm_at_time)}\n"
                    )
                out.write("\n")

                if agg.n == 0:
                    out.write("No samples.\n\n")
                    continue

                top = int(bin_step_db * math.ceil(agg.observed_max / bin_step_db))
                bottom = int(bin_step_db * math.floor(agg.observed_min / bin_step_db))
                out.write("| bin (dB) | range (dB) | count |\n")
                out.write("| ---: | --- | ---: |\n")
                b = top
                while b >= bottom:
                    lo = b - bin_step_db
                    cnt = agg.hist.get(b, 0)
                    out.write(f"| {b} | ({lo}, {b}] | {cnt} |\n")
                    b -= bin_step_db
                out.write("\n")


def main() -> int:
    args = parse_args()
    ns345_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    input_path = _resolve_path(ns345_root, args.input)
    if not os.path.exists(input_path):
        raise SystemExit(f"Input CSV not found: {input_path}")

    out_md_default, out_csv_default = _default_outputs(input_path)
    output_md = args.output or out_md_default
    output_csv = args.output_csv or out_csv_default

    aggs: Dict[Tuple[str, int, int], MetricAgg] = {}
    mapping_mismatches: List[Tuple[int, int]] = []

    with open(input_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = [
            "time_s",
            "sta_index",
            "node_id",
            "freq_mhz",
            "inst_rssi_dbm",
            "avg_rssi_dbm",
        ]
        missing = [c for c in required if c not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(f"Missing expected columns in CSV: {missing}")

        for row in reader:
            time_s = float(row["time_s"])
            sta_index = int(row["sta_index"])
            node_id = int(row["node_id"])
            freq_mhz = _to_float(row.get("freq_mhz", "nan"))

            avg_rssi_dbm = _to_float(row.get("avg_rssi_dbm", "nan"))

            # Validate observed identity pattern (common ns-3 wifi layout):
            # sta node IDs start after AP nodes; in the sample, node_id == sta_index + 6.
            if node_id != sta_index + 6:
                mapping_mismatches.append((sta_index, node_id))

            for metric, value in _iter_metrics_from_row(row, args.use_column):
                key = (metric, sta_index, node_id)
                agg = aggs.get(key)
                if agg is None:
                    agg = MetricAgg(metric=metric, sta_index=sta_index, node_id=node_id)
                    aggs[key] = agg

                agg.add(
                    time_s=time_s,
                    freq_mhz=freq_mhz,
                    value=value,
                    wifi_avg_rssi_dbm_at_time=avg_rssi_dbm,
                    bin_step_db=args.bin_step_db,
                )

    # Deterministic ordering by metric then sta_index then node_id
    ordered: List[MetricAgg] = [aggs[k] for k in sorted(aggs.keys())]

    # 1) Combined report (all metrics, backward-compatible with existing usage)
    _write_csv_summary(output_csv, ordered)
    _write_markdown_report(
        output_md,
        input_path,
        args.use_column,
        args.bin_step_db,
        ordered,
        mapping_mismatches,
        "Link quality per-STA report",
    )

    # 2) WiFi-only report
    wifi_aggs = [a for a in ordered if a.metric == "wifi_rssi"]
    wifi_csv = output_csv.replace("_report.csv", "_wifi_report.csv")
    wifi_md = output_md.replace("_report.md", "_wifi_report.md")
    _write_csv_summary(wifi_csv, wifi_aggs)
    _write_markdown_report(
        wifi_md,
        input_path,
        args.use_column,
        args.bin_step_db,
        wifi_aggs,
        mapping_mismatches,
        "WiFi RSSI per-STA report",
    )

    # 3) NR-only report (network condition for NR: RSRP/SINR)
    nr_aggs = [a for a in ordered if a.metric.startswith("nr_")]
    nr_csv = output_csv.replace("_report.csv", "_nr_report.csv")
    nr_md = output_md.replace("_report.md", "_nr_report.md")
    if nr_aggs:
        _write_csv_summary(nr_csv, nr_aggs)
        _write_markdown_report(
            nr_md,
            input_path,
            args.use_column,
            args.bin_step_db,
            nr_aggs,
            mapping_mismatches,
            "NR network condition per-STA report",
        )

    print(f"Wrote markdown report: {output_md}")
    print(f"Wrote CSV summary:    {output_csv}")
    print(f"Wrote WiFi report:    {wifi_md} and {wifi_csv}")
    if nr_aggs:
        print(f"Wrote NR report:      {nr_md} and {nr_csv}")
    else:
        print("NR metrics not found in input; skipped NR-only report.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

