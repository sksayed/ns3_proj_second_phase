#!/usr/bin/env python3
"""
Per-flow QoS metrics analyzer for the July traffic-model enhancement.

Reads the FlowMonitor XML produced by the `traffic-qos` scenario and reports
metrics separated by robot flow class (Control / Sensor / Video), which are
distinguished by destination-port range and DSCP marking:

    Flow      Transport  DSCP (ToS)      Dst port range
    --------  ---------  --------------  ----------------
    Control   UDP        EF   (0xB8)     52000-52999
    Sensor    TCP        AF31 (0x68)     53000-53999
    Video     TCP        AF41 (0x88)     54000-54999

For each flow class it computes:
  * PDR (rx/tx packets), with Control PDR highlighted (the key robot-safety KPI)
  * Mean end-to-end delay (ms) and P99 latency (ms) from the delay histogram
  * Throughput (Mbps) and per-class throughput share
  * The same metrics split by network path leg (WiFi mesh vs cellular)
  * Switch-interruption statistics separating measured from censored samples

Path-leg split
--------------
A path switch changes the STA's egress interface, so its source address changes
(192.168.x.x on the WiFi hotspot, 7.x.x.x on the LTE/NR bearer). FlowMonitor keys
on the 5-tuple, so one logical application stream appears as two flows -- one per
leg. Summing them yields a blended PDR that hides which leg is actually failing,
so both the blended and per-leg values are reported.

Censored switch interruptions
-----------------------------
The scenario stops waiting for service to resume after `--switch-timeout` seconds.
Events that hit that ceiling are right-censored: their recorded interruption is a
lower bound, not a measurement. They are reported separately rather than averaged
in with real observations.

Usage (from ns-3.45/):
    python3 examples/my-scenarios/flow_metrics.py \
        Wifi_hybrid_outputs/seed6/wifi-hybrid-flowmon_data.xml \
        --sim-time 30 \
        --switch-log Wifi_hybrid_outputs/seed6/wifi-hybrid-switch_log.csv \
        --md Wifi_hybrid_outputs/seed6/traffic-qos-metrics.md
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple
import xml.etree.ElementTree as ET


# DSCP / ToS constants matching traffic_qos.cc
TOS_CONTROL = 46 << 2  # 0xB8 EF
TOS_SENSOR = 26 << 2   # 0x68 AF31
TOS_VIDEO = 34 << 2    # 0x88 AF41

FLOW_CLASSES = ("Control", "Sensor", "Video")

# Network path legs, identified by the STA source-address prefix.
LEG_WIFI = "WiFi mesh"
LEG_CELL = "Cellular"
LEG_OTHER = "Other"
PATH_LEGS = (LEG_WIFI, LEG_CELL)

# QoS targets from the enhancement plan.
#
# The plan states two different latency figures for the Control flow, in two
# different roles, so both are carried and reported:
#   * flow_ms  -- the per-flow requirement in plan section 3.1 ("<= 50 ms (strict)")
#   * sync_ms  -- the project end-to-end sync target referenced in section 3.3
#                 ("the key metric directly tied to the 200 ms target")
# Sensor and Video have a single stated requirement, so both fields match.
QOS_TARGETS = {
    "Control": {"flow_ms": 50.0, "sync_ms": 200.0, "loss_pct": 0.0},
    "Sensor": {"flow_ms": 200.0, "sync_ms": 200.0, "loss_pct": 5.0},
    "Video": {"flow_ms": 500.0, "sync_ms": 500.0, "loss_pct": 10.0},
}

# Plan section 3.3 asks whether the control flow stays continuous "within +/-1 s
# of a switching event"; this is that window.
CONTINUITY_WINDOW_S = 1.0

# An interruption within this margin of the configured timeout is treated as
# having hit the ceiling (the scenario adds a fraction of a switch interval).
CENSOR_MARGIN_S = 0.5


def _opt_float(value: Optional[str]) -> Optional[float]:
    """Parse a CSV cell that may be blank or a sentinel."""
    if value is None:
        return None
    v = str(value).strip()
    if v in ("", "n/a", "N/A", "nan"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def leg_of(src_address: Optional[str]) -> str:
    """Map a flow source address to the network path leg it egressed on."""
    if not src_address:
        return LEG_OTHER
    if src_address.startswith("192.168."):
        return LEG_WIFI
    if src_address.startswith("7."):
        return LEG_CELL
    return LEG_OTHER


def _num(value: Optional[str]) -> float:
    if not value:
        return 0.0
    kept = "".join(c for c in value if c.isdigit() or c in ".-+eE")
    try:
        return float(kept)
    except ValueError:
        return 0.0


def _seconds(value: Optional[str]) -> float:
    if not value:
        return 0.0
    v = value.strip()
    scale = 1.0
    if v.endswith("ns"):
        scale = 1e-9
    elif v.endswith("ms"):
        scale = 1e-3
    elif v.endswith("us"):
        scale = 1e-6
    elif v.endswith("ps"):
        scale = 1e-12
    return _num(v) * scale


@dataclass
class FlowRecord:
    flow_id: int
    proto: str
    src: str
    dst: str
    src_port: int
    dst_port: int
    tx_packets: int
    rx_packets: int
    tx_bytes: int
    rx_bytes: int
    lost_packets: int
    delay_sum: float
    first_tx: float
    last_tx: float
    last_rx: float
    delay_bins: List[Tuple[float, float, int]] = field(default_factory=list)

    def classify(self) -> Optional[str]:
        if 52000 <= self.dst_port <= 52999:
            return "Control"
        if 53000 <= self.dst_port <= 53999:
            return "Sensor"
        if 54000 <= self.dst_port <= 54999:
            return "Video"
        return None

    @property
    def leg(self) -> str:
        return leg_of(self.src)

    @property
    def dark_seconds(self) -> float:
        """Time this flow kept transmitting after its last successful receive."""
        if self.rx_packets == 0:
            return max(0.0, self.last_tx - self.first_tx)
        return max(0.0, self.last_tx - self.last_rx)


def parse_flowmon(xml_path: Path) -> List[FlowRecord]:
    root = ET.parse(xml_path).getroot()

    # classifier: flowId -> 5-tuple
    tuples: Dict[int, dict] = {}
    clf = root.find(".//Ipv4FlowClassifier")
    if clf is not None:
        for f in clf.findall("Flow"):
            fid = int(f.get("flowId"))
            tuples[fid] = {
                "src": f.get("sourceAddress"),
                "dst": f.get("destinationAddress"),
                "proto": f.get("protocol"),
                "src_port": int(f.get("sourcePort") or 0),
                "dst_port": int(f.get("destinationPort") or 0),
            }

    records: List[FlowRecord] = []
    for fe in root.findall(".//FlowStats/Flow"):
        fid = int(fe.get("flowId"))
        meta = tuples.get(fid, {})
        bins: List[Tuple[float, float, int]] = []
        dh = fe.find("delayHistogram")
        if dh is not None:
            for b in dh.findall("bin"):
                start = _num(b.get("start"))
                width = _num(b.get("width"))
                count = int(_num(b.get("count")))
                bins.append((start, width, count))
        records.append(
            FlowRecord(
                flow_id=fid,
                proto=meta.get("proto", ""),
                src=meta.get("src", ""),
                dst=meta.get("dst", ""),
                src_port=meta.get("src_port", 0),
                dst_port=meta.get("dst_port", 0),
                tx_packets=int(fe.get("txPackets", 0)),
                rx_packets=int(fe.get("rxPackets", 0)),
                tx_bytes=int(fe.get("txBytes", 0)),
                rx_bytes=int(fe.get("rxBytes", 0)),
                lost_packets=int(fe.get("lostPackets", 0)),
                delay_sum=_seconds(fe.get("delaySum")),
                first_tx=_seconds(fe.get("timeFirstTxPacket")),
                last_tx=_seconds(fe.get("timeLastTxPacket")),
                last_rx=_seconds(fe.get("timeLastRxPacket")),
                delay_bins=bins,
            )
        )
    return records


def percentile_from_bins(bins: List[Tuple[float, float, int]], pct: float) -> Optional[float]:
    """Estimate a delay percentile (seconds) from a FlowMonitor delay histogram."""
    total = sum(c for _, _, c in bins)
    if total == 0:
        return None
    target = pct / 100.0 * total
    cumulative = 0
    for start, width, count in bins:
        cumulative += count
        if cumulative >= target:
            # Return the upper edge of the containing bin (conservative).
            return start + width
    return bins[-1][0] + bins[-1][1] if bins else None


@dataclass
class ClassStats:
    tx_packets: int = 0
    rx_packets: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    lost_packets: int = 0
    delay_sum: float = 0.0
    duration: float = 0.0
    flow_count: int = 0
    bins: List[Tuple[float, float, int]] = field(default_factory=list)
    # Flows that kept transmitting long after their last successful receive, i.e.
    # the application was pointed at a path that had already stopped delivering.
    dark_flow_count: int = 0
    dark_seconds_max: float = 0.0

    @property
    def pdr(self) -> float:
        return (self.rx_packets / self.tx_packets * 100.0) if self.tx_packets else 0.0

    @property
    def loss_pct(self) -> float:
        return 100.0 - self.pdr if self.tx_packets else 0.0

    @property
    def mean_delay_ms(self) -> float:
        return (self.delay_sum / self.rx_packets * 1000.0) if self.rx_packets else 0.0

    def p99_ms(self) -> Optional[float]:
        p = percentile_from_bins(self.bins, 99.0)
        return p * 1000.0 if p is not None else None

    def throughput_mbps(self, sim_time: float) -> float:
        span = sim_time if sim_time > 0 else self.duration
        return (self.rx_bytes * 8.0 / span / 1e6) if span > 0 else 0.0


def _collapse_bins(stats: Iterable["ClassStats"]) -> None:
    """Merge duplicate histogram bins accumulated from several flows."""
    for s in stats:
        merged: Dict[Tuple[float, float], int] = {}
        for start, width, count in s.bins:
            key = (round(start, 6), round(width, 6))
            merged[key] = merged.get(key, 0) + count
        s.bins = sorted(((k[0], k[1], v) for k, v in merged.items()))


def _accumulate(s: "ClassStats", r: FlowRecord) -> None:
    s.tx_packets += r.tx_packets
    s.rx_packets += r.rx_packets
    s.tx_bytes += r.tx_bytes
    s.rx_bytes += r.rx_bytes
    s.lost_packets += r.lost_packets
    s.delay_sum += r.delay_sum
    s.duration = max(s.duration, r.last_rx - r.first_tx)
    s.flow_count += 1
    s.bins.extend(r.delay_bins)


def aggregate_by_leg(records: List[FlowRecord]) -> Dict[Tuple[str, str], ClassStats]:
    """Aggregate per (flow class, path leg), so a blended PDR cannot hide a dead leg."""
    stats: Dict[Tuple[str, str], ClassStats] = {}
    for r in records:
        cls = r.classify()
        if cls is None or r.tx_packets == 0:
            continue
        key = (cls, r.leg)
        s = stats.setdefault(key, ClassStats())
        _accumulate(s, r)
        s.dark_seconds_max = max(s.dark_seconds_max, r.dark_seconds)
        if r.dark_seconds > 3.0:
            s.dark_flow_count += 1
    _collapse_bins(stats.values())
    return stats


def aggregate(records: List[FlowRecord]) -> Dict[str, ClassStats]:
    stats = {c: ClassStats() for c in FLOW_CLASSES}
    for r in records:
        cls = r.classify()
        if cls is None:
            continue
        s = stats[cls]
        s.tx_packets += r.tx_packets
        s.rx_packets += r.rx_packets
        s.tx_bytes += r.tx_bytes
        s.rx_bytes += r.rx_bytes
        s.lost_packets += r.lost_packets
        s.delay_sum += r.delay_sum
        s.duration = max(s.duration, r.last_rx - r.first_tx)
        s.flow_count += 1
        # merge histogram bins (assume aligned start/width across same-class flows)
        s.bins.extend(r.delay_bins)
    _collapse_bins(stats.values())
    return stats


def load_switch_windows(path: Path, half_window: float = 1.0) -> List[Tuple[float, float]]:
    """Return (start, end) windows around each switch event."""
    windows: List[Tuple[float, float]] = []
    try:
        with path.open() as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                t = None
                for key in ("triggerTime", "trigger_time", "time", "applyTime"):
                    if key in row and row[key]:
                        t = _num(row[key])
                        break
                if t is not None:
                    windows.append((t - half_window, t + half_window))
    except FileNotFoundError:
        pass
    return windows


@dataclass
class SwitchStats:
    """Switch-recovery statistics with censored samples held separate."""

    total: int = 0
    by_status: Dict[str, int] = field(default_factory=dict)
    measured_ms: List[float] = field(default_factory=list)
    censored_ms: List[float] = field(default_factory=list)
    # Plan section 3.3 continuity check: delay from switch trigger to the first
    # Control packet received on the new path.
    restore_delays_s: List[float] = field(default_factory=list)
    never_restored: int = 0

    @property
    def censored_pct(self) -> float:
        n = len(self.measured_ms) + len(self.censored_ms)
        return 100.0 * len(self.censored_ms) / n if n else 0.0

    @property
    def continuity_total(self) -> int:
        return len(self.restore_delays_s) + self.never_restored

    def continuity_pct(self, window_s: float = CONTINUITY_WINDOW_S) -> float:
        """Share of switch events whose Control flow resumed inside the window.

        Events that never showed a receive count against continuity, since from the
        robot's point of view the control channel did not come back.
        """
        n = self.continuity_total
        if not n:
            return 0.0
        good = sum(1 for d in self.restore_delays_s if d <= window_s)
        return 100.0 * good / n

    def restore_quantile_s(self, pct: float) -> Optional[float]:
        if not self.restore_delays_s:
            return None
        vals = sorted(self.restore_delays_s)
        return vals[min(len(vals) - 1, int(pct / 100.0 * len(vals)))]

    def quantile_ms(self, pct: float) -> Optional[float]:
        """Quantile over measured samples only."""
        if not self.measured_ms:
            return None
        vals = sorted(self.measured_ms)
        idx = min(len(vals) - 1, int(pct / 100.0 * len(vals)))
        return vals[idx]

    def under_ms_pct(self, limit: float) -> float:
        """Share of *all* events (censored included) that recovered within limit."""
        n = len(self.measured_ms) + len(self.censored_ms)
        if not n:
            return 0.0
        return 100.0 * sum(1 for v in self.measured_ms if v <= limit) / n


def analyze_switch_log(path: Path, timeout_s: float = 5.0) -> SwitchStats:
    """Read a switch log, splitting real interruption measurements from ceiling hits."""
    st = SwitchStats()
    ceiling_ms = (timeout_s - CENSOR_MARGIN_S) * 1000.0
    try:
        with path.open(newline="") as fh:
            for row in csv.DictReader(fh):
                st.total += 1
                status = (row.get("status") or "unknown").strip()
                st.by_status[status] = st.by_status.get(status, 0) + 1

                trigger = _opt_float(row.get("trigger_time_s"))
                first_rx = _opt_float(row.get("first_rx_after_switch_s"))
                if trigger is not None:
                    if first_rx is not None and first_rx > 0.0:
                        st.restore_delays_s.append(max(0.0, first_rx - trigger))
                    else:
                        st.never_restored += 1

                raw = row.get("service_interruption_ms") or row.get("serviceInterruptionMs")
                if raw is None or str(raw).strip() == "":
                    continue
                try:
                    val = float(raw)
                except ValueError:
                    continue
                # A value at or beyond the wait ceiling is a lower bound, not a
                # measurement -- regardless of the status the scenario recorded.
                if val >= ceiling_ms:
                    st.censored_ms.append(val)
                else:
                    st.measured_ms.append(val)
    except FileNotFoundError:
        pass
    return st


def build_markdown(
    stats: Dict[str, ClassStats],
    leg_stats: Dict[Tuple[str, str], ClassStats],
    switch_stats: SwitchStats,
    sim_time: float,
    switch_windows: List[Tuple[float, float]],
    xml_path: Path,
) -> str:
    total_rx_bytes = sum(s.rx_bytes for s in stats.values()) or 1
    lines: List[str] = []
    lines.append("# Traffic QoS Per-Flow Metrics\n")
    lines.append(f"Source: `{xml_path}`  |  Sim time: {sim_time:.1f} s\n")
    lines.append("")
    lines.append("## Per-flow QoS summary\n")
    lines.append(
        "| Flow | Proto | Flows | PDR (%) | Loss (%) | Mean delay (ms) | "
        "P99 latency (ms) | Throughput (Mbps) | Tput share (%) | "
        "Flow target (\u00a73.1) | Sync target (\u00a73.3) | vs flow | vs sync |"
    )
    lines.append(
        "|------|-------|-------|---------|----------|-----------------|"
        "------------------|-------------------|----------------|"
        "--------------------|---------------------|---------|---------|"
    )
    proto_of = {"Control": "UDP", "Sensor": "TCP", "Video": "TCP"}
    for cls in FLOW_CLASSES:
        s = stats[cls]
        p99 = s.p99_ms()
        p99_str = f"{p99:.1f}" if p99 is not None else "n/a"
        share = s.rx_bytes / total_rx_bytes * 100.0
        target = QOS_TARGETS[cls]
        # Control is judged on worst case (P99); the looser classes on the mean.
        latency = p99 if cls == "Control" else s.mean_delay_ms
        ok_loss = s.loss_pct <= target["loss_pct"] + 1e-9

        def verdict(limit: float) -> str:
            ok_lat = latency is not None and latency <= limit
            return "PASS" if (ok_lat and ok_loss) else "CHECK"

        lines.append(
            f"| {cls} | {proto_of[cls]} | {s.flow_count} | {s.pdr:.2f} | {s.loss_pct:.2f} | "
            f"{s.mean_delay_ms:.2f} | {p99_str} | {s.throughput_mbps(sim_time):.3f} | "
            f"{share:.1f} | \u2264 {target['flow_ms']:.0f} ms | \u2264 {target['sync_ms']:.0f} ms | "
            f"{verdict(target['flow_ms'])} | {verdict(target['sync_ms'])} |"
        )
    lines.append("")
    lines.append(
        "The plan states two latency figures for Control: \u2264 50 ms as the per-flow "
        "requirement (\u00a73.1) and \u2264 200 ms as the project end-to-end sync target "
        "(\u00a73.3). Both verdicts are shown rather than picking one."
    )
    lines.append("")

    # Per-leg breakdown: the blended row above mixes a working leg with a failing one.
    if leg_stats:
        lines.append("## Per-flow QoS by network path leg\n")
        lines.append(
            "Each switch changes the STA's egress interface and therefore its source "
            "address, so FlowMonitor reports one application stream as two flows. The "
            "table above sums them; this table keeps them apart, which is the only way "
            "to see which leg is actually losing packets.\n"
        )
        lines.append(
            "| Path leg | Flow | Sub-flows | Tx pkts | PDR (%) | Mean delay (ms) | "
            "P99 latency (ms) | Throughput (Mbps) | Dark sub-flows |"
        )
        lines.append("|---|---|---|---|---|---|---|---|---|")
        for leg in PATH_LEGS + (LEG_OTHER,):
            for cls in FLOW_CLASSES:
                s = leg_stats.get((cls, leg))
                if s is None or s.tx_packets == 0:
                    continue
                p99 = s.p99_ms()
                p99_str = f"{p99:.1f}" if p99 is not None else "n/a"
                lines.append(
                    f"| {leg} | {cls} | {s.flow_count} | {s.tx_packets} | {s.pdr:.2f} | "
                    f"{s.mean_delay_ms:.2f} | {p99_str} | "
                    f"{s.throughput_mbps(sim_time):.3f} | {s.dark_flow_count} |"
                )
        lines.append("")
        lines.append(
            "*Dark sub-flows* kept transmitting for more than 3 s after their last "
            "successful receive — the application was steered onto a path that had "
            "already stopped delivering."
        )
        lines.append("")

    # Highlighted control KPIs
    c = stats["Control"]
    c_p99 = c.p99_ms()
    lines.append("## Key robot-safety KPIs (Control flow)\n")
    lines.append(f"- Control PDR, both legs blended: **{c.pdr:.2f}%** (target 100%, loss target 0%)")
    for leg in PATH_LEGS:
        s = leg_stats.get(("Control", leg))
        if s is not None and s.tx_packets:
            lines.append(
                f"  - via {leg}: {s.pdr:.2f}% over {s.tx_packets} packets"
            )
    lines.append(
        f"- Control P99 latency: **{('%.1f ms' % c_p99) if c_p99 is not None else 'n/a'}** "
        f"— worst-case criterion for safe-stop / sync decisions "
        f"(\u00a73.1 flow requirement \u2264 50 ms; \u00a73.3 sync target \u2264 200 ms)"
    )
    lines.append(f"- Control mean delay: {c.mean_delay_ms:.2f} ms")
    lines.append("")

    if switch_stats.total:
        ss = switch_stats
        lines.append("## Switch recovery timing\n")
        status_str = ", ".join(f"{k} {v}" for k, v in sorted(ss.by_status.items()))
        lines.append(f"- Switch events: **{ss.total}** ({status_str})")
        lines.append(
            f"- Interruption samples: {len(ss.measured_ms)} measured, "
            f"{len(ss.censored_ms)} censored at the wait ceiling "
            f"({ss.censored_pct:.1f}% censored)"
        )
        for label, pct in (("median", 50.0), ("P90", 90.0), ("P99", 99.0)):
            q = ss.quantile_ms(pct)
            lines.append(
                f"- Measured interruption {label}: "
                f"{('%.0f ms' % q) if q is not None else 'n/a'}"
            )
        lines.append(
            f"- Recovered within 200 ms: **{ss.under_ms_pct(200.0):.1f}%** of all events"
        )
        lines.append(
            "- Censored events never showed service resuming before the scenario "
            "stopped waiting, so their recorded duration is a lower bound. Averaging "
            "them together with measured values would understate both the good and "
            "the bad cases."
        )
        lines.append("")

    if switch_stats.continuity_total:
        ss = switch_stats
        lines.append(
            f"## Control-flow continuity across switching (\u00b1{CONTINUITY_WINDOW_S:.0f} s)\n"
        )
        lines.append(
            "Plan \u00a73.3 asks whether the control flow stays continuous within "
            f"\u00b1{CONTINUITY_WINDOW_S:.0f} s of a switching event. This is measured per "
            "event as the delay from the switch trigger to the first Control packet "
            "received on the new path.\n"
        )
        lines.append(
            f"- Switch events assessed: **{ss.continuity_total}**"
        )
        lines.append(
            f"- Control flow restored within {CONTINUITY_WINDOW_S:.0f} s: "
            f"**{ss.continuity_pct():.1f}%**"
        )
        for label, pct in (("median", 50.0), ("P90", 90.0), ("P99", 99.0)):
            q = ss.restore_quantile_s(pct)
            lines.append(
                f"- Restore delay {label}: "
                f"{('%.0f ms' % (q * 1000.0)) if q is not None else 'n/a'}"
            )
        lines.append(
            f"- Never showed a receive on the new path: {ss.never_restored} "
            f"({100.0 * ss.never_restored / ss.continuity_total:.1f}%) — counted as a "
            "continuity failure"
        )
        lines.append("")

    if switch_windows:
        lines.append(
            f"- Windows derived from {len(switch_windows)} switch event(s) in the log. "
            "FlowMonitor aggregates over the whole run, so the switch-log timings above "
            "are the authoritative per-event source."
        )
        lines.append("")

    lines.append("## Per-flow throughput share\n")
    for cls in FLOW_CLASSES:
        s = stats[cls]
        share = s.rx_bytes / total_rx_bytes * 100.0
        lines.append(f"- {cls}: {share:.1f}% ({s.throughput_mbps(sim_time):.3f} Mbps)")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Per-flow QoS metrics for traffic-qos scenario")
    ap.add_argument("flowmon_xml", type=Path, help="FlowMonitor XML file")
    ap.add_argument("--sim-time", type=float, default=30.0, help="Simulation time (s)")
    ap.add_argument("--switch-log", type=Path, default=None, help="Optional switch log CSV")
    ap.add_argument("--md", type=Path, default=None, help="Write Markdown report to this path")
    ap.add_argument(
        "--switch-timeout",
        type=float,
        default=5.0,
        help="switchTimeoutSec used by the run; interruptions at this ceiling are censored",
    )
    ap.add_argument(
        "--leg-csv",
        type=Path,
        default=None,
        help="Write the per-path-leg breakdown as CSV to this path",
    )
    args = ap.parse_args()

    if not args.flowmon_xml.exists():
        print(f"ERROR: {args.flowmon_xml} not found", file=sys.stderr)
        return 1

    records = parse_flowmon(args.flowmon_xml)
    stats = aggregate(records)
    leg_stats = aggregate_by_leg(records)
    windows = load_switch_windows(args.switch_log) if args.switch_log else []
    switch_stats = (
        analyze_switch_log(args.switch_log, args.switch_timeout)
        if args.switch_log
        else SwitchStats()
    )

    report = build_markdown(
        stats, leg_stats, switch_stats, args.sim_time, windows, args.flowmon_xml
    )
    print(report)

    if args.md:
        args.md.parent.mkdir(parents=True, exist_ok=True)
        args.md.write_text(report)
        print(f"\nWrote Markdown report to {args.md}")

    if args.leg_csv:
        args.leg_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.leg_csv.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(
                [
                    "leg", "flow", "sub_flows", "tx_packets", "rx_packets", "pdr_pct",
                    "mean_delay_ms", "p99_ms", "throughput_mbps", "dark_sub_flows",
                ]
            )
            for (cls, leg), s in sorted(leg_stats.items()):
                p99 = s.p99_ms()
                w.writerow(
                    [
                        leg, cls, s.flow_count, s.tx_packets, s.rx_packets,
                        f"{s.pdr:.4f}", f"{s.mean_delay_ms:.4f}",
                        "" if p99 is None else f"{p99:.4f}",
                        f"{s.throughput_mbps(args.sim_time):.6f}", s.dark_flow_count,
                    ]
                )
        print(f"Wrote per-leg CSV to {args.leg_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
