#!/usr/bin/env python3
"""
FlowMonitor XML parser for the LTE playfield scenario.

Reads the FlowMonitor XML produced by `lte_playfield_traces.cc`, aggregates per-UE
statistics, prints a concise summary, and optionally writes the output to CSV/Markdown.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple
import xml.etree.ElementTree as ET


DEFAULT_FLOWMON_PATH = Path("Lte_outputs/flowmon-lte-playfield-rw.xml")
TRAFFIC_PORT_LABELS = {
    80: "HTTP",
    443: "HTTPS",
    8080: "Video",
    5060: "VoIP",
    53: "DNS",
}
UPLOAD_PORT_MIN = 51000
UPLOAD_PORT_MAX = 51999
DOWNLOAD_PORT_MIN = 50000
DOWNLOAD_PORT_MAX = 50999


def _protocol_name(proto: str | None) -> str:
    if proto == "6":
        return "TCP"
    if proto == "17":
        return "UDP"
    if not proto:
        return "N/A"
    return proto


def _classify_traffic(meta: dict) -> str:
    try:
        dst_port = int(meta.get("destinationPort") or 0)
    except ValueError:
        dst_port = 0
    try:
        src_port = int(meta.get("sourcePort") or 0)
    except ValueError:
        src_port = 0

    label = TRAFFIC_PORT_LABELS.get(dst_port)

    if label is None:
        if DOWNLOAD_PORT_MIN <= dst_port <= DOWNLOAD_PORT_MAX:
            label = "TCP Download"
        elif UPLOAD_PORT_MIN <= dst_port <= UPLOAD_PORT_MAX:
            label = "TCP Upload"
        elif dst_port != 0:
            label = f"Port {dst_port}"
        elif src_port != 0:
            label = f"Src Port {src_port}"
        else:
            label = "Unknown"

    protocol_name = _protocol_name(meta.get("protocol"))
    if protocol_name != "N/A":
        return f"{label} ({protocol_name})"
    return label


def _extract_numeric(value: str | None) -> float:
    """Convert FlowMonitor time/metric strings to floats."""
    if not value:
        return 0.0

    stripped = []
    for ch in value:
        if ch.isdigit() or ch in ".-+eE":
            stripped.append(ch)
    numeric = "".join(stripped)
    if not numeric:
        return 0.0

    try:
        return float(numeric)
    except ValueError:
        return 0.0


def _extract_seconds(value: str | None) -> float:
    """Convert FlowMonitor time strings with units to seconds."""
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

    return _extract_numeric(v) * scale


def _parse_flowmon(xml_path: Path) -> Tuple[Dict[int, dict], Dict[int, dict]]:
    """Parse FlowMonitor XML and return flow statistics and classifier entries."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    flows: Dict[int, dict] = {}
    for flow_elem in root.findall(".//FlowStats/Flow"):
        flow_id = int(flow_elem.get("flowId"))
        flows[flow_id] = {
            "txBytes": int(flow_elem.get("txBytes", 0)),
            "rxBytes": int(flow_elem.get("rxBytes", 0)),
            "txPackets": int(flow_elem.get("txPackets", 0)),
            "rxPackets": int(flow_elem.get("rxPackets", 0)),
            "lostPackets": int(flow_elem.get("lostPackets", 0)),
            "delaySum": _extract_seconds(flow_elem.get("delaySum")),
            "jitterSum": _extract_seconds(flow_elem.get("jitterSum")),
            "timeFirstTxPacket": _extract_seconds(flow_elem.get("timeFirstTxPacket")),
            "timeLastRxPacket": _extract_seconds(flow_elem.get("timeLastRxPacket")),
        }

    classifier: Dict[int, dict] = {}
    ipv4_classifier = root.find(".//Ipv4FlowClassifier")
    if ipv4_classifier is not None:
        for flow_elem in ipv4_classifier.findall("Flow"):
            flow_id = int(flow_elem.get("flowId"))
            classifier[flow_id] = {
                "sourceAddress": flow_elem.get("sourceAddress"),
                "destinationAddress": flow_elem.get("destinationAddress"),
                "protocol": flow_elem.get("protocol"),
                "sourcePort": flow_elem.get("sourcePort"),
                "destinationPort": flow_elem.get("destinationPort"),
            }

    return flows, classifier


def _ue_index_from_ip(ip: str, prefix: str, offset: int) -> int | None:
    if not ip.startswith(prefix):
        return None
    try:
        last_octet = int(ip.split(".")[-1])
    except ValueError:
        return None
    ue_id = last_octet - offset
    return ue_id if ue_id >= 0 else None


def _aggregate_ue_metrics(
    flows: Dict[int, dict],
    classifier: Dict[int, dict],
    ue_prefix: str,
    ue_offset: int,
) -> Dict[int, dict]:
    metrics: Dict[int, dict] = {}

    for flow_id, flow_info in flows.items():
        meta = classifier.get(flow_id)
        if not meta:
            continue

        ue_id = _ue_index_from_ip(meta.get("sourceAddress", ""), ue_prefix, ue_offset)
        if ue_id is None:
            continue

        entry = metrics.setdefault(
            ue_id,
            {
                "ip": meta["sourceAddress"],
                "protocols": set(),
                "traffic": set(),
                "txPackets": 0,
                "rxPackets": 0,
                "txBytes": 0,
                "rxBytes": 0,
                "lostPackets": 0,
                "delaySum": 0.0,
                "jitterSum": 0.0,
                "firstTx": None,
                "lastRx": None,
            },
        )

        entry["txPackets"] += flow_info["txPackets"]
        entry["rxPackets"] += flow_info["rxPackets"]
        entry["txBytes"] += flow_info["txBytes"]
        entry["rxBytes"] += flow_info["rxBytes"]
        entry["lostPackets"] += flow_info["lostPackets"]
        entry["delaySum"] += flow_info["delaySum"]
        entry["jitterSum"] += flow_info["jitterSum"]

        proto = meta.get("protocol")
        if proto == "6":
            entry["protocols"].add("TCP")
        elif proto == "17":
            entry["protocols"].add("UDP")
        elif proto:
            entry["protocols"].add(proto)

        traffic_label = _classify_traffic(meta)
        if traffic_label:
            entry["traffic"].add(traffic_label)

        first_tx = flow_info["timeFirstTxPacket"]
        last_rx = flow_info["timeLastRxPacket"]
        if entry["firstTx"] is None or (0.0 < first_tx < entry["firstTx"]):
            entry["firstTx"] = first_tx
        if entry["lastRx"] is None or last_rx > entry["lastRx"]:
            entry["lastRx"] = last_rx

    return metrics


def _format_rows(metrics: Dict[int, dict], sim_time: float | None) -> Iterable[dict]:
    for ue_id in sorted(metrics.keys()):
        entry = metrics[ue_id]
        tx_packets = entry["txPackets"]
        rx_packets = entry["rxPackets"]
        duration = 0.0
        if entry["firstTx"] is not None and entry["lastRx"] is not None:
            duration = max(entry["lastRx"] - entry["firstTx"], 0.0)
        if duration <= 0.0 and sim_time:
            duration = sim_time

        throughput_mbps = (
            entry["rxBytes"] * 8.0 / duration / 1e6 if duration > 0 and entry["rxBytes"] > 0 else 0.0
        )
        avg_jitter_ms = (
            (entry["jitterSum"] / max(rx_packets - 1, 1)) * 1000.0 if rx_packets > 1 else 0.0
        )

        traffic_summary = ", ".join(sorted(entry["traffic"])) if entry["traffic"] else "Unknown"
        protocol_summary = "/".join(sorted(entry["protocols"])) if entry["protocols"] else "N/A"

        yield {
            "ue_id": ue_id,
            "ip": entry["ip"],
            "traffic_types": traffic_summary,
            "l4_protocols": protocol_summary,
            "tx_packets": tx_packets,
            "rx_packets": rx_packets,
            "lost_packets": entry["lostPackets"],
            "pdr_percent": (rx_packets / tx_packets * 100.0) if tx_packets > 0 else 0.0,
            "avg_delay_ms": (entry["delaySum"] / rx_packets * 1000.0) if rx_packets > 0 else 0.0,
            "avg_jitter_ms": avg_jitter_ms,
            "throughput_mbps": throughput_mbps,
        }


def _compute_summary(rows: Iterable[dict]) -> Optional[dict]:
    rows = list(rows)
    totals = {
        "pdr_percent": 0.0,
        "avg_delay_ms": 0.0,
        "avg_jitter_ms": 0.0,
        "throughput_mbps": 0.0,
    }
    counted = 0
    for row in rows:
        if row["tx_packets"] <= 0:
            continue
        totals["pdr_percent"] += row["pdr_percent"]
        totals["avg_delay_ms"] += row["avg_delay_ms"]
        totals["avg_jitter_ms"] += row["avg_jitter_ms"]
        totals["throughput_mbps"] += row["throughput_mbps"]
        counted += 1

    if counted == 0:
        return None

    return {key: value / counted for key, value in totals.items()}


def _print_report(rows: Iterable[dict], summary: Optional[dict]) -> None:
    rows = list(rows)
    if not rows:
        print("No UE flows found in FlowMonitor XML.")
        return

    header = (
        f"{'UE':<4}"
        f"{'IP':<16}"
        f"{'Traffic':<24}"
        f"{'L4':<10}"
        f"{'PDR (%)':>10}"
        f"{'Avg Delay (ms)':>16}"
        f"{'Avg Jit (ms)':>16}"
        f"{'Throughput (Mbps)':>20}"
        f"{'TX Pkts':>10}"
        f"{'RX Pkts':>10}"
        f"{'Lost':>8}"
    )
    print(header)
    print("-" * len(header))

    for row in rows:
        print(
            f"{row['ue_id']:<4}"
            f"{row['ip']:<16}"
            f"{row['traffic_types']:<24}"
            f"{row['l4_protocols']:<10}"
            f"{row['pdr_percent']:>10.2f}"
            f"{row['avg_delay_ms']:>16.2f}"
            f"{row['avg_jitter_ms']:>16.2f}"
            f"{row['throughput_mbps']:>20.4f}"
            f"{row['tx_packets']:>10}"
            f"{row['rx_packets']:>10}"
            f"{row['lost_packets']:>8}"
        )
    print("-" * len(header))
    if summary:
        print(
            f"{'AVG':<4}"
            f"{'':<16}"
            f"{'':<24}"
            f"{'':<10}"
            f"{summary['pdr_percent']:>10.2f}"
            f"{summary['avg_delay_ms']:>16.2f}"
            f"{summary['avg_jitter_ms']:>16.2f}"
            f"{summary['throughput_mbps']:>20.4f}"
            f"{'':>10}"
            f"{'':>10}"
            f"{'':>8}"
        )
    print()


def _write_markdown(rows: Iterable[dict], md_path: Path, summary: Optional[dict]) -> None:
    rows = list(rows)
    if not rows:
        print(f"No data to write to {md_path}")
        return

    with md_path.open("w") as md_file:
        md_file.write("# LTE Per-UE FlowMonitor Metrics\n\n")
        md_file.write(
            "| UE | IP | Traffic Types | L4 Protocols | PDR (%) | Avg Delay (ms) | Avg Jitter (ms) | "
            "Throughput (Mbps) | TX Packets | RX Packets | Lost Packets |\n"
        )
        md_file.write("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n")
        for row in rows:
            md_file.write(
                f"| {row['ue_id']} | {row['ip']} | {row['traffic_types']} | {row['l4_protocols']} | "
                f"{row['pdr_percent']:.2f} | {row['avg_delay_ms']:.2f} | "
                f"{row['avg_jitter_ms']:.2f} | {row['throughput_mbps']:.4f} | {row['tx_packets']} | "
                f"{row['rx_packets']} | {row['lost_packets']} |\n"
            )
        if summary:
            md_file.write(
                f"| **Average** |  |  |  | {summary['pdr_percent']:.2f} | {summary['avg_delay_ms']:.2f} | "
                f"{summary['avg_jitter_ms']:.2f} | {summary['throughput_mbps']:.4f} |  |  |  |\n"
            )
    print(f"Wrote Markdown report to {md_path}")


def _write_csv(rows: Iterable[dict], csv_path: Path) -> None:
    rows = list(rows)
    if not rows:
        print(f"No data to write to {csv_path}")
        return

    fieldnames = [
        "ue_id",
        "ip",
        "traffic_types",
        "l4_protocols",
        "pdr_percent",
        "avg_delay_ms",
        "avg_jitter_ms",
        "throughput_mbps",
        "tx_packets",
        "rx_packets",
        "lost_packets",
    ]
    with csv_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote CSV report to {csv_path}")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--xml",
        type=Path,
        default=DEFAULT_FLOWMON_PATH,
        help="Path to FlowMonitor XML file (default: %(default)s)",
    )
    parser.add_argument(
        "--sim-time",
        type=float,
        default=None,
        help="Simulation time in seconds (used for throughput fallback)",
    )
    parser.add_argument(
        "--ue-prefix",
        default="7.0.0.",
        help="Prefix that identifies UE IPv4 addresses (default: %(default)s)",
    )
    parser.add_argument(
        "--ue-offset",
        type=int,
        default=2,
        help="Offset applied to the last octet to compute UE IDs (default: %(default)s)",
    )
    parser.add_argument(
        "--md",
        type=Path,
        help="Optional path to write Markdown output",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Optional path to write CSV output",
    )
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    xml_path = args.xml
    if not xml_path.exists():
        print(f"Error: FlowMonitor XML not found: {xml_path}", file=sys.stderr)
        return 1

    flows, classifier = _parse_flowmon(xml_path)
    metrics = _aggregate_ue_metrics(flows, classifier, args.ue_prefix, args.ue_offset)
    rows = list(_format_rows(metrics, args.sim_time))
    summary = _compute_summary(rows)

    _print_report(rows, summary)

    if args.md:
        _write_markdown(rows, args.md, summary)
    if args.csv:
        _write_csv(rows, args.csv)

    return 0


if __name__ == "__main__":
    sys.exit(main())


