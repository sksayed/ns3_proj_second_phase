#!/usr/bin/env python3
"""
FlowMonitor XML parser for Wi-Fi mesh simulations.

Reads the FlowMonitor XML output produced by `wifi-test-2-adhoc-grid.cc`,
aggregates per-STA metrics, and prints a concise report. Optionally writes
the results to a CSV file for further analysis.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple
import xml.etree.ElementTree as ET


DEFAULT_FLOWMON_PATH = Path("Wifi_hybrid_outputs/wifi-hybrid-flowmon_data.xml")
DEFAULT_SWITCH_LOG = Path("Wifi_hybrid_outputs/wifi-hybrid-switch_log.csv")
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
    """Convert FlowMonitor time/metric strings to floats (seconds/units)."""
    if not value:
        return 0.0

    stripped = []
    for ch in value:
        # Keep digits, decimal markers, exponents, and sign
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


def _aggregate_sta_metrics(
    flows: Dict[int, dict],
    classifier: Dict[int, dict],
    sta_prefix: str = "192.168.1.",
) -> Dict[str, dict]:
    """Aggregate statistics for traffic sourced by STA nodes."""
    sta_metrics: Dict[str, dict] = {}

    for flow_id, flow_info in flows.items():
        metadata = classifier.get(flow_id)
        if not metadata:
            continue

        source_ip = metadata.get("sourceAddress", "")
        if not source_ip.startswith(sta_prefix):
            continue

        metrics = sta_metrics.setdefault(
            source_ip,
            {
                "txPackets": 0,
                "rxPackets": 0,
                "txBytes": 0,
                "rxBytes": 0,
                "lostPackets": 0,
                "delaySum": 0.0,
                "jitterSum": 0.0,
                "protocols": set(),
                "traffic": set(),
                "flows": [],
                "firstTx": None,
                "lastRx": None,
            },
        )

        metrics["txPackets"] += flow_info["txPackets"]
        metrics["rxPackets"] += flow_info["rxPackets"]
        metrics["txBytes"] += flow_info["txBytes"]
        metrics["rxBytes"] += flow_info["rxBytes"]
        metrics["lostPackets"] += flow_info["lostPackets"]
        metrics["delaySum"] += flow_info["delaySum"]
        metrics["jitterSum"] += flow_info["jitterSum"]
        metrics["flows"].append(flow_id)

        proto = metadata.get("protocol")
        if proto == "6":
            metrics["protocols"].add("TCP")
        elif proto == "17":
            metrics["protocols"].add("UDP")
        elif proto:
            metrics["protocols"].add(proto)

        traffic_label = _classify_traffic(metadata)
        if traffic_label:
            metrics["traffic"].add(traffic_label)

        first_tx = flow_info["timeFirstTxPacket"]
        last_rx = flow_info["timeLastRxPacket"]
        if metrics["firstTx"] is None or (0 < first_tx < metrics["firstTx"]):
            metrics["firstTx"] = first_tx
        if metrics["lastRx"] is None or last_rx > metrics["lastRx"]:
            metrics["lastRx"] = last_rx

    return sta_metrics


def _format_rows(sta_metrics: Dict[str, dict], sim_time: float | None) -> Iterable[dict]:
    """Compute derived metrics for presentation."""
    sta_ips = sorted(sta_metrics.keys(), key=lambda ip: tuple(int(octet) for octet in ip.split(".")))

    for sta_ip in sta_ips:
        metrics = sta_metrics[sta_ip]
        tx_packets = metrics["txPackets"]
        rx_packets = metrics["rxPackets"]
        delay_sum = metrics["delaySum"]
        first_tx = metrics["firstTx"] or 0.0
        last_rx = metrics["lastRx"] or first_tx

        duration = max(last_rx - first_tx, 0.0)
        if duration <= 0.0 and sim_time:
            duration = sim_time

        throughput_mbps = 0.0
        if duration > 0:
            throughput_mbps = metrics["rxBytes"] * 8.0 / duration / 1e6

        avg_jitter_ms = (
            (metrics["jitterSum"] / max(rx_packets - 1, 1)) * 1000.0 if rx_packets > 1 else 0.0
        )

        traffic_summary = ", ".join(sorted(metrics["traffic"])) if metrics["traffic"] else "Unknown"
        protocol_summary = "/".join(sorted(metrics["protocols"])) if metrics["protocols"] else "N/A"

        row = {
            "sta_ip": sta_ip,
            "traffic_types": traffic_summary,
            "l4_protocols": protocol_summary,
            "tx_packets": tx_packets,
            "rx_packets": rx_packets,
            "lost_packets": metrics["lostPackets"],
            "pdr_percent": (rx_packets / tx_packets * 100.0) if tx_packets > 0 else 0.0,
            "avg_delay_ms": (delay_sum / rx_packets * 1000.0) if rx_packets > 0 else 0.0,
            "avg_jitter_ms": avg_jitter_ms,
            "throughput_mbps": throughput_mbps,
        }
        yield row


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


def _safe_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _parse_switch_events(switch_log_path: Path) -> List[dict]:
    """Parse switch log (supports legacy and current formats)."""
    if not switch_log_path.exists():
        return []

    events: List[dict] = []
    with switch_log_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            event = {
                "sta_index": int(_safe_float(row.get("sta_index"), -1)),
                "from": (row.get("from") or "").strip(),
                "to": (row.get("to") or "").strip(),
                "last_ok_before_s": _safe_float(row.get("last_ok_before_s"), -1.0),
                "first_rx_after_switch_s": _safe_float(row.get("first_rx_after_switch_s"), -1.0),
                "service_interruption_ms": _safe_float(row.get("service_interruption_ms"), -1.0),
                "status": (row.get("status") or "").strip().lower(),
                "rssi_avg_dbm": _safe_float(row.get("rssi_avg_dbm")),
                "service_ip": (row.get("service_ip") or "").strip(),
            }
            events.append(event)
    return events


def _compute_kpi_snapshot(rows: Iterable[dict], switch_events: Iterable[dict], delay_target_ms: float) -> dict:
    rows = [row for row in rows if row.get("tx_packets", 0) > 0]
    switch_events = list(switch_events)

    active_clients = len(rows)
    delay_compliant = sum(1 for row in rows if row["avg_delay_ms"] <= delay_target_ms)
    delay_compliance_pct = (delay_compliant / active_clients * 100.0) if active_clients > 0 else 0.0

    aggregate_rx_mbps = sum(row["throughput_mbps"] for row in rows)
    multi_client_reception_rate_mbps = aggregate_rx_mbps if active_clients >= 3 else 0.0

    cellular_switches = sum(1 for event in switch_events if event["to"] in {"lte", "nr"})
    wifi_returns = sum(1 for event in switch_events if event["to"] == "wifi")
    # KPI latency must represent completed recoveries only.
    # Prefer explicit status when available; fallback to first_rx_after_switch_s.
    resolved_events = [
        event
        for event in switch_events
        if event["service_interruption_ms"] >= 0.0
        and (
            event.get("status") == "resolved"
            or (
                event.get("status") in {"", "pending"}
                and event["first_rx_after_switch_s"] >= 0.0
            )
        )
    ]
    unresolved_events = [
        event
        for event in switch_events
        if event.get("status") == "unresolved" or event["first_rx_after_switch_s"] < 0.0
    ]
    avg_resolved_service_interruption_ms = (
        sum(event["service_interruption_ms"] for event in resolved_events) / len(resolved_events)
        if resolved_events
        else 0.0
    )

    return {
        "active_clients": active_clients,
        "delay_target_ms": delay_target_ms,
        "delay_compliant_clients": delay_compliant,
        "delay_compliance_pct": delay_compliance_pct,
        "aggregate_rx_mbps": aggregate_rx_mbps,
        "multi_client_reception_rate_mbps": multi_client_reception_rate_mbps,
        "switch_event_count": len(switch_events),
        "switches_to_cellular": cellular_switches,
        "switches_to_wifi": wifi_returns,
        "resolved_interruption_count": len(resolved_events),
        "unresolved_interruption_count": len(unresolved_events),
        "avg_resolved_service_interruption_ms": avg_resolved_service_interruption_ms,
    }


def _print_report(rows: Iterable[dict], summary: Optional[dict]) -> None:
    """Print a human-readable report to stdout."""
    rows = list(rows)
    if not rows:
        print("No STA flows found in FlowMonitor XML.")
        return

    header = (
        f"{'STA IP':<16}"
        f"{'Traffic':<24}"
        f"{'L4':<10}"
        f"{'PDR (%)':>10}"
        f"{'Avg Delay (ms)':>16}"
        f"{'Avg Jit (ms)':>16}"
        f"{'Throughput (Mbps)':>20}"
        f"{'TX Pkts':>12}"
        f"{'RX Pkts':>12}"
        f"{'Lost Pkts':>12}"
    )
    print(header)
    print("-" * len(header))

    for row in rows:
        print(
            f"{row['sta_ip']:<16}"
            f"{row['traffic_types']:<24}"
            f"{row['l4_protocols']:<10}"
            f"{row['pdr_percent']:>10.2f}"
            f"{row['avg_delay_ms']:>16.2f}"
            f"{row['avg_jitter_ms']:>16.2f}"
            f"{row['throughput_mbps']:>20.4f}"
            f"{row['tx_packets']:>12}"
            f"{row['rx_packets']:>12}"
            f"{row['lost_packets']:>12}"
        )
    print("-" * len(header))
    if summary:
        print(
            f"{'AVERAGE':<16}"
            f"{'':<24}"
            f"{'':<10}"
            f"{summary['pdr_percent']:>10.2f}"
            f"{summary['avg_delay_ms']:>16.2f}"
            f"{summary['avg_jitter_ms']:>16.2f}"
            f"{summary['throughput_mbps']:>20.4f}"
            f"{'':>12}"
            f"{'':>12}"
            f"{'':>12}"
        )
    print()


def _write_csv(rows: Iterable[dict], csv_path: Path) -> None:
    rows = list(rows)
    if not rows:
        print(f"No data to write to {csv_path}")
        return

    fieldnames = [
        "sta_ip",
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


def _write_markdown(rows: Iterable[dict],
                    md_path: Path,
                    kpi_snapshot: Optional[dict] = None,
                    switch_events: Optional[List[dict]] = None) -> None:
    rows = list(rows)
    if not rows:
        print(f"No data to write to {md_path}")
        return

    with md_path.open("w") as md_file:
        md_file.write("# FlowMonitor STA Metrics\n\n")
        md_file.write("| STA IP | Traffic Types | L4 Protocols | PDR (%) | Avg Delay (ms) | Avg Jitter (ms) | Throughput (Mbps) | TX Packets | RX Packets | Lost Packets |\n")
        md_file.write("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n")
        for row in rows:
            md_file.write(
                "| "
                + " | ".join(
                    [
                        row["sta_ip"],
                        row["traffic_types"],
                        row["l4_protocols"],
                        f"{row['pdr_percent']:.2f}",
                        f"{row['avg_delay_ms']:.2f}",
                        f"{row['avg_jitter_ms']:.2f}",
                        f"{row['throughput_mbps']:.4f}",
                        str(row["tx_packets"]),
                        str(row["rx_packets"]),
                        str(row["lost_packets"]),
                    ]
                )
                + " |\n"
            )
        summary = _compute_summary(rows)
        if summary:
            md_file.write(
                f"| **Average** |  |  | {summary['pdr_percent']:.2f} | {summary['avg_delay_ms']:.2f} | "
                f"{summary['avg_jitter_ms']:.2f} | {summary['throughput_mbps']:.4f} |  |  |  |\n"
            )

        if kpi_snapshot is not None:
            md_file.write("\n## Phase 1 KPI Snapshot\n\n")
            md_file.write(f"- Active clients: **{kpi_snapshot['active_clients']}**\n")
            md_file.write(
                f"- Delay target compliance (<= {kpi_snapshot['delay_target_ms']:.1f} ms): "
                f"**{kpi_snapshot['delay_compliant_clients']}/{kpi_snapshot['active_clients']} "
                f"({kpi_snapshot['delay_compliance_pct']:.2f}%)**\n"
            )
            md_file.write(
                f"- Switch events: **{kpi_snapshot['switch_event_count']}** "
                f"(to cellular: {kpi_snapshot['switches_to_cellular']}, "
                f"to WiFi: {kpi_snapshot['switches_to_wifi']})\n"
            )
            md_file.write(
                f"- Switching outcomes: **resolved={kpi_snapshot['resolved_interruption_count']}**, "
                f"**unresolved={kpi_snapshot['unresolved_interruption_count']}**\n"
            )
            md_file.write(
                f"- Avg resolved switching latency (ms, service interruption = recovered-lastOK): "
                f"**{kpi_snapshot['avg_resolved_service_interruption_ms']:.3f} ms**\n"
                if kpi_snapshot["resolved_interruption_count"] > 0
                else "- Avg resolved switching latency (ms, service interruption = recovered-lastOK): **N/A**\n"
            )

        if switch_events is not None:
            md_file.write("\n## Switching Events\n\n")
            if not switch_events:
                md_file.write("No switching events captured.\n")
            else:
                md_file.write(
                    "| STA | From | To | Last OK (s) | Recovered (s) | Switching Latency (ms) | RSSI (dBm) | Service IP |\n"
                )
                md_file.write(
                    "| --- | --- | --- | --- | --- | --- | --- | --- |\n"
                )
                for event in switch_events:
                    md_file.write(
                        f"| {event['sta_index']} | "
                        f"{event['from']} | "
                        f"{event['to']} | "
                        f"{event['last_ok_before_s']:.6f} | "
                        f"{event['first_rx_after_switch_s']:.6f} | "
                        f"{event['service_interruption_ms']:.3f} | "
                        f"{event['rssi_avg_dbm']:.2f} | "
                        f"{event['service_ip']} |\n"
                    )
    print(f"Wrote Markdown report to {md_path}")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "xml",
        nargs="?",
        default=DEFAULT_FLOWMON_PATH,
        type=Path,
        help="Path to FlowMonitor XML file (default: %(default)s)",
    )
    parser.add_argument(
        "--sim-time",
        type=float,
        default=None,
        help="Simulation time in seconds (used as fallback for throughput calculation)",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Optional path to write metrics as CSV",
    )
    parser.add_argument(
        "--md",
        type=Path,
        help="Optional path to write metrics as Markdown",
    )
    parser.add_argument(
        "--switch-log",
        type=Path,
        default=None,
        help=(
            "Path to switch log CSV. If omitted, defaults to "
            "<xml-dir>/wifi-hybrid-switch_log.csv when available."
        ),
    )
    parser.add_argument(
        "--delay-target-ms",
        type=float,
        default=200.0,
        help="Delay target threshold in milliseconds for KPI compliance (default: %(default)s)",
    )
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    xml_path = args.xml
    if not xml_path.exists():
        print(f"Error: FlowMonitor XML not found: {xml_path}", file=sys.stderr)
        return 1

    flows, classifier = _parse_flowmon(xml_path)
    sta_metrics = _aggregate_sta_metrics(flows, classifier)
    rows = list(_format_rows(sta_metrics, args.sim_time))
    summary = _compute_summary(rows)
    switch_log_path = args.switch_log if args.switch_log else xml_path.parent / "wifi-hybrid-switch_log.csv"
    switch_events = _parse_switch_events(switch_log_path)
    kpi_snapshot = _compute_kpi_snapshot(rows, switch_events, args.delay_target_ms)

    _print_report(rows, summary)
    if switch_events:
        print(
            f"Switching events: {kpi_snapshot['switch_event_count']} "
            f"(to cellular={kpi_snapshot['switches_to_cellular']}, to WiFi={kpi_snapshot['switches_to_wifi']})"
        )
    else:
        print("Switching events: none")
    if kpi_snapshot["resolved_interruption_count"] > 0:
        print(
            f"Switching latency (resolved only, service interruption = recovered-lastOK): "
            f"{kpi_snapshot['avg_resolved_service_interruption_ms']:.3f} ms"
        )
    else:
        print("Switching latency (resolved only, service interruption = recovered-lastOK): none")
    print(
        f"Switching outcomes: resolved={kpi_snapshot['resolved_interruption_count']}, "
        f"unresolved={kpi_snapshot['unresolved_interruption_count']}"
    )
    print(
        f"Delay target compliance <= {args.delay_target_ms:.1f} ms: "
        f"{kpi_snapshot['delay_compliant_clients']}/{kpi_snapshot['active_clients']} "
        f"({kpi_snapshot['delay_compliance_pct']:.2f}%)"
    )
    if kpi_snapshot["active_clients"] >= 3:
        print(
            f"Data reception rate (>=3 clients): "
            f"{kpi_snapshot['multi_client_reception_rate_mbps']:.4f} Mbps"
        )

    if args.csv:
        _write_csv(rows, args.csv)
    if args.md:
        _write_markdown(rows, args.md, kpi_snapshot, switch_events)

    return 0


if __name__ == "__main__":
    sys.exit(main())

