#!/usr/bin/env python3
"""Generate overview diagrams for Robot NW Connection Simulator SW units."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
import matplotlib.patches as mpatches

OUT = Path(__file__).resolve().parent


def _box(ax, xy, w, h, text, fc="#E8F1FB", ec="#2F5F8F", fontsize=9, bold=False):
    x, y = xy
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        linewidth=1.4, facecolor=fc, edgecolor=ec,
    )
    ax.add_patch(patch)
    ax.text(
        x + w / 2, y + h / 2, text,
        ha="center", va="center", fontsize=fontsize,
        fontweight="bold" if bold else "normal",
        wrap=True, color="#1a1a1a",
    )
    return patch


def _arrow(ax, p1, p2, color="#444444"):
    ax.annotate(
        "", xy=p2, xytext=p1,
        arrowprops=dict(arrowstyle="->", color=color, lw=1.6,
                        connectionstyle="arc3,rad=0"),
    )


def unit1_architecture(path: Path):
    fig, ax = plt.subplots(figsize=(12.5, 7.2), dpi=160)
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 7.2)
    ax.axis("off")
    ax.set_title(
        "Unit 1 Overview — Robot NW Hybrid Network Simulator (NS-3)",
        fontsize=14, fontweight="bold", pad=12,
    )

    # Field area
    field = FancyBboxPatch(
        (0.35, 1.6), 7.8, 5.0,
        boxstyle="round,pad=0.02,rounding_size=0.1",
        linewidth=1.2, facecolor="#F7F9FC", edgecolor="#8AA0B8",
        linestyle="--",
    )
    ax.add_patch(field)
    ax.text(4.25, 6.35, "Construction Site Field 400 × 400 × 30 m",
            ha="center", fontsize=10, fontweight="bold", color="#2F5F8F")

    # Mesh APs
    ap_pos = [(1.2, 5.3), (5.8, 5.3), (5.8, 2.4), (1.2, 2.4)]
    for i, (x, y) in enumerate(ap_pos):
        _box(ax, (x, y), 1.5, 0.7, f"Mesh AP{i}\n802.11s + Hotspot",
             fc="#D9EAD3", ec="#38761D", fontsize=8)

    # Mesh links
    for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
        ax.plot(
            [ap_pos[a][0] + 0.75, ap_pos[b][0] + 0.75],
            [ap_pos[a][1] + 0.35, ap_pos[b][1] + 0.35],
            color="#6AA84F", lw=1.2, ls=":",
        )

    # Buildings
    for i, (x, y) in enumerate([(2.9, 4.55), (4.0, 3.3), (2.9, 2.2)]):
        _box(ax, (x, y), 1.1, 0.55, f"Building {i+1}",
             fc="#FCE5CD", ec="#B45F06", fontsize=7)

    # STA robots
    _box(ax, (3.3, 4.0), 1.6, 0.65, "STA Robots\n(Gauss-Markov / Waypoint)",
         fc="#CFE2F3", ec="#1155CC", fontsize=8, bold=True)

    # Cellular
    _box(ax, (3.35, 5.55), 1.5, 0.65, "eNB / gNB\nLTE or 5G NR",
         fc="#F4CCCC", ec="#990000", fontsize=8, bold=True)

    # Internet side
    _box(ax, (8.6, 5.2), 3.4, 0.85, "ISP Router + Internet Server\nService IP reachable via WiFi & Cellular",
         fc="#FFF2CC", ec="#BF9000", fontsize=8, bold=True)
    _box(ax, (8.6, 3.8), 3.4, 0.9, "Hybrid Controller\nRSSI + PDR monitoring\nRoute rewrite WiFi ↔ Cellular",
         fc="#D0E0E3", ec="#134F5C", fontsize=8, bold=True)
    _box(ax, (8.6, 2.2), 3.4, 1.1,
         "Outputs\n• RSSI / Switch / PDR CSV\n• FlowMonitor XML\n• config_test_2.json",
         fc="#EAD1DC", ec="#741B47", fontsize=8)

    _arrow(ax, (7.0, 5.5), (8.55, 5.55))
    _arrow(ax, (7.0, 4.2), (8.55, 4.25))
    _arrow(ax, (10.3, 3.8), (10.3, 3.35))

    ax.text(
        6.25, 0.55,
        "Primary path: WiFi Mesh Hotspot  |  Fallback path: LTE (2.0 GHz) or 5G NR (3.5 GHz)\n"
        "Propagation: HybridBuildingsPropagationLossModel  |  Target sync delay ≤ 200 ms",
        ha="center", va="center", fontsize=8.5, color="#333333",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#FFFFFF", edgecolor="#CCCCCC"),
    )

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Wrote {path}")


def unit1_switching(path: Path):
    fig, ax = plt.subplots(figsize=(12.2, 6.2), dpi=160)
    ax.set_xlim(0, 12.2)
    ax.set_ylim(0, 6.2)
    ax.axis("off")
    ax.set_title(
        "Unit 1 Overview — RSSI/PDR Hybrid Switching Flow",
        fontsize=14, fontweight="bold", pad=10,
    )

    # Row 1: monitor + decide
    _box(ax, (0.4, 4.4), 2.2, 1.1, "1. Periodic Timer\n(switchInterval)", "#E8F1FB", "#2F5F8F", 8, True)
    _box(ax, (3.1, 4.4), 2.4, 1.1, "2. Read STA RSSI\n+ sliding-window PDR", "#CFE2F3", "#1155CC", 8, True)
    _box(ax, (6.0, 4.4), 2.6, 1.1, "3. Decision\nWiFi bad? (RSSI&PDR)\nor Cellular recover?", "#FFF2CC", "#BF9000", 8, True)

    ax.annotate("", xy=(3.05, 4.95), xytext=(2.65, 4.95),
                arrowprops=dict(arrowstyle="->", color="#444", lw=1.8))
    ax.annotate("", xy=(5.95, 4.95), xytext=(5.55, 4.95),
                arrowprops=dict(arrowstyle="->", color="#444", lw=1.8))

    # Branch outcomes
    _box(ax, (1.2, 2.4), 2.8, 1.2, "WiFi → Cellular\nRewrite STA route\nto LTE / 5G NR", "#F4CCCC", "#990000", 8, True)
    _box(ax, (4.6, 2.4), 2.8, 1.2, "Cellular → WiFi\nReturn when RSSI\nrecovers (hysteresis)", "#D9EAD3", "#38761D", 8, True)
    _box(ax, (8.0, 2.4), 2.8, 1.2, "No change\nKeep current path\n(WiFi or Cellular)", "#E8E8E8", "#666666", 8, True)

    # arrows from decision
    ax.annotate("", xy=(2.6, 3.65), xytext=(6.7, 4.35),
                arrowprops=dict(arrowstyle="->", color="#990000", lw=1.5,
                                connectionstyle="arc3,rad=0.15"))
    ax.annotate("", xy=(6.0, 3.65), xytext=(7.1, 4.35),
                arrowprops=dict(arrowstyle="->", color="#38761D", lw=1.5,
                                connectionstyle="arc3,rad=0.05"))
    ax.annotate("", xy=(9.4, 3.65), xytext=(7.6, 4.35),
                arrowprops=dict(arrowstyle="->", color="#666666", lw=1.5,
                                connectionstyle="arc3,rad=-0.15"))

    ax.text(3.5, 3.85, "switch down", fontsize=7.5, color="#990000", fontweight="bold")
    ax.text(6.3, 3.85, "return", fontsize=7.5, color="#38761D", fontweight="bold")
    ax.text(8.7, 3.85, "hold", fontsize=7.5, color="#666666", fontweight="bold")

    # Record event
    _box(ax, (3.5, 0.85), 5.2, 1.0,
         "On every path change: create SwitchEventRecord\n"
         "(trigger time, apply time, serviceInterruptionMs)",
         fc="#EAD1DC", ec="#741B47", fontsize=8, bold=True)
    ax.annotate("", xy=(6.1, 1.9), xytext=(2.6, 2.35),
                arrowprops=dict(arrowstyle="->", color="#741B47", lw=1.4))
    ax.annotate("", xy=(6.1, 1.9), xytext=(6.0, 2.35),
                arrowprops=dict(arrowstyle="->", color="#741B47", lw=1.4))

    ax.text(
        6.1, 0.3,
        "Logs for Unit 2 & 3: rssi_log.csv · switch_log.csv · pdr_window.csv · flowmon_data.xml",
        ha="center", fontsize=8, color="#333",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="#F7F9FC", edgecolor="#CCC"),
    )

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Wrote {path}")


def unit2_pipeline(path: Path):
    fig, ax = plt.subplots(figsize=(12.0, 5.2), dpi=160)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.2)
    ax.axis("off")
    ax.set_title(
        "Unit 2 Overview — FlowMonitor Data Parsing Pipeline",
        fontsize=14, fontweight="bold", pad=10,
    )

    items = [
        (0.4, 3.3, 2.3, 1.3, "Unit 1 Outputs\nFlowMonitor XML\n(+ optional switch log)", "#F4CCCC", "#990000"),
        (3.2, 3.3, 2.5, 1.3, "parse_wifi_flowmon.py\nXML parse + aggregate", "#CFE2F3", "#1155CC"),
        (6.2, 3.3, 2.5, 1.3, "Traffic Classification\nUpload / Download\nVoIP / HTTP / Video", "#FFF2CC", "#BF9000"),
        (9.2, 3.3, 2.4, 1.3, "KPI Report\n.md / CSV / console\nThroughput · Delay · PDR", "#D9EAD3", "#38761D"),
    ]
    for x, y, w, h, t, fc, ec in items:
        _box(ax, (x, y), w, h, t, fc=fc, ec=ec, fontsize=8, bold=True)

    for x1, x2 in [(2.7, 3.15), (5.7, 6.15), (8.7, 9.15)]:
        ax.annotate("", xy=(x2, 3.95), xytext=(x1, 3.95),
                    arrowprops=dict(arrowstyle="->", color="#444", lw=1.8))

    _box(ax, (1.5, 1.1), 9.0, 1.3,
         "Main metrics for Robot NW Connection evaluation\n"
         "• End-to-end delay (target ≤ 200 ms)   • Packet Delivery Ratio continuity\n"
         "• Throughput before/after path switch   • Optional correlation with switch interruption",
         fc="#F7F9FC", ec="#666666", fontsize=8.5)

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Wrote {path}")


def unit3_pipeline(path: Path):
    fig, ax = plt.subplots(figsize=(12.2, 6.4), dpi=160)
    ax.set_xlim(0, 12.2)
    ax.set_ylim(0, 6.4)
    ax.axis("off")
    ax.set_title(
        "Unit 3 Overview — Visualization & Analysis Pipeline",
        fontsize=14, fontweight="bold", pad=10,
    )

    _box(ax, (0.4, 4.4), 3.2, 1.4,
         "Unit 1 Log Inputs\n• wifi-hybrid-rssi_log.csv\n• wifi-hybrid-switch_log.csv\n• Unit 2 metrics (optional)",
         fc="#F4CCCC", ec="#990000", fontsize=8, bold=True)

    scripts = [
        (4.2, 5.15, "export_trajectory_viewer.py\n+ HTML template", "#CFE2F3", "#1155CC"),
        (4.2, 3.85, "generate_switching_timeline.py", "#D0E0E3", "#134F5C"),
        (4.2, 2.55, "plot_rssi_heatmap.py\nplot_trajectory_3d.py", "#FFF2CC", "#BF9000"),
        (4.2, 1.25, "generate_mobility_\ncomparison_report.py", "#EAD1DC", "#741B47"),
    ]
    for x, y, t, fc, ec in scripts:
        _box(ax, (x, y), 3.4, 1.05, t, fc=fc, ec=ec, fontsize=8)

    outs = [
        (8.3, 5.15, "Interactive HTML\nTrajectory Viewer", "#D9EAD3", "#38761D"),
        (8.3, 3.85, "Switching Timeline\nGIF / Animation", "#D9EAD3", "#38761D"),
        (8.3, 2.55, "RSSI Heatmap &\n3D Trajectory Plots", "#D9EAD3", "#38761D"),
        (8.3, 1.25, "Mobility Comparison\nMarkdown Report", "#D9EAD3", "#38761D"),
    ]
    for x, y, t, fc, ec in outs:
        _box(ax, (x, y), 3.4, 1.05, t, fc=fc, ec=ec, fontsize=8, bold=True)

    # arrows from input to scripts
    for y in [5.65, 4.35, 3.05, 1.75]:
        ax.annotate("", xy=(4.15, y), xytext=(3.65, 5.0),
                    arrowprops=dict(arrowstyle="->", color="#888", lw=1.1,
                                    connectionstyle="arc3,rad=0.05"))
    for y in [5.65, 4.35, 3.05, 1.75]:
        ax.annotate("", xy=(8.25, y), xytext=(7.65, y),
                    arrowprops=dict(arrowstyle="->", color="#444", lw=1.6))

    ax.text(
        6.1, 0.4,
        "Purpose: visualize where/when robots switch WiFi↔cellular and compare scenario performance",
        ha="center", fontsize=8.5, color="#333",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="#CCC"),
    )

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Wrote {path}")


def main():
    unit1_architecture(OUT / "SWUnit1_HybridSimulator_NS3" / "Overview_HybridSimulator_Architecture.png")
    unit1_switching(OUT / "SWUnit1_HybridSimulator_NS3" / "Overview_HybridSimulator_SwitchingFlow.png")
    unit2_pipeline(OUT / "SWUnit2_FlowmonParser" / "Overview_FlowmonParser.png")
    unit3_pipeline(OUT / "SWUnit3_Visualization" / "Overview_Visualization.png")


if __name__ == "__main__":
    main()
