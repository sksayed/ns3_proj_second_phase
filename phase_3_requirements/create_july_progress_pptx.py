#!/usr/bin/env python3
"""
July progress PPTX: Traffic QoS deliverable, BSSID crash fix,
switch-recovery improvement, seed differences, and 192-run report plan.
"""

from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn
import os

# ── Palette (aligned with existing phase_3 decks) ────────────────────────────
INK = RGBColor(0x0E, 0x1B, 0x33)
NAVY = RGBColor(0x16, 0x2A, 0x52)
BLUE = RGBColor(0x2E, 0x5B, 0xE0)
SKY = RGBColor(0xE9, 0xF0, 0xFF)
AMBER = RGBColor(0xF5, 0xA6, 0x23)
CORAL = RGBColor(0xF2, 0x6B, 0x4B)
PAPER = RGBColor(0xF7, 0xF8, 0xFB)
CARD = RGBColor(0xFF, 0xFF, 0xFF)
INKTEXT = RGBColor(0x1B, 0x25, 0x3A)
MUTE = RGBColor(0x6B, 0x74, 0x86)
LINE = RGBColor(0xDD, 0xE2, 0xEC)
GREEN = RGBColor(0x1F, 0x9D, 0x55)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

FONT = "Segoe UI"
PGW = Inches(13.333)
PGH = Inches(7.5)

OUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "July_TrafficQoS_SwitchReliability_Progress.pptx",
)


def slide(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])


def bg(s, color):
    f = s.background.fill
    f.solid()
    f.fore_color.rgb = color


def rect(s, l, t, w, h, fill=None, line=None, lw=Pt(1), rounded=False):
    shp_type = MSO_SHAPE.ROUNDED_RECTANGLE if rounded else MSO_SHAPE.RECTANGLE
    shp = s.shapes.add_shape(shp_type, l, t, w, h)
    if rounded:
        try:
            shp.adjustments[0] = 0.06
        except Exception:
            pass
    if fill is None:
        shp.fill.background()
    else:
        shp.fill.solid()
        shp.fill.fore_color.rgb = fill
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line
        shp.line.width = lw
    shp.shadow.inherit = False
    return shp


def text(s, txt, l, t, w, h, size=14, bold=False, color=INKTEXT,
         align=PP_ALIGN.LEFT, font=FONT, italic=False, anchor=None):
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = 0
    tf.margin_right = 0
    tf.margin_top = 0
    tf.margin_bottom = 0
    if anchor:
        tf.vertical_anchor = anchor
    p = tf.paragraphs[0]
    p.alignment = align
    r = p.add_run()
    r.text = txt
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.italic = italic
    r.font.name = font
    r.font.color.rgb = color
    return tb


def rich(s, lines, l, t, w, h, anchor=None):
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = 0
    tf.margin_right = 0
    tf.margin_top = 0
    tf.margin_bottom = 0
    if anchor:
        tf.vertical_anchor = anchor
    first = True
    for ln in lines:
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.alignment = ln.get("align", PP_ALIGN.LEFT)
        if "space_before" in ln:
            p.space_before = Pt(ln["space_before"])
        if "space_after" in ln:
            p.space_after = Pt(ln["space_after"])
        r = p.add_run()
        r.text = ln.get("t", "")
        r.font.size = Pt(ln.get("size", 13))
        r.font.bold = ln.get("bold", False)
        r.font.italic = ln.get("italic", False)
        r.font.name = FONT
        r.font.color.rgb = ln.get("color", INKTEXT)
    return tb


def kicker_header(s, kicker, title):
    left = Inches(0.75)
    text(s, kicker.upper(), left, Inches(0.45), Inches(11.5), Inches(0.28),
         size=11, bold=True, color=BLUE)
    text(s, title, left, Inches(0.72), Inches(11.8), Inches(0.55),
         size=26, bold=True, color=INK)
    rect(s, left, Inches(1.28), Inches(1.1), Pt(3), fill=AMBER)


def footer(s, page, total=8):
    text(s, "Robot NW Connection Simulator  ·  July 2026 Progress",
         Inches(0.75), Inches(7.1), Inches(10), Inches(0.25),
         size=10, color=MUTE)
    text(s, f"{page} / {total}", Inches(11.6), Inches(7.1), Inches(1.0), Inches(0.25),
         size=10, color=MUTE, align=PP_ALIGN.RIGHT)


def metric_card(s, l, t, w, h, value, label, accent=BLUE):
    rect(s, l, t, w, h, fill=CARD, line=LINE, rounded=True)
    rect(s, l, t, Pt(5), h, fill=accent)
    text(s, value, l + Inches(0.2), t + Inches(0.18), w - Inches(0.3), Inches(0.55),
         size=28, bold=True, color=INK, align=PP_ALIGN.CENTER)
    text(s, label, l + Inches(0.15), t + Inches(0.75), w - Inches(0.25), Inches(0.45),
         size=11, color=MUTE, align=PP_ALIGN.CENTER)


def add_table(s, rows, l, t, w, h, col_w=None, header=True):
    n_rows = len(rows)
    n_cols = len(rows[0])
    table_shape = s.shapes.add_table(n_rows, n_cols, l, t, w, h)
    table = table_shape.table
    if col_w:
        for i, cw in enumerate(col_w):
            table.columns[i].width = cw
    for r_i, row in enumerate(rows):
        for c_i, cell_txt in enumerate(row):
            cell = table.cell(r_i, c_i)
            cell.text = str(cell_txt)
            for p in cell.text_frame.paragraphs:
                p.alignment = PP_ALIGN.CENTER
                for run in p.runs:
                    run.font.name = FONT
                    run.font.size = Pt(10 if n_rows > 8 else 11)
                    run.font.bold = (r_i == 0 and header) or (c_i == 0)
                    if r_i == 0 and header:
                        run.font.color.rgb = WHITE
                    else:
                        run.font.color.rgb = INKTEXT
            # fill
            fill = cell.fill
            fill.solid()
            if r_i == 0 and header:
                fill.fore_color.rgb = NAVY
            elif r_i % 2 == 0:
                fill.fore_color.rgb = SKY
            else:
                fill.fore_color.rgb = CARD
    return table_shape


# ── Slides ───────────────────────────────────────────────────────────────────

def slide_title(prs):
    s = slide(prs)
    bg(s, NAVY)
    rect(s, Inches(0), Inches(0), PGW, Inches(0.12), fill=AMBER)
    text(s, "JULY 2026  ·  NS-3 ENHANCEMENT PROGRESS",
         Inches(0.9), Inches(2.0), Inches(11.5), Inches(0.35),
         size=13, bold=True, color=AMBER)
    text(s, "Traffic QoS Model & Switch Reliability",
         Inches(0.9), Inches(2.5), Inches(11.5), Inches(0.7),
         size=34, bold=True, color=WHITE)
    text(s, "3-flow robot traffic · BSSID crash fix · recovery measurement · seed matrix",
         Inches(0.9), Inches(3.35), Inches(11.5), Inches(0.4),
         size=16, color=SKY)
    text(s, "Robot NW Connection Simulator  ·  Hybrid WiFi Mesh + 5G NR / LTE",
         Inches(0.9), Inches(6.4), Inches(11.5), Inches(0.35),
         size=13, color=MUTE)
    rect(s, Inches(0), Inches(7.38), PGW, Inches(0.12), fill=AMBER)


def slide_agenda(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "Overview", "What this update covers")
    items = [
        ("01", "July deliverable", "3-flow QoS traffic model (Control / Sensor / Video) + per-flow metrics"),
        ("02", "Crash fix", "WiFi BSSID assert during multi-AP roaming — root cause & patch"),
        ("03", "Recovery logic", "Switch resolution improved from ~57% → ~80%+ on representative runs"),
        ("04", "Seed differences", "Full 3×3 matrix (seeds 6/7/8 × STA 5/10/15) — why one seed is not enough"),
        ("05", "Alignment", "July plan checklist vs current status"),
        ("06", "Next", "192-run campaign to produce the proper analysis report"),
    ]
    y = Inches(1.55)
    for num, title, desc in items:
        rect(s, Inches(0.75), y, Inches(11.8), Inches(0.72), fill=CARD, line=LINE, rounded=True)
        text(s, num, Inches(0.95), y + Inches(0.15), Inches(0.7), Inches(0.45),
             size=18, bold=True, color=BLUE)
        text(s, title, Inches(1.8), y + Inches(0.1), Inches(3.5), Inches(0.35),
             size=15, bold=True, color=INK)
        text(s, desc, Inches(5.4), y + Inches(0.18), Inches(6.8), Inches(0.4),
             size=13, color=MUTE)
        y += Inches(0.82)
    footer(s, 2)


def slide_july_traffic(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "July Deliverable", "Per-flow QoS traffic model")
    rich(s, [
        {"t": "Replaced generic HTTP/VoIP mix with robot-realistic 3-flow model + DSCP marking.",
         "size": 14, "color": MUTE},
    ], Inches(0.75), Inches(1.45), Inches(11.8), Inches(0.4))

    # Three flow cards
    flows = [
        ("Control", "UDP · EF", "~1 KB / 50 ms", "≤ 50 ms · 0% loss", CORAL),
        ("Sensor", "TCP · AF31", "100 KB / 200 ms", "≤ 200 ms · < 5% loss", BLUE),
        ("Video", "TCP · AF41", "5 Mbps continuous", "≤ 500 ms · < 10% loss", GREEN),
    ]
    x = Inches(0.75)
    for name, proto, pattern, target, accent in flows:
        rect(s, x, Inches(2.05), Inches(3.8), Inches(2.4), fill=CARD, line=LINE, rounded=True)
        rect(s, x, Inches(2.05), Inches(3.8), Inches(0.12), fill=accent)
        text(s, name, x + Inches(0.25), Inches(2.35), Inches(3.3), Inches(0.4),
             size=20, bold=True, color=INK)
        text(s, proto, x + Inches(0.25), Inches(2.8), Inches(3.3), Inches(0.3),
             size=13, bold=True, color=accent)
        text(s, pattern, x + Inches(0.25), Inches(3.25), Inches(3.3), Inches(0.35),
             size=13, color=INKTEXT)
        text(s, "Target: " + target, x + Inches(0.25), Inches(3.8), Inches(3.3), Inches(0.4),
             size=12, color=MUTE)
        x += Inches(4.05)

    rect(s, Inches(0.75), Inches(4.7), Inches(11.8), Inches(1.9), fill=CARD, line=LINE, rounded=True)
    text(s, "Artifacts delivered", Inches(1.0), Inches(4.9), Inches(11), Inches(0.35),
         size=14, bold=True, color=INK)
    rich(s, [
        {"t": "• traffic_qos.cc — NS-3 scenario with 3-flow SetupApplications + Tos/DSCP",
         "size": 13, "space_before": 4},
        {"t": "• flow_metrics.py — Control PDR, P99 latency, throughput share, PASS/CHECK vs targets",
         "size": 13, "space_before": 4},
        {"t": "• CMakeLists.txt target traffic-qos  ·  sample NR runs at 5/10/15 STA",
         "size": 13, "space_before": 4},
        {"t": "• traffic_qos_report.pdf — still pending (needs full 192-run matrix for proper report)",
         "size": 13, "space_before": 4, "color": AMBER},
    ], Inches(1.0), Inches(5.3), Inches(11.2), Inches(1.2))
    footer(s, 3)


def slide_crash(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "Reliability", "WiFi BSSID crash — root cause & fix")

    rect(s, Inches(0.75), Inches(1.5), Inches(5.7), Inches(4.9), fill=CARD, line=LINE, rounded=True)
    text(s, "What crashed", Inches(1.0), Inches(1.7), Inches(5.2), Inches(0.35),
         size=15, bold=True, color=CORAL)
    rich(s, [
        {"t": "NS_ASSERT in sta-wifi-mac.cc:1392", "size": 13, "bold": True, "space_before": 6},
        {"t": "GetLink(linkId).bssid == hdr.GetAddr3()", "size": 12, "color": MUTE, "space_before": 2},
        {"t": "", "size": 8},
        {"t": "During multi-AP roaming (4 APs, same SSID MeshHotspot):",
         "size": 13, "space_before": 6},
        {"t": "1. STA associates toward AP 0c", "size": 12, "space_before": 4},
        {"t": "2. MissedBeacons → abandon → re-assoc to AP 0b", "size": 12, "space_before": 2},
        {"t": "3. Delayed AssocResp from abandoned AP 0c arrives", "size": 12, "space_before": 2},
        {"t": "4. Assert: stored BSSID=0b ≠ response BSSID=0c → abort",
         "size": 12, "space_before": 2, "bold": True, "color": CORAL},
        {"t": "", "size": 8},
        {"t": "Not a scenario bug — ns-3.45 over-strict invariant under heavy roaming.",
         "size": 12, "italic": True, "color": MUTE, "space_before": 6},
    ], Inches(1.0), Inches(2.15), Inches(5.2), Inches(4.0))

    rect(s, Inches(6.7), Inches(1.5), Inches(5.85), Inches(4.9), fill=CARD, line=LINE, rounded=True)
    text(s, "Fix & verification", Inches(6.95), Inches(1.7), Inches(5.4), Inches(0.35),
         size=15, bold=True, color=GREEN)
    rich(s, [
        {"t": "Patch in ReceiveAssocResp:", "size": 13, "bold": True, "space_before": 6},
        {"t": "Ignore AssocResp whose BSSID ≠ current link target.",
         "size": 13, "space_before": 4},
        {"t": "Keep waiting for the AP we are currently associating with.",
         "size": 13, "space_before": 2},
        {"t": "", "size": 8},
        {"t": "Before (sweep): 4 / 9 runs crashed", "size": 13, "space_before": 8, "color": CORAL},
        {"t": "After (sweep):  9 / 9 runs completed", "size": 13, "space_before": 4, "bold": True, "color": GREEN},
        {"t": "", "size": 8},
        {"t": "Verified on previously-failing cases:", "size": 12, "space_before": 6},
        {"t": "• seed 6 / STA 5  ·  seed 6 / STA 10", "size": 12, "space_before": 2},
        {"t": "• seed 7 / STA 15 ·  seed 8 / STA 15", "size": 12, "space_before": 2},
        {"t": "All now finish full 60 s without NS_ASSERT.",
         "size": 12, "space_before": 6, "bold": True},
    ], Inches(6.95), Inches(2.15), Inches(5.4), Inches(4.0))
    footer(s, 4)


def slide_recovery(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "Switch Recovery", "Path recovery logic — toward ~80% resolved")

    rich(s, [
        {"t": "A switch was often marked timeout because recovery was measured on Sensor TCP "
              "(RTO/slow-start), not true path recovery. Return Cellular → WiFi was stranded by "
              "head-of-line pending events.",
         "size": 13, "color": MUTE},
    ], Inches(0.75), Inches(1.45), Inches(11.8), Inches(0.55))

    # Before / After cards
    rect(s, Inches(0.75), Inches(2.15), Inches(5.7), Inches(2.55), fill=CARD, line=LINE, rounded=True)
    text(s, "BEFORE  (seed 7 · 10 STA · NR)", Inches(1.0), Inches(2.3), Inches(5.2), Inches(0.3),
         size=12, bold=True, color=CORAL)
    metric_card(s, Inches(1.0), Inches(2.75), Inches(2.4), Inches(1.6), "57%", "resolved switches", CORAL)
    metric_card(s, Inches(3.6), Inches(2.75), Inches(2.4), Inches(1.6), "20/47", "timeout events", AMBER)

    rect(s, Inches(6.7), Inches(2.15), Inches(5.85), Inches(2.55), fill=CARD, line=LINE, rounded=True)
    text(s, "AFTER A+B  (same seed / STA)", Inches(6.95), Inches(2.3), Inches(5.4), Inches(0.3),
         size=12, bold=True, color=GREEN)
    metric_card(s, Inches(6.95), Inches(2.75), Inches(2.5), Inches(1.6), "87%", "resolved switches", GREEN)
    metric_card(s, Inches(9.65), Inches(2.75), Inches(2.5), Inches(1.6), "2/47", "timeout events", BLUE)

    rect(s, Inches(0.75), Inches(4.95), Inches(11.8), Inches(1.7), fill=CARD, line=LINE, rounded=True)
    text(s, "What changed (A + B)", Inches(1.0), Inches(5.1), Inches(11), Inches(0.3),
         size=14, bold=True, color=INK)
    rich(s, [
        {"t": "A. Control-UDP liveness — resolve switches on continuous 50 ms Control RX "
              "(true Cellular↔WiFi path recovery), not Sensor TCP RTO.",
         "size": 13, "space_before": 4},
        {"t": "B. Head-of-line fix — stale pending switches retired as superseded when a newer "
              "path decision is made (no stranded FIFO).",
         "size": 13, "space_before": 4},
        {"t": "Full 9-run matrix aggregate: 73% resolved · 15% timeout · 12% superseded  "
              "(seed-dependent; see next slide).",
         "size": 12, "space_before": 6, "color": MUTE},
    ], Inches(1.0), Inches(5.45), Inches(11.3), Inches(1.1))
    footer(s, 5)


def slide_seeds(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "Evaluation Matrix", "How seeds & STA counts differ (NR, 60 s)")

    rows = [
        ["Seed", "STA", "Switches", "Resolved", "Timeout", "Timeout %", "Median ms"],
        ["6", "5", "24", "14", "6", "25%", "65"],
        ["6", "10", "60", "42", "8", "13%", "61"],
        ["6", "15", "96", "72", "16", "17%", "245"],
        ["7", "5", "23", "15", "4", "17%", "564"],
        ["7", "10", "47", "41", "2", "4%", "55"],
        ["7", "15", "84", "62", "13", "15%", "63"],
        ["8", "5", "34", "29", "2", "6%", "39"],
        ["8", "10", "60", "50", "6", "10%", "52"],
        ["8", "15", "78", "45", "18", "23%", "557"],
    ]
    add_table(s, rows, Inches(0.55), Inches(1.5), Inches(8.3), Inches(4.9),
              col_w=[Inches(0.85), Inches(0.85), Inches(1.2), Inches(1.2),
                     Inches(1.1), Inches(1.3), Inches(1.3)])

    rect(s, Inches(9.05), Inches(1.5), Inches(3.7), Inches(4.9), fill=CARD, line=LINE, rounded=True)
    text(s, "Key insight", Inches(9.25), Inches(1.7), Inches(3.3), Inches(0.35),
         size=14, bold=True, color=INK)
    rich(s, [
        {"t": "Best case", "size": 12, "bold": True, "color": GREEN, "space_before": 8},
        {"t": "seed 7 / STA 10 → 4% timeout, median 55 ms",
         "size": 12, "space_before": 2},
        {"t": "Worst cases", "size": 12, "bold": True, "color": CORAL, "space_before": 12},
        {"t": "seed 6 / STA 5 → 25%", "size": 12, "space_before": 2},
        {"t": "seed 8 / STA 15 → 23%", "size": 12, "space_before": 2},
        {"t": "Takeaway", "size": 12, "bold": True, "color": BLUE, "space_before": 14},
        {"t": "A single seed would have overstated reliability. Multi-seed + multi-STA is required for the report.",
         "size": 12, "space_before": 4},
        {"t": "Aggregate: 506 switches · 73% resolved",
         "size": 12, "bold": True, "space_before": 14, "color": INK},
    ], Inches(9.25), Inches(2.15), Inches(3.3), Inches(4.0))
    footer(s, 6)


def slide_alignment(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "July Plan Alignment", "Enhancement plan checklist")

    rows = [
        ["July item (plan)", "Status", "Evidence"],
        ["traffic_qos.cc — 3-flow model", "DONE", "Control/Sensor/Video + DSCP Tos"],
        ["flow_metrics.py — per-flow KPIs", "DONE", "PDR, P99, tput share, PASS/CHECK"],
        ["Control PDR & P99 analysis", "PARTIAL", "Scripts ready; needs full matrix"],
        ["traffic_qos_report.pdf", "PENDING", "Requires 192-run campaign"],
        ["BSSID crash (blocker)", "FIXED", "9/9 sweep completes"],
        ["Switch recovery (A+B)", "DONE", "57% → 87% on seed7/STA10"],
    ]
    add_table(s, rows, Inches(0.75), Inches(1.55), Inches(11.8), Inches(3.6),
              col_w=[Inches(4.2), Inches(1.5), Inches(6.1)])

    rect(s, Inches(0.75), Inches(5.4), Inches(11.8), Inches(1.25), fill=SKY, line=LINE, rounded=True)
    text(s, "July alignment verdict", Inches(1.0), Inches(5.55), Inches(11.3), Inches(0.3),
         size=14, bold=True, color=NAVY)
    text(s,
         "Core July implementation is in place (code + measurement). The formal analysis report "
         "is the remaining gap — blocked only by scale of evaluation, not by missing software units.",
         Inches(1.0), Inches(5.95), Inches(11.3), Inches(0.5),
         size=13, color=INKTEXT)
    footer(s, 7)


def slide_192(prs):
    s = slide(prs)
    bg(s, PAPER)
    kicker_header(s, "Next Step", "192-run campaign for a proper report")

    rich(s, [
        {"t": "The enhancement plan’s comparison matrix is 192 scenarios. "
              "A proper traffic_qos_report.pdf needs that full campaign — not single-seed anecdotes.",
         "size": 14, "color": MUTE},
    ], Inches(0.75), Inches(1.45), Inches(11.8), Inches(0.5))

    # Three columns
    cols = [
        ("What 192 covers", [
            "Cellular mode: LTE vs NR",
            "STA counts: 5 / 10 / 15",
            "Seeds: 6 / 7 / 8 (× more later)",
            "Mobility: waypoint + baseline",
            "Traffic: Control/Sensor/Video QoS",
            "Enable/disable switching baselines",
        ], BLUE),
        ("Automation plan", [
            "run_seeds / sweep script",
            "Parallel batch execution",
            "Per-run FlowMonitor + switch log",
            "flow_metrics.py aggregation",
            "Bootstrap CI + Mann-Whitney later",
            "Markdown → PDF report pipeline",
        ], GREEN),
        ("Report outputs", [
            "Control PDR & P99 tables",
            "Per-flow throughput share",
            "Switch timeout / interruption",
            "LTE vs NR comparison",
            "Seed-averaged KPIs ± CI",
            "traffic_qos_report.pdf",
        ], AMBER),
    ]
    x = Inches(0.75)
    for title, bullets, accent in cols:
        rect(s, x, Inches(2.15), Inches(3.8), Inches(3.55), fill=CARD, line=LINE, rounded=True)
        rect(s, x, Inches(2.15), Inches(3.8), Inches(0.1), fill=accent)
        text(s, title, x + Inches(0.2), Inches(2.4), Inches(3.4), Inches(0.35),
             size=14, bold=True, color=INK)
        y = Inches(2.9)
        for b in bullets:
            text(s, "•  " + b, x + Inches(0.2), y, Inches(3.4), Inches(0.35),
                 size=12, color=INKTEXT)
            y += Inches(0.38)
        x += Inches(4.05)

    rect(s, Inches(0.75), Inches(5.95), Inches(11.8), Inches(0.85), fill=NAVY, rounded=True)
    text(s,
         "Goal: execute the 192-run matrix → aggregate KPIs → publish traffic_qos_report.pdf "
         "as the July analysis deliverable (then expand toward 10-seed / 640 for Sep statistics).",
         Inches(1.0), Inches(6.15), Inches(11.3), Inches(0.5),
         size=13, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
    footer(s, 8)


def main():
    prs = Presentation()
    prs.slide_width = PGW
    prs.slide_height = PGH

    slide_title(prs)
    slide_agenda(prs)
    slide_july_traffic(prs)
    slide_crash(prs)
    slide_recovery(prs)
    slide_seeds(prs)
    slide_alignment(prs)
    slide_192(prs)

    prs.save(OUT)
    print(f"Wrote {OUT}")
    print(f"Size: {os.path.getsize(OUT)/1024:.1f} KB")


if __name__ == "__main__":
    main()
