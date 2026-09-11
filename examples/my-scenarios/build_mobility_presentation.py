"""
Build the Phase 2 Item 1 research presentation for:
  Waypoint + Dwell-Time Mobility Model vs. Gauss-Markov Baseline
  Sheikh Sayed Bin Rahman | PIC Lab, KIT | 2026

July revision: rebuilt against the 648-run sweep (4 mobility types x 2
cellular modes x 3 STA counts x 3 payloads x 3 speeds x 3 seeds) instead of
June's single-seed, fixed-STA-count, LTE-only data. Numbers are pulled live
from generate_mobility_comparison_report.py's scanning/aggregation logic
rather than hardcoded, so the slides can't drift from the underlying
Waypoint_outputs/ data.

Same visual system as analysis_report_updated/build_final_presentation.py
(navy/gold academic palette, header/footer bars, callout boxes, styled
tables).
"""

import os
import sys
from pathlib import Path

from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN

sys.path.insert(0, os.path.dirname(__file__))
from generate_mobility_comparison_report import (  # noqa: E402
    SCENARIOS, CANON_CELLULAR, CANON_STA, CANON_PAYLOAD, CANON_SPEED,
    scan_all, agg_stats,
)

# ── colour palette (dark navy academic, matches Phase 1 deck) ──────────────
NAVY   = RGBColor(0x1A, 0x37, 0x6C)
ACCENT = RGBColor(0x2E, 0x75, 0xB6)
GOLD   = RGBColor(0xC9, 0xA0, 0x2A)
WHITE  = RGBColor(0xFF, 0xFF, 0xFF)
LIGHT  = RGBColor(0xF2, 0xF6, 0xFC)
DARK   = RGBColor(0x1A, 0x1A, 0x1A)

SLIDE_W = Inches(13.33)
SLIDE_H = Inches(7.5)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_ROOT = os.path.join(REPO_ROOT, "Waypoint_outputs")
FIGURES_DIR = os.path.join(DATA_ROOT, "report_assets")

AUTHOR_BLOCK = "Sheikh Sayed Bin Rahman  |  PIC Lab, KIT  |  2026"

CANON_LABEL = f"{CANON_CELLULAR.upper()}, {CANON_STA} STA, {CANON_PAYLOAD}, {CANON_SPEED} m/s"


# ── helpers (same API as build_final_presentation.py) ──────────────────────

def new_prs():
    prs = Presentation()
    prs.slide_width = SLIDE_W
    prs.slide_height = SLIDE_H
    return prs


def blank_slide(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])


def add_rect(slide, left, top, width, height, fill_rgb=None, line_rgb=None):
    shape = slide.shapes.add_shape(1, left, top, width, height)
    shape.line.fill.background()
    if fill_rgb:
        shape.fill.solid()
        shape.fill.fore_color.rgb = fill_rgb
    else:
        shape.fill.background()
    if line_rgb:
        shape.line.color.rgb = line_rgb
    return shape


def add_textbox(slide, text, left, top, width, height,
                 font_size=18, bold=False, color=DARK,
                 align=PP_ALIGN.LEFT, wrap=True, italic=False):
    box = slide.shapes.add_textbox(left, top, width, height)
    tf = box.text_frame
    tf.word_wrap = wrap
    p = tf.paragraphs[0]
    p.alignment = align
    run = p.add_run()
    run.text = text
    run.font.size = Pt(font_size)
    run.font.bold = bold
    run.font.italic = italic
    run.font.color.rgb = color
    return box


def add_header_bar(slide, title_text, subtitle_text=None):
    add_rect(slide, 0, 0, SLIDE_W, Inches(1.15), fill_rgb=NAVY)
    add_textbox(slide, title_text, Inches(0.3), Inches(0.08), Inches(11.5), Inches(0.65),
                font_size=26, bold=True, color=WHITE)
    if subtitle_text:
        add_textbox(slide, subtitle_text, Inches(0.3), Inches(0.72), Inches(12.5), Inches(0.38),
                    font_size=14, color=RGBColor(0xBB, 0xD3, 0xF0))


def add_footer(slide, text=AUTHOR_BLOCK):
    add_rect(slide, 0, Inches(7.18), SLIDE_W, Inches(0.32), fill_rgb=NAVY)
    add_textbox(slide, text, Inches(0.3), Inches(7.19), Inches(9), Inches(0.28),
                font_size=9, color=RGBColor(0xBB, 0xD3, 0xF0))


def add_figure(slide, fname, left, top, width, height):
    path = os.path.join(FIGURES_DIR, fname)
    if os.path.exists(path):
        slide.shapes.add_picture(path, left, top, width, height)
    else:
        add_textbox(slide, f"[missing figure: {fname}]", left, top, width, Inches(0.3),
                     font_size=10, color=RGBColor(0xAA, 0x33, 0x33))


def add_bullet_list(slide, items, left, top, width, height, font_size=14, color=DARK):
    box = slide.shapes.add_textbox(left, top, width, height)
    tf = box.text_frame
    tf.word_wrap = True
    for i, item in enumerate(items):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.space_after = Pt(4)
        run = p.add_run()
        run.text = item
        run.font.size = Pt(font_size)
        run.font.color.rgb = color
    return box


def add_kv_table(slide, headers, rows, left, top, width, col_widths=None, highlight_rows=None):
    from pptx.oxml.ns import qn
    import lxml.etree as etree

    n_cols = len(headers)
    n_rows = len(rows) + 1
    row_h = Inches(0.32)
    height = row_h * n_rows
    tbl = slide.shapes.add_table(n_rows, n_cols, left, top, width, height).table
    tbl.first_row = True
    if col_widths:
        for ci, cw in enumerate(col_widths):
            tbl.columns[ci].width = cw

    def set_cell(cell, text, bg=None, fg=DARK, bold=False, fs=11, align=PP_ALIGN.CENTER):
        cell.text = text
        tf = cell.text_frame
        tf.word_wrap = False
        p = tf.paragraphs[0]
        p.alignment = align
        run = p.runs[0] if p.runs else p.add_run()
        run.text = text
        run.font.size = Pt(fs)
        run.font.bold = bold
        run.font.color.rgb = fg
        if bg:
            tc = cell._tc
            tcPr = tc.get_or_add_tcPr()
            solidFill = etree.SubElement(tcPr, qn('a:solidFill'))
            srgbClr = etree.SubElement(solidFill, qn('a:srgbClr'))
            srgbClr.set('val', f'{bg[0]:02X}{bg[1]:02X}{bg[2]:02X}')

    for ci, h in enumerate(headers):
        set_cell(tbl.cell(0, ci), h, bg=ACCENT, fg=WHITE, bold=True, fs=11)
    highlight_rows = highlight_rows or set()
    HIGHLIGHT = RGBColor(0xFB, 0xF0, 0xD9)
    for ri, row in enumerate(rows):
        bg = HIGHLIGHT if ri in highlight_rows else (LIGHT if ri % 2 == 0 else WHITE)
        for ci, val in enumerate(row):
            set_cell(tbl.cell(ri + 1, ci), str(val), bg=bg, fs=10)
    return tbl


# ── data ────────────────────────────────────────────────────────────────────

def filt(records, **fixed):
    return [r for r in records if all(r[k] == v for k, v in fixed.items())]


def load_data():
    records = scan_all(Path(DATA_ROOT), sim_time=90.0)
    canon = {"cellular": CANON_CELLULAR, "sta": CANON_STA, "payload": CANON_PAYLOAD, "speed": CANON_SPEED}

    headline = []
    for scenario in SCENARIOS:
        runs = filt(records, scenario=scenario, **canon)
        if not runs:
            continue
        row = {"scenario": scenario, "n_runs": len(runs)}
        for key in ["events_per_100s", "burstiness", "mean_abs_rssi_delta",
                    "p95_abs_rssi_delta", "mean_interruption_ms", "pdr"]:
            row[key] = agg_stats([r[key] for r in runs])
        headline.append(row)

    speed_rows = []
    for scenario in ["patrol", "transport", "work"]:
        for speed in sorted({r["speed"] for r in filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, payload=CANON_PAYLOAD)}):
            runs = filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, payload=CANON_PAYLOAD, speed=speed)
            if not runs:
                continue
            row = {"scenario": scenario, "speed": speed, "n_runs": len(runs)}
            for key in ["events_per_100s", "burstiness", "mean_abs_rssi_delta", "p95_abs_rssi_delta"]:
                row[key] = agg_stats([r[key] for r in runs])
            speed_rows.append(row)
    speed_rows.sort(key=lambda r: (r["scenario"], r["speed"]))

    sta_rows = []
    for scenario in SCENARIOS:
        for sta in sorted({r["sta"] for r in filt(records, scenario=scenario, cellular=CANON_CELLULAR, payload=CANON_PAYLOAD, speed=CANON_SPEED)}):
            runs = filt(records, scenario=scenario, cellular=CANON_CELLULAR, payload=CANON_PAYLOAD, speed=CANON_SPEED, sta=sta)
            if not runs:
                continue
            row = {"scenario": scenario, "sta": sta, "n_runs": len(runs)}
            for key in ["events_per_100s", "mean_interruption_ms", "pdr"]:
                row[key] = agg_stats([r[key] for r in runs])
            sta_rows.append(row)
    sta_rows.sort(key=lambda r: (r["scenario"], r["sta"]))

    payload_order = {"10kb": 0, "50kb": 1, "1mb": 2, "2mb": 3}
    payload_rows = []
    for scenario in SCENARIOS:
        for payload in sorted({r["payload"] for r in filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, speed=CANON_SPEED)}, key=lambda p: payload_order.get(p, 99)):
            runs = filt(records, scenario=scenario, cellular=CANON_CELLULAR, sta=CANON_STA, speed=CANON_SPEED, payload=payload)
            if not runs:
                continue
            row = {"scenario": scenario, "payload": payload, "n_runs": len(runs)}
            for key in ["events_per_100s", "mean_interruption_ms", "pdr"]:
                row[key] = agg_stats([r[key] for r in runs])
            payload_rows.append(row)
    payload_rows.sort(key=lambda r: (r["scenario"], payload_order.get(r["payload"], 99)))

    cellular_rows = []
    for scenario in SCENARIOS:
        for cellular in ["lte", "nr"]:
            runs = filt(records, scenario=scenario, sta=CANON_STA, payload=CANON_PAYLOAD, speed=CANON_SPEED, cellular=cellular)
            if not runs:
                continue
            row = {"scenario": scenario, "cellular": cellular, "n_runs": len(runs)}
            for key in ["events_per_100s", "mean_interruption_ms", "pdr"]:
                row[key] = agg_stats([r[key] for r in runs])
            cellular_rows.append(row)
    cellular_rows.sort(key=lambda r: (r["scenario"], r["cellular"]))

    return records, headline, speed_rows, sta_rows, payload_rows, cellular_rows


def fmt(agg, decimals=2):
    if agg["mean"] != agg["mean"]:
        return "n/a"
    return f"{agg['mean']:.{decimals}f}±{agg['sd']:.{decimals}f}"


def fmt_plain(agg, decimals=2):
    if agg["mean"] != agg["mean"]:
        return "n/a"
    return f"{agg['mean']:.{decimals}f}"


# ── slides ───────────────────────────────────────────────────────────────────

def slide_title(prs):
    sl = blank_slide(prs)
    add_rect(sl, 0, 0, SLIDE_W, SLIDE_H, fill_rgb=NAVY)
    add_rect(sl, 0, Inches(4.6), SLIDE_W, Inches(0.06), fill_rgb=GOLD)
    add_textbox(sl, "Waypoint + Dwell-Time Mobility Model",
                Inches(0.7), Inches(1.5), Inches(12), Inches(1.0),
                font_size=34, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
    add_textbox(sl, "Phase 2 Item 1: Realistic Mobility vs. the Gauss-Markov Baseline (Revised)",
                Inches(0.7), Inches(2.55), Inches(12), Inches(0.6),
                font_size=19, color=RGBColor(0xBB, 0xD3, 0xF0), align=PP_ALIGN.CENTER)
    add_textbox(sl, "Multi-Robot Construction Site Communication",
                Inches(0.7), Inches(3.2), Inches(12), Inches(0.45),
                font_size=15, italic=True, color=RGBColor(0x90, 0xB8, 0xE0), align=PP_ALIGN.CENTER)
    add_rect(sl, Inches(3.4), Inches(4.85), Inches(6.5), Inches(1.3), fill_rgb=RGBColor(0x0E, 0x22, 0x48))
    add_textbox(sl, "Sheikh Sayed Bin Rahman", Inches(3.5), Inches(4.98), Inches(6.3), Inches(0.4),
                font_size=16, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
    add_textbox(sl, "PIC Lab, KIT  ·  2026", Inches(3.5), Inches(5.4), Inches(6.3), Inches(0.32),
                font_size=13, color=RGBColor(0xBB, 0xD3, 0xF0), align=PP_ALIGN.CENTER)


def slide_overview(prs, records, headline):
    sl = blank_slide(prs)
    add_header_bar(sl, "Research Overview",
                    "Does structured robot movement (patrol / transport / work) change WiFi-cellular switching behavior vs. a random walk?")
    add_footer(sl)

    add_textbox(sl, "Motivation", Inches(0.25), Inches(1.25), Inches(4.5), Inches(0.32),
                font_size=13, bold=True, color=ACCENT)
    bullets = [
        "• Phase 1 used GaussMarkovMobilityModel — a random walk",
        "• Real construction robots follow planned routes, not random paths",
        "• 3 robot archetypes modeled via WaypointMobilityModel:",
        "    patrol (survey loop), transport (corridor), work (dwell)",
        "• Each route now moves in true 3D (height varies with context)",
        "• Question: does this change switching frequency / RSSI pattern?",
    ]
    add_bullet_list(sl, bullets, Inches(0.25), Inches(1.62), Inches(4.6), Inches(2.6), font_size=13)

    add_textbox(sl, "Test Matrix (July revision)", Inches(5.0), Inches(1.25), Inches(3.8), Inches(0.32),
                font_size=13, bold=True, color=ACCENT)
    add_kv_table(sl, ["Factor", "Levels"], [
        ["Scenarios", "gaussmarkov, patrol, transport, work"],
        ["Speeds", "0.5 / 2.0 / 5.0 m/s"],
        ["STA count", "5 / 10 / 15"],
        ["Payload", "10KB / 50KB / 1MB"],
        ["Cellular", "LTE / NR"],
        ["Seeds", "7, 8, 9 (3-seed average)"],
        ["Total runs", f"{len(records)}"],
    ], Inches(5.0), Inches(1.62), Inches(3.9), col_widths=[Inches(1.6), Inches(2.3)])

    add_textbox(sl, "Topology (unchanged from Phase 1)", Inches(9.1), Inches(1.25), Inches(3.9), Inches(0.32),
                font_size=13, bold=True, color=ACCENT)
    topo = [
        "• 400×400×30 m field, 7 buildings",
        "• 4 mesh APs (802.11s, fixed corners)",
        "• RSSI −58 dBm + PDR<0.9 triggers handover",
        "• Propagation: HybridBuildingsPropLossModel",
        "• Cellular fallback: LTE and NR, both tested",
    ]
    add_bullet_list(sl, topo, Inches(9.1), Inches(1.62), Inches(3.9), Inches(2.2), font_size=13)

    add_rect(sl, Inches(0.25), Inches(4.35), Inches(12.8), Inches(0.55), fill_rgb=LIGHT)
    add_textbox(sl,
        "KPIs evaluated:   Switch events/100s · Burstiness (event clustering) · "
        "RSSI change abruptness (dB) · Interruption duration (ms) · Packet-weighted PDR (%)",
        Inches(0.4), Inches(4.4), Inches(12.5), Inches(0.44),
        font_size=12, color=NAVY, align=PP_ALIGN.CENTER)

    add_figure(sl, "chart_events_per_100s.png", Inches(1.6), Inches(5.0), Inches(4.9), Inches(2.1))
    add_figure(sl, "chart_burstiness.png", Inches(6.9), Inches(5.0), Inches(4.9), Inches(2.1))


def slide_headline(prs, headline):
    sl = blank_slide(prs)
    add_header_bar(sl, "1.  Head-to-Head @ Canonical Config", f"{CANON_LABEL} · 3-seed average (7/8/9)")
    add_footer(sl)

    gm = next((r for r in headline if r["scenario"] == "gaussmarkov_baseline"), None)
    best_pdr = max(headline, key=lambda r: r["pdr"]["mean"])
    best_interrupt = min(headline, key=lambda r: r["mean_interruption_ms"]["mean"])
    worst_interrupt = max(headline, key=lambda r: r["mean_interruption_ms"]["mean"])

    callouts = [
        ("Best PDR", f"{fmt_plain(best_pdr['pdr'])}%", best_pdr["scenario"]),
        ("Fastest recovery", f"{fmt_plain(best_interrupt['mean_interruption_ms'], 0)} ms", best_interrupt["scenario"]),
        ("Slowest recovery", f"{fmt_plain(worst_interrupt['mean_interruption_ms'], 0)} ms", worst_interrupt["scenario"]),
        ("GM RSSI abruptness", f"{fmt_plain(gm['p95_abs_rssi_delta'])} dB", "p95 |ΔRSSI|, gaussmarkov"),
    ]
    box_w = Inches(3.0)
    for i, (title, value, note) in enumerate(callouts):
        lft = Inches(0.25) + i * Inches(3.27)
        add_rect(sl, lft, Inches(1.25), box_w, Inches(1.15), fill_rgb=NAVY)
        add_textbox(sl, title, lft + Inches(0.1), Inches(1.3), box_w - Inches(0.2), Inches(0.3),
                    font_size=11, color=RGBColor(0xBB, 0xD3, 0xF0), align=PP_ALIGN.CENTER)
        add_textbox(sl, value, lft + Inches(0.05), Inches(1.58), box_w - Inches(0.1), Inches(0.4),
                    font_size=22, bold=True, color=GOLD, align=PP_ALIGN.CENTER)
        add_textbox(sl, note, lft + Inches(0.05), Inches(1.98), box_w - Inches(0.1), Inches(0.35),
                    font_size=9, color=RGBColor(0xBB, 0xD3, 0xF0), align=PP_ALIGN.CENTER)

    rows = [
        [r["scenario"], fmt(r["events_per_100s"]), fmt(r["burstiness"]),
         fmt(r["mean_abs_rssi_delta"]), fmt(r["p95_abs_rssi_delta"]),
         fmt(r["mean_interruption_ms"], 1), fmt(r["pdr"])]
        for r in headline
    ]
    add_kv_table(sl,
        ["Scenario", "Events/100s", "Burstiness", "Mean|ΔRSSI|", "p95|ΔRSSI|", "Interrupt(ms)", "PDR%"],
        rows, Inches(0.25), Inches(2.65), Inches(8.6),
        col_widths=[Inches(1.9), Inches(1.15), Inches(1.05), Inches(1.15), Inches(1.05), Inches(1.25), Inches(0.9)])
    add_textbox(sl, "All values mean±SD across seeds 7/8/9 (n=3).",
                Inches(0.25), Inches(4.55), Inches(8.6), Inches(0.3), font_size=10, italic=True, color=ACCENT)

    add_figure(sl, "chart_interruption.png", Inches(9.05), Inches(2.6), Inches(4.05), Inches(4.4))


def slide_sensitivity(prs, title, subtitle, rows, level_key, level_header, level_fmt=str):
    sl = blank_slide(prs)
    add_header_bar(sl, title, subtitle)
    add_footer(sl)

    table_rows = [
        [r["scenario"], level_fmt(r[level_key]), fmt(r["events_per_100s"]),
         fmt(r["mean_interruption_ms"], 1), fmt(r["pdr"])]
        for r in rows
    ]
    add_kv_table(sl,
        ["Scenario", level_header, "Events/100s", "Interrupt(ms)", "PDR%"],
        table_rows, Inches(0.25), Inches(1.3), Inches(8.3),
        col_widths=[Inches(2.0), Inches(1.4), Inches(1.6), Inches(1.7), Inches(1.6)])
    return sl


def slide_sta(prs, sta_rows):
    sl = slide_sensitivity(prs, "2.  STA Count Sensitivity (5 / 10 / 15)",
                            f"Cellular/payload/speed fixed ({CANON_CELLULAR.upper()}, {CANON_PAYLOAD}, {CANON_SPEED} m/s) — new in the July revision",
                            sta_rows, "sta", "STA count")
    add_textbox(sl,
        "New axis this revision — June fixed STA count at 5 throughout. More STAs generally "
        "means more contention and more switching, as expected; PDR degradation with STA count "
        "varies noticeably by robot type (see full report for per-type detail).",
        Inches(0.25), Inches(4.7), Inches(8.3), Inches(1.4), font_size=13, italic=True)
    add_figure(sl, "chart_events_per_100s.png", Inches(8.9), Inches(1.3), Inches(4.2), Inches(3.6))
    return sl


def slide_payload(prs, payload_rows):
    sl = slide_sensitivity(prs, "3.  Payload Sensitivity (10KB / 50KB / 1MB)",
                            f"Cellular/STA/speed fixed ({CANON_CELLULAR.upper()}, {CANON_STA} STA, {CANON_SPEED} m/s) — new in the July revision",
                            payload_rows, "payload", "Payload")
    add_textbox(sl,
        "Caveat: interruption times look higher at 10KB/50KB than at 1MB — this is backwards "
        "if read as \"smaller payload = slower switching.\" Raw switch logs show why: small flows "
        "finish transmitting before a switch happens, so \"time to first RX after switch\" measures "
        "how long until the next packet is generated (sometimes never — logged as timeout), not "
        "genuine path-recovery speed. Not a real payload-dependent slowdown; see full report.",
        Inches(0.25), Inches(4.7), Inches(8.3), Inches(1.9), font_size=12, italic=True,
        color=RGBColor(0x8A, 0x4B, 0x08))
    return sl


def slide_cellular(prs, cellular_rows):
    sl = slide_sensitivity(prs, "4.  Cellular Mode Comparison (LTE vs. NR)",
                            f"STA/payload/speed fixed ({CANON_STA} STA, {CANON_PAYLOAD}, {CANON_SPEED} m/s) — new in the July revision, June only tested LTE",
                            [r for r in cellular_rows], "cellular", "Cellular",
                            level_fmt=lambda c: c.upper())
    add_textbox(sl,
        "New axis this revision — June only ran LTE. Directly relevant to the project's core "
        "WiFi+LTE vs. WiFi+5G NR research question (see full report for the complete comparison "
        "across all STA counts and payloads, not just this canonical point).",
        Inches(0.25), Inches(4.7), Inches(8.3), Inches(1.4), font_size=13, italic=True)
    return sl


def slide_speed(prs, speed_rows):
    sl = blank_slide(prs)
    add_header_bar(sl, "5.  Speed Sensitivity (0.5 / 2.0 / 5.0 m/s)",
                    f"Cellular/STA/payload fixed ({CANON_CELLULAR.upper()}, {CANON_STA} STA, {CANON_PAYLOAD}) — 3-seed average (7/8/9)")
    add_footer(sl)

    rows = [
        [r["scenario"], r["speed"], fmt(r["events_per_100s"]), fmt(r["burstiness"]),
         fmt(r["mean_abs_rssi_delta"]), fmt(r["p95_abs_rssi_delta"])]
        for r in speed_rows
    ]
    add_kv_table(sl,
        ["Scenario", "Speed (m/s)", "Events/100s", "Burstiness", "Mean|ΔRSSI|", "p95|ΔRSSI|"],
        rows, Inches(0.25), Inches(1.3), Inches(6.4),
        col_widths=[Inches(1.5), Inches(1.1), Inches(1.15), Inches(1.05), Inches(1.15), Inches(1.05)])

    add_textbox(sl,
        "Clean, consistent pattern: RSSI-change abruptness rises with speed for all three robot "
        "types. This is the one result in the whole study that's unambiguous and physically "
        "obvious — faster movement covers more distance between samples, so the signal "
        "changes more per second.",
        Inches(0.25), Inches(4.4), Inches(6.5), Inches(1.7), font_size=13, italic=True)

    add_figure(sl, "chart_rssi_abruptness.png", Inches(6.95), Inches(1.3), Inches(6.15), Inches(5.75))


def slide_verdict(prs, headline):
    sl = blank_slide(prs)
    add_header_bar(sl, "6.  Verdict vs. Enhancement Plan Predictions",
                    "NS3_Simulation_Enhancement_Plan_EN.docx section 2.4 (\"Expected Effects\") — checked per robot type, not blended")
    add_footer(sl)

    gm = next((r for r in headline if r["scenario"] == "gaussmarkov_baseline"), None)
    wp = [r for r in headline if r["scenario"] != "gaussmarkov_baseline"]

    rows = []
    for r in wp:
        burst_confirmed = r["burstiness"]["mean"] > gm["burstiness"]["mean"]
        rssi_confirmed = r["p95_abs_rssi_delta"]["mean"] > gm["p95_abs_rssi_delta"]["mean"]
        rows.append([r["scenario"], "Switching concentration",
                     f"{gm['burstiness']['mean']:.2f} → {r['burstiness']['mean']:.2f}",
                     "CONFIRMED*" if burst_confirmed else "NOT CONFIRMED"])
        rows.append(["", "RSSI abruptness",
                     f"{gm['p95_abs_rssi_delta']['mean']:.2f} → {r['p95_abs_rssi_delta']['mean']:.2f} dB",
                     "CONFIRMED*" if rssi_confirmed else "NOT CONFIRMED"])

    add_kv_table(sl,
        ["Robot type", "Prediction", "Gauss-Markov → Waypoint", "Verdict"],
        rows, Inches(0.25), Inches(1.3), Inches(9.6),
        col_widths=[Inches(2.0), Inches(2.6), Inches(3.1), Inches(1.9)])

    add_textbox(sl, "Caveats", Inches(0.25), Inches(4.35), Inches(2), Inches(0.3),
                font_size=13, bold=True, color=ACCENT)
    add_bullet_list(sl, [
        "• *n=3 seeds per side — read as a preliminary signal, not a statistically",
        "   confirmed result. Full confidence (10 seeds, bootstrap CIs, significance",
        "   tests) is enhancement item 4, targeted Sep 2026.",
        "• Switching-concentration is often already at the metric's mathematical ceiling",
        "   for Gauss-Markov itself (nearly all switches cluster in the first ~10-12s of",
        "   every run, regardless of mobility model) — that check has little room to show",
        "   improvement at this operating point. RSSI abruptness doesn't have this problem.",
    ], Inches(0.25), Inches(4.7), Inches(9.6), Inches(2.2), font_size=12)

    add_figure(sl, "chart_rssi_abruptness.png", Inches(10.0), Inches(1.3), Inches(3.1), Inches(3.0))


def slide_conclusions(prs, headline):
    sl = blank_slide(prs)
    add_header_bar(sl, "7.  Key Findings & Next Steps",
                    "What the revised (3-seed, full-sweep) data shows, and what's still ahead")
    add_footer(sl)

    add_textbox(sl, "Findings", Inches(0.25), Inches(1.3), Inches(6), Inches(0.32),
                font_size=13, bold=True, color=ACCENT)
    conclusions = [
        "1.  Waypoint mobility is implemented and works correctly in 3D — patrol altitude",
        "     climbs, transport ramps at dock ends, work sites tied to real building heights.",
        "2.  Neither of the enhancement plan's core predictions (concentrated switching,",
        "     abrupt RSSI zones from planned routes) is confirmed against Gauss-Markov at",
        "     this operating point — treated as a preliminary signal, n=3 seeds.",
        "3.  Higher speed reliably produces more abrupt RSSI change — the one clean,",
        "     expected result across all three robot types.",
        "4.  STA count, payload, and cellular mode all now swept (648 runs total) — not",
        "     fixed as in June; see full report for the complete per-axis breakdown.",
        "5.  work's building-site placement was rebalanced this revision (previously",
        "     over-sampled the field's worst-covered corners); RSSI threshold standardized",
        "     to −58 dBm (was −80 dBm) to match the rest of the project.",
    ]
    add_bullet_list(sl, conclusions, Inches(0.25), Inches(1.65), Inches(6.3), Inches(3.4), font_size=12.5)

    add_rect(sl, Inches(0.25), Inches(5.1), Inches(6.3), Inches(0.04), fill_rgb=ACCENT)
    add_textbox(sl, "Next steps (enhancement item 4, Sep 2026):", Inches(0.25), Inches(5.2), Inches(6.3), Inches(0.3),
                font_size=13, bold=True, color=ACCENT)
    add_bullet_list(sl, [
        "• Expand from 3 to 10 seeds for statistical confidence",
        "• Bootstrap 95% confidence intervals",
        "• Mann-Whitney U / Kruskal-Wallis significance tests",
    ], Inches(0.25), Inches(5.55), Inches(6.3), Inches(1.4), font_size=12.5)

    add_figure(sl, "rssi_heatmap_patrol_2ms.png", Inches(6.85), Inches(1.3), Inches(6.25), Inches(5.75))


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    records, headline, speed_rows, sta_rows, payload_rows, cellular_rows = load_data()
    if not headline:
        print("No data found under Waypoint_outputs/. Run the simulations first.", file=sys.stderr)
        sys.exit(1)

    # Copy one representative heatmap into report_assets under a stable name
    # so the conclusions slide always has something to show.
    import shutil
    canon_dirname = f"{CANON_CELLULAR}_sta{CANON_STA}_{CANON_PAYLOAD}_spd{CANON_SPEED}_seed7"
    patrol_heatmap = os.path.join(DATA_ROOT, "patrol", canon_dirname, "rssi_heatmap.png")
    dest = os.path.join(FIGURES_DIR, "rssi_heatmap_patrol_2ms.png")
    if os.path.exists(patrol_heatmap):
        shutil.copyfile(patrol_heatmap, dest)

    prs = new_prs()
    slide_title(prs)
    slide_overview(prs, records, headline)
    slide_headline(prs, headline)
    slide_sta(prs, sta_rows)
    slide_payload(prs, payload_rows)
    slide_cellular(prs, cellular_rows)
    slide_speed(prs, speed_rows)
    slide_verdict(prs, headline)
    slide_conclusions(prs, headline)

    out = os.path.join(DATA_ROOT, "mobility_comparison_presentation.pptx")
    prs.save(out)
    print(f"Saved: {out}")
    print(f"Slides: {len(prs.slides)}")


if __name__ == "__main__":
    main()
