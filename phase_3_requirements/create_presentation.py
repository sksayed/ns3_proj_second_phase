"""
Generate a polished, modern PPTX for the RSSI/PDR WiFi-Cellular Failover proposal.
Condensed 8-slide edition tuned for a 3 min 30 s talk.
Design: clean editorial layout, generous whitespace, refined navy + amber palette,
aspect-ratio-correct imagery, tight copy.
"""

import os
from PIL import Image
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn

# ── Palette ─────────────────────────────────────────────────────────────────
INK        = RGBColor(0x0E, 0x1B, 0x33)
NAVY       = RGBColor(0x16, 0x2A, 0x52)
BLUE       = RGBColor(0x2E, 0x5B, 0xE0)
SKY        = RGBColor(0xE9, 0xF0, 0xFF)
AMBER      = RGBColor(0xF5, 0xA6, 0x23)
CORAL      = RGBColor(0xF2, 0x6B, 0x4B)
PAPER      = RGBColor(0xF7, 0xF8, 0xFB)
CARD       = RGBColor(0xFF, 0xFF, 0xFF)
INKTEXT    = RGBColor(0x1B, 0x25, 0x3A)
MUTE       = RGBColor(0x6B, 0x74, 0x86)
LINE       = RGBColor(0xDD, 0xE2, 0xEC)
GREEN      = RGBColor(0x1F, 0x9D, 0x55)
WHITE      = RGBColor(0xFF, 0xFF, 0xFF)

FONT = "Segoe UI"
FONT_LIGHT = "Segoe UI Light"

BASE = os.path.dirname(os.path.abspath(__file__))

def img_path(name):
    return os.path.join(BASE, name)

_ratio_cache = {}
def img_ratio(name):
    p = img_path(name)
    if p not in _ratio_cache:
        try:
            with Image.open(p) as im:
                _ratio_cache[p] = im.size[0] / im.size[1]
        except Exception:
            _ratio_cache[p] = 1.6
    return _ratio_cache[p]


# ── Low-level helpers ─────────────────────────────────────────────────────────

def slide(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])


def bg(s, color):
    f = s.background.fill
    f.solid()
    f.fore_color.rgb = color


def rect(s, l, t, w, h, fill=None, line=None, lw=Pt(1), rounded=False, shadow=False):
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
    if shadow:
        _soft_shadow(shp)
    return shp


def _soft_shadow(shp):
    spPr = shp._element.spPr
    effLst = spPr.makeelement(qn('a:effectLst'), {})
    outer = spPr.makeelement(qn('a:outerShdw'), {
        'blurRad': '90000', 'dist': '38100', 'dir': '5400000', 'rotWithShape': '0'
    })
    clr = spPr.makeelement(qn('a:srgbClr'), {'val': '1B253A'})
    alpha = spPr.makeelement(qn('a:alpha'), {'val': '18000'})
    clr.append(alpha)
    outer.append(clr)
    effLst.append(outer)
    spPr.append(effLst)


def text(s, txt, l, t, w, h, size=14, bold=False, color=INKTEXT, align=PP_ALIGN.LEFT,
         font=FONT, italic=False, anchor=None, line_spacing=1.0, letter=None):
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = 0; tf.margin_right = 0; tf.margin_top = 0; tf.margin_bottom = 0
    if anchor:
        tf.vertical_anchor = anchor
    p = tf.paragraphs[0]
    p.alignment = align
    p.line_spacing = line_spacing
    r = p.add_run()
    r.text = txt
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.italic = italic
    r.font.name = font
    r.font.color.rgb = color
    if letter is not None:
        _letter_spacing(r, letter)
    return tb


def _letter_spacing(run, pts):
    run._r.get_or_add_rPr().set('spc', str(int(pts * 100)))


def rich(s, lines, l, t, w, h, anchor=None, line_spacing=1.05):
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = 0; tf.margin_right = 0; tf.margin_top = 0; tf.margin_bottom = 0
    if anchor:
        tf.vertical_anchor = anchor
    first = True
    for ln in lines:
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.line_spacing = ln.get("line_spacing", line_spacing)
        if "space_before" in ln: p.space_before = Pt(ln["space_before"])
        if "space_after" in ln: p.space_after = Pt(ln["space_after"])
        p.alignment = ln.get("align", PP_ALIGN.LEFT)
        r = p.add_run()
        r.text = ln.get("t", "")
        r.font.size = Pt(ln.get("size", 13))
        r.font.bold = ln.get("bold", False)
        r.font.italic = ln.get("italic", False)
        r.font.name = ln.get("font", FONT)
        r.font.color.rgb = ln.get("color", INKTEXT)
    return tb


def picture_fit(s, name, l, t, w, h, align="center", valign="middle"):
    p = img_path(name)
    if not os.path.exists(p):
        rect(s, l, t, w, h, fill=SKY, line=LINE)
        text(s, f"[{name}]", l, t + h//2, w, Inches(0.4), size=11,
             color=MUTE, align=PP_ALIGN.CENTER)
        return
    ratio = img_ratio(name)
    box_ratio = w / h
    if ratio > box_ratio:
        new_w = w; new_h = int(w / ratio)
    else:
        new_h = h; new_w = int(h * ratio)
    x = l + (w - new_w)//2 if align == "center" else (l if align == "left" else l + (w - new_w))
    y = t + (h - new_h)//2 if valign == "middle" else (t if valign == "top" else t + (h - new_h))
    s.shapes.add_picture(p, x, y, new_w, new_h)


# ── Slide chrome ───────────────────────────────────────────────────────────────
PGW = Inches(13.333)
PGH = Inches(7.5)

def kicker_header(s, kicker, title):
    left = Inches(0.75)
    text(s, kicker.upper(), left, Inches(0.55), Inches(11.0), Inches(0.3),
         size=12, bold=True, color=BLUE, letter=2.2)
    text(s, title, left, Inches(0.86), Inches(11.8), Inches(0.7),
         size=27, bold=True, color=INK, font=FONT)
    rect(s, left, Inches(1.52), Inches(0.7), Pt(3.5), fill=AMBER)


def page_footer(s, n):
    text(s, "RSSI/PDR WiFi Mesh–Cellular Failover", Inches(0.75), Inches(7.05),
         Inches(8.0), Inches(0.3), size=9, color=MUTE)
    text(s, f"{n:02d}", Inches(12.2), Inches(7.02), Inches(0.55), Inches(0.32),
         size=11, bold=True, color=BLUE, align=PP_ALIGN.RIGHT)
    text(s, "PIC Lab · KIT", Inches(10.3), Inches(7.05), Inches(1.8), Inches(0.3),
         size=9, color=MUTE, align=PP_ALIGN.RIGHT)


# ══════════════════════════════════════════════════════════════════════════════
prs = Presentation()
prs.slide_width = PGW
prs.slide_height = PGH


# ── 1 · COVER ───────────────────────────────────────────────────────────────
s = slide(prs)
bg(s, INK)
rect(s, Inches(9.6), 0, Inches(3.733), PGH, fill=NAVY)
rect(s, Inches(9.6), 0, Pt(4), PGH, fill=AMBER)
for r_ in range(6):
    for c_ in range(4):
        rect(s, Inches(10.1 + c_*0.62), Inches(1.0 + r_*0.62), Pt(6), Pt(6),
             fill=BLUE if (r_+c_) % 3 else AMBER, rounded=True)

text(s, "IDEA & TECHNOLOGY PROPOSAL", Inches(0.9), Inches(1.35), Inches(8.3), Inches(0.4),
     size=13, bold=True, color=AMBER, letter=3.0)
rect(s, Inches(0.92), Inches(1.85), Inches(0.9), Pt(3), fill=BLUE)
rich(s, [
    {"t": "Automatic WiFi Mesh–Cellular", "size": 38, "bold": True, "color": WHITE, "space_after": 0, "line_spacing": 1.02},
    {"t": "Failover for Robot Fleets", "size": 38, "bold": True, "color": WHITE, "space_after": 0, "line_spacing": 1.02},
], Inches(0.9), Inches(2.35), Inches(8.4), Inches(2.0))
text(s, "RSSI/PDR-fused, session-preserving handover that keeps multi-robot\nconstruction fleets connected below a 200 ms coordination budget.",
     Inches(0.92), Inches(4.15), Inches(8.3), Inches(1.0),
     size=15, color=RGBColor(0xB9,0xC4,0xDA), line_spacing=1.25)
chips = [("< 200 ms", "sync budget"), ("20", "robots scaled"), ("LTE & 5G", "dual fallback"), ("802.11s", "WiFi mesh")]
for i, (big, small) in enumerate(chips):
    x = Inches(0.92 + i*2.05)
    rect(s, x, Inches(5.55), Inches(1.85), Inches(1.05), fill=NAVY, rounded=True)
    text(s, big, x, Inches(5.72), Inches(1.85), Inches(0.5), size=19, bold=True,
         color=AMBER, align=PP_ALIGN.CENTER)
    text(s, small, x, Inches(6.18), Inches(1.85), Inches(0.35), size=10.5,
         color=RGBColor(0xB9,0xC4,0xDA), align=PP_ALIGN.CENTER)
text(s, "PIC Lab · Kyushu Institute of Technology · 2026", Inches(0.92), Inches(6.85),
     Inches(8.0), Inches(0.35), size=11, color=MUTE)


# ── 2 · PROBLEM ─────────────────────────────────────────────────────────────
s = slide(prs)
bg(s, PAPER)
kicker_header(s, "The Problem", "No single network keeps a fleet coordinated")
page_footer(s, 2)
text(s, "Jobsite robots must act as one coordinated system — demanding sub-200 ms sync. Neither on-site network delivers that alone.",
     Inches(0.75), Inches(1.75), Inches(11.8), Inches(0.5), size=14.5, color=INKTEXT, line_spacing=1.2)
cards = [
    ("WiFi MESH", "Fast & cheap to deploy", "…but fades fast behind concrete and steel.", BLUE),
    ("CELLULAR", "Covers the whole site", "…but slower and costly to rely on continuously.", AMBER),
]
for i, (tag, pro, con, clr) in enumerate(cards):
    x = Inches(0.75 + i*3.15)
    rect(s, x, Inches(2.55), Inches(2.95), Inches(2.3), fill=CARD, rounded=True, shadow=True)
    rect(s, x, Inches(2.55), Inches(2.95), Inches(0.55), fill=clr, rounded=True)
    rect(s, x, Inches(2.9), Inches(2.95), Inches(0.2), fill=clr)
    text(s, tag, x, Inches(2.62), Inches(2.95), Inches(0.42), size=15, bold=True,
         color=WHITE, align=PP_ALIGN.CENTER, letter=1.5)
    text(s, pro, x + Inches(0.25), Inches(3.3), Inches(2.45), Inches(0.6), size=14, bold=True, color=INK)
    text(s, con, x + Inches(0.25), Inches(3.9), Inches(2.45), Inches(0.9), size=12, color=MUTE, line_spacing=1.15)
text(s, "+", Inches(3.62), Inches(3.3), Inches(0.5), Inches(0.7), size=30, bold=True, color=MUTE, align=PP_ALIGN.CENTER)
rx = Inches(7.15)
rect(s, rx, Inches(2.55), Inches(5.4), Inches(2.3), fill=SKY, rounded=True)
text(s, "WITHOUT SEAMLESS SWITCHING", rx + Inches(0.35), Inches(2.8), Inches(4.8), Inches(0.35),
     size=12, bold=True, color=NAVY, letter=1.5)
for i, c in enumerate([
    "Fleet loses coordination mid-task",
    "Active TCP sessions drop, forcing retries",
    "Manual network reconfiguration needed",
    "Safety-critical operations interrupted",
]):
    y = Inches(3.3 + i*0.38)
    rect(s, rx + Inches(0.35), y + Inches(0.06), Pt(7), Pt(7), fill=CORAL, rounded=True)
    text(s, c, rx + Inches(0.6), y, Inches(4.6), Inches(0.35), size=12.5, color=INKTEXT)
rect(s, Inches(0.75), Inches(5.2), Inches(11.83), Inches(1.5), fill=INK, rounded=True)
rect(s, Inches(0.75), Inches(5.2), Pt(4), Inches(1.5), fill=AMBER)
text(s, "THE INSIGHT", Inches(1.15), Inches(5.45), Inches(3.0), Inches(0.3), size=11, bold=True, color=AMBER, letter=2.0)
text(s, "Use WiFi where it's strong, cellular where it isn't — and switch fast enough that\nthe robot never notices. That fusion beats either technology on its own.",
     Inches(1.15), Inches(5.8), Inches(11.0), Inches(0.8), size=15, bold=True, color=WHITE, line_spacing=1.2)


# ── 3 · SOLUTION + CORE TECHNOLOGY ───────────────────────────────────────────
s = slide(prs)
bg(s, PAPER)
kicker_header(s, "Our Solution", "A network that switches itself")
page_footer(s, 3)
# left: four capabilities (stacked compact)
feats = [
    ("Always connected", "One stable identity per robot — the system picks the link."),
    ("Seamless failover", "Slides to cellular as WiFi fades and back — session intact."),
    ("Per-robot assurance", "Every switch measured against the 200 ms target."),
    ("Technology-neutral", "Identical controller over LTE or 5G NR."),
]
text(s, "WHAT THE OPERATOR GETS", Inches(0.75), Inches(1.7), Inches(5.5), Inches(0.3),
     size=12, bold=True, color=BLUE, letter=1.5)
for i, (ttl, body) in enumerate(feats):
    y = Inches(2.1 + i*1.12)
    rect(s, Inches(0.75), y, Inches(5.55), Inches(0.98), fill=CARD, rounded=True, shadow=True)
    rect(s, Inches(0.75), y, Inches(0.12), Inches(0.98), fill=BLUE)
    text(s, ttl, Inches(1.05), y + Inches(0.12), Inches(5.1), Inches(0.35), size=14, bold=True, color=INK)
    text(s, body, Inches(1.05), y + Inches(0.48), Inches(5.1), Inches(0.45), size=11.5, color=MUTE, line_spacing=1.1)
# right: core mechanism card (dark)
px = Inches(6.65)
rect(s, px, Inches(1.7), Inches(5.93), Inches(4.9), fill=INK, rounded=True, shadow=True)
text(s, "THE CONTROLLER — RE-EVALUATES EVERY ROBOT EVERY 0.5 s",
     px + Inches(0.4), Inches(1.95), Inches(5.2), Inches(0.5), size=12, bold=True, color=AMBER, letter=1.0)
mech = [
    ("Dual-signal fusion", "Smoothed RSSI + 1 s packet-delivery ratio — a momentary dip alone never switches."),
    ("Hysteresis", "Return threshold +3 dB above leave, held for 2 checks — no oscillation."),
    ("Staleness guard", "No WiFi frame for 3 s forces failover — catches slow fades."),
    ("Make-before-break", "New route installed before old drops — live TCP survives."),
]
for i, (ttl, body) in enumerate(mech):
    y = Inches(2.55 + i*0.98)
    rect(s, px + Inches(0.4), y + Inches(0.05), Pt(8), Pt(8), fill=BLUE, rounded=True)
    text(s, ttl, px + Inches(0.67), y - Inches(0.03), Inches(5.0), Inches(0.32), size=13.5, bold=True, color=WHITE)
    text(s, body, px + Inches(0.67), y + Inches(0.3), Inches(4.95), Inches(0.55), size=11,
         color=RGBColor(0xC7,0xD1,0xE6), line_spacing=1.1)
# bottom delivery band
rect(s, Inches(0.75), Inches(6.75), Inches(11.83), Inches(0.0), fill=NAVY)  # spacer safety
text(s, "Delivered as an embedded gateway module + an operator-facing connectivity-assurance layer.",
     Inches(0.75), Inches(6.7), Inches(11.8), Inches(0.35), size=11.5, italic=True, color=MUTE)


# ── 4 · ARCHITECTURE + HANDOVER ──────────────────────────────────────────────
s = slide(prs)
bg(s, PAPER)
kicker_header(s, "Architecture & Handover", "One service IP, two paths")
page_footer(s, 4)
# left: architecture image
rect(s, Inches(0.75), Inches(1.7), Inches(6.0), Inches(3.55), fill=CARD, rounded=True, shadow=True)
picture_fit(s, "architecture_diagram.png", Inches(0.95), Inches(1.9), Inches(5.6), Inches(3.15))
text(s, "Both paths reach the same address → a handover is a routing change, not a reconnection, so sessions survive.",
     Inches(0.75), Inches(5.4), Inches(6.0), Inches(0.9), size=11.5, color=MUTE, line_spacing=1.15)
# right: decision rules
tx = Inches(7.05)
text(s, "SENSE → DECIDE → ACT → RECORD  (every 0.5 s)", tx, Inches(1.7), Inches(5.6), Inches(0.3),
     size=12, bold=True, color=BLUE, letter=0.8)
rules = [
    ("RSSI < −80 dBm and PDR < 0.90", "→ Cellular", CORAL),
    ("No WiFi frame for 3 s", "→ Cellular", CORAL),
    ("RSSI > −77 dBm for 2 checks", "→ WiFi", GREEN),
    ("Signal hovering at the edge", "Hold", MUTE),
]
ry = Inches(2.15)
for i, (cond, act, clr) in enumerate(rules):
    rect(s, tx, ry, Inches(5.53), Inches(0.7), fill=SKY if i % 2 == 0 else CARD, rounded=True, shadow=(i%2==1))
    text(s, cond, tx + Inches(0.25), ry, Inches(3.7), Inches(0.7), size=12, color=INKTEXT, anchor=MSO_ANCHOR.MIDDLE)
    text(s, act, tx + Inches(4.0), ry, Inches(1.4), Inches(0.7), size=13, bold=True, color=clr, anchor=MSO_ANCHOR.MIDDLE)
    ry += Inches(0.78)
rect(s, tx, Inches(5.4), Inches(5.53), Inches(0.9), fill=INK, rounded=True)
text(s, "Asymmetric by design: easy to leave WiFi, hard to return — biased to the preferred link, never flapping.",
     tx + Inches(0.3), Inches(5.5), Inches(4.95), Inches(0.75), size=11.5, color=WHITE, line_spacing=1.15, anchor=MSO_ANCHOR.MIDDLE)


# ── 5 · DIFFERENTIATORS ──────────────────────────────────────────────────────
s = slide(prs)
bg(s, PAPER)
kicker_header(s, "Differentiators", "Why this stands apart")
page_footer(s, 5)
diffs = [
    ("Joint, environment-tuned switching",
     "Signal AND delivery, tuned to real propagation — validated on real AP profiles (TP-Link, Netgear, ASUS), not idealized radios.", BLUE, "◆"),
    ("A measurable latency guarantee",
     "A quantified per-switch interruption against a < 200 ms budget — the number a robotics customer actually needs.", AMBER, "◷"),
    ("Head-to-head LTE vs. 5G NR",
     "Both cellular technologies in the same fallback role and topology — a direct comparison few public studies offer.", BLUE, "⇄"),
    ("Session-preserving by design",
     "Make-before-break keeps live TCP alive — interruption in the millisecond range, not a socket-timeout retry.", AMBER, "✓"),
]
for i, (ttl, body, clr, ic) in enumerate(diffs):
    r_, c_ = divmod(i, 2)
    x = Inches(0.75 + c_*6.05)
    y = Inches(1.95 + r_*2.45)
    rect(s, x, y, Inches(5.8), Inches(2.2), fill=CARD, rounded=True, shadow=True)
    rect(s, x + Inches(0.35), y + Inches(0.35), Inches(0.75), Inches(0.75), fill=clr, rounded=True)
    text(s, ic, x + Inches(0.35), y + Inches(0.4), Inches(0.75), Inches(0.65), size=24, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
    text(s, ttl, x + Inches(1.3), y + Inches(0.32), Inches(4.3), Inches(0.75), size=15, bold=True, color=INK, line_spacing=1.0)
    text(s, body, x + Inches(0.4), y + Inches(1.15), Inches(5.15), Inches(0.95), size=11.5, color=MUTE, line_spacing=1.15)


# ── 6 · VALIDATION ──────────────────────────────────────────────────────────
s = slide(prs)
bg(s, PAPER)
kicker_header(s, "Validation", "What Phase 1 has proven")
page_footer(s, 6)
rect(s, Inches(0.75), Inches(1.7), Inches(11.83), Inches(0.85), fill=INK, rounded=True)
rect(s, Inches(0.75), Inches(1.7), Pt(4), Inches(0.85), fill=GREEN)
text(s, "PHASE 1 · COMPLETE (MAR 2026)", Inches(1.1), Inches(1.85), Inches(5.5), Inches(0.3), size=11, bold=True, color=GREEN, letter=1.5)
text(s, "NS-3 comparison of WiFi Mesh + LTE vs. WiFi Mesh + 5G NR across multiple seeds and node counts.",
     Inches(1.1), Inches(2.13), Inches(11.0), Inches(0.4), size=13.5, color=WHITE)
proof = [
    ("Controller validated", "RSSI/PDR hybrid controller verified in NS-3."),
    ("Scaled to 20 robots", "All mobile clients switched automatically both ways."),
    ("Latency target met", "Every client held < 200 ms end-to-end, all runs."),
    ("Reliable return path", "Cellular → WiFi recovery confirmed, not just outbound."),
    ("Measurement pipeline", "Per-switch interruption, RSSI & PDR logged to CSV."),
    ("Real hardware profiles", "Tuned against TP-Link, Netgear & ASUS AP characteristics."),
]
for i, (ttl, body) in enumerate(proof):
    r_, c_ = divmod(i, 3)
    x = Inches(0.75 + c_*3.98)
    y = Inches(2.8 + r_*1.75)
    rect(s, x, y, Inches(3.8), Inches(1.55), fill=CARD, rounded=True, shadow=True)
    rect(s, x + Inches(0.28), y + Inches(0.28), Inches(0.5), Inches(0.5), fill=GREEN, rounded=True)
    text(s, "✓", x + Inches(0.28), y + Inches(0.3), Inches(0.5), Inches(0.45), size=15, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
    text(s, ttl, x + Inches(0.92), y + Inches(0.28), Inches(2.7), Inches(0.55), size=13.5, bold=True, color=INK, line_spacing=0.95)
    text(s, body, x + Inches(0.3), y + Inches(0.85), Inches(3.25), Inches(0.6), size=11, color=MUTE, line_spacing=1.1)
text(s, "Not yet field-tested on hardware. Further simulation planned to strengthen the statistical basis.",
     Inches(0.75), Inches(6.45), Inches(11.8), Inches(0.35), size=11.5, italic=True, color=CORAL)


# ── 7 · MARKET, MODEL & ROADMAP ──────────────────────────────────────────────
s = slide(prs)
bg(s, PAPER)
kicker_header(s, "Market, Model & Roadmap", "Opportunity and the path forward")
page_footer(s, 7)
# left: markets + streams
mk = [("Construction robotics", "$15.4B by 2030", "~18.6% CAGR", BLUE),
      ("Private 5G networks", "$150.7B by 2033", "~58.9% CAGR", AMBER)]
for i, (name, fut, cagr, clr) in enumerate(mk):
    y = Inches(1.75 + i*0.95)
    rect(s, Inches(0.75), y, Inches(6.0), Inches(0.82), fill=CARD, rounded=True, shadow=True)
    rect(s, Inches(0.75), y, Inches(0.1), Inches(0.82), fill=clr)
    text(s, name, Inches(1.0), y + Inches(0.1), Inches(3.2), Inches(0.35), size=13.5, bold=True, color=INK)
    text(s, fut, Inches(1.0), y + Inches(0.45), Inches(3.2), Inches(0.3), size=11, color=MUTE)
    text(s, cagr, Inches(4.4), y + Inches(0.22), Inches(2.2), Inches(0.4), size=15, bold=True, color=clr, align=PP_ALIGN.RIGHT)
text(s, "REVENUE:  IP licensing  ·  assurance subscription  ·  integration fees",
     Inches(0.75), Inches(3.75), Inches(6.0), Inches(0.35), size=12, bold=True, color=NAVY)
text(s, "Driver: the US alone needed ~546,000 more construction workers in 2023 — echoed in Germany, Japan & Korea.",
     Inches(0.75), Inches(4.2), Inches(6.0), Inches(0.9), size=11.5, italic=True, color=MUTE, line_spacing=1.15)
# right: roadmap phases
tx = Inches(7.05)
text(s, "ROADMAP", tx, Inches(1.75), Inches(5.6), Inches(0.3), size=12, bold=True, color=BLUE, letter=1.5)
phases = [
    ("Foundation", "Complete", GREEN),
    ("Enhanced simulation", "In progress", AMBER),
    ("Lab validation", "Planned", BLUE),
    ("Field trial", "Planned", BLUE),
]
for i, (ttl, status, clr) in enumerate(phases):
    y = Inches(2.15 + i*0.82)
    rect(s, tx, y, Inches(5.53), Inches(0.68), fill=CARD, rounded=True, shadow=True)
    rect(s, tx, y, Inches(1.55), Inches(0.68), fill=clr, rounded=True)
    text(s, status.upper(), tx + Inches(0.15), y, Inches(1.4), Inches(0.68), size=10.5, bold=True, color=WHITE, anchor=MSO_ANCHOR.MIDDLE, letter=0.8)
    text(s, ttl, tx + Inches(1.75), y, Inches(3.6), Inches(0.68), size=13.5, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
text(s, "Commercialization (1–3 yrs): file IP → license the controller → add the assurance service.",
     tx, Inches(5.5), Inches(5.53), Inches(0.7), size=11.5, color=NAVY, line_spacing=1.15)


# ── 8 · CLOSING ──────────────────────────────────────────────────────────────
s = slide(prs)
bg(s, INK)
rect(s, 0, 0, Pt(6), PGH, fill=AMBER)
text(s, "IN SUMMARY", Inches(0.9), Inches(0.85), Inches(6.0), Inches(0.4), size=13, bold=True, color=AMBER, letter=3.0)
text(s, "Keeping robot fleets connected —\nwithout anyone noticing the switch.",
     Inches(0.9), Inches(1.3), Inches(11.5), Inches(1.3), size=30, bold=True, color=WHITE, line_spacing=1.05)
pts = [
    "Dual-signal RSSI + PDR fusion with hysteresis — no false or oscillating switches.",
    "Make-before-break routing keeps live TCP sessions alive through every handover.",
    "< 200 ms interruption met across all tested scenarios, scaled to 20 robots.",
    "Rare head-to-head LTE vs. 5G NR evaluation for construction robotics.",
    "Phase 1 validated in simulation; lab and field trials mapped out.",
    "Clear path to market: IP → licensing → connectivity-assurance service.",
]
for i, p in enumerate(pts):
    r_, c_ = divmod(i, 2)
    x = Inches(0.9 + c_*6.1)
    y = Inches(3.1 + r_*1.05)
    rect(s, x, y + Inches(0.05), Inches(0.32), Inches(0.32), fill=AMBER, rounded=True)
    text(s, str(i+1), x, y + Inches(0.02), Inches(0.32), Inches(0.36), size=13, bold=True, color=INK, align=PP_ALIGN.CENTER)
    text(s, p, x + Inches(0.5), y, Inches(5.4), Inches(0.95), size=13, color=RGBColor(0xD5,0xDD,0xEC), line_spacing=1.12)
rect(s, Inches(0.9), Inches(6.65), Inches(11.5), Pt(1), fill=RGBColor(0x2A,0x3A,0x5E))
text(s, "PIC Lab · Kyushu Institute of Technology", Inches(0.9), Inches(6.85), Inches(7.0), Inches(0.35), size=12, color=RGBColor(0xB9,0xC4,0xDA))
text(s, "Idea & Technology Proposal · 2026", Inches(7.0), Inches(6.85), Inches(5.4), Inches(0.35), size=12, color=AMBER, align=PP_ALIGN.RIGHT)


# ── Speaker notes — tuned for 3 min 30 s (~490 words) ────────────────────────
NOTES = [
    # 1 Cover
    "Good morning, and thank you for the time. Our technology keeps a whole team of "
    "construction robots reliably connected, automatically, under a 200-millisecond "
    "budget — no one ever has to touch the network. Let me show you the idea, how it "
    "works, and where it's headed.",
    # 2 Problem
    "Jobsite robots must act as one coordinated system, and that needs sub-200 "
    "millisecond sync. No single network delivers it: a WiFi mesh is fast and cheap "
    "but fades behind concrete; cellular covers the whole site but is slower and "
    "costly to lean on. When a link drops, the fleet loses coordination, sessions die, "
    "and safety-critical work stops. Our insight: use each network where it's best, "
    "and switch so fast the robot never notices.",
    # 3 Solution + core tech
    "So we built a network that switches itself. Each robot keeps one identity while a "
    "controller picks the link — giving always-on connectivity, seamless failover, "
    "per-robot assurance, and support for both LTE and 5G. The engine rechecks every "
    "robot twice a second. It fuses RSSI with packet delivery so a momentary dip won't "
    "switch, uses hysteresis so it won't oscillate, a three-second staleness guard for "
    "slow fades, and make-before-break routing so a live TCP session survives.",
    # 4 Architecture + handover
    "The key trick is one service IP over two paths — a WiFi mesh and one LTE or 5G "
    "cell. Because both reach the same address, a handover is a routing change, not a "
    "reconnection, so sessions survive. Every half-second it senses, decides, acts, "
    "and records. The rules are deliberately asymmetric — easy to leave WiFi, hard to "
    "return — so it favors the preferred link without ever flapping.",
    # 5 Differentiators
    "Four things set us apart: joint signal-and-delivery switching validated on real "
    "hardware; a measurable per-switch latency guarantee against 200 milliseconds; a "
    "rare direct LTE-versus-5G comparison; and session preservation by design.",
    # 6 Validation
    "This isn't just theory. Phase 1 is complete — a full NS-3 comparison scaled to "
    "twenty robots, all holding under 200 milliseconds, with reliable return to WiFi. "
    "The honest caveat: no hardware field test yet.",
    # 7 Market, model & roadmap
    "The opportunity is large: construction robotics heads toward fifteen billion "
    "dollars by 2030 and private 5G toward a hundred and fifty billion by 2033, driven "
    "by real labor shortages. We monetize through licensing, a subscription, and "
    "integration fees. Foundation is done, enhanced simulation is underway, then lab "
    "and field trials.",
    # 8 Closing
    "Bottom line — we keep robot fleets connected without anyone noticing the switch. "
    "It's validated in simulation and the path to market is clear. Thank you — I'm "
    "happy to take questions.",
]
for idx, sld in enumerate(prs.slides):
    if idx < len(NOTES):
        sld.notes_slide.notes_text_frame.text = NOTES[idx]

# ── Save ─────────────────────────────────────────────────────────────────────
out = os.path.join(BASE, "RSSI_PDR_WiFi_Cellular_Failover_Proposal_8slide.pptx")
prs.save(out)
w = sum(len(n.split()) for n in NOTES)
print(f"Saved: {out}  ({len(list(prs.slides))} slides, {w} note words, ~{w/140:.1f} min)")
