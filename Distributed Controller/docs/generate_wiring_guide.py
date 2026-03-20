"""
Generate a PDF wiring guide for the SCDTR Phase 2 distributed system.
3x RPI Pico + 3x MCP2515 CAN-BUS modules + 3x LED/LDR luminaire circuits.
"""

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm, cm
from reportlab.lib.colors import (
    black, white, red, blue, green, orange, grey, darkgrey,
    HexColor, Color
)
from reportlab.pdfgen import canvas
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import Paragraph, Frame
from reportlab.lib.enums import TA_LEFT, TA_CENTER

W, H = A4  # 210mm x 297mm

# Colors
PICO_GREEN = HexColor("#2E8B57")
CAN_BLUE = HexColor("#1E3A5F")
BUS_RED = HexColor("#CC3333")
BUS_YELLOW = HexColor("#CCAA00")
WIRE_COLORS = {
    "VCC": red,
    "GND": black,
    "SCK": orange,
    "MOSI": blue,
    "MISO": green,
    "CS": HexColor("#8B4513"),  # brown
    "INT": HexColor("#800080"),  # purple
    "CANH": BUS_RED,
    "CANL": BUS_YELLOW,
}

def draw_header(c, page_num):
    """Draw page header."""
    c.setFont("Helvetica-Bold", 16)
    c.drawString(2*cm, H - 2*cm, "SCDTR Phase 2 — Wiring Guide")
    c.setFont("Helvetica", 9)
    c.drawString(2*cm, H - 2.5*cm, "Distributed Cooperative Illumination System — 3 Luminaires + CAN-BUS Network")
    c.setFont("Helvetica", 8)
    c.drawRightString(W - 2*cm, H - 2*cm, f"Page {page_num}")
    c.setStrokeColor(darkgrey)
    c.setLineWidth(0.5)
    c.line(2*cm, H - 2.7*cm, W - 2*cm, H - 2.7*cm)


def draw_pico_pinout(c, x, y, label="Pico", highlight_pins=None):
    """Draw a simplified Pico pinout diagram."""
    pw, ph = 30*mm, 55*mm  # Pico dimensions

    # Board outline
    c.setStrokeColor(black)
    c.setLineWidth(1)
    c.setFillColor(PICO_GREEN)
    c.roundRect(x, y, pw, ph, 3*mm, fill=1, stroke=1)

    # USB connector
    c.setFillColor(grey)
    c.rect(x + pw/2 - 4*mm, y + ph - 1*mm, 8*mm, 4*mm, fill=1, stroke=1)
    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 5)
    c.drawCentredString(x + pw/2, y + ph + 0.5*mm, "USB")

    # Label
    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 7)
    c.drawCentredString(x + pw/2, y + ph - 8*mm, label)

    # Pin labels (left side, bottom to top)
    left_pins = [
        ("GP0", 1), ("GP1", 2), ("GND", 3), ("GP2", 4), ("GP3", 5),
        ("GP4", 6), ("GP5", 7), ("GND", 8), ("GP6", 9), ("GP7", 10),
        ("GP8", 11), ("GP9", 12), ("GND", 13), ("GP10", 14), ("GP11", 15),
        ("GP12", 16), ("GP13", 17), ("GND", 18), ("GP14", 19), ("GP15", 20),
    ]
    # Right side (bottom to top)
    right_pins = [
        ("VBUS", 40), ("VSYS", 39), ("GND", 38), ("3V3_EN", 37), ("3V3", 36),
        ("ADC_VREF", 35), ("GP28/A2", 34), ("GND/AGND", 33), ("GP27/A1", 32), ("GP26/A0", 31),
        ("RUN", 30), ("GP22", 29), ("GND", 28), ("GP21", 27), ("GP20", 26),
        ("GP19", 25), ("GP18", 24), ("GND", 23), ("GP17", 22), ("GP16", 21),
    ]

    pin_spacing = (ph - 12*mm) / 20
    start_y = y + 3*mm

    c.setFont("Helvetica", 4)
    for i, (name, num) in enumerate(left_pins):
        py = start_y + i * pin_spacing
        # Pin dot
        is_highlight = highlight_pins and name in highlight_pins
        if is_highlight:
            c.setFillColor(highlight_pins[name])
        elif "GND" in name:
            c.setFillColor(black)
        elif "3V3" in name or "VBUS" in name:
            c.setFillColor(red)
        else:
            c.setFillColor(darkgrey)
        c.circle(x - 1*mm, py, 1*mm, fill=1, stroke=0)
        c.setFillColor(black)
        c.drawRightString(x - 3*mm, py - 1.2, f"{name}")

    for i, (name, num) in enumerate(right_pins):
        py = start_y + i * pin_spacing
        is_highlight = highlight_pins and name in highlight_pins
        if is_highlight:
            c.setFillColor(highlight_pins[name])
        elif "GND" in name:
            c.setFillColor(black)
        elif "3V3" in name or "VBUS" in name:
            c.setFillColor(red)
        else:
            c.setFillColor(darkgrey)
        c.circle(x + pw + 1*mm, py, 1*mm, fill=1, stroke=0)
        c.setFillColor(black)
        c.drawString(x + pw + 3*mm, py - 1.2, f"{name}")

    return x, y, pw, ph


def draw_mcp2515_module(c, x, y, label="MCP2515"):
    """Draw a MCP2515 CAN-BUS module."""
    mw, mh = 22*mm, 30*mm

    c.setStrokeColor(black)
    c.setLineWidth(1)
    c.setFillColor(CAN_BLUE)
    c.roundRect(x, y, mw, mh, 2*mm, fill=1, stroke=1)

    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 6)
    c.drawCentredString(x + mw/2, y + mh - 6*mm, "MCP2515")
    c.setFont("Helvetica", 5)
    c.drawCentredString(x + mw/2, y + mh - 9*mm, label)

    # SPI pins (left side)
    spi_pins = ["VCC", "GND", "CS", "SO", "SI", "SCK", "INT"]
    pin_spacing = 2.8*mm
    start_y = y + 3*mm

    c.setFont("Helvetica", 4)
    for i, name in enumerate(spi_pins):
        py = start_y + i * pin_spacing
        c.setFillColor(darkgrey)
        c.circle(x - 1*mm, py, 0.8*mm, fill=1, stroke=0)
        c.setFillColor(black)
        c.drawRightString(x - 3*mm, py - 1, name)

    # CAN bus pins (right side)
    can_pins = ["CANH", "CANL"]
    for i, name in enumerate(can_pins):
        py = y + mh/2 + (i - 0.5) * 4*mm
        if name == "CANH":
            c.setFillColor(BUS_RED)
        else:
            c.setFillColor(BUS_YELLOW)
        c.circle(x + mw + 1*mm, py, 0.8*mm, fill=1, stroke=0)
        c.setFillColor(black)
        c.drawString(x + mw + 3*mm, py - 1, name)

    # Termination jumper indicator
    c.setFillColor(white)
    c.setFont("Helvetica", 4)
    c.drawCentredString(x + mw/2, y + 2*mm, "[TERM]")

    return x, y, mw, mh


def page1_overview(c):
    """Page 1: System overview and component list."""
    draw_header(c, 1)

    y = H - 3.5*cm

    # Section 1: Components
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "1. Components Required (per group of 3 students)")
    y -= 0.8*cm

    components = [
        ("3x", "RPI Pico W microcontrollers with GPIO headers"),
        ("3x", "Breadboards (small)"),
        ("3x", "MCP2515 Joy-It CAN-BUS modules (with termination jumpers)"),
        ("3x", "LED 10mm (85000-100000 mcd)"),
        ("3x", "LDR (light-dependent resistor)"),
        ("3x", "47 Ohm resistor (LED current limiting)"),
        ("3x", "10 kOhm resistor (LDR voltage divider)"),
        ("3x", "10 uF capacitor (LDR filter)"),
        ("3x", "8-wire male-female jumper sets (Pico to MCP2515)"),
        ("2x", "Wires for CAN-BUS line (twisted pair recommended)"),
        ("1x", "USB cable (for hub node to PC connection)"),
        ("1x", "Shoe box (opaque, white-lined interior)"),
    ]

    c.setFont("Helvetica", 9)
    for qty, desc in components:
        c.setFont("Helvetica-Bold", 9)
        c.drawString(2.5*cm, y, qty)
        c.setFont("Helvetica", 9)
        c.drawString(4*cm, y, desc)
        y -= 0.5*cm

    y -= 0.5*cm

    # Section 2: Network topology
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "2. Network Topology")
    y -= 0.8*cm

    # Draw simple network diagram
    # PC -> [USB] -> Pico 1 -> [CAN] -> Pico 2 -> [CAN] -> Pico 3
    box_w, box_h = 3*cm, 1.2*cm
    bus_y = y - 2*cm

    # PC box
    pc_x = 2*cm
    c.setFillColor(grey)
    c.roundRect(pc_x, bus_y, box_w, box_h, 2*mm, fill=1, stroke=1)
    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(pc_x + box_w/2, bus_y + box_h/2 - 3, "PC")

    # USB label
    c.setFont("Helvetica", 7)
    c.setFillColor(black)
    c.drawCentredString(pc_x + box_w + 0.8*cm, bus_y + box_h/2 + 2*mm, "USB")
    c.setStrokeColor(black)
    c.setLineWidth(1.5)
    c.line(pc_x + box_w, bus_y + box_h/2, pc_x + box_w + 1.6*cm, bus_y + box_h/2)

    # Pico boxes
    pico_positions = []
    for i in range(3):
        px = pc_x + box_w + 2*cm + i * (box_w + 1.5*cm)
        c.setFillColor(PICO_GREEN)
        c.roundRect(px, bus_y, box_w, box_h, 2*mm, fill=1, stroke=1)
        c.setFillColor(white)
        c.setFont("Helvetica-Bold", 8)
        label = f"Node {i+1}"
        if i == 0:
            label += " (Hub)"
        c.drawCentredString(px + box_w/2, bus_y + box_h/2 - 3, label)
        pico_positions.append(px)

        # CAN module below
        can_y = bus_y - 1.2*cm
        c.setFillColor(CAN_BLUE)
        c.roundRect(px + 0.3*cm, can_y, box_w - 0.6*cm, 0.8*cm, 1*mm, fill=1, stroke=1)
        c.setFillColor(white)
        c.setFont("Helvetica", 6)
        c.drawCentredString(px + box_w/2, can_y + 0.2*cm, "MCP2515")

        # SPI connection (vertical)
        c.setStrokeColor(blue)
        c.setLineWidth(0.8)
        c.setDash([2, 2])
        c.line(px + box_w/2, bus_y, px + box_w/2, can_y + 0.8*cm)
        c.setDash([])

    # CAN-BUS line
    can_bus_y = bus_y - 2*cm
    c.setStrokeColor(BUS_RED)
    c.setLineWidth(2)
    first_can_x = pico_positions[0] + 0.3*cm
    last_can_x = pico_positions[2] + box_w - 0.3*cm
    c.line(first_can_x, can_bus_y, last_can_x, can_bus_y)
    c.setFont("Helvetica-Bold", 7)
    c.setFillColor(BUS_RED)
    c.drawString(first_can_x, can_bus_y + 2*mm, "CAN_H")

    c.setStrokeColor(BUS_YELLOW)
    c.line(first_can_x, can_bus_y - 3*mm, last_can_x, can_bus_y - 3*mm)
    c.setFillColor(BUS_YELLOW)
    c.drawString(first_can_x, can_bus_y - 6*mm, "CAN_L")

    # Vertical connections from CAN modules to bus
    for px in pico_positions:
        cx = px + box_w/2
        c.setStrokeColor(BUS_RED)
        c.setLineWidth(1)
        c.line(cx - 1*mm, bus_y - 1.2*cm, cx - 1*mm, can_bus_y)
        c.setStrokeColor(BUS_YELLOW)
        c.line(cx + 1*mm, bus_y - 1.2*cm, cx + 1*mm, can_bus_y - 3*mm)

    # Termination labels
    c.setFont("Helvetica", 6)
    c.setFillColor(darkgrey)
    c.drawString(first_can_x - 5*mm, can_bus_y - 10*mm, "120 Ohm TERM")
    c.drawRightString(last_can_x + 5*mm, can_bus_y - 10*mm, "120 Ohm TERM")

    y = can_bus_y - 2.2*cm

    # Notes
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "Key Points:")
    y -= 0.6*cm

    notes = [
        "All 3 Picos run IDENTICAL firmware. The hub auto-detects via USB serial.",
        "CAN-BUS requires 120 Ohm termination at BOTH ends of the bus.",
        "The Joy-It MCP2515 modules have built-in termination jumpers — enable on the 2 end nodes.",
        "The middle node should have its termination jumper REMOVED.",
        "Use twisted pair wire for CAN_H and CAN_L to reduce noise.",
        "Each Pico connects to its MCP2515 via SPI (8 wires).",
        "The LED + LDR circuit is the same as Phase 1 (see next page).",
    ]

    c.setFont("Helvetica", 8)
    for note in notes:
        c.drawString(2.5*cm, y, f"- {note}")
        y -= 0.45*cm


def page2_spi_wiring(c):
    """Page 2: SPI wiring between Pico and MCP2515."""
    draw_header(c, 2)

    y = H - 3.5*cm
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "3. Pico to MCP2515 SPI Wiring (per node)")
    y -= 1*cm

    # Wiring table
    connections = [
        ("Pico Pin", "MCP2515 Pin", "Wire Color", "Signal"),
        ("GP16 (pin 21)", "SO (MISO)", "Green", "SPI Data Out (from MCP)"),
        ("GP17 (pin 22)", "CS", "Brown", "Chip Select"),
        ("GP18 (pin 24)", "SCK", "Orange", "SPI Clock"),
        ("GP19 (pin 25)", "SI (MOSI)", "Blue", "SPI Data In (to MCP)"),
        ("GP20 (pin 26)", "INT", "Purple", "Interrupt (active low)"),
        ("3V3  (pin 36)", "VCC", "Red", "Power 3.3V"),
        ("GND  (pin 23)", "GND", "Black", "Ground"),
    ]

    # Draw table
    table_x = 2.5*cm
    col_widths = [3.5*cm, 3.5*cm, 2.5*cm, 5*cm]
    row_h = 0.6*cm

    for i, row in enumerate(connections):
        rx = table_x
        ry = y - i * row_h

        # Header row
        if i == 0:
            c.setFillColor(HexColor("#E0E0E0"))
            c.rect(rx, ry - 0.15*cm, sum(col_widths), row_h, fill=1, stroke=0)
            c.setFont("Helvetica-Bold", 8)
        else:
            c.setFont("Helvetica", 8)

        c.setFillColor(black)
        for j, (text, w) in enumerate(zip(row, col_widths)):
            c.drawString(rx + 2*mm, ry + 1*mm, text)
            rx += w

        # Row border
        c.setStrokeColor(grey)
        c.setLineWidth(0.3)
        c.line(table_x, ry - 0.15*cm, table_x + sum(col_widths), ry - 0.15*cm)

    # Bottom border
    ry = y - len(connections) * row_h
    c.line(table_x, ry + row_h - 0.15*cm, table_x + sum(col_widths), ry + row_h - 0.15*cm)

    y = ry - 0.5*cm

    # Visual wiring diagram
    c.setFont("Helvetica-Bold", 11)
    c.drawString(2*cm, y, "Visual Wiring Diagram (one node)")
    y -= 1.2*cm

    # Draw Pico
    pico_x = 3*cm
    pico_y = y - 6*cm
    highlight = {
        "GP16": green,
        "GP17": HexColor("#8B4513"),
        "GP18": orange,
        "GP19": blue,
        "GP20": HexColor("#800080"),
        "3V3": red,
    }
    draw_pico_pinout(c, pico_x, pico_y, "Pico W", highlight)

    # Draw MCP2515
    mcp_x = 11*cm
    mcp_y = pico_y + 1*cm
    draw_mcp2515_module(c, mcp_x, mcp_y, "CAN Module")

    # Draw wires between them
    wire_connections = [
        ("GP16", green, 0),       # MISO
        ("GP17", HexColor("#8B4513"), 1),  # CS
        ("GP18", orange, 2),      # SCK
        ("GP19", blue, 3),        # MOSI
        ("GP20", HexColor("#800080"), 4),  # INT
    ]

    pin_spacing = (55*mm - 12*mm) / 20
    mcp_pin_spacing = 2.8*mm
    pico_start_y = pico_y + 3*mm

    for name, color, mcp_idx in wire_connections:
        # Find Pico pin position (right side, counting from bottom)
        right_pin_map = {
            "GP16": 0, "GP17": 1, "GP18": 3, "GP19": 4, "GP20": 5,
        }
        pico_pin_y = pico_start_y + right_pin_map[name] * pin_spacing

        # MCP pin position (left side, counting from bottom)
        # VCC=0, GND=1, CS=2, SO=3, SI=4, SCK=5, INT=6
        mcp_pin_map = {"GP16": 3, "GP17": 2, "GP18": 5, "GP19": 4, "GP20": 6}
        mcp_pin_y = (mcp_y + 3*mm) + mcp_pin_map[name] * mcp_pin_spacing

        c.setStrokeColor(color)
        c.setLineWidth(1.5)
        px = pico_x + 30*mm + 1*mm
        mx = mcp_x - 1*mm

        # Draw curved wire
        mid_x = (px + mx) / 2
        c.bezier(px, pico_pin_y, mid_x, pico_pin_y, mid_x, mcp_pin_y, mx, mcp_pin_y)

    # Power wires
    # 3V3 -> VCC
    c.setStrokeColor(red)
    c.setLineWidth(1.5)
    pico_3v3_y = pico_start_y + 16 * pin_spacing  # 3V3 is pin 36 (right side, index ~16)
    mcp_vcc_y = (mcp_y + 3*mm) + 0 * mcp_pin_spacing
    mid_x = (pico_x + 30*mm + mcp_x) / 2
    c.bezier(pico_x + 31*mm, pico_3v3_y, mid_x, pico_3v3_y, mid_x, mcp_vcc_y, mcp_x - 1*mm, mcp_vcc_y)

    # GND -> GND
    c.setStrokeColor(black)
    pico_gnd_y = pico_start_y + 7 * pin_spacing  # GND pin 23
    mcp_gnd_y = (mcp_y + 3*mm) + 1 * mcp_pin_spacing
    c.bezier(pico_x + 31*mm, pico_gnd_y, mid_x, pico_gnd_y, mid_x, mcp_gnd_y, mcp_x - 1*mm, mcp_gnd_y)

    y = pico_y - 1.5*cm

    # Important notes
    c.setFont("Helvetica-Bold", 9)
    c.setFillColor(BUS_RED)
    c.drawString(2*cm, y, "IMPORTANT:")
    c.setFillColor(black)
    c.setFont("Helvetica", 8)
    y -= 0.5*cm
    c.drawString(2.5*cm, y, "- Use SPI1 (not SPI0). The firmware is configured for these exact pins.")
    y -= 0.4*cm
    c.drawString(2.5*cm, y, "- MCP2515 runs at 3.3V (same as Pico). Do NOT connect to 5V.")
    y -= 0.4*cm
    c.drawString(2.5*cm, y, "- Keep SPI wires short (<15cm) to avoid signal integrity issues.")
    y -= 0.4*cm
    c.drawString(2.5*cm, y, "- The INT pin is active-low with internal pull-up on the Pico (configured in firmware).")


def page3_led_ldr_circuit(c):
    """Page 3: LED and LDR circuits (same as Phase 1)."""
    draw_header(c, 3)

    y = H - 3.5*cm
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "4. LED Driver & LDR Sensor Circuit (per node)")
    y -= 0.8*cm

    c.setFont("Helvetica", 9)
    c.drawString(2*cm, y, "Each node has the same LED/LDR circuit from Phase 1:")
    y -= 1.2*cm

    # LED circuit
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "A) LED Driving Circuit")
    y -= 0.8*cm

    # Draw LED circuit schematic
    cx = 4*cm
    cy = y - 2*cm

    # GP15 label
    c.setFont("Helvetica-Bold", 8)
    c.drawString(cx - 1.5*cm, cy + 5*mm, "GP15")
    c.drawString(cx - 1.5*cm, cy + 1*mm, "(PWM)")

    # Wire from GP15
    c.setStrokeColor(black)
    c.setLineWidth(1)
    c.line(cx, cy + 3*mm, cx + 1.5*cm, cy + 3*mm)

    # Resistor symbol (47 Ohm)
    rx = cx + 1.5*cm
    c.setLineWidth(1)
    for i in range(6):
        x1 = rx + i * 2*mm
        x2 = rx + (i + 1) * 2*mm
        if i % 2 == 0:
            c.line(x1, cy + 3*mm, x2, cy + 3*mm + 2*mm)
        else:
            c.line(x1, cy + 3*mm + 2*mm, x2, cy + 3*mm)
    c.setFont("Helvetica", 7)
    c.drawCentredString(rx + 6*mm, cy + 3*mm + 5*mm, "47 Ohm")

    # Wire to LED
    rx_end = rx + 12*mm
    c.line(rx_end, cy + 3*mm, rx_end + 8*mm, cy + 3*mm)

    # LED triangle symbol
    lx = rx_end + 8*mm
    c.setFillColor(HexColor("#FFD700"))
    c.setStrokeColor(black)
    # Triangle pointing right
    path = c.beginPath()
    path.moveTo(lx, cy + 3*mm + 4*mm)
    path.lineTo(lx, cy + 3*mm - 4*mm)
    path.lineTo(lx + 6*mm, cy + 3*mm)
    path.close()
    c.drawPath(path, fill=1, stroke=1)
    # LED bar
    c.line(lx + 6*mm, cy + 3*mm + 4*mm, lx + 6*mm, cy + 3*mm - 4*mm)

    c.setFont("Helvetica", 7)
    c.drawCentredString(lx + 3*mm, cy + 3*mm - 7*mm, "LED")

    # Wire to GND
    c.line(lx + 6*mm, cy + 3*mm, lx + 12*mm, cy + 3*mm)
    c.setFont("Helvetica-Bold", 8)
    c.drawString(lx + 13*mm, cy + 1*mm, "GND")

    y = cy - 2*cm

    # LDR circuit
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "B) LDR Illuminance Reading Circuit")
    y -= 0.8*cm

    # Draw voltage divider
    cx = 4*cm
    cy = y - 4*cm

    # VCC at top
    c.setFont("Helvetica-Bold", 8)
    c.setFillColor(red)
    c.drawCentredString(cx + 2*cm, cy + 4.5*cm, "3V3 (VCC)")
    c.setFillColor(black)

    # Vertical wire from VCC
    c.setStrokeColor(black)
    c.setLineWidth(1)
    top_y = cy + 4.2*cm
    c.line(cx + 2*cm, top_y, cx + 2*cm, top_y - 5*mm)

    # LDR symbol (resistor with arrows)
    ldr_top = top_y - 5*mm
    ldr_bot = ldr_top - 1.5*cm
    c.setLineWidth(1)
    c.line(cx + 2*cm, ldr_top, cx + 2*cm, ldr_top - 2*mm)
    c.rect(cx + 2*cm - 3*mm, ldr_top - 2*mm - 12*mm, 6*mm, 12*mm, fill=0, stroke=1)
    c.line(cx + 2*cm, ldr_top - 2*mm - 12*mm, cx + 2*cm, ldr_bot)
    c.setFont("Helvetica", 7)
    c.drawString(cx + 2*cm + 5*mm, ldr_top - 10*mm, "LDR")
    # Light arrows
    c.setStrokeColor(orange)
    c.setLineWidth(0.5)
    ax = cx + 2*cm + 9*mm
    ay = ldr_top - 6*mm
    c.line(ax, ay, ax + 4*mm, ay + 3*mm)
    c.line(ax, ay - 3*mm, ax + 4*mm, ay)
    c.setStrokeColor(black)

    # Middle junction (ADC point)
    mid_y = ldr_bot
    c.setLineWidth(1)
    c.setFillColor(red)
    c.circle(cx + 2*cm, mid_y, 1*mm, fill=1, stroke=1)

    # ADC label
    c.setFillColor(black)
    c.setFont("Helvetica-Bold", 7)
    c.drawString(cx + 2*cm + 5*mm, mid_y - 1*mm, "GP26 / A0")
    c.setFont("Helvetica", 6)
    c.drawString(cx + 2*cm + 5*mm, mid_y - 4*mm, "(Analog Input)")

    # Horizontal wire to ADC
    c.setStrokeColor(green)
    c.setLineWidth(1)
    c.line(cx + 2*cm, mid_y, cx + 2*cm + 4*mm, mid_y)
    c.setStrokeColor(black)

    # 10k resistor below junction
    r_top = mid_y
    r_bot = r_top - 1.5*cm
    c.line(cx + 2*cm, r_top, cx + 2*cm, r_top - 2*mm)
    c.rect(cx + 2*cm - 3*mm, r_top - 2*mm - 12*mm, 6*mm, 12*mm, fill=0, stroke=1)
    c.line(cx + 2*cm, r_top - 2*mm - 12*mm, cx + 2*cm, r_bot)
    c.setFont("Helvetica", 7)
    c.drawString(cx + 2*cm + 5*mm, r_top - 10*mm, "10 kOhm")

    # Capacitor in parallel (to the right)
    cap_x = cx + 2*cm + 2*cm
    c.line(cx + 2*cm, r_top, cap_x, r_top)
    c.line(cx + 2*cm, r_bot, cap_x, r_bot)
    c.line(cap_x, r_top, cap_x, r_top - 5*mm)
    # Capacitor plates
    c.setLineWidth(1.5)
    c.line(cap_x - 3*mm, r_top - 5*mm, cap_x + 3*mm, r_top - 5*mm)
    c.line(cap_x - 3*mm, r_top - 7*mm, cap_x + 3*mm, r_top - 7*mm)
    c.setLineWidth(1)
    c.line(cap_x, r_top - 7*mm, cap_x, r_bot)
    c.setFont("Helvetica", 7)
    c.drawString(cap_x + 4*mm, r_top - 8*mm, "10 uF")

    # GND at bottom
    c.line(cx + 2*cm, r_bot, cx + 2*cm, r_bot - 3*mm)
    # GND symbol
    gnd_y = r_bot - 3*mm
    c.setLineWidth(1.5)
    c.line(cx + 2*cm - 4*mm, gnd_y, cx + 2*cm + 4*mm, gnd_y)
    c.line(cx + 2*cm - 2.5*mm, gnd_y - 2*mm, cx + 2*cm + 2.5*mm, gnd_y - 2*mm)
    c.line(cx + 2*cm - 1*mm, gnd_y - 4*mm, cx + 2*cm + 1*mm, gnd_y - 4*mm)

    c.setFont("Helvetica-Bold", 8)
    c.drawCentredString(cx + 2*cm, gnd_y - 8*mm, "GND")

    y = gnd_y - 1.5*cm

    # Pin summary
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "LED/LDR Pin Summary:")
    y -= 0.6*cm

    c.setFont("Helvetica", 9)
    pins = [
        ("GP15 (pin 20)", "LED PWM output (via 47 Ohm resistor)"),
        ("GP26/A0 (pin 31)", "LDR analog input (voltage divider midpoint)"),
        ("3V3 (pin 36)", "LDR circuit power supply"),
        ("GND (pin 23 or 28)", "Common ground for LED and LDR"),
    ]
    for pin, desc in pins:
        c.setFont("Helvetica-Bold", 8)
        c.drawString(2.5*cm, y, pin)
        c.setFont("Helvetica", 8)
        c.drawString(7*cm, y, desc)
        y -= 0.45*cm


def page4_canbus_wiring(c):
    """Page 4: CAN-BUS wiring between modules."""
    draw_header(c, 4)

    y = H - 3.5*cm
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "5. CAN-BUS Wiring Between Nodes")
    y -= 1*cm

    # Bus topology diagram
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "Bus Topology (daisy-chain)")
    y -= 0.5*cm

    c.setFont("Helvetica", 8)
    c.drawString(2*cm, y, "CAN-BUS uses a linear bus topology. Connect all CANH together and all CANL together.")
    y -= 1.5*cm

    # Draw 3 modules with bus
    module_y = y
    spacing = 5*cm

    for i in range(3):
        mx = 2.5*cm + i * spacing
        # Module box
        c.setFillColor(CAN_BLUE)
        c.roundRect(mx, module_y, 3*cm, 1.5*cm, 2*mm, fill=1, stroke=1)
        c.setFillColor(white)
        c.setFont("Helvetica-Bold", 8)
        c.drawCentredString(mx + 1.5*cm, module_y + 0.9*cm, f"MCP2515")
        c.drawCentredString(mx + 1.5*cm, module_y + 0.4*cm, f"Node {i+1}")

        # Termination label
        c.setFillColor(black)
        c.setFont("Helvetica", 6)
        if i == 0 or i == 2:
            c.setFillColor(BUS_RED)
            c.drawCentredString(mx + 1.5*cm, module_y - 3*mm, "TERM: ON")
        else:
            c.setFillColor(grey)
            c.drawCentredString(mx + 1.5*cm, module_y - 3*mm, "TERM: OFF")

    # CANH bus line
    bus_y = module_y + 1.5*cm + 5*mm
    c.setStrokeColor(BUS_RED)
    c.setLineWidth(2.5)
    c.line(2.5*cm + 1.5*cm, bus_y, 2.5*cm + 2 * spacing + 1.5*cm, bus_y)
    c.setFont("Helvetica-Bold", 8)
    c.setFillColor(BUS_RED)
    c.drawString(2.5*cm + 2 * spacing + 2*cm, bus_y - 1*mm, "CAN_H")

    # CANL bus line
    c.setStrokeColor(BUS_YELLOW)
    c.setLineWidth(2.5)
    c.line(2.5*cm + 1.5*cm, bus_y - 5*mm, 2.5*cm + 2 * spacing + 1.5*cm, bus_y - 5*mm)
    c.setFillColor(BUS_YELLOW)
    c.drawString(2.5*cm + 2 * spacing + 2*cm, bus_y - 6*mm, "CAN_L")

    # Vertical connections
    for i in range(3):
        mx = 2.5*cm + i * spacing + 1.5*cm
        c.setStrokeColor(BUS_RED)
        c.setLineWidth(1.5)
        c.line(mx - 2*mm, module_y + 1.5*cm, mx - 2*mm, bus_y)
        c.setStrokeColor(BUS_YELLOW)
        c.line(mx + 2*mm, module_y + 1.5*cm, mx + 2*mm, bus_y - 5*mm)

    y = module_y - 1.5*cm

    # Wiring instructions
    c.setFont("Helvetica-Bold", 10)
    c.setFillColor(black)
    c.drawString(2*cm, y, "CAN-BUS Wiring Steps:")
    y -= 0.7*cm

    steps = [
        "1. Connect all three MCP2515 CANH pins together using a single wire (red recommended).",
        "2. Connect all three MCP2515 CANL pins together using a single wire (yellow recommended).",
        "3. Enable the termination jumper on Node 1 and Node 3 (the two END nodes).",
        "4. Remove/disable the termination jumper on Node 2 (the MIDDLE node).",
        "5. Keep CAN-BUS wires as short as possible. Use twisted pair if available.",
        "6. Ensure all nodes share a common GND (they do if powered from same USB hub).",
    ]

    c.setFont("Helvetica", 8)
    for step in steps:
        c.drawString(2.5*cm, y, step)
        y -= 0.5*cm

    y -= 0.5*cm

    # Termination explanation
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "6. Termination Resistors")
    y -= 0.7*cm

    c.setFont("Helvetica", 8)
    term_notes = [
        "CAN-BUS requires 120 Ohm termination resistors at each end of the bus to prevent signal reflections.",
        "The Joy-It MCP2515 modules have a built-in 120 Ohm resistor that can be activated via a jumper.",
        "",
        "For a 3-node setup:",
        "  Node 1 (end):     Termination jumper ON",
        "  Node 2 (middle):  Termination jumper OFF",
        "  Node 3 (end):     Termination jumper ON",
        "",
        "If you only have 2 nodes: both should have termination ON.",
        "If the jumper is a solder bridge, you may need to solder/desolder it.",
    ]

    for note in term_notes:
        c.drawString(2.5*cm, y, note)
        y -= 0.4*cm

    y -= 0.5*cm

    # Common issues
    c.setFont("Helvetica-Bold", 10)
    c.drawString(2*cm, y, "7. Troubleshooting")
    y -= 0.7*cm

    issues = [
        ("No CAN communication:", "Check SPI wiring (especially MISO/MOSI not swapped). Verify 3.3V power."),
        ("Intermittent errors:", "Check termination. Both ends must have 120 Ohm. Check wire connections."),
        ("Only hub sees messages:", "Other nodes may not have booted. Check CAN_H/CAN_L continuity."),
        ("Node not discovered:", "Verify MCP2515 crystal is 16 MHz (matches firmware setting MCP_16MHZ)."),
        ("SPI not working:", "Ensure using SPI1 pins (GP16-19), not SPI0 (GP0-3)."),
    ]

    for problem, solution in issues:
        c.setFont("Helvetica-Bold", 8)
        c.drawString(2.5*cm, y, problem)
        c.setFont("Helvetica", 8)
        c.drawString(2.5*cm, y - 0.4*cm, solution)
        y -= 0.9*cm


def page5_complete_pinout(c):
    """Page 5: Complete pin assignment summary."""
    draw_header(c, 5)

    y = H - 3.5*cm
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "8. Complete Pin Assignment — All Connections per Node")
    y -= 1*cm

    # Full pin table
    headers = ["Pico Pin", "GPIO", "Function", "Connects To", "Notes"]
    col_widths = [2.2*cm, 2*cm, 3*cm, 3.5*cm, 4.5*cm]
    rows = [
        ("Pin 20", "GP15", "PWM Output", "LED (via 47 Ohm)", "30 kHz, 12-bit"),
        ("Pin 31", "GP26/A0", "ADC Input", "LDR divider midpoint", "12-bit ADC"),
        ("Pin 21", "GP16", "SPI1 RX", "MCP2515 SO (MISO)", "Data from CAN"),
        ("Pin 22", "GP17", "SPI1 CSn", "MCP2515 CS", "Chip select"),
        ("Pin 24", "GP18", "SPI1 SCK", "MCP2515 SCK", "SPI clock"),
        ("Pin 25", "GP19", "SPI1 TX", "MCP2515 SI (MOSI)", "Data to CAN"),
        ("Pin 26", "GP20", "Digital In", "MCP2515 INT", "Interrupt (low)"),
        ("Pin 36", "3V3", "Power", "MCP2515 VCC + LDR VCC", "3.3V rail"),
        ("Pin 23", "GND", "Ground", "MCP2515 GND", "Common GND"),
        ("Pin 28", "GND", "Ground", "LED GND + LDR GND", "Common GND"),
        ("USB", "—", "Serial", "PC (hub node only)", "115200 baud"),
    ]

    # Draw table header
    table_x = 1.5*cm
    row_h = 0.55*cm
    c.setFillColor(HexColor("#2E4057"))
    c.rect(table_x, y - 0.15*cm, sum(col_widths), row_h, fill=1, stroke=0)
    c.setFillColor(white)
    c.setFont("Helvetica-Bold", 7)
    rx = table_x
    for h, w in zip(headers, col_widths):
        c.drawString(rx + 2*mm, y + 1*mm, h)
        rx += w

    y -= row_h

    # Draw rows
    for i, row in enumerate(rows):
        rx = table_x
        ry = y - i * row_h

        if i % 2 == 0:
            c.setFillColor(HexColor("#F5F5F5"))
            c.rect(rx, ry - 0.15*cm, sum(col_widths), row_h, fill=1, stroke=0)

        c.setFillColor(black)
        c.setFont("Helvetica", 7)
        for text, w in zip(row, col_widths):
            c.drawString(rx + 2*mm, ry + 1*mm, text)
            rx += w

    y -= len(rows) * row_h + 1*cm

    # Assembly checklist
    c.setFont("Helvetica-Bold", 12)
    c.drawString(2*cm, y, "9. Assembly Checklist")
    y -= 0.8*cm

    checklist = [
        "[ ] Each Pico has LED circuit on breadboard (GP15 -> 47 Ohm -> LED -> GND)",
        "[ ] Each Pico has LDR circuit on breadboard (3V3 -> LDR -> A0 junction -> 10k -> GND, 10uF cap)",
        "[ ] Each Pico connected to MCP2515 via 7 jumper wires (SPI + power)",
        "[ ] All 3 MCP2515 CANH pins connected together",
        "[ ] All 3 MCP2515 CANL pins connected together",
        "[ ] Termination ON for end nodes (1 and 3), OFF for middle node (2)",
        "[ ] All GND rails connected together",
        "[ ] Hub Pico (any node) connected to PC via USB",
        "[ ] LED and LDR pointing INTO the box (not at each other directly)",
        "[ ] Box interior lined with white paper for light reflection",
        "[ ] Components secured with double-face tape (NOT breadboard sticker)",
        "[ ] Upload firmware to all 3 Picos (hold BOOTSEL, connect USB, upload)",
    ]

    c.setFont("Helvetica", 8)
    for item in checklist:
        c.drawString(2.5*cm, y, item)
        y -= 0.45*cm

    y -= 0.5*cm
    c.setFont("Helvetica-Bold", 9)
    c.setFillColor(PICO_GREEN)
    c.drawString(2*cm, y, "After assembly, run: python scripts/test_boot.py")
    c.setFillColor(black)
    c.setFont("Helvetica", 8)
    y -= 0.4*cm
    c.drawString(2*cm, y, "This will verify all 3 nodes are discovered and communicating.")


def main():
    output_path = "/Users/joaorocha/IST/SCDEEC/Distributed Controller/docs/wiring_guide.pdf"
    c = canvas.Canvas(output_path, pagesize=A4)

    page1_overview(c)
    c.showPage()

    page2_spi_wiring(c)
    c.showPage()

    page3_led_ldr_circuit(c)
    c.showPage()

    page4_canbus_wiring(c)
    c.showPage()

    page5_complete_pinout(c)
    c.showPage()

    c.save()
    print(f"Generated: {output_path}")


if __name__ == "__main__":
    main()
