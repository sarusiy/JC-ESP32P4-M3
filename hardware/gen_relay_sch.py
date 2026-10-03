#!/usr/bin/env python3
"""Generates relay_ignition_control.kicad_sch: the optocoupler-isolated
relay-button driver + ignition-sense addon circuit, as a separate sheet
from bringup_wiring.kicad_sch (kept untouched/still ERC-clean on its own).
Uses the same proven approach as bringup_wiring.kicad_sch: self-defined
simple symbols, 2-point-only wires, short stub+label per net instead of
long routed wires (avoids the collision-avoidance complexity entirely).
"""
import sys

uuids = [l.strip() for l in open("uuids2.txt", encoding="utf-8") if l.strip()]
uid_iter = iter(uuids)
def nu():
    return next(uid_iter)

ROOT_UUID = nu()
PROJECT_NAME = "relay_ignition_control"

def fmt(v):
    s = f"{v:.10f}".rstrip("0").rstrip(".")
    return s if s else "0"

HEADER = f'''(kicad_sch
\t(version 20250114)
\t(generator "eeschema")
\t(generator_version "9.0")
\t(uuid "{ROOT_UUID}")
\t(paper "A4")
\t(title_block
\t\t(title "Relay remote control + ignition-sense addon")
\t\t(date "2026-09-23")
\t\t(rev "A")
\t\t(comment 1 "Separate sheet from bringup_wiring.kicad_sch -- optocoupler-isolated GPIO->relay-remote button driver. Safety interlock is CAN-bus RPM only (firmware), no ignition-sense hardware.")
\t\t(comment 2 "Run Inspect > Electrical Rules Checker before trusting this netlist.")
\t)
\t(lib_symbols
\t\t(symbol "Device:R"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 0))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "R" (at 2.032 0 90) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "R" (at 0 0 90) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "R_0_1"
\t\t\t\t(rectangle (start -1.016 -2.54) (end 1.016 2.54) (stroke (width 0.254) (type default)) (fill (type none)))
\t\t\t)
\t\t\t(symbol "R_1_1"
\t\t\t\t(pin passive line (at 0 3.81 270) (length 1.27) (name "~" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at 0 -3.81 90) (length 1.27) (name "~" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t\t(symbol "Local:Diode"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 0))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "D" (at 2.032 0 90) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "1N4148" (at 0 0 90) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Diode_0_1"
\t\t\t\t(rectangle (start -1.016 -2.54) (end 1.016 2.54) (stroke (width 0.254) (type default)) (fill (type none)))
\t\t\t)
\t\t\t(symbol "Diode_1_1"
\t\t\t\t(pin passive line (at 0 3.81 270) (length 1.27) (name "A" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at 0 -3.81 90) (length 1.27) (name "K" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t\t(symbol "Local:Conn_01x02"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 1.016) (hide yes))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "J" (at 0 3.81 0) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "Conn_01x02" (at 0 -3.81 0) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Conn_01x02_0_1"
\t\t\t\t(rectangle (start -1.27 -2.54) (end 1.27 2.54) (stroke (width 0.254) (type default)) (fill (type background)))
\t\t\t)
\t\t\t(symbol "Conn_01x02_1_1"
\t\t\t\t(pin passive line (at -5.08 1.27 0) (length 3.81) (name "Pin_1" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -1.27 0) (length 3.81) (name "Pin_2" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t\t(symbol "Local:Conn_01x04"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 1.016) (hide yes))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "J" (at 0 6.35 0) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "Conn_01x04" (at 0 -6.35 0) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Conn_01x04_0_1"
\t\t\t\t(rectangle (start -1.27 -5.08) (end 1.27 5.08) (stroke (width 0.254) (type default)) (fill (type background)))
\t\t\t)
\t\t\t(symbol "Conn_01x04_1_1"
\t\t\t\t(pin passive line (at -5.08 3.81 0) (length 3.81) (name "Pin_1" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 1.27 0) (length 3.81) (name "Pin_2" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -1.27 0) (length 3.81) (name "Pin_3" (effects (font (size 1.27 1.27)))) (number "3" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -3.81 0) (length 3.81) (name "Pin_4" (effects (font (size 1.27 1.27)))) (number "4" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t\t(symbol "Local:Conn_01x05"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 1.016) (hide yes))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "J" (at 0 7.62 0) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "Conn_01x05" (at 0 -7.62 0) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Conn_01x05_0_1"
\t\t\t\t(rectangle (start -1.27 -6.35) (end 1.27 6.35) (stroke (width 0.254) (type default)) (fill (type background)))
\t\t\t)
\t\t\t(symbol "Conn_01x05_1_1"
\t\t\t\t(pin passive line (at -5.08 5.08 0) (length 3.81) (name "Pin_1" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 2.54 0) (length 3.81) (name "Pin_2" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 0 0) (length 3.81) (name "Pin_3" (effects (font (size 1.27 1.27)))) (number "3" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -2.54 0) (length 3.81) (name "Pin_4" (effects (font (size 1.27 1.27)))) (number "4" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -5.08 0) (length 3.81) (name "Pin_5" (effects (font (size 1.27 1.27)))) (number "5" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t\t(symbol "Local:Conn_01x06"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 1.016) (hide yes))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "J" (at 0 8.89 0) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "Conn_01x06" (at 0 -8.89 0) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Conn_01x06_0_1"
\t\t\t\t(rectangle (start -1.27 -7.62) (end 1.27 7.62) (stroke (width 0.254) (type default)) (fill (type background)))
\t\t\t)
\t\t\t(symbol "Conn_01x06_1_1"
\t\t\t\t(pin passive line (at -5.08 6.35 0) (length 3.81) (name "Pin_1" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 3.81 0) (length 3.81) (name "Pin_2" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 1.27 0) (length 3.81) (name "Pin_3" (effects (font (size 1.27 1.27)))) (number "3" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -1.27 0) (length 3.81) (name "Pin_4" (effects (font (size 1.27 1.27)))) (number "4" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -3.81 0) (length 3.81) (name "Pin_5" (effects (font (size 1.27 1.27)))) (number "5" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -6.35 0) (length 3.81) (name "Pin_6" (effects (font (size 1.27 1.27)))) (number "6" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t\t(symbol "Local:Optocoupler6"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 1.016) (hide yes))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "U" (at 0 8.89 0) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "Optocoupler (6-pin)" (at 0 -8.89 0) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Optocoupler6_0_1"
\t\t\t\t(rectangle (start -1.27 -6.35) (end 1.27 6.35) (stroke (width 0.254) (type default)) (fill (type background)))
\t\t\t)
\t\t\t(symbol "Optocoupler6_1_1"
\t\t\t\t(pin passive line (at -5.08 5.08 0) (length 3.81) (name "Anode" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 0 0) (length 3.81) (name "Cathode" (effects (font (size 1.27 1.27)))) (number "2" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at -5.08 -5.08 0) (length 3.81) (name "NC" (effects (font (size 1.27 1.27)))) (number "3" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at 5.08 -5.08 180) (length 3.81) (name "Emitter" (effects (font (size 1.27 1.27)))) (number "4" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at 5.08 0 180) (length 3.81) (name "Collector" (effects (font (size 1.27 1.27)))) (number "5" (effects (font (size 1.27 1.27)))))
\t\t\t\t(pin passive line (at 5.08 5.08 180) (length 3.81) (name "Base" (effects (font (size 1.27 1.27)))) (number "6" (effects (font (size 1.27 1.27)))))
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
\t)
'''

TAIL = '''\t(sheet_instances
\t\t(path "/"
\t\t\t(page "1")
\t\t)
\t)
\t(embedded_fonts no)
)
'''

def instance(lib_id, ref, value, at, pins, ref_offset=(0, -8), value_offset=(0, 8)):
    x, y, angle = at
    body = []
    body.append(f'\t(symbol\n\t\t(lib_id "{lib_id}")\n\t\t(at {fmt(x)} {fmt(y)} {angle})\n\t\t(unit 1)\n\t\t(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)')
    u = nu()
    body.append(f'\t\t(uuid "{u}")')
    body.append(f'\t\t(property "Reference" "{ref}" (at {fmt(x+ref_offset[0])} {fmt(y+ref_offset[1])} 0) (effects (font (size 1.27 1.27))))')
    body.append(f'\t\t(property "Value" "{value}" (at {fmt(x+value_offset[0])} {fmt(y+value_offset[1])} 0) (effects (font (size 1.27 1.27))))')
    pin_uuids = {}
    for pnum in pins:
        pu = nu()
        pin_uuids[pnum] = pu
        body.append(f'\t\t(pin "{pnum}" (uuid "{pu}"))')
    body.append(f'\t\t(instances\n\t\t\t(project "{PROJECT_NAME}"\n\t\t\t\t(path "/{ROOT_UUID}"\n\t\t\t\t\t(reference "{ref}")\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)')
    body.append('\t)')
    return '\n'.join(body), pin_uuids

# world pin position for angle=0 / angle=180 placements using the same
# Y-flip formula already verified in bringup_wiring.kicad_sch: for a
# symbol placed at (ox,oy,0), a pin at local (lx,ly) lands at world
# (ox+lx, oy-ly).
def world(ox, oy, lx, ly):
    return (ox + lx, oy - ly)

components = []   # list of (block_text,)
wires = []         # list of (p1,p2)
labels = []        # list of (text, pos)

def add_wire(p1, p2):
    wires.append((p1, p2))

def add_label(text, pos):
    labels.append((text, pos))

no_connects = []
def add_no_connect(pos):
    no_connects.append(pos)

def stub_and_label(pin_pos, text, direction=(1, 0), length=2.54):
    """Short 2-point wire from the pin out in `direction`, label at the far end."""
    far = (pin_pos[0] + direction[0] * length, pin_pos[1] + direction[1] * length)
    add_wire(pin_pos, far)
    add_label(text, far)

# ---- J5: GPIO breakout (GPIO15, GPIO16, 5V, GND) ----
# No GPIO17 / ignition-sense pin, no 3V3 pin -- the safety interlock is
# CAN-bus RPM only (relay_safe_to_actuate() in firmware), no hardware
# ignition tap, and nothing on this sheet needs the 3.3V rail directly.
J5_AT = (40, 50, 0)
block, pins = instance("Local:Conn_01x04", "J5", "S3 GPIO breakout (15/16) + 5V + GND", J5_AT, ["1", "2", "3", "4"], ref_offset=(-5, -10), value_offset=(0, 12))
components.append(block)
p1 = world(*J5_AT[:2], -5.08, 3.81); stub_and_label(p1, "GPIO15", direction=(-1, 0))
p2 = world(*J5_AT[:2], -5.08, 1.27); stub_and_label(p2, "GPIO16", direction=(-1, 0))
p3 = world(*J5_AT[:2], -5.08, -1.27); stub_and_label(p3, "5V", direction=(-1, 0))
p4 = world(*J5_AT[:2], -5.08, -3.81); stub_and_label(p4, "GND", direction=(-1, 0))

# ---- R3: GPIO15 series resistor (into U1 anode) ----
R3_AT = (70, 44, 0)
block, pins = instance("Device:R", "R3", "220R", R3_AT, ["1", "2"], ref_offset=(3, -3), value_offset=(3, 3))
components.append(block)
r3p1 = world(*R3_AT[:2], 0, 3.81)
r3p2 = world(*R3_AT[:2], 0, -3.81)
add_label("GPIO15", (r3p1[0], r3p1[1] + 2.54)); add_wire(r3p1, (r3p1[0], r3p1[1] + 2.54))
add_label("OPTO_A_ANODE", (r3p2[0], r3p2[1] - 2.54)); add_wire(r3p2, (r3p2[0], r3p2[1] - 2.54))

# ---- R4: GPIO16 series resistor (into U2 anode) ----
R4_AT = (70, 60, 0)
block, pins = instance("Device:R", "R4", "220R", R4_AT, ["1", "2"], ref_offset=(3, -3), value_offset=(3, 3))
components.append(block)
r4p1 = world(*R4_AT[:2], 0, 3.81)
r4p2 = world(*R4_AT[:2], 0, -3.81)
add_label("GPIO16", (r4p1[0], r4p1[1] + 2.54)); add_wire(r4p1, (r4p1[0], r4p1[1] + 2.54))
add_label("OPTO_B_ANODE", (r4p2[0], r4p2[1] - 2.54)); add_wire(r4p2, (r4p2[0], r4p2[1] - 2.54))

# ---- U1: relay button A optocoupler ----
U1_AT = (90, 44, 0)
block, pins = instance("Local:Optocoupler6", "U1", "Relay button A driver", U1_AT, [str(i) for i in range(1, 7)], ref_offset=(-3, -10), value_offset=(0, 11))
components.append(block)
u1p1 = world(*U1_AT[:2], -5.08, 5.08); add_label("OPTO_A_ANODE", (u1p1[0]-2.54, u1p1[1])); add_wire(u1p1, (u1p1[0]-2.54, u1p1[1]))
u1p2 = world(*U1_AT[:2], -5.08, 0);    add_label("GND", (u1p2[0]-2.54, u1p2[1])); add_wire(u1p2, (u1p2[0]-2.54, u1p2[1]))
u1p4 = world(*U1_AT[:2], 5.08, -5.08); add_label("BTN_A_1", (u1p4[0]+2.54, u1p4[1])); add_wire(u1p4, (u1p4[0]+2.54, u1p4[1]))
u1p5 = world(*U1_AT[:2], 5.08, 0);     add_label("BTN_A_2", (u1p5[0]+2.54, u1p5[1])); add_wire(u1p5, (u1p5[0]+2.54, u1p5[1]))
add_no_connect(world(*U1_AT[:2], -5.08, -5.08))
add_no_connect(world(*U1_AT[:2], 5.08, 5.08))

# ---- U2: relay button B optocoupler ----
U2_AT = (90, 60, 0)
block, pins = instance("Local:Optocoupler6", "U2", "Relay button B driver", U2_AT, [str(i) for i in range(1, 7)], ref_offset=(-3, -10), value_offset=(0, 11))
components.append(block)
u2p1 = world(*U2_AT[:2], -5.08, 5.08); add_label("OPTO_B_ANODE", (u2p1[0]-2.54, u2p1[1])); add_wire(u2p1, (u2p1[0]-2.54, u2p1[1]))
u2p2 = world(*U2_AT[:2], -5.08, 0);    add_label("GND", (u2p2[0]-2.54, u2p2[1])); add_wire(u2p2, (u2p2[0]-2.54, u2p2[1]))
u2p4 = world(*U2_AT[:2], 5.08, -5.08); add_label("BTN_B_1", (u2p4[0]+2.54, u2p4[1])); add_wire(u2p4, (u2p4[0]+2.54, u2p4[1]))
u2p5 = world(*U2_AT[:2], 5.08, 0);     add_label("BTN_B_2", (u2p5[0]+2.54, u2p5[1])); add_wire(u2p5, (u2p5[0]+2.54, u2p5[1]))
add_no_connect(world(*U2_AT[:2], -5.08, -5.08))
add_no_connect(world(*U2_AT[:2], 5.08, 5.08))

# ---- J6 / J7: remote board button contact pads ----
J6_AT = (115, 44, 0)
block, pins = instance("Local:Conn_01x02", "J6", "Remote board: button A contacts", J6_AT, ["1", "2"], ref_offset=(3, -5), value_offset=(3, 6))
components.append(block)
j6p1 = world(*J6_AT[:2], -5.08, 1.27); add_label("BTN_A_1", (j6p1[0]-2.54, j6p1[1])); add_wire(j6p1, (j6p1[0]-2.54, j6p1[1]))
j6p2 = world(*J6_AT[:2], -5.08, -1.27); add_label("BTN_A_2", (j6p2[0]-2.54, j6p2[1])); add_wire(j6p2, (j6p2[0]-2.54, j6p2[1]))

J7_AT = (115, 60, 0)
block, pins = instance("Local:Conn_01x02", "J7", "Remote board: button B contacts", J7_AT, ["1", "2"], ref_offset=(3, -5), value_offset=(3, 6))
components.append(block)
j7p1 = world(*J7_AT[:2], -5.08, 1.27); add_label("BTN_B_1", (j7p1[0]-2.54, j7p1[1])); add_wire(j7p1, (j7p1[0]-2.54, j7p1[1]))
j7p2 = world(*J7_AT[:2], -5.08, -1.27); add_label("BTN_B_2", (j7p2[0]-2.54, j7p2[1])); add_wire(j7p2, (j7p2[0]-2.54, j7p2[1]))

# ---- J9: 5V+GND from S3 -> boost converter IN ----
J9_AT = (40, 120, 0)
block, pins = instance("Local:Conn_01x02", "J9", "5V->12V boost converter, IN", J9_AT, ["1", "2"], ref_offset=(-5, -5), value_offset=(0, 6))
components.append(block)
j9p1 = world(*J9_AT[:2], -5.08, 1.27); add_label("5V", (j9p1[0]-2.54, j9p1[1])); add_wire(j9p1, (j9p1[0]-2.54, j9p1[1]))
j9p2 = world(*J9_AT[:2], -5.08, -1.27); add_label("GND", (j9p2[0]-2.54, j9p2[1])); add_wire(j9p2, (j9p2[0]-2.54, j9p2[1]))

# ---- J10: boost converter OUT (12V) -> remote board power ----
J10_AT = (70, 120, 0)
block, pins = instance("Local:Conn_01x02", "J10", "Boost converter OUT -> remote board power", J10_AT, ["1", "2"], ref_offset=(3, -5), value_offset=(3, 6))
components.append(block)
j10p1 = world(*J10_AT[:2], -5.08, 1.27); add_label("REMOTE_12V", (j10p1[0]-2.54, j10p1[1])); add_wire(j10p1, (j10p1[0]-2.54, j10p1[1]))
j10p2 = world(*J10_AT[:2], -5.08, -1.27); add_label("CAR_GND", (j10p2[0]-2.54, j10p2[1])); add_wire(j10p2, (j10p2[0]-2.54, j10p2[1]))

# ---- notes ----
notes = [
    ("R3/R4 = 220R: sized for the S3's 3.3V GPIO driving U1/U2's LED side (~6-10mA).", (30, 20)),
    ("U1/U2 pins 3 and 6 intentionally not connected (NC/Base), matching the datasheet's simple-switch usage.", (30, 25)),
    ("Safety interlock is CAN-bus RPM only (relay_safe_to_actuate() in firmware, existing OBD/CAN link) -- no ignition-sense hardware on this sheet.", (30, 30)),
]

out = [HEADER]
out.extend(components)
for p1, p2 in wires:
    u = nu()
    out.append(f'\t(wire\n\t\t(pts (xy {fmt(p1[0])} {fmt(p1[1])}) (xy {fmt(p2[0])} {fmt(p2[1])}))\n\t\t(stroke (width 0) (type solid))\n\t\t(uuid "{u}")\n\t)')
for text, pos in labels:
    u = nu()
    out.append(f'\t(label "{text}"\n\t\t(at {fmt(pos[0])} {fmt(pos[1])} 0)\n\t\t(effects (font (size 1.27 1.27)) (justify left bottom))\n\t\t(uuid "{u}")\n\t)')
for pos in no_connects:
    u = nu()
    out.append(f'\t(no_connect\n\t\t(at {fmt(pos[0])} {fmt(pos[1])})\n\t\t(uuid "{u}")\n\t)')
for text, pos in notes:
    u = nu()
    out.append(f'\t(text "{text}"\n\t\t(at {fmt(pos[0])} {fmt(pos[1])} 0)\n\t\t(effects (font (size 1.27 1.27)))\n\t\t(uuid "{u}")\n\t)')
out.append(TAIL)

result = "\n".join(out) + "\n"
open("relay_ignition_control.kicad_sch", "w", encoding="utf-8").write(result)
print("written, remaining uuids:", len(list(uid_iter)))
