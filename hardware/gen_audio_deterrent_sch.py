#!/usr/bin/env python3
"""Generates audio_deterrent.kicad_sch: the in-cabin deterrent audio
subsystem (2x MAX98357A I2S amps, SD-pin channel-select resistors, the
front-speaker relay module's control side). Separate sheet from
bringup_wiring.kicad_sch and relay_ignition_control.kicad_sch (both kept
untouched/independently ERC-clean). Same proven approach as those: self-
defined simple symbols, 2-point-only wires, short stub+label per net.
"""
uuids = [l.strip() for l in open("uuids3.txt", encoding="utf-8") if l.strip()]
uid_iter = iter(uuids)
def nu():
    return next(uid_iter)

ROOT_UUID = nu()
PROJECT_NAME = "audio_deterrent"

def fmt(v):
    s = f"{v:.10f}".rstrip("0").rstrip(".")
    return s if s else "0"

def conn_symbol(n, half_height):
    pins = []
    for i in range(n):
        y = half_height - i * 2.54
        pins.append(
            f'\t\t\t(pin passive line (at -5.08 {fmt(y)} 0) (length 3.81) '
            f'(name "Pin_{i+1}" (effects (font (size 1.27 1.27)))) '
            f'(number "{i+1}" (effects (font (size 1.27 1.27)))))'
        )
    return "\n".join(pins)

def conn_lib_symbol(n):
    half_height = (n - 1) * 1.27
    body_half = half_height + 1.27
    return f'''\t\t(symbol "Local:Conn_01x{n:02d}"
\t\t\t(pin_numbers (hide yes))
\t\t\t(pin_names (offset 1.016) (hide yes))
\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)
\t\t\t(property "Reference" "J" (at 0 {fmt(body_half + 2.54)} 0) (effects (font (size 1.27 1.27))))
\t\t\t(property "Value" "Conn_01x{n:02d}" (at 0 {fmt(-(body_half + 2.54))} 0) (effects (font (size 1.27 1.27))))
\t\t\t(symbol "Conn_01x{n:02d}_0_1"
\t\t\t\t(rectangle (start -1.27 {fmt(-body_half)}) (end 1.27 {fmt(body_half)}) (stroke (width 0.254) (type default)) (fill (type background)))
\t\t\t)
\t\t\t(symbol "Conn_01x{n:02d}_1_1"
{conn_symbol(n, half_height)}
\t\t\t)
\t\t\t(embedded_fonts no)
\t\t)
'''

HEADER = f'''(kicad_sch
\t(version 20250114)
\t(generator "eeschema")
\t(generator_version "9.0")
\t(uuid "{ROOT_UUID}")
\t(paper "A4")
\t(title_block
\t\t(title "In-cabin audio deterrent")
\t\t(date "2026-09-24")
\t\t(rev "A")
\t\t(comment 1 "Separate sheet from bringup_wiring.kicad_sch and relay_ignition_control.kicad_sch -- 2x MAX98357A I2S amps, SD channel-select resistors, front-speaker relay control side. COM/NC and speaker/radio wiring stay on the car side, not shown here -- see QUADLOCK_MEDIA_TAP.md.")
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
{conn_lib_symbol(3)}{conn_lib_symbol(4)}{conn_lib_symbol(7)}{conn_lib_symbol(8)}\t)
'''

TAIL = '''\t(sheet_instances
\t\t(path "/"
\t\t\t(page "1")
\t\t)
\t)
\t(embedded_fonts no)
)
'''

components = []
wires = []
labels = []

def add_wire(p1, p2):
    wires.append((p1, p2))

def add_label(text, pos):
    labels.append((text, pos))

def instance(lib_id, ref, value, at, pins, ref_offset=(0, -8), value_offset=(0, 8)):
    x, y, angle = at
    body = [f'\t(symbol\n\t\t(lib_id "{lib_id}")\n\t\t(at {fmt(x)} {fmt(y)} {angle})\n\t\t(unit 1)\n\t\t(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)']
    body.append(f'\t\t(uuid "{nu()}")')
    body.append(f'\t\t(property "Reference" "{ref}" (at {fmt(x+ref_offset[0])} {fmt(y+ref_offset[1])} 0) (effects (font (size 1.27 1.27))))')
    body.append(f'\t\t(property "Value" "{value}" (at {fmt(x+value_offset[0])} {fmt(y+value_offset[1])} 0) (effects (font (size 1.27 1.27))))')
    for pnum in pins:
        body.append(f'\t\t(pin "{pnum}" (uuid "{nu()}"))')
    body.append(f'\t\t(instances\n\t\t\t(project "{PROJECT_NAME}"\n\t\t\t\t(path "/{ROOT_UUID}"\n\t\t\t\t\t(reference "{ref}")\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)')
    body.append('\t)')
    return '\n'.join(body)

def world(ox, oy, lx, ly):
    return (ox + lx, oy - ly)

def pin_pos(origin, n, index):
    half_height = (n - 1) * 1.27
    local_y = half_height - index * 2.54
    return world(origin[0], origin[1], -5.08, local_y)

def stub_and_label(pin_pos_, text, direction=(-1, 0), length=2.54):
    far = (pin_pos_[0] + direction[0] * length, pin_pos_[1] + direction[1] * length)
    add_wire(pin_pos_, far)
    add_label(text, far)

# ---- J1: GPIO breakout from S3 (BCLK, LRCLK, DIN, relay trigger) ----
J1_AT = (30, 40, 0)
components.append(instance("Local:Conn_01x04", "J1", "S3 GPIO breakout (I2S + relay trigger)", J1_AT, ["1", "2", "3", "4"], ref_offset=(-5, -10), value_offset=(0, 11)))
j1_names = ["BCLK", "LRCLK", "DIN", "RELAY_TRIG"]
for i, name in enumerate(j1_names):
    stub_and_label(pin_pos(J1_AT, 4, i), name)

# ---- J2: Power breakout from S3 (3V3, 5V, GND) ----
J2_AT = (30, 65, 0)
components.append(instance("Local:Conn_01x03", "J2", "S3 power breakout", J2_AT, ["1", "2", "3"], ref_offset=(-5, -8), value_offset=(0, 9)))
j2_names = ["3V3", "5V", "GND"]
for i, name in enumerate(j2_names):
    stub_and_label(pin_pos(J2_AT, 3, i), name)

# ---- U1: Left MAX98357A (BCLK, LRC, DIN, GAIN, SD, GND, V+, SPK+, SPK-) ----
# real board has 7 header pins + 2-pin speaker screw terminal = 9 total,
# but GAIN is left unconnected (not wired to anything), so only 8 are
# actually wired here -- represented as an 8-pin connector, GAIN omitted
# from the list entirely (matches "not connected" -- no floating pin
# needed on our own simplified connector symbol).
U1_AT = (65, 30, 0)
u1_pins = ["BCLK", "LRC", "DIN", "SD", "GND", "V+", "SPK+", "SPK-"]
components.append(instance("Local:Conn_01x08", "U1", "MAX98357A (Left channel)", U1_AT, [str(i) for i in range(1, 9)], ref_offset=(-5, -11), value_offset=(0, 12)))
u1_targets = ["BCLK", "LRCLK", "DIN", "U1_SD", "GND", "5V", "U1_SPK_P", "U1_SPK_N"]
for i, name in enumerate(u1_targets):
    stub_and_label(pin_pos(U1_AT, 8, i), name)

# ---- U2: Right MAX98357A ----
U2_AT = (65, 60, 0)
components.append(instance("Local:Conn_01x08", "U2", "MAX98357A (Right channel)", U2_AT, [str(i) for i in range(1, 9)], ref_offset=(-5, -11), value_offset=(0, 12)))
u2_targets = ["BCLK", "LRCLK", "DIN", "U2_SD", "GND", "5V", "U2_SPK_P", "U2_SPK_N"]
for i, name in enumerate(u2_targets):
    stub_and_label(pin_pos(U2_AT, 8, i), name)

# ---- R1: SD pull, Left board, 100k to 3.3V ----
R1_AT = (90, 20, 0)
components.append(instance("Device:R", "R1", "100k", R1_AT, ["1", "2"], ref_offset=(3, -3), value_offset=(3, 3)))
r1p1 = world(*R1_AT[:2], 0, 3.81); add_label("3V3", (r1p1[0], r1p1[1]+2.54)); add_wire(r1p1, (r1p1[0], r1p1[1]+2.54))
r1p2 = world(*R1_AT[:2], 0, -3.81); add_label("U1_SD", (r1p2[0], r1p2[1]-2.54)); add_wire(r1p2, (r1p2[0], r1p2[1]-2.54))

# ---- R2: SD pull, Right board, 220k to 3.3V ----
R2_AT = (90, 75, 0)
components.append(instance("Device:R", "R2", "220k", R2_AT, ["1", "2"], ref_offset=(3, -3), value_offset=(3, 3)))
r2p1 = world(*R2_AT[:2], 0, 3.81); add_label("3V3", (r2p1[0], r2p1[1]+2.54)); add_wire(r2p1, (r2p1[0], r2p1[1]+2.54))
r2p2 = world(*R2_AT[:2], 0, -3.81); add_label("U2_SD", (r2p2[0], r2p2[1]-2.54)); add_wire(r2p2, (r2p2[0], r2p2[1]-2.54))

# ---- J3: speaker relay module (control side + NO audio pass-through) ----
J3_AT = (100, 45, 0)
components.append(instance("Local:Conn_01x08", "J3", "2ch relay module (speaker switch)", J3_AT, [str(i) for i in range(1, 9)], ref_offset=(-5, -11), value_offset=(0, 12)))
j3_names = ["IN1", "IN2", "VCC", "GND", "L_NO_P", "L_NO_N", "R_NO_P", "R_NO_N"]
j3_targets = ["RELAY_TRIG", "RELAY_TRIG", "5V", "GND", "U1_SPK_P", "U1_SPK_N", "U2_SPK_P", "U2_SPK_N"]
for i, name in enumerate(j3_targets):
    stub_and_label(pin_pos(J3_AT, 8, i), name, direction=(1, 0))

notes = [
    ('R1/R2 select which channel each MAX98357A plays via its SD pin (Adafruit MAX98357 guide): ~100k to 3.3V = Left, ~210k (220k nearest standard value) to 3.3V = Right.', (20, 90)),
    ('GAIN pin on both U1/U2 left unconnected -- default gain, per board default.', (20, 95)),
    ('J3 IN1/IN2 both tied to the same GPIO (RELAY_TRIG) so both channels switch together -- this is 2 independent SPDT relays on one board, not a true single-coil DPDT.', (20, 100)),
    ('J3 COM/NC pins (to the physical speakers and factory radio tap) are NOT shown here -- that wiring stays entirely on the car side. See QUADLOCK_MEDIA_TAP.md.', (20, 105)),
    ("Low Level Trigger relay module: RELAY_TRIG HIGH = idle (NC, factory radio, safe default), LOW = active (NO, this system's audio).", (20, 110)),
]

out = [HEADER]
out.extend(components)
for p1, p2 in wires:
    out.append(f'\t(wire\n\t\t(pts (xy {fmt(p1[0])} {fmt(p1[1])}) (xy {fmt(p2[0])} {fmt(p2[1])}))\n\t\t(stroke (width 0) (type solid))\n\t\t(uuid "{nu()}")\n\t)')
for text, pos in labels:
    out.append(f'\t(label "{text}"\n\t\t(at {fmt(pos[0])} {fmt(pos[1])} 0)\n\t\t(effects (font (size 1.27 1.27)) (justify left bottom))\n\t\t(uuid "{nu()}")\n\t)')
for text, pos in notes:
    out.append(f'\t(text "{text}"\n\t\t(at {fmt(pos[0])} {fmt(pos[1])} 0)\n\t\t(effects (font (size 1.27 1.27)))\n\t\t(uuid "{nu()}")\n\t)')
out.append(TAIL)

result = "\n".join(out) + "\n"
open("audio_deterrent.kicad_sch", "w", encoding="utf-8").write(result)
print("written, remaining uuids:", len(list(uid_iter)))
