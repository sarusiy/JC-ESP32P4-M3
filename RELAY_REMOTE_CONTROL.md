# Remote relay control (immobilizer trigger)

Status as of 2026-09-23: hardware build in progress, firmware written and
mid-verification, app side and schematic update not yet started.

## Purpose

Remotely disconnect a physical element that prevents the car from
driving -- starter, fuel pump, or similar -- as the anti-theft
immobilizer mechanism. Not a new relay-driving circuit built from
scratch: reuses an existing, already-owned, off-the-shelf 433MHz
2-relay remote-control kit (each relay board: **12V, 10A**, SRD-style
relay module, originally triggered by the kit's small keyfob remote).
The ESP32-S3 board electrically simulates pressing the keyfob's own
buttons rather than replacing the RF/relay hardware.

**Current wiring**: only 2 relays connected today, but the design is
meant to generalize to more if needed later.

**Current rating caveat, not yet explicitly confirmed**: 10A @ 12V
(120W) is only enough to switch a *control/trigger* wire (e.g. a
starter relay's own coil, typically well under 1A), never a starter
motor's actual cranking current directly (100-300A+, would destroy this
relay). Assumed this is wired to interrupt a control line, not main
current -- worth double-checking against the actual car wiring before
relying on it.

## Signal path: S3 GPIO -> optocoupler -> remote's button contacts

Two GPIOs on the S3, each driving a **4N25** 6-pin optocoupler (confirmed
2026-09-24; same pin layout already used throughout this doc and the
schematic -- 1=Anode, 2=Cathode, 3=NC, 4=Emitter, 5=Collector, 6=Base)
wired across one button's PCB contacts on the keyfob remote's own board
(button A = both relays on, button B = both relays off, per the remote's
own labeling):

- Optocoupler pin 1 (Anode) -> 220-330Ω resistor -> S3 GPIO
- Optocoupler pin 2 (Cathode) -> S3 GND
- Optocoupler pin 3 -> not connected
- Optocoupler pin 4 (Emitter) -> one side of the button's contact pads
- Optocoupler pin 5 (Collector) -> other side of the button's contact pads
- Optocoupler pin 6 (Base) -> not connected

**Planned future substitution (2026-09-24): PC817 (4-pin) instead of
4N25 (6-pin).** Clean swap -- pins 3 (NC) and 6 (Base) were already
unused on the 4N25, so nothing is functionally lost, only the physical
pin *numbers* for Emitter/Collector change:

| Role | 4N25 (current) | PC817 (planned) |
|---|---|---|
| Anode | pin 1 | pin 1 (same) |
| Cathode | pin 2 | pin 2 (same) |
| Emitter | pin 4 | pin 3 |
| Collector | pin 5 | pin 4 |
| NC/Base | pins 3, 6 | don't exist on the 4-pin package |

The logical wiring (GPIO->resistor->Anode, Cathode->GND, Emitter+
Collector->button contact pads) is unchanged -- only update the pin
*numbers* in the schematic/BOM when this swap actually happens, not the
wiring itself.

GPIO high -> LED side of the optocoupler lights -> phototransistor
conducts -> shorts the button's two contacts, electrically identical to
a real finger press. Full galvanic isolation between the S3 and the
remote's own battery-powered circuit.

**GPIO choice**: `RELAY_BUTTON_A_GPIO` = 15, `RELAY_BUTTON_B_GPIO` = 16
(`src/bringup_s3.c`). Picked to avoid every already-reserved S3 pin:
strapping (0/3/45/46), USB (19/20), N16R8's internal octal-PSRAM pins
(33-37), and this board's own CAN (4/5), GPS (6/7), and LED (48).

## Remote board's own power: 5V -> 12V boost converter

The keyfob remote board originally ran off a 12V battery. Rather than
also converting the S3's own 5V rail down to whatever the remote's
logic actually needs, the plan is a 5V->12V **boost** (step-up) DC-DC
converter feeding the remote board directly, so it keeps running exactly
as designed. This is a separate power path from the GPIO/optocoupler
signal path above -- the two don't share a return/reference in a way
that matters, since the optocoupler already isolates them.

**Resolved concern**: the first photographed converter module visually
resembled a buck (step-down) module (single large inductor, adjustable
trim pot, IN/OUT layout typical of LM2596/XL4015-style boards), which
would be physically incapable of producing 12V from a 5V input. Verified
directly against the actual part -- it does step 5V up to 12V, not a
buck converter. No longer a concern.

## Safety interlock: no relay control while the engine could be running

Explicit requirement: the final product must refuse any relay command
while driving, and only allow it when the car is stationary (not just
"ignition off" loosely -- the goal is to never let this cut the
starter/fuel-pump/etc. while actually in motion).

**Implemented in firmware** (`relay_safe_to_actuate()` /
`relay_http_handler()` in `src/bringup_s3.c`), not left as an app-side-only
check, since the firmware is the real enforcement boundary:

- Requires a **fresh** RPM reading (`RELAY_RPM_STALE_US` = 5 seconds) --
  tracked via a new `s_obd_rpm_updated_us` timestamp, set every time
  `s_obd_state.rpm` is written (both the active Mode 01 PID 0x0C path and
  the simulator's passive-broadcast decode path).
- Requires that fresh reading to be exactly **0**.
- **Fails closed**: if OBD polling isn't currently succeeding (Passive
  CAN mode, board just booted, connection dropped, car not responding),
  the request is refused -- explicitly not treated as "no data means
  probably safe." The dangerous failure mode here is a false negative
  (thinking the engine is off when it's actually running), so "unknown"
  must mean "refused," not "allowed."
- A refused request returns HTTP 409 with the actual RPM and how stale
  the last reading was, for debugging.

**Decided 2026-09-24: no ignition-sense hardware.** The Quadlock
media-connector tap (see
`JC-ESP32P4-M3/captures/fabia_2026/QUADLOCK_MEDIA_TAP.md`) was confirmed
to expose only constant B+ and ground -- no switched-ignition pin there,
and no other convenient switched-12V tap point has been found. Rather
than adding a separate physical ignition wire, the interlock stays
**CAN-bus-only**: the existing `relay_safe_to_actuate()` RPM gate
described above, reading over the S3's existing OBD/CAN link. The
previously-planned ignition-sense optocoupler (U3/R5/D1/R6, GPIO17) has
been removed from the schematic and firmware plan.

This does mean the gate still depends on CAN/OBD polling staying
healthy -- accepted tradeoff, since it fails closed (see above: no data
means refused, not allowed) rather than failing open. If the Quadlock
tap's claimed `0x2C3` ignition-state CAN ID is ever independently
verified (see that doc's Part 2 -- currently unverified), it could be
added as a second CAN-based signal alongside RPM, still with no extra
hardware.

## Schematic

`hardware/relay_ignition_control.kicad_sch` -- a separate sheet from
`hardware/bringup_wiring.kicad_sch` (kept untouched, still independently
ERC-clean) covering everything on this page: the two relay-button
optocoupler drivers (U1/U2, from GPIO15/16, R3/R4 = 220Ω series
resistors), the S3 GPIO/5V/3V3/GND breakout (J5), connectors documenting
where each thing connects (remote board button contacts J6/J7, the
5V->12V boost converter's in/out J9/J10). No ignition-sense circuitry on
this sheet (see Safety interlock above). Validated the same way as the
original sheet -- `kicad-cli sch erc`, 0 errors, only the same benign
warning categories (off-grid stubs, unregistered custom symbol library).

Generated via `hardware/gen_relay_sch.py` rather than hand-written --
worth re-running that script (after editing it) for any future change
to this sheet, rather than hand-editing the `.kicad_sch` directly, since
it's what keeps the pin-position math and net labels consistent.

## Firmware API

`POST /api/relay`, plain-text body `"on"` (button A) or `"off"` (button
B). Returns `200 OK on` / `200 OK off` on success, or `409 Conflict`
with a human-readable reason if the safety interlock refuses it.

## Open items

- [x] Firmware build verification -- clean build (2026-09-23). First
      attempt hit an unrelated toolchain internal-compiler-error
      (segfault compiling `esp_lcd_panel_rgb.c`, a file this project
      doesn't use); a plain retry succeeded, confirming it was transient,
      not a real issue. Not yet flashed/tested on real hardware.
- [ ] Confirm the two relay boards' 10A rating is actually wired to a
      control/trigger line, not the starter's main current path.
- [ ] App-side UI: control each relay individually (first phase, per
      explicit request) -- not started.
- [x] Schematic -- `hardware/relay_ignition_control.kicad_sch`, 0 ERC
      errors, covers relay-button drivers and the boost converter's
      connections only (no ignition-sense hardware -- see Safety
      interlock above). Not yet visually confirmed by opening it in
      KiCad.
