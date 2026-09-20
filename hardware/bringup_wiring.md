# Bring-up wiring reference

Companion to `bringup_wiring.kicad_sch`. Documents current bench wiring
only — this is not a designed PCB, and the schematic uses generic
`Conn_01xN` header symbols (not exact vendor footprints) for J1/J2/J4,
since J1/J2 are off-the-shelf dev-board/breakout headers, not custom
connectors.

## J1 — ESP32-S3-WROOM-1 N16R8 dev board header

| Pin | Board label | Net           |
|-----|-------------|---------------|
| 1   | 3V3         | 3V3           |
| 2   | GND         | GND           |
| 3   | GPIO4       | CAN_TXD       |
| 4   | GPIO5       | CAN_RXD       |
| 5   | GPIO6       | not connected (spare) |
| 6   | GPIO7       | GPIO7_DIV (GPS divider midpoint) |

## J2 — SN65HVD230 CAN transceiver breakout

| Pin | Module label | Net     |
|-----|--------------|---------|
| 1   | VCC          | 3V3     |
| 2   | GND          | GND     |
| 3   | TXD          | CAN_TXD |
| 4   | RXD          | CAN_RXD |
| 5   | CANH         | CANH    |
| 6   | CANL         | CANL    |

CANH/CANL go out to J3, the physical CAN bus connector (2-pin, to the
car's OBD-II CAN_H/CAN_L). Modeled as one 6-pin connector even though
most SN65HVD230 breakouts split these across two physical headers — the
electrical connections are the same either way.

## J4 — GPS module TX/GND

| Pin | Net       | Notes                         |
|-----|-----------|--------------------------------|
| 1   | GPS_TX_5V | GPS module's 5V UART TX output |
| 2   | GND       | shared ground                  |

GPS_TX_5V feeds a resistor divider (R1 = 10k top, R2 = 15k bottom to
GND) before reaching J1 pin 6 (GPIO7): 5V * 15k/(10k+15k) = 3.0V, a
safe ESP32 logic-high level. Same 10k/15k ratio already used elsewhere
in this project for other 5V-to-3.3V signals (see
`JC-ESP32P4-M3/src/main.c:77`).

GPS module VCC/RX are not shown — this schematic covers only the TX
line, which is the one that needed level-shifting.

## Not included

- Onboard WS2812 LED (GPIO48) — internal to the dev board, no external wiring.
- Power input to the dev board itself (USB) — out of scope for this diagram.

## Verifying this file

I hand-wrote the raw `.kicad_sch`/`.kicad_pro` rather than generating
them from KiCad's GUI. The first version I sent failed to load at all
(multi-point wire segments aren't valid in this schematic format — each
wire needs exactly 2 points). I found and fixed that, plus several other
format issues, by validating directly against a real KiCad 10 install
(`kicad-cli sch erc`) on this machine and iterating until it loaded
clean: **0 errors, 38 warnings**, all cosmetic (off-grid stub points,
and the fact that the custom "Local" symbol library used for the
connector symbols isn't registered in a library table — both harmless).
`erc_report.json` in this folder is that ERC run's output, for reference.

Still worth doing yourself before trusting it further:

1. Open `bringup_wiring.kicad_pro` in KiCad (free, kicad.org).
2. Open the schematic, confirm it loads and looks right.
3. Run **Inspect > Electrical Rules Checker (ERC)** — should match the
   0-errors result above.
4. If anything looks wrong (a net that shouldn't be joined, a missing
   connection), tell me and I'll fix the file.

Once confirmed, this is a starting point for laying out a PCB
(Tools > Update PCB from Schematic) if you want to move past
breakout-module wiring to a custom board.
