# JC-ESP32S3-CAN — migration plan (ESP32-S3 + native CAN)

Branch `esp32s3-can`, checked out as its own working tree at
`C:\projects\JC-ESP32S3-CAN` (a `git worktree` off the `JC-ESP32P4-M3`
repo, not a separate repository — same history, same remote, this is just
a second checkout so both hardware generations can be worked on side by
side without stashing). Started 2026-09-17, continuing the CarTheftGuard
project on new hardware.

## Why change hardware

The current board (`JC-ESP32P4-M3`) is an ESP32-P4 (application MCU, no
native Wi-Fi/BT of its own) paired with an ESP32-C6 co-processor over
SDIO via ESP-Hosted for wireless, plus an external MCP2515 CAN controller
driven over **bit-banged SPI** for the CAN bus. Two real, documented pain
points this generation's design created:

- A real cross-task SPI race in `mcp2515_send()` (two-step buffer-write +
  RTS not atomic) that intermittently stuck `Partner:` at `UNKNOWN` and
  RPM/Speed at `0` under concurrent load on the real Fiat (fixed in commit
  `9bfc87c`, but the underlying fragility of bit-banged SPI to a discrete
  CAN controller chip remains).
- Repeated real-car brownout resets (`E BOD: Brownout detector was
  triggered`) during sustained CAN-TX-heavy operations (UDS address scans,
  VWTP probes) — see `captures/fabia_2026/UDS_BODY_MODULE_RESEARCH.md`'s
  2026-09-15 and 2026-09-17 updates and project memory. Root cause was
  never fully pinned to one component, but a simpler, single-chip design
  with a proper CAN transceiver (rather than a full external controller
  chip driven over a bit-banged bus) removes a plausible contributor and
  is worth trying regardless.

The new board eliminates both the SDIO-hosted co-processor split *and*
the external SPI CAN controller in one step: the ESP32-S3 has native
Wi-Fi + Bluetooth **and** a native TWAI (CAN 2.0B) peripheral built into
the SoC itself. Paired with a plain CAN transceiver chip (no controller
logic on the transceiver side, no SPI at all), CAN frames go straight
through two GPIOs and the chip's own hardware CAN peripheral instead of
software-bit-banging an external chip's SPI protocol.

## New hardware

**MCU board**: ESP32-S3-WROOM-1 **N16R8** module (16MB flash, 8MB PSRAM,
Wi-Fi 802.11b/g/n + Bluetooth 5 LE) on a generic 44-pin dev board ("NEW
ESP32 S3 Development Board ... IPEX 2.4G Wifi BT Module ... 44Pin Type-C
8M PSRAM", AliExpress). Notable features visible on the board:

- **Dual USB-C**: one wired to the S3's native USB (OTG/JTAG/serial), the
  other through an onboard **FTDI FT232RQ** USB-UART bridge — i.e. two
  independent ways to talk to the board, worth figuring out which is
  which before relying on one for flashing/monitoring.
- IPEX/u.FL connector for an external 2.4GHz antenna (not a chip antenna
  on the module itself, or at least an external option exists).
- Onboard WS2812-style RGB LED, plus RST and BOOT buttons.
- Pin labels silkscreened both sides: left side `GND GND 13 12 11 10 9 46
  8 18 17 16 15 3V3 3V3 RST 4 5 6 7`; right side `RX TX 1 2 42 41 40 39
  38 37 36 35 0 45 48 47 21 20 19 GND`. Not yet cross-referenced against
  which of these are ESP32-S3 strapping pins (0, 3, 45, 46 are
  strapping-sensitive on the S3 and need care if used for anything
  connected at boot).

**CAN transceiver**: SN65HVD230 CAN Bus Transceiver module (Texas
Instruments SN65HVD230 chip, pin-compatible with the more common
PCA82C250). 3.3V native (matches the S3's I/O voltage directly, no level
shifting needed — unlike 5V transceivers), ESD protected. 4-pin header
(3.3V, GND, RX, TX) for the microcontroller side, screw terminal (CANH,
CANL) for the bus side, plus an onboard jumper for the 120Ω termination
resistor (leave it **open/removed** when tapping into a car's OBD-II bus
mid-span — see [[project-real-car-realism]] and
[[project-uds-vwtp-body-module-research]] on why adding termination at an
OBD-II tap is wrong; only needed if this board is ever the true end of an
isolated bench bus).

## Migration plan (draft — not yet started)

1. **New ESP-IDF project** in this folder (`idf.py create-project` or
   copy `JC-ESP32P4-M3`'s project skeleton and strip P4/hosted-specific
   pieces) targeting `esp32s3`, not `esp32p4`.
2. **CAN layer**: replace `mcp2515.c`'s bit-banged-SPI driver entirely
   with ESP-IDF's native `driver/twai.h` (TWAI = Espressif's renamed
   SocketCAN-compatible CAN 2.0B controller driver). No SPI bus, no
   external controller chip — just `twai_driver_install` +
   `twai_transmit`/`twai_receive` against two plain GPIOs wired to the
   SN65HVD230's TX/RX pins. This is the core of the whole migration; the
   higher-level CAN logic (frame capture ring buffer, OBD/UDS request
   builders, the address/VWTP/deep-scan sweeps) all operate on abstract
   CAN frames and should port with minimal change once this layer exists.
3. **Wi-Fi/BLE**: native `esp_wifi`/NimBLE directly on the S3 — no
   ESP-Hosted, no SDIO, no co-processor link setup/teardown code at all.
   Should substantially simplify `main.c`'s boot sequence.
4. **Port the HTTP API surface** (`/api/can`, `/api/obd`, `/api/uds/*`,
   `/api/vwtp/scan`, `/api/deepscan`, `/api/health`, `/api/ota`, etc.)
   from `JC-ESP32P4-M3/src/main.c` — this is almost entirely
   hardware-independent application logic and should be a near-direct
   port once the CAN and Wi-Fi layers underneath it exist.
5. **GPS UART bridge**: port as-is, just needs new GPIO pin numbers for
   this board.
6. **Decide GPIO assignments** for: CAN TX/RX (to SN65HVD230), GPS UART
   RX/TX, anything else the current board wires up — deliberately not
   decided yet in this doc; needs a look at the S3's strapping-pin
   constraints and this specific board's silkscreen before picking.
7. **Phone app side** (`CarTheftGuard`): should need minimal-to-no
   changes, since it talks to the board purely over the same HTTP API —
   the whole point of porting the API surface unchanged in step 4.

## Open questions

- Which of the two USB-C ports is native-USB vs. FTDI-UART, and does
  flashing/monitoring work the same way as the P4 board's tooling
  (`idf.py -p COMx flash monitor`) or need adjusting?
- Final GPIO assignment for CAN TX/RX and GPS UART, avoiding strapping
  pins (0, 3, 45, 46) and whatever pin drives the onboard RGB LED.
- Whether to keep BLE-based board discovery + AP-only Wi-Fi (see
  [[project-ap-only-connectivity]]) unchanged, or reconsider now that
  there's no hosted co-processor link involved.
- Power/brownout behavior on this new board under the same CAN-TX-heavy
  scans that caused problems on the P4 board — worth deliberately
  re-running the same stress scenarios (full address sweep, VWTP sweep)
  once this board is CAN-capable, to see whether the simpler hardware
  path actually helps.
