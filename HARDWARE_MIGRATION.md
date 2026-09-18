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

## Open questions — resolved 2026-09-17 (bring-up)

A minimal bring-up firmware (`src/bringup_s3.c`, not the real application,
see its own doc comment) was built and flashed to real hardware to answer
the first two questions empirically instead of guessing from photos:

- **RGB LED confirmed: GPIO48.** A vendor pinout diagram found for this
  exact board labels it `SPICLK_P_RGB_LED`; bring-up firmware drove a
  `led_strip` (WS2812, RMT backend) on GPIO48 cycling red/green/blue/off
  every second, and this was visually confirmed against the real board.
- **Which USB-C port works, and how flashing/monitoring actually behaves
  on it — confirmed, but with real caveats, not the P4 board's
  plug-and-play experience:**
  - The boot log's reset reason reads `USB_UART_CHIP_RESET`, which is the
    chip's own designation for a reset via the **native USB-Serial/JTAG**
    peripheral (GPIO19/20) — i.e. the port labeled "ESP32-S3 Type-C USB &
    OTG" on the vendor's labeled photo, not the CH343P/FTDI
    "USB to Serial" port. The other (bridge-chip) port hasn't been tried.
  - **Auto-reset-into-bootloader does not work on this port** (no working
    auto-program circuit, or a timing mismatch with the tooling used) --
    both `esptool`'s own `--before=default-reset` and plain opening the
    port with `idf.py monitor` each independently kick the chip back to
    running the app (or into an inconsistent state) instead of entering
    download mode. **Working procedure instead**:
    1. Manually enter download mode: hold **BOOT**, tap **RST** while
       still holding BOOT, then release BOOT.
    2. Flash **immediately** with esptool's own reset disabled --
       `--before no-reset` -- e.g. (paths relative to `build/`):
       `python -m esptool --chip esp32s3 -p COMx -b 460800 --before no-reset --after hard-reset write-flash --flash-mode dio --flash-freq 80m --flash-size 16MB 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xf000 ota_data_initial.bin 0x20000 JC-ESP32S3-CAN.bin`
       (`idf.py -p COMx flash` alone reliably fails with "No serial data
       received" on this port, because its own default-reset attempt
       interferes.)
    3. **The COM port number itself changes between download mode and
       normal running mode** (observed COM5 in one state, COM8 in the
       other on the same physical connection) -- rescan
       (`Get-WmiObject Win32_SerialPort` on Windows) right before each
       flash/monitor attempt rather than assuming a fixed port number.
    4. To see the running app's log output afterward: open
       `idf.py -p COMx monitor` (this alone re-triggers download mode
       again, same as step 1's symptom), then tap **RST** once while the
       monitor is already connected and listening -- this boots normally
       and the monitor picks up the app's log output without needing to
       reconnect.
  - Worth trying the other (CH343P/FTDI) port at some point -- it may
    have a real auto-program circuit and make this whole dance
    unnecessary, but the native-USB port above is a known-working
    procedure so there was no need to chase that during bring-up.
  - A separate, real build-environment bug hit during bring-up and fixed
    along the way: the managed `espressif/led_strip` component (v2.5.5)'s
    SPI backend (`led_strip_spi_dev.c`, unused here -- only the RMT
    backend is) fails to compile against ESP-IDF v6.1-beta1 with
    `MALLOC_CAP_DEFAULT`/`heap_caps_calloc` undeclared errors, because it
    's missing an `esp_heap_caps.h` include the header reshuffling in this
    beta IDF no longer provides transitively. Patched directly in
    `managed_components/` (gitignored, so **this patch needs re-applying
    after any fresh checkout / `idf.py reconfigure`** until either the
    upstream component fixes it or this project switches to a hand-rolled
    WS2812/RMT driver instead of the managed component).

## Native Wi-Fi + BLE confirmed working end-to-end (2026-09-17 evening)

Extended the bring-up firmware to start a native SoftAP (no ESP-Hosted, no
C6 co-processor -- this chip's own Wi-Fi radio) and native NimBLE
advertising, **deliberately reusing JC-ESP32P4-M3's exact identity**
(BLE name `JC-P4-C6`, AP SSID `CarTheftGuard-P4`/password `&Car1310`) plus
its exact `POST /api/frequency` ("freq <ms>") contract, purely so the
*already-installed, unmodified* CarTheftGuard phone app could connect to
this new board with zero app changes, as a fast way to prove native
Wi-Fi+BLE actually works before porting anything else. **Confirmed working
end-to-end**: the app auto-detected the board via BLE, auto-joined its
Wi-Fi, and the Control tab's blink-rate control changed the physical LED's
blink rate in real time. This is a deliberate temporary identity, not a
real one -- expect to change it once this stops being a stand-in for the
P4 board.

**Hit and resolved the exact same brownout failure mode as the P4 board**,
but immediately, on the very first Wi-Fi radio use (PHY calibration),
before any CAN work was even involved: `phy_init: falling back to full
calibration` (no saved RF cal data yet) immediately followed by
`E BOD: Brownout detector was triggered` / `rst:0x3 RTC_SW_SYS_RST`,
looping forever since a calibration that never completes never gets saved,
so every reboot repeats the same expensive full recalibration. **Fixed by
powering the board from two USB sources simultaneously**: the existing
data connection to the PC (native USB port) left as-is, plus a second,
separate cable from a wall charger into the board's *other* USB-C port
(the CH343P/UART one) purely for extra current -- both ports share the
same on-board 5V/GND net, so this is just two power sources in parallel,
not a data conflict. One successful calibration was all it took; this
should now boot fine on a single supply too, since RF cal data is
persisted -- not yet re-tested on single-supply power to confirm.

**Why this is worth taking seriously, not just brushing off as "bad
cable"**: the user asked directly why this wouldn't have hit the P4 board
under the same cable/PC too, which is a fair challenge to "just use a
different cable." The real structural difference: the P4 board's Wi-Fi
runs on a *separate* ESP32-C6 co-processor chip with its own on-board
power circuit, splitting the P4 board's own core-CPU current draw and the
C6's Wi-Fi-radio current draw across two independent regulators/decoupling
networks. This S3 board has everything -- CPU and Wi-Fi radio both -- on
one chip, fed by a single shared onboard regulator (a basic AMS1117
linear regulator, visible on the board photos, not known for especially
fast transient response). So the likely real bottleneck is this board's
*own* onboard power regulation/decoupling being marginal for a Wi-Fi
radio current spike, not the USB cable or PC port per se -- the wall-
charger fix works by adding parallel current capacity upstream of that
marginal regulator, not by fixing it. If single-supply operation turns
out not to be reliable long-term, the real fix would be a bulk/bypass
capacitor added physically near the module, not a cable swap.

## Native CAN (TWAI) confirmed working end-to-end (2026-09-17 night)

Wired the SN65HVD230 transceiver to GPIO4 (TX) / GPIO5 (RX) and connected
CANH/CANL to the ArdunioUsbBridgeToCan simulator (common GND included) --
**no termination resistor on this board's end yet**. Added a
`can_receive_task` (legacy `driver/twai.h` API -- see below for why not
the new one) that just logs every received frame.

**Confirmed on real hardware, first try**: receiving clean, steady frames
on `0x120`/`0x180`/`0x220` -- exactly the simulator's known engine/
vehicle/body broadcast IDs (see project memory). This is the core promise
of the whole hardware migration validated end-to-end: CAN frames flowing
through this chip's native TWAI peripheral and a plain transceiver, zero
SPI, zero external CAN controller chip. Working even without the far-end
terminator, though that's still worth closing before relying on this
under heavier bus load or longer wiring -- an untenmpted bus can look
fine at low traffic and degrade under load from reflections.

**Real ESP-IDF v6.1-beta1 API-split hit while building this**: this beta
has split the TWAI driver into two separate, mutually-exclusive
components -- the new `esp_driver_twai` (handle-based `esp_twai.h`/
`esp_twai_onchip.h` API) and a legacy-compatibility one (component name
just `driver`, header still at the old `driver/twai.h` path with the
familiar `twai_driver_install`/`twai_start`/`twai_receive` calls this
bring-up code uses). The legacy header additionally emits a `#warning`
nudging toward the new API, which this project's default `-Werror`
promotes to a hard build failure -- worked around with a targeted
`target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)` in
`src/CMakeLists.txt` rather than switching APIs mid-bring-up. Worth
reconsidering migrating to the new `esp_twai.h` API for the real
application later, now that legacy is confirmed merely deprecated-with-
warning, not actually removed, in this IDF version.

## GPS UART confirmed working end-to-end (2026-09-17 night) -- bring-up complete

Wired ArdunioUsbBridgeToCan's simulated GPS output (Arduino pin 3,
SoftwareSerial, 9600 baud, 5V logic) through an existing voltage divider
down to GPIO7 (RX); GPIO6 (TX) wired but unused. A `gps_receive_task`
(plain `uart_read_bytes`, no NMEA parsing yet) confirmed clean, valid
`$GPRMC` sentences with correct checksums arriving intact.

**This closes out first-pass bring-up on all four fronts**: LED (GPIO48),
native Wi-Fi+BLE (working with the real, unmodified phone app), native CAN
(TWAI, receiving real simulator frames), and now GPS UART -- all four
confirmed on real hardware in one evening. Nothing about the *real*
application (HTTP API port, OBD/UDS logic, etc.) has been ported yet --
see the migration plan above -- but every piece of hardware this new
board needs to replace the P4+C6+MCP2515 design has now been individually
proven to work.

## Known issues found after first-pass bring-up (2026-09-17 night, not yet fixed)

**CAN went from working to silent, cause not yet found.** The first CAN
test (see above) cleanly received real simulator frames. Later the same
night, after a lot of physical handling (BOOT+RST button presses, full
USB unplug/replugs, swapping between the two USB-C ports, and manually
adding/moving termination resistors), CAN went completely silent --
`twai_get_status_info()` shows `state=RUNNING` with **all error counters
at zero** (not just zero frames -- zero `bus_err_count`, zero `rx_err`,
etc.), which is the signature of no signal reaching the RX pin at all,
not a signal-quality/termination problem. Briefly, right after adding a
120Ω terminator to the S3 side, `rx_err`/`bus_err_count` became noisy and
nonzero (real electrical activity, still zero successful frames) but then
went back to silent and hasn't moved since. Code was checked and found
unchanged/correct (same GPIO4/5 config, same `twai_driver_install`/
`twai_start` calls, same receive loop, driver consistently reports
healthy init) -- this points at a physical connection issue (likely a
breadboard jumper wire not fully seated) rather than a software
regression, but it hasn't been physically re-verified carefully (needs
daylight/fresh eyes, not more guessing late at night). **Next step**:
continuity-check every CAN wire individually with a multimeter (TXD, RXD,
CANH, CANL, GND, 3.3V to the SN65HVD230) rather than just looking at it.

**Unifying theory (raised by the user, worth taking seriously): this and
the DHCP failure below may be the *same* root cause, not two unrelated
bugs.** Both a clean CAN differential signal and a successful DHCP
exchange (the ESP32 *transmitting* DHCP offers back to the phone, not
just receiving -- more TX-load-bearing than the BLE advertising/802.11
association steps that still work fine) depend on stable power the same
way the earlier Wi-Fi-calibration brownout did. Given how much physical
handling this breadboard setup took tonight (dozens of BOOT+RST presses,
full USB unplug/replugs, port-swapping, resistor changes), a connector or
breadboard rail could plausibly have degraded generally, not just on the
CAN wires specifically. **Next time, check this before treating CAN and
DHCP as separate mysteries**: measure the board's actual 3.3V rail with a
multimeter under load, and try swapping to fresh, known-good USB cables
entirely rather than continuing to reuse tonight's (possibly worn)
connections.

**Native USB-Serial/JTAG port (the one used all night) needs manual
BOOT+RST before every flash/monitor, confirmed structural, not
timing** -- `esptool`'s own `--before=default-reset` and `idf.py
monitor`'s port-open both independently kick the chip out of whatever
state it's in instead of into bootloader mode reliably on this board.
Tried the *other* USB-C port (CH343 USB-UART bridge chip, enumerates as
"USB-Enhanced-SERIAL CH343" rather than the generic "USB Serial Device")
as an alternative: **flashing works flawlessly with zero button presses**
(true auto-reset, `idf.py flash` alone), but **monitoring over that same
port reliably hangs** right after the ROM's first boot print
(`call_start_cpu0`), even across a full board reset with clean `POWERON`
(not brownout) -- looks like `idf_monitor.py`'s own DTR/RTS handling on
port-open doesn't play well with however this board's CH343 auto-reset
circuit is wired, though this wasn't root-caused. **Practical result**:
flash via the CH343 port (COM9 in tonight's session, painless), monitor
via the native port (COM8, needs the manual BOOT+RST dance) -- or just
stay on the native port for both if not actively fighting the flash
timeout. `tools/s3-serial-monitor.ps1`/`.bat` (this session) auto-detects
whichever port is present and prints the relevant reminders either way.

**Real bug found and fixed in `tools/s3-serial-monitor.ps1`**: single-
match COM port detection could silently report "not found" even when
exactly one real match existed, because a lone WMI (`Win32_SerialPort`)
object doesn't reliably get PowerShell's synthetic `.Count` property the
way plain objects/arrays do. Fixed by wrapping the pipeline in `@(...)`
to force array semantics regardless of match count -- worth remembering
as a general PowerShell gotcha, not just for this script.

**DHCP fails on repeated Wi-Fi joins from the phone -- reproducible,
not a one-off.** The very first Wi-Fi+BLE bring-up test succeeded fully
end-to-end (app auto-connected, LED blink-rate control worked over HTTP,
which requires a real IP). Later the same night, after many board
resets/reflashes, the phone's BLE discovery of `JC-P4-C6` and the
`WifiNetworkSpecifier` request both still fire correctly, and the ESP32
AP's own Wi-Fi log confirms **the phone's station genuinely associates**
(`wifi:station: <phone MAC> join, AID=1`) -- but no DHCP lease ever
follows, and ~20 seconds later the phone disconnects on its own
(`leave, ... reason = 3`, a client-initiated deauth) followed by the
app's own 20s manual timeout (`Could not join board Wi-Fi` --
`BoardLink.connectTimeoutRunnable`, since the 2-arg `requestNetwork()`
overload has no built-in `onUnavailable()`). Reproduced 3+ times in a
row, unaffected by toggling the phone's own Wi-Fi off/on. Not yet
root-caused -- worth checking next time: whether lwIP's default AP DHCP
server (`esp_netif_lwip: DHCP server started...`, seen once at boot but
not re-logged per lease attempt) has a known issue with rapid repeated
client join/leave cycles from the same MAC, and whether explicitly
logging `IP_EVENT_AP_STAIPASSIGNED` (not currently done in
`bringup_s3.c`) would help pin down whether the ESP32 side ever even
offers a lease vs. the phone-side DHCP client itself stalling.

## CAN and DHCP resolved (2026-09-18 morning/day) -- the "unifying power theory" above was wrong

**CAN was never actually broken -- a logging blind spot, not a hardware bug.**
The user's own power/voltage/connection checks that morning (3.3V rail
confirmed good, ~60Ω across CAN_H/CAN_L with no extra resistor -- the
SN65HVD230 module has a **fixed onboard 120Ω terminator, not a
jumper-controlled one**, so 60Ω total from two in parallel is the
*correct* healthy reading, no extra resistor needed) turned out to be
enough on their own. The real issue: `can_receive_task`'s success branch
called `capture_can_frame()` completely silently on `err == ESP_OK`
(matching the original P4 design) -- so a CAN bus receiving continuously
with zero timeouts produces **zero log output**, indistinguishable from a
hung task by log inspection alone. A temporary success-branch
`ESP_LOGI` (added, confirmed the flood of real `0x120`/`0x180`/`0x220`
frames, then removed again) proved CAN was working the whole time that
morning; last night's genuine `rx_err`/timeout evidence was real at the
time, just not still true by morning.

**DHCP was a real, deep bug -- fully root-caused, not just worked around.**
The investigation (many hours, many dead ends, documented here in full
since the process itself is worth remembering):
1. Disconnecting the CAN RX wire entirely ruled out CAN-flood/CPU-
   contention as a cause -- DHCP still failed identically.
2. Adding `WIFI_EVENT_AP_STACONNECTED`/`STADISCONNECTED`/
   `IP_EVENT_AP_STAIPASSIGNED` logging to `bringup_s3.c` proved the
   ESP32 side's DHCP server **never** completed a lease -- the phone
   (and, tested separately, a plain PC Wi-Fi client with no app
   involved) associates at the 802.11 level but the DHCP handshake
   itself always failed.
3. Enabling lwIP's `DHCPS_DEBUG` (a build-time flag in
   `components/lwip/apps/dhcpserver/dhcpserver.c`, off by default, patched
   directly into the ESP-IDF install like the `led_strip` fix -- also hit
   and fixed a real pre-existing format-string bug in that debug code,
   `%x` against a `u32_t` arg, needs an `(unsigned int)` cast to build
   under `-Werror=format`) showed the **exact** failure point: the DHCP
   server correctly receives and parses the client's DISCOVER, correctly
   builds a 548-byte OFFER packet, but `udp_sendto()` fails with
   `ffffffff` (-1) on *every single attempt*, and the client just
   retransmits DISCOVER into the same wall repeatedly until it gives up.
   That -1 is lwIP's `ERR_MEM`, but it's not really a memory error --
   `wifi_transmit()` in `components/esp_wifi/src/wifi_netif.c` passes
   `esp_wifi_internal_tx()`'s raw `esp_err_t` straight through as the
   netif's `err_t` return, and `ESP_FAIL` (-1) happens to collide
   numerically with `ERR_MEM` (also -1) -- so the log is technically
   misleading. The real failure is a generic low-level WiFi TX rejection.
4. **Confirmed NOT an ESP-IDF v6.1-beta1 regression**: built and ran the
   identical code against stable ESP-IDF **v5.5.0** (already installed at
   `C:\Users\yossi\.espressif\frameworks\esp-idf-v5.5`, just needed its
   toolchain installed via `install.ps1 esp32s3`) and hit the exact same
   intermittent failure there too -- ruling out "it's just a beta bug."
   (v5.5 did initially surface two *separate*, now-fixed bugs of its own
   on the very first successful DHCP lease: a `netconn_alloc: undefined
   netconn_type` assert from a too-small `CONFIG_LWIP_TCPIP_TASK_STACK_SIZE`
   -- stock default 3072 is marginal for lwIP's TCP/IP task in an `-Og`
   build with a deep accept→tcp_process→tcp_input→ip4_input call chain,
   bumped to 8192 -- and after that, a corrupted-backtrace FreeRTOS
   scheduler crash from the httpd task's own default 4096-byte stack,
   fixed by setting `config.stack_size = 8192` on `HTTPD_DEFAULT_CONFIG()`.
   `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK` was also turned on
   permanently as a low-cost precise-overflow diagnostic for next time.)
5. **Root cause: BT/WiFi radio coexistence**, confirmed by isolation
   testing. This chip has one shared 2.4GHz radio between BLE and WiFi.
   With BLE advertising fully disabled (`start_ble()` never called),
   manual Wi-Fi join/DHCP succeeded **4/4** and later **3/3** across two
   separate clean-boot test sessions -- fully reliable. With BLE running
   at all, DHCP failed intermittently to consistently. Multiple standard
   mitigations were tried and **all failed** to fix it while BLE stayed
   active: `esp_coex_preference_set(ESP_COEX_PREFER_WIFI)` (biasing
   arbitration toward WiFi), slowing NimBLE's advertising interval from
   the ~30-60ms "fast" default to 500-1000ms (`adv.itvl_min/max`), pausing
   just the GAP advertisement (`ble_gap_adv_stop()`) while a WiFi station
   is associated, and even a full `esp_bt_controller_disable()` (not just
   pausing advertising -- confirmed via log the disable call itself
   succeeded, `ESP_OK`, and DHCP still failed identically on every retry).
   Only BLE never being initialized at all removes the contention -- a
   controller that's disabled-but-still-initialized apparently leaves
   some coexistence-arbitration state registered with the WiFi driver
   that a plain `disable()` doesn't clear. The next escalation (a full
   `esp_bt_controller_deinit()`/re-`init()` cycle around every WiFi
   connect/disconnect) was deliberately **not attempted** -- real risk of
   crashing NimBLE's host stack, which isn't designed for its controller
   to disappear and reappear underneath it, for an uncertain payoff.
   **Decision (discussed with the user): stop chasing BLE+WiFi
   concurrency on this chip.** `start_ble()` is commented out in
   `app_main()` with a comment explaining why. Real application porting
   should treat BLE and this board's WiFi AP as mutually exclusive by
   design (BLE for discovery only, fully off during WiFi use), not
   something to run concurrently.

## DHCP resolution corrected (2026-09-18 afternoon): it's CAN+GPS together, not BLE/WiFi coexistence

**The user's sharp pushback was right, and the "BT/WiFi coexistence" diagnosis above was wrong.** After the coexistence writeup, the user asked directly: yesterday's exact commit (`e9ad195`, no CAN/GPS code at all) connected instantly and reliably -- so what does CAN+GPS actually have to do with the Wi-Fi/BLE join? That question led to a proper isolation test, checked out in a separate worktree (`git worktree add ../JC-ESP32S3-CAN-yesterday e9ad195`) so today's work stayed untouched:

1. **Built and flashed `e9ad195` exactly as-is** on today's hardware/environment: connected on the very first attempt, no failures. Confirms the chip and environment are fine -- nothing rules out BLE+WiFi concurrency in general.
2. Went back to today's full code and reverted two leftover, never-proven-to-help deviations from the coex investigation (BLE advertising interval slowed to 800/1600, WiFi TX power capped to 10dBm) back to the exact `e9ad195` values -- **still failed repeatedly**. Not a config-drift issue either.
3. Disconnecting the CAN/GPS *wires* (tasks still created, just blocked on empty queues) -- **still failed**, which is what originally pointed (wrongly) at BLE/WiFi coexistence as the root cause, since blocked tasks consume ~0 CPU.
4. The real test, prompted by the user asking specifically how CAN+GPS could affect Wi-Fi: **skip calling `start_can()`/`start_gps()` entirely** (not wires -- the peripheral driver installs themselves: `twai_driver_install`, `uart_driver_install`), keeping BLE+WiFi+all three HTTP handlers otherwise identical to today's code. Result: **3/3 reliable.**
5. Isolated further: **CAN alone (GPS skipped): 3/3 reliable. GPS alone (CAN skipped): 3/3 reliable.** Only **both TWAI and UART drivers installed together** breaks Wi-Fi/BLE join reliability -- neither one alone is the problem.

**Conclusion**: this is a specific resource contention between the TWAI and UART peripheral drivers when both are active simultaneously alongside WiFi+BLE -- almost certainly a shared interrupt allocation, GPTimer, or DMA-adjacent conflict that leaves too little real-time headroom for the WiFi/BLE stack once both peripherals are running, not a fundamental BT/WiFi radio-arbitration limit. All the BLE-off/coex-preference/BT-controller-disable/advertising-interval mitigations documented in the section above this one were real work but were chasing the wrong root cause -- keep them as a record of what was ruled out, not as the explanation.

6. **CPU core affinity tested and ruled out.** Both WiFi and NimBLE are explicitly pinned to CPU0 (`CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0`, `CONFIG_BT_NIMBLE_PINNED_TO_CORE_0`), and `app_main()` itself also runs pinned to CPU0 (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0`) -- so calling `start_can()`/`start_gps()` directly from `app_main()` puts the TWAI/UART drivers' `esp_intr_alloc()`'d ISRs on the *same* core as WiFi/BLE's own interrupts. Moved both drivers' init (and their receive tasks) onto a helper task pinned to CPU1 (`sensors_init_on_core1()`/`sensors_init_task()` in `bringup_s3.c`) to test whether freeing CPU0 entirely fixes it -- **still failed identically**. CPU core placement is not the mechanism either.

**Current state, not yet root-caused further**: GPS is temporarily disabled (`start_gps()` commented out inside `sensors_init_task`) so the known-good CAN-only config stays usable while this is investigated more -- verified end-to-end: real CAN frames confirmed via a direct `GET /api/can` query (thousands of valid frames, IDs matching the simulator), and the app's **Record** tab (which calls `fetchCanRaw()` -> `/api/can`) displays them live.

The CPU-affinity code (pinning to core 1) stays in place even though it didn't fix the conflict -- freeing CPU0 for WiFi/BLE is sound practice regardless, and it costs nothing to keep.

## `/api/obd` added -- Monitor tab now works, without porting the full active query engine

The app's **Monitor** tab calls `fetchObdData()` -> `GET /api/obd`, which didn't exist in `bringup_s3.c` at all (unrelated to the CAN+GPS conflict above -- just never ported, 404d every poll). The P4's real `obd_http_handler` is fed by `obd_query_task`, an active engine that transmits Mode 01 PID requests over CAN and waits for responses (several hundred lines: request/response queueing, multi-ECU disambiguation, DTC/VIN/UDS-scan triggers) -- real feature-porting work, and out of scope for bring-up code.

**Shortcut that works for the simulator specifically**: `ArdunioUsbBridgeToCan` already broadcasts engine/vehicle state *unsolicited* on `0x120` (every 20ms) and `0x180` (every 50ms) regardless of any active OBD-II request -- this is exactly what `JC-ESP32P4-M3`'s `decode_engine_broadcast`/`decode_vehicle_broadcast` already decode into the same `obd_state_t` fields Mode 01 PID responses would. Ported those two functions verbatim into `bringup_s3.c`, hooked into `capture_can_frame()` (checks `message->identifier` against `CAN_ID_ENGINE_STATE`/`CAN_ID_VEHICLE_STATE`, decodes if it matches), and added `obd_http_handler`/`GET /api/obd` with the same JSON contract as the P4. No CAN TX, no request/response queue, no active polling needed. `supported_pids`/`supported_pids_2` stay `0` (those only come from an active PID 0x00/0x20 request) -- everything else (RPM, speed, coolant, throttle) updates live. Verified end-to-end: app connects, Monitor tab shows live data.

This only works because the simulator broadcasts unsolicited -- a real car won't, so this is explicitly a simulator-only shortcut. The full active `obd_query_task` port (needed for any real-car testing on this board) is still open, tracked below.

## Open questions still remaining

- Final confirmation that GPIO4/5 (CAN TX/RX) and GPIO6/7 (GPS UART) are
  actually free of conflicts once wired up for real -- chosen as safe,
  non-strapping, non-USB, non-PSRAM general-purpose pins per the vendor
  pinout diagram, but not yet physically tested with the SN65HVD230 or a
  GPS module.
- **The CAN+GPS/WiFi-BLE resource conflict root cause** (see the corrected
  DHCP resolution above) -- CPU-core affinity was ruled out; still not
  known what's actually contended (interrupt allocation, GPTimer, DMA).
  GPS stays disabled until this is found and fixed.
- **Port the full active `obd_query_task`** (PID request/response over
  CAN, multi-ECU disambiguation, DTC/VIN/UDS-scan triggers) -- needed for
  any real-car testing on this board; the current `/api/obd` only works
  against the simulator's unsolicited broadcasts (see above).
- Power/brownout behavior on this new board under the same CAN-TX-heavy
  scans that caused problems on the P4 board — worth deliberately
  re-running the same stress scenarios (full address sweep, VWTP sweep)
  once this board is CAN-capable, to see whether the simpler hardware
  path actually helps.
