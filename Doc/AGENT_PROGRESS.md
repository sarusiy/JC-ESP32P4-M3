# Agent Progress Log

## How to use this file
- I append updates here while working autonomously.
- Each entry has a timestamp, current step, and next step.
- This is the single place to check live progress without waiting for chat replies.

## Entries

- 2026-08-26 00:00 (local): Initialized progress log.
  - Current step: Prepare dual-target P4+C6 workflow and verify ESP-IDF task flow.
  - Next step: Implement P4<->C6 transport skeleton and validate build/flash/monitor for both targets.

- 2026-08-26 00:01 (local): Added external code reference project.
  - Reference repository: git@github.com:giltal/ESP32P4_Guition4_7_Project.git
  - Suggested usage for this project:
    - Use: study board bring-up flow, ESP-IDF project structure, and task layout for ESP32-P4 family.
    - Copy: reusable utility modules and proven initialization sequences where they match our hardware path.
    - Adopt: BLE and transport patterns that fit P4+C6 split architecture after interface validation.
  - Adoption note: keep original attribution and verify repository license compatibility before direct code copy.
  - Next step: review that repo components and map candidate files into our P4 app, C6 app, and Android integration plan.

- 2026-08-26 00:02 (local): Confirmed physical board name from user.
  - Hardware name in use: ESP32P4_Guition4_7.
  - Current repository/project naming remains JC-ESP32P4-M3 until P4+C6 integration is fully validated.
  - Next step: keep implementation aligned to ESP32P4_Guition4_7 hardware while preserving current folder and environment names for continuity.

- 2026-08-26 00:03 (local): Architecture correction after reference repo review.
  - Decision: C6 is treated as radio co-processor (ESP-Hosted path), not as a separately flashed day-to-day app target.
  - What this changes:
    - Default path becomes P4 host firmware build/flash.
    - BLE/Wi-Fi run from P4 APIs over hosted transport to C6.
    - C6 firmware update is host-managed only when version mismatch requires it.
  - Status of previous C6 standalone track:
    - The old standalone `c6_firmware/` scaffolding was removed during the ESP-IDF hosted cleanup.
    - It is no longer part of the development strategy.
  - Next step: implement hosted radio path on P4 (BLE first, Wi-Fi second) and remove dependence on manual C6 flashing.

- 2026-08-26 00:04 (local): Fast-track adoption from ESP32P4_Guition4_7 reference started.
  - Completed:
    - Added hosted dependency (`espressif/esp_hosted`) in `src/idf_component.yml`.
    - Updated component requirements in `src/CMakeLists.txt` for hosted Wi-Fi/BLE path.
    - Replaced old standalone BLE stub in `src/main.c` with:
      - hosted Wi-Fi link bring-up on P4 (`esp_wifi_init/start` path),
      - NimBLE host startup on P4 with writable freq characteristic,
      - existing USB control channel kept as fallback.
  - Validation status:
    - Project rebuild is in progress after these changes.
  - Next step:
    - finalize build verification output,
    - flash P4,
    - verify phone BLE discovery and `freq <ms>` command flow.

- 2026-08-27 00:05 (local): Progress checkpoint (hosted migration).
  - Completed since previous checkpoint:
    - Hosted migration files remain in place and tracked for P4-first flow.
    - Progress and architecture docs updated for ESP32P4_Guition4_7 and C6 co-processor model.
  - Current state:
    - Local repository has pending hosted-path changes (no commit yet).
    - Active workflow has moved to native ESP-IDF build, flash, and monitor tasks.
  - Next step:
    - run flash/upload on connected hardware,
    - verify BLE advertising on phone,
    - validate `freq <ms>` control path end-to-end.

- 2026-08-27 20:30 (local): Native ESP-IDF hosted BLE/Wi-Fi validation checkpoint.
  - Completed:
    - Removed PlatformIO and pioarduino project markers from the active workflow.
    - Installed and validated native ESP-IDF 5.5 tooling for `esp32p4`.
    - Fixed IDF component registration so `src/main.c` provides `app_main`.
    - Added `esp_wifi_remote` and configured ESP-Hosted SDIO for P4 host + onboard C6.
    - Fixed duplicate Wi-Fi netif creation during hosted Wi-Fi startup.
    - Switched BLE from generic UART HCI to hosted NimBLE VHCI over SDIO.
    - Native build and flash on COM3 succeeded.
    - Runtime logs confirmed WLAN, HCI over SDIO, hosted BT support enabled, and BLE advertising as `JC-P4-C6`.
    - Android phone connected to `JC-P4-C6` and discovered service `0xFFF0`.
  - Current state:
    - Firmware is ready for repeated BLE writes to characteristic `0xFFF1` using ASCII commands such as `freq 250`.
    - Wi-Fi transport is up, but AP credential provisioning is not implemented yet.
  - Next step:
    - Validate multiple valid/invalid BLE frequency writes.
    - Commit and push the ESP-IDF hosted baseline.
    - Build the dedicated Android app UI for scan/connect/frequency control in `C:\projects\CarTheftGuard` from `git@github.com:sarusiy/CarTheftGuard.git`.

- 2026-08-29 (local): Android BLE control milestone.
  - Completed:
    - CarTheftGuard scans and finds the advertising board from an Android phone.
    - Android connects to `JC-P4-C6`, discovers service `0xFFF0`, and writes `freq <ms>` to `0xFFF1`.
    - Valid Android frequency commands update both firmware state and the GPIO 28 LED blink timing.
    - CarTheftGuard v0.1.2 lists nearby BLE scan results and includes the `0xFFF2` response-notification client.
    - Firmware `0xFFF2` response read/notify support was built and flashed to COM3.
  - Current state:
    - End-to-end Android BLE LED control is validated.
    - `0xFFF2` notification display still needs one phone-side valid/invalid command confirmation.
  - Next step:
    - Begin BLE-based Wi-Fi credential provisioning and connection status reporting.

- 2026-08-29 (local): Wi-Fi provisioning crash-loop and no-IP bugs fixed; full flow validated end-to-end.
  - Symptom: board crash-looped every ~3-4s with `assert failed: tlsf_free tlsf.c:630 (!block_is_free(block) && "block already marked as free")`, a double-free in the vendor ESP-Hosted SDIO driver (`sdio_process_rx_task`), during the earliest WiFi-transport bring-up. Reproduced with BLE on/off, SDIO clock 20/40MHz, RX streaming/none, full power cycle, and full flash erase — none of those changed anything.
  - Root cause found by comparing against a public reference project for the exact same board module (Guition JC4880P443C / JC-ESP32P4-M3): `https://github.com/giltal/ESP32P4_Guition4_7_Project`. Their host pins `espressif/esp_hosted` to `^2.12.8`; ours was pinned to `<2.6.0` (resolved 2.0.17), an old host library version with this bug.
  - Fix 1 (`src/idf_component.yml`): bumped `espressif/esp_hosted` to `^2.12.8` (resolves 2.12.12). No `main.c` changes needed for this part; confirmed stable for 100+ boot cycles afterward. Commit `06550b6`.
  - Fix 2 (`src/main.c`): after the crash was fixed, Wi-Fi still joined but never got an IP (stuck at "OK WiFi connecting" forever, no error). Two bugs:
    - Missing `esp_netif_create_default_wifi_sta()` — no netif meant no DHCP client at all.
    - ESP-Hosted's netif "started" latch can get stuck after the STA has been up once, so a *second* `esp_wifi_connect()` associates but DHCP silently never runs again. Fixed with a full `esp_wifi_stop()` -> `set_mode` -> `set_config` -> `esp_wifi_start()` -> 300ms delay -> `esp_wifi_connect()` cycle on every connect attempt. This exact bug/fix is also documented in the reference project's `bsp_wifi.c`. Commit `6c3f670`.
  - Fix 3 (`src/main.c`): added debug `printf` logging for command processing on all three control paths (USB, BLE, HTTP) and for the actual frequency value change, to make future diagnosis faster. Commit `8a45dbb`.
  - Confirmed on hardware: full flow BLE scan -> connect -> Wi-Fi provision -> DHCP IP -> `POST /api/frequency` -> LED blink rate changes, all working end-to-end.
  - Also confirmed our board's actual C6 co-processor firmware version is 2.3.0 (visible via the newer host library's `Version mismatch: Host [x] > Co-proc [2.3.0]` log line, which doesn't exist in host 2.0.17).
  - Notes for future debugging kept in `/memories/repo/esp-hosted-crash-fix.md`: the board exposes direct UART header pins for the C6 co-processor (`connector pinout list.jpeg`: C6_U0RXD, C6_U0TXD, C6_IO9 boot-strap, C6_CHIP_PU reset) for directly reflashing the C6 slave firmware if ever needed, bypassing SDIO/WiFi entirely; and `esp_hosted_get_coprocessor_fwversion()` is a cheap non-destructive way to query the C6's actual firmware version over the existing transport, no WiFi needed.
  - Next step: none pending on firmware; feature is complete and validated. Future: consider HTTPS/auth on the frequency API before untrusted-network use.

- 2026-09-01 (local): CAN bus bridge implemented and validated end-to-end.
  - Goal: add CAN bus support via an MCP2515/TJA1050 module on header JP1,
    with a second MCP2515 module on a companion Arduino Uno
    (`git@github.com:sarusiy/ArdunioUsbBridgeToCan.git`) acting as a
    USB-CAN bridge/simulator for testing, per `Doc/electrical drawing.vsdx`.
  - Phase 1 scope: HW connectivity test only. P4 echoes back any CAN frame
    it receives; Arduino sends an incrementing ASCII digit every second and
    checks the echo matches.
  - Bug 1 (crash): writing `src/mcp2515.c` using ESP-IDF's `driver/spi_master.h`
    crashed the board before `app_main()` even ran, with `assert failed:
    xTaskCreateStaticPinnedToCore ... xPortCheckValidTCBMem(pxTaskBuffer)`,
    triggered inside `esp_hosted`'s own `__attribute__((constructor))` init
    (`managed_components/espressif__esp_hosted/host/port/esp/freertos/src/
    port_esp_hosted_host_init.c`). Confirmed via bisection this was purely
    caused by `esp_driver_spi` being linked into the binary at all (not our
    code's logic/timing - a build with the SPI code present but unreachable/
    dead-code-eliminated booted fine). Patched the vendor file to use a
    properly-sequenced `ESP_SYSTEM_INIT_FN` instead of the raw constructor
    (a real bug fix, kept), and added a diagnostic print proving 168KB free
    heap at the crash point (ruling out memory exhaustion). Neither fix
    alone resolved the crash. **Actual fix**: rewrote `mcp2515.c`/`.h` to
    bit-bang SPI over plain GPIO instead of `driver/spi_master.h`, so
    `esp_driver_spi` is never linked in at all. This sidesteps the issue
    rather than root-causing it, but is fully reliable. Commit `cc5f59a`.
    Full details in `/memories/repo/esp-hosted-crash-fix.md`.
  - Bug 2 (wrong GPIOs): `main.c`'s `CAN_SCK/MOSI/MISO/CS_GPIO` `#define`s had
    the exact opposite mapping from `Doc/electrical drawing.png` (drawing:
    49=CS, 50=SCK, 51=MOSI, 52=MISO; code had 49=SCK, 50=MOSI, 51=MISO,
    52=CS). User had wired per the drawing (correct); the firmware defines
    were the bug. Fixed in commit `f0c8970`.
  - Confirmed on hardware after both fixes: P4 boots cleanly every time
    (WiFi connects + gets IP, BLE advertises, CAN bridge initializes), and
    the full CAN round trip works with zero errors across many frames -
    Arduino TX always `OK`, P4 always logs `CAN RX ... -> echoing`, Arduino
    RX always reports "echo matches last TX".
  - Next step: Phase 2 - replace the Arduino's test payload with an actual
    USB<->CAN bridge protocol (e.g. SLCAN) so a PC can inject/observe real
    CAN frames via the Arduino.
