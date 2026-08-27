# Wi-Fi, BLE, and CarTheftGuard Kickoff

## Goal

Enable Wi-Fi and Bluetooth Low Energy (BLE) for the JC-ESP32P4-M3 system, then connect it to the Android application in the `sarusiy/CarTheftGuard` repository.

The first Android milestone is a GUI control that changes the LED blink half-period on GPIO 28.

## Repositories

- Firmware: `sarusiy/JC-ESP32P4-M3`
- Android project/app name: `CarTheftGuard`
- Android repository: `git@github.com:sarusiy/CarTheftGuard.git`
- Expected local Android checkout: `C:\projects\CarTheftGuard`

## Current firmware baseline

- Board (repository naming): JC-ESP32P4-M3
- Physical board in use: ESP32P4_Guition4_7
- Framework: native ESP-IDF
- LED: GPIO 28
- Existing control: USB CDC serial command `freq <ms>`
- BLE control: advertised device `JC-P4-C6`, service `0xFFF0`, write characteristic `0xFFF1`
- Valid half-period: 10 to 60000 ms
- Existing status output: `ON` and `OFF`
- P4 debug/control port used by this workspace: COM3
- Current build target: ESP32-P4 only

## Hardware constraint

The JC-ESP32P4-M3 is a dual-chip board containing an ESP32-P4 and an onboard
ESP32-C6. The P4 does not include an on-chip Wi-Fi or Bluetooth radio; the C6
is the board's Wi-Fi/BLE processor. BLE therefore cannot be enabled by the
current P4 application alone.

The native ESP-IDF build targets `esp32p4`. For this board class, the expected
path is ESP-Hosted: the P4 runs the host application while
the onboard C6 acts as the radio co-processor. BLE and Wi-Fi are driven from
the P4 over the hosted link, rather than by manually flashing a separate C6 app
in normal development.

Manual C6 reflashing is an exception path used only when the host determines the
co-processor firmware is incompatible and needs an update.

## Proposed first communication design

Use BLE first from the P4 host application over the C6 hosted radio path.
Wi-Fi can be initialized from the same hosted link later, but it does not need
to be part of the first frequency-control screen.

### BLE service

The firmware exposes one custom GATT service with one writable characteristic for the LED frequency:

- Service UUID: `0xFFF0`
- Frequency characteristic UUID: `0xFFF1`
- Write format: ASCII `freq <ms>` initially, matching the USB command
- Accepted values: 10 to 60000 ms
- Current response path: serial monitor log and visible LED timing change

The next firmware improvement is an optional response characteristic, for example `0xFFF2`, using read/notify for `OK` and `ERR` text responses.

## Confirmed hardware/runtime status

- Native ESP-IDF build succeeds for target `esp32p4`.
- Native ESP-IDF flash succeeds on `COM3` when serial monitors are closed.
- ESP-Hosted SDIO link to the onboard C6 comes up on slot 1.
- Runtime log confirms C6 features: WLAN, HCI over SDIO, BLE only.
- Hosted NimBLE VHCI is enabled and BLE advertises as `JC-P4-C6`.
- Android phone connection to service `0xFFF0` is confirmed.

## Firmware work plan

1. Done: enable hosted radio path on the P4 using ESP-Hosted, Wi-Fi Remote, and SDIO.
2. Done: bring up BLE initialization and advertising from the P4 host app.
3. Done: add BLE GATT service `0xFFF0` and writable frequency characteristic `0xFFF1`.
4. Done: reuse the existing blink-period validation and update logic.
5. Done: start hosted Wi-Fi transport and station interface.
6. Done: keep USB CDC control working as a diagnostic fallback.
7. Done: print hosted, BLE, and command validation events to the debug console.
8. In progress: verify multiple BLE frequency writes and invalid command handling on hardware.
9. Next: add Wi-Fi credential provisioning and connection status reporting.

## Android work plan

In `sarusiy/CarTheftGuard`:

1. Inspect the existing Android project structure and minimum SDK.
2. Add BLE scan and device connection handling.
3. Discover service `0xFFF0` and write characteristic `0xFFF1`.
4. Add the first GUI screen with:
   - Connection status
   - Scan/connect action
   - Frequency input or slider
   - Send/apply action
   - Current command result or validation error
5. Send `freq <ms>` as UTF-8 text to the board.
6. Display write success/failure clearly; later read/subscribe to response characteristic `0xFFF2` once firmware adds it.
7. Test reconnect behavior and Android Bluetooth permission handling.

## Open decisions

- Wi-Fi provisioning is a later feature after BLE LED control is stable.
- BLE service/characteristic UUIDs are currently `0xFFF0` and `0xFFF1`.
- Decide whether frequency values represent half-period milliseconds or full blink-cycle frequency.
- Confirm the desired Android minimum SDK and supported Android versions.
- Board advertises fixed name `JC-P4-C6`.
- Decide whether BLE responses use a new `0xFFF2` notify/read characteristic or write-only status in the Android UI.

## Definition of done for the kickoff milestone

- The hosted P4-to-C6 radio path is identified and operational.
- Wi-Fi hosted transport initializes without breaking the existing application.
- BLE advertises and accepts a connection from Android BLE tools.
- The Android GUI can set a valid LED half-period.
- Invalid values are rejected by both firmware and the GUI.
- GPIO 28 visibly blinks at the selected timing.
- USB CDC control remains available for diagnostics.
