# Wi-Fi, BLE, and CarTheftGuard Kickoff

## Goal

Enable Wi-Fi and Bluetooth Low Energy (BLE) for the JC-ESP32P4-M3 system, then connect it to the Android application in the `sarusiy/CarTheftGuard` repository.

The first Android milestone is a GUI control that changes the LED blink half-period on GPIO 28.

## Repositories

- Firmware: `sarusiy/JC-ESP32P4-M3`
- Android: `sarusiy/CarTheftGuard`

## Current firmware baseline

- Board: JC-ESP32P4-M3
- Framework: ESP-IDF through PlatformIO
- LED: GPIO 28
- Existing control: USB CDC serial command `freq <ms>`
- Valid half-period: 10 to 60000 ms
- Existing status output: `ON` and `OFF`
- Verified serial control port: COM5

## Hardware constraint

The ESP32-P4 does not include an on-chip Wi-Fi or Bluetooth radio. Wi-Fi/BLE
cannot be enabled by firmware on the P4 alone. This work therefore requires a
wireless companion such as an ESP32-C6 or ESP32-H2, connected through a suitable
board interface, or a separate external wireless module.

Before implementation, confirm which wireless hardware is present on the
JC-ESP32P4-M3 design and how the P4 will communicate with it. The current
board definition lists Wi-Fi and Bluetooth as connectivity metadata, but that
does not mean the P4 contains those radios.

## Proposed first communication design

Use BLE from the wireless companion for the initial Android-to-board control path. Wi-Fi can be enabled and initialized in parallel, but it does not need to be part of the first frequency-control screen.

### BLE service

Create one custom GATT service with one writable characteristic for the LED frequency:

- Service UUID: TBD
- Frequency characteristic UUID: TBD
- Write format: ASCII `freq <ms>` initially, matching the USB command
- Accepted values: 10 to 60000 ms
- Response: `OK freq=<ms> ms` or an error message

The UUIDs and whether the characteristic should use notifications will be finalized during implementation.

## Firmware work plan

1. Identify the wireless companion and its P4 connection interface.
2. Add the companion transport between the P4 and wireless controller.
3. Add Wi-Fi initialization using the selected wireless hardware and transport.
4. Add BLE initialization and advertising on the wireless controller.
5. Add a BLE GATT service and writable frequency characteristic.
6. Reuse the existing blink-period validation and update logic.
7. Keep USB CDC control working as a diagnostic fallback.
8. Print connection, write, and validation events to the debug console.
9. Verify Wi-Fi startup, BLE discovery, frequency writes, and LED timing on hardware.

## Android work plan

In `sarusiy/CarTheftGuard`:

1. Inspect the existing Android project structure and minimum SDK.
2. Add BLE scan and device connection handling.
3. Discover the board's custom service and frequency characteristic.
4. Add the first GUI screen with:
   - Connection status
   - Scan/connect action
   - Frequency input or slider
   - Send/apply action
   - Current command result or validation error
5. Send the selected half-period to the board.
6. Display the board response and connection failures clearly.
7. Test reconnect behavior and Android Bluetooth permission handling.

## Open decisions

- Confirm whether Wi-Fi is needed for the first Android control path or only for later features.
- Choose the final BLE service and characteristic UUIDs.
- Decide whether frequency values represent half-period milliseconds or full blink-cycle frequency.
- Confirm the desired Android minimum SDK and supported Android versions.
- Confirm whether the board advertises a fixed device name.
- Decide whether BLE responses use notifications or a write-only command with readback.

## Definition of done for the kickoff milestone

- The wireless companion and P4 transport are identified and operational.
- Wi-Fi initializes without breaking the existing application.
- BLE advertises and accepts a connection from `CarTheftGuard`.
- The Android GUI can set a valid LED half-period.
- Invalid values are rejected by both firmware and the GUI.
- GPIO 28 visibly blinks at the selected timing.
- USB CDC control remains available for diagnostics.
