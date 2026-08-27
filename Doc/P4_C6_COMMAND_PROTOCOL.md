# Android BLE Command Protocol

## Purpose

Define the command contract used by Android BLE clients to control the ESP32-P4
application. The onboard ESP32-C6 is used as an ESP-Hosted radio co-processor;
commands are handled by the P4 firmware, not by a separate user C6 application.

## BLE endpoint

- Advertised device name: `JC-P4-C6`
- Service UUID: `0xFFF0`
- Command characteristic UUID: `0xFFF1`
- Characteristic properties: write and write-without-response
- Payload encoding: ASCII/UTF-8 text

## Command set v0

- `freq <ms>`
  - Range: 10 to 60000
  - Semantics: blink half-period in milliseconds

## Response set v0

The current BLE characteristic is write-only from the Android user's point of
view. Responses are visible in the ESP-IDF serial monitor and by observing the
LED timing.

Expected monitor messages:

- `BLE cmd 'freq=<ms>' -> OK` for accepted writes
- `BLE cmd '<payload>' -> ERR` for invalid writes

The shared parser internally formats these response strings:

- `OK freq=<ms> ms`
- `ERR invalid value. Enter a number of ms (10-60000).`
- `ERR unknown command '<cmd>'. Type 'help'.`

## Transport framing

- BLE write payload is a single text command.
- Newline is optional for BLE writes.
- USB CDC still accepts line-based commands with `\r` or `\n` terminators.
- Maximum BLE command payload currently fits in the firmware's 63-byte command buffer.

## Current implementation status

- P4 parser is implemented in `src/main.c` via `apply_freq_command()`.
- USB CDC and BLE write callbacks use the same parser and validation range.
- ESP-Hosted SDIO link to the onboard C6 is operational.
- Hosted NimBLE VHCI is enabled; BLE advertises as `JC-P4-C6`.
- Android phone discovery and connection to service `0xFFF0` are confirmed.

## Next TODO

1. Validate several valid and invalid `freq` writes from Android.
2. Add optional response characteristic `0xFFF2` with read/notify support.
3. Build the Android app screen that scans, connects, and writes `freq <ms>`.
4. Add BLE-based Wi-Fi provisioning after LED control is stable.
