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
- Response characteristic UUID: `0xFFF2`
- Response properties: read and notify
- Wi-Fi configuration characteristic UUID: `0xFFF3`
- Wi-Fi configuration properties: write
- Payload encoding: ASCII/UTF-8 text

## Command set v0

- `freq <ms>`
  - Range: 10 to 60000
  - Semantics: blink half-period in milliseconds

## Response set v0

After each command, the firmware sends the result as a UTF-8 notification on
`0xFFF2`. The latest result can also be read from that characteristic.

Expected responses:

- `OK freq=<ms> ms` for accepted writes
- `ERR invalid value. Enter a number of ms (10-60000).` for invalid ranges
- `ERR unknown command '<cmd>'. Type 'help'.` for unknown commands

The ESP-IDF serial monitor also logs whether the write was accepted or rejected.

## Transport framing

- BLE write payload is a single text command.
- Newline is optional for BLE writes.
- USB CDC still accepts line-based commands with `\r` or `\n` terminators.
- Maximum BLE command payload currently fits in the firmware's 63-byte command buffer.

## Wi-Fi provisioning and control

`0xFFF3` accepts one UTF-8 payload in this format:

```text
SSID\npassword
```

The board responds first with `OK WiFi connecting`, then reports `WiFi connected ip=<address>` on `0xFFF2` after it gets an address. Frequency control then moves to the local network:

```text
POST http://<board-ip>/api/frequency
Body: freq <ms>
```

## Current implementation status

- P4 parser is implemented in `src/main.c` via `apply_freq_command()`.
- USB CDC and BLE write callbacks use the same parser and validation range.
- ESP-Hosted SDIO link to the onboard C6 is operational.
- Hosted NimBLE VHCI is enabled; BLE advertises as `JC-P4-C6`.
- Android phone discovery and connection to service `0xFFF0` are confirmed.
- The firmware exposes `0xFFF2` for board response read/notify.

## Next TODO

1. Validate several valid and invalid `freq` writes and `0xFFF2` notifications from Android.
2. Add BLE-based Wi-Fi provisioning after LED control is stable.
