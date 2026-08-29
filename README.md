# JC-ESP32P4-M3 ESP-IDF project

Board currently used in this workspace: ESP32P4_Guition4_7.

This project is built and flashed with pure ESP-IDF. PlatformIO and pioarduino are not used.

The board contains an ESP32-P4 application processor and an ESP32-C6 wireless companion. This firmware targets the P4 app and uses the onboard C6 as the ESP-Hosted radio co-processor over SDIO. It blinks an LED on GPIO 28 and exposes both BLE and USB CDC control paths to change the blink rate at runtime.

## Wireless status

The ESP32-P4 has no built-in Wi-Fi or Bluetooth radio. The onboard ESP32-C6 is the wireless processor that provides BLE and Wi-Fi for the board. The validated architecture is:

```text
ESP32-P4 app -> ESP-Hosted SDIO -> onboard ESP32-C6 radio
```

Validated boot output shows the C6 responding over SDIO, hosted Wi-Fi enabled, and BLE HCI running over hosted VHCI:

```text
transport: Features supported are:
transport:      * WLAN
transport:        - HCI over SDIO
transport:        - BLE only
vhci_drv: Host BT Support: Enabled
vhci_drv:      BT Transport Type: VHCI
Hosted radio link is up (P4 host, C6 co-processor).
NimBLE host started (controller on C6 over hosted link).
BLE advertising started as JC-P4-C6
```

Manual C6 flashing is not the normal development path. The C6 is expected to run ESP-Hosted slave firmware and is reset/used by the P4 host firmware.

## Project layout

- `CMakeLists.txt`: ESP-IDF project entry point
- `src/CMakeLists.txt`: component registration for the firmware app
- `sdkconfig.defaults`: default ESP-IDF build configuration
- `src/main.c`: firmware entry point and control logic
- `src/idf_component.yml`: ESP-IDF component manager dependencies for TinyUSB, ESP-Hosted, and Wi-Fi Remote
- `tools/run-idf.ps1`: local helper for building, flashing, and monitoring via ESP-IDF

## Requirements

- VS Code
- ESP-IDF installed through Espressif IDE or a standard ESP-IDF toolchain
- USB serial port to the board on COM3 or the connected port in your system

## Build and run

From the project root:

```bash
idf.py set-target esp32p4
idf.py build
idf.py -p COM3 flash
idf.py -p COM3 monitor
```

On Windows, the helper script is also available:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run-idf.ps1 -Action build -ProjectPath . -Target esp32p4
powershell -ExecutionPolicy Bypass -File .\tools\run-idf.ps1 -Action flash -ProjectPath . -Target esp32p4 -Port COM3
powershell -ExecutionPolicy Bypass -File .\tools\run-idf.ps1 -Action monitor -ProjectPath . -Target esp32p4 -Port COM3
```

The helper auto-detects the installed ESP-IDF root under `%USERPROFILE%\.espressif\frameworks` and passes `IDF_TARGET=esp32p4` so flash and monitor do not accidentally configure as another target.

If flashing reports `COM3` access denied, close all serial monitors first. Only one process can use `COM3` at a time.

## BLE control from Android

The dedicated Android project is `CarTheftGuard`:

```text
git@github.com:sarusiy/CarTheftGuard.git
C:\projects\CarTheftGuard
```

The firmware advertises as:

```text
JC-P4-C6
```

Use nRF Connect on Android to validate the current BLE control path:

1. Scan for `JC-P4-C6`.
2. Connect to the device.
3. Open service `0xFFF0`.
4. Write text to characteristic `0xFFF1`.
5. Subscribe to characteristic `0xFFF2` to receive the board's `OK` or `ERR` response.

Valid writes are ASCII text commands:

```text
freq 100
freq 250
freq 500
freq 1000
```

The command value is the LED blink half-period in milliseconds. Valid range is `10` to `60000`.

Expected serial monitor logs:

```text
BLE cmd 'freq 250' -> OK
```

The same result is sent to BLE response characteristic `0xFFF2` as UTF-8 text, for example `OK freq=250 ms`.

Invalid examples such as `freq 5`, `freq 70000`, or `hello` are rejected by the shared parser.

## Wi-Fi status

The ESP-Hosted SDIO Wi-Fi transport is up and the C6 reports WLAN support. CarTheftGuard provisions Wi-Fi credentials over BLE characteristic `0xFFF3` using an UTF-8 payload of `SSID`, newline, then password. The board reports `WiFi connected ip=<address>` over response characteristic `0xFFF2` after joining the network.

Once connected, the board exposes this local-LAN endpoint for frequency control:

```text
POST http://<board-ip>/api/frequency
Body: freq <ms>
```

The endpoint is intended for a trusted local network during development. Add authentication and HTTPS before deploying it on an untrusted network.

## LED pin

The project uses GPIO 28 as configured in the firmware source. This is the board pin currently used for the test LED.

## Live frequency control

Open the board's control USB serial port at 115200 baud. The application prints `ON` and `OFF` while blinking. Enter these commands followed by Enter:

```text
freq <ms>  set the blink half-period from 10 to 60000 ms
help       show the command menu
```

Example:

```text
freq 250
```

## ESP-IDF environment

This project expects an installed ESP-IDF under one of the standard locations, such as:

- `%USERPROFILE%\.espressif\frameworks\esp-idf*`
- `C:\Espressif\esp-idf*`

If the helper script reports that ESP-IDF is missing, open Espressif IDE or install the standard ESP-IDF toolchain first.
