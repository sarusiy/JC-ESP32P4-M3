# JC-ESP32P4-M3 PlatformIO Blink Demo

This is a minimal VS Code + PlatformIO demo project for the JC-ESP32P4-M3 board.
It blinks the on-board LED using ESP-IDF.

## Project layout

- `platformio.ini`: PlatformIO environment
- `boards/jc_esp32p4_m3.json`: Custom board definition for ESP32-P4
- `src/main.c`: Blink application (`app_main`)

## Requirements

- VS Code
- PlatformIO IDE extension (`platformio.platformio-ide`)

## How to run

1. Open this folder in VS Code.
2. Install the recommended extension when prompted.
3. Connect the JC-ESP32P4-M3 board by USB.
4. Build:
	- PlatformIO sidebar -> `Build`
5. Upload:
	- PlatformIO sidebar -> `Upload`
6. Monitor serial output:
	- PlatformIO sidebar -> `Monitor` (115200 baud)

## LED pin

The project defaults to `GPIO2`:

```ini
build_flags =
  -D LED_GPIO=2
```

If your board routes the on-board LED to a different pin, edit `platformio.ini` and change `LED_GPIO`.

## Optional CLI commands

If you have PlatformIO CLI installed locally:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```
