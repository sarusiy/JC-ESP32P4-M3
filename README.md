# JC-ESP32P4-M3 PlatformIO Blink Demo

This is a minimal VS Code + PlatformIO demo project for the JC-ESP32P4-M3 board.
It blinks an LED on GPIO 28 using ESP-IDF and provides a USB CDC control channel
for changing the blink frequency while the application is running.

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

The project uses `GPIO28`:

```ini
build_flags =
	-D LED_GPIO=28
```

If your board routes the LED to a different pin, edit `platformio.ini` and change `LED_GPIO`.

## Live frequency control

Open the board's control USB serial port at 115200 baud. The application prints
`ON` and `OFF` while blinking. Enter these commands followed by Enter:

```text
freq <ms>  set the blink half-period from 10 to 60000 ms
help       show the command menu
```

For example:

```text
freq 250
```

## Optional CLI commands

If you have PlatformIO CLI installed locally:

```bash
py -3.11 -m platformio run
py -3.11 -m platformio run -t upload --upload-port COM3
py -3.11 -m platformio device monitor -p COM5 -b 115200
```
