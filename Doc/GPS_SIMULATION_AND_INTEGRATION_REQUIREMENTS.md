# GPS Simulation and Integration Requirements

## Purpose

Define the required behavior across these three projects for adding simulated GPS
positioning to the existing CAN/OBD-II development and capture pipeline:

- `ArdunioUsbBridgeToCan`: simulated vehicle/ECU, now also a simulated GPS receiver
- `JC-ESP32P4-M3`: ingest the GPS UART feed, parse it, and expose it over Wi-Fi
- `CarTheftGuard`: display live position and log it alongside CAN recordings

This is a bench/development feature: no real GNSS antenna or module is used. The
Arduino generates realistic NMEA 0183 sentences representing a moving vehicle inside
Israel, so the rest of the pipeline (P4 parsing, HTTP API, Android display/logging)
can be built and validated without real hardware.

## System data flow

```text
Arduino (SoftwareSerial TX, NMEA sentences, 9600 baud)
        --> P4 UART RX (hardware UART, e.g. UART1)
        --> P4 NMEA parser (RMC/GGA) -> latest fix state
        --> GET /api/gps  (decoded JSON)
        --> Android: Monitor tab shows live lat/lon/speed/heading,
            optional GPS columns appended to the existing CAN capture CSV
```

A P4 TX -> Arduino RX return path is wired but not required for the first phase;
it is reserved for future commands (e.g. "jump to location", "start route").

Future command handling should also be used to validate the return UART path:
P4 GPIO35 TX -> Arduino D2 RX. A minimal first test command can be a simple
ASCII line such as `PING\n`, with the Arduino replying on the GPS stream with
a valid NMEA-style acknowledgement sentence or a simulator status sentence.

## Confirmed P4 UART wiring

The JP1 header (the same connector already used for the MCP2515 CAN wiring) has
these free GPIOs, confirmed against `Doc/electrical drawing.png`:
GPIO30, GPIO31, GPIO32, GPIO33, GPIO34, GPIO35. Everything else on that header is
already committed (GPIO52/51/50/49 = MCP2515 MISO/MOSI/SCK/CS, GPIO29 = CAN INT,
GPIO28 = status LED). The C6_* signals and I2C_SCL/SDA on the same header are
reserved for the C6 co-processor link and I2C bus, and are excluded here.

ESP32-P4 UART signals route through the GPIO matrix, so any free GPIO pair works.
Use **UART1** (UART0 is reserved for the native-USB console) on two adjacent free
pins on the same header:

| Signal | P4 GPIO | Direction |
|---|---:|---|
| GPS RX (receives NMEA from Arduino TX) | GPIO34 | Input |
| GPS TX (reserved, P4 -> Arduino, unused for now) | GPIO35 | Output |

The Arduino Uno is a 5V-logic device, so its `SoftwareSerial` TX output (D3) is 5V
and must be divided down before reaching the 3.3V-only P4 GPIO34, using the same
10K/15K divider already used for the MCP2515 `SO`/`INT` lines:

```text
Arduino D3 (5V TX) ---- 10k ----+---- P4 GPIO34 (RX)
                                 |
                                15k
                                 |
                                GND
```

The reverse direction (P4 GPIO35 TX -> Arduino RX) needs no divider: the P4's 3.3V
output is comfortably above the ATmega328P's ~3.0V logic-HIGH threshold at 5V
supply. This path is reserved/unused for now regardless.

## Important hardware constraint (Arduino Uno)

The Arduino Uno has exactly **one hardware UART** (`Serial`, pins D0/D1), and it is
already fully committed to the existing USB CAN bridge (PC monitor + status logging).
Therefore the simulated GPS output **must** use `SoftwareSerial` on two spare digital
pins, not the hardware UART.

Proposed spare-pin usage (already-used pins: D0/D1 = USB serial, D10 = MCP2515 CS,
D11/D12/D13 = SPI MOSI/MISO/SCK):

| Signal | Arduino pin |
|---|---:|
| GPS TX (Arduino -> P4 RX) | D3 |
| GPS RX (P4 -> Arduino, reserved/unused for now) | D2 |

`SoftwareSerial` at the standard GPS module baud rate of 9600 is required; higher
rates are unreliable on `SoftwareSerial` while the CAN polling loop is also running.

## Functional requirements

### Arduino GPS simulator

The simulator must:

1. Run a `SoftwareSerial` port at 9600 baud on the pins above, independent of the
   existing hardware `Serial` used by the CAN bridge.
2. Emit standard NMEA 0183 sentences at a fixed update rate (1 Hz to start):
   - `$GPRMC` (recommended minimum: time, status, lat/lon, speed, course, date)
   - `$GPGGA` (fix quality, lat/lon, altitude, satellite count)
3. Compute a valid NMEA checksum for every sentence.
4. Simulate a moving vehicle within the Israel region, for example a bounded loop
   or waypoint path inside approximately:
   - latitude 29.5&nbsp;N to 33.3&nbsp;N
   - longitude 34.2&nbsp;E to 35.9&nbsp;E
5. Vary speed and heading smoothly (no instant jumps) so downstream consumers can
   sanity-check motion, matching the style already used for the simulated
   RPM/speed/throttle in the OBD-II simulator.
6. Report a valid fix (`A`/status "active", fix quality &gt;= 1) once startup
   completes; simulate a brief "no fix" period at boot to exercise P4/Android
   handling of missing data.
7. Keep this feature additive: existing CAN bridge / OBD-II simulator behavior on
   the hardware UART must be unaffected.
8. Later accept simple command input on GPS RX (Arduino D2) to validate the
   P4 TX line and control simulator behavior. Initial commands should include:
   - `PING` to prove P4 GPIO35 TX -> Arduino D2 RX connectivity
   - update-rate control (for example, `SIMRATE,<ms>` or compatible `$PMTK220`)
   - forced fix/no-fix mode (`SIMFIX,0|1`) for Android error-state testing
   - route selection inside Israel (`SIMROUTE,TLV|JLM|HAIFA|NEGEV`)

### P4 firmware

The P4 must:

1. Configure UART1 for GPS RX at 9600 baud on GPIO34 (RX) / GPIO35 (TX, reserved),
   per the confirmed wiring table above, defined the same way as the existing CAN
   pins (`#ifndef GPS_UART_RX_GPIO / #define GPS_UART_RX_GPIO 34` etc.) so they
   can be overridden at build time without editing the parser.
2. Parse incoming NMEA sentences line-by-line, validating the checksum before use.
3. Decode at minimum: fix validity, latitude, longitude, speed (km/h), heading
   (degrees), UTC time/date, and satellite/fix-quality indicator.
4. Keep the most recent decoded fix in memory, similar to the existing
   `s_obd_state` pattern, guarded the same way (no torn reads across HTTP handler
   and UART task).
5. Expose the latest fix over a new endpoint:

   ```text
   GET /api/gps
   ```

   ```json
   {
     "fix_valid": true,
     "lat": 32.0853,
     "lon": 34.7818,
     "speed_kmh": 42.5,
     "heading_deg": 187.3,
     "utc_time": "142530",
     "utc_date": "040926",
     "satellites": 8
   }
   ```

6. Do not block or slow down the existing CAN capture path; GPS parsing runs in its
   own task, independent from `can_echo_task` and `obd_query_task`.
7. Log a rate-limited warning (not per-sentence) when checksum validation fails or
   the parser sees malformed input, to avoid flooding the console during startup
   "no fix" periods.
8. Later transmit simulator-control commands over UART1 TX (GPIO35) to validate
   the P4 TX -> Arduino RX path and support repeatable GPS edge-case tests.

### Android application

CarTheftGuard must:

1. Poll `GET /api/gps` on the same cadence as the existing `/api/obd` polling in
   the Monitor tab.
2. Display live latitude, longitude, speed, and heading; show a clear "no fix" /
   "waiting for GPS" state when `fix_valid` is false.
3. Optionally append GPS fields (lat, lon, speed, heading) as extra columns to the
   existing CAN capture CSV row, timestamp-aligned with the phone/board timestamps
   already recorded by `CanCaptureService`, so a single recording session carries
   both CAN and position data.
4. Treat GPS polling failures the same way as existing `/api/obd` failures (no
   crash, keep retrying), consistent with current error handling conventions.

## Performance requirements

- GPS updates are low-rate (1 Hz) and low-bandwidth compared to CAN capture; they
  must not compete meaningfully with the CAN ring-buffer HTTP responses.
- NMEA parsing must complete well within one 1-second update interval, with
  headroom for the existing CAN/OBD/Wi-Fi workload on the same core.

## Current status

### Implemented

- Arduino `SoftwareSerial` NMEA simulator (`$GPRMC`/`$GPGGA`) emitting Israel-region motion.
- P4 UART1 GPS parser on GPIO34 RX / GPIO35 TX and `GET /api/gps` endpoint.
- Hardware validation of Arduino TX -> P4 RX: `/api/gps` returned valid fix data.
- Android `BoardLink` polling for `GET /api/gps`.
- Android Monitor tab display of live GPS fix, latitude, longitude, speed, heading, and satellites.
- Android CAN capture CSV rows enriched with the latest GPS snapshot.

### Not implemented yet

- GPS simulator command handling on Arduino D2 RX.
- P4 UART TX command sender on GPIO35.
- End-to-end P4 TX -> Arduino RX validation command/ack path.
- Android runtime validation on phone with the GPS-enabled APK.

## Recommended implementation phases

1. Wire the confirmed P4 GPS UART pair (GPIO34 RX via 10K/15K divider, GPIO35 TX
   reserved/no divider needed) on JP1 to the Arduino SoftwareSerial pins (D3 TX /
   D2 RX reserved), matching the wiring table above.
2. Implement the Arduino `SoftwareSerial` NMEA simulator (`$GPRMC`/`$GPGGA`,
   checksum, Israel-bounded simulated movement) as an additive change alongside
   the existing CAN bridge/OBD-II simulator.
3. Implement the P4 NMEA parser and in-memory fix state, validated first over the
   existing USB serial console/log output before wiring the HTTP endpoint.
4. Add the P4 `GET /api/gps` endpoint and verify with direct HTTP polling
   (matching the existing `/api/obd` verification approach).
5. Add a P4-to-Arduino GPS command/ack test to validate GPIO35 TX -> D2 RX.
6. Add basic simulator commands for update rate, forced fix/no-fix, and Israel
   route selection.
7. Validate Android polling and Monitor-tab display of live position/speed/heading on phone.
8. Validate GPS columns in `CanCaptureService` CSV output on phone.
