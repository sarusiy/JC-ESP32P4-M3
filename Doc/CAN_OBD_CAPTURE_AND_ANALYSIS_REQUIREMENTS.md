# CAN, OBD-II Capture, and Smartphone Analysis Requirements

## Purpose

Define the required behavior across these three projects:

- `JC-ESP32P4-M3`: CAN interface, OBD-II scan tool, capture source, and Wi-Fi server
- `ArdunioUsbBridgeToCan`: simulated vehicle/ECU used for development
- `CarTheftGuard`: Android control, recording, analysis, and display application

The system must support standardized OBD-II request/response traffic and manufacturer-specific periodic CAN broadcasts at the same time.

## CAN traffic types

### Periodic broadcast traffic

Vehicle ECUs transmit status frames continuously without receiving a request. Examples include engine RPM, vehicle speed, ignition, doors, steering, and brake state.

These frames are normally manufacturer-specific. Their CAN IDs, payload layouts, scaling, counters, and checksums must not be assumed to be universal.

### OBD-II request/response traffic

The P4 acts as a diagnostic scan tool and sends standardized requests. For example:

```text
Request:  CAN ID 0x7DF, data 02 01 0C 00 00 00 00 00
Response: CAN ID 0x7E8, data 04 41 0C AA BB 00 00 00
```

- Mode `0x01`: current data
- PID `0x0C`: engine RPM
- Positive response mode: request mode plus `0x40`, therefore `0x41`

Both traffic types coexist. There is no bus-wide switch between broadcast mode and response mode.

## Functional requirements

### P4 firmware

The P4 must:

1. Continue querying and decoding supported standardized OBD-II data.
2. Receive periodic broadcast frames independently of OBD responses.
3. Timestamp every captured frame.
4. Preserve at least:
   - sequence number
   - timestamp
   - bus number
   - CAN identifier
   - standard or extended frame flag
   - RTR flag
   - DLC
   - payload bytes
5. Maintain counters for MCP2515 overflow, software-queue overflow, and transport drops.
6. Expose raw frames incrementally over Wi-Fi without requiring the phone to download duplicate frames.
7. Keep decoded OBD values available through the existing `/api/obd` endpoint.
8. Never echo arbitrary broadcast traffic. Echoing is allowed only in an explicit hardware-test mode.
9. Provide a dedicated passive vehicle-recording mode that performs no application-level transmission.
10. Use MCP2515 listen-only mode when strict passive capture is selected.

### Android application

CarTheftGuard must:

1. Continue BLE provisioning and Wi-Fi control of the P4.
2. Continue displaying decoded standardized OBD values.
3. Start and stop a raw CAN recording session.
4. Record received frames directly to a smartphone file while the screen is off.
5. Use an Android foreground service for long recording sessions.
6. Store raw data without altering it during decoding.
7. Show recording duration, received-frame count, dropped-frame count, and file size.
8. List saved capture sessions with date, duration, and vehicle/profile name.
9. Display raw frames grouped by CAN ID.
10. Later apply a DBC or user-defined signal profile to a saved recording.
11. Display decoded broadcast values after a profile is selected.
12. Export or convert recordings for SavvyCAN, `python-can`, and Cabana-compatible offline analysis where practical.

### Arduino vehicle simulator

The simulator must:

1. Respond to OBD-II Mode 01 requests on `0x7DF` and `0x7E0`.
2. Respond as ECU 1 on `0x7E8`.
3. Simulate supported PIDs including coolant temperature, RPM, speed, and throttle position.
4. Transmit generic periodic engine, vehicle, and body frames independently of OBD requests.
5. Clearly identify its broadcast IDs and payloads as test definitions, not real manufacturer data.
6. Preserve the original `0x100` echo test as a separate build mode.
7. Later simulate stored DTCs, Mode 03 responses, freeze-frame Mode 02 data, and ISO-TP multi-frame responses.

## Proposed capture transport

The first implementation uses these HTTP endpoints:

```text
GET  /api/can?after=<last-sequence>
POST /api/can/mode   body: active | passive
```

`active` keeps normal CAN operation and OBD queries enabled. `passive` places the MCP2515 in listen-only mode and suppresses P4 OBD/test transmissions.

A capture response includes:

```json
{
  "latest": 1250,
  "dropped": 0,
  "frames": [
    {
      "seq": 1249,
      "time_us": 123456789,
      "bus": 0,
      "id": 288,
      "extended": false,
      "rtr": false,
      "dlc": 8,
      "data": "1AF8643200000000"
    }
  ]
}
```

For sustained high-rate capture, migrate to a persistent TCP or WebSocket stream with binary batches, sequence numbers, acknowledgements, and reconnect support.

## Smartphone file format

The first implementation records append-only CSV with phone and board timestamps, sequence number, bus, CAN ID, frame flags, DLC, and hexadecimal payload. CSV keeps initial captures directly inspectable while the protocol is being validated.

A later high-rate format should use compact binary records and a file header containing:

- format version
- recording start time
- configured CAN bitrate
- source device identifier
- vehicle/profile name

Flush data periodically and finalize the file cleanly when recording stops. Convert binary recordings to CSV, ASC, BLF, or analysis-tool formats after capture.

## Confirmed P4 MCP2515 wiring

The unmodified MCP2515/TJA1050 module is powered from 5 V because the TJA1050 requires a 5 V supply.

| MCP2515 module signal | P4 connection |
|---|---:|
| `CS` | GPIO49 |
| `SCK` | GPIO50 |
| `SI` / MOSI | GPIO51 |
| `SO` / MISO | GPIO52 through a 5 V-to-3.3 V divider |
| `INT` | GPIO29 through a 5 V-to-3.3 V divider |
| `VCC` | 5 V |
| `GND` | Common ground |

Use this divider on both `SO` and `INT`:

```text
MCP2515 output ---- 10k ----+---- P4 GPIO
                            |
                           15k
                            |
                           GND
```

At a maximum 5.25 V module supply, the GPIO receives approximately 3.15 V. No additional pull-up or pull-down is required on `INT`. Configure GPIO29 as an input with internal pulls disabled and a falling-edge interrupt because MCP2515 `INT` is active-low.

Direct 3.3 V P4 outputs to the 5 V-powered MCP2515 `CS`, `SCK`, and `SI` inputs are retained only for the current trial configuration. A production design should level-shift these signals to guaranteed 5 V logic levels.

## Performance requirements

- The Wi-Fi link and smartphone storage are expected to handle Classic CAN traffic.
- The MCP2515 receive path is the primary bottleneck because it has only two hardware receive buffers.
- Use MCP2515 `INT` on P4 GPIO29 and drain both receive buffers immediately.
- Replace the current 10 ms polling interval for capture mode.
- Avoid per-frame logging in the receive path.
- Move received frames immediately into a larger P4 RAM ring buffer.
- Test at high simulated bus load and verify all overflow/drop counters remain zero.

## Vehicle safety requirements

When connected to a real vehicle:

- Begin in strict listen-only mode.
- Do not echo, acknowledge through normal mode, inject, or modify frames during discovery.
- Do not enable the MCP2515 module's 120-ohm termination on an already terminated vehicle bus.
- Use a shared reference ground unless the CAN interface is galvanically isolated.
- Do not transmit diagnostic requests until the bus type, bitrate, wiring, and gateway behavior are verified.
- Never perform immobilization or other safety-critical actions while the vehicle is moving.

## Current status

### Implemented

- P4 Mode 01 OBD requests and decoding for selected PIDs
- P4 `/api/obd` endpoint
- P4 interrupt-assisted reception on MCP2515 `INT` / GPIO29
- P4 simultaneous OBD response routing and raw broadcast capture
- P4 selectable active or strict MCP2515 listen-only mode through `/api/can/mode`
- P4 bounded raw-frame ring with 64-bit sequences and incremental `/api/can?after=<sequence>` endpoint
- P4 ring-overwrite and MCP2515 hardware-overflow reporting
- P4 arbitrary-frame echo disabled; only the dedicated `0x100` test frame is echoed
- Android display of decoded OBD values over Wi-Fi
- Android foreground CAN recording service and dedicated Record tab
- Android timestamped CSV capture in the app-owned `can-captures` directory
- Android list of the ten newest saved recording files
- Arduino Mode 01 simulated ECU responses
- Arduino generic periodic engine, vehicle, and body broadcast frames
- Separate Arduino echo-test build configuration

### Not implemented yet

- Full saved-session browser, sharing/export, and raw CAN-ID display
- DBC/profile-based decoding of captured broadcasts
- SavvyCAN/Cabana export or conversion
- Mode 02 freeze-frame, Mode 03 DTC, and ISO-TP multi-frame support

## Recommended implementation phases

1. Validate the Arduino periodic broadcasts and Mode 01 responses on hardware.
2. Stop arbitrary P4 frame echoing and add explicit test versus passive modes.
3. Add P4 timestamped buffering, drop counters, and incremental capture API.
4. Add Android start/stop recording with a foreground service and local files.
5. Add saved-session raw frame browsing and CAN-ID grouping.
6. Add DBC/profile import and decoded signal display.
7. Add export/conversion for desktop analysis tools.
8. Add Mode 02, Mode 03, and ISO-TP support.
