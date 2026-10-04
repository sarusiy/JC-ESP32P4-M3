# Watch Control, Link Security, and Multi-Board Requirements

## Purpose

Capture requirements that came out of the first Amazfit Balance watch app and the
S3 BLE control service (2026-10-03). Nothing in the "Required" sections below is
implemented yet; they are planned work, in no committed order.

Projects involved:

- `JC-ESP32S3-CAN`: board firmware (`src/bringup_s3.c`)
- `CarTheftGuard`: Android app (`BoardLink.java`) and the watch app in `watch/`

## Current status

### Implemented

- Watch app (`watch/`) controls the LED blink rate over BLE: service `0xFFF0`,
  write `freq <ms>` to `0xFFF1`, reply notified on `0xFFF2`.
- Two simultaneous BLE clients (phone app + watch), capped at 2 on purpose.
  Each reply goes only to the client that sent the command.
- The watch connects only with full 128-bit UUIDs and pairing disabled.
- Link alerts (implemented and verified on the watch and phone):
  - Watch app: when the board link comes up it plays two short pulses and a
    short chirp; when an established link is lost it vibrates for 5 seconds and
    plays a 5-second alarm. An on-screen Alert button (OFF / VIB / VIB+SND),
    saved on the watch, controls this. Only works while the watch app
    is open: Zepp OS does not allow the BLE client calls in background
    services, and leaving the app ends the connection.
  - Android app: posts a "Board connected" / "Board disconnected" notification
    (channel "Board link") when the board's Wi-Fi link comes up or is lost, which
    the Zepp app can mirror to the watch while the watch app is closed. The
    phone holds no BLE connection to the board (BLE is only used to find it),
    so "link" there means the Wi-Fi link. Works only while the app process is
    alive.

### Known weaknesses today

- The Wi-Fi AP password `&Car1310` is hardcoded in the firmware
  (`WIFI_AP_PASSWORD`) and in the app (`BoardLink.AP_PASSWORD`), and is in git
  history.
- The BLE control service has no authentication: any device in range can write
  to it.
- Every board advertises the same BLE name (`JC-P4-C6`), AP SSID
  (`CarTheftGuard-P4`) and IP (`192.168.4.1`), and the app has one fixed
  SSID/password, so the app can only work with one board.
- The watch app connects to the first device whose name starts with `JC-`.

## Required: arm / disarm the car from the watch (later)

Planned for when the immobilizer / central-disable hardware is ready (see the
relay design notes in `RELAY_REMOTE_CONTROL.md`).

- The watch can arm and disarm the car.
- Must use real cryptographic authentication between watch and board (for
  example a challenge-response with a per-watch secret key). BLE proximity,
  RSSI and MAC address must not be treated as proof of presence: BLE range can
  be extended by a relay device, so range is not a security boundary.
- The board must reject arm/disarm from any unauthenticated client, including a
  second connected client.
- Arm/disarm must be refused or delayed under the same safety checks as the
  existing relay actuation (for example no actuation while the engine is
  running).
- The command and result should be visible on the watch (armed / disarmed /
  rejected) and logged on the board.

## Required: stronger link security

1. **Wi-Fi password**
   - Interim: replace `&Car1310` with a random passphrase of 20 or more
     characters (WPA2 allows up to 63), changed in both `bringup_s3.c` and
     `BoardLink.java`, then reflash and rebuild the app.
   - Target: a random per-board password generated at first boot, stored in
     flash, and handed to the app once over an authenticated BLE setup step.
     No password in source, git or the APK. Include a factory-reset path for
     when the app loses the password.
2. **BLE authentication**
   - Protect the `0xFFF0` control service (at least anything beyond harmless
     settings) with authentication, either BLE bonding with an authenticated
     pairing method or an application-level challenge-response.
   - This is a prerequisite for arm/disarm.
3. **Secrets out of the repository**: no passwords or keys committed in source.
   The old password is already public in history, so it must be rotated, not
   just removed.

## Required: Android app supports more than one S3 board

- The app can know about several boards (for example one per vehicle), not a
  single hardcoded one.
- Each board needs a unique identity so the app can tell them apart:
  - BLE name and AP SSID carry a unique suffix (for example from the MAC
    address) instead of the shared `JC-P4-C6` / `CarTheftGuard-P4`.
  - Each board has its own Wi-Fi password (see the target design above), and
    the app stores credentials per board.
- The app lets the user discover, add, name (for example by vehicle) and
  select a board; it remembers the last used board.
- A phone can join only one board's SoftAP at a time, so the app connects to
  one selected board at a time and switches explicitly. The fixed AP IP
  (`192.168.4.1`) can stay the same on every board.
- Capture files and logs are labeled with the board they came from.
- The watch app should connect to a chosen, remembered board (stored MAC)
  instead of the first `JC-` device it sees.
- Existing single-board setups keep working after the change.

## Deferred ideas (not needed now)

- Watch Status screen: BLE-connected state plus board state (detected partner,
  Passive/Active mode, a few live values). Needs a new BLE status
  characteristic on the board, because the watch cannot use the board's HTTP
  API.
- Watch Passive/Active mode toggle.
- Watch alert (vibration and notification) on a theft event reported by the
  board.
- RSSI readout in the watch app to measure real BLE range.

## Open questions

- Which authentication scheme for arm/disarm: BLE bonding with a secure pairing
  method, or an application-level challenge-response with a per-watch key?
  This depends on what the Zepp OS BLE API supports for secure pairing.
- Can several watches be enrolled for one board, and how is a lost watch
  revoked?
- Does the maximum of 2 simultaneous BLE clients still suffice once multiple
  boards or multiple watches exist?
