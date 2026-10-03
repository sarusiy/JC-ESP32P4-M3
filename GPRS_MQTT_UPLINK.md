# Cellular (GPRS/4G) uplink

Status as of 2026-09-28: planning only, no firmware written yet. Mandatory
feature (not optional) -- lets the app arm/disarm and monitor the car
from anywhere with coverage, not just BLE/SoftAP range.

## Purpose and hard design rule

Extends the existing connectivity roadmap (BLE = pairing/presence,
SoftAP = near-range local control) with a far-range channel. **Cellular
is additive, never a dependency**: BLE + SoftAP must keep working fully
with the modem absent, unpowered, SIM-less, or out of coverage. See the
GPIO block comment above `GPRS_UART_TX_GPIO` in `bringup_s3.c` for the
same rule anchored in code.

## Hardware

- **Module**: Air780E (4G Cat-1, UART AT-command control). Deliberately
  not a 2G-only module (e.g. SIM800L) -- 2G is being sunset by many
  carriers, a 2G-only module risks going dead within a year or two.
  SIM7600 series is the fallback if Air780E is hard to source.
- **GPIOs** (`bringup_s3.c`): `GPRS_UART_TX_GPIO`=13,
  `GPRS_UART_RX_GPIO`=14, `GPRS_PWRKEY_GPIO`=38, on `UART_NUM_2` (GPS
  already owns `UART_NUM_1`), 115200 baud.
- **PWRKEY is a pulse, not a logic level** -- CONFIRMED from the actual
  breakout board's own pinout diagram (2026-09-28): pull `PWK` to GND
  for **1.5 seconds** to boot the module.
- **This board's own regulator**: `VIN` accepts DC 4.7-12V directly (an
  onboard regulator produces the module's own 3.3-4.3V `VBAT` rail) --
  don't need to pre-regulate to exactly 3.3-4.2V before feeding it.
- **Power supply**: current spikes up to ~2A during transmit bursts even
  at low average draw. Needs a large capacitor (1000-2200uF) right at
  the module and a supply that won't brown out under that spike -- a
  known, common failure mode on this class of module if underpowered.
- **SIM card slot -- CONFIRMED 2026-09-28**: the AliExpress listing's
  board didn't show one in 3 photos checked, but a different listing
  (found by the user, same Air780E module family, "V1.5" breakout board)
  has a clearly labeled push-push SIM tray (silkscreened `CARD1`) in the
  middle of the board, alongside the same VIN/PWK/RST/TXD/RXD/DTR/VBUS
  pinout already confirmed above -- this is the board to buy, not the
  earlier AliExpress one without a visible slot.
- **Antenna**: External GSM antenna via u.FL/IPEX (`RF1`/"TLF") -- don't
  bury it inside a closed metal enclosure. There's a **second** RF
  connector (`RF2`/"GPS") for a separate GNSS antenna -- this module has
  built-in GPS, a bonus not currently needed (the project already has
  its own GPS module) but worth knowing it's there.
- **Status LEDs**: `NET` and `PWR` are silkscreened on the board --
  useful for visual debugging without a UART monitor attached.

## Modem bring-up sequence

**Official reference**: https://doc.openluat.com/wiki/37?wiki_page_id=4491
(Air780E Development Guide) -- consult this first, the sequence below is
inferred from a real test-log screenshot, not the full manual.

**Confirmed real AT command sequence** (from an actual SSCOM terminal
session against this exact module, 2026-09-28) -- SIMCOM-style TCP/IP
stack commands (same family as SIM800/900), which is the most widely
documented dialect in the hobbyist community:

```
AT+CPIN?      -> +CPIN: READY        (SIM present/unlocked)
AT+CSQ        -> +CSQ: 29,0          (signal quality -- must be > 18)
AT+CGATT?     -> +CGATT: 1           (packet service attached -- must be 1)
AT+CSTT       -> OK                  (start task / set APN)
AT+CIICR      -> OK                  (bring up wireless connection)
AT+CIFSR      -> 10.2.144.125        (IP address -- if one comes back, it worked)
```

**Confirmed from the module's own spec sheet**: network registration is
fast, typically **2-3 seconds** under normal signal -- use this as the
realistic timeout/backoff baseline rather than guessing.

Runs in its own FreeRTOS task so a stuck/registering modem never blocks
CAN, BLE, or the SC48/SC6 relay logic:

1. Pulse `GPRS_PWRKEY_GPIO` low for 1.5s to power the module on.
2. Poll plain `AT` until the module responds, with a timeout.
3. `ATE0` to disable command echo (cleaner parsing of responses).
4. `AT+CPIN?` -- confirm a SIM is present and unlocked. No SIM is a
   normal, expected outcome here, not an error to crash on.
5. `AT+CSQ` -- check signal quality; treat anything at or below 18 as
   "no usable signal" rather than pushing ahead and failing later.
6. `AT+CGATT?` poll for packet-service attach (expect `1`), with
   backoff -- per the module's own spec this should resolve in a few
   seconds, not the tens of seconds worth planning around for weaker
   modules.
7. `AT+CSTT` (APN) -> `AT+CIICR` -> `AT+CIFSR` to bring up the data
   connection and confirm an IP came back.
8. Many modules in this class also expose a higher-level built-in MQTT
   client over AT commands (e.g. `AT+MQTTOPEN`/`AT+MQTTCONN`/
   `AT+MQTTPUB`/`AT+MQTTSUB`-style) which would mean the S3 firmware
   never has to implement raw MQTT itself -- check the openluat wiki for
   this before building an MQTT client in firmware code.
9. Open the MQTTS session to the broker (see below), subscribe to the
   command topic, publish an online status.
10. On failure at any step: back off, retry, log the failure reason into
    the cellular status below -- never block other tasks waiting on this.

**Exposed status**: a small state (`OFF` / `NO_SIM` / `SEARCHING` /
`REGISTERED_NO_MQTT` / `CONNECTED`) kept in a mutex-protected variable,
surfaced through the existing local HTTP API (e.g.
`GET /api/cellular/status`) so the app can tell -- while it's still
locally connected via BLE/SoftAP -- whether remote monitoring will
actually work today, without that status gating anything else.

## Backend: MQTT over TLS (MQTTS)

Hosted broker (HiveMQ Cloud / EMQX Cloud or similar free/cheap tier),
not self-built -- matches the user's own stated preference (easy
encrypted channel, no need to over-engineer network-layer security
against an unsophisticated thief).

### Topic structure

Prefix everything under a per-device namespace so the design doesn't
have to change if a second vehicle is ever added:

```
cartheftguard/<device_id>/...
```

`<device_id>` -- not yet decided; candidate: the S3's own MAC address,
or a fixed manually-assigned ID.

**Commands** (app publishes, car subscribes), QoS 1:
- `.../cmd/arm` -- arm SC48+SC6.
- `.../cmd/disarm` -- disarm SC48+SC6.
- `.../cmd/status_request` -- ask the car to republish full status now.

**Status** (car publishes, app subscribes), retained so a freshly
subscribing app immediately gets the last known value without waiting
for the next update:
- `.../status/online` -- `true`/`false`. Set `true` on connect; register
  the `false` value as the connection's **Last Will and Testament (LWT)**
  so the broker auto-publishes it if the car drops off ungracefully
  (power loss, lost coverage) -- this is what actually tells the app
  "car is unreachable" instead of silence.
- `.../status/armed` -- `true`/`false`, mirrors current SC48/SC6 relay
  state.
- `.../status/gps` -- `{"lat":.., "lon":.., "fix":true/false, "ts":..}`.
- `.../status/cellular` -- `{"registered":.., "signal":..}`.
- `.../status/can_mode` -- active/passive, mirrors the existing local API.

**Alerts** (car publishes, app subscribes), QoS 1, **not** retained --
one-shot events, not state:
- `.../alert/intrusion` -- published when something happens while armed
  (exact trigger condition not yet defined -- candidates: a
  door-open attempt while SC6 is cut, the audio deterrent firing, a
  future interior-camera motion trigger).

QoS 0 is fine for `status/gps` (a dropped position update doesn't
matter, the next one follows shortly); QoS 1 for commands and alerts --
losing an arm/disarm command or an intrusion alert silently is not
acceptable.

## Open items

- [ ] **Confirm the SIM card slot exists on this board** before ordering
      -- not visible in 3 product photos checked so far; check buyer
      review photos or ask the seller.
- [x] Core AT command sequence confirmed (see above): CPIN/CSQ/CGATT/
      CSTT/CIICR/CIFSR, SIMCOM-style. Still open: whether it has a
      built-in MQTT AT command set vs. needing a raw MQTT client in
      firmware -- check the openluat wiki.
- [ ] Decide `<device_id>`.
- [ ] Define the actual trigger condition(s) for `alert/intrusion`.
- [ ] Confirm brownout-safe power supply sizing (capacitor value, supply
      current rating) once the module's real datasheet is in hand.
