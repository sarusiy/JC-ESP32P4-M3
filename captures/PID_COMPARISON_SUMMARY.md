# Fiat 500 (2014) vs Skoda Fabia (2026) — OBD-II PID comparison

Source data: [`fiat500_2014/PID_SUPPORT_ANALYSIS.md`](fiat500_2014/PID_SUPPORT_ANALYSIS.md),
[`fabia_2026/FABIA_ANALYSIS.md`](fabia_2026/FABIA_ANALYSIS.md).

**Correction, 2026-09-13 evening**: the Fabia answers Mode 01 from *two* physical ECUs,
`0x7E8` and `0x7E9`, each with its own PID-support mask. Earlier versions of this table used
`981a0001`, which turned out to be `0x7E9`'s mask (a secondary/gateway module) — not
`0x7E8`'s, which is very likely the real engine ECU and supports a much larger set, closely
matching the Fiat's own profile. The table below now uses `0x7E8`'s mask throughout.
`0x7E9`'s narrower set (`01,04,05,0C,0D,0F,20` — 7 PIDs, all a subset of `0x7E8`'s) is kept
as a footnote rather than its own column, since everything it supports is also on `0x7E8`.

## 1. Master PID table

Legend: **YES** = ECU reports it supported · `no` = ECU reports not supported ·
`?` = page not queried yet · 🔵 = currently queried+decoded+shown live by our firmware
(the 4 PIDs in `FiatMonitorFragment`/`MonitorFragment` today), the rest are
confirmed-supported-but-not-yet-decoded.

| PID | Name | Fiat 500 | Fabia (`0x7E8`) | Notes |
|-----|------|----------|-------|-------|
| 0x01 | Monitor status since DTCs cleared | YES | YES | |
| 0x03 | Fuel system status | YES | YES | |
| 0x04 | Calculated engine load | YES | YES | |
| 0x05 | Engine coolant temperature | 🔵 YES | 🔵 YES | decoded+live on both |
| 0x06 | Short term fuel trim, Bank 1 | YES | YES | |
| 0x07 | Long term fuel trim, Bank 1 | YES | YES | |
| 0x0B | Intake manifold absolute pressure | YES | YES | both MAP-based, not MAF |
| 0x0C | Engine RPM | 🔵 YES | 🔵 YES | decoded+live on both; byte-level decode of real Fabia drive data confirming it's genuinely live (not frozen) in `FABIA_ANALYSIS.md`'s multi-ECU section |
| 0x0D | Vehicle speed | 🔵 YES | 🔵 YES | same confirmation as RPM |
| 0x0E | Timing advance | YES | YES | |
| 0x0F | Intake air temperature | YES | YES | |
| 0x11 | Throttle position | 🔵 YES | 🔵 YES | decoded+live on both; earlier "anomaly" was `0x7E9`'s narrower mask, not a real gap -- resolved, byte-level decode in `FABIA_ANALYSIS.md` |
| 0x13 | O2 sensors present (2 banks) | YES | YES | |
| 0x14 | O2 Sensor 1 (Bank 1, Sensor 1) | YES | **no** | only confirmed difference between the two cars' engine-ECU profiles so far |
| 0x15 | O2 Sensor 2 (Bank 1, Sensor 2) | YES | YES | |
| 0x1C | OBD standards this vehicle conforms to | YES | YES | |
| 0x1F | Run time since engine start | YES | YES | |
| 0x20 | PIDs supported [0x21-0x40] | 🔵 YES (meta) | 🔵 YES (meta) | queried by firmware to fetch page 2 |
| 0x21 | Distance traveled with MIL on | YES | YES | |
| 0x2E | Commanded evaporative purge | no | YES | Fabia-only, not on the Fiat |
| 0x2F | Fuel Tank Level Input | YES | YES | |
| 0x30 | Warm-ups since codes cleared | YES | YES | |
| 0x31 | Distance traveled since codes cleared | YES | YES | |
| 0x33 | Absolute Barometric Pressure | YES | YES | |
| 0x34 | O2S1_WR_lambda(1): wide-range ratio | no | YES | Fabia-only, not on the Fiat |
| 0x3C | Catalyst Temperature, Bank 1 Sensor 1 | YES | YES | |
| 0x40 | PIDs supported [0x41-0x60] | YES (meta) | YES (meta) | both cars: 3rd page exists, neither queried yet |

**Every other standard Mode 01 PID** (0x02, 0x08-0x0A, 0x10, 0x12, 0x16-0x1B, 0x1D-0x1E,
0x22-0x2D, 0x32, 0x35-0x3B, 0x3D-0x3F) is confirmed **not** supported by either car's
engine ECU, through page 2. Page 3 (0x41-0x60) is unqueried on both.

## 2. What's missing / what to query next

**Fiat 500** — page 1 and page 2 are both fully confirmed. Remaining work is
*decoding*, not discovery:
- Third support page (0x41-0x60) has never been queried — `0x40` being set means it exists.
- Of the 18 (+7 page-2) confirmed-supported PIDs, only 4 are actually decoded and shown
  live (coolant, RPM, speed, throttle). Worthwhile next decodes, roughly in order of
  usefulness: **0x2F Fuel Tank Level** (trivial single-byte, same formula as throttle),
  **0x04 Engine load**, **0x06/0x07 fuel trims**, **0x0B Intake MAP**, **0x33 Barometric
  pressure**, **0x3C Catalyst temperature**, **0x0E Timing advance**, **0x0F Intake air
  temp**, **0x1F Runtime since start**.

**Skoda Fabia** — pages 1 and 2 are now confirmed *per-ECU* (see above); the multi-ECU and
throttle-anomaly questions that used to block trusting this data are resolved (both ECUs'
RPM/Speed track each other and real driving closely -- see FABIA_ANALYSIS.md):
- **Query `0x7E8`'s page 3 (0x41-0x60)** — `0x40` being set means it exists, same as the
  Fiat; neither car's third page has been queried yet.
- Of the confirmed-supported PIDs, the same 4 are decoded+live as on the Fiat (coolant,
  RPM, speed, throttle) — the same next-decode priority list from the Fiat section above
  applies here too, now that `0x7E8`'s fuller mask confirms the Fabia supports all of them.
- The 17-vs-7-PID gap between `0x7E8` and `0x7E9` is itself informative: `0x7E9` most likely
  isn't the engine ECU at all, just a module that mirrors a handful of live values -- worth
  keeping in mind if a future decode target turns out to only live on one of the two.

## 3. Reverse-engineering research: what's available with ignition off

### Why standard OBD PIDs (Mode 01) don't work key-off

This matches what we've already found empirically on both cars (`baseline` captures are
always empty until the switch reaches ACC): SAE J1979 Mode 01 PIDs are answered by the
**engine/emissions ECU**, which is unpowered/asleep with the ignition off. No amount of
polling the standard `0x7DF` functional broadcast (or the Fiat's 29-bit equivalent) will
wake it — this is normal, universal OBD-II behavior, not specific to either car.

### Where key-off data actually lives: the comfort/convenience CAN bus

VAG-group vehicles (Skoda/VW/Audi/SEAT, including the Fabia's MQB-A0 platform) split
their CAN buses by function — commonly a **Drivetrain/Powertrain CAN**, a
**Comfort/Convenience CAN** ("Komfort-CAN"), an **Infotainment CAN**, etc. — all bridged
through a **Central Gateway** module (VAG part-number family **J533**). The
powertrain bus goes to sleep with the engine ECU; the comfort bus, however, is woken
independently by events that have nothing to do with the ignition switch: a door
handle touch, a remote-key button press, keyless-entry approach, alarm-system polling,
etc. This is exactly how factory remote central locking, interior lighting, and
aftermarket remote-start/alarm installer kits work with the engine fully off — they tap
the comfort CAN, not the OBD powertrain broadcast.

Practically, this means:
- The data isn't reached via Mode 01 PIDs at all — it needs **UDS (ISO 14229)** requests
  addressed to specific body/comfort modules (not the OBD functional broadcast address),
  routed through the gateway.
- Merely connecting a diagnostic tool and sending *any* valid CAN traffic can itself act
  as a wake trigger for the gateway on these platforms — this is why tools like VCDS can
  read/code comfort modules with the key fully out, even though they can't get anything
  from the engine ECU. A `TesterPresent` (`0x3E`) frame is the standard way to hold a
  woken diagnostic session open.
- A good first experiment on the actual Fabia: try a **VIN read** (Mode 09 PID 0x02, or
  UDS `22 F1 90`) with the ignition fully off. VIN is stored non-volatile in the gateway
  itself and is one of the only things that reliably answers on *any* car regardless of
  ignition state — if that responds and Mode 01 doesn't, it directly confirms the
  gateway/comfort side is reachable independently of the engine bus, and is a template
  for probing which other modules answer key-off.

### Resources found

I didn't find a public database that lists exactly *which* key-off comfort-CAN PIDs
exist for this specific Fabia — that level of detail (module diagnostic addresses,
per-signal decode) tends to live behind paid tools (VCDS/ODIS/OBD11) or scattered
forum posts rather than one crowdsourced repo. What's genuinely useful as reference
material for reverse-engineering the comfort bus on this platform:

- [iDoka/awesome-automotive-can-id](https://github.com/iDoka/awesome-automotive-can-id) —
  curated index of CAN-ID databases across brands, including several VW/Skoda/Audi
  entries (MQB Simos engine-logging variables, PQ46 drivetrain CAN ID summary, VW e-Up
  and e-Golf docs). Good starting index rather than a single source of truth.
- [mrfixpl/MQB-sniffer](https://github.com/mrfixpl/MQB-sniffer) — real sniffed UDS
  requests/responses on an MK7 Golf (MQB platform, same family as the Fabia's MQB-A0),
  e.g. instrument cluster at diagnostic address `0x714`/`0x77E` and gearbox at
  `0x7E1`/`0x7E9` — useful as a concrete example of MQB's per-module UDS addressing
  pattern (distinct from the legacy `0x7DF`/`0x7E8` Mode 01 broadcast the Fabia currently
  answers on).
- [v-ivanyshyn/parse_can_logs — VW CAN IDs Summary](https://github.com/v-ivanyshyn/parse_can_logs/blob/master/VW%20CAN%20IDs%20Summary.md) —
  older PQ46-platform (Golf V/Passat B6 era) drivetrain CAN ID list; not MQB and not
  verified against the Fabia, but shows the *kind* of thing that lives on a body/comfort
  ID (e.g. its `0x390` bundles door status, lighting, wipers, brake signals) — useful as
  a pattern to look for when scanning the Fabia's own comfort bus traffic.
- [bri3d/MQBSimosLogVariables](https://github.com/bri3d/MQBSimosLogVariables) — extended,
  manufacturer-specific logging PIDs (beyond standard J1979) for Simos18-family ECUs used
  on some MQB engines — relevant if a future Fabia engine turns out to use a Simos ECU,
  not confirmed for this one.
- Ross-Tech's VCDS wiki/forums (`wiki.ross-tech.com`, `forums.ross-tech.com`) — the
  closest thing to a canonical reference for VAG-group module addressing and coding, but
  I couldn't pull a complete address list from search snippets alone; worth a manual
  browse if this gets serious (e.g. "Ross-Tech Wiki: Module 09-Cent. Elect." for the BCM,
  "Module 19-CAN Gateway" for J533).

### Update 2026-09-13 evening: the key-off test was done, with a twist

`0x17F00010` was checked with the ignition fully off (engine off, doors closed) as
recommended above — it's still present, confirming it's not powertrain-related. But the
more important discovery was about the recording method itself, not the vehicle: comparing
a Passive-mode capture against an Active-mode capture of the same resting car showed
`0x17F00010`'s real rate is a mundane **2Hz**, not the ~99%-of-traffic dominance seen in
every prior capture — that dominance was an artifact of our board's Listen-Only mode never
ACKing the frame, causing the real sender to auto-retransmit it ~700-800x faster than
intended. Full writeup: [`fabia_2026/FABIA_ANALYSIS.md`](fabia_2026/FABIA_ANALYSIS.md#bus-fingerprint-one-can-id-dominates-almost-everything----corrected-2026-09-13-evening).

**Revised guidance**: prefer Active-mode captures over Passive when the goal is
characterizing real bus traffic on this car (or likely any real vehicle) — Passive mode's
lack of ACK measurably distorts both the rate of high-frequency messages and hides
lower-frequency ones entirely, the opposite of what "just listening" is usually assumed
to do.
