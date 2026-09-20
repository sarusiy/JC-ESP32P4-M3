# Skoda Fabia (2026) — Body/comfort module research (doors, locks, lights, horn)

Researched 2026-09-13 evening, to scope adding body-control signals (door
status, lock/unlock, lights, horn) to what the board can actively request,
alongside the existing OBD-II Mode 01 PID polling. This is genuinely a
different subsystem from Mode 01 -- see "Why this isn't just more PIDs"
below before reading the scoping section.

## What's confirmed reachable, and how we know

The Fabia's Mode 01 OBD traffic answers on the classic functional broadcast
(`0x7DF` request, `0x7E8`/`0x7E9` response) -- but that's specifically the
*emissions/powertrain* diagnostic path, standardized and public since 1996.
Body/comfort functions (locks, doors, lights) live on a completely
different, manufacturer-proprietary diagnostic scheme: **UDS (ISO 14229)**,
addressed to specific physical modules rather than a shared broadcast, all
reachable through the same OBD-II port via the vehicle's central gateway
(same architecture discussed in `../PID_COMPARISON_SUMMARY.md`).

### Candidate module addresses

Sourced from a community-maintained collection of VAG UDS diagnostic IDs
(extracted from ODIS, VAG's own factory diagnostic tool) --
[ConnorHowell/vag-uds-ids](https://github.com/ConnorHowell/vag-uds-ids).
**Caveat**: this list isn't confirmed specifically against a 2026 Fabia --
VAG addressing is fairly consistent within a UDS generation across the
group's platforms, but these are strong candidates to try, not verified
facts, until tested against the real car.

| Function | Module | Request → Response |
|---|---|---|
| Gateway (bridges all buses, already implicitly used for Mode 01 today) | Central Gateway | `0x710` → `0x77A` |
| Locks/central convenience (likely = VCDS "Module 46") | Central Module, Comfort System | `0x70D` → `0x777` |
| Locks specifically | Lock Electronics | `0x71E` → `0x788` |
| Driver door | Door Electronics, Drive Side | `0x74A` → `0x7B4` |
| Passenger door | Door Electronics, Passenger Side | `0x74B` → `0x7B5` |
| Rear driver door | Door Electronics, Rear Drive Side | `0x73E` → `0x7A8` |
| Rear passenger door | Door Electronics, Rear Passenger Side | `0x73F` → `0x7A9` |
| Headlights (leveling) | Headlight Regulation | `0x754` → `0x7BE` |
| Headlights (high-beam assist) | High Beam Assist | `0x730`/`0x748` → `0x79A`/`0x7B2` |
| Horn/alarm siren | *(no dedicated module -- actuated via Central Convenience/Lock Electronics above)* | same as locks |
| Instrument cluster (for reference/cross-check, not part of this research's goal) | Dash Board | `0x714` → `0x77E` |

### The gap: no public DID values

Unlike Mode 01 PIDs, the actual **Data Identifiers** (the 2-byte value you'd
send with a `ReadDataByIdentifier` request to one of these addresses to get
"is the driver door open" or "are the doors locked") are not published
anywhere crowdsourced -- confirmed across GitHub UDS-ID projects, Ross-Tech's
own documentation, and forum archives. Ross-Tech's docs add a specific
wrinkle: on **UDS-protocol ECUs** (which this 2026 Fabia's modules are),
VCDS uses a different "Advanced Measuring Blocks" mechanism than the older
KWP-era "Group 001, 4 fields" style some forum posts describe -- so even
those older door-status descriptions may not map cleanly onto this car's
actual DID scheme. This detail lives behind ODIS/VCDS/OBDeleven's
proprietary databases, not anywhere public.

**Practical conclusion**: addresses are knowable in advance (above); DIDs
are not -- closing that gap needs empirical probing against the real car,
the same way the Learn wizard already correlates raw CAN IDs with physical
actions, just extended to UDS request/response instead of passive
listening.

## Why this isn't just more PIDs -- what's actually different

The current firmware's whole OBD-II mechanism (`obd_send_request`,
`obd_query_task`) is built around one specific pattern that doesn't fit
UDS body-module requests at all:

1. **Functional broadcast vs. physical addressing.** Mode 01 sends one
   request to `0x7DF` and *whichever* ECU wants to answer does, on its own
   ID within a fixed `0x7E8-0x7EF` range -- this is why the responder-lock
   logic exists (see `FABIA_ANALYSIS.md`'s multi-ECU section). UDS to a
   body module is **physical, point-to-point**: request `0x71E`, response
   *only* ever on `0x788`, nothing else. No multi-ECU ambiguity, but also
   nothing in the current code recognizes `0x788` as a response to
   anything -- `can_echo_task`'s `is_obd_response` check only matches the
   `0x7E8-0x7EF` (or extended equivalent) range.
2. **Different service IDs.** Mode 01/03/04/09 (the only services this
   firmware speaks) are single-byte modes specific to J1979. UDS uses
   `0x10` (Diagnostic Session Control), `0x22` (Read Data By Identifier),
   `0x3E` (Tester Present), etc. -- a different service set, and
   `ReadDataByIdentifier`'s payload is a 2-byte DID, not a 1-byte PID.
3. **Session state.** Many UDS body modules only expose a small default
   set of DIDs until a diagnostic session is explicitly opened (`0x10
   0x03`, extended session) -- something Mode 01 never needs, since OBD-II
   emissions data is required to be available with zero setup by
   regulation.

## Firmware scoping

Proposed shape of the work, in build order -- **not yet implemented**,
this is the plan to review before writing code:

### 1. Generalize the request/response mechanism

Today, `obd_send_request()` always targets the fixed OBD functional
broadcast, and `can_echo_task` recognizes responses by range-checking
against `0x7E8-0x7EF`/extended. Needs a parallel, simpler path for UDS:
send to an exact request ID, expect a response on an exact response ID --
no range check, no responder-lock (physical addressing makes both
unnecessary by construction). `isotp_reassemble()` is already generic
enough to reuse as-is for multi-frame UDS responses.

### 2. A target table, mirroring `pids[]`

A small static table -- `{name, request_id, response_id, extended, service,
did}` -- for the confirmed-useful entries once discovered, polled in
`obd_query_task`'s loop the same way Mode 01 PIDs are today. This is the
"request list" piece: once a DID is known-good, it becomes a permanent
entry here, same status as a decoded OBD PID.

### 3. Session handling

A helper to send `0x10 0x03` (extended session) to a target module before
reading, since default-session DID availability is unknown per module
until tested. Cheap to always send once per polling cycle per target if
the module tolerates it; `0x3E` Tester Present only needed if we want to
hold a session open across multiple reads instead of re-opening it each
time -- start without it, add only if testing shows it's needed.

### 4. The actual discovery tool: a DID-sweep mode

The piece that closes the empirical gap. A new one-shot mode (same
pattern as the existing VIN read: triggered via HTTP, runs in
`obd_query_task`, reports results asynchronously) that:
- Takes a target module (request/response ID pair) and a DID range.
- Sends `0x22 <DID>` for each DID in the range, logging whatever comes
  back (or a timeout) for each one.
- Exposes the raw results via a new endpoint (e.g. `GET /api/uds/scan`)
  so the phone app -- or a raw `curl`/PowerShell call while standing at
  the car -- can review them.

Used like this: start a sweep against `0x71E` (Lock Electronics), unlock
the car, note which DID's value changed; repeat locked; the DID that
flips between the two states is the answer, found directly rather than
looked up. Same idea for `0x74A` (driver door) against open/close.

### Suggested first target

Start with **`0x71E` (Lock Electronics)** and a modest DID range (e.g.
`0x0100`-`0x0200`, a commonly-used proprietary VAG range per the research
above) toggling lock/unlock -- narrowest, clearest state to correlate
(binary, driver-triggered on demand, no ambiguity about what "should" be
different). Doors and lights are natural next targets once the mechanism
is proven against locks.

## Update 2026-09-15 — gateway architecture, and the first real-car test

**First real test (2026-09-14, Learn session)**: swept all 8 candidate
addresses above (lock/unlock/headlights/high_beam/door_open/door_close
steps) -- zero responses across the board. This predates the
`uds_scan_log_*.csv` audit trail (see below), so it's unconfirmed whether
the scans even fired; that's the first thing to check on the next visit.

Separately, researched how the OBD-II port actually reaches these modules
at all, since the addresses above are physical (point-to-point), not
broadcast. Findings:

- VAG's Gateway (**J533**) is what the OBD-II connector's CAN pins
  actually terminate at -- it bridges several separate physical CAN
  networks (Drivetrain/Powertrain, Convenience/Comfort, Infotainment,
  Extended, Discrete), each potentially at a different bit rate. A device
  on the OBD-II bus that isn't the Gateway itself is, by one
  reverse-engineer's account, not directly reachable at all
  ([aep/vag_reverse_engineering](https://github.com/aep/vag_reverse_engineering/blob/master/LOG.md)):
  *"the only thing reachable from the obd can bus is the can gateway."*
- Confirms this isn't just a logical separation: reverse-engineering notes
  for a **VW Polo** (same MQB-A0 platform as this Fabia) document the
  Comfort/Infotainment CAN bus running at **100 kbit/s**, a separate
  physical network from the 500 kbit/s diagnostic/drivetrain bus
  ([P1kachu/talking-with-cars](https://github.com/P1kachu/talking-with-cars/blob/master/notes/vw-polo-r6.txt)).
- **This does not mean the body modules are unreachable via OBD-II**,
  though -- VCDS/OBDeleven/ODIS all diagnose and code exactly these
  modules (door electronics, central locking) through the standard 16-pin
  connector on this exact platform family, every day, with no special
  wiring. That only works if the Gateway actively bridges properly-formed
  diagnostic requests across to the Comfort CAN segment. So: **OBD-II
  should be sufficient; physical rewiring to a different bus is not
  indicated** by anything found so far.

**Reframed question**: not "can we reach it," but "why isn't the Gateway
bridging *our* specific requests." Ranked candidate explanations:

1. **Wrong CAN IDs for this specific model/year** -- the source list
   (`ConnorHowell/vag-uds-ids`) is ODIS-extracted but not confirmed against
   a 2026 Fabia specifically. Still the most likely explanation.
2. **Missing a Diagnostic Session Control (`0x10`) step first** -- some
   modules (and possibly the Gateway's own routing logic) only forward or
   answer `ReadDataByIdentifier` inside a non-default session. Evidence is
   mixed (the aep notes say session control "is not needed... but might be
   needed for others"), but it's cheap to try and was already flagged as
   step 3 of the firmware scoping below, just not implemented yet.
3. **Transport-format mismatch** -- VW has historically used its own
   wrapper protocol (VW TP 2.0) in places; if our raw ISO-TP framing
   doesn't match what this gateway generation expects for a given module,
   it could silently drop the request rather than NRC it.
4. Less likely: the module genuinely isn't populated/active on this trim.

Sources: [aep/vag_reverse_engineering](https://github.com/aep/vag_reverse_engineering/blob/master/LOG.md),
[P1kachu/talking-with-cars — vw-polo-r6.txt](https://github.com/P1kachu/talking-with-cars/blob/master/notes/vw-polo-r6.txt),
[VW J533 (Gateway) Cable — openpilot wiki](https://github.com/commaai/openpilot/wiki/VW-J533-(Gateway)-Cable),
[The CAN & CAN FD in Volkswagen and Audi Vehicles](https://www.springfieldvw.com/notes/learning/the-can-can-fd-in-volkswagen-and-audi-vehicles/).

**Next step (in progress)**: implement #2 above -- send `0x10 0x03`
(extended diagnostic session) to the target module once before a DID
sweep starts, and record whether it got a positive (`0x50`) response,
negative (`0x7F`), or timeout, alongside the sweep's own results.
**Done same day** -- firmware now sends this automatically before every
sweep; result is in the status JSON's `session` field and logged to
`uds_scan_log_*.csv`. Not yet tested against the real car.

## Update 2026-09-15 (later) — ruled out SFD as the likely cause

Checked whether VAG's **SFD** (Schutz Fahrzeug Diagnose) security gate
could explain the zero-response result, since it's a well-known VAG
diagnostic restriction introduced ~MY2020. Per
[Ross-Tech's own SFD documentation](https://wiki.ross-tech.com/wiki/index.php/SFD)
(the most authoritative source checked): SFD explicitly does **not**
interfere with basic reads or diagnostic session control -- it only gates
*write*-type services (coding, adaptation, basic settings, output tests).
Since our scan tool only ever sends `0x10` (session control) and `0x22`
(read), **SFD is very unlikely to be the cause** -- this was a plausible
single clean explanation and it didn't pan out.

The same page mentions a separate, distinct **"Diagnostic Filter"**
mechanism that can impose stricter restrictions (possibly forcing
read-only mode or blocking access more broadly), but doesn't confirm
whether that produces silent timeouts versus explicit negative responses
-- a weaker, less-specific lead than SFD was, not pursued further yet.

Net effect: back to the original ranked list above (wrong CAN IDs for
this specific model/year most likely, then transport-format mismatch,
then module not populated on this trim) -- no shortcut explanation found,
still needs the real-car test with session-control logging to make
progress.

## Update 2026-09-15 (real-car test) — six for six timeouts

First real test of the session-control logging, at the car (session
folder `Rec0915`, archived to `captures/fabia_2026/20260915_session6_uds/`).
Six candidate targets tried, DID range mostly a minimal single-DID probe
(`0x0100`-`0x0100`) except Lock Electronics (full `0x0100`-`0x0200`,
triggered by the Learn wizard's "Lock" step) -- all via `uds_scan_log_*.csv`:

| Target | Address | Session |
|---|---|---|
| Lock Electronics | `0x71E`→`0x788` | **timeout** |
| Central Convenience | `0x70D`→`0x777` | **timeout** |
| Driver Door | `0x74A`→`0x7B4` | **timeout** |
| Headlight Regulation | `0x754`→`0x7BE` | **timeout** |
| Passenger Door | `0x74B`→`0x7B5` | **timeout** |
| High Beam Assist | `0x730`→`0x79A` | **timeout** |

All six `completed: true` (the firmware's sweep logic itself works
correctly -- reached `did_end` every time) and `result_count: 0`. Not one
of six different physical module addresses produced so much as a session-
control response.

**This reframes the ranked hypothesis list.** Six-for-six uniform silence
across independently-addressed modules is a weaker fit for "we guessed
six wrong addresses" (possible, but the addresses are a coherent,
internally-consistent set from one ODIS extraction, not six independent
guesses) and a better fit for something systemic -- either a
transport/framing mismatch in our own request encoding (hypothesis #3),
or these body modules genuinely aren't reachable this way on this car at
all, independent of which specific address is used.

**The decisive remaining test**: the **Gateway itself** (`0x710`→`0x77A`)
was not tried -- it isn't in `UdsTargets.java` yet (added same day, see
below). The Gateway is the one node already known to answer *something*
on this OBD-II connection (it's implicit in the working Mode 01 traffic).
If the Gateway also times out via this exact scan tool, that's strong
evidence of a transport/framing bug on our side, not six wrong addresses.
If it responds (even just to session control) while every body module
stays silent, that confirms the mechanism works and it's specifically
about how body-module traffic gets routed/addressed.

Untested: Rear Driver Door (`0x73E`→`0x7A8`), Rear Passenger Door
(`0x73F`→`0x7A9`) -- lower priority than the Gateway test given the
uniform pattern so far.

## Update 2026-09-15 (later still) — full address sweep completed: only the Gateway answers

First successful **full** address-discovery sweep (earlier attempts at the
car brownout-reset partway through, see above -- this one ran on stable
PC USB power). Swept the entire plausible diagnostic range in one pass:

```
req_start=0x700, req_end=0x7FF, final_current_req=0x7FF, completed=true
result_count=1, hits: 0x710 (positive)
```

**All 256 standard UDS addresses (`0x700`-`0x7FF`) probed; exactly one
responds: the Gateway (`0x710`) itself.** Nothing else -- not the 8
originally-guessed body-module candidates, not any other address in the
entire practical range. This is a materially stronger result than the
earlier six-for-six-timeout finding: it rules out "we just guessed wrong
addresses" as the explanation, since this covers every plausible address,
not a curated list.

**Updated conclusion**: body-module UDS access (locks, doors, lights) is
very likely **not reachable via direct point-to-point CAN addressing on
this bus segment at all**, on this car, through this connection. The
Gateway is directly addressable and responds normally; nothing else is.
Combined with the earlier gateway-architecture research (VAG's Gateway
bridges separate physical CAN networks -- Drivetrain/Powertrain,
Convenience/Comfort, Infotainment -- and body modules likely live on one
of those other segments), the remaining plausible explanations are:

1. **Real diagnostic tools (VCDS/ODIS) reach body modules *through* the
   Gateway's own routing/session mechanism**, not by addressing them
   directly on the OBD-visible bus segment -- i.e. the Gateway needs to be
   told (via some protocol-level addressing within a session opened with
   *it*, not a separate raw CAN ID per module) to forward/proxy the
   request to a module on another physical segment. This would need
   understanding VAG's actual routing protocol (possibly the "VW TP 2.0"
   wrapper mentioned in the 2026-09-15 gateway-architecture research), not
   just more address guessing.
2. Less likely given how clean this result is: the body modules are
   simply not populated/wired on this car's specific configuration --
   possible but doesn't fit with these being pretty fundamental functions
   (locks, lights) on a production 2026 Fabia.

**Practical next step, if pursued further**: research how to open a
session specifically with the Gateway (`0x710`) that then lets it route a
`ReadDataByIdentifier` through to another module's logical address, rather
than continuing to probe raw CAN IDs directly. This is a different kind of
investigation than anything built so far (protocol-level, not address-
discovery), and a bigger scope than the current DID-sweep/address-sweep
tools support.

## Update 2026-09-16 — found the likely routing mechanism: VWTP 2.0

Followed up on the "practical next step" above and found a concrete,
documented candidate: **VWTP 2.0** (VW Transport Protocol), VW's own
proprietary transport layer, described as "the underlying transport used
in all CAN VWs" by
[baconwaifu/PyVCDS](https://github.com/baconwaifu/PyVCDS) (a from-scratch
VCDS re-implementation via black-box reverse engineering). Its
`vwtp.py` gives concrete, byte-level connection-setup details:

- **Request always goes to CAN ID `0x200`** (fixed, not per-module).
- **Request payload (7 bytes)**: `[dest_addr, 0xC0, 0x00, 0x10, rx_id_lo, rx_id_hi, 0x01]`
  -- `dest_addr` is a **single-byte logical/component address** (the same
  numbers VCDS displays, e.g. `0x46`=Central Convenience, `0x42`=Door
  Electronics Driver -- a completely different addressing dimension from
  the raw CAN IDs in the candidate table above), `rx_id` is a CAN ID *we*
  choose to receive on, `0x01` marks the KWP2000 application protocol.
- **Response comes back on `0x200 + dest_addr`**: a positive ack
  (`byte[1] == 0xD0`) carries a **negotiated TX CAN ID** in bytes 4-5
  (little-endian) -- the actual diagnostic-data CAN ID pair for that
  session is allocated dynamically per-connection, not fixed like the
  `ConnorHowell/vag-uds-ids` table assumes.

This reframes the whole address-hunting approach: if this platform (or
this specific module) still needs VWTP 2.0 rather than direct UDS-over-
CAN, no amount of guessing raw CAN IDs would ever find it, since the
"module addresses" in the public list may be VCDS-internal logical
numbers, not independently-dialable bus addresses at all. It's also
consistent with the Gateway (`0x710`) answering plain UDS directly while
literally nothing else in the full `0x700`-`0x7FF` sweep did -- a modern
gateway plausibly speaks current UDS-over-CAN on its own interface while
still routing to older body modules via this legacy scheme internally.

**Built same day**: a VWTP 2.0 connection-setup discovery tool
(`POST`/`GET /api/vwtp/scan`, Record tab's "VWTP 2.0 connection probe"),
sweeping the full `0x00`-`0xFF` logical-address space with a chosen RX
id, recording any address that gets a real response (positive or
negative) plus its negotiated TX id if positive. Not yet tested against
the real car.

Sources: [baconwaifu/PyVCDS](https://github.com/baconwaifu/PyVCDS),
[vwtp.py](https://github.com/baconwaifu/PyVCDS/blob/master/vwtp.py),
[vwtp.txt](https://github.com/baconwaifu/PyVCDS/blob/master/vwtp.txt),
[aep/vag_reverse_engineering LOG.md](https://github.com/aep/vag_reverse_engineering/blob/master/LOG.md)
(independently mentions the same "session protocol that requests a pair
of canbus ids from the gateway" concept, different car/generation).

## Open risk / things that could go sideways

- These addresses might simply not respond on this specific 2026 Fabia
  (model-year/platform variance) -- first real test is address-validation
  itself, before DID content even matters.
- A DID sweep sends a lot of requests in a short window; worth rate-limiting
  (this firmware's existing `OBD_QUERY_INTERVAL_MS`-style pacing) to avoid
  looking like a flood to the module, even though UDS modules are generally
  fine with sequential polling like this at a sane rate.
- Some DIDs may require Security Access (`0x27`) even to read, not just to
  write -- if a DID sweep gets uniformly rejected/NRC'd rather than timing
  out, that's the likely cause, and would mean that module's data isn't
  reachable without a security-access implementation (a separate, larger
  piece of work, common OEM lockdown for anti-theft-adjacent data
  specifically -- notable given this project's own purpose).

## Update 2026-09-17 — real-car results: only the Gateway is reachable, by either method

**Deep Scan built**: a new tool (`POST`/`GET /api/deepscan`, Record tab's
"Deep scan" section) that combines the direct address-discovery sweep with
an automatic identification-DID sweep (the ISO 14229-1 Annex F block,
`0xF180`-`0xF1A0`) against every address that responds, so a hit gets
identified immediately instead of requiring a manual follow-up DID sweep.
Also given NVS-backed persistence: progress and results are saved as the
scan runs and it resumes automatically at boot if the board resets
mid-scan (see "brownout" below for why that turned out to matter).

**A single frame passively observed while actually driving** (not from an
explicit scan) suggested a possible new responder: request `0x78F` /
response `0x7F9` replied `7F 10 31` (negative response to Diagnostic
Session Control, NRC `0x31` requestOutOfRange) in a raw capture, despite
`0x78F` being inside the already-fully-swept `0x700`-`0x7FF` range that
previously found only the Gateway. Leading theory at the time: some
modules (steering/wheel-speed/stability-adjacent) may only be awake while
the car is actually moving, not just with ignition on. **This did not hold
up**: two dedicated follow-up DID sweeps against `0x78F`/`0x7F9` (targeting
the identification block specifically) while driving both got zero
response of any kind -- not even the negative one seen before -- and a
full Deep Scan across the entire `0x700`-`0x7FF` range (which necessarily
includes `0x78F`) also found nothing there. The original hit looks, in
retrospect, more likely to have been a corrupted/coincidental frame than a
real module. Closed as a dead end.

**Brownout during the VWTP scan, even on stable USB power**: a full VWTP
sweep reliably crashed the board (`E BOD: Brownout detector was
triggered` / `rst:0x3 SW_SYS_RESET`) partway through, repeatedly, live
serial monitor confirmed -- notably, this happened even when powered from
a PC's USB port, which had fully resolved the equivalent brownout for the
plain address-discovery sweep back on 2026-09-15. Root cause of *why*
VWTP draws enough more current to matter is still unclear (same request
pacing, same CAN-ACK-starvation-on-no-response profile as the address
sweep) -- worth a closer look if it recurs. Given the scan couldn't be
trusted to finish, it was given the same NVS-backed resume mechanism as
the Deep Scan (`vwtp_scan_resume_from_nvs`, called at boot) so an
interrupted run picks back up on its own instead of needing a restart.

**Clean, complete real-car results obtained same evening**:
- **Deep Scan, full `0x700`-`0x7FF` sweep**: exactly one hit, the Gateway
  (`0x710`) -- consistent with every earlier finding. Its own
  auto-identification data is a working sanity check for the tool itself:
  DID `0xF197` (systemNameOrEngineType) decoded to ASCII `"GW"`, `0xF193`
  to `"Har"` (very likely **Harman**, a known VAG Tier-1 supplier for
  gateway/infotainment modules), `0xF187` (vehicle manufacturer spare part
  number) to `"3Q0"` (a real VAG part-number prefix). Known limitation:
  DID values are truncated to 5 value bytes (8 total including the 3-byte
  `0x62`/DID header) in this tool's current implementation, so real VAG
  part numbers (usually 9-10 characters) are only showing their prefix --
  worth widening if a future run needs the full string.
- **VWTP scan, full `0x00`-`0xFF` sweep**: completed with zero resets this
  particular run (so the new resume mechanism, while built and deployed,
  didn't actually get exercised by a real crash this time) and found
  **zero responses across the entire logical-address space**.

**Where this leaves the research**: two independent, complete, clean
sweeps -- one by direct CAN-ID addressing, one by VWTP 2.0's session-based
scheme -- now agree that only the Gateway answers anything on this car's
OBD-II-accessible bus segment. This is a meaningfully stronger negative
result than either sweep alone (no longer "we didn't find the right
addresses," but "neither addressing scheme this project knows about
reaches anything but the Gateway"). Two directions worth considering next,
neither yet attempted:
1. Physically check the OBD-II connector pinout for DoIP (100BASE-T1
   Ethernet, typically pins 1/8 in addition to the standard CAN pins 6/14)
   -- this Fabia (VIN-decoded as Typ PJ / MQB-A0, see project memory) isn't
   expected to have it based on every public MQB-A0-vs-MQB-Evo/MEB
   reference found so far, but that's inference, not a direct check.
   **Updated 2026-09-20**: this lead got more credible, not less -- real
   commercial "ENET" DoIP cables are sold specifically for "MQB" vehicles
   (some listings bundle MQB/EVO/MEB/MLB together, imprecise about which
   MQB sub-generation), using pins 1 (activation, +5V once detected) and 8
   (activation-detect, tested via resistance to pin 5/ground) for
   100BASE-T1 Ethernet -- a completely different, richer channel than
   classic CAN diagnostics, used by ODIS for full module programming.
   Still unconfirmed whether *this specific* Typ PJ has it wired, but a
   2-minute multimeter check on pins 1/8 would settle it directly instead
   of relying on inference. If present, this is a real alternative path
   that needs no new CAN hardware -- just a standard USB-to-Ethernet
   adapter on those two pins.
2. Accept that body/comfort modules may simply not be reachable from the
   OBD-II connector on this platform via any addressing scheme, and that
   this project's anti-theft goals may be better served by the physical
   immobilizer approach already scoped separately (starter-circuit relay)
   rather than continuing to chase CAN-based body-module access.

**UPDATE 2026-09-20 -- a third, independent check adds to the negative
result: passive presence, not just active addressing.** Built
`tools/can_id_triage.py` plus a persistent `signal_dictionary.json` (see
that file for the full known-ID list) to check something neither UDS
sweep tests: whether these modules broadcast *anything unprompted*, since
direct addressing being blocked doesn't necessarily mean a module never
transmits at all. Ran the fixed-ID presence check against **every real
CAN capture ever taken on this car** -- all 5 sessions
(2026-09-13 through 2026-09-15), 209,958 frames, ~35 minutes of combined
recording time across baseline/lock/unlock/door/headlight/high-beam/
ignition/normal-drive actions and the two dedicated address-discovery
sessions. **None of the 8 candidate body-module request or response IDs
(`0x70D`/`0x777`, `0x71E`/`0x788`, `0x74A`/`0x7B4`, `0x74B`/`0x7B5`,
`0x73E`/`0x7A8`, `0x73F`/`0x7A9`, `0x754`/`0x7BE`, `0x730`/`0x79A`)
appear even once as a sender, in either direction, anywhere in that
entire dataset.** Only two unrelated genuine unknowns exist on this bus
at all (`0x17F00010`, `0x1BFC8202` -- both already known/undecoded
background broadcasts, see the signal dictionary), plus the Gateway's own
`0x77A` response id (added to the dictionary this session, previously
only `0x710` itself was tracked).

This strengthens direction 2 above from "two addressing schemes found
nothing" to "these modules are never observed transmitting on this bus
segment under any circumstance this project has tested" -- real driving,
every triggered action, two full active address sweeps. Doesn't rule out
direction 1 (DoIP) or a bus segment this OBD-II tap genuinely can't see,
but it's a meaningfully stronger negative than active addressing alone.
Re-run `python tools/can_id_triage.py <captures...> --dict
captures/fabia_2026/signal_dictionary.json --check-ids <ids>` against any
future capture to keep this check current without re-deriving it by hand.

**UPDATE 2026-09-20 -- a real architectural explanation found, upgrading
"unreachable" from a guess to a physically-grounded theory.** VW's own
2010 Polo SSP (ssp444, referenced via
[P1kachu/talking-with-cars](https://github.com/P1kachu/talking-with-cars/blob/master/notes/vw-polo-r6.txt),
same MQB-A0 platform family as this Fabia) documents the Polo's CAN buses
split **by physical bitrate**: Drive/Diagnosis at 500 kbit/s, but
**Comfort/Infotainment at 100 kbit/s** -- a genuinely separate physical
bus segment, since two bitrates can't coexist on one wire pair. Checked
both our boards: P4/MCP2515 (`mcp2515.c` `CNF1/2/3_500KBPS_8MHZ`) and
S3/native TWAI (`bringup_s3.c` `TWAI_TIMING_CONFIG_500KBITS()`) are both
hardcoded to 500 kbit/s only -- neither has ever attempted to listen on a
100 kbit/s bus.

If the Fabia is wired the same way (unconfirmed, but same platform
family), this reframes the whole body-module result: it's very unlikely
the OBD-II connector's CAN_H/CAN_L pins (6/14) expose the Comfort bus at
all -- they most plausibly carry only the Drive/Diagnosis bus. The
Gateway (`0x710`) being the only thing that ever answers isn't
necessarily "gatekeeping" the Comfort bus over the same wire -- it's more
likely the only node with a physical link to *both* buses, bridging
between them only via UDS routing (session/security-gated), never raw
frame forwarding. That would explain both negative results at once: zero
UDS responses (routing blocked) AND zero passive broadcasts (wrong bus
entirely, not just wrong addressing).

**CORRECTION, same day, a bit later: the "we need a second physical CAN
interface" practical implication above is wrong.** Checked how real
diagnostic tools actually work: Ross-Tech's VCDS (the standard VAG tool)
and OEM VAS/ODIS tools all connect through the single standard OBD-II
connector -- no second interface, no extra wiring. This works because the
Central Gateway architecture is specifically designed so **one diagnostic
connection can reach every internal bus segment**, including Comfort --
the Gateway does the CAN-to-CAN routing internally; the tool just sends a
normally-addressed UDS request and the Gateway forwards it to whichever
bus segment the target module lives on. A second physical tap is not how
any real tool does this, professional or otherwise.

So the bitrate-segmentation finding above is still very likely correct as
an architectural fact (Comfort probably is a separate 100 kbit/s bus,
unreachable by direct wire from OBD-II) -- but the conclusion to draw
from it is different: **reaching those modules is a routing/protocol
problem to solve on the existing single interface, not a hardware
problem.** Our UDS requests to the body-module addresses aren't
triggering the Gateway's routing, for some reason a real tool's requests
presumably would. Per the session's own already-tried list above,
`0x10 0x03` session-control-first didn't help (6/6 still timed out on
2026-09-15) and SFD was ruled out -- so this remains an open question,
not a newly solved one. Doesn't change the ranked candidate list from
the "Reframed question" section above (wrong CAN IDs for this specific
model/year is still the most likely explanation, then a transport-format
mismatch); it just rules out "we're on the wrong physical bus and need
new hardware" as the fix. Corrected before it became the basis for
buying/wiring anything.

## Deep dive: the J533 Gateway module, 2026-09-20

Requested follow-up: research the actual Gateway module itself, not just
its behavior over UDS. Real, useful findings:

**The Gateway is a real, physically locatable module: J533, in the
driver's footwell.** Per the [openpilot Volkswagen
wiki](https://github.com/commaai/openpilot/wiki/Volkswagen): "Most VAG
cars use the J533 Harness which connects at the gateway in the drivers
footwell." Its documented job (per the same source, echoing our own
`0x710` findings) is "the exchange of data between the CAN databus
systems ('powertrain CAN databus,' 'convenience CAN databus' and
'infotainment CAN databus') and the conversion of diagnostic data from
CAN databus systems" -- i.e. bridging exactly the three buses relevant
here, powertrain (where our OBD data comes from), convenience/comfort
(where lock/door/light modules almost certainly live), and infotainment.

**How a mature real-world project actually reaches the Comfort bus**:
openpilot (steering/ACC control, needs the Comfort/"extended" CAN bus for
LKAS-relevant signals) does **not** get there through clever OBD-II UDS
routing. It physically splices into the **J533's own connector**,
in-line, using a purpose-built harness (red plug = outbound/CAN0, black
socket = inbound/CAN2 in their terminology) -- see
[hardybm/comma-J533-harness](https://github.com/hardybm/comma-J533-harness)
for a real DIY build (wiring diagram images in that repo, not yet
reviewed here -- worth a visual check against our own Gateway's actual
connector when next at the car). This is a real, working, physical
second-tap approach -- but at a **known, specific, physically locatable
point** (the Gateway's own connector) rather than blind wire-hunting
behind a door panel as originally framed.

**A real caveat that turned out not to apply to us**: the same wiki notes
"a few MQB-A0 cars ... don't have a gateway and have to use the camera
harness" instead (OBD2-port-based, no J533 to splice into, no
experimental longitudinal control support in openpilot's case). This
doesn't apply here -- **we've already directly proven this Fabia has a
real Gateway module** (`0x710` responds to UDS and self-identifies as
"GW"/supplier "Har"/part "3Q0"), so the standard J533-splice path is the
relevant one, not the camera-harness exception.

**Where this leaves things**: a second physical tap at the J533's own
connector is a real, precedented, working method other serious projects
use for exactly this kind of access -- meaningfully more concrete than
"tap the Comfort bus somewhere in the car" was before. Still requires:
locating the Gateway physically (driver's footwell, per this source --
worth confirming against the actual car), and ideally comparing its real
connector against the wiring diagrams in the linked harness repos before
attempting a splice, since none of this has been visually verified
against a 2026 Fabia specifically yet.
