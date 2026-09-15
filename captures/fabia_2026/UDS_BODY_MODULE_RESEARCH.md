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
