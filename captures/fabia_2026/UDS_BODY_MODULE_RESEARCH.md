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
