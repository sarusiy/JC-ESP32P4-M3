# Skoda Fabia (2026) — Findings, decoded

Captured 2026-09-13, same session as the Fiat 500 work (see
`../fiat500_2014/PID_SUPPORT_ANALYSIS.md`). This car is a fresh
reverse-engineering target -- nothing about the Fiat's addressing, IDs, or
supported PIDs carries over.

## Addressing scheme: standard 11-bit (`CAR_11`)

Confirmed via `GET /api/can/partner` (Control tab's Bus Partner section):
`CAR_11`. Unlike the Fiat's 29-bit extended addressing, this Fabia answers
OBD-II Mode 01 requests on the classic `0x7DF` (request) / `0x7E8-0x7EF`
(response) standard range. Consistent with VAG/MQB generally keeping the
legacy diagnostic functional broadcast on 11-bit even when much of the rest
of the vehicle's internal bus traffic uses 29-bit extended IDs (see the
bus-dominant-ID note below).

## Multi-ECU response: real, but benign -- RESOLVED 2026-09-13 evening

**Original symptom (kept for context):** Engine running, CAN mode Active.
Engine RPM first read a plausible idle value (~980) then froze completely
-- never tracked the accelerator. Root-caused at the time to `0x7E8` and
`0x7E9` both answering the same Mode 01 broadcast, with the firmware only
matching the PID-echo byte, never which ECU actually answered -- so a
secondary module's stale echo could get accepted as readily as the real
engine ECU's answer. Fixed via a responder-lock (`obd_query_task` locks
onto whichever ECU answers first, discards the other).

**Live re-test result, 2026-09-13 evening (Active-mode Learn recording,
two genuine "normal drive" custom actions):** both ECUs answer, exactly
as before -- but decoding their actual payloads across the drive shows
**both report live, correctly-changing values that track each other
within 1-3%**, not one live + one frozen:

| | `0x7E8` | `0x7E9` |
|---|---|---|
| RPM over the drive | 1331 → 1647 → 2030 → 1297 | 1427 → 1619 → 2028 → 1290 |
| Speed over the drive | 9 → 17 → 22 → 21 km/h | 9 → 17 → 23 → 21 km/h |

**Decode detail** (formulas match `main.c`'s `obd_print_response` exactly,
confirmed against these real responses):

RPM (`0x0C`), response `04 41 0C <A> <B>`, value = `(A×256+B)/4`:

| Raw (`A B`) | `0x7E8` RPM | Raw (`A B`) | `0x7E9` RPM |
|---|---|---|---|
| `14 CE` | (20×256+206)/4 = 1331.5 | `16 4C` | (22×256+76)/4 = 1427 |
| `19 BC` | (25×256+188)/4 = 1647 | `19 4C` | (25×256+76)/4 = 1619 |
| `1F B8` | (31×256+184)/4 = 2030 | `1F B0` | (31×256+176)/4 = 2028 |
| `14 46` | (20×256+70)/4 = 1297.5 | `14 28` | (20×256+40)/4 = 1290 |

Throttle (`0x11`), response `03 41 11 <A>`, value = `A×100/255` -- only
ever answered by `0x7E8`, never `0x7E9`:

| Raw (`A`) | Throttle |
|---|---|
| `3B` (59) | 59×100/255 = 23.1% |
| `33` (51) | 51×100/255 = 20.0% |
| `1E` (30) | 30×100/255 = 11.8% |
| `1E` (30) | 30×100/255 = 11.8% |

Throttle peaks (23.1%) right where RPM peaks (2030), then both ease off
together (11.8% / 1297) -- physically coherent accelerate-then-lift
behavior, not noise.

Fully coherent with real acceleration/deceleration. **The original "frozen
after one read" symptom was real, but this raw evidence doesn't show a
collision corrupting data** -- whatever caused that specific frozen
reading, it wasn't two ECUs disagreeing; both agree closely, always. The
responder-lock fix is a reasonable safeguard to keep either way (still
correct behavior even when both sources agree), but the risk it guards
against on this car looks lower-stakes than originally feared.

**Which ECU is which**: decoding both ECUs' own PID-support masks (see
below) shows `0x7E8` has a much richer diagnostic profile (17 PIDs,
closely matching the Fiat's engine-ECU pattern) than `0x7E9` (7 PIDs) --
strong evidence `0x7E8` is the actual engine ECU and `0x7E9` a secondary
module (gateway or similar) that mirrors a handful of the same live
signals rather than a stale/static echo source.

## Throttle Position anomaly -- RESOLVED, was never an anomaly

Originally flagged: a live `Throttle Position` reading appeared despite
the (incomplete) PID-support mask suggesting `0x11` wasn't supported.
Now resolved: `0x11` throttle responses come from `0x7E8` specifically,
consistently, with plausible values that change with speed/RPM across
the drive (23.1% → 20% → 11.8%, easing off as speed peaked). It's a
real, live, correctly-supported PID -- the mask that seemed to say
otherwise was `0x7E9`'s narrower mask, not `0x7E8`'s (see below), so the
"missing" bit was never actually about throttle support at all.

## Supported PIDs (page 1) -- now decoded per-ECU, not just one combined mask

**Correction to the earlier single mask below**: `981a0001` (previously
treated as "the Fabia's PID support") is actually only `0x7E9`'s mask --
the secondary module's, not the engine ECU's. Decoding `0x7E8` and
`0x7E9` separately from the 2026-09-13 evening drive capture:

| ECU | Raw mask | Supported PIDs |
|---|---|---|
| `0x7E8` (likely the real engine ECU) | `BE3EA813` | `01,03,04,05,06,07,0B,0C,0D,0E,0F,11,13,15,1C,1F,20` (17 PIDs) |
| `0x7E9` (secondary/gateway module) | `981A0001` | `01,04,05,0C,0D,0F,20` (7 PIDs) |

`0x7E8`'s profile is nearly identical to the Fiat's engine ECU (same
MAP-based/no-MAF pattern, same fuel-trim and O2-sensor support) -- only
difference found so far is `0x14` (O2 Sensor 1, Bank 1 Sensor 1), which
the Fiat supports and `0x7E8` here does not. `0x7E8`'s page 2+ (PID
0x20 confirms it exists) has not been decoded yet -- worth doing next,
now that we know which ECU's mask is the meaningful one to query.

Original phone-photo caveat below is now superseded but kept for context:
the `981a0001` value itself was correctly read (it matches `0x7E9`'s
mask exactly, confirmed via this session's direct capture), the error
was in treating it as the car's one/only/primary PID-support profile.

## Supported PIDs (page 2) -- confirmed 2026-09-13 evening

Already present in the same two "normal drive" captures used above (PID
`0x20` responses), no new recording needed:

| ECU | Raw mask | Supported PIDs |
|---|---|---|
| `0x7E8` | `8007B011` | `21,2E,2F,30,31,33,34,3C,40` (9 PIDs) |
| `0x7E9` | `00002001` | `33,40` (2 PIDs) |

Named out for `0x7E8`: Distance traveled with MIL on (`0x21`), Commanded
evaporative purge (`0x2E`), **Fuel Tank Level Input** (`0x2F`), Warm-ups
since codes cleared (`0x30`), Distance traveled since codes cleared
(`0x31`), Absolute Barometric Pressure (`0x33`), Wide-range O2 Sensor 1
ratio (`0x34`), Catalyst Temperature Bank 1/Sensor 1 (`0x3C`), and `0x40`
being set means **a third page (0x41-0x60) exists**, not yet queried.

Same pattern as page 1: `0x7E8`'s set is a superset of the Fiat's own
page 2 (`21,2F,30,31,33,3C,40`) -- everything the Fiat has, plus `0x2E`
and `0x34`. `0x7E9` again only carries a couple of the same PIDs `0x7E8`
already covers.

## Bus fingerprint: one CAN ID dominates almost everything -- CORRECTED 2026-09-13 evening

**Original conclusion below (kept for the record) was wrong about the cause.**
`0x17F00010` dominating every capture is real, but it is **not** the
vehicle transmitting fast -- it's our own board's Listen-Only mode failing
to ACK the frame, so the real sender's CAN controller auto-retransmits it
continuously per standard CAN error-recovery behavior (no ACK = transmit
error = automatic retry). Confirmed by directly comparing a Passive-mode
capture against an Active-mode capture of the same resting car (engine
off, doors closed, nothing touched, both taken back-to-back):

| | Passive (Listen-Only, no ACK) | Active (Normal mode, ACKs) |
|---|---|---|
| `0x17F00010` inter-frame period | ~600-750 µs (~730-750 Hz) | **~500,053 µs -- essentially exactly 500ms (2 Hz)** |
| Other traffic visible | none, ever | a second ID, `0x1BFC8202`, appears at a steady ~6.17s period |

That's a ~700-800x rate difference for the *same* message depending only
on whether our board acknowledges it -- conclusive. Once Active mode lets
the real 2Hz message succeed on its first transmission, the bus frees up
enough for a previously-invisible message (`0x1BFC8202`, payload
`8800000D00000000`, ~6.17s period) to show through too. That ID never
appeared in *any* Passive-mode capture across this whole project, on this
car -- it was being crowded out of the MCP2515's 2-deep hardware RX buffer
by the storm.

**Practical implication for future sessions**: Passive/Listen-Only mode
(the default, chosen for being the "safe, non-interfering" option) is
actually the *less* accurate way to observe a real vehicle's bus -- it
measurably distorts both the rate of the dominant message and hides other
traffic entirely. Active mode (which does nothing more invasive than
standard, mandatory CAN-protocol ACKing) gives a truer picture. Prefer
Active mode when the goal is characterizing what's actually on the bus,
not just watching passively.

**Payload for `0x17F00010`** is not fixed across sessions -- `20020000852A05D6`
in the original (2026-09-13 late morning) session's `lock` capture,
`2000000000000780` in the 2026-09-13 evening resting-car session. Given
the message's real rate is 2Hz and it's present even with the car fully
off and doors closed, this is very likely some kind of periodic
status/heartbeat from a comfort/body module that stays minimally active
at rest, not anything tied to a specific action like locking.

### Original (superseded) reasoning, kept for context

Across every capture from this car, **`0x17F00010`** is overwhelmingly the
most common frame -- in the 197k-frame standalone capture it was 195,172
of 196,988 frames (~99.1%). Every short (4,000-6,000 frame) Learn-wizard
action recording (lock, unlock, horn x2, headlights x3, door open/close)
shows **only** this one ID and nothing else. This was originally read as
"very likely a genuine, very-high-frequency real message on this
vehicle" -- see the correction above for why that was wrong.

## Ignition-switch-wake behavior: nuanced, not a clean "asleep until ACC"

The original same-day `baseline` (switch-off) capture was empty -- zero
frames, matching the Fiat's pattern and read at the time as "bus wakes
only once ignition reaches ACC." The 2026-09-13 evening session
complicates that: with the car confirmed fully off and doors closed,
**both `0x17F00010` (2Hz) and `0x1BFC8202` (~6.17s) were present** in
both a Passive and an Active capture. Two ways to reconcile this, not yet
distinguished:
1. The comfort/body segment of the bus (where these two IDs live) stays
   minimally active at rest, while the powertrain/OBD segment (`0x7DF`/
   `0x7E8-0x7EF`) genuinely does stay asleep until ACC -- the original
   baseline being empty could just mean it was too short a window to catch
   a 500ms-period message reliably, not that the bus was truly silent.
2. Something about the vehicle's state differed between the two sessions
   (e.g. a lingering-awake window after recent activity) rather than a
   clean, permanent "asleep at rest" characteristic.
Next time this car is available: a genuinely long (60s+) baseline capture,
immediately after confirming zero interaction for several minutes, would
settle which of these it is.

## VIN read confirmed live on this car (2026-09-13 evening)

First real-hardware confirmation of the Mode 09 PID 0x02 VIN feature:
`TMBER9PJ3T4015474` (a plausible Skoda VIN -- `TMB` is Skoda Auto's real
WMI prefix), read via the Control tab's "Read VIN" button while CAN mode
was Active. Also re-confirmed live via `/api/obd`: `Partner: CAR_11` and
supported-PIDs page 1 `981a0001` -- the same value previously only read
off a phone photo (see the now-outdated caveat above), so that page-1
mask can be treated as confirmed, not provisional, going forward.

**Resolved** (see the multi-ECU section above, from a later capture the
same evening): RPM/Speed/Throttle are genuinely live, not frozen --
confirmed via two dedicated "normal drive" custom Learn recordings that
show values tracking real acceleration and deceleration throughout.

## What this means for finding the real lock/unlock/etc. signals

**Update 2026-09-13 evening**: `LearnFragment` no longer hardcodes Passive
mode (now a checkbox, defaulting to Active) -- lock, unlock, door_open,
and door_close were all re-recorded in Active mode this same evening.
Result: **still only `0x17F00010`, no new candidate ID found for any of
them**, even with the storm gone and the buffer no longer swamped. That's
a real (if negative) result, not an inconclusive one -- it argues against
"we just couldn't see it through the noise" and toward one of:
1. **These actions genuinely don't produce OBD-port-visible CAN traffic**
   on this car -- plausible if central locking lives entirely on a
   comfort/body bus segment this particular OBD-II tap doesn't bridge to
   (see the UDS/comfort-CAN discussion in `../PID_COMPARISON_SUMMARY.md`).
2. **The recordings are still too short.** Each was only ~6-16 rows this
   round -- comparable in wall-clock time to before, just far less
   *cluttered* per second. If the real signal is rare/low-rate rather
   than merely swamped, a short window can still miss it by bad luck.
   Worth trying meaningfully longer (60s+) captures for these specific
   actions next time, now that Active mode makes a longer capture
   actually usable (no longer guaranteed to be 99% one ID).
3. **Filter `0x17F00010` (and now `0x1BFC8202`) out** during analysis --
   both are now known, presumed-irrelevant IDs -- so whatever else is
   present becomes easier to spot even in the existing short captures.

## Capture archive

All 17 Learn-wizard files plus the one genuine standalone Fabia capture are
kept in this folder tree (`fabia_2026/` and one level up). Two other
standalone Record-tab files from this same day (`can-20260913-110155.csv`,
`can-20260913-111611.csv`) turned out to actually be Fiat data (matched by
timestamp and CAN ID fingerprint, not Fabia) -- kept alongside the Fiat
captures conceptually, physically still sitting in the flat `captures/`
root rather than moved into `fiat500_2014/`.

Two more sessions from the same day, kept in their own subfolders:
- `20260913_session2/` -- 22 files from a longer field session (Learn
  wizard + one standalone capture), all CAN-mode-unlabeled since this
  predates the app change that stamps mode into each CSV. Notable file:
  `learn-fabia_2026-other_1_car_fully_off_during_passive_mode-*.csv`,
  the first deliberate "car confirmed fully off" Passive capture.
- `20260913_session3/` -- 4 files from the evening Active-vs-Passive
  comparison test (the one that resolved the bus-fingerprint correction
  above). Each row is tagged `active` or `passive` in its own `can_mode`
  column (added specifically to make this comparison possible going
  forward without relying on memory of which mode was used when).
- `20260913_session4_active/` -- 14 files, the first full Learn-wizard
  session recorded with the new Active-mode default (baseline, lock,
  unlock, door_open/close, headlights, ignition_acc, plus two custom
  "normal_drive" recordings). This session resolved the multi-ECU and
  throttle-anomaly questions above and revealed the per-ECU PID masks.
