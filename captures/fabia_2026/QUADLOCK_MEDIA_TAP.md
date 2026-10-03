# Quadlock media-connector tap (future, not yet built)

Status as of 2026-09-24: planning only. No hardware purchased or wired
yet.

## Purpose and plan

Buy an off-the-shelf reverse-camera interface kit that includes an
adapter cable plugging into the Fabia's existing Quadlock media
connector (the factory head-unit harness connector), rather than
building a custom tap from scratch. From that adapter, pull off:
**12V, ground, CAN, front-left/right speaker pairs, and possibly more**
-- all without cutting into or otherwise damaging the car's original
Quadlock connector itself (the adapter sits inline; the factory
connector stays intact and reversible).

**Speakers**: routed through a **dual relay** so the front speakers can
be switched between the factory head unit and this system, preventing
both from driving the same speaker line simultaneously (a real risk if
both sources are ever connected at once -- driving a speaker from two
amplifier outputs at the same time can damage one or both).

**CAN**: the S3 board listens passively on whatever bus this connector
exposes, watching for indications relevant to the project (candidates:
door state, ignition state, steering-wheel input -- see the unverified
claims below).

## Anti-theft deterrent: goal and design (2026-09-24)

**Actual goal, clarified**: make a thief who's already inside the car
not want to stay there -- an in-cabin deterrent, not an outdoor alarm.
This reframes what "loud enough" means: the target is someone sitting
right next to the speakers in a small, sealed, acoustically-reflective
space, not someone standing a meter away outdoors. Car cabins have a
real, significant "cabin gain" effect at close range, so a genuinely
unpleasant, non-musical synthetic tone (not just high volume) through
the front speakers is a realistic goal even at modest amplifier power
(see the MAX98357A discussion below) -- unlike an outdoor-siren-level
deterrent, which would need far more power/a dedicated horn to achieve.

**Horn ("צפצוף") -- PAUSED pending legal check, not an active part of
the plan.** Two considerations, both real:
- Technically plausible via CAN on modern vehicles in principle, but no
  verified source found for a specific horn-trigger command on this car
  -- would need the same rigorous verification discipline already
  applied to every other CAN-ID claim in this doc (see Part 2 below),
  and even if found, likely gated behind session/security access like
  other body functions already blocked in this project's research.
- **Legal concern raised by the user, treated as valid and unresolved**:
  horn usage is specifically regulated as a traffic-safety-signaling
  device in many jurisdictions, separate from general vehicle
  modification law -- using it as a general alarm/deterrent may not be
  legal. Not a question this project can answer (not a legal source) --
  **check local regulation before building this specific feature.**
  Until then, this stays out of the active plan. The in-cabin
  speaker-noise deterrent below doesn't carry the same specific
  regulatory shadow (it's the car's own audio speakers, not the
  regulated horn), so it remains the active plan.

## Part 1: proposed 12-pin interface pinout (user's own wiring plan)

This is the actual physical plan for the custom interface box between
the Quadlock adapter and the S3 -- real wiring intent, not sourced from
anywhere external, kept here for reference during the build:

| Interface pin | Signal | Quadlock source |
|---|---|---|
| 1 | 12V Constant (B+) | Quadlock pin 18 (red/yellow) |
| 2 | Ground (GND) | Quadlock pin 12 (brown) |
| 3 | CAN High (Infotainment) | Quadlock pin 9 (orange/violet) |
| 4 | CAN Low (Infotainment) | Quadlock pin 10 (orange/brown) |
| 5 | Radio Left Front (+) | Quadlock pin 4, media side (cut) |
| 6 | Speaker Left Front (+) | Quadlock pin 4, door side (cut) |
| 7 | Radio Left Front (-) | Quadlock pin 8, media side (cut) |
| 8 | Speaker Left Front (-) | Quadlock pin 8, door side (cut) |
| 9 | Radio Right Front (+) | Quadlock pin 3, media side (cut) |
| 10 | Speaker Right Front (+) | Quadlock pin 3, door side (cut) |
| 11 | Radio Right Front (-) | Quadlock pin 7, media side (cut) |
| 12 | Speaker Right Front (-) | Quadlock pin 7, door side (cut) |

Pins 5-12 (the speaker pairs) are described as "cut" rather than
"tapped" -- these need the dual relay mentioned above to actually be
non-destructive/reversible in practice (a straight cut breaks factory
audio permanently unless the relay routes it back through when this
system isn't overriding it).

**Confirmed 2026-09-24: no switched (ignition-on-only) voltage pin at
this connector** -- only constant B+ and ground per the table above. The
ignition-sense line planned for the relay-remote safety interlock (see
[RELAY_REMOTE_CONTROL.md](../../../JC-ESP32S3-CAN/RELAY_REMOTE_CONTROL.md))
cannot be sourced from this tap and needs a different physical source
(e.g. an ignition-switched fuse in the fuse box, or directly off the
ignition switch) -- **or**, if `0x2C3` ("Ignition Status / ACC State",
see Part 2 below) turns out to be real once verified, that could serve
as a CAN-based ignition indication instead of a dedicated voltage wire.
Both remain open; neither is confirmed yet.

## Part 1b: real, sourced Quadlock pinout (2026-09-24) -- corrects
## several pin numbers from Part 1 above

The actual head unit was identified from a real photo of its label:
**"Car Radio Infotainment-System", model `MIB3 EI GP MQBC`, part
`6570358 69A`, made by JOYNEXT GmbH (Dresden, Germany)**. MIB3 is a real,
documented VW Group infotainment generation (not a guess) -- searched for
its actual Quadlock pinout and found a real source:
[Volkswagen (2015-2020) MIB STD Navigation Head Unit pinout, PinoutGuide.com](https://pinoutguide.com/Car-Stereo-Volkswagen-Porsche/Volkswagen_2015-2020_MIB__pinout.shtml).
The connector is split into 4 lettered blocks (A/B/C/D -- this is what
"Quadlock" refers to), not one flat pin range:

| Block | Pin | Function |
|---|---|---|
| A | 1 | Speaker (+), rear right |
| A | 2 | Speaker (+), front right |
| A | 3 | Speaker (+), front left |
| A | 4 | Speaker (+), rear left |
| A | 5 | Speaker (-), rear right |
| A | 6 | Speaker (-), front right |
| A | 7 | Speaker (-), front left |
| A | 8 | Speaker (-), rear left |
| A | 17 | Ground (Terminal 31) |
| A | 18 | 12V constant (Terminal 30) |
| B | 1 | Telephone microphone |
| B | 6 | **Reversing camera input** |
| B | 7 | Microphone shield |
| B | 12 | Reversing camera shield |
| C | 1 | NF-In earth (low-frequency audio ground) |
| C | 3 | USB +5V |
| C | 4 | USB earth |
| **D** | **6** | **CAN High (Infotainment)** |
| **D** | **12** | **CAN Low (Infotainment)** |

Source page only documents these specific functional pins, not every
numbered position in each 12-pin block (B/C/D) or all 52 positions in
block A -- treat gaps as "not yet documented," not "not present."

**Real, concrete corrections to Part 1's AI-generated claim**: ground is
pin **A17**, not pin 12 as claimed there; CAN High/Low are on a
**separate block (D)**, pins **6/12**, not the same block as power at
pins 9/10. This is exactly the kind of discrepancy the "treat as
unverified" warning in Part 2 was guarding against -- now caught with a
real source instead of just suspicion.

**Block B pin 6 (reversing camera input) is directly useful** -- this is
the actual documented factory video-input pin for a reverse camera,
independent of anything CAN-related. Worth using directly for the camera
video feed itself, separate from the CAN-tap discussion in Part 2.

**Not yet resolved: which physical color block is A/B/C/D.** The source
above doesn't state connector colors, and a separate general VW Quadlock
color reference (older MIB II generation, not confirmed to carry over to
this MIB3 unit) mentions blue/green/grey/black/brown -- not a confirmed
match to what's visible in the real photos (white, orange, green, blue,
plus two small pink connectors). **Safe way to identify block A
yourself**: with the ignition off, multimeter in DC voltage mode, the
block showing a **steady ~12V between two pins regardless of ignition
state** is block A (pins 17/18 are Terminal 31/30, i.e. always-on ground
and constant battery). Once A is identified, the others can be narrowed
down by pin count (A is the large ~52-pin block; B/C/D are each smaller,
~12-pin blocks) and by function-testing (e.g. checking speaker
continuity against the car's actual door speakers to confirm which
12-pin block is genuinely A vs. checking for CAN traffic with a scope/
analyzer to identify D).

## Part 2: CAN IDs and bus speed -- UNVERIFIED, treat as a starting
## hypothesis, not fact

**This section came from an AI-generated summary with no source cited**,
handed over for me to sanity-check. Recording exactly what was claimed,
then flagging what's actually checkable:

> Network Speed: 500 kbps (MQB Platform Architecture)
>
> Reading from the bus:
> - `0x208` -- Door Status Frame (mechanical lock/microswitch toggles)
> - `0x2C3` -- Ignition Status / ACC State
> - `0x5BF` -- Steering Wheel Button Clicks
>
> Injecting into the bus:
> - `0x621` -- Park Assist Camera Force Command (emulates the PDC
>   module, forces the analog video feed on demand, requires ignition on)
> - `0x662` -- Instrument Cluster Text Injection (overwrites Virtual
>   Cockpit screen lines with custom text)

**A concrete, already-checkable contradiction**: this claims the
Infotainment bus runs at 500 kbps. That directly conflicts with the real,
sourced finding from
[UDS_BODY_MODULE_RESEARCH.md](UDS_BODY_MODULE_RESEARCH.md)'s J533
research (VW's own Polo SSP document, same MQB-A0 platform family):
**Comfort/Infotainment CAN is documented there as 100 kbit/s**, a
genuinely separate physical bus from the 500 kbit/s Drive/Diagnosis bus
our OBD-II tap has always used. Either this new claim is wrong about the
speed, or "Infotainment" here means something different from what the
Polo SSP called "Comfort/Infotainment" -- **don't assume 500 kbps and
wire a 500 kbit/s-only transceiver expecting it to just work**; confirm
the real bus speed once physically tapped (or check for traffic at both
speeds if unsure) before assuming any of the rest of this section.

**Why the specific CAN IDs are treated as unverified, not just
"unconfirmed"**: this session already caught one AI-generated answer
(a claimed BCM address `0x7A2` for door lock control) that turned out to
be directly contradicted by our own real 256-address Deep Scan sweep --
nothing at that address ever answered. That's the same failure pattern
worth guarding against here: plausible-sounding, specific hex values
with no derivation shown are a well-known way these tools fabricate
detail when asked for something this specific and not well-documented
publicly. The "Instrument Cluster Text Injection via a single CAN ID"
claim in particular reads as implausible on its own -- real cluster
text/coding access on MQB is normally gated behind proper UDS
session/security access (see the Gateway-routing research in
`UDS_BODY_MODULE_RESEARCH.md`), not a bare frame injection.

**None of this should be wired to actively transmit (the "injecting"
IDs) until independently verified.** Passive listening is low-risk;
sending `0x621`/`0x662` blind, based on an unsourced claim, on a bus
connected to the actual instrument cluster/PDC module is not.

**UPDATE 2026-09-24: checked against two real, independent GitHub MQB
reverse-engineering projects.**

**The overall plan gets real, independent confirmation**:
[jrjoaoramos/mqbcan](https://github.com/jrjoaoramos/mqbcan) (a dedicated
MQB CAN reverse-engineering project, unrelated to this one) independently
describes exactly this approach -- "the radio Quadlock connector located
in the back side" as a real, known tap point for infotainment-bus CAN,
distinct from the OBD-II-reachable gateway-filtered bus. Confirms the
overall plan is sound, not just a guess. It also states infotainment
communication runs at 500kB/s -- some independent support for that
speed, though phrased as a general statement rather than explicitly
bus-specific, so the discrepancy against the Polo SSP's 100 kbit/s
Comfort/Infotainment figure above still isn't fully resolved either way.
Also mentions the CAN Gateway module at hex address `0x19` -- likely a
Network Management broadcast address (a different message class
entirely from the `0x710` UDS diagnostic address already confirmed on
this car), not a contradiction, but not yet reconciled either.

**The `0x5BF` steering-wheel claim does not hold up**: checked
[adamforbes92/MQBSteeringWheelController](https://github.com/adamforbes92/MQBSteeringWheelController),
a real, dedicated MQB steering-wheel project. It uses its own
self-chosen output ID (`0x1E0`, explicitly configurable, not a
documented factory ID) for its synthesized button broadcasts, and
separately documents a real factory-relevant ID -- **`0x38A`, a GRA
(cruise-control) frame** carrying paddle/cruise button state -- nothing
resembling `0x5BF` anywhere. `0x38A` is a genuinely different, real
candidate worth checking for cruise-control-specific buttons once wired,
but it does not corroborate the original claim; if anything it actively
contradicts it (real project, real ID, different number, different
narrower scope than "all steering wheel button clicks").

**Score after checking**: still 0 of the original 5 claimed IDs
independently confirmed. `0x208`/`0x2C3`/`0x621`/`0x662` remain entirely
unverified with no source found anywhere. `0x5BF` is now actively
contradicted, not just unconfirmed. `0x38A` (GRA/cruise) is a new, real
candidate worth adding to the `--check-ids` list once real captures
exist from this tap.

## Why this connector could matter beyond the reverse camera

If this tap genuinely reaches a different physical bus than the one
OBD-II exposes (plausible, per the bitrate discrepancy above and the
existing J533 multi-bus research), this could be the first real,
physical access point to the Comfort/body bus segment that the ongoing
body-module research (`UDS_BODY_MODULE_RESEARCH.md`) has been blocked
on -- door status and steering-wheel input, if genuinely present here,
would be exactly the kind of body-bus signals that never showed up on
the OBD-II-reachable bus across 209,958 frames of prior captures. Worth
treating this build as a real continuation of that research thread, not
just a reverse-camera side project.

## How to actually verify this once wired

Use the tooling already built for this project, not manual guessing:

1. Capture raw traffic from this tap (same `.csv`/`.log` format the app
   already produces) with the car in a few different real states (doors
   open/closed, ignition on/off, steering wheel buttons pressed).
2. Run `tools/can_id_triage.py --dict captures/fabia_2026/signal_dictionary.json
   --check-ids 0x208,0x2C3,0x5BF,0x621,0x662,0x38A` against it -- confirms
   or denies each claimed ID's presence directly (0x38A added 2026-09-24,
   the real GRA/cruise candidate from adamforbes92/MQBSteeringWheelController,
   independent of the original unsourced list), the same way the
   body-module claims were checked.
3. If `0x208`/`0x2C3`/`0x5BF` do show up, correlate their payload changes
   against the triggered action (door open vs closed, ignition state)
   using the same baseline-diff approach already used elsewhere in this
   project, rather than trusting the claimed byte-level meaning.
4. Once anything is genuinely confirmed, promote it into
   `signal_dictionary.json` with `confidence: confirmed` -- don't add the
   claims above to the dictionary as-is; they haven't been checked yet.

## Connector plan: tapped wires -> interface box (2026-09-24)

Non-destructive tap point is the reverse-camera kit's own Quadlock
breakout adapter (already planned for purchase) -- the factory connector
itself is never cut. From that breakout's flying leads into the
interface box, grouped by signal type rather than one large connector
(keeps power/audio noise away from the CAN pair and the video signal):

**Revised 2026-09-24 -- the first version of this table had a real flaw**:
power, 5V, CAN, and video were all plain 2-pin connectors, some sharing
the same family -- mechanically similar enough to plug into the wrong
socket by mistake. Fixed by making every group **physically impossible**
to cross-mate with any other, via a mix of different connector
families/pitches and deliberately different pin counts.

**Revised again 2026-09-24 -- power and 5V combined into one connector**
(GND shared between them, so it's 12V + 5V + one common GND = 3 pins,
not two separate 2-pin connectors).

**Revised again 2026-09-24 -- dropped C3 (5V) entirely in favor of a
dedicated local 12V->5V converter.** Product selected: "12V/24V to 5V 3A
Power Adapter Converter DC-DC Step Down" (15W, USB-C output variant
available directly from the seller -- no adapter needed), ~EUR3.10,
4.8 stars/564 reviews/5000+ sold. Takes A18 (12V const) as input, powers
the S3 directly over USB-C with far more current headroom (3A) than C3's
unknown/likely-lower USB-peripheral-rated capacity -- directly addresses
this board's documented WiFi-burst brownout sensitivity (see
RELAY_REMOTE_CONTROL.md / HARDWARE_MIGRATION.md). This makes C3 (5V)
unnecessary -- the power connector group below is back to 2 pins
(12V + GND only).

**Revised again 2026-09-24 -- speaker scope clarified and the relay
topology worked out.** Only **2 speakers** (front left/right), switched
via **one DPDT relay module** (single coil, two independent switch
sections -- guarantees both channels switch together, never one on
factory and one on override; simpler than two separate SPDT relays that
would need to stay synchronized). Per channel: relay COM -> the
physical speaker, relay NC (default, unpowered) -> the factory radio tap
(fail-safe: if the relay isn't driven, or fails, the speaker stays on
factory audio), relay NO (only when actively overriding) -> the S3's own
amplified audio output. **The S3 generates the audio itself (via an
amplifier), not a pass-through of the factory signal.**

This means the factory-radio-tap and physical-speaker wiring **stays
entirely on the car side**, between the Quadlock tap and the relay
itself -- it never needs to reach the S3/interface box. What actually
needs to reach the S3 side is much smaller than originally assumed:

- **Relay trigger** (1 signal, direct GPIO -> a relay module. **Product
  selected 2026-09-24**: "1/2/4 Channel 3.3V Low Level Trigger Relay
  Module, 8550 SMD Transistor" -- 2-channel variant, ~EUR2.26,
  4.8 stars/45 reviews/700+ sold. This is **two independent SPDT relays**
  on one board (each with its own In1/In2 trigger pin, standard
  COM/NC/NO screw terminals per channel), not a true single-coil DPDT --
  wire **both trigger inputs to the same GPIO** so both channels always
  switch together, achieving the same synchronized result as a true DPDT
  relay. Explicitly 3.3V-compatible (the 8550 is a PNP transistor sized
  by the manufacturer for exactly this), so no separate optocoupler is
  needed here -- unlike the starter/fuel-pump relay-button drivers and
  the ignition-sense circuit, which needed isolation because the GPIO
  was interfacing with *another* live circuit; here the relay itself
  already provides that isolation between its coil (GPIO side) and its
  switched contacts (audio side).

  **Important firmware note -- "Low Level Trigger" means active-LOW**:
  the relay energizes when the GPIO is driven **LOW (0V)**, not HIGH, the
  reverse of the intuitive assumption. The GPIO must default **HIGH** at
  boot (external pull-up or set explicitly before any other logic runs)
  to keep the relay in its NC/factory-radio state as the safe default --
  getting this backwards would invert the fail-safe behavior. Also worth
  confirming the chosen GPIO doesn't glitch LOW momentarily during the
  S3's own boot/reset sequence.
- **S3's amplified audio out**: Left (+/-), Right (+/-) -- 4 wires

| # | Group | Signals | Pins | Connector | Why it can't be confused with the others |
|---|---|---|---|---|---|
| 1 | Power | A18 (12V const), GND, **1 spare/unused** -- into the 12V->5V converter, not to the S3 directly | 3 | JST-XH (2.54mm) | Different pin count from #2 as well as different family -- kept 3-pin deliberately (user preference, 2026-09-24) for an extra, easy-to-eyeball layer of distinction from the CAN connector, on top of the family difference |
| 2 | CAN | D6 (CAN H), D12 (CAN L) | 2 | JST-PH (2.0mm) | Different pitch/family from XH entirely |
| 3 | Camera video | B6 (video), B12 (shield/GND) | 2 | **RCA connector** (standard composite video) | Not a JST at all; also the technically-correct connector for video signal quality |
| 4 | Speaker relay interface | Relay trigger, S3 audio L(+/-), R(+/-) | 5 | JST-XH (2.54mm) | Same family as #1, but 5 pins -- different from #1 (3-pin), can't be confused |

**Total: 4 distinct connector pairs (1 male + 1 female each) across 4
physically-different connector types** (XH-3pin, PH-2pin, RCA, XH-5pin).

**Current check on the #1 Power connector (2026-09-24)**: the actual
load is on the 12V *input* side of the converter, not its 5V/3A output --
15W / 12V ~= 1.25A, maybe up to ~1.5A accounting for conversion
inefficiency. JST-XH is rated 3A per pin (JST's own spec for the series),
so this is comfortably within margin (~2x headroom). 22AWG wire (already
the general recommendation above) is also rated well above this current
for a short run -- no upgrade needed, though 20AWG would add further
margin if preferred.

**Still open, not yet decided**: which amplifier drives the S3's audio
output (needs I2S or similar digital audio in, speaker-level analog out
-- e.g. something in the MAX98357A family, though that's typically mono
per chip and this needs stereo -- two chips, or a dedicated stereo I2S
class-D amp). Not yet specified or purchased.

**Amplifier finalized and ordered 2026-09-24**: 2x "Snvi MAX98357A I2S
Audio Amplifier Module" (3W Class-D, filterless), ~$3.16 each,
4.9 stars/800+ sold. Board pins (per the actual product photo): `L/RC`,
`BCLK`, `DIN`, `GAIN`, `SD`, `GND`, `V+`.

**Full wiring, both boards:**

| Board pin | Connects to | Shared or per-board |
|---|---|---|
| BCLK | GPIO9 | Shared (both boards) |
| L/RC | GPIO10 | Shared (both boards) |
| DIN | GPIO11 | Shared (both boards) |
| V+ | 5V (from the 12V->5V converter) | Shared |
| GND | GND | Shared |
| GAIN | Not connected (default gain) | Per-board, same on both |
| **SD** | **Left board: 100k-ohm to 3.3V. Right board: 220k-ohm to 3.3V** | **Per-board -- this is what makes each board output a different channel** (per [Adafruit's MAX98357 guide](https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp/pinouts): SD<0.16V=shutdown, 0.16-0.77V=stereo-average/(L+R)/2 [not wanted here], ~100k to 3.3V=Left, ~210k to 3.3V=Right -- 220k is the nearest standard resistor value) |

**Full GPIO plan for this whole subsystem (2026-09-24):**

| Signal | GPIO |
|---|---|
| I2S BCLK | 9 |
| I2S LRCLK (L/RC) | 10 |
| I2S DIN | 11 |
| Speaker relay trigger (both channels, LOW-active) | 12 |

Checked against every other GPIO already committed elsewhere in this
project: CAN=4/5, GPS=6/7, remote-button drivers=15/16, ignition-sense=
17, LED=48 (see RELAY_REMOTE_CONTROL.md) -- no conflicts. Also avoids
strapping pins (0/3/45/46), native USB (19/20), and the N16R8 module's
internal octal-PSRAM pins (33-37).

**Male/female convention**: female (socket, recessed pins) on the
harness/car side, male (exposed pins) on the interface-box side -- keeps
exposed pins away from whatever could be live when disconnected.

**AliExpress search terms** (no specific listings linked -- search and
pick a reputable seller):
- `"JST XH 2.54mm connector kit"` -- covers #1 (3-pin) and #4 (5-pin) in
  one kit
- `"JST PH 2.0mm connector kit"` -- covers #2
- `"RCA panel mount connector male female pair"` -- covers #3
- `"SN-28B crimping tool"` -- covers both JST pitches (not RCA, which is
  usually solder or screw-terminal, not crimped)
- `"22AWG silicone wire"` -- flexible, heat-resistant; different colors
  per signal for identification (e.g. red=12V, black=GND, orange/yellow=
  CAN H/L, white=video)
- Some sellers bundle PH+XH kits with the crimp tool in one listing --
  worth searching `"JST connector kit SN-28B"` to catch those.

## Open items

- [ ] Purchase the reverse-camera interface kit.
- [ ] Purchase the JST-PH/XH connector kits, RCA pair, and crimp tool
      per the plan above (4 distinct connector pairs total).
- [ ] Build the interface box per the pinout above.
- [ ] Build/wire the front-L/R speaker relay switch (2 channels, DPDT or
      2x SPDT, NC=factory radio, NO=S3 amplified audio -- see the
      Speaker relay interface section above).
- [x] Amplifier: 2x Snvi MAX98357A ordered 2026-09-24. Still need to
      solder the SD-pin resistors (100k/220k) once the boards arrive.
- [x] Firmware: I2S tone generator + relay control + `/api/deterrent`
      (on/off) written and building clean in `bringup_s3.c` (2026-09-24).
      GPIOs: BCLK=9, LRCLK=10, DIN=11, relay trigger=12. Not yet
      flashed/tested on real hardware -- amp boards and relay module
      haven't arrived yet.
- [x] Schematic: `hardware/audio_deterrent.kicad_sch`, 0 ERC errors,
      covers both MAX98357A modules, SD resistors, and the relay
      module's control side.
- [ ] Confirm the actual bus speed at this connector before assuming
      500 kbps.
- [ ] Capture real traffic and run the triage script's `--check-ids`
      against the 5 originally claimed IDs plus `0x38A` before trusting
      any of them.
- [ ] Do not transmit `0x621`/`0x662` (or anything else) onto this bus
      until independently verified -- passive listening only until then.
