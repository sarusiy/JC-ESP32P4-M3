# Fiat 500 (2014) — Supported OBD-II PIDs, decoded

Source: `GET /api/obd` -> `supported_pids` field, captured live from the real
car (engine running, CAN mode Active, 29-bit extended addressing locked).
Raw value: **`be3eb813`**

Mode 01 PID 0x00's response is 4 bytes (A B C D); each bit marks one PID as
supported, MSB-first starting at PID 0x01 (byte A bit 7) through PID 0x20
(byte D bit 0). Decoded below.

## Byte A = 0xBE = `1011 1110` (PIDs 0x01–0x08)

| PID  | Name                          | Supported |
|------|-------------------------------|-----------|
| 0x01 | Monitor status since DTCs cleared | YES |
| 0x02 | Freeze DTC                    | no |
| 0x03 | Fuel system status             | YES |
| 0x04 | Calculated engine load          | YES |
| 0x05 | Engine coolant temperature      | YES |
| 0x06 | Short term fuel trim, Bank 1    | YES |
| 0x07 | Long term fuel trim, Bank 1     | YES |
| 0x08 | Short term fuel trim, Bank 2    | no |

## Byte B = 0x3E = `0011 1110` (PIDs 0x09–0x10)

| PID  | Name                          | Supported |
|------|-------------------------------|-----------|
| 0x09 | Long term fuel trim, Bank 2    | no |
| 0x0A | Fuel pressure                  | no |
| 0x0B | Intake manifold absolute pressure | YES |
| 0x0C | **Engine RPM**                  | YES |
| 0x0D | **Vehicle speed**               | YES |
| 0x0E | Timing advance                  | YES |
| 0x0F | Intake air temperature          | YES |
| 0x10 | MAF air flow rate               | no |

## Byte C = 0xB8 = `1011 1000` (PIDs 0x11–0x18)

| PID  | Name                          | Supported |
|------|-------------------------------|-----------|
| 0x11 | Throttle position               | YES |
| 0x12 | Commanded secondary air status  | no |
| 0x13 | Oxygen sensors present (2 banks)| YES |
| 0x14 | O2 Sensor 1 (Bank 1, Sensor 1)  | YES |
| 0x15 | O2 Sensor 2 (Bank 1, Sensor 2)  | YES |
| 0x16 | O2 Sensor 3                     | no |
| 0x17 | O2 Sensor 4                     | no |
| 0x18 | O2 Sensor 5                     | no |

## Byte D = 0x13 = `0001 0011` (PIDs 0x19–0x20)

| PID  | Name                          | Supported |
|------|-------------------------------|-----------|
| 0x19 | O2 Sensor 6                     | no |
| 0x1A | O2 Sensor 7                     | no |
| 0x1B | O2 Sensor 8                     | no |
| 0x1C | OBD standards this vehicle conforms to | YES |
| 0x1D | Oxygen sensors present (4 banks)| no |
| 0x1E | Auxiliary input status           | no |
| 0x1F | Run time since engine start      | YES |
| 0x20 | PIDs supported [0x21-0x40]       | YES |

## Interpretation

The real Fiat 500 genuinely supports **18 standard PIDs**:
`01, 03, 04, 05, 06, 07, 0B, 0C, 0D, 0E, 0F, 11, 13, 14, 15, 1C, 1F, 20`

This is a coherent, realistic set for a small single-bank gasoline engine:
- **MAP-based, not MAF-based fueling**: `0x0B` (Intake MAP) is supported but
  `0x10` (MAF air flow) is not — consistent with the Fiat 500's small
  1.2L MPI engine, which (like many small NA engines) uses speed-density
  fueling off the manifold pressure sensor rather than a mass airflow meter.
- **Single upstream+downstream O2 sensor pair**: `0x13/0x14/0x15` supported,
  `0x16-0x1B` (sensors 3-8) not — matches a single-bank engine with one
  pre-cat and one post-cat oxygen sensor, no secondary bank.
- Standard diagnostics all present: fuel trims (`0x06/0x07`), engine load
  (`0x04`), coolant temp (`0x05`), RPM/speed/throttle (`0x0C/0x0D/0x11`),
  timing advance (`0x0E`), intake air temp (`0x0F`), runtime since start
  (`0x1F`), and it correctly reports which OBD standard it conforms to
  (`0x1C`) plus that PIDs 0x21-0x40 have their own support bitmask (`0x20`).
- `0x20` being set means there's a **second PID-support page** (PIDs
  0x21-0x40) -- now queried and confirmed, see below.

## Second PID-support page (PIDs 0x21–0x40) — confirmed 2026-09-13

Raw value, captured live from the real car (switch on/ACC, engine not
running): **`8003a011`**

| Byte | Value | Binary      | PIDs covered |
|------|-------|-------------|--------------|
| A    | 0x80  | `1000 0000` | 0x21–0x28 |
| B    | 0x03  | `0000 0011` | 0x29–0x30 |
| C    | 0xA0  | `1010 0000` | 0x31–0x38 |
| D    | 0x11  | `0001 0001` | 0x39–0x40 |

Confirmed supported: `0x21, 0x2F, 0x30, 0x31, 0x33, 0x3C, 0x40`

| PID  | Name                                    |
|------|-----------------------------------------|
| 0x21 | Distance traveled with MIL on           |
| 0x2F | **Fuel Tank Level Input**               |
| 0x30 | Warm-ups since codes cleared            |
| 0x31 | Distance traveled since codes cleared   |
| 0x33 | Absolute Barometric Pressure            |
| 0x3C | Catalyst Temperature, Bank 1 Sensor 1   |
| 0x40 | PIDs supported [0x41-0x60] -- **a third page exists**, not yet queried |

`0x2F` (fuel level) is the standout here -- a genuinely useful value
(single byte, `value * 100 / 255` = percent, same formula as throttle) and
an easy, worthwhile next PID to decode whenever PID work resumes on this
car. `0x33` (barometric pressure) and `0x3C` (catalyst temp) are also
straightforward single/two-byte values. `0x40` means there's a third
support page (`0x41-0x60`) still completely unexplored.

## For comparison: the bench Arduino simulator's mask

Captured live from the same `/api/obd` endpoint while on the bench today:
raw value **`08188000`** decodes to exactly **`05, 0C, 0D, 11`** (coolant,
RPM, speed, throttle) -- precisely the 4 PIDs the simulator firmware
actually implements (see `pids[]` list in `main.c`), nothing more. This is
expected and fine for bench testing, but a reminder that the simulator's
"supported PIDs" response is not a realistic stand-in for what a real
ECU reports -- don't infer anything about real-car PID support from it.

## Capture archive note

Pulled the same day from the phone (`can-captures/` on the device) into
this folder. Of the Learn wizard steps recorded, only `ignition_acc`
(598 KB) actually captured CAN traffic -- `baseline` (both attempts),
`lock`, `unlock`, `horn`, `headlights` (both attempts), `door_open`, and
`door_close` are all empty (header row only). This is consistent with
the already-established finding that this Fiat's CAN bus is silent until
the ignition switch reaches at least ACC -- those steps were apparently
recorded with the switch off, so there was genuinely nothing on the bus
to capture, not a tool failure.
