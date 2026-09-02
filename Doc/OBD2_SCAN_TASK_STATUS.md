# OBD-II scan-tool task — status (2026-09-01)

## What was asked
Add a task on the P4 that requests every supported piece of car info from the
simulated ECU (on the Arduino, see `ArdunioUsbBridgeToCan` repo) one PID at a
time over the CAN bus, and prints the results.

## What's done
`obd_query_task()` added to `src/main.c`, running as a second FreeRTOS task
alongside the existing `can_echo_task()`:
- Loops over PIDs `0x00` (supported-PIDs bitmask), `0x05` (coolant temp),
  `0x0C` (engine RPM), `0x0D` (vehicle speed), `0x11` (throttle position).
- For each PID: sends an OBD-II mode `0x01` request on `0x7DF`, waits up to
  500ms for the `0x7E8` response, decodes and prints it (or logs a timeout),
  waits 1s, moves to the next PID, and loops forever.
- `can_echo_task()` was refactored to route `0x7E8` responses into a
  `QueueHandle_t` (`s_obd_response_queue`) instead of echoing them, so the
  two tasks don't race over the same MCP2515 RX poll. Phase-1 echo-test
  frames (`0x100`) are still echoed as before.
- `get_errors` on `src/main.c`: no lint issues.

## What's NOT done
- **Not build-verified.** Could not get `idf.py build` to succeed on this
  machine — blocked by an unrelated ESP-IDF/toolchain environment issue (see
  below), not by anything in this new code.
- Not flashed to hardware, not tested against the real Arduino ECU simulator
  on the CAN bus, not committed to git.

## Blocking issue: ESP-IDF toolchain can't compile anything for esp32p4
- `idf.py build` fails at CMake's very first generic "can the compiler
  compile a test program" check (`CMakeTestCCompiler`), before any project
  code is touched:
  ```
  riscv32-esp-elf-gcc.exe: error: '-march=rv32imafc_zicsr_zifencei_xesppie':
  extension 'xesppie' starts with 'x' but is unsupported non-standard extension
  ```
- Root cause: the compiler build that this ESP-IDF's own `tools.json` lists
  as "recommended" (`riscv32-esp-elf esp-15.2.0_20251204`) does not support
  the `xesppie` RISC-V extension that ESP-IDF's default toolchain flags file
  requests for the esp32p4 target's default march string.
- **Ruled out**: stale/beta IDF checkout. This machine's ESP-IDF was
  upgraded from the internal `v6.1-beta1` snapshot (commit `b1d13e9`,
  2026-06-24) to the official released `v6.1` tag (commit `fff9895`,
  2026-08-25) via `git checkout v6.1` + `git submodule update --init
  --recursive`. **Identical failure** on the release tag — same march
  string, same "recommended" toolchain version in `tools.json`. This is a
  genuine upstream packaging inconsistency in this ESP-IDF release for the
  esp32p4 target on Windows, not a version-pinning mistake on this machine.
- Not attempted (out of scope for now): hand-picking/downloading an
  alternate compiler build, patching GCC's target spec, or other deeper
  toolchain surgery. Stopped here per agreed plan to avoid open-ended
  environment debugging.

## How to resume
1. Check if Espressif has since fixed this (newer patch tag/tools.json
   bumping the riscv32-esp-elf version) — `cd C:\esp\v6.1-beta1\esp-idf; git
   fetch --tags origin` then look for a newer `v6.1.x` tag.
2. Or try Espressif's EIM GUI tool (`C:\Users\yossi\.espressif\eim_gui\
   eim-gui-windows-x64-v0.18.0.exe`) to repair/reinstall the toolchain -
   this is interactive/GUI, needs to be run by hand.
3. Or find/restore a working ESP-IDF v5.5 install (what this project was
   originally validated against - repo memory says v5.5, but no v5.5
   checkout exists on this machine anymore as of this session).
4. Once a working build environment exists: `idf.py build`, flash to the
   P4, then power up the Arduino running the OBD-II ECU simulator
   (`ArdunioUsbBridgeToCan`, default `uno` env) on the same CAN bus, and
   watch the P4's serial console for `OBD PID 0x.. -> ...` lines cycling
   through all 5 PIDs every ~1s each.
5. Commit `src/main.c` once verified.

## Reference
Environment investigation details also logged in
`/memories/repo/esp-hosted-crash-fix.md` (agent memory, not in git).
