# ESP32 MCP2515/TJA1050 Trial Wiring

This guide applies to the unmodified common MCP2515 + TJA1050 CAN module connected to an ESP32 with 3.3 V GPIO.

## Scope and warning

The board must be powered at 5 V because the TJA1050 transceiver requires a 5 V supply. With the board at 5 V, MCP2515 `SO` is a 5 V output and must not be connected directly to an ESP32 GPIO.

The direct 3.3 V connections from the ESP32 to MCP2515 `CS`, `SI`, and `SCK` described here are a trial-only configuration. They are outside the MCP2515 guaranteed input-high specification at a 5 V supply. Use a 74HCT125 or 74AHCT125 level shifter for a dependable installation.

## Connections

| MCP2515/TJA1050 board pin | Connection |
|---|---|
| `VCC` | Regulated 5 V supply |
| `GND` | 5 V supply ground and ESP32 ground |
| `CS` | ESP32 chip-select GPIO, direct for trial |
| `SI` | ESP32 MOSI GPIO, direct for trial |
| `SCK` | ESP32 SPI clock GPIO, direct for trial |
| `SO` | ESP32 MISO GPIO through the divider below |
| `INT` | Leave unconnected when using MCP2515 polling |
| `CANH` | CAN bus CANH |
| `CANL` | CAN bus CANL |

Keep the ESP32-to-module SPI wiring under 10 cm, preferably 5 cm.

## MISO (`SO`) 5 V-to-3.3 V divider

Install the divider only on the board `SO` output before it reaches the ESP32 MISO GPIO:

```text
Board SO ---- 10k ----+---- ESP32 MISO
                       |
                      15k
                       |
                      GND
```

At the maximum TJA1050/MCP2515 supply of 5.25 V, this gives:

$$V_{MISO} = 5.25 \times \frac{15}{10+15} = 3.15\text{ V}$$

This is safe for an ESP32 3.3 V GPIO input.

Do not connect the board `SO` directly to ESP32 MISO. Do not add 5 V pull-ups to ESP32-driven `CS`, `SI`, or `SCK` lines.

## SPI speed for the trial

1. Start at 100 kHz.
2. After error-free testing, try 250 kHz.
3. Then try 500 kHz.
4. Do not exceed 1 MHz in this direct 3.3 V-to-5 V trial configuration.

## CAN bus requirements

- Connect `CANH` to `CANH` and `CANL` to `CANL`.
- Enable 120 ohm termination only at each physical end of the CAN bus.
- Connect a ground/reference wire between non-isolated CAN nodes.

## Validation

1. With the module powered at 5 V, confirm `SO` is not directly connected to an ESP32 GPIO.
2. Measure the ESP32-side MISO divider output; it must be no higher than 3.3 V.
3. At 100 kHz SPI, reset the ESP32 repeatedly and verify that MCP2515 register reads work each time.
4. Send and receive several thousand CAN frames with no errors before increasing SPI speed.

## Recommended production interface

Use a 74HCT125 or 74AHCT125 powered from 5 V for ESP32 `CS`, `SI`, and `SCK` outputs to the MCP2515. Keep the `SO` divider above, or use a dedicated 5 V-to-3.3 V level shifter. If `INT` is used, shift it down to 3.3 V as well.
