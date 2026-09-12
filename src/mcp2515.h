#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Minimal MCP2515 (SPI CAN controller) driver, supporting both standard
 * 11-bit and extended 29-bit CAN 2.0B IDs (NOT CAN FD -- classic CAN only,
 * same frame format/bit rate/8-byte payload either way; extended IDs are
 * just a longer identifier field, part of CAN since the 2.0B spec).
 * Fixed at 500 kbps assuming an 8 MHz crystal on the MCP2515 module.
 *
 * SPI is bit-banged over plain GPIO instead of the esp_driver_spi component:
 * linking esp_driver_spi alongside esp_hosted crashes the board before
 * app_main() even runs (see /memories/repo/esp-hosted-crash-fix.md). This
 * driver avoids that entirely, and the trial wiring (direct 3.3V GPIO to a
 * 5V-powered module) is only rated for <=1 MHz anyway. */

typedef struct {
    int sck_gpio;
    int mosi_gpio;
    int miso_gpio;
    int cs_gpio;
} mcp2515_config_t;

/* Resets the chip, configures 500 kbps @ 8 MHz, receive-all filters, and
 * starts in safe listen-only mode so it does not transmit until explicitly
 * switched back to active mode. */
esp_err_t mcp2515_init(const mcp2515_config_t *config);

/* Non-blocking poll. Returns true and fills outputs if a frame was pending.
 * *extended is set true for a 29-bit frame (in which case *id holds the full
 * 29-bit value), false for an 11-bit standard frame. Pass NULL for extended
 * if the caller only ever expects standard frames.
 * Does NOT check/clear the overflow flag itself (see mcp2515_check_overflow)
 * -- that used to happen on every call, which is a real cost under sustained
 * high traffic since overflow is the rare case. Callers should call
 * mcp2515_check_overflow() periodically (e.g. once per poll cycle), not once
 * per received frame. */
bool mcp2515_receive(uint32_t *id, bool *extended, uint8_t *dlc, uint8_t *data);

/* Checks REG_EFLG for RX buffer overflow and clears it if set, updating the
 * counter returned by mcp2515_get_receive_overflow_count(). Call this
 * periodically (once per poll cycle is plenty), not once per frame. */
void mcp2515_check_overflow(void);

/* Number of RX buffers that overflowed since initialization. */
uint32_t mcp2515_get_receive_overflow_count(void);

/* Selects passive listen-only mode or normal active CAN operation. */
esp_err_t mcp2515_set_listen_only(bool enabled);

/* Sends a frame via TXB0 (fire-and-forget, no confirmation wait). Pass
 * extended=true for a 29-bit id (e.g. the Fiat 500's 0x18DB33F1 OBD request),
 * false for a standard 11-bit id (e.g. the usual 0x7DF). */
esp_err_t mcp2515_send(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data);
