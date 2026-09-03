#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Minimal MCP2515 (SPI CAN controller) driver, standard 11-bit IDs only.
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

/* Resets the chip, configures 500 kbps @ 8 MHz, receive-all filters, normal mode. */
esp_err_t mcp2515_init(const mcp2515_config_t *config);

/* Non-blocking poll. Returns true and fills outputs if a frame was pending. */
bool mcp2515_receive(uint32_t *id, uint8_t *dlc, uint8_t *data);

/* Number of RX buffers that overflowed since initialization. */
uint32_t mcp2515_get_receive_overflow_count(void);

/* Selects passive listen-only mode or normal active CAN operation. */
esp_err_t mcp2515_set_listen_only(bool enabled);

/* Sends a standard-ID frame via TXB0 (fire-and-forget, no confirmation wait). */
esp_err_t mcp2515_send(uint32_t id, uint8_t dlc, const uint8_t *data);
