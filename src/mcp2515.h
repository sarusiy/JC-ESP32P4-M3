#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "hal/spi_types.h"

/* Minimal MCP2515 (SPI CAN controller) driver, standard 11-bit IDs only.
 * Fixed at 500 kbps assuming an 8 MHz crystal on the MCP2515 module. */

typedef struct {
    spi_host_device_t spi_host;
    int sck_gpio;
    int mosi_gpio;
    int miso_gpio;
    int cs_gpio;
} mcp2515_config_t;

/* Resets the chip, configures 500 kbps @ 8 MHz, receive-all filters, normal mode. */
esp_err_t mcp2515_init(const mcp2515_config_t *config);

/* Non-blocking poll. Returns true and fills outputs if a frame was pending. */
bool mcp2515_receive(uint32_t *id, uint8_t *dlc, uint8_t *data);

/* Sends a standard-ID frame via TXB0 (fire-and-forget, no confirmation wait). */
esp_err_t mcp2515_send(uint32_t id, uint8_t dlc, const uint8_t *data);
