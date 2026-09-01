#include "mcp2515.h"

#include <string.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "mcp2515";

/* Instructions (MCP2515 datasheet section 12). */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
#define MCP_READ_STATUS 0xA0
#define MCP_RTS_TXB0    0x81
#define MCP_READ_RXB0   0x90

/* Registers. */
#define REG_CANCTRL   0x0F
#define REG_CANSTAT   0x0E
#define REG_CNF1      0x2A
#define REG_CNF2      0x29
#define REG_CNF3      0x28
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
#define REG_RXB0CTRL  0x60
#define REG_RXB0SIDH  0x61
#define REG_RXB1CTRL  0x70
#define REG_TXB0SIDH  0x31

#define MODE_CONFIG   0x80
#define MODE_NORMAL   0x00

/* 500 kbps @ 8 MHz crystal (standard MCP2515 timing table). */
#define CNF1_500KBPS_8MHZ 0x00
#define CNF2_500KBPS_8MHZ 0x90
#define CNF3_500KBPS_8MHZ 0x02

static spi_device_handle_t s_spi;

static void mcp2515_cmd(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_transmit(s_spi, &t);
}

static uint8_t mcp2515_read_reg(uint8_t addr)
{
    uint8_t tx[3] = { MCP_READ, addr, 0x00 };
    uint8_t rx[3] = {0};
    mcp2515_cmd(tx, rx, sizeof(tx));
    return rx[2];
}

static void mcp2515_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t tx[3] = { MCP_WRITE, addr, value };
    mcp2515_cmd(tx, NULL, sizeof(tx));
}

static uint8_t mcp2515_read_status(void)
{
    uint8_t tx[2] = { MCP_READ_STATUS, 0x00 };
    uint8_t rx[2] = {0};
    mcp2515_cmd(tx, rx, sizeof(tx));
    return rx[1];
}

static bool mcp2515_set_mode(uint8_t mode)
{
    mcp2515_write_reg(REG_CANCTRL, mode);
    for (int i = 0; i < 20; i++) {
        if ((mcp2515_read_reg(REG_CANSTAT) & 0xE0) == mode) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

esp_err_t mcp2515_init(const mcp2515_config_t *config)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = config->sck_gpio,
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = config->miso_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t err = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = config->cs_gpio,
        .queue_size = 1,
    };
    err = spi_bus_add_device(config->spi_host, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t reset_cmd = MCP_RESET;
    mcp2515_cmd(&reset_cmd, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!mcp2515_set_mode(MODE_CONFIG)) {
        ESP_LOGE(TAG, "chip did not enter config mode; check wiring/power");
        return ESP_ERR_TIMEOUT;
    }

    mcp2515_write_reg(REG_CNF1, CNF1_500KBPS_8MHZ);
    mcp2515_write_reg(REG_CNF2, CNF2_500KBPS_8MHZ);
    mcp2515_write_reg(REG_CNF3, CNF3_500KBPS_8MHZ);

    /* RXM=11 (receive any message, no filtering), BUKT=1 (RXB0 overflow rolls into RXB1). */
    mcp2515_write_reg(REG_RXB0CTRL, 0x64);
    mcp2515_write_reg(REG_RXB1CTRL, 0x60);
    mcp2515_write_reg(REG_CANINTE, 0x03); /* RX0IE | RX1IE, unused while polling but harmless. */

    if (!mcp2515_set_mode(MODE_NORMAL)) {
        ESP_LOGE(TAG, "chip did not enter normal mode; check wiring/power");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "MCP2515 ready: 500 kbps (8 MHz osc), normal mode.");
    return ESP_OK;
}

bool mcp2515_receive(uint32_t *id, uint8_t *dlc, uint8_t *data)
{
    uint8_t status = mcp2515_read_status();
    bool rxb0_pending = status & 0x01;
    bool rxb1_pending = status & 0x02;
    if (!rxb0_pending && !rxb1_pending) {
        return false;
    }

    /* MCP_READ_RXB0 (0x90) reads RXB0; RXB1 is at +4 (0x94) and auto-clears its own flag. */
    uint8_t cmd = rxb0_pending ? MCP_READ_RXB0 : (MCP_READ_RXB0 + 4);
    uint8_t tx[14] = {0};
    uint8_t rx[14] = {0};
    tx[0] = cmd;
    mcp2515_cmd(tx, rx, sizeof(tx));

    uint8_t sidh = rx[1];
    uint8_t sidl = rx[2];
    uint8_t dlc_byte = rx[5] & 0x0F;
    if (dlc_byte > 8) {
        dlc_byte = 8;
    }

    *id = ((uint32_t)sidh << 3) | (sidl >> 5);
    *dlc = dlc_byte;
    memcpy(data, &rx[6], dlc_byte);
    return true;
}

esp_err_t mcp2515_send(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    if (dlc > 8) {
        dlc = 8;
    }

    uint8_t tx[1 + 5 + 8] = {0};
    uint8_t idx = 0;
    tx[idx++] = MCP_WRITE;
    tx[idx++] = REG_TXB0SIDH;
    tx[idx++] = (uint8_t)(id >> 3);
    tx[idx++] = (uint8_t)((id & 0x07) << 5);
    tx[idx++] = 0x00; /* EID8, unused (standard frame) */
    tx[idx++] = 0x00; /* EID0, unused (standard frame) */
    tx[idx++] = dlc;
    memcpy(&tx[idx], data, dlc);
    idx += dlc;
    mcp2515_cmd(tx, NULL, idx);

    uint8_t rts = MCP_RTS_TXB0;
    mcp2515_cmd(&rts, NULL, 1);
    return ESP_OK;
}
