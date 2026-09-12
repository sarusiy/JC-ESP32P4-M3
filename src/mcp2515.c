#include "mcp2515.h"

#include <string.h>
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "mcp2515";

/* Instructions (MCP2515 datasheet section 12). */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BIT_MODIFY  0x05
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
#define REG_EFLG      0x2D
#define REG_RXB0CTRL  0x60
#define REG_RXB1CTRL  0x70
#define REG_TXB0SIDH  0x31

#define EFLG_RX0OVR   0x40
#define EFLG_RX1OVR   0x80

#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02

#define MODE_CONFIG      0x80
#define MODE_LISTEN_ONLY 0x60
#define MODE_NORMAL      0x00

/* 500 kbps @ 8 MHz crystal (standard MCP2515 timing table). */
#define CNF1_500KBPS_8MHZ 0x00
#define CNF2_500KBPS_8MHZ 0x90
#define CNF3_500KBPS_8MHZ 0x02

/* Bit-bang half-period. The trial wiring doc (direct 3.3V GPIO to a 5V-
 * powered module) documented 1 MHz as a safe cap; this was originally set to
 * 2us (250 kHz, 4x under that cap) purely as an untested starting point, not
 * because 250 kHz was ever shown to be a real ceiling. Tightened to 1us
 * (500 kHz, still 2x margin under the documented-safe cap) once load testing
 * showed the MCP2515's 2 receive buffers overflowing under real-car-scale
 * traffic (~750 fps) -- each bit-banged byte transfer is the dominant per-
 * frame cost, so halving it materially raises the chip-level receive rate. */
#define BITBANG_HALF_PERIOD_US 1

static int s_sck_gpio;
static int s_mosi_gpio;
static int s_miso_gpio;
static int s_cs_gpio;
static SemaphoreHandle_t s_spi_mutex;
static uint32_t s_receive_overflow_count;
static portMUX_TYPE s_overflow_lock = portMUX_INITIALIZER_UNLOCKED;

/* SPI mode 0 (CPOL=0, CPHA=0), MSB first: set MOSI while SCK is low, sample
 * MISO on the rising edge. Avoids linking esp_driver_spi entirely (see
 * mcp2515.h for why). */
static uint8_t spi_transfer_byte(uint8_t out)
{
    uint8_t in = 0;
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(s_mosi_gpio, (out >> bit) & 1);
        ets_delay_us(BITBANG_HALF_PERIOD_US);
        gpio_set_level(s_sck_gpio, 1);
        ets_delay_us(BITBANG_HALF_PERIOD_US);
        in = (uint8_t)((in << 1) | gpio_get_level(s_miso_gpio));
        gpio_set_level(s_sck_gpio, 0);
    }
    return in;
}

static void mcp2515_cmd(const uint8_t *tx, uint8_t *rx, size_t len)
{
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    gpio_set_level(s_cs_gpio, 0);
    ets_delay_us(BITBANG_HALF_PERIOD_US);
    for (size_t i = 0; i < len; i++) {
        uint8_t b = spi_transfer_byte(tx[i]);
        if (rx) {
            rx[i] = b;
        }
    }
    ets_delay_us(BITBANG_HALF_PERIOD_US);
    gpio_set_level(s_cs_gpio, 1);
    xSemaphoreGive(s_spi_mutex);
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

static void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t tx[4] = { MCP_BIT_MODIFY, addr, mask, value };
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
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_spi_mutex = xSemaphoreCreateMutex();
    if (s_spi_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_sck_gpio = config->sck_gpio;
    s_mosi_gpio = config->mosi_gpio;
    s_miso_gpio = config->miso_gpio;
    s_cs_gpio = config->cs_gpio;
    s_receive_overflow_count = 0;

    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << s_sck_gpio) | (1ULL << s_mosi_gpio) | (1ULL << s_cs_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << s_miso_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    gpio_set_level(s_sck_gpio, 0);
    gpio_set_level(s_cs_gpio, 1);

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

    /* Start in listen-only mode by default so the P4 does not transmit on the
     * vehicle bus during discovery or while it is simply being monitored. */
    if (!mcp2515_set_mode(MODE_LISTEN_ONLY)) {
        ESP_LOGE(TAG, "chip did not enter listen-only mode; check wiring/power");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "MCP2515 ready: 500 kbps (8 MHz osc), listen-only mode, bit-banged SPI.");
    return ESP_OK;
}

/* Checking REG_EFLG is a full extra SPI transaction (~100us bit-banged) on
 * top of the actual receive -- fine at frame-scan-rate polling, but a real
 * cost when called once per frame under sustained high traffic (which is
 * most of the time, since overflow is the rare case). Split out so callers
 * can check it periodically (e.g. once per can_echo_task wake-up) instead
 * of once per mcp2515_receive() call. */
void mcp2515_check_overflow(void)
{
    uint8_t overflow_flags = mcp2515_read_reg(REG_EFLG) & (EFLG_RX0OVR | EFLG_RX1OVR);
    if (overflow_flags == 0) {
        return;
    }
    portENTER_CRITICAL(&s_overflow_lock);
    s_receive_overflow_count += (overflow_flags & EFLG_RX0OVR ? 1 : 0) +
                                (overflow_flags & EFLG_RX1OVR ? 1 : 0);
    portEXIT_CRITICAL(&s_overflow_lock);
    mcp2515_bit_modify(REG_EFLG, overflow_flags, 0);
}

bool mcp2515_receive(uint32_t *id, bool *extended, uint8_t *dlc, uint8_t *data)
{
    uint8_t status = mcp2515_read_status();
    bool rxb0_pending = status & 0x01;
    bool rxb1_pending = status & 0x02;
    if (!rxb0_pending && !rxb1_pending) {
        return false;
    }

    /* MCP_READ_RXB0 (0x90) reads RXB0; RXB1 is at +4 (0x94). The datasheet says
     * this instruction auto-clears the associated RXnIF flag once the buffer
     * is fully read, but that only holds if CS is released for long enough
     * before the next transaction starts. With bit-banged SPI polling in a
     * tight loop (no inter-transaction delay) the flag can still read as set
     * on the very next status check, causing the same stale buffer to be
     * read repeatedly. Explicitly clear the flag via Bit Modify below so the
     * receive loop can never get stuck spinning on one buffered frame. */
    bool use_rxb0 = rxb0_pending;
    uint8_t cmd = use_rxb0 ? MCP_READ_RXB0 : (MCP_READ_RXB0 + 4);
    uint8_t tx[14] = {0};
    uint8_t rx[14] = {0};
    tx[0] = cmd;
    mcp2515_cmd(tx, rx, sizeof(tx));
    mcp2515_bit_modify(REG_CANINTF, use_rxb0 ? CANINTF_RX0IF : CANINTF_RX1IF, 0);

    uint8_t sidh = rx[1];
    uint8_t sidl = rx[2];
    uint8_t eid8 = rx[3];
    uint8_t eid0 = rx[4];
    uint8_t dlc_byte = rx[5] & 0x0F;
    if (dlc_byte > 8) {
        dlc_byte = 8;
    }

    /* SIDL bit 3 (IDE) marks an extended frame; RXM=11 (receive-any, set at
     * init) lets both standard and extended frames through unfiltered. */
    bool is_extended = (sidl & 0x08) != 0;
    if (is_extended) {
        *id = ((uint32_t)sidh << 21) | (((uint32_t)(sidl & 0xE0)) << 13) |
              (((uint32_t)(sidl & 0x03)) << 16) | ((uint32_t)eid8 << 8) | eid0;
    } else {
        *id = ((uint32_t)sidh << 3) | (sidl >> 5);
    }
    if (extended != NULL) {
        *extended = is_extended;
    }
    *dlc = dlc_byte;
    memcpy(data, &rx[6], dlc_byte);
    return true;
}

uint32_t mcp2515_get_receive_overflow_count(void)
{
    portENTER_CRITICAL(&s_overflow_lock);
    uint32_t count = s_receive_overflow_count;
    portEXIT_CRITICAL(&s_overflow_lock);
    return count;
}

esp_err_t mcp2515_set_listen_only(bool enabled)
{
    return mcp2515_set_mode(enabled ? MODE_LISTEN_ONLY : MODE_NORMAL)
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_send(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data)
{
    if (dlc > 8) {
        dlc = 8;
    }

    uint8_t sidh, sidl, eid8, eid0;
    if (extended) {
        /* 29-bit id = SID(11 bits):EID(18 bits). EXIDE (SIDL bit 3) marks
         * the frame as extended -- see mcp2515_receive() for the matching
         * reconstruction on the RX side. */
        sidh = (uint8_t)(id >> 21);
        sidl = (uint8_t)(((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03));
        eid8 = (uint8_t)(id >> 8);
        eid0 = (uint8_t)id;
    } else {
        sidh = (uint8_t)(id >> 3);
        sidl = (uint8_t)((id & 0x07) << 5);
        eid8 = 0x00;
        eid0 = 0x00;
    }

    uint8_t tx[1 + 5 + 8] = {0};
    uint8_t idx = 0;
    tx[idx++] = MCP_WRITE;
    tx[idx++] = REG_TXB0SIDH;
    tx[idx++] = sidh;
    tx[idx++] = sidl;
    tx[idx++] = eid8;
    tx[idx++] = eid0;
    tx[idx++] = dlc;
    memcpy(&tx[idx], data, dlc);
    idx += dlc;
    mcp2515_cmd(tx, NULL, idx);

    uint8_t rts = MCP_RTS_TXB0;
    mcp2515_cmd(&rts, NULL, 1);
    return ESP_OK;
}
