#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_hosted.h"
#include "esp_freertos_hooks.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "nvs_flash.h"
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#endif
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "mcp2515.h"

#ifndef LED_GPIO
#define LED_GPIO 28
#endif

/* MCP2515 CAN module wiring on header JP1 (see Doc/electrical drawing.png). */
#ifndef CAN_CS_GPIO
#define CAN_CS_GPIO 49
#endif
#ifndef CAN_SCK_GPIO
#define CAN_SCK_GPIO 50
#endif
#ifndef CAN_MOSI_GPIO
#define CAN_MOSI_GPIO 51
#endif
#ifndef CAN_MISO_GPIO
#define CAN_MISO_GPIO 52
#endif
#ifndef CAN_INT_GPIO
#define CAN_INT_GPIO 29
#endif
#define CAN_TEST_ID 0x100

/* Simulated GPS UART wiring on JP1 (see Doc/GPS_SIMULATION_AND_INTEGRATION_REQUIREMENTS.md).
 * GPIO34/35 are free on the same header as the MCP2515 wiring above. The
 * Arduino simulator's SoftwareSerial TX is 5V logic and is stepped down to
 * GPIO34 with a 10k/15k divider, matching the CAN SO/INT dividers. */
#ifndef GPS_UART_RX_GPIO
#define GPS_UART_RX_GPIO 34
#endif
#ifndef GPS_UART_TX_GPIO
#define GPS_UART_TX_GPIO 35
#endif
#define GPS_UART_NUM UART_NUM_1
#define GPS_UART_BAUD 9600

/* Quarter-dBm units: 40 = 10 dBm. Reduces Wi-Fi current spikes on USB-powered bench setups. */
#define WIFI_MAX_TX_POWER_QDBM 40

/* OBD-II (SAE J1979) scan-tool request/response IDs, matching the simulated
 * ECU in the ArdunioUsbBridgeToCan repo. */
#define OBD_REQUEST_ID 0x7DF
#define OBD_RESPONSE_ID 0x7E8
#define OBD_MODE_CURRENT_DATA 0x01
#define OBD_QUERY_INTERVAL_MS 200
#define OBD_RESPONSE_TIMEOUT_MS 500

#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

/* Half-period of the blink, in ms; changed live from the control channel. */
static volatile uint32_t blink_half_period_ms = 500;
static bool wifi_started;
static httpd_handle_t http_server;

static const char *TAG = "main";
/* Runtime log verbosity, changed live via the "ll" control-channel command. */
static esp_log_level_t s_log_level = ESP_LOG_INFO;

typedef struct {
    uint32_t supported_pids;
    int coolant_c;
    uint16_t rpm;
    uint8_t speed_kmh;
    uint8_t throttle_pct;
} obd_state_t;

static obd_state_t s_obd_state;

typedef struct {
    uint32_t idle0_pct;
    uint32_t idle1_pct;
    uint32_t idle0_delta;
    uint32_t idle1_delta;
    uint32_t idle0_max_delta;
    uint32_t idle1_max_delta;
    size_t heap_free;
    size_t heap_min_free;
    size_t internal_free;
    size_t internal_min_free;
    size_t psram_free;
    size_t psram_min_free;
    uint32_t flash_size;
    uint32_t flash_partitioned;
    uint32_t app_partition_size;
    uint32_t uptime_s;
    esp_reset_reason_t restart_reason;
} health_state_t;

static health_state_t s_health_state;
static portMUX_TYPE s_health_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_idle0_count;
static volatile uint32_t s_idle1_count;

typedef struct {
    bool fix_valid;
    double lat;
    double lon;
    float speed_kmh;
    float heading_deg;
    char utc_time[8];
    char utc_date[8];
    uint8_t satellites;
} gps_state_t;

static gps_state_t s_gps_state;
static portMUX_TYPE s_gps_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_last_gps_log_us;

typedef struct {
    uint8_t dlc;
    uint8_t data[8];
} obd_frame_t;

typedef struct {
    uint64_t sequence;
    int64_t timestamp_us;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_capture_frame_t;

#define CAN_CAPTURE_CAPACITY 256
#define CAN_CAPTURE_HTTP_BATCH 32
#define CAN_CAPTURE_RESPONSE_SIZE 8192

static can_capture_frame_t s_can_capture[CAN_CAPTURE_CAPACITY];
static uint64_t s_can_capture_sequence;
static portMUX_TYPE s_can_capture_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_can_rx_task;
static volatile bool s_can_passive;

/* Filled by can_echo_task whenever it sees an 0x7E8 response, drained by obd_query_task. */
static QueueHandle_t s_obd_response_queue;

static void capture_can_frame(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    int64_t timestamp_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_can_capture_lock);
    uint64_t sequence = ++s_can_capture_sequence;
    can_capture_frame_t *frame = &s_can_capture[(sequence - 1) % CAN_CAPTURE_CAPACITY];
    frame->sequence = sequence;
    frame->timestamp_us = timestamp_us;
    frame->id = id;
    frame->dlc = dlc;
    memcpy(frame->data, data, dlc);
    portEXIT_CRITICAL(&s_can_capture_lock);
}

/* Counters used to verify GPIO29 wiring: isr_count only increments on a real
 * falling edge; timeout_count increments when the 100ms fallback fires instead. */
static volatile uint32_t s_can_isr_count;
static volatile uint32_t s_can_timeout_count;

static void IRAM_ATTR can_int_isr(void *arg)
{
    (void)arg;
    s_can_isr_count++;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_can_rx_task != NULL) {
        vTaskNotifyGiveFromISR(s_can_rx_task, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
static void ble_publish_response(uint16_t conn_handle, const char *message);
#endif

/* Shared command parser used by USB CDC and future companion transport. */
static bool apply_freq_command(const char *line, char *out_msg, size_t out_msg_len)
{
    if (strncmp(line, "freq ", 5) != 0) {
        snprintf(out_msg, out_msg_len, "ERR unknown command '%s'. Type 'help'.\r\n", line);
        return false;
    }

    char *endptr = NULL;
    long value = strtol(line + 5, &endptr, 10);
    if (endptr == line + 5 || *endptr != '\0' ||
        value < BLINK_HALF_PERIOD_MIN_MS || value > BLINK_HALF_PERIOD_MAX_MS) {
        snprintf(out_msg, out_msg_len, "ERR invalid value. Enter a number of ms (%d-%d).\r\n",
                 BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS);
        return false;
    }

    blink_half_period_ms = (uint32_t)value;
    printf("Blink half-period set to %ld ms\n", value);
    snprintf(out_msg, out_msg_len, "OK freq=%ld ms\r\n", value);
    return true;
}

static const char *log_level_to_string(esp_log_level_t level)
{
    switch (level) {
        case ESP_LOG_ERROR: return "ERROR";
        case ESP_LOG_WARN:  return "WARNING";
        case ESP_LOG_INFO:  return "INFO";
        case ESP_LOG_DEBUG: return "DEBUG";
        default:            return "UNKNOWN";
    }
}

/* Levels are numbered 0-3 for the "ll" command: 0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR. */
static bool log_level_from_number(const char *text, esp_log_level_t *out_level)
{
    static const esp_log_level_t levels[] = { ESP_LOG_DEBUG, ESP_LOG_INFO, ESP_LOG_WARN, ESP_LOG_ERROR };

    char *endptr = NULL;
    long value = strtol(text, &endptr, 10);
    if (endptr == text || *endptr != '\0' || value < 0 || value >= (long)(sizeof(levels) / sizeof(levels[0]))) {
        return false;
    }

    *out_level = levels[value];
    return true;
}

/* Shared command parser: "ll <0-3>" (0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR), controls our own TAG's verbosity. */
static bool apply_ll_command(const char *line, char *out_msg, size_t out_msg_len)
{
    esp_log_level_t level;
    if (strncmp(line, "ll ", 3) != 0 || !log_level_from_number(line + 3, &level)) {
        snprintf(out_msg, out_msg_len, "ERR usage: ll <0-3> (0=DEBUG 1=INFO 2=WARNING 3=ERROR)\r\n");
        return false;
    }

    s_log_level = level;
    esp_log_level_set(TAG, level);
    snprintf(out_msg, out_msg_len, "OK ll=%s\r\n", log_level_to_string(level));
    return true;
}

static void init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

/* Phase 1 echo test frames get echoed back; OBD-II responses get routed to
 * obd_query_task via a queue instead of being echoed. Single task owns the
 * MCP2515 RX poll so the two consumers never race for the same frame. */
static void can_echo_task(void *arg)
{
    (void)arg;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) {
            s_can_timeout_count++;
        }
        /* Drain both hardware RX buffers before sleeping; otherwise a second
         * frame arriving while the first is still pending gets left behind
         * and can overflow/reorder under back-to-back multi-frame traffic. */
        while (mcp2515_receive(&id, &dlc, data)) {
            capture_can_frame(id, dlc, data);
            if (id == OBD_RESPONSE_ID) {
                if (!s_can_passive) {
                    obd_frame_t frame = { .dlc = dlc };
                    memcpy(frame.data, data, sizeof(frame.data));
                    if (xQueueSend(s_obd_response_queue, &frame, 0) != pdPASS) {
                        ESP_LOGW(TAG, "OBD response queue full; dropping response");
                    }
                }
                continue;
            }
            if (!s_can_passive && id == CAN_TEST_ID) {
                printf("CAN RX id=0x%03lx dlc=%d data='%.*s' -> echoing\n",
                       (unsigned long)id, dlc, dlc, data);
                mcp2515_send(id, dlc, data);
            }
        }
    }
}

/* Decodes and prints one PID's response payload (buf[0]=len, buf[1]=mode+0x40,
 * buf[2]=pid, buf[3..]=data), per the SAE J1979 formulas for each PID. */
static void obd_print_response(uint8_t pid, const uint8_t *buf, uint8_t len)
{
    /* buf[0]=len, buf[1]=mode+0x40, buf[2]=pid echo, buf[3..]=actual parameter bytes. */
    switch (pid) {
        case 0x00:
            if (len >= 7) {
                s_obd_state.supported_pids = ((uint32_t)buf[3] << 24) |
                                              ((uint32_t)buf[4] << 16) |
                                              ((uint32_t)buf[5] << 8) | buf[6];
                ESP_LOGI(TAG, "OBD PID 0x00 (supported PIDs)   -> bitmask %02x %02x %02x %02x",
                         buf[3], buf[4], buf[5], buf[6]);
            }
            break;
        case 0x05:
            if (len >= 4) {
                s_obd_state.coolant_c = buf[3] - 40;
                ESP_LOGI(TAG, "OBD PID 0x05 (coolant temp)      -> %d C", buf[3] - 40);
            }
            break;
        case 0x0C:
            if (len >= 5) {
                s_obd_state.rpm = ((unsigned)buf[3] * 256 + buf[4]) / 4;
                ESP_LOGI(TAG, "OBD PID 0x0C (engine RPM)        -> %u rpm",
                         ((unsigned)buf[3] * 256 + buf[4]) / 4);
            }
            break;
        case 0x0D:
            if (len >= 4) {
                s_obd_state.speed_kmh = buf[3];
                ESP_LOGI(TAG, "OBD PID 0x0D (vehicle speed)     -> %u km/h", buf[3]);
            }
            break;
        case 0x11:
            if (len >= 4) {
                s_obd_state.throttle_pct = (buf[3] * 100u) / 255u;
                ESP_LOGI(TAG, "OBD PID 0x11 (throttle position) -> %u %%", (buf[3] * 100u) / 255u);
            }
            break;
        default:
            ESP_LOGI(TAG, "OBD PID 0x%02x -> unrecognized response", pid);
            break;
    }
}

/* Acts as a minimal scan tool: requests each supported PID in turn on the
 * broadcast functional ID, waits for the ECU's 0x7E8 reply, and prints it. */
static void obd_query_task(void *arg)
{
    (void)arg;
    static const uint8_t pids[] = { 0x00, 0x05, 0x0C, 0x0D, 0x11 };

    while (1) {
        for (size_t i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
            if (s_can_passive) {
                vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
                continue;
            }

            uint8_t request[8] = { 0x02, OBD_MODE_CURRENT_DATA, pids[i], 0, 0, 0, 0, 0 };
            mcp2515_send(OBD_REQUEST_ID, sizeof(request), request);

            obd_frame_t response;
            bool got = xQueueReceive(s_obd_response_queue, &response, pdMS_TO_TICKS(OBD_RESPONSE_TIMEOUT_MS));
            if (got && response.dlc >= 3 && response.data[2] == pids[i]) {
                obd_print_response(pids[i], response.data, response.dlc);
            } else if (got) {
                /* A stale/misordered response for a different PID; discard rather than misdecode it. */
                ESP_LOGW(TAG, "OBD PID 0x%02x -> mismatched response (got pid 0x%02x), discarding",
                         pids[i], response.data[2]);
            } else {
                ESP_LOGI(TAG, "OBD PID 0x%02x -> no response (timeout)", pids[i]);
            }

            vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
        }
    }
}

static void start_can_bridge(void)
{
    mcp2515_config_t config = {
        .sck_gpio = CAN_SCK_GPIO,
        .mosi_gpio = CAN_MOSI_GPIO,
        .miso_gpio = CAN_MISO_GPIO,
        .cs_gpio = CAN_CS_GPIO,
    };

    if (mcp2515_init(&config) != ESP_OK) {
        ESP_LOGE(TAG, "CAN bridge init failed; check MCP2515 wiring/power");
        return;
    }

    s_obd_response_queue = xQueueCreate(4, sizeof(obd_frame_t));
    if (s_obd_response_queue == NULL) {
        ESP_LOGE(TAG, "CAN bridge queue allocation failed");
        return;
    }

    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << CAN_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_conf));
    esp_err_t isr_result = gpio_install_isr_service(0);
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_result);
    }
    if (xTaskCreate(can_echo_task, "can_echo", 4096, NULL, tskIDLE_PRIORITY + 1,
                    &s_can_rx_task) != pdPASS) {
        ESP_LOGE(TAG, "CAN receive task allocation failed");
        return;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAN_INT_GPIO, can_int_isr, NULL));
    xTaskCreate(obd_query_task, "obd_query", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    ESP_LOGI(TAG, "CAN bridge ready: INT on GPIO%d, capturing broadcasts, echoing test id=0x%03x, querying OBD-II",
             CAN_INT_GPIO, CAN_TEST_ID);
}

/* Converts NMEA "ddmm.mmmm"/"dddmm.mmmm" plus hemisphere letter to signed
 * decimal degrees. Returns NAN on malformed input. */
static double nmea_coord_to_decimal(const char *value, char hemisphere)
{
    if (value == NULL || value[0] == '\0') {
        return NAN;
    }

    char *dot = strchr(value, '.');
    if (dot == NULL || (dot - value) < 2) {
        return NAN;
    }

    int degree_digits = (int)(dot - value) - 2;
    char degree_buf[4] = {0};
    if (degree_digits < 0 || degree_digits >= (int)sizeof(degree_buf)) {
        return NAN;
    }
    memcpy(degree_buf, value, degree_digits);

    double degrees_part = atof(degree_buf);
    double minutes_part = atof(value + degree_digits);
    double decimal = degrees_part + minutes_part / 60.0;

    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/* Splits an NMEA sentence body (already stripped of leading '$' and the
 * trailing "*checksum") into up to max_fields comma-separated fields. Empty
 * fields between consecutive commas become empty strings, not skipped. */
static size_t nmea_split_fields(char *body, char **fields, size_t max_fields)
{
    size_t count = 0;
    char *field = body;
    while (count < max_fields) {
        fields[count++] = field;
        char *comma = strchr(field, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        field = comma + 1;
    }
    return count;
}

/* buf[0]="GPRMC", [1]=time, [2]=status A/V, [3]=lat, [4]=N/S, [5]=lon,
 * [6]=E/W, [7]=speed_knots, [8]=heading_deg, [9]=date. */
static void nmea_parse_rmc(char **fields, size_t count)
{
    if (count < 10) {
        return;
    }

    bool fix_valid = (fields[2][0] == 'A');
    double lat = NAN;
    double lon = NAN;
    float speed_kmh = 0;
    float heading_deg = 0;
    if (fix_valid) {
        lat = nmea_coord_to_decimal(fields[3], fields[4][0]);
        lon = nmea_coord_to_decimal(fields[5], fields[6][0]);
        speed_kmh = (float)(atof(fields[7]) * 1.852);
        heading_deg = (float)atof(fields[8]);
    }

    portENTER_CRITICAL(&s_gps_lock);
    s_gps_state.fix_valid = fix_valid;
    if (fix_valid) {
        s_gps_state.lat = lat;
        s_gps_state.lon = lon;
        s_gps_state.speed_kmh = speed_kmh;
        s_gps_state.heading_deg = heading_deg;
    }
    snprintf(s_gps_state.utc_time, sizeof(s_gps_state.utc_time), "%s", fields[1]);
    snprintf(s_gps_state.utc_date, sizeof(s_gps_state.utc_date), "%s", fields[9]);
    portEXIT_CRITICAL(&s_gps_lock);

    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_gps_log_us >= 1000000) {
        s_last_gps_log_us = now_us;
        if (fix_valid) {
            ESP_LOGI(TAG, "GPS fix -> lat=%.6f lon=%.6f speed=%.1f km/h heading=%.1f deg utc=%s date=%s",
                     lat, lon, (double)speed_kmh, (double)heading_deg, fields[1], fields[9]);
        } else {
            ESP_LOGI(TAG, "GPS fix -> waiting for valid fix utc=%s date=%s", fields[1], fields[9]);
        }
    }
}

/* buf[0]="GPGGA", [1]=time, [2]=lat, [3]=N/S, [4]=lon, [5]=E/W,
 * [6]=fix_quality, [7]=satellites. */
static void nmea_parse_gga(char **fields, size_t count)
{
    if (count < 8) {
        return;
    }

    portENTER_CRITICAL(&s_gps_lock);
    s_gps_state.satellites = (uint8_t)atoi(fields[7]);
    portEXIT_CRITICAL(&s_gps_lock);
    (void)fields[6];
}

/* Validates the NMEA checksum (XOR of all bytes between '$' and '*') before
 * handing the sentence to a talker-specific parser. */
static void nmea_parse_line(char *line)
{
    if (line[0] != '$') {
        return;
    }

    char *star = strchr(line, '*');
    if (star == NULL || strlen(star) < 3) {
        ESP_LOGW(TAG, "GPS: malformed sentence (no checksum)");
        return;
    }

    uint8_t checksum = 0;
    for (char *p = line + 1; p < star; p++) {
        checksum ^= (uint8_t)*p;
    }
    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    if (checksum != expected) {
        ESP_LOGW(TAG, "GPS: checksum mismatch");
        return;
    }
    *star = '\0';

    char *fields[12];
    size_t count = nmea_split_fields(line + 1, fields, 12);
    if (count == 0) {
        return;
    }

    if (strcmp(fields[0], "GPRMC") == 0) {
        nmea_parse_rmc(fields, count);
    } else if (strcmp(fields[0], "GPGGA") == 0) {
        nmea_parse_gga(fields, count);
    }
}

/* Reads raw bytes from the GPS UART and reassembles them into '\n'-terminated
 * NMEA lines before parsing; NMEA sentences are short so a single-line buffer
 * is enough. */
static void gps_uart_task(void *arg)
{
    (void)arg;
    static char line_buf[96];
    size_t line_len = 0;
    uint8_t rx_buf[64];

    while (1) {
        int read = uart_read_bytes(GPS_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(200));
        for (int i = 0; i < read; i++) {
            char c = (char)rx_buf[i];
            if (c == '\r') {
                continue;
            }
            if (c == '\n') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    nmea_parse_line(line_buf);
                    line_len = 0;
                }
                continue;
            }
            if (line_len < sizeof(line_buf) - 1) {
                line_buf[line_len++] = c;
            } else {
                /* Line too long for the buffer; discard it and resync on the next '\n'. */
                line_len = 0;
            }
        }
    }
}

static void start_gps_bridge(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (xTaskCreate(gps_uart_task, "gps_uart", 3072, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "GPS UART task allocation failed");
        return;
    }
    ESP_LOGI(TAG, "GPS bridge ready: UART1 RX on GPIO%d, TX on GPIO%d, %d baud",
             GPS_UART_RX_GPIO, GPS_UART_TX_GPIO, GPS_UART_BAUD);
}

static bool health_idle0_hook(void)
{
    s_idle0_count++;
    return false;
}

static bool health_idle1_hook(void)
{
    s_idle1_count++;
    return false;
}

static const char *reset_reason_to_string(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXT";
        case ESP_RST_SW: return "SW";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "UNKNOWN";
    }
}

static uint32_t get_partitioned_flash_size(void)
{
    uint32_t total = 0;
    esp_partition_iterator_t iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY,
                                                           ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (iterator != NULL) {
        const esp_partition_t *partition = esp_partition_get(iterator);
        if (partition != NULL) {
            total += partition->size;
        }
        iterator = esp_partition_next(iterator);
    }
    esp_partition_iterator_release(iterator);
    return total;
}

static void health_task(void *arg)
{
    (void)arg;
    uint32_t last_idle0 = s_idle0_count;
    uint32_t last_idle1 = s_idle1_count;
    uint32_t max_idle0_delta = 1;
    uint32_t max_idle1_delta = 1;
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    uint32_t flash_partitioned = get_partitioned_flash_size();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    uint32_t app_partition_size = running_partition == NULL ? 0 : running_partition->size;
    esp_reset_reason_t restart_reason = esp_reset_reason();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t idle0 = s_idle0_count;
        uint32_t idle1 = s_idle1_count;
        uint32_t idle0_delta = idle0 - last_idle0;
        uint32_t idle1_delta = idle1 - last_idle1;
        last_idle0 = idle0;
        last_idle1 = idle1;
        if (idle0_delta > max_idle0_delta) {
            max_idle0_delta = idle0_delta;
        }
        if (idle1_delta > max_idle1_delta) {
            max_idle1_delta = idle1_delta;
        }

        health_state_t state = {
            .idle0_pct = (idle0_delta * 100u) / max_idle0_delta,
            .idle1_pct = (idle1_delta * 100u) / max_idle1_delta,
            .idle0_delta = idle0_delta,
            .idle1_delta = idle1_delta,
            .idle0_max_delta = max_idle0_delta,
            .idle1_max_delta = max_idle1_delta,
            .heap_free = esp_get_free_heap_size(),
            .heap_min_free = esp_get_minimum_free_heap_size(),
            .internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            .internal_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
            .psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
            .psram_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
            .flash_size = flash_size,
            .flash_partitioned = flash_partitioned,
            .app_partition_size = app_partition_size,
            .uptime_s = (uint32_t)(esp_timer_get_time() / 1000000),
            .restart_reason = restart_reason,
        };

        portENTER_CRITICAL(&s_health_lock);
        s_health_state = state;
        portEXIT_CRITICAL(&s_health_lock);

        ESP_LOGI(TAG, "HEALTH uptime=%lus heap=%u/%u internal=%u/%u psram=%u/%u idle=%lu%%/%lu%% restart=%s",
                 (unsigned long)state.uptime_s,
                 (unsigned int)state.heap_free, (unsigned int)state.heap_min_free,
                 (unsigned int)state.internal_free, (unsigned int)state.internal_min_free,
                 (unsigned int)state.psram_free, (unsigned int)state.psram_min_free,
                 (unsigned long)state.idle0_pct, (unsigned long)state.idle1_pct,
                 reset_reason_to_string(state.restart_reason));
    }
}

static void start_health_monitor(void)
{
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(health_idle0_hook, 0));
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(health_idle1_hook, 1));
    xTaskCreate(health_task, "health", 3072, NULL, tskIDLE_PRIORITY + 1, NULL);
}

static void cdc_send(const char *msg)
{
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)msg, strlen(msg));
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

static void send_menu(void)
{
    char menu[320];
    snprintf(menu, sizeof(menu),
             "\r\n=== JC-ESP32P4-M3 control channel ===\r\n"
             "freq <ms>  set blink half-period, %d-%d ms (current: %lu). Example: freq 250\r\n"
             "ll <0-3>   set log verbosity: 0=DEBUG 1=INFO 2=WARNING 3=ERROR (current: %s)\r\n"
             "help       show this menu\r\n",
             BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS,
             (unsigned long)blink_half_period_ms,
             log_level_to_string(s_log_level));
    cdc_send(menu);
}

/* Parses "freq <ms>" / "ll <0-3>" / "help" typed into the control channel. */
static void handle_command(char *line)
{
    char msg[96];

    if (strcmp(line, "help") == 0) {
        send_menu();
    } else if (strlen(line) > 0) {
        bool ok = (strncmp(line, "ll ", 3) == 0)
                      ? apply_ll_command(line, msg, sizeof(msg))
                      : apply_freq_command(line, msg, sizeof(msg));
        printf("USB cmd '%s' -> %s", line, ok ? "OK\n" : "ERR\n");
        cdc_send(msg);
    }
}

/* Placeholder for P4<->C6 command ingress once transport pins are confirmed. */
__attribute__((unused))
static void handle_companion_command(const char *line)
{
    char msg[96];
    bool ok = apply_freq_command(line, msg, sizeof(msg));
    printf("C6 bridge cmd '%s' -> %s", line, ok ? "OK\n" : "ERR\n");
}

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    static char line[16];
    static size_t line_len = 0;

    uint8_t buf[64];
    size_t rx_size = 0;

    if (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size) != ESP_OK) {
        return;
    }

    for (size_t i = 0; i < rx_size; i++) {
        char c = (char)buf[i];
        if (c == '\r' || c == '\n') {
            line[line_len] = '\0';
            handle_command(line);
            line_len = 0;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        }
    }
}

static void tinyusb_cdc_line_state_callback(int itf, cdcacm_event_t *event)
{
    if (event->line_state_changed_data.dtr) {
        send_menu();
    }
}

/* Fallback in case a terminal's DTR toggle isn't caught in time to show the menu. */
static void control_channel_banner_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    send_menu();
    vTaskDelete(NULL);
}

/* Independent USB-CDC control channel on the board's second USB-C port. */
static void start_control_channel(void)
{
    tinyusb_config_t tusb_cfg = TINYUSB_CONFIG_HIGH_SPEED(NULL, NULL);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_callback,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));

    xTaskCreate(control_channel_banner_task, "usb_banner", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
}

static esp_err_t frequency_http_handler(httpd_req_t *request)
{
    char command[64] = {0};
    int received = httpd_req_recv(request, command, sizeof(command) - 1);
    char response[96];

    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected freq <ms>");
        return ESP_FAIL;
    }

    command[received] = '\0';
    bool accepted = apply_freq_command(command, response, sizeof(response));
    printf("HTTP cmd '%s' -> %s", command, accepted ? "OK\n" : "ERR\n");
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, response);
    return accepted ? ESP_OK : ESP_FAIL;
}

static esp_err_t obd_http_handler(httpd_req_t *request)
{
    obd_state_t state = s_obd_state;
    char response[192];
    snprintf(response, sizeof(response),
             "{\"supported_pids\":\"%08lx\",\"coolant_c\":%d,\"rpm\":%u,\"speed_kmh\":%u,\"throttle_pct\":%u}",
             (unsigned long)state.supported_pids, state.coolant_c,
             state.rpm, state.speed_kmh, state.throttle_pct);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

static esp_err_t can_capture_http_handler(httpd_req_t *request)
{
    uint64_t after = 0;
    char query[48];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK) {
        char value[24];
        if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK) {
            after = strtoull(value, NULL, 10);
        }
    }

    can_capture_frame_t batch[CAN_CAPTURE_HTTP_BATCH];
    size_t count = 0;
    uint64_t latest;
    uint64_t dropped = 0;

    portENTER_CRITICAL(&s_can_capture_lock);
    latest = s_can_capture_sequence;
    uint64_t oldest = latest >= CAN_CAPTURE_CAPACITY ? latest - CAN_CAPTURE_CAPACITY + 1 : 1;
    uint64_t first = after + 1;
    if (first < oldest) {
        dropped = oldest - first;
        first = oldest;
    }
    for (uint64_t sequence = first; sequence <= latest && count < CAN_CAPTURE_HTTP_BATCH; sequence++) {
        can_capture_frame_t frame = s_can_capture[(sequence - 1) % CAN_CAPTURE_CAPACITY];
        if (frame.sequence == sequence) {
            batch[count++] = frame;
        }
    }
    portEXIT_CRITICAL(&s_can_capture_lock);

    char *response = malloc(CAN_CAPTURE_RESPONSE_SIZE);
    if (response == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_ERR_NO_MEM;
    }

    size_t used = (size_t)snprintf(response, CAN_CAPTURE_RESPONSE_SIZE,
                                   "{\"latest\":%llu,\"dropped\":%llu,\"hardware_overflow\":%lu,"
                                   "\"passive\":%s,\"isr_count\":%lu,\"timeout_count\":%lu,\"frames\":[",
                                   (unsigned long long)latest, (unsigned long long)dropped,
                                   (unsigned long)mcp2515_get_receive_overflow_count(),
                                   s_can_passive ? "true" : "false",
                                   (unsigned long)s_can_isr_count, (unsigned long)s_can_timeout_count);
    for (size_t i = 0; i < count; i++) {
        can_capture_frame_t *frame = &batch[i];
        used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used,
                                 "%s{\"seq\":%llu,\"time_us\":%lld,\"bus\":0,\"id\":%lu,"
                                 "\"extended\":false,\"rtr\":false,\"dlc\":%u,\"data\":\"",
                                 i == 0 ? "" : ",", (unsigned long long)frame->sequence,
                                 (long long)frame->timestamp_us, (unsigned long)frame->id, frame->dlc);
        for (uint8_t j = 0; j < frame->dlc && used + 2 < CAN_CAPTURE_RESPONSE_SIZE; j++) {
            used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used,
                                     "%02X", frame->data[j]);
        }
        used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used, "\"}");
    }
    snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used, "]}");

    httpd_resp_set_type(request, "application/json");
    esp_err_t result = httpd_resp_sendstr(request, response);
    free(response);
    return result;
}

static esp_err_t can_mode_http_handler(httpd_req_t *request)
{
    char mode[16] = {0};
    int received = httpd_req_recv(request, mode, sizeof(mode) - 1);
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected active or passive");
        return ESP_FAIL;
    }
    mode[received] = '\0';

    bool passive;
    if (strcmp(mode, "passive") == 0) {
        passive = true;
    } else if (strcmp(mode, "active") == 0) {
        passive = false;
    } else {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected active or passive");
        return ESP_FAIL;
    }

    if (mcp2515_set_listen_only(passive) != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "CAN mode change failed");
        return ESP_FAIL;
    }

    s_can_passive = passive;
    if (!passive) {
        xQueueReset(s_obd_response_queue);
    }
    ESP_LOGI(TAG, "CAN mode changed to %s", passive ? "passive" : "active");
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, passive ? "OK passive" : "OK active");
    return ESP_OK;
}

static esp_err_t gps_http_handler(httpd_req_t *request)
{
    gps_state_t state;
    portENTER_CRITICAL(&s_gps_lock);
    state = s_gps_state;
    portEXIT_CRITICAL(&s_gps_lock);

    char response[256];
    snprintf(response, sizeof(response),
             "{\"fix_valid\":%s,\"lat\":%.6f,\"lon\":%.6f,\"speed_kmh\":%.1f,"
             "\"heading_deg\":%.1f,\"utc_time\":\"%s\",\"utc_date\":\"%s\",\"satellites\":%u}",
             state.fix_valid ? "true" : "false", state.lat, state.lon,
             (double)state.speed_kmh, (double)state.heading_deg,
             state.utc_time, state.utc_date, state.satellites);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

static esp_err_t health_http_handler(httpd_req_t *request)
{
    health_state_t state;
    portENTER_CRITICAL(&s_health_lock);
    state = s_health_state;
    portEXIT_CRITICAL(&s_health_lock);

    uint32_t flash_free = state.flash_size > state.flash_partitioned
                              ? state.flash_size - state.flash_partitioned
                              : 0;
    char response[512];
    snprintf(response, sizeof(response),
             "{\"uptime_s\":%lu,\"restart_reason\":\"%s\"," 
             "\"heap_free\":%u,\"heap_min_free\":%u,"
             "\"internal_free\":%u,\"internal_min_free\":%u,"
             "\"psram_free\":%u,\"psram_min_free\":%u,"
             "\"flash_size\":%lu,\"flash_partitioned\":%lu,\"flash_free\":%lu,"
             "\"app_partition_size\":%lu,"
             "\"idle0_pct\":%lu,\"idle1_pct\":%lu,"
             "\"busy0_pct\":%lu,\"busy1_pct\":%lu,"
             "\"idle0_delta\":%lu,\"idle1_delta\":%lu,"
             "\"idle0_max_delta\":%lu,\"idle1_max_delta\":%lu}",
             (unsigned long)state.uptime_s, reset_reason_to_string(state.restart_reason),
             (unsigned int)state.heap_free, (unsigned int)state.heap_min_free,
             (unsigned int)state.internal_free, (unsigned int)state.internal_min_free,
             (unsigned int)state.psram_free, (unsigned int)state.psram_min_free,
             (unsigned long)state.flash_size, (unsigned long)state.flash_partitioned,
             (unsigned long)flash_free, (unsigned long)state.app_partition_size,
             (unsigned long)state.idle0_pct, (unsigned long)state.idle1_pct,
             (unsigned long)(100u - state.idle0_pct), (unsigned long)(100u - state.idle1_pct),
             (unsigned long)state.idle0_delta, (unsigned long)state.idle1_delta,
             (unsigned long)state.idle0_max_delta, (unsigned long)state.idle1_max_delta);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

static void start_frequency_http_server(void)
{
    if (http_server != NULL) {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t frequency_uri = {
        .uri = "/api/frequency",
        .method = HTTP_POST,
        .handler = frequency_http_handler,
    };
    httpd_uri_t obd_uri = {
        .uri = "/api/obd",
        .method = HTTP_GET,
        .handler = obd_http_handler,
    };
    httpd_uri_t can_capture_uri = {
        .uri = "/api/can",
        .method = HTTP_GET,
        .handler = can_capture_http_handler,
    };
    httpd_uri_t can_mode_uri = {
        .uri = "/api/can/mode",
        .method = HTTP_POST,
        .handler = can_mode_http_handler,
    };
    httpd_uri_t gps_uri = {
        .uri = "/api/gps",
        .method = HTTP_GET,
        .handler = gps_http_handler,
    };
    httpd_uri_t health_uri = {
        .uri = "/api/health",
        .method = HTTP_GET,
        .handler = health_http_handler,
    };

    if (httpd_start(&http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(http_server, &frequency_uri);
        httpd_register_uri_handler(http_server, &obd_uri);
        httpd_register_uri_handler(http_server, &can_capture_uri);
        httpd_register_uri_handler(http_server, &can_mode_uri);
        httpd_register_uri_handler(http_server, &gps_uri);
        httpd_register_uri_handler(http_server, &health_uri);
        ESP_LOGI(TAG, "WiFi frequency API ready: POST /api/frequency");
        ESP_LOGI(TAG, "OBD monitor API ready: GET /api/obd");
        ESP_LOGI(TAG, "CAN capture API ready: GET /api/can?after=<sequence>");
        ESP_LOGI(TAG, "CAN mode API ready: POST /api/can/mode");
        ESP_LOGI(TAG, "GPS API ready: GET /api/gps");
        ESP_LOGI(TAG, "Health API ready: GET /api/health");
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;
        char address[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, address, sizeof(address));
        ESP_LOGI(TAG, "WiFi connected, IP: %s", address);
        start_frequency_http_server();
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
        char response[64];
        snprintf(response, sizeof(response), "WiFi connected ip=%s\r\n", address);
        ble_publish_response(BLE_HS_CONN_HANDLE_NONE, response);
#endif
        return;
    }

    /* Without this, a failed join (bad password, AP out of range) leaves the app waiting forever. */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = event_data;
        printf("WiFi disconnected, reason: %d\n", event->reason);
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
        char response[64];
        snprintf(response, sizeof(response), "ERR WiFi disconnected reason=%d\r\n", event->reason);
        ble_publish_response(BLE_HS_CONN_HANDLE_NONE, response);
#endif
    }
}

static bool start_wifi_connection(const char *ssid, const char *password, char *response, size_t response_len)
{
    if (!wifi_started) {
        snprintf(response, response_len, "ERR WiFi is not ready\r\n");
        return false;
    }

    size_t ssid_len = strlen(ssid);
    size_t password_len = strlen(password);
    if (ssid_len == 0 || ssid_len > 32 || password_len < 8 || password_len > 63) {
        snprintf(response, response_len, "ERR invalid WiFi credentials\r\n");
        return false;
    }

    wifi_config_t config = {0};
    memcpy(config.sta.ssid, ssid, ssid_len);
    memcpy(config.sta.password, password, password_len);
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /*
     * ESP-Hosted's netif "started" latch can get stuck false after the STA
     * was already up once, so a later esp_wifi_connect() associates but
     * WIFI_EVENT_STA_CONNECTED never reaches esp_netif: DHCP never runs and
     * no IP ever arrives, with no error either. A stop/start cycle clears
     * that latch unconditionally before each connect attempt.
     */
    esp_wifi_stop();
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &config);
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(300));
        err = esp_wifi_connect();
    }
    if (err != ESP_OK) {
        snprintf(response, response_len, "ERR WiFi connect failed: %s\r\n", esp_err_to_name(err));
        return false;
    }

    snprintf(response, response_len, "OK WiFi connecting\r\n");
    return true;
}

#if defined(CONFIG_ESP_HOSTED_ENABLED) && CONFIG_ESP_HOSTED_ENABLED
static void start_hosted_wifi_link(void)
{
    esp_err_t err = esp_hosted_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ESP-Hosted init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_hosted_connect_to_slave();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ESP-Hosted slave connect failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "WiFi netif init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "WiFi event loop init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Without this, the STA associates but never gets a netif/DHCP client, so it never gets an IP. */
    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "WiFi STA netif create failed");
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Default country setting only scans channels 1-11; many routers (EU/IL) use 12-13. */
    wifi_country_t country = {
        .cc = "01",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    esp_wifi_set_country(&country);

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi max TX power limited to %.2f dBm", WIFI_MAX_TX_POWER_QDBM / 4.0f);
    } else {
        ESP_LOGW(TAG, "WiFi TX power limit failed: %s", esp_err_to_name(err));
    }

    wifi_started = true;
    ESP_LOGI(TAG, "Hosted radio link is up (P4 host, C6 co-processor).");
}
#else
static void start_hosted_wifi_link(void)
{
    ESP_LOGI(TAG, "ESP-Hosted disabled in this build; hosted radio link not started.");
}
#endif

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
#define BLE_COMPANION_SERVICE_UUID 0xFFF0
#define BLE_COMPANION_CHAR_UUID    0xFFF1
#define BLE_RESPONSE_CHAR_UUID     0xFFF2
#define BLE_WIFI_CONFIG_CHAR_UUID  0xFFF3

static uint8_t s_ble_addr_type = 0;
static uint16_t s_ble_response_handle;
static uint16_t s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static char s_ble_response[96] = "Ready\r\n";

static void ble_start_advertising(void);

static int ble_response_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return os_mbuf_append(ctxt->om, s_ble_response, strlen(s_ble_response)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static void ble_publish_response(uint16_t conn_handle, const char *message)
{
    snprintf(s_ble_response, sizeof(s_ble_response), "%s", message);
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        conn_handle = s_ble_conn_handle;
    }
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        printf("BLE response not sent (no active connection): %s", message);
        return;
    }
    int rc = ble_gatts_notify(conn_handle, s_ble_response_handle);
    printf("BLE notify conn_handle=%d rc=%d: %s", (int)conn_handle, rc, message);
    if (rc != 0) {
        printf("BLE response notify not sent: rc=%d\n", rc);
    }
}

static int ble_wifi_config_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle;
    (void)arg;
    char credentials[98] = {0};
    uint16_t length = OS_MBUF_PKTLEN(ctxt->om);
    if (length >= sizeof(credentials) || ble_hs_mbuf_to_flat(ctxt->om, credentials, length, NULL) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    credentials[length] = '\0';

    char *password = strchr(credentials, '\n');
    char response[96];
    if (password == NULL) {
        snprintf(response, sizeof(response), "ERR WiFi format is ssid\\npassword\r\n");
    } else {
        *password++ = '\0';
        start_wifi_connection(credentials, password, response, sizeof(response));
    }
    ble_publish_response(conn_handle, response);
    return 0;
}

static int ble_freq_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    char cmd[64] = {0};
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= sizeof(cmd)) {
        len = sizeof(cmd) - 1;
    }

    if (ble_hs_mbuf_to_flat(ctxt->om, cmd, len, NULL) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    cmd[len] = '\0';

    char msg[96];
    bool ok = apply_freq_command(cmd, msg, sizeof(msg));
    printf("BLE cmd '%s' -> %s", cmd, ok ? "OK\n" : "ERR\n");
    ble_publish_response(conn_handle, msg);
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_COMPANION_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BLE_COMPANION_CHAR_UUID),
                .access_cb = ble_freq_write_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_RESPONSE_CHAR_UUID),
                .access_cb = ble_response_read_cb,
                .val_handle = &s_ble_response_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_WIFI_CONFIG_CHAR_UUID),
                .access_cb = ble_wifi_config_write_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {0},
        },
    },
    {0},
};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_CONNECT && event->connect.status == 0) {
        s_ble_conn_handle = event->connect.conn_handle;
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_start_advertising();
    }
    return 0;
}

static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    const char *name = "JC-P4-C6";
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv fields failed: rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_ble_addr_type, NULL, BLE_HS_FOREVER, &adv, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv start failed: rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE advertising started as %s", name);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE addr infer failed: rc=%d", rc);
        return;
    }
    ble_start_advertising();
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void start_ble_hosted(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "NimBLE init failed: rc=%d", rc);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE count cfg failed: rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE add service failed: rc=%d", rc);
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "NimBLE host started (controller on C6 over hosted link).");
}
#else
static void start_ble_hosted(void)
{
    ESP_LOGI(TAG, "BLE disabled in this build; hosted BLE not started.");
}
#endif

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    init_led();
    esp_log_level_set(TAG, s_log_level);
    start_health_monitor();

    ESP_LOGI(TAG, "JC-ESP32P4-M3 booted. Blinking LED on GPIO %d", LED_GPIO);
    ESP_LOGI(TAG, "Free heap before hosted init: %lu bytes (internal: %lu)",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    start_control_channel();
    start_hosted_wifi_link();
    start_ble_hosted();
    start_can_bridge();
    start_gps_bridge();

    uint32_t count = 0;
    while (1) {
        uint32_t half_period = blink_half_period_ms;

        gpio_set_level(LED_GPIO, 1);
        ESP_LOGD(TAG, "ON");
        vTaskDelay(pdMS_TO_TICKS(half_period));

        gpio_set_level(LED_GPIO, 0);
        ESP_LOGD(TAG, "OFF");
        vTaskDelay(pdMS_TO_TICKS(half_period));

        count++;
    }
}
