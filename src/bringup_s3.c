/* Bring-up firmware for the new ESP32-S3 N16R8 board -- NOT the real
 * application yet (see HARDWARE_MIGRATION.md). Confirmed so far: GPIO48
 * drives the onboard WS2812 RGB LED (matches a vendor pinout diagram,
 * verified against the physical board), and the working flash/monitor
 * procedure for this board's finicky USB-Serial/JTAG port (documented in
 * HARDWARE_MIGRATION.md's "Open questions -- resolved" section).
 *
 * This step adds just enough Wi-Fi/BLE/HTTP for the *already-installed,
 * unmodified* CarTheftGuard phone app to connect to this board and control
 * the LED's blink rate, to prove native Wi-Fi+BLE (no ESP-Hosted, no C6
 * co-processor) actually works on this chip before porting anything else.
 * Deliberately reuses the JC-ESP32P4-M3 board's exact BLE advertised name
 * ("JC-P4-C6") and AP SSID/password ("CarTheftGuard-P4" / "&Car1310") --
 * that's the ONLY way the phone app (which only knows to auto-join a board
 * advertising that specific name, see BoardLink.java's scanCallback) will
 * recognize this board at all. This is a deliberate, temporary bring-up
 * shortcut, not a real identity for this new hardware -- expect to pick
 * real distinct values once this becomes its own product line rather than
 * a stand-in for the P4 board during testing.
 *
 * POST /api/frequency ("freq <ms>" body) and its response format
 * ("OK freq=<ms> ms\r\n" / "ERR ...\r\n") are copied verbatim from
 * JC-ESP32P4-M3's apply_freq_command/frequency_http_handler so the app's
 * existing BoardLink.sendFrequency() needs zero changes to work here.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_coexist.h"
#include "esp_bt.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "bringup";

#define LED_GPIO_CANDIDATE 48
/* First real test of the native TWAI (CAN) controller -- the whole point
 * of this hardware migration (see HARDWARE_MIGRATION.md's "Why change
 * hardware"): no SPI, no external MCP2515 controller chip, just these two
 * GPIOs straight into an SN65HVD230 transceiver's TXD/RXD pins. 500 kbps
 * to match ArdunioUsbBridgeToCan's simulator (see project memory). Chosen
 * as safe general-purpose pins per the vendor pinout diagram: not
 * strapping (0/3/45/46), not USB (19/20), not PSRAM (35/36/37), not the
 * confirmed LED pin (48). */
#define CAN_TX_GPIO 4
#define CAN_RX_GPIO 5
/* ArdunioUsbBridgeToCan's simulated GPS goes out over SoftwareSerial on
 * the Arduino's pin 3 (GPS_TX_PIN in its main.cpp) at 9600 baud, 5V logic
 * -- routed through a voltage divider down to 3.3V before reaching here,
 * since S3 GPIOs aren't 5V-tolerant. TX_GPIO is wired but unused (nothing
 * needs to transmit back to the simulator's GPS input). */
#define GPS_TX_GPIO 6
#define GPS_RX_GPIO 7
#define GPS_BAUD 9600
#define GPS_UART_PORT UART_NUM_1
#define WIFI_AP_SSID "CarTheftGuard-P4"
#define WIFI_AP_PASSWORD "&Car1310"
#define WIFI_AP_IP "192.168.4.1"
#define BLE_DEVICE_NAME "JC-P4-C6"
#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

static led_strip_handle_t s_led;
static volatile uint32_t s_blink_half_period_ms = 500;
static uint8_t s_ble_addr_type = 0;

/* GET /api/gps state + JSON contract copied verbatim from
 * JC-ESP32P4-M3's gps_state_t/gps_http_handler so BoardLink.java's
 * existing GPS polling needs zero changes. */
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

/* GET /api/obd -- same JSON contract as JC-ESP32P4-M3's obd_http_handler,
 * but populated passively: the simulator broadcasts engine/vehicle state
 * unsolicited on 0x120/0x180 every 20/50ms regardless of any active OBD-II
 * request (see JC-ESP32P4-M3's decode_engine_broadcast/
 * decode_vehicle_broadcast, ported verbatim below), so no CAN TX or active
 * Mode 01 polling is needed to drive the app's Monitor tab against the
 * simulator. supported_pids/supported_pids_2 stay 0 -- those only come
 * from an active PID 0x00/0x20 request, not implemented here. */
typedef struct {
    uint32_t supported_pids;
    uint32_t supported_pids_2;
    int coolant_c;
    uint16_t rpm;
    uint8_t speed_kmh;
    uint8_t throttle_pct;
} obd_state_t;

#define CAN_ID_ENGINE_STATE 0x120
#define CAN_ID_VEHICLE_STATE 0x180

static obd_state_t s_obd_state;
static portMUX_TYPE s_obd_lock = portMUX_INITIALIZER_UNLOCKED;

static void decode_engine_broadcast(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 4) {
        return;
    }
    uint16_t rpm_raw = ((uint16_t)data[0] << 8) | data[1];
    int coolant_c = (int)data[2] - 40;
    uint8_t throttle_pct = (uint8_t)((uint16_t)data[3] * 100u / 255u);

    portENTER_CRITICAL(&s_obd_lock);
    s_obd_state.rpm = rpm_raw / 4;
    s_obd_state.coolant_c = coolant_c;
    s_obd_state.throttle_pct = throttle_pct;
    portEXIT_CRITICAL(&s_obd_lock);
}

static void decode_vehicle_broadcast(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 2) {
        return;
    }
    uint16_t speed_centi_kmh = ((uint16_t)data[0] << 8) | data[1];

    portENTER_CRITICAL(&s_obd_lock);
    s_obd_state.speed_kmh = (uint8_t)(speed_centi_kmh / 100);
    portEXIT_CRITICAL(&s_obd_lock);
}

/* GET /api/can ring buffer + JSON contract, same idea -- see
 * JC-ESP32P4-M3's can_capture_frame_t/can_capture_http_handler. Unlike
 * that board, extended/rtr here reflect the *real* flags from
 * twai_message_t (the P4 board's MCP2515 path never tracked them and
 * always reported false; the native TWAI driver already gives us the
 * real values for free). "passive"/"isr_count"/"timeout_count" are kept
 * as fields for JSON-shape compatibility but aren't meaningful for the
 * native driver yet (no listen-only mode wired up in this bring-up) --
 * always report false/0/0. */
typedef struct {
    uint64_t sequence;
    int64_t timestamp_us;
    uint32_t id;
    bool extended;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
} can_capture_frame_t;

#define CAN_CAPTURE_CAPACITY 4096
#define CAN_CAPTURE_HTTP_BATCH 128
#define CAN_CAPTURE_RESPONSE_SIZE 24576

static can_capture_frame_t s_can_capture[CAN_CAPTURE_CAPACITY];
static uint64_t s_can_capture_sequence;
static portMUX_TYPE s_can_capture_lock = portMUX_INITIALIZER_UNLOCKED;

static void capture_can_frame(const twai_message_t *message)
{
    int64_t timestamp_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_can_capture_lock);
    uint64_t sequence = ++s_can_capture_sequence;
    can_capture_frame_t *frame = &s_can_capture[(sequence - 1) % CAN_CAPTURE_CAPACITY];
    frame->sequence = sequence;
    frame->timestamp_us = timestamp_us;
    frame->id = message->identifier;
    frame->extended = message->extd;
    frame->rtr = message->rtr;
    frame->dlc = message->data_length_code > 8 ? 8 : message->data_length_code;
    memcpy(frame->data, message->data, frame->dlc);
    portEXIT_CRITICAL(&s_can_capture_lock);

    if (message->identifier == CAN_ID_ENGINE_STATE) {
        decode_engine_broadcast(message->data, frame->dlc);
    } else if (message->identifier == CAN_ID_VEHICLE_STATE) {
        decode_vehicle_broadcast(message->data, frame->dlc);
    }
}

/* Same parsing/response contract as JC-ESP32P4-M3's apply_freq_command --
 * see this file's top doc comment. */
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
    s_blink_half_period_ms = (uint32_t)value;
    ESP_LOGI(TAG, "Blink half-period set to %ld ms", value);
    snprintf(out_msg, out_msg_len, "OK freq=%ld ms\r\n", value);
    return true;
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
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, response);
    return accepted ? ESP_OK : ESP_FAIL;
}

/* JSON contract copied verbatim from JC-ESP32P4-M3's gps_http_handler. */
/* JSON contract copied verbatim from JC-ESP32P4-M3's obd_http_handler
 * (see the obd_state_t doc comment for what's populated vs. left at 0). */
static esp_err_t obd_http_handler(httpd_req_t *request)
{
    obd_state_t state;
    portENTER_CRITICAL(&s_obd_lock);
    state = s_obd_state;
    portEXIT_CRITICAL(&s_obd_lock);
    char response[224];
    snprintf(response, sizeof(response),
             "{\"supported_pids\":\"%08lx\",\"supported_pids_2\":\"%08lx\",\"coolant_c\":%d,\"rpm\":%u,\"speed_kmh\":%u,\"throttle_pct\":%u}",
             (unsigned long)state.supported_pids, (unsigned long)state.supported_pids_2,
             state.coolant_c, state.rpm, state.speed_kmh, state.throttle_pct);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
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

/* JSON contract copied verbatim from JC-ESP32P4-M3's
 * can_capture_http_handler -- see capture_can_frame's doc comment for the
 * one real difference (extended/rtr are the real flags here, not always
 * false). */
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

    twai_status_info_t status = {0};
    twai_get_status_info(&status);

    char *response = malloc(CAN_CAPTURE_RESPONSE_SIZE);
    if (response == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_ERR_NO_MEM;
    }
    size_t used = (size_t)snprintf(response, CAN_CAPTURE_RESPONSE_SIZE,
                                   "{\"latest\":%llu,\"dropped\":%llu,\"hardware_overflow\":%lu,"
                                   "\"passive\":false,\"isr_count\":0,\"timeout_count\":0,\"frames\":[",
                                   (unsigned long long)latest, (unsigned long long)dropped,
                                   (unsigned long)status.rx_overrun_count);
    for (size_t i = 0; i < count; i++) {
        can_capture_frame_t *frame = &batch[i];
        used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used,
                                 "%s{\"seq\":%llu,\"time_us\":%lld,\"bus\":0,\"id\":%lu,"
                                 "\"extended\":%s,\"rtr\":%s,\"dlc\":%u,\"data\":\"",
                                 i == 0 ? "" : ",", (unsigned long long)frame->sequence,
                                 (long long)frame->timestamp_us, (unsigned long)frame->id,
                                 frame->extended ? "true" : "false", frame->rtr ? "true" : "false",
                                 frame->dlc);
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

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return;
    }
    httpd_uri_t frequency_uri = {
        .uri = "/api/frequency",
        .method = HTTP_POST,
        .handler = frequency_http_handler,
    };
    httpd_uri_t gps_uri = {
        .uri = "/api/gps",
        .method = HTTP_GET,
        .handler = gps_http_handler,
    };
    httpd_uri_t can_uri = {
        .uri = "/api/can",
        .method = HTTP_GET,
        .handler = can_capture_http_handler,
    };
    httpd_uri_t obd_uri = {
        .uri = "/api/obd",
        .method = HTTP_GET,
        .handler = obd_http_handler,
    };
    httpd_register_uri_handler(server, &frequency_uri);
    httpd_register_uri_handler(server, &gps_uri);
    httpd_register_uri_handler(server, &can_uri);
    httpd_register_uri_handler(server, &obd_uri);
    ESP_LOGI(TAG, "HTTP API ready: POST /api/frequency, GET /api/gps, GET /api/can?after=<seq>, GET /api/obd");
}

/* BLE+WiFi concurrency was tested exhaustively and abandoned -- see
 * HARDWARE_MIGRATION.md's DHCP section for the full record. Nothing short
 * of BLE never being initialized at all (not paused, not merely disabled)
 * gave reliable DHCP: esp_coex_preference_set(ESP_COEX_PREFER_WIFI), a
 * much slower advertising interval, pausing just the GAP advertisement,
 * and even a full esp_bt_controller_disable() all still lost the DHCP
 * OFFER transmit on every retry once a WiFi station associated. Real
 * porting work should treat BLE and this board's WiFi AP as mutually
 * exclusive by design (BLE for discovery only, fully off during WiFi use)
 * rather than trying to run both concurrently on this chip's single
 * shared 2.4GHz radio. */
static void wifi_ap_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED: mac=%02x:%02x:%02x:%02x:%02x:%02x aid=%d",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED: mac=%02x:%02x:%02x:%02x:%02x:%02x aid=%d",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *evt = (ip_event_ap_staipassigned_t *)event_data;
        ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED: DHCP leased " IPSTR " to mac=%02x:%02x:%02x:%02x:%02x:%02x",
                 IP2STR(&evt->ip), evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
    }
}

static void start_wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "WiFi AP netif create failed");
        return;
    }
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_ap_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_country_t country = { .cc = "01", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_config = {0};
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", WIFI_AP_SSID);
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s", WIFI_AP_PASSWORD);
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.max_connection = 4;
    ap_config.ap.channel = 6;
    /* Same PMF reasoning as JC-ESP32P4-M3: Android's WifiNetworkSpecifier
     * needs PMF-capable or it can silently fail to auto-join. */
    ap_config.ap.pmf_cfg.capable = true;
    ap_config.ap.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* TX power cap tried during the coex investigation and reverted --
     * never proven to help (DHCP still failed with it on), and it's a
     * real deviation from the exact e9ad195 config that was 100% reliable
     * on the first bring-up test. Don't carry unproven changes forward. */

    ESP_LOGI(TAG, "SoftAP up: ssid=%s ip=%s", WIFI_AP_SSID, WIFI_AP_IP);
    start_http_server();
}

static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.name = (const uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
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
    /* Reverted back to NimBLE's plain default interval (itvl_min/max left
     * at 0) -- matches the exact config from the first bring-up commit
     * (e9ad195) that was 100% reliable. A slowed-down interval (800/1600)
     * was tried during the coex investigation and never definitively
     * proven to help (it didn't fix DHCP with BLE active either), so
     * don't carry a deviation from the known-good config without reason. */
    rc = ble_gap_adv_start(s_ble_addr_type, NULL, BLE_HS_FOREVER, &adv, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv start failed: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE advertising started as %s", BLE_DEVICE_NAME);
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

static void start_ble(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "NimBLE init failed: rc=%d", rc);
        return;
    }
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "NimBLE host started (native controller).");
}

/* Just logs every frame it receives -- proves the physical CAN wiring
 * (S3 -> SN65HVD230 -> CANH/CANL -> simulator) works before any real
 * protocol logic gets built on top. No termination resistor on this end
 * yet (see HARDWARE_MIGRATION.md) -- if frames show up reliably anyway,
 * that's still worth knowing, but don't conclude the bus is healthy
 * long-term from that alone; a missing terminator causing reflections
 * can look fine at low traffic/short wire runs and get worse under load. */
static void can_receive_task(void *arg)
{
    (void)arg;
    while (1) {
        twai_message_t message;
        esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            capture_can_frame(&message);
        } else if (err == ESP_ERR_TIMEOUT) {
            twai_status_info_t diag = {0};
            twai_get_status_info(&diag);
            const char *state_str = diag.state == TWAI_STATE_RUNNING ? "RUNNING"
                    : diag.state == TWAI_STATE_BUS_OFF ? "BUS_OFF"
                    : diag.state == TWAI_STATE_STOPPED ? "STOPPED" : "RECOVERING";
            ESP_LOGI(TAG, "CAN RX: no frames -- state=%s tx_err=%lu rx_err=%lu "
                     "bus_err_count=%lu arb_lost=%lu tx_failed=%lu rx_missed=%lu rx_overrun=%lu",
                     state_str, (unsigned long)diag.tx_error_counter, (unsigned long)diag.rx_error_counter,
                     (unsigned long)diag.bus_error_count, (unsigned long)diag.arb_lost_count,
                     (unsigned long)diag.tx_failed_count, (unsigned long)diag.rx_missed_count,
                     (unsigned long)diag.rx_overrun_count);
        } else {
            ESP_LOGW(TAG, "twai_receive error: %s", esp_err_to_name(err));
        }

        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
            ESP_LOGE(TAG, "CAN controller is BUS-OFF (too many errors) -- recovering...");
            twai_initiate_recovery();
            vTaskDelay(pdMS_TO_TICKS(500));
            twai_start();
        }
    }
}

/* NMEA parsing below is ported verbatim from JC-ESP32P4-M3's
 * nmea_coord_to_decimal/nmea_split_fields/nmea_parse_rmc/nmea_parse_gga/
 * nmea_parse_line -- see that file for the field-index doc comments this
 * was copied from. Confirmed against the real simulator earlier tonight
 * (raw $GPRMC sentences with correct checksums received cleanly on
 * GPIO7); this adds the parsing on top of that already-proven wiring. */
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
            ESP_LOGI(TAG, "GPS fix -> lat=%.6f lon=%.6f speed=%.1f km/h heading=%.1f deg",
                     lat, lon, (double)speed_kmh, (double)heading_deg);
        } else {
            ESP_LOGI(TAG, "GPS -> waiting for valid fix");
        }
    }
}

static void nmea_parse_gga(char **fields, size_t count)
{
    if (count < 8) {
        return;
    }
    portENTER_CRITICAL(&s_gps_lock);
    s_gps_state.satellites = (uint8_t)atoi(fields[7]);
    portEXIT_CRITICAL(&s_gps_lock);
}

static void nmea_parse_line(char *line)
{
    if (line[0] != '$') {
        return;
    }
    char *star = strchr(line, '*');
    if (star == NULL || strlen(star) < 3) {
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

/* Reassembles raw UART bytes into '\n'-terminated NMEA lines before
 * parsing -- sentences are short so a single-line buffer is enough. */
static void gps_receive_task(void *arg)
{
    (void)arg;
    static char line_buf[96];
    size_t line_len = 0;
    uint8_t rx_buf[64];

    while (1) {
        int read = uart_read_bytes(GPS_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(1000));
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
            }
        }
    }
}

static void start_gps(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_GPIO, GPS_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "GPS UART ready: TX=GPIO%d RX=GPIO%d %d baud", GPS_TX_GPIO, GPS_RX_GPIO, GPS_BAUD);
    xTaskCreatePinnedToCore(gps_receive_task, "gps_rx", 4096, NULL, 5, NULL, 1);
}

static void start_can(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "CAN ready: TX=GPIO%d RX=GPIO%d 500kbps, free heap=%lu (internal=%lu)",
             CAN_TX_GPIO, CAN_RX_GPIO, (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    /* 2026-09-18 morning: after adding GPS/CAN-capture/HTTP, one boot showed
     * "CAN ready" print but then NEVER logged a single can_receive_task
     * line again (not even a "no frames" timeout, which normally fires
     * every ~1s) -- looks like this xTaskCreate silently failed (returned
     * pdFAIL, task never actually runs) rather than a wiring problem, since
     * a genuinely running receive task logs *something* every second
     * regardless of what's on the wire. Checking the return value and
     * free heap here to catch that precisely instead of guessing. */
    BaseType_t created = xTaskCreatePinnedToCore(can_receive_task, "can_rx", 4096, NULL, 5, NULL, 1);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "CAN: xTaskCreate FAILED (result=%d) -- can_receive_task never started. "
                 "Free heap=%lu internal=%lu", (int)created,
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
}

/* GPS temporarily disabled -- confirmed CAN+GPS together (not either
 * alone) break Wi-Fi/BLE reliability, root cause not yet found (moving
 * both to CPU1 didn't fix it either). Keeping the known-good CAN-only
 * config live so the app's own Monitor tab can be verified end-to-end
 * over real Wi-Fi while that's investigated further. Re-enable
 * start_gps() once the actual conflict is found and fixed. */
static void sensors_init_task(void *arg)
{
    SemaphoreHandle_t done = (SemaphoreHandle_t)arg;
    start_can();
    // start_gps();
    xSemaphoreGive(done);
    vTaskDelete(NULL);
}

static void sensors_init_on_core1(void)
{
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(sensors_init_task, "sensors_init", 4096, done, 5, NULL, 1);
    xSemaphoreTake(done, portMAX_DELAY);
    vSemaphoreDelete(done);
}

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "JC-ESP32S3-CAN bring-up: cores=%d rev=v%d.%d flash=%luMB psram=%s",
             chip_info.cores, chip_info.revision / 100, chip_info.revision % 100,
             (unsigned long)(flash_size / (1024 * 1024)),
#if CONFIG_SPIRAM
             "enabled"
#else
             "NOT enabled (check sdkconfig)"
#endif
    );

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_CANDIDATE,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip init on GPIO%d failed: %s", LED_GPIO_CANDIDATE, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "led_strip init OK on GPIO%d", LED_GPIO_CANDIDATE);
    }

    /* CAN+GPS together (not either alone) intermittently break Wi-Fi/BLE
     * join reliability -- see HARDWARE_MIGRATION.md's corrected DHCP
     * resolution. Leading theory being tested here: WiFi and BLE are both
     * pinned to CPU0 (CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0,
     * CONFIG_BT_NIMBLE_PINNED_TO_CORE_0), and app_main() itself runs
     * pinned to CPU0 too (CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0) -- so
     * calling start_can()/start_gps() directly here puts the TWAI and
     * UART drivers' esp_intr_alloc()'d ISRs on the SAME core as WiFi/BLE's
     * own timing-critical interrupts. Running their init from a task
     * pinned to CPU1 instead should move those ISRs off CPU0 entirely,
     * leaving it free for WiFi/BLE. */
    sensors_init_on_core1();
    start_wifi_ap();
    start_ble();

    ESP_LOGI(TAG, "Blinking LED at the app-controlled rate (default %lu ms half-period) -- "
             "connect with the CarTheftGuard app and adjust it from the Control tab.",
             (unsigned long)s_blink_half_period_ms);

    bool on = false;
    while (1) {
        if (s_led != NULL) {
            if (on) {
                led_strip_set_pixel(s_led, 0, 24, 24, 24);
            } else {
                led_strip_clear(s_led);
            }
            led_strip_refresh(s_led);
        }
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(s_blink_half_period_ms));
    }
}
