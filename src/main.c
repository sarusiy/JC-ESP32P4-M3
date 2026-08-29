#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_hosted.h"
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

#ifndef LED_GPIO
#define LED_GPIO 28
#endif

#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

/* Half-period of the blink, in ms; changed live from the control channel. */
static volatile uint32_t blink_half_period_ms = 500;
static bool wifi_started;
static httpd_handle_t http_server;

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
    snprintf(out_msg, out_msg_len, "OK freq=%ld ms\r\n", value);
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

static void cdc_send(const char *msg)
{
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)msg, strlen(msg));
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

static void send_menu(void)
{
    char menu[220];
    snprintf(menu, sizeof(menu),
             "\r\n=== JC-ESP32P4-M3 control channel ===\r\n"
             "freq <ms>  set blink half-period, %d-%d ms (current: %lu). Example: freq 250\r\n"
             "help       show this menu\r\n",
             BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS,
             (unsigned long)blink_half_period_ms);
    cdc_send(menu);
}

/* Parses "freq <ms>" / "help" typed into the control channel. */
static void handle_command(char *line)
{
    char msg[96];

    if (strcmp(line, "help") == 0) {
        send_menu();
    } else if (strlen(line) > 0) {
        apply_freq_command(line, msg, sizeof(msg));
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
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, response);
    return accepted ? ESP_OK : ESP_FAIL;
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

    if (httpd_start(&http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(http_server, &frequency_uri);
        printf("WiFi frequency API ready: POST /api/frequency\n");
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
        printf("WiFi connected, IP: %s\n", address);
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

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err == ESP_OK) {
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
        printf("ESP-Hosted init failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_hosted_connect_to_slave();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("ESP-Hosted slave connect failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("WiFi netif init failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("WiFi event loop init failed: %s\n", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        printf("WiFi init failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        printf("WiFi set mode failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        printf("WiFi start failed: %s\n", esp_err_to_name(err));
        return;
    }

    wifi_started = true;
    printf("Hosted radio link is up (P4 host, C6 co-processor).\n");
}
#else
static void start_hosted_wifi_link(void)
{
    printf("ESP-Hosted disabled in this build; hosted radio link not started.\n");
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
        return;
    }
    int rc = ble_gatts_notify(conn_handle, s_ble_response_handle);
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
        printf("BLE adv fields failed: rc=%d\n", rc);
        return;
    }

    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_ble_addr_type, NULL, BLE_HS_FOREVER, &adv, ble_gap_event, NULL);
    if (rc != 0) {
        printf("BLE adv start failed: rc=%d\n", rc);
        return;
    }

    printf("BLE advertising started as %s\n", name);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_ble_addr_type);
    if (rc != 0) {
        printf("BLE addr infer failed: rc=%d\n", rc);
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
        printf("NimBLE init failed: rc=%d\n", rc);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        printf("BLE count cfg failed: rc=%d\n", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        printf("BLE add service failed: rc=%d\n", rc);
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    printf("NimBLE host started (controller on C6 over hosted link).\n");
}
#else
static void start_ble_hosted(void)
{
    printf("BLE disabled in this build; hosted BLE not started.\n");
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

    printf("JC-ESP32P4-M3 booted. Blinking LED on GPIO %d\n", LED_GPIO);

    start_control_channel();
    start_hosted_wifi_link();
    start_ble_hosted();

    uint32_t count = 0;
    while (1) {
        uint32_t half_period = blink_half_period_ms;

        gpio_set_level(LED_GPIO, 1);
        printf("ON\n");
        vTaskDelay(pdMS_TO_TICKS(half_period));

        gpio_set_level(LED_GPIO, 0);
        printf("OFF\n");
        vTaskDelay(pdMS_TO_TICKS(half_period));

        count++;
    }
}
