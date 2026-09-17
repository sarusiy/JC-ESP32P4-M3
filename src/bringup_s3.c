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
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "bringup";

#define LED_GPIO_CANDIDATE 48
#define WIFI_AP_SSID "CarTheftGuard-P4"
#define WIFI_AP_PASSWORD "&Car1310"
#define WIFI_AP_IP "192.168.4.1"
#define BLE_DEVICE_NAME "JC-P4-C6"
#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

static led_strip_handle_t s_led;
static volatile uint32_t s_blink_half_period_ms = 500;
static uint8_t s_ble_addr_type = 0;

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

static void start_frequency_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
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
    httpd_register_uri_handler(server, &frequency_uri);
    ESP_LOGI(TAG, "HTTP API ready: POST /api/frequency (\"freq <ms>\")");
}

static void start_wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "WiFi AP netif create failed");
        return;
    }
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

    ESP_LOGI(TAG, "SoftAP up: ssid=%s ip=%s", WIFI_AP_SSID, WIFI_AP_IP);
    start_frequency_http_server();
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
