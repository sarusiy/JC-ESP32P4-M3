/* Bring-up firmware for the new ESP32-S3 N16R8 board -- NOT the real
 * application yet (see HARDWARE_MIGRATION.md). Just enough to answer two
 * concrete open questions before wiring anything to the CAN transceiver:
 *   1. Which of the board's two USB-C ports actually gives a working
 *      flash/monitor connection (native USB-Serial-JTAG vs the onboard
 *      FTDI FT232RQ) -- whichever port this was flashed/monitored through
 *      is the answer.
 *   2. Which GPIO drives the onboard WS2812 RGB LED -- cycles it through
 *      red/green/blue/off on GPIO48 (the pin the official Espressif
 *      ESP32-S3-DevKitC-1 v1.1 reference design uses for this, and this
 *      board's silkscreen breaks 48 out as a plain GPIO, so it's the most
 *      likely candidate for a board that copies that reference layout).
 *      If nothing lights up, the LED is on a different, not-yet-identified
 *      pin -- harmless either way, this doesn't block CAN/GPS wiring.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "led_strip.h"

static const char *TAG = "bringup";

#define LED_GPIO_CANDIDATE 48

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
    ESP_LOGI(TAG, "If you can read this over a serial monitor, THIS USB-C port works for flash+monitor.");

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_CANDIDATE,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    led_strip_handle_t led = NULL;
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip init on GPIO%d failed: %s -- LED pin guess is wrong or GPIO busy",
                 LED_GPIO_CANDIDATE, esp_err_to_name(err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            ESP_LOGI(TAG, "still alive, no LED test possible on GPIO%d", LED_GPIO_CANDIDATE);
        }
    }
    ESP_LOGI(TAG, "led_strip init OK on GPIO%d -- watch the onboard LED: "
             "red -> green -> blue -> off, repeating", LED_GPIO_CANDIDATE);

    int step = 0;
    while (1) {
        switch (step % 4) {
            case 0: led_strip_set_pixel(led, 0, 32, 0, 0); ESP_LOGI(TAG, "LED: red"); break;
            case 1: led_strip_set_pixel(led, 0, 0, 32, 0); ESP_LOGI(TAG, "LED: green"); break;
            case 2: led_strip_set_pixel(led, 0, 0, 0, 32); ESP_LOGI(TAG, "LED: blue"); break;
            case 3: led_strip_clear(led); ESP_LOGI(TAG, "LED: off"); break;
        }
        led_strip_refresh(led);
        step++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
