#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
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

    if (strncmp(line, "freq ", 5) == 0) {
        char *endptr = NULL;
        long value = strtol(line + 5, &endptr, 10);
        if (endptr == line + 5 || *endptr != '\0' ||
            value < BLINK_HALF_PERIOD_MIN_MS || value > BLINK_HALF_PERIOD_MAX_MS) {
            snprintf(msg, sizeof(msg), "ERR invalid value. Enter a number of ms (%d-%d).\r\n",
                     BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS);
        } else {
            blink_half_period_ms = (uint32_t)value;
            snprintf(msg, sizeof(msg), "OK freq=%ld ms\r\n", value);
        }
        cdc_send(msg);
    } else if (strcmp(line, "help") == 0) {
        send_menu();
    } else if (strlen(line) > 0) {
        snprintf(msg, sizeof(msg), "ERR unknown command '%s'. Type 'help'.\r\n", line);
        cdc_send(msg);
    }
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

void app_main(void)
{
    init_led();

    printf("JC-ESP32P4-M3 booted. Blinking LED on GPIO %d\n", LED_GPIO);

    start_control_channel();

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
