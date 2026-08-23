#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#ifndef LED_GPIO
#define LED_GPIO 2
#endif

#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

/* Half-period of the blink, in ms; changed live from the serial console. */
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

/* Reads blink half-period updates (ms) typed into the serial monitor. */
static void console_task(void *arg)
{
    (void)arg;
    char digits[16];
    size_t len = 0;

    /* stdin is fully buffered by default; disable that so getchar() sees
       each byte immediately instead of waiting for the buffer to fill. */
    setvbuf(stdin, NULL, _IONBF, 0);

    printf("Type a number (%d-%d) + Enter to set the blink half-period in ms.\n",
           BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS);

    while (1) {
        int c = getchar();

        if (c == '\r' || c == '\n') {
            if (len == 0) {
                continue;
            }
            digits[len] = '\0';
            len = 0;

            char *endptr = NULL;
            long value = strtol(digits, &endptr, 10);
            if (endptr == digits || value < BLINK_HALF_PERIOD_MIN_MS || value > BLINK_HALF_PERIOD_MAX_MS) {
                printf("Invalid input. Enter a number of ms (%d-%d).\n",
                       BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS);
                continue;
            }

            blink_half_period_ms = (uint32_t)value;
            printf("Blink half-period set to %ld ms\n", value);
        } else if (c >= '0' && c <= '9' && len < sizeof(digits) - 1) {
            digits[len++] = (char)c;
        } else if (c < 0) {
            /* No input available right now; avoid busy-looping. */
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

void app_main(void)
{
    init_led();

    printf("JC-ESP32P4-M3 booted. Blinking LED on GPIO %d\n", LED_GPIO);

    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

    uint32_t count = 0;
    while (1) {
        uint32_t half_period = blink_half_period_ms;

        gpio_set_level(LED_GPIO, 1);
        printf("LED ON  (tick %lu)\n", (unsigned long)count);
        vTaskDelay(pdMS_TO_TICKS(half_period));

        gpio_set_level(LED_GPIO, 0);
        printf("LED OFF (tick %lu)\n", (unsigned long)count);
        vTaskDelay(pdMS_TO_TICKS(half_period));

        count++;
    }
}
