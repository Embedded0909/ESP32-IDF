#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

void app_main(void)
{
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,         // Disable interrupt
        .mode = GPIO_MODE_INPUT,                // Input
        .pin_bit_mask = (1ULL << GPIO_NUM_4),   // GPIO4
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // No pull-down
        .pull_up_en = GPIO_PULLUP_ENABLE,       // pull-up
    };
    gpio_config(&io_conf);
    while (1)
    {
        gpio_set_level(2, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(2, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
