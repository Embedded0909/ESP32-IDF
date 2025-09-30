#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
        .atten = ADC_ATTEN_DB_12,         
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);

    int value;
    while (1)
    {
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &value);
        printf("ADC1_CH0 (GPIO36) value: %d\n", value);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
