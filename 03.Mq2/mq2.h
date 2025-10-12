#ifndef _MQ2_
#define _MQ2_

#include "stdint.h"
#include "esp_adc/adc_oneshot.h"

typedef enum
{
    MQ2_OK  = 0,        // Success
    MQ2_ERR = 1         // Error
} MQ2_Status_t;

/**
 * Structure containing MQ2 sensor configuration and readings
 */
typedef struct
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t channel;
    adc_unit_t unit;
    adc_bitwidth_t bitwidth;
    adc_atten_t atten;
} MQ2_Typedef;

/**
 * @brief Initialize MQ2 sensor
 *
 * This function initializes the ADC unit and channel used by the MQ2 gas sensor.
 *
 * @param mq2 Pointer to MQ2_Typedef structure instance
 * @param unit ADC unit to use (ADC_UNIT_1 or ADC_UNIT_2)
 * @param channel ADC channel (e.g. ADC_CHANNEL_0)
 * @return MQ2_Status_t MQ2_OK if successful, otherwise error code
 */
MQ2_Status_t mq2_init(MQ2_Typedef *mq2, adc_unit_t unit, adc_channel_t channel);
MQ2_Status_t mq2_read_voltage(MQ2_Typedef *mq2, float *value);

#endif