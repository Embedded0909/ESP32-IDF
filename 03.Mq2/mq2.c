#include "mq2.h"

MQ2_Status_t mq2_init(MQ2_Typedef *mq2, adc_unit_t unit, adc_channel_t channel)
{
    MQ2_Status_t status = MQ2_OK;
    if (!mq2)
    {
        status = MQ2_ERR;
    }

    mq2->unit = unit;
    mq2->channel = channel;
    mq2->bitwidth = ADC_BITWIDTH_DEFAULT;
    mq2->atten = ADC_ATTEN_DB_12;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = mq2->unit,
    };

    adc_oneshot_new_unit(&init_config, &mq2->adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = mq2->bitwidth,
        .atten = mq2->atten,
    };
    adc_oneshot_config_channel(mq2->adc_handle, mq2->channel, &config);
    printf("[MQ2] Status MQ2 %d \r\n", status);
    return status;
}
MQ2_Status_t mq2_read_voltage(MQ2_Typedef *mq2, float *value)
{
    MQ2_Status_t status;
    status = MQ2_OK;
    if (!mq2)
    {
        status = MQ2_ERR;
        return status;
    }
    int raw;
    adc_oneshot_read(mq2->adc_handle, mq2->channel, &raw);
    *value = (raw * 3.3f) / 4095.0f;
    return status;
}