#include "encoder.h"

void encoder_start(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);

    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
}

int32_t read_encoder_count(TIM_HandleTypeDef *htim)
{
    uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0U);

    /* A 16-bit timer counts downward from 0 through 65535. */
    if (count < 0x8000U) {
        return (int32_t)count;
    }
    return (int32_t)count - 65536;
}
