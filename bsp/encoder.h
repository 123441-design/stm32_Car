#ifndef ENCODER_H
#define ENCODER_H

#include "tim.h"
#include <stdint.h>

/* Start TIM1, TIM3, TIM4 and TIM8 in encoder mode.
 * Call after their MX_TIMx_Init() functions.
 */
void encoder_start(void);

/* Read signed counts since the previous call, then clear the counter.
 * Pass &htim1, &htim3, &htim4 or &htim8.
 * Read often enough that fewer than 32768 counts occur between calls.
 */
int32_t read_encoder_count(TIM_HandleTypeDef *htim);

#endif /* ENCODER_H */
