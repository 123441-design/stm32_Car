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

/* Convert counts measured over sample_period_s into wheel speed (m/s).
 * counts_per_wheel_rev includes encoder resolution, quadrature factor,
 * and gear ratio. A negative count gives a negative speed.
 * Returns 0 when a physical parameter is invalid.
 */
float encoder_counts_to_mps(int32_t counts, float counts_per_wheel_rev,
                            float wheel_diameter_m, float sample_period_s);

#endif /* ENCODER_H */
