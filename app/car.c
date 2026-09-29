#include "car.h"
#include <stddef.h>

static float car_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void car_move(const car_config_t *config, float vx_mps, float vy_mps,
              float wz_radps, float wheel_target_mps[CAR_WHEEL_COUNT])
{
    float turn;
    float max_abs = 0.0f;
    float scale;
    int i;

    if (wheel_target_mps == NULL) {
        return;
    }
    if (config == NULL || config->half_length_m <= 0.0f ||
        config->half_width_m <= 0.0f || config->max_wheel_speed_mps <= 0.0f) {
        for (i = 0; i < CAR_WHEEL_COUNT; i++) {
            wheel_target_mps[i] = 0.0f;
        }
        return;
    }

    turn = (config->half_length_m + config->half_width_m) * wz_radps;
    wheel_target_mps[CAR_WHEEL_FRONT_LEFT]  = vx_mps - vy_mps - turn;
    wheel_target_mps[CAR_WHEEL_FRONT_RIGHT] = vx_mps + vy_mps + turn;
    wheel_target_mps[CAR_WHEEL_REAR_LEFT]   = vx_mps + vy_mps - turn;
    wheel_target_mps[CAR_WHEEL_REAR_RIGHT]  = vx_mps - vy_mps + turn;

    for (i = 0; i < CAR_WHEEL_COUNT; i++) {
        float absolute_speed = car_abs(wheel_target_mps[i]);
        if (absolute_speed > max_abs) {
            max_abs = absolute_speed;
        }
    }
    if (max_abs > config->max_wheel_speed_mps) {
        scale = config->max_wheel_speed_mps / max_abs;
        for (i = 0; i < CAR_WHEEL_COUNT; i++) {
            wheel_target_mps[i] *= scale;
        }
    }
}
