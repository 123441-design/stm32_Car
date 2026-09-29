#ifndef CAR_H
#define CAR_H

/* Wheel indices describe physical locations, not motor or timer numbers. */
typedef enum {
    CAR_WHEEL_FRONT_LEFT = 0,
    CAR_WHEEL_FRONT_RIGHT,
    CAR_WHEEL_REAR_LEFT,
    CAR_WHEEL_REAR_RIGHT,
    CAR_WHEEL_COUNT
} car_wheel_t;

typedef struct {
    float half_length_m;       /* Center to front/rear axle. */
    float half_width_m;        /* Center to left/right wheel. */
    float max_wheel_speed_mps; /* Positive speed limit. */
} car_config_t;

/* X-layout mecanum wheels; +x forward, +y left, +wz counterclockwise.
 * Calculate wheel speed targets in m/s in car_wheel_t order.
 * The caller later applies encoder feedback, PID, and motor PWM.
 * Invalid geometry or speed limit produces four zero targets.
 */
void car_move(const car_config_t *config, float vx_mps, float vy_mps,
              float wz_radps, float wheel_target_mps[CAR_WHEEL_COUNT]);

#endif /* CAR_H */
