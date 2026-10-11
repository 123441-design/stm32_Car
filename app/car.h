#ifndef CAR_H
#define CAR_H

#include <stdint.h>

/* 小车使用的手柄状态；由 remote.c 在收到完整数据帧后更新。 */
typedef struct {
    int16_t left_x;       /* 左摇杆左右：中位 0，向左为负。 */
    int16_t left_y;       /* 左摇杆上下：中位 0，向上为负。 */
    int16_t right_x;      /* 右摇杆左右：中位 0，向左为负。 */
    int16_t right_y;      /* 右摇杆上下：中位 0，向上为负。 */
    uint8_t y;
    uint8_t a;
    uint8_t x;
    uint8_t b;
    uint8_t l1;
    uint8_t l2;
    uint8_t r1;
    uint8_t r2;
    uint8_t select;
    uint8_t start;
    uint8_t up;           /* 左侧方向键。 */
    uint8_t down;
    uint8_t left;
    uint8_t right;
    uint8_t valid;       /* 1：手柄数据有效；0：无数据或已超时。 */
} car_remote_t;

/* 可在 Keil Watch 中查看解析后的摇杆和按键状态。 */
extern volatile car_remote_t car_remote;

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
