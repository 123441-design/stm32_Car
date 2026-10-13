#ifndef CAR_H
#define CAR_H

#include <stdint.h>

/* 从车顶向下看，四轮滚子轴线的延长线形成 X 或 O。当前实车为 X 型。 */
#define CAR_MECANUM_LAYOUT_X 1U
#define CAR_MECANUM_LAYOUT_O 2U

/* 默认使用 X 型布局,可选择 */
#ifndef CAR_MECANUM_LAYOUT
#define CAR_MECANUM_LAYOUT CAR_MECANUM_LAYOUT_X
#endif

/* 轮心间距实测：前后 120 mm、左右 160 mm。
 * O 型转向计算使用车体中心到轮心的半轴距和半轮距。
 */
#ifndef CAR_MECANUM_HALF_LENGTH_MM
#define CAR_MECANUM_HALF_LENGTH_MM 60
#endif
#ifndef CAR_MECANUM_HALF_WIDTH_MM
#define CAR_MECANUM_HALF_WIDTH_MM 80
#endif

#if CAR_MECANUM_LAYOUT != CAR_MECANUM_LAYOUT_X && \
    CAR_MECANUM_LAYOUT != CAR_MECANUM_LAYOUT_O
#error "CAR_MECANUM_LAYOUT must be X or O"
#endif

#if CAR_MECANUM_LAYOUT == CAR_MECANUM_LAYOUT_O && \
    (CAR_MECANUM_HALF_LENGTH_MM <= 0 || CAR_MECANUM_HALF_WIDTH_MM <= 0 || \
     CAR_MECANUM_HALF_LENGTH_MM == CAR_MECANUM_HALF_WIDTH_MM)
#error "O layout needs measured, unequal half-length and half-width for turning"
#endif

/* 小车使用的手柄状态；由 remote.c 在收到完整数据帧后更新。 */
typedef struct 
{
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

/* 车身坐标系混合：正值分别为前进、向左横移、向左旋转。
 * 输入与输出均为相对轮速百分比，输出顺序见 car_wheel_t。
 * 合成值超限时四轮按相同比例缩小。
 */
void car_mix_mecanum_percent(int16_t forward, int16_t left, int16_t turn,
                             uint8_t max_percent,
                             int16_t wheel_percent[CAR_WHEEL_COUNT]);

typedef struct 
{
    float half_length_m;       /* Center to front/rear axle. */
    float half_width_m;        /* Center to left/right wheel. */
    float max_wheel_speed_mps; /* Positive speed limit. */
} car_config_t;

/* 预留的物理速度接口，当前尚未实现；现有遥控运动使用上面的
 * car_mix_mecanum_percent() 生成四轮目标值。
 */
void car_move(const car_config_t *config, float vx_mps, float vy_mps,
              float wz_radps, float wheel_target_mps[CAR_WHEEL_COUNT]);

#endif /* CAR_H */
