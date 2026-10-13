#ifndef CAR_ENCODER_H
#define CAR_ENCODER_H

#include <stdint.h>
#include "car.h"

/* 电机与车轮参数：333 rpm 指减速器输出轴的标称转速。 */
#define CAR_MOTOR_RATED_OUTPUT_RPM 333.0f
#define CAR_WHEEL_DIAMETER_M       0.080f
/* 小车机械参数：编码器位于电机轴，车轮位于减速器输出轴。 */
#define ENCODER_LINES_PER_MOTOR_REV  11.0f
#define ENCODER_COUNT_MULTIPLIER     4.0f
#define MOTOR_GEAR_RATIO             30.0f
#define ENCODER_SAMPLE_PERIOD_S       0.020f


/* 最近一次 20 ms 采样的四轮编码器计数；由编码器任务更新。
 * 下标顺序：左前、右前、左后、右后（见 car_wheel_t）。
 * 正负号目前按示例工程修正，仍需逐轮实测确认。
 */
extern volatile int32_t encoder_wheel_counts[CAR_WHEEL_COUNT];

/* 与上述计数一一对应的四轮线速度，单位 m/s；由编码器任务更新。
 * 可直接在 Keil Watch 中查看 encoder_wheel_speed_mps[0]～[3]。
 */
extern volatile float encoder_wheel_speed_mps[CAR_WHEEL_COUNT];

/* 将 20 ms 采样周期内的有符号计数换算为车轮线速度，单位 m/s。 */
float encoder_counts_to_mps(int32_t counts);

/* 一次换算四个车轮在同一采样周期内的计数。
 * 输入和输出均按 CAR_WHEEL_FRONT_LEFT、FRONT_RIGHT、REAR_LEFT、REAR_RIGHT 排列。
 * 本函数只计算速度，不读取或清零定时器。
 */
void car_encoder_calculate_wheel_speeds(const int32_t counts[CAR_WHEEL_COUNT],
                                        float speeds_mps[CAR_WHEEL_COUNT]);

#endif /* CAR_ENCODER_H */
