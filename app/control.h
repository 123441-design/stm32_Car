#ifndef CONTROL_H
#define CONTROL_H

#include "car.h"

/* 开环测试用的两个 PWM 档位，单位为百分比（0～100）。 */
#define CAR_NORMAL_PWM 30U
#define CAR_L2_PWM     50U

/* 电机编号与车轮位置的实测对应关系。 */
#define CAR_MOTOR_FRONT_LEFT_ID  0U
#define CAR_MOTOR_REAR_RIGHT_ID  1U
#define CAR_MOTOR_REAR_LEFT_ID   2U
#define CAR_MOTOR_FRONT_RIGHT_ID 3U

/* 四个电机的正 PWM 均已校准为小车前进方向。 */
void car_forward(uint8_t duty);
void car_backward(uint8_t duty);
void car_stop(void);

/* 左摇杆前后决定行驶方向，左右幅度决定转向速度；L2 选择高速档。 */
void car_control_update(car_remote_t remote);

#endif /* CONTROL_H */
