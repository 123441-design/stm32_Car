#ifndef CONTROL_H
#define CONTROL_H

#include "car.h"

/* 开环测试用的两个 PWM 档位，单位为百分比（0～100）。 */
#define CAR_NORMAL_PWM 30U
#define CAR_L2_PWM     50U

/* 四个电机的正 PWM 均已校准为小车前进方向。 */
void car_forward(uint8_t duty);
void car_backward(uint8_t duty);
void car_stop(void);

/* 根据一份遥控状态控制前进、后退或停车。 */
void car_control_update(car_remote_t remote);

#endif /* CONTROL_H */
