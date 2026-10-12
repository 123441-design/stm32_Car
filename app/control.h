#ifndef CONTROL_H
#define CONTROL_H

#include "car.h"

/* 摇杆推到底时的 PWM 上限，单位为百分比。 */
#define CAR_NORMAL_PWM 50U
#define CAR_L2_PWM     80U

/* 每 10 ms 最多改变 5 个 PWM 百分点；0 到 100 约需 200 ms。 */
#define CAR_PWM_RAMP_INTERVAL_MS 10U
#define CAR_PWM_RAMP_STEP        5

/* 电机编号与车轮位置的实测对应关系。 */
#define CAR_MOTOR_FRONT_LEFT_ID  0U
#define CAR_MOTOR_REAR_RIGHT_ID  1U
#define CAR_MOTOR_REAR_LEFT_ID   2U
#define CAR_MOTOR_FRONT_RIGHT_ID 3U

/* 四个电机的正 PWM 均已校准为小车前进方向。 */
void car_forward(uint8_t duty);
void car_backward(uint8_t duty);
void car_stop(void);

/* 左摇杆两轴按幅度控制前后和转向；L2 提高上限，PWM 平滑变化。 */
void car_control_update(car_remote_t remote);

#endif /* CONTROL_H */
