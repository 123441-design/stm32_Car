#ifndef CONTROL_H
#define CONTROL_H

#include "car.h"
#include "car_encoder.h"

/* 标称输出转速 × 车轮周长 ÷ 60，理论轮缘速度约 1.395 m/s。
 * 实车带载可达速度仍需用编码器测量后调整目标上限。
 */
#define CAR_MAX_WHEEL_SPEED_MPS \
    ((CAR_MOTOR_RATED_OUTPUT_RPM * 3.1415926f * CAR_WHEEL_DIAMETER_M) / 60.0f)

/* 摇杆推到底时的目标速度：占理论最高轮速的百分比。 */
#define CAR_NORMAL_SPEED_PERCENT 50U
#define CAR_L2_SPEED_PERCENT     80U

/* 每 10 ms 最多改变 5 个目标速度百分点。 */
#define CAR_SPEED_RAMP_INTERVAL_MS 10U
#define CAR_SPEED_RAMP_STEP_PERCENT 5

/* 电机编号与车轮位置的实测对应关系。 */
#define CAR_MOTOR_FRONT_LEFT_ID  0U
#define CAR_MOTOR_REAR_RIGHT_ID  1U
#define CAR_MOTOR_REAR_LEFT_ID   2U
#define CAR_MOTOR_FRONT_RIGHT_ID 3U

/* 四个电机各自的速度 PID 参数；修改这里后重新编译下载。 */
/* 电机 0：左前。 */
#define CAR_MOTOR_0_PID_KP 500.0f
#define CAR_MOTOR_0_PID_KI 1.0f
#define CAR_MOTOR_0_PID_KD  0.10f

/* 电机 1：右后。 */
#define CAR_MOTOR_1_PID_KP 500.0f
#define CAR_MOTOR_1_PID_KI 1.0f
#define CAR_MOTOR_1_PID_KD  0.10f

/* 电机 2：左后。 */
#define CAR_MOTOR_2_PID_KP 500.0f
#define CAR_MOTOR_2_PID_KI 1.0f
#define CAR_MOTOR_2_PID_KD  0.10f

/* 电机 3：右前。 */
#define CAR_MOTOR_3_PID_KP 500.0f
#define CAR_MOTOR_3_PID_KI 1.0f
#define CAR_MOTOR_3_PID_KD  0.10f

/* 四轮目标线速度，单位 m/s；顺序见 car_wheel_t。
 * 由遥控摇杆指令换算，不随 PID 输出的 PWM 改变。
 */
extern volatile float car_wheel_target_mps[CAR_WHEEL_COUNT];

/* 四个电机的正 PWM 均已校准为小车前进方向。 */
void car_forward(uint8_t duty);
void car_backward(uint8_t duty);
void car_stop(void);

/* 左摇杆控制前后/横移，右摇杆左右控制旋转；L2 提高轮速上限。 */
void car_control_update(car_remote_t remote);

/* 编码器任务启动时初始化一次；每取得一组新的实际轮速后更新一次速度环。 */
void car_speed_control_init(void);
void car_speed_control_update(const volatile float target_mps[CAR_WHEEL_COUNT],
                              const float actual_mps[CAR_WHEEL_COUNT]);

#endif /* CONTROL_H */
