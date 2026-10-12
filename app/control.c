#include "control.h"
#include "motor.h"

/* 左侧为电机 0、2，右侧为电机 1、3；只更新变化的输出。 */
static void car_set_sides_pwm(int16_t left_duty, int16_t right_duty)
{
    static int16_t last_left_duty = MOTOR_PWM_MAX + 1;
    static int16_t last_right_duty = MOTOR_PWM_MAX + 1;

    if (left_duty != last_left_duty) {
        motor_set_pwm(CAR_MOTOR_FRONT_LEFT_ID, left_duty);
        motor_set_pwm(CAR_MOTOR_REAR_LEFT_ID, left_duty);
        last_left_duty = left_duty;
    }

    if (right_duty != last_right_duty) {
        motor_set_pwm(CAR_MOTOR_FRONT_RIGHT_ID, right_duty);
        motor_set_pwm(CAR_MOTOR_REAR_RIGHT_ID, right_duty);
        last_right_duty = right_duty;
    }
}

void car_forward(uint8_t duty)
{
    car_set_sides_pwm((int16_t)duty, (int16_t)duty);
}

void car_backward(uint8_t duty)
{
    car_set_sides_pwm(-(int16_t)duty, -(int16_t)duty);
}

void car_stop(void)
{
    car_set_sides_pwm(0, 0);
}

void car_control_update(car_remote_t remote)
{
    uint8_t duty;
    int16_t forward_duty = 0;
    int16_t turn_duty;

    /* 遥控数据失效或两个轴均回中时停车。 */
    if (remote.valid == 0U ||
        (remote.left_y == 0 && remote.left_x == 0)) {
        car_stop();
        return;
    }

    /* 前后保持原有固定档位；L2 只切换档位，不累加速度。 */
    duty = (remote.l2 != 0U) ? CAR_L2_PWM : CAR_NORMAL_PWM;

    if (remote.left_y < 0) {
        forward_duty = (int16_t)duty;
    } else if (remote.left_y > 0) {
        forward_duty = -(int16_t)duty;
    }

    /* 向左拨动 left_x 为负，turn_duty 为正，右轮快于左轮。
     * 左右拨动越大，左右轮的差速越大；仅拨左右则原地转向。
     */
    turn_duty = (int16_t)((-(int32_t)remote.left_x * duty) / 127);
    car_set_sides_pwm(forward_duty - turn_duty,
                      forward_duty + turn_duty);
}
