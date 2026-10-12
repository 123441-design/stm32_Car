#include "control.h"
#include "motor.h"
#include "stm32f1xx_hal.h"

static int16_t current_left_duty;
static int16_t current_right_duty;
static uint8_t pwm_has_been_written;

/* 左侧为电机 0、2，右侧为电机 1、3；只更新变化的输出。 */
static void car_set_sides_pwm(int16_t left_duty, int16_t right_duty)
{
    if (pwm_has_been_written == 0U || left_duty != current_left_duty) {
        motor_set_pwm(CAR_MOTOR_FRONT_LEFT_ID, left_duty);
        motor_set_pwm(CAR_MOTOR_REAR_LEFT_ID, left_duty);
    }

    if (pwm_has_been_written == 0U || right_duty != current_right_duty) {
        motor_set_pwm(CAR_MOTOR_FRONT_RIGHT_ID, right_duty);
        motor_set_pwm(CAR_MOTOR_REAR_RIGHT_ID, right_duty);
    }

    current_left_duty = left_duty;
    current_right_duty = right_duty;
    pwm_has_been_written = 1U;
}

/* 摇杆原始范围约为 -128～127，映射到当前档位的 PWM 上限。 */
static int16_t car_axis_to_duty(int16_t axis, uint8_t max_pwm)
{
    int32_t duty = (-(int32_t)axis * max_pwm) / 127;

    if (duty > max_pwm) {
        duty = max_pwm;
    } else if (duty < -(int32_t)max_pwm) {
        duty = -(int32_t)max_pwm;
    }
    return (int16_t)duty;
}

/* 左右轮混合后若超出档位上限，保持比例一起缩小。 */
static void car_limit_sides(int16_t *left, int16_t *right, uint8_t max_pwm)
{
    int32_t left_abs = (*left < 0) ? -(int32_t)*left : *left;
    int32_t right_abs = (*right < 0) ? -(int32_t)*right : *right;
    int32_t largest = (left_abs > right_abs) ? left_abs : right_abs;

    if (largest > max_pwm) {
        *left = (int16_t)(((int32_t)*left * max_pwm) / largest);
        *right = (int16_t)(((int32_t)*right * max_pwm) / largest);
    }
}

static int16_t car_ramp_duty(int16_t current, int16_t target)
{
    if (target > current + CAR_PWM_RAMP_STEP) {
        return current + CAR_PWM_RAMP_STEP;
    }
    if (target < current - CAR_PWM_RAMP_STEP) {
        return current - CAR_PWM_RAMP_STEP;
    }
    return target;
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
    static uint32_t last_ramp_tick;
    uint32_t now = HAL_GetTick();
    uint8_t max_pwm;
    int16_t forward_duty;
    int16_t turn_duty;
    int16_t left_target;
    int16_t right_target;

    /* 遥控失联直接停车，不等待斜坡减速。 */
    if (remote.valid == 0U) {
        car_stop();
        last_ramp_tick = now;
        return;
    }

    max_pwm = (remote.l2 != 0U) ? CAR_L2_PWM : CAR_NORMAL_PWM;
    forward_duty = car_axis_to_duty(remote.left_y, max_pwm);
    turn_duty = car_axis_to_duty(remote.left_x, max_pwm);

    /* 向左拨动时 turn_duty 为正，右轮比左轮快。 */
    left_target = forward_duty - turn_duty;
    right_target = forward_duty + turn_duty;
    car_limit_sides(&left_target, &right_target, max_pwm);

    if ((uint32_t)(now - last_ramp_tick) < CAR_PWM_RAMP_INTERVAL_MS) 
    {
        return;
    }
    last_ramp_tick = now;
    car_set_sides_pwm(car_ramp_duty(current_left_duty, left_target),
                      car_ramp_duty(current_right_duty, right_target));
}
