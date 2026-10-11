#include "control.h"
#include "motor.h"

/* 给四个电机下发相同的带符号 PWM。 */
static void car_set_all_pwm(int16_t duty)
{
    /* 遥控任务每 1 ms 调用一次；指令没变时不重复清零并设置 PWM。 */
    static int16_t last_duty = MOTOR_PWM_MAX + 1;
    uint8_t id;

    if (duty == last_duty) {
        return;
    }

    for (id = 0U; id < MOTOR_COUNT; ++id) {
        motor_set_pwm(id, duty);
    }
    last_duty = duty;
}

void car_forward(uint8_t duty)
{
    car_set_all_pwm((int16_t)duty);
}

void car_backward(uint8_t duty)
{
    car_set_all_pwm(-(int16_t)duty);
}

void car_stop(void)
{
    car_set_all_pwm(0);
}

void car_control_update(car_remote_t remote)
{
    uint8_t duty;

    /* 遥控数据失效或摇杆回中时，立即给四个电机下发零 PWM。 */
    if (remote.valid == 0U || remote.left_y == 0) 
    {
        car_stop();
        return;
    }

    /* L2 只是选择较高档位，不随任务循环次数累加速度。 */
    duty = (remote.l2 != 0U) ? CAR_L2_PWM : CAR_NORMAL_PWM;

    if (remote.left_y < 0) 
    {
        car_forward(duty);
    } 
     else 
    {
        car_backward(duty);
    }
}
