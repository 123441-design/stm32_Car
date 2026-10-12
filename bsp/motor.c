#include "motor.h"
#include "tim.h"

void motor_init(void)
{
    /* Keep both TB6612 chips in standby while starting the PWM outputs. */
    HAL_GPIO_WritePin(TB6612_1_STBY_PORT, TB6612_1_STBY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TB6612_2_STBY_PORT, TB6612_2_STBY_PIN, GPIO_PIN_RESET);

    motor_set_pwm(0, 0);//左前
    motor_set_pwm(1, 0);//右后
    motor_set_pwm(2, 0);//左后
    motor_set_pwm(3, 0);//右前

    if (HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_0_PWM_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_1_PWM_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_2_PWM_CHANNEL) != HAL_OK ||
        HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_3_PWM_CHANNEL) != HAL_OK) {
        Error_Handler();
        return;
    }

    HAL_GPIO_WritePin(TB6612_1_STBY_PORT, TB6612_1_STBY_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TB6612_2_STBY_PORT, TB6612_2_STBY_PIN, GPIO_PIN_SET);
}

void motor_set_pwm(uint8_t id, int16_t duty)
{
    GPIO_TypeDef *in1_port;
    GPIO_TypeDef *in2_port;
    uint16_t in1_pin;
    uint16_t in2_pin;
    uint32_t channel;
    uint32_t magnitude;
    uint32_t period;

    switch (id) {
    case 0:
        in1_port = MOTOR_0_IN1_PORT; in1_pin = MOTOR_0_IN1_PIN;
        in2_port = MOTOR_0_IN2_PORT; in2_pin = MOTOR_0_IN2_PIN;
        channel = MOTOR_0_PWM_CHANNEL;
        break;
    case 1:
        in1_port = MOTOR_1_IN1_PORT; in1_pin = MOTOR_1_IN1_PIN;
        in2_port = MOTOR_1_IN2_PORT; in2_pin = MOTOR_1_IN2_PIN;
        channel = MOTOR_1_PWM_CHANNEL;
        break;
    case 2:
        in1_port = MOTOR_2_IN1_PORT; in1_pin = MOTOR_2_IN1_PIN;
        in2_port = MOTOR_2_IN2_PORT; in2_pin = MOTOR_2_IN2_PIN;
        channel = MOTOR_2_PWM_CHANNEL;
        break;
    case 3:
        in1_port = MOTOR_3_IN1_PORT; in1_pin = MOTOR_3_IN1_PIN;
        in2_port = MOTOR_3_IN2_PORT; in2_pin = MOTOR_3_IN2_PIN;
        channel = MOTOR_3_PWM_CHANNEL;
        break;
    default:
        return;
    }

    if (duty > MOTOR_PWM_MAX) {
        duty = MOTOR_PWM_MAX;
    } else if (duty < -MOTOR_PWM_MAX) {
        duty = -MOTOR_PWM_MAX;
    }

    magnitude = (uint32_t)((duty < 0) ? -duty : duty);
    period = __HAL_TIM_GET_AUTORELOAD(MOTOR_PWM_TIMER) + 1U;

    /* Set PWM to zero before changing direction. */
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, channel, 0U);
    if (duty > 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    } else if (duty < 0) {
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(MOTOR_PWM_TIMER, channel,
                          (magnitude * period) / MOTOR_PWM_MAX);
}
