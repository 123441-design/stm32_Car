#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Hardware mapping: change these macros when the wiring changes. */
#define TB6612_1_STBY_PORT GPIOB
#define TB6612_1_STBY_PIN  GPIO_PIN_14
#define TB6612_2_STBY_PORT GPIOC
#define TB6612_2_STBY_PIN  GPIO_PIN_15

#define MOTOR_PWM_TIMER (&htim2)

#define MOTOR_0_PWM_CHANNEL TIM_CHANNEL_4
#define MOTOR_0_IN1_PORT     GPIOB
#define MOTOR_0_IN1_PIN      GPIO_PIN_12
#define MOTOR_0_IN2_PORT     GPIOB
#define MOTOR_0_IN2_PIN      GPIO_PIN_13   

#define MOTOR_1_PWM_CHANNEL TIM_CHANNEL_1
#define MOTOR_1_IN1_PORT     GPIOC
#define MOTOR_1_IN1_PIN      GPIO_PIN_14
#define MOTOR_1_IN2_PORT     GPIOC
#define MOTOR_1_IN2_PIN      GPIO_PIN_13

#define MOTOR_2_PWM_CHANNEL TIM_CHANNEL_3
#define MOTOR_2_IN1_PORT     GPIOC
#define MOTOR_2_IN1_PIN      GPIO_PIN_8
#define MOTOR_2_IN2_PORT     GPIOC
#define MOTOR_2_IN2_PIN      GPIO_PIN_9

#define MOTOR_3_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR_3_IN1_PORT     GPIOC
#define MOTOR_3_IN1_PIN      GPIO_PIN_1
#define MOTOR_3_IN2_PORT     GPIOC
#define MOTOR_3_IN2_PIN      GPIO_PIN_0

#define MOTOR_COUNT   4U
#define MOTOR_PWM_MAX 100

/* Call after MX_GPIO_Init() and MX_TIM2_Init(). */
void motor_init(void);

/* id: 0..3; duty: -100..100 percent. Sign selects direction.
 * Invalid id is ignored; out-of-range duty is clamped.
 * Zero sets PWM to zero and both direction pins low.
 */
void motor_set_pwm(uint8_t id, int16_t duty);

#endif /* MOTOR_H */
