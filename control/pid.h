#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output;
    float output;
    float last_error;
    float previous_error;
    uint8_t has_previous_error;
} pid_controller_t;

/* Gains use seconds: ki multiplies error * dt_s, kd divides by dt_s.
 * max_output is the absolute output limit (for PWM percent, use 100).
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_output);
void pid_reset(pid_controller_t *pid);

/* Incremental PID. Call at a regular interval with dt_s > 0.
 * target and actual must use the same unit.
 */
float pid_update(pid_controller_t *pid, float target, float actual,
                 float dt_s);

#endif /* PID_H */
