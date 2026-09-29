#include "pid.h"
#include <stddef.h>

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->output = 0.0f;
    pid->last_error = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = 0U;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_output)
{
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = (max_output > 0.0f) ? max_output : 0.0f;
    pid_reset(pid);
}

float pid_update(pid_controller_t *pid, float target, float actual,
                 float dt_s)
{
    float error;
    float change;

    if (pid == NULL) {
        return 0.0f;
    }
    if (dt_s <= 0.0f) {
        return pid->output;
    }

    error = target - actual;
    change = pid->ki * error * dt_s;

    if (pid->has_previous_error != 0U) {
        change += pid->kp * (error - pid->last_error);
        change += pid->kd *
                  (error - 2.0f * pid->last_error + pid->previous_error)
                  / dt_s;
        pid->previous_error = pid->last_error;
    } else {
        change += pid->kp * error;
        pid->previous_error = error;
    }

    pid->output += change;
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    pid->last_error = error;
    pid->has_previous_error = 1U;
    return pid->output;
}
