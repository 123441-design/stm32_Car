#include "pid.h"
#include <stddef.h>

/* 只清除运行状态，保留 Kp、Ki、Kd 和输出上限等配置。 */
void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->output = 0.0f;
    pid->integral = 0.0f; /* 位置式 PID 的积分也要一起清零。 */
    pid->last_error = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = 0U;
}

/* 设置 PID 参数，并从零输出、零误差开始运行。 */
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

/* 增量式 PID：先算本周期输出变化量，再加到上一次输出上。
 * target 和 actual 单位相同；dt_s 是两次调用的间隔，单位秒。
 */
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

    /* 误差 = 目标值 - 实际值；积分项每周期贡献 Ki × 误差 × 时间。 */
    error = target - actual;
    change = pid->ki * error * dt_s;

    /* P 看误差相对上次的变化；D 看误差变化量又变化了多少。 */
    if (pid->has_previous_error != 0U) {
        change += pid->kp * (error - pid->last_error);
        change += pid->kd *
                  (error - 2.0f * pid->last_error + pid->previous_error)
                  / dt_s;
        pid->previous_error = pid->last_error;
    } else {
        /* 第一次调用还没有历史误差，先用当前误差建立初始比例输出。 */
        change += pid->kp * error;
        pid->previous_error = error;
    }

    /* 本次输出 = 上次输出 + 变化量，并限制在允许范围内。 */
    pid->output += change;
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    /* 保存当前误差，下一次调用时作为历史误差。 */
    pid->last_error = error;
    pid->has_previous_error = 1U;
    return pid->output;
}

/* 位置式 PID：每次直接计算本周期应输出的 PWM，而不是计算 PWM 增量。 */
float pid_update_position(pid_controller_t *pid, float target, float actual,
                          float dt_s)
{
    float error;
    float proportional;
    float derivative = 0.0f;
    float next_integral;
    float next_output;

    if (pid == NULL) {
        return 0.0f;
    }
    if (dt_s <= 0.0f) {
        return pid->output;
    }

    /* P：当前误差越大，比例项给出的输出越大。 */
    error = target - actual;
    proportional = pid->kp * error;
    /* D：首次没有上一周期误差，不计算微分。 */
    if (pid->has_previous_error != 0U) {
        derivative = pid->kd * (error - pid->last_error) / dt_s;
    }

    /* I：将本周期的误差乘以采样时间后累加，并限制积分大小。 */
    next_integral = pid->integral + pid->ki * error * dt_s;
    if (next_integral > pid->max_output) {
        next_integral = pid->max_output;
    } else if (next_integral < -pid->max_output) {
        next_integral = -pid->max_output;
    }

    next_output = proportional + next_integral + derivative;
    /* 若继续积分只会使输出超过上限，就暂不接受本周期的积分。 */
    if (!((next_output > pid->max_output && error > 0.0f) ||
          (next_output < -pid->max_output && error < 0.0f))) {
        pid->integral = next_integral;
    }

    /* 三项相加得到本次输出，再限制为电机允许的 PWM 范围。 */
    pid->output = proportional + pid->integral + derivative;
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    /* 保存误差，供下一周期计算微分。 */
    pid->last_error = error;
    pid->has_previous_error = 1U;
    return pid->output;
}
