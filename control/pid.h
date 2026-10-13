#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;                   /* 比例系数：响应当前误差。 */
    float ki;                   /* 积分系数：消除持续误差。 */
    float kd;                   /* 微分系数：响应误差变化。 */
    float max_output;           /* 输出绝对值上限，例如 PWM 的 100%。 */
    float output;               /* 上一次计算得到的输出。 */
    float integral;             /* 位置式 PID 累积的积分输出。 */
    float last_error;           /* 上一次误差。 */
    float previous_error;       /* 上上次误差，供增量式 PID 使用。 */
    uint8_t has_previous_error; /* 是否已经计算过一次，用于跳过首次微分。 */
} pid_controller_t;

/* 初始化参数；dt_s 使用秒，ki 乘 dt_s，kd 除 dt_s。
 * max_output 是输出绝对值上限，PWM 百分比通常传 100。
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_output);
void pid_reset(pid_controller_t *pid);

/* 增量式 PID：保留原接口，计算本次输出相对上次的变化量。
 * 按固定周期调用；dt_s > 0，目标与实际值必须使用相同单位。
 */
float pid_update(pid_controller_t *pid, float target, float actual,
                 float dt_s);

/* 位置式 PID：直接由 P + I + D 计算本次输出。
 * 按固定周期调用；dt_s > 0，目标与实际值必须使用相同单位。
 * 输出受 max_output 限制，积分也会限制以减轻饱和后的超调。
 */
float pid_update_position(pid_controller_t *pid, float target, float actual,
                          float dt_s);

#endif /* PID_H */
