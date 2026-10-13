#include "control.h"
#include "motor.h"
#include "pid.h"
#include "stm32f1xx_hal.h"

/* 四轮当前目标速度百分比；斜坡按车轮分别限制变化量。 */
static int16_t current_wheel_command_percent[CAR_WHEEL_COUNT];
static pid_controller_t wheel_pid[CAR_WHEEL_COUNT];
static uint8_t speed_control_initialized;
static volatile uint8_t remote_command_valid;
static volatile uint8_t pid_reset_requested;

/* pid后实际输出的 pwm 值 */
volatile int16_t pwm[CAR_WHEEL_COUNT];

/* pid输入的目标速度 */
volatile float car_wheel_target_mps[CAR_WHEEL_COUNT] = {0.0f};

/* 下标顺序：左前、右前、左后、右后。 */
static const uint8_t wheel_motor_ids[CAR_WHEEL_COUNT] = {
    CAR_MOTOR_FRONT_LEFT_ID,
    CAR_MOTOR_FRONT_RIGHT_ID,
    CAR_MOTOR_REAR_LEFT_ID,
    CAR_MOTOR_REAR_RIGHT_ID
};

/* 左侧为电机 0、2，右侧为电机 1、3；这里只负责输出 PWM。 */
static void car_set_sides_pwm(int16_t left_duty, int16_t right_duty)
{
    motor_set_pwm(CAR_MOTOR_FRONT_LEFT_ID, left_duty);
    motor_set_pwm(CAR_MOTOR_REAR_LEFT_ID, left_duty);
    motor_set_pwm(CAR_MOTOR_FRONT_RIGHT_ID, right_duty);
    motor_set_pwm(CAR_MOTOR_REAR_RIGHT_ID, right_duty);
}

/* 摇杆原始范围约为 -128～127，映射到目标速度百分比。 */
static int16_t car_axis_to_percent(int16_t axis, uint8_t max_percent)
{
    int32_t percent = (-(int32_t)axis * max_percent) / 127;

    if (percent > max_percent) {
        percent = max_percent;
    } else if (percent < -(int32_t)max_percent) {
        percent = -(int32_t)max_percent;
    }
    return (int16_t)percent;
}

static int16_t car_ramp_percent(int16_t current, int16_t target)
{
    if (target > current + CAR_SPEED_RAMP_STEP_PERCENT) 
    {
        return current + CAR_SPEED_RAMP_STEP_PERCENT;
    }
    if (target < current - CAR_SPEED_RAMP_STEP_PERCENT) 
    {
        return current - CAR_SPEED_RAMP_STEP_PERCENT;
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
    int i;

    remote_command_valid = 0U;
    pid_reset_requested = 1U;
    for (i = 0; i < CAR_WHEEL_COUNT; ++i) {
        current_wheel_command_percent[i] = 0;
        car_wheel_target_mps[i] = 0.0f;
    }
    car_set_sides_pwm(0, 0);
}

//初始化pid
void car_speed_control_init(void)
{
    pid_init(&wheel_pid[CAR_WHEEL_FRONT_LEFT],
             CAR_MOTOR_0_PID_KP, CAR_MOTOR_0_PID_KI, CAR_MOTOR_0_PID_KD,
             (float)MOTOR_PWM_MAX);
    pid_init(&wheel_pid[CAR_WHEEL_REAR_RIGHT],
             CAR_MOTOR_1_PID_KP, CAR_MOTOR_1_PID_KI, CAR_MOTOR_1_PID_KD,
             (float)MOTOR_PWM_MAX);
    pid_init(&wheel_pid[CAR_WHEEL_REAR_LEFT],
             CAR_MOTOR_2_PID_KP, CAR_MOTOR_2_PID_KI, CAR_MOTOR_2_PID_KD,
             (float)MOTOR_PWM_MAX);
    pid_init(&wheel_pid[CAR_WHEEL_FRONT_RIGHT],
             CAR_MOTOR_3_PID_KP, CAR_MOTOR_3_PID_KI, CAR_MOTOR_3_PID_KD,
             (float)MOTOR_PWM_MAX);
    speed_control_initialized = 1U;
}

/*更新四轮速度环，实际轮速由编码器任务传入,由编码器任务调用*/
void car_speed_control_update(const volatile float target_mps[CAR_WHEEL_COUNT],
                              const float actual_mps[CAR_WHEEL_COUNT])
{
    float output;
    float target;
    int i;

    /* PID 的历史误差和输出只由编码器任务维护。 */
    if (pid_reset_requested != 0U) //小车停止
    {
        for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
        {
            pid_reset(&wheel_pid[i]);
        }
        pid_reset_requested = 0U;
    }

    /* 检查速度控制是否已初始化，以及是否有有效的遥控命令和实际轮速 */
    if (speed_control_initialized == 0U || target_mps == 0 || actual_mps == 0 ||
        remote_command_valid == 0U) {
        for (i = 0; i < CAR_WHEEL_COUNT; ++i) {
            pid_reset(&wheel_pid[i]);
        }
        car_set_sides_pwm(0, 0);
        return;
    }

    /* 计算每个车轮的目标 PWM */
    for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
    {
        target = target_mps[i];
        if (target == 0.0f) 
        {
            /* 松开摇杆时清除积分和输出，避免再次起步沿用旧状态。 */
            pid_reset(&wheel_pid[i]);
            pwm[i] = 0;
        } 
        else 
        {
            /* 四个车轮各用自己的位置式 PID：目标轮速与编码器轮速都为 m/s。 */
            output = pid_update_position(&wheel_pid[i], target, actual_mps[i],
                                         ENCODER_SAMPLE_PERIOD_S);

            /* 浮点数 PWM四舍五入 */
            pwm[i] = (int16_t)(output + ((output >= 0.0f) ? 0.5f : -0.5f));
        }
    }

    /* 遥控任务可能在计算期间检测到失联。 */
    if (remote_command_valid == 0U) {
        car_set_sides_pwm(0, 0);
        return;
    }

    for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
    {
        motor_set_pwm(wheel_motor_ids[i], pwm[i]);
    }
}



/* 左摇杆控制车身前后和左右平移，右摇杆左右控制车头旋转。 */
void car_control_update(car_remote_t remote)
{
    static uint32_t last_ramp_tick;
    uint32_t now = HAL_GetTick();
    uint8_t max_speed_percent;
    int16_t forward_percent;
    int16_t strafe_percent;
    int16_t turn_percent;
    int16_t wheel_target_percent[CAR_WHEEL_COUNT];
    int i;

    /* 遥控失联直接停车，不等待斜坡减速。 */
    if (remote.valid == 0U) 
    {
        car_stop();
        last_ramp_tick = now;
        return;
    }

    max_speed_percent = (remote.l2 != 0U) ? CAR_L2_SPEED_PERCENT :
                                           CAR_NORMAL_SPEED_PERCENT;
    /* 正值分别表示向前、向车身左侧平移、车头向左旋转。 */
    forward_percent = car_axis_to_percent(remote.left_y, max_speed_percent);
    strafe_percent = car_axis_to_percent(remote.left_x, max_speed_percent);
    turn_percent = car_axis_to_percent(remote.right_x, max_speed_percent);

    /* 根据 car.h 中的 X/O 安装配置，合成并限制四轮目标速度。 */
    car_mix_mecanum_percent(forward_percent, strafe_percent, turn_percent,
                            max_speed_percent, wheel_target_percent);

    if ((uint32_t)(now - last_ramp_tick) < CAR_SPEED_RAMP_INTERVAL_MS)
    {
        return;
    }
    last_ramp_tick = now;

    /* 每轮独立斜坡；这里只更新目标轮速，PWM 由 20 ms 速度环计算。 */
    for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
    {
        current_wheel_command_percent[i] =
            car_ramp_percent(current_wheel_command_percent[i],
                             wheel_target_percent[i]);
        car_wheel_target_mps[i] =
            (float)current_wheel_command_percent[i] *
            CAR_MAX_WHEEL_SPEED_MPS / 100.0f;
    }

    remote_command_valid = 1U;
}
