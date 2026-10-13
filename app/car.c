#include "car.h"
#include <stddef.h>

volatile car_remote_t car_remote;


/* 根据麦轮安装方式，将车身运动指令换算为四个车轮的目标速度百分比。
 * forward：正数前进，负数后退。
 * left：正数向左平移，负数向右平移。
 * turn：正数车头向左转，负数向右转。
 * max_percent：车轮目标值的最大绝对值。
 * wheel_percent：输出数组，顺序为左前、右前、左后、右后。
 * 多个方向同时输入且超限时，四轮按相同比例缩小。
 * X/O 安装方式由 CAR_MECANUM_LAYOUT 选择。
 */
void car_mix_mecanum_percent(int16_t forward, int16_t left, int16_t turn,
                             uint8_t max_percent,
                             int16_t wheel_percent[CAR_WHEEL_COUNT])
{
    int32_t command[CAR_WHEEL_COUNT];
    int32_t largest = 0;
    int i;
#if CAR_MECANUM_LAYOUT == CAR_MECANUM_LAYOUT_O
    int32_t o_turn;
#endif

    if (wheel_percent == NULL) {
        return;
    }

#if CAR_MECANUM_LAYOUT == CAR_MECANUM_LAYOUT_X
    /* X 型：正转向使左轮后退、右轮前进。 */
    command[CAR_WHEEL_FRONT_LEFT] = forward - left - turn;
    command[CAR_WHEEL_FRONT_RIGHT] = forward + left + turn;
    command[CAR_WHEEL_REAR_LEFT] = forward + left - turn;
    command[CAR_WHEEL_REAR_RIGHT] = forward - left + turn;
#else
    /* O 型滚子方向相反；理想转向力臂为半轴距与半轮距之差。
     * turn 按 X 型的角速度刻度输入，换算为 O 型所需的轮速分量。
     */
    o_turn = ((int32_t)turn *
              (CAR_MECANUM_HALF_LENGTH_MM - CAR_MECANUM_HALF_WIDTH_MM)) /
             (CAR_MECANUM_HALF_LENGTH_MM + CAR_MECANUM_HALF_WIDTH_MM);

    command[CAR_WHEEL_FRONT_LEFT] = forward + left + o_turn;
    command[CAR_WHEEL_FRONT_RIGHT] = forward - left - o_turn;
    command[CAR_WHEEL_REAR_LEFT] = forward - left + o_turn;
    command[CAR_WHEEL_REAR_RIGHT] = forward + left - o_turn;
#endif

    /* 超出档位上限时统一缩小，避免逐轮截断改变运动方向。 */
    for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
    {
        int32_t absolute = (command[i] < 0) ? -command[i] : command[i];
        if (absolute > largest) 
        {
            largest = absolute;
        }
    }
    for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
    {
        if (largest > max_percent) 
        {
            command[i] = command[i] * max_percent / largest;
        }
        wheel_percent[i] = (int16_t)command[i];
    }
}

