#include "car_encoder.h"

/* 编码器任务每 20 ms 写入一次；按 car_wheel_t 的车轮顺序保存。 */
/* 编码器实际计数 ,约定轮子推动小车前进时，encoder_wheel_counts 为正*/
volatile int32_t encoder_wheel_counts[CAR_WHEEL_COUNT] = {0};

/* 编码器实际计算得到的速度 */
volatile float encoder_wheel_speed_mps[CAR_WHEEL_COUNT] = {0};


/* 将一个采样周期内的有符号编码器计数换算为车轮线速度，单位 m/s。 */
float encoder_counts_to_mps(int32_t counts)
{
    const float pi = 3.1415926f;
    const float counts_per_wheel_rev = ENCODER_LINES_PER_MOTOR_REV *
                                       ENCODER_COUNT_MULTIPLIER *
                                       MOTOR_GEAR_RATIO;

    /* 计数 / 每圈计数 = 车轮转数；再乘周长、除采样时间得到 m/s。 */
    return ((float)counts / counts_per_wheel_rev) *
           (pi * CAR_WHEEL_DIAMETER_M) / ENCODER_SAMPLE_PERIOD_S;
}

void car_encoder_calculate_wheel_speeds(const int32_t counts[CAR_WHEEL_COUNT],
                                        float speeds_mps[CAR_WHEEL_COUNT])
{
    int i;

    if (counts == 0 || speeds_mps == 0) {
        return;
    }

    for (i = 0; i < CAR_WHEEL_COUNT; ++i) 
    {
        speeds_mps[i] = encoder_counts_to_mps(counts[i]);
    }
}
