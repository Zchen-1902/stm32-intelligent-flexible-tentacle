# include "kalman_filter.h"

kalman_filter_t motor_speed_kalman_filter;

// 初始化函数
void kalman_filter_init(kalman_filter_t *kf, float r, float q)
{
    kf->z = 0;
    kf->p = 1;
    kf->r = r;
    kf->q = q;
}

// 滤波计算函数
float kalman_filter_calc(kalman_filter_t *kf, float input)
{
    kf->p = kf->p + kf->q;
    float g = kf->p / (kf->p + kf->r);
    kf->z = kf->z + g * (input - kf->z);
    kf->p = (1 - g) * kf->p;
    return kf->z;
}

/*
先固定 q，调 r：

    r越大 → 越不信任采样值 → 滤波越平滑，响应越慢；
    r越小 → 越信任采样值 → 滤波越贴近原始值，降噪效果差；
    电流采样推荐r=0.01 0.5，转速采样推荐r=0.1 1.0；

再微调 q：

    q越大 → 越不信任系统预测 → 滤波响应越快，平滑性下降；
    q越小 → 越信任系统预测 → 滤波越平滑，响应越慢；
    电机控制推荐q=0.001 0.1（q通常远小于r）；

要平滑 → 增大r、减小q；
要响应 → 减小r、增大q；

*/

// 使用示例（q轴电流滤波）
// kalman_filter_t iq_kf;
// kalman_filter_init(&iq_kf, 0.1f, 0.01f);
// motor_i_q_filtered = kalman_filter_calc(&iq_kf, motor_i_q_raw);
