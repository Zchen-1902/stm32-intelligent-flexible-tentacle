#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

// 卡尔曼滤波结构体
typedef struct
{
    float z;  // 最优估计值
    float p;  // 误差协方差
    float r;  // 测量噪声
    float q;  // 过程噪声
} kalman_filter_t;

extern kalman_filter_t motor_speed_kalman_filter;
extern kalman_filter_t i_d_kalman_filter;
extern kalman_filter_t i_q_kalman_filter;

void kalman_filter_init(kalman_filter_t *kf, float r, float q);
float kalman_filter_calc(kalman_filter_t *kf, float input);

#endif 

