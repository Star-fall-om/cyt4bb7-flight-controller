#ifndef HORIZONTAL_VELOCITY_KF_H
#define HORIZONTAL_VELOCITY_KF_H

#include <stdint.h>

typedef struct {
    /* 单轴状态：速度和慢变水平加速度偏置，单位分别为cm/s、cm/s2。 */
    float velocity_cms;
    float accel_bias_cms2;
    float p_vv;
    float p_vb;
    float p_bb;
    float innovation_cms;
    float velocity_gain;
    float bias_gain;
    float innovation_variance;
    uint8_t initialized;
} horizontal_velocity_kf_axis_t;

typedef struct {
    /* X/Y分别独立估计，避免在实时控制周期内引入通用矩阵运算。 */
    horizontal_velocity_kf_axis_t x;
    horizontal_velocity_kf_axis_t y;
    uint32_t reset_count;
} horizontal_velocity_kf_t;

void horizontal_velocity_kf_init(horizontal_velocity_kf_t *kf);
void horizontal_velocity_kf_reset(horizontal_velocity_kf_t *kf);
void horizontal_velocity_kf_set_bias_seed(horizontal_velocity_kf_t *kf,
                                           float bias_x_cms2,
                                           float bias_y_cms2);
void horizontal_velocity_kf_predict(horizontal_velocity_kf_t *kf,
                                    float acc_x_cms2,
                                    float acc_y_cms2,
                                    float dt_s,
                                    float accel_trust);
uint8_t horizontal_velocity_kf_update(horizontal_velocity_kf_t *kf,
                                      float flow_x_cms,
                                      float flow_y_cms,
                                      uint8_t valid_x,
                                      uint8_t valid_y,
                                      float measurement_noise_x,
                                      float measurement_noise_y,
                                      uint8_t bias_update_enable);

#endif
