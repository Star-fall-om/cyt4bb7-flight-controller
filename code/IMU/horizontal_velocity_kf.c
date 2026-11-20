#include "horizontal_velocity_kf.h"
#include <math.h>

#define HVEL_KF_MIN_DT_S             (0.0005f)
#define HVEL_KF_MAX_DT_S             (0.0200f)
#define HVEL_KF_MIN_TRUST             (0.05f)
#define HVEL_KF_MAX_COV               (1000000.0f)
#define HVEL_KF_MIN_VELOCITY_COV      (25.0f)
#define HVEL_KF_MIN_MEAS_NOISE        (9.0f)
#define HVEL_KF_ACCEL_NOISE_CMS2      (900.0f)
#define HVEL_KF_BIAS_RW_CMS2          (4.0f)
#define HVEL_KF_BIAS_FREEZE_TRUST     (0.5f)
#define HVEL_KF_MAX_INNOVATION_CMS    (250.0f)
#define HVEL_KF_MAX_VELOCITY_CMS      (400.0f)

/* 本模块使用显式展开的二状态Kalman公式，避免IAR工程引入动态矩阵库。 */

static float hvel_kf_clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void hvel_kf_axis_reset(horizontal_velocity_kf_axis_t *axis)
{
    axis->velocity_cms = 0.0f;
    axis->accel_bias_cms2 = 0.0f;
    axis->p_vv = 400.0f;
    axis->p_vb = 0.0f;
    axis->p_bb = 400.0f;
    axis->innovation_cms = 0.0f;
    axis->velocity_gain = 0.0f;
    axis->bias_gain = 0.0f;
    axis->innovation_variance = 0.0f;
    axis->initialized = 1u;
}

static void hvel_kf_axis_predict(horizontal_velocity_kf_axis_t *axis,
                                 float acc_cms2,
                                 float dt_s,
                                 float accel_trust)
{
    float old_p_vb;
    float old_p_bb;
    float q_acc;
    float q_bias;

    if (axis == 0) return;
    if (!axis->initialized) hvel_kf_axis_reset(axis);
    if (!isfinite(dt_s) || dt_s < HVEL_KF_MIN_DT_S || dt_s > HVEL_KF_MAX_DT_S) {
        dt_s = 0.001f;
    }
    if (!isfinite(acc_cms2)) acc_cms2 = 0.0f;
    accel_trust = hvel_kf_clamp(accel_trust, HVEL_KF_MIN_TRUST, 1.0f);

    /* 加速度可信度降低时增大过程噪声，让后续有效光流获得更高权重。 */
    axis->velocity_cms += (acc_cms2 - axis->accel_bias_cms2) * dt_s;
    axis->velocity_cms = hvel_kf_clamp(axis->velocity_cms,
                                       -HVEL_KF_MAX_VELOCITY_CMS,
                                        HVEL_KF_MAX_VELOCITY_CMS);

    old_p_vb = axis->p_vb;
    old_p_bb = axis->p_bb;
    q_acc = HVEL_KF_ACCEL_NOISE_CMS2 * (1.0f + 4.0f * (1.0f - accel_trust));
    q_bias = (accel_trust < HVEL_KF_BIAS_FREEZE_TRUST) ?
             0.0f : HVEL_KF_BIAS_RW_CMS2 * (0.5f + accel_trust);
    axis->p_vv += -2.0f * dt_s * old_p_vb
                + dt_s * dt_s * old_p_bb
                + q_acc * dt_s * dt_s;
    if (accel_trust >= HVEL_KF_BIAS_FREEZE_TRUST) {
        axis->p_vb -= dt_s * old_p_bb;
        axis->p_bb += q_bias * dt_s;
    } else {
        axis->p_vb = old_p_vb;
        axis->p_bb = old_p_bb;
    }
    /* 保留最低速度协方差，避免连续光流更新后观测增益塌缩。 */
    axis->p_vv = hvel_kf_clamp(axis->p_vv,
                               HVEL_KF_MIN_VELOCITY_COV,
                               HVEL_KF_MAX_COV);
    axis->p_bb = hvel_kf_clamp(axis->p_bb, 0.0001f, HVEL_KF_MAX_COV);
}

static void hvel_kf_axis_update(horizontal_velocity_kf_axis_t *axis,
                                float measurement_cms,
                                uint8_t valid,
                                float measurement_noise,
                                uint8_t bias_update_enable)
{
    float innovation;
    float innovation_variance;
    float gain_v;
    float gain_b;
    float old_p_vb;

    if (axis == 0 || !valid || !isfinite(measurement_cms)) return;
    if (!isfinite(measurement_noise) || measurement_noise < HVEL_KF_MIN_MEAS_NOISE) {
        measurement_noise = HVEL_KF_MIN_MEAS_NOISE;
    }
    /* 光流是速度观测；无效帧由调用方屏蔽，不把无效帧当作零速度。 */
    innovation = measurement_cms - axis->velocity_cms;
    innovation = hvel_kf_clamp(innovation,
                               -HVEL_KF_MAX_INNOVATION_CMS,
                                HVEL_KF_MAX_INNOVATION_CMS);
    innovation_variance = axis->p_vv + measurement_noise;
    if (!isfinite(innovation_variance) || innovation_variance <= 0.0f) return;

    gain_v = axis->p_vv / innovation_variance;
    gain_b = bias_update_enable ? (axis->p_vb / innovation_variance) : 0.0f;
    old_p_vb = axis->p_vb;
    axis->velocity_cms += gain_v * innovation;
    if (bias_update_enable) {
        axis->accel_bias_cms2 += gain_b * innovation;
    }
    axis->velocity_cms = hvel_kf_clamp(axis->velocity_cms,
                                       -HVEL_KF_MAX_VELOCITY_CMS,
                                        HVEL_KF_MAX_VELOCITY_CMS);
    axis->p_vv = (1.0f - gain_v) * axis->p_vv;
    if (bias_update_enable) {
        axis->p_vb = (1.0f - gain_v) * old_p_vb;
        axis->p_bb -= gain_b * old_p_vb;
    }
    axis->p_vv = hvel_kf_clamp(axis->p_vv,
                               HVEL_KF_MIN_VELOCITY_COV,
                               HVEL_KF_MAX_COV);
    axis->p_bb = hvel_kf_clamp(axis->p_bb, 0.0001f, HVEL_KF_MAX_COV);
    axis->innovation_cms = innovation;
    axis->velocity_gain = gain_v;
    axis->bias_gain = gain_b;
    axis->innovation_variance = innovation_variance;
}

void horizontal_velocity_kf_init(horizontal_velocity_kf_t *kf)
{
    if (kf == 0) return;
    kf->reset_count = 0u;
    hvel_kf_axis_reset(&kf->x);
    hvel_kf_axis_reset(&kf->y);
}

void horizontal_velocity_kf_reset(horizontal_velocity_kf_t *kf)
{
    if (kf == 0) return;
    hvel_kf_axis_reset(&kf->x);
    hvel_kf_axis_reset(&kf->y);
    kf->reset_count++;
}

void horizontal_velocity_kf_set_bias_seed(horizontal_velocity_kf_t *kf,
                                           float bias_x_cms2,
                                           float bias_y_cms2)
{
    if (kf == 0) return;
    if (isfinite(bias_x_cms2)) kf->x.accel_bias_cms2 = bias_x_cms2;
    if (isfinite(bias_y_cms2)) kf->y.accel_bias_cms2 = bias_y_cms2;
}

void horizontal_velocity_kf_predict(horizontal_velocity_kf_t *kf,
                                    float acc_x_cms2,
                                    float acc_y_cms2,
                                    float dt_s,
                                    float accel_trust)
{
    if (kf == 0) return;
    hvel_kf_axis_predict(&kf->x, acc_x_cms2, dt_s, accel_trust);
    hvel_kf_axis_predict(&kf->y, acc_y_cms2, dt_s, accel_trust);
}

uint8_t horizontal_velocity_kf_update(horizontal_velocity_kf_t *kf,
                                      float flow_x_cms,
                                      float flow_y_cms,
                                      uint8_t valid_x,
                                      uint8_t valid_y,
                                      float measurement_noise_x,
                                      float measurement_noise_y,
                                      uint8_t bias_update_enable)
{
    uint8_t update_mask = 0u;

    if (kf == 0) return 0u;
    if (valid_x && isfinite(flow_x_cms)) {
        hvel_kf_axis_update(&kf->x, flow_x_cms, 1u,
                            measurement_noise_x, bias_update_enable);
        update_mask |= 0x01u;
    }
    if (valid_y && isfinite(flow_y_cms)) {
        hvel_kf_axis_update(&kf->y, flow_y_cms, 1u,
                            measurement_noise_y, bias_update_enable);
        update_mask |= 0x02u;
    }
    return update_mask;
}
