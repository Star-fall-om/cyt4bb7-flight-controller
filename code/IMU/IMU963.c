#include "IMU963.h"
#include "zf_common_headfile.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "System/usr_system.h"
#include "Algorithm/PID/pid.h"
#include "horizontal_velocity_kf.h"


// ==================== 参数调优区 ====================
#define GYRO_CUTOFF_HZ    70.0f
#define ACC_CUTOFF_HZ     10.0f
#define NAV_ACC_CUTOFF_HZ 15.0f
#define HORIZONTAL_CORRECTION_TAU_FAST_S 0.025f
#define HORIZONTAL_CORRECTION_TAU_SLOW_S 0.060f
#define HORIZONTAL_RECOVERY_TAU_S 0.120f
#define HORIZONTAL_INNOVATION_SLOW_CMS 8.0f
#define HORIZONTAL_INNOVATION_FAST_CMS 25.0f
#define HORIZONTAL_INNOVATION_LIMIT_CMS 60.0f
#define HORIZONTAL_VELOCITY_LIMIT_CMS   400.0f
#define HORIZONTAL_BIAS_INNOVATION_LIMIT_CMS 20.0f
#define HORIZONTAL_RECOVERY_FRAME_COUNT 2u
#define HORIZONTAL_NAV_ACC_LIMIT_CMS2 350.0f
#define HORIZONTAL_BIAS_ERROR_TAU_S 0.5f
#define HORIZONTAL_BIAS_GAIN_PER_S  2.5f
#define HORIZONTAL_FLOW_FRESH_S     0.06f
#ifndef HORIZONTAL_VISUAL_VEL_ENABLE
#define HORIZONTAL_VISUAL_VEL_ENABLE 1u
#endif
// KF默认接入速度环；改为0可保留旧估计器作为对照或紧急回退。
#ifndef HORIZONTAL_KF_CONTROL_ENABLE
#define HORIZONTAL_KF_CONTROL_ENABLE 1u
#endif
#ifndef HORIZONTAL_KF_VISUAL_FUSION_ENABLE
#define HORIZONTAL_KF_VISUAL_FUSION_ENABLE 0u
#endif
#define HORIZONTAL_ESTIMATOR_BUILD_MARKER 0x26081803UL
#define HORIZONTAL_KF_ACCEL_WEIGHT_MODE 1u
#define HORIZONTAL_KF_CONFIG_RESIDUAL_WEIGHT (1u << 3)
#define HORIZONTAL_KF_CONFIG_BIAS_FREEZE (1u << 4)
#define HORIZONTAL_KF_BIAS_FREEZE_TRUST 0.5f
#define HORIZONTAL_VISUAL_FRESH_S            0.12f
#define HORIZONTAL_CONTROL_FLOW_HOLD_S       0.030f
#define HORIZONTAL_CONTROL_FLOW_TIMEOUT_S    0.080f
#define HORIZONTAL_CONTROL_DECAY_S            0.200f
#define HORIZONTAL_VISUAL_INNOVATION_LIMIT_CMS 100.0f
#define HORIZONTAL_VISUAL_TAU_WITH_FLOW_S    0.50f
#define HORIZONTAL_VISUAL_TAU_NO_FLOW_S      0.12f
#define HORIZONTAL_DECAY_DELAY_S    0.10f
#define HORIZONTAL_DECAY_TAU_S      0.5f
#define HORIZONTAL_GROUND_RESET_HEIGHT_MM 150.0f
#define HORIZONTAL_KF_VISUAL_MEAS_NOISE_CMS2 900.0f

// 默认启用；定义为0时保留检测遥测，但不对Mahony和零偏学习实际降权。
#ifndef IMU_EXTERNAL_ACCEL_PROTECT_ENABLE
#define IMU_EXTERNAL_ACCEL_PROTECT_ENABLE 1u
#endif
// 唯一新增调参量：二维残差模长达到120cm/s2时，瞬时可信度约为0.5。
#ifndef IMU_EXTERNAL_ACCEL_SCALE_CMS2
#define IMU_EXTERNAL_ACCEL_SCALE_CMS2 120.0f
#endif
// 线缆冲击纯光流保护：加速度单位cm/s2，时间按IMU实际dt累计。
#define IMU_IMPACT_ENTER_CMS2              160.0f
#define IMU_IMPACT_STRONG_CMS2             240.0f
#define IMU_IMPACT_EXIT_CMS2                80.0f
#define IMU_IMPACT_CONFIRM_S                 0.015f
#define IMU_IMPACT_MIN_HOLD_S                0.100f
#define IMU_IMPACT_EXIT_CONFIRM_S            0.030f
#define IMU_IMPACT_FF_RECOVERY_S             0.300f
#define IMU_IMPACT_KF_FLOW_MAX_DIFF_CMS     15.0f
#define IMU_IMPACT_FLOW_RECOVERY_FRAMES      3u
#define MAHONY_KP_NORMAL  2.5f
#define MAHONY_KI_NORMAL  0.005f

// ==================== 变量定义 ====================
float pitch_filtered = 0.0f, roll_filtered = 0.0f, yaw_filtered = 0.0f;
float pitch_gyro_filtered = 0.0f, roll_gyro_filtered = 0.0f, yaw_gyro_filtered = 0.0f;

float accl_m_s2[3] = {0};
float raw_accl_m_s2[3] = {0};
float gyro_offset[3] = {0};

float level_offset_pitch = 0.0f;
float level_offset_roll = 0.0f;
float level_offset_yaw = 0.0f;

float  wz_R = 0.02f;
pt2_filter_t gyro_pt2[3];
pt1_filter_t acc_pt1[3];
static pt1_filter_t nav_acc_pt1[2];
mahony_filter_t mahony = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 定义全局变量存储测得的 R
float R_roll = 0.05f, R_pitch = 0.05f, R_yaw = 0.05f;

static TickType_t last_wake_time;
static float dt_s = 0.001f;
adaptive_kalman akf_gx;
adaptive_kalman akf_gy;
adaptive_kalman akf_gz;

float vertical_velocity = 0.0f;
// ====== 新增估算器全局状态 ======
float est_z_cm = 0.0f;           // 估算高度 (cm)
float est_v_cms = 0.0f;          // 估算速度 (cm/s)
static float est_a_bias_cms2 = 0.0f;    // 动态加速度偏置 (cm/s2) - 核心修正项

float vertical_velocity_smoothed = 0.0f; // 最终给 PID 使用的平滑速度
float vertical_accel_cms2 = 0.0f;         // 导航系去重力垂直加速度
float vertical_accel_corrected_cms2 = 0.0f; // 扣除估计偏置后的垂直加速度
float vertical_tof_error_cm = 0.0f;       // ToF高度与估计高度的有效创新
float vertical_tof_z_correction_cm = 0.0f; // ToF对估计高度的本次修正量
float vertical_tof_v_correction_cms = 0.0f; // ToF对估计速度的本次修正量

// 新增：加速度计零偏校准（可选）
static float acc_nav_z_offset = 0.0f; // 导航系Z轴加速度零偏 (m/s2)
static float acc_body_x_offset = 0.0f; // 机体系X轴静态物理零偏
static float acc_body_y_offset = 0.0f; // 机体系Y轴静态物理零偏

// ==================== 辅助宏 ====================
// 增强版 CLAMP，确保 NaN 不会被比较逻辑漏过
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
// 增强版 NaN 检查
#define IS_VALID(x) (!isnan(x) && !isinf(x))


void integrate_vertical_velocity(float dt);
static void integrate_horizontal_velocity(float dt);

static float est_vx_cms = 0.0f;
static float est_vy_cms = 0.0f;
static float est_pos_x_cm = 0.0f;
static float est_pos_y_cm = 0.0f;
static float est_a_bias_x_cms2 = 0.0f;
static float est_a_bias_y_cms2 = 0.0f;
static horizontal_velocity_kf_t horizontal_velocity_kf;
static float horizontal_kf_trim_acc_sum_x_cms2 = 0.0f;
static float horizontal_kf_trim_acc_sum_y_cms2 = 0.0f;
static uint32 horizontal_kf_trim_acc_count = 0u;
static uint8 horizontal_kf_trim_seeded = 0u;
static float horizontal_bias_error_x_cms = 0.0f;
static float horizontal_bias_error_y_cms = 0.0f;
static float horizontal_flow_vx_cms = 0.0f;
static float horizontal_flow_vy_cms = 0.0f;
static float horizontal_flow_age_x_s = 1000.0f;
static float horizontal_flow_age_y_s = 1000.0f;
static float horizontal_correction_remaining_x_s = 0.0f;
static float horizontal_correction_remaining_y_s = 0.0f;
static float horizontal_last_innovation_x_cms = 0.0f;
static float horizontal_last_innovation_y_cms = 0.0f;
static float horizontal_last_nav_acc_x_cms2 = 0.0f;
static float horizontal_last_nav_acc_y_cms2 = 0.0f;
static float horizontal_last_imu_predicted_vx_cms = 0.0f;
static float horizontal_last_imu_predicted_vy_cms = 0.0f;
static float horizontal_last_flow_correction_x_cms = 0.0f;
static float horizontal_last_flow_correction_y_cms = 0.0f;
static float horizontal_last_correction_tau_x_s = HORIZONTAL_CORRECTION_TAU_SLOW_S;
static float horizontal_last_correction_tau_y_s = HORIZONTAL_CORRECTION_TAU_SLOW_S;
static uint8 horizontal_have_flow_x = 0;
static uint8 horizontal_have_flow_y = 0;
static float horizontal_last_valid_flow_x_cms = 0.0f;
static float horizontal_last_valid_flow_y_cms = 0.0f;
static uint8 horizontal_ground_reset_latched = 0;
static uint8 horizontal_flow_recovery_frames_x = 0;
static uint8 horizontal_flow_recovery_frames_y = 0;
static uint8 horizontal_flow_trust_level_x = 0u;
static uint8 horizontal_flow_trust_level_y = 0u;
static float horizontal_flow_consistency_error_x_cms = 0.0f;
static float horizontal_flow_consistency_error_y_cms = 0.0f;
static uint32 horizontal_full_reset_count = 0;
static uint32 horizontal_position_reset_count = 0;
static float horizontal_visual_vx_cms = 0.0f;
static float horizontal_visual_vy_cms = 0.0f;
static float horizontal_visual_age_x_s = 1000.0f;
static float horizontal_visual_age_y_s = 1000.0f;
static float horizontal_visual_correction_remaining_x_s = 0.0f;
static float horizontal_visual_correction_remaining_y_s = 0.0f;
static float horizontal_last_visual_innovation_x_cms = 0.0f;
static float horizontal_last_visual_innovation_y_cms = 0.0f;
static uint8 horizontal_have_visual_x = 0u;
static uint8 horizontal_have_visual_y = 0u;
static uint8 horizontal_visual_source = 0u;

typedef struct {
    float vx_cms;
    float vy_cms;
    float dt_s;
    uint8 valid_x;
    uint8 valid_y;
    uint8 trust_level_x;
    uint8 trust_level_y;
    float consistency_error_x_cms;
    float consistency_error_y_cms;
} horizontal_flow_observation_t;

typedef struct {
    float vx_cms;
    float vy_cms;
    float dt_s;
    uint8 valid_x;
    uint8 valid_y;
    uint8 source;
} horizontal_visual_observation_t;

static volatile horizontal_flow_observation_t pending_flow_observation;
static volatile uint8 pending_flow_observation_ready = 0;
static volatile horizontal_visual_observation_t pending_visual_observation;
static volatile uint8 pending_visual_observation_ready = 0;
static volatile uint8 horizontal_position_reset_requested = 0;
static volatile uint8 horizontal_full_reset_requested = 0;
static imu_horizontal_state_t horizontal_published_state;
static float external_accel_expected_x_cms2 = 0.0f;
static float external_accel_expected_y_cms2 = 0.0f;
static float external_accel_baseline_x_cms2 = 0.0f;
static float external_accel_baseline_y_cms2 = 0.0f;
static float external_accel_residual_x_cms2 = 0.0f;
static float external_accel_residual_y_cms2 = 0.0f;
static float external_accel_trust = 1.0f;
static float external_accel_applied_trust = 1.0f;
static uint8 external_accel_shock_active = 0u;
static uint8 external_accel_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
static float external_accel_guard_recovery = 1.0f;
static float external_accel_trigger_time_s = 0.0f;
static float external_accel_active_time_s = 0.0f;
static float external_accel_exit_time_s = 0.0f;
static float external_accel_recovery_time_s = 0.0f;
static uint8 external_accel_flow_recovery_frames = 0u;
static volatile uint8 external_accel_control_ready_mask = 0u;
static float horizontal_kf_accel_used_x_cms2 = 0.0f;
static float horizontal_kf_accel_used_y_cms2 = 0.0f;
static uint32 horizontal_flow_update_count_x = 0u;
static uint32 horizontal_flow_update_count_y = 0u;
static float horizontal_flow_update_age_x_s = 1000.0f;
static float horizontal_flow_update_age_y_s = 1000.0f;

extern uint16_t tof_blind_timer;

// ==================== 滤波器算法 (保持不变) ====================

void pt2_filter_init(pt2_filter_t *filter, float f_cut, float dt) {
    float rc = 1.0f / (2.0f * M_PI * f_cut);
    filter->k = dt / (rc + dt);
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
}

float pt2_filter_apply(pt2_filter_t *filter, float input) {
    if(!IS_VALID(input)) return filter->state2;
    float next_state1 = filter->state1 + filter->k * (input - filter->state1);
    float next_state2 = filter->state2 + filter->k * (next_state1 - filter->state2);
    if(IS_VALID(next_state2)) {
        filter->state1 = next_state1;
        filter->state2 = next_state2;
        return filter->state2;
    }
    return filter->state2;
}

void pt1_filter_init(pt1_filter_t *filter, float f_cut, float dt) {
    float rc = 1.0f / (2.0f * M_PI * f_cut);
    filter->k = dt / (rc + dt);
    filter->state = 0.0f;
}

float pt1_filter_apply(pt1_filter_t *filter, float input) {
    if(!IS_VALID(input)) return filter->state;
    float next_state = filter->state + filter->k * (input - filter->state);
    if(IS_VALID(next_state)) {
        filter->state = next_state;
        return filter->state;
    }
    return filter->state;
}

float invSqrt(float x) {
    if (x <= 1e-6f || !IS_VALID(x)) return 0.0f;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


void measure_all_gyro_R_values(int samples) {
    float sum[3] = {0}, mean[3] = {0}, variance[3] = {0};
    float *data_x = (float*)pvPortMalloc(sizeof(float) * samples);
    float *data_y = (float*)pvPortMalloc(sizeof(float) * samples);
    float *data_z = (float*)pvPortMalloc(sizeof(float) * samples);

    printf("R-Measurement: Starting... Keep Still!\r\n");

    for (int i = 0; i < samples; i++) {
        imu963ra_get_gyro();
        // 获取三个轴的 PT2 滤波后数据
        float gx = pt2_filter_apply(&gyro_pt2[0], imu963ra_gyro_transition(imu963ra_gyro_x) - gyro_offset[0]);
        float gy = pt2_filter_apply(&gyro_pt2[1], imu963ra_gyro_transition(imu963ra_gyro_y) - gyro_offset[1]);
        float gz = pt2_filter_apply(&gyro_pt2[2], imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_offset[2]);

        data_x[i] = gx; sum[0] += gx;
        data_y[i] = gy; sum[1] += gy;
        data_z[i] = gz; sum[2] += gz;
        system_delay_ms(2);
    }

    for (int j = 0; j < 3; j++) {
        mean[j] = sum[j] / (float)samples;
        float temp_v = 0;
        for (int i = 0; i < samples; i++) {
            float val = (j==0)?data_x[i]:(j==1?data_y[i]:data_z[i]);
            temp_v += (val - mean[j]) * (val - mean[j]);
        }
        variance[j] = temp_v / (float)samples;
    }

    // 释放内存
    vPortFree(data_x); vPortFree(data_y); vPortFree(data_z);

    // 赋值给全局变量（乘以系数 3.0 是为了给飞行时的震动留出余量）
    R_roll  = variance[0] * 3.0f;
    R_pitch = variance[1] * 3.0f;
    R_yaw   = variance[2] * 3.0f;

    printf("R Measured -> Roll: %f, Pitch: %f, Yaw: %f\r\n", R_roll, R_pitch, R_yaw);
}

float adaptive_kalman_filter(float Z, adaptive_kalman* kalman)
{
    // 输入有效性检查
    if (!isfinite(Z)) {
        // 如果输入无效，直接返回上次估计值
        return kalman->X_hat;
    }

    float X_hat_pred = kalman->X_hat;
    float P_pred = kalman->P + kalman->Q;

    // 防止分母为零（理论上不会，但做保护）
    float denom = P_pred + kalman->R;
    if (denom < 1e-12f) denom = 1e-12f;

    float K = P_pred / denom;
    float IV = Z - X_hat_pred;

    float X_hat_new = X_hat_pred + K * IV;
    float P_new = (1.0f - K) * P_pred;

    // 检查更新后的值是否有效
    if (isfinite(X_hat_new) && isfinite(P_new) && P_new >= 0.0f) {
        kalman->X_hat = X_hat_new;
        kalman->P = P_new;
    } else {
        // 如果出现无效值，保持原状态，并重置一些参数以防发散
        kalman->P = 1.0f;  // 重置为初始值
        // 不更新 X_hat
    }

    // 更新新息缓冲区
    kalman->IV_buf[kalman->iv_idx] = IV * IV;
    kalman->iv_idx = (kalman->iv_idx + 1) % 10;

    // 计算平均新息
    float CIV = 0.0f;
    for (uint8_t i = 0; i < 10; i++) {
        CIV += kalman->IV_buf[i];
    }
    CIV /= 10.0f;

    // 更新 Q，并限制 Q 的范围防止过大
    float Q_new = K * K * CIV;
    if (Q_new > 100.0f) Q_new = 100.0f;  // 根据实际调整上限
    if (Q_new < 1e-6f) Q_new = 1e-6f;
    kalman->Q = Q_new;

    return kalman->X_hat;
}


// ==================== 6轴 Mahony 算法 (增强稳定性) ====================
// ==================== 线缆冲击与外力加速度可信度 ====================
static void reset_external_accel_protection(void)
{
    external_accel_expected_x_cms2 = 0.0f;
    external_accel_expected_y_cms2 = 0.0f;
    external_accel_baseline_x_cms2 = 0.0f;
    external_accel_baseline_y_cms2 = 0.0f;
    external_accel_residual_x_cms2 = 0.0f;
    external_accel_residual_y_cms2 = 0.0f;
    external_accel_trust = 1.0f;
    external_accel_applied_trust = 1.0f;
    external_accel_shock_active = 0u;
    external_accel_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
    external_accel_guard_recovery = 1.0f;
    external_accel_trigger_time_s = 0.0f;
    external_accel_active_time_s = 0.0f;
    external_accel_exit_time_s = 0.0f;
    external_accel_recovery_time_s = 0.0f;
    external_accel_flow_recovery_frames = 0u;
    external_accel_control_ready_mask = 0u;
}

void imu_horizontal_set_impact_recovery_ready(uint8 axis_mask, uint8 ready)
{
    taskENTER_CRITICAL();
    if (ready) {
        external_accel_control_ready_mask |= axis_mask;
    } else {
        external_accel_control_ready_mask &= (uint8)(~axis_mask);
    }
    taskEXIT_CRITICAL();
}

static void external_accel_guard_clear_trigger(void)
{
    external_accel_trigger_time_s = 0.0f;
}

static void external_accel_guard_enter_active(void)
{
    external_accel_guard_stage = HORIZONTAL_IMPACT_GUARD_ACTIVE;
    external_accel_guard_recovery = 0.0f;
    external_accel_shock_active = 1u;
    external_accel_active_time_s = 0.0f;
    external_accel_exit_time_s = 0.0f;
    external_accel_recovery_time_s = 0.0f;
    external_accel_flow_recovery_frames = 0u;
    external_accel_control_ready_mask = 0u;
    external_accel_guard_clear_trigger();
}

static void update_external_accel_guard(float residual_norm_cms2, float dt)
{
    uint8 strong_impact =
        (residual_norm_cms2 >= IMU_IMPACT_STRONG_CMS2) ? 1u : 0u;
    uint8 normal_impact =
        (residual_norm_cms2 >= IMU_IMPACT_ENTER_CMS2) ? 1u : 0u;

    if (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) {
        external_accel_active_time_s += dt;
        if (residual_norm_cms2 <= IMU_IMPACT_EXIT_CMS2) {
            external_accel_exit_time_s += dt;
        } else {
            external_accel_exit_time_s = 0.0f;
        }
        if ((external_accel_active_time_s >= IMU_IMPACT_MIN_HOLD_S) &&
            (external_accel_exit_time_s >= IMU_IMPACT_EXIT_CONFIRM_S)) {
            external_accel_guard_stage = HORIZONTAL_IMPACT_GUARD_RECOVERY;
            external_accel_guard_recovery = 0.0f;
            external_accel_recovery_time_s = 0.0f;
            external_accel_flow_recovery_frames = 0u;
            external_accel_control_ready_mask = 0u;
            external_accel_guard_clear_trigger();
        }
        return;
    }

    if (strong_impact) {
        external_accel_guard_enter_active();
        return;
    }
    if (normal_impact) {
        external_accel_trigger_time_s += dt;
        if (external_accel_trigger_time_s >= IMU_IMPACT_CONFIRM_S) {
            external_accel_guard_enter_active();
            return;
        }
    } else {
        external_accel_guard_clear_trigger();
    }

    if (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_RECOVERY) {
        external_accel_recovery_time_s += dt;
        external_accel_guard_recovery = constrain_float(
            external_accel_recovery_time_s / IMU_IMPACT_FF_RECOVERY_S,
            0.0f, 1.0f);
    } else {
        external_accel_guard_recovery = 1.0f;
        external_accel_shock_active = 0u;
    }
}

static void external_accel_guard_try_finish(uint8 observation_ready,
                                            uint8 valid_x,
                                            uint8 valid_y)
{
    uint8 flow_fresh;
    uint8 kf_aligned;

    if (external_accel_guard_stage != HORIZONTAL_IMPACT_GUARD_RECOVERY) {
        return;
    }

    flow_fresh = (uint8)(horizontal_have_flow_x && horizontal_have_flow_y &&
                         (horizontal_flow_age_x_s <= HORIZONTAL_FLOW_FRESH_S) &&
                         (horizontal_flow_age_y_s <= HORIZONTAL_FLOW_FRESH_S));
    kf_aligned = (uint8)(
        fabsf(horizontal_velocity_kf.x.velocity_cms - horizontal_flow_vx_cms)
            <= IMU_IMPACT_KF_FLOW_MAX_DIFF_CMS &&
        fabsf(horizontal_velocity_kf.y.velocity_cms - horizontal_flow_vy_cms)
            <= IMU_IMPACT_KF_FLOW_MAX_DIFF_CMS);

    if (observation_ready) {
        if (valid_x && valid_y && flow_fresh && kf_aligned) {
            if (external_accel_flow_recovery_frames < 255u) {
                external_accel_flow_recovery_frames++;
            }
        } else {
            external_accel_flow_recovery_frames = 0u;
        }
    } else if (!flow_fresh || !kf_aligned) {
        external_accel_flow_recovery_frames = 0u;
    }

    if ((external_accel_flow_recovery_frames >=
         IMU_IMPACT_FLOW_RECOVERY_FRAMES) &&
        ((external_accel_control_ready_mask &
          (HORIZONTAL_IMPACT_READY_PITCH | HORIZONTAL_IMPACT_READY_ROLL)) ==
         (HORIZONTAL_IMPACT_READY_PITCH | HORIZONTAL_IMPACT_READY_ROLL))) {
        external_accel_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
        external_accel_guard_recovery = 1.0f;
        external_accel_shock_active = 0u;
        external_accel_active_time_s = 0.0f;
        external_accel_exit_time_s = 0.0f;
        external_accel_recovery_time_s = 0.0f;
        external_accel_flow_recovery_frames = 0u;
        external_accel_control_ready_mask = 0u;
        external_accel_guard_clear_trigger();
    }
}

static void update_external_accel_protection(float measured_acc_x_cms2,
                                             float measured_acc_y_cms2,
                                             float dt)
{
    float expected_x_cms2 = 0.0f;
    float expected_y_cms2 = 0.0f;
    float raw_residual_x_cms2;
    float raw_residual_y_cms2;
    float residual_norm_sq;
    float residual_norm_cms2;
    float scale_sq;
    float trust_raw;
    float recovery_alpha;
    float baseline_alpha;

    // 锁定或低高度时不进行外力判定，防止搬动飞机和地面振动污染残差基线。
    if ((lockflag != 0u) || (height < HORIZONTAL_GROUND_RESET_HEIGHT_MM)
    || !IS_VALID(measured_acc_x_cms2) || !IS_VALID(measured_acc_y_cms2)
    || !IS_VALID(dt) || (dt <= 0.0f)) {
        reset_external_accel_protection();
        return;
    }

    horizontal_velocity_predict_expected_accel(pitch_filtered,
                                               roll_filtered,
                                               &expected_x_cms2,
                                               &expected_y_cms2);
    if (!IS_VALID(expected_x_cms2) || !IS_VALID(expected_y_cms2)) {
        expected_x_cms2 = 0.0f;
        expected_y_cms2 = 0.0f;
    }
    expected_x_cms2 = constrain_float(expected_x_cms2,
                                      -HORIZONTAL_NAV_ACC_LIMIT_CMS2,
                                       HORIZONTAL_NAV_ACC_LIMIT_CMS2);
    expected_y_cms2 = constrain_float(expected_y_cms2,
                                      -HORIZONTAL_NAV_ACC_LIMIT_CMS2,
                                       HORIZONTAL_NAV_ACC_LIMIT_CMS2);

    raw_residual_x_cms2 = measured_acc_x_cms2 - expected_x_cms2;
    raw_residual_y_cms2 = measured_acc_y_cms2 - expected_y_cms2;
    external_accel_residual_x_cms2 = raw_residual_x_cms2
                                   - external_accel_baseline_x_cms2;
    external_accel_residual_y_cms2 = raw_residual_y_cms2
                                   - external_accel_baseline_y_cms2;

    scale_sq = IMU_EXTERNAL_ACCEL_SCALE_CMS2 * IMU_EXTERNAL_ACCEL_SCALE_CMS2;
    residual_norm_sq = external_accel_residual_x_cms2
                     * external_accel_residual_x_cms2
                     + external_accel_residual_y_cms2
                     * external_accel_residual_y_cms2;
    trust_raw = scale_sq / (scale_sq + residual_norm_sq);
    trust_raw = constrain_float(trust_raw, 0.0f, 1.0f);
    residual_norm_cms2 = sqrtf(residual_norm_sq);
    update_external_accel_guard(residual_norm_cms2, dt);

    // 线缆冲击到来时立即降权；残差消失后按现有恢复时间常数平滑恢复。
    if (trust_raw < external_accel_trust) {
        external_accel_trust = trust_raw;
    } else {
        recovery_alpha = dt / (HORIZONTAL_RECOVERY_TAU_S + dt);
        external_accel_trust += recovery_alpha
                              * (trust_raw - external_accel_trust);
    }
    external_accel_trust = constrain_float(external_accel_trust, 0.0f, 1.0f);

    // 纯光流保护期间冻结残差基线，避免把线缆冲击学习成稳态偏置。
    if (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) {
        baseline_alpha = dt / (HORIZONTAL_BIAS_ERROR_TAU_S + dt);
        external_accel_baseline_x_cms2 += baseline_alpha * external_accel_trust
                                       * (raw_residual_x_cms2
                                        - external_accel_baseline_x_cms2);
        external_accel_baseline_y_cms2 += baseline_alpha * external_accel_trust
                                       * (raw_residual_y_cms2
                                        - external_accel_baseline_y_cms2);
    }

    external_accel_expected_x_cms2 = expected_x_cms2;
    external_accel_expected_y_cms2 = expected_y_cms2;

#if IMU_EXTERNAL_ACCEL_PROTECT_ENABLE
    external_accel_applied_trust = external_accel_trust;
#else
    // A/B关闭时保留完整检测波形，仅取消对姿态和零偏学习的实际降权。
    external_accel_applied_trust = 1.0f;
#endif
}

// ==================== 6轴 Mahony 算法（带连续加速度可信度） ====================
void mahony_update_6dof(float Kp, float Ki, float dt, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float ex, ey, ez;
    float qa, qb, qc;

    // 0. 输入保护
    if(!IS_VALID(gx) || !IS_VALID(gy) || !IS_VALID(gz) || !IS_VALID(ax) || !IS_VALID(ay) || !IS_VALID(az)) {
        return;
    }

    // 【新增】备份旧四元数，用于 NaN 回滚
    float q0_old = mahony.q0;
    float q1_old = mahony.q1;
    float q2_old = mahony.q2;
    float q3_old = mahony.q3;

    // 1. 加速度修正
    float acc_sq_sum = ax*ax + ay*ay + az*az;
    if(acc_sq_sum > 0.0001f && IS_VALID(acc_sq_sum)) {
        recipNorm = invSqrt(acc_sq_sum);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // 估计重力方向向量
        halfvx = mahony.q1 * mahony.q3 - mahony.q0 * mahony.q2;
        halfvy = mahony.q0 * mahony.q1 + mahony.q2 * mahony.q3;
        halfvz = mahony.q0 * mahony.q0 - 0.5f + mahony.q3 * mahony.q3;

        // 叉积误差
        ex = (ay * halfvz - az * halfvy);
        ey = (az * halfvx - ax * halfvz);
        ez = (ax * halfvy - ay * halfvx);

        if(Ki > 0.0f) {
            // 外力冲击时只抑制本周期新积分，已经建立的积分继续参与姿态解算，不产生突变。
            mahony.integralFBx += Ki * ex * dt * external_accel_applied_trust;
            mahony.integralFBy += Ki * ey * dt * external_accel_applied_trust;
            mahony.integralFBz += Ki * ez * dt * external_accel_applied_trust;
            gx += mahony.integralFBx;
            gy += mahony.integralFBy;
            gz += mahony.integralFBz;
        } else {
            mahony.integralFBx = 0; mahony.integralFBy = 0; mahony.integralFBz = 0;
        }

        // 可信度只缩放加速度修正，陀螺积分和已有积分状态保持原有链路。
        gx += Kp * ex * external_accel_applied_trust;
        gy += Kp * ey * external_accel_applied_trust;
        gz += Kp * ez * external_accel_applied_trust;
    }

    // 2. 积分
    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = mahony.q0; qb = mahony.q1; qc = mahony.q2;
    mahony.q0 += (-qb * gx - qc * gy - mahony.q3 * gz);
    mahony.q1 += (qa * gx + qc * gz - mahony.q3 * gy);
    mahony.q2 += (qa * gy - qb * gz + mahony.q3 * gx);
    mahony.q3 += (qa * gz + qb * gy - qc * gx);

    // 3. 归一化与防炸机回滚
    float q_sq_sum = mahony.q0 * mahony.q0 + mahony.q1 * mahony.q1 + mahony.q2 * mahony.q2 + mahony.q3 * mahony.q3;

    // 【关键修改】如果结果出现 NaN 或 0，立刻回滚到上一次的状态
    if (!IS_VALID(q_sq_sum) || q_sq_sum < 1e-6f) {
        mahony.q0 = q0_old;
        mahony.q1 = q1_old;
        mahony.q2 = q2_old;
        mahony.q3 = q3_old;
    } else {
        recipNorm = invSqrt(q_sq_sum);
        mahony.q0 *= recipNorm;
        mahony.q1 *= recipNorm;
        mahony.q2 *= recipNorm;
        mahony.q3 *= recipNorm;
    }
}

void imu_process_data(float current_kp, float current_ki, float dt) {
    imu963ra_get_acc();
    imu963ra_get_gyro();


    float raw_gx = imu963ra_gyro_transition(imu963ra_gyro_x) - gyro_offset[0];
    float raw_gy = imu963ra_gyro_transition(imu963ra_gyro_y) - gyro_offset[1];
    float raw_gz = imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_offset[2];

    //printf("%f,%f,%f\r\n",raw_gx,raw_gy,raw_gz);
    float raw_ax = imu963ra_acc_transition(imu963ra_acc_x);
    float raw_ay = imu963ra_acc_transition(imu963ra_acc_y);
    float raw_az = imu963ra_acc_transition(imu963ra_acc_z);
    raw_accl_m_s2[0] = raw_ax;
    raw_accl_m_s2[1] = raw_ay;
    raw_accl_m_s2[2] = raw_az;
    //printf("%f,%f,%f\r\n",raw_ax,raw_ay,raw_az);



    roll_gyro_filtered  = pt2_filter_apply(&gyro_pt2[0], raw_gx);
    pitch_gyro_filtered = pt2_filter_apply(&gyro_pt2[1], raw_gy);
    yaw_gyro_filtered   = pt2_filter_apply(&gyro_pt2[2], raw_gz);
    roll_gyro_filtered  = adaptive_kalman_filter(roll_gyro_filtered,  &akf_gx);
    pitch_gyro_filtered = adaptive_kalman_filter(pitch_gyro_filtered, &akf_gy);
    yaw_gyro_filtered   = adaptive_kalman_filter(yaw_gyro_filtered,   &akf_gz);

    accl_m_s2[0] = pt1_filter_apply(&acc_pt1[0], raw_ax);
    accl_m_s2[1] = pt1_filter_apply(&acc_pt1[1], raw_ay);
    accl_m_s2[2] = pt1_filter_apply(&acc_pt1[2], raw_az);


    mahony_update_6dof(current_kp, current_ki, dt,
                  roll_gyro_filtered  * (M_PI / 180.0f),
                  pitch_gyro_filtered * (M_PI / 180.0f),
                  yaw_gyro_filtered   * (M_PI / 180.0f),
                  accl_m_s2[0], // Acc X
                  accl_m_s2[1], // Acc Y
                  accl_m_s2[2]);
}


void adaptive_kalman_init(adaptive_kalman* filter,
                          float init_state,
                          float P0,
                          float R0,
                          float Q0)
{
    // 清零结构体
    memset(filter, 0, sizeof(adaptive_kalman));

    filter->X_hat = init_state;
    filter->P = P0;
    filter->R = R0;
    filter->Q = Q0;
    filter->iv_idx = 0;
    // IV_buf 已经通过 memset 清零，无需再循环赋值
}
/**
 * @brief 校准加速度计零偏（静态时调用）
 */
void calibrate_acc_offset(void)
{
    const int samples = 500;
    double sum_z = 0, sum_x_bias = 0, sum_y_bias = 0;

    for (int i = 0; i < samples; i++) {
        imu963ra_get_acc();
        float raw_ax = imu963ra_acc_transition(imu963ra_acc_x);
        float raw_ay = imu963ra_acc_transition(imu963ra_acc_y);
        float raw_az = imu963ra_acc_transition(imu963ra_acc_z);

        // 使用当前四元数
        float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;

        // 1. Z轴零偏 (导航系)
        float acc_nav_z = 2.0f*(q1*q3 - q0*q2)*raw_ax
                        + 2.0f*(q0*q1 + q2*q3)*raw_ay
                        + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*raw_az;
        sum_z += acc_nav_z;

        // 2. X/Y轴传感器零偏 (机体系)
        // 机体倾斜时，加速度计理论上应该测到的重力投影分量：
        float expected_ax = 2.0f * (q1*q3 - q0*q2) * 9.81f;
        float expected_ay = 2.0f * (q0*q1 + q2*q3) * 9.81f;

        // 传感器固有零偏 = 实际读取值 - 理论重力分量
        sum_x_bias += (raw_ax - expected_ax);
        sum_y_bias += (raw_ay - expected_ay);

        system_delay_ms(2);
    }

    acc_nav_z_offset = (float)(sum_z / samples) - 9.81f;
    acc_body_x_offset = (float)(sum_x_bias / samples);
    acc_body_y_offset = (float)(sum_y_bias / samples);

    printf("Acc Z offset (nav) = %f m/s2\r\n", acc_nav_z_offset);
    printf("Acc X/Y bias (body) = %f, %f m/s2\r\n", acc_body_x_offset, acc_body_y_offset);
}

// ==================== 主逻辑 ====================

void imu_init(void) {
    horizontal_velocity_kf_init(&horizontal_velocity_kf);

    while(imu963ra_init()) {
        system_delay_ms(100);
    }

    last_wake_time = xTaskGetTickCount();

    pt2_filter_init(&gyro_pt2[0], GYRO_CUTOFF_HZ, 0.001f);
    pt2_filter_init(&gyro_pt2[1], GYRO_CUTOFF_HZ, 0.001f);
    pt2_filter_init(&gyro_pt2[2], GYRO_CUTOFF_HZ, 0.001f);

    for(int i=0; i<3; i++) {
        pt1_filter_init(&acc_pt1[i], ACC_CUTOFF_HZ, 0.001f);
    }
    pt1_filter_init(&nav_acc_pt1[0], NAV_ACC_CUTOFF_HZ, 0.001f);
    pt1_filter_init(&nav_acc_pt1[1], NAV_ACC_CUTOFF_HZ, 0.001f);

    printf("IMU: Calibrating Gyro...\r\n");
    double gyro_sum[3] = {0};
    const int calib_samples = 2000;

    for(int i=0; i<200; i++) {
        imu963ra_get_gyro();
        system_delay_ms(2);
    }

    for(int i=0; i<calib_samples; i++) {
        imu963ra_get_gyro();
        float raw_gx = imu963ra_gyro_transition(imu963ra_gyro_x);
        float raw_gy = imu963ra_gyro_transition(imu963ra_gyro_y);
        float raw_gz = imu963ra_gyro_transition(imu963ra_gyro_z);

        if(IS_VALID(raw_gx)) gyro_sum[0] += raw_gx;
        if(IS_VALID(raw_gy)) gyro_sum[1] += raw_gy;
        if(IS_VALID(raw_gz)) gyro_sum[2] += raw_gz;
        system_delay_ms(2);
    }

    gyro_offset[0] = (float)(gyro_sum[0] / calib_samples);
    gyro_offset[1] = (float)(gyro_sum[1] / calib_samples);
    gyro_offset[2] = (float)(gyro_sum[2] / calib_samples);

        //UBaseType_t mark1 = uxTaskGetStackHighWaterMark(NULL);
    //printf("Before measure_all_gyro_R_values: %d\n", mark1);
    measure_all_gyro_R_values(500);
    //UBaseType_t mark2 = uxTaskGetStackHighWaterMark(NULL);
    //printf("After measure_all_gyro_R_values: %d\n", mark2);

    //使用实测的 R 值初始化卡尔曼滤波器
    adaptive_kalman_init(&akf_gx, 0.0f, 0.01f, R_roll,  0.002f);
    adaptive_kalman_init(&akf_gy, 0.0f, 0.01f, R_pitch, 0.002f);
    adaptive_kalman_init(&akf_gz, 0.0f, 0.01f, R_yaw,   0.002f);

    printf("IMU: Converging Gravity...\r\n");
    for(int i=0; i<1000; i++) {
        imu_process_data(10.0f, 0.0f, 0.002f);
        system_delay_ms(2);
    }

    // 初始校准逻辑
    float q0=mahony.q0, q1=mahony.q1, q2=mahony.q2, q3=mahony.q3;
    float abs_roll  = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = CLAMP(sinp, -1.0f, 1.0f);
    float abs_pitch = asin(sinp) * (180.0f / M_PI);
    float abs_yaw   = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI);

    level_offset_pitch = abs_pitch;
    level_offset_roll  = abs_roll;
    level_offset_yaw   = abs_yaw;

    system_delay_ms(1000);  // 等待滤波器收敛
    calibrate_acc_offset();



    printf("IMU Ready.\r\n");
}


void imu_process(void) {
    TickType_t current_time = xTaskGetTickCount();
    TickType_t delta_ticks = current_time - last_wake_time;
    last_wake_time = current_time;

    if(delta_ticks > 0 && delta_ticks < 20) {
        dt_s = (float)delta_ticks / configTICK_RATE_HZ;
    } else {
        dt_s = 0.001f;
    }

    imu_process_data(MAHONY_KP_NORMAL, MAHONY_KI_NORMAL, dt_s);

    integrate_vertical_velocity(dt_s);
    integrate_horizontal_velocity(dt_s);

    // 3. 四元数转欧拉角
    float q0=mahony.q0, q1=mahony.q1, q2=mahony.q2, q3=mahony.q3;

    // Roll 计算
    float abs_roll  = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);

    // Pitch 计算 (增加极其严格的 NaN 防护)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    // 确保 sinp 不是 NaN 再进行 CLAMP，如果是 NaN 则强制设为 0（平飞状态）或保持旧值
    if (!IS_VALID(sinp)) sinp = 0.0f;

    // 强制截断到 -1.0 ~ 1.0，防止浮点误差导致 1.000001 这种让 asin 返回 NaN 的情况
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    float abs_pitch = asin(sinp) * (180.0f / M_PI);

    // Yaw 计算
    float abs_yaw   = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI);

    // 4. 减去初始 Offset
    float new_pitch = abs_pitch;//- level_offset_pitch;
    float new_roll  = abs_roll -180;// - level_offset_roll;
    float new_yaw   = abs_yaw   - level_offset_yaw;



    // 【修改】Roll 角范围修正：确保在 -180 ~ 180 之间
    // 原理：如果 new_roll 是 185，则变成 -175
    while (new_roll > 180.0f) new_roll -= 360.0f;
    while (new_roll < -180.0f) new_roll += 360.0f;

    // Yaw 角范围修正 (原本已有)
    while (new_yaw > 180.0f) new_yaw -= 360.0f;
    while (new_yaw < -180.0f) new_yaw += 360.0f;

    // 5. 输出赋值 (NaN 保护: 如果算出 NaN，保持上一次的值，绝对不更新)
    if(IS_VALID(new_pitch)) pitch_filtered = new_pitch;
    if(IS_VALID(new_roll))  roll_filtered  = new_roll;
    if(IS_VALID(new_yaw))   yaw_filtered   = new_yaw;


}


/**
 * @brief 利用四元数将机体加速度转换到导航系，得到垂直运动加速度
 * @return 垂直加速度 (cm/s2)
 */
static float get_vertical_acceleration(void)
{
    float ax = accl_m_s2[0];
    float ay = accl_m_s2[1];
    float az = accl_m_s2[2];

    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;

    // 导航系Z轴加速度
    float acc_nav_z = 2.0f * (q1 * q3 - q0 * q2) * ax
                    + 2.0f * (q0 * q1 + q2 * q3) * ay
                    + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az;
    return (acc_nav_z - 9.81f - acc_nav_z_offset) * 100.0f;
}

/**
 * @brief 1ms 周期：整合加速度，执行预测
 */
void integrate_vertical_velocity(float dt)
{
    float acc_vertical = get_vertical_acceleration();
    float acc_corrected = acc_vertical - est_a_bias_cms2;
    vertical_accel_cms2 = acc_vertical;
    vertical_accel_corrected_cms2 = acc_corrected;

    est_z_cm += est_v_cms * dt;
    est_v_cms += acc_corrected * dt;


    if (tof_blind_timer > 30) {
        est_v_cms *= 0.99f;
    }

    est_v_cms = constrain_float(est_v_cms, -800.0f, 800.0f);
    est_a_bias_cms2 = constrain_float(est_a_bias_cms2, -300.0f, 300.0f);

    if (height < 50.0f && fabsf(pitch_filtered) < 5.0f && fabsf(roll_filtered) < 5.0f) {
        est_v_cms = 0.0f;
        est_z_cm = height / 10.0f;
        est_a_bias_cms2 = 0.0f;
    }

    vertical_velocity_smoothed += 0.08f * (est_v_cms - vertical_velocity_smoothed);
}

/** * @brief 10ms 周期：利用 ToF 激光高度进行修正 (状态观测器更新)
 */
void correct_vertical_velocity(float measured_height_mm, float dt_update)
{
    static uint8_t first = 1;
    float measured_z_cm = measured_height_mm / 10.0f;

    if (first) {
        est_z_cm = measured_z_cm;
        est_v_cms = 0.0f;
        vertical_velocity_smoothed = 0.0f;
        est_a_bias_cms2 = 0.0f;
        vertical_tof_error_cm = 0.0f;
        vertical_tof_z_correction_cm = 0.0f;
        vertical_tof_v_correction_cms = 0.0f;
        first = 0;
        return;
    }

    float err_cm = measured_z_cm - est_z_cm;

    if (err_cm > 15.0f) err_cm = 15.0f;
    if (err_cm < -15.0f) err_cm = -15.0f;
    vertical_tof_error_cm = err_cm;


    if (tof_blind_timer > 0) {
        vertical_tof_z_correction_cm = 0.1f * err_cm;
        vertical_tof_v_correction_cms = 0.0f;
        est_z_cm += vertical_tof_z_correction_cm;
        return;
    }

    float K_z = 0.25f;
    float K_v = 1.5f;
    float K_a = 0.02f;

    vertical_tof_z_correction_cm = K_z * err_cm;
    vertical_tof_v_correction_cms = K_v * err_cm;
    est_z_cm  += vertical_tof_z_correction_cm;
    est_v_cms += vertical_tof_v_correction_cms;
    est_a_bias_cms2 -= K_a * err_cm;
}


/**
 * @brief 获取去重力后的机体线性加速度 (cm/s2)
 */
static void get_horizontal_acceleration(float *acc_x, float *acc_y) {
    float ax = raw_accl_m_s2[0] - acc_body_x_offset;
    float ay = raw_accl_m_s2[1] - acc_body_y_offset;

    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;

    // 计算重力向量在机体坐标系下的投影 (1g)
    float gx = 2.0f * (q1 * q3 - q0 * q2);
    float gy = 2.0f * (q0 * q1 + q2 * q3);

    // 减去重力分量，得到纯粹的线性运动加速度 (m/s2)
    float lin_ax = ax - gx * 9.81f;
    float lin_ay = ay - gy * 9.81f;

    // 导航坐标系与光流保持一致：+X 向右，+Y 向后。
    *acc_x = constrain_float(pt1_filter_apply(&nav_acc_pt1[0], -lin_ay * 100.0f),
                             -HORIZONTAL_NAV_ACC_LIMIT_CMS2,
                              HORIZONTAL_NAV_ACC_LIMIT_CMS2);
    *acc_y = constrain_float(pt1_filter_apply(&nav_acc_pt1[1], lin_ax * 100.0f),
                             -HORIZONTAL_NAV_ACC_LIMIT_CMS2,
                              HORIZONTAL_NAV_ACC_LIMIT_CMS2);
}

static float horizontal_get_correction_tau(float innovation_cms)
{
    float abs_innovation = fabsf(innovation_cms);
    float ratio;

    if(abs_innovation <= HORIZONTAL_INNOVATION_SLOW_CMS) {
        return HORIZONTAL_CORRECTION_TAU_SLOW_S;
    }
    if(abs_innovation >= HORIZONTAL_INNOVATION_FAST_CMS) {
        return HORIZONTAL_CORRECTION_TAU_FAST_S;
    }

    ratio = (abs_innovation - HORIZONTAL_INNOVATION_SLOW_CMS)
          / (HORIZONTAL_INNOVATION_FAST_CMS - HORIZONTAL_INNOVATION_SLOW_CMS);
    return HORIZONTAL_CORRECTION_TAU_SLOW_S
         + ratio * (HORIZONTAL_CORRECTION_TAU_FAST_S - HORIZONTAL_CORRECTION_TAU_SLOW_S);
}

static void reset_horizontal_velocity_estimator_internal(void)
{
    est_vx_cms = 0.0f;
    est_vy_cms = 0.0f;
    est_pos_x_cm = 0.0f;
    est_pos_y_cm = 0.0f;
    est_a_bias_x_cms2 = 0.0f;
    est_a_bias_y_cms2 = 0.0f;
    horizontal_velocity_kf_reset(&horizontal_velocity_kf);
    horizontal_kf_trim_acc_sum_x_cms2 = 0.0f;
    horizontal_kf_trim_acc_sum_y_cms2 = 0.0f;
    horizontal_kf_trim_acc_count = 0u;
    horizontal_kf_trim_seeded = 0u;
    horizontal_bias_error_x_cms = 0.0f;
    horizontal_bias_error_y_cms = 0.0f;
    horizontal_flow_vx_cms = 0.0f;
    horizontal_flow_vy_cms = 0.0f;
    horizontal_flow_age_x_s = 1000.0f;
    horizontal_flow_age_y_s = 1000.0f;
    horizontal_correction_remaining_x_s = 0.0f;
    horizontal_correction_remaining_y_s = 0.0f;
    horizontal_last_innovation_x_cms = 0.0f;
    horizontal_last_innovation_y_cms = 0.0f;
    horizontal_last_nav_acc_x_cms2 = 0.0f;
    horizontal_last_nav_acc_y_cms2 = 0.0f;
    horizontal_last_imu_predicted_vx_cms = 0.0f;
    horizontal_last_imu_predicted_vy_cms = 0.0f;
    horizontal_last_flow_correction_x_cms = 0.0f;
    horizontal_last_flow_correction_y_cms = 0.0f;
    horizontal_last_correction_tau_x_s = HORIZONTAL_CORRECTION_TAU_SLOW_S;
    horizontal_last_correction_tau_y_s = HORIZONTAL_CORRECTION_TAU_SLOW_S;
    horizontal_flow_recovery_frames_x = 0;
    horizontal_flow_recovery_frames_y = 0;
    horizontal_flow_trust_level_x = 0u;
    horizontal_flow_trust_level_y = 0u;
    horizontal_flow_consistency_error_x_cms = 0.0f;
    horizontal_flow_consistency_error_y_cms = 0.0f;
    horizontal_have_flow_x = 0;
    horizontal_have_flow_y = 0;
    horizontal_last_valid_flow_x_cms = 0.0f;
    horizontal_last_valid_flow_y_cms = 0.0f;
    horizontal_visual_vx_cms = 0.0f;
    horizontal_visual_vy_cms = 0.0f;
    horizontal_visual_age_x_s = 1000.0f;
    horizontal_visual_age_y_s = 1000.0f;
    horizontal_visual_correction_remaining_x_s = 0.0f;
    horizontal_visual_correction_remaining_y_s = 0.0f;
    horizontal_last_visual_innovation_x_cms = 0.0f;
    horizontal_last_visual_innovation_y_cms = 0.0f;
    horizontal_have_visual_x = 0u;
    horizontal_have_visual_y = 0u;
    horizontal_visual_source = 0u;
    horizontal_kf_accel_used_x_cms2 = 0.0f;
    horizontal_kf_accel_used_y_cms2 = 0.0f;
    horizontal_flow_update_count_x = 0u;
    horizontal_flow_update_count_y = 0u;
    horizontal_flow_update_age_x_s = 1000.0f;
    horizontal_flow_update_age_y_s = 1000.0f;
    reset_external_accel_protection();
    horizontal_full_reset_count++;
}

static float horizontal_control_decay(float value, float dt)
{
    float alpha = dt / HORIZONTAL_CONTROL_DECAY_S;

    alpha = constrain_float(alpha, 0.0f, 1.0f);
    return value * (1.0f - alpha);
}

static void update_horizontal_control_axis(float kf_velocity_cms,
                                           float flow_velocity_cms,
                                           float flow_age_s,
                                           uint8 flow_have,
                                           float visual_age_s,
                                           uint8 visual_have,
                                           float dt,
                                           float *control_velocity_cms,
                                           uint8 *control_valid,
                                           float *control_confidence,
                                           uint8 *control_source,
                                           float *last_valid_flow_cms,
                                           uint8 impact_flow_only)
{
    uint8 flow_valid = (uint8)(flow_have &&
                               (flow_age_s <= HORIZONTAL_FLOW_FRESH_S));
    uint8 visual_valid = (uint8)(visual_have &&
                                 (visual_age_s <= HORIZONTAL_VISUAL_FRESH_S));

    if (impact_flow_only) {
        if (flow_have && (flow_age_s <= HORIZONTAL_CONTROL_FLOW_HOLD_S)) {
            *last_valid_flow_cms = flow_velocity_cms;
            *control_velocity_cms = flow_velocity_cms;
            *control_valid = 1u;
            *control_confidence = 1.0f;
            *control_source = HORIZONTAL_CONTROL_SOURCE_FLOW_IMPACT;
        } else if (flow_have &&
                   (flow_age_s <= HORIZONTAL_CONTROL_FLOW_TIMEOUT_S)) {
            // 冲击期间只保持最后光流，严禁混入冻结的KF速度。
            *control_velocity_cms = *last_valid_flow_cms;
            *control_valid = 1u;
            *control_confidence = 1.0f
                                - 0.5f * (flow_age_s -
                                          HORIZONTAL_CONTROL_FLOW_HOLD_S)
                                / (HORIZONTAL_CONTROL_FLOW_TIMEOUT_S -
                                   HORIZONTAL_CONTROL_FLOW_HOLD_S);
            *control_source = HORIZONTAL_CONTROL_SOURCE_FLOW_HOLD;
        } else {
            *last_valid_flow_cms = horizontal_control_decay(
                *last_valid_flow_cms, dt);
            *control_velocity_cms = *last_valid_flow_cms;
            *control_valid = 0u;
            *control_confidence = 0.0f;
            *control_source = HORIZONTAL_CONTROL_SOURCE_FLOW_TIMEOUT;
        }
        return;
    }

    if (flow_valid || visual_valid) {
        if (flow_valid) {
            *last_valid_flow_cms = flow_velocity_cms;
        }
        *control_velocity_cms = kf_velocity_cms;
        *control_valid = 1u;
        *control_confidence = 1.0f;
        *control_source = HORIZONTAL_CONTROL_SOURCE_KF_VALID;
    } else if (flow_age_s <= HORIZONTAL_CONTROL_FLOW_HOLD_S) {
        *control_velocity_cms = kf_velocity_cms;
        *control_valid = 1u;
        *control_confidence = 1.0f;
        *control_source = HORIZONTAL_CONTROL_SOURCE_KF_SHORT_PREDICT;
    } else if (flow_age_s <= HORIZONTAL_CONTROL_FLOW_TIMEOUT_S) {
        /* 保留最近有效光流作为主参考，KF只提供短时连续性。 */
        *control_velocity_cms = 0.70f * (*last_valid_flow_cms)
                              + 0.30f * kf_velocity_cms;
        *control_valid = 1u;
        *control_confidence = 1.0f
                            - 0.5f * (flow_age_s - HORIZONTAL_CONTROL_FLOW_HOLD_S)
                            / (HORIZONTAL_CONTROL_FLOW_TIMEOUT_S
                               - HORIZONTAL_CONTROL_FLOW_HOLD_S);
        *control_source = HORIZONTAL_CONTROL_SOURCE_FLOW_HOLD;
    } else {
        *last_valid_flow_cms = horizontal_control_decay(*last_valid_flow_cms, dt);
        *control_velocity_cms = *last_valid_flow_cms;
        *control_valid = 0u;
        *control_confidence = 0.0f;
        *control_source = HORIZONTAL_CONTROL_SOURCE_FLOW_TIMEOUT;
    }
}

static void publish_horizontal_state(float dt)
{
    taskENTER_CRITICAL();
    horizontal_published_state.vx_cms = est_vx_cms;
    horizontal_published_state.vy_cms = est_vy_cms;
#if HORIZONTAL_KF_CONTROL_ENABLE
    // 速度环使用KF发布值；旧估计量仍保留用于对比诊断，位置积分在末端同步切换。
    horizontal_published_state.vx_cms = horizontal_velocity_kf.x.velocity_cms;
    horizontal_published_state.vy_cms = horizontal_velocity_kf.y.velocity_cms;
#endif
    update_horizontal_control_axis(horizontal_published_state.vx_cms,
                                   horizontal_flow_vx_cms,
                                   horizontal_flow_age_x_s,
                                   horizontal_have_flow_x,
                                   horizontal_visual_age_x_s,
                                   HORIZONTAL_KF_VISUAL_FUSION_ENABLE ?
                                   horizontal_have_visual_x : 0u,
                                   dt,
                                   &horizontal_published_state.control_vx_cms,
                                   &horizontal_published_state.control_velocity_valid_x,
                                   &horizontal_published_state.control_velocity_confidence_x,
                                   &horizontal_published_state.control_velocity_source_x,
                                   &horizontal_last_valid_flow_x_cms,
                                   (uint8)(external_accel_guard_stage !=
                                             HORIZONTAL_IMPACT_GUARD_NORMAL));
    horizontal_published_state.control_last_flow_vx_cms =
        horizontal_last_valid_flow_x_cms;
    update_horizontal_control_axis(horizontal_published_state.vy_cms,
                                   horizontal_flow_vy_cms,
                                   horizontal_flow_age_y_s,
                                   horizontal_have_flow_y,
                                   horizontal_visual_age_y_s,
                                   HORIZONTAL_KF_VISUAL_FUSION_ENABLE ?
                                   horizontal_have_visual_y : 0u,
                                   dt,
                                   &horizontal_published_state.control_vy_cms,
                                   &horizontal_published_state.control_velocity_valid_y,
                                   &horizontal_published_state.control_velocity_confidence_y,
                                   &horizontal_published_state.control_velocity_source_y,
                                   &horizontal_last_valid_flow_y_cms,
                                   (uint8)(external_accel_guard_stage !=
                                             HORIZONTAL_IMPACT_GUARD_NORMAL));
    horizontal_published_state.control_last_flow_vy_cms =
        horizontal_last_valid_flow_y_cms;
    horizontal_published_state.pos_x_cm = est_pos_x_cm;
    horizontal_published_state.pos_y_cm = est_pos_y_cm;
    horizontal_published_state.acc_bias_x_cms2 = est_a_bias_x_cms2;
    horizontal_published_state.acc_bias_y_cms2 = est_a_bias_y_cms2;
    horizontal_published_state.flow_age_x_s = horizontal_flow_age_x_s;
    horizontal_published_state.flow_age_y_s = horizontal_flow_age_y_s;
    horizontal_published_state.flow_age_s = (horizontal_flow_age_x_s > horizontal_flow_age_y_s)
                                        ? horizontal_flow_age_x_s : horizontal_flow_age_y_s;
    horizontal_published_state.flow_vx_cms = horizontal_flow_vx_cms;
    horizontal_published_state.flow_vy_cms = horizontal_flow_vy_cms;
    horizontal_published_state.visual_vx_cms = horizontal_visual_vx_cms;
    horizontal_published_state.visual_vy_cms = horizontal_visual_vy_cms;
    horizontal_published_state.visual_age_s =
        (horizontal_visual_age_x_s > horizontal_visual_age_y_s) ?
        horizontal_visual_age_x_s : horizontal_visual_age_y_s;
    horizontal_published_state.innovation_x_cms = horizontal_last_innovation_x_cms;
    horizontal_published_state.innovation_y_cms = horizontal_last_innovation_y_cms;
    horizontal_published_state.visual_innovation_x_cms =
        horizontal_last_visual_innovation_x_cms;
    horizontal_published_state.visual_innovation_y_cms =
        horizontal_last_visual_innovation_y_cms;
    horizontal_published_state.nav_acc_x_cms2 = horizontal_last_nav_acc_x_cms2;
    horizontal_published_state.nav_acc_y_cms2 = horizontal_last_nav_acc_y_cms2;
    horizontal_published_state.expected_acc_x_cms2 = external_accel_expected_x_cms2;
    horizontal_published_state.expected_acc_y_cms2 = external_accel_expected_y_cms2;
    horizontal_published_state.imu_predicted_vx_cms =
        horizontal_last_imu_predicted_vx_cms;
    horizontal_published_state.imu_predicted_vy_cms =
        horizontal_last_imu_predicted_vy_cms;
    horizontal_published_state.flow_correction_x_cms =
        horizontal_last_flow_correction_x_cms;
    horizontal_published_state.flow_correction_y_cms =
        horizontal_last_flow_correction_y_cms;
    horizontal_published_state.external_accel_residual_x_cms2 =
        external_accel_residual_x_cms2;
    horizontal_published_state.external_accel_residual_y_cms2 =
        external_accel_residual_y_cms2;
    horizontal_published_state.external_accel_trust = external_accel_trust;
    horizontal_published_state.external_accel_guard_stage =
        external_accel_guard_stage;
    horizontal_published_state.external_accel_guard_recovery =
        external_accel_guard_recovery;
    horizontal_published_state.external_accel_flow_recovery_frames =
        external_accel_flow_recovery_frames;
    horizontal_published_state.correction_tau_x_s = horizontal_last_correction_tau_x_s;
    horizontal_published_state.correction_tau_y_s = horizontal_last_correction_tau_y_s;
    horizontal_published_state.full_reset_count = horizontal_full_reset_count;
    horizontal_published_state.position_reset_count = horizontal_position_reset_count;
    horizontal_published_state.flow_valid_x = (horizontal_have_flow_x
                                             && (horizontal_flow_age_x_s <= HORIZONTAL_FLOW_FRESH_S));
    horizontal_published_state.flow_valid_y = (horizontal_have_flow_y
                                             && (horizontal_flow_age_y_s <= HORIZONTAL_FLOW_FRESH_S));
    horizontal_published_state.flow_valid = horizontal_published_state.flow_valid_x
                                          && horizontal_published_state.flow_valid_y;
    horizontal_published_state.visual_valid_x =
        (uint8)(horizontal_have_visual_x &&
                (horizontal_visual_age_x_s <= HORIZONTAL_VISUAL_FRESH_S));
    horizontal_published_state.visual_valid_y =
        (uint8)(horizontal_have_visual_y &&
                (horizontal_visual_age_y_s <= HORIZONTAL_VISUAL_FRESH_S));
    horizontal_published_state.visual_valid =
        horizontal_published_state.visual_valid_x &&
        horizontal_published_state.visual_valid_y;
    horizontal_published_state.velocity_valid_x =
        horizontal_published_state.flow_valid_x ||
        horizontal_published_state.visual_valid_x;
    horizontal_published_state.velocity_valid_y =
        horizontal_published_state.flow_valid_y ||
        horizontal_published_state.visual_valid_y;
    horizontal_published_state.velocity_valid =
        horizontal_published_state.velocity_valid_x &&
        horizontal_published_state.velocity_valid_y;
    horizontal_published_state.visual_source = horizontal_visual_source;
    horizontal_published_state.flow_recovery_frames_x = horizontal_flow_recovery_frames_x;
    horizontal_published_state.flow_recovery_frames_y = horizontal_flow_recovery_frames_y;
    horizontal_published_state.flow_recovery_frames =
        (horizontal_flow_recovery_frames_x < horizontal_flow_recovery_frames_y)
      ? horizontal_flow_recovery_frames_x : horizontal_flow_recovery_frames_y;
    horizontal_published_state.flow_trust_level_x = horizontal_flow_trust_level_x;
    horizontal_published_state.flow_trust_level_y = horizontal_flow_trust_level_y;
    horizontal_published_state.flow_consistency_error_x_cms =
        horizontal_flow_consistency_error_x_cms;
    horizontal_published_state.flow_consistency_error_y_cms =
        horizontal_flow_consistency_error_y_cms;
    horizontal_published_state.kf_vx_cms = horizontal_velocity_kf.x.velocity_cms;
    horizontal_published_state.kf_vy_cms = horizontal_velocity_kf.y.velocity_cms;
    horizontal_published_state.kf_acc_bias_x_cms2 =
        horizontal_velocity_kf.x.accel_bias_cms2;
    horizontal_published_state.kf_acc_bias_y_cms2 =
        horizontal_velocity_kf.y.accel_bias_cms2;
    horizontal_published_state.kf_innovation_x_cms =
        horizontal_velocity_kf.x.innovation_cms;
    horizontal_published_state.kf_innovation_y_cms =
        horizontal_velocity_kf.y.innovation_cms;
    horizontal_published_state.kf_velocity_gain_x =
        horizontal_velocity_kf.x.velocity_gain;
    horizontal_published_state.kf_velocity_gain_y =
        horizontal_velocity_kf.y.velocity_gain;
    horizontal_published_state.kf_bias_gain_x =
        horizontal_velocity_kf.x.bias_gain;
    horizontal_published_state.kf_bias_gain_y =
        horizontal_velocity_kf.y.bias_gain;
    horizontal_published_state.kf_reset_count = horizontal_velocity_kf.reset_count;
    horizontal_published_state.kf_trim_acc_count = horizontal_kf_trim_acc_count;
    horizontal_published_state.kf_trim_seeded = horizontal_kf_trim_seeded;
    horizontal_published_state.external_accel_shock_active = external_accel_shock_active;
    horizontal_published_state.estimator_build_marker =
        HORIZONTAL_ESTIMATOR_BUILD_MARKER;
    horizontal_published_state.estimator_config_flags =
        (HORIZONTAL_KF_CONTROL_ENABLE ? 0x01u : 0u)
      | (HORIZONTAL_KF_VISUAL_FUSION_ENABLE ? 0x02u : 0u)
      | (IMU_EXTERNAL_ACCEL_PROTECT_ENABLE ? 0x04u : 0u)
      | HORIZONTAL_KF_CONFIG_RESIDUAL_WEIGHT
      | HORIZONTAL_KF_CONFIG_BIAS_FREEZE
      | ((uint32)HORIZONTAL_KF_ACCEL_WEIGHT_MODE << 8);
    horizontal_published_state.kf_accel_used_x_cms2 =
        horizontal_kf_accel_used_x_cms2;
    horizontal_published_state.kf_accel_used_y_cms2 =
        horizontal_kf_accel_used_y_cms2;
    horizontal_published_state.flow_update_count_x =
        horizontal_flow_update_count_x;
    horizontal_published_state.flow_update_count_y =
        horizontal_flow_update_count_y;
    horizontal_published_state.flow_update_age_x_s =
        horizontal_flow_update_age_x_s;
    horizontal_published_state.flow_update_age_y_s =
        horizontal_flow_update_age_y_s;
    taskEXIT_CRITICAL();
}

void imu_horizontal_submit_flow(float flow_vx_cms, float flow_vy_cms,
                                float flow_dt_s, uint8 valid_x, uint8 valid_y,
                                uint8 trust_level_x, uint8 trust_level_y,
                                float consistency_error_x_cms,
                                float consistency_error_y_cms)
{
    uint8 observation_valid_x = valid_x;
    uint8 observation_valid_y = valid_y;

    if(!IS_VALID(flow_vx_cms)) {
        observation_valid_x = 0;
    }
    if(!IS_VALID(flow_vy_cms)) {
        observation_valid_y = 0;
    }
    if(!IS_VALID(flow_dt_s)) {
        observation_valid_x = 0;
        observation_valid_y = 0;
    }
    if(observation_valid_x || observation_valid_y) {
        flow_dt_s = constrain_float(flow_dt_s, 0.01f, 0.05f);
    }

    taskENTER_CRITICAL();
    pending_flow_observation.vx_cms = flow_vx_cms;
    pending_flow_observation.vy_cms = flow_vy_cms;
    pending_flow_observation.dt_s = flow_dt_s;
    pending_flow_observation.valid_x = observation_valid_x;
    pending_flow_observation.valid_y = observation_valid_y;
    pending_flow_observation.trust_level_x = trust_level_x;
    pending_flow_observation.trust_level_y = trust_level_y;
    pending_flow_observation.consistency_error_x_cms = consistency_error_x_cms;
    pending_flow_observation.consistency_error_y_cms = consistency_error_y_cms;
    pending_flow_observation_ready = 1;
    taskEXIT_CRITICAL();
}

void imu_horizontal_submit_visual_velocity(float vx_right_cms,
                                           float vy_back_cms,
                                           float observation_dt_s,
                                           uint8 valid_x,
                                           uint8 valid_y,
                                           uint8 source)
{
    uint8 observation_valid_x = valid_x;
    uint8 observation_valid_y = valid_y;

#if !HORIZONTAL_VISUAL_VEL_ENABLE
    observation_valid_x = 0u;
    observation_valid_y = 0u;
#endif
    if (!IS_VALID(vx_right_cms)) observation_valid_x = 0u;
    if (!IS_VALID(vy_back_cms)) observation_valid_y = 0u;
    if (!IS_VALID(observation_dt_s)) {
        observation_valid_x = 0u;
        observation_valid_y = 0u;
    }
    if (observation_valid_x || observation_valid_y) {
        observation_dt_s = constrain_float(observation_dt_s, 0.005f, 0.12f);
    } else {
        observation_dt_s = 0.0f;
    }

    taskENTER_CRITICAL();
    pending_visual_observation.vx_cms = vx_right_cms;
    pending_visual_observation.vy_cms = vy_back_cms;
    pending_visual_observation.dt_s = observation_dt_s;
    pending_visual_observation.valid_x = observation_valid_x;
    pending_visual_observation.valid_y = observation_valid_y;
    pending_visual_observation.source = source;
    pending_visual_observation_ready = 1u;
    taskEXIT_CRITICAL();
}

void imu_horizontal_request_position_reset(void)
{
    taskENTER_CRITICAL();
    horizontal_position_reset_requested = 1;
    taskEXIT_CRITICAL();
}

void imu_horizontal_request_full_reset(void)
{
    taskENTER_CRITICAL();
    horizontal_full_reset_requested = 1;
    taskEXIT_CRITICAL();
}

void imu_horizontal_get_state(imu_horizontal_state_t *state)
{
    if(state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *state = horizontal_published_state;
    taskEXIT_CRITICAL();
}

uint8 imu_horizontal_impact_guard_active(void)
{
    uint8 active;

    taskENTER_CRITICAL();
    active = (external_accel_guard_stage !=
              HORIZONTAL_IMPACT_GUARD_NORMAL) ? 1u : 0u;
    taskEXIT_CRITICAL();
    return active;
}

/**
 * @brief 1ms horizontal prediction and continuous optical-flow correction.
 */
static void integrate_horizontal_velocity(float dt)
{
    horizontal_flow_observation_t observation = {0};
    horizontal_visual_observation_t visual_observation = {0};
    uint8 observation_ready = 0;
    uint8 visual_observation_ready = 0;
    uint8 position_reset_requested = 0;
    uint8 full_reset_requested = 0;
    float acc_x, acc_y;
    float err_x, err_y;
    float raw_err_x, raw_err_y;
    float correction_tau_x, correction_tau_y;
    float correction_alpha_x, correction_alpha_y;
    float flow_correction_x_cms;
    float flow_correction_y_cms;
    float kf_measurement_noise_x;
    float kf_measurement_noise_y;
    float accel_trust;
    float kf_accel_x_cms2;
    float kf_accel_y_cms2;
    uint8 kf_bias_update_enable;
    uint8 flow_update_mask;
    attitude_auto_trim_debug_t trim_debug;
    float bias_error_alpha;
    float decay_alpha;
    float accel_trust_for_bias;
    float visual_raw_err_x;
    float visual_raw_err_y;
    float visual_err_x;
    float visual_err_y;
    float visual_tau_x;
    float visual_tau_y;
    float visual_alpha_x;
    float visual_alpha_y;
    uint8 flow_fresh_x;
    uint8 flow_fresh_y;
    uint8 visual_fresh_x;
    uint8 visual_fresh_y;

    if(!IS_VALID(dt) || (dt < 0.0005f) || (dt > 0.02f)) {
        dt = 0.001f;
    }
    taskENTER_CRITICAL();
    if(pending_flow_observation_ready) {
        observation.vx_cms = pending_flow_observation.vx_cms;
        observation.vy_cms = pending_flow_observation.vy_cms;
        observation.dt_s = pending_flow_observation.dt_s;
        observation.valid_x = pending_flow_observation.valid_x;
        observation.valid_y = pending_flow_observation.valid_y;
        observation.trust_level_x = pending_flow_observation.trust_level_x;
        observation.trust_level_y = pending_flow_observation.trust_level_y;
        observation.consistency_error_x_cms =
            pending_flow_observation.consistency_error_x_cms;
        observation.consistency_error_y_cms =
            pending_flow_observation.consistency_error_y_cms;
        pending_flow_observation_ready = 0;
        observation_ready = 1;
    }
    if(pending_visual_observation_ready) {
        visual_observation.vx_cms = pending_visual_observation.vx_cms;
        visual_observation.vy_cms = pending_visual_observation.vy_cms;
        visual_observation.dt_s = pending_visual_observation.dt_s;
        visual_observation.valid_x = pending_visual_observation.valid_x;
        visual_observation.valid_y = pending_visual_observation.valid_y;
        visual_observation.source = pending_visual_observation.source;
        pending_visual_observation_ready = 0u;
        visual_observation_ready = 1u;
    }
    position_reset_requested = horizontal_position_reset_requested;
    full_reset_requested = horizontal_full_reset_requested;
    horizontal_position_reset_requested = 0;
    horizontal_full_reset_requested = 0;
    taskEXIT_CRITICAL();

    if(full_reset_requested) {
        reset_horizontal_velocity_estimator_internal();
    } else if(position_reset_requested) {
        est_pos_x_cm = 0.0f;
        est_pos_y_cm = 0.0f;
        horizontal_position_reset_count++;
    }

    if((height < HORIZONTAL_GROUND_RESET_HEIGHT_MM)
    && (fabsf(pitch_filtered) < 5.0f)
    && (fabsf(roll_filtered) < 5.0f)) {
        if(!horizontal_ground_reset_latched && !full_reset_requested) {
            reset_horizontal_velocity_estimator_internal();
        }
        horizontal_ground_reset_latched = 1;
        reset_external_accel_protection();
        publish_horizontal_state(dt);
        return;
    }
    horizontal_ground_reset_latched = 0;

    horizontal_flow_update_age_x_s += dt;
    horizontal_flow_update_age_y_s += dt;

    if(observation_ready) {
        horizontal_flow_trust_level_x = observation.trust_level_x;
        horizontal_flow_trust_level_y = observation.trust_level_y;
        horizontal_flow_consistency_error_x_cms =
            observation.consistency_error_x_cms;
        horizontal_flow_consistency_error_y_cms =
            observation.consistency_error_y_cms;
        horizontal_correction_remaining_x_s = 0.0f;
        horizontal_correction_remaining_y_s = 0.0f;
        if(observation.valid_x) {
            if(!horizontal_have_flow_x
            || (horizontal_flow_age_x_s > HORIZONTAL_FLOW_FRESH_S)) {
                horizontal_flow_recovery_frames_x = 1;
            } else if(horizontal_flow_recovery_frames_x < 255u) {
                horizontal_flow_recovery_frames_x++;
            }
            horizontal_flow_vx_cms = observation.vx_cms;
            horizontal_flow_age_x_s = 0.0f;
            horizontal_correction_remaining_x_s = observation.dt_s;
            horizontal_have_flow_x = 1;
        }
        if(observation.valid_y) {
            if(!horizontal_have_flow_y
            || (horizontal_flow_age_y_s > HORIZONTAL_FLOW_FRESH_S)) {
                horizontal_flow_recovery_frames_y = 1;
            } else if(horizontal_flow_recovery_frames_y < 255u) {
                horizontal_flow_recovery_frames_y++;
            }
            horizontal_flow_vy_cms = observation.vy_cms;
            horizontal_flow_age_y_s = 0.0f;
            horizontal_correction_remaining_y_s = observation.dt_s;
            horizontal_have_flow_y = 1;
        }
    }

    if(visual_observation_ready) {
        horizontal_visual_source = visual_observation.source;
        if(visual_observation.valid_x) {
            horizontal_visual_vx_cms = visual_observation.vx_cms;
            horizontal_visual_age_x_s = 0.0f;
            horizontal_visual_correction_remaining_x_s =
                visual_observation.dt_s;
            horizontal_have_visual_x = 1u;
        } else {
            horizontal_visual_age_x_s = 1000.0f;
            horizontal_visual_correction_remaining_x_s = 0.0f;
            horizontal_last_visual_innovation_x_cms = 0.0f;
            horizontal_have_visual_x = 0u;
        }
        if(visual_observation.valid_y) {
            horizontal_visual_vy_cms = visual_observation.vy_cms;
            horizontal_visual_age_y_s = 0.0f;
            horizontal_visual_correction_remaining_y_s =
                visual_observation.dt_s;
            horizontal_have_visual_y = 1u;
        } else {
            horizontal_visual_age_y_s = 1000.0f;
            horizontal_visual_correction_remaining_y_s = 0.0f;
            horizontal_last_visual_innovation_y_cms = 0.0f;
            horizontal_have_visual_y = 0u;
        }
    }

    horizontal_flow_age_x_s += dt;
    horizontal_flow_age_y_s += dt;
    horizontal_visual_age_x_s += dt;
    horizontal_visual_age_y_s += dt;
    if(horizontal_flow_age_x_s > HORIZONTAL_FLOW_FRESH_S) {
        horizontal_flow_recovery_frames_x = 0;
        horizontal_last_innovation_x_cms = 0.0f;
        horizontal_last_correction_tau_x_s = HORIZONTAL_CORRECTION_TAU_SLOW_S;
    }
    if(horizontal_flow_age_y_s > HORIZONTAL_FLOW_FRESH_S) {
        horizontal_flow_recovery_frames_y = 0;
        horizontal_last_innovation_y_cms = 0.0f;
        horizontal_last_correction_tau_y_s = HORIZONTAL_CORRECTION_TAU_SLOW_S;
    }
    if(horizontal_visual_age_x_s > HORIZONTAL_VISUAL_FRESH_S) {
        horizontal_last_visual_innovation_x_cms = 0.0f;
        horizontal_visual_correction_remaining_x_s = 0.0f;
    }
    if(horizontal_visual_age_y_s > HORIZONTAL_VISUAL_FRESH_S) {
        horizontal_last_visual_innovation_y_cms = 0.0f;
        horizontal_visual_correction_remaining_y_s = 0.0f;
    }
    get_horizontal_acceleration(&acc_x, &acc_y);
    horizontal_last_nav_acc_x_cms2 = acc_x;
    horizontal_last_nav_acc_y_cms2 = acc_y;
    update_external_accel_protection(acc_x, acc_y, dt);
    accel_trust_for_bias =
        (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) ?
        external_accel_applied_trust : 0.0f;

    /* 动态零点学习期间，用去重力后的水平加速度估计KF的初始加速度偏置。 */
    attitude_auto_trim_get_debug(&trim_debug);
    if ((external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) &&
        (trim_debug.state == ATTITUDE_TRIM_LEARN) &&
        attitude_auto_trim_zero_speed_hold_active() &&
        observation_ready && observation.valid_x && observation.valid_y &&
        isfinite(acc_x) && isfinite(acc_y) &&
        (fabsf(acc_x) < 150.0f) && (fabsf(acc_y) < 150.0f)) {
        horizontal_kf_trim_acc_sum_x_cms2 += acc_x;
        horizontal_kf_trim_acc_sum_y_cms2 += acc_y;
        if (horizontal_kf_trim_acc_count < 0xFFFFFFFFu) {
            horizontal_kf_trim_acc_count++;
        }
    }
    if (!horizontal_kf_trim_seeded &&
        (trim_debug.state == ATTITUDE_TRIM_TRANSFER ||
         trim_debug.state == ATTITUDE_TRIM_FROZEN) &&
        (horizontal_kf_trim_acc_count >= 100u)) {
        horizontal_velocity_kf_set_bias_seed(
            &horizontal_velocity_kf,
            horizontal_kf_trim_acc_sum_x_cms2 /
                (float)horizontal_kf_trim_acc_count,
            horizontal_kf_trim_acc_sum_y_cms2 /
                (float)horizontal_kf_trim_acc_count);
        horizontal_kf_trim_seeded = 1u;
    }

    accel_trust = external_accel_applied_trust;
    if (!isfinite(accel_trust)) accel_trust = 1.0f;
    if (accel_trust < 0.05f) accel_trust = 0.05f;
    if (accel_trust > 1.0f) accel_trust = 1.0f;

    kf_accel_x_cms2 = external_accel_expected_x_cms2
                    + external_accel_baseline_x_cms2
                    + accel_trust * external_accel_residual_x_cms2;
    kf_accel_y_cms2 = external_accel_expected_y_cms2
                    + external_accel_baseline_y_cms2
                    + accel_trust * external_accel_residual_y_cms2;
    if (!IS_VALID(kf_accel_x_cms2)) kf_accel_x_cms2 = acc_x;
    if (!IS_VALID(kf_accel_y_cms2)) kf_accel_y_cms2 = acc_y;
    kf_bias_update_enable = (uint8)(
        (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) &&
        (accel_trust >= HORIZONTAL_KF_BIAS_FREEZE_TRUST));
    if (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) {
        horizontal_kf_accel_used_x_cms2 = kf_accel_x_cms2;
        horizontal_kf_accel_used_y_cms2 = kf_accel_y_cms2;
        horizontal_velocity_kf_predict(&horizontal_velocity_kf,
                                       kf_accel_x_cms2,
                                       kf_accel_y_cms2,
                                       dt,
                                       accel_trust);
    } else {
        // ACTIVE/RECOVERY只允许光流修正速度，不执行任何加速度预测。
        horizontal_kf_accel_used_x_cms2 = 0.0f;
        horizontal_kf_accel_used_y_cms2 = 0.0f;
    }
    /* 光流是当前速度环的主观测；按可信等级降低观测噪声，提高KF对光流的跟随。 */
    kf_measurement_noise_x = (observation.trust_level_x == 0u) ?
                              225.0f : ((observation.trust_level_x == 1u) ?
                              64.0f : 16.0f);
    kf_measurement_noise_y = (observation.trust_level_y == 0u) ?
                              225.0f : ((observation.trust_level_y == 1u) ?
                              64.0f : 16.0f);
    if (observation_ready) {
        flow_update_mask = horizontal_velocity_kf_update(
            &horizontal_velocity_kf,
            observation.vx_cms,
            observation.vy_cms,
            observation.valid_x,
            observation.valid_y,
            kf_measurement_noise_x,
            kf_measurement_noise_y,
            kf_bias_update_enable);
        if (flow_update_mask & 0x01u) {
            horizontal_flow_update_count_x++;
            horizontal_flow_update_age_x_s = 0.0f;
        }
        if (flow_update_mask & 0x02u) {
            horizontal_flow_update_count_y++;
            horizontal_flow_update_age_y_s = 0.0f;
        }
    }
    if (HORIZONTAL_KF_VISUAL_FUSION_ENABLE && visual_observation_ready) {
        /* 视觉差分速度只保留为遥测诊断，默认不再进入KF观测更新。 */
        horizontal_velocity_kf_update(
            &horizontal_velocity_kf,
            visual_observation.vx_cms,
            visual_observation.vy_cms,
            (uint8)(visual_observation.valid_x &&
                    (!observation_ready || !observation.valid_x)),
            (uint8)(visual_observation.valid_y &&
                    (!observation_ready || !observation.valid_y)),
            HORIZONTAL_KF_VISUAL_MEAS_NOISE_CMS2,
            HORIZONTAL_KF_VISUAL_MEAS_NOISE_CMS2,
            kf_bias_update_enable);
    }
    external_accel_guard_try_finish(observation_ready,
                                    observation.valid_x,
                                    observation.valid_y);

    if (external_accel_guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) {
        est_vx_cms += (acc_x - est_a_bias_x_cms2) * dt;
        est_vy_cms += (acc_y - est_a_bias_y_cms2) * dt;
    }
    horizontal_last_imu_predicted_vx_cms = est_vx_cms;
    horizontal_last_imu_predicted_vy_cms = est_vy_cms;
    flow_correction_x_cms = 0.0f;
    flow_correction_y_cms = 0.0f;
    horizontal_last_flow_correction_x_cms = 0.0f;
    horizontal_last_flow_correction_y_cms = 0.0f;

    bias_error_alpha = dt / (HORIZONTAL_BIAS_ERROR_TAU_S + dt);

    if(horizontal_have_flow_x
    && (horizontal_flow_age_x_s <= HORIZONTAL_FLOW_FRESH_S)
    && (horizontal_correction_remaining_x_s > 0.0f)) {
        raw_err_x = horizontal_flow_vx_cms - est_vx_cms;
        horizontal_last_innovation_x_cms = raw_err_x;
        err_x = constrain_float(raw_err_x,
                                -HORIZONTAL_INNOVATION_LIMIT_CMS,
                                 HORIZONTAL_INNOVATION_LIMIT_CMS);

        correction_tau_x = horizontal_get_correction_tau(raw_err_x);
        if(horizontal_flow_recovery_frames_x <= HORIZONTAL_RECOVERY_FRAME_COUNT) {
            if(correction_tau_x < HORIZONTAL_RECOVERY_TAU_S) {
                correction_tau_x = HORIZONTAL_RECOVERY_TAU_S;
            }
        }
        horizontal_last_correction_tau_x_s = correction_tau_x;
        correction_alpha_x = dt / (correction_tau_x + dt);
        flow_correction_x_cms = correction_alpha_x * err_x;
        est_vx_cms += flow_correction_x_cms;
        horizontal_last_flow_correction_x_cms = flow_correction_x_cms;

        if(fabsf(raw_err_x) <= HORIZONTAL_BIAS_INNOVATION_LIMIT_CMS) {
            // 冲击期间同时冻结误差学习和零偏积分，避免拉线加速度被误学成传感器偏置。
            horizontal_bias_error_x_cms += bias_error_alpha
                                         * accel_trust_for_bias
                                         * (err_x - horizontal_bias_error_x_cms);
            est_a_bias_x_cms2 -= HORIZONTAL_BIAS_GAIN_PER_S
                               * accel_trust_for_bias
                               * horizontal_bias_error_x_cms * dt;
        } else {
            horizontal_bias_error_x_cms += bias_error_alpha * (0.0f - horizontal_bias_error_x_cms);
        }

        horizontal_correction_remaining_x_s -= dt;
        if(horizontal_correction_remaining_x_s < 0.0f) {
            horizontal_correction_remaining_x_s = 0.0f;
        }
    } else {
        horizontal_bias_error_x_cms += bias_error_alpha * (0.0f - horizontal_bias_error_x_cms);
    }

    if(horizontal_have_flow_y
    && (horizontal_flow_age_y_s <= HORIZONTAL_FLOW_FRESH_S)
    && (horizontal_correction_remaining_y_s > 0.0f)) {
        raw_err_y = horizontal_flow_vy_cms - est_vy_cms;
        horizontal_last_innovation_y_cms = raw_err_y;
        err_y = constrain_float(raw_err_y,
                                -HORIZONTAL_INNOVATION_LIMIT_CMS,
                                 HORIZONTAL_INNOVATION_LIMIT_CMS);

        correction_tau_y = horizontal_get_correction_tau(raw_err_y);
        if(horizontal_flow_recovery_frames_y <= HORIZONTAL_RECOVERY_FRAME_COUNT) {
            if(correction_tau_y < HORIZONTAL_RECOVERY_TAU_S) {
                correction_tau_y = HORIZONTAL_RECOVERY_TAU_S;
            }
        }
        horizontal_last_correction_tau_y_s = correction_tau_y;
        correction_alpha_y = dt / (correction_tau_y + dt);
        flow_correction_y_cms = correction_alpha_y * err_y;
        est_vy_cms += flow_correction_y_cms;
        horizontal_last_flow_correction_y_cms = flow_correction_y_cms;

        if(fabsf(raw_err_y) <= HORIZONTAL_BIAS_INNOVATION_LIMIT_CMS) {
            horizontal_bias_error_y_cms += bias_error_alpha
                                         * accel_trust_for_bias
                                         * (err_y - horizontal_bias_error_y_cms);
            est_a_bias_y_cms2 -= HORIZONTAL_BIAS_GAIN_PER_S
                               * accel_trust_for_bias
                               * horizontal_bias_error_y_cms * dt;
        } else {
            horizontal_bias_error_y_cms += bias_error_alpha * (0.0f - horizontal_bias_error_y_cms);
        }

        horizontal_correction_remaining_y_s -= dt;
        if(horizontal_correction_remaining_y_s < 0.0f) {
            horizontal_correction_remaining_y_s = 0.0f;
        }
    } else {
        horizontal_bias_error_y_cms += bias_error_alpha * (0.0f - horizontal_bias_error_y_cms);
    }

    flow_fresh_x = (uint8)(horizontal_have_flow_x &&
                           (horizontal_flow_age_x_s <= HORIZONTAL_FLOW_FRESH_S));
    flow_fresh_y = (uint8)(horizontal_have_flow_y &&
                           (horizontal_flow_age_y_s <= HORIZONTAL_FLOW_FRESH_S));
    visual_fresh_x = (uint8)(HORIZONTAL_KF_VISUAL_FUSION_ENABLE &&
                             horizontal_have_visual_x &&
                             (horizontal_visual_age_x_s <= HORIZONTAL_VISUAL_FRESH_S));
    visual_fresh_y = (uint8)(HORIZONTAL_KF_VISUAL_FUSION_ENABLE &&
                             horizontal_have_visual_y &&
                             (horizontal_visual_age_y_s <= HORIZONTAL_VISUAL_FRESH_S));

    if(visual_fresh_x &&
       (horizontal_visual_correction_remaining_x_s > 0.0f)) {
        visual_raw_err_x = horizontal_visual_vx_cms - est_vx_cms;
        horizontal_last_visual_innovation_x_cms = visual_raw_err_x;
        visual_err_x = constrain_float(
            visual_raw_err_x,
            -HORIZONTAL_VISUAL_INNOVATION_LIMIT_CMS,
             HORIZONTAL_VISUAL_INNOVATION_LIMIT_CMS);
        visual_tau_x = flow_fresh_x ?
            HORIZONTAL_VISUAL_TAU_WITH_FLOW_S :
            HORIZONTAL_VISUAL_TAU_NO_FLOW_S;
        visual_alpha_x = dt / (visual_tau_x + dt);
        est_vx_cms += visual_alpha_x * visual_err_x;
        horizontal_visual_correction_remaining_x_s -= dt;
        if(horizontal_visual_correction_remaining_x_s < 0.0f) {
            horizontal_visual_correction_remaining_x_s = 0.0f;
        }
    }

    if(visual_fresh_y &&
       (horizontal_visual_correction_remaining_y_s > 0.0f)) {
        visual_raw_err_y = horizontal_visual_vy_cms - est_vy_cms;
        horizontal_last_visual_innovation_y_cms = visual_raw_err_y;
        visual_err_y = constrain_float(
            visual_raw_err_y,
            -HORIZONTAL_VISUAL_INNOVATION_LIMIT_CMS,
             HORIZONTAL_VISUAL_INNOVATION_LIMIT_CMS);
        visual_tau_y = flow_fresh_y ?
            HORIZONTAL_VISUAL_TAU_WITH_FLOW_S :
            HORIZONTAL_VISUAL_TAU_NO_FLOW_S;
        visual_alpha_y = dt / (visual_tau_y + dt);
        est_vy_cms += visual_alpha_y * visual_err_y;
        horizontal_visual_correction_remaining_y_s -= dt;
        if(horizontal_visual_correction_remaining_y_s < 0.0f) {
            horizontal_visual_correction_remaining_y_s = 0.0f;
        }
    }

    if(!flow_fresh_x && !visual_fresh_x &&
       horizontal_flow_age_x_s > HORIZONTAL_DECAY_DELAY_S &&
       horizontal_visual_age_x_s > HORIZONTAL_DECAY_DELAY_S) {
        decay_alpha = dt / (HORIZONTAL_DECAY_TAU_S + dt);
        est_vx_cms += decay_alpha * (0.0f - est_vx_cms);
    }
    if(!flow_fresh_y && !visual_fresh_y &&
       horizontal_flow_age_y_s > HORIZONTAL_DECAY_DELAY_S &&
       horizontal_visual_age_y_s > HORIZONTAL_DECAY_DELAY_S) {
        decay_alpha = dt / (HORIZONTAL_DECAY_TAU_S + dt);
        est_vy_cms += decay_alpha * (0.0f - est_vy_cms);
    }

    est_vx_cms = constrain_float(est_vx_cms,
                                 -HORIZONTAL_VELOCITY_LIMIT_CMS,
                                  HORIZONTAL_VELOCITY_LIMIT_CMS);
    est_vy_cms = constrain_float(est_vy_cms,
                                 -HORIZONTAL_VELOCITY_LIMIT_CMS,
                                  HORIZONTAL_VELOCITY_LIMIT_CMS);
    est_a_bias_x_cms2 = constrain_float(est_a_bias_x_cms2, -200.0f, 200.0f);
    est_a_bias_y_cms2 = constrain_float(est_a_bias_y_cms2, -200.0f, 200.0f);

#if HORIZONTAL_KF_CONTROL_ENABLE
    // KF接管速度环时，位置积分同步采用KF速度，避免位置环和速度环使用不同速度状态。
    est_pos_x_cm += horizontal_velocity_kf.x.velocity_cms * dt;
    est_pos_y_cm += horizontal_velocity_kf.y.velocity_cms * dt;
#else
    est_pos_x_cm += est_vx_cms * dt;
    est_pos_y_cm += est_vy_cms * dt;
#endif
    publish_horizontal_state(dt);
}
