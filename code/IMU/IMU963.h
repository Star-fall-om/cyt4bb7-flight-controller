#ifndef IMU963_H
#define IMU963_H

#include "zf_common_typedef.h"
#include "zf_device_type.h"
#include "zf_device_imu963ra.h"
#include "math.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

// ==================== 必需的外部变量 ====================
// 滤波后的姿态角 (单位：度)
extern float pitch_filtered;
extern float roll_filtered;
extern float yaw_filtered;

// 滤波后的角速度 (单位：度/秒) - 用于PID控制
extern float pitch_gyro_filtered;
extern float roll_gyro_filtered;
extern float yaw_gyro_filtered;

// Filtered accelerometer output (m/s2), axis order: X/Y/Z.
extern float accl_m_s2[3];

// Raw accelerometer output after unit conversion (m/s2), axis order: X/Y/Z.
extern float raw_accl_m_s2[3];

extern float vertical_velocity;  // 垂直速度，单位 cm/s
extern float vertical_velocity_smoothed;
extern float est_z_cm;
extern float est_v_cms;
extern float vertical_accel_cms2;
extern float vertical_accel_corrected_cms2;
extern float vertical_tof_error_cm;
extern float vertical_tof_z_correction_cm;
extern float vertical_tof_v_correction_cms;

// ==================== 滤波器与数据结构 ====================

// PT2滤波器 (二阶级联PT1) - 比巴特沃斯更稳，无过冲
typedef struct {
    float state1;   // 第一级状态
    float state2;   // 第二级状态
    float k;        // 滤波系数
} pt2_filter_t;

// PT1滤波器 (用于加速度计)
typedef struct {
    float state;
    float k;
} pt1_filter_t;

// Mahony算法结构体
typedef struct {
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
} mahony_filter_t;

typedef struct{
    float X_hat;        // 状态估计值，滤波后角速度
    float P;           // 均方误差
    float R;    // 量测噪声方差
    float Q;          // 系统噪声方差（自适应更新）
    float IV_buf[10];     // 新息缓存数组
    uint8_t iv_idx;        // 新息缓存索引
}adaptive_kalman;

// ==================== 接口函数 ====================
void imu_init(void);
void imu_process(void);

void adaptive_kalman_init(adaptive_kalman* filter,
                          float init_state,
                          float P0,
                          float R0,
                          float Q0);
float adaptive_kalman_filter(float Z, adaptive_kalman* kalman);

void correct_vertical_velocity(float measured_height_mm, float dt);

#define HORIZONTAL_CONTROL_SOURCE_KF_VALID          (0u)
#define HORIZONTAL_CONTROL_SOURCE_KF_SHORT_PREDICT (1u)
#define HORIZONTAL_CONTROL_SOURCE_FLOW_HOLD         (2u)
#define HORIZONTAL_CONTROL_SOURCE_FLOW_TIMEOUT     (3u)
#define HORIZONTAL_CONTROL_SOURCE_FLOW_IMPACT      (4u)

#define HORIZONTAL_IMPACT_GUARD_NORMAL             (0u)
#define HORIZONTAL_IMPACT_GUARD_ACTIVE             (1u)
#define HORIZONTAL_IMPACT_GUARD_RECOVERY           (2u)
#define HORIZONTAL_IMPACT_READY_PITCH              (0x01u)
#define HORIZONTAL_IMPACT_READY_ROLL               (0x02u)

// Horizontal estimator frame: +X right, +Y backward.
typedef struct {

    float vx_cms;
    float vy_cms;
    float control_vx_cms;
    float control_vy_cms;
    float control_last_flow_vx_cms;
    float control_last_flow_vy_cms;
    float pos_x_cm;
    float pos_y_cm;
    float acc_bias_x_cms2;
    float acc_bias_y_cms2;
    float flow_age_s;
    float flow_age_x_s;
    float flow_age_y_s;
    float flow_vx_cms;
    float flow_vy_cms;
    float visual_vx_cms;
    float visual_vy_cms;
    float visual_age_s;
    float innovation_x_cms;
    float innovation_y_cms;
    float visual_innovation_x_cms;
    float visual_innovation_y_cms;
    float nav_acc_x_cms2;
    float nav_acc_y_cms2;
    float expected_acc_x_cms2;
    float expected_acc_y_cms2;
    float imu_predicted_vx_cms;
    float imu_predicted_vy_cms;
    float flow_correction_x_cms;
    float flow_correction_y_cms;
    float external_accel_residual_x_cms2;
    float external_accel_residual_y_cms2;
    float external_accel_trust;
    /* 仅供M70内部控制使用，不进入双核共享内存和串口协议。 */
    uint8 external_accel_guard_stage;
    float external_accel_guard_recovery;
    uint8 external_accel_flow_recovery_frames;
    float correction_tau_x_s;
    float correction_tau_y_s;
    uint32 full_reset_count;
    uint8 flow_valid;
    uint8 flow_valid_x;
    uint8 flow_valid_y;
    uint8 visual_valid;
    uint8 visual_valid_x;
    uint8 visual_valid_y;
    uint8 velocity_valid;
    uint8 velocity_valid_x;
    uint8 velocity_valid_y;
    uint8 control_velocity_valid_x;
    uint8 control_velocity_valid_y;
    uint8 control_velocity_source_x;
    uint8 control_velocity_source_y;
    float control_velocity_confidence_x;
    float control_velocity_confidence_y;
    uint8 visual_source;
    uint8 flow_recovery_frames;
    uint8 flow_recovery_frames_x;
    uint8 flow_recovery_frames_y;
    uint8 external_accel_shock_active;
    uint8 flow_trust_level_x;
    uint8 flow_trust_level_y;
    float flow_consistency_error_x_cms;
    float flow_consistency_error_y_cms;
    float kf_vx_cms;
    float kf_vy_cms;
    float kf_acc_bias_x_cms2;
    float kf_acc_bias_y_cms2;
    float kf_innovation_x_cms;
    float kf_innovation_y_cms;
    float kf_velocity_gain_x;
    float kf_velocity_gain_y;
    float kf_bias_gain_x;
    float kf_bias_gain_y;
    uint32 kf_reset_count;
    uint32 kf_trim_acc_count;
    uint8 kf_trim_seeded;
    uint32 estimator_build_marker;
    uint32 estimator_config_flags;
    float kf_accel_used_x_cms2;
    float kf_accel_used_y_cms2;
    uint32 flow_update_count_x;
    uint32 flow_update_count_y;
    float flow_update_age_x_s;
    float flow_update_age_y_s;
    uint32 position_reset_count; // IMU任务真正执行位置清零的累计次数
} imu_horizontal_state_t;

void imu_horizontal_submit_flow(float flow_vx_cms, float flow_vy_cms,
                                float flow_dt_s, uint8 valid_x, uint8 valid_y,
                                uint8 trust_level_x, uint8 trust_level_y,
                                float consistency_error_x_cms,
                                float consistency_error_y_cms);
void imu_horizontal_submit_visual_velocity(float vx_right_cms,
                                           float vy_back_cms,
                                           float observation_dt_s,
                                           uint8 valid_x,
                                           uint8 valid_y,
                                           uint8 source);
void imu_horizontal_request_position_reset(void);
void imu_horizontal_request_full_reset(void);
void imu_horizontal_get_state(imu_horizontal_state_t *state);
uint8 imu_horizontal_impact_guard_active(void);
void imu_horizontal_set_impact_recovery_ready(uint8 axis_mask, uint8 ready);

#endif
