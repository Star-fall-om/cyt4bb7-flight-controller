#ifndef __LC302_H_
#define __LC302_H_

#include "zf_common_headfile.h"
#include "zf_device_pmw3901.h"
#include "System/usr_system.h"

#define PMW3901_COUNTS_PER_RAD  (585.0f)

#define PMW3901_DT_S            (0.02f)    // 20ms 周期
#define DEG2RAD                 (0.01745329252f)

// LC302陀螺时间对齐模式：SHADOW只记录对比数据，ACTIVE才使用帧边界补偿。
#define LC302_GYRO_ALIGN_SHADOW  (0u)
#define LC302_GYRO_ALIGN_ACTIVE  (1u)
#ifndef LC302_GYRO_ALIGN_MODE
#define LC302_GYRO_ALIGN_MODE    LC302_GYRO_ALIGN_SHADOW
#endif

#if (LC302_GYRO_ALIGN_MODE != LC302_GYRO_ALIGN_SHADOW) && \
    (LC302_GYRO_ALIGN_MODE != LC302_GYRO_ALIGN_ACTIVE)
#error Invalid LC302_GYRO_ALIGN_MODE
#endif

// ==================================================================
//   速度估计结构体 (单位：cm/s)
// ==================================================================
typedef struct
{
    int16   raw_dx;         // 中值滤波后的原始 Count X
    int16   raw_dy;         // 中值滤波后的原始 Count Y

    float   comp_dx;        // 陀螺仪补偿后的纯平移 Count X
    float   comp_dy;        // 陀螺仪补偿后的纯平移 Count Y

    float   vx_cms;         // 最终 X 速度 (cm/s)
    float   vy_cms;         // 最终 Y 速度 (cm/s)
    float   dt_s;

    float   sum_x_cm;       // 累计物理位置 (cm)
    float   sum_y_cm;       // 累计物理位置 (cm)

    uint8   valid;          // 数据是否有效
    uint8   valid_x;
    uint8   valid_y;
    float   quality;        // 表面质量 (0~255)
} flow_velocity_t;

typedef struct
{
    float candidate_vx_cms;
    float candidate_vy_cms;
    float max_delta_cms;
    float max_delta_x_cms;
    float max_delta_y_cms;
    uint32 reject_count;
    uint32 reject_count_x;
    uint32 reject_count_y;
    uint32 rx_frame_count;
    uint32 processed_frame_count;
    uint32 no_frame_poll_count;
    uint32 frame_overrun_count;
    uint16 invalid_reason;        // bit0 raw,1 gyro,2 range,3 zero,4 discard,5 ground,6 height,7 dt,8/9 axis reject
    uint8 candidate_pending;       // bit0=X, bit1=Y
    uint8 transition_confirmed;    // bit0=X, bit1=Y
    uint8 last_update_had_new_frame;
    uint8 raw_valid;
    float filter_alpha_x;
    float filter_alpha_y;
    uint8 fast_filter_mask;         // bit0=X, bit1=Y
    uint32 frame_interval_us;        // 相邻完整LC302帧头间隔(us)
    uint32 integration_time_us;      // 本帧LC302积分时间(us)
    float legacy_delta_roll_rad;     // 原任务取帧窗口Roll积分(rad)
    float legacy_delta_pitch_rad;    // 原任务取帧窗口Pitch积分(rad)
    float aligned_delta_roll_rad;    // 相邻帧头窗口Roll积分(rad)
    float aligned_delta_pitch_rad;   // 相邻帧头窗口Pitch积分(rad)
    float legacy_comp_vx_cms;        // 原补偿得到的未滤波X速度(cm/s)
    float legacy_comp_vy_cms;        // 原补偿得到的未滤波Y速度(cm/s)
    float aligned_comp_vx_cms;       // 对齐补偿得到的未滤波X速度(cm/s)
    float aligned_comp_vy_cms;       // 对齐补偿得到的未滤波Y速度(cm/s)
    uint8 timing_aligned_valid;      // 1=帧间隔与integration_timespan匹配
    float candidate_delta_x_cms;
    float candidate_delta_y_cms;
    float predicted_delta_x_cms;
    float predicted_delta_y_cms;
    float consistency_error_x_cms;
    float consistency_error_y_cms;
    uint8 trust_level_x;              // 0=slow, 1=normal, 2=fast
    uint8 trust_level_y;              // 0=slow, 1=normal, 2=fast
    float final_delta_x_cms;
    float final_delta_y_cms;
} lc302_filter_debug_t;

// ==================================================================
//   LC302原始数据结构（V1.30协议）
// ==================================================================
typedef struct {
    int16_t  pixel_flow_x_integral;     // X轴光流积分值（需除以10000）
    int16_t  pixel_flow_y_integral;     // Y轴光流积分值（需除以10000）
    uint16_t integration_timespan;      // 积分时间（微秒）
    uint16_t ground_distance;           // 地面距离（毫米）
    uint8_t  valid;                     // 数据有效标志
    uint8_t  version;                   // 版本号
} lc302_data_t;

// ==================================================================
//   通用接口函数（PMW3901和LC302共用）
// ==================================================================
void FLOW_reset_position(void);
uint8 FLOW_init(void);
void FLOW_update_velocity(float height_mm);

extern flow_velocity_t  flow_vel;

// ==================================================================
//   LC302专用函数
// ==================================================================
#if USE_LC302_FLOW
uint8_t LC302_init(void);
void LC302_uart_callback(uint8_t data);
void LC302_accumulate_gyro(float roll_rate_dps, float pitch_rate_dps,
                           uint32_t now_us);
void LC302_reset_runtime(void);
uint8_t LC302_update_velocity(float height_mm);
void LC302_get_filter_debug(lc302_filter_debug_t *debug);
void LC302_get_snapshot(flow_velocity_t *flow, lc302_filter_debug_t *debug);
extern lc302_data_t lc302_data;
#endif

#endif
