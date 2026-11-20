#ifndef __TOF_H_
#define __TOF_H_

#include "zf_common_headfile.h"
#include "System/usr_system.h"
#include "IMU/IMU963.h"

#define TOF_SENSOR_MODE_OLD_PAIR        (0)
#define TOF_SENSOR_MODE_DL1B_DL1B3      (1)

#ifndef TOF_SENSOR_MODE
#define TOF_SENSOR_MODE                 (TOF_SENSOR_MODE_DL1B_DL1B3)
#endif

// ==========================================
// 外部全局变量声明 (供其他文件调用)
// ==========================================
extern volatile float height;             // 最终融合输出的安全高度 (mm)
extern uint16_t tof_blind_timer;          // 盲飞定时器 (用于IMU预测衰减)
extern float tof_dl1b3_ground_mm;
extern uint8_t tof_dl1b3_valid_count;
extern uint16_t tof_dl1b3_edge_mask;

#define TOF_REJECT_NO_MEASUREMENT       (1u << 0)
#define TOF_REJECT_TILT_HOLD            (1u << 1)
#define TOF_REJECT_ANGLE_RATE_HOLD      (1u << 2)
#define TOF_REJECT_RATE                  (1u << 3)
#define TOF_REJECT_VARIANCE              (1u << 4)
#define TOF_REJECT_RISE                 (1u << 5)
#define TOF_REJECT_DROP                 (1u << 6)
#define TOF_REJECT_VALID_COUNT           (1u << 7)
#define TOF_REJECT_RECOVERY              (1u << 8)
#define TOF_REJECT_OUTPUT_STEP           (1u << 9)
#define TOF_REJECT_CABLE                 (1u << 10)

#define TOF_MODE_NORMAL                  0u
#define TOF_MODE_GEOMETRIC               1u
#define TOF_MODE_PREDICT                 2u

typedef struct {
    float dl1b_raw_mm;
    float dl1b3_raw_median_mm;
    float dl1b3_zone_cos_median_mm;
    float tilt_factor;
    float compensated_measurement_mm;
    float trusted_height_mm;
    float pitch_deg;
    float roll_deg;
    float pitch_rate_dps;
    float roll_rate_dps;
    uint8_t measure_available;
    uint8_t accepted;
    uint8_t hold_active;
    uint8_t recovery_active;
    uint16_t reject_reason;
    uint16_t relock_streak;
    uint16_t recovery_count;
    uint16_t blind_timer;
    float dl1b3_min_height_mm;
    float dl1b3_max_height_mm;
    float dl1b3_spread_mm;
    float min_vertical_factor;
    float max_vertical_factor;
    float innovation_mm;
    uint8_t mode;
    uint8_t using_tof3;
    uint8_t using_prediction;
    uint8_t cable_suspect;
} tof_debug_t;

extern volatile tof_debug_t tof_debug;

// ==========================================
// 接口函数
// ==========================================
/**
 * @brief TOF 模块初始化与基准高度预热
 */
void TOF_Init(void);

/**
 * @brief TOF 核心处理函数 (建议在 10ms 任务中调用)
 */
void TOF_Process(void);

#endif
