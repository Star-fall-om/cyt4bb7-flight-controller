// LC302光流模块驱动
// 用于智能车竞赛四旋翼无人机的光流定位
#include "LC302.h"
#include <math.h>
#include "IMU/IMU963.h"
#include "System/usr_system.h"
#include "zf_driver_spi.h"
#include "zf_driver_uart.h"
#include "FreeRTOS.h"
#include "task.h"

// 全局光流速度输出结构体
flow_velocity_t flow_vel = {0};

// 函数前置声明
static float MedianFilter3_Float(float new_val, float *history);
static float SlewRateLimiter(float new_val, float last_val, float max_delta);

/*
 * LC302安装方向矫正标志
 * 仅当原始轴向仍需软件顺时针90度矫正时设为1
 * 当前测量的原始轴向已匹配机体坐标系输出映射，设为0
 */
#define LC302_MOUNT_CW90        (0)

#if USE_LC302_FLOW
// ============================================================================
//   LC302 UART驱动实现
// ============================================================================
lc302_data_t lc302_data = {0};  // 光流数据缓存

// ===== UART接收状态机变量 =====
static uint8_t lc302_rx_buffer[16] = {0};     // 接收缓冲区
static uint8_t lc302_rx_state = 0;            // 状态机当前状态
static uint8_t lc302_rx_index = 0;            // 数据接收索引
static volatile uint8_t lc302_data_ready = 0; // 新帧就绪标志（中断设置，任务读取）
static uint8_t lc302_xor_check = 0;           // XOR校验累加值

// ===== LC302 UART6配置 =====
#define LC302_UART          UART_6
#define LC302_UART_BAUD     19200
#define LC302_TX_PIN        UART6_TX_P03_1
#define LC302_RX_PIN        UART6_RX_P03_0

// ===== 光流启停与防炸保护参数 =====
#define LC302_MIN_ACTIVE_HEIGHT_CM          (15.0f)   // 最小激活高度(cm)
#define LC302_MIN_ACTIVE_GROUND_MM         (150U)     // 最小激活地距(mm)，用于贴地检测
#define LC302_MAX_GYRO_FRAME_RAD           (0.35f)    // 单帧最大陀螺旋转量(rad)，超过视为异常
#define LC302_MAX_FLOW_FRAME_COUNTS        (3500)     // 单帧最大光流像素位移
#define LC302_DISCARD_FRAMES_AFTER_RESET   (2U)       // 复位后丢弃帧数
#define LC302_GYRO_BLEND_START_DPS          (30.0f)   // 自适应融合起始角速度(度/秒)
#define LC302_GYRO_BLEND_END_DPS            (70.0f)   // 自适应融合结束角速度(度/秒)
#define LC302_GYRO_MIN_FLOW_WEIGHT          (0.10f)   // 光流最小权重（角速度大时）
#define LC302_MAX_TRANSLATIONAL_ACCEL_CMS2  (2000.0f)
#define LC302_MAX_TRANSLATIONAL_BRAKE_CMS2  (3000.0f)
#define LC302_MAX_VALID_SPEED_CMS            (400.0f)  // 候选速度物理保护，单帧超范围直接拒绝
#define LC302_FILTER_ALPHA_SLOW             (0.10f)   // 单帧与IMU预测明显不一致时的低增益
#define LC302_FILTER_ALPHA_NORMAL           (0.85f)
#define LC302_FILTER_ALPHA_FAST             (0.90f)
#define LC302_FILTER_FAST_MIN_ACCEL_CMS2    (20.0f)
#define LC302_FILTER_FAST_MIN_DELTA_CMS     (2.0f)
#define LC302_FILTER_FAST_VALID_FRAMES      (2U)
#define LC302_FILTER_FAST_MAX_ERROR_CMS     (8.0f)    // 快速通道允许的最小变化量误差(cm/s)
#define LC302_FILTER_FAST_ERROR_RATIO       (0.35f)   // 预测变化较大时按预测量放宽
#define LC302_FILTER_SLOW_MIN_ERROR_CMS     (20.0f)   // 超过该误差进入低增益(cm/s)
#define LC302_TIMING_MATCH_TOLERANCE_US      (3000U)
#define LC302_GYRO_ACCUM_MAX_DT_US           (20000U)
#define LC302_INVALID_RAW_FLAG               (0x0001u)
#define LC302_INVALID_GYRO                   (0x0002u)
#define LC302_INVALID_FLOW_RANGE             (0x0004u)
#define LC302_INVALID_ZERO_FLOW              (0x0008u)
#define LC302_INVALID_DISCARD                (0x0010u)
#define LC302_INVALID_GROUND                 (0x0020u)
#define LC302_INVALID_HEIGHT                 (0x0040u)
#define LC302_INVALID_DT                     (0x0080u)
#define LC302_INVALID_CANDIDATE_X            (0x0100u)
#define LC302_INVALID_CANDIDATE_Y            (0x0200u)

// ===== LC302运行状态变量 =====
// 这些变量必须能被初始化/落地/解锁复位路径统一清空，不能藏在函数局部static里
static float lc302_vx_med_hist[3] = {0.0f, 0.0f, 0.0f};  // X轴速度中值滤波历史
static float lc302_vy_med_hist[3] = {0.0f, 0.0f, 0.0f};  // Y轴速度中值滤波历史
static float lc302_last_valid_vx = 0.0f;                 // 上次有效X速度(cm/s)
static float lc302_last_valid_vy = 0.0f;                 // 上次有效Y速度(cm/s)
static uint8_t lc302_loss_count = 0;                     // 连续丢帧计数
static float lc302_delayed_gyro_x = 0.0f;                // 延迟后的陀螺仪Roll累积(rad)
static float lc302_delayed_gyro_y = 0.0f;                // 延迟后的陀螺仪Pitch累积(rad)
static uint8_t lc302_discard_frames = 0;                 // 剩余需要丢弃的帧数
static uint8_t lc302_full_reset_latched = 0;             // 完全复位锁存标志
typedef struct
{
    float pending_cms;
    float confirmed_cms;
    float last_candidate_cms;
    float last_max_delta_cms;
    uint32 reject_count;
    uint8 candidate_pending;
    uint8 large_transition_confirmed;
    uint8 history_stale;
    uint8 valid_streak;
    uint8 fast_filter_active;
    uint8 trust_level;
    float filter_alpha;
    float velocity_delta_cms;          // 候选速度相对上一最终值的变化
    float predicted_delta_cms;         // IMU导航加速度预测的速度变化
    float consistency_error_cms;       // 候选变化与IMU预测变化的差值
    float final_delta_cms;             // 本次最终滤波输出的变化
} lc302_axis_filter_state_t;

static lc302_axis_filter_state_t lc302_axis_x = {0};
static lc302_axis_filter_state_t lc302_axis_y = {0};
static uint32_t lc302_candidate_reject_count = 0;
static volatile uint32_t lc302_rx_frame_count = 0;
static uint32_t lc302_processed_frame_count = 0;
static uint32_t lc302_no_frame_poll_count = 0;
static volatile uint32_t lc302_frame_overrun_count = 0;
static uint16 lc302_last_invalid_reason = 0;
static uint8_t lc302_last_update_had_new_frame = 0;
static uint8_t lc302_last_raw_valid = 0;

typedef struct
{
    uint32_t head_time_us;
    float gyro_total_roll_rad;
    float gyro_total_pitch_rad;
} lc302_frame_timing_t;

// IMU任务持续累计总角度；UART帧头只快照，不在ISR内做浮点运算。
static volatile float lc302_gyro_total_roll_rad = 0.0f;
static volatile float lc302_gyro_total_pitch_rad = 0.0f;
static uint32_t lc302_gyro_last_update_us = 0u;
static uint8_t lc302_gyro_total_inited = 0u;
static lc302_frame_timing_t lc302_rx_head_timing = {0};
static volatile lc302_frame_timing_t lc302_ready_timing = {0};
static lc302_frame_timing_t lc302_previous_timing = {0};
static uint8_t lc302_previous_timing_valid = 0u;

static uint32_t lc302_last_frame_interval_us = 0u;
static uint32_t lc302_last_integration_time_us = 0u;
static float lc302_last_legacy_delta_roll_rad = 0.0f;
static float lc302_last_legacy_delta_pitch_rad = 0.0f;
static float lc302_last_aligned_delta_roll_rad = 0.0f;
static float lc302_last_aligned_delta_pitch_rad = 0.0f;
static float lc302_last_legacy_comp_vx_cms = 0.0f;
static float lc302_last_legacy_comp_vy_cms = 0.0f;
static float lc302_last_aligned_comp_vx_cms = 0.0f;
static float lc302_last_aligned_comp_vy_cms = 0.0f;
static uint8_t lc302_last_timing_aligned_valid = 0u;

static uint32_t LC302_abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

void LC302_accumulate_gyro(float roll_rate_dps, float pitch_rate_dps,
                           uint32_t now_us)
{
    uint32_t dt_us;
    float dt_s;

    taskENTER_CRITICAL();
    if(!lc302_gyro_total_inited) {
        lc302_gyro_last_update_us = now_us;
        lc302_gyro_total_inited = 1u;
        taskEXIT_CRITICAL();
        return;
    }

    dt_us = now_us - lc302_gyro_last_update_us;
    lc302_gyro_last_update_us = now_us;
    if((dt_us > 0u) && (dt_us <= LC302_GYRO_ACCUM_MAX_DT_US)) {
        dt_s = (float)dt_us * 0.000001f;
        lc302_gyro_total_roll_rad += roll_rate_dps * DEG2RAD * dt_s;
        lc302_gyro_total_pitch_rad += pitch_rate_dps * DEG2RAD * dt_s;
    }
    taskEXIT_CRITICAL();
}

static void LC302_fill_median_history(float *history, float value)
{
    history[0] = value;
    history[1] = value;
    history[2] = value;
}

static void LC302_mark_axis_history_stale(lc302_axis_filter_state_t *state)
{
    state->candidate_pending = 0;
    state->large_transition_confirmed = 0;
    state->history_stale = 1;
    state->valid_streak = 0;
    state->fast_filter_active = 0;
    state->trust_level = 0;
    state->filter_alpha = LC302_FILTER_ALPHA_NORMAL;
    state->velocity_delta_cms = 0.0f;
    state->predicted_delta_cms = 0.0f;
    state->consistency_error_cms = 0.0f;
    state->final_delta_cms = 0.0f;
}

static void LC302_reset_axis_filter_state(lc302_axis_filter_state_t *state)
{
    state->pending_cms = 0.0f;
    state->confirmed_cms = 0.0f;
    state->last_candidate_cms = 0.0f;
    state->last_max_delta_cms = 0.0f;
    state->reject_count = 0;
    state->candidate_pending = 0;
    state->large_transition_confirmed = 0;
    state->history_stale = 1;
    state->valid_streak = 0;
    state->fast_filter_active = 0;
    state->trust_level = 0;
    state->filter_alpha = LC302_FILTER_ALPHA_NORMAL;
    state->velocity_delta_cms = 0.0f;
    state->predicted_delta_cms = 0.0f;
    state->consistency_error_cms = 0.0f;
    state->final_delta_cms = 0.0f;
}

static float LC302_select_axis_filter_alpha(float candidate_cms,
                                             float last_valid_cms,
                                             float update_dt,
                                             float nav_acc_cms2,
                                             float acc_bias_cms2,
                                             lc302_axis_filter_state_t *state)
{
    float unbiased_acc_cms2 = nav_acc_cms2 - acc_bias_cms2;
    float velocity_delta_cms = candidate_cms - last_valid_cms;
    float predicted_delta_cms = unbiased_acc_cms2 * update_dt;
    float consistency_error_cms = fabsf(velocity_delta_cms - predicted_delta_cms);
    float fast_error_limit_cms = LC302_FILTER_FAST_MAX_ERROR_CMS;
    uint8 direction_agrees = 0;
    uint8 transition_ready;

    if((fabsf(predicted_delta_cms) * LC302_FILTER_FAST_ERROR_RATIO)
       > fast_error_limit_cms) {
        fast_error_limit_cms = fabsf(predicted_delta_cms)
                             * LC302_FILTER_FAST_ERROR_RATIO;
    }

    state->velocity_delta_cms = velocity_delta_cms;
    state->predicted_delta_cms = predicted_delta_cms;
    state->consistency_error_cms = consistency_error_cms;

    if((fabsf(unbiased_acc_cms2) >= LC302_FILTER_FAST_MIN_ACCEL_CMS2)
    && (fabsf(velocity_delta_cms) >= LC302_FILTER_FAST_MIN_DELTA_CMS)
    && ((velocity_delta_cms * predicted_delta_cms) > 0.0f)) {
        direction_agrees = 1;
    }

    transition_ready = (state->valid_streak >= LC302_FILTER_FAST_VALID_FRAMES)
                    || state->large_transition_confirmed;
    if(direction_agrees && transition_ready
    && (consistency_error_cms <= fast_error_limit_cms)) {
        state->fast_filter_active = 1;
        state->trust_level = 2;
        state->filter_alpha = LC302_FILTER_ALPHA_FAST;
    } else if(consistency_error_cms >= LC302_FILTER_SLOW_MIN_ERROR_CMS) {
        state->fast_filter_active = 0;
        state->trust_level = 0;
        state->filter_alpha = LC302_FILTER_ALPHA_SLOW;
    } else {
        state->fast_filter_active = 0;
        state->trust_level = 1;
        state->filter_alpha = LC302_FILTER_ALPHA_NORMAL;
    }

    return state->filter_alpha;
}

static uint8_t LC302_accept_axis_candidate(float candidate_cms,
                                           float last_valid_cms,
                                           float max_delta_cms,
                                           lc302_axis_filter_state_t *state)
{
    (void)last_valid_cms;
    state->last_candidate_cms = candidate_cms;
    state->last_max_delta_cms = max_delta_cms;
    if((candidate_cms != candidate_cms)
    || (fabsf(candidate_cms) > LC302_MAX_VALID_SPEED_CMS)) {
        state->candidate_pending = 0;
        state->large_transition_confirmed = 0;
        state->history_stale = 1;
        state->reject_count++;
        return 0;
    }
    // 直接接收当前候选值，避免真实快速平移被二帧确认额外延迟。
    // 后续仍保留加速度斜率限制、中值滤波和低通滤波，抑制单帧尖峰。
    state->candidate_pending = 0;
    state->large_transition_confirmed = 0;
    return 1;
}

void LC302_get_snapshot(flow_velocity_t *flow, lc302_filter_debug_t *debug)
{
    if((flow == 0) && (debug == 0)) {
        return;
    }

    taskENTER_CRITICAL();
    if(flow != 0) {
        *flow = flow_vel;
    }
    if(debug != 0) {
        debug->candidate_vx_cms = lc302_axis_x.last_candidate_cms;
        debug->candidate_vy_cms = lc302_axis_y.last_candidate_cms;
        debug->max_delta_x_cms = lc302_axis_x.last_max_delta_cms;
        debug->max_delta_y_cms = lc302_axis_y.last_max_delta_cms;
        debug->max_delta_cms = (debug->max_delta_x_cms > debug->max_delta_y_cms)
                             ? debug->max_delta_x_cms : debug->max_delta_y_cms;
        debug->reject_count = lc302_candidate_reject_count;
        debug->reject_count_x = lc302_axis_x.reject_count;
        debug->reject_count_y = lc302_axis_y.reject_count;
        debug->rx_frame_count = lc302_rx_frame_count;
        debug->processed_frame_count = lc302_processed_frame_count;
        debug->no_frame_poll_count = lc302_no_frame_poll_count;
        debug->frame_overrun_count = lc302_frame_overrun_count;
        debug->invalid_reason = lc302_last_invalid_reason;
        debug->candidate_pending = (lc302_axis_x.candidate_pending ? 1u : 0u)
                                 | (lc302_axis_y.candidate_pending ? 2u : 0u);
        debug->transition_confirmed = (lc302_axis_x.large_transition_confirmed ? 1u : 0u)
                                    | (lc302_axis_y.large_transition_confirmed ? 2u : 0u);
        debug->last_update_had_new_frame = lc302_last_update_had_new_frame;
        debug->raw_valid = lc302_last_raw_valid;
        debug->filter_alpha_x = lc302_axis_x.filter_alpha;
        debug->filter_alpha_y = lc302_axis_y.filter_alpha;
        debug->fast_filter_mask = (lc302_axis_x.fast_filter_active ? 1u : 0u)
                                | (lc302_axis_y.fast_filter_active ? 2u : 0u);
        debug->frame_interval_us = lc302_last_frame_interval_us;
        debug->integration_time_us = lc302_last_integration_time_us;
        debug->legacy_delta_roll_rad = lc302_last_legacy_delta_roll_rad;
        debug->legacy_delta_pitch_rad = lc302_last_legacy_delta_pitch_rad;
        debug->aligned_delta_roll_rad = lc302_last_aligned_delta_roll_rad;
        debug->aligned_delta_pitch_rad = lc302_last_aligned_delta_pitch_rad;
        debug->legacy_comp_vx_cms = lc302_last_legacy_comp_vx_cms;
        debug->legacy_comp_vy_cms = lc302_last_legacy_comp_vy_cms;
        debug->aligned_comp_vx_cms = lc302_last_aligned_comp_vx_cms;
        debug->aligned_comp_vy_cms = lc302_last_aligned_comp_vy_cms;
        debug->timing_aligned_valid = lc302_last_timing_aligned_valid;
        debug->candidate_delta_x_cms = lc302_axis_x.velocity_delta_cms;
        debug->candidate_delta_y_cms = lc302_axis_y.velocity_delta_cms;
        debug->predicted_delta_x_cms = lc302_axis_x.predicted_delta_cms;
        debug->predicted_delta_y_cms = lc302_axis_y.predicted_delta_cms;
        debug->consistency_error_x_cms = lc302_axis_x.consistency_error_cms;
        debug->consistency_error_y_cms = lc302_axis_y.consistency_error_cms;
        debug->trust_level_x = lc302_axis_x.trust_level;
        debug->trust_level_y = lc302_axis_y.trust_level;
        debug->final_delta_x_cms = lc302_axis_x.final_delta_cms;
        debug->final_delta_y_cms = lc302_axis_y.final_delta_cms;
    }
    taskEXIT_CRITICAL();
}

void LC302_get_filter_debug(lc302_filter_debug_t *debug)
{
    LC302_get_snapshot(0, debug);
}

/**
 * @brief 清空光流输出数据
 * @param clear_position 是否同时清空位置积分（1:清空，0:保留）
 */
static void LC302_clear_output(uint8_t clear_position)
{
    // 清空原始和补偿后的位移数据
    flow_vel.raw_dx = 0;
    flow_vel.raw_dy = 0;
    flow_vel.comp_dx = 0.0f;
    flow_vel.comp_dy = 0.0f;
    // 清空速度数据
    flow_vel.vx_cms = 0.0f;
    flow_vel.vy_cms = 0.0f;
    flow_vel.dt_s = 0.0f;
    flow_vel.valid = 0;
    flow_vel.valid_x = 0;
    flow_vel.valid_y = 0;
    flow_vel.quality = 0.0f;

    // 可选：清空位置积分
    if(clear_position) {
        flow_vel.sum_x_cm = 0.0f;
        flow_vel.sum_y_cm = 0.0f;
    }
}

/**
 * @brief 复位LC302运行状态（用于落地、解锁等场景）
 * @param clear_position 是否同时清空位置积分
 */
static void LC302_reset_runtime_state(uint8_t clear_position)
{
    lc302_last_update_had_new_frame = 0;
    // 清空输出数据
    LC302_clear_output(clear_position);

    // 清空速度估计历史
    lc302_last_valid_vx = 0.0f;
    lc302_last_valid_vy = 0.0f;
    lc302_loss_count = 0;
    // 清空陀螺仪补偿延迟状态
    lc302_delayed_gyro_x = 0.0f;
    lc302_delayed_gyro_y = 0.0f;
    // 设置丢弃帧数（复位后前几帧可能包含旧积分）
    lc302_discard_frames = LC302_DISCARD_FRAMES_AFTER_RESET;
    LC302_reset_axis_filter_state(&lc302_axis_x);
    LC302_reset_axis_filter_state(&lc302_axis_y);
    lc302_candidate_reject_count = 0;

    // 清空中值滤波器历史
    for(int i = 0; i < 3; i++) {
        lc302_vx_med_hist[i] = 0.0f;
        lc302_vy_med_hist[i] = 0.0f;
    }

    // 临界区：清空中断相关数据
    taskENTER_CRITICAL();
    flow_gyro_accum_roll  = 0.0f;  // 清空陀螺仪累积量
    flow_gyro_accum_pitch = 0.0f;
    lc302_gyro_total_roll_rad = 0.0f;
    lc302_gyro_total_pitch_rad = 0.0f;
    lc302_gyro_last_update_us = GetSystemTimer();
    lc302_gyro_total_inited = 1u;
    lc302_rx_head_timing.head_time_us = 0u;
    lc302_rx_head_timing.gyro_total_roll_rad = 0.0f;
    lc302_rx_head_timing.gyro_total_pitch_rad = 0.0f;
    lc302_ready_timing.head_time_us = 0u;
    lc302_ready_timing.gyro_total_roll_rad = 0.0f;
    lc302_ready_timing.gyro_total_pitch_rad = 0.0f;
    lc302_previous_timing.head_time_us = 0u;
    lc302_previous_timing.gyro_total_roll_rad = 0.0f;
    lc302_previous_timing.gyro_total_pitch_rad = 0.0f;
    lc302_previous_timing_valid = 0u;
    lc302_data_ready = 0;          // 清空新帧标志
    // 复位UART接收状态机
    lc302_rx_state = 0;
    lc302_rx_index = 0;
    lc302_xor_check = 0;
    // 清空光流数据缓存
    lc302_data.pixel_flow_x_integral = 0;
    lc302_data.pixel_flow_y_integral = 0;
    lc302_data.integration_timespan = 0;
    lc302_data.ground_distance = 0;
    lc302_data.valid = 0;
    lc302_data.version = 0;
    taskEXIT_CRITICAL();

    lc302_last_frame_interval_us = 0u;
    lc302_last_integration_time_us = 0u;
    lc302_last_legacy_delta_roll_rad = 0.0f;
    lc302_last_legacy_delta_pitch_rad = 0.0f;
    lc302_last_aligned_delta_roll_rad = 0.0f;
    lc302_last_aligned_delta_pitch_rad = 0.0f;
    lc302_last_legacy_comp_vx_cms = 0.0f;
    lc302_last_legacy_comp_vy_cms = 0.0f;
    lc302_last_aligned_comp_vx_cms = 0.0f;
    lc302_last_aligned_comp_vy_cms = 0.0f;
    lc302_last_timing_aligned_valid = 0u;

    // 复位锁存（防止短时间内重复请求IMU复位）
    if(clear_position && !lc302_full_reset_latched) {
        lc302_full_reset_latched = 1;
        imu_horizontal_request_full_reset();
    }
}

void LC302_reset_runtime(void)
{
    LC302_reset_runtime_state(1u);
}

/**
 * @brief LC302初始化
 * @return 0-成功, 1-失败
 */
uint8_t LC302_init(void)
{
    // 初始化UART6
    uart_init(LC302_UART, LC302_UART_BAUD, LC302_TX_PIN, LC302_RX_PIN);

    // 上电/重新初始化时必须清掉历史帧、滤波状态和陀螺积分，防止首帧带旧积分起飞
    LC302_reset_runtime_state(1);

    // 使能UART6接收中断
    uart_rx_interrupt(LC302_UART, 1);

    return 0;  // 成功
}

/**
 * @brief LC302串口接收回调函数（状态机解析）
 * @param data 接收到的字节
 *
 * LC302数据帧格式：
 * [0xFE][0x0A][Data 10 bytes][XOR Check][0x55]
 * Data内容：
 *   - pixel_flow_x_integral (int16, 2 bytes)
 *   - pixel_flow_y_integral (int16, 2 bytes)
 *   - integration_timespan  (uint16, 2 bytes, 单位:微秒)
 *   - ground_distance       (uint16, 2 bytes, 单位:毫米)
 *   - valid                 (uint8, 1 byte, 0xF5表示有效)
 *   - version               (uint8, 1 byte)
 */
void LC302_uart_callback(uint8_t data)
{
    switch(lc302_rx_state)
    {
        case 0:  // 状态0：等待帧头1 (0xFE)
            if(data == 0xFE) {
                // 帧头最接近LC302本次积分结束，只做时间和累计角快照。
                lc302_rx_head_timing.head_time_us = GetSystemTimer();
                lc302_rx_head_timing.gyro_total_roll_rad = lc302_gyro_total_roll_rad;
                lc302_rx_head_timing.gyro_total_pitch_rad = lc302_gyro_total_pitch_rad;
                lc302_rx_state = 1;
            }
            break;

        case 1:  // 状态1：等待帧头2 (0x0A)
            if(data == 0x0A) {
                lc302_rx_state = 2;  // 进入数据接收状态
                lc302_rx_index = 0;  // 重置数据索引
                lc302_xor_check = 0; // 重置校验和
            } else {
                lc302_rx_state = 0;  // 不匹配，回到等待帧头1
            }
            break;

        case 2:  // 状态2：接收数据（10字节）
            lc302_rx_buffer[lc302_rx_index++] = data;
            lc302_xor_check ^= data;  // 累积XOR校验

            if(lc302_rx_index >= 10) {
                lc302_rx_state = 3;  // 10字节接收完毕，等待校验字节
            }
            break;

        case 3:  // 状态3：接收校验字节
            if(data == lc302_xor_check) {
                lc302_rx_state = 4;  // 校验通过，等待帧尾
            } else {
                lc302_rx_state = 0;  // 校验失败，重新开始
            }
            break;

        case 4:  // 状态4：接收帧尾 (0x55)
            if(data == 0x55) {
                // 帧尾正确，解析数据
                // 小端序：低字节在前，高字节在后
                lc302_data.pixel_flow_x_integral = (int16_t)(lc302_rx_buffer[0] | (lc302_rx_buffer[1] << 8));
                lc302_data.pixel_flow_y_integral = (int16_t)(lc302_rx_buffer[2] | (lc302_rx_buffer[3] << 8));
                lc302_data.integration_timespan = (uint16_t)(lc302_rx_buffer[4] | (lc302_rx_buffer[5] << 8));
                lc302_data.ground_distance = (uint16_t)(lc302_rx_buffer[6] | (lc302_rx_buffer[7] << 8));
                lc302_data.valid = lc302_rx_buffer[8];
                lc302_data.version = lc302_rx_buffer[9];

                if(lc302_data_ready != 0) {
                    lc302_frame_overrun_count++;
                }
                lc302_ready_timing = lc302_rx_head_timing;
                lc302_data_ready = 1;  // 标记数据就绪
                lc302_rx_frame_count++;
            }
            lc302_rx_state = 0;  // 无论成功与否，都回到初始状态
            break;

        default:
            lc302_rx_state = 0;
            break;
    }
}

/**
 * @brief LC302速度更新函数
 * @param height_mm 高度（毫米）
 */
uint8_t LC302_update_velocity(float height_mm)
{
    // 物理参数
    const float dt = PMW3901_DT_S;     // 0.02s

    float h_cm = height_mm * 0.1f;
    if(h_cm < 5.0f)   h_cm = 5.0f;
    if(h_cm > 500.0f) h_cm = 500.0f;

    // 低空/落地时光流不可信：硬清所有光流运行态和陀螺积分，避免下次起飞吃到旧积分
    lc302_last_update_had_new_frame = 0;
    if (h_cm < LC302_MIN_ACTIVE_HEIGHT_CM) {
        lc302_last_invalid_reason = LC302_INVALID_HEIGHT;
        LC302_reset_runtime_state(1);
        return 0u;
    }

    // 1. 原子地截取 LC302 帧与覆盖该帧周期的陀螺积分。
    // 本次更新周期陀螺仪的累积旋转量（弧度）
    float delta_roll_rad = 0.0f;
    float delta_pitch_rad = 0.0f;
    float aligned_delta_roll_rad = 0.0f;
    float aligned_delta_pitch_rad = 0.0f;
    uint8_t timing_aligned_valid = 0u;
    // 光流数据快照，避免在中断和任务间竞态
    lc302_data_t data_snapshot = {0};
    lc302_frame_timing_t timing_snapshot = {0};
    // 标志位：本周期是否收到新的光流帧
    uint8_t has_new_frame = 0;

    // === 临界区：原子读取光流数据和陀螺仪累积量 ===
    taskENTER_CRITICAL();
    // 检查是否有新的光流帧（UART中断设置此标志）
    if(lc302_data_ready != 0) {
        data_snapshot = lc302_data;      // 拷贝光流数据
        timing_snapshot.head_time_us = lc302_ready_timing.head_time_us;
        timing_snapshot.gyro_total_roll_rad = lc302_ready_timing.gyro_total_roll_rad;
        timing_snapshot.gyro_total_pitch_rad = lc302_ready_timing.gyro_total_pitch_rad;
        lc302_data_ready = 0;             // 清除新帧标志
        has_new_frame = 1;                // 本地标记有新帧
        delta_roll_rad  = flow_gyro_accum_roll;
        delta_pitch_rad = flow_gyro_accum_pitch;
        flow_gyro_accum_roll  = 0.0f;
        flow_gyro_accum_pitch = 0.0f;
    }
    taskEXIT_CRITICAL();

    if(has_new_frame == 0) {
        lc302_no_frame_poll_count++;
        return 0u;
    }

    lc302_last_update_had_new_frame = 1;
    lc302_processed_frame_count++;
    lc302_last_raw_valid = data_snapshot.valid;
    lc302_last_invalid_reason = 0;

    lc302_last_integration_time_us = data_snapshot.integration_timespan;
    lc302_last_frame_interval_us = 0u;
    if(lc302_previous_timing_valid) {
        lc302_last_frame_interval_us = timing_snapshot.head_time_us
                                     - lc302_previous_timing.head_time_us;
        aligned_delta_roll_rad = timing_snapshot.gyro_total_roll_rad
                               - lc302_previous_timing.gyro_total_roll_rad;
        aligned_delta_pitch_rad = timing_snapshot.gyro_total_pitch_rad
                                - lc302_previous_timing.gyro_total_pitch_rad;
        if((lc302_last_frame_interval_us >= 10000u)
        && (lc302_last_frame_interval_us <= 50000u)
        && (LC302_abs_diff_u32(lc302_last_frame_interval_us,
                               data_snapshot.integration_timespan)
            <= LC302_TIMING_MATCH_TOLERANCE_US)) {
            timing_aligned_valid = 1u;
        }
    }
    // 每个协议完整帧都推进时间基准，传感器valid位不影响下一帧边界。
    lc302_previous_timing = timing_snapshot;
    lc302_previous_timing_valid = 1u;
    lc302_last_legacy_delta_roll_rad = delta_roll_rad;
    lc302_last_legacy_delta_pitch_rad = delta_pitch_rad;
    lc302_last_aligned_delta_roll_rad = aligned_delta_roll_rad;
    lc302_last_aligned_delta_pitch_rad = aligned_delta_pitch_rad;
    lc302_last_timing_aligned_valid = timing_aligned_valid;
    lc302_last_legacy_comp_vx_cms = 0.0f;
    lc302_last_legacy_comp_vy_cms = 0.0f;
    lc302_last_aligned_comp_vx_cms = 0.0f;
    lc302_last_aligned_comp_vy_cms = 0.0f;

    // 最终输出的速度（单位：cm/s）
    float final_vx_cm = 0.0f;
    float final_vy_cm = 0.0f;
    float update_dt = dt;
    float max_vel_delta_x = LC302_MAX_TRANSLATIONAL_ACCEL_CMS2 * dt;
    float max_vel_delta_y = LC302_MAX_TRANSLATIONAL_ACCEL_CMS2 * dt;
    imu_horizontal_state_t horizontal_state = {0};

    // 策略1：复位后前几帧丢弃
    // 原因：复位/落地后传感器内部可能残留旧的积分值，需要丢弃
    if(lc302_discard_frames > 0) {
        lc302_discard_frames--;
        lc302_last_invalid_reason = LC302_INVALID_DISCARD;
        LC302_clear_output(0);  // 清零输出但保留位置
        return 1u;
    }

    // 策略2：LC302自带地距判定
    // 即使飞控高度未及时更新，也能通过LC302自身的地距数据判断是否贴地
    if((data_snapshot.ground_distance > 0U)
    && (data_snapshot.ground_distance < LC302_MIN_ACTIVE_GROUND_MM)) {
        lc302_last_invalid_reason = LC302_INVALID_GROUND;
        LC302_reset_runtime_state(1);  // 贴地时复位运行状态
        lc302_last_update_had_new_frame = 1;
        return 1u;
    }

    // 策略3：读取LC302原始像素位移数据
    int16_t raw_dx = data_snapshot.pixel_flow_x_integral;
    int16_t raw_dy = data_snapshot.pixel_flow_y_integral;
    // 质量值（0-255，越大越好）
    flow_vel.quality = (float)data_snapshot.valid;

    // 策略4：数据有效性判断
    // 策略4：数据有效性判断
    bool is_valid = true;
    uint8_t force_zero_output = 0;

    // 检查1：LC302有效标志位检查（0xF5表示有效）
    // LC302 没有 Quality 概念，仅严格判断 Valid 标志位
    if (data_snapshot.valid != 0xF5) {
        is_valid = false;
        lc302_last_invalid_reason |= LC302_INVALID_RAW_FLAG;
    }

    // 检查2：陀螺仪旋转量异常（跨周期累积或剧烈旋转）
    // 首帧/丢帧/落地后最危险的是陀螺积分跨周期累积；超过单帧合理角度就丢帧清零
    {
        float gyro_check_roll_rad = delta_roll_rad;
        float gyro_check_pitch_rad = delta_pitch_rad;
#if LC302_GYRO_ALIGN_MODE == LC302_GYRO_ALIGN_ACTIVE
        if(timing_aligned_valid) {
            gyro_check_roll_rad = aligned_delta_roll_rad;
            gyro_check_pitch_rad = aligned_delta_pitch_rad;
        }
#endif
        if ((fabsf(gyro_check_roll_rad)  > LC302_MAX_GYRO_FRAME_RAD)
         || (fabsf(gyro_check_pitch_rad) > LC302_MAX_GYRO_FRAME_RAD)) {
            is_valid = false;
            lc302_last_invalid_reason |= LC302_INVALID_GYRO;
            force_zero_output = 1;  // 强制输出清零，防止漂移
        }
    }

    // 检查3：原始积分过大，可能是旧帧/串口错帧/内部积分残留
    if ((raw_dx > LC302_MAX_FLOW_FRAME_COUNTS) || (raw_dx < -LC302_MAX_FLOW_FRAME_COUNTS)
     || (raw_dy > LC302_MAX_FLOW_FRAME_COUNTS) || (raw_dy < -LC302_MAX_FLOW_FRAME_COUNTS)) {
        is_valid = false;
        lc302_last_invalid_reason |= LC302_INVALID_FLOW_RANGE;
        force_zero_output = 1;
    }

    // 检查4：异常突变检测
    // 原始数据全0且上一时刻速度较大时，可能是传感器死机突变，进行异常拦截
    if (raw_dx == 0 && raw_dy == 0 && (fabsf(lc302_last_valid_vx) > 10.0f || fabsf(lc302_last_valid_vy) > 10.0f)) {
        is_valid = false;
        lc302_last_invalid_reason |= LC302_INVALID_ZERO_FLOW;
    }

    // 提取帧时间间隔（单位：秒）
    update_dt = (float)data_snapshot.integration_timespan / 1000000.0f;
    // 时间间隔合理性检查（10ms-50ms），不合理时使用默认20ms
    if(update_dt < 0.01f || update_dt > 0.05f) {
        lc302_last_invalid_reason |= LC302_INVALID_DT;
        update_dt = 0.02f;
    }
    max_vel_delta_x = LC302_MAX_TRANSLATIONAL_ACCEL_CMS2 * update_dt;
    max_vel_delta_y = LC302_MAX_TRANSLATIONAL_ACCEL_CMS2 * update_dt;
    imu_horizontal_get_state(&horizontal_state);

    // ========================================================================
    // === 数据有效时的完整处理流程 ===
    // ========================================================================
    if (is_valid)
    {
        // 保存原始数据供调试
        flow_vel.raw_dx = raw_dx;
        flow_vel.raw_dy = raw_dy;
        //printf("%d,%d\r\n", (int)raw_dx, (int)raw_dy);

        // 步骤5：LC302数据格式转换（原始值除以10000得到弧度）
        float flow_x_rad = -1.0f * (float)raw_dx / 10000.0f;  // 负号用于匹配机体坐标系
        float flow_y_rad = -1.0f * (float)raw_dy / 10000.0f;

#if LC302_MOUNT_CW90
        // 如果光流模块安装时顺时针旋转了90度，需要软件矫正坐标系
        {
            float old_flow_x_rad = flow_y_rad;
            float old_flow_y_rad = -flow_x_rad;
            flow_x_rad = old_flow_x_rad;
            flow_y_rad = old_flow_y_rad;
        }
#endif

        // 步骤6：直接使用当前光流数据
        // 重要：光流本身已是迟滞信号，切忌再加滤波，否则会进一步增加延迟
        float current_flow_x = flow_x_rad;   // 注意：修改极性时需同步修改
        float current_flow_y = flow_y_rad;   // 注意：修改极性时需同步修改

        // 步骤7：陀螺仪补偿策略
        // 问题：陀螺仪更新快(1ms)，光流更新慢(~20ms)，直接相减会有时间不对齐问题
        // 解决：对陀螺仪积分加一阶低通滤波，模拟延迟，使其与光流时间对齐
        const float GYRO_DELAY_ALPHA = 0.45f;  // 延迟滤波系数

        // 对陀螺仪累积量进行一阶低通滤波（模拟延迟）
        lc302_delayed_gyro_x = delta_roll_rad * GYRO_DELAY_ALPHA + lc302_delayed_gyro_x * (1.0f - GYRO_DELAY_ALPHA);  // 注意：修改极性时需同步修改
        lc302_delayed_gyro_y = delta_pitch_rad  * GYRO_DELAY_ALPHA + lc302_delayed_gyro_y * (1.0f - GYRO_DELAY_ALPHA); // 注意：修改极性时需同步修改

        // 陀螺仪补偿增益（根据实际测试调整，用于校正陀螺仪与光流的尺度差异）
        const float GYRO_SCALE_X = 1.07f;
        const float GYRO_SCALE_Y = 1.07f;

        // 用延迟后的陀螺仪数据补偿光流（去除飞机旋转引起的虚假位移）
        float comp_x_rad = current_flow_x - (lc302_delayed_gyro_x * GYRO_SCALE_X);
        float comp_y_rad = current_flow_y - (lc302_delayed_gyro_y * GYRO_SCALE_Y);
        float legacy_comp_x_rad = comp_x_rad;
        float legacy_comp_y_rad = comp_y_rad;
        float aligned_comp_x_rad = current_flow_x - (aligned_delta_roll_rad * GYRO_SCALE_X);
        float aligned_comp_y_rad = current_flow_y - (aligned_delta_pitch_rad * GYRO_SCALE_Y);
        float selected_delta_roll_rad = delta_roll_rad;
        float selected_delta_pitch_rad = delta_pitch_rad;

        lc302_last_legacy_comp_vx_cms = legacy_comp_x_rad * (h_cm / update_dt);
        lc302_last_legacy_comp_vy_cms = legacy_comp_y_rad * (h_cm / update_dt);
        lc302_last_aligned_comp_vx_cms = aligned_comp_x_rad * (h_cm / update_dt);
        lc302_last_aligned_comp_vy_cms = aligned_comp_y_rad * (h_cm / update_dt);

#if LC302_GYRO_ALIGN_MODE == LC302_GYRO_ALIGN_ACTIVE
        if(timing_aligned_valid) {
            comp_x_rad = aligned_comp_x_rad;
            comp_y_rad = aligned_comp_y_rad;
            selected_delta_roll_rad = aligned_delta_roll_rad;
            selected_delta_pitch_rad = aligned_delta_pitch_rad;
        }
#endif

        // 步骤7.5：自适应融合策略
        // 当角速度较大时，光流数据不可靠（运动模糊），需降低光流权重
        float roll_rate_dps = fabsf(selected_delta_roll_rad) / (update_dt * DEG2RAD);   // 计算横滚角速度(度/秒)
        float pitch_rate_dps = fabsf(selected_delta_pitch_rad) / (update_dt * DEG2RAD); // 计算俯仰角速度(度/秒)

        // 计算融合系数：角速度在30-70度/秒之间线性插值
        float blend_x = (roll_rate_dps - LC302_GYRO_BLEND_START_DPS)
                      / (LC302_GYRO_BLEND_END_DPS - LC302_GYRO_BLEND_START_DPS);
        float blend_y = (pitch_rate_dps - LC302_GYRO_BLEND_START_DPS)
                      / (LC302_GYRO_BLEND_END_DPS - LC302_GYRO_BLEND_START_DPS);

        // 限制融合系数在[0, 1]范围内
        if(blend_x < 0.0f) blend_x = 0.0f;
        if(blend_x > 1.0f) blend_x = 1.0f;
        if(blend_y < 0.0f) blend_y = 0.0f;
        if(blend_y > 1.0f) blend_y = 1.0f;

        // 计算光流权重（角速度越大，光流权重越小，最小保留10%）
        float flow_weight_x = 1.0f - blend_x * (1.0f - LC302_GYRO_MIN_FLOW_WEIGHT);
        float flow_weight_y = 1.0f - blend_y * (1.0f - LC302_GYRO_MIN_FLOW_WEIGHT);


        // 步骤8：转换为速度 (cm/s)
        // 公式：速度 = 角位移 × 高度 / 时间
        // LC302输出已经是弧度，直接用于速度计算
        float vx_raw = comp_x_rad * (h_cm / update_dt);
        float vy_raw = comp_y_rad * (h_cm / update_dt);

        // 应用自适应融合：角速度大时更多依赖上一时刻速度（降低光流权重）
        vx_raw = lc302_last_valid_vx
               + (vx_raw - lc302_last_valid_vx) * flow_weight_x;
        vy_raw = lc302_last_valid_vy
               + (vy_raw - lc302_last_valid_vy) * flow_weight_y;

        // 反向计算补偿后的角位移（用于输出调试）
        comp_x_rad = vx_raw * update_dt / h_cm;
        comp_y_rad = vy_raw * update_dt / h_cm;

        // 转换为LC302格式保存（乘以10000）
        flow_vel.comp_dx = comp_x_rad * 10000.0f;
        flow_vel.comp_dy = comp_y_rad * 10000.0f;

        uint8_t valid_x;
        uint8_t valid_y;

        if((fabsf(vx_raw) < fabsf(lc302_last_valid_vx))
        || ((vx_raw * lc302_last_valid_vx) < 0.0f)) {
            max_vel_delta_x = LC302_MAX_TRANSLATIONAL_BRAKE_CMS2 * update_dt;
        }
        if((fabsf(vy_raw) < fabsf(lc302_last_valid_vy))
        || ((vy_raw * lc302_last_valid_vy) < 0.0f)) {
            max_vel_delta_y = LC302_MAX_TRANSLATIONAL_BRAKE_CMS2 * update_dt;
        }

        valid_x = LC302_accept_axis_candidate(vx_raw,
                                               lc302_last_valid_vx,
                                               max_vel_delta_x,
                                               &lc302_axis_x);
        valid_y = LC302_accept_axis_candidate(vy_raw,
                                               lc302_last_valid_vy,
                                               max_vel_delta_y,
                                               &lc302_axis_y);

        if(!valid_x || !valid_y) {
            lc302_candidate_reject_count++;
        }
        if(!valid_x) {
            lc302_last_invalid_reason |= LC302_INVALID_CANDIDATE_X;
        }
        if(!valid_y) {
            lc302_last_invalid_reason |= LC302_INVALID_CANDIDATE_Y;
        }

        if(valid_x) {
            float vx_med;
            float filter_alpha_x;
            if(lc302_axis_x.valid_streak < 255u) {
                lc302_axis_x.valid_streak++;
            }
            filter_alpha_x = LC302_select_axis_filter_alpha(vx_raw,
                                                             lc302_last_valid_vx,
                                                             update_dt,
                                                             horizontal_state.nav_acc_x_cms2,
                                                             horizontal_state.acc_bias_x_cms2,
                                                             &lc302_axis_x);
            vx_raw = SlewRateLimiter(vx_raw, lc302_last_valid_vx, max_vel_delta_x);
            if(lc302_axis_x.history_stale) {
                LC302_fill_median_history(lc302_vx_med_hist, vx_raw);
            }
            vx_med = MedianFilter3_Float(vx_raw, lc302_vx_med_hist);
            final_vx_cm = vx_med * filter_alpha_x
                        + lc302_last_valid_vx * (1.0f - filter_alpha_x);
            lc302_axis_x.final_delta_cms = final_vx_cm - lc302_last_valid_vx;
            lc302_last_valid_vx = final_vx_cm;
            lc302_axis_x.history_stale = 0;
        } else {
            lc302_axis_x.valid_streak = 0;
            lc302_axis_x.fast_filter_active = 0;
            lc302_axis_x.trust_level = 0;
            lc302_axis_x.filter_alpha = LC302_FILTER_ALPHA_NORMAL;
            lc302_axis_x.final_delta_cms = 0.0f;
            final_vx_cm = lc302_last_valid_vx;
        }

        if(valid_y) {
            float vy_med;
            float filter_alpha_y;
            if(lc302_axis_y.valid_streak < 255u) {
                lc302_axis_y.valid_streak++;
            }
            filter_alpha_y = LC302_select_axis_filter_alpha(vy_raw,
                                                             lc302_last_valid_vy,
                                                             update_dt,
                                                             horizontal_state.nav_acc_y_cms2,
                                                             horizontal_state.acc_bias_y_cms2,
                                                             &lc302_axis_y);
            vy_raw = SlewRateLimiter(vy_raw, lc302_last_valid_vy, max_vel_delta_y);
            if(lc302_axis_y.history_stale) {
                LC302_fill_median_history(lc302_vy_med_hist, vy_raw);
            }
            vy_med = MedianFilter3_Float(vy_raw, lc302_vy_med_hist);
            final_vy_cm = vy_med * filter_alpha_y
                        + lc302_last_valid_vy * (1.0f - filter_alpha_y);
            lc302_axis_y.final_delta_cms = final_vy_cm - lc302_last_valid_vy;
            lc302_last_valid_vy = final_vy_cm;
            lc302_axis_y.history_stale = 0;
        } else {
            lc302_axis_y.valid_streak = 0;
            lc302_axis_y.fast_filter_active = 0;
            lc302_axis_y.trust_level = 0;
            lc302_axis_y.filter_alpha = LC302_FILTER_ALPHA_NORMAL;
            lc302_axis_y.final_delta_cms = 0.0f;
            final_vy_cm = lc302_last_valid_vy;
        }

        if(valid_x || valid_y) {
            lc302_loss_count = 0;
            lc302_full_reset_latched = 0;
        } else if(lc302_loss_count < 255u) {
            lc302_loss_count++;
        }
        flow_vel.valid_x = valid_x;
        flow_vel.valid_y = valid_y;
        flow_vel.valid = valid_x && valid_y;
    }
    else  // 数据无效分支
    {
        LC302_mark_axis_history_stale(&lc302_axis_x);
        LC302_mark_axis_history_stale(&lc302_axis_y);
        // 数据无效，使用保持/衰减逻辑
        // 如果判定为旧积分/旧帧，则强制清零并丢掉后续几帧
        lc302_loss_count++;
        // 清空延迟陀螺补偿（无效数据时不能继续补偿）
        lc302_delayed_gyro_x = 0.0f;
        lc302_delayed_gyro_y = 0.0f;

        if(force_zero_output) {
            // 强制清零模式（检测到旧帧残留或异常跳变）
            LC302_clear_output(0);
            final_vx_cm = 0.0f;
            final_vy_cm = 0.0f;
            lc302_last_valid_vx = 0.0f;
            lc302_last_valid_vy = 0.0f;
            lc302_loss_count = 11;  // 设置为长期丢帧状态
            lc302_discard_frames = LC302_DISCARD_FRAMES_AFTER_RESET;  // 丢弃后续几帧
            // 清空中值滤波器历史
            for(int i = 0; i < 3; i++) {
                lc302_vx_med_hist[i] = 0.0f;
                lc302_vy_med_hist[i] = 0.0f;
            }
        } else if (lc302_loss_count <= 3) {
            // 短期丢帧：完全保持上次有效速度
            final_vx_cm = lc302_last_valid_vx;
            final_vy_cm = lc302_last_valid_vy;
        } else if (lc302_loss_count <= 10) {
            // 中期丢帧：逐渐衰减速度（避免漂移累积）
            final_vx_cm = lc302_last_valid_vx * 0.8f;
            final_vy_cm = lc302_last_valid_vy * 0.8f;
            lc302_last_valid_vx = final_vx_cm;
            lc302_last_valid_vy = final_vy_cm;
        } else {
            // 长期丢帧：清零所有速度和历史
            final_vx_cm = 0.0f;
            final_vy_cm = 0.0f;
            lc302_last_valid_vx = 0.0f;
            lc302_last_valid_vy = 0.0f;
            for(int i = 0; i < 3; i++) {
                lc302_vx_med_hist[i] = 0.0f;
                lc302_vy_med_hist[i] = 0.0f;
            }
        }
        flow_vel.valid = 0;
        flow_vel.valid_x = 0;
        flow_vel.valid_y = 0;
    }

    flow_vel.vx_cms = final_vx_cm;   /////////////////////极性修改处
    flow_vel.vy_cms = final_vy_cm;   /////////////////////极性修改处
    flow_vel.dt_s = update_dt;

    // 位置积分
    if(flow_vel.valid_x) {
        flow_vel.sum_x_cm += flow_vel.vx_cms * update_dt;
    }
    if(flow_vel.valid_y) {
        flow_vel.sum_y_cm += flow_vel.vy_cms * update_dt;
    }
    return 1u;
}

void FLOW_update_velocity(float height_mm)
{
    (void)LC302_update_velocity(height_mm);
}

#else
// ============================================================================
//   PMW3901 SPI驱动实现（原有代码）
// ============================================================================

void FLOW_update_velocity(float height_mm)
{
    // 滤波器历史状态
    static float vx_med_hist[3] = {0.0f, 0.0f, 0.0f};
    static float vy_med_hist[3] = {0.0f, 0.0f, 0.0f};

    static float last_valid_vx = 0.0f;
    static float last_valid_vy = 0.0f;
    static uint8_t loss_count = 0;

    // 物理参数
    const float dt = PMW3901_DT_S;     // 0.02s
    const float MAX_VEL_DELTA = 30.0f;

    float h_cm = height_mm * 0.1f;
    if(h_cm < 5.0f)   h_cm = 5.0f;
    if(h_cm > 500.0f) h_cm = 500.0f;

    // 1. 获取安全的陀螺仪积分
    float delta_roll_rad = 0.0f;
    float delta_pitch_rad = 0.0f;

    taskENTER_CRITICAL();
    delta_roll_rad  = flow_gyro_accum_roll;
    delta_pitch_rad = flow_gyro_accum_pitch;
    flow_gyro_accum_roll  = 0.0f;
    flow_gyro_accum_pitch = 0.0f;
    taskEXIT_CRITICAL();

    // 2. 调用逐飞库官方接口读取位移
    pmw3901_get_motion();
    int16 raw_dx = pmw3901_delta_x;
    int16 raw_dy = pmw3901_delta_y;

    // 3. 读取表面质量 SQUAL (寄存器 0x07)
    uint8 squal = 0;
    PMW3901_CS(0);
    spi_read_8bit_registers(PMW3901_SPI, 0x07, &squal, 1);
    PMW3901_CS(1);
    flow_vel.quality = (float)squal;

    float final_vx_cm = 0.0f;
    float final_vy_cm = 0.0f;

    // 4. 智能数据有效性判断
    bool is_valid = true;
    if (flow_vel.quality < 10.0f) {
        is_valid = false;
    }
    if (raw_dx == 0 && raw_dy == 0 && (fabsf(last_valid_vx) > 10.0f || fabsf(last_valid_vy) > 10.0f)) {
        is_valid = false;
    }

    if (is_valid)
    {
        flow_vel.raw_dx = raw_dx;
        flow_vel.raw_dy = raw_dy;

        // (1) 陀螺仪旋转量计算
        float rot_x_counts_raw = -delta_roll_rad * PMW3901_COUNTS_PER_RAD;
        float rot_y_counts_raw = -delta_pitch_rad  * PMW3901_COUNTS_PER_RAD;

        static float rot_x_counts_lpf = 0.0f;
        static float rot_y_counts_lpf = 0.0f;

        const float COMP_DELAY_ALPHA = 0.95f;

        rot_x_counts_lpf = rot_x_counts_raw * COMP_DELAY_ALPHA + rot_x_counts_lpf * (1.0f - COMP_DELAY_ALPHA);
        rot_y_counts_lpf = rot_y_counts_raw * (COMP_DELAY_ALPHA+0.15f) + rot_y_counts_lpf * (1.0f - COMP_DELAY_ALPHA - 0.15f);

        // (2) 像素补偿
        float comp_dx = (float)raw_dx - rot_x_counts_lpf * 0.99f;
        float comp_dy = (float)raw_dy - rot_y_counts_lpf * 1.02f;

        flow_vel.comp_dx = comp_dx;
        flow_vel.comp_dy = comp_dy;

        // (3) 转换为物理速度 cm/s
        float vx_raw = (comp_dx / PMW3901_COUNTS_PER_RAD) * (h_cm / dt);
        float vy_raw = (comp_dy / PMW3901_COUNTS_PER_RAD) * (h_cm / dt);

        // (4) 加速度限幅
        vx_raw = SlewRateLimiter(vx_raw, last_valid_vx, MAX_VEL_DELTA);
        vy_raw = SlewRateLimiter(vy_raw, last_valid_vy, MAX_VEL_DELTA);

        // (5) 中值滤波
        float vx_med = MedianFilter3_Float(vx_raw, vx_med_hist);
        float vy_med = MedianFilter3_Float(vy_raw, vy_med_hist);

        // (6) 低通滤波
        final_vx_cm = vx_med * 0.5f + last_valid_vx * 0.5f;
        final_vy_cm = vy_med * 0.5f + last_valid_vy * 0.5f;

        last_valid_vx = final_vx_cm;
        last_valid_vy = final_vy_cm;
        loss_count = 0;
        flow_vel.valid = 1;
        flow_vel.valid_x = 1;
        flow_vel.valid_y = 1;
    }
    else
    {
        loss_count++;
        flow_vel.valid = 0;
        flow_vel.valid_x = 0;
        flow_vel.valid_y = 0;

        if (loss_count <= 3) {
            final_vx_cm = last_valid_vx;
            final_vy_cm = last_valid_vy;
        } else if (loss_count <= 10) {
            final_vx_cm = last_valid_vx * 0.8f;
            final_vy_cm = last_valid_vy * 0.8f;
            last_valid_vx = final_vx_cm;
            last_valid_vy = final_vy_cm;
        } else {
            final_vx_cm = 0.0f;
            final_vy_cm = 0.0f;
            last_valid_vx = 0.0f;
            last_valid_vy = 0.0f;
            for(int i=0; i<3; i++) { vx_med_hist[i] = 0.0f; vy_med_hist[i] = 0.0f; }
        }
    }

    flow_vel.vx_cms = -final_vx_cm;
    flow_vel.vy_cms = -final_vy_cm;
    flow_vel.dt_s = dt;

    // 位置积分
    if(loss_count <= 3) {
        flow_vel.sum_x_cm += flow_vel.vx_cms * dt;
        flow_vel.sum_y_cm += flow_vel.vy_cms * dt;
    }
}

#endif

// ============================================================================
//   通用函数（PMW3901和LC302共用）
// ============================================================================


// 中值滤波
/**
 * @brief 三点中值滤波器
 * @param new_val 新的输入值
 * @param history 历史值数组[3]（history[0]最新，history[2]最旧）
 * @return 中值
 *
 * 作用：去除偶发的尖峰噪声
 * 原理：取最近3个值的中间值，对单次异常值有抑制作用
 */
static float MedianFilter3_Float(float new_val, float *history)
{
    // 更新历史值：新值->最新，最新->次新，次新->最旧
    history[2] = history[1];
    history[1] = history[0];
    history[0] = new_val;

    // 冒泡排序取中值
    float a = history[0], b = history[1], c = history[2];
    if (a > b) { float tmp = a; a = b; b = tmp; }
    if (b > c) { float tmp = b; b = c; c = tmp; }
    if (a > b) { float tmp = a; a = b; b = tmp; }
    return b;  // 返回中间值
}

/**
 * @brief 变化率限幅器（加速度限幅）
 * @param new_val 新的目标值
 * @param last_val 上一次的值
 * @param max_delta 最大允许变化量
 * @return 限幅后的值
 *
 * 作用：限制值的变化速度，防止突变过大
 * 应用：限制速度估计的加速度，避免控制震荡
 */
static float SlewRateLimiter(float new_val, float last_val, float max_delta)
{
    float delta = new_val - last_val;
    if (delta > max_delta) return last_val + max_delta;   // 增加过快，限制增速
    if (delta < -max_delta) return last_val - max_delta;  // 减少过快，限制减速
    return new_val;  // 变化在允许范围内，直接返回
}

/**
 * @brief 光流传感器初始化（根据宏定义选择）
 * @return 0-成功, 1-失败
 */
uint8 FLOW_init(void)
{
#if USE_LC302_FLOW
    return LC302_init();
#else
    if (pmw3901_init() == 0) return 1;
    return 0;
#endif
}

/**
 * @brief 重置光流位置积分
 * 用于飞行中需要清零位置的场景
 */
void FLOW_reset_position(void)
{
    taskENTER_CRITICAL();
    flow_vel.sum_x_cm = 0.0f;
    flow_vel.sum_y_cm = 0.0f;
    taskEXIT_CRITICAL();
    imu_horizontal_request_position_reset();
}
