#ifndef PID_H
#define PID_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "zf_common_function.h"
#include "zf_driver_timer.h"

#ifndef __FLT_MAX__
#define __FLT_MAX__ FLT_MAX
#endif

#ifndef __FLT_MIN__
#define __FLT_MIN__ FLT_MIN
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 宏定义     控制频率
 * 参数说明   CTRL_HZ          控制器运行频率(Hz)
 * 备注信息   用于计算固定时间步长，确保PID计算的稳定性
-------------------------------------------------------------------------------------------------------------------*/
#define CTRL_HZ 1000

/* 全局常量 */
extern const float numeric_limits_float_epsilon;
static const float dt_inner = 1.0f / CTRL_HZ;       // 0.001 秒

/*-------------------------------------------------------------------------------------------------------------------
 * 类型定义     系统时钟获取函数指针
 * 返回参数     uint32_t        系统时钟计数值
 * 备注信息     用于实现时间无关的PID计算，便于移植到不同平台
-------------------------------------------------------------------------------------------------------------------*/
typedef uint32_t (*GetSystemTickFunc)(void);

/*-------------------------------------------------------------------------------------------------------------------
 * 结构体定义   PID参数结构体
 * 成员说明     dt              时间间隔(s)
 * 成员说明     last_time       上次时间戳
 * 成员说明     kp, ki, kd      PID三个增益参数
 * 成员说明     imax            积分项限幅值
 * 成员说明     outmax          输出限幅值
 * 成员说明     integrator      积分器累加值
 * 成员说明     last_error      上一次误差
 * 成员说明     last_derivative 上一次微分值
 * 成员说明     last_last_error 上上次误差（用于增量式PID）
 * 成员说明     var_speed_I_A/B 变速积分参数
 * 成员说明     I_separate_thresh 积分分离阈值
 * 成员说明     feed_forward    前馈项
 * 成员说明     anti_windup_enabled 抗积分饱和使能标志
 * 成员说明     Get_SystemTick  系统时钟获取函数指针
 * 成员说明     各种标志位（抗饱和、输出限幅等）
 * 备注信息     包含位置式和增量式PID所需的所有参数
-------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    /* 时间相关 */
    float    dt;                // 时间间隔(s)
    uint32_t last_time;         // 上次时间戳
    uint32_t (*Get_SystemTick)(void); // 系统时钟获取函数指针（可选）

    /* PID 系数与限幅 */
    float kp;                   // 比例增益
    float ki;                   // 积分增益
    float kd;                   // 微分增益
    float imax;                 // I项输出限幅（或用于等价积分限幅的基准）
    float outmax;               // 总输出限幅

    /* 输出项 */
    float out_p;                // P项输出
    float out_i;                // I项输出
    float out_d;                // D项输出
    float out;                  // 总输出值
    float feed_forward;         // 前馈项

    /* 误差/积分状态 */
    float integrator;           // 积分器累加值
    float last_error;           // 上一次误差
    float last_last_error;      // 上上次误差（增量式PID可用）
    float last_derivative;      // 上一次微分值（可选，用于某些D实现）

    /* 变速积分 / 积分分离参数 */
    float var_speed_I_A;        // 变速积分参数A（线性段长度）
    float var_speed_I_B;        // 变速积分参数B（零积分区）
    float I_separate_thresh;    // 积分分离阈值

    /* 死区与误差滤波（可选） */
    float   DeadZone;           // 死区范围
    uint8_t efilter;            // 误差滤波使能标志（0/1）
    float   e_term_lpf;         // 误差低通滤波系数(0~1)

    /* D项配置：滤波 + 模式 */
    uint8_t d_eflag;            // D项滤波使能标志（0/1）
    uint8_t d_mode;             // D模式：0=对误差求导，1=对测量值求导(需meas/last_meas)
    float   d_term_lpf;         // D项低通滤波后的“导数状态/结果”
    float   last_d_output;      // 上一次D输出（增量式D可能用到）
    uint8_t d_filter_mode;      // 0=legacy single-pole, 1=limited two-pole
    uint8_t d_input_valid;      // measurement validity gate for D-on-measurement
    uint8_t i_input_valid;      // integration update gate; existing I output is retained while frozen
    float   d_term_lpf2;        // second pole state
    float   d_filter_cutoff_hz;
    float   d_raw_limit;
    float   d_output_limit;
    float   d_output_slew_rate;

    /* D on measurement 需要的测量值缓存（由外部写入 meas） */
    float meas;                 // 当前测量值（例如gyro角速度），外部每次更新
    float last_meas;            // 上一次测量值（内部更新）
    uint8_t meas_inited;

    /* 抗积分饱和相关（可选：你可以只用其中一套） */
    float   windup_limit;        // 抗饱和限幅/回灌参数（按你的实现定义）
    uint8_t anti_windup_flag;    // 抗饱和标志（运行时状态）
    uint8_t i_decrease_flag;     // 积分分离/减弱标志（运行时状态）
    uint8_t imax_flag;           // I项限幅使能标志（0/1）
    uint8_t outmax_flag;         // 输出限幅使能标志（0/1）

} pid_param_t;

typedef struct {
    float target_cmd_cms;
    float target_ref_cms;
    float target_accel_cms2;
    float kp;                    // 速度环当前实际使用的比例增益
    float measured_cms;
    float derivative_raw_cms2;
    float derivative_filtered_cms2;
    float out_p_deg;
    float out_i_deg;
    float out_d_deg;
    float out_ff_deg;
    float out_total_deg;
    uint32_t i_reset_count;
    uint16_t i_stable_count;
    uint8_t flow_valid;
    uint8_t i_enabled;
    float control_confidence;
    float output_limit_scale;
    uint8_t control_source;
    uint8_t output_limited;
    uint8_t d_input_valid;
    float out_p_accel_cms2;
    float out_i_accel_cms2;
    float out_d_accel_cms2;
    float out_ff_accel_cms2;
    float out_total_accel_cms2;
    float brake_scale;
    uint8_t accel_limited;
    float feedforward_effective_cms;
    uint8_t feedforward_conflict_limited;
} horizontal_velocity_axis_debug_t;

typedef struct {
    horizontal_velocity_axis_debug_t pitch;
    horizontal_velocity_axis_debug_t roll;
    uint32_t full_reset_count;
} horizontal_velocity_debug_t;

// 姿态动态零点状态：每次飞行只学习一次，锁定后重新清零。
typedef enum {
    ATTITUDE_TRIM_IDLE = 0,
    ATTITUDE_TRIM_LEARN,
    ATTITUDE_TRIM_TRANSFER,
    ATTITUDE_TRIM_FROZEN
} attitude_auto_trim_state_t;

// 动态零点停止原因，供遥测区分正常完成、提前跟随和安全退出。
typedef enum {
    ATTITUDE_TRIM_STOP_NONE = 0,
    ATTITUDE_TRIM_STOP_TIME_COMPLETE,
    ATTITUDE_TRIM_STOP_FOLLOW_STARTED,
    ATTITUDE_TRIM_STOP_HEIGHT_EXIT,
    ATTITUDE_TRIM_STOP_LANDING_STARTED,
    ATTITUDE_TRIM_STOP_LOCKED,
    ATTITUDE_TRIM_STOP_RC_INPUT
} attitude_auto_trim_stop_reason_t;

typedef struct {
    attitude_auto_trim_state_t state;
    attitude_auto_trim_stop_reason_t stop_reason;
    uint32_t valid_learn_ms;
    uint16_t sample_gate_mask;
    uint8_t rc_input_active;
    uint8_t rc_stable;
    uint8_t learn_allowed;
    uint16_t learn_block_mask;
    float pitch_candidate_deg;
    float roll_candidate_deg;
    float pitch_target_deg;
    float roll_target_deg;
    float pitch_applied_deg;
    float roll_applied_deg;
    float pitch_effective_zero_deg;
    float roll_effective_zero_deg;
} attitude_auto_trim_debug_t;


/*-------------------------------------------------------------------------------------------------------------------
 * 全局变量声明
 * 变量说明      pitch_acc_pid    俯仰角加速度PID控制器
 * 变量说明      roll_acc_pid     横滚角加速度PID控制器
 * 变量说明      yaw_acc_pid      偏航角加速度PID控制器
 * 变量说明      pitch_angle_pid  俯仰角PID控制器
 * 变量说明      roll_angle_pid   横滚角PID控制器
 * 变量说明      yaw_angle_pid    偏航角PID控制器
 * 变量说明      pitch_vel_pid    俯仰速度PID控制器
 * 变量说明      roll_vel_pid     横滚速度PID控制器
 * 变量说明      yaw_vel_pid      偏航速度PID控制器
 * 备注信息     三环PID控制：外环角度->中环角速度->内环角加速度
-------------------------------------------------------------------------------------------------------------------*/
// 角加速度结构体（内环）
extern pid_param_t pitch_acc_pid;
extern pid_param_t roll_acc_pid;
extern pid_param_t yaw_acc_pid;

// 角速度结构体（中环）
extern pid_param_t pitch_angle_pid;
extern pid_param_t roll_angle_pid;
extern pid_param_t yaw_angle_pid;

// 速度结构体（外环）
extern pid_param_t pitch_vel_pid;
extern pid_param_t roll_vel_pid;
extern pid_param_t yaw_vel_pid;

extern pid_param_t pitch_pos_pid;
extern pid_param_t roll_pos_pid;

extern pid_param_t height_pos_pid;   // 高度位置环（外环）
extern pid_param_t height_vel_pid;    // 高度速度环（内环）

// 姿态机械安装零偏：菜单和Flash参数模块通过这两个运行时变量统一读写。
extern float pitch_zero;
extern float roll_zero;

// 位置环目标值
extern float roll_pos_target_cm;
extern float pitch_pos_target_cm;

/*-------------------------------------------------------------------------------------------------------------------
 * 函数接口声明
-------------------------------------------------------------------------------------------------------------------*/

/* 数值限制函数 */
float constrain_float(float amt, float low, float high);

/* 积分权重计算函数 */
float calc_I_weight(pid_param_t *pid, float error);

/* 通用PID结构体初始化 */
void PID_Structure_Init(pid_param_t* pid);

/* 带参数的PID初始化 */
void PID_Init(pid_param_t* pid, float kp, float ki, float kd,
              float imax, float outmax);

/* 各轴各环PID初始化函数 */
void pid_pitch_pos_init(pid_param_t* pid);
void pid_pitch_vel_init(pid_param_t* pid);
void pid_pitch_angle_init(pid_param_t* pid);
void pid_pitch_gyro_init(pid_param_t* pid);
void pid_yaw_vel_init(pid_param_t* pid);
void pid_yaw_angle_init(pid_param_t* pid);
void pid_yaw_gyro_init(pid_param_t* pid);
void pid_roll_pos_init(pid_param_t* pid);
void pid_roll_vel_init(pid_param_t* pid);
void pid_roll_angle_init(pid_param_t* pid);
void pid_roll_gyro_init(pid_param_t* pid);
void pid_height_pos_init(pid_param_t* pid);
void pid_height_vel_init(pid_param_t* pid);

/* 完整轴初始化函数 */
void pid_pitch_init(void);
void pid_yaw_init(void);
void pid_roll_init(void);
void pid_height_init(void);

/* 时间戳更新函数 */
uint8_t PID_UpdateTimeStamp(pid_param_t* pid);

/* 输出清零函数 */
void PID_CleanOut(pid_param_t* pid);

/* 积分项清零函数 */
void PID_CleanIntegral(pid_param_t* pid);

/* 误差计算函数 */
float get_error(float target, float data);

/* 各轴PID计算主函数 */
float pitch_pid(float position_target_y_cm, float speed_target_vy_cms,
                uint8_t position_enable, uint8_t position_deadzone_suppress);
float roll_pid(float position_target_x_cm, float speed_target_vx_cms,
               uint8_t position_enable, uint8_t position_deadzone_suppress);
void horizontal_control_set_attitude_only(uint8_t enable);
uint8_t horizontal_control_attitude_only_enabled(void);
void horizontal_velocity_control_reset(void);
void horizontal_velocity_get_debug(horizontal_velocity_debug_t *debug);
void horizontal_velocity_predict_expected_accel(float pitch_deg,
                                                float roll_deg,
                                                float *acc_x_cms2,
                                                float *acc_y_cms2);
void attitude_auto_trim_reset(void);
uint8_t attitude_auto_trim_start(void);
void attitude_auto_trim_step_1ms(uint8_t height_active,
                                uint8_t follow_active,
                                uint8_t manual_xy_active,
                                uint8_t rc_direction_input_active,
                                uint8_t landing_active,
                                float yaw_rate_dps);
void attitude_auto_trim_observe_10ms(void);
uint8_t attitude_auto_trim_zero_speed_hold_active(void);
uint8_t attitude_auto_trim_take_reanchor_request(void);
void attitude_auto_trim_get_debug(attitude_auto_trim_debug_t *debug);
float yaw_pid(float target);
float height_pid(float target);
float height_pid_with_velocity_ff(float target, float target_velocity_cms);

/* PID算法核心函数 */
float PidLocCtrl_dt(pid_param_t *pid, float error, float dt);  // 位置式PID
float PidIncCtrl_dt(pid_param_t* pid, float error,float dt);  // 增量式PID

float PidInterCtrl(pid_param_t *pid, float error, float dt);
void PidReset(pid_param_t *pid);
void PID_SetParam(pid_param_t *pid, float kp, float ki, float kd);
#endif /* PID_H */
