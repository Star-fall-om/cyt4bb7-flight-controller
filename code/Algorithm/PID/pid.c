
#include "Algorithm/PID/pid.h"
#include "Config/tuning_param_config.h"
#include "IMU/IMU963.h"
#include "System/usr_system.h"
#include "LC302/LC302.h"
#include "Utility/VOFA/vofa_tune.h"

#define D_FILTER_ALPHA  0.3f
#define VOFA_RATE_CMD_EPS 0.001f

// ========== 线缆冲击高度保护（仅作用于正常定高模式） ==========
#define HEIGHT_IMPACT_DROP_MM              (-50.0f)  // 冲击ACTIVE时立即降低的目标高度
#define HEIGHT_IMPACT_RECOVERY_RATE_MM_S   (50.0f)   // 进入RECOVERY后目标高度恢复速度
#define HEIGHT_IMPACT_POSITION_DT_S        (0.02f)   // 高度位置环实际更新周期：10次×2ms
#define HEIGHT_IMPACT_MIN_TARGET_MM        (0.0f)   // 防止异常目标高度减150mm后为负值

// ========== 水平速度环（光流速度环，10ms周期）配置参数 ==========

#ifndef HVEL_D_ENABLE
#define HVEL_D_ENABLE                (1u)  // 水平速度环D项使能开关（1=启用微分项）
#endif
#ifndef HVEL_FF_ENABLE
#define HVEL_FF_ENABLE               (0u)  // 加速度型速度环默认关闭目标变化前馈，避免重复制动
#endif

// --- 基础PID参数 ---
#define HVEL_DT_S                    (0.01f)   // 控制周期：10ms = 0.01s（光流速度环更新频率100Hz）
#define HPOS_DT_S                    (0.02f)   // 水平位置环周期：20ms = 0.02s（50Hz）
#define HPOS_UPDATE_DIVIDER          (2u)      // 位置环每两个速度环周期更新一次，保持严格2:1串级关系
#define HVEL_PITCH_KP                TUNE_DEFAULT_PITCH_VEL_KP
#define HVEL_ROLL_KP                 TUNE_DEFAULT_ROLL_VEL_KP
#define HVEL_KI                      TUNE_DEFAULT_PITCH_VEL_KI
#define HVEL_KD                      TUNE_DEFAULT_PITCH_VEL_KD

// --- 输出限幅 ---
#define HVEL_IMAX_ACCEL_CMS2         (50.0f)   // 加速度型速度环积分项限幅：±50cm/s²
#define HVEL_OUTMAX_DEG              (30.0f)   // PID总输出限幅：±30度（限制最大姿态角指令）

// 速度环输出先限制为水平加速度，再转换为目标倾角。
#define HVEL_ACCEL_CMD_LIMIT_CMS2    (560.0f) // 正常加速时的期望水平加速度限幅，约对应29.7度
#define HVEL_BRAKE_CMD_LIMIT_CMS2    (560.0f) // 松杆/反向时的期望水平减速度限幅，约对应29.7度
#define HVEL_BRAKE_FULL_SPEED_CMS    (50.0f)  // 高于该速度时允许完整制动
#define HVEL_BRAKE_ZERO_SPEED_CMS    (20.0f)  // 低于该速度时逐渐释放制动
#define HVEL_ZERO_BRAKE_SCALE_ENABLE (0u)     // 暂时关闭新增零速制动衰减，保留速度误差闭环制动

#define HVEL_CMD_ACTIVE_CMS          (3.0f)
#define HVEL_CMD_ZERO_CMS            (1.0f)
#define HVEL_I_REF_ZERO_CMS          (1.0f)
#define HVEL_I_ERROR_LIMIT_CMS       (8.0f)
#define HVEL_I_STABLE_CYCLES         (50u)

// --- 位置环与视觉速度前馈冲突保护 ---
#define HVEL_POS_FF_CONFLICT_ENABLE      (1u)   // 启用冲突限幅，反向前馈不超过位置修正允许的幅值
#define HVEL_POS_FF_CONFLICT_MIN_POS_CMS (5.0f)
#define HVEL_POS_FF_CONFLICT_DEADZONE_MIN_POS_CMS (1.0f) // 死区影子位置速度门槛，兼顾保护与中心抗噪
#define HVEL_POS_FF_CONFLICT_MIN_FF_CMS  (10.0f)
#define HVEL_POS_FF_CONFLICT_RATIO       (1.0f)  // 重新启用时，反向前馈最多限制到位置项等幅
#define HVEL_POS_FF_DEADZONE_SAME_DIR_ENABLE (0u) // 暂时关闭死区内同向前馈衰减，保留代码便于后续恢复//////////////////////////////////////////
#define HVEL_POS_FF_DEADZONE_SAME_DIR_SCALE (0.5f) // 死区内位置与前馈同向时，前馈减半以降低冲出风险

// --- 加速度限制 ---
#define HVEL_ACCEL_LIMIT_CMS2        (10000.0f) // 目标参考速度快速跟随摇杆目标
#define HVEL_BRAKE_LIMIT_CMS2        (10000.0f) // 目标参考速度快速回零，实际制动由速度误差闭环限幅

// --- D项滤波与限制 ---
#define HVEL_D_CUTOFF_HZ             (4.0f)    // D项低通滤波截止频率：4Hz（过滤高频噪声）
#define HVEL_D_RAW_LIMIT_CMS2        (600.0f)  // D项原始输入限幅：600cm/s²（限制微分项对异常值的响应）
#define HVEL_D_OUTPUT_LIMIT_DEG      (1.2f)    // D项输出限幅：±1.2度（防止微分项过大导致震荡）
#define HVEL_D_SLEW_RATE_DEG_S       (20.0f)   // D项输出变化率限制：20度/秒（平滑微分输出，抑制突变）

// --- 前馈FF参数 ---
#define HVEL_FF_GAIN                 (0.025f)  // 目标加速度前馈增益，先以约1度姿态补偿提高速度响应
#define HVEL_FF_LIMIT_DEG            (1.5f)    // 前馈输出限幅：±1.5度（防止前馈过大）
#define HVEL_FF_RISE_STEP_DEG        (0.3f)    // 前馈上升步进：0.3度/步（前馈增加时的渐变速度）
#define HVEL_FF_RELEASE_STEP_DEG     (0.6f)    // 前馈释放步进：0.6度/步（前馈减小时的渐变速度，允许更快释放）

// --- 线缆冲击纯光流保护 ---
#define HVEL_IMPACT_BRAKE_HOLD_CYCLES          (100u)  // 100个10ms周期：冲击结束后保持零速制动1秒
#define HVEL_IMPACT_FF_RECOVERY_CYCLES          (30u)  // 30个10ms周期：车速前馈300ms恢复
#define HVEL_IMPACT_POSITION_RELEASE_CMS2  (100.0f)   // RECOVERY位置速度修正恢复斜率
#define HVEL_IMPACT_POSITION_READY_CMS       (0.01f)  // 位置修正追上正常值的判定误差
#define HVEL_IMPACT_HOLD_OUTMAX_DEG           (12.0f)  // ACTIVE及零速保持阶段的最大姿态指令

// --- 物理常数 ---
#define HVEL_GRAVITY_CMS2            (981.0f)  // 重力加速度：981cm/s²（用于姿态角到加速度的转换）
#define HVEL_RAD_TO_DEG              (57.2957795f)  // 弧度到角度转换系数：180/π

// --- 姿态动态零点 AutoTrim 参数 ---
#define ATT_TRIM_LEARN_REQUIRED_MS       (3000u)  // 双轴光流有效的累计学习时间
#define ATT_TRIM_LEARN_DT_S              (0.01f)  // 候选值与速度环同频更新：10ms
#define ATT_TRIM_LPF_TAU_S               (0.8f)   // 只提取速度环输出中的低频平衡分量
#define ATT_TRIM_LIMIT_DEG               (1.5f)   // 单轴动态补偿最大允许值
#define ATT_TRIM_TRANSFER_RATE_DEG_S     (0.20f)  // 补偿转入角度零点的最大速度
#define ATT_TRIM_TRANSFER_STEP_DEG       (ATT_TRIM_TRANSFER_RATE_DEG_S * 0.001f)
#define ATT_TRIM_DONE_EPS_DEG            (0.00001f) // 小于1ms转移步长，最终吸附也不会产生角度跳变
#define ATT_TRIM_YAW_RATE_LIMIT_DPS      (10.0f)  // 偏航过快时暂停学习，避免旋转光流污染
#define ATT_TRIM_RC_STABLE_REQUIRED_MS   (300u)   // 摇杆回中后等待稳定，避免残余运动进入学习
#define ATT_TRIM_SPEED_LIMIT_CMS         (5.0f)   // 动态零点学习要求融合速度接近零
#define ATT_TRIM_FLOW_ERROR_LIMIT_CMS    (20.0f)  // 光流与IMU预测一致性上限

#define ATT_TRIM_GATE_PITCH_FLOW         (1u << 0)
#define ATT_TRIM_GATE_ROLL_FLOW          (1u << 1)
#define ATT_TRIM_GATE_MANUAL_CLEAR       (1u << 2)
#define ATT_TRIM_GATE_YAW_QUIET          (1u << 3)
#define ATT_TRIM_GATE_HEIGHT_ACTIVE      (1u << 4)
#define ATT_TRIM_GATE_NOT_LANDING        (1u << 5)
#define ATT_TRIM_GATE_FLOW_QUALITY       (1u << 6)
#define ATT_TRIM_GATE_SPEED_STABLE       (1u << 7)
#define ATT_TRIM_GATE_RC_STABLE          (1u << 8)
#define ATT_TRIM_GATE_ALL                (0x01FFu)

// --- 滤波器类型 ---
#define PID_D_FILTER_LIMITED_PT2     (1u)      // 使用受限二阶低通滤波器处理D项

static uint8_t VofaTune_GetNonzeroRateCommand(uint8_t target_id, float *cmd)
{
    float local_cmd = 0.0f;

    if (!VofaTune_GetActiveCommand(target_id, &local_cmd)) {
        return 0u;
    }
    if (fabsf(local_cmd) <= VOFA_RATE_CMD_EPS) {
        return 0u;
    }
    if (cmd != 0) {
        *cmd = local_cmd;
    }
    return 1u;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 全局PID参数实例
 * 变量说明      pitch_acc_pid    俯仰角加速度PID控制器（内环）
 * 变量说明      roll_acc_pid     横滚角加速度PID控制器（内环）
 * 变量说明      yaw_acc_pid      偏航角加速度PID控制器（内环）
 * 变量说明      pitch_angle_pid  俯仰角PID控制器（中环）
 * 变量说明      roll_angle_pid   横滚角PID控制器（中环）
 * 变量说明      yaw_angle_pid    偏航角PID控制器（中环）
 * 变量说明      pitch_vel_pid    俯仰速度PID控制器（外环）
 * 变量说明      roll_vel_pid     横滚速度PID控制器（外环）
 * 变量说明      yaw_vel_pid      偏航速度PID控制器（外环）
 * 备注信息     采用三环级联控制结构，提高姿态控制精度和稳定性
-------------------------------------------------------------------------------------------------------------------*/
// 角加速度结构体（内环-最快响应）
pid_param_t pitch_acc_pid;
pid_param_t roll_acc_pid;
pid_param_t yaw_acc_pid;

// 角速度结构体（中环）
pid_param_t pitch_angle_pid;
pid_param_t roll_angle_pid;
pid_param_t yaw_angle_pid;

// 速度结构体（外环）
pid_param_t pitch_vel_pid;
pid_param_t roll_vel_pid;

// 位置结构体（外环-最慢响应）
pid_param_t pitch_pos_pid;
pid_param_t roll_pos_pid;

pid_param_t height_pos_pid;   // 高度位置环（外环）
pid_param_t height_vel_pid;    // 高度速度环（内环）

// 标称零点只表示当前机械安装基准；AutoTrim 只在本次飞行中叠加 RAM 补偿。
float pitch_zero = TUNE_DEFAULT_PITCH_ZERO_DEG;
float roll_zero = TUNE_DEFAULT_ROLL_ZERO_DEG;

typedef struct {
    float target_cmd_cms;
    float target_ref_cms;
    float target_accel_cms2;
    float feed_forward_deg;
    float last_target_cmd_cms;
    float impact_position_release_cms;
    uint16_t impact_brake_hold_cycles;
    uint16_t impact_feedforward_recovery_cycles;
    uint32_t last_full_reset_count;
    uint32_t i_reset_count;
    uint16_t i_stable_count;
    uint8_t reset_count_inited;
    uint8_t flow_was_valid;
    uint8_t command_was_active;
    uint8_t i_enabled;
    uint8_t last_impact_guard_stage;
} horizontal_velocity_axis_state_t;

static horizontal_velocity_axis_state_t pitch_hvel_state;
static horizontal_velocity_axis_state_t roll_hvel_state;
static horizontal_velocity_debug_t horizontal_velocity_debug;
static volatile uint8_t horizontal_attitude_only_mode; // 启动模式：1=仅姿态内环，0=正常水平级联
static attitude_auto_trim_debug_t attitude_auto_trim_debug;
static uint8_t attitude_auto_trim_attempted;
static uint8_t attitude_auto_trim_reanchor_pending;
static uint8_t attitude_auto_trim_height_active;
static uint8_t attitude_auto_trim_manual_xy_active;
static uint8_t attitude_auto_trim_landing_active;
static uint8_t attitude_auto_trim_yaw_quiet;
static uint8_t attitude_auto_trim_rc_input_active;
static uint16_t attitude_auto_trim_rc_stable_ms;

/* 函数声明 */
float PidLocCtrl(pid_param_t *pid, float error,float dt);
float PidIncCtrl(pid_param_t *pid, float error, float dt);

/* 浮点数精度常量 */
const float numeric_limits_float_epsilon = 1.19209e-07f;

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     数值限制函数
 * 参数说明     amt              输入值
 * 参数说明     low              下限值
 * 参数说明     high             上限值
 * 返回参数     float            限制后的值
 * 使用示例     float val = constrain_float(raw, -100.0f, 100.0f);
 * 备注信息     用于防止PID输出饱和，确保系统稳定运行
-------------------------------------------------------------------------------------------------------------------*/
float constrain_float(float amt, float low, float high)
{
    if (amt < low) return low;
    else if (amt > high) return high;
    else return amt;
}

/**
 * @brief 重置单个水平速度轴的状态（俯仰或横滚）
 * @param state 轴状态结构体指针
 * @param pid PID控制器结构体指针
 * @note 清空目标速度、参考速度、加速度、前馈输出及光流有效标志
 */
static void horizontal_velocity_axis_reset(horizontal_velocity_axis_state_t *state,
                                           pid_param_t *pid)
{
    state->target_cmd_cms = 0.0f;        // 清空目标指令速度
    state->target_ref_cms = 0.0f;        // 清空参考速度（经过加速度限制后的平滑速度）
    state->target_accel_cms2 = 0.0f;     // 清空目标加速度
    state->feed_forward_deg = 0.0f;      // 清空前馈角度输出
    state->last_target_cmd_cms = 0.0f;
    state->impact_position_release_cms = 0.0f;
    state->impact_brake_hold_cycles = 0u;
    state->impact_feedforward_recovery_cycles = 0u;
    state->last_full_reset_count = 0u;   // 清空上次全局重置计数
    state->i_reset_count = 0u;
    state->i_stable_count = 0u;
    state->reset_count_inited = 0u;      // 清空重置计数初始化标志
    state->flow_was_valid = 0u;          // 清空光流有效历史标志
    state->command_was_active = 0u;
    state->i_enabled = 0u;
    state->last_impact_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
    PidReset(pid);                       // 重置PID控制器内部状态
}

/**
 * @brief 重置整个水平速度控制器（俯仰+横滚双轴）
 * @note 清空两个轴的状态和调试信息，用于起飞、失控保护等场景
 */
void horizontal_velocity_control_reset(void)
{
    horizontal_velocity_debug_t cleared = {0};

    horizontal_velocity_axis_reset(&pitch_hvel_state, &pitch_vel_pid);  // 重置俯仰轴
    horizontal_velocity_axis_reset(&roll_hvel_state, &roll_vel_pid);    // 重置横滚轴

    taskENTER_CRITICAL();  // 进入临界区保护调试数据
    horizontal_velocity_debug = cleared;
    taskEXIT_CRITICAL();
}

/* 设置本次启动的水平控制模式；菜单启动前写入，飞行中不切换。 */
void horizontal_control_set_attitude_only(uint8_t enable)
{
    uint8_t new_mode = enable ? 1u : 0u;

    if (new_mode != horizontal_attitude_only_mode) {
        horizontal_attitude_only_mode = new_mode;
        horizontal_velocity_control_reset();
    }
}

/* 查询是否旁路水平位置环和水平速度环。 */
uint8_t horizontal_control_attitude_only_enabled(void)
{
    return horizontal_attitude_only_mode;
}

/**
 * @brief 获取水平速度控制器的调试数据
 * @param debug 输出参数，存储调试信息的结构体指针
 * @note 线程安全地读取调试数据，用于VOFA+等上位机监控
 */
void horizontal_velocity_get_debug(horizontal_velocity_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    taskENTER_CRITICAL();  // 进入临界区避免读取过程中数据被修改
    *debug = horizontal_velocity_debug;
    taskEXIT_CRITICAL();
}

void horizontal_velocity_predict_expected_accel(float pitch_deg,
                                                float roll_deg,
                                                float *acc_x_cms2,
                                                float *acc_y_cms2)
{
    horizontal_velocity_debug_t debug_snapshot;
    float pitch_zero_deg;
    float roll_zero_deg;
    float demand_x_cms;
    float demand_y_cms;
    float demand_norm_sq;
    float demand_inv_norm;
    float demand_unit_x;
    float demand_unit_y;
    float tilt_acc_x_cms2;
    float tilt_acc_y_cms2;
    float projected_acc_cms2;

    if ((acc_x_cms2 == NULL) || (acc_y_cms2 == NULL)) {
        return;
    }

    *acc_x_cms2 = 0.0f;
    *acc_y_cms2 = 0.0f;

    // 一次性快照速度环状态和动态零点，避免两轴更新之间读到不一致的数据。
    taskENTER_CRITICAL();
    debug_snapshot = horizontal_velocity_debug;
    pitch_zero_deg = pitch_zero + attitude_auto_trim_debug.pitch_applied_deg;
    roll_zero_deg = roll_zero + attitude_auto_trim_debug.roll_applied_deg;
    taskEXIT_CRITICAL();

    // 速度环坐标映射到估计器坐标：X轴取Roll反号，Y轴保持Pitch原符号。
    demand_x_cms = (-debug_snapshot.roll.target_ref_cms)
                 - (-debug_snapshot.roll.measured_cms)
                 + (-debug_snapshot.roll.target_accel_cms2) * HVEL_DT_S;
    demand_y_cms = debug_snapshot.pitch.target_ref_cms
                 - debug_snapshot.pitch.measured_cms
                 + debug_snapshot.pitch.target_accel_cms2 * HVEL_DT_S;

    // 实机正方向已确认：Roll正为左倾、Pitch正为后倾；小角度模型足够覆盖当前姿态限幅。
    tilt_acc_x_cms2 = -HVEL_GRAVITY_CMS2
                    * (roll_deg - roll_zero_deg) / HVEL_RAD_TO_DEG;
    tilt_acc_y_cms2 = HVEL_GRAVITY_CMS2
                    * (pitch_deg - pitch_zero_deg) / HVEL_RAD_TO_DEG;

    demand_norm_sq = demand_x_cms * demand_x_cms
                   + demand_y_cms * demand_y_cms;
    if (demand_norm_sq <= 1.0e-6f) {
        return;
    }

    demand_inv_norm = 1.0f / sqrtf(demand_norm_sq);
    demand_unit_x = demand_x_cms * demand_inv_norm;
    demand_unit_y = demand_y_cms * demand_inv_norm;
    projected_acc_cms2 = tilt_acc_x_cms2 * demand_unit_x
                       + tilt_acc_y_cms2 * demand_unit_y;

    // 只保留倾角在当前运动需求方向上的正投影，反向或侧向分量交给残差检测。
    if (projected_acc_cms2 > 0.0f) {
        *acc_x_cms2 = demand_unit_x * projected_acc_cms2;
        *acc_y_cms2 = demand_unit_y * projected_acc_cms2;
    }
}

static void attitude_auto_trim_sync_effective_zero(void)
{
    attitude_auto_trim_debug.pitch_effective_zero_deg =
        pitch_zero + attitude_auto_trim_debug.pitch_applied_deg;
    attitude_auto_trim_debug.roll_effective_zero_deg =
        roll_zero + attitude_auto_trim_debug.roll_applied_deg;
}

static float attitude_auto_trim_slew(float current_deg, float target_deg)
{
    float delta_deg = constrain_float(target_deg - current_deg,
                                      -ATT_TRIM_TRANSFER_STEP_DEG,
                                       ATT_TRIM_TRANSFER_STEP_DEG);
    return current_deg + delta_deg;
}

static void attitude_auto_trim_freeze_current(attitude_auto_trim_stop_reason_t reason)
{
    uint8_t was_learning =
        (attitude_auto_trim_debug.state == ATTITUDE_TRIM_LEARN) ? 1u : 0u;

    // 降落或退出定高时不再继续改变零点，保持当前已经应用的值以避免姿态跳变。
    attitude_auto_trim_debug.pitch_target_deg = attitude_auto_trim_debug.pitch_applied_deg;
    attitude_auto_trim_debug.roll_target_deg = attitude_auto_trim_debug.roll_applied_deg;
    attitude_auto_trim_debug.state = ATTITUDE_TRIM_FROZEN;
    attitude_auto_trim_debug.stop_reason = reason;
    if (was_learning) {
        attitude_auto_trim_reanchor_pending = 1u;
    }
    attitude_auto_trim_sync_effective_zero();
}

static void attitude_auto_trim_begin_transfer(attitude_auto_trim_stop_reason_t reason)
{
    // 候选值在此刻锁定；进入跟随后只执行慢速转移，不再从运动数据继续学习。
    attitude_auto_trim_debug.pitch_target_deg =
        constrain_float(attitude_auto_trim_debug.pitch_candidate_deg,
                        -ATT_TRIM_LIMIT_DEG, ATT_TRIM_LIMIT_DEG);
    attitude_auto_trim_debug.roll_target_deg =
        constrain_float(attitude_auto_trim_debug.roll_candidate_deg,
                        -ATT_TRIM_LIMIT_DEG, ATT_TRIM_LIMIT_DEG);
    attitude_auto_trim_debug.state = ATTITUDE_TRIM_TRANSFER;
    attitude_auto_trim_debug.stop_reason = reason;
    attitude_auto_trim_reanchor_pending = 1u;

    if ((fabsf(attitude_auto_trim_debug.pitch_target_deg -
               attitude_auto_trim_debug.pitch_applied_deg) <= ATT_TRIM_DONE_EPS_DEG) &&
        (fabsf(attitude_auto_trim_debug.roll_target_deg -
               attitude_auto_trim_debug.roll_applied_deg) <= ATT_TRIM_DONE_EPS_DEG)) {
        attitude_auto_trim_debug.state = ATTITUDE_TRIM_FROZEN;
    }
    attitude_auto_trim_sync_effective_zero();
}

void attitude_auto_trim_reset(void)
{
    attitude_auto_trim_debug_t cleared = {0};

    cleared.state = ATTITUDE_TRIM_IDLE;
    cleared.stop_reason = ATTITUDE_TRIM_STOP_LOCKED;
    cleared.pitch_effective_zero_deg = pitch_zero;
    cleared.roll_effective_zero_deg = roll_zero;

    attitude_auto_trim_debug = cleared;
    attitude_auto_trim_attempted = 0u;
    attitude_auto_trim_reanchor_pending = 0u;
    attitude_auto_trim_height_active = 0u;
    attitude_auto_trim_manual_xy_active = 0u;
    attitude_auto_trim_landing_active = 0u;
    attitude_auto_trim_yaw_quiet = 0u;
    attitude_auto_trim_rc_input_active = 0u;
    attitude_auto_trim_rc_stable_ms = 0u;
}

uint8_t attitude_auto_trim_start(void)
{
    if (attitude_auto_trim_attempted ||
        attitude_auto_trim_debug.state != ATTITUDE_TRIM_IDLE) {
        return 0u;
    }

    // 每次解锁后的第一次定高才允许学习，防止任务途中反复改变平衡零点。
    attitude_auto_trim_debug.state = ATTITUDE_TRIM_LEARN;
    attitude_auto_trim_debug.stop_reason = ATTITUDE_TRIM_STOP_NONE;
    attitude_auto_trim_debug.valid_learn_ms = 0u;
    attitude_auto_trim_debug.sample_gate_mask = 0u;
    attitude_auto_trim_debug.pitch_candidate_deg = 0.0f;
    attitude_auto_trim_debug.roll_candidate_deg = 0.0f;
    attitude_auto_trim_debug.pitch_target_deg = 0.0f;
    attitude_auto_trim_debug.roll_target_deg = 0.0f;
    attitude_auto_trim_debug.pitch_applied_deg = 0.0f;
    attitude_auto_trim_debug.roll_applied_deg = 0.0f;
    attitude_auto_trim_attempted = 1u;
    attitude_auto_trim_reanchor_pending = 0u;
    attitude_auto_trim_sync_effective_zero();
    return 1u;
}

void attitude_auto_trim_step_1ms(uint8_t height_active,
                                uint8_t follow_active,
                                uint8_t manual_xy_active,
                                uint8_t rc_direction_input_active,
                                uint8_t landing_active,
                                float yaw_rate_dps)
{
    attitude_auto_trim_height_active = height_active ? 1u : 0u;
    attitude_auto_trim_manual_xy_active = manual_xy_active ? 1u : 0u;
    attitude_auto_trim_landing_active = landing_active ? 1u : 0u;
    attitude_auto_trim_rc_input_active = rc_direction_input_active ? 1u : 0u;
    if (attitude_auto_trim_rc_input_active) {
        attitude_auto_trim_rc_stable_ms = 0u;
        if (attitude_auto_trim_debug.state == ATTITUDE_TRIM_LEARN ||
            attitude_auto_trim_debug.state == ATTITUDE_TRIM_TRANSFER) {
            attitude_auto_trim_freeze_current(ATTITUDE_TRIM_STOP_RC_INPUT);
        }
    } else if (attitude_auto_trim_rc_stable_ms < ATT_TRIM_RC_STABLE_REQUIRED_MS) {
        attitude_auto_trim_rc_stable_ms++;
    }
    attitude_auto_trim_yaw_quiet =
        (fabsf(yaw_rate_dps) <= ATT_TRIM_YAW_RATE_LIMIT_DPS) ? 1u : 0u;

    if (attitude_auto_trim_debug.state == ATTITUDE_TRIM_LEARN) {
        if (attitude_auto_trim_landing_active) {
            attitude_auto_trim_freeze_current(ATTITUDE_TRIM_STOP_LANDING_STARTED);
        } else if (!attitude_auto_trim_height_active) {
            attitude_auto_trim_freeze_current(ATTITUDE_TRIM_STOP_HEIGHT_EXIT);
        } else if (follow_active) {
            // 学习不足3秒也直接采用已有候选值，并立即把控制权交回正常跟随链路。
            attitude_auto_trim_begin_transfer(ATTITUDE_TRIM_STOP_FOLLOW_STARTED);
        }
    }

    if (attitude_auto_trim_debug.state == ATTITUDE_TRIM_TRANSFER) {
        if (attitude_auto_trim_landing_active) {
            attitude_auto_trim_freeze_current(ATTITUDE_TRIM_STOP_LANDING_STARTED);
        } else if (!attitude_auto_trim_height_active) {
            attitude_auto_trim_freeze_current(ATTITUDE_TRIM_STOP_HEIGHT_EXIT);
        } else {
            attitude_auto_trim_debug.pitch_applied_deg =
                attitude_auto_trim_slew(attitude_auto_trim_debug.pitch_applied_deg,
                                        attitude_auto_trim_debug.pitch_target_deg);
            attitude_auto_trim_debug.roll_applied_deg =
                attitude_auto_trim_slew(attitude_auto_trim_debug.roll_applied_deg,
                                        attitude_auto_trim_debug.roll_target_deg);

            if ((fabsf(attitude_auto_trim_debug.pitch_target_deg -
                       attitude_auto_trim_debug.pitch_applied_deg) <= ATT_TRIM_DONE_EPS_DEG) &&
                (fabsf(attitude_auto_trim_debug.roll_target_deg -
                       attitude_auto_trim_debug.roll_applied_deg) <= ATT_TRIM_DONE_EPS_DEG)) {
                attitude_auto_trim_debug.pitch_applied_deg =
                    attitude_auto_trim_debug.pitch_target_deg;
                attitude_auto_trim_debug.roll_applied_deg =
                    attitude_auto_trim_debug.roll_target_deg;
                attitude_auto_trim_debug.state = ATTITUDE_TRIM_FROZEN;
            }
        }
    }

    attitude_auto_trim_sync_effective_zero();
}

void attitude_auto_trim_observe_10ms(void)
{
    horizontal_velocity_debug_t hvel_debug;
    imu_horizontal_state_t horizontal_state;
    uint8_t gate_mask = 0u;
    float alpha;
    float pitch_sample_deg;
    float roll_sample_deg;

    horizontal_velocity_get_debug(&hvel_debug);
    imu_horizontal_get_state(&horizontal_state);

    if (hvel_debug.pitch.flow_valid) gate_mask |= ATT_TRIM_GATE_PITCH_FLOW;
    if (hvel_debug.roll.flow_valid) gate_mask |= ATT_TRIM_GATE_ROLL_FLOW;
    if (!attitude_auto_trim_manual_xy_active) gate_mask |= ATT_TRIM_GATE_MANUAL_CLEAR;
    if (attitude_auto_trim_yaw_quiet) gate_mask |= ATT_TRIM_GATE_YAW_QUIET;
    if (attitude_auto_trim_height_active) gate_mask |= ATT_TRIM_GATE_HEIGHT_ACTIVE;
    if (!attitude_auto_trim_landing_active) gate_mask |= ATT_TRIM_GATE_NOT_LANDING;
    if (!attitude_auto_trim_rc_input_active &&
        (attitude_auto_trim_rc_stable_ms >= ATT_TRIM_RC_STABLE_REQUIRED_MS)) {
        gate_mask |= ATT_TRIM_GATE_RC_STABLE;
    }
    if (horizontal_state.flow_trust_level_x >= 1u &&
        horizontal_state.flow_trust_level_y >= 1u &&
        horizontal_state.flow_consistency_error_x_cms <= ATT_TRIM_FLOW_ERROR_LIMIT_CMS &&
        horizontal_state.flow_consistency_error_y_cms <= ATT_TRIM_FLOW_ERROR_LIMIT_CMS) {
        gate_mask |= ATT_TRIM_GATE_FLOW_QUALITY;
    }
    if (horizontal_state.velocity_valid_x && horizontal_state.velocity_valid_y &&
        fabsf(horizontal_state.vx_cms) <= ATT_TRIM_SPEED_LIMIT_CMS &&
        fabsf(horizontal_state.vy_cms) <= ATT_TRIM_SPEED_LIMIT_CMS) {
        gate_mask |= ATT_TRIM_GATE_SPEED_STABLE;
    }
    attitude_auto_trim_debug.sample_gate_mask = gate_mask;
    attitude_auto_trim_debug.learn_block_mask =
        (uint16_t)(ATT_TRIM_GATE_ALL & (uint16_t)(~gate_mask));
    attitude_auto_trim_debug.rc_input_active = attitude_auto_trim_rc_input_active;
    attitude_auto_trim_debug.rc_stable =
        (attitude_auto_trim_rc_stable_ms >= ATT_TRIM_RC_STABLE_REQUIRED_MS) ? 1u : 0u;
    attitude_auto_trim_debug.learn_allowed =
        (gate_mask == ATT_TRIM_GATE_ALL) ? 1u : 0u;

    if (attitude_auto_trim_debug.state != ATTITUDE_TRIM_LEARN ||
        gate_mask != ATT_TRIM_GATE_ALL) {
        return;
    }

    // 零速度保持期间FF目标为零；低通总输出可同时覆盖P项稳态误差和已建立的I项。
    pitch_sample_deg = constrain_float(hvel_debug.pitch.out_total_deg,
                                       -ATT_TRIM_LIMIT_DEG, ATT_TRIM_LIMIT_DEG);
    roll_sample_deg = constrain_float(hvel_debug.roll.out_total_deg,
                                      -ATT_TRIM_LIMIT_DEG, ATT_TRIM_LIMIT_DEG);
    alpha = ATT_TRIM_LEARN_DT_S / (ATT_TRIM_LPF_TAU_S + ATT_TRIM_LEARN_DT_S);
    attitude_auto_trim_debug.pitch_candidate_deg +=
        alpha * (pitch_sample_deg - attitude_auto_trim_debug.pitch_candidate_deg);
    attitude_auto_trim_debug.roll_candidate_deg +=
        alpha * (roll_sample_deg - attitude_auto_trim_debug.roll_candidate_deg);

    if (attitude_auto_trim_debug.valid_learn_ms < ATT_TRIM_LEARN_REQUIRED_MS) {
        attitude_auto_trim_debug.valid_learn_ms += 10u;
    }
    if (attitude_auto_trim_debug.valid_learn_ms >= ATT_TRIM_LEARN_REQUIRED_MS) {
        attitude_auto_trim_begin_transfer(ATTITUDE_TRIM_STOP_TIME_COMPLETE);
    }
}

uint8_t attitude_auto_trim_zero_speed_hold_active(void)
{
    return (attitude_auto_trim_debug.state == ATTITUDE_TRIM_LEARN &&
            attitude_auto_trim_height_active &&
            !attitude_auto_trim_landing_active &&
            !attitude_auto_trim_manual_xy_active &&
            !attitude_auto_trim_rc_input_active &&
            (attitude_auto_trim_rc_stable_ms >= ATT_TRIM_RC_STABLE_REQUIRED_MS)) ? 1u : 0u;
}

uint8_t attitude_auto_trim_take_reanchor_request(void)
{
    uint8_t requested = attitude_auto_trim_reanchor_pending;
    attitude_auto_trim_reanchor_pending = 0u;
    return requested;
}

void attitude_auto_trim_get_debug(attitude_auto_trim_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *debug = attitude_auto_trim_debug;
    taskEXIT_CRITICAL();
}

/**
 * @brief 应用水平速度环PID参数到控制器
 * @param pid PID控制器结构体指针
 * @param kp 比例系数（俯仰/横滚通道可能不同）
 * @note 配置PID三项系数、限幅、微分模式、滤波器等完整参数
 */
static void horizontal_velocity_apply_pid_constants(pid_param_t *pid, float kp)
{
    pid->kp = kp;                                    // 比例系数（俯仰/横滚可能不同）
    pid->ki = HVEL_KI;                               // 积分系数（当前为0）
    pid->kd = HVEL_D_ENABLE ? HVEL_KD : 0.0f;        // 微分系数（受使能开关控制）
    pid->imax = HVEL_IMAX_ACCEL_CMS2;                // 加速度积分项限幅
    pid->outmax = HVEL_ACCEL_CMD_LIMIT_CMS2;         // 期望水平加速度限幅
    pid->I_separate_thresh = 1e9f;                   // 积分分离阈值（1e9表示不分离）
    pid->d_mode = 1u;                                // 微分模式：1=对测量值微分（避免目标突变）
    pid->d_eflag = 1u;                               // 微分误差标志（启用）
    pid->d_filter_mode = PID_D_FILTER_LIMITED_PT2;   // 微分滤波器：受限二阶低通
    pid->d_filter_cutoff_hz = HVEL_D_CUTOFF_HZ;      // 微分滤波截止频率4Hz
    pid->d_raw_limit = HVEL_D_RAW_LIMIT_CMS2;        // 微分原始值限幅600cm/s²
    pid->d_output_limit = 0.0f;
    pid->d_output_slew_rate = 0.0f;
}

/**
 * @brief 平滑渐变前馈输出（避免前馈角度突变）
 * @param current_deg 当前前馈角度
 * @param target_deg 目标前馈角度
 * @return 经过步进限制后的新前馈角度
 * @note 使用不对称步进策略：增加时0.3度/步，释放时0.6度/步（允许更快减速）
 */
static float horizontal_velocity_slew_feed_forward(float current_deg, float target_deg)
{
    float step_deg = HVEL_FF_RISE_STEP_DEG;  // 默认使用上升步进0.3度
    float delta_deg;

    // 判断是否需要释放前馈（符号相反或目标值更小）
    if ((current_deg * target_deg < 0.0f) ||       // 符号相反（方向改变）
        (fabsf(target_deg) < fabsf(current_deg))) { // 目标值更小（减速/释放）
        step_deg = HVEL_FF_RELEASE_STEP_DEG;        // 使用释放步进0.6度（更快）
    }

    // 限制单步变化量
    delta_deg = constrain_float(target_deg - current_deg, -step_deg, step_deg);
    return current_deg + delta_deg;
}

static float horizontal_velocity_accel_to_angle(float accel_cms2)
{
    return atanf(accel_cms2 / HVEL_GRAVITY_CMS2) * HVEL_RAD_TO_DEG;
}

static float horizontal_velocity_zero_brake_scale(float target_ref_cms,
                                                  float measured_cms)
{
    float speed_cms;

    // 只有松杆零速目标才执行零速附近的制动衰减；反向目标仍需完整穿越零速。
    if (fabsf(target_ref_cms) >= HVEL_CMD_ZERO_CMS) {
        return 1.0f;
    }

    speed_cms = fabsf(measured_cms);
    if (speed_cms >= HVEL_BRAKE_FULL_SPEED_CMS) {
        return 1.0f;
    }
    if (speed_cms <= HVEL_BRAKE_ZERO_SPEED_CMS) {
        return 0.0f;
    }

    return (speed_cms - HVEL_BRAKE_ZERO_SPEED_CMS) /
           (HVEL_BRAKE_FULL_SPEED_CMS - HVEL_BRAKE_ZERO_SPEED_CMS);
}

/**
 * @brief 限制与位置环方向相反的视觉速度前馈
 * @param position_cmd_cms 位置环输出的速度目标
 * @param feedforward_cmd_cms 视觉小车速度前馈目标
 * @param min_position_cmd_cms 位置速度触发门槛，死区内使用更低门槛
 * @return 限幅后的视觉速度前馈目标
 * @note 仅在两个分量都足够大且方向相反时生效，避免中心附近噪声误触发。
 */
static float horizontal_velocity_limit_conflicting_feedforward(
    float position_cmd_cms, float feedforward_cmd_cms,
    float min_position_cmd_cms)
{
    float feedforward_limit_cms;

    if (!HVEL_POS_FF_CONFLICT_ENABLE) {
        return feedforward_cmd_cms;
    }

    if ((fabsf(position_cmd_cms) < min_position_cmd_cms) ||
        (fabsf(feedforward_cmd_cms) < HVEL_POS_FF_CONFLICT_MIN_FF_CMS) ||
        (position_cmd_cms * feedforward_cmd_cms >= 0.0f)) {
        return feedforward_cmd_cms;
    }

    feedforward_limit_cms = fabsf(position_cmd_cms) *
                            HVEL_POS_FF_CONFLICT_RATIO;
    if (fabsf(feedforward_cmd_cms) > feedforward_limit_cms) {
        feedforward_cmd_cms = (feedforward_cmd_cms > 0.0f) ?
                              feedforward_limit_cms : -feedforward_limit_cms;
    }

    return feedforward_cmd_cms;
}

/*
 * 仅限制位置环与车速前馈同向时的位置追赶分量。
 * 反向时保留完整位置修正，避免飞机越过小车后无法及时拉回。
 */
/*
 * 死区内不叠加位置修正，但保留影子位置速度判断飞机位于小车的哪一侧。
 * 当影子位置速度与车速前馈同向且均达到抗噪门槛时，将前馈衰减到固定比例。
 */
static float horizontal_velocity_decay_deadzone_same_direction_feedforward(
    float position_cmd_cms, float feedforward_cmd_cms,
    uint8_t position_deadzone_suppress)
{
    if ((!HVEL_POS_FF_DEADZONE_SAME_DIR_ENABLE) ||
        (!position_deadzone_suppress) ||
        (fabsf(position_cmd_cms) < HVEL_POS_FF_CONFLICT_DEADZONE_MIN_POS_CMS) ||
        (fabsf(feedforward_cmd_cms) < HVEL_POS_FF_CONFLICT_MIN_FF_CMS) ||
        ((position_cmd_cms * feedforward_cmd_cms) <= 0.0f)) {
        return feedforward_cmd_cms;
    }

    return feedforward_cmd_cms * HVEL_POS_FF_DEADZONE_SAME_DIR_SCALE;
}

/* 同向追赶时只限制位置分量；该规则与上面的死区前馈衰减相互独立。 */
static float horizontal_velocity_limit_same_direction_catchup(
    float position_cmd_cms, float feedforward_cmd_cms)
{
    if ((position_cmd_cms * feedforward_cmd_cms) <= 0.0f) {
        return position_cmd_cms;
    }

    return constrain_float(position_cmd_cms,
                           -TUNE_DEFAULT_HVEL_FOLLOW_CATCHUP_MAX_CMS,
                            TUNE_DEFAULT_HVEL_FOLLOW_CATCHUP_MAX_CMS);
}

static uint8_t horizontal_impact_brake_hold_update(
    horizontal_velocity_axis_state_t *state,
    uint8_t guard_stage)
{
    if (guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) {
        state->impact_brake_hold_cycles = 0u;
        state->impact_feedforward_recovery_cycles = 0u;
        state->impact_position_release_cms = 0.0f;
        return 1u;
    }

    if ((guard_stage == HORIZONTAL_IMPACT_GUARD_RECOVERY) &&
        (state->impact_brake_hold_cycles < HVEL_IMPACT_BRAKE_HOLD_CYCLES)) {
        state->impact_brake_hold_cycles++;
        state->impact_feedforward_recovery_cycles = 0u;
        state->impact_position_release_cms = 0.0f;
        return 1u;
    }

    if (guard_stage == HORIZONTAL_IMPACT_GUARD_NORMAL) {
        state->impact_brake_hold_cycles = HVEL_IMPACT_BRAKE_HOLD_CYCLES;
        state->impact_feedforward_recovery_cycles =
            HVEL_IMPACT_FF_RECOVERY_CYCLES;
    }

    return 0u;
}

static float horizontal_impact_position_command(
    horizontal_velocity_axis_state_t *state,
    float normal_position_cmd_cms,
    uint8_t guard_stage,
    uint8_t brake_hold_active)
{
    float release_step_cms;
    float release_delta_cms;

    if ((guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) ||
        brake_hold_active) {
        state->impact_position_release_cms = 0.0f;
    } else if (guard_stage == HORIZONTAL_IMPACT_GUARD_RECOVERY) {
        release_step_cms = HVEL_IMPACT_POSITION_RELEASE_CMS2 * HVEL_DT_S;
        release_delta_cms = normal_position_cmd_cms -
                            state->impact_position_release_cms;
        state->impact_position_release_cms += constrain_float(
            release_delta_cms, -release_step_cms, release_step_cms);
    } else {
        state->impact_position_release_cms = normal_position_cmd_cms;
    }

    return state->impact_position_release_cms;
}

static float horizontal_impact_feedforward_scale(
    horizontal_velocity_axis_state_t *state,
    uint8_t guard_stage,
    uint8_t brake_hold_active)
{
    if ((guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) ||
        brake_hold_active) {
        state->impact_feedforward_recovery_cycles = 0u;
        return 0.0f;
    }

    if (guard_stage == HORIZONTAL_IMPACT_GUARD_RECOVERY) {
        if (state->impact_feedforward_recovery_cycles <
            HVEL_IMPACT_FF_RECOVERY_CYCLES) {
            state->impact_feedforward_recovery_cycles++;
        }
        return (float)state->impact_feedforward_recovery_cycles /
               (float)HVEL_IMPACT_FF_RECOVERY_CYCLES;
    }

    state->impact_feedforward_recovery_cycles =
        HVEL_IMPACT_FF_RECOVERY_CYCLES;
    return 1.0f;
}

static float horizontal_impact_output_limit_deg(
    const horizontal_velocity_axis_state_t *state,
    uint8_t guard_stage)
{
    float recovery_ratio;

    if (guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) {
        return HVEL_IMPACT_HOLD_OUTMAX_DEG;
    }

    if (guard_stage == HORIZONTAL_IMPACT_GUARD_RECOVERY) {
        // 零速保持结束前恢复计数始终为0，因此全程维持12度限幅。
        recovery_ratio = (float)state->impact_feedforward_recovery_cycles /
                         (float)HVEL_IMPACT_FF_RECOVERY_CYCLES;
        recovery_ratio = constrain_float(recovery_ratio, 0.0f, 1.0f);
        return HVEL_IMPACT_HOLD_OUTMAX_DEG +
               (HVEL_OUTMAX_DEG - HVEL_IMPACT_HOLD_OUTMAX_DEG) *
               recovery_ratio;
    }

    return HVEL_OUTMAX_DEG;
}

/**
 * @brief 更新单个水平速度轴的控制（核心控制逻辑）
 * @param state 轴状态结构体指针
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param target_cmd_cms 目标指令速度（cm/s）
 * @param measured_cms 光流测量速度（cm/s）
 * @param flow_valid 光流数据有效标志
 * @param full_reset_count 全局重置计数器
 * @param debug 调试信息输出结构体
 * @note 完整流程：加速度限制→前馈计算→PID控制→调试数据记录
 */
static void horizontal_velocity_update_axis(horizontal_velocity_axis_state_t *state,
                                            pid_param_t *pid,
                                            float kp,
                                            float target_cmd_cms,
                                            float measured_cms,
                                            uint8_t control_valid,
                                            float control_confidence,
                                            uint8_t control_source,
                                            uint32_t full_reset_count,
                                            uint8_t impact_guard_stage,
                                            horizontal_velocity_axis_debug_t *debug)
{
    float velocity_error_cms;
    float control_error_cms;
    float accel_limit_cms2;
    float delta_v_cms;
    float feed_forward_target_deg = 0.0f;
    float accel_output_cms2;
    float accel_p_cms2;
    float accel_i_cms2;
    float accel_d_cms2;
    float accel_ff_cms2;
    float angle_output_deg;
    float angle_limit_deg;
    float impact_accel_limit_cms2;
    float impact_output_limit_scale;
    float brake_scale = 1.0f;
    float output_limit_scale = 1.0f;
    uint8_t pid_measurement_valid;
    uint8_t braking = 0u;
    uint8_t accel_limited = 0u;
    uint8_t reset_integral = 0u;

    pid_measurement_valid = (uint8_t)(
        control_valid &&
        ((control_source == HORIZONTAL_CONTROL_SOURCE_KF_VALID) ||
         (control_source == HORIZONTAL_CONTROL_SOURCE_FLOW_IMPACT)));
    if (control_source == HORIZONTAL_CONTROL_SOURCE_FLOW_HOLD) {
        output_limit_scale = 0.70f;
    } else if (control_source == HORIZONTAL_CONTROL_SOURCE_FLOW_TIMEOUT) {
        output_limit_scale = 0.50f;
    }
    // 检测全局重置事件（如姿态失效、重新解锁等）
    if (!state->reset_count_inited ||
        state->last_full_reset_count != full_reset_count) {
        horizontal_velocity_axis_reset(state, pid);
        state->last_full_reset_count = full_reset_count;
        state->reset_count_inited = 1u;
    }

    /* Clear stale motion I once when entering zero-speed hold or reversing. */
    if ((state->command_was_active &&
         fabsf(target_cmd_cms) < HVEL_CMD_ZERO_CMS) ||
        (((state->last_target_cmd_cms * target_cmd_cms) < 0.0f) &&
         (fabsf(state->last_target_cmd_cms) > HVEL_CMD_ZERO_CMS) &&
         (fabsf(target_cmd_cms) > HVEL_CMD_ZERO_CMS))) {
        reset_integral = 1u;
    }
    if ((impact_guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) &&
        (state->last_impact_guard_stage != HORIZONTAL_IMPACT_GUARD_ACTIVE)) {
        reset_integral = 1u;
        // 同步清除冲击前由大位置P留下的平滑速度参考，立即进入零速制动。
        state->target_ref_cms = target_cmd_cms;
        state->target_accel_cms2 = 0.0f;
        state->feed_forward_deg = 0.0f;
    }
    state->last_impact_guard_stage = impact_guard_stage;

    // 先完成状态复位，再计算冲击限幅，避免复位周期沿用旧恢复进度。
    angle_limit_deg = horizontal_impact_output_limit_deg(
        state, impact_guard_stage);
    impact_accel_limit_cms2 = HVEL_GRAVITY_CMS2 *
        tanf(angle_limit_deg / HVEL_RAD_TO_DEG);
    impact_output_limit_scale = constrain_float(
        impact_accel_limit_cms2 / HVEL_ACCEL_CMD_LIMIT_CMS2,
        0.0f, 1.0f);
    if (impact_output_limit_scale < output_limit_scale) {
        output_limit_scale = impact_output_limit_scale;
    }

    if (reset_integral) {
        pid->integrator = 0.0f;
        pid->out_i = 0.0f;
        state->i_stable_count = 0u;
        state->i_enabled = 0u;
        state->i_reset_count++;
    }

    if (fabsf(target_cmd_cms) > HVEL_CMD_ACTIVE_CMS) {
        state->command_was_active = 1u;
    } else if (fabsf(target_cmd_cms) < HVEL_CMD_ZERO_CMS) {
        state->command_was_active = 0u;
    }
    state->last_target_cmd_cms = target_cmd_cms;

    // ========== 1. 加速度限制阶段：平滑目标速度避免突变 ==========
    state->target_cmd_cms = target_cmd_cms;
    velocity_error_cms = state->target_cmd_cms - state->target_ref_cms;  // 指令与当前参考的差值

    // 根据加速/制动状态选择不同的加速度限制
    accel_limit_cms2 = (state->target_ref_cms * velocity_error_cms < 0.0f)
                     ? HVEL_BRAKE_LIMIT_CMS2
                     : HVEL_ACCEL_LIMIT_CMS2;

    // 限制单次速度变化量 = 加速度限制 × 控制周期
    delta_v_cms = constrain_float(velocity_error_cms,
                                  -accel_limit_cms2 * HVEL_DT_S,
                                   accel_limit_cms2 * HVEL_DT_S);
    state->target_ref_cms += delta_v_cms;               // 更新平滑后的参考速度
    state->target_accel_cms2 = delta_v_cms / HVEL_DT_S; // 记录实际加速度

    /* Freeze I through braking; re-enable after 0.5 s of stable zero-speed hold. */
    control_error_cms = state->target_ref_cms - measured_cms;
    if ((fabsf(state->target_cmd_cms) < HVEL_CMD_ZERO_CMS) &&
        (fabsf(state->target_ref_cms) < HVEL_I_REF_ZERO_CMS) &&
        (fabsf(control_error_cms) < HVEL_I_ERROR_LIMIT_CMS) &&
        pid_measurement_valid) {
        if (state->i_stable_count < HVEL_I_STABLE_CYCLES) {
            state->i_stable_count++;
        }
    } else {
        state->i_stable_count = 0u;
    }
    state->i_enabled = (state->i_stable_count >= HVEL_I_STABLE_CYCLES) ? 1u : 0u;

    // ========== 2. 目标变化前馈：加速度型速度环默认关闭 ==========
    if (HVEL_FF_ENABLE && pid_measurement_valid) {
        // 前馈原理：a = g * tan(θ)，反解得 θ = atan(a/g)
        feed_forward_target_deg = HVEL_FF_GAIN
                                * atanf(state->target_accel_cms2 / HVEL_GRAVITY_CMS2)
                                * HVEL_RAD_TO_DEG;
        // 限制前馈输出范围
        feed_forward_target_deg = constrain_float(feed_forward_target_deg,
                                                  -HVEL_FF_LIMIT_DEG,
                                                   HVEL_FF_LIMIT_DEG);
    }
    // 平滑渐变前馈输出（不对称步进）
    state->feed_forward_deg = horizontal_velocity_slew_feed_forward(
                                  state->feed_forward_deg,
                                  feed_forward_target_deg);

    // ========== 3. PID控制：基于速度误差计算期望水平加速度 ==========
    horizontal_velocity_apply_pid_constants(pid, kp);  // 应用加速度型PID参数
    pid->outmax = HVEL_ACCEL_CMD_LIMIT_CMS2 * output_limit_scale;
    pid->meas = measured_cms;                          // 设置测量值
    pid->d_input_valid = pid_measurement_valid;        // 仅新鲜融合反馈允许D项
    pid->i_input_valid = state->i_enabled;

    // 处理光流从无效到有效的转换（重新初始化测量值）
    if (pid_measurement_valid && !state->flow_was_valid) {
        pid->meas_inited = 0u;  // 强制重新初始化微分项
    }
    state->flow_was_valid = pid_measurement_valid;
    pid->feed_forward = 0.0f;                           // 加速度型速度环默认不叠加目标变化前馈

    // 执行PID计算：误差 = 参考速度 - 测量速度；PID输出单位为cm/s²。
    PidInterCtrl(pid, control_error_cms, HVEL_DT_S);

    accel_p_cms2 = pid->out_p;
    accel_i_cms2 = pid->out_i;
    accel_d_cms2 = pid->out_d;
    accel_ff_cms2 = pid->feed_forward;
    accel_output_cms2 = pid->out;

    // 实际速度与目标速度误差方向相反时，当前输出属于制动输出。
    braking = (uint8_t)((fabsf(measured_cms) > 0.001f) &&
                        ((control_error_cms * measured_cms) < 0.0f));
    accel_limit_cms2 = (braking ? HVEL_BRAKE_CMD_LIMIT_CMS2
                                : HVEL_ACCEL_CMD_LIMIT_CMS2) *
                       output_limit_scale;
    accel_limit_cms2 = fabsf(accel_limit_cms2);

    if (HVEL_ZERO_BRAKE_SCALE_ENABLE &&
        braking && (fabsf(state->target_ref_cms) < HVEL_CMD_ZERO_CMS)) {
        brake_scale = horizontal_velocity_zero_brake_scale(
            state->target_ref_cms, measured_cms);
        accel_output_cms2 *= brake_scale;
    }

    if (accel_output_cms2 > accel_limit_cms2) {
        accel_output_cms2 = accel_limit_cms2;
        accel_limited = 1u;
    } else if (accel_output_cms2 < -accel_limit_cms2) {
        accel_output_cms2 = -accel_limit_cms2;
        accel_limited = 1u;
    }
    angle_output_deg = constrain_float(
        horizontal_velocity_accel_to_angle(accel_output_cms2),
        -angle_limit_deg,
         angle_limit_deg);
    pid->out = angle_output_deg;

    // ========== 4. 调试数据记录 ==========
    debug->target_cmd_cms = state->target_cmd_cms;              // 原始指令速度
    debug->target_ref_cms = state->target_ref_cms;              // 平滑后参考速度
    debug->target_accel_cms2 = state->target_accel_cms2;        // 实际加速度
    debug->kp = pid->kp;                                        // 当前速度环实时Kp
    debug->measured_cms = measured_cms;                         // 光流测量速度
    debug->derivative_raw_cms2 = pid->last_derivative;          // 原始微分值
    debug->derivative_filtered_cms2 = pid->d_term_lpf2;         // 滤波后微分值
    debug->out_p_accel_cms2 = accel_p_cms2;
    debug->out_i_accel_cms2 = accel_i_cms2;
    debug->out_d_accel_cms2 = accel_d_cms2;
    debug->out_ff_accel_cms2 = accel_ff_cms2;
    debug->out_total_accel_cms2 = accel_output_cms2;
    debug->brake_scale = brake_scale;
    debug->accel_limited = accel_limited;
    // 保留原有遥测字段：这些字段继续输出角度等效值。
    debug->out_p_deg = horizontal_velocity_accel_to_angle(accel_p_cms2);
    debug->out_i_deg = horizontal_velocity_accel_to_angle(accel_i_cms2);
    debug->out_d_deg = horizontal_velocity_accel_to_angle(accel_d_cms2);
    debug->out_ff_deg = horizontal_velocity_accel_to_angle(accel_ff_cms2);
    debug->out_total_deg = angle_output_deg;
    debug->i_reset_count = state->i_reset_count;
    debug->i_stable_count = state->i_stable_count;
    debug->flow_valid = control_valid;                          // 控制反馈可用标志
    debug->i_enabled = state->i_enabled;
    debug->control_confidence = control_confidence;
    debug->output_limit_scale = output_limit_scale;
    debug->control_source = control_source;
    debug->output_limited = accel_limited;
    debug->d_input_valid = pid_measurement_valid;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     变速积分权重计算函数
 * 参数说明     *pid             PID结构体指针
 * 参数说明     error            当前误差值
 * 返回参数     float            积分权重系数(0.0~1.0)
 * 使用示例     float weight = calc_I_weight(&pitch_pid, current_error);
 * 备注信息     实现积分分离和变速积分功能，减小大误差时的积分积累
 *              当误差较大时减小积分作用，防止超调和积分饱和
-------------------------------------------------------------------------------------------------------------------*/
float calc_I_weight(pid_param_t *pid, float error)
{
    float ae = fabsf(error);  // 误差绝对值

    if (ae <= pid->var_speed_I_B)
        return 1.0f;  // 小误差区：全积分
    else if (ae <= pid->var_speed_I_A + pid->var_speed_I_B)
        // 中误差区：线性减小积分权重
        return (pid->var_speed_I_A + pid->var_speed_I_B - ae) / pid->var_speed_I_A;
    else
        return 0.0f;  // 大误差区：零积分（积分分离）
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     通用PID结构体初始化
 * 参数说明     *pid             PID结构体指针
 * 返回参数     void
 * 使用示例     pid_param_t my_pid;
 *             PID_Structure_Init(&my_pid);
 * 备注信息     将所有PID参数清零，防止未初始化导致的控制异常
-------------------------------------------------------------------------------------------------------------------*/
void PID_Structure_Init(pid_param_t* pid)
{
    if (!pid) return;

    /* 时间相关 */
    pid->dt = 0.0f;
    pid->last_time = 0;
    pid->Get_SystemTick = GetSystemTimer;   // 你工程已有 GetSystemTimer()

    /* 输出/状态清零 */
    pid->out = 0.0f;
    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out_d = 0.0f;

    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->last_derivative = 0.0f;

    /* 滤波/微分状态 */
    pid->d_term_lpf = 0.0f;
    pid->d_term_lpf2 = 0.0f;
    pid->last_d_output = 0.0f;
    pid->d_filter_mode = 0u;
    pid->d_input_valid = 1u;
    pid->i_input_valid = 1u;
    pid->d_filter_cutoff_hz = 0.0f;
    pid->d_raw_limit = 0.0f;
    pid->d_output_limit = 0.0f;
    pid->d_output_slew_rate = 0.0f;

    /* 前馈 */
    pid->feed_forward = 0.0f;

    /* 死区 + 误差滤波默认关 */
    pid->DeadZone = 0.0f;
    pid->efilter = 0;
    pid->e_term_lpf = 0.0f;

    /* D项默认：对误差求导 + 滤波默认开/关由各环自己决定 */
    pid->d_mode = 0;   // 0:D on error, 1:D on meas
    pid->d_eflag = 0;  // 默认先关，内环你可以打开

    /* D on measurement 支持字段 */
    pid->meas = 0.0f;
    pid->last_meas = 0.0f;
    pid->meas_inited = 0;

    /* 变速积分/积分分离参数默认值 */
    pid->var_speed_I_A = 0.0f;
    pid->var_speed_I_B = 0.0f;
    pid->I_separate_thresh = 0.0f;

    /* 抗饱和相关默认 */
    pid->windup_limit = 0.0f;
    pid->anti_windup_flag = 0;
    pid->i_decrease_flag = 0;

    /* 限幅标志默认：输出限幅开、I限幅按需开 */
    pid->imax_flag = 1;
    pid->outmax_flag = 1;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     俯仰轴位置环PID初始化
 * 参数说明     *pid             位置PID结构体指针
 * 返回参数     void
 * 使用示例     pid_pitch_pos_init(&pitch_pos_pid);
 * 备注信息     专用于俯仰轴位置控制环，可根据需要设置特定参数
-------------------------------------------------------------------------------------------------------------------*/
void pid_pitch_pos_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_PITCH_POS_KP;
    pid->ki = TUNE_DEFAULT_PITCH_POS_KI;
    pid->kd = TUNE_DEFAULT_PITCH_POS_KD;

    pid->imax = 0.0f;
    pid->outmax = 500.0f; // 最大期望速度限制为 500 cm/s (5 m/s)

    pid->d_mode = 0;
    pid->d_eflag = 0;
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     俯仰轴速度环PID初始化
 * 参数说明     *pid             俯仰速度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_pitch_vel_init(&pitch_vel_pid);
 * 备注信息     专用于俯仰轴速度控制环
-------------------------------------------------------------------------------------------------------------------*/
void pid_pitch_vel_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_PITCH_VEL_KP;
    pid->ki = TUNE_DEFAULT_PITCH_VEL_KI;
    pid->kd = HVEL_D_ENABLE ? TUNE_DEFAULT_PITCH_VEL_KD : 0.0f;

    pid->imax = HVEL_IMAX_ACCEL_CMS2;
    pid->outmax = HVEL_ACCEL_CMD_LIMIT_CMS2;

    pid->d_mode = 1;
    pid->d_eflag = 1;
    pid->d_filter_mode = PID_D_FILTER_LIMITED_PT2;
    pid->d_filter_cutoff_hz = HVEL_D_CUTOFF_HZ;
    pid->d_raw_limit = HVEL_D_RAW_LIMIT_CMS2;
    pid->d_output_limit = 0.0f;
    pid->d_output_slew_rate = 0.0f;
    pid->I_separate_thresh = 1e9f;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     俯仰轴角度环PID初始化
 * 参数说明     *pid             俯仰角度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_pitch_angle_init(&pitch_angle_pid);
 * 备注信息     专用于俯仰轴角度控制环，可根据需要设置特定参数
-------------------------------------------------------------------------------------------------------------------*/
void pid_pitch_angle_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_PITCH_ANGLE_KP;
    pid->ki = TUNE_DEFAULT_PITCH_ANGLE_KI;
    pid->kd = TUNE_DEFAULT_PITCH_ANGLE_KD;

    pid->imax = 15.0f;
    pid->outmax = 450.0f;

    /* 角度环通常不用 D on meas */
    pid->d_mode = 0;
    pid->d_eflag = 0;

}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     俯仰轴角加速度环PID初始化
 * 参数说明     *pid             俯仰角加速度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_pitch_gyro_init(&pitch_acc_pid);
 * 备注信息     专用于俯仰轴角加速度控制环（内环），响应最快
-------------------------------------------------------------------------------------------------------------------*/
void pid_pitch_gyro_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_PITCH_GYRO_KP;
    pid->ki = TUNE_DEFAULT_PITCH_GYRO_KI;
    pid->kd = TUNE_DEFAULT_PITCH_GYRO_KD;

    pid->imax = 300.0f;
    pid->outmax = 430.0f;

    /* 内环推荐：D on measurement + D滤波打开 */
    pid->d_mode = 1;
    pid->d_eflag = 1;
    pid->meas_inited = 0;

    /* 如果你用 PidInterCtrl 的积分分离逻辑：建议给个非零阈值（否则I永远不进） */
    pid->I_separate_thresh = 1e9f;   // 等于“默认不做积分分离”
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     横滚轴位置环PID初始化
 * 参数说明     *pid             横滚位置PID结构体指针
 * 返回参数     void
 * 使用示例     pid_roll_pos_init(&roll_pos_pid);
 * 备注信息     专用于横滚轴位置控制环，可根据需要设置特定参数
-------------------------------------------------------------------------------------------------------------------*/
void pid_roll_pos_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_ROLL_POS_KP;
    pid->ki = TUNE_DEFAULT_ROLL_POS_KI;
    pid->kd = TUNE_DEFAULT_ROLL_POS_KD;

    pid->imax = 0.0f;
    pid->outmax = 500.0f; // 最大期望速度限制为 500 cm/s (5 m/s)

    pid->d_mode = 0;
    pid->d_eflag = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     横滚轴速度环PID初始化
 * 参数说明     *pid             横滚速度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_roll_vel_init(&roll_vel_pid);
 * 备注信息     专用于横滚轴速度控制环
-------------------------------------------------------------------------------------------------------------------*/
void pid_roll_vel_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_ROLL_VEL_KP;
    pid->ki = TUNE_DEFAULT_ROLL_VEL_KI;
    pid->kd = HVEL_D_ENABLE ? TUNE_DEFAULT_ROLL_VEL_KD : 0.0f;

    pid->imax = HVEL_IMAX_ACCEL_CMS2;
    pid->outmax = HVEL_ACCEL_CMD_LIMIT_CMS2;

    pid->d_mode = 1;
    pid->d_eflag = 1;
    pid->d_filter_mode = PID_D_FILTER_LIMITED_PT2;
    pid->d_filter_cutoff_hz = HVEL_D_CUTOFF_HZ;
    pid->d_raw_limit = HVEL_D_RAW_LIMIT_CMS2;
    pid->d_output_limit = 0.0f;
    pid->d_output_slew_rate = 0.0f;
    pid->I_separate_thresh = 1e9f;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     横滚轴角度环PID初始化
 * 参数说明     *pid             横滚角度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_roll_angle_init(&roll_angle_pid);
 * 备注信息     专用于横滚轴角度控制环，可根据需要设置特定参数
-------------------------------------------------------------------------------------------------------------------*/
void pid_roll_angle_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_ROLL_ANGLE_KP;
    pid->ki = TUNE_DEFAULT_ROLL_ANGLE_KI;
    pid->kd = TUNE_DEFAULT_ROLL_ANGLE_KD;

    pid->imax = 15.0f;
    pid->outmax = 450.0f;

    pid->d_mode = 0;
    pid->d_eflag = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     横滚轴角加速度环PID初始化
 * 参数说明     *pid             横滚角加速度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_roll_gyro_init(&roll_acc_pid);
 * 备注信息     专用于横滚轴角加速度控制环（内环），响应最快
-------------------------------------------------------------------------------------------------------------------*/
void pid_roll_gyro_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_ROLL_GYRO_KP;
    pid->ki = TUNE_DEFAULT_ROLL_GYRO_KI;
    pid->kd = TUNE_DEFAULT_ROLL_GYRO_KD;

    pid->imax = 300.0f;
    pid->outmax = 430.0f;

    pid->d_mode = 1;//d项低通滤波是否开启
    pid->d_eflag = 1;
    pid->meas_inited = 0;

    pid->I_separate_thresh = 1e9f;
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     偏航轴角度环PID初始化
 * 参数说明     *pid             偏航角度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_yaw_angle_init(&yaw_angle_pid);
 * 备注信息     专用于偏航轴角度控制环，可根据需要设置特定参数
-------------------------------------------------------------------------------------------------------------------*/
void pid_yaw_angle_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_YAW_ANGLE_KP;
    pid->ki = TUNE_DEFAULT_YAW_ANGLE_KI;
    pid->kd = TUNE_DEFAULT_YAW_ANGLE_KD;

    pid->imax = 0.0f;
    pid->outmax = 425.0f;

    pid->d_mode = 0;
    pid->d_eflag = 0;
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     偏航轴角加速度环PID初始化
 * 参数说明     *pid             偏航角加速度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_yaw_gyro_init(&yaw_acc_pid);
 * 备注信息     专用于偏航轴角加速度控制环（内环），响应最快
-------------------------------------------------------------------------------------------------------------------*/
void pid_yaw_gyro_init(pid_param_t* pid)
{
    if (!pid) return;
    PID_Structure_Init(pid);

    pid->kp = TUNE_DEFAULT_YAW_GYRO_KP;
    pid->ki = TUNE_DEFAULT_YAW_GYRO_KI;
    pid->kd = TUNE_DEFAULT_YAW_GYRO_KD;

    pid->imax = 400.0f;
    pid->outmax = 600.0f;

    pid->d_mode = 1;
    pid->d_eflag = 1;
    pid->meas_inited = 0;

    pid->I_separate_thresh = 1e9f;
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     高度轴位置环PID初始化
 * 参数说明     *pid             高度位置PID结构体指针
 * 返回参数     void
 * 使用示例     pid_height_pos_init(&height_pos_pid);
 * 备注信息     专用于高度位置控制环
-------------------------------------------------------------------------------------------------------------------*/
void pid_height_pos_init(pid_param_t* pid)
{
    PID_Structure_Init(pid);
    pid->kp = TUNE_DEFAULT_HEIGHT_POS_KP;
    pid->ki = TUNE_DEFAULT_HEIGHT_POS_KI;
    pid->kd = TUNE_DEFAULT_HEIGHT_POS_KD;

    pid->imax = 30.0f;
    pid->outmax = 200.0f;      // 最大期望速度 ±50 cm/s
    pid->d_mode = 0;
    pid->d_eflag = 0;
    pid->I_separate_thresh = 1e9f;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     高度轴速度环PID初始化
 * 参数说明     *pid             高度速度PID结构体指针
 * 返回参数     void
 * 使用示例     pid_height_vel_init(&height_vel_pid);
 * 备注信息     专用于高度速度控制环
-------------------------------------------------------------------------------------------------------------------*/
void pid_height_vel_init(pid_param_t* pid)
{
    PID_Structure_Init(pid);
    pid->kp = TUNE_DEFAULT_HEIGHT_VEL_KP;
    pid->ki = TUNE_DEFAULT_HEIGHT_VEL_KI;
    pid->kd = TUNE_DEFAULT_HEIGHT_VEL_KD;

    pid->imax = 1500.0f;
    pid->outmax = 2000.0f;     // 油门修正量限幅
    pid->d_mode = 1;          // D on measurement
    pid->d_eflag = 1;
    pid->meas_inited = 0;
    pid->I_separate_thresh = 1e9f;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     完整俯仰轴三环PID初始化
 * 参数说明     void
 * 返回参数     void
 * 使用示例     pid_pitch_init();
 * 备注信息     一次性初始化俯仰轴的角度环、速度环和加速度环
 *              应在系统启动时调用一次
-------------------------------------------------------------------------------------------------------------------*/
void pid_pitch_init(void){
    pid_pitch_pos_init(&pitch_pos_pid);
    pid_pitch_vel_init(&pitch_vel_pid);
    pid_pitch_angle_init(&pitch_angle_pid);
    pid_pitch_gyro_init(&pitch_acc_pid);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     完整偏航轴三环PID初始化
 * 参数说明     void
 * 返回参数     void
 * 使用示例     pid_yaw_init();
 * 备注信息     一次性初始化偏航轴的三环控制器
-------------------------------------------------------------------------------------------------------------------*/
void pid_yaw_init(void){
    pid_yaw_angle_init(&yaw_angle_pid);
    pid_yaw_gyro_init(&yaw_acc_pid);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     完整横滚轴三环PID初始化
 * 参数说明     void
 * 返回参数     void
 * 使用示例     pid_roll_init();
 * 备注信息     一次性初始化横滚轴的三环控制器
-------------------------------------------------------------------------------------------------------------------*/
void pid_roll_init(void){
    pid_roll_pos_init(&roll_pos_pid);
     pid_roll_vel_init(&roll_vel_pid);
     pid_roll_angle_init(&roll_angle_pid);
     pid_roll_gyro_init(&roll_acc_pid);
}

void pid_height_init(void){
    pid_height_pos_init(&height_pos_pid);
    pid_height_vel_init(&height_vel_pid);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     PID时间戳更新函数
 * 参数说明     *pid             PID结构体指针
 * 返回参数     uint8_t          状态码（0=成功，1=失败）
 * 使用示例     if(PID_UpdateTimeStamp(&pitch_pid) == 0) { 成功更新  }
 * 备注信息     计算两次调用之间的时间差(dt)，用于与时间相关的PID计算
 *              自动处理定时器溢出（回绕）情况
 *              对dt进行保护，防止过小或过大的时间间隔
-------------------------------------------------------------------------------------------------------------------*/
uint8_t PID_UpdateTimeStamp(pid_param_t* pid)
{
    if (!pid || !pid->Get_SystemTick) {
        return 1;  // 参数无效
    }

    uint32_t now = pid->Get_SystemTick();

    if (pid->last_time == 0) {
        pid->last_time = now;
        pid->dt = 0.0f;
        return 1;  // 首次调用
    }

    uint32_t delta = now - pid->last_time;   // 自动处理回绕（无符号数减法）
    pid->last_time = now;

    // 假设定时器配置为微秒模式
    pid->dt = (float)delta * 1e-6f;  // 转换为秒

    // dt保护：防止过小或过大的时间间隔导致计算不稳定
    if (pid->dt < 1e-6f) pid->dt = 1e-6f;  // 最小1微秒
    if (pid->dt > 0.05f) pid->dt = 0.05f;  // 最大50毫秒

    return 0;  // 成功
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     PID输出清零函数
 * 参数说明     *pid             PID结构体指针
 * 返回参数     void
 * 使用示例     PID_CleanOut(&pitch_pid);
 * 备注信息     在模式切换或出现异常时调用，重置控制器输出
-------------------------------------------------------------------------------------------------------------------*/
void PID_CleanOut(pid_param_t* pid) {
    if (pid == NULL) return;
    pid->out = 0.0f;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     PID积分项清零函数
 * 参数说明     *pid             PID结构体指针
 * 返回参数     void
 * 使用示例     PID_CleanIntegral(&pitch_pid);
 * 备注信息     用于重置积分器，防止积分饱和和模式切换时的冲击
-------------------------------------------------------------------------------------------------------------------*/
void PID_CleanIntegral(pid_param_t* pid) {
    if (pid == NULL) return;
    pid->integrator = 0.0f;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     误差计算函数
 * 参数说明     target           目标值（设定点）
 * 参数说明     data             测量值（反馈值）
 * 返回参数     float            误差值（target - data）
 * 使用示例     float error = get_error(desired_angle, measured_angle);
 * 备注信息     简单误差计算，可根据需要修改为其他误差形式
-------------------------------------------------------------------------------------------------------------------*/
float get_error(float target,float data){
    return target - data;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     俯仰轴PID控制主函数
 * 参数说明     target           目标俯仰角（度或弧度）
 * 返回参数     float            控制输出（通常为电机PWM差值）
 * 使用示例     float pitch_output = pitch_pid(10.0f, 0.0f, 1u, 0u);
 * 备注信息     实现俯仰轴的双环级联控制：
 *              外环（100Hz）：角度环（位置式PID）
 *              内环（1000Hz）：角加速度环（增量式PID）
 *              使用静态定时器实现多速率控制
-------------------------------------------------------------------------------------------------------------------*/
float pitch_pid(float position_target_y_cm, float speed_target_vy_cms,
                uint8_t position_enable, uint8_t position_deadzone_suppress){
    static uint16_t timer = 0,timer2 = 0;
    static uint8_t position_divider = 0u;
    static uint8_t position_was_enabled = 1u;
    imu_horizontal_state_t horizontal_state;
    horizontal_velocity_axis_debug_t hvel_debug;
    float tune_cmd = 0.0f;
    float tune_rate_cmd = 0.0f;
    uint8_t rate_tune_active;
    float angle_error;
    float rate_target;
    float position_speed_target_cms = 0.0f;
    float normal_position_speed_target_cms;
    float position_conflict_target_cms;
    float effective_speed_target_vy_cms;
    float feedforward_scale;
    uint8_t impact_brake_hold;
    uint8_t impact_recovery_ready;
    timer ++;
    timer2 ++;
    rate_tune_active = VofaTune_GetNonzeroRateCommand(VOFA_TUNE_TARGET_PG, &tune_rate_cmd);
    // 外环控制（）：速度环
    if (timer2 > 9) {
        if (!horizontal_control_attitude_only_enabled()) {
            imu_horizontal_get_state(&horizontal_state);

            if (position_enable) {
                if (!position_was_enabled) {
                    PidReset(&pitch_pos_pid);
                    position_divider = 0u;
                }
                position_divider++;
                if (position_divider >= HPOS_UPDATE_DIVIDER) {
                    PidInterCtrl(&pitch_pos_pid, -horizontal_state.pos_y_cm + position_target_y_cm, HPOS_DT_S);//位置环50Hz更新
                    position_divider = 0u;
                }
                // 两次位置环更新之间保持上一次输出，供100Hz速度环连续使用
                position_speed_target_cms = pitch_pos_pid.out;
            } else {
                if (position_was_enabled) {
                    PidReset(&pitch_pos_pid);
                }
                position_divider = 0u;
                position_speed_target_cms = 0.0f;
            }
            position_was_enabled = position_enable ? 1u : 0u;

            position_speed_target_cms =
                horizontal_velocity_limit_same_direction_catchup(
                    position_speed_target_cms, speed_target_vy_cms);
            position_conflict_target_cms = position_speed_target_cms;

            // 死区内保留位置环计算结果用于冲突判断，实际速度目标不叠加位置修正。
            if (position_deadzone_suppress) {
                position_speed_target_cms = 0.0f;
            }
            normal_position_speed_target_cms = position_speed_target_cms;
            impact_brake_hold = horizontal_impact_brake_hold_update(
                &pitch_hvel_state,
                horizontal_state.external_accel_guard_stage);
            position_speed_target_cms = horizontal_impact_position_command(
                &pitch_hvel_state,
                normal_position_speed_target_cms,
                horizontal_state.external_accel_guard_stage,
                impact_brake_hold);
            feedforward_scale = horizontal_impact_feedforward_scale(
                &pitch_hvel_state,
                horizontal_state.external_accel_guard_stage,
                impact_brake_hold);

            if (impact_brake_hold) {
                // ACTIVE及冲击结束后1秒内，位置修正和车速前馈都保持为0。
                effective_speed_target_vy_cms = 0.0f;
            } else {
                // NORMAL保持原路径；保持结束后平滑恢复位置修正和车速前馈。
                effective_speed_target_vy_cms =
                    horizontal_velocity_limit_conflicting_feedforward(
                        (horizontal_state.external_accel_guard_stage ==
                         HORIZONTAL_IMPACT_GUARD_RECOVERY) ?
                            position_speed_target_cms :
                            position_conflict_target_cms,
                        speed_target_vy_cms * feedforward_scale,
                        position_deadzone_suppress ?
                            HVEL_POS_FF_CONFLICT_DEADZONE_MIN_POS_CMS :
                            HVEL_POS_FF_CONFLICT_MIN_POS_CMS);
            }
            hvel_debug.feedforward_conflict_limited =
                (uint8_t)(fabsf(effective_speed_target_vy_cms -
                                speed_target_vy_cms * feedforward_scale) > 0.001f);

            if (horizontal_state.external_accel_guard_stage !=
                HORIZONTAL_IMPACT_GUARD_ACTIVE) {
                effective_speed_target_vy_cms =
                    horizontal_velocity_decay_deadzone_same_direction_feedforward(
                        (horizontal_state.external_accel_guard_stage ==
                         HORIZONTAL_IMPACT_GUARD_RECOVERY) ?
                            position_speed_target_cms :
                            position_conflict_target_cms,
                        effective_speed_target_vy_cms,
                        position_deadzone_suppress);
            }
            hvel_debug.feedforward_effective_cms = effective_speed_target_vy_cms;

            impact_recovery_ready = (uint8_t)(
                (horizontal_state.external_accel_guard_stage ==
                HORIZONTAL_IMPACT_GUARD_RECOVERY) &&
                (!impact_brake_hold) &&
                (horizontal_state.external_accel_guard_recovery >= 1.0f) &&
                (pitch_hvel_state.impact_feedforward_recovery_cycles >=
                 HVEL_IMPACT_FF_RECOVERY_CYCLES) &&
                (fabsf(position_speed_target_cms -
                       normal_position_speed_target_cms) <=
                 HVEL_IMPACT_POSITION_READY_CMS));
            imu_horizontal_set_impact_recovery_ready(
                HORIZONTAL_IMPACT_READY_PITCH, impact_recovery_ready);

            horizontal_velocity_update_axis(&pitch_hvel_state,
                                            &pitch_vel_pid,
                                            HVEL_PITCH_KP,
                                            position_speed_target_cms +
                                                effective_speed_target_vy_cms,
                                            horizontal_state.control_vy_cms,
                                            horizontal_state.control_velocity_valid_y,
                                            horizontal_state.control_velocity_confidence_y,
                                            horizontal_state.control_velocity_source_y,
                                            horizontal_state.full_reset_count,
                                            horizontal_state.external_accel_guard_stage,
                                            &hvel_debug);

            // 调试与AutoTrim继续表示LC302本身有效性；速度环内部已使用组合速度有效位。
            hvel_debug.flow_valid = horizontal_state.flow_valid_y;

            taskENTER_CRITICAL();
            horizontal_velocity_debug.pitch = hvel_debug;
            horizontal_velocity_debug.full_reset_count = horizontal_state.full_reset_count;
            taskEXIT_CRITICAL();
        }

        timer2 = 0;
    }

    // 外环控制（）：角度环
    if (timer > 1) {
        if (VofaTune_GetActiveCommand(VOFA_TUNE_TARGET_PA, &tune_cmd)) {
            angle_error = pitch_filtered
                        - (pitch_zero + attitude_auto_trim_debug.pitch_applied_deg)
                        - tune_cmd;
        } else if (horizontal_control_attitude_only_enabled()) {
            angle_error = pitch_filtered
                        - (pitch_zero + attitude_auto_trim_debug.pitch_applied_deg);
        } else {
            angle_error = pitch_filtered
                        - (pitch_zero + attitude_auto_trim_debug.pitch_applied_deg)
                        - pitch_vel_pid.out;
        }
        // 角度环：目标角度 - 当前角度                补偿
        PidLocCtrl(&pitch_angle_pid, angle_error,0.002f);
        timer = 0;
    }

    // 内环控制（1000Hz）：角加速度环
    // 角加速度环：角度环输出 - 当前角速度
            // 第一次启用/上电时：对齐 last_meas，避免D尖峰
    pitch_acc_pid.meas = pitch_gyro_filtered;
    if (!pitch_acc_pid.meas_inited) {
        pitch_acc_pid.last_meas = pitch_acc_pid.meas;
        pitch_acc_pid.d_term_lpf = 0.0f;
        pitch_acc_pid.last_d_output = 0.0f;
        pitch_acc_pid.meas_inited = 1;
    }
    rate_target = pitch_angle_pid.out;
    if (rate_tune_active) {
        rate_target += tune_rate_cmd;
    }
    PidInterCtrl(&pitch_acc_pid, -pitch_gyro_filtered + rate_target,0.001f);

    return pitch_acc_pid.out;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     横滚轴PID控制主函数
 * 参数说明     target           目标横滚角（度或弧度）
 * 返回参数     float            控制输出
 * 使用示例     float roll_output = roll_pid(5.0f, 0.0f, 1u, 0u);
 * 备注信息     实现与俯仰轴相同的双环级联控制结构
-------------------------------------------------------------------------------------------------------------------*/
float roll_pid(float position_target_x_cm, float speed_target_vx_cms,
               uint8_t position_enable, uint8_t position_deadzone_suppress){
    static uint16_t timer = 0,timer2 = 0;
    static uint8_t position_divider = 0u;
    static uint8_t position_was_enabled = 1u;
    imu_horizontal_state_t horizontal_state;
    horizontal_velocity_axis_debug_t hvel_debug;
    float tune_cmd = 0.0f;
    float tune_rate_cmd = 0.0f;
    uint8_t rate_tune_active;
    float angle_error;
    float rate_target;
    float position_speed_target_cms = 0.0f;
    float normal_position_speed_target_cms;
    float position_conflict_target_cms;
    float effective_speed_target_vx_cms;
    float feedforward_scale;
    uint8_t impact_brake_hold;
    uint8_t impact_recovery_ready;
    timer ++;
    timer2 ++;
    rate_tune_active = VofaTune_GetNonzeroRateCommand(VOFA_TUNE_TARGET_RG, &tune_rate_cmd);
    // 外环控制（）：速度环
    if (timer2 > 9) {
        if (!horizontal_control_attitude_only_enabled()) {
            imu_horizontal_get_state(&horizontal_state);

            if (position_enable) {
                if (!position_was_enabled) {
                    PidReset(&roll_pos_pid);
                    position_divider = 0u;
                }
                position_divider++;
                if (position_divider >= HPOS_UPDATE_DIVIDER) {
                    PidInterCtrl(&roll_pos_pid,+ horizontal_state.pos_x_cm + position_target_x_cm , HPOS_DT_S);//位置环50Hz更新
                    position_divider = 0u;
                }
                // 两次位置环更新之间保持上一次输出，供100Hz速度环连续使用
                position_speed_target_cms = roll_pos_pid.out;
            } else {
                if (position_was_enabled) {
                    PidReset(&roll_pos_pid);
                }
                position_divider = 0u;
                position_speed_target_cms = 0.0f;
            }
            position_was_enabled = position_enable ? 1u : 0u;

            position_speed_target_cms =
                horizontal_velocity_limit_same_direction_catchup(
                    position_speed_target_cms, speed_target_vx_cms);
            position_conflict_target_cms = position_speed_target_cms;

            // 死区内保留位置环计算结果用于冲突判断，实际速度目标不叠加位置修正。
            if (position_deadzone_suppress) {
                position_speed_target_cms = 0.0f;
            }
            normal_position_speed_target_cms = position_speed_target_cms;
            impact_brake_hold = horizontal_impact_brake_hold_update(
                &roll_hvel_state,
                horizontal_state.external_accel_guard_stage);
            position_speed_target_cms = horizontal_impact_position_command(
                &roll_hvel_state,
                normal_position_speed_target_cms,
                horizontal_state.external_accel_guard_stage,
                impact_brake_hold);
            feedforward_scale = horizontal_impact_feedforward_scale(
                &roll_hvel_state,
                horizontal_state.external_accel_guard_stage,
                impact_brake_hold);

            if (impact_brake_hold) {
                effective_speed_target_vx_cms = 0.0f;
            } else {
                effective_speed_target_vx_cms =
                    horizontal_velocity_limit_conflicting_feedforward(
                        (horizontal_state.external_accel_guard_stage ==
                         HORIZONTAL_IMPACT_GUARD_RECOVERY) ?
                            position_speed_target_cms :
                            position_conflict_target_cms,
                        speed_target_vx_cms * feedforward_scale,
                        position_deadzone_suppress ?
                            HVEL_POS_FF_CONFLICT_DEADZONE_MIN_POS_CMS :
                            HVEL_POS_FF_CONFLICT_MIN_POS_CMS);
            }
            hvel_debug.feedforward_conflict_limited =
                (uint8_t)(fabsf(effective_speed_target_vx_cms -
                                speed_target_vx_cms * feedforward_scale) > 0.001f);

            if (horizontal_state.external_accel_guard_stage !=
                HORIZONTAL_IMPACT_GUARD_ACTIVE) {
                effective_speed_target_vx_cms =
                    horizontal_velocity_decay_deadzone_same_direction_feedforward(
                        (horizontal_state.external_accel_guard_stage ==
                         HORIZONTAL_IMPACT_GUARD_RECOVERY) ?
                            position_speed_target_cms :
                            position_conflict_target_cms,
                        effective_speed_target_vx_cms,
                        position_deadzone_suppress);
            }
            hvel_debug.feedforward_effective_cms = effective_speed_target_vx_cms;

            impact_recovery_ready = (uint8_t)(
                (horizontal_state.external_accel_guard_stage ==
                HORIZONTAL_IMPACT_GUARD_RECOVERY) &&
                (!impact_brake_hold) &&
                (horizontal_state.external_accel_guard_recovery >= 1.0f) &&
                (roll_hvel_state.impact_feedforward_recovery_cycles >=
                 HVEL_IMPACT_FF_RECOVERY_CYCLES) &&
                (fabsf(position_speed_target_cms -
                       normal_position_speed_target_cms) <=
                 HVEL_IMPACT_POSITION_READY_CMS));
            imu_horizontal_set_impact_recovery_ready(
                HORIZONTAL_IMPACT_READY_ROLL, impact_recovery_ready);

            horizontal_velocity_update_axis(&roll_hvel_state,
                                            &roll_vel_pid,
                                            HVEL_ROLL_KP,
                                            position_speed_target_cms +
                                                effective_speed_target_vx_cms,
                                            -horizontal_state.control_vx_cms,
                                            horizontal_state.control_velocity_valid_x,
                                            horizontal_state.control_velocity_confidence_x,
                                            horizontal_state.control_velocity_source_x,
                                            horizontal_state.full_reset_count,
                                            horizontal_state.external_accel_guard_stage,
                                            &hvel_debug);

            hvel_debug.flow_valid = horizontal_state.flow_valid_x;
            taskENTER_CRITICAL();
            horizontal_velocity_debug.roll = hvel_debug;
            horizontal_velocity_debug.full_reset_count = horizontal_state.full_reset_count;
            taskEXIT_CRITICAL();
        }

        timer2 = 0;
    }

    // 外环控制（）：角度环
    if (timer > 1) {
        if (VofaTune_GetActiveCommand(VOFA_TUNE_TARGET_RA, &tune_cmd)) {
            angle_error = -roll_filtered
                        + (roll_zero + attitude_auto_trim_debug.roll_applied_deg)
                        + tune_cmd;
        } else if (horizontal_control_attitude_only_enabled()) {
            angle_error = -roll_filtered
                        + (roll_zero + attitude_auto_trim_debug.roll_applied_deg);
        } else {
            angle_error = -roll_filtered
                        + (roll_zero + attitude_auto_trim_debug.roll_applied_deg)
                        + roll_vel_pid.out;
        }
        // 角度环：目标角度 - 当前角度
        PidLocCtrl(&roll_angle_pid, angle_error,0.002f);
        timer = 0;
    }

    roll_acc_pid.meas = roll_gyro_filtered;
    if (!roll_acc_pid.meas_inited) {
        roll_acc_pid.last_meas = roll_acc_pid.meas;
        roll_acc_pid.d_term_lpf = 0.0f;
        roll_acc_pid.last_d_output = 0.0f;
        roll_acc_pid.meas_inited = 1;
    }
    rate_target = roll_angle_pid.out;
    if (rate_tune_active) {
        rate_target += tune_rate_cmd;
    }
    PidInterCtrl(&roll_acc_pid, -roll_gyro_filtered + rate_target,0.001f);

    return roll_acc_pid.out;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     偏航轴PID控制主函数
 * 参数说明     target           目标偏航角（度或弧度）
 * 返回参数     float            控制输出
 * 使用示例     float yaw_output = yaw_pid(90.0f); // 控制到90度偏航
 * 备注信息     实现偏航轴的双环级联控制
 *              注意：偏航轴控制通常比俯仰/横滚轴响应慢
-------------------------------------------------------------------------------------------------------------------*/
float yaw_pid(float target){
    static uint16_t timer = 0;
    float tune_rate_cmd = 0.0f;
    uint8_t rate_tune_active;
    float yaw_target = target;
    float rate_target;
    timer += 1;
    rate_tune_active = VofaTune_GetNonzeroRateCommand(VOFA_TUNE_TARGET_YG, &tune_rate_cmd);

    if (timer > 1) {
        timer = 0;

        VofaTune_GetYawTarget(&yaw_target);

        //走最短路径
        float error = yaw_filtered - yaw_target;
        while (error >  180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        PidLocCtrl(&yaw_angle_pid, error, 0.002f);
    }

    yaw_acc_pid.meas = yaw_gyro_filtered;
    if (!yaw_acc_pid.meas_inited) {
        yaw_acc_pid.last_meas = yaw_acc_pid.meas;
        yaw_acc_pid.d_term_lpf = 0.0f;
        yaw_acc_pid.last_d_output = 0.0f;
        yaw_acc_pid.meas_inited = 1;
    }
    rate_target = yaw_angle_pid.out;
    if (rate_tune_active) {
        rate_target += tune_rate_cmd;
    }
    PidInterCtrl(&yaw_acc_pid, -yaw_gyro_filtered + rate_target, 0.001f);

    return yaw_acc_pid.out;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     高度轴PID控制主函数
 * 参数说明     target           目标高度（单位：毫米）
 * 返回参数     float            控制输出
 * 使用示例     float height_output = height_pid(1000.0f); // 控制到1000毫米高度
 * 备注信息     实现高度轴的双环级联控制
 *              注意：高度轴控制通常比俯仰/横滚轴响应慢
-------------------------------------------------------------------------------------------------------------------*/
/* Height cascade with optional vertical-speed feedforward in cm/s. */
static float height_impact_target(float normal_target_mm,
                                  uint8_t position_update_due)
{
    static uint8_t protection_active = 0u;
    static uint8_t last_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
    static float saved_normal_target_mm = 0.0f;
    static float protected_target_mm = 0.0f;
    imu_horizontal_state_t horizontal_state;
    uint8_t protection_allowed;
    uint8_t guard_stage;
    float drop_target_mm;
    float recovery_step_mm;

    /* down_flag在usr_system.c中定义；降落流程必须保持原有高度目标语义。 */
    extern volatile uint8_t down_flag;

    protection_allowed = (uint8_t)((lockflag == 0u) &&
                                   (height_Loc_flag == 2u) &&
                                   (down_flag == 0u) &&
                                   (height >= 150.0f));
    if (!protection_allowed) {
        protection_active = 0u;
        last_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
        saved_normal_target_mm = 0.0f;
        protected_target_mm = 0.0f;
        return normal_target_mm;
    }

    imu_horizontal_get_state(&horizontal_state);
    guard_stage = horizontal_state.external_accel_guard_stage;
    drop_target_mm = saved_normal_target_mm - HEIGHT_IMPACT_DROP_MM;
    if (drop_target_mm < HEIGHT_IMPACT_MIN_TARGET_MM) {
        drop_target_mm = HEIGHT_IMPACT_MIN_TARGET_MM;
    }

    /* 首次进入ACTIVE以及RECOVERY期间再次冲击，都立即重新降低目标。 */
    if ((guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) &&
        ((protection_active == 0u) ||
         (last_guard_stage != HORIZONTAL_IMPACT_GUARD_ACTIVE))) {
        saved_normal_target_mm = normal_target_mm;
        drop_target_mm = saved_normal_target_mm - HEIGHT_IMPACT_DROP_MM;
        if (drop_target_mm < HEIGHT_IMPACT_MIN_TARGET_MM) {
            drop_target_mm = HEIGHT_IMPACT_MIN_TARGET_MM;
        }
        protected_target_mm = drop_target_mm;
        protection_active = 1u;
    } else if (protection_active == 0u) {
        last_guard_stage = guard_stage;
        return normal_target_mm;
    }

    if (guard_stage == HORIZONTAL_IMPACT_GUARD_ACTIVE) {
        protected_target_mm = drop_target_mm;
    } else if (position_update_due != 0u) {
        recovery_step_mm = HEIGHT_IMPACT_RECOVERY_RATE_MM_S *
                           HEIGHT_IMPACT_POSITION_DT_S;
        protected_target_mm += recovery_step_mm;
        if (protected_target_mm >= saved_normal_target_mm) {
            protected_target_mm = saved_normal_target_mm;
            protection_active = 0u;
            last_guard_stage = HORIZONTAL_IMPACT_GUARD_NORMAL;
            saved_normal_target_mm = 0.0f;
            return normal_target_mm;
        }
    }

    last_guard_stage = guard_stage;
    return protected_target_mm;
}

float height_pid_with_velocity_ff(float target, float target_velocity_cms)
{
    static uint16_t timer_pos = 0, timer_vel = 0;
    float tune_target = target;
    float tune_cmd = 0.0f;
    uint8_t position_update_due;
    uint8_t tune_hv_active = VofaTune_GetActiveCommand(VOFA_TUNE_TARGET_HV, &tune_cmd);
    timer_pos++;
    timer_vel++;
    position_update_due = (timer_pos >= 10u) ? 1u : 0u;

    VofaTune_GetHeightTarget(&tune_target);
    tune_target = height_impact_target(tune_target, position_update_due);

    // 外环位置环（10ms）
    if (timer_pos >= 10) {
        if (!tune_hv_active) {
            PidInterCtrl(&height_pos_pid, tune_target - height, 0.01f);
        }
        timer_pos = 0;
    }

    // 内环速度环（2ms）
    if (timer_vel >= 2) {
        timer_vel = 0;
        // 调用增量式PID（或位置式）
        height_vel_pid.meas = vertical_velocity_smoothed;   // 用于 D on measurement
        if (tune_hv_active) {
            PidInterCtrl(&height_vel_pid, tune_cmd - vertical_velocity_smoothed, 0.002f);
        } else {
            PidInterCtrl(&height_vel_pid,
                         height_pos_pid.out + target_velocity_cms - vertical_velocity_smoothed,
                         0.002f);
        }
    }

    return height_vel_pid.out;
}

float height_pid(float target)
{
    return height_pid_with_velocity_ff(target, 0.0f);
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     位置式PID控制器
 * 参数说明     *pid             PID结构体指针
 * 参数说明     error            当前误差值
 * 返回参数     float            PID输出值
 * 使用示例     float output = PidLocCtrl(&angle_pid, current_error);
 * 备注信息     标准位置式PID算法，包含：
 *              1. 比例项：kp * error
 *              2. 积分项：ki * ∫error dt
 *              3. 微分项：kd * d(error)/dt
 *              4. 积分限幅和输出限幅
 *              5. 前馈项支持
 *              适用于外环（角度环）控制
-------------------------------------------------------------------------------------------------------------------*/
float PidLocCtrl(pid_param_t *pid, float error,float dt)
{
    // 积分项累加（注意：dt应为全局变量或通过其他方式获取）
    pid->integrator += error * dt;

    // 计算各项输出
    pid->out_p = pid->kp * error;              // 比例项
    pid->out_i = pid->ki * pid->integrator;    // 积分项

    // 微分项计算（后向差分）
    float derivative = (error - pid->last_error) / dt;
    pid->out_d = pid->kd * derivative;

    // 积分限幅（抗饱和）
    if (pid->imax_flag)
        pid->out_i = constrain_float(pid->out_i, -pid->imax, pid->imax);

    pid->last_error = error;  // 更新误差历史

    // 计算总输出（比例 + 积分 + 微分 + 前馈）
    pid->out = pid->out_p + pid->out_i + pid->out_d + pid->feed_forward;

    // 输出限幅
    if (pid->outmax_flag)
        pid->out = constrain_float(pid->out, -pid->outmax, pid->outmax);

    return pid->out;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     增量式PID控制器
 * 参数说明     *pid             PID结构体指针
 * 参数说明     error            当前误差值
 * 返回参数     float            PID输出值
 * 使用示例     float output = PidIncCtrl(&rate_pid, current_error);
 * 备注信息     增量式PID算法，包含：
 *              1. 只计算输出的增量Δu
 *              2. 累加增量得到总输出
 *              3. 适合执行机构带积分特性的场合（如电机）
 *              4. 无积分饱和问题
 *              5. 易于实现手动/自动无扰切换
 *              适用于内环（角速度/加速度环）控制
-------------------------------------------------------------------------------------------------------------------*/
float PidIncCtrl(pid_param_t *pid, float error, float dt)
{
    if(!pid) return 0.0f;
    if(dt < 1e-6f) dt = 2e-3f; // 防止除0，给默认值

    float e  = error;
    float e1 = pid->last_error;

    // --- 1. P项 (比例增量) ---
    float du_p = pid->kp * (e - e1);

    // --- 2. D项 (微分项重构 + 滤波) ---
    // 原来的写法噪声太大：pid->kd * (e - 2.0f*e1 + e2) / dt;
    // 改为：计算当前的微分值，然后与上一次的微分值做增量，并滤波
    // 这种写法虽然稍微偏离纯增量公式，但对工程更友好

    // 当前的瞬时微分 (Rate of Change)
    float derivative_raw = (e - e1) / dt;

    // 对微分值进行低通滤波 (LPF) !非常重要!
    pid->d_term_lpf = pid->d_term_lpf * (1.0f - D_FILTER_ALPHA) + derivative_raw * D_FILTER_ALPHA;

    // 计算D项的输出 (位置式D)
    float d_output = pid->kd * pid->d_term_lpf;

    // 计算D项的增量 (当前D输出 - 上次D输出)

    float du_d = d_output - pid->last_d_output;
    pid->last_d_output = d_output;

    // --- 3. I项 (积分增量) ---
    // 解决“匀速倾倒”的关键
    float du_i = pid->ki * dt * e;

    // --- 4. 预测输出 ---
    float out_pred = pid->out + du_p + du_i + du_d;

    // --- 5. 抗风up (积分限幅与输出限幅) ---
    float out_hi =  pid->outmax;
    float out_lo = -pid->outmax;

    // 检查是否饱和
    if (out_pred > out_hi) {
        out_pred = out_hi;
        // 如果输出饱和且误差还在往同方向推，扣除刚才加上的I项增量(抗饱和)
        if (e > 0 && du_i > 0) out_pred -= du_i;
    } else if (out_pred < out_lo) {
        out_pred = out_lo;
        if (e < 0 && du_i < 0) out_pred -= du_i;
    }

    // --- 6. 更新状态 ---
    pid->out = out_pred; // 核心：保存累加值
    pid->last_last_error = e1;
    pid->last_error = e;

    return pid->out + pid->feed_forward;
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     PID参数设置函数
 * 参数说明     *pid             PID结构体指针
 * 参数说明     kp               比例系数
 * 参数说明     ki               积分系数
 * 参数说明     kd               微分系数
 * 返回参数     void
 * 使用示例     PID_SetParam(&pitch_pid, 1.0f, 0.5f, 0.1f);
 * 备注信息     用于动态调整PID参数，支持在线调参和自适应控制
 * -------------------------------------------------------------------------------------------------------------------*/
void PID_SetParam(pid_param_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}



float PidInterCtrl(pid_param_t *pid, float error, float dt)
{
    if (!pid) return 0.0f;

    // dt保护
    if (dt < 1e-6f) dt = 1e-6f;
    pid->dt = dt;

    float e  = error;
    float e1 = pid->last_error;
    // 死区（建议：仍更新历史，避免退出死区时D尖峰）
    if (fabsf(e) < pid->DeadZone) {
        e=0;
        if (pid->d_mode == 1) {
            pid->last_meas = pid->meas;
            pid->meas_inited = 1;
        }
        return pid->out;
    }

    // 误差滤波（如果开）
    if (pid->efilter != 0) {
        float a = pid->e_term_lpf;  // 0~1
        e = e * (1.0f - a) + e1 * a;
    }

    // ---------------- P ----------------
    pid->out_p = pid->kp * e;

    // ---------------- D（支持 D on error / D on meas）----------------
    float deriv_raw = 0.0f;
    if (pid->d_mode == 0) {
        deriv_raw = (e - e1) / dt;
    } else if (!pid->d_input_valid) {
        pid->last_meas = pid->meas;
        pid->meas_inited = 1u;
        pid->d_term_lpf = 0.0f;
        pid->d_term_lpf2 = 0.0f;
    } else if (!pid->meas_inited) {
        pid->last_meas = pid->meas;
        pid->d_term_lpf = 0.0f;
        pid->d_term_lpf2 = 0.0f;
        pid->meas_inited = 1u;
    } else {
        deriv_raw = -(pid->meas - pid->last_meas) / dt;
    }

    if (pid->d_raw_limit > 0.0f) {
        deriv_raw = constrain_float(deriv_raw, -pid->d_raw_limit, pid->d_raw_limit);
    }
    pid->last_derivative = deriv_raw;

    if (pid->d_filter_mode == PID_D_FILTER_LIMITED_PT2) {
        float cutoff_hz = pid->d_filter_cutoff_hz;
        float rc;
        float a;
        float d_target;

        if (cutoff_hz < 0.01f) {
            cutoff_hz = 0.01f;
        }
        rc = 1.0f / (6.2831853f * cutoff_hz);
        a = dt / (rc + dt);

        if (pid->d_mode == 1 && !pid->d_input_valid) {
            pid->d_term_lpf = 0.0f;
            pid->d_term_lpf2 = 0.0f;
        } else {
            pid->d_term_lpf += a * (deriv_raw - pid->d_term_lpf);
            pid->d_term_lpf2 += a * (pid->d_term_lpf - pid->d_term_lpf2);
        }

        d_target = pid->kd * pid->d_term_lpf2;
        if (pid->d_output_limit > 0.0f) {
            d_target = constrain_float(d_target,
                                       -pid->d_output_limit,
                                        pid->d_output_limit);
        }
        if (pid->d_output_slew_rate > 0.0f) {
            float d_step = pid->d_output_slew_rate * dt;
            pid->out_d += constrain_float(d_target - pid->out_d, -d_step, d_step);
        } else {
            pid->out_d = d_target;
        }
        pid->last_d_output = pid->out_d;
    } else {
        if (pid->d_eflag) {
            float a = D_FILTER_ALPHA;
            pid->d_term_lpf = pid->d_term_lpf * (1.0f - a) + deriv_raw * a;
        } else {
            pid->d_term_lpf = deriv_raw;
        }
        pid->out_d = pid->kd * pid->d_term_lpf;
    }

    // ---------------- 变速积分系数（先算）----------------
    float abs_err = fabsf(e);
    float I_VarSpeedf = 1.0f;

    // 如果A、B都为0，认为“不启用变速积分”，权重=1
    if (pid->var_speed_I_A > 1e-6f || pid->var_speed_I_B > 1e-6f) {
        I_VarSpeedf = 0.0f;
        if (abs_err <= pid->var_speed_I_B) {
            I_VarSpeedf = 1.0f;
        } else if (abs_err <= (pid->var_speed_I_A + pid->var_speed_I_B)) {
            if (pid->var_speed_I_A > 1e-6f) {
                I_VarSpeedf = (pid->var_speed_I_A + pid->var_speed_I_B - abs_err) / pid->var_speed_I_A;
            }
        }
    }

    // ---------------- anti-windup：先用“当前I”估算本次是否已饱和 ----------------
    float out_i_now = 0.0f;
    if (fabsf(pid->ki) > 1e-6f) {
        out_i_now = pid->ki * pid->integrator;
        out_i_now = constrain_float(out_i_now, -pid->imax, pid->imax);
    }

    float out_pre = pid->out_p + out_i_now + pid->out_d + pid->feed_forward;

    // 积分“推力方向”：dt>0，I_VarSpeedf>=0，所以方向由 ki*e 决定
    uint8_t allow_integrate = 1;
    float i_push = pid->ki * I_VarSpeedf * e;

    // 已经上饱和还想往上推 -> 禁止积分
    if (out_pre > pid->outmax && i_push > 0.0f) {
        allow_integrate = 0;
    }
    // 已经下饱和还想往下推 -> 禁止积分
    else if (out_pre < -pid->outmax && i_push < 0.0f) {
        allow_integrate = 0;
    }

    // ---------------- 积分更新（积分分离 + 抗饱和）----------------
    if (fabsf(pid->ki) > 1e-6f &&
        abs_err < pid->I_separate_thresh &&
        pid->i_input_valid &&
        allow_integrate) {

        // integrator限幅：保证 out_i 不超过 imax
        float integral_max = fabsf(pid->imax / pid->ki);
        pid->integrator += I_VarSpeedf * e * dt;
        pid->integrator = constrain_float(pid->integrator, -integral_max, integral_max);
    }

    // ---------------- I 输出 ----------------
    if (fabsf(pid->ki) > 1e-6f && abs_err < pid->I_separate_thresh) {
        pid->out_i = pid->ki * pid->integrator;
        pid->out_i = constrain_float(pid->out_i, -pid->imax, pid->imax);
    } else {
        pid->out_i = 0.0f;
        if (fabsf(pid->ki) <= 1e-6f) {
            pid->integrator = 0.0f; // ki=0 时清掉，避免以后突然开ki冲击
        }
    }

    // ---------------- 合成 + 输出限幅 ----------------
    pid->out = pid->out_p + pid->out_i + pid->out_d + pid->feed_forward;
    pid->out = constrain_float(pid->out, -pid->outmax, pid->outmax);

    // ---------------- 更新历史 ----------------
    pid->last_error = e;          // 存滤波后的误差
    if (pid->d_mode == 1) {
        pid->last_meas = pid->meas;
    }

    return pid->out;
}



void PidReset(pid_param_t *pid)
{
    if (!pid) return;

    pid->out_p = 0.0f;
    pid->integrator = 0.0f;
    pid->out_i = 0.0f;

    pid->d_term_lpf = 0.0f;
    pid->d_term_lpf2 = 0.0f;
    pid->out_d = 0.0f;
    pid->last_d_output = 0.0f;
    pid->last_derivative = 0.0f;

    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->meas = 0.0f;
    pid->last_meas = 0.0f;
    pid->meas_inited = 0u;
    pid->d_input_valid = 1u;
    pid->i_input_valid = 1u;

    pid->feed_forward = 0.0f;
    pid->out = 0.0f;
}
