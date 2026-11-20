#ifndef TUNING_PARAM_CONFIG_H
#define TUNING_PARAM_CONFIG_H

/*
 * ========================================================================
 * M7_0 菜单调参统一配置（TFT180 OLED 菜单 + Flash 存储）
 * ========================================================================
 *
 * 【命名约定】
 *   TUNE_DEFAULT_<通道>_<环>_<项> ：该参数的代码默认值。
 *                                   仅在 Flash 中没有有效参数时才使用
 *                                   （首次上电 / Flash 被清空 / 参数校验失败）。
 *   TUNE_STEP_<通道>_<环>_<项>   ：菜单编辑模式下，每按一次按键该参数的增减步长。
 *
 * 【参数生效优先级】（由高到低）
 *   1. Flash 中已保存的有效参数（在 OLED 菜单里调好并 Save 过的值）——保存后始终优先；
 *   2. 本文件 TUNE_DEFAULT_* 默认值——只负责"出厂"初值，不会自动覆盖 Flash 已有参数。
 *
 * 【串级控制架构】（自外向内，越靠内环周期越短、响应越快）
 *   水平位置环(20ms) → 水平速度环(10ms) → 俯仰/横滚 角度环(2ms) → 角速度环(1ms) → 电机
 *   高度位置环(10ms) → 高度速度环(2ms)  → 总油门 min_motor
 *   偏航角度环(2ms)  → 偏航角速度环(1ms) → 电机
 *   视觉外环(10ms)  → 输出"位置目标(cm)"给水平位置环，同时提供小车速度前馈(cm/s)
 *
 * 【单位约定】
 *   角度/姿态：度(deg)；角速度：度/秒(deg/s)；高度：毫米(mm)；水平位置：厘米(cm)；
 *   水平速度：厘米/秒(cm/s)；垂直速度：厘米/秒(cm/s)；角速度环输出/油门增量：PWM 微秒(us)。
 *   各宏后面的"OLED 项"指该参数在 OLED 调参菜单中的显示名（见 oled.c build_param_lists）。
 */

// ==================== 姿态角度环与角速度环 ====================
// 角度环：输入"目标角度-实际角度"的角度误差(deg)，输出"目标角速度"(deg/s) 作为角速度环的目标。
//         周期 2ms(500Hz)，位置式 PID（PidLocCtrl，dt=0.002）。
// 角速度环：输入"目标角速度-实际角速度"的角速度误差(deg/s)，输出电机 PWM 增量(us)。
//         周期 1ms(1000Hz)，增量式 PID（PidInterCtrl，dt=0.001），输出直接叠加到总油门做电机混控。

// ----- 俯仰(Pitch) 角度环 ----- （OLED 项：P_Ang_P / P_Ang_I / P_Ang_D）
#define TUNE_DEFAULT_PITCH_ANGLE_KP       (8.8f)   // 俯仰角度环比例增益：决定角度误差收敛速度，过大会俯仰震荡、过小会发飘
#define TUNE_DEFAULT_PITCH_ANGLE_KI       (0.04f)  // 俯仰角度环积分增益：消除角度静差（线缆拉力/重心偏移等造成的长期低头抬头）
#define TUNE_DEFAULT_PITCH_ANGLE_KD       (0.0f)   // 俯仰角度环微分增益：抑制超调，默认 0 禁用（角度环通常不用 D，会放大噪声）
#define TUNE_STEP_PITCH_ANGLE_KP          (0.10f)  // 菜单步长：每按一次 P_Ang_P ±0.10
#define TUNE_STEP_PITCH_ANGLE_KI          (0.05f)  // 菜单步长：每按一次 P_Ang_I ±0.05
#define TUNE_STEP_PITCH_ANGLE_KD          (0.001f) // 菜单步长：每按一次 P_Ang_D ±0.001

// ----- 俯仰(Pitch) 角速度环（内环） ----- （OLED 项：P_Gyr_P / P_Gyr_I / P_Gyr_D）
#define TUNE_DEFAULT_PITCH_GYRO_KP        (2.32f)  // 俯仰角速度环比例增益：姿态内环，直接决定抗风/抗扰能力和整体刚度
#define TUNE_DEFAULT_PITCH_GYRO_KI        (0.85f)  // 俯仰角速度环积分增益：消除稳态角速度误差，过大会低频摆动
#define TUNE_DEFAULT_PITCH_GYRO_KD        (0.09f)  // 俯仰角速度环微分增益(D on measurement)：抑制高频抖动，过大会放大陀螺噪声
#define TUNE_STEP_PITCH_GYRO_KP           (0.05f)  // 菜单步长：每按一次 P_Gyr_P ±0.05
#define TUNE_STEP_PITCH_GYRO_KI           (0.05f)  // 菜单步长：每按一次 P_Gyr_I ±0.05
#define TUNE_STEP_PITCH_GYRO_KD           (0.005f) // 菜单步长：每按一次 P_Gyr_D ±0.005

// ----- 横滚(Roll) 角度环 ----- （OLED 项：R_Ang_P / R_Ang_I / R_Ang_D）
#define TUNE_DEFAULT_ROLL_ANGLE_KP        (8.8f)   // 横滚角度环比例增益：作用同俯仰角度环，用于横滚轴
#define TUNE_DEFAULT_ROLL_ANGLE_KI        (0.04f)  // 横滚角度环积分增益：消除横滚轴角度静差
#define TUNE_DEFAULT_ROLL_ANGLE_KD        (0.0f)   // 横滚角度环微分增益：默认 0 禁用
#define TUNE_STEP_ROLL_ANGLE_KP           (0.10f)  // 菜单步长：每按一次 R_Ang_P ±0.10
#define TUNE_STEP_ROLL_ANGLE_KI           (0.05f)  // 菜单步长：每按一次 R_Ang_I ±0.05
#define TUNE_STEP_ROLL_ANGLE_KD           (0.001f) // 菜单步长：每按一次 R_Ang_D ±0.001

// ----- 横滚(Roll) 角速度环（内环） ----- （OLED 项：R_Gyr_P / R_Gyr_I / R_Gyr_D）
#define TUNE_DEFAULT_ROLL_GYRO_KP         (2.32f)  // 横滚角速度环比例增益：作用同俯仰角速度环，用于横滚轴
#define TUNE_DEFAULT_ROLL_GYRO_KI         (0.85f)  // 横滚角速度环积分增益：消除横滚轴稳态角速度误差
#define TUNE_DEFAULT_ROLL_GYRO_KD         (0.09f)  // 横滚角速度环微分增益(D on measurement)：抑制横滚轴高频抖动
#define TUNE_STEP_ROLL_GYRO_KP            (0.05f)  // 菜单步长：每按一次 R_Gyr_P ±0.05
#define TUNE_STEP_ROLL_GYRO_KI            (0.05f)  // 菜单步长：每按一次 R_Gyr_I ±0.05
#define TUNE_STEP_ROLL_GYRO_KD            (0.005f) // 菜单步长：每按一次 R_Gyr_D ±0.005

// ----- 偏航(Yaw) 角度环 ----- （OLED 项：Y_Ang_P / Y_Ang_I / Y_Ang_D）
#define TUNE_DEFAULT_YAW_ANGLE_KP         (3.21f)  // 偏航角度环比例增益：决定偏航转角速度，偏航无恢复力矩，一般比俯仰/横滚小
#define TUNE_DEFAULT_YAW_ANGLE_KI         (0.0f)   // 偏航角度环积分增益：默认 0 禁用（偏航积分易积累出缓慢漂移）
#define TUNE_DEFAULT_YAW_ANGLE_KD         (0.0f)   // 偏航角度环微分增益：默认 0 禁用
#define TUNE_STEP_YAW_ANGLE_KP            (0.05f)  // 菜单步长：每按一次 Y_Ang_P ±0.05
#define TUNE_STEP_YAW_ANGLE_KI            (0.001f) // 菜单步长：每按一次 Y_Ang_I ±0.001
#define TUNE_STEP_YAW_ANGLE_KD            (0.001f) // 菜单步长：每按一次 Y_Ang_D ±0.001

// ----- 偏航(Yaw) 角速度环（内环） ----- （OLED 项：Y_Gyr_P / Y_Gyr_I / Y_Gyr_D）
#define TUNE_DEFAULT_YAW_GYRO_KP          (3.5f)   // 偏航角速度环比例增益：偏航内环刚度，yaw 锁尾/扫场性能关键参数
#define TUNE_DEFAULT_YAW_GYRO_KI          (0.6f)   // 偏航角速度环积分增益：消除偏航稳态角速度误差（如电机扭矩不均）
#define TUNE_DEFAULT_YAW_GYRO_KD          (0.0f)   // 偏航角速度环微分增益：默认 0 禁用
#define TUNE_STEP_YAW_GYRO_KP             (0.05f)  // 菜单步长：每按一次 Y_Gyr_P ±0.05
#define TUNE_STEP_YAW_GYRO_KI             (0.02f)  // 菜单步长：每按一次 Y_Gyr_I ±0.02
#define TUNE_STEP_YAW_GYRO_KD             (0.001f) // 菜单步长：每按一次 Y_Gyr_D ±0.001

// ----- 姿态机械安装零偏（陀螺仪零位标定值） ----- （OLED 项：P_Zero / R_Zero）
// 作用：飞机用水平仪调平时 IMU 仍会读到的小角度残差（传感器安装不水平所致）。
//       在角度环误差计算中先减去该值：angle_error = 实际角度 - zero - 目标，
//       使飞机真正水平时角度误差为 0。已保存在 Flash 中时以 Flash 为准。
#define TUNE_DEFAULT_PITCH_ZERO_DEG       (0.68f)  // 俯仰机械零偏(deg)：水平放置时的俯仰角读数补偿
#define TUNE_DEFAULT_ROLL_ZERO_DEG        (-4.30f) // 横滚机械零偏(deg)：水平放置时的横滚角读数补偿

#define TUNE_STEP_PITCH_ZERO_DEG          (0.02f)  // 菜单步长：每按一次 P_Zero ±0.02 度（微调）
#define TUNE_STEP_ROLL_ZERO_DEG           (0.02f)  // 菜单步长：每按一次 R_Zero ±0.02 度（微调）

// ==================== 水平位置环与速度环 ====================
// 位置环：输入"目标位置-光流/视觉估计位置"的位置误差(cm)，输出"目标速度"(cm/s)。周期 20ms(50Hz)。
// 速度环：输入"目标速度-光流测量速度"的速度误差(cm/s)，输出"目标俯仰/横滚角"(deg) 作为角度环目标。周期 10ms(100Hz)。
//         注意：速度环输出限幅 ±12°，是水平控制能给出的最大姿态角指令。

// ----- 俯仰(Pitch) 水平位置环 ----- （OLED 项：P_Pos_P / P_Pos_I / P_Pos_D）
#define TUNE_DEFAULT_PITCH_POS_KP         (0.3f)   // 俯仰位置环比例增益：把前后方向的位置误差(cm)放大为目标速度(cm/s)，过大会来回冲
#define TUNE_DEFAULT_PITCH_POS_KI         (0.0f)   // 俯仰位置环积分增益：消除位置静差，默认 0 禁用（光流位置漂移时积分危险）
#define TUNE_DEFAULT_PITCH_POS_KD         (0.0f)   // 俯仰位置环微分增益：默认 0 禁用
#define TUNE_STEP_PITCH_POS_KP            (0.01f)  // 菜单步长：每按一次 P_Pos_P ±0.01
#define TUNE_STEP_PITCH_POS_KI            (0.001f) // 菜单步长：每按一次 P_Pos_I ±0.001
#define TUNE_STEP_PITCH_POS_KD            (0.001f) // 菜单步长：每按一次 P_Pos_D ±0.001

// ----- 俯仰(Pitch) 水平速度环 ----- （OLED 项：P_Vel_P / P_Vel_I / P_Vel_D）
#define TUNE_DEFAULT_PITCH_VEL_KP         (2.3f)  // 俯仰速度误差到期望水平加速度的比例增益，单位cm/s²/(cm/s)，与原0.16角度P近似等效
#define TUNE_DEFAULT_PITCH_VEL_KI         (0.005f) // 仅在零速稳定阶段允许积分，补偿长期静差
#define TUNE_DEFAULT_PITCH_VEL_KD         (0.0f)   // 俯仰速度环微分增益(D on measurement)：默认 0 禁用
#define TUNE_STEP_PITCH_VEL_KP            (0.1f)   // 菜单步长：每按一次P_Vel_P ±0.1 cm/s²/(cm/s)
#define TUNE_STEP_PITCH_VEL_KI            (0.001f)// 菜单步长：每按一次 P_Vel_I ±0.0001
#define TUNE_STEP_PITCH_VEL_KD            (0.001f) // 菜单步长：每按一次 P_Vel_D ±0.001

// ----- 横滚(Roll) 水平位置环 ----- （OLED 项：R_Pos_P / R_Pos_I / R_Pos_D）
#define TUNE_DEFAULT_ROLL_POS_KP          (0.3f)   // 横滚位置环比例增益：作用同俯仰位置环，用于左右方向
#define TUNE_DEFAULT_ROLL_POS_KI          (0.0f)   // 横滚位置环积分增益：默认 0 禁用
#define TUNE_DEFAULT_ROLL_POS_KD          (0.0f)   // 横滚位置环微分增益：默认 0 禁用
#define TUNE_STEP_ROLL_POS_KP             (0.01f)  // 菜单步长：每按一次 R_Pos_P ±0.01
#define TUNE_STEP_ROLL_POS_KI             (0.001f) // 菜单步长：每按一次 R_Pos_I ±0.001
#define TUNE_STEP_ROLL_POS_KD             (0.001f) // 菜单步长：每按一次 R_Pos_D ±0.001

// ----- 横滚(Roll) 水平速度环 ----- （OLED 项：R_Vel_P / R_Vel_I / R_Vel_D）
#define TUNE_DEFAULT_ROLL_VEL_KP          (2.3f)  // 横滚速度误差到期望水平加速度的比例增益，单位cm/s²/(cm/s)，与原0.16角度P近似等效
#define TUNE_DEFAULT_ROLL_VEL_KI          (0.005f) // 仅在零速稳定阶段允许积分
#define TUNE_DEFAULT_ROLL_VEL_KD          (0.0f)   // 横滚速度环微分增益(D on measurement)：默认 0 禁用
#define TUNE_STEP_ROLL_VEL_KP             (0.1f)   // 菜单步长：每按一次R_Vel_P ±0.1 cm/s²/(cm/s)
#define TUNE_STEP_ROLL_VEL_KI             (0.001f)// 菜单步长：每按一次 R_Vel_I ±0.0001
#define TUNE_STEP_ROLL_VEL_KD             (0.001f) // 菜单步长：每按一次 R_Vel_D ±0.001

// ==================== 高度位置环与速度环 ====================
// 高度位置环：输入"目标高度-TOF实际高度"的高度误差(mm)，输出"目标垂直速度"(cm/s)。周期 10ms(100Hz)。
// 高度速度环：输入"目标垂直速度-估计垂直速度"的速度误差(cm/s)，输出"油门 PWM 增量"(us) 叠加到总油门。周期 2ms(500Hz)。

// ----- 高度位置环 ----- （OLED 项：H_Pos_P / H_Pos_I / H_Pos_D）
#define TUNE_DEFAULT_HEIGHT_POS_KP        (0.30f)   // 高度位置环比例增益：高度误差(mm)→目标垂直速度(cm/s)，小幅加硬外环
#define TUNE_DEFAULT_HEIGHT_POS_KI        (0.002f) // 高度位置环积分增益：消除定高静差（气动/线缆拉力造成的持续高度偏差）
#define TUNE_DEFAULT_HEIGHT_POS_KD        (0.0f)   // 高度位置环微分增益：默认 0 禁用
#define TUNE_STEP_HEIGHT_POS_KP           (0.01f)  // 菜单步长：每按一次 H_Pos_P ±0.01
#define TUNE_STEP_HEIGHT_POS_KI           (0.0005f)// 菜单步长：每按一次 H_Pos_I ±0.0005
#define TUNE_STEP_HEIGHT_POS_KD           (0.001f) // 菜单步长：每按一次 H_Pos_D ±0.001

// ----- 高度速度环 ----- （OLED 项：H_Vel_P / H_Vel_I / H_Vel_D）
#define TUNE_DEFAULT_HEIGHT_VEL_KP        (0.45f)  // 高度速度环比例增益：垂直速度误差(cm/s)→油门增量(us)，决定垂直方向阻尼/刚度
#define TUNE_DEFAULT_HEIGHT_VEL_KI        (0.15f)  // 高度速度环积分增益：消除垂直方向稳态误差，过大会造成定高上下漂
#define TUNE_DEFAULT_HEIGHT_VEL_KD        (0.03f)  // 高度速度环微分增益(D on measurement)：抑制垂直速度突变，抑制油门抖动
#define TUNE_STEP_HEIGHT_VEL_KP           (0.01f)  // 菜单步长：每按一次 H_Vel_P ±0.01
#define TUNE_STEP_HEIGHT_VEL_KI           (0.01f)  // 菜单步长：每按一次 H_Vel_I ±0.01
#define TUNE_STEP_HEIGHT_VEL_KD           (0.005f) // 菜单步长：每按一次 H_Vel_D ±0.005

// ----- 定高目标高度 ----- （OLED 项：H_Target）
#define TUNE_DEFAULT_TARGET_HEIGHT_MM     (1200.0f) // 定高模式默认目标高度：1300mm = 1.3m（该高度下 170° 摄像头向下视角覆盖范围最佳）
#define TUNE_STEP_TARGET_HEIGHT_MM        (10.0f)  // 菜单步长：每按一次 H_Target ±10mm

// ==================== 视觉位置与速度前馈 ====================
// 视觉外环（10ms/100Hz）PID：输入为摄像头识别到的"目标相对画面中心的偏差"(cm)，
// 输出为"水平位置目标"(cm) 供水平位置环执行；同时把小车的运动速度做前馈叠加给水平速度环，
// 使用处：Vision.c 中 g_vision_params 结构体初始化 + VisionPID_Update()。
// 注意：max_speed 是遗留字段名（历史上叫"最大速度"），实际含义是"位置目标输出限幅"。

#define TUNE_DEFAULT_VISION_KP_X          (1.30f)  // 视觉 X 方向比例增益：左右共用
#define TUNE_DEFAULT_VISION_KP_Y          (1.30f)  // 视觉 Y 方向比例增益：前后共用
#define TUNE_DEFAULT_VISION_KI_X          (0.0f)   // 视觉 X 方向积分增益：补偿长期位置静差（线缆拉力/安装偏心），默认 0 禁用
#define TUNE_DEFAULT_VISION_KI_Y          (0.0f)   // 视觉 Y 方向积分增益：默认 0 禁用
#define TUNE_DEFAULT_VISION_KD_X          (0.0f)   // 视觉 X 方向微分增益：抑制超调，过大会放大摄像头识别噪声，默认 0 禁用
#define TUNE_DEFAULT_VISION_KD_Y          (0.0f)   // 视觉 Y 方向微分增益：默认 0 禁用
#define TUNE_DEFAULT_VISION_IMAX          (30.0f)  // 视觉 PID 积分项输出限幅(cm)：防止积分饱和，积分最多贡献 ±30cm 位置目标
#define TUNE_DEFAULT_VISION_MAX_SPEED     (300.0f) // 位置目标输出限幅(cm)：遗留字段名，实际限制位置目标的幅度上限（近/远端包络封顶）
#define TUNE_DEFAULT_VISION_MAX_STEP      (30.0f)  // 单周期位置目标最大变化量(cm)：输出斜率限制，防止换灯/误识别跳变导致位置指令突变
#define TUNE_DEFAULT_VISION_SLEW_RATE_CMS (560.0f)  // 视觉位置修正变化率限幅(cm/s)，不限制车速前馈
#define TUNE_DEFAULT_VISION_DEADZONE_CM   (8.0f)   // 误差死区(cm)：偏差绝对值小于 5cm 视为 0，避免目标居中时位置指令微抖

#define TUNE_DEFAULT_VISION_FF_RIGHT      (0.31f) // 机体系向右(X+)车速前馈比例
#define TUNE_DEFAULT_VISION_FF_LEFT       (0.31f) // 机体系向左(X-)车速前馈比例
#define TUNE_DEFAULT_VISION_FF_BACK       (0.31f) // 机体系向后(Y+)车速前馈比例
#define TUNE_DEFAULT_VISION_FF_FRONT      (0.31f) // 机体系向前(Y-)车速前馈比例
#define TUNE_DEFAULT_VISION_FF_NEAR_SCALE (0.29f / 0.31f) // 车距信标≤200cm或飞机距车≤10cm时，按方向前馈参数等比缩小
#define TUNE_DEFAULT_VISION_CMD_FF_SCALE  (3.8f) // 车端cmd到预测物理速度的统一比例，再乘现有四方向前馈增益

#define TUNE_DEFAULT_VISION_FF_MAX_SPEED  (1000.0f) // 前馈速度限幅(cm/s)：防止前馈量过大导致速度指令饱和

// ----- Vision 模糊增益调度 -----
// DISABLED：完全使用原固定PID；SHADOW：只计算并记录；ACTIVE：模糊输出直接接管。
#define VISION_FUZZY_MODE_DISABLED        (0u)
#define VISION_FUZZY_MODE_SHADOW          (1u)
#define VISION_FUZZY_MODE_ACTIVE          (2u)
#define TUNE_DEFAULT_VISION_FUZZY_MODE    VISION_FUZZY_MODE_ACTIVE
#define TUNE_DEFAULT_VISION_FUZZY_KP_MIN  (0.4f)  // 提高近处位置修正，减少小误差阶段被线缆持续拉着走///////////////////////////////
#define TUNE_DEFAULT_VISION_FUZZY_KP_MAX  (1.30f)  // 降低大误差追赶增益，减轻误差积累后的猛烈追回和过冲////////////////////////////////////
#define TUNE_DEFAULT_VISION_FUZZY_TD_MAX_S (0.12f) // 相对速度阻尼时间常数上限，降低阻尼压过位置P项的概率
#define TUNE_DEFAULT_VISION_FUZZY_DAMPING_MAX_CM (6.0f) // 限制模糊阻尼位置偏移，避免制动指令反向压过位置项
#define TUNE_DEFAULT_VISION_FUZZY_REL_VEL_DEADBAND_CMS (5.0f) // 飞机与小车相对速度死区，抑制同速跟随时的速度噪声
#define TUNE_DEFAULT_VISION_FUZZY_CENTER_EPS_CM (1.0f)

// 视觉位置修正与车速前馈同向时允许的最大额外追赶速度，防止飞机追过小车。
#define TUNE_DEFAULT_HVEL_FOLLOW_CATCHUP_MAX_CMS (15.0f)////////////////////////////////////////////////////////////////////////////////

// 小车速度前馈抗噪：先滤波和限速，再叠加到飞机速度目标
#define TUNE_DEFAULT_VISION_FF_FILTER_TAU_S       (0.10f) // 车速低通时间常数，单位 s
#define TUNE_DEFAULT_VISION_FF_ACCEL_RATE_CMS2    (600.0f) // 车速前馈加速变化率，单位 cm/s^2
#define TUNE_DEFAULT_VISION_FF_DECEL_RATE_CMS2    (1800.0f) // 车速前馈减速变化率，单位 cm/s^2
#define TUNE_DEFAULT_VISION_CAR_STOP_SPEED_CMS    (15.0f) // 滤波车速低于此值后开始确认停车，单位 cm/s
#define TUNE_DEFAULT_VISION_CAR_STOP_RELEASE_CMS  (20.0f) // 停车状态退出阈值，形成简单迟滞，单位 cm/s
#define TUNE_DEFAULT_VISION_CAR_STOP_CONFIRM_SAMPLES (2u) // 连续低速样本数，当前车速包周期约20 ms

// Vision 外环停车减速阻尼：车速减速时，用飞机融合速度换算为等效位置修正，抑制冲过头。
#define TUNE_DEFAULT_VISION_STOP_CAR_SPEED_CMS       (8.0f)
#define TUNE_DEFAULT_VISION_DECEL_MIN_DELTA_CMS      (5.0f)
#define TUNE_DEFAULT_VISION_STOP_DAMPING_TAU_X_S     (0.35f)
#define TUNE_DEFAULT_VISION_STOP_DAMPING_TAU_Y_S     (0.35f)
#define TUNE_DEFAULT_VISION_STOP_DAMPING_MAX_OFFSET_CM (20.0f)
#define TUNE_DEFAULT_VISION_STOP_DAMPING_MIN_VEL_CMS  (3.0f)

#define TUNE_STEP_VISION_KP_X             (0.01f)  // 菜单步长：每按一次 Vis_Kp_X ±0.01
#define TUNE_STEP_VISION_KP_Y             (0.01f)  // 菜单步长：每按一次 Vis_Kp_Y ±0.01
#define TUNE_STEP_VISION_KI_X             (0.001f) // 菜单步长：每按一次 Vis_Ki_X ±0.001
#define TUNE_STEP_VISION_KI_Y             (0.001f) // 菜单步长：每按一次 Vis_Ki_Y ±0.001
#define TUNE_STEP_VISION_KD_X             (0.001f) // 菜单步长：每按一次 Vis_Kd_X ±0.001
#define TUNE_STEP_VISION_KD_Y             (0.001f) // 菜单步长：每按一次 Vis_Kd_Y ±0.001
#define TUNE_STEP_VISION_IMAX             (1.0f)   // 菜单步长：每按一次 Vis_Imax ±1.0
#define TUNE_STEP_VISION_MAX_SPEED        (5.0f)   // 菜单步长：每按一次 Vis_Pmax ±5.0
#define TUNE_STEP_VISION_MAX_STEP         (1.0f)   // 菜单步长：每按一次 Vis_PStep ±1.0
#define TUNE_STEP_VISION_DEADZONE_CM      (1.0f)   // 菜单步长：每按一次 Vis_Dead ±1.0
#define TUNE_STEP_VISION_FF_X             (0.01f)  // 菜单步长：每按一次 Vis_FFx ±0.05
#define TUNE_STEP_VISION_FF_Y             (0.01f)  // 菜单步长：每按一次 Vis_FFy ±0.05
#define TUNE_STEP_VISION_FF_MAX_SPEED     (5.0f)   // 菜单步长：每按一次 Vis_FFMax ±5.0
#define TUNE_STEP_VISION_FUZZY_KP_MIN     (0.01f) // 模糊最小Kp菜单步长
#define TUNE_STEP_VISION_FUZZY_KP_MAX     (0.01f) // 模糊最大Kp菜单步长
#define TUNE_STEP_VISION_FUZZY_TD_MAX_S   (0.001f) // 阻尼时间常数菜单步长
#define TUNE_STEP_VISION_FUZZY_DAMPING_MAX_CM (0.1f) // 阻尼位置限幅菜单步长
#define TUNE_STEP_VISION_FUZZY_REL_VEL_DEADBAND_CMS (1.0f) // 相对速度死区菜单步长
#define TUNE_STEP_VISION_FUZZY_CENTER_EPS_CM (0.1f) // 中心误差阈值菜单步长

#endif
