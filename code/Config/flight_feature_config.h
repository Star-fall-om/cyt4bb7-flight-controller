/**
 * @file flight_feature_config.h
 * @brief 飞机视觉、跟踪与任务功能的统一宏配置
 *
 * 修改说明：
 * 1. “用户可修改配置”区域是日常切换功能时唯一需要修改的位置。
 * 2. “模式常量”和“派生宏”由代码内部使用，不要直接修改派生宏。
 * 3. 本文件同时供 M7_0 与 M7_1 使用，修改后需要成对更新两核固件。
 */

#ifndef FLIGHT_FEATURE_CONFIG_H
#define FLIGHT_FEATURE_CONFIG_H

/* ==================== 模式常量：不要修改 ==================== */



/* 小车位置融合状态，仅供派生宏使用。 */
#define M71_CAR_POSE_FUSION_OFF               0u
#define M71_CAR_POSE_FUSION_ACTIVE            2u



/* ==================== 用户可修改配置 ==================== */

/**
 * 是否根据车端惯导位置选择灭灯后的首次 yaw 扫描方向。
 * 0：关闭，先回到最近的场地主轴，再按默认方向摆扫。
 * 1：开启，回到最近主轴后，优先朝场地中心所在一侧开始摆扫。
 */
#define VIS_YAW_CENTER_SCAN_ENABLE             0u

/**
 * yaw 搜索和回轴每个视觉控制周期使用的目标角增量，单位为度。
 * 搜索步进越大，摆扫速度越快；回轴步进较小可降低回到主轴时的突变。
 */
#define VIS_YAW_SEARCH_STEP                    2.0f  /* 启用中心扫描时的偏航步长，单位为度。 */
#define VIS_YAW_AXIS_RETURN_STEP               0.6f

/**
 * 进入信标模式后的小车入场旋转。
 * M70 和 M71 都从 ready_flag 的首次置位开始各自计时：
 *   - M70 在等待期间保持当前 yaw，不进入原有信标搜索；
 *   - M71 向小车发送零位移和递增的虚拟 yaw。
 * 7 秒结束后两核分别恢复原有搜索、跟随与任务流程，不新增双核字段。
 */
#define MISSION_ENTRY_CAR_YAW_SWEEP_ENABLE     0u
#define MISSION_ENTRY_CAR_YAW_WAIT_MS          4000u
#define MISSION_ENTRY_CAR_YAW_STEP_DEG         4.2f

/**
 * 场地主轴对应的实际 IMU yaw，单位为度。
 * 该角度增加或减少 180 度仍表示同一条场地主轴。
 */
#define VIS_YAW_SEARCH_AXIS_DEG                (0.0f)

/**
 * 主轴两侧的最大搜索角度，单位为度。
 * 运行时会取绝对值并限制在 0～90 度范围内。
 */
#define VIS_YAW_SEARCH_HALF_RANGE_DEG          90.0f

/**
 * 灭灯后首次向场地中心摆扫时的方向符号。
 * 1：保持当前方向定义；-1：当首次摆扫远离场地中心时翻转方向。
 */
#define VIS_YAW_CENTER_SCAN_DIR_SIGN           (-1)



/* 场地坐标过滤模式。 */
#define M71_FIELD_FILTER_OFF                  0u  /* 不使用场地边界过滤。 */
#define M71_FIELD_FILTER_CAR_ODOM             1u  /* 使用小车里程计场地坐标过滤。 */
#define M71_FIELD_FILTER_CAR_DRONE_FUSION     2u  /* 使用小车与飞机融合坐标过滤。 */
/**
 * 场地坐标过滤模式。
 * M71_FIELD_FILTER_OFF：关闭场地边界过滤。
 * M71_FIELD_FILTER_CAR_ODOM：使用小车里程计位置过滤。
 * M71_FIELD_FILTER_CAR_DRONE_FUSION：融合小车与飞机位置后过滤。
 */
#define M71_FIELD_FILTER_MODE                  M71_FIELD_FILTER_OFF


/* 小车速度前馈来源。 */
#define CAR_SPEED_FF_SOURCE_VISUAL             0u  /* 只使用视觉差分估计车速。 */
#define CAR_SPEED_FF_SOURCE_UART               1u  /* 只使用小车串口上报车速。 */
#define CAR_SPEED_FF_SOURCE_AUTO               2u  /* UART优先，UART无效时回退视觉估速。 */
/**
 * 小车速度前馈来源。
 * 当前 AUTO 模式优先采用 UART 车速；UART 超时或无效时回退视觉估速。
 */
#define CAR_SPEED_FF_SOURCE                    CAR_SPEED_FF_SOURCE_AUTO

/**
 * 灭灯接近阶段是否提前请求 yaw 回到场地主轴。
 * 0：关闭，保持发现信标时的 yaw，丢灯后再进入正常搜索。
 * 1：开启，满足任务状态机距离和帧数条件后提前回轴。
 */
#define AXIS_RETURN_ENABLE                     0u

/* 完成任务需要确认熄灭的信标数量。 */
#define MISSION_COMPLETE_COUNT                 10u

/**
 * 达到任务完成数量后是否进入确认搜索、车辆撤离和自动降落流程。
 * 0：关闭自动完成流程，仅保留灭灯计数。
 * 1：开启确认搜索、车辆撤离和自动降落请求。
 */
#define MISSION_AUTO_LAND_ENABLE               0u




/* ==================== 派生宏：无需手动修改 ==================== */

/* 只要场地过滤模式不是 OFF，就编译场地边界过滤链路。 */
#define M71_CAR_POSE_FIELD_FILTER_ENABLE \
    (M71_FIELD_FILTER_MODE != M71_FIELD_FILTER_OFF)

/* 仅融合模式启用小车/飞机位置融合，其余模式保持关闭。 */
#define M71_CAR_POSE_FUSION_MODE \
    ((M71_FIELD_FILTER_MODE == M71_FIELD_FILTER_CAR_DRONE_FUSION) ? \
        M71_CAR_POSE_FUSION_ACTIVE : M71_CAR_POSE_FUSION_OFF)

/* ==================== 配置合法性检查 ==================== */

#if (VIS_YAW_CENTER_SCAN_ENABLE != 0u) && \
    (VIS_YAW_CENTER_SCAN_ENABLE != 1u)
#error Invalid VIS_YAW_CENTER_SCAN_ENABLE
#endif

#if (VIS_YAW_CENTER_SCAN_DIR_SIGN != 1) && \
    (VIS_YAW_CENTER_SCAN_DIR_SIGN != -1)
#error Invalid VIS_YAW_CENTER_SCAN_DIR_SIGN
#endif

#if (MISSION_ENTRY_CAR_YAW_SWEEP_ENABLE != 0u) && \
    (MISSION_ENTRY_CAR_YAW_SWEEP_ENABLE != 1u)
#error Invalid MISSION_ENTRY_CAR_YAW_SWEEP_ENABLE
#endif

#if (MISSION_ENTRY_CAR_YAW_SWEEP_ENABLE == 1u) && \
    (MISSION_ENTRY_CAR_YAW_WAIT_MS < 1u)
#error Invalid MISSION_ENTRY_CAR_YAW_WAIT_MS
#endif

#if (M71_FIELD_FILTER_MODE != M71_FIELD_FILTER_OFF) && \
    (M71_FIELD_FILTER_MODE != M71_FIELD_FILTER_CAR_ODOM) && \
    (M71_FIELD_FILTER_MODE != M71_FIELD_FILTER_CAR_DRONE_FUSION)
#error Invalid M71_FIELD_FILTER_MODE
#endif

#if (CAR_SPEED_FF_SOURCE != CAR_SPEED_FF_SOURCE_VISUAL) && \
    (CAR_SPEED_FF_SOURCE != CAR_SPEED_FF_SOURCE_UART) && \
    (CAR_SPEED_FF_SOURCE != CAR_SPEED_FF_SOURCE_AUTO)
#error Invalid CAR_SPEED_FF_SOURCE
#endif

#if (AXIS_RETURN_ENABLE != 0u) && (AXIS_RETURN_ENABLE != 1u)
#error Invalid AXIS_RETURN_ENABLE
#endif

#if (MISSION_AUTO_LAND_ENABLE != 0u) && \
    (MISSION_AUTO_LAND_ENABLE != 1u)
#error Invalid MISSION_AUTO_LAND_ENABLE
#endif

#endif
