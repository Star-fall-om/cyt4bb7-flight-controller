#include "Navigation.h"
#include <math.h>

// 全局坐标系位置（厘米）
static volatile float nav_x_cm = 0.0f;
static volatile float nav_y_cm = 250.0f;
// 全局坐标系速度（厘米/秒）
static volatile float nav_vx_cms = 0.0f;
static volatile float nav_vy_cms = 0.0f;
// 光流传感器测得的速度（厘米/秒）
static volatile float nav_flow_vx_cms = 0.0f;
static volatile float nav_flow_vy_cms = 0.0f;
static volatile float nav_flow_dt_s = 0.02f;
// 机体坐标系速度（厘米/秒）
static volatile float nav_body_x_cms = 0.0f;
static volatile float nav_body_y_cms = 0.0f;
// 偏航参考角度（度）
static volatile float nav_yaw_ref_deg = 0.0f;
static volatile uint32_t nav_epoch = 0u;
// 导航数据有效标志
static volatile uint8_t nav_valid = 0;
// 跟踪激活标志
static volatile uint8_t nav_tracking_active = 0;
// 导航初始化标志
static uint8_t s_nav_initialized = 0;
// 零速度计数器
static uint8_t s_nav_zero_speed_count = 0;

// 导航起始位置（厘米）
#define NAV_START_X_CM          (0.0f)
#define NAV_START_Y_CM          (250.0f)
// 导航位置范围限制（厘米）
#define NAV_X_MIN_CM            (-100.0f)
#define NAV_X_MAX_CM            (800.0f)
#define NAV_Y_MIN_CM            (-100.0f)
#define NAV_Y_MAX_CM            (600.0f)
// 速度衰减系数（失效时）
#define NAV_SPEED_DECAY         (0.80f)
// 速度归零阈值（厘米/秒）
#define NAV_SPEED_SNAP_CMS      (0.5f)
// 时间间隔容错值（秒）
#define NAV_DT_FALLBACK_S       (0.02f)
// 光流X/Y轴缩放系数
#define NAV_FLOW_SCALE_X        (1.0f)
#define NAV_FLOW_SCALE_Y        (1.0f)
// 光流安装偏航角（度）
#define NAV_FLOW_MOUNT_DEG      (0.0f)
// 机体坐标系速度偏差补偿（厘米/秒）
#define NAV_BODY_BIAS_X_CMS     (0.0f)
#define NAV_BODY_BIAS_Y_CMS     (20.0f)
// Y轴偏差补偿速度范围（厘米/秒）
#define NAV_Y_BIAS_START_SPEED_CMS (5.0f)
#define NAV_Y_BIAS_FULL_SPEED_CMS  (50.0f)
// 零速度判定阈值和帧数
#define NAV_ZERO_SPEED_CMS      (6.0f)
#define NAV_ZERO_SPEED_FRAMES   (5u)
// 角度转弧度系数
#define NAV_DEG_TO_RAD          (0.01745329252f)
// 搜索模式下解锁后积分测试开关
#define NAV_TEST_INTEGRATE_ON_ARMED_FOR_SEARCH (1)

// 限幅函数
static float Nav_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float Nav_WrapDeg(float angle)
{
    // 常见输入已在半周范围内，直接保留输入和正负 180° 的边界。
    if (angle >= -180.0f && angle <= 180.0f) {
        return angle;
    }

    // 先去掉完整圈数，再把余角归入 [-180°, 180°]。
    const float remainder_deg = fmodf(angle, 360.0f);
    if (remainder_deg > 180.0f) {
        return remainder_deg - 360.0f;
    }
    if (remainder_deg < -180.0f) {
        return remainder_deg + 360.0f;
    }
    return remainder_deg;
}

// 时间间隔合法性检查
static float Nav_SanitizeDt(float dt_s)
{
    if (dt_s < 0.005f || dt_s > 0.08f) {
        return NAV_DT_FALLBACK_S;
    }
    return dt_s;
}

// Y轴偏差补偿缩放系数（根据速度）
static float Nav_YBiasScale(float speed_cms)
{
    if (speed_cms <= NAV_Y_BIAS_START_SPEED_CMS) {
        return 0.0f;
    }
    if (speed_cms >= NAV_Y_BIAS_FULL_SPEED_CMS) {
        return 1.0f;
    }
    return (speed_cms - NAV_Y_BIAS_START_SPEED_CMS) /
           (NAV_Y_BIAS_FULL_SPEED_CMS - NAV_Y_BIAS_START_SPEED_CMS);
}

// 速度衰减函数（光流失效时）
static void Nav_DecayVelocity(void)
{
    nav_vx_cms *= NAV_SPEED_DECAY;
    nav_vy_cms *= NAV_SPEED_DECAY;
    if (fabsf(nav_vx_cms) < NAV_SPEED_SNAP_CMS) {
        nav_vx_cms = 0.0f;
    }
    if (fabsf(nav_vy_cms) < NAV_SPEED_SNAP_CMS) {
        nav_vy_cms = 0.0f;
    }
}

// 初始化跟踪模式导航
static void Nav_InitForTracking(float yaw_deg)
{
    nav_x_cm = NAV_START_X_CM;
    nav_y_cm = NAV_START_Y_CM;
    nav_vx_cms = 0.0f;
    nav_vy_cms = 0.0f;
    nav_yaw_ref_deg = yaw_deg;
    nav_epoch++;
    if (nav_epoch == 0u) {
        nav_epoch = 1u;
    }
    nav_valid = 1u;
    s_nav_initialized = 1u;
    s_nav_zero_speed_count = 0u;
    nav_flow_vx_cms = 0.0f;
    nav_flow_vy_cms = 0.0f;
    nav_flow_dt_s = NAV_DT_FALLBACK_S;
    nav_body_x_cms = 0.0f;
    nav_body_y_cms = 0.0f;
}

// 获取跟踪激活标志指针
volatile uint8_t *Navigation_GetTrackingActivePtr(void)
{
    return &nav_tracking_active;
}

// 根据光流数据更新导航位置和速度
// flow_right_cms: 光流测得的向右速度（cm/s）
// flow_back_cms: 光流测得的向后速度（cm/s）
// flow_valid: 光流数据有效标志
// dt_s: 时间间隔（秒）
// yaw_deg: 当前偏航角（度）
// lock_flag: 锁定标志（跟踪目标时）
// down_flag: 下降标志
// search_switch: 搜索模式开关
void Navigation_UpdateFromFlow(float flow_right_cms,
                               float flow_back_cms,
                               uint8_t flow_valid,
                               float dt_s,
                               float yaw_deg,
                               uint8_t lock_flag,
                               uint8_t down_flag,
                               uint8_t search_switch)
{
    float dt = Nav_SanitizeDt(dt_s);

    // 保存光流数据
    nav_flow_vx_cms = flow_right_cms;
    nav_flow_vy_cms = flow_back_cms;
    nav_flow_dt_s = dt;

    // 下降或搜索开关未激活时重置导航
#if NAV_TEST_INTEGRATE_ON_ARMED_FOR_SEARCH
    if (down_flag != 0 || search_switch < 50u) {
#else
    if (lock_flag != 0 || down_flag != 0 || search_switch < 50u) {
#endif
        nav_valid = 0u;
        s_nav_initialized = 0u;
        s_nav_zero_speed_count = 0u;
        nav_body_x_cms = 0.0f;
        nav_body_y_cms = 0.0f;
        Nav_DecayVelocity();
        return;
    }

    (void)lock_flag;

    // 跟踪未激活时进行速度衰减
    if (!nav_tracking_active) {
        nav_valid = s_nav_initialized ? 1u : 0u;
        s_nav_zero_speed_count = 0u;
        nav_body_x_cms = 0.0f;
        nav_body_y_cms = 0.0f;
        Nav_DecayVelocity();
        return;
    }

    // 首次跟踪时初始化导航
    if (!s_nav_initialized) {
        Nav_InitForTracking(yaw_deg);
    }

    // 光流失效时衰减速度
    if (!flow_valid) {
        s_nav_zero_speed_count = 0u;
        nav_body_x_cms = 0.0f;
        nav_body_y_cms = 0.0f;
        Nav_DecayVelocity();
        return;
    }

    // 光流有效，进行坐标变换和位置更新
    {
        // 光流速度转机体坐标系
        float raw_body_x_cms = flow_right_cms * NAV_FLOW_SCALE_X;
        float raw_body_y_cms = -flow_back_cms * NAV_FLOW_SCALE_Y;
        // 补偿光流安装角度
        float mount_rad = NAV_FLOW_MOUNT_DEG * NAV_DEG_TO_RAD;
        float mount_c = cosf(mount_rad);
        float mount_s = sinf(mount_rad);
        float body_x_cms = raw_body_x_cms * mount_c - raw_body_y_cms * mount_s;
        float body_y_cms = raw_body_x_cms * mount_s + raw_body_y_cms * mount_c;
        float speed_cms;
        float y_bias_scale;
        float yaw_delta_rad;
        float c;
        float s;
        float field_vx_cms;
        float field_vy_cms;

        // 计算速度并应用Y轴偏差补偿
        speed_cms = sqrtf(body_x_cms * body_x_cms + body_y_cms * body_y_cms);
        y_bias_scale = Nav_YBiasScale(speed_cms);
        body_x_cms -= NAV_BODY_BIAS_X_CMS;
        body_y_cms -= NAV_BODY_BIAS_Y_CMS * y_bias_scale;
        speed_cms = sqrtf(body_x_cms * body_x_cms + body_y_cms * body_y_cms);
        nav_body_x_cms = body_x_cms;
        nav_body_y_cms = body_y_cms;

        // 零速度检测
        if (speed_cms < NAV_ZERO_SPEED_CMS) {
            if (s_nav_zero_speed_count < NAV_ZERO_SPEED_FRAMES) {
                s_nav_zero_speed_count++;
            }
            if (s_nav_zero_speed_count >= NAV_ZERO_SPEED_FRAMES) {
                nav_body_x_cms = 0.0f;
                nav_body_y_cms = 0.0f;
                nav_vx_cms = 0.0f;
                nav_vy_cms = 0.0f;
                nav_valid = 1u;
                return;
            }
        } else {
            s_nav_zero_speed_count = 0u;
        }

        // 机体坐标系转全局坐标系
        yaw_delta_rad = Nav_WrapDeg(yaw_deg - nav_yaw_ref_deg) * NAV_DEG_TO_RAD;
        c = cosf(yaw_delta_rad);
        s = sinf(yaw_delta_rad);
        field_vx_cms = body_x_cms * c + body_y_cms * s;
        field_vy_cms = -body_x_cms * s + body_y_cms * c;

        // 更新速度和位置
        nav_vx_cms = field_vx_cms;
        nav_vy_cms = field_vy_cms;
        nav_x_cm = Nav_Clamp(nav_x_cm + field_vx_cms * dt,
                             NAV_X_MIN_CM,
                             NAV_X_MAX_CM);
        nav_y_cm = Nav_Clamp(nav_y_cm + field_vy_cms * dt,
                             NAV_Y_MIN_CM,
                             NAV_Y_MAX_CM);
        nav_valid = 1u;
    }
}

// 填充共享数据结构
void Navigation_FillSharedData(volatile SharedData_Down_t *data)
{
    data->nav_x_cm = nav_x_cm;
    data->nav_y_cm = nav_y_cm;
    data->nav_vx_cms = nav_vx_cms;
    data->nav_vy_cms = nav_vy_cms;
    data->nav_flow_vx_cms = nav_flow_vx_cms;
    data->nav_flow_vy_cms = nav_flow_vy_cms;
    data->nav_flow_dt_ms = nav_flow_dt_s * 1000.0f;
    data->nav_body_x_cms = nav_body_x_cms;
    data->nav_body_y_cms = nav_body_y_cms;
    data->nav_yaw_ref_deg = nav_yaw_ref_deg;
    data->nav_epoch = nav_epoch;
    data->nav_valid = nav_valid;
}
