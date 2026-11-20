#include "TOF.h"
#include "zf_device_dl1b.h"
#include "zf_device_dl1b2.h"
#include "zf_device_dl1b3.h"
#include "Algorithm/PID/pid.h"
#include <math.h>

// ==========================================
// TOF 防扫线优化参数定义
// ==========================================
// 防扫线缆：双传感器差异阈值 (mm)
// 两个TOF平行向下后，平坦地面上任意roll角度两者读数应一致，差异=障碍物
#define TOF_CABLE_MIN_DISCREPANCY   200.0f  // 一个传感器比另一个近这么多 → 疑似线缆
#define TOF_CABLE_BELOW_TRUSTED     300.0f  // 且比trusted_Z低这么多 → 确认扫到线缆/障碍物

#define TOF_TILT_DIAGNOSTIC_FACTOR 0.85f  // 仅用于区分遥测中的大角度几何补偿模式

// ==========================================
// 局部静态状态变量
// ==========================================
#ifndef DEG2RAD
#define DEG2RAD                 (0.01745329251994329577f)
#endif

#define TOF_HEIGHT_MAX_MM           4000.0f
#define TOF_DL1B3_MIN_MM            30.0f
#define TOF_DL1B3_MAX_MM            4000.0f
#define TOF_DL1B3_MIN_VALID_ZONES   4U

// 4x4 各区视线与光轴夹角的余弦（45° FOV，区间距 11.25°）：
// 角部区 16.9°x16.9°→0.919，边中区→0.953，中心区→0.990。
// 测距值是沿视线的斜距，乘以本表换算为光轴方向分量，
// 消除"只取四周区"带来的 5%~9% 系统性偏长（1.3m 高度时约 60~110mm）
// 按4x4区域行优先顺序排列的单位光线方向，x为机头方向，y为机体左侧，z为向下。
// z分量沿用原有区域余弦，保证水平状态下新旧高度结果一致。
static const float tof_dl1b3_ray_x[16] = {
    -0.272582f, -0.093443f,  0.093443f,  0.272582f,
    -0.287798f, -0.097508f,  0.097508f,  0.287798f,
    -0.287798f, -0.097508f,  0.097508f,  0.287798f,
    -0.272582f, -0.093443f,  0.093443f,  0.272582f
};

static const float tof_dl1b3_ray_y[16] = {
    -0.284847f, -0.289190f, -0.289190f, -0.284847f,
    -0.097648f, -0.097980f, -0.097980f, -0.097648f,
     0.097648f,  0.097980f,  0.097980f,  0.097648f,
     0.284847f,  0.289190f,  0.289190f,  0.284847f
};

static const float tof_dl1b3_ray_z[16] = {
    0.9190f, 0.9527f, 0.9527f, 0.9190f,
    0.9527f, 0.9904f, 0.9904f, 0.9527f,
    0.9527f, 0.9904f, 0.9904f, 0.9527f,
    0.9190f, 0.9527f, 0.9527f, 0.9190f
};

static float trusted_Z = 1000.0f;

uint16_t tof_blind_timer = 0;
volatile float height = 0.0f; // 全局输出高度 (mm)
float tof_dl1b3_ground_mm = 8192.0f;
uint8_t tof_dl1b3_valid_count = 0;
uint16_t tof_dl1b3_edge_mask = 0;
volatile tof_debug_t tof_debug = {0};

// ==========================================
// 内部工具函数
// ==========================================
static uint8_t TOF_IsFinite(float value)
{
    return (!isnan(value) && !isinf(value));
}

// 强制重置所有状态，彻底防止微分死锁
static void TOF_Reset_States(float new_Z)
{
    if(!TOF_IsFinite(new_Z)) {
        new_Z = 100.0f;
    }
    new_Z = constrain_float(new_Z, 0.0f, TOF_HEIGHT_MAX_MM);

    trusted_Z = new_Z;
    tof_blind_timer = 0;
}

static float TOF_TiltFactor(void)
{
    return cosf(pitch_filtered * DEG2RAD) * cosf(roll_filtered * DEG2RAD);
}

static float TOF_GetZoneVerticalFactor(uint8_t zone)
{
    float pitch_rad = pitch_filtered * DEG2RAD;
    float roll_rad = roll_filtered * DEG2RAD;
    float sp = sinf(pitch_rad);
    float cp = cosf(pitch_rad);
    float sr = sinf(roll_rad);
    float cr = cosf(roll_rad);

    // pitch后倾为正、roll左倾为正，计算每个区域光线在世界竖直方向的分量。
    return (-tof_dl1b3_ray_x[zone] * sp * cr)
         + ( tof_dl1b3_ray_y[zone] * sr)
         + ( tof_dl1b3_ray_z[zone] * cp * cr);
}

static uint8_t TOF_IsValidMm(float distance_mm)
{
    return (TOF_IsFinite(distance_mm)
            && distance_mm > TOF_DL1B3_MIN_MM
            && distance_mm < TOF_DL1B3_MAX_MM);
}

static uint8_t TOF_Dl1b3StatusValid(uint8_t status)
{
    return (status == 5U || status == 9U);
}

static void TOF_SortFloat(float *data, uint8_t count)
{
    for(uint8_t i = 0; i + 1U < count; i++) {
        for(uint8_t j = 0; j + 1U < count - i; j++) {
            if(data[j] > data[j + 1U]) {
                float t = data[j];
                data[j] = data[j + 1U];
                data[j + 1U] = t;
            }
        }
    }
}

static uint8_t TOF_GetDl1b3Ground(float *ground_mm)
{
    float compensated_samples[16];
    float raw_samples[16];
    uint8_t zone_limit = (dl1b3_zone_count < 16U) ? dl1b3_zone_count : 16U;
    uint8_t valid_count = 0U;
    float min_vertical_factor = 10.0f;
    float max_vertical_factor = -10.0f;
    tof_dl1b3_valid_count = 0U;
    tof_dl1b3_edge_mask = 0U;
    tof_debug.dl1b3_raw_median_mm = 0.0f;
    tof_debug.dl1b3_zone_cos_median_mm = 0.0f;
    tof_debug.dl1b3_min_height_mm = 0.0f;
    tof_debug.dl1b3_max_height_mm = 0.0f;
    tof_debug.dl1b3_spread_mm = 0.0f;
    tof_debug.min_vertical_factor = 0.0f;
    tof_debug.max_vertical_factor = 0.0f;

    if(!dl1b3_frame_ready) {
        return 0U;
    }

    tof_dl1b3_ground_mm = 8192.0f;
    if(zone_limit < TOF_DL1B3_MIN_VALID_ZONES) {
        return 0U;
    }

    for(uint8_t i = 0U; i < zone_limit; i++) {
        float raw_mm = (float)dl1b3_distance_mm[i];
        float vertical_factor = TOF_GetZoneVerticalFactor(i);
        float compensated_mm = raw_mm * vertical_factor;

        if(!TOF_Dl1b3StatusValid(dl1b3_target_status[i])
           || vertical_factor <= 0.0f
           || !TOF_IsValidMm(compensated_mm)) {
            continue;
        }
        if(vertical_factor < min_vertical_factor) {
            min_vertical_factor = vertical_factor;
        }
        if(vertical_factor > max_vertical_factor) {
            max_vertical_factor = vertical_factor;
        }
        compensated_samples[valid_count] = compensated_mm;
        raw_samples[valid_count] = raw_mm;
        tof_dl1b3_edge_mask |= (uint16_t)(1U << i);
        valid_count++;
    }

    if(valid_count < TOF_DL1B3_MIN_VALID_ZONES) {
        tof_debug.reject_reason |= TOF_REJECT_VALID_COUNT;
        return 0U;
    }

    TOF_SortFloat(compensated_samples, valid_count);
    TOF_SortFloat(raw_samples, valid_count);
    tof_debug.dl1b3_raw_median_mm = raw_samples[valid_count / 2U];
    tof_debug.dl1b3_zone_cos_median_mm = compensated_samples[valid_count / 2U];
    tof_dl1b3_valid_count = valid_count;
    tof_dl1b3_ground_mm = compensated_samples[valid_count / 2U];
    tof_debug.dl1b3_min_height_mm = compensated_samples[0U];
    tof_debug.dl1b3_max_height_mm = compensated_samples[valid_count - 1U];
    tof_debug.dl1b3_spread_mm = tof_debug.dl1b3_max_height_mm
                              - tof_debug.dl1b3_min_height_mm;
    tof_debug.min_vertical_factor = min_vertical_factor;
    tof_debug.max_vertical_factor = max_vertical_factor;
    *ground_mm = tof_dl1b3_ground_mm;
    return 1U;
}

#if 0
static uint8_t TOF_GetDl1b3Ground_Legacy(float *ground_mm)
{
    float zone_mm[16];
    uint8_t zone_valid[16];
    float samples[16];
    float ref_samples[16];
    float raw_distance_samples[16];
    uint8_t zone_limit = (dl1b3_zone_count < 16U) ? dl1b3_zone_count : 16U;
    float tilt_factor = TOF_TiltFactor();
    uint8_t raw_valid_count = 0U;
    float ground_ref_mm;
    float car_drop_threshold;

    if(!dl1b3_frame_ready) {
        if(tof_dl1b3_hold_count > 0U
           && tilt_factor >= TOF_DL1B3_MIN_TILT_FACTOR
           && TOF_IsValidMm(tof_dl1b3_ground_mm)) {
            tof_dl1b3_hold_count--;
            *ground_mm = tof_dl1b3_ground_mm;
            return 1U;
        }
        return 0U;
    }

    tof_dl1b3_ground_mm = 8192.0f;
    tof_dl1b3_valid_count = 0U;
    tof_dl1b3_edge_mask = 0U;

    if(zone_limit < 16U || tilt_factor < TOF_DL1B3_MIN_TILT_FACTOR) {
        tof_dl1b3_hold_count = 0U;
        return 0U;
    }

    for(uint8_t i = 0; i < 16U; i++) {
        zone_mm[i] = 8192.0f;
        zone_valid[i] = 0U;
        raw_distance_samples[i] = 8192.0f;
    }

    for(uint8_t i = 0; i < zone_limit; i++) {
        float raw_mm;
        float vertical_mm;

        // 斜距 × 区角余弦 = 光轴方向分量（仅适用于 4x4 模式）
        raw_mm = (float)dl1b3_distance_mm[i] * tof_dl1b3_zone_cos[i];
        if(!TOF_Dl1b3StatusValid(dl1b3_target_status[i]) || !TOF_IsValidMm(raw_mm)) {
            continue;
        }

        vertical_mm = raw_mm * tilt_factor;
        if(!TOF_IsValidMm(vertical_mm)) {
            continue;
        }

        zone_mm[i] = vertical_mm;
        zone_valid[i] = 1U;
        raw_distance_samples[raw_valid_count] = (float)dl1b3_distance_mm[i];
        ref_samples[raw_valid_count] = vertical_mm;
        raw_valid_count++;
    }

    // 至少 4 格才输出：盖板串扰下实测出现过"仅 2 格有效、读数减半"的鬼影帧
    // (掩码 0003/1001/2001，611~763mm)，格数过少宁可走保持/盲区估计
    if(raw_valid_count < 4U) {
        tof_debug.reject_reason |= TOF_REJECT_VALID_COUNT;
        tof_dl1b3_hold_count = 0U;
        return 0U;
    }

    TOF_SortFloat(ref_samples, raw_valid_count);
    TOF_SortFloat(raw_distance_samples, raw_valid_count);
    tof_debug.dl1b3_raw_median_mm = raw_distance_samples[raw_valid_count / 2U];
    tof_debug.dl1b3_zone_cos_median_mm = ref_samples[raw_valid_count / 2U];
    if(raw_valid_count >= 5U) {
        uint8_t start = raw_valid_count / 2U;
        uint8_t len = raw_valid_count - start;
        ground_ref_mm = ref_samples[start + len / 2U];
    } else {
        ground_ref_mm = (ref_samples[raw_valid_count - 1U]
                       + ref_samples[raw_valid_count - 2U]) * 0.5f;
    }

    car_drop_threshold = TOF_Dl1b3CarDropThreshold(ground_ref_mm);

    for(uint8_t i = 0; i < zone_limit; i++) {
        if(!zone_valid[i]) {
            continue;
        }

        if((ground_ref_mm - zone_mm[i]) > car_drop_threshold) {
            continue;
        }

        samples[tof_dl1b3_valid_count] = zone_mm[i];
        tof_dl1b3_edge_mask |= (uint16_t)(1U << i);
        tof_dl1b3_valid_count++;
    }

    if(tof_dl1b3_valid_count < 4U) {
        tof_debug.reject_reason |= TOF_REJECT_VALID_COUNT;
        tof_dl1b3_hold_count = 0U;
        return 0U;
    }

    TOF_SortFloat(samples, tof_dl1b3_valid_count);

    if(tof_dl1b3_valid_count >= 5U) {
        uint8_t start = tof_dl1b3_valid_count / 2U;
        uint8_t len = tof_dl1b3_valid_count - start;
        tof_dl1b3_ground_mm = samples[start + len / 2U];
    } else {
        tof_dl1b3_ground_mm = (samples[tof_dl1b3_valid_count - 1U]
                             + samples[tof_dl1b3_valid_count - 2U]) * 0.5f;
    }

    *ground_mm = tof_dl1b3_ground_mm;
    tof_debug.compensated_measurement_mm = tof_dl1b3_ground_mm;
    tof_dl1b3_hold_count = TOF_DL1B3_HOLD_CYCLES;
    return 1U;
}

#endif

static uint8_t TOF_GetDl1bHeight(float *height_mm)
{
    float tilt_factor = TOF_TiltFactor();
    float raw_mm = (float)dl1b_distance_mm;
    tof_debug.dl1b_raw_mm = raw_mm;

    if(!TOF_IsValidMm(raw_mm)) {
        return 0U;
    }

    *height_mm = raw_mm * tilt_factor;
    tof_debug.compensated_measurement_mm = *height_mm;
    return TOF_IsValidMm(*height_mm);
}

// ==========================================
// 接口实现
// ==========================================
#define TOF_INIT_TARGET_SAMPLES     (10U)
#define TOF_INIT_MAX_LOOPS          (40)

static uint8_t TOF_TryDl1b3Init(void)
{
    static const uint32 dl1b3_iic_delays[] = { DL1B3_SOFT_IIC_DELAY, 50U, 100U };
    uint8 i;

    for (i = 0U; i < 3U; i++) {
        if (0U == dl1b3_init_with_iic_delay(dl1b3_iic_delays[i])) {
            printf("dl1b3 init ok (iic_delay=%lu)\r\n", (unsigned long)dl1b3_iic_delays[i]);
            return 1U;
        }
        printf("dl1b3 init fail step=%d status=%d (delay=%lu try %u/3)\r\n",
               dl1b3_init_step, dl1b3_last_status,
               (unsigned long)dl1b3_iic_delays[i], (unsigned)(i + 1U));
        dl1b3_stop();
        system_delay_ms(50);
    }
    return 0U;
}

void TOF_Init(void)
{
    uint32_t t0 = GetSystemTimer();

    dl1b_init();
#if TOF_SENSOR_MODE == TOF_SENSOR_MODE_OLD_PAIR
    dl1b2_init();
#else
    (void)TOF_TryDl1b3Init();
#endif
    printf("TOF sensor init: %lu us\r\n", (unsigned long)(GetSystemTimer() - t0));

    float init_sum = 0.0f;
    uint8_t init_cnt = 0;

    for (int i = 0; i < TOF_INIT_MAX_LOOPS && init_cnt < TOF_INIT_TARGET_SAMPLES; i++) {
        dl1b_get_distance();

#if TOF_SENSOR_MODE == TOF_SENSOR_MODE_OLD_PAIR
        dl1b2_get_distance();

        uint8_t valid_R = TOF_IsValidMm((float)dl1b_distance_mm);
        uint8_t valid_L = TOF_IsValidMm((float)dl1b2_distance_mm);

        if (valid_L && valid_R) {
            init_sum += ((float)dl1b_distance_mm + (float)dl1b2_distance_mm) * 0.5f;
            init_cnt++;
        } else if (valid_R) {
            init_sum += (float)dl1b_distance_mm;
            init_cnt++;
        } else if (valid_L) {
            init_sum += (float)dl1b2_distance_mm;
            init_cnt++;
        }
#else
        dl1b3_poll();

        float z_L = 0.0f;
        uint8_t valid_L = TOF_GetDl1b3Ground(&z_L);

        if (valid_L) {
            init_sum += z_L;
            init_cnt++;
        }
#endif
        system_delay_ms(2);
    }

    if (init_cnt > 0U) {
        TOF_Reset_States(init_sum / (float)init_cnt);
    } else {
        TOF_Reset_States(100.0f);
    }
    printf("TOF warmup: %lu us (%u samples)\r\n",
           (unsigned long)(GetSystemTimer() - t0), (unsigned)init_cnt);
}

void TOF_Process(void)
{
    float z_dl1b = 0.0f;
    float z_tof3 = 0.0f;
    float predicted_Z;
    float tilt_factor = TOF_TiltFactor();
    uint8_t valid_dl1b;
    uint8_t valid_tof3;
    uint8_t measurement_accepted = 0U;

    tof_debug.pitch_deg = pitch_filtered;
    tof_debug.roll_deg = roll_filtered;
    tof_debug.pitch_rate_dps = pitch_gyro_filtered;
    tof_debug.roll_rate_dps = roll_gyro_filtered;
    tof_debug.tilt_factor = tilt_factor;
    tof_debug.measure_available = 0U;
    tof_debug.accepted = 0U;
    tof_debug.hold_active = 0U;
    tof_debug.recovery_active = 0U;
    tof_debug.recovery_count = 0U;
    tof_debug.relock_streak = 0U;
    tof_debug.reject_reason = 0U;
    tof_debug.compensated_measurement_mm = 0.0f;
    tof_debug.innovation_mm = 0.0f;
    tof_debug.mode = TOF_MODE_PREDICT;
    tof_debug.using_tof3 = 0U;
    tof_debug.using_prediction = 0U;
    tof_debug.cable_suspect = 0U;

    dl1b_get_distance();
    dl1b3_poll();
    valid_dl1b = TOF_GetDl1bHeight(&z_dl1b);
    valid_tof3 = TOF_GetDl1b3Ground(&z_tof3);
    tof_debug.measure_available = valid_tof3;

    /* DL1B只参与线缆/遮挡判断，不再参与高度融合。 */
    if(valid_dl1b && valid_tof3
       && (z_dl1b + TOF_CABLE_MIN_DISCREPANCY < z_tof3)
       && ((trusted_Z - z_dl1b) > TOF_CABLE_BELOW_TRUSTED)) {
        tof_debug.reject_reason |= TOF_REJECT_CABLE;
        tof_debug.cable_suspect = 1U;
    }
    if(!valid_tof3) {
        tof_debug.reject_reason |= TOF_REJECT_NO_MEASUREMENT;
    }
    if(valid_tof3) {
        tof_debug.compensated_measurement_mm = z_tof3;
        tof_debug.innovation_mm = z_tof3 - trusted_Z;
    } else {
        tof_debug.compensated_measurement_mm = 0.0f;
    }

    predicted_Z = trusted_Z + vertical_velocity_smoothed * 0.1f;
    predicted_Z = constrain_float(predicted_Z, 0.0f, TOF_HEIGHT_MAX_MM);

    if(valid_tof3) {
        // TOF3是高度主测量。只要当前帧有效，真实的高/低高度变化都立即发布。
        trusted_Z = z_tof3;
        tof_blind_timer = 0U;
        measurement_accepted = 1U;
        tof_debug.mode = (tilt_factor < TOF_TILT_DIAGNOSTIC_FACTOR)
                       ? TOF_MODE_GEOMETRIC : TOF_MODE_NORMAL;
        tof_debug.using_tof3 = 1U;
    } else {
        // 仅在TOF3当前帧无效时使用垂直速度预测，不进行长时间重锁或历史高度冻结。
        if(tof_blind_timer < 0xFFFFU) {
            tof_blind_timer++;
        }
        trusted_Z = predicted_Z;
        tof_debug.using_prediction = 1U;
    }

    if(measurement_accepted) {
        tof_debug.accepted = 1U;
    }
    trusted_Z = constrain_float(trusted_Z, 0.0f, TOF_HEIGHT_MAX_MM);
    height = trusted_Z;
    tof_debug.trusted_height_mm = trusted_Z;
    tof_debug.blind_timer = tof_blind_timer;

    correct_vertical_velocity(height, 0.01f);
}

#if 0
void TOF_Process(void)
{
    float z_R = 0.0f;
    float z_L = 0.0f;
    uint8_t valid_R = 0U;
    uint8_t valid_L = 0U;
    float raw_measured_Z = 0.0f;
    uint8_t measure_available = 0U;
    float tilt_factor = TOF_TiltFactor();
    float angle_rate_dps = fabsf(pitch_gyro_filtered) + fabsf(roll_gyro_filtered);
    uint8_t motion_hold = (uint8_t)((tilt_factor < TOF_TILT_HOLD_FACTOR)
                                    || (angle_rate_dps > TOF_ANGLE_RATE_HOLD_DPS));

    tof_debug.pitch_deg = pitch_filtered;
    tof_debug.roll_deg = roll_filtered;
    tof_debug.pitch_rate_dps = pitch_gyro_filtered;
    tof_debug.roll_rate_dps = roll_gyro_filtered;
    tof_debug.tilt_factor = tilt_factor;
    tof_debug.measure_available = 0U;
    tof_debug.accepted = 0U;
    tof_debug.hold_active = motion_hold;
    tof_debug.reject_reason = 0U;
    tof_debug.compensated_measurement_mm = 0.0f;

#if TOF_SENSOR_MODE == TOF_SENSOR_MODE_OLD_PAIR
    dl1b_get_distance();
    dl1b2_get_distance();

    z_R = (float)dl1b_distance_mm;
    z_L = (float)dl1b2_distance_mm;
    valid_R = TOF_IsValidMm(z_R);
    valid_L = TOF_IsValidMm(z_L);

    if (valid_L && valid_R) {
        float discrepancy = fabsf(z_L - z_R);
        float closer_Z  = (z_L < z_R) ? z_L : z_R;
        float farther_Z = (z_L < z_R) ? z_R : z_L;

        if (discrepancy > TOF_CABLE_MIN_DISCREPANCY
            && (trusted_Z - closer_Z) > TOF_CABLE_BELOW_TRUSTED) {
            raw_measured_Z = farther_Z;
        }
        else if (z_L < last_measured_Z - TOF_BOTH_DROP_FREEZE_MM
                 && z_R < last_measured_Z - TOF_BOTH_DROP_FREEZE_MM) {
            raw_measured_Z = trusted_Z + vertical_velocity_smoothed * 0.1f;
        }
        else if (discrepancy <= 150.0f) {
            raw_measured_Z = (z_R + z_L) * 0.5f;
        }
        else {
            float diff_L = fabsf(z_L - trusted_Z);
            float diff_R = fabsf(z_R - trusted_Z);
            raw_measured_Z = (diff_L < diff_R) ? z_L : z_R;
        }
        measure_available = 1U;
    }
    else if (valid_R) {
        raw_measured_Z = z_R;
        measure_available = 1U;
    }
    else if (valid_L) {
        raw_measured_Z = z_L;
        measure_available = 1U;
    }
#else
    dl1b_get_distance();
    dl1b3_poll();

    valid_R = TOF_GetDl1bHeight(&z_R);
    valid_L = TOF_GetDl1b3Ground(&z_L);

    if (valid_L) {
        raw_measured_Z = z_L;

        if (valid_R) {
            float discrepancy = fabsf(z_L - z_R);
            uint8_t dl1b_is_obstacle = (z_R + TOF_CABLE_MIN_DISCREPANCY < z_L)
                                    && ((trusted_Z - z_R) > TOF_CABLE_BELOW_TRUSTED);

            if (!dl1b_is_obstacle && discrepancy < TOF_DL1B3_CLOSE_FUSE_MM) {
                raw_measured_Z = z_L * 0.8f + z_R * 0.2f;
            }
        }
        measure_available = 1U;
    }
    else if (valid_R) {
        raw_measured_Z = z_R;
        measure_available = 1U;
    }
#endif
    tof_debug.measure_available = measure_available;
    if (!measure_available) {
        tof_debug.reject_reason |= TOF_REJECT_NO_MEASUREMENT;
    }
    if (tilt_factor < TOF_TILT_HOLD_FACTOR) {
        tof_debug.reject_reason |= TOF_REJECT_TILT_HOLD;
    }
    if (angle_rate_dps > TOF_ANGLE_RATE_HOLD_DPS) {
        tof_debug.reject_reason |= TOF_REJECT_ANGLE_RATE_HOLD;
    }
    if (motion_hold) {
        tof_hold_relock_pending = 1U;
        tof_recovery_active = 0U;
        tof_recovery_count = 0U;
        relock_streak = 0U;
        tof_down_relock_streak = 0U;
    }
    if (measure_available) {
        tof_debug.compensated_measurement_mm = raw_measured_Z;
    }

    // 3. 动态滤波与异常排斥 (五层防御)
    if (measure_available && !motion_hold)
    {
        // 移位并插入历史缓冲区
        z_hist[4]=z_hist[3]; z_hist[3]=z_hist[2]; z_hist[2]=z_hist[1]; z_hist[1]=z_hist[0];
        z_hist[0] = raw_measured_Z;

        // 快速中值滤波
        float sort[5];
        for(int i=0;i<5;i++) sort[i] = z_hist[i];
        for(int i=0;i<4;i++) {
            for(int j=0;j<4-i;j++) {
                if(sort[j] > sort[j+1]) { float t=sort[j]; sort[j]=sort[j+1]; sort[j+1]=t; }
            }
        }
        float measured_Z = sort[2];

        // 计算方差
        float mean = (z_hist[0] + z_hist[1] + z_hist[2] + z_hist[3] + z_hist[4]) / 5.0f;
        float variance = 0.0f;
        for(int i=0; i<5; i++) {
            float diff_var = z_hist[i] - mean;
            variance += diff_var * diff_var;
        }
        variance /= 5.0f;
        float std_dev = sqrtf(variance);

        uint8_t is_reliable = 1;
        // cm/s × 0.01s × 10mm/cm = ×0.1，得到 10ms 内的 mm 位移
        float predicted_Z = trusted_Z + vertical_velocity_smoothed * 0.1f;

        // 【关键修复1：隐式速度必须转为 cm/s】
        float dt = 0.01f; // 10ms
        float implied_velocity_cms = ((measured_Z - last_measured_Z) / dt) / 10.0f;

        if (implied_velocity_cms > TOF_MAX_CLIMB_RATE || implied_velocity_cms < TOF_MAX_DESCENT_RATE) {
            is_reliable = 0;
            tof_debug.reject_reason |= TOF_REJECT_RATE;
        }

        float diff = trusted_Z - measured_Z;
        float variance_threshold = TOF_VARIANCE_BASE;
        if (trusted_Z < 500.0f) variance_threshold = TOF_VARIANCE_LOW_ALT;
        else if (trusted_Z > 1000.0f) variance_threshold = TOF_VARIANCE_HIGH_ALT;

        if (std_dev > variance_threshold) {
            is_reliable = 0;
            tof_debug.reject_reason |= TOF_REJECT_VARIANCE;
        }

        float drop_threshold = TOF_DROP_BASE;
        float rise_threshold = TOF_RISE_BASE;

        float vel_factor = fabsf(vertical_velocity_smoothed) / 100.0f;
        drop_threshold += vel_factor * TOF_VEL_ADAPT_DROP;
        rise_threshold += vel_factor * TOF_VEL_ADAPT_RISE;

        if (trusted_Z < 400.0f) {
            drop_threshold *= 0.7f;
            rise_threshold *= 0.8f;
        }

        if (diff > drop_threshold) {
            is_reliable = 0;
            tof_debug.reject_reason |= TOF_REJECT_DROP;
        }
        else if (diff < -rise_threshold) {
            is_reliable = 0;
            tof_debug.reject_reason |= TOF_REJECT_RISE;
        }

        if (tof_hold_relock_pending) {
            if (fabsf(measured_Z - predicted_Z) <= TOF_RELOCK_MATCH_MM) {
                if (relock_streak == 0U) {
                    relock_streak = 1U;
                    relock_last_mm = measured_Z;
                } else if (fabsf(measured_Z - relock_last_mm) <= TOF_RELOCK_CONSISTENT_MM) {
                    if (relock_streak < TOF_RELOCK_CYCLES) {
                        relock_streak++;
                    }
                    relock_last_mm = measured_Z;
                } else {
                    relock_streak = 1U;
                    relock_last_mm = measured_Z;
                }
            } else {
                relock_streak = 0U;
            }

            if (relock_streak >= TOF_RELOCK_CYCLES) {
                tof_hold_relock_pending = 0U;
                tof_recovery_active = 1U;
                tof_recovery_count = 0U;
                is_reliable = 1U;
                tof_debug.reject_reason |= TOF_REJECT_RECOVERY;
            } else {
                is_reliable = 0U;
                tof_debug.reject_reason |= TOF_REJECT_RECOVERY;
            }
        }

        // 状态机更新
        if (is_reliable) {
            tof_blind_timer = 0;
            relock_streak = 0U;
            if (tof_recovery_active) {
                float recovery_alpha = 1.0f / (float)TOF_RECOVERY_CYCLES;
                trusted_Z += (measured_Z - trusted_Z) * recovery_alpha;
                tof_recovery_count++;
                tof_debug.reject_reason |= TOF_REJECT_RECOVERY;
                if (tof_recovery_count >= TOF_RECOVERY_CYCLES) {
                    tof_recovery_active = 0U;
                    tof_recovery_count = 0U;
                }
            } else {
                trusted_Z = predicted_Z * 0.3f + measured_Z * 0.7f;
            }
            last_good_Z = trusted_Z;
            last_measured_Z = measured_Z; // 更新上次有效测量值
            tof_down_relock_streak = 0U;
            tof_debug.accepted = 1U;
        } else {
            // 防死锁强制重锁：测量持续存在且自洽，却一直被门限拒绝，
            // 说明 trusted_Z 已与真实高度脱节（如降落时被冻结在高位），
            // 强制以当前测量重新锁定，避免高度永久卡死
            if (tof_hold_relock_pending) {
                goto BLIND_ESTIMATE;
            }
            if (fabsf(measured_Z - relock_last_mm) < TOF_RELOCK_CONSISTENT_MM) {
                if (relock_streak < TOF_RELOCK_CYCLES) {
                    relock_streak++;
                }
            } else {
                relock_streak = 1U;
            }
            relock_last_mm = measured_Z;

            if (relock_streak >= TOF_RELOCK_CYCLES
                && fabsf(measured_Z - predicted_Z) <= TOF_RELOCK_MATCH_MM) {
                relock_streak = 0U;
                tof_recovery_active = 1U;
                tof_recovery_count = 0U;
                tof_debug.reject_reason |= TOF_REJECT_RECOVERY;
            } else {
                goto BLIND_ESTIMATE;
            }
        }
    }
    else
    {
BLIND_ESTIMATE:
        tof_blind_timer++;
        if (measure_available && !motion_hold
            && raw_measured_Z < trusted_Z - TOF_DROP_BASE) {
            if (fabsf(raw_measured_Z - tof_down_relock_last_mm)
                <= TOF_DOWN_RELOCK_STABLE_MM) {
                if (tof_down_relock_streak < TOF_DOWN_RELOCK_CYCLES) {
                    tof_down_relock_streak++;
                }
            } else {
                tof_down_relock_streak = 1U;
            }
            tof_down_relock_last_mm = raw_measured_Z;

            if (tof_down_relock_streak >= TOF_DOWN_RELOCK_CYCLES) {
                float down_step = trusted_Z - raw_measured_Z;
                if (down_step > TOF_MAX_DOWN_STEP_MM) {
                    down_step = TOF_MAX_DOWN_STEP_MM;
                }
                trusted_Z -= down_step;
                last_good_Z = trusted_Z;
                last_measured_Z = raw_measured_Z;
                tof_blind_timer = 0U;
                tof_debug.accepted = 1U;
                tof_debug.reject_reason |= TOF_REJECT_RECOVERY;
                goto TOF_PUBLISH;
            }
        } else {
            tof_down_relock_streak = 0U;
        }

        // cm/s × 0.01s × 10mm/cm = ×0.1，得到 10ms 内的 mm 位移
        float predicted_Z = trusted_Z + vertical_velocity_smoothed * 0.1f;

        if (tof_blind_timer <= 50) {
            // < 500ms：完全信任 IMU 短期速度积分预测
            trusted_Z = predicted_Z;
        }
        else if (tof_blind_timer <= 200) {
            // < 2000ms：将预测高度平滑拉回最后已知的好高度 (防止积分漂移过头)
            float blend = (tof_blind_timer - 50) * 0.00667f;
            if (blend > 1.0f) blend = 1.0f;
            trusted_Z = predicted_Z * (1.0f - blend) + last_good_Z * blend;
        }
        else if (tof_blind_timer <= 300) {
            // < 3000ms：如果看到了新数据，小心翼翼地融合
            float confidence = (tof_blind_timer - 200) * 0.01f;
            if (confidence > 1.0f) confidence = 1.0f;

            // 防扫车：双传感器一致且同时偏低 → 判定为车顶，拒绝融合
            uint8_t both_valid = (valid_L && valid_R);
            float sensor_diff = both_valid ? fabsf(z_R - z_L) : 0.0f;
            uint8_t both_see_obstacle = (both_valid && sensor_diff < 100.0f
                                         && raw_measured_Z < last_good_Z - 100.0f);

            if (measure_available && !motion_hold
                && fabsf(raw_measured_Z - last_good_Z) < 400.0f
                && !both_see_obstacle) {
                trusted_Z = last_good_Z * (1.0f - confidence) + raw_measured_Z * confidence;
            } else {
                trusted_Z = last_good_Z;
            }
        }
        else {
            // > 3000ms 盲区超时：带物理合理性检查的谨慎重锁定
            if (measure_available && !motion_hold) {
                // 防扫车：检查新测量值是否物理合理
                // 若下降量超过最大允许值且超过3秒内垂直速度能解释的范围 → 障碍物
                float drop_from_last_good = last_good_Z - raw_measured_Z;
                // cm/s × 3s × 10mm/cm = ×30，3 秒内速度可解释的 mm 降幅
                float predicted_drop = fabsf(vertical_velocity_smoothed) * 30.0f;

                if (drop_from_last_good > TOF_BLIND_RESET_MAX_DROP
                    && drop_from_last_good > predicted_drop) {
                    // 判定为障碍物（车顶等），保持IMU预测，不复位
                    trusted_Z = last_good_Z;
                } else {
                    // 物理合理：接受新测量值
                    TOF_Reset_States(raw_measured_Z);
                }
            } else {
                trusted_Z = last_good_Z;
            }
        }
    }

TOF_PUBLISH:
    // 4. 更新最终高度全局变量
    if(!TOF_IsFinite(trusted_Z)) {
        trusted_Z = last_good_Z;
    }
    if(!TOF_IsFinite(trusted_Z)) {
        trusted_Z = 100.0f;
    }
    if ((motion_hold || tof_blind_timer > 0U || tof_hold_relock_pending
         || tof_recovery_active)
        && height > 1.0f
        && fabsf(trusted_Z - height) > TOF_MAX_OUTPUT_STEP_MM) {
        trusted_Z = height + ((trusted_Z > height) ? TOF_MAX_OUTPUT_STEP_MM
                                                    : -TOF_MAX_OUTPUT_STEP_MM);
        tof_debug.reject_reason |= TOF_REJECT_OUTPUT_STEP;
    }
    trusted_Z = constrain_float(trusted_Z, 0.0f, TOF_HEIGHT_MAX_MM);
    height = trusted_Z;
    tof_debug.trusted_height_mm = trusted_Z;
    tof_debug.recovery_active = tof_recovery_active;
    tof_debug.recovery_count = tof_recovery_count;
    tof_debug.relock_streak = relock_streak;
    tof_debug.blind_timer = tof_blind_timer;

    // 5. 更新卡尔曼状态观测器
    correct_vertical_velocity(height, 0.01f);
}
#endif
