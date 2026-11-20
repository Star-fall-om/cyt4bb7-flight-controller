// ================== 视觉摄像头模式配置 ==================
// 用于在单摄和双摄模式之间切换
// 单摄模式：主摄识别红外信标和小车红外灯带
// 双摄模式：主摄识别红外信标，辅助摄像头识别小车红外灯带

#ifndef __VISION_MODE_H__
#define __VISION_MODE_H__

#include "Config/flight_feature_config.h"

// 摄像头模式定义
#define VISION_CAMERA_MODE_SINGLE          1u  // 单摄模式：主摄同时识别小车与信标
#define VISION_CAMERA_MODE_DUAL_LEGACY     2u  // 旧双摄：主摄信标+辅助小车
#define VISION_CAMERA_MODE_DUAL_IR_BACKUP  3u  // 双红外主备：两台摄像头都识别小车与信标
#define VISION_CAMERA_MODE_DUAL             VISION_CAMERA_MODE_DUAL_IR_BACKUP

// 摄像头模式选择
#ifndef VISION_CAMERA_MODE
#define VISION_CAMERA_MODE          VISION_CAMERA_MODE_DUAL_IR_BACKUP
#endif

// 模式合法性检查
#if (VISION_CAMERA_MODE != VISION_CAMERA_MODE_SINGLE) && \
    (VISION_CAMERA_MODE != VISION_CAMERA_MODE_DUAL_LEGACY) && \
    (VISION_CAMERA_MODE != VISION_CAMERA_MODE_DUAL_IR_BACKUP)
#error Unsupported VISION_CAMERA_MODE
#endif

// 是否启用辅助摄像头（双摄模式自动启用）
#define VISION_CAMERA_USES_AUX      \
    ((VISION_CAMERA_MODE == VISION_CAMERA_MODE_DUAL_LEGACY) || \
     (VISION_CAMERA_MODE == VISION_CAMERA_MODE_DUAL_IR_BACKUP))

// 主摄在单摄和双红外主备模式下都识别小车红外灯带。
#define VISION_MAIN_CAMERA_DETECTS_CAR  \
    ((VISION_CAMERA_MODE == VISION_CAMERA_MODE_SINGLE) || \
     (VISION_CAMERA_MODE == VISION_CAMERA_MODE_DUAL_IR_BACKUP))

#define VISION_DUAL_IR_BACKUP_ENABLED \
    (VISION_CAMERA_MODE == VISION_CAMERA_MODE_DUAL_IR_BACKUP)

#endif
