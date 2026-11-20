#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include "Vision/Vision.h"

// 获取跟踪激活标志指针
volatile uint8_t *Navigation_GetTrackingActivePtr(void);

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
                               uint8_t search_switch);

// 填充共享数据结构
void Navigation_FillSharedData(volatile SharedData_Down_t *data);

#endif // NAVIGATION_H
