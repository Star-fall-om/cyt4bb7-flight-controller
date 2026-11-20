#ifndef VISION_SHARED_DEBUG_H
#define VISION_SHARED_DEBUG_H

#include <stdint.h>

/*
 * M71 -> M70 的视觉原始坐标调试快照。
 *
 * 该结构独立于 SharedData_Up_t，避免改变现有双核控制通信布局。
 * writer 先写入所有字段，最后提交偶数 commit_seq；奇数表示正在更新。
 */
typedef struct {
    uint32_t commit_seq;
    uint32_t frame_id;
    uint32_t timestamp_ms;
    float car_x_cm;
    float car_y_cm;
    float beacon_x_cm;
    float beacon_y_cm;
    float control_input_x_cm;
    float control_input_y_cm;
    float beacon_minus_car_x_cm;
    float beacon_minus_car_y_cm;
    float car_vx_cms;
    float car_vy_cms;
    uint8_t car_valid;
    uint8_t beacon_valid;
    uint8_t beacon_held;
    uint8_t pair_valid;
    uint8_t car_source;
    uint8_t beacon_source;
    uint8_t target_mode;
    uint8_t car_updated;
    uint16_t car_age_frames;
    uint16_t beacon_age_frames;
    uint32_t extinguished_count;
    uint8_t beacon_lost_frames;
    uint8_t candidate_count;
    uint8_t reserved0[2];
    float beacon_guide_x_cms;
    float beacon_guide_y_cms;
    float beacon_guide_gain;
    float beacon_guide_distance_cm;
    uint8_t beacon_guide_active;
    uint8_t beacon_guide_reason;
    float visual_drone_relative_x_cms;
    float visual_drone_relative_y_cms;
    float visual_drone_car_ff_x_cms;
    float visual_drone_car_ff_y_cms;
    float visual_drone_raw_vx_right_cms;
    float visual_drone_raw_vy_back_cms;
    float beacon_car_distance_cm;
    uint8_t car_near_beacon;
    uint8_t reserved[1];
} vision_shared_debug_t;

/* 自然对齐会在两个 uint8_t 标志后为后续 float 插入2字节填充，实际大小为124字节。 */
typedef char VisionSharedDebug_Size_Must_Be_124[
    (sizeof(vision_shared_debug_t) == 124u) ? 1 : -1];

#endif
