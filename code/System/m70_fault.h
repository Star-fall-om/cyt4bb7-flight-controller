#ifndef M70_FAULT_H
#define M70_FAULT_H

#include <stdint.h>

#define M70_TASK_INDEX_DEFAULT          (1u)
#define M70_TASK_INDEX_REMOTE           (2u)
#define M70_TASK_INDEX_IMU              (3u)
#define M70_TASK_INDEX_SBUS             (4u)
#define M70_TASK_INDEX_CONTROL          (5u)
#define M70_TASK_INDEX_FLOW             (6u)
#define M70_TASK_INDEX_VISION           (7u)
#define M70_TASK_INDEX_VOFA             (8u)

void M70Fault_BootInit(void);
void M70Fault_Report(void);
void M70Fault_ThreadCreateFailed(uint32_t task_index,
                                 const char *task_name,
                                 uint8_t runtime_started);

#endif
