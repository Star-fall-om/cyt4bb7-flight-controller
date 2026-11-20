#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"  // 根据实际情况包含头文件

#define PWM_CH1              (TCPWM_CH57_P17_4)  // 17.4
#define PWM_CH2              (TCPWM_CH31_P10_3)  // 10.3
#define PWM_CH3              (TCPWM_CH21_P08_2)  // 8.2
#define PWM_CH4              (TCPWM_CH12_P05_3)  // 5.3
#define PWM_FREQ            (395)

#define MIN_US              (1000)
#define MAX_US              (2000)

#define DUTY_US(us)  ((uint16)((float)PWM_DUTY_MAX * ((float)(us) / (1000000.0f / (float)PWM_FREQ))))
extern volatile float min_motor;
void Motor_Init(void);
void motor(uint16 PH,uint16 Duty_us);



#endif
