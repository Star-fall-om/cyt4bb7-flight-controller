#include "zf_common_headfile.h"
#include "motor.h"
volatile float min_motor = 1000.0f;
void Motor_Init(void)
{
    pwm_init(PWM_CH1, PWM_FREQ, 0);
    pwm_init(PWM_CH2, PWM_FREQ, 0);
    pwm_init(PWM_CH3, PWM_FREQ, 0);
    pwm_init(PWM_CH4, PWM_FREQ, 0);
    gpio_init(P17_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P10_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P08_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P05_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);

}

void motor(uint16 PH,uint16 us)
{
    switch (PH)
    {
    case 1:
        pwm_set_duty(PWM_CH1, DUTY_US(us));
        break;
    case 2:
        pwm_set_duty(PWM_CH2, DUTY_US(us));
        break;
    case 3:
        pwm_set_duty(PWM_CH3, DUTY_US(us));
        break;
    case 4:
        pwm_set_duty(PWM_CH4, DUTY_US(us));
        break;
    default:
        break;
    }
}
