#include "zf_common_headfile.h"

#include "PWM.h"

//P11 P12����PWM1����    P13 P14����PWM2����
void PWM1_Init(void)
{
	pwm_init(PWM_CH1,17000,0);
	pwm_init(PWM_CH2,17000,0);
	gpio_init(IO_P62, GPO, 1, GPO_PUSH_PULL);
	gpio_init(IO_P60, GPO, 1, GPO_PUSH_PULL);
	gpio_init(IO_P64, GPO, 1, GPO_PUSH_PULL);
	gpio_init(IO_P66, GPO, 1, GPO_PUSH_PULL);
}

void PWM1_Set(int16 speed)
{
	if(speed >= 0)
	{
	P60 = 0;
	P64 = 1;
	pwm_set_duty(PWM_CH1, speed * 100);
	}
	else
	{
	P60 = 1;
	P64 = 0;
	pwm_set_duty(PWM_CH1, - speed * 100);
	}
}
void PWM2_Set(int16 speed)
{
	if(speed >= 0)
	{
		P62 = 0;
		P66 = 1;
		pwm_set_duty(PWM_CH2, speed * 100);
	}
	else
	{
		P62 = 1;
		P66 = 0;
		pwm_set_duty(PWM_CH2, -speed * 100);
	}
}
