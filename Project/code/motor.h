#ifndef __MOTER_H
#define __MOTER_H

void Motor_Init(void);
void PWM1_Set(int16 speed);
void PWM2_Set(int16 speed);

#define PWM_CH1                 (PWMA_CH1P_P10)
#define PWM_CH2                 (PWMA_CH2P_P22)
#define PWM_CH3                 (PWMA_CH3P_P64)
#define PWM_CH4                 (PWMA_CH4P_P34)	

#endif
