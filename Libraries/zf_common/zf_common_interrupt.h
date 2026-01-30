#ifndef __ZF_DRIVER_NVIC_H
#define __ZF_DRIVER_NVIC_H

#include "zf_common_typedef.h"

//该枚举体禁止用户修改
//中断优先级控制枚举体
typedef enum
{
	INT0_IRQn = 0x00,
	TIMER0_IRQn,
	INT1_IRQn,
	TIMER1_IRQn,
	UART1_IRQn,
	ADC_IRQn,
	LVD_IRQn,			//低压检测中断

	UART2_IRQn = 0x10,
	SPI_IRQn,
	PWM1_IRQn,
	PWM2_IRQn,
	INT4_IRQn,
	CMP_IRQn,
	IIC_IRQn,
	USB_IRQn,	//增强型 PWM2 异常检测中断 和 触摸按键中断

	UART3_IRQn = 0x20,
	UART4_IRQn,

	// 其余不能设置的，中断优先级全部为最低优先级0
    
} irqn_type_enum;


void interrupt_set_priority(irqn_type_enum irqn, uint8 priority);

void interrupt_global_disable(void);
void interrupt_global_enable(void);


#endif