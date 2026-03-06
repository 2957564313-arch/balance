#ifndef __BOARD_H

#define __BOARD_H

#include "zf_common_typedef.h"

#include "zf_driver_uart.h"





#define SYSTEM_CLOCK_22_1184M 	22118400

#define SYSTEM_CLOCK_24M      	24000000

#define SYSTEM_CLOCK_27M      	27000000

#define SYSTEM_CLOCK_30M      	30000000

#define SYSTEM_CLOCK_33_1776M 	33177600

#define SYSTEM_CLOCK_35M      	35000000


#define EXTERNAL_CRYSTA_ENABLE 	0			// ʹ���ⲿ����0Ϊ��ʹ�ã�1Ϊʹ�ã�����ʹ���ڲ�����

#define FOSC					0			// FOSC��ֵ����Ϊ0�����ں�Ƶ��ͨ���Ĵ���ǿ�����á�
											// ����STC-ISP��������ʱ��ѡ����٣����������õ�Ƶ�ʡ�
											
//#define FOSC      	SYSTEM_CLOCK_30M	// FOSC��ֵ����Ϊ30Mhz,
											// ʹ��STC-ISP�������ص�ʱ��
											// ��Ƶ����Ҫ��STC-ISP�����е� <�����û���������ʱ��IRCƵ��>ѡ���Ƶ��һ�¡�

extern int32 system_clock;

void clock_init (uint32 clock);                                               // ����ʱ�ӳ�ʼ��


#endif



