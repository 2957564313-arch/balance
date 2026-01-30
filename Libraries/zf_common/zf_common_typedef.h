#ifndef __COMMON_H_
#define __COMMON_H_
#include "STC32Gxx.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"


#ifndef ENABLE
#define ENABLE 1
#endif

#ifndef DISABLE
#define DISABLE 0
#endif

//数据类型声明
typedef unsigned char   uint8  ;	//  8 bits
typedef unsigned int  	uint16 ;	// 16 bits
typedef unsigned long  	uint32 ;	// 32 bits


typedef signed char     int8   ;	//  8 bits
typedef signed int      int16  ;	// 16 bits
typedef signed long     int32  ;	// 32 bits


typedef volatile int8   vint8  ;	//  8 bits
typedef volatile int16  vint16 ;	// 16 bits
typedef volatile int32  vint32 ;	// 32 bits


typedef volatile uint8  vuint8 ;	//  8 bits
typedef volatile uint16 vuint16;	// 16 bits
typedef volatile uint32 vuint32;	// 32 bits



typedef enum //无线模块
{
    NO_WIRELESS_MODE = 0,   // 没有无线模块
    WIRELESS_SI24R1 = 1,    // 无线转串口
    WIRELESS_CH9141 = 2,    // 蓝牙转串口
    WIRELESS_CH573 = 3,     // CH573模块
    WIRELESS_BLE6A20 = 4,   // BLE6A20蓝牙模块
    
} WIRELESS_TYPE_enum;


extern void (*uart1_irq_handler)(uint8 dat);
extern void (*uart2_irq_handler)(uint8 dat);
extern void (*uart3_irq_handler)(uint8 dat);
extern void (*uart4_irq_handler)(uint8 dat);

extern void (*tim0_irq_handler)(void);
extern void (*tim1_irq_handler)(void);
extern void (*tim2_irq_handler)(void);
extern void (*tim3_irq_handler)(void);
extern void (*tim4_irq_handler)(void);
extern void (*tim11_irq_handler)(void);

extern void (*int0_irq_handler)(void);
extern void (*int1_irq_handler)(void);
extern void (*int2_irq_handler)(void);
extern void (*int3_irq_handler)(void);
extern void (*int4_irq_handler)(void);

#define ZF_ENABLE           (1)
#define ZF_DISABLE          (0)

#define ZF_TRUE             (1)
#define ZF_FALSE            (0)

#endif
