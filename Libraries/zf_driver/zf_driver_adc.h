#ifndef __ZF_DRIVER_ADC_H
#define __ZF_DRIVER_ADC_H

#include "zf_common_typedef.h"

//此枚举定义不允许用户修改
typedef enum
{
    ADC_CH0_P10 = 0,
    ADC_CH1_P11,
    ADC_CH2_P12,
    ADC_CH3_P13,
    ADC_CH4_P14,
    ADC_CH5_P15,
    ADC_CH6_P16,
    ADC_CH7_P17,
    
    ADC_CH8_P00,
    ADC_CH9_P01,
    ADC_CH10_P02        	,
    ADC_CH11_P03        	,
    ADC_CH12_P04        	,
    ADC_CH13_P05        	,
    ADC_CH14_P06        	,
    ADC_CH15_POWR = 0x0f	, //内部AD 1.19V
} adc_channel_enum;

//此枚举定义不允许用户修改
typedef enum
{
    ADC_SYSclk_DIV_2 = 0,
    ADC_SYSclk_DIV_4,
    ADC_SYSclk_DIV_6,
    ADC_SYSclk_DIV_8,
    ADC_SYSclk_DIV_10,
    ADC_SYSclk_DIV_12,
    ADC_SYSclk_DIV_14,
    ADC_SYSclk_DIV_16,
    ADC_SYSclk_DIV_18,
    ADC_SYSclk_DIV_20,
    ADC_SYSclk_DIV_22,
    ADC_SYSclk_DIV_24,
    ADC_SYSclk_DIV_26,
    ADC_SYSclk_DIV_28,
    ADC_SYSclk_DIV_30,
    ADC_SYSclk_DIV_32,
} adc_speed_enum;


//此枚举定义不允许用户修改
typedef enum    // 枚举ADC通道
{

    ADC_12BIT = 0,  //12位分辨率
    ADC_11BIT,		//11位分辨率
    ADC_10BIT,		//10位分辨率
    ADC_9BIT,    	//9位分辨率
    ADC_8BIT,     	//8位分辨率
    
} adc_resolution_enum;


uint16  adc_convert             (adc_channel_enum ch);
uint16  adc_mean_filter_convert (adc_channel_enum ch, const uint8 count);
void    adc_init                (adc_channel_enum ch, adc_resolution_enum resolution);



#endif