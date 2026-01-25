#include "zf_common_debug.h"
#include "zf_driver_adc.h"


static uint8 adc_resolution = {ADC_12BIT};


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC转换N次，平均值滤波
// 参数说明     ch              选择ADC通道
// 参数说明     resolution      分辨率(8位 10位 12位)
// 参数说明     count           转换次数
// 返回参数     void
// 使用示例     adc_mean_filter(ADC_IN0_A0, ADC_8BIT,5);  //采集A0端口返回8位分辨率的AD值，采集五次取平均值
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_mean_filter_convert (adc_channel_enum ch, const uint8 count)
{
    uint8 i;
    uint32 sum;

	zf_assert(count);//断言次数不能为0
	
    sum = 0;
    for(i=0; i<count; i++)
    {
        sum += adc_convert(ch);
    }

    sum = sum/count;
    return sum;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC转换一次
//  @param      adcn            选择ADC通道
//  @param      resolution      分辨率
//  @return     void
//  Sample usage:               adc_convert(ADC_P10, ADC_10BIT);
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_convert(adc_channel_enum ch)
{
    uint16 adc_value;
    
    ADC_CONTR &= (0xF0);			//清除ADC_CHS[3:0] ： ADC 模拟通道选择位
    ADC_CONTR |= ch;
    
    ADC_CONTR |= 0x40;  			// 启动 AD 转换
    
    while (!(ADC_CONTR & 0x20));  	// 查询 ADC 完成标志
    
    ADC_CONTR &= ~0x20;  			// 清完成标志
    
    adc_value = ADC_RES;  			//存储 ADC 的 12 位结果的高 4 位
    adc_value <<= 8;
    adc_value |= ADC_RESL;  		//存储 ADC 的 12 位结果的低 8 位
    
    ADC_RES = 0;
    ADC_RESL = 0;
    
    adc_value >>= adc_resolution;	//取多少位
    
    
    return adc_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC初始化
// 参数说明     ch      			ch(可选择范围由zf_adc.h内adc_channel_enum枚举值确定)
// 参数说明     resolution       resolution(可选择范围由zf_adc.h内adc_resolution_enum枚举值确定)
// 返回参数     void
// 使用示例     adc_init(ADC_P10, ADC_12BIT);//初始化P10引脚为ADC功能，12位分辨率
//-------------------------------------------------------------------------------------------------------------------
void adc_init(adc_channel_enum ch, adc_resolution_enum resolution)
{
    ADC_CONTR |= 1 << 7;				//1 ：打开 ADC 电源
    
    ADC_CONTR &= (0xF0);			//清除ADC_CHS[3:0] ： ADC 模拟通道选择位
    ADC_CONTR |= ch;
    
    if((ch >> 3) == 1) //P0.0
    {
        //IO口需要设置为高阻输入
        P0M0 &= ~(1 << (ch & 0x07));
        P0M1 |= (1 << (ch & 0x07));
    }
    else if((ch >> 3) == 0) //P1.0
    {
        //IO口需要设置为高阻输入
        P1M0 &= ~(1 << (ch & 0x07));
        P1M1 |= (1 << (ch & 0x07));
    }
    
    ADCCFG |= ADC_SYSclk_DIV_16 & 0x0F;			//ADC时钟频率SYSclk/16&0x0F;
    
    ADCCFG |= 1 << 5;							//转换结果右对齐。 ADC_RES 保存结果的高 2 位， ADC_RESL 保存结果的低 8 位。
	
	adc_resolution = resolution;           // 记录ADC精度 将在采集时使用
}