#include "line.h"
#include "zf_driver_adc.h" 

// =================================================================
// 引脚映射 
// =================================================================
#define TR_L2 ADC_CH8_P00
#define TR_L1 ADC_CH9_P01
#define TR_M  ADC_CH13_P05
#define TR_R1 ADC_CH14_P06
#define TR_R2 ADC_CH3_P13

// =================================================================
// 算法参数
// =================================================================
// 权重分配：越靠边的传感器权重越大，转向力度越强
#define WEIGHT_L2  -4.0f
#define WEIGHT_L1  -2.0f
#define WEIGHT_M    0.0f
#define WEIGHT_R1   2.0f
#define WEIGHT_R2   4.0f

// 黑白阈值 (0-4095)
// 调试方法：
// 1. 放在白纸上，看串口打印值 (例如 300)
// 2. 放在黑线上，看串口打印值 (例如 3800)
// 3. 取中间值 (300+3800)/2 = 2050
#define THRESHOLD  2000 

static float last_position = 0.0f;

void Line_Init(void)
{
    // 初始化 ADC 引脚为 12位精度
    adc_init(TR_L2, ADC_12BIT);
    adc_init(TR_L1, ADC_12BIT);
    adc_init(TR_M,  ADC_12BIT);
    adc_init(TR_R1, ADC_12BIT);
    adc_init(TR_R2, ADC_12BIT);
}

// 供 mode.c 调用的状态检查函数
// 返回 1: 只要有一个传感器压线 (检测到弯道/黑线)
// 返回 0: 全部白 (检测到直道/出弯)
uint8 Track_Is_Line_Exist(void)
{
    // 传感器逻辑：黑线电压高 (> 阈值)
    if(adc_convert(TR_L2) > THRESHOLD) return 1;
    if(adc_convert(TR_L1) > THRESHOLD) return 1;
    if(adc_convert(TR_M)  > THRESHOLD) return 1;
    if(adc_convert(TR_R1) > THRESHOLD) return 1;
    if(adc_convert(TR_R2) > THRESHOLD) return 1;
    
    return 0; 
}

// 计算加权误差
float Track_Get_Weighted_Error(void)
{
    float weight_sum = 0.0f;
    uint8 active_cnt = 0;
    
    // 读取 ADC 值 (0-4095)
    uint16 vL2 = adc_convert(TR_L2);
    uint16 vL1 = adc_convert(TR_L1);
    uint16 vM  = adc_convert(TR_M);
    uint16 vR1 = adc_convert(TR_R1);
    uint16 vR2 = adc_convert(TR_R2);
    
    // 累加权重
    if(vL2 > THRESHOLD) { weight_sum += WEIGHT_L2; active_cnt++; }
    if(vL1 > THRESHOLD) { weight_sum += WEIGHT_L1; active_cnt++; }
    if(vM  > THRESHOLD) { weight_sum += WEIGHT_M;  active_cnt++; }
    if(vR1 > THRESHOLD) { weight_sum += WEIGHT_R1; active_cnt++; }
    if(vR2 > THRESHOLD) { weight_sum += WEIGHT_R2; active_cnt++; }
    
    if(active_cnt > 0) {
        // 计算平均误差
        last_position = weight_sum / active_cnt;
        return last_position;
    }
    else {
        // 全白（直道）：返回 0.0f
        // 这样 PID_Turn 只有 D 项(陀螺仪阻尼)工作，P 项为 0
        // 车子会依靠惯性和机械对称性走直线
        return 0.0f; 
    }
}
