#include "zf_driver_gpio.h"
#include "zf_driver_encoder.h"

static volatile uint8 encoder_dir_pin[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     定时器编码器解码取值
// 参数说明     encoder_n      定时器枚举体
// 返回参数     void
// 备注信息
// 使用示例    encoder_get_count(TIM2_ENCOEDER)  // 获取定时器2的采集到的编码器数据
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_count(encoder_index_enum encoder_n)
{
    int16 dat = 0;
    switch(encoder_n)
    {
        case TIM0_ENCOEDER:
        {
            dat = (uint16)TH0 << 8;
            dat = ((uint8)TL0) | dat;
            break;
        }
        case TIM1_ENCOEDER:
        {
            dat = (uint16)TH1 << 8;
            dat = ((uint8)TL1) | dat;
            break;
        }
        case TIM2_ENCOEDER:
        {
            dat = (uint16)T2H << 8;
            dat = ((uint8)T2L) | dat;
            break;
        }
        case TIM3_ENCOEDER:
        {
            dat = (uint16)T3H << 8;
            dat = ((uint8)T3L) | dat;
            break;
        }
        case TIM4_ENCOEDER:
        {
            dat = (uint16)T4H << 8;
            dat = ((uint8)T4L) | dat;
            break;
        }
    }
    if(gpio_get_level(encoder_dir_pin[encoder_n]))
    {
        return (-dat);
    }
    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     定时器的计数器清空
// 参数说明     encoder_n      定时器枚举体
// 返回参数     void
// 备注信息
// 使用示例    encoder_clear_count(TIM1_ENCOEDER)  //清除定时器1采集到的编码器数据
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear_count(encoder_index_enum encoder_n)
{
    switch(encoder_n)
    {
        case TIM0_ENCOEDER:
        {
            TR0 = 0;
            TH0 = 0;
            TL0 = 0;
            TR0 = 1;
            break;
        }
        case TIM1_ENCOEDER:
        {
            TR1 = 0;
            TH1 = 0;
            TL1 = 0;
            TR1 = 1;
            break;
        }
        case TIM2_ENCOEDER:
        {
            AUXR &= ~(1 << 4);
            T2H = 0;
            T2L = 0;
            AUXR |= 1 << 4;
            break;
        }
        case TIM3_ENCOEDER:
        {
            T4T3M &= ~(1 << 3);
            T3H = 0;
            T3L = 0;
            T4T3M |= (1 << 3);
            break;
        }
        case TIM4_ENCOEDER:
        {
            T4T3M &= ~(1 << 7);
            T4H = 0;
            T4L = 0;
            T4T3M |= (1 << 7);
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器解码初始化
// 参数说明     timer_ch        定时器枚举体
// 参数说明     lsb_pin         编码器脉冲引脚
// 参数说明     dir_pin         编码器方向引脚
// 返回参数     void
//          推荐使用带方向解码编码器。
// 使用示例      encoder_init_dir(TIM1_ENCOEDER, TIM1_CH1_ENCOEDER_E9, TIM1_CH2_ENCOEDER_E11)
//                              // 使用定时器1 做带方向的编码器解码， 通道1方向信号引脚E9，通道2脉冲信号引脚E11
//-------------------------------------------------------------------------------------------------------------------
void encoder_dir_init(encoder_index_enum encoder_n, gpio_pin_enum dir_pin, encoder_channel_enum lsb_pin)
{
    // 如果程序在输出了断言信息 并且提示出错位置在这里
    // 就去查看你在什么地方调用这个函数 检查你的传入参数
    // 这里是检查是否有重复使用定时器
    // 比如初始化了 TIM1_PWM 然后又初始化成 TIM1_ENCODER 这种用法是不允许的
    zf_assert(timer_funciton_check(encoder_n, TIMER_FUNCTION_ENCODER));
    zf_assert((dir_pin >> 8) == 0x00);
    zf_assert((lsb_pin >> 8) == encoder_n);
    // 初始化方向引脚
    gpio_init(dir_pin, GPI, 0, GPI_PULL_UP);
    gpio_init(lsb_pin&0xFF, GPI, 0, GPI_PULL_UP);
    encoder_dir_pin[encoder_n] = dir_pin;                               // 将方向引脚号存入数组中
    switch(encoder_n)
    {
        case TIM0_ENCOEDER:
        {
            TL0 = 0;
            TH0 = 0;
            TMOD |= 0x04; //外部计数模式
            TR0 = 1; //启动定时器
            break;
        }
        case TIM1_ENCOEDER:
        {
            TL1 = 0x00;
            TH1 = 0x00;
            TMOD |= 0x40; // 外部计数模式
            TR1 = 1; // 启动定时器
            break;
        }
        case TIM2_ENCOEDER:
        {
            T2L = 0x00;
            T2H = 0x00;
            AUXR |= 0x18; // 设置外部计数模式并启动定时器
            break;
        }
        case TIM3_ENCOEDER:
        {
            T3L = 0;
            T3H = 0;
            T4T3M |= 0x0c; // 设置外部计数模式并启动定时器
            break;
        }
        case TIM4_ENCOEDER:
        {
            T4L = 0;
            T4H = 0;
            T4T3M |= 0xc0; // 设置外部计数模式并启动定时器
            break;
        }
    }
}