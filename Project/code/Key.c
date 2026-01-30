#include "Key.h" 

// 状态定义
#define KEY_PRESSED             1
#define KEY_UNPRESSED           0

// 时间阈值设定 (基于 5ms 中断周期)
// 注意：Key_Tick 是在 5ms 中断里调用的，所以计数要除以 5
// 原来是 1ms，现在是 5ms，数值要缩小 5 倍
#define KEY_Time_DOUBLE         (300 / 5)   // 300ms 窗口 / 5ms = 60次
#define KEY_Time_LONG           (1000 / 5)  // 1000ms / 5ms = 200次
#define KEY_Time_REPEAT         (200 / 5)   // 200ms / 5ms = 40次

uint8 Key_Flag[KEY_COUNT]; // 事件标志位数组

void Key_Init(void)
{
    // 初始化引脚为输入上拉模式
    // 逐飞库: gpio_init(引脚, 模式, 默认电平, 驱动方式)
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);
}

// 读取物理按键状态
// 返回：KEY_PRESSED (1) 或 KEY_UNPRESSED (0)
static uint8 Key_GetState(uint8 n)
{
    uint8 level = 1;
    switch(n)
    {
        case 0: level = gpio_get_level(KEY1_PIN); break;
        case 1: level = gpio_get_level(KEY2_PIN); break;
        case 2: level = gpio_get_level(KEY3_PIN); break;
        case 3: level = gpio_get_level(KEY4_PIN); break;
        default: return KEY_UNPRESSED;
    }
    
    // 低电平有效（按下为0）
    return (level == 0) ? KEY_PRESSED : KEY_UNPRESSED;
}

// 检查并清除标志位 (核心 API)
uint8 Key_Check(uint8 n, uint8 Flag)
{
    if (Key_Flag[n] & Flag)
    {
        // 如果不是 HOLD 状态，读完自动清除标志位 (模拟事件消耗)
        if (Flag != KEY_HOLD)
        {
            Key_Flag[n] &= ~Flag;
        }
        return 1; // 事件发生
    }
    return 0;
}

// 简易检测 (给 main.c 用)
// 轮询 4 个按键，只要有单击事件就返回按键号
uint8 Key_Check_Simple(void)
{
    if(Key_Check(0, KEY_SINGLE)) return 1;
    if(Key_Check(1, KEY_SINGLE)) return 2;
    if(Key_Check(2, KEY_SINGLE)) return 3;
    if(Key_Check(3, KEY_SINGLE)) return 4;
    return 0;
}

// 核心状态机函数
// 在 5ms 定时器中断中调用
void Key_Tick(void)
{
    static uint8 i;
    static uint8 CurrState[KEY_COUNT]; 
    static uint8 PrevState[KEY_COUNT]; 
    static uint8 S[KEY_COUNT];         
    static uint16 Time[KEY_COUNT];     
    
    // 1. 全局计时器递减
    for (i = 0; i < KEY_COUNT; i++)
    {
        if (Time[i] > 0) Time[i]--;
    }

    // 2. 按键扫描
    // 由于现在是在 5ms 中断里调用，天然自带消抖，不需要额外的分频 Count
    // 直接每次进来都扫一遍
    
    for (i = 0; i < KEY_COUNT; i++)
    {
        PrevState[i] = CurrState[i];
        CurrState[i] = Key_GetState(i);
        
        // --- 边沿事件 ---
        if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
            Key_Flag[i] |= KEY_DOWN;
        
        if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
            Key_Flag[i] |= KEY_UP;
        
        // --- 高级状态机 ---
        switch(S[i])
        {
            case 0: // 空闲
                if (CurrState[i] == KEY_PRESSED)
                {
                    Time[i] = KEY_Time_LONG; 
                    S[i] = 1; 
                }
                break;
                
            case 1: // 等待释放或长按
                if (CurrState[i] == KEY_UNPRESSED)
                {
                    Time[i] = KEY_Time_DOUBLE; 
                    S[i] = 2; // 等待双击
                }
                else if (Time[i] == 0)
                {
                    Time[i] = KEY_Time_REPEAT; 
                    Key_Flag[i] |= KEY_LONG;
                    S[i] = 4; // 连发
                }
                break;
                
            case 2: // 等待第二次按下
                if (CurrState[i] == KEY_PRESSED)
                {
                    Key_Flag[i] |= KEY_DOUBLE;
                    S[i] = 3; 
                }
                else if (Time[i] == 0)
                {
                    Key_Flag[i] |= KEY_SINGLE; // 时间到，确认单击
                    S[i] = 0; 
                }
                break;
                
            case 3: // 双击后等待松手
                if (CurrState[i] == KEY_UNPRESSED)
                {
                    S[i] = 0;
                }
                break;
                
            case 4: // 长按连发
                if (CurrState[i] == KEY_UNPRESSED)
                {
                    S[i] = 0;
                }
                else if (Time[i] == 0)
                {
                    Time[i] = KEY_Time_REPEAT;
                    Key_Flag[i] |= KEY_REPEAT;
                }
                break;
        }
    }
}
