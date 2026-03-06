#include "Key.h"

#define KEY_PRESSED             1                       // 物理按下
#define KEY_UNPRESSED           0                       // 物理松开

#define KEY_Time_LONG           (200)                   // 200 × 5ms = 1000ms  长按判定
#define KEY_Time_REPEAT         (40)                    // 40  × 5ms = 200ms   连发间隔

// ================================== 对外状态变量 ==================================
uint8 Key_Flag[KEY_COUNT];                              // 每个按键的事件标志位


// ================================== 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        读取指定按键的物理电平状态
// 参数说明        n                按键编号 (0~KEY_COUNT-1)
// 返回参数        uint8            KEY_PRESSED / KEY_UNPRESSED
// 使用示例        Key_GetState(0);
// 备注信息        内部静态函数，按键输入为低电平有效
//--------------------------------------------------------------------------------------------------
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
    return (level == 0) ? KEY_PRESSED : KEY_UNPRESSED;  // 低电平有效
}


// ==================================== 对外 API ====================================

//--------------------------------------------------------------------------------------------------
// 函数简介        按键引脚初始化
// 参数说明        void
// 返回参数        void
// 使用示例        Key_Init();
// 备注信息        配置按键引脚为输入上拉模式，低电平有效
//--------------------------------------------------------------------------------------------------
void Key_Init(void)
{
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        按键扫描状态机驱动
// 参数说明        void
// 返回参数        void
// 使用示例        Key_Tick();
// 备注信息        内置消抖，支持单击、长按与连发检测，无需额外延时
//--------------------------------------------------------------------------------------------------
void Key_Tick(void)
{
    static uint8  i;
    static uint8  CurrState[KEY_COUNT];                 // 当前物理状态
    static uint8  PrevState[KEY_COUNT];                 // 上一次物理状态
    static uint8  S[KEY_COUNT];                         // 状态机阶段: 0空闲 1等待 2连发
    static uint16 Time[KEY_COUNT];                      // 定时计数器

    // 第 1 步：计时器递减
    for (i = 0; i < KEY_COUNT; i++)
    {
        if (Time[i] > 0) Time[i]--;
    }

    // 第 2 步：逐键扫描并生成事件
    for (i = 0; i < KEY_COUNT; i++)
    {
        PrevState[i] = CurrState[i];
        CurrState[i] = Key_GetState(i);

        // HOLD：实时反映按键是否处于按住状态
        if (CurrState[i] == KEY_PRESSED)
            Key_Flag[i] |= KEY_HOLD;
        else
            Key_Flag[i] &= ~KEY_HOLD;

        // 边沿检测：按下与抬起瞬间各触发一次
        if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
            Key_Flag[i] |= KEY_DOWN;                    // 下降沿

        if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
            Key_Flag[i] |= KEY_UP;                      // 上升沿

        // 状态机：单击、长按、连发
        switch(S[i])
        {
            case 0:                                     // 空闲态：等待按下
                if (CurrState[i] == KEY_PRESSED)
                {
                    Time[i] = KEY_Time_LONG;            // 启动长按判定计时（1000ms）
                    S[i] = 1;
                }
                break;

            case 1:                                     // 按下态：判定单击或长按
                if (CurrState[i] == KEY_UNPRESSED)
                {
                    Key_Flag[i] |= KEY_SINGLE;          // 未超时松手，判定为单击
                    S[i] = 0;
                }
                else if (Time[i] == 0)
                {
                    Key_Flag[i] |= KEY_LONG;            // 超时未松手，判定为长按
                    Time[i] = KEY_Time_REPEAT;          // 切换为连发计时
                    S[i] = 2;
                }
                break;

            case 2:                                     // 长按态：持续连发
                if (CurrState[i] == KEY_UNPRESSED)
                {
                    S[i] = 0;                           // 松手后回到空闲态
                }
                else if (Time[i] == 0)
                {
                    Key_Flag[i] |= KEY_REPEAT;          // 每 200ms 触发一次
                    Time[i] = KEY_Time_REPEAT;
                }
                break;
        }
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        查询按键事件
// 参数说明        n                按键编号 (0~KEY_COUNT-1)
// 参数说明        Flag             需要查询的事件标志（如 KEY_SINGLE、KEY_LONG）
// 返回参数        uint8            1=事件发生 0=无事件
// 使用示例        Key_Check(KEY_NAME_UP, KEY_SINGLE);
// 备注信息        除 KEY_HOLD 外，事件读取后自动清除（一次性消费）
//--------------------------------------------------------------------------------------------------
uint8 Key_Check(uint8 n, uint8 Flag)
{
    if (Key_Flag[n] & Flag)
    {
        if (Flag != KEY_HOLD)                           // HOLD 为持续状态，不执行清除
            Key_Flag[n] &= ~Flag;
        return 1;
    }
    return 0;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        简易单击检测
// 参数说明        void
// 返回参数        uint8            发生单击的按键号 (1~4) 无事件返回 0
// 使用示例        uint8 key = Key_Check_Simple();
// 备注信息        轮询全部按键，返回第一个发生单击事件的按键号
//--------------------------------------------------------------------------------------------------
uint8 Key_Check_Simple(void)
{
    if(Key_Check(0, KEY_SINGLE)) return 1;
    if(Key_Check(1, KEY_SINGLE)) return 2;
    if(Key_Check(2, KEY_SINGLE)) return 3;
    if(Key_Check(3, KEY_SINGLE)) return 4;
    return 0;
}
