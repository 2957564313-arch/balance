#include "Buzzer.h"

// 倒计时计数器
static uint16 beep_timer = 0;

void Buzzer_Init(void)
{

    // 初始化为推挽输出，默认低电平(不响)
    // 如果你的蜂鸣器一上电就响，把这里的 0 改为 1
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);

}

// 设置鸣叫时间 (非阻塞)
// 参数 ms: 想要响多少毫秒
void Buzzer_Beep(uint16 ms)
{

    // 计算需要多少个 5ms 周期
    if(ms < 5) ms = 5;
    beep_timer = ms / 5;

    // 开启蜂鸣器 (高电平响)
    gpio_set_level(BEEP_PIN, 1); 

}

// 周期性任务 (放在 5ms 中断里)
void Buzzer_Task(void)
{
    if(beep_timer > 0)
    {
        beep_timer--;
        if(beep_timer == 0)
        {
            // 倒计时结束，关闭蜂鸣器
            gpio_set_level(BEEP_PIN, 0); 
        }
    }
}
