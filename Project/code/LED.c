#include "LED.h"

void LED_Init(void)
{
    // 初始化为推挽输出，默认高电平(灭)
    gpio_init(LED_PIN, GPO, 1, GPO_PUSH_PULL);
}

void LED_ON(void)
{
    gpio_set_level(LED_PIN, 0); // 低电平亮
}

void LED_OFF(void)
{
    gpio_set_level(LED_PIN, 1); // 高电平灭
}
