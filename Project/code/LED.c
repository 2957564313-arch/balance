#include "zf_common_headfile.h"

void LED_Init(void)
{
	gpio_init(LED_PIN, GPO, 1, GPO_PUSH_PULL);
}

void LED_ON(void)
{
	LED_PIN = 0;
}

void LED_OFF(void)
{
	LED_PIN = 1;
}
