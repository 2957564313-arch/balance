#include "zf_common_headfile.h"

void Buzzer_Init(void)
{
	gpio_init(IO_P67, GPO, 1, GPO_PUSH_PULL);
}

void Buzzer_ON(void)
{
	BUZZER_PIN = 0;
}

void Buzzer_OFF(void)
{
	BUZZER_PIN = 1;
}
