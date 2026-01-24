#include "zf_common_headfile.h"
#include "zf_device_oled.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();

    OLED_Init();

    OLED_Clear();
    OLED_ShowString(1, 1, "HELLO");
    OLED_ShowNum(2, 1, 12345, 5);
    OLED_ShowSignedNum(3, 1, -678, 3);
    OLED_ShowFloat(4, 1, 3.1415f, 1, 3);

    OLED_Update();   

    while(1)
    {
        system_delay_ms(100);
    }
}
