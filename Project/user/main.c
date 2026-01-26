#include "zf_common_headfile.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();

    while (1)
    {

        system_delay_ms(200);
    }
}