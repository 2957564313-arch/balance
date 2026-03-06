#include "zf_common_headfile.h"

extern volatile uint32 bt_rxloss_cnt;
extern volatile uint32 g_ms_tick;

/* UART1 DMA RX 中断 */
void DMA_UART1_IRQHandler(void) interrupt 4
{
    if (DMA_UR1R_STA & 0x01)
    {
        DMA_UR1R_STA &= (uint8)(~0x01);
        uart_rx_start_buff(UART_1);

        if (uart1_irq_handler != NULL)
        {
            uart1_irq_handler(uart_rx_buff[UART_1][0]);
        }
    }

    if (DMA_UR1R_STA & 0x02)
    {
        DMA_UR1R_STA &= (uint8)(~0x02);
        uart_rx_start_buff(UART_1);
    }
}

/* UART4 DMA RX 中断：蓝牙调参 */
void DMA_UART4_IRQHandler(void) interrupt 18
{
    uint8 sta;
    uint8 dat;

    sta = (uint8)(DMA_UR4R_STA & 0x03u);
    if (sta)
    {
        dat = uart_rx_buff[UART_4][0];

        /* 清除 0x01/0x02 两个标志：否则会一直进中断=假死 */
        DMA_UR4R_STA &= (uint8)(~0x03u);

        uart_rx_start_buff(UART_4);

        if (uart4_irq_handler != NULL)
        {
            uart4_irq_handler(dat);
        }
    }
}

//TIM0
void TM0_IRQHandler() interrupt 1
{
    TIM0_CLEAR_FLAG;
    if (tim0_irq_handler != NULL)
    {
        tim0_irq_handler();
    }
}

//TIM1
void TM1_IRQHandler() interrupt 3
{
    TIM1_CLEAR_FLAG;
    if (tim1_irq_handler != NULL)
    {
        tim1_irq_handler();
    }
}


void USER_IRQHandler(void) interrupt 13
{
    unsigned char f0;
    unsigned char f3;

    P_SW2 |= 0x80;

    f0 = P0INTF;
    if (f0)
    {
        P0INTF = 0x00u;
    }

    f3 = P3INTF;
    if (f3)
    {
        P3INTF = 0x00u;
    }
}
