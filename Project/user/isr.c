#include "zf_common_headfile.h"

// =========================================================================
// UART1 中断：【保留】用于无线下载/调试
// =========================================================================
void DMA_UART1_IRQHandler(void) interrupt 4
{
    static vuint8 dwon_count = 0;

    if (DMA_UR1R_STA & 0x01) // 接收完成
    {
        DMA_UR1R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_1); // 设置下一次接收，务必保留

        // 程序自动下载逻辑 (收到 0x7F 复位进 ISP)
        if (uart_rx_buff[UART_1][0] == 0x7F)
        {
            if (dwon_count++ > 20)
            {
                IAP_CONTR = 0x60;
            }
        }
        else
        {
            dwon_count = 0;
        }

        if (uart1_irq_handler != NULL)
        {
            uart1_irq_handler(uart_rx_buff[UART_1][0]);
        }
    }

    if (DMA_UR1R_STA & 0x02) // 数据丢弃
    {
        DMA_UR1R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_1); // 设置下一次接收
    }
}

// =========================================================================
// UART2 中断：【未使用，注释掉】
// =========================================================================
/*
void DMA_UART2_IRQHandler(void) interrupt 8
{
    if (DMA_UR2R_STA & 0x01) // 接收完成
    {
        DMA_UR2R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_2); // 设置下一次接收

        if (uart2_irq_handler != NULL)
        {
            uart2_irq_handler(uart_rx_buff[UART_2][0]);
        }
    }

    if (DMA_UR2R_STA & 0x02) // 数据丢弃
    {
        DMA_UR2R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_2); // 设置下一次接收
    }
}
*/

// =========================================================================
// UART3 中断：【未使用，注释掉】
// 注意：原计划用于蓝牙，但因 P0.0/P0.1 冲突已改用 UART4
// =========================================================================
/*
void DMA_UART3_IRQHandler(void) interrupt 17
{
    if (DMA_UR3R_STA & 0x01) // 接收完成
    {
        DMA_UR3R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_3); // 设置下一次接收

        if (uart3_irq_handler != NULL)
        {
            uart3_irq_handler(uart_rx_buff[UART_3][0]);
        }
    }

    if (DMA_UR3R_STA & 0x02) // 数据丢弃
    {
        DMA_UR3R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_3); // 设置下一次接收
    }
}
*/

// =========================================================================
// UART4 中断：【保留】用于蓝牙模块 (HC-04 接 P0.2/P0.3)
// =========================================================================
void DMA_UART4_IRQHandler(void) interrupt 18
{
    if (DMA_UR4R_STA & 0x01) // 接收完成
    {
        DMA_UR4R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_4); // 设置下一次接收，务必保留

        if (uart4_irq_handler != NULL)
        {
            uart4_irq_handler(uart_rx_buff[UART_4][0]);
        }
    }

    if (DMA_UR4R_STA & 0x02) // 数据丢弃
    {
        DMA_UR4R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_4); // 设置下一次接收
    }
}

// =========================================================================
// 定时器中断：【必须注释掉！】
// 原因：我们在 main.c 中直接使用了 interrupt 1 和 3 接管了控制权
// 如果不注释，会报 MULTIPLE PUBLIC DEFINITIONS 错误
// =========================================================================

/*
void TM0_IRQHandler() interrupt 1
{
    TIM0_CLEAR_FLAG;
    if (tim0_irq_handler != NULL) tim0_irq_handler();
}
*/

/*
void TM1_IRQHandler() interrupt 3
{
    TIM1_CLEAR_FLAG;
    if (tim1_irq_handler != NULL) tim1_irq_handler();
}
*/

/*
void TM2_IRQHandler() interrupt 12
{
    TIM2_CLEAR_FLAG;
    if (tim2_irq_handler != NULL) tim2_irq_handler();
}
*/

/*
void TM3_IRQHandler() interrupt 19
{
    TIM3_CLEAR_FLAG;
    if (tim3_irq_handler != NULL) tim3_irq_handler();
}
*/

/*
void TM4_IRQHandler() interrupt 20
{
    TIM4_CLEAR_FLAG;
    if (tim4_irq_handler != NULL) tim4_irq_handler();
}
*/
