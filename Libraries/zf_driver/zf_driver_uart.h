#ifndef __ZF_DRIVER_UART_H
#define __ZF_DRIVER_UART_H


#include "zf_common_typedef.h"
#include "zf_driver_timer.h"
#include "zf_driver_gpio.h"


#define	UART1_CLEAR_RX_FLAG (SCON  &= ~0x01)
#define	UART2_CLEAR_RX_FLAG (S2CON &= ~0x01)
#define	UART3_CLEAR_RX_FLAG (S3CON &= ~0x01)
#define	UART4_CLEAR_RX_FLAG (S4CON &= ~0x01)

#define	UART1_CLEAR_TX_FLAG (SCON  &= ~0x02)
#define	UART2_CLEAR_TX_FLAG (S2CON &= ~0x02)
#define	UART3_CLEAR_TX_FLAG (S3CON &= ~0x02)
#define	UART4_CLEAR_TX_FLAG (S4CON &= ~0x02)


#define UART1_GET_RX_FLAG   (SCON  & 0x01)
#define UART2_GET_RX_FLAG   (S2CON & 0x01)
#define UART3_GET_RX_FLAG   (S3CON & 0x01)
#define UART4_GET_RX_FLAG   (S4CON & 0x01)

#define UART1_GET_TX_FLAG   (SCON  & 0x02)
#define UART2_GET_TX_FLAG   (S2CON & 0x02)
#define UART3_GET_TX_FLAG   (S3CON & 0x02)
#define UART4_GET_TX_FLAG   (S4CON & 0x02)


//此枚举定义不允许用户修改
typedef enum //枚举串口号
{
    UART_1,
    UART_2,
    UART_3,
    UART_4,
	UART_RESERVE,	//仅占位使用
} uart_index_enum;


typedef enum
{
    UART_FUNCTION_INIT = 0,                  // 功能未初始化    
    UART_FUNCTION_UART,                    	 // 用作 UART
    UART_FUNCTION_SPI,                       // 用作 SPI
}uart_function_enum;



//此枚举定义不允许用户修改
typedef enum //枚举串口引脚
{
    UART1_RX_P30 = 0x0000 | IO_P30, UART1_TX_P31 = 0x0000 | IO_P31,		//只能使用同一行的RX和TX引脚号。不允许混用
    UART1_RX_P36 = 0x0100 | IO_P36, UART1_TX_P37 = 0x0100 | IO_P37,		//例如:UART1_RX_P30,UART1_TX_P37。这样不行。
    UART1_RX_P16 = 0x0200 | IO_P16, UART1_TX_P17 = 0x0200 | IO_P17,
    UART1_RX_P43 = 0x0300 | IO_P43, UART1_TX_P44 = 0x0300 | IO_P44,
                                                               
    UART2_RX_P10 = 0x1000 | IO_P10, UART2_TX_P11 = 0x1000 | IO_P11,
    UART2_RX_P46 = 0x1100 | IO_P46, UART2_TX_P47 = 0x1100 | IO_P47,
                                                               
    UART3_RX_P00 = 0x2000 | IO_P00, UART3_TX_P01 = 0x2000 | IO_P01,
    UART3_RX_P50 = 0x2100 | IO_P50, UART3_TX_P51 = 0x2100 | IO_P51,
                                                               
    UART4_RX_P02 = 0x3000 | IO_P02, UART4_TX_P03 = 0x3000 | IO_P03,
    UART4_RX_P52 = 0x3100 | IO_P52, UART4_TX_P53 = 0x3100 | IO_P53,
} uart_pin_enum;



typedef struct
{
	uint32 dma_urxt_cfg;
	uint32 dma_urxt_cr;
	uint32 dma_urxt_sta;
	uint32 dma_urxt_amt;
	uint32 dma_urxt_done;
	uint32 dma_urxt_txah;
	uint32 dma_urxt_txal;
	
	uint32 dma_urxt_amth;

	
	uint32 dma_urxr_cfg;
	uint32 dma_urxr_cr;
	uint32 dma_urxr_sta;
	uint32 dma_urxr_amt;
	uint32 dma_urxr_done;
	uint32 dma_urxr_rxah;
	uint32 dma_urxr_rxal;

	uint32 dma_urxr_amth;
	
}uart_addr_struct;




extern uint8 xdata uart_rx_buff[UART_RESERVE][1];

uint8 	uart_funciton_check 	(uart_index_enum index, uart_function_enum mode);
void 	uart_rx_start_buff		(uart_index_enum uart_n);

void    uart_write_byte         (uart_index_enum uart_n, const uint8 dat);
void    uart_write_buffer       (uart_index_enum uart_n, const uint8 *buff, uint16 len);
void    uart_write_string       (uart_index_enum uart_n, const char *str);

uint8   uart_read_byte          (uart_index_enum uart_n);
uint8   uart_query_byte         (uart_index_enum uart_n, uint8 *dat);

//void    uart_tx_interrupt       (uart_index_enum uart_n, uint8 status); 暂不支持TX中断
void    uart_rx_interrupt       (uart_index_enum uart_n, uint8 status);

void    uart_init               (uart_index_enum uart_n, uint32 baud, uart_pin_enum tx_pin, uart_pin_enum rx_pin);


#endif