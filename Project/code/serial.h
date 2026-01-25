#ifndef __SERIAL_H
#define __SERIAL_H
#define UART_INDEX              (DEBUG_UART_INDEX   )         // 默认 UART_1
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)       // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )       // 默认 UART1_TX_P31
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )       // 默认 UART1_RX_P30
void uart_rx_interrupt_handler (uint8 dat);
void serial_Init(void);
extern uint8       uart_get_data[64]  ;                // 串口接收数据缓冲区

extern uint8       fifo_get_data[64]   ;         // fifo 输出读出缓冲区

extern uint32      fifo_data_count; 
extern fifo_struct uart_data_fifo;
void serial_Receive(void);



#endif
