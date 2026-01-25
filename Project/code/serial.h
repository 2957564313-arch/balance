#ifndef __SERIAL_H
#define __SERIAL_H

#define UART_INDEX              (DEBUG_UART_INDEX   )         // Ĭ�� UART_1
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)       // Ĭ�� 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )       // Ĭ�� UART1_TX_P31
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )       // Ĭ�� UART1_RX_P30

void uart_rx_interrupt_handler (uint8 dat);
void serial_Init(void);

extern uint8       uart_get_data[64]  ;                // ���ڽ������ݻ�����

extern uint8       fifo_get_data[64]   ;         // fifo �������������

extern uint32      fifo_data_count; 
extern fifo_struct uart_data_fifo;

void serial_Receive(void);

#endif
