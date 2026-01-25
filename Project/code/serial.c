#include "zf_common_headfile.h"
#include "serial.h"
uint8       uart_get_data[64] = {0};                   // 串口接收数据缓冲区

uint8       fifo_get_data[64] = {0};                 // fifo 输出读出缓冲区

uint32      fifo_data_count = 0;  
fifo_struct uart_data_fifo = {0};
void uart_rx_interrupt_handler (uint8 dat)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_INDEX, &dat);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &dat, 1);                           // 将数据写入 fifo 中
}  
void serial_Init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
	uart_rx_interrupt(UART_INDEX, ZF_ENABLE);   	
	uart1_irq_handler = uart_rx_interrupt_handler;	
}
void serial_Receive(void)
{
	fifo_data_count = fifo_used(&uart_data_fifo);    
	while(fifo_data_count)
	{
       fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
		printf("%s",fifo_get_data);
	}
}
