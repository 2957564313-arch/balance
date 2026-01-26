#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "zf_common_headfile.h"

// ========== 蓝牙模块串口配置（可单独定义，区分电脑调试串口） ==========
#define UART_INDEX              (DEBUG_UART_INDEX   )         // 蓝牙模块串口（若和电脑共用UART1则不变）
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)       // 蓝牙模块波特率（通常9600/115200）
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )       // 蓝牙TX引脚
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )       // 蓝牙RX引脚

// ========== 函数声明 ==========
void uart_rx_interrupt_handler (uint8 dat);        // 中断回调函数
void serial_Init(void);                           // 串口初始化
void serial_Receive(void);                        // 接收数据并转发给蓝牙
void Serial_SendToBluetooth(uint8 dat);            // 向蓝牙发送单个字节
void Serial_SendStringToBluetooth(uint8 *str);     // 向蓝牙发送字符串

// ========== 全局变量声明 ==========
extern uint8       uart_get_data[64];              // FIFO接收缓冲区
extern uint8       fifo_get_data[64];              // FIFO读取缓冲区
extern uint8       Serial_RxPacket[64];            // [xxx]数据包缓冲区
extern uint32      fifo_data_count;                // FIFO数据计数
extern fifo_struct uart_data_fifo;                 // FIFO结构体
extern uint8       Serial_RxFlag;                  // 数据包接收完成标志

#endif