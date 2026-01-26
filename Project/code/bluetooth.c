#include "zf_common_headfile.h"
#include "bluetooth.h"

// ========== 全局变量定义 ==========
uint8       uart_get_data[64] = {0};                   // FIFO接收缓冲区（蓝牙模块串口）
uint8       fifo_get_data[64] = {0};                   // FIFO读取缓冲区
uint8       Serial_RxPacket[64] = {0};                 // [xxx]数据包解析缓冲区
uint32      fifo_data_count = 0;  
fifo_struct uart_data_fifo = {0};

// ========== 数据包解析专用变量 ==========
static uint8 RxState = 0;        // 解析状态：0-等待起始符，1-接收数据
static uint8 pRxPacket = 0;      // 数据包缓冲区指针
uint8       Serial_RxFlag = 0;   // 数据包接收完成标志

/**
 * @brief 向蓝牙模块发送单个字节（ZF库API）
 * @param dat 要发送给蓝牙的字节
 */
void Serial_SendToBluetooth(uint8 dat)
{
    // 用ZF库的uart_write_byte发送数据到蓝牙串口（UART_INDEX=蓝牙模块对应的串口）
    uart_write_byte(UART_INDEX, dat);
}

/**
 * @brief 向蓝牙模块发送字符串（ZF库API）
 * @param str 要发送给蓝牙的字符串
 */
void Serial_SendStringToBluetooth(uint8 *str)
{
    uint8 i = 0;
    while(str[i] != '\0')
    {
        Serial_SendToBluetooth(str[i]);
        i++;
    }
    // 可选：添加换行符（蓝牙模块常用\r\n）
    Serial_SendToBluetooth('\r');
    Serial_SendToBluetooth('\n');
}

/**
 * @brief 串口接收中断回调（接收蓝牙模块/电脑发来的数据）
 * @note  接收的数据先解析，再决定是否转发给蓝牙/电脑
 */
void uart_rx_interrupt_handler (uint8 dat)
{
    // 1. 读取串口接收到的字节（来自蓝牙模块/电脑）
    uart_query_byte(UART_INDEX, &dat);                                     
    
    // 2. [xxx]数据包解析逻辑
    if (RxState == 0)
    {
        if (dat == '[')
        {
            RxState = 1;
            pRxPacket = 0;
            memset(Serial_RxPacket, 0, sizeof(Serial_RxPacket));
        }
    }
    else if (RxState == 1)
    {
        if (dat == ']')
        {
            RxState = 0;
            Serial_RxPacket[pRxPacket] = '\0';
            Serial_RxFlag = 1;
            // 解析到完整数据包后，发送给蓝牙模块
            Serial_SendStringToBluetooth((uint8*)"Received: ");
            Serial_SendStringToBluetooth((uint8*)Serial_RxPacket);
        }
        else if (pRxPacket < 63)
        {
            Serial_RxPacket[pRxPacket] = dat;
            pRxPacket++;
        }
    }
    
    // 3. 写入FIFO缓冲区（保留FIFO机制，方便后续处理）
    fifo_write_buffer(&uart_data_fifo, &dat, 1);                           
}  

/**
 * @brief 串口初始化（蓝牙模块串口）
 */
void serial_Init(void)
{
    // 1. 初始化FIFO接收缓冲区
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);             
    
    // 2. 初始化蓝牙模块对应的串口
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    
    // 3. 开启串口接收中断
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);   	
    
    // 4. 绑定中断回调函数
    uart1_irq_handler = uart_rx_interrupt_handler;	
}

/**
 * @brief 串口接收处理（转发FIFO数据到蓝牙模块）
 * @note  把接收到的原始数据直接发送给蓝牙模块，而非电脑
 */
void serial_Receive(void)
{
    // 读取FIFO中的原始数据
    fifo_data_count = fifo_used(&uart_data_fifo);    
    while(fifo_data_count > 0)
    {
        // 从FIFO读取数据
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
        
        // 把数据发送给蓝牙模块（替换原printf，核心修正点）
        uart_write_buffer(UART_INDEX, fifo_get_data, fifo_data_count);
    }
}