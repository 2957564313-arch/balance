#ifndef __ZF_BLUETOOTH_H
#define __ZF_BLUETOOTH_H

// 引入逐飞库核心头文件（C251兼容）
#include "zf_common_headfile.h"

// 全局变量声明（逐飞库风格）
extern char    Serial_RxPacket[100];
extern uint8   Serial_RxFlag;

//-------------------------------------------------------------------------------------------------------------------
// 函数声明（逐飞库风格，C251兼容）
//-------------------------------------------------------------------------------------------------------------------
/**
 * @brief 蓝牙串口初始化（HC-04，UART4，P02/P03，9600波特率）
 * @param 无
 * @return 无
 */
void bluetooth_init(void);

/**
 * @brief 发送单个字节
 * @param dat 要发送的字节
 * @return 无
 */
void bluetooth_send_byte(uint8 dat);

/**
 * @brief 发送指定长度数组
 * @param buff 数组首地址
 * @param len  数组长度
 * @return 无
 */
void bluetooth_send_array(uint8 *buff, uint16 len);

/**
 * @brief 发送字符串
 * @param str 字符串首地址
 * @return 无
 */
void bluetooth_send_string(char *str);

/**
 * @brief 幂运算（用于数字转字符串）
 * @param X 底数
 * @param Y 指数
 * @return 运算结果
 */
uint32 bluetooth_pow(uint32 X, uint8 Y);

/**
 * @brief 发送数字（指定位数）
 * @param num 要发送的数字
 * @param len 数字位数
 * @return 无
 */
void bluetooth_send_number(uint32 num, uint8 len);

/**
 * @brief 格式化打印函数（支持printf风格）
 * @param format 格式化字符串
 * @param ...    可变参数
 * @return 无
 */
void bluetooth_printf(char *format, ...);

/**
 * @brief UART4中断处理函数（解析[数据包]）
 * @param 无
 * @return 无
 */
void uart4_isr_handler(void);

#endif // __ZF_BLUETOOTH_H