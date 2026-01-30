#ifndef _zf_common_debug_h_
#define _zf_common_debug_h_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"


#include "zf_common_typedef.h"
#define PRINTF_ENABLE               (1)                                          // 使能printf

// 如果修改串口并开启了 debug UART 的中断接收 需要同步更换 debug_interrupr_handler 函数到对应的中断服务函数
// 如果修改串口并开启了 debug UART 的中断接收 需要同步更换 debug_interrupr_handler 函数到对应的中断服务函数
// 如果修改串口并开启了 debug UART 的中断接收 需要同步更换 debug_interrupr_handler 函数到对应的中断服务函数
#define DEBUG_UART_INDEX            (UART_1)                               	// 指定 debug uart 所使用的的串口
#define DEBUG_UART_BAUDRATE         (115200)                				// 指定 debug uart 所使用的的串口波特率
#define DEBUG_UART_TX_PIN           (UART1_TX_P31 )                        	// 指定 debug uart 所使用的的串口引脚
#define DEBUG_UART_RX_PIN           (UART1_RX_P30 )                        	// 指定 debug uart 所使用的的串口引脚

#define DEBUG_UART_USE_INTERRUPT    (1)                                         // 是否启用 debug uart 接收中断

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     断言
// 参数说明     x           判断是否触发断言 0-触发断言 1-不触发断言
// 返回参数     void
// 使用示例     zf_assert(0);
// 备注信息     一般用于参数判断 zf_assert(0) 就断言报错
//              默认情况下会在 Debug UART 输出
//              但如果使用开源库内屏幕接口初始化了屏幕 则会在屏幕上显示
//-------------------------------------------------------------------------------------------------------------------
#define zf_assert(x)                (debug_assert_handler((x), __FILE__, __LINE__))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Log 信息输出
// 参数说明     x           判断是否触发输出 0-触发输出 1-不触发输出
// 参数说明     *str        需要输出的 Log 信息
// 返回参数     void
// 使用示例     printf( "Error");
// 备注信息     调试信息输出 用来做一些报错或者警告之类的输出
//              默认情况下会在 Debug UART 输出
//              但如果使用开源库内屏幕接口初始化了屏幕 则会在屏幕上显示
//-------------------------------------------------------------------------------------------------------------------
#define zf_log(x, str)              (debug_log_handler((x), (str), __FILE__, __LINE__))


//typedef struct
//{
//    uint16 type_index;

//    uint16 display_x_max;
//    uint16 display_y_max;

//    uint8 font_x_size;
//    uint8 font_y_size;

//    void (*output_uart)             (const char *str);
//    void (*output_screen)           (uint16 x, uint16 y, const char *str);
//    void (*output_screen_clear)     (void);
//}debug_output_struct;

uint32      debug_send_buffer(const uint8 *buff, uint32 len);
#if DEBUG_UART_USE_INTERRUPT                                                    // 如果启用 debug uart 接收中断
#define     DEBUG_RING_BUFFER_LEN   (64)                                        // 定义环形缓冲区大小 默认 64byte
#endif

uint32      debug_read_buffer(uint8 *buff, uint32 len);
//void        debug_assert_enable         (void);
//void        debug_assert_disable        (void);
void        debug_assert_handler        (uint8 pass, char *file, int line);
void        debug_log_handler           (uint8 pass, char *str, char *file, int line);
//void        debug_output_struct_init    (debug_output_struct *info);
//void        debug_output_init           (debug_output_struct *info);
void        debug_init                  (void);

#endif