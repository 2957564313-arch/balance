
#ifndef _zf_device_ips200_h_
#define _zf_device_ips200_h_

#include "zf_common_typedef.h"
#include "zf_device_type.h"

#define IPS200_USE_INTERFACE             (HARDWARE_SPI)                                     // 默认使用硬件 SPI 方式驱动 建议使用硬件 SPI 方式驱动
#if (IPS200_USE_INTERFACE==SOFT_SPI)                                                          // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 SPI 驱动====================================================
	// 暂不支持
//====================================================软件 SPI 驱动====================================================
#elif (IPS200_USE_INTERFACE==HARDWARE_SPI)
//====================================================硬件 SPI 驱动====================================================
	#define IPS200_SPI_SPEED                ((uint32)20 * 1000 * 1000U)             // 硬件 SPI 速率 这里设置为系统时钟二分频
	#define IPS200_SPI                      (SPI_2)                                 // 硬件 SPI 号
	#define IPS200_SCL_PIN                  (SPI2_CH2_SCLK_P25)                     // 硬件 SPI SCK 引脚
	#define IPS200_SDA_PIN                  (SPI2_CH2_MOSI_P23)                     // 硬件 SPI MOSI 引脚
//====================================================硬件 SPI 驱动====================================================
#endif

#define IPS200_RST_PIN                  (P20 )                                   // 液晶复位引脚定义
#define IPS200_DC_PIN                   (P21 )                                   // 液晶命令位引脚定义
#define IPS200_CS_PIN                   (P22 )                                   // CS 片选引脚
#define IPS200_BLK_PIN                  (P27 )                                   // 液晶背光引脚定义

#define IPS200_DEFAULT_DISPLAY_DIR      (IPS200_PORTAIT)                  		// 默认的显示方向
#define IPS200_DEFAULT_PENCOLOR         (RGB565_RED)                            // 默认的画笔颜色
#define IPS200_DEFAULT_BGCOLOR          (RGB565_WHITE)                          // 默认的背景颜色
//#define IPS200_DEFAULT_DISPLAY_FONT     (IPS200_8X16_FONT)                      // 默认的字体模式

#define IPS200_DC(x)                    ( IPS200_DC_PIN  = x )
#define IPS200_RST(x)                   ( IPS200_RST_PIN = x )
#define IPS200_CS(x)                    ( IPS200_CS_PIN  = x )
#define IPS200_BLK(x)                   ( IPS200_BLK_PIN = x )

typedef enum
{
    IPS200_PORTAIT                      = 0,                                    // 竖屏模式
    IPS200_PORTAIT_180                  = 1,                                    // 竖屏模式  旋转180
    IPS200_CROSSWISE                    = 2,                                    // 横屏模式
    IPS200_CROSSWISE_180                = 3,                                    // 横屏模式  旋转180
}ips200_dir_enum;




void    ips200_init                     (void);
void    ips200_clear                    (uint16 color);
void    ips200_set_dir                  (ips200_dir_enum dir);
void    ips200_set_color                (uint16 pen, uint16 bgcolor);
void    ips200_draw_point               (uint16 x, uint16 y, uint16 color);
															 
void    ips200_show_char                (uint16 x, uint16 y, const char dat);
void    ips200_show_string              (uint16 x, uint16 y, const char dat[]);
void    ips200_show_int8                (uint16 x, uint16 y, int8 dat);
void    ips200_show_uint8               (uint16 x, uint16 y, uint8 dat);
void    ips200_show_int16               (uint16 x, uint16 y, int16 dat);
void    ips200_show_uint16              (uint16 x, uint16 y, uint16 dat);
void    ips200_show_int32               (uint16 x, uint16 y, int32 dat, uint8 num);
void    ips200_show_float               (uint16 x, uint16 y, double dat, uint8 num, uint8 pointnum);

void    ips200_show_wave                (uint16 x, uint16 y, uint8 *p, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max);

void    ips200_init                     (void);

#endif
