#ifndef _zf_driver_soft_spi_h_
#define _zf_driver_soft_spi_h_

#include "zf_driver_gpio.h"

#define SOFT_SPI_PIN_NULL       (0xFFFF)                                        // 用于区分是否分配引脚

typedef struct
{
	uint8       mode     ;                                         // SPI 模式
	uint8       use_miso ;                                         // 是否使用 MISO 引脚}
	uint8       use_cs   ;                                         // 是否使用 CS 引脚	
}config_info_struct;


typedef struct
{
	config_info_struct  config;
    gpio_pin_enum       sck_pin;                                                // 用于记录对应的引脚编号
    gpio_pin_enum       mosi_pin;                                               // 用于记录对应的引脚编号
    gpio_pin_enum       miso_pin;                                               // 用于记录对应的引脚编号
    gpio_pin_enum       cs_pin;                                                 // 用于记录对应的引脚编号
    uint32              delay;                                                  // 模拟 SPI 软延时时长
}soft_spi_info_struct;

void        soft_spi_write_8bit                 (soft_spi_info_struct *soft_spi_obj, const uint8 dat);
void        soft_spi_write_8bit_array           (soft_spi_info_struct *soft_spi_obj, const uint8 *dat, uint32 len);

void        soft_spi_write_16bit                (soft_spi_info_struct *soft_spi_obj, const uint16 dat);
void        soft_spi_write_16bit_array          (soft_spi_info_struct *soft_spi_obj, const uint16 *dat, uint32 len);

void        soft_spi_write_8bit_register        (soft_spi_info_struct *soft_spi_obj, const uint8 register_name, const uint8 dat);
void        soft_spi_write_8bit_registers       (soft_spi_info_struct *soft_spi_obj, const uint8 register_name, const uint8 *dat, uint32 len);

void        soft_spi_write_16bit_register       (soft_spi_info_struct *soft_spi_obj, const uint16 register_name, const uint16 dat);
void        soft_spi_write_16bit_registers      (soft_spi_info_struct *soft_spi_obj, const uint16 register_name, const uint16 *dat, uint32 len);

uint8       soft_spi_read_8bit                  (soft_spi_info_struct *soft_spi_obj);
void        soft_spi_read_8bit_array            (soft_spi_info_struct *soft_spi_obj, uint8 *dat, uint32 len);

uint16      soft_spi_read_16bit                 (soft_spi_info_struct *soft_spi_obj);
void        soft_spi_read_16bit_array           (soft_spi_info_struct *soft_spi_obj, uint16 *dat, uint32 len);

uint8       soft_spi_read_8bit_register         (soft_spi_info_struct *soft_spi_obj, const uint8 register_name);
void        soft_spi_read_8bit_registers        (soft_spi_info_struct *soft_spi_obj, const uint8 register_name, uint8 *dat, uint32 len);

uint16      soft_spi_read_16bit_register        (soft_spi_info_struct *soft_spi_obj, const uint16 register_name);
void        soft_spi_read_16bit_registers       (soft_spi_info_struct *soft_spi_obj, const uint16 register_name, uint16 *dat, uint32 len);

void        soft_spi_transfer_8bit              (soft_spi_info_struct *soft_spi_obj, const uint8 *write_buffer, uint8 *read_buffer, uint32 len);
void        soft_spi_transfer_16bit             (soft_spi_info_struct *soft_spi_obj, const uint16 *write_buffer, uint16 *read_buffer, uint32 len);

void        soft_spi_init                       (soft_spi_info_struct *soft_spi_obj, uint8 mode, uint32 delay, gpio_pin_enum sck_pin, gpio_pin_enum mosi_pin, uint32 miso_pin, uint32 cs_pin);

#endif
