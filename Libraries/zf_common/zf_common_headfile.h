#ifndef __HEADFILE_H_
#define __HEADFILE_H_


#pragma warning disable = 115
#pragma warning disable = 188

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "intrins.h"
#include "isr.h"

#include "STC32Gxx.h"
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_fifo.h"
#include "zf_common_debug.h"
#include "zf_common_interrupt.h"
#include "zf_common_font.h"
#include "zf_common_function.h"

#include "zf_driver_uart.h"
#include "zf_driver_gpio.h"
#include "zf_driver_adc.h"
#include "zf_driver_spi.h"
#include "zf_driver_timer.h"
#include "zf_driver_pwm.h"

#include "zf_driver_exti.h"
#include "zf_driver_delay.h"
#include "zf_driver_eeprom.h"
#include "zf_driver_pit.h"
#include "zf_driver_encoder.h"

#include "zf_device_config.h"
#include "zf_device_type.h"

#include "zf_device_icm20602.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"
#include "zf_device_imu963ra.h"

#include "zf_device_tft180.h"
#include "zf_device_ips114.h"
#include "zf_device_ips200.h"
#include "zf_device_ips200pro.h"

#include "zf_device_dl1a.h"
#include "zf_device_dl1b.h"

#include "zf_device_ble6a20.h"
#include "zf_device_tsl1401.h"
#include "zf_device_wireless_uart.h"

#include "zf_device_gps_tau1201.h"


#include "seekfree_assistant.h"

#include "seekfree_assistant_interface.h"

//写了什么文件就把.h加到下面
#include "Key.h"
#include "OLED.h"
#include "Menu.h"
#include "imu_mpu6050.h"
#include "attitude.h"
#include "param.h"
#include "motor.h"
#include "pid.h"
#include "balance.h"
#include "line.h"
#include "mode.h"
#include "bluetooth.h"
#include "LED.h"
#include "Buzzer.h"
#include "filter.h"

#endif
