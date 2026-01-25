#ifndef _zf_driver_encoder_h
#define _zf_driver_encoder_h


#include "zf_common_debug.h"
#include "zf_common_clock.h"

typedef enum
{
	
    TIM0_ENCOEDER_P34  = 0x0000 | IO_P34,
    TIM1_ENCOEDER_P35  = 0x0100 | IO_P35,
	TIM2_ENCOEDER_P12  = 0x0200 | IO_P12,
	TIM3_ENCOEDER_P04  = 0x0300 | IO_P04,
	TIM4_ENCOEDER_P06  = 0x0400 | IO_P06,
}encoder_channel_enum;


typedef enum
{
	TIM0_ENCOEDER = 0,
    TIM1_ENCOEDER,
    TIM2_ENCOEDER,
    TIM3_ENCOEDER,
    TIM4_ENCOEDER,
}encoder_index_enum;


int16   encoder_get_count   (encoder_index_enum encoder_n);
void    encoder_clear_count (encoder_index_enum encoder_n);
void    encoder_dir_init    (encoder_index_enum encoder_n, gpio_pin_enum dir_pin, encoder_channel_enum lsb_pin);


#endif
