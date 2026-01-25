#ifndef __ZF_DRIVER_EEPROM_H
#define __ZF_DRIVER_EEPROM_H

#include "zf_common_typedef.h"


void iap_init(void);
void iap_idle(void);
void iap_set_tps(void);
uint8 iap_get_cmd_state(void);
uint8 iap_read_byte(uint32 addr);
void iap_write_byte(uint32 addr, uint8 byte);
void iap_read_buff(uint32 addr, uint8 *buf, uint16 len);
void iap_write_buff(uint32 addr, uint8 *buf, uint16 len);
void iap_erase_page(uint32 addr);
void extern_iap_write_buff(uint16 addr, uint8 *buf, uint16 len);


#endif


