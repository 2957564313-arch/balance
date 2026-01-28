#ifndef _zf_driver_flash_h
#define _zf_driver_flash_h

#include "zf_driver_eeprom.h"
#include "zf_common_typedef.h"


#define FLASH_BASE_ADDR             (0x00)               		// FALSH首地址
#define FLASH_MAX_PAGE_INDEX        (3)
#define FLASH_MAX_SECTION_INDEX     (63)
#define FLASH_PAGE_SIZE             (0x00000100)                // 256  byte
#define FLASH_SECTION_SIZE          (FLASH_PAGE_SIZE*4)         // 1024 byte
#define FLASH_OPERATION_TIME_OUT    0x00FF

#define FLASH_DATA_BUFFER_SIZE      (FLASH_PAGE_SIZE/sizeof(flash_data_union))  // 自动计算每个页能够存下多少个数据

typedef union                                                                   // 固定的数据缓冲单元格式
{
    float   float_type;                                                       // float  类型
    uint32  uint32_type;                                                      // uint32 类型
    int32   int32_type;                                                       // int32  类型
    uint16  uint16_type;                                                      // uint16 类型
    int16   int16_type;                                                       // int16  类型
    uint8   uint8_type;                                                       // uint8  类型
    int8    int8_type;                                                        // int8   类型
}flash_data_union;                                                            // 所有类型数据共用同一个 32bit 地址


/*
  STC-ISP可以调整EEPROM大小，默认使用1K。最大可以设置为128K。
 一个扇区256个字节。
*/


extern flash_data_union flash_union_buffer[FLASH_DATA_BUFFER_SIZE];

uint8   flash_check                         (uint32 sector_num, uint32 page_num);
uint8   flash_erase_sector                  (uint32 sector_num, uint32 page_num);
void    flash_read_page                     (uint32 sector_num, uint32 page_num, uint32 *buf, uint16 len);
uint8   flash_write_page                    (uint32 sector_num, uint32 page_num, const uint32 *buf, uint16 len);

void    flash_read_page_to_buffer           (uint32 sector_num, uint32 page_num);
uint8   flash_write_page_from_buffer        (uint32 sector_num, uint32 page_num);
void    flash_buffer_clear                  (void);

#endif
