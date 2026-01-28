#include "zf_common_debug.h"
#include "zf_common_interrupt.h"
#include "zf_common_clock.h"

#include "zf_driver_flash.h"

flash_data_union flash_union_buffer[FLASH_DATA_BUFFER_SIZE];               // FLASH 操作的数据缓冲区

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     校验 FLASH 是否有数据
// 参数说明     sector_num      需要写入的扇区编号 参数范围 <0 - 63>
// 参数说明     page_num        当前扇区页的编号   参数范围 <0 - 3>
// 返回参数     uint8           1-有数据 0-没有数据 如果需要对有数据的区域写入新的数据则应该对所在扇区进行擦除操作
// 使用示例     flash_check(63, 3);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 flash_check (uint32 sector_num, uint32 page_num)
{
    //zf_assert(sector_num <= FLASH_MAX_SECTION_INDEX);                                                   // 参数范围 0-63
    //zf_assert(page_num <= FLASH_MAX_PAGE_INDEX);                                                        // 参数范围 0-3

    uint8  return_state = 0;
    uint16 temp_loop;
    uint32 flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_num+FLASH_PAGE_SIZE*page_num));     // 提取当前 Flash 地址

	interrupt_global_disable();

    //clock_reset();                                  // 复位时钟
    //clock_set_freq(SYSTEM_CLOCK_120M);          // 设置系统频率为120Mhz

    for(temp_loop = 0; temp_loop < FLASH_PAGE_SIZE; temp_loop+=4)                                       // 循环读取 Flash 的值
    {
        if( (*(__IO u32*) (flash_addr+temp_loop)) != 0xFFFFFFFF )                                       // 该单片机擦除后如果不是 0xE339E339 那就是有值
        {
            return_state = 1;
            break;
        }
    }

    //clock_reset();                                  // 复位时钟
    //clock_set_freq(system_clock);       // 设置回原来的系统频率
    interrupt_global_enable();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     擦除一个扇区数据(4KB)
// 参数说明     sector_num      需要写入的扇区编号 参数范围 <0 - 63>
// 参数说明     page_num        当前扇区页的编号   参数范围 <0 - 3>
// 返回参数     uint8           1-表示失败 0-表示成功
// 使用示例     flash_erase_page(63, 3);
// 备注信息
//          标准擦除只能是擦一个扇区的数据，4KB字节长度
//-------------------------------------------------------------------------------------------------------------------
uint8 flash_erase_sector (uint32 sector_num, uint32 page_num)
{
    //zf_assert(sector_num <= FLASH_MAX_SECTION_INDEX);                                                   // 参数范围 0-63
    //zf_assert(page_num <= FLASH_MAX_PAGE_INDEX);                                                        // 参数范围 0-3

    uint8 return_state = 0;

    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32 flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_num+FLASH_PAGE_SIZE*page_num));     // 提取当前 Flash 地址

    uint32 primask = interrupt_global_disable();
    //clock_reset();                      // 复位时钟
    //clock_set_freq(SYSTEM_CLOCK_120M);          // 设置系统频率为120Mhz

    FLASH_Unlock();                                                                                     // 解锁 Flash
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);                           // 清除操作标志
    gFlashStatus = FLASH_ErasePage(flash_addr);                                                         // 擦除
    FLASH_ClearFlag(FLASH_FLAG_EOP );                                                                   // 清楚操作标志
    FLASH_Lock();                                                                                       // 锁定 Flash
    if(gFlashStatus != FLASH_COMPLETE)          // 判断操作是否成功
    {
        return_state = 1;
    }

    //clock_reset();                      // 复位时钟
    //clock_set_freq(system_clock);       // 设置回原来的系统频率
    interrupt_global_enable(primask);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取一页
// 参数说明     sector_num      需要写入的扇区编号 参数范围 <0 - 63>
// 参数说明     page_num        当前扇区页的编号   参数范围 <0 - 3>
// 参数说明     buf             需要读取的数据地址   传入的数组类型必须为uint32
// 参数说明     len             需要写入的数据长度   参数范围 1-256
// 返回参数     void
// 使用示例     flash_read_page(63, 3, data_buffer, 256);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void flash_read_page (uint32 sector_num, uint32 page_num, uint32 *buf, uint16 len)
{
    //zf_assert(sector_num <= FLASH_MAX_SECTION_INDEX);                                                   // 参数范围 0-63
    //zf_assert(page_num <= FLASH_MAX_PAGE_INDEX);                                                        // 参数范围 0-3
    //zf_assert(len <= FLASH_DATA_BUFFER_SIZE);

    uint16 temp_loop = 0;
    uint32 flash_addr = 0;
    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_num+FLASH_PAGE_SIZE*page_num));            // 提取当前 Flash 地址

    uint32 primask = interrupt_global_disable();
    //clock_reset();                      // 复位时钟
    //clock_set_freq(SYSTEM_CLOCK_120M);          // 设置系统频率为120Mhz


    for(temp_loop = 0; temp_loop < len; temp_loop++)                                                    // 根据指定长度读取
    {
        *buf++ = *(__IO uint32*)(flash_addr+temp_loop*4);                                               // 循环读取 Flash 的值
    }
                                                                                       // 锁定 Flash
    //clock_reset();                      // 复位时钟
    //clock_set_freq(system_clock);       // 设置回原来的系统频率
    interrupt_global_enable(primask);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编程一页
// 参数说明     sector_num      需要写入的扇区编号 参数范围 <0 - 63>
// 参数说明     page_num        当前扇区页的编号   参数范围 <0 - 3>
// 参数说明     buf             需要写入的数据地址   传入的数组类型必须为 uint32
// 参数说明     len             需要写入的数据长度   参数范围 1-256
// 返回参数     uint8           1-表示失败 0-表示成功
// 使用示例     flash_write_page(63, 3, data_buffer, 256);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 flash_write_page (uint32 sector_num, uint32 page_num, const uint32 *buf, uint16 len)
{
    //zf_assert(sector_num <= FLASH_MAX_SECTION_INDEX);                                                   // 参数范围 0-63
    //zf_assert(page_num <= FLASH_MAX_PAGE_INDEX);                                                        // 参数范围 0-3
    //zf_assert(len <= FLASH_DATA_BUFFER_SIZE);
    uint8 return_state = 0;
    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32 flash_addr = 0;
    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_num+FLASH_PAGE_SIZE*page_num));            // 提取当前 Flash 地址

    if(flash_check(sector_num, page_num))                                                               // 判断是否有数据 这里是冗余的保护 防止有人没擦除就写入
    {
        flash_erase_sector(sector_num, page_num);                                                       // 擦除这一扇区
    }

    uint32 primask = interrupt_global_disable();
    //clock_reset();                      // 复位时钟
    //clock_set_freq(SYSTEM_CLOCK_120M);          // 设置系统频率为120Mhz
    FLASH_Unlock();                                                                                     // 解锁 Flash
    while(len--)                                                                                        // 根据长度
    {
        gFlashStatus = FLASH_ProgramWord(flash_addr, *buf++);                                           // 按字 32bit 写入数据
        if(gFlashStatus != FLASH_COMPLETE)                                                              // 反复确认操作是否成功
        {
            return_state = 1;
            break;
        }
        flash_addr += 4;                                                                                // 地址自增
    }
    FLASH_Lock();                                                                                       // 锁定 Flash
    //clock_reset();                      // 复位时钟
    //clock_set_freq(system_clock);       // 设置回原来的系统频率
    interrupt_global_enable(primask);

    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从指定 FLASH 的扇区的指定页码读取数据到缓冲区
// 参数说明     sector_num      需要写入的扇区编号 参数范围 <0 - 63>
// 参数说明     page_num        当前扇区页的编号   参数范围 <0 - 3>
// 返回参数     void
// 使用示例     flash_read_page_to_buffer(63, 3);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void flash_read_page_to_buffer (uint32 sector_num, uint32 page_num)
{
    //zf_assert(sector_num <= FLASH_MAX_SECTION_INDEX);                                                   // 参数范围 0-63
    //zf_assert(page_num <= FLASH_MAX_PAGE_INDEX);                                                        // 参数范围 0-3
    uint16 temp_loop;
    uint32 flash_addr = ((FLASH_BASE_ADDR + FLASH_SECTION_SIZE*sector_num + FLASH_PAGE_SIZE*page_num)); // 提取当前 Flash 地址



    for(temp_loop = 0; temp_loop < FLASH_DATA_BUFFER_SIZE; temp_loop++)                                 // 根据指定长度读取
    {
        flash_union_buffer[temp_loop].uint32_type = *(__IO uint32*)(flash_addr+temp_loop*4);            // 循环读取 Flash 的值
    }


}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向指定 FLASH 的扇区的指定页码写入缓冲区的数据
// 参数说明     sector_num      需要写入的扇区编号 参数范围 <0 - 63>
// 参数说明     page_num        当前扇区页的编号   参数范围 <0 - 3>
// 返回参数     uint8           1-表示失败 0-表示成功
// 使用示例     flash_write_page_from_buffer(63, 3);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 flash_write_page_from_buffer (uint32 sector_num, uint32 page_num)
{
    //zf_assert(sector_num <= FLASH_MAX_SECTION_INDEX);                                                   // 参数范围 0-63
    //zf_assert(page_num <= FLASH_MAX_PAGE_INDEX);                                                        // 参数范围 0-3
    uint8 return_state = 0;

    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32 flash_addr = 0;
    uint16 len = 0;
    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_num+FLASH_PAGE_SIZE*page_num));            // 提取当前 Flash 地址

    if(flash_check(sector_num, page_num))                                                               // 判断是否有数据 这里是冗余的保护 防止有人没擦除就写入
        flash_erase_sector(sector_num, page_num);                                                       // 擦除这一页

    uint32 primask = interrupt_global_disable();
    //clock_reset();                      // 复位时钟
    //clock_set_freq(SYSTEM_CLOCK_120M);          // 设置系统频率为120Mhz

    FLASH_Unlock();                                                                                     // 解锁 Flash
    while(len < FLASH_DATA_BUFFER_SIZE)                                                                 // 根据长度
    {
        gFlashStatus = FLASH_ProgramWord(flash_addr, flash_union_buffer[len].uint32_type);              // 按字 32bit 写入数据
        if(gFlashStatus != FLASH_COMPLETE)                                                              // 反复确认操作是否成功
        {
            return_state = 1;
            break;
        }

        len++;                                                                                          // 长度自增
        flash_addr += 4;                                                                                // 地址自增
    }
    FLASH_Lock();                                                                                       // 锁定 Flash

    //clock_reset();                      // 复位时钟
    //clock_set_freq(system_clock);       // 设置回原来的系统频率
    interrupt_global_enable(primask);

    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空数据缓冲区
// 参数说明     void
// 返回参数     void
// 使用示例     flash_buffer_clear();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void flash_buffer_clear (void)
{
    memset(flash_union_buffer, 0xFF, FLASH_PAGE_SIZE);
}

