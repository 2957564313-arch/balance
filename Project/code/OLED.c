#include "zf_common_headfile.h"
#include "OLED.h"
#include "oledfont.h" 
#include <stdio.h> 

// =========================================================
// 引脚定义
// =========================================================
#define OLED_SCL_PIN  IO_P25
#define OLED_SDA_PIN  IO_P23

// =========================================================
// I2C 底层驱动 (极速版)
// =========================================================
static void OLED_I2C_Delay(void)
{
    volatile uint8 i = 1; 
    while(i--);
}

void OLED_I2C_Init(void)
{
    gpio_init(OLED_SCL_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(OLED_SDA_PIN, GPO, 1, GPO_PUSH_PULL);
}

void OLED_I2C_SendByte(uint8 Byte)
{
    uint8 i;
    bit ea_save = EA;
    EA = 0; 
    
    for (i = 0; i < 8; i++)
    {
        gpio_set_level(OLED_SDA_PIN, (Byte & 0x80) ? 1 : 0);
        Byte <<= 1;
        OLED_I2C_Delay();
        gpio_set_level(OLED_SCL_PIN, 1);
        OLED_I2C_Delay();
        gpio_set_level(OLED_SCL_PIN, 0);
        OLED_I2C_Delay();
    }
    gpio_set_level(OLED_SCL_PIN, 1);
    OLED_I2C_Delay();
    gpio_set_level(OLED_SCL_PIN, 0);
    OLED_I2C_Delay();
    
    EA = ea_save; 
}

void OLED_I2C_Start(void)
{
    bit ea_save = EA;
    EA = 0;
    gpio_set_level(OLED_SDA_PIN, 1);
    gpio_set_level(OLED_SCL_PIN, 1);
    OLED_I2C_Delay();
    gpio_set_level(OLED_SDA_PIN, 0);
    OLED_I2C_Delay();
    gpio_set_level(OLED_SCL_PIN, 0);
    OLED_I2C_Delay();
    EA = ea_save;
}

void OLED_I2C_Stop(void)
{
    bit ea_save = EA;
    EA = 0;
    gpio_set_level(OLED_SDA_PIN, 0);
    gpio_set_level(OLED_SCL_PIN, 1);
    OLED_I2C_Delay();
    gpio_set_level(OLED_SDA_PIN, 1);
    OLED_I2C_Delay();
    EA = ea_save;
}

void OLED_WriteCommand(uint8 Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x00);
    OLED_I2C_SendByte(Command); 
    OLED_I2C_Stop();
}

void OLED_WriteData(uint8 Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40);
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

// =========================================================
// 显示逻辑
// =========================================================

void OLED_SetCursor(uint8 Y, uint8 X)
{
    OLED_WriteCommand(0xB0 | Y);                  
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); 
    OLED_WriteCommand(0x00 | (X & 0x0F));        
}

void OLED_Clear(void)
{  
    uint8 i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++) OLED_WriteData(0x00);
    }
}

void OLED_ShowChar(uint8 Line, uint8 Column, char Char)
{            
    uint8 i;
    if(Line > 4 || Column > 16) return;
    
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

void OLED_ShowString(uint8 Line, uint8 Column, char *String)
{
    uint8 i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

// 旧版浮点显示 (保留兼容)
void OLED_Show_Float(uint8 Line, uint8 Column, float dat, uint8 num, uint8 pointnum)
{
    char buff[32];
    uint8 len;
    
    if(pointnum == 0)      len = sprintf(buff, "%.0f", dat);
    else if(pointnum == 1) len = sprintf(buff, "%.1f", dat);
    else if(pointnum == 2) len = sprintf(buff, "%.2f", dat);
    else                   len = sprintf(buff, "%.3f", dat);
    
    while(len < num)
    {
        buff[len] = ' ';
        len++;
    }
    buff[len] = '\0';
    
    OLED_ShowString(Line, Column, buff);
}

// =========================================================
// 新增：极速显示函数实现
// =========================================================

// 1. 快速显示整数
void OLED_Show_Int_Fast(uint8 row, uint8 col, int32 num) {
    char buf[10];
    uint8 i = 0, j = 0;
    
    if(num == 0) { 
        OLED_ShowChar(row, col, '0'); 
        return; 
    }
    
    while(num > 0) { 
        buf[i++] = (num % 10) + '0'; 
        num /= 10; 
    }
    
    while(i > 0) { 
        OLED_ShowChar(row, col + j, buf[--i]); 
        j++; 
    }
}

// 2. 快速显示浮点 (解决 main.c 报错的核心)
// 变量定义全部前置，严格遵守 C89
void OLED_Show_Float_Safe(uint8 row, uint8 col, float val) {
    int32 val_int;
    int32 val_dec;
    
    if(val < 0) { 
        OLED_ShowChar(row, col, '-'); 
        val = -val; 
    } else { 
        OLED_ShowChar(row, col, ' '); 
    }
    
    val_int = (int32)val;
    val_dec = (int32)((val - val_int) * 100); 
    if(val_dec < 0) val_dec = 0;
    
    // 显示整数部分
    if(val_int < 10) { 
        OLED_ShowChar(row, col+1, ' '); 
        OLED_ShowChar(row, col+2, ' ');
        OLED_Show_Int_Fast(row, col+3, val_int); 
    } else if(val_int < 100) { 
        OLED_ShowChar(row, col+1, ' '); 
        OLED_Show_Int_Fast(row, col+2, val_int); 
    } else { 
        OLED_Show_Int_Fast(row, col+1, val_int); 
    }

    OLED_ShowChar(row, col+4, '.');
    
    // 显示小数部分 (2位)
    if(val_dec < 10) { 
        OLED_ShowChar(row, col+5, '0'); 
        OLED_Show_Int_Fast(row, col+6, val_dec); 
    } else { 
        OLED_Show_Int_Fast(row, col+5, val_dec); 
    }
}

void OLED_Init(void)
{
    system_delay_ms(100);
    OLED_I2C_Init();
    
    OLED_WriteCommand(0xAE); 
    
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x02); 
    OLED_WriteCommand(0xC8); 
    OLED_WriteCommand(0xA1); 
    
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14); 
    
    OLED_Clear(); 
    OLED_WriteCommand(0xAF); 
}
