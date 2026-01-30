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
    // 【核心提速点】
    // 之前是 10，现在改成 1。
    // STC32G 35MHz 下，这个速度接近 OLED 硬件极限，刷新率极高！
    // 如果万一又出现花屏，把这里改成 2 或 3 即可。
    volatile uint8 i = 1; 
    while(i--);
}

void OLED_I2C_Init(void)
{
    gpio_init(OLED_SCL_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(OLED_SDA_PIN, GPO, 1, GPO_PUSH_PULL);
}

// 发送一个字节 (带原子锁保护，防止被中断打断导致花屏)
void OLED_I2C_SendByte(uint8 Byte)
{
    uint8 i;
    
    // 关中断保护（哪怕速度快了，这个锁也不能去掉，否则必花屏）
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

// 智能浮点显示 (带补空格)
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

// 初始化函数
void OLED_Init(void)
{
    system_delay_ms(100);
    OLED_I2C_Init();
    
    OLED_WriteCommand(0xAE); // 关显示
    
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x02); // 页模式
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
    OLED_WriteCommand(0xAF); // 开显示
}
