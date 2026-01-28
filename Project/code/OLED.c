#include "OLED.h"

// =========================================================
// 内部：软件模拟 I2C (推挽模式)
// =========================================================
static uint8 g_oled_addr7 = 0x3C; // OLED默认地址
static uint8 xdata g_buf[OLED_W * (OLED_H / 8)]; // 显存缓冲 (1024字节)

// 延时函数：适配 STC32G @ 30MHz
static void iic_delay(void)
{
    volatile uint8 i = OLED_IIC_DELAY;
    while(i--);
}

// I2C 初始化：配置为推挽输出
static void IIC_Init_Pin(void)
{
    gpio_init(OLED_SCL_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(OLED_SDA_PIN, GPO, 1, GPO_PUSH_PULL);
}

// SCL/SDA 电平操作宏
#define SCL_H()     gpio_set_level(OLED_SCL_PIN, 1)
#define SCL_L()     gpio_set_level(OLED_SCL_PIN, 0)
#define SDA_H()     gpio_set_level(OLED_SDA_PIN, 1)
#define SDA_L()     gpio_set_level(OLED_SDA_PIN, 0)
#define SDA_READ()  gpio_get_level(OLED_SDA_PIN)

static void iic_start(void)
{
    SDA_H();
    SCL_H();
    iic_delay();
    SDA_L();
    iic_delay();
    SCL_L();
}

static void iic_stop(void)
{
    SCL_L();
    SDA_L();
    iic_delay();
    SCL_H();
    iic_delay();
    SDA_H();
    iic_delay();
}

static uint8 iic_wait_ack(void)
{
    uint8 ack;
    // 注意：推挽模式下读取ACK其实有风险(短路)，但OLED通常不回ACK或回得很弱
    // 释放SDA(拉高)
    SDA_H(); 
    iic_delay();
    SCL_H();
    iic_delay();
    ack = SDA_READ();
    SCL_L();
    iic_delay();
    return ack;
}

static void iic_send_byte(uint8 dat)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        SCL_L();
        if(dat & 0x80) SDA_H();
        else           SDA_L();
        dat <<= 1;
        iic_delay();
        SCL_H();
        iic_delay();
    }
    SCL_L();
}

static void OLED_WriteCommand(uint8 cmd)
{
    iic_start();
    iic_send_byte(g_oled_addr7 << 1);
    if(iic_wait_ack()) { iic_stop(); return; }
    
    iic_send_byte(0x00); // Command Mode
    if(iic_wait_ack()) { iic_stop(); return; }
    
    iic_send_byte(cmd);
    iic_wait_ack();
    iic_stop();
}

// OLED_WriteData 已被删除，因为 OLED_Update 中直接操作 I2C，避免了 Unreferenced Warning

static void OLED_SetCursor(uint8 page, uint8 x)
{
    OLED_WriteCommand(0xB0 | page);              // 设置页地址
    OLED_WriteCommand(0x10 | ((x & 0xF0) >> 4)); // 设置列地址高4位
    OLED_WriteCommand(0x00 | (x & 0x0F));        // 设置列地址低4位
}

static uint32 OLED_Pow(uint32 X, uint32 Y)
{
    uint32 Result = 1;
    while(Y--) Result *= X;
    return Result;
}

// =========================================================
// API 实现
// =========================================================

void OLED_Init(void)
{
    system_delay_ms(200);
    IIC_Init_Pin();
    
    // 初始化序列
    OLED_WriteCommand(0xAE); // Display Off
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14); // Charge Pump
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x00); // Memory Mode
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xAF); // Display On
    
    OLED_Clear();
    OLED_Update();
}

void OLED_Clear(void)
{
    uint16 i;
    for(i=0; i<sizeof(g_buf); i++) g_buf[i] = 0;
}

void OLED_Update(void)
{
    uint8 page;
    uint8 i;
    uint16 buf_idx = 0;
    
    for(page=0; page<8; page++)
    {
        OLED_SetCursor(page, 0);
        
        iic_start();
        iic_send_byte(g_oled_addr7 << 1);
        if(iic_wait_ack()) { iic_stop(); return; }
        
        iic_send_byte(0x40); // 随后发送全是数据
        if(iic_wait_ack()) { iic_stop(); return; }
        
        for(i=0; i<128; i++)
        {
            iic_send_byte(g_buf[buf_idx++]);
            iic_wait_ack();
        }
        iic_stop();
    }
}

// 辅助函数：绘制字符到缓冲区
static void buf_draw_char(uint8 Line, uint8 Column, char c)
{
    uint8 i;
    uint16 top, bot;
    
    c = c - ' ';
    top = (Line-1)*2 * 128 + (Column-1)*8;
    bot = (Line-1)*2 * 128 + 128 + (Column-1)*8;
    
    for(i=0; i<8; i++)
    {
        g_buf[top+i] = OLED_F8x16[c][i];
        g_buf[bot+i] = OLED_F8x16[c][i+8];
    }
}

void OLED_ShowChar(uint8 Line, uint8 Column, char Char)
{
    if(Line > 4 || Column > 16) return;
    buf_draw_char(Line, Column, Char);
}

void OLED_ShowString(uint8 Line, uint8 Column, char *String)
{
    uint8 i = 0;
    while(String[i] != '\0')
    {
        OLED_ShowChar(Line, Column, String[i]);
        Column++;
        if(Column > 16) { Column=1; Line++; }
        if(Line > 4) break;
        i++;
    }
}

void OLED_ShowNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length)
{
    uint8 i;
    for(i=0; i<Length; i++)
    {
        OLED_ShowChar(Line, Column+i, Number / OLED_Pow(10, Length-i-1) % 10 + '0');
    }
}

void OLED_ShowSignedNum(uint8 Line, uint8 Column, int32 Number, uint8 Length)
{
    uint32 n;
    if(Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        n = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        n = -Number;
    }
    OLED_ShowNum(Line, Column+1, n, Length);
}

// [新增] 十六进制显示
void OLED_ShowHexNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length)
{
    uint8 i;
    uint8 single_num;
    for(i=0; i<Length; i++)
    {
        single_num = (Number / OLED_Pow(16, Length-i-1)) % 16;
        if(single_num < 10)
            OLED_ShowChar(Line, Column+i, single_num + '0');
        else
            OLED_ShowChar(Line, Column+i, single_num - 10 + 'A');
    }
}

// [新增] 二进制显示
void OLED_ShowBinNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length)
{
    uint8 i;
    for(i=0; i<Length; i++)
    {
        OLED_ShowChar(Line, Column+i, (Number / OLED_Pow(2, Length-i-1)) % 2 + '0');
    }
}

void OLED_ShowFloat(uint8 Line, uint8 Column, float Number, uint8 IntLength, uint8 FracLength)
{
    uint32 pow10 = OLED_Pow(10, FracLength);
    uint32 int_part;
    uint32 frac_part;
    
    if(Number < 0)
    {
        OLED_ShowChar(Line, Column, '-');
        Number = -Number;
        Column++;
    }
    
    Number += 0.5f / pow10; // 四舍五入
    int_part = (uint32)Number;
    frac_part = (uint32)((Number - int_part) * pow10);
    
    OLED_ShowNum(Line, Column, int_part, IntLength);
    OLED_ShowChar(Line, Column + IntLength, '.');
    OLED_ShowNum(Line, Column + IntLength + 1, frac_part, FracLength);
}
