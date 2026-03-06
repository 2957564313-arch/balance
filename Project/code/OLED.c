#include "zf_common_headfile.h"
#include "OLED.h"
#include "oledfont.h"
#include <stdio.h>

#define OLED_SCL_PIN  IO_P25                            // OLED SCL 引脚
#define OLED_SDA_PIN  IO_P23                            // OLED SDA 引脚


// ================================== 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        I2C 微秒级延时
// 参数说明        void
// 返回参数        void
// 使用示例        OLED_I2C_Delay();
// 备注信息        内部静态函数 极速版延时
//--------------------------------------------------------------------------------------------------
static void OLED_I2C_Delay(void)
{
    volatile uint8 i = 10;
    while(i--);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        I2C 引脚初始化
// 参数说明        void
// 返回参数        void
// 使用示例        OLED_I2C_Init();
// 备注信息        配置 SCL/SDA 为推挽输出 默认高电平
//--------------------------------------------------------------------------------------------------
void OLED_I2C_Init(void)
{
    gpio_init(OLED_SCL_PIN, GPO, 1, GPO_PUSH_PULL);     // SCL 推挽输出 默认高
    gpio_init(OLED_SDA_PIN, GPO, 1, GPO_PUSH_PULL);     // SDA 推挽输出 默认高
}

//--------------------------------------------------------------------------------------------------
// 函数简介        I2C 发送一个字节
// 参数说明        Byte             待发送的字节数据
// 返回参数        void
// 使用示例        OLED_I2C_SendByte(0x78);
// 备注信息        内部函数 关中断保护
//--------------------------------------------------------------------------------------------------
void OLED_I2C_SendByte(uint8 Byte)
{
    uint8 i;
    bit ea_save = EA;                                   // 保存中断状态
    EA = 0;                                             // 关中断保护时序

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
    gpio_set_level(OLED_SCL_PIN, 1);                    // ACK 时钟
    OLED_I2C_Delay();
    gpio_set_level(OLED_SCL_PIN, 0);
    OLED_I2C_Delay();

    EA = ea_save;                                       // 恢复中断状态
}

//--------------------------------------------------------------------------------------------------
// 函数简介        I2C 起始信号
// 参数说明        void
// 返回参数        void
// 使用示例        OLED_I2C_Start();
// 备注信息        内部函数 SCL高电平时SDA拉低
//--------------------------------------------------------------------------------------------------
void OLED_I2C_Start(void)
{
    bit ea_save = EA;
    EA = 0;
    gpio_set_level(OLED_SDA_PIN, 1);                    // SDA 拉高
    gpio_set_level(OLED_SCL_PIN, 1);                    // SCL 拉高
    OLED_I2C_Delay();
    gpio_set_level(OLED_SDA_PIN, 0);                    // SDA 拉低 -> 起始信号
    OLED_I2C_Delay();
    gpio_set_level(OLED_SCL_PIN, 0);                    // SCL 拉低
    OLED_I2C_Delay();
    EA = ea_save;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        I2C 停止信号
// 参数说明        void
// 返回参数        void
// 使用示例        OLED_I2C_Stop();
// 备注信息        内部函数 SCL高电平时SDA拉高
//--------------------------------------------------------------------------------------------------
void OLED_I2C_Stop(void)
{
    bit ea_save = EA;
    EA = 0;
    gpio_set_level(OLED_SDA_PIN, 0);                    // SDA 拉低
    gpio_set_level(OLED_SCL_PIN, 1);                    // SCL 拉高
    OLED_I2C_Delay();
    gpio_set_level(OLED_SDA_PIN, 1);                    // SDA 拉高 -> 停止信号
    OLED_I2C_Delay();
    EA = ea_save;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        写命令到 OLED
// 参数说明        Command          SSD1306 命令字节
// 返回参数        void
// 使用示例        OLED_WriteCommand(0xAE);
// 备注信息        内部函数 地址0x78 控制字艂0x00
//--------------------------------------------------------------------------------------------------
void OLED_WriteCommand(uint8 Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);                            // 从机地址
    OLED_I2C_SendByte(0x00);                            // 控制字节：命令
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        写数据到 OLED
// 参数说明        Data             显示数据字节
// 返回参数        void
// 使用示例        OLED_WriteData(0xFF);
// 备注信息        内部函数 地址0x78 控制字艂0x40
//--------------------------------------------------------------------------------------------------
void OLED_WriteData(uint8 Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);                            // 从机地址
    OLED_I2C_SendByte(0x40);                            // 控制字节：数据
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        设置 OLED 光标位置
// 参数说明        Y                页地址 (0~7)
// 参数说明        X                列地址 (0~127)
// 返回参数        void
// 使用示例        OLED_SetCursor(0, 0);
// 备注信息        内部函数
//--------------------------------------------------------------------------------------------------
void OLED_SetCursor(uint8 Y, uint8 X)
{
    OLED_WriteCommand(0xB0 | Y);                        // 设置页地址
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));        // 设置列地址高 4 位
    OLED_WriteCommand(0x00 | (X & 0x0F));               // 设置列地址低 4 位
}


// ==================================== 对外 API ====================================

//--------------------------------------------------------------------------------------------------
// 函数简介        OLED 初始化
// 参数说明        void
// 返回参数        void
// 使用示例        OLED_Init();
// 备注信息        SSD1306 初始化序列 包含 I2C 初始化、配置寄存器、清屏、开显示
//--------------------------------------------------------------------------------------------------
void OLED_Init(void)
{
    system_delay_ms(100);                               // 等待 OLED 上电稳定
    OLED_I2C_Init();                                    // 初始化 I2C 引脚

    OLED_WriteCommand(0xAE);                            // 关闭显示

    OLED_WriteCommand(0x20); OLED_WriteCommand(0x02);   // 页地址模式
    OLED_WriteCommand(0xC8);                            // COM 扫描方向
    OLED_WriteCommand(0xA1);                            // 段重映射

    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);   // 显示时钟分频
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);   // 复用率 64
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);   // 显示偏移
    OLED_WriteCommand(0x40);                            // 起始行
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);   // COM 引脚配置
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);   // 对比度
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);   // 预充电周期
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);   // VCOMH 电压
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);   // 开启电荷泵

    OLED_Clear();                                       // 清屏
    OLED_WriteCommand(0xAF);                            // 开启显示
}

//--------------------------------------------------------------------------------------------------
// 函数简介        清屏
// 参数说明        void
// 返回参数        void
// 使用示例        OLED_Clear();
// 备注信息        全屏填充 0x00 熄灭所有像素
//--------------------------------------------------------------------------------------------------
void OLED_Clear(void)
{
    uint8 i, j;
    for (j = 0; j < 8; j++)                             // 遍历 8 页
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++) OLED_WriteData(0x00);  // 每页 128 列填 0
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        显示单个字符
// 参数说明        Line             行号 (1~4)
// 参数说明        Column           列号 (1~16)
// 参数说明        Char             待显示的 ASCII 字符
// 返回参数        void
// 使用示例        OLED_ShowChar(1, 1, 'A');
// 备注信息        8x16 字体 占两页高度
//--------------------------------------------------------------------------------------------------
void OLED_ShowChar(uint8 Line, uint8 Column, char Char)
{
    uint8 i;
    if(Line > 4 || Column > 16) return;                 // 越界保护

    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);   // 上半部分
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i]);

    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8); // 下半部分
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        显示字符串
// 参数说明        Line             行号 (1~4)
// 参数说明        Column           起始列号 (1~16)
// 参数说明        *String          待显示的字符串指针
// 返回参数        void
// 使用示例        OLED_ShowString(1, 1, "Hello");
// 备注信息
//--------------------------------------------------------------------------------------------------
void OLED_ShowString(uint8 Line, uint8 Column, char *String)
{
    uint8 i;
    for (i = 0; String[i] != '\0'; i++)                 // 遍历字符串至结束符
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        显示整数
// 参数说明        row              行号 (1~4)
// 参数说明        col              起始列号 (1~16)
// 参数说明        num              待显示的整数
// 返回参数        void
// 使用示例        OLED_ShowNum(1, 1, 12345);
// 备注信息        仅支持非负整数
//--------------------------------------------------------------------------------------------------
void OLED_ShowNum(uint8 row, uint8 col, int32 num)
{
    char buf[10];                                       // 数字缓冲区
    uint8 i = 0, j = 0;

    if(num == 0) {                                      // 特殊处理: 0
        OLED_ShowChar(row, col, '0');
        return;
    }

    while(num > 0) {                                    // 逆序提取各位数字
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }

    while(i > 0) {                                      // 正序输出
        OLED_ShowChar(row, col + j, buf[--i]);
        j++;
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        显示浮点数
// 参数说明        row              行号 (1~4)
// 参数说明        col              起始列号 (1~16)
// 参数说明        val              待显示的浮点数
// 返回参数        void
// 使用示例        OLED_ShowFloat(1, 1, 3.14);
// 备注信息        显示 2 位小数 支持负数 整数部分最多 3 位
//--------------------------------------------------------------------------------------------------
void OLED_ShowFloat(uint8 row, uint8 col, float val)
{
    int32 val_int;                                      // 整数部分
    int32 val_dec;                                      // 小数部分

    if(val < 0) {                                       // 负数处理
        OLED_ShowChar(row, col, '-');
        val = -val;
    } else {
        OLED_ShowChar(row, col, ' ');
    }

    val_int = (int32)val;
    val_dec = (int32)((val - val_int) * 100);
    if(val_dec < 0) val_dec = 0;

    // 显示整数部分（右对齐 最多 3 位）
    if(val_int < 10) {
        OLED_ShowChar(row, col+1, ' ');
        OLED_ShowChar(row, col+2, ' ');
        OLED_ShowNum(row, col+3, val_int);
    } else if(val_int < 100) {
        OLED_ShowChar(row, col+1, ' ');
        OLED_ShowNum(row, col+2, val_int);
    } else {
        OLED_ShowNum(row, col+1, val_int);
    }

    OLED_ShowChar(row, col+4, '.');                     // 小数点

    // 显示小数部分（2 位 不足前补 0）
    if(val_dec < 10) {
        OLED_ShowChar(row, col+5, '0');
        OLED_ShowNum(row, col+6, val_dec);
    } else {
        OLED_ShowNum(row, col+5, val_dec);
    }
}
