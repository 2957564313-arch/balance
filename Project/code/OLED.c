#include "zf_common_headfile.h"
#include "oled.h"

// =========================================================
// 内部：I2C(推挽) + SSD1306
// =========================================================
static uint8 g_oled_addr7 = 0;                 // 探测到的7bit地址
static uint8 g_buf[OLED_W * (OLED_H / 8)];     // 128*8 = 1024字节显存缓冲

static void iic_delay(void)
{
    volatile uint16 i = OLED_IIC_DELAY;
    while(i--);
}

// SCL 推挽输出
static void SCL_OUT_PP(void)    { gpio_init(OLED_SCL_PIN, GPO, 1, GPO_PUSH_PULL); }
// SDA 推挽输出（发送时）
static void SDA_OUT_PP(void)    { gpio_init(OLED_SDA_PIN, GPO, 1, GPO_PUSH_PULL); }
// SDA 输入上拉（读ACK时）
static void SDA_IN_PULLUP(void) { gpio_init(OLED_SDA_PIN, GPI, 1, GPI_PULL_UP);  }

static void SCL_H(void) { gpio_high(OLED_SCL_PIN); }
static void SCL_L(void) { gpio_low (OLED_SCL_PIN); }
static void SDA_H(void) { gpio_high(OLED_SDA_PIN); }
static void SDA_L(void) { gpio_low (OLED_SDA_PIN); }
static uint8 SDA_READ(void) { return gpio_get_level(OLED_SDA_PIN); }

static void iic_start(void)
{
    SCL_OUT_PP();
    SDA_OUT_PP();
    SDA_H(); SCL_H(); iic_delay();
    SDA_L();          iic_delay();
    SCL_L();          iic_delay();
}

static void iic_stop(void)
{
    SCL_OUT_PP();
    SDA_OUT_PP();
    SDA_L(); SCL_L(); iic_delay();
    SCL_H();          iic_delay();
    SDA_H();          iic_delay();
}

// 返回0=ACK, 1=NACK
static uint8 iic_wait_ack(void)
{
    uint8 ack;
    SDA_IN_PULLUP();
    SDA_H();
    iic_delay();
    SCL_H(); iic_delay();

    ack = SDA_READ();

    SCL_L(); iic_delay();
    SDA_OUT_PP();
    return ack;
}

static void iic_send_byte(uint8 dat)
{
    uint8 i;
    SDA_OUT_PP();
    for(i=0;i<8;i++)
    {
        SCL_L();
        if(dat & 0x80) SDA_H(); else SDA_L();
        dat <<= 1;
        iic_delay();
        SCL_H();
        iic_delay();
    }
    SCL_L();
}

// 探测7bit地址是否存在
static uint8 iic_probe7(uint8 addr7)
{
    uint8 ok;
    iic_start();
    iic_send_byte((addr7 << 1) & 0xFE);
    ok = (iic_wait_ack() == 0);
    iic_stop();
    return ok;
}

// SSD1306 写命令
static void OLED_WriteCommand(uint8 cmd)
{
    iic_start();
    iic_send_byte((g_oled_addr7 << 1) & 0xFE);
    if(iic_wait_ack()) { iic_stop(); return; }

    iic_send_byte(0x00);                  // 控制字节：命令
    if(iic_wait_ack()) { iic_stop(); return; }

    iic_send_byte(cmd);
    iic_wait_ack();
    iic_stop();
}

// SSD1306：设置页+列
static void OLED_SetCursor(uint8 page, uint8 x)
{
    OLED_WriteCommand(0xB0 | page);
    OLED_WriteCommand(0x10 | ((x & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (x & 0x0F));
}

// 次方
static uint32 OLED_Pow(uint32 X, uint32 Y)
{
    uint32 Result = 1;
    while(Y--) Result *= X;
    return Result;
}

// =========================================================
// 缓冲区绘制（只改g_buf，不直接刷屏）
// =========================================================
static void buf_clear(void)
{
    uint16 i;
    for(i=0;i<sizeof(g_buf);i++) g_buf[i] = 0x00;
}

// 在缓冲区写一个8x16字符（Line:1~4, Column:1~16）
static void buf_draw_char_8x16(uint8 Line, uint8 Column, char ch)
{
    uint8 i;
    uint8 page;
    uint8 x;
    uint16 idx_top;
    uint16 idx_bot;
    uint8 c;

    if(Line < 1 || Line > 4) return;
    if(Column < 1 || Column > 16) return;

    if(ch < ' ' || ch > '~') ch = ' ';
    c = (uint8)(ch - ' ');

    page = (uint8)((Line - 1) * 2);       // 0,2,4,6
    x    = (uint8)((Column - 1) * 8);

    // top page
    idx_top = (uint16)page * OLED_W + x;
    // bottom page
    idx_bot = (uint16)(page + 1) * OLED_W + x;

    for(i=0;i<8;i++)
    {
        g_buf[idx_top + i] = OLED_F8x16[c][i];
        g_buf[idx_bot + i] = OLED_F8x16[c][i + 8];
    }
}

// =========================================================
// 对外API
// =========================================================
void OLED_Init(void)
{
    uint8 a;

    system_delay_ms(200);

    // 线拉到已知状态
    SCL_OUT_PP();
    SDA_OUT_PP();
    SCL_H();
    SDA_H();
    system_delay_ms(50);

    // 探测地址（优先0x3C/0x3D）
    g_oled_addr7 = 0;
    if(iic_probe7(0x3C)) g_oled_addr7 = 0x3C;
    else if(iic_probe7(0x3D)) g_oled_addr7 = 0x3D;
    else
    {
        for(a=0x08; a<=0x77; a++)
        {
            if(iic_probe7(a)) { g_oled_addr7 = a; break; }
        }
    }

    if(g_oled_addr7 == 0)
    {
        
        return;
    }

    // SSD1306 init（128x64 通用）
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);

    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14); // charge pump
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x00); // horizontal addressing

    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);

    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xAF);

    OLED_Clear();
    OLED_Update(); // 上电就刷一次，确保立刻生效
}

void OLED_Clear(void)
{
    buf_clear();
}

// 整屏刷新：一次性把1024字节刷到屏上
void OLED_Update(void)
{
    uint8 page;
    uint8 x;
    uint16 base;

    if(g_oled_addr7 == 0) return;

    for(page=0; page<8; page++)
    {
        OLED_SetCursor(page, 0);

        // 这一页用“单次I2C事务”连续写128字节，速度快、不会闪
        iic_start();
        iic_send_byte((g_oled_addr7 << 1) & 0xFE);
        if(iic_wait_ack()) { iic_stop(); return; }

        iic_send_byte(0x40);              // 数据模式
        if(iic_wait_ack()) { iic_stop(); return; }

        base = (uint16)page * OLED_W;
        for(x=0; x<OLED_W; x++)
        {
            iic_send_byte(g_buf[base + x]);
            iic_wait_ack();
        }
        iic_stop();
    }
}

void OLED_ShowChar(uint8 Line, uint8 Column, char Char)
{
    buf_draw_char_8x16(Line, Column, Char);
}

void OLED_ShowString(uint8 Line, uint8 Column, char *String)
{
    uint8 i = 0;
    if(String == 0) return;

    while(String[i] != '\0')
    {
        if(Column > 16) break;
        buf_draw_char_8x16(Line, Column, String[i]);
        Column++;
        i++;
    }
}

void OLED_ShowNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length)
{
    uint8 i;
    for(i=0;i<Length;i++)
    {
        buf_draw_char_8x16(Line, (uint8)(Column + i),
            (char)(Number / OLED_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

void OLED_ShowSignedNum(uint8 Line, uint8 Column, int32 Number, uint8 Length)
{
    uint8 i;
    uint32 n;

    if(Number >= 0)
    {
        buf_draw_char_8x16(Line, Column, '+');
        n = (uint32)Number;
    }
    else
    {
        buf_draw_char_8x16(Line, Column, '-');
        n = (uint32)(-Number);
    }

    for(i=0;i<Length;i++)
    {
        buf_draw_char_8x16(Line, (uint8)(Column + i + 1),
            (char)(n / OLED_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

void OLED_ShowHexNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length)
{
    uint8 i;
    uint8 s;

    for(i=0;i<Length;i++)
    {
        s = (uint8)(Number / OLED_Pow(16, Length - i - 1) % 16);
        if(s < 10) buf_draw_char_8x16(Line, (uint8)(Column + i), (char)(s + '0'));
        else       buf_draw_char_8x16(Line, (uint8)(Column + i), (char)(s - 10 + 'A'));
    }
}

void OLED_ShowBinNum(uint8 Line, uint8 Column, uint32 Number, uint8 Length)
{
    uint8 i;
    for(i=0;i<Length;i++)
    {
        buf_draw_char_8x16(Line, (uint8)(Column + i),
            (char)(Number / OLED_Pow(2, Length - i - 1) % 2 + '0'));
    }
}

void OLED_ShowFloat(uint8 Line, uint8 Column, float Number, uint8 IntLength, uint8 FracLength)
{
    uint32 pow10;
    int32 scaled;

    pow10 = OLED_Pow(10, FracLength);

    if(Number < 0)
    {
        buf_draw_char_8x16(Line, Column, '-');
        Number = -Number;
        Column++;
    }

    // 四舍五入到FracLength位
    scaled = (int32)(Number * (float)pow10 + 0.5f);

    OLED_ShowNum(Line, Column, (uint32)(scaled / (int32)pow10), IntLength);
    buf_draw_char_8x16(Line, (uint8)(Column + IntLength), '.');
    OLED_ShowNum(Line, (uint8)(Column + IntLength + 1), (uint32)(scaled % (int32)pow10), FracLength);
}
