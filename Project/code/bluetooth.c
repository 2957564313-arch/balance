#include "zf_common_headfile.h"
#include "bluetooth.h"
#include <string.h>

// ========================= 对外导出变量 =========================
int16 remote_speed = 0;                 // LV：前后
int16 remote_turn  = 0;                 // LH：左右
volatile uint8  last_byte = 0;
volatile uint32 rx_count  = 0;

// ========================= 配置 =========================
#define BT_ECHO_ENABLE       (1)     // 1=回显（联调），0=关闭
#define BT_PARSE_MAX_BYTES   (96)    // 每次最多处理字节数
#define BT_MAX_FRAME_LEN     (96)    // 一帧最大长度（不含[]）
#define BT_JOY_DEADZONE      (0)     // 死区

// ========================= 内部解析状态 =========================
static uint8  xdata bt_frame_buf[BT_MAX_FRAME_LEN];
static uint8  bt_frame_len = 0;
static uint8  bt_in_frame  = 0;

// ========================= 工具 =========================
static int16 bt_abs_i16(int16 v)
{
    return (v < 0) ? (int16)(-v) : v;
}

static int16 bt_read_int(const uint8 *s, uint8 *idx, uint8 *ok)
{
    int16 sign = 1;
    int16 val  = 0;
    uint8 i = *idx;

    *ok = 0;

    while(s[i] != '\0')
    {
        if(s[i] == '-') break;
        if(s[i] >= '0' && s[i] <= '9') break;
        i++;
    }
    if(s[i] == '\0') { *idx = i; return 0; }

    if(s[i] == '-') { sign = -1; i++; }
    if(!(s[i] >= '0' && s[i] <= '9')) { *idx = i; return 0; }

    *ok = 1;
    while(s[i] >= '0' && s[i] <= '9')
    {
        val = (int16)(val * 10 + (s[i] - '0'));
        i++;
    }

    *idx = i;
    return (int16)(val * sign);
}

static void bt_parse_frame(uint8 *frame)
{
    uint8 i = 0;
    uint8 ok;
    int16 lh, lv, rh, rv;

    while(frame[i] == ' ' || frame[i] == '\t') i++;
    if(frame[i] == '\0') return;

    // [joystick,lh,lv,rh,rv] / [j,lh,lv,rh,rv]
    if(frame[i] != 'j' && frame[i] != 'J') return;

    lh = bt_read_int(frame, &i, &ok); if(!ok) return;
    lv = bt_read_int(frame, &i, &ok); if(!ok) return;
    rh = bt_read_int(frame, &i, &ok); if(!ok) return;
    rv = bt_read_int(frame, &i, &ok); if(!ok) return;

    (void)rh;
    (void)rv;

    remote_turn  = lh;
    remote_speed = lv;

    if(BT_JOY_DEADZONE > 0)
    {
        if(bt_abs_i16(remote_turn)  < BT_JOY_DEADZONE) remote_turn = 0;
        if(bt_abs_i16(remote_speed) < BT_JOY_DEADZONE) remote_speed = 0;
    }
}

static void bt_feed_byte(uint8 ch)
{
    if(!bt_in_frame)
    {
        if(ch == '[')
        {
            bt_in_frame = 1;
            bt_frame_len = 0;
        }
        return;
    }

    if(ch == ']')
    {
        if(bt_frame_len >= BT_MAX_FRAME_LEN) bt_frame_len = (BT_MAX_FRAME_LEN - 1);
        bt_frame_buf[bt_frame_len] = '\0';
        bt_parse_frame(bt_frame_buf);

        bt_in_frame = 0;
        bt_frame_len = 0;
        return;
    }

    if(bt_frame_len < (BT_MAX_FRAME_LEN - 1))
    {
        bt_frame_buf[bt_frame_len++] = ch;
    }
    else
    {
        bt_in_frame = 0;
        bt_frame_len = 0;
    }
}

// ========================= UART4：原始初始化=========================
static void bt_uart4_set_map_p0(void)
{
    uint8 ea_save = EA;
    uint8 psw2_save = P_SW2;

    EA = 0;

    // 开 EAXFR
    P_SW2 = psw2_save | 0x80;

    // UART4 -> P0.2/P0.3（清 bit2）
    P_SW2 &= (uint8)(~0x04);

    // P0.2 RX：高阻输入(10)，P0.3 TX：推挽输出(01)
    P0M1 = (P0M1 & (uint8)(~0x0C)) | 0x04;
    P0M0 = (P0M0 & (uint8)(~0x0C)) | 0x08;

    // 关 EAXFR（保留其它位，只强制 bit7=0，bit2=0）
    P_SW2 = (uint8)((psw2_save & (uint8)(~0x80)) & (uint8)(~0x04));

    EA = ea_save;
}

static void bt_uart4_set_baud(uint32 baud)
{
    uint16 brt;

    // brt = 65536 - system_clock/(baud+2)/4
    brt = (uint16)(65536 - (system_clock / (baud + 2) / 4));

    // REN4=1
    S4CON |= 0x10;

    // Timer2 装载
    T2L = (uint8)brt;
    T2H = (uint8)(brt >> 8);

    // Timer2 1T + run
    AUXR |= 0x14;

    // 清 RI4/TI4
    S4CON &= (uint8)(~0x03);
}

static void bt_uart4_raw_init(uint32 baud)
{
    bt_uart4_set_map_p0();
    bt_uart4_set_baud(baud);
}

static uint8 bt_uart4_read_byte(uint8 *dat)
{
    if(S4CON & 0x01)               // RI4
    {
        S4CON &= (uint8)(~0x01);   // 清 RI4
        *dat = S4BUF;
        return 1;
    }
    return 0;
}

// 回显发送加超时，防止 TI4 异常卡死
static void bt_uart4_write_byte_timeout(uint8 dat)
{
    uint16 t;

    S4BUF = dat;

    t = 60000;
    while(!(S4CON & 0x02))
    {
        if(--t == 0) break;
    }
    S4CON &= (uint8)(~0x02);
}

// ========================= API =========================
void BT_Init(void)
{
    bt_in_frame = 0;
    bt_frame_len = 0;

    remote_speed = 0;
    remote_turn  = 0;
    last_byte    = 0;
    rx_count     = 0;

    bt_uart4_raw_init((uint32)BT_BAUD_RATE);
}

void BT_Parse_Task(void)
{
    uint8 dat;
    uint16 n = 0;

    while(n < BT_PARSE_MAX_BYTES)
    {
        if(!bt_uart4_read_byte(&dat)) break;

        last_byte = dat;
        rx_count++;

#if BT_ECHO_ENABLE
        bt_uart4_write_byte_timeout(dat);
#endif
        bt_feed_byte(dat);
        n++;
    }
}
