#include "zf_common_headfile.h"

// ================================== 对外变量定义 ==================================
int16 remote_speed = 0;                               // 遥控速度目标值（内部量纲）
int16 remote_turn  = 0;                               // 遥控转向目标值（内部量纲）
volatile uint32 bt_last_joy_ms = 0u;                  // 最后一次收到 joystick 帧的时间戳（ms）

// ================================== 外部变量声明 ==================================
// 由 main.c 提供的 1ms 系统时基
extern volatile uint32 g_ms_tick;

// ================================== 内部宏定义 ==================================
// joystick 映射：-100..100 -> 目标量级
#define BT_JOY_LIM                (100)
#define BT_JOY_DEADZONE           (3)

// 速度目标：单位 counts/20ms
// 表示速度环目标对应的编码器 20ms 增量
#define BT_SPEED_MAX_CNT20        (35)

// 转向目标：对齐 mode2/3 的 turn_cmd 量级（Motor_Turn_Tick_20ms 内部会 *10）
#define BT_TURN_MAX_CMD           (200)

// 串口配置：UART4 P0.2/P0.3
#define BT_UART_INDEX        (UART_4)
#define BT_UART_TX_PIN       (UART4_TX_P03)
#define BT_UART_RX_PIN       (UART4_RX_P02)

// 所有 UART 共用 Timer2，需保持相同波特率
#define BT_UART_BAUDRATE     (DEBUG_UART_BAUDRATE)

// 每次任务最多处理字节数
#define BT_PARSE_MAX_BYTES   (220u)
// 帧最大长度（不含[]）
#define BT_MAX_FRAME_LEN     (32u)

// RX 环形缓冲区大小
#define BT_RX_BUF_SIZE       (256u)

// ================================== 内部状态变量 ==================================
static uint8  s_rx_buf[BT_RX_BUF_SIZE];                // RX 环形缓冲区数据区
static volatile uint16 s_rx_head = 0u;                // 环形缓冲区写指针
static volatile uint16 s_rx_tail = 0u;                // 环形缓冲区读指针

static uint8 s_in_frame = 0u;                         // 当前是否处于帧接收状态
static uint8 s_len      = 0u;                         // 当前帧长度
static char  s_buf[BT_MAX_FRAME_LEN + 1];              // 帧内容缓存（不含 []）

// ================================== 内部函数声明 ==================================
static void  bt_rx_push(uint8 dat);
static uint8 bt_rx_pop(uint8 *dat);
static void  bt_uart4_rx_isr(uint8 dat);
static int32 bt_parse_int32(const char *s);
static float bt_parse_float(const char *s);
static uint8 bt_split(char *str, char *argv[], uint8 max_args);
static void  bt_apply_slider(uint8 m, float n);
static void  bt_parse_frame(char *frame);

// ================================== 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        RX 环形缓冲区写入一个字节（ISR 调用）
// 参数说明        dat              接收到的字节数据
// 返回参数        void
// 使用示例        bt_rx_push(dat);
// 备注信息        内部静态函数，若缓冲区已满则丢弃当前字节
//--------------------------------------------------------------------------------------------------
static void bt_rx_push(uint8 dat)
{
    uint16 next;

    next = (uint16)(s_rx_head + 1u);
    if (next >= (uint16)BT_RX_BUF_SIZE)
    {
        next = 0u;
    }

    if (next != s_rx_tail)
    {
        s_rx_buf[s_rx_head] = dat;
        s_rx_head = next;
    }
    else
    {
        // 缓冲区已满，丢弃当前字节
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        从 RX 环形缓冲区弹出一个字节
// 参数说明        dat              输出参数 弹出的字节
// 返回参数        uint8            1=成功弹出 0=缓冲区为空
// 使用示例        if (bt_rx_pop(&dat)) { ... }
// 备注信息        内部静态函数 主循环调用
//--------------------------------------------------------------------------------------------------
static uint8 bt_rx_pop(uint8 *dat)
{
    if (s_rx_head == s_rx_tail)
    {
        return 0u;
    }

    *dat = s_rx_buf[s_rx_tail];

    s_rx_tail++;
    if (s_rx_tail >= (uint16)BT_RX_BUF_SIZE)
    {
        s_rx_tail = 0u;
    }

    return 1u;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        UART4 接收中断回调（单字节）
// 参数说明        dat              本次接收到的字节
// 返回参数        void
// 使用示例        uart4_irq_handler = bt_uart4_rx_isr;
// 备注信息        内部静态函数 由 user/isr.c 中 UART4 IRQ 间接调用
//--------------------------------------------------------------------------------------------------
static void bt_uart4_rx_isr(uint8 dat)
{
    bt_rx_push(dat);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        解析十进制整数字符串为 int32
// 参数说明        s                输入字符串 支持前导负号
// 返回参数        int32            解析得到的整数值
// 使用示例        int32 v = bt_parse_int32(str);
// 备注信息        内部静态函数，遇到非数字字符即停止解析
//--------------------------------------------------------------------------------------------------
static int32 bt_parse_int32(const char *s)
{
    int32 sign;
    int32 v;

    sign = 1;
    v = 0;

    if ((0 != s) && ('-' == *s))
    {
        sign = -1;
        s++;
    }

    while ((0 != s) && (*s >= '0') && (*s <= '9'))
    {
        v = v * 10 + (int32)(*s - '0');
        s++;
    }

    return (sign * v);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        解析十进制浮点数字符串为 float
// 参数说明        s                输入字符串 支持前导负号与小数点
// 返回参数        float            解析得到的浮点值
// 使用示例        float n = bt_parse_float(str);
// 备注信息        内部静态函数，小数部分最多解析 6 位
//--------------------------------------------------------------------------------------------------
static float bt_parse_float(const char *s)
{
    int32 sign;
    uint32 ip;
    uint32 fp;
    uint32 div;

    sign = 1;
    ip = 0u;
    fp = 0u;
    div = 1u;

    if ((0 != s) && ('-' == *s))
    {
        sign = -1;
        s++;
    }

    while ((0 != s) && (*s >= '0') && (*s <= '9'))
    {
        ip = ip * 10u + (uint32)(*s - '0');
        s++;
    }

    if ((0 != s) && ('.' == *s))
    {
        s++;
        while ((0 != s) && (*s >= '0') && (*s <= '9') && (div < 1000000ul))
        {
            fp = fp * 10u + (uint32)(*s - '0');
            div *= 10u;
            s++;
        }
    }

    return (float)sign * ((float)ip + ((float)fp / (float)div));
}

//--------------------------------------------------------------------------------------------------
// 函数简介        按逗号分割字符串
// 参数说明        str              待分割字符串
// 参数说明        argv             输出参数 子串指针数组
// 参数说明        max_args         最多分割参数个数
// 返回参数        uint8            实际分割得到的参数数量
// 使用示例        argc = bt_split(frame, argv, 8u);
// 备注信息        内部静态函数，原地改写分隔符为 '\0'
//--------------------------------------------------------------------------------------------------
static uint8 bt_split(char *str, char *argv[], uint8 max_args)
{
    uint8 argc;
    char *p;

    argc = 0u;
    p = str;

    while ((0 != *p) && (argc < max_args))
    {
        argv[argc++] = p;

        while ((0 != *p) && (',' != *p))
        {
            p++;
        }

        if (',' == *p)
        {
            *p = '\0';
            p++;
        }
        else
        {
            break;
        }
    }

    return argc;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        将 slider 参数应用到系统变量
// 参数说明        m                slider 参数编号
// 参数说明        n                slider 参数值
// 返回参数        void
// 使用示例        bt_apply_slider(m, n);
// 备注信息        内部静态函数，当前支持 1~10 号参数映射
//--------------------------------------------------------------------------------------------------
static void bt_apply_slider(uint8 m, float n)
{
    if ((m < 1u) || (m > 10u))
    {
        return;
    }

    switch (m)
    {
        case 1u:
            g_sys_param.rate_kp = n;
            balance_cascade.angular_speed_cycle.p = n;
            balance_cascade_resave.angular_speed_cycle.p = n;
            break;
        case 2u:
            g_sys_param.rate_ki = n;
            balance_cascade.angular_speed_cycle.i = n;
            balance_cascade_resave.angular_speed_cycle.i = n;
            break;
        case 3u:
            g_sys_param.rate_kd = n;
            balance_cascade.angular_speed_cycle.d = n;
            balance_cascade_resave.angular_speed_cycle.d = n;
            break;

        case 4u:
            g_sys_param.angle_kp = n;
            balance_cascade.angle_cycle.p = n;
            balance_cascade_resave.angle_cycle.p = n;
            break;
        case 5u:
            g_sys_param.angle_ki = n;
            balance_cascade.angle_cycle.i = n;
            balance_cascade_resave.angle_cycle.i = n;
            break;
        case 6u:
            g_sys_param.angle_kd = n;
            balance_cascade.angle_cycle.d = n;
            balance_cascade_resave.angle_cycle.d = n;
            break;

        case 7u:
            g_sys_param.speed_kp = n;
            balance_cascade.speed_cycle.p = n;
            balance_cascade_resave.speed_cycle.p = n;
            break;
        case 8u:
            g_sys_param.speed_ki = n;
            balance_cascade.speed_cycle.i = n;
            balance_cascade_resave.speed_cycle.i = n;
            break;
        case 9u:
            g_sys_param.speed_kd = n;
            balance_cascade.speed_cycle.d = n;
            balance_cascade_resave.speed_cycle.d = n;
            break;

        case 10u:
            g_sys_param.mech_zero_pitch = n;
            balance_cascade.cascade_value.mechanical_zero = n;
            balance_cascade_resave.cascade_value.mechanical_zero = n;
            break;

        default:
            break;
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        解析单帧蓝牙文本命令
// 参数说明        frame            不含中括号的帧字符串
// 返回参数        void
// 使用示例        bt_parse_frame(s_buf);
// 备注信息        内部静态函数，当前支持 slider 与 joystick 指令（小写、无空格）
//--------------------------------------------------------------------------------------------------
static void bt_parse_frame(char *frame)
{
    char *argv[8];
    uint8 argc;
    char *cmd;
    uint8 m;
    float n;

    int32 ly;
    int32 rx;
    int32 v;
    int16 spd;
    int16 trn;

    if ((0 == frame) || (0 == *frame))
    {
        return;
    }

    argc = bt_split(frame, argv, 8u);
    if (argc < 1u)
    {
        return;
    }

    cmd = argv[0];

    if ((0 == strcmp(cmd, "slider")) || (0 == strcmp(cmd, "s")))
    {
        if (argc < 3u)
        {
            return;
        }

        m = (uint8)bt_parse_int32(argv[1]);
        n = bt_parse_float(argv[2]);

        bt_apply_slider(m, n);
        return;
    }

    // 支持 4 轴摇杆格式，仅使用 ly/rx 两个通道
    // [joystick,lx,ly,rx,ry]
    // 映射关系：speed = ly（左杆前后），turn = rx（右杆左右）
    if ((0 == strcmp(cmd, "joystick")) || (0 == strcmp(cmd, "j")))
    {
        if (argc != 5u)
        {
            return;
        }

        ly = bt_parse_int32(argv[2]);
        rx = bt_parse_int32(argv[3]);

        v = ly;

        if (v > (int32)BT_JOY_LIM) v = (int32)BT_JOY_LIM;
        if (v < (int32)(-BT_JOY_LIM)) v = (int32)(-BT_JOY_LIM);
        if ((v <= (int32)BT_JOY_DEADZONE) && (v >= (int32)(-BT_JOY_DEADZONE))) v = 0;
        spd = (int16)((v * (int32)BT_SPEED_MAX_CNT20) / (int32)BT_JOY_LIM);

        v = rx;
        if (v > (int32)BT_JOY_LIM) v = (int32)BT_JOY_LIM;
        if (v < (int32)(-BT_JOY_LIM)) v = (int32)(-BT_JOY_LIM);
        if ((v <= (int32)BT_JOY_DEADZONE) && (v >= (int32)(-BT_JOY_DEADZONE))) v = 0;
        trn = (int16)((v * (int32)BT_TURN_MAX_CMD) / (int32)BT_JOY_LIM);

        remote_speed = spd;
        remote_turn  = trn;

        bt_last_joy_ms = g_ms_tick;
        return;
    }
}

// ================================== 对外 API 函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        蓝牙模块初始化
// 参数说明        void
// 返回参数        void
// 使用示例        bluetooth_init();
// 备注信息        初始化 UART4，注册中断回调并清空解析状态
//--------------------------------------------------------------------------------------------------
void bluetooth_init(void)
{
    s_in_frame = 0u;
    s_len = 0u;
    s_buf[0] = '\0';

    s_rx_head = 0u;
    s_rx_tail = 0u;

    remote_speed = 0;
    remote_turn  = 0;
    bt_last_joy_ms = 0u;

    uart_init(BT_UART_INDEX, BT_UART_BAUDRATE, BT_UART_TX_PIN, BT_UART_RX_PIN);

    uart4_irq_handler = bt_uart4_rx_isr;

    uart_rx_interrupt(BT_UART_INDEX, 1u);

    // 启动第一次 DMA 单字节接收
    uart_rx_start_buff(BT_UART_INDEX);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        蓝牙解析任务（主循环周期调用）
// 参数说明        void
// 返回参数        void
// 使用示例        bluetooth_parse_task();
// 备注信息        从 RX 队列取字节，并按 [ ... ] 帧格式解析指令
//--------------------------------------------------------------------------------------------------
void bluetooth_parse_task(void)
{
    uint16 loop;
    uint8 dat;

    loop = 0u;
    while (loop < (uint16)BT_PARSE_MAX_BYTES)
    {
        if (!bt_rx_pop(&dat))
        {
            break;
        }

        if (dat == (uint8)'[')
        {
            s_in_frame = 1u;
            s_len = 0u;
            loop++;
            continue;
        }

        if (0u == s_in_frame)
        {
            loop++;
            continue;
        }

        if (dat == (uint8)']')
        {
            s_buf[s_len] = '\0';
            bt_parse_frame(s_buf);

            s_in_frame = 0u;
            s_len = 0u;
        }
        else
        {
            if (s_len < (uint8)BT_MAX_FRAME_LEN)
            {
                s_buf[s_len++] = (char)dat;
            }
            else
            {
                s_in_frame = 0u;
                s_len = 0u;
            }
        }

        loop++;
    }
}
