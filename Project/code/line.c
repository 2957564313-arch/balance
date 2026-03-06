#include "line.h"

// ================================== 内部宏定义 ==================================
// 引脚映射（GPIO 数字输入）：黑线=高电平(1)，白线=低电平(0)
#define TR_L2   IO_P00
#define TR_L1   IO_P01
#define TR_M    IO_P05
#define TR_R1   IO_P06
#define TR_R2   IO_P13

// 权重（5 路数字灰度），可按传感器实际间距微调
#define WEIGHT_L2   (-4.0f)
#define WEIGHT_L1   (-2.0f)
#define WEIGHT_M    ( 0.0f)
#define WEIGHT_R1   ( 2.0f)
#define WEIGHT_R2   ( 4.0f)

// 消抖与事件参数（基于 5ms 节拍）
#define LINE_DB_WHITE_TICKS              (3u)    // 连续15ms判定全白稳定
#define LINE_DB_HASBLACK_TICKS           (2u)    // 连续10ms判定"有黑"稳定

#define LINE_DB_ALLBLACK_TICKS           (2u)    // 连续10ms判定全黑稳定
#define LINE_DB_NOTALLBLACK_TICKS        (2u)    // 连续10ms判定退出全黑

#define LINE_EVT_WB_COOLDOWN_TICKS       (4u)    // 黑白事件最小间隔 20ms
#define LINE_EVT_ALLBLACK_COOLDOWN_TICKS (4u)    // 全黑事件最小间隔 20ms

#define LINE_CENTER_DEAD_BAND            (0.30f) // 误差接近0时 不强行改方向（防抖）

// 位定义：raw_bits = [L2 L1 M R1 R2] -> bit0..bit4
#define LINE_BIT_L2                      (0x01u)
#define LINE_BIT_L1                      (0x02u)
#define LINE_BIT_M                       (0x04u)
#define LINE_BIT_R1                      (0x08u)
#define LINE_BIT_R2                      (0x10u)

// ================================== 内部状态变量 ==================================
static line_info_t s_line;

static uint8 s_cnt_white;                               // 全白消抖计数
static uint8 s_cnt_has_black;                           // 有黑消抖计数
static uint8 s_cnt_all_black;                           // 全黑消抖计数
static uint8 s_cnt_not_all_black;                       // 非全黑消抖计数

static uint8 s_cd_wb;                                   // 黑白事件冷却计数
static uint8 s_cd_allblack;                             // 全黑事件冷却计数

// ================================== 内部函数声明 ==================================
static void line_gpio_init_all(void);
static void line_sample_raw_and_error(void);
static void line_update_debounce_and_events(void);
static uint8 line_read_sensor_bits(void);
static uint8 line_count_black_bits(uint8 bits);
static float line_calc_weighted_error_from_bits(uint8 bits, uint8 black_cnt);
static void line_update_last_seen_side(float err);


// ================================== 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        巡线传感器 GPIO 初始化
// 参数说明        void
// 返回参数        void
// 使用示例        line_gpio_init_all();
// 备注信息        内部静态函数，配置为上拉输入，模块输出高电平表示黑线
//--------------------------------------------------------------------------------------------------
static void line_gpio_init_all(void)
{
    gpio_init(TR_L2, GPI, GPIO_HIGH, GPI_PULL_UP);      // L2 上拉输入
    gpio_init(TR_L1, GPI, GPIO_HIGH, GPI_PULL_UP);      // L1 上拉输入
    gpio_init(TR_M,  GPI, GPIO_HIGH, GPI_PULL_UP);      // M  上拉输入
    gpio_init(TR_R1, GPI, GPIO_HIGH, GPI_PULL_UP);      // R1 上拉输入
    gpio_init(TR_R2, GPI, GPIO_HIGH, GPI_PULL_UP);      // R2 上拉输入
}

//--------------------------------------------------------------------------------------------------
// 函数简介        读取 5 路传感器电平并打包为 bit 位
// 参数说明        void
// 返回参数        uint8            bit0~bit4 对应 L2 L1 M R1 R2
// 使用示例        uint8 bits = line_read_sensor_bits();
// 备注信息        内部静态函数，1 表示黑，0 表示白
//--------------------------------------------------------------------------------------------------
static uint8 line_read_sensor_bits(void)
{
    uint8 bits;
    uint8 v;

    bits = 0u;

    v = gpio_get_level(TR_L2);
    if(v) { bits |= LINE_BIT_L2; }

    v = gpio_get_level(TR_L1);
    if(v) { bits |= LINE_BIT_L1; }

    v = gpio_get_level(TR_M);
    if(v) { bits |= LINE_BIT_M; }

    v = gpio_get_level(TR_R1);
    if(v) { bits |= LINE_BIT_R1; }

    v = gpio_get_level(TR_R2);
    if(v) { bits |= LINE_BIT_R2; }

    return bits;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        统计 bit 位中黑点数量
// 参数说明        bits             传感器位图
// 返回参数        uint8            黑点数 0~5
// 使用示例        uint8 cnt = line_count_black_bits(bits);
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static uint8 line_count_black_bits(uint8 bits)
{
    uint8 cnt;

    cnt = 0u;
    if(bits & LINE_BIT_L2) { cnt++; }
    if(bits & LINE_BIT_L1) { cnt++; }
    if(bits & LINE_BIT_M)  { cnt++; }
    if(bits & LINE_BIT_R1) { cnt++; }
    if(bits & LINE_BIT_R2) { cnt++; }

    return cnt;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        根据 bit 位计算加权误差
// 参数说明        bits             传感器位图
// 参数说明        black_cnt        黑点数量
// 返回参数        float            加权平均误差 (-4.0 ~ +4.0)
// 使用示例        float err = line_calc_weighted_error_from_bits(bits, cnt);
// 备注信息        内部静态函数，black_cnt 必须大于 0
//--------------------------------------------------------------------------------------------------
static float line_calc_weighted_error_from_bits(uint8 bits, uint8 black_cnt)
{
    float sum;

    sum = 0.0f;

    if(bits & LINE_BIT_L2) { sum += WEIGHT_L2; }
    if(bits & LINE_BIT_L1) { sum += WEIGHT_L1; }
    if(bits & LINE_BIT_M)  { sum += WEIGHT_M;  }
    if(bits & LINE_BIT_R1) { sum += WEIGHT_R1; }
    if(bits & LINE_BIT_R2) { sum += WEIGHT_R2; }

    if(black_cnt > 0u)
    {
        return (sum / (float)black_cnt);
    }

    return 0.0f;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        更新最近看到黑线的方向记忆
// 参数说明        err              当前加权误差
// 返回参数        void
// 使用示例        line_update_last_seen_side(err);
// 备注信息        内部静态函数，误差位于居中死区时保持原方向记忆
//--------------------------------------------------------------------------------------------------
static void line_update_last_seen_side(float err)
{
    if(err > LINE_CENTER_DEAD_BAND)
    {
        s_line.last_seen_side = 1;                      // 黑线在右侧
    }
    else if(err < (-LINE_CENTER_DEAD_BAND))
    {
        s_line.last_seen_side = -1;                     // 黑线在左侧
    }
    else
    {
        // 居中附近保持原方向记忆
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        采样原始传感器数据并计算误差
// 参数说明        void
// 返回参数        void
// 使用示例        line_sample_raw_and_error();
// 备注信息        内部静态函数，更新 s_line 中的原始状态和误差数据
//--------------------------------------------------------------------------------------------------
static void line_sample_raw_and_error(void)
{
    uint8 bits;
    uint8 cnt;
    float err;

    bits = line_read_sensor_bits();                      // 读取传感器位图
    cnt  = line_count_black_bits(bits);                  // 统计黑点数

    s_line.raw_bits = bits;
    s_line.raw_black_count = cnt;

    s_line.raw_line_exist = (cnt > 0u) ? 1u : 0u;       // 是否有黑
    s_line.raw_all_white  = (cnt == 0u) ? 1u : 0u;      // 是否全白
    s_line.raw_all_black  = (cnt == 5u) ? 1u : 0u;      // 是否全黑
    s_line.raw_mixed      = ((cnt > 0u) && (cnt < 5u)) ? 1u : 0u; // 是否混合

    s_line.error_prev = s_line.error;                   // 保存上一次误差

    if(cnt > 0u)
    {
        err = line_calc_weighted_error_from_bits(bits, cnt); // 计算加权误差
        s_line.error = err;
        line_update_last_seen_side(err);                 // 更新方向记忆
    }
    else
    {
        err = s_line.error;                             // 全白时保持上次误差
    }

    s_line.error_diff = s_line.error - s_line.error_prev; // 计算误差差分
}

//--------------------------------------------------------------------------------------------------
// 函数简介        消抖状态更新与事件生成
// 参数说明        void
// 返回参数        void
// 使用示例        line_update_debounce_and_events();
// 备注信息        内部静态函数，包含全白/有黑、全黑/非全黑两组消抖与边沿事件
//--------------------------------------------------------------------------------------------------
static void line_update_debounce_and_events(void)
{
    uint8 prev_deb_white;
    uint8 prev_deb_allblack;

    prev_deb_white = s_line.deb_all_white;
    prev_deb_allblack = s_line.deb_all_black;

    // 冷却计数递减
    if(s_cd_wb > 0u)
    {
        s_cd_wb--;
    }
    if(s_cd_allblack > 0u)
    {
        s_cd_allblack--;
    }

    // 全白 / 有黑（二态）消抖
    if(s_line.raw_all_white)
    {
        if(s_cnt_white < 255u)
        {
            s_cnt_white++;
        }
        s_cnt_has_black = 0u;
    }
    else
    {
        if(s_cnt_has_black < 255u)
        {
            s_cnt_has_black++;
        }
        s_cnt_white = 0u;
    }

    if(s_cnt_white >= LINE_DB_WHITE_TICKS)
    {
        s_line.deb_all_white = 1u;
    }
    else if(s_cnt_has_black >= LINE_DB_HASBLACK_TICKS)
    {
        s_line.deb_all_white = 0u;
    }
    else
    {
        // 保持原状态
    }

    // 黑白边沿事件（白<->有黑）
    if(prev_deb_white != s_line.deb_all_white)
    {
        if(s_cd_wb == 0u)
        {
            if(prev_deb_white == 1u)
            {
                s_line.event_flags_latched |= LINE_EVT_WHITE_TO_BLACK;  // 全白 -> 有黑（见线）
            }
            else
            {
                s_line.event_flags_latched |= LINE_EVT_BLACK_TO_WHITE;  // 有黑 -> 全白（丢线）
            }
            s_cd_wb = LINE_EVT_WB_COOLDOWN_TICKS;
        }
    }

    // 全黑 / 非全黑（二态）消抖
    if(s_line.raw_all_black)
    {
        if(s_cnt_all_black < 255u)
        {
            s_cnt_all_black++;
        }
        s_cnt_not_all_black = 0u;
    }
    else
    {
        if(s_cnt_not_all_black < 255u)
        {
            s_cnt_not_all_black++;
        }
        s_cnt_all_black = 0u;
    }

    if(s_cnt_all_black >= LINE_DB_ALLBLACK_TICKS)
    {
        s_line.deb_all_black = 1u;
    }
    else if(s_cnt_not_all_black >= LINE_DB_NOTALLBLACK_TICKS)
    {
        s_line.deb_all_black = 0u;
    }
    else
    {
        // 保持原状态
    }

    // 全黑边沿事件
    if(prev_deb_allblack != s_line.deb_all_black)
    {
        if(s_cd_allblack == 0u)
        {
            if(s_line.deb_all_black)
            {
                s_line.event_flags_latched |= LINE_EVT_ENTER_ALL_BLACK; // 进入全黑
            }
            else
            {
                s_line.event_flags_latched |= LINE_EVT_EXIT_ALL_BLACK;  // 退出全黑
            }
            s_cd_allblack = LINE_EVT_ALLBLACK_COOLDOWN_TICKS;
        }
    }
}


// ================================== 对外 API 函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        巡线模块初始化
// 参数说明        void
// 返回参数        void
// 使用示例        Line_Init();
// 备注信息        初始化 GPIO 并复位内部状态
//--------------------------------------------------------------------------------------------------
void Line_Init(void)
{
    line_gpio_init_all();
    Line_Reset_State();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        复位内部状态
// 参数说明        void
// 返回参数        void
// 使用示例        Line_Reset_State();
// 备注信息        可在模式切换时调用，会立即采样一次原始值但不触发事件
//--------------------------------------------------------------------------------------------------
void Line_Reset_State(void)
{
    s_line.raw_bits = 0u;
    s_line.raw_black_count = 0u;
    s_line.raw_line_exist = 0u;
    s_line.raw_all_white = 1u;
    s_line.raw_all_black = 0u;
    s_line.raw_mixed = 0u;

    s_line.deb_all_white = 1u;
    s_line.deb_all_black = 0u;

    s_line.last_seen_side = 0;

    s_line.error = 0.0f;
    s_line.error_prev = 0.0f;
    s_line.error_diff = 0.0f;

    s_line.event_flags_latched = 0u;

    s_cnt_white = 0u;
    s_cnt_has_black = 0u;
    s_cnt_all_black = 0u;
    s_cnt_not_all_black = 0u;

    s_cd_wb = 0u;
    s_cd_allblack = 0u;

    line_sample_raw_and_error();                         // 立即采样一次，避免上电后使用未更新数据

    // 用当前原始值初始化消抖状态（不产生事件）
    if(s_line.raw_all_white)
    {
        s_line.deb_all_white = 1u;
    }
    else
    {
        s_line.deb_all_white = 0u;
    }

    if(s_line.raw_all_black)
    {
        s_line.deb_all_black = 1u;
    }
    else
    {
        s_line.deb_all_black = 0u;
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        5ms 定时任务
// 参数说明        void
// 返回参数        void
// 使用示例        Line_Task_5ms();
// 备注信息        完成原始数据采样、消抖更新与事件检测
//--------------------------------------------------------------------------------------------------
void Line_Task_5ms(void)
{
    line_sample_raw_and_error();                         // 采样原始数据并计算误差
    line_update_debounce_and_events();                   // 消抖与事件更新
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取加权误差
// 参数说明        void
// 返回参数        float            加权误差 (-4.0 ~ +4.0)
// 使用示例        float err = Track_Get_Weighted_Error();
// 备注信息        兼容旧接口
//--------------------------------------------------------------------------------------------------
float Track_Get_Weighted_Error(void)
{
    return s_line.error;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        判断是否存在黑线
// 参数说明        void
// 返回参数        uint8            1=有黑线 0=无黑线
// 使用示例        uint8 exist = Track_Is_Line_Exist();
// 备注信息        兼容旧接口，返回原始即时状态（未消抖）
//--------------------------------------------------------------------------------------------------
uint8 Track_Is_Line_Exist(void)
{
    return s_line.raw_line_exist;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取巡线信息结构体指针
// 参数说明        void
// 返回参数        const line_info_t *    状态快照指针
// 使用示例        const line_info_t *info = Line_Get_Info();
// 备注信息        返回内部状态指针，不建议在外部修改
//--------------------------------------------------------------------------------------------------
const line_info_t * Line_Get_Info(void)
{
    return &s_line;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取原始传感器位图
// 参数说明        void
// 返回参数        uint8            bit0~bit4 对应 L2 L1 M R1 R2
// 使用示例        uint8 bits = Line_Get_Raw_Bits();
// 备注信息
//--------------------------------------------------------------------------------------------------
uint8 Line_Get_Raw_Bits(void)
{
    return s_line.raw_bits;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取黑点数量
// 参数说明        void
// 返回参数        uint8            黑点数 0~5
// 使用示例        uint8 cnt = Line_Get_Black_Count();
// 备注信息
//--------------------------------------------------------------------------------------------------
uint8 Line_Get_Black_Count(void)
{
    return s_line.raw_black_count;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取消抖后全白状态
// 参数说明        void
// 返回参数        uint8            1=全白稳定 0=有黑
// 使用示例        uint8 white = Line_Is_AllWhite_Debounced();
// 备注信息
//--------------------------------------------------------------------------------------------------
uint8 Line_Is_AllWhite_Debounced(void)
{
    return s_line.deb_all_white;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取消抖后全黑状态
// 参数说明        void
// 返回参数        uint8            1=全黑稳定 0=非全黑
// 使用示例        uint8 black = Line_Is_AllBlack_Debounced();
// 备注信息
//--------------------------------------------------------------------------------------------------
uint8 Line_Is_AllBlack_Debounced(void)
{
    return s_line.deb_all_black;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取最近看到黑线的方向
// 参数说明        void
// 返回参数        int8             -1=左 0=居中/未知 +1=右
// 使用示例        int8 side = Line_Get_Last_Seen_Side();
// 备注信息
//--------------------------------------------------------------------------------------------------
int8 Line_Get_Last_Seen_Side(void)
{
    return s_line.last_seen_side;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        获取误差差分
// 参数说明        void
// 返回参数        float            本次误差 - 上次误差
// 使用示例        float diff = Line_Get_Error_Diff();
// 备注信息
//--------------------------------------------------------------------------------------------------
float Line_Get_Error_Diff(void)
{
    return s_line.error_diff;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        读取事件标志（不清除）
// 参数说明        void
// 返回参数        uint8            累积事件标志位
// 使用示例        uint8 flags = Line_Get_EventFlags();
// 备注信息
//--------------------------------------------------------------------------------------------------
uint8 Line_Get_EventFlags(void)
{
    return s_line.event_flags_latched;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        读取并清除事件标志
// 参数说明        void
// 返回参数        uint8            读取前的事件标志位
// 使用示例        uint8 flags = Line_Pop_EventFlags();
// 备注信息        建议状态机每 20ms 调用一次
//--------------------------------------------------------------------------------------------------
uint8 Line_Pop_EventFlags(void)
{
    uint8 ret;

    ret = s_line.event_flags_latched;
    s_line.event_flags_latched = 0u;
    return ret;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        按位清除指定事件标志
// 参数说明        flags            要清除的事件位
// 返回参数        void
// 使用示例        Line_Clear_EventFlags(LINE_EVT_WHITE_TO_BLACK);
// 备注信息
//--------------------------------------------------------------------------------------------------
void Line_Clear_EventFlags(uint8 flags)
{
    s_line.event_flags_latched = (uint8)(s_line.event_flags_latched & (uint8)(~flags));
}