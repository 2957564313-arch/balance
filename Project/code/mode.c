#include "mode.h"

// ================================== 对外变量定义 ==================================
uint8 current_mode = MODE_1_BALANCE;                  // 当前运行模式
int16 target_speed_val = 0;                           // Mode2/3 速度目标（counts/20ms）
int16 target_turn_val  = 0;                           // Mode2/3 转向目标

volatile uint8 g_speed_pid_reset_req = 0u;               // 请求 main 在下个20ms清速度环积分

// ================================== 外部变量声明 ==================================
extern int16 imu660ra_gyro_z;                         // IMU 陀螺 Z 轴原始值（yaw-rate LSB）
extern volatile int16 car_speed;                                   // 车体速度（counts/20ms）

// ================================== Mode2/3 说明 ==================================
// MODE2 / MODE3（混合闭环）
//  - 主转向：灰度加权误差 PD（黑线圆弧段）
//  - 白色断路线段：通过 gyro_z 积分进行航向保持（yaw 补偿）
//  - 黑/白事件：做段切换 + 节点提示（蜂鸣）
//
// 注意：
// 1) 这里不依赖 line.c 的“事件层”，直接在 mode.c 内做 20ms 连续判定消抖。
// 2) 保留编码器/gyro 的几何补偿策略，降低事件漏检导致流程停滞的风险。
// 3) 如方向定义与实车不一致，优先调整下方 3 个 SIGN 宏。
// ================================== Mode2/3 参数区 ==================================

// 灰度传感器引脚（与 line.c 一致：黑线=1 白底=0）
#define M23_TR_L2_IO                    (IO_P00)
#define M23_TR_L1_IO                    (IO_P01)
#define M23_TR_M_IO                     (IO_P05)
#define M23_TR_R1_IO                    (IO_P06)
#define M23_TR_R2_IO                    (IO_P13)

// 路径几何参数
#define M23_RADIUS_CM                   (40L)
#define M23_ARC_HALF_LEN_CM             (126L)  // π*40≈126cm（半圆弧长度，赛道半径可略有偏差）
#define M23_STRAIGHT_AB_CD_CM           (100L)
#define M23_DIAG_AC_BD_CM_X10           (1562L)  // 对角线 sqrt(100^2+120^2)=156.2cm

// 距离标定
#define M23_CNT_100CM_DEFAULT           (6712L)
#define M23_CNT_PER_100CM               (M23_CNT_100CM_DEFAULT)
#define M23_CNT_STRAIGHT_100CM          ((int32)M23_CNT_PER_100CM)
#define M23_CNT_ARC_HALF                ((int32)((M23_CNT_PER_100CM * M23_ARC_HALF_LEN_CM) / 100L))
#define M23_CNT_DIAG_AC_BD              ((int32)((M23_CNT_PER_100CM * M23_DIAG_AC_BD_CM_X10) / 1000L))

// gyro_z 原始 LSB 积分（20ms 调用一次）
#define M23_YAW_SUM_180DEG              (295200L)  // 1000dps 档约 32.8LSB/dps，对应 180deg@20ms
#define M23_YAW_SWITCH_MARGIN           (18000L)
#define M23_YAW_ARC_DONE_MARGIN         (14000L)

// 方向符号
#define M23_GZ_SUM_SIGN                 (1)  // gyro_z 积分方向
#define M23_TURN_CMD_SIGN               (1)  // 转向指令方向
#define M23_SPEED_CMD_SIGN              (1)  // 前进速度符号

// 速度与转向参数
#define M23_SPEED_CMD_STRAIGHT          (8)
#define M23_SPEED_CMD_ARC               (5)
#define M23_SPEED_CMD_ARC_LOST          (4)   // 弧线丢线时间较长时进一步降速（防冲出弯）

// 轻量防后溜补偿（避免恢复大段速度补偿代码导致 CODE/HCONST 再爆）
#define M23_REV_GUARD_DB                (1)
#define M23_REV_GUARD_BOOST_WHITE       (2)
#define M23_REV_GUARD_BOOST_ARC         (10)
#define M23_TURN_MAX                    (350)
#define M23_ARC_TURN_FF                 (4)  // 圆弧段小前馈，减轻迟滞
#define M23_SPD_SLEW_PER_TICK           (1)   // 20ms 速度指令斜坡，防阶跃扰动
#define M23_PREDECEL_START_PCT          (90L) // 直道 70~85% 处开始预减速

// 白段（断路）yaw 保持
#define M23_YAW_P_DIV_WHITE             (520L)
#define M23_YAW_D_DIV_WHITE             (140L)

// ============================= Mode3（对角+半圆）白段子阶段参数 =============================
// 说明：
//  - 仅对 Mode3 的两段“白斜线”（seg_idx=0/2）生效。
//  - 白段内部拆三段：对角对准（低速）-> 对角保持 -> 末端回归直线（便于进入弯道）
//  - 对角角度不取几何对角线全角（约 50deg），默认更小，以降低横向越线风险。
//  - 角度/距离均为“起跑默认值”，后续你可按实车测量再微调。

// 对角偏置角度（度），建议 25~40
#define M23_M3_DIAG_DEG                 (71L)

// 对角偏置方向：seg0(A->C) 与 seg2(B->D) 默认相反（奇偶翻转）
#define M23_M3_DIAG_SIGN_SEG0           (+1)
#define M23_M3_DIAG_SIGN_SEG2           (-1)

// 对角偏置对应 yaw 积分目标（与 M23_YAW_SUM_180DEG 同量纲）
#define M23_M3_DIAG_OFFS_SUM            ((int32)((M23_YAW_SUM_180DEG * (int32)M23_M3_DIAG_DEG) / 180L))

// “对准完成”允许误差（度）
#define M23_M3_YAW_OK_DEG               (6L)
#define M23_M3_YAW_OK_SUM               ((int32)((M23_YAW_SUM_180DEG * (int32)M23_M3_YAW_OK_DEG) / 180L))

// 对准阶段最大距离（cm）：超过后若仍未对准，则直接进入对角保持，避免流程停滞
#define M23_M3_ALIGN_MAX_CM             (32L)

// 对角保持阶段结束点（占本段 seg_dist 百分比）：之后进入“直线回归”
#define M23_M3_DIAG_RUN_PCT             (72L)

// 直线回归阶段：至少前进该距离（cm）后才允许“检测黑线切段”
#define M23_M3_TAN_MIN_CM               (10L)

// Mode3 白段段切换瞬态抑制（20ms）：原有 M23_SEG_TRANSIENT_TICKS 较长时，会影响对角对准
#define M23_M3_SEG_TRANSIENT_TICKS      (10u)   // 20ms：Mode3 白段更早开始纠偏

// 直线回归阶段 yaw P 增益（除数越小控制越强）；0 表示沿用 M23_YAW_P_DIV_WHITE
#define M23_M3_YAW_P_DIV_TAN            (320L)

// Mode3 白段三段式速度（counts/20ms）。速度过小可能卡静摩擦。
#define M23_M3_SPD_ALIGN                (2)
#define M23_M3_SPD_DIAG                 (10)
#define M23_M3_SPD_TAN                  (2)

// 段切换瞬态抑制：刚出弯/刚切段时 gyro_z 往往仍有残余角速度
// 该现象会使 yaw 的 D 项在首帧出现瞬时反向修正，并被当作新航向基准。
// 用几帧时间关闭 D 项，让车头先稳定再进入正常航向保持。
#define M23_SEG_TRANSIENT_TICKS         (30u)   // 4*20ms=80ms

// 圆弧段 yaw 补偿 / 修正
#define M23_YAW_TRIM_DIV_ARC_END        (3000L)  // 出弯末段加大yaw收敛，车头对齐切线
#define M23_YAW_P_DIV_ARC_EXIT          (550L)   // 出弯末段全白时：使用 yaw P 对齐（不再持续保持固定舵量）
#define M23_ARC_EXIT_RELAX_PCT          (85L)    // 弧线末段比例：开始减小舵量并对齐
#define M23_ARC_FF_FADE_START_PCT       (85L)    // 前馈开始衰减比例：避免出弯后继续维持较大舵量

// 黑线段主控：灰度加权 PD
#define M23_LINE_KP                     (55.0f)
#define M23_LINE_KD                     (14.0f)
#define M23_LINE_OUT_LIMIT              (350.0f)

// 事件层消抖（20ms 节拍）
#define M23_DB_WHITE_TICKS              (2u)  // 40ms
#define M23_DB_BLACK_TICKS              (2u)  // 40ms
#define M23_DB_ALLBLACK_TICKS           (2u)  // 40ms

// 弧线丢线鲁棒性（20ms 节拍）
#define M23_ARC_LOST_HOLD_TICKS         (8u)  // 120ms：短暂盲区保持上一转向，抑制方向抖动
#define M23_ARC_END_WHITE_TICKS         (14u)  // 160ms：弧线结束判定需要更长全白
#define M23_ARC_END_NEAR_PCT            (94L) // 弧线结束里程阈值（非终点弧线）
#define M23_ARC_END_NEAR_PCT_FINISH     (99L) // 终点弧线更严格：避免到A前误判结束
#define M23_ARC_FAILSAFE_PCT            (135L) // 补偿策略：超过预估弧长 135% 且未切段时强制推进（避免流程停滞）

// 段切换最小行驶比例（避免起始抖动误切换）
#define M23_WHITE_SWITCH_MIN_PCT        (45L)  // 直道至少行驶 45% 后才允许检测黑线切段
#define M23_ARC_SWITCH_MIN_PCT          (55L)  // 半圆至少行驶 55% 后才允许检测全白切段

// 蜂鸣参数
#define M23_NODE_BEEP_MS                (70u)
#define M23_DONE_BEEP_MS                (120u)
#define M23_FINISH_HOLD_ENABLE          (1)

// 状态定义
typedef enum
{
    M23_IDLE = 0,
    M23_CAL,
    M23_RUN,
    M23_DONE
} m23_run_state_t;

typedef enum
{
    M23_SEG_WHITE = 0,  // 白底断路线段：yaw 补偿
    M23_SEG_ARC_BLACK  // 黑线半圆弧：灰度PD主控
} m23_seg_kind_t;

typedef struct
{
    uint8 kind;  // 段类型：m23_seg_kind_t
    int32 dist_counts;  // 本段距离阈值（编码器平均增量累计绝对值）
    int32 yaw_delta_sum;  // 本段目标航向变化（arc 段使用，white 段为 0）
    int16 spd_cmd;
    int16 arc_ff;  // arc 段前馈（white 段=0）
} m23_seg_t;

#define M23_MAX_SEGMENTS                (8)

// ================================== Mode2/3 内部状态变量 ==================================
static m23_seg_t s_m23_seg[M23_MAX_SEGMENTS];         // 路线段序列
static uint8  s_m23_seg_num = 0u;                     // 当前路线段总数
static uint8  s_m23_seg_idx = 0u;                     // 当前执行段索引
static uint8  s_m23_laps_total = 0u;                  // 总圈数
static uint8  s_m23_laps_done = 0u;                   // 已完成圈数

static uint8  s_m23_active = 0u;                      // Mode2/3 激活标志
static uint8  s_m23_state  = (uint8)M23_IDLE;         // 当前运行状态

// 当前段运行量
static int32  s_m23_dist_acc = 0;                     // 编码器距离累计
static int32  s_m23_seg_yaw_acc = 0;                  // 航向积分累计
static int16  s_m23_gz_last = 0;                      // 上一次 gyro_z 校正值

// 全局 yaw（从 RUN 开始累计，不随段清零）
static int32  s_m23_yaw_global_acc = 0;               // yaw 累计（LSB 积分）
static int32  s_m23_yaw_goal_base = 0;                // 当前段起始期望航向（LSB）
static uint8  s_m23_seg_transient = 0u;               // 段切换瞬态计数
static uint8  s_m23_m3_phase = 0u;                  // Mode3 白段子阶段：0无/1对准/2对角/3直线回归

// 陀螺零偏估计
static int32  s_m23_bias_sum = 0;                     // 标定累加和
static uint16 s_m23_bias_cnt = 0u;                    // 标定样本数
static uint16 s_m23_cal_cnt  = 0u;                    // 标定倒计数

// 灰度 PD 状态
static float  s_m23_line_err_last = 0.0f;             // 上一次灰度误差
static uint8  s_m23_arc_seen_black = 0u;              // 圆弧段是否已见黑线
static uint8  s_m23_arc_lost_cnt = 0u;                // 圆弧段丢线（全白）连续计数（20ms）
static int16  s_m23_turn_hold = 0;                    // 丢线保持的转向指令（未限幅前）
static int16  s_m23_spd_cmd_now = 0;                  // 速度指令斜坡输出（counts/20ms）

// 事件层计数/稳定态
static uint8  s_m23_white_cnt = 0u;                   // 全白连续计数
static uint8  s_m23_black_cnt = 0u;                   // 有黑连续计数
static uint8  s_m23_allblack_cnt = 0u;                // 全黑连续计数
static uint8  s_m23_white_stable = 0u;                // 全白消抖稳定态
static uint8  s_m23_black_stable = 0u;                // 有黑消抖稳定态
static uint8  s_m23_allblack_stable = 0u;             // 全黑消抖稳定态

// ================================== Mode2/3 内部函数声明 ==================================

static void  mode23_build_route(uint8 mode);
static void  mode23_reset_runtime(void);
static void  mode23_enter_cal(void);
static void  mode23_reset_event_layer(void);
static void  mode23_update_events(uint8 pattern, uint8 *edge_white_on, uint8 *edge_black_on, uint8 *edge_allblack_on);
static void  mode23_advance_segment(void);
static void  mode23_stop_to_done(void);
static uint8 mode23_line_pattern_read(void);
static int32 mode23_abs32(int32 x);
static int16 mode23_limit_s16(int16 x, int16 lim);
static int16 mode23_line_pd_turn(void);
static int16 mode23_yaw_pd_turn(int32 yaw_err, int16 gz_corr, int32 p_div, int32 d_div);
static int32 mode23_segment_min_switch_dist(const m23_seg_t *seg);
static void  mode23_apply_zero_output(void);

// ================================== Mode2/3 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        计算 int32 绝对值
// 参数说明        x                输入值
// 返回参数        int32            绝对值结果
// 使用示例        int32 a = mode23_abs32(x);
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static int32 mode23_abs32(int32 x)
{
    if (x < 0) return -x;
    return x;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        int16 对称限幅
// 参数说明        x                待限幅值
// 参数说明        lim              限幅绝对值上限
// 返回参数        int16            限幅后的值
// 使用示例        x = mode23_limit_s16(x, 200);
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static int16 mode23_limit_s16(int16 x, int16 lim)
{
    if (x > lim)  return lim;
    if (x < -lim) return (int16)(-lim);
    return x;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        读取 5 路灰度传感器并打包为 bit 位
// 参数说明        void
// 返回参数        uint8            bit0~bit4 对应 L2 L1 M R1 R2（1=黑 0=白）
// 使用示例        uint8 p = mode23_line_pattern_read();
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static uint8 mode23_line_pattern_read(void)
{
    uint8 p;
    p = 0u;
    if (gpio_get_level(M23_TR_L2_IO)) p |= 0x01u;
    if (gpio_get_level(M23_TR_L1_IO)) p |= 0x02u;
    if (gpio_get_level(M23_TR_M_IO )) p |= 0x04u;
    if (gpio_get_level(M23_TR_R1_IO)) p |= 0x08u;
    if (gpio_get_level(M23_TR_R2_IO)) p |= 0x10u;
    return p;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        复位事件层消抖状态
// 参数说明        void
// 返回参数        void
// 使用示例        mode23_reset_event_layer();
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static void mode23_reset_event_layer(void)
{
    s_m23_white_cnt = 0u;
    s_m23_black_cnt = 0u;
    s_m23_allblack_cnt = 0u;
    s_m23_white_stable = 0u;
    s_m23_black_stable = 0u;
    s_m23_allblack_stable = 0u;
    s_m23_arc_seen_black = 0u;
    s_m23_arc_lost_cnt = 0u;
    s_m23_turn_hold = 0;
    s_m23_line_err_last = 0.0f;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        更新事件层消抖并检测边沿事件
// 参数说明        pattern          当前传感器位图
// 参数说明        edge_white_on    输出 全白上升沿
// 参数说明        edge_black_on    输出 有黑上升沿
// 参数说明        edge_allblack_on 输出 全黑上升沿
// 返回参数        void
// 使用示例        mode23_update_events(pattern, &ew, &eb, &eab);
// 备注信息        内部静态函数 20ms 节拍消抖
//--------------------------------------------------------------------------------------------------
static void mode23_update_events(uint8 pattern, uint8 *edge_white_on, uint8 *edge_black_on, uint8 *edge_allblack_on)
{
    uint8 now_white;
    uint8 now_black;
    uint8 now_allblack;
    uint8 white_st;
    uint8 black_st;
    uint8 allblack_st;

    now_white    = (uint8)((pattern == 0u) ? 1u : 0u);
    now_black    = (uint8)((pattern != 0u) ? 1u : 0u);
    now_allblack = (uint8)((pattern == 0x1Fu) ? 1u : 0u);

    if (now_white)
    {
        if (s_m23_white_cnt < 255u) s_m23_white_cnt++;
    }
    else
    {
        s_m23_white_cnt = 0u;
    }

    if (now_black)
    {
        if (s_m23_black_cnt < 255u) s_m23_black_cnt++;
    }
    else
    {
        s_m23_black_cnt = 0u;
    }

    if (now_allblack)
    {
        if (s_m23_allblack_cnt < 255u) s_m23_allblack_cnt++;
    }
    else
    {
        s_m23_allblack_cnt = 0u;
    }

    white_st    = (uint8)((s_m23_white_cnt    >= M23_DB_WHITE_TICKS)    ? 1u : 0u);
    black_st    = (uint8)((s_m23_black_cnt    >= M23_DB_BLACK_TICKS)    ? 1u : 0u);
    allblack_st = (uint8)((s_m23_allblack_cnt >= M23_DB_ALLBLACK_TICKS) ? 1u : 0u);

    *edge_white_on    = (uint8)((white_st    == 1u) && (s_m23_white_stable    == 0u));
    *edge_black_on    = (uint8)((black_st    == 1u) && (s_m23_black_stable    == 0u));
    *edge_allblack_on = (uint8)((allblack_st == 1u) && (s_m23_allblack_stable == 0u));

    s_m23_white_stable    = white_st;
    s_m23_black_stable    = black_st;
    s_m23_allblack_stable = allblack_st;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        灰度加权 PD 计算转向命令
// 参数说明        void
// 返回参数        int16            转向输出（含限幅）
// 使用示例        int16 t = mode23_line_pd_turn();
// 备注信息        内部静态函数 用于黑线圆弧段
//--------------------------------------------------------------------------------------------------
static int16 mode23_line_pd_turn(void)
{
    float err;
    float derr;
    float out_f;
    int16 out_i;

    err = Track_Get_Weighted_Error();
    derr = err - s_m23_line_err_last;
    s_m23_line_err_last = err;

    out_f = (M23_LINE_KP * err) + (M23_LINE_KD * derr);

    if (out_f > M23_LINE_OUT_LIMIT)  out_f = M23_LINE_OUT_LIMIT;
    if (out_f < -M23_LINE_OUT_LIMIT) out_f = -M23_LINE_OUT_LIMIT;

    if (out_f >= 0.0f) out_i = (int16)(out_f + 0.5f);
    else               out_i = (int16)(out_f - 0.5f);

    return out_i;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        基于 yaw 误差的 PD 转向计算
// 参数说明        yaw_err          航向误差（LSB 积分）
// 参数说明        gz_corr          校正后角速度
// 参数说明        p_div            P 项除数
// 参数说明        d_div            D 项除数（0=禁用 D）
// 返回参数        int16            转向输出
// 使用示例        int16 t = mode23_yaw_pd_turn(err, gz, 520, 140);
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static int16 mode23_yaw_pd_turn(int32 yaw_err, int16 gz_corr, int32 p_div, int32 d_div)
{
    int32 out32;
    int32 d_term;

    out32 = 0;
    d_term = 0;

    if (p_div != 0L)
    {
        out32 = -(yaw_err / p_div);
    }

    if (d_div != 0L)
    {
        d_term = -((int32)gz_corr / d_div);
        out32 += d_term;
    }

    if (out32 > 32767L) out32 = 32767L;
    if (out32 < -32768L) out32 = -32768L;

    return (int16)out32;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        计算当前段允许切换的最小行驶距离
// 参数说明        seg              当前段描述指针
// 返回参数        int32            最小行驶距离（counts）
// 使用示例        int32 d = mode23_segment_min_switch_dist(seg);
// 备注信息        内部静态函数 避免起步抖动误触发段切换
//--------------------------------------------------------------------------------------------------
static int32 mode23_segment_min_switch_dist(const m23_seg_t *seg)
{
    int32 min_dist;
    int32 pct;

    if (seg->dist_counts <= 0) return 1L;

    if (seg->kind == (uint8)M23_SEG_WHITE)
    {
        pct = M23_WHITE_SWITCH_MIN_PCT;
    }
    else
    {
        pct = M23_ARC_SWITCH_MIN_PCT;
    }

    min_dist = (int32)((seg->dist_counts * pct) / 100L);
    if (min_dist < 1L) min_dist = 1L;
    return min_dist;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        清零模式输出（速度 + 转向目标归零）
// 参数说明        void
// 返回参数        void
// 使用示例        mode23_apply_zero_output();
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static void mode23_apply_zero_output(void)
{
    target_speed_val = 0;
    target_turn_val  = 0;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        根据模式编号构建路线段序列
// 参数说明        mode             模式编号（MODE_2_TRACK_1 / MODE_3_TRACK_2）
// 返回参数        void
// 使用示例        mode23_build_route(MODE_2_TRACK_1);
// 备注信息        内部静态函数 填充 s_m23_seg 数组
//--------------------------------------------------------------------------------------------------
static void mode23_build_route(uint8 mode)
{
    int32 cnt_s100;
    int32 cnt_arc;
    int32 cnt_diag;
    int32 yaw180;
    int16 v_st;
    int16 v_arc;
    int16 ff;

    cnt_s100 = (int32)M23_CNT_STRAIGHT_100CM;
    cnt_arc  = (int32)M23_CNT_ARC_HALF;
    cnt_diag = (int32)M23_CNT_DIAG_AC_BD;
    yaw180   = (int32)M23_YAW_SUM_180DEG;

    v_st  = (int16)(M23_SPEED_CMD_SIGN * M23_SPEED_CMD_STRAIGHT);
    v_arc = (int16)(M23_SPEED_CMD_SIGN * M23_SPEED_CMD_ARC);
    ff    = (int16)M23_ARC_TURN_FF;

    s_m23_seg_num = 0u;
    s_m23_laps_done = 0u;

    if (mode == (uint8)MODE_2_TRACK_1)
    {
        // A->B(白直) -> B->C(右半圆黑线) -> C->D(白直) -> D->A(左半圆黑线)
        s_m23_seg[0].kind = (uint8)M23_SEG_WHITE;     s_m23_seg[0].dist_counts = cnt_s100; s_m23_seg[0].yaw_delta_sum = 0;      s_m23_seg[0].spd_cmd = v_st;  s_m23_seg[0].arc_ff = 0;
        s_m23_seg[1].kind = (uint8)M23_SEG_ARC_BLACK; s_m23_seg[1].dist_counts = cnt_arc;  s_m23_seg[1].yaw_delta_sum = -yaw180; s_m23_seg[1].spd_cmd = v_arc; s_m23_seg[1].arc_ff = (int16)(-ff);
        s_m23_seg[2].kind = (uint8)M23_SEG_WHITE;     s_m23_seg[2].dist_counts = cnt_s100; s_m23_seg[2].yaw_delta_sum = 0;      s_m23_seg[2].spd_cmd = v_st;  s_m23_seg[2].arc_ff = 0;
        s_m23_seg[3].kind = (uint8)M23_SEG_ARC_BLACK; s_m23_seg[3].dist_counts = cnt_arc;  s_m23_seg[3].yaw_delta_sum = +yaw180; s_m23_seg[3].spd_cmd = v_arc; s_m23_seg[3].arc_ff = (int16)(+ff);
        s_m23_seg_num = 4u;
        s_m23_laps_total = 1u;
    }
    else
    {
        // MODE3：A->C(白斜) -> C->B(右半圆黑线) -> B->D(白斜) -> D->A(左半圆黑线)，4圈
        s_m23_seg[0].kind = (uint8)M23_SEG_WHITE;     s_m23_seg[0].dist_counts = cnt_diag; s_m23_seg[0].yaw_delta_sum = 0;      s_m23_seg[0].spd_cmd = v_st;  s_m23_seg[0].arc_ff = 0;
        // C->B：右半圆从下到上，几何上为左转（CCW）。yaw_mdeg 增大=右转(CW)，因此此段 yaw_delta_sum 为负。
        // 说明：圆弧段仍以灰度 PD 为主，yaw_delta_sum 主要用于末段全白时的补偿对齐。
        s_m23_seg[1].kind = (uint8)M23_SEG_ARC_BLACK; s_m23_seg[1].dist_counts = cnt_arc;  s_m23_seg[1].yaw_delta_sum = -yaw180; s_m23_seg[1].spd_cmd = v_arc; s_m23_seg[1].arc_ff = (int16)(-ff);
        s_m23_seg[2].kind = (uint8)M23_SEG_WHITE;     s_m23_seg[2].dist_counts = cnt_diag; s_m23_seg[2].yaw_delta_sum = 0;      s_m23_seg[2].spd_cmd = v_st;  s_m23_seg[2].arc_ff = 0;
        // D->A：左半圆从下到上，几何上为右转（CW），因此此段 yaw_delta_sum 为正。
        s_m23_seg[3].kind = (uint8)M23_SEG_ARC_BLACK; s_m23_seg[3].dist_counts = cnt_arc;  s_m23_seg[3].yaw_delta_sum = +yaw180; s_m23_seg[3].spd_cmd = v_arc; s_m23_seg[3].arc_ff = (int16)(+ff);
        s_m23_seg_num = 4u;
        s_m23_laps_total = 4u;
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        复位 Mode2/3 运行时状态
// 参数说明        void
// 返回参数        void
// 使用示例        mode23_reset_runtime();
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static void mode23_reset_runtime(void)
{
    s_m23_seg_idx = 0u;
    s_m23_dist_acc = 0;
    s_m23_seg_yaw_acc = 0;
    s_m23_gz_last = 0;

    s_m23_yaw_global_acc = 0;
    s_m23_yaw_goal_base = 0;
    s_m23_seg_transient = 0u;
    s_m23_m3_phase = 0u;

    s_m23_spd_cmd_now = 0;
    mode23_reset_event_layer();
    mode23_apply_zero_output();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        进入陀螺标定阶段
// 参数说明        void
// 返回参数        void
// 使用示例        mode23_enter_cal();
// 备注信息        内部静态函数 400ms 标定窗口
//--------------------------------------------------------------------------------------------------
static void mode23_enter_cal(void)
{
    s_m23_bias_sum = 0;
    s_m23_bias_cnt = 0u;
    s_m23_cal_cnt = 20u;  // 20*20ms=400ms
    s_m23_state = (uint8)M23_CAL;
    mode23_reset_runtime();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        推进到下一路线段或下一圈
// 参数说明        void
// 返回参数        void
// 使用示例        mode23_advance_segment();
// 备注信息        内部静态函数 到尾则计圈并判断是否结束
//--------------------------------------------------------------------------------------------------
static void mode23_advance_segment(void)
{
    g_speed_pid_reset_req = 1u;
    s_m23_dist_acc = 0;
    s_m23_seg_yaw_acc = 0;
    s_m23_gz_last = 0;
    mode23_reset_event_layer();

    // 新段起始的“航向参考”先取当前 yaw（防止出弯残余角速度被当作偏航记住）
    s_m23_yaw_goal_base = s_m23_yaw_global_acc;

    // 段切换后短暂抑制 yaw D 项，避免“出弯反打一下”被固化为新航向
    s_m23_seg_transient = (uint8)M23_SEG_TRANSIENT_TICKS;
    s_m23_m3_phase = 0u;

    if ((uint8)(s_m23_seg_idx + 1u) < s_m23_seg_num)
    {
        s_m23_seg_idx++;
        Buzzer_Beep((uint16)M23_NODE_BEEP_MS);
        return;
    }

    s_m23_laps_done++;

    if (s_m23_laps_done < s_m23_laps_total)
    {
        s_m23_seg_idx = 0u;
        Buzzer_Beep((uint16)M23_NODE_BEEP_MS);
        return;
    }

    mode23_stop_to_done();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        停车并进入 DONE 状态
// 参数说明        void
// 返回参数        void
// 使用示例        mode23_stop_to_done();
// 备注信息        内部静态函数 全圈完成时调用
//--------------------------------------------------------------------------------------------------
static void mode23_stop_to_done(void)
{
    s_m23_state = (uint8)M23_DONE;
    s_m23_spd_cmd_now = 0;
    s_m23_m3_phase = 0u;
    mode23_apply_zero_output();
    Buzzer_Beep((uint16)M23_DONE_BEEP_MS);

#if (M23_FINISH_HOLD_ENABLE == 0)
    run_flag = 0;
#endif
}

// ================================== Mode2/3 对外 API 函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介      模式模块初始化
// 参数说明      void
// 返回参数      void
// 使用示例      Mode_Init();
// 备注信息      上电调用一次，复位 Mode2/3 状态与走线参数
//--------------------------------------------------------------------------------------------------
void Mode_Init(void)
{
    current_mode = MODE_1_BALANCE;
    target_speed_val = 0;
    target_turn_val  = 0;

    s_m23_seg_num = 0u;
    s_m23_seg_idx = 0u;
    s_m23_laps_total = 0u;
    s_m23_laps_done = 0u;

    s_m23_active = 0u;
    s_m23_state = (uint8)M23_IDLE;

    s_m23_dist_acc = 0;
    s_m23_seg_yaw_acc = 0;
    s_m23_gz_last = 0;
    s_m23_spd_cmd_now = 0;

    s_m23_yaw_global_acc = 0;
    s_m23_yaw_goal_base = 0;
    s_m23_seg_transient = 0u;
    s_m23_m3_phase = 0u;

    s_m23_bias_sum = 0;
    s_m23_bias_cnt = 0u;
    s_m23_cal_cnt = 0u;

    s_m23_line_err_last = 0.0f;
    mode23_reset_event_layer();
}

//--------------------------------------------------------------------------------------------------
// 函数简介      启动指定运行模式
// 参数说明      mode    模式编号（MODE_2_TRACK_1 / MODE_3_TRACK_2 等）
// 返回参数      void
// 使用示例      Mode_Start(MODE_2_TRACK_1);
// 备注信息      Mode2/3 时构建路线并进入标定，其他模式仅设置 current_mode
//--------------------------------------------------------------------------------------------------
void Mode_Start(uint8 mode)
{
    current_mode = mode;
    mode23_apply_zero_output();

    if ((mode == (uint8)MODE_2_TRACK_1) || (mode == (uint8)MODE_3_TRACK_2))
    {
        s_m23_active = 1u;
        mode23_build_route(mode);
        mode23_enter_cal();
        return;
    }

    s_m23_active = 0u;
    s_m23_state = (uint8)M23_IDLE;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      停止当前运行模式
// 参数说明      void
// 返回参数      void
// 使用示例      Mode_Stop();
// 备注信息      停止 Mode2/3 走线，清除输出
//--------------------------------------------------------------------------------------------------
void Mode_Stop(void)
{
    s_m23_active = 0u;
    s_m23_state = (uint8)M23_IDLE;
    g_speed_pid_reset_req = 1u;
    s_m23_spd_cmd_now = 0;

    s_m23_yaw_global_acc = 0;
    s_m23_yaw_goal_base = 0;
    s_m23_seg_transient = 0u;
    s_m23_m3_phase = 0u;

    mode23_apply_zero_output();
}

//--------------------------------------------------------------------------------------------------
// 函数简介      Mode2/3 主控制任务（20ms 周期）
// 参数说明      void
// 返回参数      void
// 使用示例      Mode_Task_20ms();
// 备注信息      包含灰度读取、事件检测、段切换、PD 转向、速度控制
//--------------------------------------------------------------------------------------------------
void Mode_Task_20ms(void)
{
    uint8 pattern;
    uint8 edge_white_on;
    uint8 edge_black_on;
    uint8 edge_allblack_on;
    uint8 now_white;
    uint8 now_has_black;
    uint8 white_long;
    int16 spd_next;
    int32 decel_start;
    int32 decel_range;
    int32 decel_prog;
    int32 near_dist;
    m23_seg_t *seg_next;
    int16 inc_l;
    int16 inc_r;
    int16 inc;
    int16 gz_now;
    int16 gz_bias;
    int16 gz_corr;
    int16 turn_cmd;
    int16 yaw_turn;
    int16 line_turn;
    int16 spd_cmd;
    int32 dist_abs;
    int32 seg_dist;
    int32 min_switch_dist;
    int32 yaw_target;
    int32 yaw_err;
    int32 dist_for_yaw;
    int32 exit_relax_dist;
    int32 ff_fade_start;
    int32 ff_num;
    int32 ff_den;
    int32 arc_end_near_pct;
    int16 arc_ff_now;
    m23_seg_t *seg;


    // Mode3 白斜段三段式控制（仅 seg_idx=0/2）
    uint8  m3_diag_en;
    int8   m3_sign;
    int32  m3_align_max;
    int32  m3_diag_end;
    int32  m3_tan_min;
    int32  m3_allow_black;
    int32  m3_p_div;
    int32  m3_yaw_off;
    int32  m3_abs_yaw;
    mode23_apply_zero_output();

    if (!s_m23_active)
    {
        return;
    }

    if ((current_mode != (uint8)MODE_2_TRACK_1) && (current_mode != (uint8)MODE_3_TRACK_2))
    {
        return;
    }

    if (run_flag == 0u)
    {
        return;
    }

    gz_now = (int16)imu660ra_gyro_z;

    if (s_m23_state == (uint8)M23_CAL)
    {
        if ((gz_now <= 10) && (gz_now >= -10))
        {
            s_m23_bias_sum += (int32)gz_now;
            s_m23_bias_cnt++;
        }

        if (s_m23_cal_cnt > 0u)
        {
            s_m23_cal_cnt--;
        }

        if (s_m23_cal_cnt == 0u)
        {
            mode23_reset_runtime();
            s_m23_state = (uint8)M23_RUN;
        }
        return;
    }

    if (s_m23_state == (uint8)M23_DONE)
    {
        mode23_apply_zero_output();
        return;
    }

    if (s_m23_state != (uint8)M23_RUN)
    {
        return;
    }

    if (s_m23_seg_num == 0u)
    {
        mode23_stop_to_done();
        return;
    }

    pattern = mode23_line_pattern_read();

    edge_white_on = 0u;
    edge_black_on = 0u;
    edge_allblack_on = 0u;
    mode23_update_events(pattern, &edge_white_on, &edge_black_on, &edge_allblack_on);

    inc_l = (int16)left_motor_encoder_data;
    inc_r = (int16)right_motor_encoder_data;
    inc = (int16)((inc_l + inc_r) / 2);

    gz_bias = 0;
    if (s_m23_bias_cnt > 0u)
    {
        gz_bias = (int16)(s_m23_bias_sum / (int32)s_m23_bias_cnt);
    }

    gz_corr = (int16)(gz_now - gz_bias);
    // M23_GZ_SUM_SIGN: +1=保持，-1=取反
#if (M23_GZ_SUM_SIGN == -1)
    gz_corr = (int16)(-gz_corr);
#endif

    s_m23_seg_yaw_acc += (int32)gz_corr;
    s_m23_yaw_global_acc += (int32)gz_corr;
    s_m23_dist_acc += (int32)inc;

    dist_abs = mode23_abs32(s_m23_dist_acc);

    seg = &s_m23_seg[s_m23_seg_idx];
    seg_dist = seg->dist_counts;
    if (seg_dist <= 0L) seg_dist = 1L;
    min_switch_dist = mode23_segment_min_switch_dist(seg);

    spd_cmd = seg->spd_cmd;
    turn_cmd = 0;
    yaw_turn = 0;
    line_turn = 0;

    now_white = (uint8)((pattern == 0u) ? 1u : 0u);
    now_has_black = (uint8)((pattern != 0u) ? 1u : 0u);
    white_long = 0u;
    spd_next = 0;
    decel_start = 0;
    decel_range = 0;
    decel_prog = 0;
    near_dist = 0;
    seg_next = 0;

    if (seg->kind == (uint8)M23_SEG_WHITE)
    {
        // ----------------------------------------------------------
        // Mode3：白斜段（seg0/seg2）三段式
        //  - 先对角对准（偏置角更小，以降低直接越线风险）
        //  - 再对角保持
        //  - 末端回归直线（yaw_off=0），并前进一小段后再允许“检测黑线切段”
        // ----------------------------------------------------------
        m3_diag_en = 0u;
        m3_sign = 0;
        m3_align_max = 0;
        m3_diag_end = 0;
        m3_tan_min = 0;
        m3_allow_black = 0;
        m3_p_div = 0;
        m3_yaw_off = 0;
        m3_abs_yaw = 0;

        if ((current_mode == (uint8)MODE_3_TRACK_2) && ((s_m23_seg_idx == 0u) || (s_m23_seg_idx == 2u)))
        {
            m3_diag_en = 1u;
            if (s_m23_seg_idx == 0u) m3_sign = (int8)M23_M3_DIAG_SIGN_SEG0;
            else                     m3_sign = (int8)M23_M3_DIAG_SIGN_SEG2;

            m3_align_max = (int32)(((int32)M23_CNT_PER_100CM * (int32)M23_M3_ALIGN_MAX_CM) / 100L);
            if (m3_align_max < 1L) m3_align_max = 1L;

            m3_diag_end = (int32)((seg_dist * (int32)M23_M3_DIAG_RUN_PCT) / 100L);
            if (m3_diag_end < m3_align_max) m3_diag_end = m3_align_max;

            m3_tan_min = (int32)(((int32)M23_CNT_PER_100CM * (int32)M23_M3_TAN_MIN_CM) / 100L);
            if (m3_tan_min < 1L) m3_tan_min = 1L;

            m3_allow_black = m3_diag_end + m3_tan_min;
            if (m3_allow_black > seg_dist) m3_allow_black = seg_dist;

            // Mode3 白段：避免段切换瞬态抑制过长导致“对角对不起来”
            if (s_m23_seg_transient > (uint8)M23_M3_SEG_TRANSIENT_TICKS)
            {
                s_m23_seg_transient = (uint8)M23_M3_SEG_TRANSIENT_TICKS;
            }
        }

        // 直道：yaw 保持为主；若下一段是弧线，则在 70~85% 处开始预减速（避免高速冲弯）
        if ((m3_diag_en == 0u) && ((uint8)(s_m23_seg_idx + 1u) < s_m23_seg_num))
        {
            seg_next = &s_m23_seg[s_m23_seg_idx + 1u];
            if (seg_next->kind == (uint8)M23_SEG_ARC_BLACK)
            {
                spd_next = seg_next->spd_cmd;
                decel_start = (int32)((seg_dist * (int32)M23_PREDECEL_START_PCT) / 100L);
                if (decel_start < 1L) decel_start = 1L;

                if (dist_abs >= decel_start)
                {
                    decel_range = seg_dist - decel_start;
                    if (decel_range < 1L) decel_range = 1L;
                    decel_prog = dist_abs - decel_start;
                    if (decel_prog > decel_range) decel_prog = decel_range;

                    spd_cmd = (int16)(seg->spd_cmd + (int16)(((int32)(spd_next - seg->spd_cmd) * decel_prog) / decel_range));
                }
            }
        }

        // 直道：使用“全局 yaw 误差”做航向保持。
        // 段切换后短时间内，先让航向参考跟随当前 yaw（只做角速度阻尼），
        // 避免出弯残余角速度/车体姿态回归的瞬态被固化为“新的偏航基准”。
        if (s_m23_seg_transient > 0u)
{
    s_m23_m3_phase = 0u;
    s_m23_yaw_goal_base = s_m23_yaw_global_acc;
    yaw_err = 0;

    if (m3_diag_en != 0u)
    {
        yaw_turn = 0;
    }
    else
    {
        yaw_turn = (int16)(-((int32)gz_corr / (int32)M23_YAW_D_DIV_WHITE));
    }

    s_m23_seg_transient--;
}
        else
        {
            if (m3_diag_en != 0u)
            {
                // 先按“对角偏置”计算 yaw_err，用于判断是否仍处于对准阶段
                m3_yaw_off = (int32)m3_sign * (int32)M23_M3_DIAG_OFFS_SUM;
                yaw_err = s_m23_yaw_global_acc - (s_m23_yaw_goal_base + m3_yaw_off);
                m3_abs_yaw = mode23_abs32(yaw_err);

                if ((dist_abs < m3_align_max) && (m3_abs_yaw > (int32)M23_M3_YAW_OK_SUM))
                {
                    s_m23_m3_phase = 1u;
                    // 子阶段 0-1 / 2-1：对角对准（低速）
                    spd_cmd = (int16)(M23_SPEED_CMD_SIGN * (int16)M23_M3_SPD_ALIGN);
                    m3_p_div = (int32)M23_M3_YAW_P_DIV_TAN;
                    if (m3_p_div <= 0L) m3_p_div = (int32)M23_YAW_P_DIV_WHITE;
                    yaw_turn = mode23_yaw_pd_turn(yaw_err, gz_corr, m3_p_div, 0L);
                }
                else if (dist_abs < m3_diag_end)
                {
                    s_m23_m3_phase = 2u;
                    // 子阶段 0-2 / 2-2：对角保持
                    spd_cmd = (int16)(M23_SPEED_CMD_SIGN * (int16)M23_M3_SPD_DIAG);
                    yaw_turn = mode23_yaw_pd_turn(yaw_err, gz_corr, (int32)M23_YAW_P_DIV_WHITE, 0L);
                }
                else
                {
                    if (s_m23_m3_phase != 3u)
                    {
                        g_speed_pid_reset_req = 1u;
                        s_m23_spd_cmd_now = (int16)(M23_SPEED_CMD_SIGN * (int16)M23_M3_SPD_TAN);
                        s_m23_m3_phase = 3u;
                    }
                    // 子阶段 0-3 / 2-3：末端回归直线（yaw_off=0）
                    spd_cmd = (int16)(M23_SPEED_CMD_SIGN * (int16)M23_M3_SPD_TAN);
                    yaw_err = s_m23_yaw_global_acc - s_m23_yaw_goal_base;

                    m3_p_div = (int32)M23_M3_YAW_P_DIV_TAN;
                    if (m3_p_div <= 0L) m3_p_div = (int32)M23_YAW_P_DIV_WHITE;

                    yaw_turn = mode23_yaw_pd_turn(yaw_err, gz_corr, m3_p_div, 0L);
                }
            }
            else
            {
                s_m23_m3_phase = 0u;
                yaw_err = s_m23_yaw_global_acc - s_m23_yaw_goal_base;
                yaw_turn = mode23_yaw_pd_turn(yaw_err, gz_corr, (int32)M23_YAW_P_DIV_WHITE, (int32)M23_YAW_D_DIV_WHITE);
            }
        }
        turn_cmd = yaw_turn;

        // 直道->弧线：以“稳定检测到黑线”为主切段；距离阈值仅用于防止流程停滞
        // Mode3 白斜段：必须进入“直线回归”并前进一小段后，才允许“检测黑线切段”
        if (m3_diag_en != 0u)
        {
            if (min_switch_dist < m3_allow_black) min_switch_dist = m3_allow_black;
        }

        if ((dist_abs >= min_switch_dist) && (((m3_diag_en != 0u) && (now_has_black != 0u)) || (edge_black_on != 0u) || (edge_allblack_on != 0u) || (s_m23_black_stable != 0u)))
        {
            mode23_advance_segment();
            s_m23_gz_last = gz_corr;
            return;
        }

        if (dist_abs >= seg_dist)
        {
            mode23_advance_segment();
            s_m23_gz_last = gz_corr;
            return;
        }
    }
    else
    {
        // 弧线：检测到黑线时以灰度 PD 为主；出弯末段自动减小舵量并对齐切线；短时丢线保持，末段丢线改为 yaw 对齐
        dist_for_yaw = dist_abs;
        if (dist_for_yaw > seg_dist) dist_for_yaw = seg_dist;

        yaw_target = (int32)((seg->yaw_delta_sum * dist_for_yaw) / seg_dist);
        yaw_err = s_m23_seg_yaw_acc - yaw_target;

        exit_relax_dist = (int32)((seg_dist * (int32)M23_ARC_EXIT_RELAX_PCT) / 100L);
        if (exit_relax_dist < 1L) exit_relax_dist = 1L;

        ff_fade_start = (int32)((seg_dist * (int32)M23_ARC_FF_FADE_START_PCT) / 100L);
        if (ff_fade_start < 1L) ff_fade_start = 1L;

        // 前馈在末段逐步衰减到 0：避免出弯后继续保持较大舵量导致直线对齐不足
        arc_ff_now = seg->arc_ff;
        if (dist_abs >= ff_fade_start)
        {
            ff_den = seg_dist - ff_fade_start;
            if (ff_den < 1L) ff_den = 1L;

            ff_num = seg_dist - dist_abs;
            if (ff_num < 0L) ff_num = 0L;

            arc_ff_now = (int16)(((int32)seg->arc_ff * ff_num) / ff_den);
        }

        if (now_has_black != 0u)
        {
            s_m23_arc_seen_black = 1u;
            s_m23_arc_lost_cnt = 0u;

            line_turn = mode23_line_pd_turn();
            turn_cmd = (int16)(line_turn + arc_ff_now);
            s_m23_turn_hold = turn_cmd;
        }
        else
        {
            if (now_white != 0u)
            {
                if (s_m23_arc_lost_cnt < 255u) s_m23_arc_lost_cnt++;
            }

            if (s_m23_arc_seen_black == 0u)
            {
                // 刚进入弧线但尚未捕获黑线：使用温和 yaw 补偿与前馈执行寻线
                turn_cmd = (int16)(yaw_turn + arc_ff_now);
                s_m23_turn_hold = turn_cmd;
            }
            else
            {
                if (dist_abs >= exit_relax_dist)
                {
                    // 出弯末段：传感器全白时不再持续保持固定舵量，改为 yaw 对齐并小比例沿用上一舵量（抑制抖动）
                    yaw_turn = mode23_yaw_pd_turn(yaw_err, gz_corr, (int32)M23_YAW_P_DIV_ARC_EXIT, 0L);
                    turn_cmd = (int16)(yaw_turn + (s_m23_turn_hold / 4));
                }
                else
                {
                    if (s_m23_arc_lost_cnt <= (uint8)M23_ARC_LOST_HOLD_TICKS)
                    {
                        // 短暂盲区：保持上一转向（连续性最重要）
                        turn_cmd = s_m23_turn_hold;
                    }
                    else
                    {
                        // 丢线较久：进一步降速，并衰减保持舵量，避免脱轨后原地死命乱转
                        spd_cmd = (int16)(M23_SPEED_CMD_SIGN * M23_SPEED_CMD_ARC_LOST);
                        turn_cmd = (int16)((s_m23_turn_hold / 2) + yaw_turn);
                    }
                }
            }
        }

        // 弧线结束：必须“全白持续超过宽限期”且“里程接近”才允许切段
        // 说明：yaw 不再作为提前放行条件，避免临近A/C时因为短暂全白+yaw接近造成误判。
        white_long = (uint8)((s_m23_white_cnt >= (uint8)M23_ARC_END_WHITE_TICKS) ? 1u : 0u);

        if ((s_m23_arc_seen_black != 0u) && (white_long != 0u))
        {
            arc_end_near_pct = (int32)M23_ARC_END_NEAR_PCT;

            // 最后一圈最后一段弧线（到A停车）：阈值更严格，避免提前结束
            if ((s_m23_seg_idx == (uint8)(s_m23_seg_num - 1u)) &&
                (s_m23_laps_done == (uint8)(s_m23_laps_total - 1u)))
            {
                arc_end_near_pct = (int32)M23_ARC_END_NEAR_PCT_FINISH;
            }

            near_dist = (int32)((seg_dist * arc_end_near_pct) / 100L);
            if (near_dist < 1L) near_dist = 1L;

            if (dist_abs >= near_dist)
            {
                mode23_advance_segment();
                s_m23_gz_last = gz_corr;
                return;
            }
        }

        // 补偿防护：超过预估弧长一定比例且仍未切段时，执行强制推进
        if (dist_abs >= (int32)((seg_dist * (int32)M23_ARC_FAILSAFE_PCT) / 100L))
        {
            mode23_advance_segment();
            s_m23_gz_last = gz_corr;
            return;
        }
    }

    if ((spd_cmd > 0) && (car_speed < (int16)(-(int16)M23_REV_GUARD_DB)))
    {
        if (seg->kind == (uint8)M23_SEG_ARC_BLACK)
        {
            spd_cmd = (int16)(spd_cmd + (int16)M23_REV_GUARD_BOOST_ARC);
        }
        else
        {
            spd_cmd = (int16)(spd_cmd + (int16)M23_REV_GUARD_BOOST_WHITE);
        }
    }

    // M23_TURN_CMD_SIGN: +1=保持，-1=取反
#if (M23_TURN_CMD_SIGN == -1)
    turn_cmd = (int16)(-turn_cmd);
#endif
    turn_cmd = mode23_limit_s16(turn_cmd, (int16)M23_TURN_MAX);

    // 速度指令斜坡：避免直道/弧线切换时的速度阶跃扰动
    if (s_m23_spd_cmd_now < (int16)(spd_cmd - (int16)M23_SPD_SLEW_PER_TICK))
    {
        s_m23_spd_cmd_now = (int16)(s_m23_spd_cmd_now + (int16)M23_SPD_SLEW_PER_TICK);
    }
    else if (s_m23_spd_cmd_now > (int16)(spd_cmd + (int16)M23_SPD_SLEW_PER_TICK))
    {
        s_m23_spd_cmd_now = (int16)(s_m23_spd_cmd_now - (int16)M23_SPD_SLEW_PER_TICK);
    }
    else
    {
        s_m23_spd_cmd_now = spd_cmd;
    }

    target_speed_val = s_m23_spd_cmd_now;
target_turn_val  = turn_cmd;

    s_m23_gz_last = gz_corr;
}
// ================================== Mode4 说明 ==================================
// MODE4：直线录制/回放（录距离 + 回放直线保持）
//
// 录制：
//   - 关闭速度环（避免与人工操作叠加），保持直立
//   - 记录 D_total（counts）：从 A 推到 B 的总里程（可正可负）
//   - 记录 yaw_end（mdeg）：仅用于评估漂移，不参与回放控制
//
// 回放：
//   - 低速匀速跑到 D_total
//   - 到点后进入 STOPPING：清速度环状态一次 + 速度目标=0，等待车速稳定为0
//   - 航向保持：使用 yaw-rate hold（角速度 PI），不依赖 yaw 角度长期可靠
//
// yaw_dbg（mdeg）仍然积分用于观察漂移；积分启用 NMNI（速度小且gz小则不积）。
// ================================== Mode4 参数区 ==================================

// Flash 存储（双备份，每槽 512B）
#define M4_PAGE_SIZE_BYTES        (512UL)
#define M4_SLOT_SIZE_BYTES        (512UL)
#define M4_SLOT_PAGES             (1u)

// 存储地址（需确保 STC-ISP EEPROM 充足）
#define M4_STORE_SLOT0_ADDR       (0x0800UL)
#define M4_STORE_SLOT1_ADDR       (0x0A00UL)

#define M4_STORE_MAGIC            (0x4D34u)
#define M4_STORE_VERSION          (2u)

// payload: dist_total(int32) + yaw_end(int32) = 8 bytes
#define M4_PAYLOAD_BYTES          (8u)

// 标定/积分参数
#define M4_CAL_TICKS              (400u)  // 5ms*400=2000ms
#define M4_CAL_SHIFT              (4)  // 标定用缩放：u = mdps>>4
#define M4_CAL_WARMUP_SAMPLES     (30u)
#define M4_CAL_K2                 (16L)  // 4sigma^2
#define M4_CAL_VAR_MIN            (25L)
#define M4_CAL_MIN_TH2            (160L)

// gyro 换算：默认 1000dps 档 32.8LSB/(deg/s)
// mdps = lsb * 10000 / 328
#define M4_GYRO_LSB_PER_DPS_X10   (328L)
#define M4_GYRO_TO_MDPS_NUM       (10000L)
#define M4_TICK_DIV_5MS           (200L)  // 5ms = 1/200s

// 方向约定：gz_mdps>0=右转(CW) yaw_mdeg增大=右转 turn_cmd>0=右转
#define M4_GZ_SIGN                (1)  // 如 yaw_dbg 方向反了改为 -1
#define M4_TURN_SIGN              (1)  // 如转向反了改为 -1

// NMNI 与 bias 在线微调
#define M4_MOVE_SPEED_DB          (2)  // counts/20ms
#define M4_NMNI_GZ_DB_MDPS        (1200L)  // 1.2deg/s 以内认为可静止
#define M4_BIAS_UPDATE_LIM_MDPS   (3000L)
#define M4_BIAS_UPDATE_SHIFT      (6)  // 1/64 EMA

// 回放速度（可调）
// 说明：回放采用“全程匀速 + 末端轻微降速”，降低长距离场景下因量化或静摩擦造成的间歇运动。
#define M4_PLAY_SPEED_CMD         (8)  // counts/20ms（建议 5~10；过低容易卡滞）
#define M4_PLAY_END_SPEED_CMD     (5)  // counts/20ms（末端轻微降速，避免越过目标点）
#define M4_END_SLOW_COUNTS        (450L)  // counts：进入末端收速区（约 5~8cm，按实测换算）
#define M4_STOP_DIST_DB           (12L)  // counts：到点判定死区（约 0.1~0.3cm，预留余量用于抑制抖动）

// 防停滞：目标速度较低时，长距离运行可能因量化或静摩擦导致速度接近零
// 检测到“持续低速”后，短时间提高目标速度（用于起步与克服静摩擦，不影响到点精度）
#define M4_STALL_SPEED_DB         (2)   // counts/20ms：判定为接近静止
#define M4_STALL_DETECT_TICKS     (6u) // 12*20ms=240ms：连续低速判定
#define M4_STALL_BOOST_TICKS      (10u) // 10*20ms=200ms：增速持续时间
#define M4_STALL_BOOST_ADD        (5)   // counts/20ms：增速幅度
#define M4_STALL_DISABLE_REMAIN   (900L) // counts：距离终点较近时不启用增速（避免越过目标点）

// STOPPING：先关速度环若干帧 再开速度环刹停
#define M4_STOP_CLR_TICKS         (3u)  // 40ms
#define M4_STOP_HOLD_TICKS        (8u)  // 连续静止判定 6*20ms=120ms
#define M4_STOP_SPEED_DB          (1)  // counts/20ms

// yaw-rate hold（角速度 PI 直线保持）
// turn = -Kp_rate * gz_corr - Ki_rate * ∫gz_corr
// 同时叠加一个很弱的 yaw 角度 P（仅用于把方向“拉回去”，避免长期偏航）
#define M4_YAW_P_DIV_MDEG         (400L)    // 5deg(5000mdeg)->6
#define M4_YAW_ERR_DB_MDEG        (180L)    // 0.18deg：小死区抑制抖动
#define M4_RATE_P_DIV_MDPS        (450L)    // 越小越强：5deg/s(5000mdps)->7
#define M4_RATE_I_DIV_ACC         (15000L)  // 越小I越强：用来学trim
#define M4_RATE_I_MAX_CMD         (90)      // I项最大贡献（cmd单位）

#define M4_TURN_CMD_MAX           (360)
#define M4_TURN_LPF_NUM           (1)
#define M4_TURN_LPF_DEN           (2)

// UI 刷新
#define M4_UI_REFRESH_DIV         (20u)  // 5ms*20=100ms

typedef struct
{
    uint16 magic;
    uint16 version;
    uint16 length;
    uint16 seq;
    uint16 crc16;
    uint16 rsv;
} M4StoreHdr_t;

typedef enum
{
    M4_IDLE = 0,
    M4_CAL,
    M4_RECORDING,
    M4_REC_DONE,
    M4_PLAYING,
    M4_STOPPING,
    M4_DONE
} M4State_e;

static uint8     s_m4_active = 0u;
static M4State_e s_m4_state  = M4_IDLE;

// 录制/回放数据
static uint8  s_m4_have_data      = 0u;    // 是否有有效录制数据
static int32  s_m4_dist_total     = 0;     // counts，带符号
static int32  s_m4_yaw_end_mdeg   = 0;     // mdeg，仅展示

// 运行态
static int32  s_m4_pos_base       = 0;     // car_pos_cnt 起点
static int32  s_m4_dist_now       = 0;     // 当前录制距离
static int32  s_m4_dist_run_abs   = 0;     // 回放已走距离（abs）
static int32  s_m4_total_abs      = 0;     // abs(dist_total)

// yaw_dbg：mdeg，bias：mdps
static int32  s_m4_yaw_mdeg       = 0;     // 当前航向角（毫度）
static int32  s_m4_yaw_rem_mdps   = 0;     // 积分余量（mdps）
static int32  s_m4_bias_mdps      = 0;     // 陀螺零偏（mdps）
static int32  s_m4_gz_corr_mdps   = 0;     // 修正后角速度（mdps）
static uint8  s_m4_moving         = 0u;    // 运动判定标志

// yaw-rate I 积分（mdps 累加，20ms更新）
static int32  s_m4_rate_i_acc     = 0;     // I 积分累加器

// 航向输出低通
static int16  s_m4_turn_lp        = 0;     // 转向指令低通滤波值

// STOPPING 计数
static uint8  s_m4_stop_clr_cnt   = 0u;    // 速度环清除计数
static uint8  s_m4_stop_hold_cnt  = 0u;    // 静止保持计数

// 防卡滞：低速增速计数
static uint8  s_m4_stall_cnt      = 0u;    // 连续低速计数
static uint8  s_m4_boost_cnt      = 0u;    // 增速剩余计数

// UI
static uint8  s_m4_ui_dirty       = 0u;    // UI 需要全量重绘标志
static uint8  s_m4_ui_div         = 0u;    // UI 刷新分频计数

// CAL：2s 标定（均值/方差 + 异常剔除）
static uint16 s_m4_cal_cnt        = 0u;    // 标定剩余节拍数
static M4State_e s_m4_cal_next    = M4_IDLE; // 标定完成后进入的状态
static uint16 s_m4_cal_acc        = 0u;    // 已接受样本数
static int32  s_m4_cal_mean       = 0;     // Welford 在线均值
static int32  s_m4_cal_M2         = 0;     // Welford M2 统计量
static int32  s_m4_cal_var        = 0;     // 方差估计
static uint16 s_m4_cal_reject     = 0u;    // 被剔除样本数

// Flash slot
static uint16 s_m4_last_seq       = 0u;    // 最新有效序号
static uint8  s_m4_last_slot      = 0u;    // 最新有效槽号（0/1）

// 保存动作：从 ISR(20ms) 挪到主循环
static uint8  s_m4_need_save      = 0u;    // 需要保存标志
static int32  s_m4_save_dist      = 0;     // 待保存距离
static int32  s_m4_save_yaw_end   = 0;     // 待保存航向

// ================================== Mode4 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介      32 位绝对值
// 参数说明      x       有符号 32 位整数
// 返回参数      int32   |x|
// 使用示例      d = m4_abs32(dist);
// 备注信息      内部静态函数
//--------------------------------------------------------------------------------------------------
static int32 m4_abs32(int32 x)
{
    if (x < 0) return -x;
    return x;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      16 位对称限幅
// 参数说明      x       输入值
//              lim     限幅绝对值上界
// 返回参数      int16   限幅后的值
// 使用示例      cmd = m4_limit_s16(cmd, 200);
// 备注信息      返回值范围 [-lim, +lim]
//--------------------------------------------------------------------------------------------------
static int16 m4_limit_s16(int16 x, int16 lim)
{
    if (x > lim)  return lim;
    if (x < -lim) return (int16)(-lim);
    return x;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      OLED 显示有符号 32 位整数
// 参数说明      row     显示行号
//              col     起始列号
//              v       有符号 32 位整数
// 返回参数      void
// 使用示例      m4_show_s32(2, 3, s_m4_dist_now);
// 备注信息      自动处理负号，右端补空格
//--------------------------------------------------------------------------------------------------
static void m4_show_s32(uint8 row, uint8 col, int32 v)
{
    int32 a;

    a = v;
    if (a < 0)
    {
        OLED_ShowChar(row, col, '-');
        a = -a;
    }
    else
    {
        OLED_ShowChar(row, col, ' ');
    }

    OLED_ShowString(row, (uint8)(col + 1u), "           ");

    if (a == 0)
    {
        OLED_ShowChar(row, (uint8)(col + 1u), '0');
    }
    else
    {
        uint8 i;
        uint8 j;
        char buf[11];

        i = 0u;
        while ((a > 0) && (i < 10u))
        {
            buf[i++] = (char)('0' + (a % 10));
            a /= 10;
        }

        j = 0u;
        while (i > 0u)
        {
            i--;
            OLED_ShowChar(row, (uint8)(col + 1u + j), buf[i]);
            j++;
        }
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介      Mode4 OLED 全屏绘制
// 参数说明      void
// 返回参数      void
// 使用示例      m4_draw();
// 备注信息      根据当前 s_m4_state 绘制对应 UI 页面
//--------------------------------------------------------------------------------------------------
static void m4_draw(void)
{
    OLED_Clear();

    if (s_m4_state == M4_IDLE)
    {
        OLED_ShowString(1, 1, "M4 OK=REC BK=EX");
        OLED_ShowString(2, 1, "D:");
        if (s_m4_have_data) m4_show_s32(2, 3, s_m4_dist_total);
        else                m4_show_s32(2, 3, 0);
        OLED_ShowString(3, 1, "Yend:");
        m4_show_s32(3, 6, s_m4_yaw_end_mdeg);
        OLED_ShowString(4, 1, s_m4_have_data ? "DATA:YES" : "DATA:NO ");
        return;
    }

    if (s_m4_state == M4_CAL)
    {
        OLED_ShowString(1, 1, "M4 CAL 2.0s");
        OLED_ShowString(2, 1, "Keep still");
        OLED_ShowString(3, 1, "acc/rej:");
        m4_show_s32(3, 9, (int32)s_m4_cal_acc);
        OLED_ShowString(4, 1, "BK=HOME");
        return;
    }

    if (s_m4_state == M4_RECORDING)
    {
        OLED_ShowString(1, 1, "REC OK=STOP");
        OLED_ShowString(2, 1, "D:");
        m4_show_s32(2, 3, s_m4_dist_now);
        OLED_ShowString(3, 1, "Y:");
        m4_show_s32(3, 3, s_m4_yaw_mdeg);
        OLED_ShowString(4, 1, "BK=HOME");
        return;
    }

    if (s_m4_state == M4_REC_DONE)
    {
        OLED_ShowString(1, 1, "OK=PLAY BK=HM");
        OLED_ShowString(2, 1, "D:");
        m4_show_s32(2, 3, s_m4_dist_total);
        OLED_ShowString(3, 1, "Yend:");
        m4_show_s32(3, 6, s_m4_yaw_end_mdeg);
        OLED_ShowString(4, 1, "Move->A OK");
        return;
    }

    if (s_m4_state == M4_PLAYING)
    {
        OLED_ShowString(1, 1, "PLAY...");
        OLED_ShowString(2, 1, "d/t:");
        m4_show_s32(2, 6, s_m4_dist_run_abs);
        OLED_ShowString(3, 1, "Y:");
        m4_show_s32(3, 3, s_m4_yaw_mdeg);
        OLED_ShowString(4, 1, "BK=ABRT");
        return;
    }

    if (s_m4_state == M4_STOPPING)
    {
        OLED_ShowString(1, 1, "STOPPING...");
        OLED_ShowString(2, 1, "Hold still");
        OLED_ShowString(3, 1, "d/t:");
        m4_show_s32(3, 6, s_m4_dist_run_abs);
        OLED_ShowString(4, 1, "BK=ABRT");
        return;
    }

    // DONE：只显示结束，不再刷数据
    OLED_ShowString(1, 1, "DONE BK=HOME");
    OLED_ShowString(2, 1, "Stop & Balance");
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(4, 1, "                ");
}

//--------------------------------------------------------------------------------------------------
// 函数简介      重置标定统计量
// 参数说明      void
// 返回参数      void
// 使用示例      m4_cal_reset();
// 备注信息      在每次进入 CAL 状态前调用，清空 Welford 均值/方差
//--------------------------------------------------------------------------------------------------
static void m4_cal_reset(void)
{
    s_m4_cal_acc = 0u;
    s_m4_cal_mean = 0;
    s_m4_cal_M2 = 0;
    s_m4_cal_var = 0;
    s_m4_cal_reject = 0u;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      标定阶段尝试接收一个陀螺采样
// 参数说明      x_mdps  陀螺仪角速度（mdps）
// 返回参数      void
// 使用示例      m4_cal_try_add(gz_mdps);
// 备注信息      Welford 在线均值/方差 + 预热后异常剔除（方差驱动）
//--------------------------------------------------------------------------------------------------
static void m4_cal_try_add(int32 x_mdps)
{
    int32 x_u;
    int32 diff;
    int32 diff2;
    int32 th2;
    int32 var_eff;

    x_u = 0;
    diff = 0;
    diff2 = 0;
    th2 = 0;
    var_eff = 0;

    // mdps -> u：缩放避免 M2 溢出（保持符号）
    if (x_mdps >= 0)
    {
        x_u = (int32)(x_mdps >> (int32)M4_CAL_SHIFT);
    }
    else
    {
        x_u = (int32)(-((int32)((-x_mdps) >> (int32)M4_CAL_SHIFT)));
    }

    // 预热阶段：全收，先建立均值/方差
    if (s_m4_cal_acc < (uint16)M4_CAL_WARMUP_SAMPLES)
    {
        s_m4_cal_acc++;
        diff = (int32)(x_u - s_m4_cal_mean);
        s_m4_cal_mean += (int32)(diff / (int32)s_m4_cal_acc);
        diff = (int32)(x_u - s_m4_cal_mean);
        s_m4_cal_M2 += (int32)(diff * diff);
        return;
    }

    // 方差估计：var = M2/(n-1)
    if (s_m4_cal_acc > 1u)
    {
        s_m4_cal_var = (int32)(s_m4_cal_M2 / (int32)(s_m4_cal_acc - 1u));
    }
    else
    {
        s_m4_cal_var = (int32)M4_CAL_VAR_MIN;
    }

    var_eff = s_m4_cal_var;
    if (var_eff < (int32)M4_CAL_VAR_MIN) var_eff = (int32)M4_CAL_VAR_MIN;

    // 以当前均值/方差做异常剔除：diff^2 > K2*var + min_th2
    diff = (int32)(x_u - s_m4_cal_mean);
    diff2 = (int32)(diff * diff);
    th2 = (int32)((int32)M4_CAL_K2 * var_eff + (int32)M4_CAL_MIN_TH2);

    if (diff2 > th2)
    {
        s_m4_cal_reject++;
        return;
    }

    // 接收样本并更新
    s_m4_cal_acc++;
    diff = (int32)(x_u - s_m4_cal_mean);
    s_m4_cal_mean += (int32)(diff / (int32)s_m4_cal_acc);
    diff = (int32)(x_u - s_m4_cal_mean);
    s_m4_cal_M2 += (int32)(diff * diff);
}

//--------------------------------------------------------------------------------------------------
// 函数简介      退出 Mode4 回到主菜单
// 参数说明      void
// 返回参数      void
// 使用示例      m4_exit_to_home();
// 备注信息      清除活跃标志、停止运行、调用 Menu_Exit()
//--------------------------------------------------------------------------------------------------
static void m4_exit_to_home(void)
{
    s_m4_active = 0u;
    s_m4_state  = M4_IDLE;

    s_m4_need_save = 0u;

    run_flag = 0u;
    Menu_Exit();
}

// ================================== 对外 API 函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介      Mode4 初始化
// 参数说明      void
// 返回参数      void
// 使用示例      Mode4_Init();
// 备注信息      上电调用一次，加载 Flash 录制数据并复位全部状态
//--------------------------------------------------------------------------------------------------
void Mode4_Init(void)
{
    s_m4_active = 0u;
    s_m4_state  = M4_IDLE;

    s_m4_have_data    = 0u;
    s_m4_dist_total   = 0;
    s_m4_yaw_end_mdeg = 0;

    s_m4_pos_base     = 0;
    s_m4_dist_now     = 0;
    s_m4_dist_run_abs = 0;
    s_m4_total_abs    = 0;

    s_m4_yaw_mdeg     = 0;
    s_m4_yaw_rem_mdps = 0;
    s_m4_bias_mdps    = 0;
    s_m4_gz_corr_mdps = 0;
    s_m4_moving       = 0u;

    s_m4_rate_i_acc   = 0;
    s_m4_turn_lp      = 0;

    s_m4_stop_clr_cnt  = 0u;
    s_m4_stop_hold_cnt = 0u;

    s_m4_stall_cnt     = 0u;
    s_m4_boost_cnt     = 0u;

    s_m4_ui_dirty     = 0u;
    s_m4_ui_div       = 0u;

    s_m4_cal_cnt      = 0u;
    s_m4_cal_next     = M4_IDLE;
    m4_cal_reset();

    s_m4_need_save    = 0u;
    s_m4_save_dist    = 0;
    s_m4_save_yaw_end = 0;

    s_m4_last_seq     = 0u;
    s_m4_last_slot    = 0u;

    // 从参数区加载（避免额外 Flash 区域写入导致掉电重启风险）
    s_m4_dist_total   = g_sys_param.mode4_dist_total;
    s_m4_yaw_end_mdeg = g_sys_param.mode4_yaw_end_mdeg;

    if ((s_m4_dist_total > 200000L) || (s_m4_dist_total < -200000L))
    {
        s_m4_dist_total = 0;
    }

    if ((s_m4_yaw_end_mdeg > 360000L) || (s_m4_yaw_end_mdeg < -360000L))
    {
        s_m4_yaw_end_mdeg = 0;
    }

    s_m4_have_data = (s_m4_dist_total != 0) ? 1u : 0u;

}

//--------------------------------------------------------------------------------------------------
// 函数简介      进入 Mode4 界面
// 参数说明      void
// 返回参数      void
// 使用示例      Mode4_Enter();
// 备注信息      复位运行态、绘制初始 UI，由菜单调用
//--------------------------------------------------------------------------------------------------
void Mode4_Enter(void)
{
    s_m4_active = 1u;
    s_m4_state  = M4_IDLE;

    s_m4_pos_base     = 0;
    s_m4_dist_now     = 0;
    s_m4_dist_run_abs = 0;
    s_m4_total_abs    = m4_abs32(s_m4_dist_total);

    s_m4_yaw_mdeg     = 0;
    s_m4_yaw_rem_mdps = 0;
    s_m4_gz_corr_mdps = 0;
    s_m4_moving       = 0u;

    s_m4_rate_i_acc   = 0;
    s_m4_turn_lp      = 0;

    s_m4_stop_clr_cnt  = 0u;
    s_m4_stop_hold_cnt = 0u;

    s_m4_stall_cnt     = 0u;
    s_m4_boost_cnt     = 0u;

    s_m4_ui_dirty     = 1u;
    s_m4_ui_div       = 0u;

    s_m4_cal_cnt      = 0u;
    s_m4_cal_next     = M4_IDLE;
    m4_cal_reset();

    s_m4_need_save    = 0u;

    m4_draw();
}

//--------------------------------------------------------------------------------------------------
// 函数简介      查询 Mode4 是否活跃
// 参数说明      void
// 返回参数      uint8   1=活跃  0=非活跃
// 使用示例      if (Mode4_IsActive()) { ... }
// 备注信息      对外接口
//--------------------------------------------------------------------------------------------------
uint8 Mode4_IsActive(void)
{
    return s_m4_active;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      Mode4 5ms 固定时基任务
// 参数说明      void
// 返回参数      void
// 使用示例      Mode4_Tick_5ms();
// 备注信息      gyro_z 转 mdps、CAL 标定、NMNI yaw 积分、静止 bias 微调
//--------------------------------------------------------------------------------------------------
void Mode4_Tick_5ms(void)
{
    int16 gz_lsb;
    int32 gz_mdps;
    int32 d_mdps;
    int32 tmp;
    int32 q;
    int32 diff;
    int32 abs_gz;

    gz_lsb = 0;
    gz_mdps = 0;
    d_mdps = 0;
    tmp = 0;
    q = 0;
    diff = 0;
    abs_gz = 0;

    if (!s_m4_active)
    {
        return;
    }

    gz_lsb  = (int16)((int16)M4_GZ_SIGN * imu660ra_gyro_z);
    gz_mdps = (int32)gz_lsb * (int32)M4_GYRO_TO_MDPS_NUM / (int32)M4_GYRO_LSB_PER_DPS_X10;

    // CAL：方差驱动异常剔除
    if (s_m4_state == M4_CAL)
    {
        m4_cal_try_add(gz_mdps);

        if (s_m4_cal_cnt > 0u) s_m4_cal_cnt--;

        if (s_m4_cal_cnt == 0u)
        {
            s_m4_bias_mdps = (int32)(s_m4_cal_mean << (int32)M4_CAL_SHIFT);
            s_m4_yaw_mdeg  = 0;
            s_m4_yaw_rem_mdps = 0;
            s_m4_turn_lp   = 0;
            s_m4_rate_i_acc = 0;

            if (s_m4_cal_next == M4_RECORDING)
            {
                s_m4_pos_base = (int32)car_pos_cnt;
                s_m4_dist_now = 0;
                s_m4_yaw_mdeg = 0;
                s_m4_yaw_rem_mdps = 0;
                s_m4_moving = 0u;

                run_flag = 1u;
                s_m4_state = M4_RECORDING;
            }
            else if (s_m4_cal_next == M4_PLAYING)
            {
                s_m4_pos_base = (int32)car_pos_cnt;
                s_m4_dist_run_abs = 0;
                s_m4_total_abs = m4_abs32(s_m4_dist_total);

                s_m4_yaw_mdeg = 0;
                s_m4_yaw_rem_mdps = 0;
                s_m4_turn_lp = 0;
                s_m4_rate_i_acc = 0;
                s_m4_moving = 0u;

                s_m4_stop_clr_cnt = 0u;
                s_m4_stop_hold_cnt = 0u;

                run_flag = 1u;
                s_m4_state = M4_PLAYING;
            }
            else
            {
                s_m4_state = M4_IDLE;
            }

            s_m4_ui_dirty = 1u;
        }

        return;
    }

    // RUN：计算 gyro corrected
    s_m4_gz_corr_mdps = (int32)(gz_mdps - s_m4_bias_mdps);

    if ((s_m4_state == M4_RECORDING) || (s_m4_state == M4_PLAYING) || (s_m4_state == M4_STOPPING))
    {
        abs_gz = s_m4_gz_corr_mdps;
        if (abs_gz < 0) abs_gz = -abs_gz;

        // yaw_dbg（mdeg）积分策略：
        // 1) PLAYING：即使低速也需要 yaw 随转动累计，否则“拉回方向”的 P 项无效。
        // 2) RECORDING/STOPPING：保留 NMNI，静止时不积分并允许 bias 微调。
        if ((s_m4_state == M4_PLAYING) || (s_m4_moving != 0u) || (abs_gz > (int32)M4_NMNI_GZ_DB_MDPS))
        {
            d_mdps = s_m4_gz_corr_mdps;
            tmp = d_mdps + s_m4_yaw_rem_mdps;
            q = tmp / (int32)M4_TICK_DIV_5MS;
            s_m4_yaw_mdeg += q;
            s_m4_yaw_rem_mdps = tmp - q * (int32)M4_TICK_DIV_5MS;
        }
        else
        {
            // 静止时：允许很慢地微调 bias
            diff = (int32)(gz_mdps - s_m4_bias_mdps);
            if (diff < 0) diff = -diff;
            if (diff <= (int32)M4_BIAS_UPDATE_LIM_MDPS)
            {
                s_m4_bias_mdps += (int32)((gz_mdps - s_m4_bias_mdps) >> (int32)M4_BIAS_UPDATE_SHIFT);
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介      Mode4 主循环 5ms 任务
// 参数说明      void
// 返回参数      void
// 使用示例      Mode4_Task_5ms();
// 备注信息      UI 刷新、按键处理、Flash 保存（非 ISR）
//--------------------------------------------------------------------------------------------------
void Mode4_Task_5ms(void)
{
    uint8 ok;
    uint8 bk;

    if (!s_m4_active)
    {
        return;
    }

    ok = Key_Check(KEY_NAME_CONFIRM, KEY_SINGLE);
    bk = Key_Check(KEY_NAME_BACK, KEY_SINGLE);

    if (bk)
    {
        run_flag = 0u;
        m4_exit_to_home();
        return;
    }

    if (s_m4_need_save != 0u)
    {
        g_sys_param.mode4_dist_total   = s_m4_save_dist;
        g_sys_param.mode4_yaw_end_mdeg = s_m4_save_yaw_end;
        Param_Save();

        s_m4_dist_total   = s_m4_save_dist;
        s_m4_yaw_end_mdeg = s_m4_save_yaw_end;
        s_m4_have_data    = (s_m4_dist_total != 0) ? 1u : 0u;

        s_m4_need_save = 0u;
        s_m4_ui_dirty = 1u;
    }

    if (s_m4_ui_dirty)
    {
        s_m4_ui_dirty = 0u;
        m4_draw();
        return;
    }

    // DONE 不刷任何数据
    if (s_m4_state == M4_DONE)
    {
        return;
    }

    s_m4_ui_div++;
    if (s_m4_ui_div >= (uint8)M4_UI_REFRESH_DIV)
    {
        s_m4_ui_div = 0u;
        if (s_m4_state == M4_RECORDING)
        {
            OLED_ShowString(2, 1, "D:");
            m4_show_s32(2, 3, s_m4_dist_now);
            OLED_ShowString(3, 1, "Y:");
            m4_show_s32(3, 3, s_m4_yaw_mdeg);
        }
        else if (s_m4_state == M4_PLAYING)
        {
            OLED_ShowString(2, 1, "d/t:");
            m4_show_s32(2, 6, s_m4_dist_run_abs);
            OLED_ShowString(3, 1, "Y:");
            m4_show_s32(3, 3, s_m4_yaw_mdeg);
        }
        else if (s_m4_state == M4_STOPPING)
        {
            OLED_ShowString(3, 1, "d/t:");
            m4_show_s32(3, 6, s_m4_dist_run_abs);
        }
    }

    if (!ok)
    {
        return;
    }

    if (s_m4_state == M4_IDLE)
    {
        interrupt_global_disable();
        m4_cal_reset();
        s_m4_cal_cnt  = (uint16)M4_CAL_TICKS;
        s_m4_cal_next = M4_RECORDING;
        s_m4_state    = M4_CAL;
        s_m4_ui_dirty = 1u;
        interrupt_global_enable();
        return;
    }

    if (s_m4_state == M4_RECORDING)
    {
        run_flag = 0u;

        s_m4_dist_total   = (int32)((int32)car_pos_cnt - s_m4_pos_base);
        s_m4_dist_now     = s_m4_dist_total;
        s_m4_yaw_end_mdeg = s_m4_yaw_mdeg;
        s_m4_have_data    = (s_m4_dist_total != 0) ? 1u : 0u;

        if (s_m4_have_data)
        {
            s_m4_need_save    = 1u;
            s_m4_save_dist    = s_m4_dist_total;
            s_m4_save_yaw_end = s_m4_yaw_end_mdeg;
        }

        s_m4_state    = M4_REC_DONE;
        s_m4_ui_dirty = 1u;
        return;
    }

    if (s_m4_state == M4_REC_DONE)
    {
        if (!s_m4_have_data)
        {
            return;
        }

        interrupt_global_disable();
        run_flag = 0u;
        m4_cal_reset();
        s_m4_cal_cnt  = (uint16)M4_CAL_TICKS;
        s_m4_cal_next = M4_PLAYING;
        s_m4_state    = M4_CAL;
        s_m4_ui_dirty = 1u;
        interrupt_global_enable();
        return;
    }

    // STOPPING/DONE 状态下 OK 不处理
}

//--------------------------------------------------------------------------------------------------
// 函数简介      Mode4 20ms 控制任务
// 参数说明      spd_cmd_out             速度指令输出指针
//              turn_cmd_out            转向指令输出指针
//              speed_pid_enable_out    速度环使能输出指针
// 返回参数      void
// 使用示例      Mode4_Task_20ms(&spd, &turn, &en);
// 备注信息      录制/回放速度指令、yaw-rate 航向保持 PI、STOPPING 逻辑
//--------------------------------------------------------------------------------------------------
void Mode4_Task_20ms(int16 *spd_cmd_out, int16 *turn_cmd_out, uint8 *speed_pid_enable_out)
{
    int16 spd_cmd;
    int16 turn_cmd;
    int16 turn_p;
    int16 turn_i;
    int16 turn_yaw;
    int32 pos_now;
    int32 dist_signed;
    int32 dist_abs;
    int32 dist_left;
    int32 gz_corr;
    int32 abs_spd;
    int32 abs_gz;
    int32 i_lim;

    spd_cmd = 0;
    turn_cmd = 0;
    turn_p = 0;
    turn_i = 0;
    turn_yaw = 0;
    pos_now = 0;
    dist_signed = 0;
    dist_abs = 0;
    dist_left = 0;
    gz_corr = 0;
    abs_spd = 0;
    abs_gz = 0;
    i_lim = 0;

    if (!s_m4_active)
    {
        *spd_cmd_out = 0;
        *turn_cmd_out = 0;
        *speed_pid_enable_out = 1u;
        return;
    }

    // moving 判定：用于 NMNI / bias 微调
    if (car_speed <= (int16)M4_MOVE_SPEED_DB && car_speed >= (int16)(-M4_MOVE_SPEED_DB))
    {
        s_m4_moving = 0u;
    }
    else
    {
        s_m4_moving = 1u;
    }

    if (s_m4_state == M4_CAL)
    {
        *spd_cmd_out = 0;
        *turn_cmd_out = 0;
        *speed_pid_enable_out = 0u;
        return;
    }

    if (s_m4_state == M4_RECORDING)
    {
        *spd_cmd_out = 0;
        *turn_cmd_out = 0;
        *speed_pid_enable_out = 0u;

        pos_now = (int32)car_pos_cnt;
        dist_signed = (int32)(pos_now - s_m4_pos_base);
        s_m4_dist_now = dist_signed;
        return;
    }

    if (s_m4_state == M4_REC_DONE)
    {
        *spd_cmd_out = 0;
        *turn_cmd_out = 0;
        *speed_pid_enable_out = 0u;
        return;
    }

    // 计算距离（PLAYING/STOPPING 共用）
    pos_now = (int32)car_pos_cnt;
    dist_signed = (int32)(pos_now - s_m4_pos_base);
    dist_abs = m4_abs32(dist_signed);
    s_m4_dist_run_abs = dist_abs;

    s_m4_total_abs = m4_abs32(s_m4_dist_total);

    if ((s_m4_state == M4_PLAYING) || (s_m4_state == M4_STOPPING))
    {
        // yaw-rate hold：基于 gz_corr 做 PI，和速度无关
        gz_corr = s_m4_gz_corr_mdps;

        // yaw 角度弱 P：把方向“拉回去”（yaw_mdeg 在 PLAYING 低速下也会积分）
        if (M4_YAW_P_DIV_MDEG != 0L)
        {
            if ((s_m4_yaw_mdeg > (int32)M4_YAW_ERR_DB_MDEG) || (s_m4_yaw_mdeg < (int32)(-M4_YAW_ERR_DB_MDEG)))
            {
                turn_yaw = (int16)(-(s_m4_yaw_mdeg / (int32)M4_YAW_P_DIV_MDEG));
            }
            else
            {
                turn_yaw = 0;
            }
        }

        // P
        if (M4_RATE_P_DIV_MDPS != 0L)
        {
            turn_p = (int16)(-(gz_corr / (int32)M4_RATE_P_DIV_MDPS));
        }
        else
        {
            turn_p = 0;
        }

        // I：只在确实在运动时积累（避免静止偏置导致I发散）
        abs_spd = car_speed;
        if (abs_spd < 0) abs_spd = -abs_spd;
        abs_gz = gz_corr;
        if (abs_gz < 0) abs_gz = -abs_gz;

        if ((abs_spd > (int32)M4_STOP_SPEED_DB) || (abs_gz > (int32)M4_NMNI_GZ_DB_MDPS))
        {
            s_m4_rate_i_acc += gz_corr;
        }

        // I 限幅：turn_i 不超过 M4_RATE_I_MAX_CMD
        i_lim = (int32)M4_RATE_I_DIV_ACC * (int32)M4_RATE_I_MAX_CMD;
        if (i_lim < 1) i_lim = 1;
        if (s_m4_rate_i_acc > i_lim) s_m4_rate_i_acc = i_lim;
        if (s_m4_rate_i_acc < -i_lim) s_m4_rate_i_acc = -i_lim;

        if (M4_RATE_I_DIV_ACC != 0L)
        {
            turn_i = (int16)(-(s_m4_rate_i_acc / (int32)M4_RATE_I_DIV_ACC));
        }
        else
        {
            turn_i = 0;
        }

        turn_cmd = (int16)((int16)M4_TURN_SIGN * (turn_yaw + turn_p + turn_i));
        turn_cmd = m4_limit_s16(turn_cmd, (int16)M4_TURN_CMD_MAX);
        s_m4_turn_lp = (int16)(((int32)s_m4_turn_lp * (int32)M4_TURN_LPF_NUM + (int32)turn_cmd) / (int32)M4_TURN_LPF_DEN);

        *turn_cmd_out = s_m4_turn_lp;
    }

    if (s_m4_state == M4_PLAYING)
    {
        if (!s_m4_have_data)
        {
            s_m4_state = M4_DONE;
            s_m4_ui_dirty = 1u;
            *spd_cmd_out = 0;
            *turn_cmd_out = 0;
            *speed_pid_enable_out = 1u;
            return;
        }

        if (s_m4_total_abs <= 0)
        {
            s_m4_state = M4_DONE;
            s_m4_ui_dirty = 1u;
            *spd_cmd_out = 0;
            *turn_cmd_out = 0;
            *speed_pid_enable_out = 1u;
            return;
        }

        // 到点：进入 STOPPING（略留死区，避免量化抖动）
        if ((int32)(dist_abs + (int32)M4_STOP_DIST_DB) >= s_m4_total_abs)
        {
            s_m4_state = M4_STOPPING;
            s_m4_ui_dirty = 1u;
            s_m4_stop_clr_cnt = 0u;
            s_m4_stop_hold_cnt = 0u;
            s_m4_stall_cnt = 0u;
            s_m4_boost_cnt = 0u;
            s_m4_turn_lp = 0;
            // 清一次 I，避免到点后还在“学trim”
            s_m4_rate_i_acc = 0;

            *spd_cmd_out = 0;
            *turn_cmd_out = 0;
            *speed_pid_enable_out = 0u;
            return;
        }

        // 剩余距离（abs）
        dist_left = (int32)(s_m4_total_abs - dist_abs);

        // 末端轻微降速（避免越过目标点，同时保持“近似全程匀速”）
        if (dist_left <= (int32)M4_END_SLOW_COUNTS)
        {
            spd_cmd = (int16)M4_PLAY_END_SPEED_CMD;
        }
        else
        {
            spd_cmd = (int16)M4_PLAY_SPEED_CMD;
        }

        // 防停滞：若长距离运行中速度接近静止，短时增速以克服静摩擦（仅在远离终点时启用）
        abs_spd = (int32)car_speed;
        if (abs_spd < 0) abs_spd = -abs_spd;

        if (dist_left > (int32)M4_STALL_DISABLE_REMAIN)
        {
            if (abs_spd <= (int32)M4_STALL_SPEED_DB)
            {
                if (s_m4_stall_cnt < 255u) s_m4_stall_cnt++;
            }
            else
            {
                s_m4_stall_cnt = 0u;
            }

            if ((s_m4_stall_cnt >= (uint8)M4_STALL_DETECT_TICKS) && (s_m4_boost_cnt == 0u))
            {
                s_m4_boost_cnt = (uint8)M4_STALL_BOOST_TICKS;
                s_m4_stall_cnt = 0u;
            }

            if (s_m4_boost_cnt > 0u)
            {
                s_m4_boost_cnt--;
                spd_cmd = (int16)(spd_cmd + (int16)M4_STALL_BOOST_ADD);
            }
        }
        else
        {
            s_m4_stall_cnt = 0u;
            s_m4_boost_cnt = 0u;
        }

        // 方向：录制距离为负则回放也为负
        if (s_m4_dist_total < 0) spd_cmd = (int16)(-spd_cmd);

        *spd_cmd_out = spd_cmd;
        *speed_pid_enable_out = 1u;
        return;
    }

    if (s_m4_state == M4_STOPPING)
    {
        // 先关速度环若干帧，让 main.c 清 speed_cycle 状态；再开速度环把车速压到0
        if (s_m4_stop_clr_cnt < (uint8)M4_STOP_CLR_TICKS)
        {
            s_m4_stop_clr_cnt++;
            *spd_cmd_out = 0;
            *turn_cmd_out = 0;
            *speed_pid_enable_out = 0u;
            return;
        }

        *spd_cmd_out = 0;
        *speed_pid_enable_out = 1u;

        // 静止判定：车速连续小于阈值
        if (car_speed <= (int16)M4_STOP_SPEED_DB && car_speed >= (int16)(-M4_STOP_SPEED_DB))
        {
            if (s_m4_stop_hold_cnt < 255u) s_m4_stop_hold_cnt++;
        }
        else
        {
            s_m4_stop_hold_cnt = 0u;
        }

        if (s_m4_stop_hold_cnt >= (uint8)M4_STOP_HOLD_TICKS)
        {
            s_m4_state = M4_DONE;
            s_m4_ui_dirty = 1u;
            *spd_cmd_out = 0;
            *turn_cmd_out = 0;
            *speed_pid_enable_out = 1u;
            return;
        }

        *turn_cmd_out = 0;
        return;
    }

    if (s_m4_state == M4_DONE)
    {
        *spd_cmd_out = 0;
        *turn_cmd_out = 0;
        *speed_pid_enable_out = 1u;
        return;
    }

    *spd_cmd_out = 0;
    *turn_cmd_out = 0;
    *speed_pid_enable_out = 1u;
}
