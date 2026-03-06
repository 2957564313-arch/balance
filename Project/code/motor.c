#include "motor.h"

// ================================== 对外变量定义 ==================================
volatile uint8 run_flag = 0;                          // 电机运行使能标志 1=输出 0=停止
volatile int16 car_speed = 0;                         // 当前速度（counts/20ms）
volatile int32 car_pos_cnt = 0;                       // 累计位置计数（counts）
volatile short int left_motor_encoder_data = 0;       // 左轮 20ms 增量
volatile short int right_motor_encoder_data = 0;      // 兼容旧逻辑 镜像=left

volatile int16 motor_turn_out = 0;                    // 转向差动输出

int16 left_motor_duty = 0;                            // 左电机占空比命令
int16 right_motor_duty = 0;                           // 右电机占空比命令

// ================================== 内部状态变量 ==================================
// 左轮：PWMB 硬件正交解码
static int16 s_pwmb_last_cnt = 0;                    // PWMB 计数器上一次读数（16bit）

// 20ms 窗口累加
static int16 s_enc_l_sum20 = 0;
static uint8 s_tick_20ms   = 0;

// 转向环：左右轮速度差闭环（保留）
#define TURN_CMD_TO_DIFFSPEED      (4.0f)
#define TURN_CMD_DEADZONE          (3)
#define TURN_OUT_MAX_PWM           (3500.0f)
static pid_cycle_struct s_turn_cycle;
static int16 s_dif_filt20 = 0;

// 转向环：PPDD + gyro_z 抑振（保留）
static float s_turn_e_last = 0.0f;

// ================================== 内部函数声明 ==================================
static int16 m_abs_i16(int16 x);
static int16 m_limit_i16(int16 x, int16 lo, int16 hi);
static void m_pid_reset_one(pid_cycle_struct *p);
static void m_balance_pid_reset_all(void);
static void m_tb6612_set_left(int16 duty_signed);
static void m_tb6612_set_right(int16 duty_signed);
static void m_disable_portint_all(void);
static void m_pwmb_encoder_init(void);
static int16 m_pwmb_encoder_get_count(void);
static void m_turn_reset(void);
static void m_turn_init(void);

// ================================== 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        计算 int16 绝对值
// 参数说明        x                输入值
// 返回参数        int16            绝对值结果
// 使用示例        int16 y = m_abs_i16(x);
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static int16 m_abs_i16(int16 x)
{
    if (x >= 0) return x;
    return (int16)(-x);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        int16 限幅
// 参数说明        x                待限幅值
// 参数说明        lo               下限
// 参数说明        hi               上限
// 返回参数        int16            限幅后的结果
// 使用示例        x = m_limit_i16(x, -100, 100);
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static int16 m_limit_i16(int16 x, int16 lo, int16 hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        清零单个 PID 环状态
// 参数说明        p                PID 结构体指针
// 返回参数        void
// 使用示例        m_pid_reset_one(&balance_cascade.speed_cycle);
// 备注信息        内部静态函数，仅清状态量，不修改参数
//--------------------------------------------------------------------------------------------------
static void m_pid_reset_one(pid_cycle_struct *p)
{
    p->i_value = 0.0f;
    p->p_value_last = 0.0f;
    p->incremental_data[0] = 0.0f;
    p->incremental_data[1] = 0.0f;
    p->out = 0.0f;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        清零平衡级联中的全部 PID 环状态
// 参数说明        void
// 返回参数        void
// 使用示例        m_balance_pid_reset_all();
// 备注信息        内部静态函数，停车或复位时调用
//--------------------------------------------------------------------------------------------------
static void m_balance_pid_reset_all(void)
{
    m_pid_reset_one(&balance_cascade.angular_speed_cycle);
    m_pid_reset_one(&balance_cascade.angle_cycle);
    m_pid_reset_one(&balance_cascade.speed_cycle);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        设置左电机 TB6612 输出
// 参数说明        duty_signed      带符号占空比命令
// 返回参数        void
// 使用示例        m_tb6612_set_left(duty);
// 备注信息        内部静态函数，包含方向映射与短刹车逻辑
//--------------------------------------------------------------------------------------------------
static void m_tb6612_set_left(int16 duty_signed)
{
    uint16 duty_u;
    int16 u;

    u = (int16)(duty_signed * MOTOR_L_DIR);
    if (u > 0)
    {
        duty_u = (uint16)m_abs_i16(u);
        if (duty_u > (uint16)MOTOR_MAX_DUTY) duty_u = (uint16)MOTOR_MAX_DUTY;

        gpio_high(MOTOR_L_IN1_IO);
        gpio_low (MOTOR_L_IN2_IO);
        pwm_set_duty(MOTOR_L_EN_PWM, (uint32)duty_u);
    }
    else if (u < 0)
    {
        duty_u = (uint16)m_abs_i16(u);
        if (duty_u > (uint16)MOTOR_MAX_DUTY) duty_u = (uint16)MOTOR_MAX_DUTY;

        gpio_low (MOTOR_L_IN1_IO);
        gpio_high(MOTOR_L_IN2_IO);
        pwm_set_duty(MOTOR_L_EN_PWM, (uint32)duty_u);
    }
    // 短刹车
    else
    {
        gpio_high(MOTOR_L_IN1_IO);
        gpio_high(MOTOR_L_IN2_IO);
        pwm_set_duty(MOTOR_L_EN_PWM, 0);
    }
    // 空转（coast）模式
//    else
//    {
//        // coast（IN1=IN2=L）会让轮子自由滑行
//        gpio_low(MOTOR_L_IN1_IO);
//        gpio_low(MOTOR_L_IN2_IO);
//        pwm_set_duty(MOTOR_L_EN_PWM, 0);
//    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        设置右电机 TB6612 输出
// 参数说明        duty_signed      带符号占空比命令
// 返回参数        void
// 使用示例        m_tb6612_set_right(duty);
// 备注信息        内部静态函数，包含方向映射与短刹车逻辑
//--------------------------------------------------------------------------------------------------
static void m_tb6612_set_right(int16 duty_signed)
{
    uint16 duty_u;
    int16 u;

    u = (int16)(duty_signed * MOTOR_R_DIR);
    if (u > 0)
    {
        duty_u = (uint16)m_abs_i16(u);
        if (duty_u > (uint16)MOTOR_MAX_DUTY) duty_u = (uint16)MOTOR_MAX_DUTY;

        gpio_high(MOTOR_R_IN1_IO);
        gpio_low (MOTOR_R_IN2_IO);
        pwm_set_duty(MOTOR_R_EN_PWM, (uint32)duty_u);
    }
    else if (u < 0)
    {
        duty_u = (uint16)m_abs_i16(u);
        if (duty_u > (uint16)MOTOR_MAX_DUTY) duty_u = (uint16)MOTOR_MAX_DUTY;

        gpio_low (MOTOR_R_IN1_IO);
        gpio_high(MOTOR_R_IN2_IO);
        pwm_set_duty(MOTOR_R_EN_PWM, (uint32)duty_u);
    }
    // 短刹车
    else
    {
        gpio_high(MOTOR_R_IN1_IO);
        gpio_high(MOTOR_R_IN2_IO);
        pwm_set_duty(MOTOR_R_EN_PWM, 0);
    }
    // 空转（coast）模式
//    else
//    {
//        gpio_low(MOTOR_R_IN1_IO);
//        gpio_low(MOTOR_R_IN2_IO);
//        pwm_set_duty(MOTOR_R_EN_PWM, 0);
//    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        禁用 P0/P3 端口中断
// 参数说明        void
// 返回参数        void
// 使用示例        m_disable_portint_all();
// 备注信息        内部静态函数，关闭软解码相关端口中断
//--------------------------------------------------------------------------------------------------
static void m_disable_portint_all(void)
{
    P_SW2 |= 0x80;

    // P0 端口中断全部关闭
    P0INTE = 0x00u;
    P0IM0  = 0x00u;
    P0IM1  = 0x00u;
    P0INTF = 0x00u;

    // P3 端口中断全部关闭
    P3INTE = 0x00u;
    P3IM0  = 0x00u;
    P3IM1  = 0x00u;
    P3INTF = 0x00u;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        初始化 PWMB 为硬件正交编码器模式
// 参数说明        void
// 返回参数        void
// 使用示例        m_pwmb_encoder_init();
// 备注信息        内部静态函数，使用 P2.0/P2.1 作为 A/B 相输入
//--------------------------------------------------------------------------------------------------
static void m_pwmb_encoder_init(void)
{
    // P2.0/P2.1 输入上拉（A/B 相）
    gpio_init(ENCODER_L_A_IO, GPI, 1, GPI_PULL_UP);
    gpio_init(ENCODER_L_B_IO, GPI, 1, GPI_PULL_UP);

    P_SW2 |= 0x80;

    // PWMB Encoder Mode 3
    PWMB_ENO   = 0x00u;
    PWMB_BKR   = 0x00u;

    // CH1/CH2 选择 PS0：P2.0/P2.1
    PWMB_PS   &= (uint8)(~0x0Fu);

    PWMB_PSCRH = 0x00u;
    PWMB_PSCRL = 0x00u;
    PWMB_ARRH  = 0xFFu;
    PWMB_ARRL  = 0xFFu;

    // 输入+滤波：CCxS=01 输入，滤波 4 个时钟
    PWMB_CCMR1 = 0x21u;
    PWMB_CCMR2 = 0x21u;

    // Encoder Mode 3
    PWMB_SMCR  = 0x03u;

    // 使能输入捕获通道并设置极性
    PWMB_CCER1 = 0x55u;

    PWMB_CNTRH = 0x00u;
    PWMB_CNTRL = 0x00u;

    PWMB_IER   = 0x00u;
    PWMB_SR1   = 0x00u;
    PWMB_SR2   = 0x00u;

    PWMB_EGR   = 0x01u;
    PWMB_CR1   = 0x01u;

    s_pwmb_last_cnt = 0;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        读取 PWMB 当前计数值
// 参数说明        void
// 返回参数        int16            当前编码器计数（16bit）
// 使用示例        int16 cnt = m_pwmb_encoder_get_count();
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static int16 m_pwmb_encoder_get_count(void)
{
    uint16 cnt;

    cnt = (uint16)PWMB_CNTRH << 8;
    cnt = (uint16)((uint8)PWMB_CNTRL) | cnt;

    return (int16)cnt;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        复位转向环状态
// 参数说明        void
// 返回参数        void
// 使用示例        m_turn_reset();
// 备注信息        内部静态函数，清零积分与输出状态
//--------------------------------------------------------------------------------------------------
static void m_turn_reset(void)
{
    s_turn_cycle.i_value = 0.0f;
    s_turn_cycle.p_value_last = 0.0f;
    s_turn_cycle.incremental_data[0] = 0.0f;
    s_turn_cycle.incremental_data[1] = 0.0f;
    s_turn_cycle.out = 0.0f;

    s_dif_filt20 = 0;
    motor_turn_out = 0;

    s_turn_e_last = 0.0f;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        初始化转向环默认参数与状态
// 参数说明        void
// 返回参数        void
// 使用示例        m_turn_init();
// 备注信息        内部静态函数
//--------------------------------------------------------------------------------------------------
static void m_turn_init(void)
{
    s_turn_cycle.p = 6.0f;
    s_turn_cycle.i = 0.3f;
    s_turn_cycle.d = 0.0f;

    s_turn_cycle.i_value = 0.0f;
    s_turn_cycle.i_value_max = 2000.0f;
    s_turn_cycle.i_value_pro = 0.01f;

    s_turn_cycle.p_value_last = 0.0f;
    s_turn_cycle.incremental_data[0] = 0.0f;
    s_turn_cycle.incremental_data[1] = 0.0f;

    s_turn_cycle.out = 0.0f;
    s_turn_cycle.out_max = TURN_OUT_MAX_PWM;

    m_turn_reset();
}

// ================================== 对外 API 函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介        电机模块初始化
// 参数说明        void
// 返回参数        void
// 使用示例        Motor_Init();
// 备注信息        初始化 PWM、方向引脚、编码器与转向环
//--------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    pwm_init(MOTOR_L_EN_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_R_EN_PWM, PWM_FREQ, 0);

    gpio_init(MOTOR_L_IN1_IO, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_L_IN2_IO, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_IN1_IO, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_IN2_IO, GPO, 0, GPO_PUSH_PULL);

    m_disable_portint_all();
    m_pwmb_encoder_init();

    s_pwmb_last_cnt = m_pwmb_encoder_get_count();

    s_enc_l_sum20 = 0;
    s_tick_20ms   = 0;

    left_motor_encoder_data  = 0;
    right_motor_encoder_data = 0;

    car_speed = 0;
    car_pos_cnt = 0;

    m_turn_init();

    Motor_Stop();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        电机停止并复位控制状态
// 参数说明        void
// 返回参数        void
// 使用示例        Motor_Stop();
// 备注信息        清零输出并复位 PID 与编码器窗口状态
//--------------------------------------------------------------------------------------------------
void Motor_Stop(void)
{
    pwm_set_duty(MOTOR_L_EN_PWM, 0);
    pwm_set_duty(MOTOR_R_EN_PWM, 0);

    gpio_low(MOTOR_L_IN1_IO);
    gpio_low(MOTOR_L_IN2_IO);
    gpio_low(MOTOR_R_IN1_IO);
    gpio_low(MOTOR_R_IN2_IO);

    m_turn_reset();
    m_balance_pid_reset_all();

    // 防止停机期间手动转轮导致下一次 delta 突增
    s_pwmb_last_cnt = m_pwmb_encoder_get_count();

    s_enc_l_sum20 = 0;
    s_tick_20ms   = 0;

    left_motor_encoder_data  = 0;
    right_motor_encoder_data = 0;

    car_speed = 0;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        设置左电机占空比命令
// 参数说明        duty             带符号占空比命令
// 返回参数        void
// 使用示例        Motor_Set_L(1000);
// 备注信息        对外接口，实际输出包含方向映射
//--------------------------------------------------------------------------------------------------
void Motor_Set_L(int16 duty)
{
    m_tb6612_set_left(duty);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        设置右电机占空比命令
// 参数说明        duty             带符号占空比命令
// 返回参数        void
// 使用示例        Motor_Set_R(1000);
// 备注信息        对外接口，实际输出包含方向映射
//--------------------------------------------------------------------------------------------------
void Motor_Set_R(int16 duty)
{
    m_tb6612_set_right(duty);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        20ms 转向命令映射（旧接口）
// 参数说明        turn_cmd_raw     原始转向命令
// 返回参数        void
// 使用示例        Motor_Turn_Tick_20ms(turn_cmd);
// 备注信息        对外接口，包含死区与限幅
//--------------------------------------------------------------------------------------------------
void Motor_Turn_Tick_20ms(int16 turn_cmd_raw)
{
    int16 cmd;
    int16 out;

    cmd = turn_cmd_raw;

    if ((cmd <= (int16)TURN_CMD_DEADZONE) && (cmd >= (int16)(-TURN_CMD_DEADZONE)))
    {
        cmd = 0;
    }

    // 经验映射：turn_cmd 量级通常为 0~200，缩放到 PWM 差动约 0~2000
    out = (int16)((int32)cmd * 10L);
    out = m_limit_i16(out, (int16)(-2000), (int16)(2000));

    motor_turn_out = out;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        对外复位转向环状态
// 参数说明        void
// 返回参数        void
// 使用示例        Motor_Turn_Reset();
// 备注信息        对外接口
//--------------------------------------------------------------------------------------------------
void Motor_Turn_Reset(void)
{
    m_turn_reset();
}

//--------------------------------------------------------------------------------------------------
// 函数简介        5ms 转向控制计算（新接口）
// 参数说明        turn_err         转向误差
// 参数说明        gyro_z           Z 轴角速度
// 返回参数        void
// 使用示例        Motor_Turn_Tick_5ms(err, gyro_z);
// 备注信息        对外接口，使用 turn_kp/kp2/kd/kd2 组合
//--------------------------------------------------------------------------------------------------
void Motor_Turn_Tick_5ms(int16 turn_err, int16 gyro_z)
{
    float e;
    float de;
    float ae;
    float out;
    int16 out_i;

    e = 0.0f;
    de = 0.0f;
    ae = 0.0f;
    out = 0.0f;
    out_i = 0;

    if (0u == run_flag)
    {
        m_turn_reset();
        return;
    }

    if (0 == turn_err)
    {
        motor_turn_out = 0;
        s_turn_e_last = 0.0f;
        return;
    }

    e = (float)turn_err;
    de = e - s_turn_e_last;
    s_turn_e_last = e;

    if (e >= 0.0f) ae = e;
    else           ae = -e;

    out = (g_sys_param.turn_kp  * e)
        + (g_sys_param.turn_kp2 * ae * e)
        + (g_sys_param.turn_kd  * de)
        + (g_sys_param.turn_kd2 * (float)gyro_z);

    if (out > TURN_OUT_MAX_PWM)  out = TURN_OUT_MAX_PWM;
    if (out < -TURN_OUT_MAX_PWM) out = -TURN_OUT_MAX_PWM;

    if (out >= 0.0f) out_i = (int16)(out + 0.5f);
    else             out_i = (int16)(out - 0.5f);

    motor_turn_out = out_i;
}

//--------------------------------------------------------------------------------------------------
// 函数简介        动态电机控制输出
// 参数说明        void
// 返回参数        void
// 使用示例        dynamic_motor_control();
// 备注信息        对外接口，融合平衡输出与转向输出并限幅
//--------------------------------------------------------------------------------------------------
void dynamic_motor_control(void)
{
    int16 u_balance;
    int16 u_turn;
    int16 u_l;
    int16 u_r;

    u_balance = 0;
    u_turn = 0;
    u_l = 0;
    u_r = 0;

    if (run_flag == 1)
    {
        u_balance = (int16)balance_cascade.angular_speed_cycle.out;
        u_turn    = motor_turn_out;

        u_l = (int16)(u_balance - u_turn);
        u_r = (int16)(u_balance + u_turn);

        left_motor_duty  = (int16)m_limit_i16(u_l, (int16)(-MOTOR_MAX_DUTY), (int16)(MOTOR_MAX_DUTY));
        right_motor_duty = (int16)m_limit_i16(u_r, (int16)(-MOTOR_MAX_DUTY), (int16)(MOTOR_MAX_DUTY));

        Motor_Set_L(left_motor_duty);
        Motor_Set_R(right_motor_duty);
    }
    else
    {
        Motor_Stop();
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介        1ms 编码器采样任务
// 参数说明        void
// 返回参数        void
// 使用示例        Motor_Encoder_Tick_1ms();
// 备注信息        对外接口，计算 20ms 速度增量与累计里程
//--------------------------------------------------------------------------------------------------
void Motor_Encoder_Tick_1ms(void)
{
    int16 now_cnt;
    int16 dl;

    now_cnt = 0;
    dl = 0;

    now_cnt = m_pwmb_encoder_get_count();
    dl = (int16)(now_cnt - s_pwmb_last_cnt);
    s_pwmb_last_cnt = now_cnt;

#if (MOTOR_ENC_L_INV != 0)
    dl = (int16)(-dl);
#endif

    s_enc_l_sum20 = (int16)(s_enc_l_sum20 + dl);

    // 单编码器里程：直接使用左轮
    car_pos_cnt += (int32)dl;

    s_tick_20ms++;
    if (s_tick_20ms >= 20u)
    {
        s_tick_20ms = 0u;

        left_motor_encoder_data = (short int)s_enc_l_sum20;
        right_motor_encoder_data = left_motor_encoder_data;

        car_speed = (int16)left_motor_encoder_data;

        s_enc_l_sum20 = 0;
    }
}
