#include "zf_common_headfile.h"

// Mode5：蓝牙遥控相关参数
#define BT_JOYSTICK_LOST_MS      (300u)               // joystick 掉线判定时间（ms）
#define MODE5_SPEED_CMD_DB       (2)                  // 松手边沿判定死区（counts/20ms）

// Mode2/3：单编码器速度环补偿
#define M23_SPEED_FB_TURN_COMP_DB   (20)              // turn_cmd 绝对值超过此门限才做补偿
#define M23_SPEED_FB_TURN_COMP_DIV  (14)              // 单编码器转弯测速补偿系数：speed_real = enc + turn/div
#define M23_SPEED_STALL_DB          (1)               // 认为几乎没走起来的速度阈值
#define M23_SPEED_CMD_CRAWL_DB      (3)               // 极低速段（如Mode3回正直线）
#define M23_SPEED_OUT_MIN_RUN       (220)             // 直/斜线最小前倾输出（speed_cycle.out）
#define M23_SPEED_OUT_MIN_ARC       (250)             // 弯道最小前倾输出（speed_cycle.out）
#define M23_SPEED_OUT_MIN_CRAWL_RUN (15)              // 极低速直/斜线最小前倾，避免回正段猛冲
#define M23_SPEED_OUT_MIN_CRAWL_ARC (80)             // 极低速弯道最小前倾

volatile uint32 g_ms_tick = 0u;                       // 1ms 系统时基计数（蓝牙/掉线保护用）

extern volatile uint8 g_speed_pid_reset_req;             // Mode2/3 段切换请求清速度环

static int16 s_spd_cmd  = 0;                          // 20ms 速度环目标
static int16 s_turn_cmd = 0;                          // 20ms 转向命令

static uint8 s_run_prev      = 0u;                    // run_flag 上升沿检测
static uint8 s_m23_started   = 0u;                    // Mode2/3 启动标志
static uint8 s_m23_last_mode = 0u;                    // Mode2/3 上次启动的模式号

static uint8 s_speed_pid_en  = 1u;                    // 速度环使能（Mode4 录制时关闭）
static uint8 s_m5_moving_prev = 0u;                   // Mode5 上一帧运动状态

//--------------------------------------------------------------------------------------------------
// 函数简介      清零速度环 PID 状态
// 参数说明      void
// 返回参数      void
// 使用示例      speed_pid_reset();
// 备注信息      运动→静止边沿 / run 上升沿 / 速度环禁用时调用，避免积分残留
//--------------------------------------------------------------------------------------------------
static void speed_pid_reset(void)
{
    balance_cascade.speed_cycle.i_value = 0.0f;
    balance_cascade.speed_cycle.p_value_last = 0.0f;
    balance_cascade.speed_cycle.incremental_data[0] = 0.0f;
    balance_cascade.speed_cycle.incremental_data[1] = 0.0f;
    balance_cascade.speed_cycle.out = 0.0f;
}

static void pit_1ms_isr(void)
{
    static uint32 tick_count = 0u;

    /* 20ms 用变量 */
    uint8  mode_is_5;
    uint32 bt_age_ms;
	uint8  m5_moving;
    int16  speed_real;
    int16  abs_turn;
    int16  turn_comp;

    tick_count++;
    g_ms_tick++;

    /* 1ms：角速度环 */
    imu660ra_get_gyro();
    imu660ra_correct_gyro_offset(1, 0, 3, 10);

    Motor_Encoder_Tick_1ms();

    pid_control_d_lead(&balance_cascade.angular_speed_cycle,
                       balance_cascade.angle_cycle.out,
                       -*balance_cascade.cascade_value.gyro_raw_data);

    dynamic_motor_control();

    /* 5ms：角度环 + 模式事件 */
    if ((tick_count % 5u) == 0u)
    {
        imu660ra_get_acc();

        first_order_complementary_filtering(&balance_cascade.cascade_value,
                                            *balance_cascade.cascade_value.gyro_raw_data,
                                            *balance_cascade.cascade_value.acc_raw_data);

        pid_control_angle_gyro_d(&balance_cascade.angle_cycle,
                                 balance_cascade.speed_cycle.out,
                                 -balance_cascade.cascade_value.filtering_angle,
                                 imu660ra_gyro_y,
                                 balance_cascade.cascade_value.gyro_ration,
                                 balance_cascade.cascade_value.call_cycle);

        /* Mode2/3：循迹采样放在"已发车(run_flag=1)"后再跑，避免仅切菜单模式就触发额外负载/异常 */
        if ((g_sys_param.start_mode == 2u) || (g_sys_param.start_mode == 3u))
        {
            // 已发车才执行循迹采样与蜂鸣器任务
            if (run_flag == 1u)
            {
                Line_Task_5ms();
                Buzzer_Task();
            }

        }

        if (g_sys_param.start_mode == 4u)
        {
            Mode4_Tick_5ms();
        }

        Key_Tick();
    }

    /* 20ms：速度环 + 模式任务 */
    if ((tick_count % 20u) == 0u)
    {
        mode_is_5 = (g_sys_param.start_mode == 5u) ? 1u : 0u;

        /* 默认允许速度环 */
        s_speed_pid_en = 1u;

        // Mode2/3 启动/停止管理
        // run_flag=1 且 start_mode=2/3：若未启动过或模式变化 -> 启动
        // run_flag=0 或 start_mode!=2/3：若曾启动过 -> 停止
        if ((g_sys_param.start_mode == 2u) || (g_sys_param.start_mode == 3u))
        {
            if (run_flag == 1u)
            {
                if ((s_m23_started == 0u) || (s_m23_last_mode != g_sys_param.start_mode))
                {
                    Mode_Start(g_sys_param.start_mode);
                    s_m23_started = 1u;
                    s_m23_last_mode = g_sys_param.start_mode;
                }
            }
            else
            {
                if (s_m23_started != 0u)
                {
                    Mode_Stop();
                    s_m23_started = 0u;
                }
            }
        }
        else
        {
            if (s_m23_started != 0u)
            {
                Mode_Stop();
                s_m23_started = 0u;
            }
        }

        // 生成 s_spd_cmd / s_turn_cmd
        if (mode_is_5)
        {
            bt_age_ms = (uint32)(g_ms_tick - bt_last_joy_ms);

            // 掉线保护：不直接断电机（会摔） 而是让目标回零
            if ((bt_last_joy_ms == 0u) || (bt_age_ms > (uint32)BT_JOYSTICK_LOST_MS))
            {
                s_spd_cmd  = 0;
                s_turn_cmd = 0;
            }
            else
            {
                s_spd_cmd  = remote_speed;
                s_turn_cmd = remote_turn;
            }
        }
        else if (g_sys_param.start_mode == 4u)
        {
            Mode4_Task_20ms(&s_spd_cmd, &s_turn_cmd, &s_speed_pid_en);
        }
        else if ((g_sys_param.start_mode == 2u) || (g_sys_param.start_mode == 3u))
        {
            Mode_Task_20ms();
            s_spd_cmd  = target_speed_val;
            s_turn_cmd = target_turn_val;
        }
        else
        {
            s_spd_cmd  = 0;
            s_turn_cmd = 0;
        }

        // Mode5：处理“速度 PI 的位移记忆”
        // 松手进入静止时清速度环积分 让当前位置成为新停住点
        if (mode_is_5)
        {
            if ((s_spd_cmd <= (int16)MODE5_SPEED_CMD_DB) && (s_spd_cmd >= (int16)(-MODE5_SPEED_CMD_DB)))
            {
                m5_moving = 0u;
            }
            else
            {
                m5_moving = 1u;
            }

            // 运动 -> 静止 边沿：清速度环状态
            if ((m5_moving == 0u) && (s_m5_moving_prev == 1u))
            {
                speed_pid_reset();
            }

            s_m5_moving_prev = m5_moving;
        }
        else
        {
            s_m5_moving_prev = 0u;
        }
        // run 上升沿清速度环积分 避免停->跑瞬间冲一下
        if ((run_flag == 1u) && (s_run_prev == 0u))
        {
            speed_pid_reset();
        }
        s_run_prev = run_flag;

        // 速度环（可被 Mode4 录制阶段关闭）
        if (s_speed_pid_en)
        {
        if (g_speed_pid_reset_req != 0u)
        {
            speed_pid_reset();
            g_speed_pid_reset_req = 0u;
        }

            speed_real = left_motor_encoder_data;
            abs_turn = s_turn_cmd;
            turn_comp = 0;
            if (abs_turn < 0)
            {
                abs_turn = (int16)(-abs_turn);
            }

            if (((g_sys_param.start_mode == 2u) || (g_sys_param.start_mode == 3u)) &&
                (abs_turn >= (int16)M23_SPEED_FB_TURN_COMP_DB))
            {
                turn_comp = (int16)(s_turn_cmd / (int16)M23_SPEED_FB_TURN_COMP_DIV);
                speed_real = (int16)(speed_real + turn_comp);
            }

            pid_control(&balance_cascade.speed_cycle, s_spd_cmd, speed_real);

            if (((g_sys_param.start_mode == 2u) || (g_sys_param.start_mode == 3u)) && (run_flag == 1u))
            {
                if ((s_spd_cmd > 0) && (speed_real <= (int16)M23_SPEED_STALL_DB))
                {
                    if (abs_turn >= (int16)M23_SPEED_FB_TURN_COMP_DB)
                    {
                        if (s_spd_cmd <= (int16)M23_SPEED_CMD_CRAWL_DB)
                        {
                            if (balance_cascade.speed_cycle.out < (float)M23_SPEED_OUT_MIN_CRAWL_ARC)
                            {
                                balance_cascade.speed_cycle.out = (float)M23_SPEED_OUT_MIN_CRAWL_ARC;
                            }
                        }
                        else
                        {
                            if (balance_cascade.speed_cycle.out < (float)M23_SPEED_OUT_MIN_ARC)
                            {
                                balance_cascade.speed_cycle.out = (float)M23_SPEED_OUT_MIN_ARC;
                            }
                        }
                    }
                    else
                    {
                        if (s_spd_cmd <= (int16)M23_SPEED_CMD_CRAWL_DB)
                        {
                            if (balance_cascade.speed_cycle.out < (float)M23_SPEED_OUT_MIN_CRAWL_RUN)
                            {
                                balance_cascade.speed_cycle.out = (float)M23_SPEED_OUT_MIN_CRAWL_RUN;
                            }
                        }
                        else
                        {
                            if (balance_cascade.speed_cycle.out < (float)M23_SPEED_OUT_MIN_RUN)
                            {
                                balance_cascade.speed_cycle.out = (float)M23_SPEED_OUT_MIN_RUN;
                            }
                        }
                    }
                }
            }
        }
        else
        {
            speed_pid_reset();
        }

        // 转向：开环差动（Motor_Turn_Tick_20ms 内部限幅）
        Motor_Turn_Tick_20ms(s_turn_cmd);
    }
}

void main(void)
{
    clock_init(SYSTEM_CLOCK_35M);

    LED_Init();
	Buzzer_Init();
    Key_Init();
    Motor_Init();
    OLED_Init();
    Line_Init();

    if (imu660ra_init())
    {
        while (1)
        {
            system_delay_ms(200);
            LED_ON();
        }
    }

    balance_cascade_init();
    Param_Init();
    Param_ApplyToBalanceCascade();

    Menu_Init();
    Mode_Init();
    Mode4_Init();

    bluetooth_init();

    tim1_irq_handler = pit_1ms_isr;
    pit_ms_init(TIM1_PIT, 1);

    interrupt_global_enable();

    while (1)
    {
        bluetooth_parse_task();
        Menu_Task();
    }
}
