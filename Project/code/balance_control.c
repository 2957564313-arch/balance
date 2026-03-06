#include "balance_control.h"

cascade_value_struct balance_cascade;
cascade_value_struct balance_cascade_resave;

cascade_value_struct steer_balance_cascade;
cascade_value_struct steer_balance_cascade_resave;

//--------------------------------------------------------------------------------------------------
// 函数简介        一阶互补滤波
// 参数说明        filter_value         滤波参数结构体
// 参数说明        gyro_raw_data        陀螺仪原始数据
// 参数说明        acc_raw_data         加速度原始数据
// 返回参数        void
// 使用示例        first_order_complementary_filtering(&balance_cascade.cascade_value, *balance_cascade.cascade_value.gyro_raw_data, *balance_cascade.cascade_value.acc_raw_data);
// 备注信息
//--------------------------------------------------------------------------------------------------
void first_order_complementary_filtering (cascade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data)
{
    float gyro_temp;           // 角速度计算临时变量
    float acc_temp;            // 加速度计算临时变量

    gyro_temp = gyro_raw_data * filter_value->gyro_ration;                                  // 角速度数据  * 角速度置信度(一般给4)

    acc_temp  = (acc_raw_data - filter_value->angle_temp) * filter_value->acc_ration;      // 加速度微分数据 * 加速度置信度(一般给4)

    filter_value->angle_temp += ((gyro_temp + acc_temp) * filter_value->call_cycle);       // 两数之和  * 调用周期  并积分到角度输出

    filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero; // 最终滤波角度域去零点位置即可
}

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（位置式）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值
// 返回参数        void
// 使用示例        pid_control(&balance_cascade.speed_cycle, 0, (left_motor.encoder_data + right_motor.encoder_data) / 2);
// 备注信息
//--------------------------------------------------------------------------------------------------
void pid_control (pid_cycle_struct *pid_cycle, float target, float real)
{
    float   proportion_value     = 0;    // 比例量
    float   differential_value   = 0;    // 微分量

    proportion_value = target - real;    // 比例量  = 目标值 - 实际值

    pid_cycle->i_value += (proportion_value * pid_cycle->i_value_pro);                     // 积分量 = 积分量 + 比例量 * 积分程度

    pid_cycle->i_value = func_limit_ab(pid_cycle->i_value, -pid_cycle->i_value_max, pid_cycle->i_value_max); // 积分量限幅

    differential_value = proportion_value - pid_cycle->p_value_last;                       // 微分量 = 比例量 - 上一次比例量

    pid_cycle->out = (pid_cycle->p * proportion_value + pid_cycle->i * pid_cycle->i_value + pid_cycle->d * differential_value); // PID融合

    pid_cycle->out = func_limit_ab(pid_cycle->out, -pid_cycle->out_max, pid_cycle->out_max); // PID输出限幅

    pid_cycle->p_value_last = proportion_value;                                            // 保存比例量
}

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（位置式，微分先行）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值(测量值)
// 返回参数        void
// 使用示例        pid_control_d_lead(&balance_cascade.angle_cycle, target_angle, current_angle);
// 备注信息        D项对测量值求导而非误差，避免目标值突变引起的微分冲击
//               D项 = -Kd * d(real)/dt，差分近似并做一阶低通滤波
//               incremental_data[0] 保存微分低通滤波状态
//               incremental_data[1] 保存初始化标志（0 未初始化 / 1 已初始化）
//               p_value_last 复用为上一次测量值
//--------------------------------------------------------------------------------------------------
void pid_control_d_lead (pid_cycle_struct *pid_cycle, float target, float real)
{
    float   proportion_value = 0;    // 比例量
    float   d_meas           = 0;    // 微分测量量（未滤波）
    float   d_filt           = 0;    // 微分测量量（低通滤波后）

    proportion_value = target - real;                                                       // 比例量 = 目标值 - 实际值

    pid_cycle->i_value += (proportion_value * pid_cycle->i_value_pro);                      // 积分量 = 积分量 + 比例量 * 积分程度

    pid_cycle->i_value = func_limit_ab(pid_cycle->i_value, -pid_cycle->i_value_max, pid_cycle->i_value_max); // 积分量限幅

    if (pid_cycle->incremental_data[1] == 0.0f)                                             // 首次调用初始化，避免D项突刺
    {
        pid_cycle->p_value_last = real;                                                     // 保存当前测量值作为上一次测量值
        pid_cycle->incremental_data[0] = 0.0f;                                              // 微分低通滤波状态清零
        pid_cycle->incremental_data[1] = 1.0f;                                              // 标记为已初始化
    }

    d_meas = real - pid_cycle->p_value_last;                                                // 微分测量量 = 当前测量值 - 上一次测量值

    d_filt = pid_cycle->incremental_data[0];                                                // 取出上一次滤波值
    d_filt += (d_meas - d_filt) * 0.25f;                                                    // 一阶低通滤波（alpha = 0.25）
    pid_cycle->incremental_data[0] = d_filt;                                                // 保存滤波值

    pid_cycle->out = (pid_cycle->p * proportion_value + pid_cycle->i * pid_cycle->i_value - pid_cycle->d * d_filt); // PID融合（D项取负号）

    pid_cycle->out = func_limit_ab(pid_cycle->out, -pid_cycle->out_max, pid_cycle->out_max); // PID输出限幅

    pid_cycle->p_value_last = real;                                                         // 保存当前测量值
}

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（位置式，微分先行，陀螺仪D项）
// 参数说明        pid_cycle            PID参数结构体  
// 参数说明        target               目标值
// 参数说明        real                 当前值(测量值)
// 参数说明        gyro_raw_for_d       用于D项计算的陀螺仪原始数据
// 参数说明        gyro_ration          角速度数据转换为角度增量的比例系数
// 参数说明        call_cycle           PID调用周期，单位秒
// 返回参数        void
// 使用示例        pid_control_angle_gyro_d(&balance_cascade.angle_cycle, target_angle, current_angle, *balance_cascade.cascade_value.gyro_raw_data, balance_cascade.cascade_value.gyro_ration, balance_cascade.cascade_value.call_cycle);
// 备注信息        D项使用陀螺仪数据估算的角度增量，避免测量值突变引起的微分冲击，并且对陀螺仪数据做小死区和轻滤波以抑制噪声
void pid_control_angle_gyro_d(pid_cycle_struct *pid_cycle,float target,float real,int16 gyro_raw_for_d,float gyro_ration,float call_cycle)
{
    float proportion_value;
    float d_inc_est;
    float d_filt;
    float gyro_d_raw;

    const float gyro_d_deadzone = 8.0f;   /* 原始计数死区：±1000dps量程先试 6~12 */
    const float d_lpf_alpha     = 0.30f;  /* 轻IIR：0.25~0.40 先试 */

    proportion_value = target - real;

    pid_cycle->i_value += (proportion_value * pid_cycle->i_value_pro);
    pid_cycle->i_value = func_limit_ab(pid_cycle->i_value,
                                       -pid_cycle->i_value_max,
                                       pid_cycle->i_value_max);

    /* 首次调用初始化，避免D项突刺 */
    if(pid_cycle->incremental_data[1] == 0.0f)
    {
        pid_cycle->incremental_data[0] = 0.0f;  /* D滤波状态 */
        pid_cycle->incremental_data[1] = 1.0f;  /* init flag */
    }

    /* 只在角度环D支路做小死区（不要改全局gyro） */
    gyro_d_raw = (float)gyro_raw_for_d;
    if((gyro_d_raw > -gyro_d_deadzone) && (gyro_d_raw < gyro_d_deadzone))
    {
        gyro_d_raw = 0.0f;
    }

    /* 用gyro估算本次(5ms)角度增量，单位尽量与“角度差分”接近 */
    d_inc_est = gyro_d_raw * gyro_ration * call_cycle;

    /* 轻IIR，仅作用于D支路 */
    d_filt = pid_cycle->incremental_data[0];
    d_filt += (d_inc_est - d_filt) * d_lpf_alpha;
    pid_cycle->incremental_data[0] = d_filt;

    /* P + I - D (D项取负抑振) */
    pid_cycle->out = (pid_cycle->p * proportion_value
                    + pid_cycle->i * pid_cycle->i_value
                    - pid_cycle->d * d_filt);

    pid_cycle->out = func_limit_ab(pid_cycle->out,
                                   -pid_cycle->out_max,
                                   pid_cycle->out_max);
}

//--------------------------------------------------------------------------------------------------
// 函数简介        PID闭环计算（增量式）
// 参数说明        pid_cycle            PID参数结构体
// 参数说明        target               目标值
// 参数说明        real                 当前值
// 返回参数        void
// 使用示例        pid_control_incremental(&balance_cascade.speed_cycle, 0, (left_motor.encoder_data + right_motor.encoder_data) / 2);
// 备注信息
//--------------------------------------------------------------------------------------------------
void pid_control_incremental (pid_cycle_struct *pid_cycle, float target, float real)
{
    float e;                             // 当前误差 e(k)
    float de;                            // 误差变化量 e(k) - e(k-1)
    float dde;                           // 误差二阶差分 e(k) - 2*e(k-1) + e(k-2)
    float du;                            // 增量输出

    e   = target - real;                                                                    // 当前误差 e(k)

    de  = e - pid_cycle->incremental_data[0];                                               // P项: e(k) - e(k-1)

    dde = e - 2.0f * pid_cycle->incremental_data[0] + pid_cycle->incremental_data[1];       // D项: e(k) - 2*e(k-1) + e(k-2)

    du = pid_cycle->p * de + pid_cycle->i * e + pid_cycle->d * dde;                         // PID融合

    pid_cycle->incremental_data[1] = pid_cycle->incremental_data[0];                        // 保存 e(k-2) = e(k-1)

    pid_cycle->incremental_data[0] = e;                                                     // 保存 e(k-1) = e(k)

    pid_cycle->out += (int16)du;                                                            // 增量累加到输出

    pid_cycle->out = func_limit_ab(pid_cycle->out, -pid_cycle->out_max, pid_cycle->out_max); // PID输出限幅
}

//--------------------------------------------------------------------------------------------------
// 函数简介        串级平衡算法初始化
// 返回参数        void
// 使用示例        balance_cascade_init();
// 备注信息        设置串级平衡参数
//--------------------------------------------------------------------------------------------------
void balance_cascade_init (void)
{
    balance_cascade.cascade_value.gyro_raw_data      = &imu660ra_gyro_y;    // 指向陀螺仪Y轴数据
    balance_cascade.cascade_value.acc_raw_data       = &imu660ra_acc_x;     // 指向加速度X轴数据
    balance_cascade.cascade_value.gyro_ration        = 8.0f;                // 互补滤波中的陀螺仪滤波信任程度，一般给4
    balance_cascade.cascade_value.acc_ration         = 1.6f;                // 互补滤波中的加速度滤波信任程度，一般给4
    balance_cascade.cascade_value.call_cycle         = 0.005f;              // 互补滤波调用周期，单位秒，一般给0.005（5ms）
    balance_cascade.cascade_value.mechanical_zero    = 0;		            //机械零点,前倒后倒值加起来除二，但是要反号,85

    balance_cascade.cascade_value.filtering_angle    = -balance_cascade.cascade_value.mechanical_zero;      // 滤波后的角度
    balance_cascade.cascade_value.angle_temp         = -balance_cascade.cascade_value.mechanical_zero;      // 互补滤波中保存上一次计算值

    balance_cascade.angular_speed_cycle.i_value_max  = 1000;                     // 角速度环积分最大值
    balance_cascade.angular_speed_cycle.i_value_pro  = 0.1f;                     // 角速度环积分程度
    balance_cascade.angular_speed_cycle.out_max      = 10000;                    // 角速度环输出最大值
    balance_cascade.angle_cycle.i_value_max          = 4000;                     // 角度环积分最大值
    balance_cascade.angle_cycle.i_value_pro          = 0.05f;                    // 角度环积分程度
    balance_cascade.angle_cycle.out_max              = 8000;                     // 角度环输出最大值 
    balance_cascade.speed_cycle.i_value_max          = 4000;                     // 速度环积分最大值
    balance_cascade.speed_cycle.i_value_pro          = 0.5f;                     // 速度环积分程度
    balance_cascade.speed_cycle.out_max              = 1500;                    // 速度环输出最大值1500

    balance_cascade.angular_speed_cycle.p = 1.0f;                     // 角速度环比例系数
    balance_cascade.angular_speed_cycle.i = 0.0f;                     // 角速度环积分系数
    balance_cascade.angular_speed_cycle.d = 0.0f;                     // 角速度环微分系数

    balance_cascade.angle_cycle.p = 6.5f;                     // 角度环比例系数
    balance_cascade.angle_cycle.i = 0.0f;                     // 角度环积分系数
    balance_cascade.angle_cycle.d = 20.0f;                     // 角度环微分系数

    balance_cascade.speed_cycle.p = 20.0f;                  // 速度环比例系数
    balance_cascade.speed_cycle.i = 0.236f;                  // 速度环积分系数
    balance_cascade.speed_cycle.d = 0.0f;                  // 速度环微分系数

    steer_balance_cascade.cascade_value.gyro_raw_data   = &imu660ra_gyro_y;   // 指向陀螺仪Y轴数据
    steer_balance_cascade.cascade_value.acc_raw_data    = &imu660ra_acc_x;    // 指向加速度X轴数据
    steer_balance_cascade.cascade_value.gyro_ration     = 0.0f;               // 互补滤波中的陀螺仪滤波信任程度，一般给4
    steer_balance_cascade.cascade_value.acc_ration      = 0.0f;               // 互补滤波中的加速度滤波信任程度，一般给4
    steer_balance_cascade.cascade_value.call_cycle      = 0.0f;               // 互补滤波调用周期，单位秒，一般给0.005（5ms）
    steer_balance_cascade.cascade_value.mechanical_zero = 0;                  //机械零点

    steer_balance_cascade.cascade_value.filtering_angle = -steer_balance_cascade.cascade_value.mechanical_zero;     // 滤波后的角度
    steer_balance_cascade.cascade_value.angle_temp      = -steer_balance_cascade.cascade_value.mechanical_zero;     // 互补滤波中保存上一次计算值

    steer_balance_cascade.angular_speed_cycle.i_value_max = 1000;                     // 角速度环积分最大值
    steer_balance_cascade.angular_speed_cycle.i_value_pro = 0.1f;                     // 角速度环积分程度
    steer_balance_cascade.angular_speed_cycle.out_max     = 10000;                    // 角速度环输出最大值
    steer_balance_cascade.angle_cycle.i_value_max         = 4000;                     // 角度环积分最大值
    steer_balance_cascade.angle_cycle.i_value_pro         = 0.05f;                    // 角度环积分程度
    steer_balance_cascade.angle_cycle.out_max             = 8000;                     // 角度环输出最大值
    steer_balance_cascade.speed_cycle.i_value_max         = 4000;                     // 速度环积分最大值
    steer_balance_cascade.speed_cycle.i_value_pro         = 0.05f;                    // 速度环积分程度
    steer_balance_cascade.speed_cycle.out_max             = 1500;                     // 速度环输出最大值

    steer_balance_cascade.angular_speed_cycle.p = 0.0f;
    steer_balance_cascade.angular_speed_cycle.i = 0.0f;
    steer_balance_cascade.angular_speed_cycle.d = 0.0f;

    steer_balance_cascade.angle_cycle.p = 0.0f;
    steer_balance_cascade.angle_cycle.i = 0.0f;
    steer_balance_cascade.angle_cycle.d = 0.0f;

    steer_balance_cascade.speed_cycle.p = 0.0f;
    steer_balance_cascade.speed_cycle.i = 0.0f;
    steer_balance_cascade.speed_cycle.d = 0.0f;

    memcpy(&balance_cascade_resave, &balance_cascade, sizeof(balance_cascade_resave));
    memcpy(&steer_balance_cascade_resave, &steer_balance_cascade, sizeof(steer_balance_cascade_resave));
}
