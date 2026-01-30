#include "motor.h"
#include "filter.h" // 引用 filter.h 以使用 limit_f 函数

// 死区补偿值 (300 对应 10000 的满量程，约 3%)
// 如果电机有轻微嗡嗡声但不转，可以适当减小
#define MOTOR_DEAD_ZONE  300

// 软件编码器变量
volatile int32 soft_enc_r_count = 0;
static uint8 soft_enc_r_last = 0;

void Motor_Init(void)
{
    // 1. 初始化 PWM (P6.0, P6.2, P6.4, P6.6)
    // 初始占空比设为 0
    pwm_init(MOTOR_L_IN1_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_L_IN2_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_R_IN1_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_R_IN2_PWM, PWM_FREQ, 0);

    // 2. 左编码器 (硬件 Timer1)
    // 注意：P3.4/P3.5 必须对应 Timer1 的硬件引脚
    encoder_dir_init(ENCODER_L_TIM, ENCODER_L_A, ENCODER_L_B);

    // 3. 右编码器 (软件 GPIO)
    // 设置为浮空输入或上拉输入 (视硬件电路而定，GPI_PULL_UP 更通用)
    gpio_init(ENCODER_R_A_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(ENCODER_R_B_PIN, GPI, 0, GPI_PULL_UP);

    // 初始化软件编码器状态
    soft_enc_r_last = (gpio_get_level(ENCODER_R_A_PIN) << 1) | gpio_get_level(ENCODER_R_B_PIN);
}

// 软件编码器扫描 (必须在 100us 定时器中断中调用)
void Soft_Encoder_Scan(void)
{
    uint8 curr = (gpio_get_level(ENCODER_R_A_PIN) << 1) | gpio_get_level(ENCODER_R_B_PIN);

    if (curr != soft_enc_r_last) 
    {
        // 简单的状态机判断方向
        // 00 -> 01 (正转)
        // 01 -> 11 (正转)
        // ...
        if ((soft_enc_r_last == 0 && curr == 1) || (soft_enc_r_last == 1 && curr == 3) || 
            (soft_enc_r_last == 3 && curr == 2) || (soft_enc_r_last == 2 && curr == 0)) 
            soft_enc_r_count++;
        else 
            soft_enc_r_count--;
        soft_enc_r_last = curr;
    }
}

// 获取编码器速度值 (并清零计数值)
// 周期性调用 (如 5ms 一次)
void Encoder_Get_Val(int16 *L, int16 *R)
{
    // 读取硬件编码器
    *L = encoder_get_count(ENCODER_L_TIM);
    encoder_clear_count(ENCODER_L_TIM);

    // 读取软件编码器
    // 必须强制转换类型，因为 soft_enc_r_count 是 int32
    *R = (int16)soft_enc_r_count;
    soft_enc_r_count = 0;

    // === 极性调整 ===
    // 如果车子向前推，读数是负数，就需要在这里加负号
    // *L = -(*L); 
    // *R = -(*R);
}

// 左电机输出
void Motor_Set_L(int16 duty)
{
    // 死区补偿
    if (duty > 0) duty += MOTOR_DEAD_ZONE;
    else if (duty < 0) duty -= MOTOR_DEAD_ZONE;

    // 限幅 (使用 filter.h 中的 limit_f)
    duty = (int16)limit_f((float)duty, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);

    // 输出 PWM
    if (duty >= 0)
    {
        pwm_set_duty(MOTOR_L_IN1_PWM, duty);
        pwm_set_duty(MOTOR_L_IN2_PWM, 0);
    }
    else
    {
        pwm_set_duty(MOTOR_L_IN1_PWM, 0);
        pwm_set_duty(MOTOR_L_IN2_PWM, -duty); // 取正数
    }
}

// 右电机输出
void Motor_Set_R(int16 duty)
{
    // 死区补偿
    if (duty > 0) duty += MOTOR_DEAD_ZONE;
    else if (duty < 0) duty -= MOTOR_DEAD_ZONE;
    
    // 限幅
    duty = (int16)limit_f((float)duty, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);

    // 输出 PWM
    if (duty >= 0)
    {
        pwm_set_duty(MOTOR_R_IN1_PWM, duty);
        pwm_set_duty(MOTOR_R_IN2_PWM, 0);
    }
    else
    {
        pwm_set_duty(MOTOR_R_IN1_PWM, 0);
        pwm_set_duty(MOTOR_R_IN2_PWM, -duty);
    }
}
