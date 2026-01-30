#include "motor.h"
#include "filter.h"

// ========================= 可开关：现在没装编码器就先关掉 =========================
#define MOTOR_ENCODER_ENABLE   (0)

// 软件编码器变量（右轮）
volatile int32 soft_enc_r_count = 0;
static uint8 soft_enc_r_last = 0;

static int16 motor_clip_i16(int16 v, int16 lo, int16 hi)
{
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

// ======= 新增：停机（不给你改 motor.h，也能被 main 通过 extern 调用） =======
void Motor_Stop(void)
{
    pwm_set_duty(MOTOR_L_IN1_PWM, 0);
    pwm_set_duty(MOTOR_L_IN2_PWM, 0);
    pwm_set_duty(MOTOR_R_IN1_PWM, 0);
    pwm_set_duty(MOTOR_R_IN2_PWM, 0);
}

void Motor_Init(void)
{
    // 1) PWM 初始化（四路）
    pwm_init(MOTOR_L_IN1_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_L_IN2_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_R_IN1_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_R_IN2_PWM, PWM_FREQ, 0);

    Motor_Stop();

#if MOTOR_ENCODER_ENABLE
    // ?? 先不启用：你当前工程的 encoder_dir_init 传参很可能触发 zf_assert 卡死
    encoder_dir_init(TIM1_ENCOEDER, ENCODER_L_A, TIM1_ENCOEDER_P35);

    gpio_init(ENCODER_R_A_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(ENCODER_R_B_PIN, GPI, 0, GPI_PULL_UP);
    soft_enc_r_last = (gpio_get_level(ENCODER_R_A_PIN) << 1) | gpio_get_level(ENCODER_R_B_PIN);
#else
    soft_enc_r_count = 0;
    soft_enc_r_last  = 0;
#endif
}

// 软件编码器扫描（100us 中断里调用）
// MOTOR_ENCODER_ENABLE=0：空跑
void Soft_Encoder_Scan(void)
{
#if MOTOR_ENCODER_ENABLE
    uint8 curr = (gpio_get_level(ENCODER_R_A_PIN) << 1) | gpio_get_level(ENCODER_R_B_PIN);

    if(curr != soft_enc_r_last)
    {
        if ((soft_enc_r_last == 0 && curr == 1) || (soft_enc_r_last == 1 && curr == 3) ||
            (soft_enc_r_last == 3 && curr == 2) || (soft_enc_r_last == 2 && curr == 0))
            soft_enc_r_count++;
        else
            soft_enc_r_count--;

        soft_enc_r_last = curr;
    }
#endif
}

// 获取编码器计数（5ms 调一次常见）
void Encoder_Get_Val(int16 *L, int16 *R)
{
#if MOTOR_ENCODER_ENABLE
    *L = encoder_get_count(TIM1_ENCOEDER);
    encoder_clear_count(TIM1_ENCOEDER);

    *R = (int16)soft_enc_r_count;
    soft_enc_r_count = 0;
#else
    *L = 0;
    *R = 0;
#endif
}

// 左电机输出
void Motor_Set_L(int16 duty)
{
    // 死区补偿（使用 motor.h 里的 MOTOR_DEAD_ZONE，避免重定义）
    if(duty > 0) duty += MOTOR_DEAD_ZONE;
    else if(duty < 0) duty -= MOTOR_DEAD_ZONE;

    duty = motor_clip_i16(duty, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);

    // TB6612：一边 PWM 另一边 0（不需要互补 PWM）
    if(duty >= 0)
    {
        pwm_set_duty(MOTOR_L_IN1_PWM, (uint32)duty);
        pwm_set_duty(MOTOR_L_IN2_PWM, 0);
    }
    else
    {
        pwm_set_duty(MOTOR_L_IN1_PWM, 0);
        pwm_set_duty(MOTOR_L_IN2_PWM, (uint32)(-duty));
    }
}

// 右电机输出
void Motor_Set_R(int16 duty)
{
    if(duty > 0) duty += MOTOR_DEAD_ZONE;
    else if(duty < 0) duty -= MOTOR_DEAD_ZONE;

    duty = motor_clip_i16(duty, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);

    if(duty >= 0)
    {
        pwm_set_duty(MOTOR_R_IN1_PWM, (uint32)duty);
        pwm_set_duty(MOTOR_R_IN2_PWM, 0);
    }
    else
    {
        pwm_set_duty(MOTOR_R_IN1_PWM, 0);
        pwm_set_duty(MOTOR_R_IN2_PWM, (uint32)(-duty));
    }
}
