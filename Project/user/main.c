#include "zf_common_headfile.h"
#include "OLED.h"
#include "motor.h"
#include "param.h"
#include "imu_mpu6050.h"
#include "filter.h"

// main 里显式声明（即便 motor.h 里没写，也不会再 L127）
extern void Motor_Stop(void);

// ======================= 控制周期 =======================
#define CTRL_HZ                 (200)
#define CTRL_DT_MS              (5)

// ======================= 安全解锁/保护 ===================
#define ARM_ANGLE_DEG           (10.0f)
#define ARM_STABLE_MS           (500)
#define FALL_ANGLE_DEG          (35.0f)
#define PWM_LIMIT               (9000)

// ======================= 方向/符号(救命开关) =============
#define BAL_PWM_DIR             ( 1)        // 越倒越快就改 -1
#define PITCH_DIR               ( 1.0f)     // pitch 方向反就改 -1
#define GYRO_Y_DIR              ( 1.0f)

static float absf(float x) { return (x < 0.0f) ? -x : x; }

// 直立 PD（不依赖 pid.c）
static int16 balance_pd(float pitch_deg, float gyro_y_dps, float target_deg)
{
    float err;
    float out;

    err = pitch_deg - (g_sys_param.mech_zero_pitch + target_deg);
    out = g_sys_param.balance_kp * err + g_sys_param.balance_kd * gyro_y_dps;

    return (int16)out;
}

void main(void)
{
    uint8  imu_fail;
    uint16 stable_ms;
    uint8  armed;
    uint8  ui_div;

    float gx, gy, gz, ax, ay, az;
    float pitch_now, gyro_y;
    float target_angle;
    int16 pwm_balance, pwm_l, pwm_r;

    clock_init(SYSTEM_CLOCK_35M);

    // 立车阶段先关总中断，减少冲突
    interrupt_global_disable();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "BOOT...");

    // Param 现在是“只读失败则默认”，不会再因为首次保存卡死
    Param_Init();

    // Motor 现在不初始化编码器，不会 assert 卡死
    Motor_Init();

    OLED_ShowString(2, 1, "IMU INIT");
    imu_fail = mpu6050_init();
    if(imu_fail)
    {
        OLED_ShowString(3, 1, "MPU FAIL");
        while(1)
        {
            Motor_Stop();
        }
    }

    OLED_ShowString(3, 1, "IMU CAL...");
    mpu6050_calibration();
    OLED_ShowString(3, 1, "IMU OK   ");
    system_delay_ms(200);

    Filter_Init((float)CTRL_HZ);

    stable_ms = 0;
    armed = 0;
    ui_div = 0;
    pwm_balance = 0;

    OLED_Clear();
    OLED_ShowString(1, 1, "SAFE P:");
    OLED_ShowString(2, 1, "PWM  :");
    OLED_ShowString(3, 1, "KP/KD:");
    OLED_ShowString(4, 1, "ZERO:");

    while(1)
    {
        // IMU
        mpu6050_get_gyro();
        mpu6050_get_acc();

        gx = mpu6050_gyro_transition(mpu6050_gyro_x);
        gy = mpu6050_gyro_transition(mpu6050_gyro_y);
        gz = mpu6050_gyro_transition(mpu6050_gyro_z);

        ax = mpu6050_acc_transition(mpu6050_acc_x);
        ay = mpu6050_acc_transition(mpu6050_acc_y);
        az = mpu6050_acc_transition(mpu6050_acc_z);

        Filter_Update(gx, gy, gz, ax, ay, az);

        {
            float roll, yaw;
            Filter_GetEuler(&pitch_now, &roll, &yaw);
        }

        pitch_now = pitch_now * PITCH_DIR;
        gyro_y    = gy * GYRO_Y_DIR;

        // SAFE/ARM
        if(!armed)
        {
            Motor_Stop();
            pwm_balance = 0;

            if(absf(pitch_now - g_sys_param.mech_zero_pitch) < ARM_ANGLE_DEG)
            {
                stable_ms += CTRL_DT_MS;
                if(stable_ms >= ARM_STABLE_MS)
                {
                    armed = 1;
                    stable_ms = 0;
                }
            }
            else
            {
                stable_ms = 0;
            }
        }
        else
        {
            if(absf(pitch_now - g_sys_param.mech_zero_pitch) > FALL_ANGLE_DEG)
            {
                armed = 0;
                Motor_Stop();
                pwm_balance = 0;
            }
            else
            {
                target_angle = 0.0f;

                pwm_balance = balance_pd(pitch_now, gyro_y, target_angle);
                pwm_balance = (int16)(pwm_balance * BAL_PWM_DIR);

                if(pwm_balance >  PWM_LIMIT) pwm_balance =  PWM_LIMIT;
                if(pwm_balance < -PWM_LIMIT) pwm_balance = -PWM_LIMIT;

                pwm_l = pwm_balance;
                pwm_r = pwm_balance;

                Motor_Set_L(pwm_l);
                Motor_Set_R(pwm_r);
            }
        }

        // OLED 每 50ms 刷新
        ui_div++;
        if(ui_div >= 10)
        {
            int16 pitch_x10;
            ui_div = 0;

            pitch_x10 = (int16)(pitch_now * 10.0f);

            if(armed) OLED_ShowString(1, 1, "ARM  P:");
            else      OLED_ShowString(1, 1, "SAFE P:");

            OLED_Show_Int_Fast(1, 8, (int32)pitch_x10);
            OLED_Show_Int_Fast(2, 7, (int32)pwm_balance);

            OLED_Show_Int_Fast(3, 6, (int32)g_sys_param.balance_kp);
            OLED_ShowChar(3, 10, '/');
            OLED_Show_Int_Fast(3, 12, (int32)g_sys_param.balance_kd);

            OLED_Show_Int_Fast(4, 6, (int32)(g_sys_param.mech_zero_pitch * 10.0f));
        }

        system_delay_ms(CTRL_DT_MS);
    }
}
