#include "zf_common_headfile.h"
#include "OLED.h"
#include "imu_mpu6050.h" 
#include "balance.h" 

<<<<<<< Updated upstream
// 适配补丁
#ifndef pit_timer_ms
    #define pit_timer_ms(tim, ms)  pit_init(tim, (uint32)(ms) * (system_clock / 1000))
#endif

extern mahony_t m_imu; 

void TIM2_IRQHandler(void) interrupt 12
{
    Balance_Task(); 
}

=======
// =================================================================
// 寄存器直接配置定时器 (适配 STC32G @ 35MHz)

>>>>>>> Stashed changes
void main(void)
{
    uint8 mpu_status = 0;
    
    // --- 硬件初始化 ---
    clock_init(SYSTEM_CLOCK_35M); 
    debug_init();                 
    
    OLED_Init();                  
    
    mahony_init(&m_imu, 200.0f, 0.5f, 0.0f);
    
    OLED_ShowString(1, 1, "Init MPU...");
    mpu_status = mpu6050_init(); 
    
    if(mpu_status == 0)
    {
        OLED_ShowString(2, 1, "MPU OK!     ");
        pit_timer_ms(TIM_2, 5); 
        interrupt_global_enable(); 
    }
    else
    {
        OLED_ShowString(2, 1, "MPU Error!  ");
        while(1); 
    }
    
    // --- 静态UI ---
    OLED_Clear(); 
    OLED_ShowString(1, 1, "=== BALANCE ===");
    OLED_ShowString(2, 1, "Pit:");
    OLED_ShowString(3, 1, "Rol:");
    OLED_ShowString(4, 1, "Yaw:");

<<<<<<< Updated upstream
    // --- 主循环 ---
    while(1)
    {

        system_delay_ms(5); 
        
        // 刷新显示
        OLED_Show_Float(2, 6, m_imu.pitch,     8, 1);
        OLED_Show_Float(3, 6, m_imu.roll,      8, 1);
        OLED_Show_Float(4, 6, m_imu.total_yaw, 8, 1);
=======
    // 3. 硬件底层初始化
    LED_Init();    // 心跳灯
    Buzzer_Init(); // 蜂鸣器
    Motor_Init();  // 电机 & 编码器
    BT_Init();     // 蓝牙
    OLED_Init();   // 屏幕
    Key_Init();    // 按键
    
    // 4. 应用层初始化
    Param_Init();  // 读取 Flash 参数 (必须先于 PID)
//    Balance_Init();// PID & IMU (必须最后，确保参数已就绪)
    
    // 提示音：初始化完成
    Buzzer_Beep(200);
    
    // 5. 开启定时器 (开始干活)
    EA = 1; 

    // 7. 主循环 (处理低优先级事务)
    while(1)
    {

>>>>>>> Stashed changes
    }
}
