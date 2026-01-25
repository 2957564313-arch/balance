#include "zf_common_headfile.h"
	imu_data_t imu;
	mahony_t   mahony;
void main(void)
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();

	Key_Init();
    OLED_Init();

    OLED_Clear();
    OLED_ShowString(1, 1, "HELLO");
    OLED_ShowNum(2, 1, 12345, 5);
    OLED_ShowSignedNum(3, 1, -678, 3);
    OLED_ShowFloat(4, 1, 3.1415f, 1, 3);

    OLED_Update();   
	
	

	imu_mpu6050_init();
	imu_mpu6050_gyro_calibrate(200);
	mahony_init(&mahony, 100.0f, 5.0f, 0.05f);

	while(1)
{
		imu_mpu6050_read(&imu);

		mahony_update(&mahony,
                  imu.gx, imu.gy, imu.gz,
                  imu.ax, imu.ay, imu.az);
		system_delay_ms(100);
	}
}