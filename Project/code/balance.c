//#include "balance.h"
//#include "motor.h"
//#include "pid.h"
//#include "param.h"
//#include "imu_mpu6050.h" 
//#include "attitude.h"
//#include "mode.h" 
//#include "Buzzer.h" 

<<<<<<< Updated upstream
// å®šä¹‰å…¨å±€å§¿æ€å˜é‡ Pitch (ä¾›å¤–éƒ¨è°ƒç”¨)
float pitch = 0.0f;

// å†…éƒ¨å˜é‡ï¼šä¸²çº§æ§åˆ¶ä¸­é—´é‡
static float target_tilt_angle = 0.0f; // é€Ÿåº¦ç¯è®¡ç®—å‡ºçš„"ç›®æ ‡å€¾è§’"
static uint8 velocity_div_count = 0;   // é€Ÿåº¦ç¯åˆ†é¢‘è®¡æ•°å™¨

// å¸¸é‡å®šä¹‰
#define RAD_TO_DEG 57.29578f
#define DEG_TO_RAD 0.0174533f

// =================================================================
// å¹³è¡¡æ§åˆ¶åˆå§‹åŒ–
// =================================================================
void Balance_Init(void)
{

    // 1. åˆå§‹åŒ– PID
    PID_Init();
    // å¦‚æœ Flash å‚æ•°æœªåˆå§‹åŒ–ï¼ŒåŠ è½½é»˜è®¤å€¼
    if(g_sys_param.balance_kp < 0.1f) Param_SetDefaults();

    // 2. åˆå§‹åŒ– MPU6050
    mpu6050_init(); 

    // 3. åˆå§‹åŒ– Mahony å§¿æ€è§£ç®—
    // é‡‡æ ·é¢‘ç‡ 200Hz, Kp=0.5, Ki=0.0 (ç»éªŒå€¼)
    mahony_init(&m_imu, 200.0f, 0.5f, 0.0f); 
}

// =================================================================
// å¹³è¡¡æ§åˆ¶ä»»åŠ¡ (å¿…é¡»æ¯ 5ms æ‰§è¡Œä¸€æ¬¡)
// =================================================================
void Balance_Task(void)
{
    // C251 å˜é‡å®šä¹‰ç½®é¡¶
    int16 speed_l, speed_r, speed_avg;
    int16 vertical_pwm, turn_pwm;
    int16 motor_out_l, motor_out_r;
    float gyro_x_rad, gyro_y_rad, gyro_z_rad;
    float acc_x_g, acc_y_g, acc_z_g;

    // -----------------------------------------------------------
    // 1. è·å–ä¼ æ„Ÿå™¨åŸå§‹æ•°æ®
    // -----------------------------------------------------------
    mpu6050_get_gyro(); 
    mpu6050_get_acc();  

    // -----------------------------------------------------------
    // 2. æ•°æ®è½¬æ¢ (å…³é”®ä¿®æ­£ï¼šè½¬ä¸º rad/s ä¾› Mahony ä½¿ç”¨)
    // -----------------------------------------------------------
    acc_x_g = mpu6050_acc_transition(mpu6050_acc_x);
    acc_y_g = mpu6050_acc_transition(mpu6050_acc_y);
    acc_z_g = mpu6050_acc_transition(mpu6050_acc_z);

    // ã€ä¿®æ­£ã€‘å°† deg/s è½¬æ¢ä¸º rad/s
    gyro_x_rad = mpu6050_gyro_transition(mpu6050_gyro_x) * DEG_TO_RAD;
    gyro_y_rad = mpu6050_gyro_transition(mpu6050_gyro_y) * DEG_TO_RAD;
    gyro_z_rad = mpu6050_gyro_transition(mpu6050_gyro_z) * DEG_TO_RAD;

    // -----------------------------------------------------------
    // 3. å§¿æ€è§£ç®— (Mahony ç®—æ³•)
    // -----------------------------------------------------------
    // å¿…é¡»ä¼ å…¥ rad/sï¼Œå¦åˆ™è§£ç®—ä¼šå‘æ•£
    mahony_update(&m_imu, 
                  acc_x_g, acc_y_g, acc_z_g, 
                  gyro_x_rad, gyro_y_rad, gyro_z_rad);
    
    // è·å–æœ€æ–°çš„ Pitch è§’åº¦ (mahony å†…éƒ¨å·²è½¬å›è§’åº¦åˆ¶)
    pitch = m_imu.pitch;

    // -----------------------------------------------------------
    // 4. é€»è¾‘è°ƒåº¦
    // -----------------------------------------------------------
    Mode_Handler(); // è®¡ç®— target_speed_val å’Œ target_turn_val
    Buzzer_Task();  // åˆ·æ–°èœ‚é¸£å™¨

    // -----------------------------------------------------------
    // 5. å®‰å…¨ä¿æŠ¤ (å€’åœ°åœè½¦)
    // -----------------------------------------------------------
    if ((pitch > 45.0f || pitch < -45.0f) || 
        (current_mode == MODE_4_REPLAY && path_state == REC_ON))
    {
        Motor_Set_L(0);
        Motor_Set_R(0);
        PID_Clear_Integral(); // å€’åœ°åæ¸…é™¤ç§¯åˆ†
        return; 
    }

    // -----------------------------------------------------------
    // 6. ä¸²çº§ PID è®¡ç®—
    // -----------------------------------------------------------

    // === A. é€Ÿåº¦ç¯ (å¤–ç¯) ===
    // é™é¢‘å¤„ç†ï¼šæ¯ 20ms (4 * 5ms) æ‰§è¡Œä¸€æ¬¡
    velocity_div_count++;
    if(velocity_div_count >= 4)
    {
        velocity_div_count = 0;

        Encoder_Get_Val(&speed_l, &speed_r);
        speed_avg = (speed_l + speed_r) / 2;

        // ã€ä¸²çº§æ ¸å¿ƒã€‘è®¡ç®—ç›®æ ‡å€¾è§’ (è¾“å‡ºçš„æ˜¯ float è§’åº¦)
        // è¾“å…¥ï¼šç›®æ ‡é€Ÿåº¦ï¼Œå½“å‰é€Ÿåº¦
        // è¾“å‡ºï¼štarget_tilt_angle (ä¾‹å¦‚ï¼šä¸ºäº†åŠ é€Ÿï¼Œéœ€è¦å‰å€¾ 3.5 åº¦)
        target_tilt_angle = PID_Velocity(target_speed_val, speed_avg);
    }

    // === B. ç›´ç«‹ç¯ (å†…ç¯) ===
    // æ¯ 5ms æ‰§è¡Œä¸€æ¬¡
    // ã€ä¸²çº§æ ¸å¿ƒã€‘ç›®æ ‡è§’åº¦ = é€Ÿåº¦ç¯è¾“å‡º + æœºæ¢°ä¸­å€¼
    // ä¼ å…¥å‚æ•°ï¼šå½“å‰è§’åº¦, å½“å‰è§’é€Ÿåº¦(deg/s), ç›®æ ‡è§’åº¦
    // æ³¨æ„ï¼šPID_Vertical å†…éƒ¨ä½¿ç”¨çš„æ˜¯ deg/sï¼Œæ‰€ä»¥è¿™é‡Œä¼ å…¥ transition åçš„åŸå§‹å€¼(æœªè½¬rad)
    vertical_pwm = PID_Vertical(pitch, mpu6050_gyro_transition(mpu6050_gyro_y), target_tilt_angle);

    // === C. è½¬å‘ç¯ ===
    // ä¿®æ­£ï¼šä¼ å…¥åŸå§‹æ•°æ®ç»™ PID_Turn (å®ƒåªåšç®€å•æ¯”ä¾‹æ§åˆ¶)
    turn_pwm = PID_Turn(target_turn_val, mpu6050_gyro_z);

    // -----------------------------------------------------------
    // 7. ç”µæœºè¾“å‡ºåˆæˆ
    // -----------------------------------------------------------
    motor_out_l = vertical_pwm + turn_pwm;
    motor_out_r = vertical_pwm - turn_pwm;

    Motor_Set_L(motor_out_l);
    Motor_Set_R(motor_out_r);
}
=======
//// ¶¨ÒåÈ«¾Ö×ËÌ¬±äÁ¿ Pitch (¹©Íâ²¿µ÷ÓÃ)
//float pitch = 0.0f;

//// ÄÚ²¿±äÁ¿£º´®¼¶¿ØÖÆÖĞ¼äÁ¿
//static float target_tilt_angle = 0.0f; // ËÙ¶È»·¼ÆËã³öµÄ"Ä¿±êÇã½Ç"
//static uint8 velocity_div_count = 0;   // ËÙ¶È»··ÖÆµ¼ÆÊıÆ÷

//// ³£Á¿¶¨Òå
//#define RAD_TO_DEG 57.29578f
//#define DEG_TO_RAD 0.0174533f

//// =================================================================
//// Æ½ºâ¿ØÖÆ³õÊ¼»¯
//// =================================================================
//void Balance_Init(void)
//{
//    // 1. ³õÊ¼»¯ PID
//    PID_Init();
//    // Èç¹û Flash ²ÎÊıÎ´³õÊ¼»¯£¬¼ÓÔØÄ¬ÈÏÖµ
//    if(g_sys_param.balance_kp < 0.1f) Param_SetDefaults();
//    
//    // 2. ³õÊ¼»¯ MPU6050
//    mpu6050_init(); 
//    
//    // 3. ³õÊ¼»¯ Mahony ×ËÌ¬½âËã
//    // ²ÉÑùÆµÂÊ 200Hz, Kp=0.5, Ki=0.0 (¾­ÑéÖµ)
//    mahony_init(&m_imu, 200.0f, 0.5f, 0.0f); 
//}

//// =================================================================
//// Æ½ºâ¿ØÖÆÈÎÎñ (±ØĞëÃ¿ 5ms Ö´ĞĞÒ»´Î)
//// =================================================================
//void Balance_Task(void)
//{
//    // C251 ±äÁ¿¶¨ÒåÖÃ¶¥
//    int16 speed_l, speed_r, speed_avg;
//    int16 vertical_pwm, turn_pwm;
//    int16 motor_out_l, motor_out_r;
//    float gyro_x_rad, gyro_y_rad, gyro_z_rad;
//    float acc_x_g, acc_y_g, acc_z_g;

//    // -----------------------------------------------------------
//    // 1. »ñÈ¡´«¸ĞÆ÷Ô­Ê¼Êı¾İ
//    // -----------------------------------------------------------
//    mpu6050_get_gyro(); 
//    mpu6050_get_acc();  
//    
//    // -----------------------------------------------------------
//    // 2. Êı¾İ×ª»» (¹Ø¼üĞŞÕı£º×ªÎª rad/s ¹© Mahony Ê¹ÓÃ)
//    // -----------------------------------------------------------
//    acc_x_g = mpu6050_acc_transition(mpu6050_acc_x);
//    acc_y_g = mpu6050_acc_transition(mpu6050_acc_y);
//    acc_z_g = mpu6050_acc_transition(mpu6050_acc_z);
//    
//    // ¡¾ĞŞÕı¡¿½« deg/s ×ª»»Îª rad/s
//    gyro_x_rad = mpu6050_gyro_transition(mpu6050_gyro_x) * DEG_TO_RAD;
//    gyro_y_rad = mpu6050_gyro_transition(mpu6050_gyro_y) * DEG_TO_RAD;
//    gyro_z_rad = mpu6050_gyro_transition(mpu6050_gyro_z) * DEG_TO_RAD;

//    // -----------------------------------------------------------
//    // 3. ×ËÌ¬½âËã (Mahony Ëã·¨)
//    // -----------------------------------------------------------
//    // ±ØĞë´«Èë rad/s£¬·ñÔò½âËã»á·¢É¢
//    mahony_update(&m_imu, 
//                  acc_x_g, acc_y_g, acc_z_g, 
//                  gyro_x_rad, gyro_y_rad, gyro_z_rad);
//    
//    // »ñÈ¡×îĞÂµÄ Pitch ½Ç¶È (mahony ÄÚ²¿ÒÑ×ª»Ø½Ç¶ÈÖÆ)
//    pitch = m_imu.pitch;

//    // -----------------------------------------------------------
//    // 4. Âß¼­µ÷¶È
//    // -----------------------------------------------------------
//    Mode_Handler(); // ¼ÆËã target_speed_val ºÍ target_turn_val
//    Buzzer_Task();  // Ë¢ĞÂ·äÃùÆ÷

//    // -----------------------------------------------------------
//    // 5. °²È«±£»¤ (µ¹µØÍ£³µ)
//    // -----------------------------------------------------------
//    if ((pitch > 45.0f || pitch < -45.0f) || 
//        (current_mode == MODE_4_REPLAY && path_state == REC_ON))
//    {
//        Motor_Set_L(0);
//        Motor_Set_R(0);
//        PID_Clear_Integral(); // µ¹µØºóÇå³ı»ı·Ö
//        return; 
//    }

//    // -----------------------------------------------------------
//    // 6. ´®¼¶ PID ¼ÆËã
//    // -----------------------------------------------------------
//    
//    // === A. ËÙ¶È»· (Íâ»·) ===
//    // ½µÆµ´¦Àí£ºÃ¿ 20ms (4 * 5ms) Ö´ĞĞÒ»´Î
//    velocity_div_count++;
//    if(velocity_div_count >= 4)
//    {
//        velocity_div_count = 0;
//        
//        Encoder_Get_Val(&speed_l, &speed_r);
//        speed_avg = (speed_l + speed_r) / 2;
//        
//        // ¡¾´®¼¶ºËĞÄ¡¿¼ÆËãÄ¿±êÇã½Ç (Êä³öµÄÊÇ float ½Ç¶È)
//        // ÊäÈë£ºÄ¿±êËÙ¶È£¬µ±Ç°ËÙ¶È
//        // Êä³ö£ºtarget_tilt_angle (ÀıÈç£ºÎªÁË¼ÓËÙ£¬ĞèÒªÇ°Çã 3.5 ¶È)
//        target_tilt_angle = PID_Velocity(target_speed_val, speed_avg);
//    }
//    
//    // === B. Ö±Á¢»· (ÄÚ»·) ===
//    // Ã¿ 5ms Ö´ĞĞÒ»´Î
//    // ¡¾´®¼¶ºËĞÄ¡¿Ä¿±ê½Ç¶È = ËÙ¶È»·Êä³ö + »úĞµÖĞÖµ
//    // ´«Èë²ÎÊı£ºµ±Ç°½Ç¶È, µ±Ç°½ÇËÙ¶È(deg/s), Ä¿±ê½Ç¶È
//    // ×¢Òâ£ºPID_Vertical ÄÚ²¿Ê¹ÓÃµÄÊÇ deg/s£¬ËùÒÔÕâÀï´«Èë transition ºóµÄÔ­Ê¼Öµ(Î´×ªrad)
//    vertical_pwm = PID_Vertical(pitch, mpu6050_gyro_transition(mpu6050_gyro_y), target_tilt_angle);
//    
//    // === C. ×ªÏò»· ===
//    // ĞŞÕı£º´«ÈëÔ­Ê¼Êı¾İ¸ø PID_Turn (ËüÖ»×ö¼òµ¥±ÈÀı¿ØÖÆ)
//    turn_pwm = PID_Turn(target_turn_val, mpu6050_gyro_z);

//    // -----------------------------------------------------------
//    // 7. µç»úÊä³öºÏ³É
//    // -----------------------------------------------------------
//    motor_out_l = vertical_pwm + turn_pwm;
//    motor_out_r = vertical_pwm - turn_pwm;
//    
//    Motor_Set_L(motor_out_l);
//    Motor_Set_R(motor_out_r);
//}
>>>>>>> Stashed changes
