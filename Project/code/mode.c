//#include "mode.h"
//#include "line.h"
//#include "motor.h"
//#include "bluetooth.h"
//#include "Buzzer.h"
//#include "param.h"
//#include "balance.h" // 引用 pitch, roll, yaw
//#include <math.h>

//// 全局变量定义
//RunMode_e current_mode = MODE_1_BALANCE;
//int16 target_speed_val = 0;
//int16 target_turn_val = 0;

//// === 赛道状态机 ===
//typedef enum {
//    STATE_STRAIGHT = 0, // 直道
//    STATE_CURVE         // 弯道
//} TrackState_e;

//static TrackState_e track_state = STATE_STRAIGHT; 
//static float enter_curve_yaw = 0.0f; 
//static uint8 curve_cnt = 0; 

//// 路径记忆变量
//PathState_e path_state = REC_OFF;
//#define MAX_PATH 1000 
//int16 xdata path_spd_buf[MAX_PATH];
//int16 xdata path_turn_buf[MAX_PATH];
//uint16 path_len = 0;
//uint16 path_ptr = 0;

//static float last_pos_err = 0.0f;
//static uint8 mode4_div_cnt = 0;

//void Mode_Init(void) {
//    Line_Init();
//}

//void Mode_Path_Key_Handler(uint8 key) {
//    if(current_mode != MODE_4_REPLAY) return;
//    
//    if(key == 1) { 
//        if(path_state == REC_OFF) { 
//            path_state = REC_ON; 
//            path_len = 0; 
//            mode4_div_cnt = 0; 
//            Buzzer_Beep(100); 
//        }
//        else if(path_state == REC_ON) { 
//            path_state = REC_OFF; 
//            Buzzer_Beep(200); 
//        }
//    }
//    if(key == 2 && path_len > 0) { 
//        path_state = REC_PLAY; 
//        path_ptr = 0; 
//        mode4_div_cnt = 0; 
//        Buzzer_Beep(500);
//    }
//}

//// 辅助函数：计算角度差 (处理 -180 到 180 跳变)
//float Calc_Angle_Diff(float angle_now, float angle_target) {
//    float diff = angle_now - angle_target;
//    while(diff > 180.0f)  diff -= 360.0f;
//    while(diff < -180.0f) diff += 360.0f;
//    return (diff >= 0) ? diff : -diff;
//}

//// 5ms 定时中断调用
//void Mode_Handler(void) {
//    // 变量定义置顶 (C251)
//    float pos_err;
//    int16 spd_L, spd_R;
//    uint8 has_line;
//    float angle_diff;
//    float abs_err;
//    int16 dyn_spd;
//    
//    switch(current_mode) {
//        case MODE_1_BALANCE:
//            target_speed_val = 0;
//            target_turn_val = 0;
//            track_state = STATE_STRAIGHT;
//            curve_cnt = 0;
//            last_pos_err = 0.0f; 
//            break;
//            
//        case MODE_2_TRACK_1: 
//        case MODE_3_TRACK_4: 
//            has_line = Track_Is_Line_Exist();
//            
//            if(track_state == STATE_STRAIGHT) 
//            {
//                target_turn_val = 0;
//                target_speed_val = 40; 
//                
//                if(has_line) 
//                {
//                    track_state = STATE_CURVE;
//                    // 【修复】直接使用全局变量 yaw
//                    enter_curve_yaw = yaw; 
//                    Buzzer_Beep(100); 
//                }
//            }
//            else // STATE_CURVE
//            {
//                pos_err = Track_Get_Weighted_Error();
//                target_turn_val = (int16)(pos_err * g_sys_param.turn_kp + (pos_err - last_pos_err) * 50.0f);
//                last_pos_err = pos_err;
//                
//                abs_err = (pos_err > 0) ? pos_err : -pos_err;
//                dyn_spd = g_sys_param.track_speed - (int16)(5.0f * abs_err);
//                if(dyn_spd < 20) dyn_spd = 20;
//                target_speed_val = dyn_spd;
//                
//                // 【修复】使用全局变量 yaw 和 辅助函数
//                angle_diff = Calc_Angle_Diff(yaw, enter_curve_yaw);
//                
//                if(!has_line && angle_diff > 150.0f) 
//                {
//                    track_state = STATE_STRAIGHT;
//                    curve_cnt++; 
//                    Buzzer_Beep(100); 
//                    
//                    if(current_mode == MODE_2_TRACK_1 && curve_cnt >= 2) {
//                        target_speed_val = 0;
//                        current_mode = MODE_1_BALANCE;
//                        Buzzer_Beep(500); 
//                    }
//                    if(current_mode == MODE_3_TRACK_4 && curve_cnt >= 8) {
//                        target_speed_val = 0;
//                        current_mode = MODE_1_BALANCE;
//                        Buzzer_Beep(500);
//                    }
//                }
//            }
//            break;
//            
//        case MODE_4_REPLAY:
//            mode4_div_cnt++;
//            if(mode4_div_cnt >= 4)
//            {
//                mode4_div_cnt = 0;
//                if(path_state == REC_ON) {
//                    Encoder_Get_Val(&spd_L, &spd_R);
//                    if(path_len < MAX_PATH) {
//                        path_spd_buf[path_len] = (spd_L + spd_R) / 2;
//                        path_turn_buf[path_len] = (spd_L - spd_R); 
//                        path_len++;
//                    }
//                    target_speed_val = 0; 
//                    target_turn_val = 0;
//                } 
//                else if(path_state == REC_PLAY) {
//                    if(path_ptr < path_len) {
//                        target_speed_val = path_spd_buf[path_ptr];
//                        target_turn_val = path_turn_buf[path_ptr];
//                        path_ptr++;
//                    } else { 
//                        target_speed_val = 0; 
//                        path_state = REC_OFF; 
//                    }
//                } 
//                else { 
//                    target_speed_val = 0; 
//                }
//            }
//            break;
//            
//        case MODE_5_REMOTE:
//            break;
//    }
//}
