#include "mode.h"
#include "line.h"
#include "motor.h"
#include "bluetooth.h"
#include "Buzzer.h"
#include "param.h"
#include "attitude.h" 

// 全局变量定义
RunMode_e current_mode = MODE_1_BALANCE;
int16 target_speed_val = 0;
int16 target_turn_val = 0;

// === 赛道状态机 ===
typedef enum {
    STATE_STRAIGHT = 0, // 直道
    STATE_CURVE         // 弯道
} TrackState_e;

static TrackState_e track_state = STATE_STRAIGHT; 
static float enter_curve_yaw = 0.0f; 
static uint8 curve_cnt = 0; 

// 路径记忆变量
PathState_e path_state = REC_OFF;
#define MAX_PATH 1000 
int16 xdata path_spd_buf[MAX_PATH];
int16 xdata path_turn_buf[MAX_PATH];
uint16 path_len = 0;
uint16 path_ptr = 0;

// 循迹 PID 的 D项 记忆
static float last_pos_err = 0.0f;

// Mode 4 分频计数器
static uint8 mode4_div_cnt = 0;

void Mode_Init(void) {
    Line_Init();
}

void Mode_Path_Key_Handler(uint8 key) {
    if(current_mode != MODE_4_REPLAY) return;
    
    // 按键1：录制开关
    if(key == 1) { 
        if(path_state == REC_OFF) { 
            path_state = REC_ON; 
            path_len = 0; 
            mode4_div_cnt = 0; // 重置分频
            Buzzer_Beep(100); 
        }
        else if(path_state == REC_ON) { 
            path_state = REC_OFF; 
            Buzzer_Beep(200); 
        }
    }
    // 按键2：开始回放
    if(key == 2 && path_len > 0) { 
        path_state = REC_PLAY; 
        path_ptr = 0; 
        mode4_div_cnt = 0; 
        Buzzer_Beep(500);
    }
}

float my_abs(float f) { return f >= 0 ? f : -f; }

// 5ms 定时中断调用
void Mode_Handler(void) {
    // 变量定义置顶 (C251)
    float pos_err;
    int16 spd_L, spd_R;
    uint8 has_line;
    float angle_diff;
    float abs_err;
    int16 dyn_spd;
    
    switch(current_mode) {
        case MODE_1_BALANCE:
            target_speed_val = 0;
            target_turn_val = 0;
            
            // 重置所有状态
            track_state = STATE_STRAIGHT;
            curve_cnt = 0;
            m_imu.total_yaw = 0.0f; 
            last_pos_err = 0.0f; // 【修复】清除上次误差，防止下次循迹抖动
            break;
            
        case MODE_2_TRACK_1: 
        case MODE_3_TRACK_4: 
            has_line = Track_Is_Line_Exist();
            
            if(track_state == STATE_STRAIGHT) 
            {
                // 直道策略：无脑冲
                target_turn_val = 0;
                target_speed_val = 40; 
                
                // 进弯判断
                if(has_line) 
                {
                    track_state = STATE_CURVE;
                    enter_curve_yaw = m_imu.total_yaw; 
                    Buzzer_Beep(100); 
                }
            }
            else // STATE_CURVE
            {
                // 弯道策略：PID 循迹
                pos_err = Track_Get_Weighted_Error();
                
                // 循迹 PD 计算 (注意：50.0f 是视觉微分系数，不是 param 里的 turn_kd)
                target_turn_val = (int16)(pos_err * g_sys_param.turn_kp + (pos_err - last_pos_err) * 50.0f);
                last_pos_err = pos_err;
                
                // 动态限速
                abs_err = (pos_err > 0) ? pos_err : -pos_err;
                dyn_spd = g_sys_param.track_speed - (int16)(5.0f * abs_err);
                if(dyn_spd < 20) dyn_spd = 20;
                target_speed_val = dyn_spd;
                
                // 出弯判断：没线了 + 转够了(150度)
                angle_diff = my_abs(m_imu.total_yaw - enter_curve_yaw);
                if(!has_line && angle_diff > 150.0f) 
                {
                    track_state = STATE_STRAIGHT;
                    curve_cnt++; 
                    Buzzer_Beep(100); 
                    
                    // 圈数判断
                    if(current_mode == MODE_2_TRACK_1 && curve_cnt >= 2) {
                        target_speed_val = 0;
                        current_mode = MODE_1_BALANCE;
                        Buzzer_Beep(500); 
                    }
                    if(current_mode == MODE_3_TRACK_4 && curve_cnt >= 8) {
                        target_speed_val = 0;
                        current_mode = MODE_1_BALANCE;
                        Buzzer_Beep(500);
                    }
                }
            }
            break;
            
        case MODE_4_REPLAY:
            // 【关键修复】降频处理！每 20ms 处理一次
            // 目的：1. 匹配速度环的控制周期 2. 延长录制时间到 20秒
            mode4_div_cnt++;
            if(mode4_div_cnt >= 4)
            {
                mode4_div_cnt = 0;
                
                if(path_state == REC_ON) {
                    // 录制：读取过去 20ms 内的编码器总值
                    // 此时 Balance_Task 读取到的速度为 0 (被这里抢走了)，正好方便推车
                    Encoder_Get_Val(&spd_L, &spd_R);
                    
                    if(path_len < MAX_PATH) {
                        path_spd_buf[path_len] = (spd_L + spd_R) / 2;
                        path_turn_buf[path_len] = (spd_L - spd_R); // 这里的系数可以根据手感调，暂定1:1
                        path_len++;
                    }
                    target_speed_val = 0; 
                    target_turn_val = 0;
                } 
                else if(path_state == REC_PLAY) {
                    // 回放：每 20ms 更新一次目标
                    if(path_ptr < path_len) {
                        target_speed_val = path_spd_buf[path_ptr];
                        target_turn_val = path_turn_buf[path_ptr];
                        path_ptr++;
                    } else { 
                        target_speed_val = 0; 
                        path_state = REC_OFF; 
                    }
                } 
                else { 
                    target_speed_val = 0; 
                }
            }
            break;
            
        case MODE_5_REMOTE:
            // 这里的变量需要在 bluetooth.c 里更新
            // target_speed_val = remote_speed;
            // target_turn_val = remote_turn;
            break;
    }
}
