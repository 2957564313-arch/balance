#include "bluetooth.h"
#include <stdarg.h>
#include <stdio.h> 

// =================================================================
// 全局变量定义
// =================================================================
int16 remote_speed = 0;
int16 remote_turn = 0;

// 接收相关变量
uint8 bt_rx_buffer[BT_RX_MAX_LEN]; 
uint8 bt_rx_index = 0;             
uint8 bt_packet_started = 0;       

void BT_Init(void)
{
    // 初始化 UART4, 9600波特率
    uart_init(BT_UART_INDEX, BT_BAUD_RATE, BT_TX_PIN, BT_RX_PIN);
    
    // 清空缓冲区
    {
        uint8 i;
        for(i=0; i<BT_RX_MAX_LEN; i++) bt_rx_buffer[i] = 0;
    }
    
    bt_rx_index = 0;
    bt_packet_started = 0;
    
    remote_speed = 0;
    remote_turn = 0;
}

void BT_Send_Byte(uint8 dat)
{
    uart_write_byte(BT_UART_INDEX, dat);
}

void BT_Send_Str(char *str)
{
    while(*str)
    {
        uart_write_byte(BT_UART_INDEX, *str++);
    }
}

// 格式化打印函数，方便调试
void BT_Printf(const char *fmt, ...)
{
    char buff[128];
    va_list args;
    
    va_start(args, fmt);
    vsprintf(buff, fmt, args);
    va_end(args);
    
    BT_Send_Str(buff);
}

// =================================================================
// 手写简易解析工具：从字符串中提取整数
// 替代标准库的 atoi/strtok，更轻量
// =================================================================
int My_Atoi(char *str, uint8 *p_idx)
{
    int res = 0;
    int sign = 1;
    uint8 i = *p_idx;
    
    // 1. 跳过非数字字符 (逗号、空格、字母等)
    // 注意：不要跳过负号 '-'
    while(str[i] != '\0' && (str[i] < '0' || str[i] > '9') && str[i] != '-') {
        i++;
    }
    
    // 2. 检测负号
    if(str[i] == '-') {
        sign = -1;
        i++;
    }
    
    // 3. 提取连续数字
    while(str[i] >= '0' && str[i] <= '9') {
        res = res * 10 + (str[i] - '0');
        i++;
    }
    
    *p_idx = i; // 更新外部索引指针
    return res * sign;
}

// =================================================================
// 核心：解析江协小程序数据包
// 格式: [joystick,LH,LV,RH,RV]  例: [joystick,0,100,0,0]
// =================================================================
void BT_Parse_Packet(char *packet)
{
    uint8 i = 0;
    int lh, lv, rh, rv;
    
    // 1. 头部匹配 "joy"
    if(packet[0] == 'j' && packet[1] == 'o' && packet[2] == 'y')
    {
        // 2. 跳过 "joystick" (8个字符)
        i = 8; 
        
        // 3. 依次提取 4 个摇杆值
        lh = My_Atoi(packet, &i); // 左横
        lv = My_Atoi(packet, &i); // 左纵 (速度)
        rh = My_Atoi(packet, &i); // 右横 (转向)
        rv = My_Atoi(packet, &i); // 右纵
        
        // 4. 映射到控制变量
        // LV (左摇杆纵向) -> 速度 (放大 2 倍)
        remote_speed = lv * 2; 
        
        // RH (右摇杆横向) -> 转向 (放大 5 倍，转向需要灵敏点)
        remote_turn = rh * 5;  
        
        // 5. 死区过滤 (防止归位误差导致车子慢飘)
        if(remote_speed > -15 && remote_speed < 15) remote_speed = 0;
        if(remote_turn > -15 && remote_turn < 15) remote_turn = 0;
    }
}

// =================================================================
// 接收状态机 (在主循环调用)
// =================================================================
void BT_Check_Rx(void)
{
    uint8 dat;
    
    // 查询方式读取串口数据
    while(uart_query_byte(BT_UART_INDEX, &dat))
    {
        // 1. 等待包头 '['
        if (dat == '[') 
        {
            bt_packet_started = 1;
            bt_rx_index = 0;
            bt_rx_buffer[0] = '\0'; 
        }
        // 2. 接收中
        else if (bt_packet_started)
        {
            // 3. 等待包尾 ']'
            if (dat == ']') 
            {
                bt_packet_started = 0;
                bt_rx_buffer[bt_rx_index] = '\0'; // 补上结束符
                
                // 解析完整包
                BT_Parse_Packet((char *)bt_rx_buffer);
            }
            // 4. 存入缓冲区
            else 
            {
                if (bt_rx_index < BT_RX_MAX_LEN - 1)
                {
                    bt_rx_buffer[bt_rx_index++] = dat;
                }
                else // 溢出保护
                {
                    bt_packet_started = 0;
                    bt_rx_index = 0;
                }
            }
        }
    }
}
