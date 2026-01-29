#include "param.h"
#include "intrins.h" 

SysParam_t g_sys_param;

// =================================================================
// 寄存器定义 (适配 STC32G)
// =================================================================

#define IAP_TPS_VAL     35      // 35MHz
#define CMD_IDLE        0       
#define CMD_READ        1       
#define CMD_PROGRAM     2       
#define CMD_ERASE       3       
#define ENABLE_IAP      0x80    

// 使用 60KB 处 (Sector 120)
#define PARAM_FLASH_SECTOR  120
#define PARAM_FLASH_ADDR    (PARAM_FLASH_SECTOR * 512UL) 

// =================================================================
// 内部 IAP 驱动 (STC32G 专用)
// =================================================================

void IapIdle(void)
{

    IAP_CONTR = 0;      
    IAP_CMD = 0;        
    IAP_TRIG = 0;       
    IAP_ADDRH = 0x80;   
    IAP_ADDRL = 0;
}

uint8 IapReadByte(uint32 addr)
{
    uint8 dat;
    IAP_CONTR = ENABLE_IAP; 
    IAP_TPS = IAP_TPS_VAL;  
    IAP_CMD = CMD_READ;     

    IAP_ADDRL = addr;       
    IAP_ADDRH = addr >> 8;  

    IAP_TRIG = 0x5A;        
    IAP_TRIG = 0xA5;
    _nop_();

    dat = IAP_DATA;         
    IapIdle();              
    return dat;
}

void IapProgramByte(uint32 addr, uint8 dat)
{

    IAP_CONTR = ENABLE_IAP; 
    IAP_TPS = IAP_TPS_VAL;  
    IAP_CMD = CMD_PROGRAM;  

    IAP_ADDRL = addr;
    IAP_ADDRH = addr >> 8;
    IAP_DATA = dat;         

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    IapIdle();
}

void IapEraseSector(uint32 addr)
{

    IAP_CONTR = ENABLE_IAP;
    IAP_TPS = IAP_TPS_VAL;
    IAP_CMD = CMD_ERASE;

    IAP_ADDRL = addr;
    IAP_ADDRH = addr >> 8;

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    IapIdle();
}

// =================================================================
// 业务逻辑
// =================================================================

void Param_SetDefaults(void) 
{

    g_sys_param.flag = 0x55AA;
    g_sys_param.balance_kp = 180.0f; 
    g_sys_param.balance_kd = 1.0f;
    g_sys_param.velocity_kp = 0.5f;
    g_sys_param.velocity_ki = 0.005f;
    g_sys_param.turn_kp = 25.0f; 
    g_sys_param.turn_kd = 5.0f;   
    g_sys_param.track_speed = 30; 
    g_sys_param.mech_zero_pitch = 0.0f; 
}

void Param_Init(void) 
{

    uint16 i;
    uint8 *p = (uint8*)&g_sys_param;

    for(i = 0; i < sizeof(SysParam_t); i++)
    {
        *p++ = IapReadByte(PARAM_FLASH_ADDR + i);
    }

    if(g_sys_param.flag != 0x55AA)
    {
        Param_SetDefaults(); 
        Param_Save();        
    }
}

void Param_Save(void) 
{

    uint16 i;
    uint8 *p = (uint8*)&g_sys_param;

    IapEraseSector(PARAM_FLASH_ADDR);

    for(i = 0; i < sizeof(SysParam_t); i++)
    {
        IapProgramByte(PARAM_FLASH_ADDR + i, *p++);
    }
}
