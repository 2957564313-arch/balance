#include "param.h"
#include "intrins.h"

SysParam_t g_sys_param;

// =================================================================
// IAP 配置（STC32G）
// =================================================================
#define IAP_TPS_VAL                 35      // 35MHz
#define CMD_IDLE                    0
#define CMD_READ                    1
#define CMD_PROGRAM                 2
#define CMD_ERASE                   3
#define ENABLE_IAP                  0x80

// 你原来用 60KB 处（Sector 120）
#define PARAM_FLASH_SECTOR          120
#define PARAM_FLASH_ADDR            (PARAM_FLASH_SECTOR * 512UL)

// =================== 关键：默认不在开机自动写 Flash ===================
// 0：只读，不匹配就加载默认但不保存（推荐，防 BOOT 卡死）
// 1：不匹配就加载默认并立刻保存（你原来的行为）
#define PARAM_AUTOSAVE_ON_FIRST_BOOT    0

// =================================================================
// 内部 IAP 驱动（加 EA 保护）
// =================================================================
static void IapIdle(void)
{
    IAP_CONTR = 0;
    IAP_CMD   = 0;
    IAP_TRIG  = 0;
    IAP_ADDRH = 0x80;     // 保留你原来的写法
    IAP_ADDRL = 0;
}

static uint8 IapReadByte(uint32 addr)
{
    uint8 dat;
    bit ea_save = EA;
    EA = 0;

    IAP_CONTR = ENABLE_IAP;
    IAP_TPS   = IAP_TPS_VAL;
    IAP_CMD   = CMD_READ;

    IAP_ADDRL = (uint8)(addr & 0xFF);
    IAP_ADDRH = (uint8)((addr >> 8) & 0xFF);

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_(); _nop_();

    dat = IAP_DATA;
    IapIdle();

    EA = ea_save;
    return dat;
}

static void IapProgramByte(uint32 addr, uint8 dat)
{
    bit ea_save = EA;
    EA = 0;

    IAP_CONTR = ENABLE_IAP;
    IAP_TPS   = IAP_TPS_VAL;
    IAP_CMD   = CMD_PROGRAM;

    IAP_ADDRL = (uint8)(addr & 0xFF);
    IAP_ADDRH = (uint8)((addr >> 8) & 0xFF);
    IAP_DATA  = dat;

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_(); _nop_();

    IapIdle();
    EA = ea_save;
}

static void IapEraseSector(uint32 addr)
{
    bit ea_save = EA;
    EA = 0;

    IAP_CONTR = ENABLE_IAP;
    IAP_TPS   = IAP_TPS_VAL;
    IAP_CMD   = CMD_ERASE;

    IAP_ADDRL = (uint8)(addr & 0xFF);
    IAP_ADDRH = (uint8)((addr >> 8) & 0xFF);

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_(); _nop_();

    IapIdle();
    EA = ea_save;
}

// =================================================================
// 参数有效性检查（防止读到垃圾浮点）
// =================================================================
static uint8 Param_IsValid(const SysParam_t *p)
{
    if(p->flag != 0x55AA) return 0;

    // 粗范围：别让垃圾值把车直接打飞
    if(p->balance_kp < 0.0f || p->balance_kp > 2000.0f) return 0;
    if(p->balance_kd < 0.0f || p->balance_kd > 200.0f)  return 0;

    if(p->velocity_kp < 0.0f || p->velocity_kp > 50.0f)  return 0;
    if(p->velocity_ki < 0.0f || p->velocity_ki > 5.0f)   return 0;

    if(p->turn_kp < 0.0f || p->turn_kp > 500.0f)          return 0;
    if(p->turn_kd < 0.0f || p->turn_kd > 500.0f)          return 0;

    if(p->track_speed < -2000 || p->track_speed > 2000)   return 0;

    if(p->mech_zero_pitch < -45.0f || p->mech_zero_pitch > 45.0f) return 0;

    return 1;
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

    // 只读
    for(i = 0; i < (uint16)sizeof(SysParam_t); i++)
    {
        p[i] = IapReadByte(PARAM_FLASH_ADDR + (uint32)i);
    }

    // 校验失败：加载默认（默认不写 Flash，避免 BOOT 卡死）
    if(!Param_IsValid(&g_sys_param))
    {
        Param_SetDefaults();
#if PARAM_AUTOSAVE_ON_FIRST_BOOT
        Param_Save();
#endif
    }
}

void Param_Save(void)
{
    uint16 i;
    uint8 *p = (uint8*)&g_sys_param;

    // 确保 flag 正确
    g_sys_param.flag = 0x55AA;

    // 擦一扇区再写入
    IapEraseSector(PARAM_FLASH_ADDR);

    for(i = 0; i < (uint16)sizeof(SysParam_t); i++)
    {
        IapProgramByte(PARAM_FLASH_ADDR + (uint32)i, p[i]);
    }
}
