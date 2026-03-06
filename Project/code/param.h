#ifndef __PARAM_H__
#define __PARAM_H__

#include "zf_common_headfile.h"

// ================================== 对外宏定义 ==================================
// 参数存储采用 Flash 双备份（slot0 / slot1），单页 512B

// Flash 布局
#define PARAM_SLOT_SIZE_BYTES     (512UL)              // 每个 slot 大小
#define PARAM_SLOT0_ADDR          (0x0400UL)            // slot0 基地址
#define PARAM_SLOT1_ADDR          (0x0600UL)            // slot1 基地址

// 存储头标识
#define PARAM_STORE_MAGIC         (0x55AAU)             // 有效标记
#define PARAM_STORE_VERSION       (0x0004U)             // 当前版本号（v4）
#define PARAM_STORE_VERSION_V3    (0x0003U)             // 历史版本 v3
#define PARAM_STORE_VERSION_V2    (0x0002U)             // 历史版本 v2
#define PARAM_STORE_VERSION_V1    (0x0001U)             // 历史版本 v1

// ================================== 对外类型定义 ==================================
typedef struct
{
    uint8 start_mode;              // 启动模式（1~5）

    // 角速度环（1ms）
    float rate_kp;                 // 角速度环 P
    float rate_ki;                 // 角速度环 I
    float rate_kd;                 // 角速度环 D

    // 角度环（5ms）
    float angle_kp;                // 角度环 P
    float angle_ki;                // 角度环 I
    float angle_kd;                // 角度环 D

    // 速度环（20ms）
    float speed_kp;                // 速度环 P
    float speed_ki;                // 速度环 I
    float speed_kd;                // 速度环 D

    // 转向：PPDD 参数（turn_out = kp*e + kp2*|e|*e + kd*de + kd2*gyro_z）
    float turn_kp;                 // 转向 P
    float turn_kp2;                // 转向 P2（非线性项）
    float turn_kd;                 // 转向 D
    float turn_kd2;                // 转向 D2（角速度项）

    // 机械零点（pitch 偏置）
    float mech_zero_pitch;         // 机械零点角度

    // Mode4：录制数据（直线录距 + yaw_end，仅用于显示）
    int32 mode4_dist_total;        // 录制总距离（counts）
    int32 mode4_yaw_end_mdeg;      // 录制结束航向（mdeg）

} SysParam_t;

// ================================== 对外变量声明 ==================================
extern SysParam_t g_sys_param;                         // 全局系统参数实例

extern uint8 g_param_last_load_ok;                     // 上次加载结果（1=成功）
extern uint8 g_param_last_save_ok;                     // 上次保存结果（1=成功）

// ================================== 对外 API 声明 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介      参数模块初始化
// 参数说明      void
// 返回参数      void
// 使用示例      Param_Init();
// 备注信息      上电调用一次；加载 Flash，失败时写入默认值
//--------------------------------------------------------------------------------------------------
void  Param_Init(void);

//--------------------------------------------------------------------------------------------------
// 函数简介      将当前参数保存到 Flash
// 参数说明      void
// 返回参数      void
// 使用示例      Param_Save();
// 备注信息      交替写入双备份 slot，并在写入后执行校验
//--------------------------------------------------------------------------------------------------
void  Param_Save(void);

//--------------------------------------------------------------------------------------------------
// 函数简介      从 Flash 加载参数
// 参数说明      void
// 返回参数      uint8   1=成功  0=无有效数据
// 使用示例      ok = Param_Load();
// 备注信息      自动选择最新 slot，支持 v1~v4 版本迁移
//--------------------------------------------------------------------------------------------------
uint8 Param_Load(void);

//--------------------------------------------------------------------------------------------------
// 函数简介      恢复出厂默认参数
// 参数说明      void
// 返回参数      void
// 使用示例      Param_SetDefaults();
// 备注信息      仅写 g_sys_param，不写 Flash
//--------------------------------------------------------------------------------------------------
void  Param_SetDefaults(void);

//--------------------------------------------------------------------------------------------------
// 函数简介      将参数应用到平衡级联 PID
// 参数说明      void
// 返回参数      void
// 使用示例      Param_ApplyToBalanceCascade();
// 备注信息      需在 balance_cascade_init() 之后调用
//--------------------------------------------------------------------------------------------------
void  Param_ApplyToBalanceCascade(void);

#endif