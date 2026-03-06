#include "param.h"

// ================================== 对外变量定义 ==================================
SysParam_t g_sys_param;                               // 全局系统参数实例

uint8 g_param_last_load_ok = 0;                        // 上次加载结果（1=成功）
uint8 g_param_last_save_ok = 0;                        // 上次保存结果（1=成功）

// ================================== 内部宏定义 ==================================

#define PARAM_PAYLOAD_MAX_BYTES   (256u)                // 负载缓冲区最大字节数

// ================================== 内部类型定义 ==================================

typedef struct
{
    uint16 magic;
    uint16 version;
    uint16 length;
    uint16 seq;
    uint16 crc16;
    uint16 rsv;

    uint8  payload[PARAM_PAYLOAD_MAX_BYTES];

} ParamStore_t;

typedef struct
{
    uint8 start_mode;

    float rate_kp;
    float rate_ki;
    float rate_kd;

    float angle_kp;
    float angle_ki;
    float angle_kd;

    float speed_kp;
    float speed_ki;
    float speed_kd;

    float mech_zero_pitch;

} SysParam_v1_t;

typedef struct
{
    uint8 start_mode;

    float mid_angle;

    float rate_kp;
    float rate_ki;
    float rate_kd;

    float angle_kp;
    float angle_ki;
    float angle_kd;

    float speed_kp;
    float speed_ki;
    float speed_kd;

    float pos_kp;
    float pos_ki;

    float turn_kp;
    float turn_kp2;
    float turn_kd;
    float turn_kd2;

    float mech_zero_pitch;

} SysParam_v2_t;

typedef struct
{
    uint8 start_mode;

    float rate_kp;
    float rate_ki;
    float rate_kd;

    float angle_kp;
    float angle_ki;
    float angle_kd;

    float speed_kp;
    float speed_ki;
    float speed_kd;

    float turn_kp;
    float turn_kp2;
    float turn_kd;
    float turn_kd2;

    float mech_zero_pitch;

} SysParam_v3_t;


// ================================== 内部状态变量 ==================================

static uint16 s_last_seq  = 0;                         // 最新有效序号
static uint8  s_last_slot = 0;                         // 最新有效槽号（0/1）

// 两个槽的读取缓冲（使用全局静态区，减少栈占用）
static ParamStore_t s_slot0;                           // slot0 读取缓冲
static ParamStore_t s_slot1;                           // slot1 读取缓冲

// 写入与回读校验缓冲（使用全局静态区，降低 8051 栈压力）
static ParamStore_t s_wr;                              // 写入缓冲
static ParamStore_t s_rd_verify;                       // 回读校验缓冲

// ================================== 内部调用函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介      CRC16-CCITT 校验计算
// 参数说明      buf     数据缓冲区指针
//              len     数据长度（字节）
// 返回参数      uint16  CRC16 校验值
// 使用示例      crc = param_crc16_ccitt(payload, sizeof(payload));
// 备注信息      多项式 0x1021，初始值 0xFFFF
//--------------------------------------------------------------------------------------------------
static uint16 param_crc16_ccitt(uint8 *buf, uint16 len)
{
    uint16 crc;
    uint16 i;

    crc = 0xFFFF;
    while (len--)
    {
        crc ^= (uint16)(*buf++) << 8;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x8000) crc = (uint16)((crc << 1) ^ 0x1021);
            else              crc <<= 1;
        }
    }
    return crc;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      浮点数有限性检测
// 参数说明      x       待检测浮点数
// 返回参数      uint8   1=有限  0=NaN
// 使用示例      if (!param_is_finite(val)) val = 0.0f;
// 备注信息      NaN != NaN
//--------------------------------------------------------------------------------------------------
static uint8 param_is_finite(float x)
{
    // 简单 NaN 检测：NaN != NaN
    if (x != x) return 0;
    return 1;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      参数有效性清理
// 参数说明      p       系统参数结构体指针
// 返回参数      void
// 使用示例      param_sanitize(&g_sys_param);
// 备注信息      start_mode 限定为 [1,5]；PID NaN/负值置 0；转向参数仅处理 NaN
//--------------------------------------------------------------------------------------------------
static void param_sanitize(SysParam_t *p)
{
    // start_mode
    if (p->start_mode < 1) p->start_mode = 1;
    if (p->start_mode > 5) p->start_mode = 5;

    // PID：NaN -> 0，负值 -> 0
    if (!param_is_finite(p->rate_kp)  || p->rate_kp  < 0.0f) p->rate_kp  = 0.0f;
    if (!param_is_finite(p->rate_ki)  || p->rate_ki  < 0.0f) p->rate_ki  = 0.0f;
    if (!param_is_finite(p->rate_kd)  || p->rate_kd  < 0.0f) p->rate_kd  = 0.0f;

    if (!param_is_finite(p->angle_kp) || p->angle_kp < 0.0f) p->angle_kp = 0.0f;
    if (!param_is_finite(p->angle_ki) || p->angle_ki < 0.0f) p->angle_ki = 0.0f;
    if (!param_is_finite(p->angle_kd) || p->angle_kd < 0.0f) p->angle_kd = 0.0f;

    if (!param_is_finite(p->speed_kp) || p->speed_kp < 0.0f) p->speed_kp = 0.0f;
    if (!param_is_finite(p->speed_ki) || p->speed_ki < 0.0f) p->speed_ki = 0.0f;
    if (!param_is_finite(p->speed_kd) || p->speed_kd < 0.0f) p->speed_kd = 0.0f;

    // 转向 PPDD：允许正负值，仅进行 NaN 有效性处理
    if (!param_is_finite(p->turn_kp))  p->turn_kp  = 0.0f;
    if (!param_is_finite(p->turn_kp2)) p->turn_kp2 = 0.0f;
    if (!param_is_finite(p->turn_kd))  p->turn_kd  = 0.0f;
    if (!param_is_finite(p->turn_kd2)) p->turn_kd2 = 0.0f;

    if (!param_is_finite(p->mech_zero_pitch)) p->mech_zero_pitch = 0.0f;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      校验 Flash 存储头与 CRC
// 参数说明      ps      ParamStore_t 结构体指针
// 返回参数      uint8   1=校验通过  0=无效
// 使用示例      ok = store_verify(&s_slot0);
// 备注信息      检查 magic / version / length / CRC16，支持 v1~v4
//--------------------------------------------------------------------------------------------------
static uint8 store_verify(ParamStore_t *ps)
{
    uint16 crc_calc;
    uint16 expect_len;

    if (ps->magic != (uint16)PARAM_STORE_MAGIC) return 0;

    // 版本与长度需严格匹配，避免后续字段映射错位
    expect_len = 0;
    if (ps->version == (uint16)PARAM_STORE_VERSION)
    {
        expect_len = (uint16)sizeof(SysParam_t);
    }
    else if (ps->version == (uint16)PARAM_STORE_VERSION_V3)
    {
        expect_len = (uint16)sizeof(SysParam_v3_t);
    }
    else if (ps->version == (uint16)PARAM_STORE_VERSION_V2)
    {
        expect_len = (uint16)sizeof(SysParam_v2_t);
    }
    else if (ps->version == (uint16)PARAM_STORE_VERSION_V1)
    {
        expect_len = (uint16)sizeof(SysParam_v1_t);
    }
    else
    {
        return 0;
    }

    if (ps->length != expect_len) return 0;

    if (expect_len > (uint16)PARAM_PAYLOAD_MAX_BYTES) return 0;

    crc_calc = param_crc16_ccitt((uint8 *)ps->payload, expect_len);
    if (crc_calc != ps->crc16) return 0;

    return 1;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      从 Flash 读取一个 slot
// 参数说明      addr    Flash 地址
//              out     输出缓冲区指针
// 返回参数      void
// 使用示例      store_read_slot(PARAM_SLOT0_ADDR, &s_slot0);
// 备注信息      内部静态函数
//--------------------------------------------------------------------------------------------------
static void store_read_slot(uint32 addr, ParamStore_t *out)
{
    iap_read_buff(addr, (uint8 *)out, (uint16)sizeof(ParamStore_t));
}

//--------------------------------------------------------------------------------------------------
// 函数简介      写入并校验一个 Flash slot
// 参数说明      addr    Flash 地址
//              in      待写数据指针
// 返回参数      uint8   1=成功  0=校验失败
// 使用示例      ok = store_write_slot(addr, &s_wr);
// 备注信息      执行擦除、写入、延时与回读校验
//--------------------------------------------------------------------------------------------------
static uint8 store_write_slot(uint32 addr, ParamStore_t *in)
{
    iap_erase_page(addr);
    iap_write_buff(addr, (uint8 *)in, (uint16)sizeof(ParamStore_t));
    system_delay_ms(20);

    store_read_slot(addr, &s_rd_verify);
    if (!store_verify(&s_rd_verify)) return 0;

    return 1;
}

// ================================== 对外 API 函数 ==================================

//--------------------------------------------------------------------------------------------------
// 函数简介      从 Flash 加载参数
// 参数说明      void
// 返回参数      uint8   1=成功  0=无有效数据
// 使用示例      ok = Param_Load();
// 备注信息      自动选择最新 slot，支持 v1~v4 版本迁移
//--------------------------------------------------------------------------------------------------
uint8 Param_Load(void)
{
    uint8 ea_bak;
    ParamStore_t *sel;
    uint8 v0;
    uint8 v1;
    uint16 d;

    ea_bak = EA;
    interrupt_global_disable();

    iap_init();

    store_read_slot(PARAM_SLOT0_ADDR, &s_slot0);
    store_read_slot(PARAM_SLOT1_ADDR, &s_slot1);

    iap_idle();

    if (ea_bak)
    {
        interrupt_global_enable();
    }

    v0 = store_verify(&s_slot0);
    v1 = store_verify(&s_slot1);

    if (!v0 && !v1)
    {
        g_param_last_load_ok = 0;
        return 0;
    }

    sel = 0;

    if (v0 && v1)
    {
        // 选择序号更新的记录（处理 16bit 回卷）
        d = (uint16)(s_slot1.seq - s_slot0.seq);
        if (d != 0 && d < 0x8000U)
        {
            sel = &s_slot1;
        }
        else
        {
            sel = &s_slot0;
        }
    }
    else if (v1)
    {
        sel = &s_slot1;
    }
    else
    {
        sel = &s_slot0;
    }

    if (sel == 0)
    {
        g_param_last_load_ok = 0;
        return 0;
    }

    // 记录最新序号与槽位
    s_last_seq  = sel->seq;
    s_last_slot = (sel == &s_slot1) ? 1u : 0u;

    if (sel->version == (uint16)PARAM_STORE_VERSION)
    {
        uint16 i;
        uint8 *dst;
        uint8 *src;

        dst = (uint8 *)&g_sys_param;
        src = (uint8 *)sel->payload;
        for (i = 0; i < (uint16)sizeof(SysParam_t); i++)
        {
            dst[i] = src[i];
        }
    }
    else if (sel->version == (uint16)PARAM_STORE_VERSION_V3)
    {
        SysParam_v3_t *pv3;

        Param_SetDefaults();

        pv3 = (SysParam_v3_t *)sel->payload;

        g_sys_param.start_mode = pv3->start_mode;

        g_sys_param.rate_kp = pv3->rate_kp;
        g_sys_param.rate_ki = pv3->rate_ki;
        g_sys_param.rate_kd = pv3->rate_kd;

        g_sys_param.angle_kp = pv3->angle_kp;
        g_sys_param.angle_ki = pv3->angle_ki;
        g_sys_param.angle_kd = pv3->angle_kd;

        g_sys_param.speed_kp = pv3->speed_kp;
        g_sys_param.speed_ki = pv3->speed_ki;
        g_sys_param.speed_kd = pv3->speed_kd;

        g_sys_param.turn_kp  = pv3->turn_kp;
        g_sys_param.turn_kp2 = pv3->turn_kp2;
        g_sys_param.turn_kd  = pv3->turn_kd;
        g_sys_param.turn_kd2 = pv3->turn_kd2;

        g_sys_param.mech_zero_pitch = pv3->mech_zero_pitch;

        // v3 不包含 mode4 字段，保持默认值（0）
    }
    else if (sel->version == (uint16)PARAM_STORE_VERSION_V2)
    {
        SysParam_v2_t *pv2;

        Param_SetDefaults();

        pv2 = (SysParam_v2_t *)sel->payload;

        g_sys_param.start_mode = pv2->start_mode;

        g_sys_param.rate_kp = pv2->rate_kp;
        g_sys_param.rate_ki = pv2->rate_ki;
        g_sys_param.rate_kd = pv2->rate_kd;

        g_sys_param.angle_kp = pv2->angle_kp;
        g_sys_param.angle_ki = pv2->angle_ki;
        g_sys_param.angle_kd = pv2->angle_kd;

        g_sys_param.speed_kp = pv2->speed_kp;
        g_sys_param.speed_ki = pv2->speed_ki;
        g_sys_param.speed_kd = pv2->speed_kd;

        g_sys_param.turn_kp  = pv2->turn_kp;
        g_sys_param.turn_kp2 = pv2->turn_kp2;
        g_sys_param.turn_kd  = pv2->turn_kd;
        g_sys_param.turn_kd2 = pv2->turn_kd2;

        g_sys_param.mech_zero_pitch = pv2->mech_zero_pitch;

    }
    else
    {
        SysParam_v1_t *pv1;

        Param_SetDefaults();

        pv1 = (SysParam_v1_t *)sel->payload;
        g_sys_param.start_mode = pv1->start_mode;

        g_sys_param.rate_kp = pv1->rate_kp;
        g_sys_param.rate_ki = pv1->rate_ki;
        g_sys_param.rate_kd = pv1->rate_kd;

        g_sys_param.angle_kp = pv1->angle_kp;
        g_sys_param.angle_ki = pv1->angle_ki;
        g_sys_param.angle_kd = pv1->angle_kd;

        g_sys_param.speed_kp = pv1->speed_kp;
        g_sys_param.speed_ki = pv1->speed_ki;
        g_sys_param.speed_kd = pv1->speed_kd;

        g_sys_param.mech_zero_pitch = pv1->mech_zero_pitch;

    }

    param_sanitize(&g_sys_param);
    g_param_last_load_ok = 1;
    return 1;
}

//--------------------------------------------------------------------------------------------------
// 函数简介      将当前参数保存到 Flash
// 参数说明      void
// 返回参数      void
// 使用示例      Param_Save();
// 备注信息      交替写入双备份 slot，写后校验
//--------------------------------------------------------------------------------------------------
void Param_Save(void)
{
    uint32 addr;
    uint16 next_seq;
    uint8 ea_bak;
    uint16 i;
    uint8 *src;

    // 交替写入双备份槽位
    if (s_last_slot == 0)
    {
        addr = PARAM_SLOT1_ADDR;
        s_last_slot = 1;
    }
    else
    {
        addr = PARAM_SLOT0_ADDR;
        s_last_slot = 0;
    }

    next_seq = (uint16)(s_last_seq + 1);

    param_sanitize(&g_sys_param);

    s_wr.magic   = (uint16)PARAM_STORE_MAGIC;
    s_wr.version = (uint16)PARAM_STORE_VERSION;
    s_wr.length  = (uint16)sizeof(SysParam_t);
    s_wr.seq     = next_seq;
    s_wr.rsv     = 0;

    // 先填充 payload，再复制有效字段
    for (i = 0; i < (uint16)PARAM_PAYLOAD_MAX_BYTES; i++)
    {
        s_wr.payload[i] = 0xFFu;
    }

    src = (uint8 *)&g_sys_param;
    for (i = 0; i < (uint16)sizeof(SysParam_t); i++)
    {
        s_wr.payload[i] = src[i];
    }

    s_wr.crc16 = param_crc16_ccitt((uint8 *)s_wr.payload, (uint16)sizeof(SysParam_t));

    ea_bak = EA;
    interrupt_global_disable();

    iap_init();
    g_param_last_save_ok = store_write_slot(addr, &s_wr) ? 1 : 0;
    iap_idle();

    if (ea_bak)
    {
        interrupt_global_enable();
    }

    if (g_param_last_save_ok)
    {
        s_last_seq = next_seq;
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介      恢复出厂默认参数
// 参数说明      void
// 返回参数      void
// 使用示例      Param_SetDefaults();
// 备注信息      仅写 g_sys_param，不写 Flash
//--------------------------------------------------------------------------------------------------
void Param_SetDefaults(void)
{
    g_sys_param.start_mode = 3;

    g_sys_param.rate_kp  = 1.0f;
    g_sys_param.rate_ki  = 0.0f;
    g_sys_param.rate_kd  = 0.0f;

    g_sys_param.angle_kp = 6.5f;
    g_sys_param.angle_ki = 0.0f;
    g_sys_param.angle_kd = 20.0f;

    g_sys_param.speed_kp = 20.0f;
    g_sys_param.speed_ki = 0.236f;
    g_sys_param.speed_kd = 0.0f;

    g_sys_param.turn_kp  = 0.0f;
    g_sys_param.turn_kp2 = 0.0f;
    g_sys_param.turn_kd  = 0.0f;
    g_sys_param.turn_kd2 = 0.0f;

    g_sys_param.mech_zero_pitch = -50.0f;

    g_sys_param.mode4_dist_total = 0;
    g_sys_param.mode4_yaw_end_mdeg = 0;

    param_sanitize(&g_sys_param);
}

//--------------------------------------------------------------------------------------------------
// 函数简介      参数模块初始化
// 参数说明      void
// 返回参数      void
// 使用示例      Param_Init();
// 备注信息      上电调用一次；加载 Flash，失败时写入默认值
//--------------------------------------------------------------------------------------------------
void Param_Init(void)
{
    if (!Param_Load())
    {
        Param_SetDefaults();
        Param_Save();
    }
}

//--------------------------------------------------------------------------------------------------
// 函数简介      将参数应用到平衡级联 PID
// 参数说明      void
// 返回参数      void
// 使用示例      Param_ApplyToBalanceCascade();
// 备注信息      需在 balance_cascade_init() 之后调用
//--------------------------------------------------------------------------------------------------
void Param_ApplyToBalanceCascade(void)
{

    // 角速度环（1ms）
    balance_cascade.angular_speed_cycle.p = g_sys_param.rate_kp;
    balance_cascade.angular_speed_cycle.i = g_sys_param.rate_ki;
    balance_cascade.angular_speed_cycle.d = g_sys_param.rate_kd;

    balance_cascade_resave.angular_speed_cycle.p = g_sys_param.rate_kp;
    balance_cascade_resave.angular_speed_cycle.i = g_sys_param.rate_ki;
    balance_cascade_resave.angular_speed_cycle.d = g_sys_param.rate_kd;

    // 角度环（5ms）
    balance_cascade.angle_cycle.p = g_sys_param.angle_kp;
    balance_cascade.angle_cycle.i = g_sys_param.angle_ki;
    balance_cascade.angle_cycle.d = g_sys_param.angle_kd;

    balance_cascade_resave.angle_cycle.p = g_sys_param.angle_kp;
    balance_cascade_resave.angle_cycle.i = g_sys_param.angle_ki;
    balance_cascade_resave.angle_cycle.d = g_sys_param.angle_kd;

    // 速度环（20ms）
    balance_cascade.speed_cycle.p = g_sys_param.speed_kp;
    balance_cascade.speed_cycle.i = g_sys_param.speed_ki;
    balance_cascade.speed_cycle.d = g_sys_param.speed_kd;

    balance_cascade_resave.speed_cycle.p = g_sys_param.speed_kp;
    balance_cascade_resave.speed_cycle.i = g_sys_param.speed_ki;
    balance_cascade_resave.speed_cycle.d = g_sys_param.speed_kd;

    // 机械零点
    balance_cascade.cascade_value.mechanical_zero = g_sys_param.mech_zero_pitch;
    balance_cascade_resave.cascade_value.mechanical_zero = g_sys_param.mech_zero_pitch;
}
