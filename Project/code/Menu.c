#include "Menu.h"


// ================================== 内部状态变量 ==================================
static uint8 s_menu_active = 0;  // 0=主页面  1=菜单页面
static uint8 s_menu_index  = 0;  // 当前菜单项索引 0..MENU_ITEM_MAX-1
static uint8 s_edit_active = 0;  // 0=浏览  1=编辑

static uint8 s_dirty_page  = 1;  // 菜单页面需要重绘标志
static uint8 s_dirty_value = 1;  // 菜单数值需要重绘标志
static uint8 s_home_dirty  = 1;  // 主页面需要重绘标志

// ================================== 内部调用函数（显示辅助） ==================================
//--------------------------------------------------------------------------------
// 函数简介      清空指定 OLED 行
// 参数说明      line            行号（1~4）
// 返回参数      void
// 使用示例      oled_clear_line(2);
// 备注信息      用空格覆盖整行（16 列），实现行清除
//--------------------------------------------------------------------------------
static void oled_clear_line(uint8 line)
{
    OLED_ShowString(line, 1, "                ");
}

//--------------------------------------------------------------------------------
// 函数简介      uint32 转十进制字符串
// 参数说明      v               待转换的无符号整数
//              out             输出字符串缓冲区
// 返回参数      void
// 使用示例      u32_to_str(123, buf);
// 备注信息      将 uint32 转换为十进制 ASCII 字符串
//--------------------------------------------------------------------------------
static void u32_to_str(uint32 v, char *out)
{
    char tmp[11];
    uint8 i;
    uint8 n;

    i = 0;
    if (v == 0)
    {
        out[0] = '0';
        out[1] = '\0';
        return;
    }

    while (v && i < 10)
    {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }

    n = 0;
    while (i)
    {
        out[n++] = tmp[--i];
    }
    out[n] = '\0';
}

//--------------------------------------------------------------------------------
// 函数简介      浮点数转定点字符串
// 参数说明      v               待转换的浮点数
//              dec             小数位数
//              out             输出字符串缓冲区
// 返回参数      void
// 使用示例      float_to_str_fixed(-3.14f, 2, buf);
// 备注信息      将浮点数转换为固定小数位数的 ASCII 字符串
//--------------------------------------------------------------------------------
static void float_to_str_fixed(float v, uint8 dec, char *out)
{
    float av;
    uint32 mul;
    uint8 i;
    int32 ip;
    int32 fp;
    char intbuf[12];
    uint8 pos;
    uint32 pow10;

    av = v;
    if (av < 0.0f) av = -av;

    mul = 1;
    for (i = 0; i < dec; i++) mul *= 10;

    ip = (int32)av;
    fp = (int32)((av - (float)ip) * (float)mul + 0.5f);

    if (fp >= (int32)mul)
    {
        fp -= (int32)mul;
        ip += 1;
    }

    pos = 0;
    out[pos++] = (v < 0.0f) ? '-' : ' ';

    u32_to_str((uint32)ip, intbuf);
    i = 0;
    while (intbuf[i] != '\0')
    {
        out[pos++] = intbuf[i++];
    }

    out[pos++] = '.';

    pow10 = mul / 10;
    for (i = 0; i < dec; i++)
    {
        out[pos++] = (char)('0' + (fp / (int32)pow10));
        fp = (int32)(fp % (int32)pow10);
        if (pow10 > 1) pow10 /= 10;
    }

    out[pos] = '\0';
}

//--------------------------------------------------------------------------------
// 函数简介      OLED 显示键值对（浮点）
// 参数说明      row             行号（1~4）
//              k               键名字符串
//              v               浮点数值
//              dec             小数位数
// 返回参数      void
// 使用示例      oled_show_kv_float(3, "Kp=", 1.5f, 2);
// 备注信息      在 OLED 指定行显示 "键名=浮点值" 格式
//--------------------------------------------------------------------------------
static void oled_show_kv_float(uint8 row, const char *k, float v, uint8 dec)
{
    char buf[17];
    char vbuf[16];
    uint8 i;
    uint8 pos;

    for (i = 0; i < 16; i++) buf[i] = ' ';
    buf[16] = '\0';

    pos = 0;
    while (*k && pos < 16)
    {
        buf[pos++] = *k++;
    }

    float_to_str_fixed(v, dec, vbuf);

    pos = 6;
    i = 0;
    while (vbuf[i] != '\0' && pos < 16)
    {
        buf[pos++] = vbuf[i++];
    }

    OLED_ShowString(row, 1, buf);
}

//--------------------------------------------------------------------------------
// 函数简介      OLED 显示键值对（整数）
// 参数说明      row             行号（1~4）
//              k               键名字符串
//              v               uint8 数值
// 返回参数      void
// 使用示例      oled_show_kv_u8(3, "Mode=", 2);
// 备注信息      在 OLED 指定行显示 "键名=整数值" 格式
//--------------------------------------------------------------------------------
static void oled_show_kv_u8(uint8 row, const char *k, uint8 v)
{
    char buf[17];
    char vbuf[6];
    uint8 i;
    uint8 pos;

    for (i = 0; i < 16; i++) buf[i] = ' ';
    buf[16] = '\0';

    pos = 0;
    while (*k && pos < 16)
    {
        buf[pos++] = *k++;
    }

    u32_to_str((uint32)v, vbuf);

    pos = 6;
    i = 0;
    while (vbuf[i] != '\0' && pos < 16)
    {
        buf[pos++] = vbuf[i++];
    }

    OLED_ShowString(row, 1, buf);
}

// ================================== 内部调用函数（主页面） ==================================
//--------------------------------------------------------------------------------
// 函数简介      绘制主页面
// 参数说明      void
// 返回参数      void
// 使用示例      home_draw();
// 备注信息      绘制主页面：BALANCE CAR / OK=RUN / BK=MENU / MODE + RUN/STOP
//--------------------------------------------------------------------------------
static void home_draw(void)
{
    char buf[17];

    OLED_Clear();

    // 第 1 行
    OLED_ShowString(1, 1, "BALANCE CAR");

    // 第 2 行
    OLED_ShowString(2, 1, "OK=RUN");

    // 第 3 行
    OLED_ShowString(3, 1, "BK=MENU");

    // 第 4 行：MODE + RUN/STOP
    for (buf[0] = 0; buf[0] < 0; ) {;} // 占位语句，用于避免特定编译器告警（无副作用）
    oled_clear_line(4);

    OLED_ShowString(4, 1, "MODE:");
    OLED_ShowNum(4, 6, (int32)g_sys_param.start_mode);

    OLED_ShowString(4, 9, (run_flag ? "RUN " : "STOP"));

    s_home_dirty = 0;
}

// ================================== 内部调用函数（菜单页） ==================================
//--------------------------------------------------------------------------------
// 函数简介      获取菜单项名称
// 参数说明      idx             菜单项索引
// 返回参数      const char*     菜单项名称字符串
// 使用示例      const char *n = menu_item_name(MENU_ITEM_MODE);
// 备注信息      根据索引返回菜单项显示名称
//--------------------------------------------------------------------------------
static const char* menu_item_name(uint8 idx)
{
    switch ((MenuItem_e)idx)
    {
        case MENU_ITEM_MODE:      return "StartMode";

        case MENU_ITEM_RATE_KP:   return "Rate Kp";
        case MENU_ITEM_RATE_KI:   return "Rate Ki";
        case MENU_ITEM_RATE_KD:   return "Rate Kd";

        case MENU_ITEM_ANGLE_KP:  return "AngleKp";
        case MENU_ITEM_ANGLE_KI:  return "AngleKi";
        case MENU_ITEM_ANGLE_KD:  return "AngleKd";

        case MENU_ITEM_SPEED_KP:  return "SpeedKp";
        case MENU_ITEM_SPEED_KI:  return "SpeedKi";
        case MENU_ITEM_SPEED_KD:  return "SpeedKd";

        case MENU_ITEM_TURN_KP:   return "TurnKp";
        case MENU_ITEM_TURN_KP2:  return "TurnKp2";
        case MENU_ITEM_TURN_KD:   return "TurnKd";
        case MENU_ITEM_TURN_KD2:  return "TurnKd2";

        case MENU_ITEM_MECH_ZERO: return "ZeroPitch";
        case MENU_ITEM_SAVE:      return "SAVE";

        default:                  return "NA";
    }
}

//--------------------------------------------------------------------------------
// 函数简介      绘制菜单页面框架
// 参数说明      void
// 返回参数      void
// 使用示例      menu_draw_page();
// 备注信息      绘制菜单页面：标题行 + 当前项名称 + 底部按键提示
//--------------------------------------------------------------------------------
static void menu_draw_page(void)
{
    uint8 page;

    page = (uint8)(s_menu_index + 1);

    OLED_Clear();

    if (s_edit_active) OLED_ShowString(1, 1, "EDIT Pg:");
    else               OLED_ShowString(1, 1, "MENU Pg:");

    OLED_ShowNum(1, 9, (int32)page);
    OLED_ShowChar(1, 11, '/');
    OLED_ShowNum(1, 12, (int32)(MENU_ITEM_MAX));

    oled_clear_line(2);
    OLED_ShowString(2, 1, (char*)menu_item_name(s_menu_index));

    OLED_ShowString(4, 1, "UP DN OK BK");

    s_dirty_value = 1;
}

//--------------------------------------------------------------------------------
// 函数简介      绘制当前菜单项数值
// 参数说明      void
// 返回参数      void
// 使用示例      menu_draw_value();
// 备注信息      在第 3 行显示当前菜单项对应的参数数值
//--------------------------------------------------------------------------------
static void menu_draw_value(void)
{
    switch ((MenuItem_e)s_menu_index)
    {
        case MENU_ITEM_MODE:
            oled_show_kv_u8(3, "Mode=", g_sys_param.start_mode);
            break;

        case MENU_ITEM_RATE_KP:
            oled_show_kv_float(3, "Kp=", g_sys_param.rate_kp, 2);
            break;
        case MENU_ITEM_RATE_KI:
            oled_show_kv_float(3, "Ki=", g_sys_param.rate_ki, 4);
            break;
        case MENU_ITEM_RATE_KD:
            oled_show_kv_float(3, "Kd=", g_sys_param.rate_kd, 2);
            break;

        case MENU_ITEM_ANGLE_KP:
            oled_show_kv_float(3, "Kp=", g_sys_param.angle_kp, 2);
            break;
        case MENU_ITEM_ANGLE_KI:
            oled_show_kv_float(3, "Ki=", g_sys_param.angle_ki, 4);
            break;
        case MENU_ITEM_ANGLE_KD:
            oled_show_kv_float(3, "Kd=", g_sys_param.angle_kd, 2);
            break;

        case MENU_ITEM_SPEED_KP:
            oled_show_kv_float(3, "Kp=", g_sys_param.speed_kp, 2);
            break;
        case MENU_ITEM_SPEED_KI:
            oled_show_kv_float(3, "Ki=", g_sys_param.speed_ki, 3);
            break;
        case MENU_ITEM_SPEED_KD:
            oled_show_kv_float(3, "Kd=", g_sys_param.speed_kd, 2);
            break;

        case MENU_ITEM_TURN_KP:
            oled_show_kv_float(3, "Kp=", g_sys_param.turn_kp, 2);
            break;
        case MENU_ITEM_TURN_KP2:
            oled_show_kv_float(3, "K2=", g_sys_param.turn_kp2, 4);
            break;
        case MENU_ITEM_TURN_KD:
            oled_show_kv_float(3, "Kd=", g_sys_param.turn_kd, 2);
            break;
        case MENU_ITEM_TURN_KD2:
            oled_show_kv_float(3, "D2=", g_sys_param.turn_kd2, 4);
            break;

        case MENU_ITEM_MECH_ZERO:
            oled_show_kv_float(3, "Z=", g_sys_param.mech_zero_pitch, 2);
            break;

        case MENU_ITEM_SAVE:
            oled_clear_line(3);
            OLED_ShowString(3, 1, "OK=Save");
            break;

        default:
            oled_clear_line(3);
            OLED_ShowString(3, 1, "NA");
            break;
    }
}

//--------------------------------------------------------------------------------
// 函数简介      调整当前菜单项参数
// 参数说明      dir             调参方向（+1=增大  -1=减小）
//              fast            是否快速步进（长按重复时为 1）
// 返回参数      void
// 使用示例      menu_adjust_param(+1, 0);
// 备注信息      根据当前菜单项和方向/速度调整对应参数，调参后立即生效
//--------------------------------------------------------------------------------
static void menu_adjust_param(int8 dir, uint8 fast)
{
    float step_big;
    float step_small;
    float turn_kp_step;
    float turn_kp2_step;
    float turn_kd_step;
    float turn_kd2_step;

    step_big   = fast ? 1.0f : 0.1f;
    step_small = fast ? 0.01f : 0.001f;

    turn_kp_step  = fast ? 2.0f   : 0.2f;
    turn_kp2_step = fast ? 0.002f : 0.0002f;
    turn_kd_step  = fast ? 1.0f   : 0.1f;
    turn_kd2_step = fast ? 0.02f  : 0.002f;

    switch ((MenuItem_e)s_menu_index)
    {
        case MENU_ITEM_MODE:
            if (dir > 0)
            {
                if (g_sys_param.start_mode < 5) g_sys_param.start_mode++;
            }
            else
            {
                if (g_sys_param.start_mode > 1) g_sys_param.start_mode--;
            }
            break;

        case MENU_ITEM_RATE_KP:
            g_sys_param.rate_kp += (dir > 0) ? (fast ? 0.5f : 0.1f) : (fast ? -0.5f : -0.1f);
            if (g_sys_param.rate_kp < 0.0f) g_sys_param.rate_kp = 0.0f;
            break;
        case MENU_ITEM_RATE_KI:
            g_sys_param.rate_ki += (dir > 0) ? step_small : -step_small;
            if (g_sys_param.rate_ki < 0.0f) g_sys_param.rate_ki = 0.0f;
            break;
        case MENU_ITEM_RATE_KD:
            g_sys_param.rate_kd += (dir > 0) ? (fast ? 0.5f : 0.1f) : (fast ? -0.5f : -0.1f);
            if (g_sys_param.rate_kd < 0.0f) g_sys_param.rate_kd = 0.0f;
            break;

        case MENU_ITEM_ANGLE_KP:
            g_sys_param.angle_kp += (dir > 0) ? (fast ? 0.5f : 0.1f) : (fast ? -0.5f : -0.1f);
            if (g_sys_param.angle_kp < 0.0f) g_sys_param.angle_kp = 0.0f;
            break;
        case MENU_ITEM_ANGLE_KI:
            g_sys_param.angle_ki += (dir > 0) ? step_small : -step_small;
            if (g_sys_param.angle_ki < 0.0f) g_sys_param.angle_ki = 0.0f;
            break;
        case MENU_ITEM_ANGLE_KD:
            g_sys_param.angle_kd += (dir > 0) ? (fast ? 0.5f : 0.1f) : (fast ? -0.5f : -0.1f);
            if (g_sys_param.angle_kd < 0.0f) g_sys_param.angle_kd = 0.0f;
            break;

        case MENU_ITEM_SPEED_KP:
            g_sys_param.speed_kp += (dir > 0) ? (fast ? 0.5f : 0.1f) : (fast ? -0.5f : -0.1f);
            if (g_sys_param.speed_kp < 0.0f) g_sys_param.speed_kp = 0.0f;
            break;
        case MENU_ITEM_SPEED_KI:
            g_sys_param.speed_ki += (dir > 0) ? (fast ? 0.05f : 0.01f) : (fast ? -0.05f : -0.01f);
            if (g_sys_param.speed_ki < 0.0f) g_sys_param.speed_ki = 0.0f;
            break;
        case MENU_ITEM_SPEED_KD:
            g_sys_param.speed_kd += (dir > 0) ? (fast ? 0.5f : 0.1f) : (fast ? -0.5f : -0.1f);
            if (g_sys_param.speed_kd < 0.0f) g_sys_param.speed_kd = 0.0f;
            break;

        case MENU_ITEM_MECH_ZERO:
            g_sys_param.mech_zero_pitch += (dir > 0) ? step_big : -step_big;
            break;

        case MENU_ITEM_TURN_KP:
            g_sys_param.turn_kp += (dir > 0) ? turn_kp_step : -turn_kp_step;
            break;
        case MENU_ITEM_TURN_KP2:
            g_sys_param.turn_kp2 += (dir > 0) ? turn_kp2_step : -turn_kp2_step;
            break;
        case MENU_ITEM_TURN_KD:
            g_sys_param.turn_kd += (dir > 0) ? turn_kd_step : -turn_kd_step;
            break;
        case MENU_ITEM_TURN_KD2:
            g_sys_param.turn_kd2 += (dir > 0) ? turn_kd2_step : -turn_kd2_step;
            break;

        default:
            break;
    }

    // 参数调整后立即生效
    Param_ApplyToBalanceCascade();

    s_dirty_value = 1;
    s_home_dirty  = 1; // 主页面需同步刷新 MODE 显示
}

// ================================== 对外 API 函数 ==================================
//--------------------------------------------------------------------------------
// 函数简介      菜单模块初始化
// 参数说明      void
// 返回参数      void
// 使用示例      Menu_Init();
// 备注信息      上电后初始化菜单模块，进入主页面，并将 run_flag 清零
//--------------------------------------------------------------------------------
void Menu_Init(void)
{
    s_menu_active = 0;
    s_menu_index  = 0;
    s_edit_active = 0;

    s_dirty_page  = 1;
    s_dirty_value = 1;
    s_home_dirty  = 1;

    // 上电后 run_flag 清零，需按 OK 才进入 RUN 状态
    run_flag = 0;

    home_draw();
}

//--------------------------------------------------------------------------------
// 函数简介      进入菜单页面
// 参数说明      void
// 返回参数      void
// 使用示例      Menu_Enter();
// 备注信息      进入菜单页面，置停电机输出，重绘菜单页面
//--------------------------------------------------------------------------------
void Menu_Enter(void)
{
    // 进入菜单时关闭电机输出
    run_flag = 0;

    s_menu_active = 1;
    s_menu_index  = 0;
    s_edit_active = 0;
    s_dirty_page  = 1;
    s_dirty_value = 1;

    menu_draw_page();
    menu_draw_value();
}

//--------------------------------------------------------------------------------
// 函数简介      退出菜单页面
// 参数说明      void
// 返回参数      void
// 使用示例      Menu_Exit();
// 备注信息      退出菜单回到主页面，置停电机输出
//--------------------------------------------------------------------------------
void Menu_Exit(void)
{
    // 退出菜单并返回主页面时关闭电机输出
    run_flag = 0;

    s_menu_active = 0;
    s_edit_active = 0;

    s_home_dirty = 1;
    home_draw();
}

//--------------------------------------------------------------------------------
// 函数简介      菜单周期任务
// 参数说明      void
// 返回参数      uint8           0=当前在菜单页  1=当前在主页面
// 使用示例      uint8 page = Menu_Task();
// 备注信息      UI 周期任务：主页面 OK=RUN/BK=MENU；菜单页 UP/DN/OK/BK
//--------------------------------------------------------------------------------
uint8 Menu_Task(void)
{
    uint8 up_evt;
    uint8 dn_evt;
    uint8 up_rep;
    uint8 dn_rep;
    uint8 fast;

    // 主页面逻辑
    if (!s_menu_active)
    {
        // Mode4 激活时接管 OLED 显示与 OK/BK 键处理
        if (Mode4_IsActive())
        {
            Mode4_Task_5ms();
            return 1;
        }

        if (s_home_dirty) home_draw();

        // OK 键启动运行
        if (Key_Check(KEY_NAME_CONFIRM, KEY_SINGLE))
        {
            // 启动前清零所有 PID 环状态，避免残留积分/微分导致启动异常
            balance_cascade.angular_speed_cycle.i_value = 0.0f;
            balance_cascade.angular_speed_cycle.p_value_last = 0.0f;
            balance_cascade.angular_speed_cycle.incremental_data[0] = 0.0f;
            balance_cascade.angular_speed_cycle.incremental_data[1] = 0.0f;

            balance_cascade.angle_cycle.i_value = 0.0f;
            balance_cascade.angle_cycle.p_value_last = 0.0f;
            balance_cascade.angle_cycle.incremental_data[0] = 0.0f;
            balance_cascade.angle_cycle.incremental_data[1] = 0.0f;

            balance_cascade.speed_cycle.i_value = 0.0f;
            balance_cascade.speed_cycle.p_value_last = 0.0f;
            balance_cascade.speed_cycle.incremental_data[0] = 0.0f;
            balance_cascade.speed_cycle.incremental_data[1] = 0.0f;

            run_flag = 1;
            s_home_dirty = 1;

            // start_mode=4 时进入 Mode4 精简界面
            if (g_sys_param.start_mode == 4u)
            {
                Mode4_Enter();
            }
        }

        // Back 键进入菜单页面
        if (Key_Check(KEY_NAME_BACK, KEY_SINGLE))
        {
            Menu_Enter();
        }

        return 1;
    }

    // 菜单页面逻辑
    up_evt = Key_Check(KEY_NAME_UP, KEY_SINGLE);
    dn_evt = Key_Check(KEY_NAME_DOWN, KEY_SINGLE);
    up_rep = Key_Check(KEY_NAME_UP, KEY_REPEAT);
    dn_rep = Key_Check(KEY_NAME_DOWN, KEY_REPEAT);

    // BACK：退出编辑或返回主页面
    if (Key_Check(KEY_NAME_BACK, KEY_SINGLE))
    {
        if (s_edit_active)
        {
            s_edit_active = 0;
            s_dirty_page = 1;
        }
        else
        {
            Menu_Exit();
            return 1;
        }
    }

    // CONFIRM：切换编辑状态或执行保存
    if (Key_Check(KEY_NAME_CONFIRM, KEY_SINGLE))
    {
        if ((MenuItem_e)s_menu_index == MENU_ITEM_SAVE)
        {
            Param_Save();
            Param_ApplyToBalanceCascade();
            system_delay_ms(500);
            s_dirty_value = 1;
        }
        else
        {
            s_edit_active = (uint8)(!s_edit_active);
            s_dirty_page = 1;
        }
    }

    if (up_evt || dn_evt || up_rep || dn_rep)
    {
        fast = (uint8)((up_rep || dn_rep) ? 1 : 0);

        if (!s_edit_active)
        {
            if (up_evt || up_rep)
            {
                if (s_menu_index == 0) s_menu_index = (uint8)(MENU_ITEM_MAX - 1);
                else s_menu_index--;
                s_dirty_page = 1;
            }
            if (dn_evt || dn_rep)
            {
                if (s_menu_index >= (uint8)(MENU_ITEM_MAX - 1)) s_menu_index = 0;
                else s_menu_index++;
                s_dirty_page = 1;
            }
        }
        else
        {
            if (up_evt || up_rep) menu_adjust_param(+1, fast);
            if (dn_evt || dn_rep) menu_adjust_param(-1, fast);
        }
    }

    if (s_dirty_page)
    {
        s_dirty_page = 0;
        menu_draw_page();
    }
    if (s_dirty_value)
    {
        s_dirty_value = 0;
        menu_draw_value();
    }

    return 0;
}
