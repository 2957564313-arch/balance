#include "menu.h"
#include "OLED.h"
#include "Key.h"
#include "param.h"
#include "buzzer.h"
#include <stdarg.h> // 必须包含！用于 Menu_Connect

// =================================================================
// 1. 定义所有节点 (所有“文件夹”和“文件”)
// =================================================================

// [Level 1] 根目录 (主菜单)
static MenuNode_t m_pid, m_track, m_system;

// [Level 2] PID 设置文件夹内的文件
static MenuNode_t s_bal_kp, s_bal_kd, s_vel_kp, s_vel_ki, s_turn_kp;

// [Level 2] 循迹设置文件夹内的文件
static MenuNode_t s_tr_speed, s_mech_zero;

// [Level 2] 系统设置文件夹内的文件
static MenuNode_t s_save;

// =================================================================
// 2. 全局变量
// =================================================================
static MenuNode_t *cursor = NULL;      // 光标选中的节点
static MenuNode_t *page_start = NULL;  // 当前屏幕第一行显示的节点
static uint8 is_editing = 0;           // 0=浏览模式, 1=编辑数值模式

// =================================================================
// 3. 超强工具函数 (帮你自动连线，不用看懂内部逻辑，会用就行)
// =================================================================

// 填充节点数据的快捷方式
void Menu_Set_Item(MenuNode_t *node, char *name, ItemType_e type, void *ptr, float step, float min, float max) {
    node->name = name; node->type = type; node->ptr = ptr;
    node->step = step; node->limit_min = min; node->limit_max = max;
}

// 自动连接兄弟和父子关系
// parent: 父节点指针 (如果是主菜单填 NULL)
// count: 这一层有多少个项目
// ... : 列出所有项目指针
void Menu_Connect(MenuNode_t *parent, int count, ...) {
    va_list args;
    MenuNode_t *first, *last, *curr;
    int i;
    
    if(count <= 0) return;

    va_start(args, count);
    first = va_arg(args, MenuNode_t*);
    
    // 建立父子关系
    if(parent) {
        parent->ptr = first; // 爸爸的 ptr 指向 大儿子
    }
    first->parent = parent;  // 大儿子的 parent 指向 爸爸

    // 建立兄弟关系
    curr = first;
    for(i = 1; i < count; i++) {
        last = curr;
        curr = va_arg(args, MenuNode_t*);
        
        curr->parent = parent; // 所有兄弟都有同一个爸爸
        last->next = curr;     // 哥哥指弟弟
        curr->prev = last;     // 弟弟指哥哥
    }
    // 闭环 (最后一个指向第一个)
    curr->next = first;
    first->prev = curr;
    
    va_end(args);
}

// =================================================================
// 4. 菜单初始化 (在这里 修改/增删 菜单)
// =================================================================
void Menu_Init(void) {
    // --- Step 1: 填充数据 (给每个文件写名字和内容) ---
    
    // Level 1: 主菜单项
    Menu_Set_Item(&m_pid,    "1. PID Param", ITEM_TYPE_MENU, NULL, 0,0,0);
    Menu_Set_Item(&m_track,  "2. Track Set", ITEM_TYPE_MENU, NULL, 0,0,0);
    Menu_Set_Item(&m_system, "3. System",    ITEM_TYPE_MENU, NULL, 0,0,0);

    // Level 2: PID 参数
    Menu_Set_Item(&s_bal_kp, "Bal Kp",       ITEM_TYPE_FLOAT, &g_sys_param.balance_kp, 1.0f,  0.0f, 500.0f);
    Menu_Set_Item(&s_bal_kd, "Bal Kd",       ITEM_TYPE_FLOAT, &g_sys_param.balance_kd, 0.1f,  0.0f, 20.0f);
    Menu_Set_Item(&s_vel_kp, "Vel Kp",       ITEM_TYPE_FLOAT, &g_sys_param.velocity_kp,0.01f, 0.0f, 2.0f);
    Menu_Set_Item(&s_vel_ki, "Vel Ki",       ITEM_TYPE_FLOAT, &g_sys_param.velocity_ki,0.001f,0.0f, 0.5f);
    Menu_Set_Item(&s_turn_kp,"Turn Kp",      ITEM_TYPE_FLOAT, &g_sys_param.turn_kp,    1.0f,  0.0f, 100.0f);

    // Level 2: 循迹参数
    Menu_Set_Item(&s_tr_speed, "Base Speed", ITEM_TYPE_INT,   &g_sys_param.track_speed, 5.0f,  10.0f, 100.0f);
    Menu_Set_Item(&s_mech_zero,"Mech Zero",  ITEM_TYPE_FLOAT, &g_sys_param.mech_zero_pitch, 0.1f, -10.0f, 10.0f);

    // Level 2: 系统功能
    Menu_Set_Item(&s_save,     "[Save Flash]",ITEM_TYPE_FUNC,  (void*)Param_Save,        0, 0, 0);

    // --- Step 2: 建立连接 (把文件装进文件夹) ---
    
    // 连接主菜单 (3个文件夹)
    Menu_Connect(NULL, 3, &m_pid, &m_track, &m_system);
    
    // 连接 PID 文件夹的内容 (5个参数)
    Menu_Connect(&m_pid, 5, &s_bal_kp, &s_bal_kd, &s_vel_kp, &s_vel_ki, &s_turn_kp);
    
    // 连接 循迹 文件夹的内容 (2个参数)
    Menu_Connect(&m_track, 2, &s_tr_speed, &s_mech_zero);
    
    // 连接 系统 文件夹的内容 (1个功能)
    Menu_Connect(&m_system, 1, &s_save);

    // --- Step 3: 复位光标 ---
    cursor = &m_pid;
    page_start = &m_pid;
    is_editing = 0;
}

// =================================================================
// 5. 业务逻辑 (数值调整)
// =================================================================
void Menu_Adjust(uint8 is_add) {
    float *f_ptr;
    int16 *i_ptr;
    if(cursor->type == ITEM_TYPE_FLOAT) {
        f_ptr = (float*)cursor->ptr;
        if(is_add) *f_ptr += cursor->step; else *f_ptr -= cursor->step;
        if(*f_ptr > cursor->limit_max) *f_ptr = cursor->limit_max;
        if(*f_ptr < cursor->limit_min) *f_ptr = cursor->limit_min;
    } else if(cursor->type == ITEM_TYPE_INT) {
        i_ptr = (int16*)cursor->ptr;
        if(is_add) *i_ptr += (int16)cursor->step; else *i_ptr -= (int16)cursor->step;
        if(*i_ptr > (int16)cursor->limit_max) *i_ptr = (int16)cursor->limit_max;
        if(*i_ptr < (int16)cursor->limit_min) *i_ptr = (int16)cursor->limit_min;
    }
}

// =================================================================
// 6. 显示与交互 (主循环调用)
// =================================================================
void Menu_Show(void) {
    MenuNode_t *draw_node;
    uint8 i;
    
    // --- A. 按键逻辑 ---
    if(!is_editing) { // 浏览模式
        // UP: 上移
        if(Key_Check(KEY_NAME_UP, KEY_SINGLE | KEY_REPEAT)) {
            cursor = cursor->prev;
            // 如果光标跑到了当前页面的上面，把页面往上拉
            if(cursor == page_start->prev) page_start = cursor;
        }
        // DOWN: 下移
        if(Key_Check(KEY_NAME_DOWN, KEY_SINGLE | KEY_REPEAT)) {
            cursor = cursor->next;
            // 如果光标跑到了页面下面(第5个)，页面往下滚
            // 简单的判断：检查光标是不是 page_start 的 +1, +2, +3
            // 严谨写法：
            if(cursor != page_start && cursor != page_start->next && cursor != page_start->next->next && cursor != page_start->next->next->next) {
                 page_start = page_start->next; // 页面下滚
            }
            // 循环回滚处理 (如果光标回到了链表头，页面也回表头)
            // 这里的逻辑对于父子菜单略复杂，简单处理：如果 cursor->parent == page_start->parent，说明在同一层
        }
        // CONFIRM: 进入/编辑/执行
        if(Key_Check(KEY_NAME_CONFIRM, KEY_SINGLE)) {
            if(cursor->type == ITEM_TYPE_MENU) {
                // 【核心】进入子菜单
                if(cursor->ptr != NULL) {
                    cursor = (MenuNode_t*)cursor->ptr; // 光标跳到大儿子
                    page_start = cursor;               // 页面显示子菜单
                }
            } else if(cursor->type == ITEM_TYPE_FUNC) {
                // 执行函数
                void (*func)(void) = cursor->ptr;
                func();
//                Buzzer_Beep(500);
            } else {
                // 进入数值编辑
                is_editing = 1;
//                Buzzer_Beep(100);
            }
        }
        // BACK: 返回上一级
        if(Key_Check(KEY_NAME_BACK, KEY_SINGLE)) {
            if(cursor->parent != NULL) {
                // 【核心】返回父菜单
                cursor = cursor->parent;
                page_start = cursor; // 简单处理：把父菜单设为页面起始
                // 优化：如果你想让光标回到刚才进入的位置，需要更复杂的记录，这里暂不需要
            }
        }
    } else { // 编辑模式
        if(Key_Check(KEY_NAME_UP, KEY_SINGLE | KEY_REPEAT)) Menu_Adjust(1);
        if(Key_Check(KEY_NAME_DOWN, KEY_SINGLE | KEY_REPEAT)) Menu_Adjust(0);
        // 确认或返回都退出编辑
        if(Key_Check(KEY_NAME_CONFIRM, KEY_SINGLE) || Key_Check(KEY_NAME_BACK, KEY_SINGLE)) {
            is_editing = 0;
//            Buzzer_Beep(100);
        }
    }

    // --- B. 屏幕绘制 ---
    OLED_Clear();
    
    // 绘制标题栏 (可选，显示当前层级名字? 暂不加，省空间)
    
    draw_node = page_start;
    for(i=0; i<4; i++) { // OLED 显示 4 行
        // 1. 绘制光标
        if(draw_node == cursor) {
            if(is_editing) OLED_ShowString(i+1, 1, ">");
            else           OLED_ShowString(i+1, 1, "->");
        } else {
            OLED_ShowString(i+1, 1, "  ");
        }
        
        // 2. 绘制名称
 //       OLED_ShowString(i+1, 3, draw_node->name);
        
        // 3. 绘制内容
        if(draw_node->type == ITEM_TYPE_MENU) {
            OLED_ShowString(i+1, 14, ">>"); // 子菜单指示符
        } else if(draw_node->type == ITEM_TYPE_FLOAT) {
            float val = *(float*)draw_node->ptr;
//            OLED_ShowFloat(i+1, 10, val, 2, 2); // 显示浮点
        } else if(draw_node->type == ITEM_TYPE_INT) {
            int16 val = *(int16*)draw_node->ptr;
 //           OLED_ShowNum(i+1, 12, val, 3);      // 显示整数
        }
        
        draw_node = draw_node->next;
        // 如果是单行菜单或到底了，怎么处理？
        // 由于是循环链表，它会自动画回头部，这也是一种显示风格
    }
    
}