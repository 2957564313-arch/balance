#ifndef _MENU_H_
#define _MENU_H_

#include "zf_common_headfile.h"

// 菜单项类型
typedef enum {
    ITEM_TYPE_MENU = 0,  // 【文件夹】子菜单，点击进入下一级
    ITEM_TYPE_FLOAT,     // 【文件】浮点数参数，点击进入编辑
    ITEM_TYPE_INT,       // 【文件】整数参数，点击进入编辑
    ITEM_TYPE_FUNC       // 【程序】功能项，点击立即执行
} ItemType_e;

// 树状链表节点
typedef struct MenuNode_t {
    // --- 展示内容 ---
    char *name;           // 名字
    ItemType_e type;      // 类型
    
    // --- 数据关联 ---
    void *ptr;            // 核心指针：
                          // 如果是参数：指向变量地址 (&kp)
                          // 如果是菜单：指向第一个子节点地址 (&child)
                          // 如果是功能：指向函数地址 (Save)
    float step;           // 调节步长
    float limit_min;      // 最小值
    float limit_max;      // 最大值
<<<<<<< Updated upstream

    // --- 关系网 (指针域) ---
    struct MenuNode_t *prev;   // 上一个兄弟 (哥哥)
    struct MenuNode_t *next;   // 下一个兄弟 (弟弟)
    struct MenuNode_t *parent; // 父节点 (爸爸) - 用于返回上一级
} MenuNode_t;

void Menu_Init(void);
void Menu_Show(void);
=======
>>>>>>> Stashed changes

    // --- 关系网 (指针域) ---
    struct MenuNode_t *prev;   // 上一个兄弟 (哥哥)
    struct MenuNode_t *next;   // 下一个兄弟 (弟弟)
    struct MenuNode_t *parent; // 父节点 (爸爸) - 用于返回上一级
} MenuNode_t;

void Menu_Init(void);
void Menu_Show(void);

#endif