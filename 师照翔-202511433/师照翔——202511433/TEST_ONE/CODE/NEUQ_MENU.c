#include"NEUQ_lib_include.h"
#include<stdint.h>
 MENU_NODE nodes[MENU_COUNT];//存储所有节点，nodes[i]就是一个节点
 MENU_NODE *page;//当前页面的节点
 MENU_NODE *select;//选择高亮项的节点
 uint8_t editing;//0->浏览模式，1->编辑模式
 uint32_t edit_temp;//用于临时存储编辑模式下更改变量的值


// ==================== 菜单配置表 ====================
static const Menuconfig menu_table[MENU_COUNT]={
    {"LED",      MENU_NONE,       NULL, 0, 0, 0},
    {"stategy1", MENU_LED_TEXT,  NULL, 0, 0, 0},
    {"stategy2", MENU_LED_TEXT,  NULL, 0, 0, 0},
    {"stategy3", MENU_LED_TEXT,  NULL, 0, 0, 0},
    {"stategy4", MENU_LED_TEXT,  NULL, 0, 0, 0},
};

// ==================== 内部函数：构建链表 ====================
static void Build_links(void)
{
    // 初始化所有节点
    for(uint16_t i=0; i<MENU_COUNT; i++){
        nodes[i].info   = &menu_table[i];
        nodes[i].parent = NULL;
        nodes[i].child  = NULL;
        nodes[i].prev   = NULL;
        nodes[i].next   = NULL;
    }

    // 构建层级关系 + 同级双向循环链表
    for (uint16_t i = 0; i < MENU_COUNT; i++)
    {
        MENU_NODE *node = &nodes[i];
        if (node->info->parent_id == MENU_NONE) continue;
        
        MENU_NODE *parent = &nodes[node->info->parent_id];
        node->parent = parent;

        MENU_NODE *first = parent->child;
        if (first == NULL)
        {
            parent->child = node;
            node->prev = node;
            node->next = node;
        }
        else
        {
            MENU_NODE *last = first->prev;
            node->prev   = last;
            last->next   = node;
            node->next   = first;
            first->prev  = node;
        }
    }
}

// ==================== 菜单更新逻辑 ====================
uint8_t Menu_Update(KEY_EVENT_ENUM Key)
{
    // 编辑模式（当前菜单项无参数，暂不会进入）
    if(editing == 1){
        const Menuconfig *item = select->info;
        switch(Key){
            case KEY_UP_SHORT:
                break;
            case KEY_DOWN_SHORT:
                break;
            case KEY_FORWORD_SHORT:
                *(item->value) = edit_temp;
                editing = 0;
                break;
            case KEY_BACK_SHORT:
                editing = 0;
                break;
            case KEY_UP_LONG:
                edit_temp += item->step;
                break;  
            case KEY_DOWN_LONG:
                edit_temp -= item->step;
                break;
            default:
                return 0;
        }
        if(edit_temp < item->min) edit_temp = item->min;
        if(edit_temp > item->max) edit_temp = item->max;
        return 1;
    }

    // 浏览模式
    else{
        switch(Key){
            case KEY_UP_SHORT:
                select = select->prev;
                break;
            case KEY_DOWN_SHORT:
                select = select->next;
                break;

            case KEY_FORWORD_SHORT:
                // 有子菜单：进入子页面
                if(select->child != NULL){
                    page = select;
                    select = select->child;
                }
                // 有参数：进入编辑模式
                else if(select->info->value != NULL){
                    editing = 1;
                    edit_temp = *(select->info->value);
                }
                // 方案菜单项：提交切换请求
                else{
                    uint8_t idx = select - nodes;
                    if(idx >= MENU_STRATEGY1 && idx <= MENU_STRATEGY4){
                        stratage_req = idx - MENU_STRATEGY1 + 1;
                    }
                }
                break;

            case KEY_BACK_SHORT:
                if(page->parent == NULL) return 0;
                select = page;
                page = page->parent;
                break;

            default:
                return 0;
        }
        return 1;
    }
}

// ==================== 菜单初始化 ====================
void Menu_Init(void)
{
    // 硬件初始化
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();

    // 构建菜单链表
    Build_links();

    // 初始页面：LED方案列表页
    page   = &nodes[MENU_LED_TEXT];
    select = page->child;
    editing = 0;

    // 首次刷新屏幕
    Menu_Show();
}

// ==================== 屏幕显示逻辑 ====================
void Menu_Show(void)
{
    MENU_NODE *item = page->child;
    NEUQ_LCD200_Clear();
    char line[32];
    uint16_t y = 48;

    // 显示页面标题
    snprintf(line, sizeof(line), "%s", page->info->name);
    NEUQ_LCD200_DisplayString(8, 8, line);

    if (item != NULL)
    {
        do
        {
            char mark = ' ';
            const Menuconfig *info = item->info;
            uint8_t run_idx = MENU_STRATEGY1 + stratage_run - 1;

            // 光标选中标记
            if (item == select) {
                mark = '>';
                 NEUQ_LCD200_SetPenColor(LCD_RED);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
            }

            // 正在运行的方案：蓝底白字高亮
            if (item == &nodes[run_idx])
            {
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 220, 24);
            }

            // 显示菜单项
            if (info->value != NULL)
            {
                int32_t value = ((item == select) && (editing)) ? edit_temp : (*info->value);
                snprintf(line, sizeof(line), "%c %s:%ld", mark, info->name, value);
            }
            else
            {
                snprintf(line, sizeof(line), "%c %s", mark, info->name);
            }
            NEUQ_LCD200_DisplayString(8, y, line);

            // 恢复默认颜色
            NEUQ_LCD200_SetPenColor(LCD_BLACK);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);

            y += 32;
            item = item->next;
        } while (item != page->child);
    }
}

// ==================== 菜单任务封装 ====================
void Menu_Task(void)
{
    KEY_EVENT_ENUM Key = NEUQ_Key_Event();
    if(Menu_Update(Key)){
        Menu_Show();
    }
}