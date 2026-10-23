#include "NEUQ_lib_include.h"

volatile uint8_t current_scheme = 1;
volatile uint8_t next_scheme = 1;

typedef enum {//菜单ID
    MENU_NONE = -1,
    MENU_MAIN = 0,
    MENU_SCHEME_1,
    MENU_SCHEME_2,
    MENU_SCHEME_3,
    MENU_SCHEME_4,
    MENU_COUNT
} MENUID;

typedef struct {
    const char *name;
    MENUID parent_id;
} MENUCONFIG;

typedef struct MENUNODE {//链表节点关系
    const MENUCONFIG *info;  // 节点内容
    struct MENUNODE *parent; // 层级父节点
    struct MENUNODE *child;  // 层级子节点
    struct MENUNODE *prev;   // 同级前节点
    struct MENUNODE *next;   // 同级后节点
} MENUNODE;

static const MENUCONFIG MENUTABLE[MENU_COUNT] = {
    [MENU_MAIN]     = {"MAIN MENU", MENU_NONE},
    [MENU_SCHEME_1] = {"SCHEME_1",  MENU_MAIN},
    [MENU_SCHEME_2] = {"SCHEME_2",  MENU_MAIN},
    [MENU_SCHEME_3] = {"SCHEME_3",  MENU_MAIN},
    [MENU_SCHEME_4] = {"SCHEME_4",  MENU_MAIN},
};
static MENUNODE nodes[MENU_COUNT];                 // 固定内存存储节点
static MENUNODE *page = &nodes[MENU_MAIN];         // 当前页面
static MENUNODE *selected = &nodes[MENU_SCHEME_1]; // 当前节点（selected 永远是 page 的子节点）

static void MENU_BuildLinks(void) {//链表链接
    //清空链表链接
    for (uint8_t i = 0; i < MENU_COUNT; i++) {
        nodes[i].info = &MENUTABLE[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    for (uint8_t i = 1; i < MENU_COUNT; i++) {
        MENUNODE *node = &nodes[i];
        node->parent = &nodes[MENU_MAIN];
        if (nodes[MENU_MAIN].child == NULL) {
            nodes[MENU_MAIN].child = node;
            node->prev = node;
            node->next = node;
        } else {
            MENUNODE *first = nodes[MENU_MAIN].child; // child 只是 scheme_1
            MENUNODE *last = first->prev;
            node->prev = last;
            last->next = node;
            node->next = first;
            first->prev = node;
        }
    }
}

void NEUQ_LCD_INIT(void){
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();
}

void NEUQ_MENU_INIT(void) {
    MENU_BuildLinks();
    NEUQ_LCD_INIT();
    NEUQ_GPIO_init(PA8, GPIO_Mode_Output_PushPull, GPIO_High);//LED
}

void LED_Handler(void) {
    static uint32_t led_tick = 0;   //LED时钟
    led_tick++;
    //切换逻辑
    if (led_tick >= 200) {
        led_tick = 0;
        current_scheme = next_scheme;
    }

    uint8_t led_on = 0;
    switch (current_scheme) {//依据方案亮灭
        case MENU_SCHEME_1:
            if (led_tick < 100) {
                led_on = 1;
            }else{
                led_on = 0;
               }
            break;
        case MENU_SCHEME_2:
            if (led_tick < 20) {
                led_on = 1;
            }else{
                led_on = 0;
            }
            break;
        case MENU_SCHEME_3:
            if (led_tick < 20 || (led_tick >= 40 && led_tick < 60)) {
                led_on = 1;
            }else{
                led_on = 0;
            }
            break;
        case MENU_SCHEME_4:
            if (led_tick < 20 || (led_tick >= 40 && led_tick < 100)) {
                led_on = 1;
            }else{
                led_on = 0;
            }
            break;
        default:
            break;
    }
    NEUQ_GPIO_write(PA8, led_on);
}

static void MENU_Display_Update(void) {//菜单显示逻辑
    char line[32];
    //顶部显示
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_ClearRect(0, 0, 240, 36);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    sprintf(line, "Current:%d Next:%d", current_scheme, next_scheme);
    NEUQ_LCD200_DisplayString(8, 8, line);
    MENUNODE *item = page->child;   //scheme1
    uint16_t y = 40;
    if (item != NULL) {
        do {
            if (item == selected) {
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
                NEUQ_LCD200_ClearRect(8, y, 220, 24);
                sprintf(line, " > %s", item->info->name);
                NEUQ_LCD200_DisplayString(8, y, line);
            } else {
                NEUQ_LCD200_SetPenColor(LCD_BLACK);
                NEUQ_LCD200_SetBackColor(LCD_WHITE);
                NEUQ_LCD200_ClearRect(0, y, 220, 24);
                sprintf(line, "   %s", item->info->name);
                NEUQ_LCD200_DisplayString(8, y, line);
            }
            y += 32;
            item = item->next;
        } while (item != page->child);
    }
}


void NEUQ_MENU_TASK(void) {
    static uint8_t last_next_scheme = MENU_MAIN;
    static uint8_t last_current_scheme = MENU_MAIN;
    static MENUNODE *last_selected = NULL;
    KEY_EVENT_ENUM key = NEUQ_KEY_EVENTGET();
    if (page == &nodes[MENU_MAIN]) {
        if (key == KEY_FORWARD_SHORT) {
            selected = selected->next;
        } else if (key == KEY_BACK_SHORT) {
            selected = selected->prev;
        }
        next_scheme = (uint8_t)(selected - &nodes[MENU_MAIN]);
        if (next_scheme != last_next_scheme || selected != last_selected ||
            current_scheme != last_current_scheme) {
            last_next_scheme = next_scheme;
            last_selected = selected;
            last_current_scheme = current_scheme;
            MENU_Display_Update();
        }
    }
}