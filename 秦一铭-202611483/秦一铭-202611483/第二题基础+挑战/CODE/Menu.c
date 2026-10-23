#include "NEUQ_lib_include.h"

typedef enum{
    NONE = -1,      // 没有父页
    MAIN = 0,       // 一级菜单
    SIX,            // 六轴数据显示
    WAVE,           // 波形与滤波设置
    AE,             // 姿态解算
    AXIS,           // Wave 页：发送轴
    SEND,           // Wave 页：发送开关
    FP,             // Wave 页：滤波参数
    SAVE,           // Wave 页：保存
    NUM,
}ID;

typedef struct{
    const char  *name;
    int16_t      parent;
    int32_t     *value;
    const char **texts;
    int32_t      min;
    int32_t      max;
    int32_t      step;
}MenuConfig;

static const char *axis_text[6]  = {"Gyro X","Gyro Y","Gyro Z","Acc X","Acc Y","Acc Z"};
static const char *onoff_text[2] = {"Off","On"};
static const char *axis_unit[6]  = {"deg/s","deg/s","deg/s","g","g","g"};

// 菜单表：一行就是一个页面或一个菜单项
// FP 行的 value 先指向 param[0]，换轴时在 ChangeValue 里改成 &param[send_axis]
static MenuConfig table[NUM] = {
    [MAIN] = {"Main",        NONE, NULL,       NULL,       0, 0,  0},
    [SIX]  = {"6-Axis Data", MAIN, NULL,       NULL,       0, 0,  0},
    [WAVE] = {"Wave&Filter", MAIN, NULL,       NULL,       0, 0,  0},
    [AE]   = {"Attitude",    MAIN, NULL,       NULL,       0, 0,  0},

    [AXIS] = {"Send Axis",   WAVE, &send_axis, axis_text,  0, 5,  1},
    [SEND] = {"Send",        WAVE, &send_on,   onoff_text, 0, 1,  1},
    [FP]   = {"FP",          WAVE, &param[0],  NULL,       1, 99, 1},
    [SAVE] = {"Save",        WAVE, NULL,       NULL,       0, 0,  0},
};

typedef struct Node{
    const MenuConfig *info;     // 这个节点对应的表项
    struct Node *parent;        // 上一级
    struct Node *child;         // 第一个子项
    struct Node *prev;          // 同级前一个
    struct Node *next;          // 同级后一个
}Node;

static Node nodes[NUM];
static Node *page;              // 当前页面
static Node *selected;          // 光标所在项（永远是 page 的子项）
static uint8_t editing = 0;     // 0 浏览，1 编辑
static uint8_t saved   = 0;     // 已保存过

static void Build(void){
    uint8_t i;

    for (i = 0; i < NUM; i++){          // 先全部清空
        nodes[i].info   = &table[i];
        nodes[i].parent = NULL;
        nodes[i].child  = NULL;
        nodes[i].prev   = NULL;
        nodes[i].next   = NULL;
    }

    for (i = 0; i < NUM; i++){
        Node *n = &nodes[i];
        Node *p;

        if (table[i].parent == NONE) continue;

        p = &nodes[table[i].parent];
        n->parent = p;

        if (p->child == NULL){          // 第一个子项：自己成环
            p->child = n;
            n->prev  = n;
            n->next  = n;
        }else{                          // 插到环尾
            n->prev = p->child->prev;
            n->next = p->child;
            n->prev->next  = n;
            p->child->prev = n;
        }
    }
}

/* ==================== 显示 ====================*/
// 列表页：一行一项，选中行蓝色
static void ShowList(void){
    Node *n = page->child;
    char line[24];
    char mark;
    uint16_t y = 20;

    do{
        const MenuConfig *it = n->info;
        mark = (n == selected) ? (editing ? '*' : '>') : ' ';

        if (it->value == NULL){                             // 只显示名字
            if (n == &nodes[SAVE] && saved) snprintf(line, 24, "%c Saved", mark);
            else                            snprintf(line, 24, "%c %s", mark, it->name);
        }else if (it->texts != NULL){                       // 文字
            snprintf(line, 24, "%c %s %s", mark, it->name, it->texts[*it->value]);
        }else{                                              // 数字
            snprintf(line, 24, "%c %s 0.%02d", mark, it->name, (int)*it->value);
        }

        if (n == selected){
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);
            NEUQ_LCD200_ClearRect(0, y - 4, 320, 32);
        }
        NEUQ_LCD200_DisplayString(8, y, line);
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);

        y += 40;
        n = n->next;
    }while (n != page->child);
}

// 六轴页：刷新 6 个实时数值
static void ShowSixNum(void){
    uint8_t i;
    uint16_t y = 40;

    for (i = 0; i < 6; i++){
        NEUQ_LCD200_ClearRect(100, y, 88, 24);
        NEUQ_LCD200_DisplayFloat(100, y, filt[i], 7, (i < 3) ? 1 : 2);
        y += 33;
    }
}

// 六轴页：轴名和单位
static void ShowSix(void){
    uint8_t i;
    uint16_t y = 40;

    for (i = 0; i < 6; i++){
        NEUQ_LCD200_DisplayString(8, y, (char *)axis_text[i]);
        NEUQ_LCD200_DisplayString(200, y, (char *)axis_unit[i]);
        y += 33;
    }
    ShowSixNum();
}

// 姿态页：三个角度
static void ShowAeNum(void){
    NEUQ_LCD200_ClearRect(160, 40, 88, 24);
    NEUQ_LCD200_DisplayFloat(160, 40, roll, 7, 1);
    NEUQ_LCD200_ClearRect(160, 70, 88, 24);
    NEUQ_LCD200_DisplayFloat(160, 70, pitch, 7, 1);
    NEUQ_LCD200_ClearRect(160, 100, 88, 24);
    NEUQ_LCD200_DisplayFloat(160, 100, yaw, 7, 1);
}

// 姿态页：标签
static void ShowAe(void){
    NEUQ_LCD200_DisplayString(8, 40, "Roll (deg)");
    NEUQ_LCD200_DisplayString(8, 70, "Pitch (deg)");
    NEUQ_LCD200_DisplayString(8, 100, "Yaw (deg)");
    ShowAeNum();
}

static void Display(void){
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_Clear();

    if (page == &nodes[SIX])      ShowSix();
    else if (page == &nodes[AE])  ShowAe();
    else                          ShowList();
}

/*==================== 按键====================*/

// α从1~99循环
static void ChangeValue(int8_t d){
    const MenuConfig *it = selected->info;

    *it->value += d * it->step;
    if (*it->value > it->max) *it->value = it->min;
    if (*it->value < it->min) *it->value = it->max;

    if (selected == &nodes[AXIS])               // 换发送轴，FP 行对应α跟着换
        table[FP].value = &param[send_axis];
}

// 返回 1 表示画面需要重画
static uint8_t Update(KeyEvent key){
    const MenuConfig *it = selected->info;

    if (key == KEY_NONE) return 0;
    saved = 0;

    if (editing){                               // 编辑：上下改值，前后退出
        switch (key){
            case KEY_UP:      ChangeValue(-1); break;
            case KEY_DOWN:    ChangeValue(+1); break;
            case KEY_FORWARD:
            case KEY_BACK:    editing = 0;     break;
        }
        return 1;
    }

    if (page == &nodes[SIX] || page == &nodes[AE]){   // 数据页只认返回
        if (key != KEY_BACK) return 0;
        selected = page;
        page = page->parent;
        return 1;
    }

    switch (key){
        case KEY_UP:    selected = selected->prev; break;
        case KEY_DOWN:  selected = selected->next; break;

        case KEY_FORWARD:
            if (selected->child != NULL){           // 有子页面就进去
                page = selected;
                selected = page->child;
            }else if (selected == &nodes[SIX] || selected == &nodes[AE]){
                page = selected;                    // 数据页没有光标
            }else if (it->value != NULL){           // 有参数的项可以编辑
                editing = 1;
            }else{                                  // Save功能
                Param_Save();
                saved = 1;
            }
            break;

        case KEY_BACK:
            if (page->parent == NULL) return 0;
            selected = page;
            page = page->parent;
            break;

    }
    return 1;
}

void Menu_Init(void){
    Build();
    Param_Load();

    page     = &nodes[MAIN];
    selected = page->child;
    editing  = 0;

    table[FP].value = &param[send_axis];        //指向"当前选中的发送轴"对应的系数 

    NEUQ_LCD200_init();                         
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_Clear();

    Display();
}

void Menu_Task(void){
    if (Update(Key_Get())){
        Display();                              // 有按键：整屏重画
    }else if (page == &nodes[SIX]){
        ShowSixNum();                           // 数据页：只刷数字
    }else if (page == &nodes[AE]){
        ShowAeNum();
    }
}