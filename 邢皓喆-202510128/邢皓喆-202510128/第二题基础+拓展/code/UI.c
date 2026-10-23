#include "NEUQ_lib_include.h"
#include "Attitude.h"

static Attitude_Data attitude_view;

NEUQ_MANU_Param_t manu_param = {
    .MPU_alpha = &mpu_filter_alpha,
    .MPU_filter_select = &filter_select,
    .MPU_raw = {raw, raw + 1, raw + 2, raw + 3, raw + 4, raw + 5},
    .MPU_filterd = {filterd, filterd + 1, filterd + 2, filterd + 3, filterd + 4, filterd + 5}
};
typedef enum {
    SAVE_NONE = 0,
    SAVE_OK,
    SAVE_FAILED
} SaveState;
static SaveState save_state = SAVE_NONE;
typedef enum {
    MANU_NONE = -1,
    MANU_MAIN = 0,
    MANU_SIX_AXIS,
    MANU_WAVEFORM,
    MANU_ATTITUDE,
    MANU_ALPHA,
    MANU_UPLOAD,
    MANU_FILTER_SELECT,
    MANU_raw_x,
    MANU_raw_y,
    MANU_raw_z,
    MANU_Gyro_x,
    MANU_Gyro_y,
    MANU_Gyro_z,
    MANU_ROLL,
    MANU_PITCH,
    MANU_YAW,
    MANU_CNT
} MANUID;

typedef enum {
    MENU_VALUE_NONE = 0,
    MENU_VALUE_FLOAT,
    MENU_VALUE_U8
} MENU_ValueType;

typedef struct {
    char *name;
    MANUID parent_id;
    void *value;
    float min;
    float max;
    float step;
    MENU_ValueType type;
}MANUConfig;

typedef struct MANUNode {
    const MANUConfig* info;
    struct MANUNode* parent;
    struct MANUNode* child;
    struct MANUNode* next;
    struct MANUNode* prev;
} MANUNode;

static MANUConfig manu_config[MANU_CNT] = {
    {"Main Menu", MANU_NONE, NULL, 0, 0, 0, MENU_VALUE_NONE},
    {"6-Axis Data", MANU_MAIN, NULL, 0, 0, 0, MENU_VALUE_NONE},
    {"Waveform", MANU_MAIN, NULL, 0, 0, 0, MENU_VALUE_NONE},
    {"Attitude", MANU_MAIN, NULL, 0, 0, 0, MENU_VALUE_NONE},

    {"Alpha", MANU_WAVEFORM, &mpu_filter_alpha, 0.01f, 1.0f, 0.01f, MENU_VALUE_FLOAT},
    {"Upload", MANU_WAVEFORM, &upload_enable, 0, 1, 1, MENU_VALUE_U8},
    {"Filter Select", MANU_WAVEFORM, &filter_select, 0, 5, 1, MENU_VALUE_U8},

    {"Acc X(g)",    MANU_SIX_AXIS, &raw[0], -8,    8,    0, MENU_VALUE_FLOAT},
    {"Acc Y(g)",    MANU_SIX_AXIS, &raw[1], -8,    8,    0, MENU_VALUE_FLOAT},
    {"Acc Z(g)",    MANU_SIX_AXIS, &raw[2], -8,    8,    0, MENU_VALUE_FLOAT},
    {"Gyro X(dps)", MANU_SIX_AXIS, &raw[3], -2000, 2000, 0, MENU_VALUE_FLOAT},
    {"Gyro Y(dps)", MANU_SIX_AXIS, &raw[4], -2000, 2000, 0, MENU_VALUE_FLOAT},
    {"Gyro Z(dps)", MANU_SIX_AXIS, &raw[5], -2000, 2000, 0, MENU_VALUE_FLOAT},
    {"Roll(deg)", MANU_ATTITUDE, &attitude_view.roll, -180, 180, 0, MENU_VALUE_FLOAT},
    {"Pitch(deg)", MANU_ATTITUDE, &attitude_view.pitch, -90, 90, 0, MENU_VALUE_FLOAT},
    {"Yaw(deg)", MANU_ATTITUDE, &attitude_view.yaw, -180, 180, 0, MENU_VALUE_FLOAT}
};
static float MANU_ReadValue(const MANUConfig *info)
{
    if (info->value == NULL)
        return 0.0f;

    switch (info->type)
    {
        case MENU_VALUE_FLOAT:
            return *(float *)info->value;

        case MENU_VALUE_U8:
            return *(uint8_t *)info->value;

        default:
            return 0.0f;
    }
}

static void MANU_WriteValue(const MANUConfig *info, float value)
{
    if (info->value == NULL)
        return;

    if (value < info->min)
        value = info->min;
    if (value > info->max)
        value = info->max;

    switch (info->type)
    {
        case MENU_VALUE_FLOAT:
            *(float *)info->value = value;
            break;

        case MENU_VALUE_U8:
            *(uint8_t *)info->value = (uint8_t)value;
            break;

        default:
            break;
    }
}

static MANUNode manu_nodes[MANU_CNT];
static MANUNode *page;
static MANUNode *selected;
static int8_t editing = 0;
static void NEUQ_MANU_BuildTree()
{
    uint8_t i;
    MANUNode *parent;
    MANUNode *node;
    MANUNode *first;
    MANUNode *last;
    for (i = 0; i < MANU_CNT; i++)
    {
        manu_nodes[i].info = &manu_config[i];
        manu_nodes[i].parent = NULL;
        manu_nodes[i].child = NULL;
        manu_nodes[i].next = NULL;
        manu_nodes[i].prev = NULL;
    }
    for (i = 0; i < MANU_CNT; i++)
    {
        node = &manu_nodes[i];
        if (node->info->parent_id != MANU_NONE)
        {
            parent = &manu_nodes[node->info->parent_id];
            node->parent = parent;
            if (parent->child == NULL)
            {
                parent->child = node;
                node->next = node;
                node->prev = node;
            }
            else
            {
                first = parent->child;
                last = first->prev;
                last->next = node;
                node->prev = last;
                first->prev = node;
                node->next = first;
            }
        }
        else {
            continue;
        }
    }
}
static void MANU_AttitudeSnapshot(void)
{
    /* Copy only: do not draw or format strings with interrupts disabled. */
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    Attitude_GetData(&attitude_view);
    __set_PRIMASK(saved_primask);
}

static void MANU_AttitudeStatus(void)
{
    char text[32];
    if (page != &manu_nodes[MANU_ATTITUDE]) return;
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    if (attitude_view.status == ATTITUDE_READY)
        snprintf(text, sizeof(text), "%-25s", "Ready - yaw relative");
    else if (attitude_view.status == ATTITUDE_SENSOR_ERROR)
        snprintf(text, sizeof(text), "%-25s", "Sensor error");
    else
        snprintf(text, sizeof(text), "Cal %3u/300 keep still   ",
                 (unsigned int)attitude_view.calibration_count);
    NEUQ_LCD200_DisplayString(8, 152, text);
    snprintf(text, sizeof(text), "dt:%5.1f ms  X/Y/Z RH   ", (double)attitude_view.dt_ms);
    NEUQ_LCD200_DisplayString(8, 176, text);
    NEUQ_LCD200_DisplayString(8, 208, "Hold OK: recalibrate");
}

float evit_value;
static uint8_t NEUQ_MANU_UPDate(KEY_EventTypeDef key_event)
{
    if (!editing && page == &manu_nodes[MANU_ATTITUDE] &&
        key_event == KEY_FORWARD_LONG)
    {
        Attitude_RequestCalibration();
        return 1U;
    }
   
    //编辑模式
    if (editing)
    {
        const MANUConfig* item = selected->info;
        switch (key_event)
        {
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
                evit_value+= item->step;
                break;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:
                evit_value -= item->step;
                break;
            case KEY_FORWARD_SHORT:
            case KEY_FORWARD_LONG:
            {
                MANU_WriteValue(item, evit_value);
                if (selected == &manu_nodes[MANU_ALPHA])
                {
                    save_state = Setting_Save() ? SAVE_OK : SAVE_FAILED;
                }
                editing = 0;
            }
                break;
            case KEY_BEHIND_SHORT:
            case KEY_BEHIND_LONG:
                editing = 0;
                break;
                default:
                return 0;
        }
        if (evit_value < item->min)
            {
                evit_value = item->min;
            }
            if (evit_value > item->max)
            {
                evit_value = item->max;
            }
         return 1;
    }
    //浏览模式
        switch (key_event)
        {
            case KEY_UP_SHORT:
            case KEY_UP_LONG:
                selected = selected->prev;
                break;
            case KEY_DOWN_SHORT:
            case KEY_DOWN_LONG:
                selected = selected->next;
                break;
            case KEY_FORWARD_SHORT:
            case KEY_FORWARD_LONG:
                if (selected->child != NULL)
                {
                    page = selected;
                    selected = selected->child;
                }
                else if (selected->info->value != NULL && selected->info->step > 0.0f)
                {
                    editing = 1;
                    if (selected == &manu_nodes[MANU_ALPHA])
                    {
                        save_state = SAVE_NONE;
                    }
                    evit_value = MANU_ReadValue(selected->info);
                }
                else
                {
                    return 0;
                }
                break;
            case KEY_BEHIND_SHORT:
            case KEY_BEHIND_LONG:
                if (page->parent != NULL)
                {
                    selected = page;
                    page = page->parent;
                }
                else
                {
                    return 0;
                }
                break;
            default:
                return 0;
        }
    return 1;
}
static void NEUQ_MANU_Display(void)
{
    MANUNode *item = page->child;
    char line[32];
    uint8_t y = 48;
    NEUQ_LCD200_Clear();
    MANU_AttitudeStatus();
    if (item != NULL)
    {
    do
     {char mark = ' ';
        const MANUConfig *Info = item->info;
    if(item == selected)
    {   mark = (editing)?'*':'>';
        NEUQ_LCD200_SetPenColor(LCD_WHITE);
        NEUQ_LCD200_SetBackColor(LCD_BLUE);
        NEUQ_LCD200_ClearRect(8, y ,304, 24); 
    }
    if (Info->value != NULL)
    {
        float value = ((item == selected) && editing) ? evit_value : MANU_ReadValue(Info);

    if (Info->parent_id == MANU_ATTITUDE && attitude_view.status != ATTITUDE_READY)
    {
        snprintf(line, sizeof(line), "%c %s:--", mark, Info->name);
    }
    else if (Info->type == MENU_VALUE_FLOAT)
    {
        snprintf(line, sizeof(line), "%c %s:%.2f", mark, Info->name, (double)value);}
    else
    {
        snprintf(line, sizeof(line), "%c %s:%u", mark, Info->name, (unsigned int)value);
    }
    }
    else
    {
        snprintf (line, sizeof(line), "%c %s", mark, Info ->name);
    }
     NEUQ_LCD200_DisplayString(8, y, line);
     NEUQ_LCD200_SetPenColor(LCD_BLACK);
     NEUQ_LCD200_SetBackColor(LCD_WHITE);
     y+= 32;
     item = item->next;

     }while (item != page->child);
    if (page == &manu_nodes[MANU_WAVEFORM])
    {
    if (save_state == SAVE_OK)
    {
        NEUQ_LCD200_DisplayString(8, 208, "Saved");
    }
    else if (save_state == SAVE_FAILED)
    {
        NEUQ_LCD200_DisplayString(8, 208, "Save failed");
    }
    }
    }

}
// 只刷新六轴页面的数值
static void NEUQ_MANU_RefreshValues(void)
{
    if (page != &manu_nodes[MANU_SIX_AXIS] &&
        page != &manu_nodes[MANU_ATTITUDE])
        return;

    MANUNode *item = page->child;
    uint16_t y = 48;
    char text[16];

    if (item == NULL)
        return;

    do
    {
        const MANUConfig *info = item->info;

        if (info->value != NULL)
        {
            NEUQ_LCD200_SetPenColor(item == selected ? LCD_WHITE : LCD_BLACK);
            NEUQ_LCD200_SetBackColor(item == selected ? LCD_BLUE : LCD_WHITE);
            uint16_t x =8U + (uint16_t)(strlen(info->name) + 3U) * 12U;
            if (info->parent_id == MANU_ATTITUDE && attitude_view.status != ATTITUDE_READY)
                snprintf(text, sizeof(text), "%-9s", "--");
            else
                snprintf(text, sizeof(text), "%-9.2f", (double)MANU_ReadValue(info));
            NEUQ_LCD200_DisplayString(x, y, text);
        }

        y += 32;
        item = item->next;
    }
    while (item != page->child);

    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    MANU_AttitudeStatus();
}
void NEUQ_MANU_Init()
{
    NEUQ_MANU_BuildTree();
    KEY_Init();
    page = &manu_nodes[MANU_MAIN];
    selected = page->child;
    editing = 0;
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_Clear();
    NEUQ_MANU_Display();
}
void NEUQ_MANU_Task(void)
{
    static uint32_t last_refresh = 0;

    MANU_AttitudeSnapshot();
    KEY_EventTypeDef key = KEY_GetEvent();
    uint8_t changed = NEUQ_MANU_UPDate(key);
    uint32_t now = TIMER_GET_MS();

    if (changed)
    {
        NEUQ_MANU_Display();
        last_refresh = TIMER_GET_MS();
    }
    else if ((page == &manu_nodes[MANU_SIX_AXIS] ||
              page == &manu_nodes[MANU_ATTITUDE]) &&
             (uint32_t)(now - last_refresh) >= 200U)
    {
        NEUQ_MANU_RefreshValues();
        last_refresh = TIMER_GET_MS();
    }
}