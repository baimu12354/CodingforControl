#include "NEUQ_lib_include.h"

extern int32_t current_axis;
extern int32_t wave_send_enable;

static const MenuConfig MenuTable[MENU_COUNT] = {
    {"Main Menu", MENU_NONE, NULL},
    {"6Axis Data", MENU_MAIN, NULL},
    {"Filter&Wave", MENU_MAIN, NULL},
    {"Attitude", MENU_MAIN, NULL},
    {"Kp", MENU_FILTER, &filter_kp},
    {"Axis", MENU_FILTER, &current_axis},
    {"Wave", MENU_FILTER, &wave_send_enable}
};
static const char *axisNames[6] = {
    "AccX(g)", "GyroX(dps)", "AccY(g)",
    "GyroY(dps)", "AccZ(g)", "GyroZ(dps)"
};
static const char axisLetters[3] = {'X', 'Y', 'Z'};
static MenuNode nodes[MENU_COUNT];
static MenuNode *page;
static MenuNode *selected;
static uint8_t editing = 0;
static int32_t edit_value;
static uint8_t menu_changed = 1;
static const char *save_status = "";

static void Menu_DefaultColors(void)
{
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
}


static void Menu_ShowLine(uint16_t y, const char *text)
{
    char line[31];
    snprintf(line, sizeof(line), "%-30.30s", text);
    NEUQ_LCD200_DisplayString(8, y, line);
}

static void Menu_BuildLinks(void)
{
    memset(nodes, 0, sizeof(nodes));
    for (uint8_t i = 0; i < MENU_COUNT; i++) {
        nodes[i].info = &MenuTable[i];
        if (MenuTable[i].parent_id == MENU_NONE) continue;
        MenuNode *parent = &nodes[MenuTable[i].parent_id];
        MenuNode *node = &nodes[i];
        node->parent = parent;
        if (parent->child == NULL) {
            parent->child = node;
            node->prev = node;
            node->next = node;
        } else {
            MenuNode *first = parent->child;
            MenuNode *last = first->prev;
            node->prev = last;
            node->next = first;
            last->next = node;
            first->prev = node;
        }
    }
}

void NEUQ_Menu_Init(void)
{
    /* sector 0 的前 4 字节保存 filter_kp；读失败或越界时使用代码默认值 5。 */
    int32_t saved_kp;
    filter_kp = 5;
    if (NEUQ_Flash_read_sector(0, (uint8_t *)&saved_kp, sizeof(saved_kp)) == NEUQ_FLASH_OK &&
        saved_kp >= 0 && saved_kp <= 100) {
        filter_kp = saved_kp;
    }
    Menu_BuildLinks();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font20);
    page = &nodes[MENU_MAIN];
    selected = page->child;
    editing = 0;
    menu_changed = 1;
    save_status = "";
}

void NEUQ_Menu_SendKey(KeyEventEnum key)
{
    if (key == KEY_EVENT_NONE) return;
    menu_changed = 1;

    if (editing) {
        int32_t maximum = 100;
        if (selected == &nodes[MENU_FILTER_AXIS]) maximum = 5;
        if (selected == &nodes[MENU_FILTER_SEND]) maximum = 1;
        switch (key) {
            case KEY_UP_SHORT:
                if (edit_value < maximum) edit_value++;
                break;
            case KEY_DOWN_SHORT:
                if (edit_value > 0) edit_value--;
                break;
            case KEY_FORWARD_SHORT:
                if (selected == &nodes[MENU_FILTER_KP]) {
                    if (NEUQ_Flash_write_sector(0, (const uint8_t *)&edit_value,
                                               sizeof(edit_value)) != NEUQ_FLASH_OK) {
                        save_status = "Save failed: OK retry";
                        return;
                    }
                    save_status = "Flash saved";
                }
                *selected->info->value = edit_value;
                editing = 0;
                break;
            case KEY_BACK_SHORT:
                editing = 0;
                save_status = "";
                break;
            default:
                break;
        }
        return;
    }

    if (key == KEY_BACK_SHORT) {
        if (page->parent != NULL) {
            selected = page;
            page = page->parent;
        }
        return;
    }
    if (page->child == NULL) return;

    switch (key) {
        case KEY_UP_SHORT:
            selected = selected->prev;
            break;
        case KEY_DOWN_SHORT:
            selected = selected->next;
            break;
        case KEY_FORWARD_SHORT:
            if (selected->info->value != NULL) {
                editing = 1;
                edit_value = *selected->info->value;
                save_status = "";
            } else {
                page = selected;
                selected = page->child;
            }
            break;
        default:
            break;
    }
}

void NEUQ_Menu_Task(void)
{
    static MenuNode *last_page = NULL;
    static uint8_t last_calibrating = 2;
    static uint32_t menu_tick = 0;
    char line[64];

    if (last_page != page || last_calibrating != imu_calibrating) {
        Menu_DefaultColors();
        NEUQ_LCD200_Clear();
        last_page = page;
        last_calibrating = imu_calibrating;
        menu_changed = 1;
        if (imu_calibrating) {
            Menu_ShowLine(8, "Calibrating...");
            Menu_ShowLine(50, "Keep board still");
        } else {
            snprintf(line, sizeof(line), "== %s ==", page->info->name);
            Menu_ShowLine(8, line);
        }
    }
    if (imu_calibrating) return;

    if (page->child != NULL) {
        if (!menu_changed) return;
        menu_changed = 0;
        MenuNode *item = page->child;
        uint16_t y = 50;
        do {
            char mark = ' ';
            if (item == selected) {
                mark = editing ? '*' : '>';
                NEUQ_LCD200_SetPenColor(LCD_WHITE);
                NEUQ_LCD200_SetBackColor(LCD_BLUE);
            } else {
                Menu_DefaultColors();
            }
            NEUQ_LCD200_ClearRect(8, y, 300, 24);

            if (item->info->value != NULL) {
                int32_t value = *item->info->value;
                if (item == selected && editing) value = edit_value;
                if (item == &nodes[MENU_FILTER_AXIS]) {
                    snprintf(line, sizeof(line), "%cAxis:%s", mark, axisNames[value]);
                } else if (item == &nodes[MENU_FILTER_SEND]) {
                    snprintf(line, sizeof(line), "%cWave:%s", mark, value ? "ON" : "OFF");
                } else {
                    snprintf(line, sizeof(line), "%cKp:%ld", mark, (long)value);
                }
            } else {
                snprintf(line, sizeof(line), "%c%s", mark, item->info->name);
            }
            Menu_ShowLine(y, line);
            Menu_DefaultColors();
            y += 30;
            item = item->next;
        } while (item != page->child);
        if (page == &nodes[MENU_FILTER]) Menu_ShowLine(150, save_status);
        return;
    }

    if (!menu_changed && HAL_GetTick() - menu_tick < 200) return;
    menu_changed = 0;
    menu_tick = HAL_GetTick();
    if (page == &nodes[MENU_6AXIS]) {
        for (uint8_t i = 0; i < 3; i++) {
            snprintf(line, sizeof(line), "Acc%c:%.3f g", axisLetters[i], accel_data[i]);
            Menu_ShowLine(40 + i * 26, line);
            snprintf(line, sizeof(line), "Gyro%c:%.2f dps", axisLetters[i], gyro_data[i]);
            Menu_ShowLine(118 + i * 26, line);
        }
    } else if (page == &nodes[MENU_ATTITUDE]) {
        snprintf(line, sizeof(line), "Roll(X):%.2f deg", roll_angle);
        Menu_ShowLine(50, line);
        snprintf(line, sizeof(line), "Pitch(Y):%.2f deg", pitch_angle);
        Menu_ShowLine(80, line);
        snprintf(line, sizeof(line), "Yaw(Z):%.2f deg", yaw_angle);
        Menu_ShowLine(110, line);
        Menu_ShowLine(150, "IMU target:10ms");
    }
}

MenuNode *NEUQ_Menu_GetCurrentNode(void) { return selected; }
uint8_t NEUQ_Menu_GetCurrentPageID(void) { return (uint8_t)(page - nodes); }
