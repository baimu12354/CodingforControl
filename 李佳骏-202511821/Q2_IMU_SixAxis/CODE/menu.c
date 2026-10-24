/* menu.c - 二级菜单状态机 */

#include "menu.h"
#include "board_config.h"
#include "app_tick.h"
#include "bsp_key.h"
#include "imu_filter.h"
#include "imu_attitude.h"
#include "param_store.h"

/* 长按连续动作的最小间隔，避免一按住就飞过去 */
#define MENU_REPEAT_MS      (120U)

static menu_page_t s_page        = MENU_PAGE_MAIN;
static uint8_t     s_main_cursor = 0;   /* 一级页面的高亮项，返回时要恢复 */
static uint8_t     s_cursor      = 0;   /* 当前二级页面的高亮项 */

static imu_axis_t  s_axis        = IMU_AX;
static uint8_t     s_upload_on   = 0;

static uint8_t     s_editing     = 0;
static float       s_edit_value  = 0.0f;
static imu_axis_t  s_edit_axis   = IMU_AX;

static uint8_t     s_saved_flag  = 0;

static uint32_t    s_last_repeat_ms   = 0;

void Menu_Init(void)
{
    s_page        = MENU_PAGE_MAIN;
    s_main_cursor = 0;
    s_cursor      = 0;
    s_axis        = IMU_AX;
    s_upload_on   = 0;
    s_editing     = 0;
    s_saved_flag  = 0;
}

static uint8_t Menu_KeyAction(key_id_t id, uint8_t allow_repeat)
{
    key_event_t ev = Key_GetEvent(id);
    uint32_t    now;

    if (ev == KEY_EVENT_SHORT)
    {
        s_last_repeat_ms = AppTick_GetMs();
        return 1U;
    }

    if ((ev == KEY_EVENT_LONG) && (allow_repeat != 0U))
    {
        now = AppTick_GetMs();
        if ((uint32_t)(now - s_last_repeat_ms) >= MENU_REPEAT_MS)
        {
            s_last_repeat_ms = now;
            return 1U;
        }
    }

    return 0U;
}

static void Menu_MoveCursor(int8_t direction)
{
    uint8_t count;

    if (s_page == MENU_PAGE_MAIN)
    {
        count = 3U;

        if (direction > 0)
        {
            s_main_cursor = (uint8_t)((s_main_cursor + 1U) % count);
        }
        else
        {
            s_main_cursor = (uint8_t)((s_main_cursor + count - 1U) % count);
        }
        s_cursor = s_main_cursor;
    }
    else if (s_page == MENU_PAGE_WAVE)
    {
        if (direction > 0)
        {
            s_cursor = (uint8_t)((s_cursor + 1U) % MENU_WAVE_ITEM_COUNT);
        }
        else
        {
            s_cursor = (uint8_t)((s_cursor + MENU_WAVE_ITEM_COUNT - 1U) % MENU_WAVE_ITEM_COUNT);
        }
    }
}

/* 菜单按键状态机，返回 1 表示需要重画 */
uint8_t Menu_HandleKey(void)
{
    uint8_t redraw = 0;

    /* 返回键 */
    if (Menu_KeyAction(KEY_ID_BACK, 0U))
    {
        if (s_editing != 0U)
        {
            s_editing = 0;                      /* 编辑中按返回 = 放弃本次修改 */
            redraw = 1U;
        }
        else if (s_page != MENU_PAGE_MAIN)
        {
            s_page   = MENU_PAGE_MAIN;          /* 回一级页面 */
            s_cursor = s_main_cursor;           /* 恢复原来的高亮项 */
            redraw   = 1U;
        }
    }

    /* 上 / 下 */
    if (Menu_KeyAction(KEY_ID_UP, 1U))
    {
        if (s_editing != 0U)
        {
            s_edit_value += T2_FILTER_ALPHA_STEP;
            if (s_edit_value > T2_FILTER_ALPHA_MAX)
            {
                s_edit_value = T2_FILTER_ALPHA_MAX;
            }
        }
        else
        {
            Menu_MoveCursor(-1);
        }
        redraw = 1U;
    }

    if (Menu_KeyAction(KEY_ID_DOWN, 1U))
    {
        if (s_editing != 0U)
        {
            s_edit_value -= T2_FILTER_ALPHA_STEP;
            if (s_edit_value < T2_FILTER_ALPHA_MIN)
            {
                s_edit_value = T2_FILTER_ALPHA_MIN;
            }
        }
        else
        {
            Menu_MoveCursor(1);
        }
        redraw = 1U;
    }

    /* 确认键 */
    if (Menu_KeyAction(KEY_ID_ENTER, 0U))
    {
        if (s_page == MENU_PAGE_MAIN)
        {
            s_main_cursor = s_cursor;
            switch (s_cursor)
            {
                case 0: s_page = MENU_PAGE_SIXAXIS;  break;
                case 1: s_page = MENU_PAGE_WAVE;     break;
                default: s_page = MENU_PAGE_ATTITUDE; break;
            }
            s_cursor = 0;
            redraw   = 1U;
        }
        else if (s_page == MENU_PAGE_WAVE)
        {
            switch (s_cursor)
            {
                case MENU_WAVE_ITEM_AXIS:
                    /* 依次切换要观察/上传的轴 */
                    s_axis = (imu_axis_t)((s_axis + 1U) % IMU_AXIS_COUNT);
                    redraw = 1U;
                    break;

                case MENU_WAVE_ITEM_UPLOAD:
                    s_upload_on = (uint8_t)(s_upload_on ? 0U : 1U);
                    redraw = 1U;
                    break;

                case MENU_WAVE_ITEM_ALPHA:
                    if (s_editing == 0U)
                    {
                        /* 进入编辑：把当前轴的真实系数复制到临时值 */
                        s_edit_axis  = s_axis;
                        s_edit_value = ImuFilter_GetAlpha(s_axis);
                        s_editing    = 1U;
                    }
                    else
                    {
                        /* 确认写回：只有确认才真正生效 */
                        ImuFilter_SetAlpha(s_edit_axis, s_edit_value);
                        s_editing = 0U;
                    }
                    redraw = 1U;
                    break;

                case MENU_WAVE_ITEM_SAVE:
                    if (ParamStore_SaveCurrent() != 0U)
                    {
                        s_saved_flag = 1U;
                    }
                    redraw = 1U;
                    break;

                default:
                    break;
            }
        }
        else if (s_page == MENU_PAGE_ATTITUDE)
        {
            /* 姿态页只做一件事：按确认键重新做一次零偏校准。
             * 不往一级菜单加第四个入口，是为了不改动基础题
             * 「一级功能选择页提供三个入口」的结构。 */
            Attitude_StartCalibration();
            redraw = 1U;
        }
    }

    return redraw;
}

menu_page_t Menu_GetPage(void)
{
    return s_page;
}

uint8_t Menu_GetCursor(void)
{
    return (s_page == MENU_PAGE_MAIN) ? s_main_cursor : s_cursor;
}

imu_axis_t Menu_GetAxis(void)
{
    return s_axis;
}

uint8_t Menu_IsUploadOn(void)
{
    return s_upload_on;
}

uint8_t Menu_IsEditing(void)
{
    return s_editing;
}

float Menu_GetEditValue(void)
{
    return s_edit_value;
}

imu_axis_t Menu_GetEditingAxis(void)
{
    return s_edit_axis;
}

uint8_t Menu_TakeSavedFlag(void)
{
    uint8_t flag = s_saved_flag;
    s_saved_flag = 0;
    return flag;
}
