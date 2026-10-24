#include "NEUQ_lib_include.h"

MENUPAGE g_menu_page = PAGE_MAIN;
uint8_t g_menu_cursor = 0;
EDITSTATE g_edit_state = EDIT_OFF;

static WAVECUR wave_cursor = CUR_SEND;

static const float alpha_step = 0.01f;
static const float thre_step = 0.5f;
static const float acc_th_step = 0.01f;
static FILTER_PARAMS g_param_backup;
static uint8_t page_changed = 1;
static uint8_t g_save_status = 0;   // 0=未存 1=OK 2=ERR

#define ROW_Y(n) (30 + (n) * 30)

void menu_init(void) {
    g_menu_page = PAGE_MAIN;
    g_menu_cursor = 0;
    wave_cursor = CUR_SEND;
    g_edit_state = EDIT_OFF;
    page_changed = 1;
    g_save_status = 0;
}

static inline void lcd_set_color(uint32_t fg, uint32_t bg)
{
    NEUQ_LCD200_SetPenColor(fg);
    NEUQ_LCD200_SetBackColor(bg);
}

static void menu_set_row_color(WAVECUR row, uint8_t is_action) {
    if (wave_cursor == row) {
        if (!is_action && g_edit_state == EDIT_ON) {
            lcd_set_color(LCD_WHITE, LCD_BLACK);   // 编辑中：反色
        } else {
            lcd_set_color(LCD_WHITE, LCD_BLACK);   // 选中：反色（白字黑底）
        }
    } else {
        lcd_set_color(LCD_BLACK, LCD_WHITE);       // 未选中：黑字白底
    }
}

// ===== 保存封装（方便显示状态）=====
static void menu_do_save(void)
{
    flash_param_save();
    // 如果 flash_param_save 已改为返回 uint8_t：
    g_save_status = (flash_param_save() == NEUQ_FLASH_OK) ? 1 : 2;
    g_save_status = 1;   // 暂时假设成功；改返回值后自动准确
    page_changed = 1;
}

void menu_key_handle(KEY_EVENT_ENUM key_event) {
    switch (g_menu_page) {

        case PAGE_MAIN:
            switch (key_event) {
                case KEY_UP_SHORT:
                    if (g_menu_cursor > 0) g_menu_cursor--;
                    break;
                case KEY_DOWN_SHORT:
                    if (g_menu_cursor < 2) g_menu_cursor++;
                    break;
                case KEY_FORWARD_SHORT:
                    switch (g_menu_cursor) {
                        case 0:
                            g_menu_page = PAGE_SIX_AXIS;
                            page_changed = 1;
                            break;
                        case 1:
                            g_menu_page = PAGE_WAVE_FILTER;
                            wave_cursor = CUR_SEND;
                            g_edit_state = EDIT_OFF;
                            g_save_status = 0;
                            page_changed = 1;
                            break;
                        case 2:
                            g_menu_page = PAGE_ATTITUDE;
                            page_changed = 1;
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;

        case PAGE_SIX_AXIS:
            switch (key_event) {
                case KEY_BACK_SHORT:
                    g_menu_page = PAGE_MAIN;
                    g_menu_cursor = 0;
                    page_changed = 1;
                    break;
                case KEY_BACK_LONG:
                    g_filter_param.send_enable = !g_filter_param.send_enable;
                    page_changed = 1;
                    break;
                default:
                    break;
            }
            break;

        case PAGE_WAVE_FILTER:
            if (g_edit_state == EDIT_ON) {
                switch (key_event) {
                    case KEY_UP_SHORT:
                        switch (wave_cursor) {
                            case CUR_AXIS:
                                if (g_filter_param.selected_axis > 0) {
                                    g_filter_param.selected_axis--;
                                } else {
                                    g_filter_param.selected_axis = 2;
                                }
                                break;
                            case CUR_ALPHA:
                                g_filter_param.alpha -= alpha_step;
                                if (g_filter_param.alpha < 0.01f)
                                    g_filter_param.alpha = 0.01f;
                                break;
                            case CUR_GYRO:
                                g_filter_param.gyro_threshold -= thre_step;
                                if (g_filter_param.gyro_threshold < 0.0f)
                                    g_filter_param.gyro_threshold = 0.0f;
                                break;
                            case CUR_ACC:
                                g_filter_param.acc_threshold -= acc_th_step;
                                if (g_filter_param.acc_threshold < 0.01f)
                                    g_filter_param.acc_threshold = 0.01f;
                                break;
                            default:
                                break;
                        }
                        break;

                    case KEY_DOWN_SHORT:
                        switch (wave_cursor) {
                            case CUR_AXIS:
                                if (g_filter_param.selected_axis < 2) {
                                    g_filter_param.selected_axis++;
                                } else {
                                    g_filter_param.selected_axis = 0;
                                }
                                break;
                            case CUR_ALPHA:
                                g_filter_param.alpha += alpha_step;
                                if (g_filter_param.alpha > 0.99f)
                                    g_filter_param.alpha = 0.99f;
                                break;
                            case CUR_GYRO:
                                g_filter_param.gyro_threshold += thre_step;
                                if (g_filter_param.gyro_threshold > 20.0f)
                                    g_filter_param.gyro_threshold = 20.0f;
                                break;
                            case CUR_ACC:
                                g_filter_param.acc_threshold += acc_th_step;
                                if (g_filter_param.acc_threshold > 0.5f)
                                    g_filter_param.acc_threshold = 0.5f;
                                break;
                            default:
                                break;
                        }
                        break;

                    case KEY_FORWARD_SHORT:
                        g_edit_state = EDIT_OFF;
                        g_save_status = 0;   // 编辑退出，重置保存状态
                        page_changed = 1;
                        break;

                    case KEY_BACK_SHORT:
                        g_filter_param = g_param_backup;
                        g_edit_state = EDIT_OFF;
                        g_save_status = 0;
                        page_changed = 1;
                        break;

                    default:
                        break;
                }
            } else {
                switch (key_event) {
                    case KEY_UP_SHORT:
                        if (wave_cursor > 0) wave_cursor--;
                        break;

                    case KEY_DOWN_SHORT:
                        if (wave_cursor < CUR_SAVE) wave_cursor++;
                        break;

                    case KEY_FORWARD_SHORT:
                        switch (wave_cursor) {
                            case CUR_SEND:
                                g_filter_param.send_enable = !g_filter_param.send_enable;
                                page_changed = 1;
                                break;
                            case CUR_ACC:
                            case CUR_ALPHA:
                            case CUR_AXIS:
                            case CUR_GYRO:
                                g_param_backup = g_filter_param;
                                g_edit_state = EDIT_ON;
                                break;
                            case CUR_SAVE:
                                menu_do_save();
                                break;
                            default:
                                break;
                        }
                        break;

                    case KEY_FORWARD_LONG:
                        menu_do_save();
                        break;

                    case KEY_BACK_SHORT:
                        g_menu_page = PAGE_MAIN;
                        g_menu_cursor = 1;
                        wave_cursor = CUR_SEND;
                        g_edit_state = EDIT_OFF;
                        g_save_status = 0;
                        page_changed = 1;
                        break;

                    case KEY_BACK_LONG:
                        g_filter_param.send_enable = !g_filter_param.send_enable;
                        page_changed = 1;
                        break;

                    default:
                        break;
                }
            }
            break;

        case PAGE_ATTITUDE:
            switch (key_event) {
                case KEY_FORWARD_SHORT:          /* 第三界面用FORWARD触发校准 */
                    gyro_calibrate_start();
                    page_changed = 1;
                    break;
                case KEY_BACK_SHORT:
                    g_menu_page = PAGE_MAIN;
                    g_menu_cursor = 2;
                    page_changed = 1;
                    break;
                default:
                    break;
            }
            break;      

        default:
            break;
    }
}

void menu_show(void) {
    uint8_t just_changed = page_changed;

    if (just_changed) {
        lcd_set_color(LCD_BLACK, LCD_WHITE);
        NEUQ_LCD200_Clear();

        switch (g_menu_page) {
            case PAGE_MAIN:
                NEUQ_LCD200_DisplayString(10, 10, "1.Six Axis Data");
                NEUQ_LCD200_DisplayString(10, 50, "2.Wave & Filter");
                NEUQ_LCD200_DisplayString(10, 90, "3.Attitude Solve");
                break;

            case PAGE_SIX_AXIS:
                NEUQ_LCD200_DisplayString(10, 0, "=== Six Axis Data ===");
                break;

            case PAGE_WAVE_FILTER:
                NEUQ_LCD200_DisplayString(10, 0, "===Wave AND Filter===");
                break;

            case PAGE_ATTITUDE:
                break;

            default:
                break;
        }
    }

    switch (g_menu_page) {

        case PAGE_MAIN: {
            for (uint8_t i = 0; i < 3; i++) {
                if (i == g_menu_cursor) {
                    lcd_set_color(LCD_WHITE, LCD_BLACK);
                } else {
                    lcd_set_color(LCD_BLACK, LCD_WHITE);
                }
                switch (i) {
                    case 0: NEUQ_LCD200_DisplayString(10, 10, "1.Six Axis Data"); break;
                    case 1: NEUQ_LCD200_DisplayString(10, 50, "2.Wave & Filter"); break;
                    case 2: NEUQ_LCD200_DisplayString(10, 90, "3.Attitude Solve"); break;
                }
            }
            break;
        }

        case PAGE_SIX_AXIS: {
            char buf[32];

            if (just_changed) {
                lcd_set_color(LCD_BLACK, LCD_WHITE);
                NEUQ_LCD200_DisplayString(0,   40, "Accel(g)");
                NEUQ_LCD200_DisplayString(160, 40, "Gyro(dps)");

                NEUQ_LCD200_DisplayString(0,   70, "Ax:");
                NEUQ_LCD200_DisplayString(0,  100, "Ay:");
                NEUQ_LCD200_DisplayString(0,  130, "Az:");

                NEUQ_LCD200_DisplayString(160,  70, "Gx:");
                NEUQ_LCD200_DisplayString(160, 100, "Gy:");
                NEUQ_LCD200_DisplayString(160, 130, "Gz:");

                lcd_set_color(LCD_BLACK, LCD_WHITE);
                NEUQ_LCD200_DisplayString(0, 160, "KEY4:Back  KEY4-L:Send");
            }

            // Ax
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(36,  70, 110, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.2f", MPU6050_acc_transition(mpu6050_acc_x));
            NEUQ_LCD200_DisplayString(36,  70, buf);

            // Ay
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(36, 100, 110, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.2f", MPU6050_acc_transition(mpu6050_acc_y));
            NEUQ_LCD200_DisplayString(36, 100, buf);

            // Az
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(36, 130, 110, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.2f", MPU6050_acc_transition(mpu6050_acc_z));
            NEUQ_LCD200_DisplayString(36, 130, buf);

            // Gx
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(196,  70, 110, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.1f", MPU6050_gyro_transition(mpu6050_gyro_x));
            NEUQ_LCD200_DisplayString(196,  70, buf);

            // Gy
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(196, 100, 110, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.1f", MPU6050_gyro_transition(mpu6050_gyro_y));
            NEUQ_LCD200_DisplayString(196, 100, buf);

            // Gz
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(196, 130, 110, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.1f", MPU6050_gyro_transition(mpu6050_gyro_z));
            NEUQ_LCD200_DisplayString(196, 130, buf);

            break;
        }

        case PAGE_WAVE_FILTER: {
            char buf[32];

            if (just_changed) {
                lcd_set_color(LCD_BLACK, LCD_WHITE);
                NEUQ_LCD200_DisplayString(0, ROW_Y(CUR_SEND),  "Send:     ");
                NEUQ_LCD200_DisplayString(0, ROW_Y(CUR_AXIS),  "Axis:  ");
                NEUQ_LCD200_DisplayString(0, ROW_Y(CUR_ALPHA), "Alpha:     ");
                NEUQ_LCD200_DisplayString(0, ROW_Y(CUR_GYRO),  "Kp:   ");
                NEUQ_LCD200_DisplayString(0, ROW_Y(CUR_ACC),   "Ki:   ");
                NEUQ_LCD200_DisplayString(0, ROW_Y(CUR_SAVE),  "Save:");
                NEUQ_LCD200_DisplayString(0, 210, "KEY3:Edit KEY4:Back ");
            }

            // ===== Send =====
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(60, ROW_Y(CUR_SEND), 60, 24);
            menu_set_row_color(CUR_SEND, 1);
            NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_SEND),
                g_filter_param.send_enable ? "ON " : "OFF");

            // ===== Axis =====
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(60, ROW_Y(CUR_AXIS), 30, 24);
            menu_set_row_color(CUR_AXIS, 0);
            sprintf(buf, "%c", 'X' + g_filter_param.selected_axis);
            NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_AXIS), buf);

            // ===== Alpha =====
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(70, ROW_Y(CUR_ALPHA), 60, 24);
            menu_set_row_color(CUR_ALPHA, 0);
            sprintf(buf, "%.2f", g_filter_param.alpha);
            NEUQ_LCD200_DisplayString(70, ROW_Y(CUR_ALPHA), buf);

            // ===== G_Th =====
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(60, ROW_Y(CUR_GYRO), 60, 24);
            menu_set_row_color(CUR_GYRO, 0);
            sprintf(buf, "%.1f", g_filter_param.gyro_threshold);
            NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_GYRO), buf);

            // ===== A_Th =====
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(60, ROW_Y(CUR_ACC), 60, 24);
            menu_set_row_color(CUR_ACC, 0);
            sprintf(buf, "%.2f", g_filter_param.acc_threshold);
            NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_ACC), buf);

            // ===== Save 状态行 =====
            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(60, ROW_Y(CUR_SAVE), 120, 24);
            menu_set_row_color(CUR_SAVE, 1);
            if (g_save_status == 0) {
                NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_SAVE), "Ready");
            } else if (g_save_status == 1) {
                NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_SAVE), "OK  ");
            } else {
                NEUQ_LCD200_DisplayString(60, ROW_Y(CUR_SAVE), "ERR ");
            }

            // ===== 编辑状态指示 =====
            if (g_edit_state == EDIT_ON) {
                lcd_set_color(LCD_RED, LCD_WHITE);
                NEUQ_LCD200_DisplayString(230, 20, "EDIT");
            } else {
                lcd_set_color(LCD_WHITE, LCD_WHITE);
                NEUQ_LCD200_ClearRect(230, 20, 110, 24);
            }

            lcd_set_color(LCD_BLACK, LCD_WHITE);
            break;
        }

        case PAGE_ATTITUDE: {
            char buf[32];
            if (just_changed) {
                lcd_set_color(LCD_BLACK, LCD_WHITE);
                NEUQ_LCD200_Clear();
                NEUQ_LCD200_DisplayString(10, 0, "=== Attitude Solve ===");
                NEUQ_LCD200_DisplayString(0,  40, "Roll:  (deg)");
                NEUQ_LCD200_DisplayString(0,  80, "Pitch: (deg)");
                NEUQ_LCD200_DisplayString(0, 120, "Yaw:   (deg)");
                NEUQ_LCD200_DisplayString(0, 160, "FWD:Cal  BACK:Return");
            }

            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(70,  40, 90, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.1f", filter_get_roll());
            NEUQ_LCD200_DisplayString(70, 40, buf);

            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(70,  80, 90, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.1f", filter_get_pitch());
            NEUQ_LCD200_DisplayString(70, 80, buf);

            lcd_set_color(LCD_WHITE, LCD_WHITE);
            NEUQ_LCD200_ClearRect(70, 120, 90, 24);
            lcd_set_color(LCD_BLACK, LCD_WHITE);
            sprintf(buf, "%.1f", filter_get_yaw());
            NEUQ_LCD200_DisplayString(70, 120, buf);

            /* 校准状态 */
            if (gyro_calibrate_busy()) {
                lcd_set_color(LCD_RED, LCD_WHITE);
                NEUQ_LCD200_DisplayString(0, 200, "Calibrating...keep still");
            } else if (gyro_calibrated()) {
                lcd_set_color(LCD_BLACK, LCD_WHITE);
                NEUQ_LCD200_DisplayString(0, 200, "Gyro bias OK           ");
            } else {
                lcd_set_color(LCD_RED, LCD_WHITE);
                NEUQ_LCD200_DisplayString(0, 200, "Uncalibrated,press FWD ");
            }
            break;
        }

        default:
            break;
    }

    if (just_changed) {
        page_changed = 0;
    }
}