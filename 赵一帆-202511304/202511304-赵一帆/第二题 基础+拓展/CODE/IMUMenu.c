#include "IMUMenu.h"
#include "IMU.h"
#include "Attitude.h"

static Page_t page = PAGE_MAIN;
static uint8_t cursor = 0;      // 主菜单光标
static uint8_t filter_row = 0;  // 滤波页行
static KeyEvent_t key_event = KEY_NONE;
static const char *axis_name[6] = {"AX","AY","AZ","GX","GY","GZ"};

void Key_Init(void) {
    NEUQ_GPIO_init(KEY_UP_PIN,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN_PIN,  GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_ENTER_PIN, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK_PIN,  GPIO_Mode_Input_PullUp, GPIO_High);
}

void Key_Scan(void) {
    static uint8_t  state = 0;
    static uint32_t press_tick = 0;
    static uint8_t  pressed_key = 0;

    uint8_t up    = (NEUQ_GPIO_read(KEY_UP_PIN)    == GPIO_Low);
    uint8_t down  = (NEUQ_GPIO_read(KEY_DOWN_PIN)  == GPIO_Low);
    uint8_t enter = (NEUQ_GPIO_read(KEY_ENTER_PIN) == GPIO_Low);
    uint8_t back  = (NEUQ_GPIO_read(KEY_BACK_PIN)  == GPIO_Low);
    uint8_t any   = up || down || enter || back;

    switch (state) {
        case 0:
            if (any) {
                press_tick = HAL_GetTick();
                if (up)          pressed_key = 1;
                else if (down)   pressed_key = 2;
                else if (enter)  pressed_key = 3;
                else if (back)   pressed_key = 4;
                state = 1;
            }
            break;
        case 1:
            if (!any) state = 0;
            else if (HAL_GetTick() - press_tick >= 20) state = 2;
            break;
        case 2:
            if (!any) {
                if (pressed_key == 1)      key_event = KEY_UP;
                else if (pressed_key == 2) key_event = KEY_DOWN;
                else if (pressed_key == 3) key_event = KEY_ENTER;
                else if (pressed_key == 4) key_event = KEY_BACK;
                state = 0;
            }
            break;
    }
}

void Menu_Init(void) { page = PAGE_MAIN; cursor = 0; filter_row = 0; }

void Menu_Task(void) {
    KeyEvent_t evt = key_event;
    key_event = KEY_NONE;
    if (evt == KEY_NONE) return;

    switch (page) {
        case PAGE_MAIN:
            if (evt == KEY_UP)        cursor = (cursor + 2) % 3;
            else if (evt == KEY_DOWN) cursor = (cursor + 1) % 3;
            else if (evt == KEY_ENTER) {
                page = (cursor == 0) ? PAGE_IMU : (cursor == 1) ? PAGE_FILTER : PAGE_ATTITUDE;
                if (page == PAGE_FILTER) filter_row = 0;
                NEUQ_LCD200_Clear();
            }
            break;
        case PAGE_IMU:
            if (evt == KEY_BACK) { page = PAGE_MAIN; NEUQ_LCD200_Clear(); }
            break;
        case PAGE_FILTER:
            if (evt == KEY_UP) {
                if (filter_row < 3) IMU_EditParam(filter_row, +1);
            }
            else if (evt == KEY_DOWN) {
                if (filter_row < 3) IMU_EditParam(filter_row, -1);
            }
            else if (evt == KEY_ENTER) {
                if (filter_row == 3) { 
                    IMU_SaveParams();
                    page = PAGE_MAIN; 
                    filter_row = 0; 
                    NEUQ_LCD200_Clear();
                } else {              // 否则光标向下移
                    filter_row = (filter_row + 1) % 4;
                }
            }
            else if (evt == KEY_BACK) {   // BACK 键也可以随时保存并返回
                IMU_SaveParams();
                page = PAGE_MAIN; filter_row = 0; NEUQ_LCD200_Clear();
            }
            break;
        case PAGE_ATTITUDE:
            if (evt == KEY_BACK) { page = PAGE_MAIN; NEUQ_LCD200_Clear(); }
            break;
    }
}

void Menu_LCD_Task(void) {
    static uint32_t last_tick = 0;
    static Page_t   last_page = (Page_t)0xFF;

    if (HAL_GetTick() - last_tick < 100) return;  // 10Hz 刷新
    last_tick = HAL_GetTick();

    if (page != last_page) {
        NEUQ_LCD200_Clear();

        switch (page) {
            case PAGE_MAIN:
                NEUQ_LCD200_DisplayString(0, 10, "Main Menu");
                NEUQ_LCD200_DisplayString(20, 40, "1.IMU Data");
                NEUQ_LCD200_DisplayString(20, 70, "2.Filter Set");
                NEUQ_LCD200_DisplayString(20, 100, "3.Attitude");
                break;
            case PAGE_IMU:
                NEUQ_LCD200_DisplayString(0, 10, "IMU Data (g / deg/s)");
                NEUQ_LCD200_DisplayString(0, 40,  "AX:");
                NEUQ_LCD200_DisplayString(0, 70,  "AY:");
                NEUQ_LCD200_DisplayString(0, 100, "AZ:");
                NEUQ_LCD200_DisplayString(0, 130, "GX:");
                NEUQ_LCD200_DisplayString(0, 160, "GY:");
                NEUQ_LCD200_DisplayString(0, 190, "GZ:");
                break;
            case PAGE_FILTER:
                NEUQ_LCD200_DisplayString(0, 10, "Filter Set");
                NEUQ_LCD200_DisplayString(10, 40,  "1.Axis:");
                NEUQ_LCD200_DisplayString(10, 70,  "2.Send:");
                NEUQ_LCD200_DisplayString(10, 100, "3.Alpha:");
                NEUQ_LCD200_DisplayString(10, 130, "4.Save & Return");
                break;
            case PAGE_ATTITUDE:
                NEUQ_LCD200_DisplayString(0, 10, "Attitude(deg) 5ms");
                NEUQ_LCD200_DisplayString(0, 40,  "Roll :");
                NEUQ_LCD200_DisplayString(0, 70,  "Pitch:");
                NEUQ_LCD200_DisplayString(0, 100, "Yaw  :");
                break;
        }
        last_page = page;
    }

    switch (page) {
        case PAGE_MAIN:
            // 动态擦除光标并重绘
            NEUQ_LCD200_DisplayString(0, 40,  " ");
            NEUQ_LCD200_DisplayString(0, 70,  " ");
            NEUQ_LCD200_DisplayString(0, 100, " ");
            NEUQ_LCD200_DisplayString(0, 40 + cursor * 30, ">");
            break;
        case PAGE_IMU:
            NEUQ_LCD200_DisplayFloat(60, 40,  imu.ax, 7, 3);
            NEUQ_LCD200_DisplayFloat(60, 70,  imu.ay, 7, 3);
            NEUQ_LCD200_DisplayFloat(60, 100, imu.az, 7, 3);
            NEUQ_LCD200_DisplayFloat(60, 130, imu.gx, 7, 2);
            NEUQ_LCD200_DisplayFloat(60, 160, imu.gy, 7, 2);
            NEUQ_LCD200_DisplayFloat(60, 190, imu.gz, 7, 2);
            break;
        case PAGE_FILTER:
            NEUQ_LCD200_DisplayString(100, 40,  (char*)axis_name[imu.axis]);
            NEUQ_LCD200_DisplayString(100, 70,  imu.send_enable ? "ON " : "OFF");
            NEUQ_LCD200_DisplayFloat(100, 100, imu.alpha, 4, 2);
            NEUQ_LCD200_DisplayString(0, 40,  " ");
            NEUQ_LCD200_DisplayString(0, 70,  " ");
            NEUQ_LCD200_DisplayString(0, 100, " ");
            NEUQ_LCD200_DisplayString(0, 130, " ");
            NEUQ_LCD200_DisplayString(0, 40 + filter_row * 30, ">");
            break;
        case PAGE_ATTITUDE:
            NEUQ_LCD200_DisplayFloat(60, 40,  euler.roll,  7, 2);
            NEUQ_LCD200_DisplayFloat(60, 70,  euler.pitch, 7, 2);
            NEUQ_LCD200_DisplayFloat(60, 100, euler.yaw,   7, 2);
            break;
    }
}