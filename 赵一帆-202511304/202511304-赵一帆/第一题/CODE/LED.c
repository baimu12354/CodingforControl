#include "LED.h"

LedSystem_t ledSys = {0, 0, 0};
static KeyEvent_t key_event = KEY_NONE;

void LED_Init(void) {
    NEUQ_GPIO_init(LED_PIN,       GPIO_Mode_Output_PushPull, GPIO_High);
    NEUQ_GPIO_init(KEY_UP_PIN,    GPIO_Mode_Input_PullUp,  GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN_PIN,  GPIO_Mode_Input_PullUp,  GPIO_High);
    NEUQ_GPIO_init(KEY_ENTER_PIN, GPIO_Mode_Input_PullUp,  GPIO_High);
    NEUQ_GPIO_init(KEY_BACK_PIN,  GPIO_Mode_Input_PullUp,  GPIO_High);
}

void LED_Task(void) {        
    switch (ledSys.current_mode) {
        case 0: // 亮1000 灭1000
            NEUQ_GPIO_write(LED_PIN, (ledSys.tick < 1000) ? GPIO_High : GPIO_Low);
            break;
        case 1: // 亮200 灭1800
            NEUQ_GPIO_write(LED_PIN, (ledSys.tick < 200) ? GPIO_High : GPIO_Low);
            break;
        case 2: // 亮200 灭200 亮200 灭1400
            if (ledSys.tick < 200)      NEUQ_GPIO_write(LED_PIN, GPIO_High);
            else if (ledSys.tick < 400) NEUQ_GPIO_write(LED_PIN, GPIO_Low);
            else if (ledSys.tick < 600) NEUQ_GPIO_write(LED_PIN, GPIO_High);
            else                        NEUQ_GPIO_write(LED_PIN, GPIO_Low);
            break;
        case 3: // 亮200 灭200 亮600 灭1000
            if (ledSys.tick < 200)       NEUQ_GPIO_write(LED_PIN, GPIO_High);
            else if (ledSys.tick < 400)  NEUQ_GPIO_write(LED_PIN, GPIO_Low);
            else if (ledSys.tick < 1000) NEUQ_GPIO_write(LED_PIN, GPIO_High);
            else                         NEUQ_GPIO_write(LED_PIN, GPIO_Low);
            break;
        default: break;
    }
}

void Key_Scan(void) {               // 消抖 20ms，松开产生一次事件
    static uint8_t  state = 0;
    static uint32_t press_tick = 0;
    static uint8_t  pressed_key = 0;

    uint8_t up    = (NEUQ_GPIO_read(KEY_UP_PIN)    == GPIO_Low);
    uint8_t down  = (NEUQ_GPIO_read(KEY_DOWN_PIN)  == GPIO_Low);
    uint8_t enter = (NEUQ_GPIO_read(KEY_ENTER_PIN) == GPIO_Low);
    uint8_t back  = (NEUQ_GPIO_read(KEY_BACK_PIN)  == GPIO_Low);
    uint8_t any   = up || down || enter || back;

    switch (state) {
        case 0: // 记录按键按下
            if (any) {
                press_tick = HAL_GetTick();
                if (up)          pressed_key = 1;
                else if (down)   pressed_key = 2;
                else if (enter)  pressed_key = 3;
                else if (back)   pressed_key = 4;
                state = 1;
            }
            break;
        case 1: // 消抖确认
            if (!any) state = 0;
            else if (HAL_GetTick() - press_tick >= 20) state = 2;
            break;
        case 2: // 等待释放
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

void LED_Menu_Task(void) {
    if (key_event == KEY_UP || key_event == KEY_DOWN)
        ledSys.target_mode = (ledSys.target_mode + 1) % 4;
    key_event = KEY_NONE;
}

void LED_LCD_Task(void) {
    NEUQ_LCD200_DisplayString(10, 10, "Task1: LED Control");
    NEUQ_LCD200_DisplayString(10, 50, "Current:");
    NEUQ_LCD200_Display_int32(100, 50, ledSys.current_mode + 1, 1);
    NEUQ_LCD200_DisplayString(10, 80, "Target:");
    NEUQ_LCD200_Display_int32(100, 80, ledSys.target_mode + 1, 1);
    NEUQ_LCD200_DisplayString(10, 150, "UP/DOWN: change");
}