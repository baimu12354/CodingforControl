#include "key.h"

// 按键状态：1=当前正按着，用于“按一次只触发一次”的等待释放
static uint8_t s_key_pressed = 0;

void key_init(void)
{
    NEUQ_GPIO_init(KEY_UP,      GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_DOWN,    GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_FORWARD, GPIO_Mode_Input_PullUp, GPIO_High);
    NEUQ_GPIO_init(KEY_BACK,    GPIO_Mode_Input_PullUp, GPIO_High);
}

// 扫描按键：返回 KEY_UP/KEY_DOWN/KEY_FORWARD/KEY_BACK，无按键返回 KEY_NONE(0)
uint8_t key_scan(void)
{
    uint8_t key = KEY_NONE;

    if      (NEUQ_GPIO_read(KEY_UP)      == GPIO_Low) key = KEY_UP;
    else if (NEUQ_GPIO_read(KEY_DOWN)    == GPIO_Low) key = KEY_DOWN;
    else if (NEUQ_GPIO_read(KEY_FORWARD) == GPIO_Low) key = KEY_FORWARD;
    else if (NEUQ_GPIO_read(KEY_BACK)    == GPIO_Low) key = KEY_BACK;

    if (key == KEY_NONE) {               // 松开
        s_key_pressed = 0;
        return KEY_NONE;
    }
    if (s_key_pressed) return KEY_NONE;  // 还没松开，返回无，防止一次按下连续触发
    s_key_pressed = 1;
    return key;
}
