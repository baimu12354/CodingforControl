#include "NEUQ_lib_include.h"

static const GPIO_Ports_ENUM keyPins[KEY_NUM] = {PE7, PE8, PE9, PE10};
static const KeyEventEnum releaseEvents[KEY_NUM] = {
    KEY_UP_SHORT, KEY_DOWN_SHORT, KEY_FORWARD_SHORT, KEY_BACK_SHORT
};
static uint8_t pressed[KEY_NUM];
static uint8_t debounce[KEY_NUM];
static volatile KeyEventEnum key_event[KEY_NUM];

void NEUQ_Key_Init(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        NEUQ_GPIO_init(keyPins[i], GPIO_Mode_Input_PullUp, GPIO_High);
        pressed[i] = 0;
        debounce[i] = 0;
        key_event[i] = KEY_EVENT_NONE;
    }
}

void NEUQ_Key_Scan(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        uint8_t sample = (NEUQ_GPIO_read(keyPins[i]) == GPIO_Low);

        if (sample == pressed[i]) {
            debounce[i] = 0;
            continue;
        }

        if (++debounce[i] < 2) continue;

        pressed[i] = sample;
        debounce[i] = 0;
        if (!sample) {
            key_event[i] = releaseEvents[i];
        }
    }
}

KeyEventEnum NEUQ_Key_EventGet(void)
{
    KeyEventEnum event = KEY_EVENT_NONE;
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        if (key_event[i] != KEY_EVENT_NONE) {
            event = key_event[i];
            key_event[i] = KEY_EVENT_NONE;
            break;
        }
    }
    return event;
}
