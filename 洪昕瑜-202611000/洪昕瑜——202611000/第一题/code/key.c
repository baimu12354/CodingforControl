#include "key.h"
#include "code.h"
#include "config.h"
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"

//--------------------------- 按键状态 ---------------------------
static const GPIO_Ports_ENUM key_pins[4] = {KEY1_PIN, KEY2_PIN, KEY3_PIN, KEY4_PIN};
static uint8_t  key_state[4] = {0, 0, 0, 0}; // 每键状态：0释放 1按下消抖 2按下稳定 3释放消抖
static uint32_t key_tick[4]  = {0, 0, 0, 0}; // 每键进入当前状态的时刻(ms)
static uint8_t  key_event    = 0;            // 一次有效按下：1~4 对应按键编号

//----------------------------------------------------------------
// 函数简介     扫描按键、消抖
//----------------------------------------------------------------
void Key_Scan(void){
    uint32_t now = Tick_ms;
    for (uint8_t k = 0; k < 4; k++){                     // 依次扫描 4 个按键
        uint8_t pressed = (NEUQ_GPIO_read(key_pins[k]) == GPIO_Low); // 低电平=按下
        switch (key_state[k]){
            case 0:                                      // 释放
                if (pressed) { key_state[k] = 1; key_tick[k] = now; }
                break;
            case 1:                                      // 按下消抖
                if (!pressed){
                    key_state[k] = 0;
                } else if ((now - key_tick[k]) >= KEY_DEBOUNCE_MS){
                    key_state[k] = 2;                    // 按下稳定
                    key_event    = k + 1;                // 记录按键编号 1~4
                }
                break;
            case 2:                                      // 按下稳定
                if (!pressed) { key_state[k] = 3; key_tick[k] = now; }
                break;
            case 3:                                      // 释放消抖
                if (pressed){
                    key_state[k] = 2;
                } else if ((now - key_tick[k]) >= KEY_DEBOUNCE_MS){
                    key_state[k] = 0;                    // 释放稳定
                }
                break;
        }
    }
}

//----------------------------------------------------------------
// 函数简介     取走并清除按键事件
// 返回参数     1~4 对应按键编号，无事件返回 0
//----------------------------------------------------------------
uint8_t Key_GetEvent(void){
    uint8_t e = key_event;
    key_event = 0;
    return e;
}
