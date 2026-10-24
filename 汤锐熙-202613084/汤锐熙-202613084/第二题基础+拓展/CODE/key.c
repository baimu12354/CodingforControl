/*********************************************************************************************************************
* 文件名称   key
* 功能说明   按键扫描实现。按键“低电平有效”（按下 = 低电平）。
* 注意       key_pin 数组顺序必须固定为：上、下、确定、返回。
*********************************************************************************************************************/
#include "key.h"

// 四个按键引脚，按你板子实际接线修改。
// 避开板载Flash(PA5/6/7、PB0)和LCD(PB3/4/5)占用的引脚。
// 数组顺序：上、下、确定、返回
static const GPIO_Ports_ENUM key_pin[4] = { PE7, PE9, PE8, PE10 };

#define KEY_DEBOUNCE_MS   20     // 消抖时间：按下/松开都要电平稳定这么久才算数
#define KEY_REPEAT_GAP_MS 200    // 长按连发间隔：普通按一下(约100ms)不会被误判为长按

void Key_Init(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        NEUQ_GPIO_init(key_pin[i], GPIO_Mode_Input_PullUp, GPIO_High);
    }
}

KeyEvent_t Key_Scan(void)
{
    static uint32_t last_change[4] = {0};   // 电平最近一次变化的时间
    static uint8_t  stable[4]     = {0};    // 消抖确认后的电平：1=按下 0=松开
    static uint8_t  last_raw[4]   = {0};    // 上一次读到的原始电平
    static uint8_t  repeat_on[4]  = {0};    // 是否已进入长按连发
    static uint32_t last_fire[4]  = {0};    // 上次触发(首次/连发)时间
    uint32_t now = HAL_GetTick();

    for (uint8_t k = 0; k < 4; k++) {
        uint8_t raw = (NEUQ_GPIO_read(key_pin[k]) == GPIO_Low);   // 1 = 按下

        if (raw != last_raw[k]) {           // 电平刚变，重新计时消抖
            last_raw[k]    = raw;
            last_change[k] = now;
        }

        if ((now - last_change[k]) >= KEY_DEBOUNCE_MS) {   // 电平已稳定够久才认可
            if (raw != stable[k]) {         // 这是一次新的、已消抖的电平变化
                stable[k] = raw;
                if (raw) {                  // 确认“按下”：首次触发
                    repeat_on[k] = 1;
                    last_fire[k] = now;
                    return (KeyEvent_t)(k + 1);
                }
                repeat_on[k] = 0;           // 确认“松开”：停止连发
            } else if (raw && repeat_on[k] && (now - last_fire[k]) >= KEY_REPEAT_GAP_MS) {
                last_fire[k] = now;         // 长按连发
                return (KeyEvent_t)(k + 1);
            }
        }
    }
    return KEY_NONE;
}
