/*********************************************************************************************************************
* 文件名称          config
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
********************************************************************************************************************/
#ifndef __CONFIG_H__
#define __CONFIG_H__

// ===== LED =====
#define LED_PIN         PA8         // LED 引脚
#define LED_ON          GPIO_High   // LED 点亮
#define LED_OFF         GPIO_Low    // LED 熄灭

// ===== 按键 =====
#define KEY1_PIN        PE10        // 按键1（方案1）
#define KEY2_PIN        PE8         // 按键2（方案2）
#define KEY3_PIN        PE7         // 按键3（方案3）
#define KEY4_PIN        PE9         // 按键4（方案4）
#define KEY_DEBOUNCE_MS 20          // 按键消抖时间(ms)

// ===== 时序 / 方案 =====
#define CYCLE_MS        2000        // 每套动作周期(ms)
#define SCHEME_COUNT    4           // 方案数量

#endif
