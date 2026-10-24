/* board_config.h - 引脚与参数配置 */

#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#define T1_LED_PIN              (PA8)

/* 板载 LED 高电平点亮；若实测相反改为 GPIO_Low */
#define T1_LED_ON_LEVEL         (GPIO_High)

/* 按键：上拉输入，按下为低 */
#define T1_KEY_UP_PIN           (PE9)
#define T1_KEY_DOWN_PIN         (PE7)

/* 计数消抖：10 ms 扫描，连续低电平计数 */
#define T1_KEY_SHORT_MIN_TICKS  (2U)    /* 2 × 10 ms = 20 ms，低于此值视为毛刺 */
#define T1_KEY_LONG_TICKS       (50U)   /* 50 × 10 ms = 500 ms 进入长按 */

/* 置 1 时在页面显示四键电平，用于排查 */
#define T1_SHOW_KEY_LEVELS      (0)

#define T1_TICK_MS              (10U)   /* PIT_CH1(TIM6) 中断周期 */
#define T1_CYCLE_MS             (2000U) /* 每套动作的完整周期 */
#define T1_PATTERN_COUNT        (4U)    /* 方案套数 */
#define T1_TICK_PER_CYCLE       (T1_CYCLE_MS / T1_TICK_MS)

#endif /* _BOARD_CONFIG_H_ */
