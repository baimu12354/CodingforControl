/*********************************************************************************************************************
 * NEUQ_KEY.c —— 按键模块实现【第一题配套】
 *
 * 依赖：NEUQ 库的 NEUQ_GPIO_init / NEUQ_GPIO_read；
 *       引脚宏与事件宏见同目录 NEUQ_KEY.h。
 *
 * 使用方式（三处接线）：
 *   1. 系统初始化时调用一次 NEUQ_KEY_Init();（本工程已在 LED_Menu_Init 中调用）
 *   2. 在 1ms 定时中断回调 NEUQ_PIT_1_CALLBACK 中调用 KEY_Scan();
 *   3. 在主循环 while(1) 里调用 NEUQ_KEY_Get() 消费按键事件。
 ********************************************************************************************************************/
#include "../Library/NEUQ_lib/NEUQ_lib_include.h"   /* NEUQ 库统一入口：包含 GPIO 接口、数据类型定义等 */
#include "NEUQ_KEY.h"                                /* 本模块头文件：引脚宏、事件宏、接口声明 */

/* ============================ 模块内部全局变量（static：仅本文件可见） ============================ */
static volatile uint8_t key_event = KEY_NONE;        /* 按键事件缓存：中断里写入、主循环里读出；
                                                      * volatile 关键字：告诉编译器该变量可能被中断异步修改，
                                                      * 禁止优化成寄存器副本或乱序，保证主循环每次都读到最新值 */

/* ============================ 1. 初始化：配置按键 GPIO ============================ */
void NEUQ_KEY_Init(void)
{
    /* GPIO_Mode_Input_PullUp = 上拉输入：平时引脚被芯片内部上拉电阻拉到高电平（读到 GPIO_High），
     * 按键按下把引脚接到 GND 后读到 GPIO_Low，从而判定"按下"。
     * 第三个参数是初始电平，对输入模式无实际作用，按库函数签名传 GPIO_Low 即可。 */
    NEUQ_GPIO_init(KEY_UP_PIN,   GPIO_Mode_Input_PullUp, GPIO_Low);   /* 上键 PE9：配置为上拉输入 */
    NEUQ_GPIO_init(KEY_DOWN_PIN, GPIO_Mode_Input_PullUp, GPIO_Low);   /* 下键 PE8：配置为上拉输入 */
}

/* ============================ 2. 1ms 中断里调用：采样 + 消抖 + 沿触发 ============================ */
void KEY_Scan(void)
{
    static uint8_t stable   = KEY_NONE;   /* "已经消抖确认"的电平状态：作为连续计数比较的基准；
                                           * static 关键字：函数返回后值仍保留，不会每次调用重新初始化 */
    static uint8_t cnt      = 0;          /* 连续读到"与 stable 相同电平"的次数；每 1ms 调用一次，
                                           * 达到 5 次 = 固定 5ms 消抖时长 */
    static uint8_t reported = 0;          /* 标志位：本次按下是否已经上报过事件（1 = 已上报）；
                                           * 用于"沿触发"——一次按下只产生一个事件，长按不连发 */
    uint8_t now = KEY_NONE;               /* 本次采样得到的原始电平对应的键值：KEY_UP / KEY_DOWN / KEY_NONE */

    /* ---- 第 1 步：读取两个按键的实时电平，合成本次键值 now ---- */
    if      (NEUQ_GPIO_read(KEY_UP_PIN)   == GPIO_Low) now = KEY_UP;   /* PE9 读到低电平 = 上键被按下 */
    else if (NEUQ_GPIO_read(KEY_DOWN_PIN) == GPIO_Low) now = KEY_DOWN; /* 否则再看 PE8：读到低 = 下键被按下 */
    /* 两个键都读到高（都没按）时 now 保持 KEY_NONE（无键按下）；
     * 两个键同时按下时，else if 结构使"上键优先"，行为确定、可预期 */

    /* ---- 第 2 步：消抖判断（本电平 vs 上次确认的稳定电平） ---- */
    if (now == stable) {                  /* 本次电平与稳定电平相同 -> 说明不是抖动，是持续稳定的状态 */
        if (++cnt >= 5) {                 /* 计数加 1 后达到 5 次（连续 5ms 电平不变）-> 消抖完成 */
            if (now != KEY_NONE) {        /* 只有真实按键电平（按下态）才允许上报；
                                           * 无键状态不置 reported，避免上电后第一次按键被误吞 */
                if (!reported) {          /* 且本次按下还没有上报过事件 -> 只在"按下沿"产生一次 */
                    reported = 1;         /* 置位上报标志：本次按下不再产生新事件（长按也不会连发） */
                    if (key_event == KEY_NONE) {    /* 上一个事件还没被主循环消费时，丢弃新沿（防止事件堆积） */
                        key_event = now;  /* 写入按键事件：主循环的 NEUQ_KEY_Get() 会取走它 */
                    }
                }
            }
            cnt = 5;                      /* 钳位：cnt 停止增长，防止 uint8_t 在长按期间溢出回绕 */
        }
    } else {                              /* 本次电平与稳定电平不同 -> 电平刚发生变化 */
        stable = now;                     /* 把稳定状态更新为当前电平，重新开始观察 */
        cnt    = 0;                       /* 计数清零，从 0 重新累计（重新消抖） */
        if (now == KEY_NONE) {            /* 且当前无键按下（说明刚松开） */
            reported = 0;                 /* 清除上报标志：允许下一次按下重新产生事件（支持快速连按） */
        }
    }
    /* 行为小结：
     * 按下 -> 5ms 消抖 -> 产生一次 KEY_UP/KEY_DOWN 事件 -> 长按期间不再产生；
     * 松开 -> 清除 reported -> 再次按下 -> 再次消抖 -> 再次产生事件。 */
}

/* ============================ 3. 主循环里调用：消费按键事件 ============================ */
uint8_t NEUQ_KEY_Get(void)
{
    uint8_t k = key_event;                /* 先读出当前未消费的事件值（KEY_UP / KEY_DOWN / KEY_NONE） */
    key_event = KEY_NONE;                 /* 再清零事件缓存：本事件已被取走（消费），下次查询不会重复返回 */
    return k;                             /* 返回事件值：主循环据此修改待执行方案 */
}
