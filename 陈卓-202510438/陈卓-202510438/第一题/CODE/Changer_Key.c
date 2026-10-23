#include "..\NEUQ_lib_include.h"

STATE KEY_STATE={0,0,0,0};
uint32_t key_lockout_timer[4] = {0};

void Changer_Key_Init(void)
{
    NEUQ_GPIO_init(PE9,GPIO_Mode_Input_PullUp,GPIO_High);
    NEUQ_GPIO_init(PE8,GPIO_Mode_Input_PullUp,GPIO_High);
    NEUQ_GPIO_init(PE7,GPIO_Mode_Input_PullUp,GPIO_High);
    NEUQ_GPIO_init(PE10,GPIO_Mode_Input_PullUp,GPIO_High); 
    
}


/* 消抖时间 = DEBOUNCE_CNT × PIT中断周期
 * 例：PIT周期1ms，取10 → 约10ms消抖（按键抖动一般为5~10ms，可取5~20ms）
 */
#define DEBOUNCE_CNT  10

void KEY_PRESS_DETECT(STATE *KEY_STATE)   /* 按键检测，PIT中断中周期调用 */
{
    /* static变量：函数退出后数值保留，用于“记住”每个按键的历史状态 */
    static unsigned char cnt_up  = 0, cnt_down = 0;
    static unsigned char cnt_in  = 0, cnt_out  = 0;
    static unsigned char lock_up = 0, lock_down = 0;  /* 锁定标志：长按只触发一次 */
    static unsigned char lock_in = 0, lock_out = 0;

    /* ---------------- KEY_UP ---------------- */
    if (NEUQ_GPIO_read(KEY_UP) == GPIO_Low)   /* 采样到低电平 */
    {
        if (lock_up == 0)                     /* 本次按下尚未确认 */
        {
            cnt_up++;
            if (cnt_up >= DEBOUNCE_CNT)       /* 连续N次为低 → 真正按下 */
            {
                KEY_STATE->KEY_UP_STATE = 1;  /* 置位按键事件 */
                lock_up = 1;                  /* 锁定，防止长按重复触发 */
            }
        }
    }
    else                                       /* 高电平：松开或抖动 */
    {
        cnt_up  = 0;                           /* 计数清零（消抖核心） */
        lock_up = 0;                           /* 解锁，等待下次按下 */
    }

    /* ---------------- KEY_DOWN ---------------- */
    if (NEUQ_GPIO_read(KEY_DOWN) == GPIO_Low)
    {
        if (lock_down == 0)
        {
            cnt_down++;
            if (cnt_down >= DEBOUNCE_CNT)
            {
                KEY_STATE->KEY_DOWN_STATE = 1;
                lock_down = 1;
            }
        }
    }
    else
    {
        cnt_down  = 0;
        lock_down = 0;
    }

    /* ---------------- KEY_IN ---------------- */
    if (NEUQ_GPIO_read(KEY_IN) == GPIO_Low)
    {
        if (lock_in == 0)
        {
            cnt_in++;
            if (cnt_in >= DEBOUNCE_CNT)
            {
                KEY_STATE->KEY_IN_STATE = 1;
                lock_in = 1;
            }
        }
    }
    else
    {
        cnt_in  = 0;
        lock_in = 0;
    }

    /* ---------------- KEY_OUT ---------------- */
    if (NEUQ_GPIO_read(KEY_OUT) == GPIO_Low)
    {
        if (lock_out == 0)
        {
            cnt_out++;
            if (cnt_out >= DEBOUNCE_CNT)
            {
                KEY_STATE->KEY_OUT_STATE = 1;
                lock_out = 1;
            }
        }
    }
    else
    {
        cnt_out  = 0;
        lock_out = 0;
    }
}



