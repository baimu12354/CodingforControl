#include "NEUQ_lib_include.h"

static GPIO_Ports_ENUM pin[4] = {PE10, PE8, PE7, PE9};  // UP DOWN FORWARD BACK
static uint8_t cnt[4];          // 连续按下的拍数，用来消抖
static volatile uint8_t ev[4];  // 每个键一个事件标志：1 表示还有一次没被取走
//没写初始值的静态变量(static)，程序启动时自动清零

// 上拉输入：松开是高电平，按下是低电平
void Key_Init(void){
    uint8_t i;

    for (i = 0; i < 4; i++)
        NEUQ_GPIO_init(pin[i], GPIO_Mode_Input_PullUp, GPIO_High);
}

// 中断里每 2ms 调用一次，内部分频成 每10ms 扫描一次
void Key_Scan(void){
    static uint8_t p = 0;
    uint8_t i;

    if (++p < 5) return;      // 2ms × 5 = 10ms
    p = 0;

    for (i = 0; i < 4; i++){
        if (NEUQ_GPIO_read(pin[i]) == GPIO_Low){
            if (cnt[i] < 2) cnt[i]++;       // 连续20ms才算按下
        }else{
            if (cnt[i] == 2) ev[i] = 1;     // 松手时记一次事件
            cnt[i] = 0;
        }
    }
}

// 取一个事件，取走就清除
KeyEvent Key_Get(void){
    uint8_t i;

    for (i = 0; i < 4; i++){
        if (ev[i] == 1){
            ev[i] = 0;
            return (KeyEvent)(i + 1);
        }
    }
    return KEY_NONE;
}