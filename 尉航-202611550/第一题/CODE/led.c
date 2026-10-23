#include "led.h"

//LED初始化推挽输出模式
void LED_Init(void){
    NEUQ_GPIO_init(PA8,GPIO_Mode_Output_PushPull,GPIO_Low);
};

uint8_t LED_cnt = 0;

//初始时循环播放方案一
static volatile uint8_t nowplan = 1;
static volatile uint8_t nextplan = 1;

//计数器每10ms增加1，每2000ms清空一次
void LED_cnt_start(void){
    LED_cnt++;
    if(LED_cnt == 200){
        LED_cnt = 0;
    }
};

//编辑四种方案
// 1亮 1000 ms → 灭 1000 ms 
// 2亮 200 ms → 灭 1800 ms 
// 3亮 200 ms → 灭 200 ms → 亮 200 ms → 灭 1400 ms 
// 4亮 200 ms → 灭 200 ms → 亮 600 ms → 灭 1000 ms 
void LED_Shine(void){
    if(LED_cnt == 0){
        nowplan = nextplan;
    }

    switch(nowplan){
        case 1:
            if(LED_cnt <100){
                NEUQ_GPIO_write(PA8,GPIO_High);
            }
            if(LED_cnt >=100){
                NEUQ_GPIO_write(PA8,GPIO_Low);
            }
            break;
        case 2:
            if(LED_cnt <20){
                NEUQ_GPIO_write(PA8,GPIO_High);
            }
            if(LED_cnt >=20){
                NEUQ_GPIO_write(PA8,GPIO_Low);
            }
            break;
        case 3:
             if(LED_cnt <20){
                NEUQ_GPIO_write(PA8,GPIO_High);
            }
            if(LED_cnt >=20 && LED_cnt < 40){
                NEUQ_GPIO_write(PA8,GPIO_Low);
            }
            if(LED_cnt >=40 && LED_cnt < 60){
                NEUQ_GPIO_write(PA8,GPIO_High);
            }
            if(LED_cnt >=60){
                NEUQ_GPIO_write(PA8,GPIO_Low);
            }
            break;
        case 4:
        if(LED_cnt <20){
                NEUQ_GPIO_write(PA8,GPIO_High);
            }
            if(LED_cnt >=20 && LED_cnt < 40){
                NEUQ_GPIO_write(PA8,GPIO_Low);
            }
            if(LED_cnt >=40 && LED_cnt < 100){
                NEUQ_GPIO_write(PA8,GPIO_High);
            }
            if(LED_cnt >=100){
                NEUQ_GPIO_write(PA8,GPIO_Low);
            }
        break;
    }
};

//检测到按键按下时，将nextplan更新
void LED_Update(void){
    KEY_EVENT_ENUM k = KEY_Get();
    if(k == KEY_EVENT_PRESS){
        nextplan++;
        if(nextplan > 4){
            nextplan = 1;
        }
    }
};

//获取目前进行中的方案
uint8_t Getnowplan(void){
    return nowplan;
};
//获取下一次要进行的方案
uint8_t Getnextplan(void){
    return nextplan;
};
