
#include"key.h"

uint16_t work=1;       // 当前运行的模式（1~4）
uint32_t time_cnt=0;   // PIT 计数，1 tick = 10ms
extern uint16_t selected;   // 定义在 menu.c，表示当前菜单选中的方案号

void led_init(void){
    NEUQ_GPIO_init(PA8,GPIO_Mode_Output_PushPull,GPIO_Low);
}

void led_set_mode(uint16_t mode){
    if(mode>=1 && mode<=4){
        work = mode;
        time_cnt = 0;
    }
}

void led_task(KEY_EVENT_ENUM key2){
    // 上/下键由菜单负责移动光标，这里只处理 FORWARD 确认执行
    if(key2==KEY_FORWARD_SHORT || key2==KEY_FORWARD_LONG){
        led_set_mode(selected);
    }
}

// 每个模式的 亮/灭 时序，单位：PIT tick（1 tick = 10ms）
// 所有模式周期统一为 200 tick = 2000ms
void led_pit_callback(void){
    time_cnt++;

    GPIO_Level level = GPIO_Low;   // 默认灭
    uint16_t t = time_cnt % 200;   // 当前周期内的时刻

    switch(work){
        case 1:   // sch_1: 亮1000ms 灭1000ms
            level = (t < 100) ? GPIO_High : GPIO_Low;
            break;
        case 2:   // sch_2: 亮200ms 灭1800ms
            level = (t < 20) ? GPIO_High : GPIO_Low;
            break;
        case 3:   // sch_3: 亮200 灭200 亮200 灭1400
            level = ((t < 20) || (t >= 40 && t < 60)) ? GPIO_High : GPIO_Low;
            break;
        case 4:   // sch_4: 亮200 灭200 亮600 灭1000
        default:
            level = ((t < 20) || (t >= 40 && t < 100)) ? GPIO_High : GPIO_Low;
            break;
    }

    NEUQ_GPIO_write(PA8, level);
}
