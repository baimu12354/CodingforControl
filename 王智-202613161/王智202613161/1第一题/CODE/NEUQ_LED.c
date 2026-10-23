#include "NEUQ_lib_include.h"

// 毫秒节拍,中断,每 1ms 增一
extern volatile uint32_t tick;

#define LED_PIN             PA8

typedef struct {
    uint8_t  level;
    uint32_t time;
} LEDConfig;

// 方案结构
typedef struct {
    const LEDConfig *steps;   // 步骤序列
    uint8_t          count;   // 步骤个数
} LEDPlan;

// 4 种方案，与菜单 Plan1~Plan4 一一对应
static const LEDConfig Plan1_Steps[] = { {1,1000}, {0,1000} };
static const LEDConfig Plan2_Steps[] = { {1,200},  {0,1800} };
static const LEDConfig Plan3_Steps[] = { {1,200},  {0,200},  {1,200}, {0,1400} };
static const LEDConfig Plan4_Steps[] = { {1,200},  {0,200},  {1,600}, {0,1000} };

static const LEDPlan PLANS[PlanCount] = {
    [Plan1] = { Plan1_Steps, sizeof(Plan1_Steps)/sizeof(Plan1_Steps[0]) },
    [Plan2] = { Plan2_Steps, sizeof(Plan2_Steps)/sizeof(Plan2_Steps[0]) },
    [Plan3] = { Plan3_Steps, sizeof(Plan3_Steps)/sizeof(Plan3_Steps[0]) },
    [Plan4] = { Plan4_Steps, sizeof(Plan4_Steps)/sizeof(Plan4_Steps[0]) },
};
static uint8_t  step_index = 0;    // 当前执行到第几步
static uint32_t step_start = 0;    // 本步开始时刻（tick）

PLANS_ID work;
char line[32];
void NEUQ_LED_Task(void){
    const LEDPlan *p=&PLANS[work];
    NEUQ_GPIO_write(LED_PIN,p->steps[step_index].level);
    if(tick -step_start >=p->steps[step_index].time){
        step_start += p->steps[step_index].time;
        step_index++;
        if (step_index < p->count){
            NEUQ_GPIO_write(LED_PIN,p->steps[step_index].level);
        } 
    }
    if (step_index >= p->count){
        step_index=0;
        if(Pending==1){
            Working=0;
        }
        work=NEUQ_Menu_WorkGet();
        NEUQ_LCD200_ClearRect(8,232,240,24);     
        snprintf(line,sizeof(line),"%s",working->info->name);
        NEUQ_LCD200_DisplayString(8,232,line); 
    }
}


void NEUQ_LED_Init(void){
    NEUQ_GPIO_init(LED_PIN,GPIO_Mode_Output_PushPull,0);
    tick=0;
    work=0;
}