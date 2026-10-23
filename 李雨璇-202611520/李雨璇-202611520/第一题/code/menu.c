#include "NEUQ_lib_include.h"

#define LED_PIN       PD12
#define LED_ON_LEVEL  GPIO_High
#define LED_OFF_LEVEL GPIO_Low

typedef struct{
    uint8_t  on;     //0:灭  1:亮
    uint16_t time;   //该阶段持续时间(ms)
}LED_Phase;

//四套闪烁方案表
static const LED_Phase SCHEME_1[] = {{1,1000},{0,1000}};                     //亮1000ms -> 灭1000ms
static const LED_Phase SCHEME_2[] = {{1,200},{0,1800}};                      //亮200ms -> 灭1800ms
static const LED_Phase SCHEME_3[] = {{1,200},{0,200},{1,200},{0,1400}};      //亮200ms -> 灭200ms -> 亮200ms -> 灭1400ms
static const LED_Phase SCHEME_4[] = {{1,200},{0,200},{1,600},{0,1000}};      //亮200ms -> 灭200ms -> 亮600ms -> 灭1000ms

static const LED_Phase *SCHEMES[4] = {SCHEME_1, SCHEME_2, SCHEME_3, SCHEME_4};
static uint8_t SCHEME_PHASE_NUM[4] = {2, 2, 4, 4};  //当前方案的阶段总数

static uint8_t s_current  = 0;   //当前正执行方案号（0-3）
static uint8_t s_pending  = 0;   //待执行方案号（0-3）
static uint8_t s_phase    = 0;   //正执行方案的当前阶段数
static uint16_t s_running = 0;   //当前阶段已运行毫秒数
static uint8_t s_shown_current = 0;   //已显示到屏幕上的当前方案
static uint8_t s_shown_pending = 0;   //已显示到屏幕上的待执行方案

static void Led_Apply(uint8_t on){
    NEUQ_GPIO_write(LED_PIN, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

//当前正执行方案
static void Menu_ShowCurrent(void){
    char buf[4];
    snprintf(buf, sizeof(buf), "%d", s_current + 1);
    NEUQ_LCD200_ClearRect(192, 10, 100, 24);
    NEUQ_LCD200_DisplayString(192, 10, buf);
    s_shown_current = s_current;
}

//待执行方案
static void Menu_ShowPending(void){
    char buf[2];
    snprintf(buf, sizeof(buf), "%d", s_pending + 1);
    NEUQ_LCD200_ClearRect(192, 50, 100, 24);
    NEUQ_LCD200_DisplayString(192, 50, buf);
    s_shown_pending = s_pending;
}

void Menu_Init(void){
    NEUQ_GPIO_init(LED_PIN, GPIO_Mode_Output_PushPull, LED_OFF_LEVEL);
   
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_Clear();

    NEUQ_LCD200_DisplayString(8, 10,  "Current Scheme:");
    NEUQ_LCD200_DisplayString(8, 50, "Pending Scheme:");

    s_current = 0;
    s_pending = 0;
    s_phase   = 0;
    s_running = 0;
    Led_Apply(SCHEMES[0][0].on);

    Menu_ShowCurrent();
    Menu_ShowPending();
}

void Menu_Tick(void){
    const LED_Phase *scheme = SCHEMES[s_current];
    uint8_t phase_num = SCHEME_PHASE_NUM[s_current];

    s_running++;
    if (s_running >= scheme[s_phase].time){
        s_running = 0;
        s_phase++;
        if (s_phase >= phase_num){
            s_phase   = 0;
            s_current = s_pending;
            scheme    = SCHEMES[s_current];
        }
        Led_Apply(scheme[s_phase].on);
    }
}

void Menu_Task(void){
    KEY_EVENT key = Key_EventGet();
    
     if (key != KEY_EVENT_NONE){
        uint8_t target = key - 1;
        if(target != s_pending){
            s_pending = target;   //按键修改待执行方案
        }
    }

    if (s_current != s_shown_current){
        Menu_ShowCurrent();
    }

    if (s_pending != s_shown_pending){
        Menu_ShowPending();
    }
}