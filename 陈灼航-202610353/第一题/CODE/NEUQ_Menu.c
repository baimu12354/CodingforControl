#include "NEUQ_lib_include.h"

// 屏幕坐标定义
#define MENU_X_LABEL   8    //标签列
#define MENU_X_VALUE   96   //数值列
#define MENU_Y_CUR     48   //"当前方案"这一行的 y 坐标
#define MENU_Y_PEND    88   //"待执行方案"这一行的 y 坐标
#define MENU_VALUE_W   200  //数值区域清除宽度
#define MENU_VALUE_H   16   //数值区域清除高度

static uint8_t last_current; //上次显示的当前方案
static uint8_t last_pending; //上次显示的待执行方案

//清除
static void Menu_ClearValue(uint16_t x, uint16_t y){
    NEUQ_LCD200_SetBackColor(LCD_WHITE);                     //设背景色为白色
    NEUQ_LCD200_ClearRect(x, y, MENU_VALUE_W, MENU_VALUE_H); //用背景色填充该区域
}

//展示
static void Menu_ShowValue(uint16_t x, uint16_t y, char *str){
    Menu_ClearValue(x, y);                //第一步：擦掉旧内容
    NEUQ_LCD200_SetPenColor(LCD_BLACK);   //设前景色为黑色
    NEUQ_LCD200_DisplayString(x, y, str); //第二步：写入新字符串
}

static void Menu_ShowScheme(void){
    char line[16]; //临时缓冲区

    //CUR行
    snprintf(line, sizeof(line), "SCHEME_%d", NEUQ_LED_GetCurrent());
    Menu_ShowValue(MENU_X_VALUE, MENU_Y_CUR, line);

    //PEND行
    snprintf(line, sizeof(line), "SCHEME_%d", NEUQ_LED_GetPending());
    Menu_ShowValue(MENU_X_VALUE, MENU_Y_PEND, line);

    last_current = NEUQ_LED_GetCurrent();
    last_pending = NEUQ_LED_GetPending();
}

//菜单初始化
void NEUQ_Menu_Init(void){
    //初始化屏幕
    NEUQ_LCD200_init();                        //LCD 底层初始化
    NEUQ_LCD200_SetPenColor(LCD_BLACK);        //前景色：黑色
    NEUQ_LCD200_SetBackColor(LCD_WHITE);       //背景色：白色
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);//方向：横屏（宽320 高240）
    NEUQ_LCD200_Clear();                       //整屏清成背景色

    //显示标题
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);   
    NEUQ_LCD200_DisplayString(8, 8, "LED TIMING"); 

    //显示标签和提示
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16); 
    NEUQ_LCD200_DisplayString(MENU_X_LABEL, MENU_Y_CUR,  "CUR :"); 
    NEUQ_LCD200_DisplayString(MENU_X_LABEL, MENU_Y_PEND, "PEND:"); 
    NEUQ_LCD200_DisplayString(MENU_X_LABEL, 176,         "UP/DN: SELECT");

    //显示方案编号
    last_current = 0; 
    last_pending = 0;
    Menu_ShowScheme();
}

//菜单任务
void NEUQ_Menu_Task(void){
    KEY_EVENT_ENUM key = NEUQ_Key_EventGet();

    switch(key){
        case KEY_UP_SHORT:{       
            uint8_t p = NEUQ_LED_GetPending(); 
            p = (p <= LED_SCHEME_1) ? LED_SCHEME_NUM : p - 1; 
            NEUQ_LED_SetPending(p);        
            break;
        }
        case KEY_DOWN_SHORT:{      
            uint8_t p = NEUQ_LED_GetPending(); 
            p = (p >= LED_SCHEME_NUM) ? LED_SCHEME_1 : p + 1; 
            NEUQ_LED_SetPending(p);           
            break;
        }
        default:
            break; 
    }

    if(NEUQ_LED_GetPending() != last_pending || NEUQ_LED_GetCurrent() != last_current){
        Menu_ShowScheme(); 
    }
}