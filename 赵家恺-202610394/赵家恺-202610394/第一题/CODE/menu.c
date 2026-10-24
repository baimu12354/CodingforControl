#include <stdio.h>
#include <stdint.h>
#include "key.h"         
#include "NEUQ_lib_include.h"        
void NEUQ_Menu_Task(void);                        
/*
给n年后的自己
总结一下，我就是想的让主循环负责显示和这个按键，然后在中断里做一下定时，到时见去解析按键与现有进行换页，然后再2s后改变，把nest给now并解析led。*/

menuparamas paramas = {
    .now  = 1,
    .next = 1,
};

static const menuconfig MenuTable[menucount] = {
    [first]  = {"first",  second, fourth, &paramas.next,1},
    [second] = {"second", third,  first,  &paramas.next,2},
    [third]  = {"third",  fourth, second, &paramas.next,3},
    [fourth] = {"fourth", first,  third,  &paramas.next,4},
};
static menunode selected;
static menunode nodes[menucount];
void NEUQ_Menu_Init(void){
    Menu_BuildLinks();
    selected = nodes[first];
}

void Menu_BuildLinks(void)
{

    uint8_t i = 0;

    for (i = 0; i < menucount; i++)
    {
        nodes[i].info = &MenuTable[i];
        nodes[i].next = &nodes[MenuTable[i].next];
        nodes[i].prev = &nodes[MenuTable[i].up];
    }
}


int8_t  NEUQ_Menu_Update(KEY_EVENT_ENUM key){
const menuconfig *item = selected.info;
     switch (key)
     {
     case KEY_UP_SHORT :
        selected = *(selected.prev);
             return 1;

     case KEY_DOWN_SHORT:              
        selected = *(selected.next);
             return 1;
     case KEY_FORWARD_SHORT:
        *item->xianzai = item->id;
        
             return 1;
     default:
     return 0;
     }
}  






void TURN(int8_t tick)
{
    uint8_t on = 0;                   

    if (tick == 0) paramas.now = paramas.next;  

    if (tick < 0 || tick > 19)         // 拍号不对就按第 0 拍算
        tick = 0;

    if (paramas.now == 1)              // 方案 1：亮 1000ms，灭 1000ms
    {
        if (tick < 10)                 // 前 10 拍（第 0~9 拍）亮
            on = 1;
    }
    else if (paramas.now == 2)         // 方案 2：亮 200ms，灭 1800ms
    {
        if (tick < 2)                  // 只有第 0~1 拍亮
            on = 1;
    }
    else if (paramas.now == 3)         // 方案 3：亮200 灭200 亮200 灭1400
    {
        if (tick < 2 || (tick >= 4 && tick < 6))     // 第 0~1 拍、第 4~5 拍亮
            on = 1;
    }
    else if (paramas.now == 4)         // 方案 4：亮200 灭200 亮600 灭1000
    {
        if (tick < 2 || (tick >= 4 && tick < 10))    // 第 0~1 拍、第 4~9 拍亮
            on = 1;
    }
    if (on)
        NEUQ_GPIO_write(PA8, GPIO_High);    // 亮
    else
        NEUQ_GPIO_write(PA8, GPIO_Low);     // 灭
}

void menushow (void);
 uint8_t menutask(void){
    static int8_t lastnow =1,lastnext= 1 ;
    static uint8_t first = 1;               
    KEY_EVENT_ENUM key = NEUQ_Key_EventGet();
    if(NEUQ_Menu_Update(key) || first||lastnext != paramas.next||lastnow != paramas.now){
        first = 0;   
        lastnext = paramas.next;
        lastnow = paramas.now;                     
        menushow();
    }
    return 1;
}
void menushow (void){
    menunode draw = nodes[first];
    char line [32];
    int32_t y ;
    NEUQ_LCD200_SetPenColor(LCD_BLUE);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);   
    sprintf (line,"now:%d          byzhaoyuan",paramas.now);
    NEUQ_LCD200_ClearRect(10,10,310,24);
    NEUQ_LCD200_DisplayString(10,10,line);
    sprintf (line,"next:%d",paramas.next);
    NEUQ_LCD200_ClearRect(10,40,310,24);
    NEUQ_LCD200_DisplayString(10,40,line);
    do
    {
        sprintf (line,"case:%s",draw.info->zhushi);

        y = 40 + (draw.info->id)*30;
        if (draw.info == selected.info)
        {
            NEUQ_LCD200_SetPenColor(LCD_WHITE);
            NEUQ_LCD200_SetBackColor(LCD_BLUE);

        }else{
            NEUQ_LCD200_SetPenColor(LCD_BLUE);
            NEUQ_LCD200_SetBackColor(LCD_WHITE);            
        }
        
        NEUQ_LCD200_DisplayString(10,y,line);

        draw = *(draw.next);
    } while ( draw.info != nodes[first].info);
}