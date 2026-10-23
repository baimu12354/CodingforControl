
#include"key.h"
 char* sch_name[]={
"sch_1",
"sch_2",
"sch_3",
"sch_4"
};

uint16_t selected=1;
uint8_t menu_update(KEY_EVENT_ENUM *key_out){
    KEY_EVENT_ENUM key;
key=EventGet();
*key_out=key;
if(key==KEY_EVENT_NONE){
    return 0;
}
if(key==KEY_UP_LONG||key==KEY_UP_SHORT){
if(selected==1){
    selected=4;
}else{
selected--;
}
}
if(key==KEY_DOWN_LONG||key==KEY_DOWN_SHORT){
if(selected==4){
    selected=1;
}else{
selected++;
}
}
return 1;
}
void menu_init(void){
NEUQ_LCD200_init();
NEUQ_LCD200_SetPenColor(LCD_BLACK);
NEUQ_LCD200_SetBackColor(LCD_WHITE);
NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
NEUQ_LCD200_Clear();
menu_show();

}
void menu_show(void){
NEUQ_LCD200_Clear();
for(uint16_t i=1;i<=4;i++){
    if(i==selected){
        NEUQ_LCD200_SetPenColor(LCD_RED);
    }
    else{
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
    }
    NEUQ_LCD200_DisplayString(0,(i-1)*40,sch_name[i-1]);
}
}
void menu_task(void){
    KEY_EVENT_ENUM evt;
if(menu_update(&evt)==1){
    menu_show();
    led_task(evt);
}
}

