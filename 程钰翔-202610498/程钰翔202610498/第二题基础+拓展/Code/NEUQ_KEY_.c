#include "NEUQ_lib_include.h"
#include "NEUQ_KEY_.h"

#define KEY_NUM  4
#define KEY_DEBOUNCE_TIME  10u   // 按键消抖时间，单位：ms
static  key_type key_table[KEY_NUM]={
    {KEY_UP_PIN,GPIO_High,EVENT_KEY_UP,0},
    {KEY_DOWN_PIN,GPIO_High,EVENT_KEY_DOWN,0},
    {KEY_BACK_PIN,GPIO_High,EVENT_KEY_BACK,0},
    {KEY_IN_PIN,GPIO_High,EVENT_KEY_IN,0},
};

/*extern全工程声明*/  /*volatile易变的，无论是中断还是赋值，让计算机去读最新的内存中的值*/
extern volatile uint8_t  current_key_id=0;   //当前的按键编码
volatile uint32_t g_key_tick=0;              //1ms 系统节拍，PIT1 中断中自增


void  NEUQ_KEY_Init(key_type*key,GPIO_Ports_ENUM pin,KeyEvent_t Press_event){
    key->Port=pin;
    key->Last_level=GPIO_High;
    key->Press_event=Press_event;
    key->Last_change_tick=g_key_tick;
    NEUQ_GPIO_init(pin,GPIO_Mode_Input_PullUp, GPIO_High);
}

void NEUQ_KEY_ALL_Init(void){
    NEUQ_KEY_Init(&key_table[0],KEY_UP_PIN,EVENT_KEY_UP);
    NEUQ_KEY_Init(&key_table[1],KEY_DOWN_PIN,EVENT_KEY_DOWN);
    NEUQ_KEY_Init(&key_table[2],KEY_BACK_PIN,EVENT_KEY_BACK);
    NEUQ_KEY_Init(&key_table[3],KEY_IN_PIN,EVENT_KEY_IN);

}
void NEUQ_KEY_Scan(key_type*key){
    GPIO_Level raw = NEUQ_GPIO_read(key->Port);

  /** if(raw != key->Last_level){
        // 原始电平发生变化，重新开始计时
        key->Last_level = raw;
        key->Last_change_tick = now;
    }else if((now - key->Last_change_tick) >= KEY_DEBOUNCE_TIME){
        // 电平已经稳定超过消抖时间，确认新状态
        if(raw != key->Now_level){
            key->Now_level = raw;
            if(raw == GPIO_Low){              // 上拉输入低电平为按下
                current_key_id = (uint8_t)key->Press_event;
            }
        }
    }*/
     if(raw == key->Last_level){
        // 原始电平发生变化，重新开始计时
        key->Last_change_tick=g_key_tick;
    }else if((g_key_tick - key->Last_change_tick) >= KEY_DEBOUNCE_TIME){
        // 电平已经稳定超过消抖时间，确认新状态
        key->Last_level=raw;
        if(raw == GPIO_Low){              // 上拉输入低电平为按下
            current_key_id = (uint8_t)key->Press_event;//按键里的KeyEvent_t编码赋值给全局变量current_key_id
        }
    }
}
void NEUQ_KEY_ALL_SCAN(void){
    for(int i=0;i<KEY_NUM;i++){
        NEUQ_KEY_Scan(&key_table[i]);//在这里写入按键id
    }
}


/*void NEUQ_KEY_Scan(key_type*key){
    for(int i=0;i<KEY_NUM;i++){
        key_type*key=&key_table[i];
    key->Now_level=NEUQ_GPIO_read(key->Port);
       if((key->Last_level==GPIO_High)&&(key->Now_level==GPIO_Low)){
        if(key->Port==KEY_UP_PIN){current_key_id=1;}
        if(key->Port==KEY_DOWN_PIN){current_key_id=2;}
        if(key->Port==KEY_BACK_PIN){current_key_id=3;}
        if(key->Port==KEY_IN_PIN){current_key_id=4;}
       }key->Last_level=key->Now_level;
    }
}
void NEUQ_KEY_ALL_SCAN(void){
    NEUQ_KEY_Scan(&key_table[0]);
    NEUQ_KEY_Scan(&key_table[1]);
    NEUQ_KEY_Scan(&key_table[2]);
    NEUQ_KEY_Scan(&key_table[3]);
}*/
