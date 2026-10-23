#ifndef _CHANGER_KEY_H_
#define _CHANGER_KEY_H_

#define KEY_UP      (PE7)
#define KEY_DOWN     (PE8)
#define KEY_IN      (PE9)
#define KEY_OUT     (PE10)



typedef struct
{
    uint8_t KEY_UP_STATE;
    uint8_t KEY_DOWN_STATE;
    uint8_t KEY_IN_STATE;
    uint8_t KEY_OUT_STATE;
}STATE;

extern STATE KEY_STATE;
extern uint32_t  key_lockout_timer[4];

void Changer_Key_Init();
void KEY_PRESS_DETECT(STATE *KEY_STATE);
uint8_t KEY_EXIT_Debounce(uint8_t key_index,uint32_t count);
#endif 
