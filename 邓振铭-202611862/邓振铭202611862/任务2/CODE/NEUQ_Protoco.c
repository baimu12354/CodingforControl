#include"NEUQ_lib_include.h"
#include"NEUQ_Protoco.h"

typedef enum{

    WAIT_HEAD,
    WAIT_LEM,
    WAIT_CMD,
    WAIT_CHECK,
    WAIT_TAIL,



}ProtoState;

uint8_t led_cmd=0;
uint8_t led_update=0;

static void LED_off(void){
    NEUQ_PWM_set_duty(LED_PORT,0);
}

static void LED_single(void){

    while(1){

        NEUQ_PWM_set_duty(LED_PORT,10000);
        NEUQ_Delay_ms(500);

        NEUQ_PWM_set_duty(LED_PORT,0);
         NEUQ_Delay_ms(500);

if(led_update){
    return;
}
    }

}
static void LED_doouble(void){

    while(1){

        NEUQ_PWM_set_duty(LED_PORT,10000);
        NEUQ_Delay_ms(200);

        NEUQ_PWM_set_duty(LED_PORT,0);
         NEUQ_Delay_ms(200);
NEUQ_PWM_set_duty(LED_PORT,10000);
        NEUQ_Delay_ms(200);

        NEUQ_PWM_set_duty(LED_PORT,0);
         NEUQ_Delay_ms(400);

if(led_update){
    return;
}
    }

}

static void LED_breath(void){
    while(1){
        for(uint16_t i=0;i<1000;i++){
            NEUQ_PWM_set_duty(LED_PORT,i*10);
            NEUQ_Delay_ms(1);
        }
         for(uint16_t i=1000;i>0;i--){
            NEUQ_PWM_set_duty(LED_PORT,i*10);
            NEUQ_Delay_ms(1);
        }
        if(led_update){
    return;
}
    }
}



void NEUQ_Protoco_Init(void){
    NEUQ_PWM_init(LED_PORT,1000,0);
    NEUQ_Uart_init(UART_0,UART_BAUD_115200);
     NEUQ_Uart_Enable_RX_Interrupt(UART_0,0);
}
void NEUQ_Protoco_Analysis(uint8_t dat){
    static ProtoState state = WAIT_HEAD;
    static uint8_t rx_len = 0;
    static uint8_t rx_cmd =0;
    static uint8_t rx_check =0;
    switch (state)
    {
    case WAIT_HEAD:{
        if(dat==PROTO_HEAD){
            state=WAIT_LEM;

        }
        break;
    }
        case WAIT_LEM:{
            rx_len=dat;
            state=WAIT_CMD;
            break;
        }
        case WAIT_CMD:{
            rx_cmd=dat;
            state =WAIT_CHECK;
            break;
        }
    case WAIT_CHECK:{
        rx_check=dat;
        state=WAIT_TAIL;
        break;

    }
    case WAIT_TAIL:{
        if((dat==PROTO_TAIL)&&(rx_check==(uint8_t)(rx_len+rx_cmd))){
led_cmd=rx_cmd;
led_update=1;
        }
        state=WAIT_HEAD;
        break;
    }
    }
}
void NEUQ_Protoco_Task(void){
    if(led_update==1){
        led_update=0;
        switch (led_cmd)
        {
        case CMD_OFF:{
            LED_off();
            break;
        }
          case CMD_SINGE:{
            LED_single();
            break;
          }
          case CMD_DOUBLE:{
            LED_doouble();
            break;
          }
          case CMD_BREATH:{
            LED_breath();
            break;

          }
          default:{
            break;
          }
        }
    }
}