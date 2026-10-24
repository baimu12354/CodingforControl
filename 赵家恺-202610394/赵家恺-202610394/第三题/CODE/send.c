#include "NEUQ_lib_include.h"
#include "send.h"
#include <stdint.h>


static int8_t cmdin;
static int8_t open1,open3 = 0;
static int8_t open2,open4= 0;
static uint8_t b = 0;
static uint8_t a[12];
static NEUQ_FIFO_t s_fifo;            
static uint8_t     s_fifo_buf[256];   
static float kp, ki, kd;
static int32_t goalzancun = 0;
volatile float p =0, i =0, d= 0;/*FINAL*/
volatile int32_t goal;/*FINAL*/
volatile float actual = 0.0f;/*final*/
static   float  pwm_out = 0.0f;           /* 累计输出 */
static   int8_t dir_last = 0;             /* 方向脚上次电平1  0  -1 */
IncrementalPID pid_a;
volatile uint8_t run = 0;
volatile int8_t sendflag ;


void NEUQ_SEND_INIT (void){
    NEUQ_Uart_init(UART_0,UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0,0);
    NEUQ_FIFO_init(&s_fifo, s_fifo_buf, sizeof(s_fifo_buf));
}
typedef enum state{
    WAITHEAD,
    WAITCMD,
    WATILEN,
    WAITTAIL,
}SENDSTATE;
void NEUQ_SEND_PUSH   (uint8_t dat){
    NEUQ_FIFO_write_byte(&s_fifo,dat);
}
void NEUQ_SEND_ANAL(uint8_t dat);/*WEILGUOBIANYI*/
void NEUQ_FEED_ANAL (void){
    uint8_t c ;
    while (NEUQ_FIFO_read_byte(&s_fifo,&c) == NEUQ_FIFO_OK){
        NEUQ_SEND_ANAL(c);
    }
    if (run == 0 ){Ipid_parameters_setting(&pid_a, p, i, d); }

}
void NEUQ_SEND_ANAL(uint8_t dat){

    static SENDSTATE SST = WAITHEAD;
    switch (SST)
    {
    case WAITHEAD:
    if ( dat == head)
    {   SST = WAITCMD;}
    break;
    case WAITCMD:
    open1 = open2 = open3 = open4 = 0;b = 0;
    if ( dat == cmd_on)
    {   
        Ipid_parameters_setting(&pid_a,p,i,d);
        pwm_out = 0.0f;
        run = 1;
        SST = WAITTAIL;
    }
    else if (dat == cmd_off)
    {
        /*guanji*/
        run = 0;
        SST = WAITTAIL;
    }
    else if (dat == cmd_set_goal)
    {
        SST = WATILEN;open1 = 1; open2 = 0; b = 0;open3 = 1 ; open4 = 0;
    }
    else if (dat == cmd_set_pid)
    {   if (run == 1){SST = WAITHEAD;break;}   /* 运行中拒绝修改PID */
        SST = WATILEN;open1 = 0; open2 = 1; b = 0;open3 = 0 ; open4 = 1;
    }
    else {
        SST = WAITHEAD;
    }
    break;
    case WATILEN:
    if (  open1 == 1)
    {
    a[b++] = dat;
    if (b>=4){      
    int32_t u = ((uint32_t)a[0]|((uint32_t)a[1] << 8)|(uint32_t)a[2] << 16|(uint32_t)a[3]<<24);
    goalzancun = u;
    open1 = 0;
    SST = WAITTAIL;
    u = 0;
    b = 0;
    }}
    else if (open2 == 1)
    {
    a[b++] = dat;
    if (b>=12){      
    memcpy(&kp, &a[0],  4);
    memcpy(&ki, &a[4], 4);
    memcpy(&kd, &a[8], 4);

    b = 0;
    open2 = 0;
    SST = WAITTAIL;
    }}
    else SST = WAITHEAD;
    break;
    case (WAITTAIL):
    if(dat == tail){
    if (open4 == 1)
    {
        p = kp;
        i = ki; 
        d = kd;
       open4 = 0;
        SST = WAITHEAD;
        }else if (open3 == 1)
    {   goal = goalzancun;
        open3 = 0;
        SST = WAITHEAD;
    }
    }   SST = WAITHEAD;
        open3= 0;open4 = 0;open1= 0;open2 = 0;   }  

    
}
/*解包*/





void pid_task(void){

    int32_t detla = (int32_t)NEUQ_Encoder_Read(Encoder_TIM4);
    NEUQ_Encoder_Clear(Encoder_TIM4);
    actual = (float)detla * RPM_PER_COUNT;  
    if (run == 0){
    pwm_out = 0.0f;
    NEUQ_PWM_set_duty(PWM_CH, 0);
    return;
    }
    float du = IncrementalPID_Update(&pid_a,(float)goal,actual);
    pwm_out += du;
    if ( pwm_out>(float)PWM_MAX)
    {
     pwm_out=(float)PWM_MAX;
    }
    if ( pwm_out<-(float)PWM_MAX)
    {
     pwm_out= -(float)PWM_MAX;
    }
    if ( pwm_out >= 0.0f)
    {
        if ( dir_last != 1)
        {
        NEUQ_GPIO_write(DIR_PIN, GPIO_Low); dir_last = 1;
        }
        NEUQ_PWM_set_duty(PWM_CH, (uint32_t)pwm_out);
    }else  {
        if ( dir_last != -1)
        {
        NEUQ_GPIO_write(DIR_PIN, GPIO_High); dir_last = -1;
        }
        NEUQ_PWM_set_duty(PWM_CH, (int32_t)-pwm_out);
    }
}

void NEUQ_SEND_TASK (void){
   if (sendflag == 1 )
   {sendflag = 0;
   }else{return ;}
   
   static uint32_t last = 0;
   uint8_t k;
   uint8_t tx[24];
   float   c[5];

   c[0] = (float)goal;  
   c[1] = actual;       
   c[2] = pid_a.Kp;      
   c[3] = pid_a.Ki;
   c[4] = pid_a.Kd;

   for (k = 0U; k < 5U; k++){
       memcpy(&tx[k * 4U], &c[k], 4U);
   }
   tx[20] = 0x00U;      
   tx[21] = 0x00U;
   tx[22] = 0x80U;
   tx[23] = 0x7FU;

   NEUQ_Uart_write_data(UART_0, tx, 24U);
}/*执行对接这个pid模块，发送*/