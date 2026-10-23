#include "protocol.h"
typedef enum{
    HEAD = 0,
    CMD,
    LEN,
    DATA,
    CHECK,
    TAIL,
}ANA_State;
volatile float rgd = 0.0f;
volatile uint8_t flag = 0;
static ANA_State State = HEAD;
static uint8_t checksum = 0;
static uint8_t rx_checksum = 0;
static uint8_t cmd,len,data[12],data_idx = 0; 
PositionalPID wheel_pid = {0};

void wheel_pid_reset(PositionalPID *p)
{
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->prev_output = 0.0f;
}

float wheel_pid_update(PositionalPID *p, float setpoint, float actual)
{
    float error = setpoint - actual;
    float output;
    float delta;

    p->integral += error;
    if (p->integral > 10000.0f)  p->integral = 10000.0f;
    if (p->integral < -10000.0f) p->integral = -10000.0f;

    output = p->Kp * error
           + p->Ki * p->integral
           + p->Kd * (error - p->prev_error);

    if (output > 10000.0f) output = 10000.0f;
    if (output < 0.0f)     output = 0.0f;

    delta = output - p->prev_output;
    if (delta > PID_MAX_STEP)  output = p->prev_output + PID_MAX_STEP;
    if (delta < -PID_MAX_STEP) output = p->prev_output - PID_MAX_STEP;
    p->prev_output = output;

    p->prev_error = error;
    return output;
}

float Get_float(uint8_t *Data)
{
union{float f;uint8_t d[4];}u;
u.d[0] = Data[0];
u.d[1] = Data[1];
u.d[2] = Data[2];
u.d[3] = Data[3];
return u.f;

}

void Handle_Data(void){
switch (cmd)
{
case START:
    wheel_pid_reset(&wheel_pid);
    flag = 1;
    printf("2.0\n");
    break;
case STOP:

   flag = 0;
    NEUQ_PWM_set_duty(TIM2_CH3_PA2,0);
    printf("1.0\n");
    break;
case SPEED:
    if (len == 4) {
        rgd = Get_float(data);
        if (rgd<0){
            NEUQ_GPIO_write(PC7,GPIO_Low);
        }
        else{NEUQ_GPIO_write(PC7,GPIO_High);}
       
    }
    break;
case KP:
    if (len == 4&&flag == 0) {
        wheel_pid.Kp = Get_float(data);
        printf("KP:%f\n",wheel_pid.Kp);
    }
    break;
case KI:
    if (len == 4&&flag == 0) {
        wheel_pid.Ki = Get_float(data);
        printf("KI:%f\n", wheel_pid.Ki);
    }
    break;
case KD:
    if (len == 4&&flag == 0) {
        wheel_pid.Kd = Get_float(data);
        printf("KD:%f\n", wheel_pid.Kd);
    }
    break;

}
}


void Analysis_Data(uint8_t b)
{
switch (State)
{
case HEAD:
    if (b == 0xAA)
    {
        checksum = 0;
        State = CMD;
    }
    
    break;

case CMD:
   cmd = b;
   checksum += b;
   State = LEN;
   break;

case LEN:
   len = b;
   checksum += b;
   data_idx = 0;
   if (len > sizeof(data)) {
       State = HEAD;
   } else {
       State = (len == 0) ? CHECK : DATA;
   }
   break;

case DATA:
   
   if (data_idx >=len)
   {
    State = CHECK;
   }
   break;

case CHECK:
   rx_checksum = b;
   State = TAIL;
   break;

case TAIL:
   if (b == 0xBB && rx_checksum == checksum)
   {
    Handle_Data();
   
   }
   
   State = HEAD;
   break;
}
}

