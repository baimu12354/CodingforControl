#include "NEUQ_lib_include.h"


//写入FIFO
void NEUQ_PROTO_Push(uint8_t data){
    NEUQ_FIFO_write_byte(&NEUQ_PROTO_FIFO, data);
}


//协议解析
static void PROTO_CMD_Apply(uint8_t cmd,const uint8_t *DATA,uint8_t len){
    switch (cmd)
    {
    case PROTO_CMD_START:
        MOTOR_Start();
        break;
    case PROTO_CMD_END:
        MOTOR_Stop();
        break;
    case PROTO_CMD_PID_P:
        if(len==4)
        {
            float Kp =0;
            memcpy(&Kp,DATA,4);
            MOTOR_Set_PID(Kp,motor.pid.Ki, motor.pid.Kd);
        }
        break;
    case PROTO_CMD_PID_I:
        if(len==4)
        {
            float Ki =0;
            memcpy(&Ki,DATA,4);
            MOTOR_Set_PID(motor.pid.Kp, Ki, motor.pid.Kd);
        }
        break;
    case PROTO_CMD_PID_D:
        if(len==4)
        {
            float Kd =0;
            memcpy(&Kd, DATA, 4);
            MOTOR_Set_PID(motor.pid.Kp, motor.pid.Ki, Kd);
        }
        break;
    case PROTO_CMD_RPM:
        if(len==4)
        {
            float RPM =0;
            memcpy(&RPM,DATA,4);
            MOTOR_Set_RPM((int32_t)RPM);
        }
        break;
    default:
        break;
    }
}   


//协议解析状态机
static void PROTO_Recive(uint8_t byte){
    switch (state)
    {
        case Head:
            if(byte==PROTO_HEAD){
                state=Cmd;
                data_cnt=0;
            }
            break;
        case Cmd:
            cmd=byte;
            state=Len;
            break;
        case Len:
            len=byte;
            if(len>DATA_MAX_LEN){
                state=Head;
            }else if(len==0){
                state=Tail;
            }else{
                state=Data;
            }
            break;
        case Data:
            data[data_cnt++]=byte;
            if(data_cnt>=len){
                state=Tail;
            }
            break;
        case Tail:   
            if(byte==PROTO_TAIL){
                PROTO_CMD_Apply(cmd,data,len);
            }
            state=Head;
            break;
    }
}


//初始化FIFO和串口
void NEUQ_PROTO_Init(void){
    NEUQ_FIFO_init(&NEUQ_PROTO_FIFO, PROTO_FIFO_buffer, sizeof(PROTO_FIFO_buffer));
    NEUQ_Uart_init(PROTO_UART,UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(PROTO_UART, 1);
    state=Head;
    cmd=len=data_cnt=0;
    last_upload_ms=0;
}


//发送float
static void PROTO_Send(float f){
    union{float f;uint8_t b[4]; } u;
    u.f =f;
    NEUQ_Uart_write_data(PROTO_UART,u.b,4);
}


//上传vofa
static void PROTO_Upload(void){
    PROTO_Send(motor.target_rpm);//ch0
    PROTO_Send(motor.current_rpm);//ch1
    PROTO_Send(motor.pid.Kp);//ch2
    PROTO_Send(motor.pid.Ki);//ch3
    PROTO_Send(motor.pid.Kd);//ch4

    static uint8_t tail[4]={0x00,0x00,0x80,0x7F};   // 帧尾
    NEUQ_Uart_write_data(PROTO_UART,tail,sizeof(tail));
}


void NEUQ_PROTO_Task(){
    uint8_t byte;
    int8_t cnt=0;
    //读取解析
    while((cnt<=64)&&(NEUQ_FIFO_read_byte(&NEUQ_PROTO_FIFO,&byte)==NEUQ_FIFO_OK)){
        PROTO_Recive(byte);
        cnt++;
    }
    //周期回传
    if((motor_ms-last_upload_ms)>=PROTO_UPLOAD_MS){
        PROTO_Upload();
        last_upload_ms = motor_ms;
    }
}