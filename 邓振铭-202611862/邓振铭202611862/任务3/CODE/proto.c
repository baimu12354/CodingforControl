#include "NEUQ_lib_include.h"
#include"stdint.h"

// 下行协议（VOFA -> 单片机）：0xAA LEN CMD DATA[LEN-1] 0xBB
//   LEN = 1(CMD) + 数据字节数；无校验（便于 VOFA 控件用 %% 占位符直接发）
//   CMD: 0x00 停止  0x01 运行  0x02 设目标转速(float)  0x03 设Kp  0x04 设Ki  0x05 设Kd

typedef enum{

    WAIT_HEAD,
    WAIT_LEN,
    WAIT_CMD,
    WAIT_DATA,
    WAIT_TAIL,



}ProtoState;

uint16_t motor_cmd=0;
uint16_t motor_update=0;
float proto_tar_spd=0.0f;
float proto_kp;
float proto_ki;
float proto_kd;

// 接收 FIFO：串口中断只入 FIFO，解析放到主循环
static uint8_t rx_fifo_buf[256];
static NEUQ_FIFO_t rx_fifo;

void NEUQ_Protoco_Init(void){

NEUQ_FIFO_init(&rx_fifo, rx_fifo_buf, sizeof(rx_fifo_buf));
NEUQ_Uart_init(UART_0,UART_BAUD_115200);      // 单串口 UART_0(PA9/PA10)：上行 JustFloat 遥测 + 下行 0xAA 命令
NEUQ_Uart_Enable_RX_Interrupt(UART_0,0);      // 命令接收中断

}

// 串口接收中断调用：只入 FIFO，不做解析
void NEUQ_Protoco_Push(uint8_t dat){
    NEUQ_FIFO_write_byte(&rx_fifo, dat);
}

void NEUQ_Protoco_Analysis(uint8_t dat){
static ProtoState state =WAIT_HEAD;
static uint16_t rx_len;
static uint16_t rx_cmd;
static uint16_t data_cnt;
static uint8_t rx_buf[20];      // 数据缓冲区必须是 uint8_t，否则 float 拼不出来
switch (state)
{
case WAIT_HEAD : {
    if(dat==PROTO_HEAD){
        state=WAIT_LEN;
    }
    break;
}
case WAIT_LEN:{
    rx_len=dat;
    if(rx_len<1 || rx_len>21){   // 1(cmd) + 最多20字节数据，防越界
        state=WAIT_HEAD;
        break;
    }
    data_cnt=rx_len-1;
    state=WAIT_CMD;
    break;
}
case WAIT_CMD:{
rx_cmd=dat;
if(data_cnt>0){
    state=WAIT_DATA;
}
else{
state=WAIT_TAIL;
}
break;
}
case WAIT_DATA:{
    rx_buf[rx_len-1-data_cnt]=dat;
    data_cnt--;
    if(data_cnt==0){
        state=WAIT_TAIL;
    }
    break;
}
case WAIT_TAIL:{
    if(dat==PROTO_TAIL){
        motor_cmd=rx_cmd;
        motor_update=1;   // 有效帧：STOP/RUN/设速/设PID 都触发一次处理
        if(motor_cmd==CMD_SET_TAR_SPD){
            memcpy(&proto_tar_spd,rx_buf,sizeof(float));
        }
        else if(motor_cmd==CMD_SET_KP){
            memcpy(&proto_kp,rx_buf,sizeof(float));
        }
        else if(motor_cmd==CMD_SET_KI){
            memcpy(&proto_ki,rx_buf,sizeof(float));
        }
        else if(motor_cmd==CMD_SET_KD){
            memcpy(&proto_kd,rx_buf,sizeof(float));
        }
    }
    state=WAIT_HEAD;
    break;
}
default:
state=WAIT_HEAD;
}
}

// 上传遥测给 VOFA：JustFloat 裸 float 流（6 通道，各 4 字节小端）
static void Proto_Send_Telemetry(void){
    float data[6];
    uint8_t end_flag[4] = {0x00, 0x00, 0x80, 0x7F};   // JustFloat 帧尾(float +inf)
    data[0]=target_speed;        // 通道0：目标转速
    data[1]=real_speed;          // 通道1：实际转速
    data[2]=(float)motor_state;  // 通道2：电机状态 0=停 1=运行
    data[3]=speed_pid.OUT;       // 通道3：PID原始输出(放大前)
    data[4]=(float)NEUQ_GPIO_read(PC7);  // 通道4：PH 方向电平
    data[5]=(float)NEUQ_GPIO_read(PB1);  // 通道5：nSLEEP 唤醒电平
    NEUQ_Uart_write_data(UART_0,(uint8_t*)data,sizeof(data));
    NEUQ_Uart_write_data(UART_0,end_flag,sizeof(end_flag));   // 追加帧尾
}

void NEUQ_Protoco_Task(void){
    // 1) 主循环解析：从 FIFO 逐字节喂给状态机
    uint8_t dat;
    while(NEUQ_FIFO_read_byte(&rx_fifo,&dat)==NEUQ_FIFO_OK){
        NEUQ_Protoco_Analysis(dat);
    }

    // 2) 处理已解析出的指令
    if(motor_update==1){
switch (motor_cmd)
{
case CMD_MOTOR_STOP:{
motor_state=motor_stop;
target_speed=0.0f;
break;
}
  case CMD_MOTOR_RUN:{
    motor_state=motor_run;
    break;
  }
  case CMD_SET_TAR_SPD:{        // 运行中可改目标转速
    target_speed=proto_tar_spd;
    break;
  }
  case CMD_SET_KP:{
    if(motor_state!=motor_stop) break;   // 只有停机状态才能改 PID
    speed_pid.Kp=proto_kp;
    speed_pid.Ek=0.0f;
    speed_pid.Ek1=0.0f;
    speed_pid.Ek2=0.0f;
    speed_pid.Ek_total=0.0f;
    break;
  }
  case CMD_SET_KI:{
    if(motor_state!=motor_stop) break;
    speed_pid.Ki=proto_ki;
    speed_pid.Ek=0.0f;
    speed_pid.Ek1=0.0f;
    speed_pid.Ek2=0.0f;
    speed_pid.Ek_total=0.0f;
    break;
  }
  case CMD_SET_KD:{
    if(motor_state!=motor_stop) break;
    speed_pid.Kd=proto_kd;
    speed_pid.Ek=0.0f;
    speed_pid.Ek1=0.0f;
    speed_pid.Ek2=0.0f;
    speed_pid.Ek_total=0.0f;
    break;
  }
default:
break;
}
motor_update=0;
    }

    // 3) 周期性上传遥测给 VOFA（JustFloat 裸 float）
    static uint16_t last_send_us=0;
    uint16_t now_us=(uint16_t)NEUQ_Get_Timer_Value_us();
    if((uint16_t)(now_us-last_send_us) >= 20000){   // 20ms；16位差值可正确处理65ms回绕
        last_send_us=now_us;
        Proto_Send_Telemetry();
    }
}
