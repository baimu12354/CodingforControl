#include "NEUQ_lib_include.h"

MotorControl motor = {0};
pid_value    speed_pid = {0};

#define MOTOR_PWM          TIM3_CH1_PC6  // PWM通道:PC6
#define MOTOR_DIR          PD3           // 方向脚DIR:PB12
#define PWM_LIMIT          10000         // PWM最大值
#define PID_MAX_I          12500         // 积分限幅
#define DEFAULT_KP         10.0f         // 默认Kp
#define DEFAULT_KI         0.2f          // 默认Ki
#define DEFAULT_KD         0.0f          // 默认Kd
#define CONTROL_PERIOD_MS  5             // 采样周期
#define UPLOAD_PERIOD_MS   50            // 上传周期
#define ENCODER_PPR        11            // 编码器线数
#define GEAR_RATIO         10.0f         // 减速比
#define MAX_OUTPUT_RPM     982.0f        // 目标=10000对应的实际转速rpm
#define RPM_FILTER_A       0.3f          // 实测转速的一阶低通系数

int32_t last_Enc = 0;              // 编码器计数值
static float rpm_filtered = 0.0f;  // 一阶低通滤波后的实测转速
static uint16_t upload_cnt = 0;    // 上传计数器

// 解包步骤
#define RX_WAIT_H1     0     // 等待帧头1
#define RX_WAIT_H2     1     // 等待帧头2
#define RX_WAIT_CMD    2     // 接收命令码
#define RX_WAIT_LEN    3     // 接收数据长度
#define RX_WAIT_DAT    4     // 接收数据

static uint8_t Rx_State = RX_WAIT_H1;

static uint8_t     Rx_Cmd = 0;    // 命令码
static uint8_t     Rx_Len = 0;    // 数据长度
static uint8_t     Rx_Index = 0;  // 缓冲区数组的成员序号
static uint8_t     Rx_Data[32];   // 拆包临时缓冲区
NEUQ_FIFO_t        Rx_Fifo;       // 缓冲区基本信息
static uint8_t     Rx_Buf[256];   // FIFO存储缓冲区

// 电机输出函数
static void Motor_SetOutput(int32_t pwm){
    if (pwm > PWM_LIMIT) pwm =  PWM_LIMIT;

    if (pwm > 0){                            // 正转
        NEUQ_GPIO_write(MOTOR_DIR, GPIO_High);
        NEUQ_PWM_set_duty(MOTOR_PWM, (uint32_t)pwm);
    }else{                                   // 停转
        NEUQ_GPIO_write(MOTOR_DIR, GPIO_Low);
        NEUQ_PWM_set_duty(MOTOR_PWM, 0);
    }
}

// 电机控制
void Motor_Control(void){
    // 编码器M法测速并换算
    int32_t cnt = NEUQ_Encoder_Read(Encoder_TIM4);
    int32_t delta = cnt - last_Enc;
    last_Enc = cnt;

    // 防止编码器计数溢出 
    if(delta >  32768) delta -= 65536;
    else if(delta < -32768) delta += 65536;

    // delta换算成rpm
    float rpm_raw = (float)delta * (1000.0f / (float)CONTROL_PERIOD_MS)/ (4.0f * (float)ENCODER_PPR) * 60.0f / GEAR_RATIO;
    // 一阶低通滤波
    rpm_filtered += (rpm_raw - rpm_filtered) * RPM_FILTER_A;
    motor.actual_rpm = rpm_filtered;

    // 电机闭环
    if(motor.status){
        float err = (float)motor.target_rpm - motor.actual_rpm;

        speed_pid.Ek_total += err;
        if(speed_pid.Ek_total > PID_MAX_I) speed_pid.Ek_total = PID_MAX_I;
        if(speed_pid.Ek_total < 0) speed_pid.Ek_total = 0;

        float out = motor.kp * err + motor.ki * speed_pid.Ek_total + motor.kd * (err - speed_pid.Ek1);
        speed_pid.Ek1 = err;

        if(out > PWM_LIMIT) out = PWM_LIMIT;
        if(out < 0) out = 0;
        motor.pwm_output = (int32_t)out;
    }else{
        motor.pwm_output = 0;
        speed_pid.Ek_total = 0;
        speed_pid.Ek1 = 0;
    }
    Motor_SetOutput(motor.pwm_output);

    // 通知主循环上传（每 UPLOAD_PERIOD_MS 上传一次）
    upload_cnt++;
    if(upload_cnt >= UPLOAD_PERIOD_MS / CONTROL_PERIOD_MS){  //计算几个采样周期上传一次
        upload_cnt = 0;
        motor.upload_tick = 1;
    }
}

// VOFA+指令实现
static void Apply_Command(uint8_t cmd, const uint8_t *data, uint8_t len){
    switch(cmd){
        case CMD_START:                                   // 启动
            motor.status = 1;
            motor.upload_enable = 1;
            break;
        case CMD_STOP:                                    // 停止
            motor.status = 0;
            motor.pwm_output = 0;
            motor.upload_enable = 0;
            break;
        case CMD_SET_SPEED:                               // 目标转速
            if(len == 4){
                float rpm_f;
                memcpy(&rpm_f, data, 4);
                motor.target_rpm = (int32_t)(rpm_f / 10000.0f * MAX_OUTPUT_RPM);
            }
            break;
        case CMD_SET_KP:                                  // 单独设置Kp
            if(len == 4 && motor.status == 0){
                float kp;
                memcpy(&kp, data, 4);
                motor.kp = kp;
                pid_parameters_setting(&speed_pid, motor.kp, motor.ki, motor.kd);
            }
            break;
        case CMD_SET_KI:                                  // 单独设置Ki
            if(len == 4 && motor.status == 0){
                float ki;
                memcpy(&ki, data, 4);
                motor.ki = ki;
                pid_parameters_setting(&speed_pid, motor.kp, motor.ki, motor.kd);
            }
            break;
        case CMD_SET_KD:                                  // 单独设置Kd
            if(len == 4 && motor.status == 0){
                float kd;
                memcpy(&kd, data, 4);
                motor.kd = kd;
                pid_parameters_setting(&speed_pid, motor.kp, motor.ki, motor.kd);
            }
            break;
        default:
            break;
    }
}

// 解析VOFA指令
static void Parse_Byte(uint8_t a){
    switch(Rx_State){
        case RX_WAIT_H1:
            if(a == FRAME_HEAD1) Rx_State = RX_WAIT_H2;
            break;
        case RX_WAIT_H2:
            if (a == FRAME_HEAD2){
                Rx_State = RX_WAIT_CMD;
            }
            else if (a == FRAME_HEAD1){         // 连续帧头
                Rx_State = RX_WAIT_H2;
            }
            else{                               // 错误帧头
                Rx_State = RX_WAIT_H1;
            }
            break;
        case RX_WAIT_CMD:
            Rx_Cmd = a;
            Rx_State = RX_WAIT_LEN;
            break;
        case RX_WAIT_LEN:
            Rx_Len = a;
            Rx_Index = 0;
            //防止数据溢出
            if(Rx_Len > sizeof(Rx_Data)){
                Rx_State = RX_WAIT_H1;
                break;
            }
            if(Rx_Len == 0){
                Apply_Command(Rx_Cmd, Rx_Data, Rx_Len);
                Rx_State = RX_WAIT_H1;
            }else{
                Rx_State = RX_WAIT_DAT;
            }
            break;
        case RX_WAIT_DAT:
            Rx_Data[Rx_Index] = a;
            Rx_Index++;
            if(Rx_Index >= Rx_Len){
                Apply_Command(Rx_Cmd, Rx_Data, Rx_Len);
                Rx_State = RX_WAIT_H1;
            }
            break;
        default:
            Rx_State = RX_WAIT_H1;
            break;
    }
}

// 缓存区数据更新
void Motor_Process(void){
    uint8_t a;
    while(NEUQ_FIFO_read_byte(&Rx_Fifo, &a) == NEUQ_FIFO_OK){
        Parse_Byte(a);
    }
}

// 数据上传VOFA
void Motor_Upload(void){
    if(motor.upload_tick == 0 || motor.upload_enable == 0) return;
    motor.upload_tick = 0;
    // 以特定结构发给VOFA+
    float ch[5];
    ch[0] = (float)motor.target_rpm;   // 通道0：目标转速
    ch[1] = motor.actual_rpm;          // 通道1：实际转速
    ch[2] = motor.kp;                  // 通道2：Kp
    ch[3] = motor.ki;                  // 通道3：Ki
    ch[4] = motor.kd;                  // 通道4：Kd
    NEUQ_Uart_write_data(UART_0, (uint8_t *)&ch[0], sizeof(float) * 5);
    
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    NEUQ_Uart_write_data(UART_0, tail, 4);
}

//电机初始化
void Motor_Init(void){
    NEUQ_GPIO_init(MOTOR_DIR, GPIO_Mode_Output_PushPull, GPIO_Low);

    // PWM初始化
    NEUQ_PWM_init(MOTOR_PWM, 8000, 0);

    // 正交编码器初始化（定时器4）
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);
    last_Enc = NEUQ_Encoder_Read(Encoder_TIM4);
    Motor_SetOutput(0);

    // 串口初始化
    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 1);

    // FIFO初始化
    NEUQ_FIFO_init(&Rx_Fifo, Rx_Buf, sizeof(Rx_Buf));

    // PID设置（上电默认值，运行中可通过串口字节包修改）
    motor.kp = DEFAULT_KP;
    motor.ki = DEFAULT_KI;
    motor.kd = DEFAULT_KD;
    pid_parameters_setting(&speed_pid, motor.kp, motor.ki, motor.kd);

    // 上电默认开启上传
    motor.upload_enable = 1;
}