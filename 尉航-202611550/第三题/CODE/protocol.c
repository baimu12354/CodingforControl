#include "protocol.h"


static volatile uint8_t g_run    = 0;       
static volatile float   g_target = 0.0f;    // 目标PWM
static volatile float   g_actual = 0.0f;    // 实际PWM
static uint8_t g_locked = 0;                //锁定更改
static float   g_kp = 0.1f, g_ki = 0.01f, g_kd = 0.0f;
static IncrementalPID g_pid;
static int32_t g_last_cnt = 0;
static float   g_pwm_acc  = 0.0f;

/* 串口中断收到完整一帧后放这里，再置 g_frame_ready */
static volatile uint8_t g_frame_ready = 0;
static volatile uint8_t g_cmd = 0;
static volatile uint8_t g_len = 0;
static uint8_t g_data[32];



static float GetFloat(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}
//控制环
void control_loop(void)
{
    if (!g_run) {
        NEUQ_PWM_set_duty(MACHINE_PORT, 0);
        g_pwm_acc  = 0.0f;
        g_last_cnt = NEUQ_Encoder_Read(Encoder_TIM4);
        g_actual   = 0.0f;
        return;
    }

    int32_t cnt = NEUQ_Encoder_Read(Encoder_TIM4);
    g_actual = (float)(cnt - g_last_cnt) * FEEDBACK_K;
    g_last_cnt = cnt;

    g_pwm_acc += IncrementalPID_Update(&g_pid, g_target, g_actual);
    if (g_pwm_acc >  PWM_MAX) g_pwm_acc =  PWM_MAX;
    if (g_pwm_acc < -PWM_MAX) g_pwm_acc = -PWM_MAX;

    NEUQ_GPIO_write(DIR_PORT, g_pwm_acc >= 0.0f ? GPIO_High : GPIO_Low);
    NEUQ_PWM_set_duty(MACHINE_PORT,
                      g_pwm_acc >= 0.0f ? (uint32_t)g_pwm_acc
                                        : (uint32_t)(-g_pwm_acc));
}

//------电机动作
//关闭电机
static void Machine_off(void){
    g_locked = 1;
    g_target = 0.0f;
    g_pwm_acc = 0.0f;
    NEUQ_PWM_set_duty(MACHINE_PORT, 0);
}
//开启电机
static void Machine_start(void){
    g_locked = 0;
    g_run = 1;
    NEUQ_Encoder_Clear(Encoder_TIM4);
    g_last_cnt = 0;
    Ipid_parameters_setting(&g_pid, g_kp, g_ki, g_kd);
}
//更新电机PWM
static void Machine_update(void){
    if (g_locked) return;
    if (g_len < 4) return;              //负载不够时丢弃
    float v = GetFloat(g_data);
    if (v >  PWM_MAX || v < -PWM_MAX) return;
    g_target = v;
}

//设置PWM参数
static void Machine_pid(void){
    if (g_len < 12) return;             //kp ki kd各4字节
    float kp = GetFloat(g_data);
    float ki = GetFloat(g_data + 4);
    float kd = GetFloat(g_data + 8);
    if (kp < 0.0f || kp > 500.0f) return;
    if (ki < 0.0f || ki > 50.0f)  return;
    if (kd < 0.0f || kd > 50.0f)  return;
    g_kp = kp; g_ki = ki; g_kd = kd;
    Ipid_parameters_setting(&g_pid, g_kp, g_ki, g_kd);
}


//------------------帧解析
//协议帧状态机枚举
typedef enum {
    WAIT_HEAD,
    WAIT_LEN,
    WAIT_CMD,
    WAIT_DATA,
    WAIT_CHECK,
    WAIT_TAIL,
} ProtoState;



//协议初始化
void protocol_init(void)
{
    NEUQ_PWM_init(MACHINE_PORT, 1000, 0);
    NEUQ_GPIO_init(DIR_PORT, GPIO_Mode_Output_PushPull, GPIO_Low);
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);

    NEUQ_Uart_init(UART_0, UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 2);
    NEUQ_PIT_init_us(PIT_CH1, CTRL_PERIOD_US);

    Ipid_parameters_setting(&g_pid, g_kp, g_ki, g_kd);
    g_pid.max_delta_pwm = 6000;
}
//解包
void protocol_analysis(uint8_t dat){
    static ProtoState state = WAIT_HEAD;
    static uint8_t rx_len = 0, rx_cmd = 0, rx_check = 0, rx_idx = 0;
    static uint8_t rx_data[32]; //暂存负载
    switch (state){
        case WAIT_HEAD:{
            if (dat == PROTO_HEAD){
                state = WAIT_LEN;

            }
            break;
        }
        case WAIT_LEN:{
            rx_len = dat;
            rx_idx = 0;
            state = WAIT_CMD;
            break;
        }
        case WAIT_CMD:{
            rx_cmd = dat;
            if(rx_len > sizeof(rx_data)){ state = WAIT_HEAD; break; }  // 长度限制
            state = (rx_len > 0) ? WAIT_DATA : WAIT_CHECK;
            break;
        }
        case WAIT_DATA:{
            if (dat == PROTO_HEAD) { rx_idx = 0; state = WAIT_LEN; break; }
            rx_data[rx_idx++] = dat;
            if (rx_idx >= rx_len) state = WAIT_CHECK;
            break;
        }
        case WAIT_CHECK:{
            rx_check = dat;
            if (rx_check != (uint8_t)(rx_cmd + rx_len)) { state = WAIT_HEAD; break; }
            state = WAIT_TAIL;
            break;
        }
        case WAIT_TAIL:{
            if((dat == PROTO_TAIL) && ( rx_check == (rx_cmd + rx_len))){
                g_cmd = rx_cmd;
                g_len = rx_len;
                memcpy(g_data, rx_data, rx_len);
                g_frame_ready = 1;
            }
            state = WAIT_HEAD;
            break;
        }
        default:{
            state = WAIT_HEAD;
            break;
        }
    }
};

//---------------执行任务
void protocol_task(void){
    if (!g_frame_ready) return;
    g_frame_ready = 0;
    switch (g_cmd) {
    case CMD_OFF:{
        Machine_off();
        break;
    }       
    case CMD_START:{
        Machine_start();
        break;
    }
    case CMD_UPDATE:{
        if (g_run == 1) Machine_update();   // 关闭电机时不允许修改目标转速
        break;
    }       
    case CMD_PID:{
        Machine_pid();
        break;
    }
    case CMD_KP: {
        if (g_len < 4) break;
        float v = GetFloat(g_data);
        if (v >= 0.0f && v <= 500.0f) {
            g_kp = v;
            Ipid_parameters_setting(&g_pid, g_kp, g_ki, g_kd);
        }
        break;
    }
    case CMD_KI: {
        if (g_len < 4) break;
        float v = GetFloat(g_data);
        if (v >= 0.0f && v <= 50.0f) {
            g_ki = v;
            Ipid_parameters_setting(&g_pid, g_kp, g_ki, g_kd);
        }
        break;
    }
    case CMD_KD: {
        if (g_len < 4) break;
        float v = GetFloat(g_data);
        if (v >= 0.0f && v <= 50.0f) {
            g_kd = v;
            Ipid_parameters_setting(&g_pid, g_kp, g_ki, g_kd);
        }
        break;
    }
    default:
        break;
    }
}
//回传并在vofa上显示图像
void sendvofa(void){
    uint8_t f[12];
    float target = g_target;
    float actual = g_actual;
    memcpy(&f[0], &target, 4);  // 曲线1：目标
    memcpy(&f[4], &actual, 4);  // 曲线2：实际
    // JustFloat 帧尾 00 00 80 7F
    f[8]  = 0x00; f[9]  = 0x00; 
    f[10] = 0x80; f[11] = 0x7F;
    NEUQ_Uart_write_data(UART_0, f, 12);
}


