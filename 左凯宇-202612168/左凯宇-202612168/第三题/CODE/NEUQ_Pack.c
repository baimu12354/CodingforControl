#include "NEUQ_lib_include.h"

NEUQ_FIFO_t rx_fifo;
volatile uint8_t upload_flag = 0;
static uint8_t rx_buf[64];//串口中断接收到的写入fifo

/*定义帧
AA 55 + CMD + LEN + DATA + CHK
  头    命令 数据长度 数据   校验
*/
//状态机FSM存数据用
static RX_NUM curnt = RX_H1;
static uint8_t rbuf[32];//存DATA
static uint8_t read_bits;//读到的字节数
static uint8_t len;
static uint8_t chk;
static char   txt_buf[32];    // 文本行缓冲
static uint8_t txt_len = 0;   // 已收到字符数

void NEUQ_Pack_init(void){
    //初始化fifo
    NEUQ_FIFO_init(&rx_fifo, rx_buf, sizeof(rx_buf));
    curnt = RX_H1;
}

//命令 数据 数据长度
static void cmd_switch(uint8_t cmd,uint8_t *d,uint8_t n){
    float kp, ki, kd; int16_t spd;
    switch(cmd){
        case CMD_START:
            if (n != 0) break;
            params.les_spd_total = 0; 
            params.les_spd_last = 0;
            params.motor_run = 1;
            break;
        case CMD_STOP:
            if (n != 0) break;
            params.motor_run = 0;
            params.motor_duty = 0;
            params.les_spd_total = 0; 
            params.les_spd_last = 0;
            break;
        case CMD_SPEED:
            if (n != 2) break;
            memcpy(&spd, d, 2);
            params.set_spd = RPM_TO_CNT10MS(spd);   // 150 → 11.0 计数/10ms
            break;

        case CMD_PID:
            if (n != 12 || params.motor_run != 0) break;
                memcpy(&kp, d, 4);
                memcpy(&ki, d+4, 4);
                memcpy(&kd, d+8, 4);
                params.kp = kp;
                params.ki = ki;
                params.kd = kd;
                params.les_spd_total = 0;
                params.les_spd_last = 0;
            break;
        default:
            break;
    }
}

static uint8_t command;

//状态机
static void NEUQ_FSM(uint8_t b){
    switch(curnt){
        case RX_H1:
            if (b == 0xAA) curnt = RX_H2;
            break;
        case RX_H2:
            if (b == 0x55){ 
                curnt = RX_CMD;
                chk = 0xAA + 0x55;
            }else{
                curnt = RX_H1;
            }
            break;
        case RX_CMD:
            command = b;
            chk += b;
            curnt = RX_LEN;
            break;
        case RX_LEN:
            if (b > sizeof(rbuf)){
                curnt = RX_H1;
                break;
            }
            len = b;
            chk += b;
            read_bits = 0;
            if(len == 0){
                curnt = RX_CHK;
            }else{
                curnt = RX_DATA;
            }
            break;
        case RX_DATA:
            rbuf[read_bits] = b;
            read_bits++;
            chk += b;
            if(read_bits >= len) curnt = RX_CHK;
            break;
        case RX_CHK:
            if (b == chk) cmd_switch(command,rbuf,len);
            curnt = RX_H1;
            break;
    }
}

//vofa+ 指令转为需要的数
static int32_t txt_atoi(const char *p){
    int32_t v = 0;
    int32_t sign = 1;

    if (*p == '-'){ sign = -1; p++; }
    else if (*p == '+'){ p++; }

    while (*p >= '0' && *p <= '9'){
        v = v * 10 + (*p - '0');
        p++;
    }
    return sign * v;
}

// 指令分发：以后加新指令，在这里加一个 else if 即可
static void text_cmd_dispatch(const char *key, const char *val){
    if (strcmp(key, "set_spd") == 0){ params.set_spd = RPM_TO_CNT10MS(txt_atoi(val));}
    else if (strcmp(key, "pwr")  == 0){ params.motor_run = txt_atoi(val); } 
    else if (params.motor_run == 0){
        if      (strcmp(key, "kp")  == 0){ params.kp = txt_atoi(val); }
        else if (strcmp(key, "ki")  == 0){ params.ki = txt_atoi(val); }
        else if (strcmp(key, "kd")  == 0){ params.kd = txt_atoi(val); }
    }
    // 示例，以后按需加
    // else if (strcmp(key, "xxxxx") == 0){ ... }
}

static void text_cmd_parse(uint8_t b){
    if (b == '\r') return;
    if (b == '\n'){
        txt_buf[txt_len] = '\0';
        txt_len = 0;
        char *colon = strchr(txt_buf, ':');   // 找冒号
        if (colon != NULL){
            *colon = '\0';                    // 切成 key / val 两段
            text_cmd_dispatch(txt_buf, colon + 1);
        }
        return;
    }
    if (txt_len < sizeof(txt_buf) - 1)
        txt_buf[txt_len++] = (char)b;
}

void recv_update(void){
    uint8_t b;
    while(NEUQ_FIFO_read_byte(&rx_fifo, &b) == NEUQ_FIFO_OK){
        RX_NUM before = curnt;              // 处理前 FSM 状态
        NEUQ_FSM(b);
        if (before == RX_H1 && curnt == RX_H1){   // 处理前后都在空闲状态，表明是纯文本字节
            text_cmd_parse(b);
        }
    }
}


typedef union{
    float f;
    uint8_t sending[4];
}send;

void vofa_send(void){
    uint8_t buf[24];
    send sed;
    int n = 0;
    int i = 0;
    float dt[5] = {
        (float)params.set_spd,
        (float)params.speed,
        params.kp,
        params.ki,
        params.kd
/*      测试用：
            (float)params.motor_duty,  // 通道3 输出占空比
            (float)params.motor_run,   // 通道4 启动标志
            (float)params.les_spd      // 通道5 偏差 */
    };
    for(i=0;i<5;i++){
        sed.f = dt[i];
        buf[n++] = sed.sending[0];
        buf[n++] = sed.sending[1];
        buf[n++] = sed.sending[2];
        buf[n++] = sed.sending[3];
    }
    buf[20]=0x00;
    buf[21]=0x00;
    buf[22]=0x80;
    buf[23]=0x7F;
    NEUQ_Uart_write_data(UART_0, buf, 24);
}