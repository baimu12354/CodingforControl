#include "NEUQ_lib_include.h"
#include <stdlib.h>

//命令表
static const CmdItem cmd_table[] = {
    {"speed_aim",   &speed_aim,   TYPE_FLOAT},
    {"kp",          &g_kp,        TYPE_FLOAT},
    {"ki",          &g_ki,        TYPE_FLOAT},
    {"kd",          &g_kd,        TYPE_FLOAT},
    {"run",         &g_run,         TYPE_INT},
};

#define CMD_COUNT   (sizeof(cmd_table) / sizeof(cmd_table[0]))

void NEUQ_VOFA_Init(void){
    // NEUQ_Uart_init(UART_3,UART_BAUD_115200);
    // NEUQ_Uart_Enable_RX_Interrupt(UART_3, 5);
    NEUQ_Uart_init(UART_0,UART_BAUD_115200);
    NEUQ_Uart_Enable_RX_Interrupt(UART_0, 5);
}

//把传入参数的':'换成'\0',并给对应参数赋值
// 1 解析成功；0 无法识别
uint8_t NEUQ_VOFA_Parse_Command(char *str){
    char *eq_pos;
    char *cmd_str;
    char *val_str;

    if(str == NULL){return 0;}

    eq_pos = strchr(str,':');
    if(eq_pos == NULL){return 0;}

    *eq_pos = '\0';
    cmd_str = str;
    val_str = eq_pos + 1;

    for(uint32_t i = 0; i < CMD_COUNT; i++){
        if(strcmp(cmd_str,cmd_table[i].name) == 0){
            if(cmd_table[i].type == TYPE_FLOAT){
                *(float *)cmd_table[i].ptr = atof(val_str);
            } else{
                *(int *)cmd_table[i].ptr = atoi(val_str);
            }

            return 1;
        }
    }

    return 0;
}

static char cmd_line[VOFA_LINE_MAX];
static uint16_t s_len = 0;
static volatile uint8_t s_ready = 0;

//volatile uint32_t vofa_cnt_ok   = 0;
//volatile uint32_t vofa_cnt_bad  = 0;
//volatile uint32_t vofa_cnt_drop = 0;


//接收字节函数，只做更新标志位处理，放入中断
void NEUQ_VOFA_rx_byte(uint8_t data){
    if(data == (uint8_t)'\n'){
        cmd_line[s_len] = '\0';
        s_ready = 1;
        return;
    }

    if(data == (uint8_t)'\r')return;
    if(s_ready == 1)return;

    if(s_len < (VOFA_LINE_MAX - 1)){
        cmd_line[s_len++] = (char)data;
    } else{
        s_len = 0;
        //vofa_cnt_drop++;
    }
}

void NEUQ_VOFA_task(void){
    if(s_ready == 1){
        // if (NEUQ_VOFA_Parse_Command(cmd_line)) vofa_cnt_ok++;
        // else                                  vofa_cnt_bad++;
        NEUQ_VOFA_Parse_Command(cmd_line);

        s_len = 0;
        s_ready = 0;
    }

    static uint32_t next_ms = 0;
    if(HAL_GetTick() >= next_ms){
        next_ms = HAL_GetTick() + 20;
        // printf("speed_aim:%.2f\n",speed_aim);
        // printf("speed_now:%.2f\n",speed_now);
        // printf("kp:%.2f\n",g_kp);
        // printf("ki:%.2f\n",g_ki);
        // printf("kd:%.2f\n",g_kd);
        printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",speed_aim, speed_now, g_kp, g_ki, g_kd);
    }
}
