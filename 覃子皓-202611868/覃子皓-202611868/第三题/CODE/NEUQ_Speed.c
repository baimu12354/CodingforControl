#include "NEUQ_lib_include.h"

static int32_t speed_last_cnt = 0;
static float speed_rpm = 0.0f;

void NEUQ_Speed_Init(void){  //初始化
    NEUQ_Encoder_init(SPEED_ENCODER_DEV,SPEED_ENCODER_A,SPEED_ENCODER_B);
    NEUQ_Encoder_Clear(SPEED_ENCODER_DEV);
    speed_last_cnt = 0;
    speed_rpm = 0.0f;
}
void NEUQ_Speed_Update(void){  //速度更新
    int32_t now_cnt = NEUQ_Encoder_Read(SPEED_ENCODER_DEV);
    int32_t diff = now_cnt - speed_last_cnt;  //增量
    if (diff > 32767)  diff -= 65536;  //溢出重置
    if (diff < -32768) diff += 65536;
    speed_last_cnt = now_cnt;
    float dt = SPEED_SAMPLE_PERIOD_MS / 1000.0f;  //ms转s
    float rpm_raw = (diff / (float)ENCODER_CNT_PER_OUT_REV) / dt * 60.0f;  //转速原始值
    speed_rpm = speed_rpm * 0.7f + rpm_raw * 0.3f;  //低通滤波
}
float NEUQ_Speed_GetRPM(void){  //获取转速
    return speed_rpm;
}
void NEUQ_Speed_Reset(void){  //速度测量复位
    NEUQ_Encoder_Clear(SPEED_ENCODER_DEV);
    speed_last_cnt = 0;
    speed_rpm = 0.0f;
}