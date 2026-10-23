#include "NEUQ_lib_include.h"

float NEUQ_ENCODER_GETSPEED(void){//计算实际速度
    static uint8_t  read_first = 1;     // 是否第一次读取
    static int32_t  last_read  = 0;     // 上一次的编码器计数值
    static float  last_speed = 0.0f;    // 上一次滤波后的速度
    int32_t read_value = 0;             // 本次编码器计数值
    int32_t delta = 0;                  // 脉冲差
    float raw_speed = 0.0f;             // 原始转速
    float result = 0.0f;                // 滤波后的转速
    read_value = NEUQ_Encoder_Read(Encoder_TIM4);
    if(read_first == 1){
        read_first = 0;
        last_read = read_value;
        return 0.0f;
    }
    delta = read_value - last_read;
    last_read = read_value;
    if(delta >  32767){
         delta -= 65536; 
        }
    if(delta < -32768){
         delta += 65536; 
        }
    // 转速 = 脉冲差 / 每转脉冲数 / 采样周期
    raw_speed = (float)delta / 440.0f/0.01f;
    result = 0.7f * raw_speed + 0.3f * last_speed;    //一阶低通滤波
    last_speed = result;
    return result;
}
