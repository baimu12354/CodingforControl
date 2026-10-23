/* 编码器测速模块实现*/
#include "NEUQ_lib_include.h"  

#define ENCODER_DEV              Encoder_TIM4   // 定时器做编码器，初始化用
#define ENCODER_PULSE_PER_REV    440.0f         // 轴转一圈的计数值11 线 × 4 倍频 × 减速比 10
#define ENCODER_SAMPLE_PERIOD_S  0.01f          // 采样周期

float NEUQ_ENCODER_GETSPEED(void){
    static uint8_t  read_first = 1;     // 是否第一次读取，做基准
    static int32_t  last_read  = 0;     // 上一次的编码器计数值
    static float  last_speed = 0.0f;    // 上一次滤波后的速度

    int32_t read_value = 0;             // 本次从编码器读到的计数值
    int32_t delta = 0;                  // 两次采样的脉冲差
    float raw_speed = 0.0f;             // 原始转速
    float result = 0.0f;                // 滤波后的转速

    //读编码器
    read_value = NEUQ_Encoder_Read(ENCODER_DEV);

    //第一次调用
    if(read_first == 1){
        read_first = 0;                 // 清除首次标志，之后正常测速
        last_read = read_value;         // 记录基准值
        return 0.0f;                    // 首次无速度可算
    }

    //差分
    delta = read_value - last_read;
    last_read = read_value;             // 更新供下次使用

    //回绕补偿：跨过回绕点时补一个周期
    if(delta >  32767){
         delta -= 65536;                // 正向回绕
        }
    if(delta < -32768){
         delta += 65536;                // 负向回绕
        }

    //转速换算
    raw_speed = (float)delta / ENCODER_PULSE_PER_REV / ENCODER_SAMPLE_PERIOD_S;

    result = 0.7f * raw_speed + 0.3f * last_speed;    //一阶低通滤波
    last_speed = result;                // 更新滤波历史值

    return result;                      // 返回滤波后的转速
}
