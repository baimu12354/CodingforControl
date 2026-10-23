#include "NEUQ_lib_include.h"

#define ENCODER_DEV              Encoder_TIM4   // 必须和 User_Main.c 里初始化的定时器一致
#define ENCODER_PULSE_PER_REV    440.0f         // 输出轴转一圈的计数值（11 线 × 4 倍频 × 减速比 10）
#define ENCODER_SAMPLE_PERIOD_S  0.01f          // 采样周期 10ms（必须和 PIT 周期一致）

float NEUQ_ENCODER_GETSPEED(void){
    static uint8_t  read_first = 1;     // 是否第一次读取
    static int32_t  last_read  = 0;     // 上一次的编码器计数值
    static float  last_speed = 0.0f;    // 上一次滤波后的速度
    int32_t read_value = 0;             // 本次编码器计数值
    int32_t delta = 0;                  // 脉冲差
    float raw_speed = 0.0f;             // 原始转速
    float result = 0.0f;                // 滤波后的转速

    read_value = NEUQ_Encoder_Read(ENCODER_DEV);
    if(read_first == 1){
        read_first = 0;
        last_read = read_value;
        return 0.0f;
    }
    delta = read_value - last_read;
    last_read = read_value;

    // 计数器是 16 位、在 0x7FFF 附近回绕：跨过回绕点时补一个周期，
    // 否则会算出 ±65535 的假脉冲，PID 会突然抽一下（连续跑 30s 时一定会碰上）。
    if(delta >  32767){
         delta -= 65536; 
        }
    if(delta < -32768){
         delta += 65536; 
        }

    // 转速 = 脉冲差 / 每转脉冲数 / 采样周期
    raw_speed = (float)delta / ENCODER_PULSE_PER_REV / ENCODER_SAMPLE_PERIOD_S;
    result = 0.7f * raw_speed + 0.3f * last_speed;    //一阶低通滤波
    last_speed = result;

    return result;
}
