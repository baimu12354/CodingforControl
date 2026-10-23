#ifndef _NEUQ_SPEED_H_
#define _NEUQ_SPEED_H_

#define SPEED_ENCODER_DEV (Encoder_TIM4)
#define SPEED_ENCODER_A (Encoder_TIM4_PhaseA_PB6)
#define SPEED_ENCODER_B (Encoder_TIM4_PhaseB_PB7)
#define ENCODER_PPR (11)  //每转脉冲数
#define ENCODER_CNT_PER_REV (ENCODER_PPR * 4)  //电机轴计数值，四倍频
#define REDUCTION_RATIO (10)  //减速比
#define ENCODER_CNT_PER_OUT_REV (ENCODER_CNT_PER_REV * REDUCTION_RATIO)  //输出轴计数值
#define SPEED_SAMPLE_PERIOD_MS (10)  //采样周期

void NEUQ_Speed_Init(void);
void NEUQ_Speed_Update(void);
float NEUQ_Speed_GetRPM(void);
void NEUQ_Speed_Reset(void);

#endif 