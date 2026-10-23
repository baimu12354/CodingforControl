#ifndef _CHANGER_IMU_H_
#define _CHANGER_IMU   _H_



#define Q2_AXIS_NUM     6       /* 六轴 */
#define Q2_AXIS_AX      0
#define Q2_AXIS_AY      1
#define Q2_AXIS_AZ      2
#define Q2_AXIS_GX      3
#define Q2_AXIS_GY      4
#define Q2_AXIS_GZ      5

#define Q2_AXIS_NUM     6       /* 六轴 */
#define Q2_AXIS_AX      0
#define Q2_AXIS_AY      1
#define Q2_AXIS_AZ      2
#define Q2_AXIS_GX      3
#define Q2_AXIS_GY      4
#define Q2_AXIS_GZ      5

#define Q2_R_MIN        0.05f   /* 卡尔曼观测噪声下限(滤波最弱) */
#define Q2_R_MAX        5.00f   /* 卡尔曼观测噪声上限(滤波最强) */
#define Q2_R_STEP       0.05f   /* 界面调节步进 */
#define Q2_R_DEF        0.05f   /* 默认滤波参数(与 Kanman_Init 一致) */

#define Q2_Q_MIN        0.001f  /* 卡尔曼过程噪声下限(滤波最强) */
#define Q2_Q_MAX        0.50f  /* 卡尔曼过程噪声上限(滤波最弱) */
#define Q2_Q_STEP       0.001f  /* 界面调节步进 */
#define Q2_Q_DEF        0.048f  /* 默认滤波参数(与 Kanman_Init 一致) */

/* 航向防漂移(见 GET_ANGLE.c 的 Madgwick_Update 航向解耦块):
 * 显示航向 = 去零偏+滤波后的 GZ 独立积分, 积分器前设两道闸门 --
 * 静止冻结(零速判定成立时不积分) + 死区(|GZ| 低于死区不积分),
 * 静置噪声与残余零偏都进不了积分器, 静态航向漂移被结构性消除 */
#define Q2_YAW_DEADBAND_DPS 1.0f    /* 航向积分死区(°/s), 手持转动几十°/s 远高于此值 */

extern volatile float Q2_raw[Q2_AXIS_NUM];    /* 六轴原始值(已去零偏), g / °/s */
extern volatile float Q2_raw_o[Q2_AXIS_NUM];
extern volatile float Q2_filt[Q2_AXIS_NUM];   /* 六轴滤波后值(后台持续更新, 与页面显示一致) */
extern float Q2_kalman_r;                      /* 当前生效滤波系数 */
extern float Q2_kalman_q;                      /* 当前生效过程噪声 */
extern uint8_t Q2_axis;                        /* 当前选中的发送轴 0..5 */
extern uint8_t Q2_send_on;                     /* 波形发送开关 0/1 */
extern uint8_t Q2_cal_ok;                      /* 上电零偏校准完成标志 */
extern const char *Q2_AXIS_NAMES[Q2_AXIS_NUM];
extern uint8_t Q2_imu_still; 

void Q2_Config_Load(void);                     /* 从 Flash 恢复参数, 无有效记录用默认值 */
uint8_t Q2_Config_Save(void);                  /* 参数写入 Flash, 返回 1 成功 */
void Q2_IMU_Tick(void);   
    
#endif 
