#include "NEUQ_lib_include.h"

volatile uint32_t NEUQ_sys_tick = 0;

float acc_filt[3]  = {0.0f, 0.0f, 0.0f};
float gyro_filt[3] = {0.0f, 0.0f, 0.0f};

static float alpha_f = 0.50f;        /* 滤波系数，默认 0.50 与菜单默认 alpha=50 一致 */
static float f_acc[3]  = {0.0f, 0.0f, 0.0f};
static float f_gyro[3] = {0.0f, 0.0f, 0.0f};
static uint8_t f_init = 0;           /* 首次采样直接赋初值，避免从 0 爬升 */

void NEUQ_Sensor_SetAlpha(int32_t alpha_pct){
    alpha_f = (float)alpha_pct / 100.0f;
}

void NEUQ_Sensor_Init(void){
    if (NEUQ_MPU6050_init() == 0){
        printf("MPU6050 init fail\r\n");     /* 传感器异常时便于排查，可删 */
    }
    NEUQ_Uart_init(VOFA_UART, UART_BAUD_115200);
    NEUQ_Sensor_SetAlpha(Params.alpha);      /* 上电后与 Flash 恢复的参数保持一致 */
}

/* 一阶低通：y = y + a*(x-y)，α 由菜单控制，能明显压波动且保留趋势 */
static float lowpass(float x, float y, float a){
    return y + a * (x - y);
}

/* JustFloat 协议：2 个 float（小端）+ 尾帧 00 00 80 7F */
static void Vofa_Send2(float v1, float v2){
    uint8_t  buf[12];
    uint32_t u1, u2;
    memcpy(&u1, &v1, 4);
    memcpy(&u2, &v2, 4);
    buf[0] = (uint8_t)(u1      ); buf[1] = (uint8_t)(u1 >> 8);
    buf[2] = (uint8_t)(u1 >> 16); buf[3] = (uint8_t)(u1 >> 24);
    buf[4] = (uint8_t)(u2      ); buf[5] = (uint8_t)(u2 >> 8);
    buf[6] = (uint8_t)(u2 >> 16); buf[7] = (uint8_t)(u2 >> 24);
    buf[8] = 0x00; buf[9] = 0x00; buf[10] = 0x80; buf[11] = 0x7F;
    NEUQ_Uart_write_data(VOFA_UART, buf, 12);
}

void NEUQ_Sensor_Task(void){
    int16_t ra[3], rg[3];
    float   xa[3], xg[3];       /* 本次采样的物理值（g、°/s） */

    /* 1) 同一拍只读一次 MPU6050：6 轴原始值来自同一次采样 */
    NEUQ_MPU6050_read_data();
    ra[0] = mpu6050_acc_x;  ra[1] = mpu6050_acc_y;  ra[2] = mpu6050_acc_z;
    rg[0] = mpu6050_gyro_x; rg[1] = mpu6050_gyro_y; rg[2] = mpu6050_gyro_z;

    /* 2) 换算物理单位 */
    for (int i = 0; i < 3; i++){
        xa[i] = MPU6050_acc_transition(ra[i]);
        xg[i] = MPU6050_gyro_transition(rg[i]);
    }

    /* 3) 六轴分别滤波——此任务与页面无关，切页不停止采集与滤波 */
    if (!f_init){
        for (int i = 0; i < 3; i++){ f_acc[i] = xa[i]; f_gyro[i] = xg[i]; }
        f_init = 1;
    } else {
        for (int i = 0; i < 3; i++){
            f_acc[i]  = lowpass(xa[i], f_acc[i],  alpha_f);
            f_gyro[i] = lowpass(xg[i], f_gyro[i], alpha_f);
        }
    }
    for (int i = 0; i < 3; i++){ acc_filt[i] = f_acc[i]; gyro_filt[i] = f_gyro[i]; }

    Attitude_Update(gyro_filt[0] * DEG2RAD,
                    gyro_filt[1] * DEG2RAD,
                    gyro_filt[2] * DEG2RAD,
                    acc_filt[0], acc_filt[1], acc_filt[2],
                    0.01f);

    /* 4) VOFA+ 发送：只发选中轴的“原始+滤波”两个值，同一拍采样，不夹带其他轴；
          关闭上传（send_enable=0）时仅停发送，采集滤波照常进行 */
    if (Params.send_enable){
        float v1, v2;
        int axis = Params.send_axis;
        if (axis < 3){ v1 = xa[axis];    v2 = f_acc[axis]; }
        else         { v1 = xg[axis-3];  v2 = f_gyro[axis-3]; }
        Vofa_Send2(v1, v2);
    }
}
