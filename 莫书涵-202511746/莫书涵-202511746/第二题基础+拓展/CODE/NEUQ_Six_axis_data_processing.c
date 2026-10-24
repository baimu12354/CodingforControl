#include "NEUQ_lib_include.h"
#include <math.h>

NEUQ_SIX_AXIS_DATA Six_axis_data;
NEUQ_QUATERNION Quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
NEUQ_ATTITUDE Attitude = {0.0f, 0.0f, 0.0f};

void NEUQ_SIX_AXIS_DATA_Init(void){
    NEUQ_MPU6050_init();
}

void NEUQ_SIX_AXIS_DATA_Get(void){
    NEUQ_MPU6050_read_data();
    Six_axis_data.Axis_data[0] = MPU6050_acc_transition(mpu6050_acc_x);
    Six_axis_data.Axis_data[1] = MPU6050_acc_transition(mpu6050_acc_y);
    Six_axis_data.Axis_data[2] = MPU6050_acc_transition(mpu6050_acc_z);
    Six_axis_data.Axis_data[3] = MPU6050_gyro_transition(mpu6050_gyro_x);
    Six_axis_data.Axis_data[4] = MPU6050_gyro_transition(mpu6050_gyro_y);
    Six_axis_data.Axis_data[5] = MPU6050_gyro_transition(mpu6050_gyro_z);

    for(uint8_t i = 0; i < 6; i++){
        float alpha = Pragma.alpha_1000[i] / 1000.0f;
        Six_axis_data.Axis_lvbodata[i] += alpha * (Six_axis_data.Axis_data[i] - Six_axis_data.Axis_lvbodata[i]);
    }
}

void NEUQ_SIX_AXIS_DATA_NameDisplay(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);

    NEUQ_LCD200_DisplayString(8,   8, "6-Axis  raw    flt");
    NEUQ_LCD200_DisplayString(8,  48, "Xa/g  :");
    NEUQ_LCD200_DisplayString(8,  80, "Ya/g  :");
    NEUQ_LCD200_DisplayString(8, 112, "Za/g  :");
    NEUQ_LCD200_DisplayString(8, 144, "Xg/dps:");
    NEUQ_LCD200_DisplayString(8, 176, "Yg/dps:");
    NEUQ_LCD200_DisplayString(8, 208, "Zg/dps:");
}

void NEUQ_SIX_AXIS_DATA_DataDisplay(void){
    // for(uint8_t i = 0; i < 6; i++){
    //     NEUQ_LCD200_ClearRect(80,48 + i*32,120,24);
    //     NEUQ_LCD200_DisplayFloat(80,48 + i*32,Six_axis_data.Axis_data[i],3,2);
    // }
    char line[24];

    for(uint8_t i = 0; i < 6; i++){
        uint16_t y = 48 + i * 32;

        /* 一行画两个值：左=原始  右=滤波 */
        NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
        NEUQ_LCD200_SetPenColor(LCD_BLACK);
        NEUQ_LCD200_SetBackColor(LCD_WHITE);

        NEUQ_LCD200_ClearRect(62, y, 192, 16);

        snprintf(line, sizeof(line), "0%-6.2f %-6.2f",Six_axis_data.Axis_data[i],Six_axis_data.Axis_lvbodata[i]);
        NEUQ_LCD200_DisplayString(62, y, line);
    } 
}

//向VOFA+发送数据
void NEUQ_SIX_AXIS_DATA_SendtoVOFA(AXIS_NAME AXIS){
    //printf("x_a:%.2f,%.2f,\n",Six_axis_data.Axis_data[AXIS],Six_axis_data.Axis_lvbodata[AXIS]);
    float raw,flt;
    __disable_irq();
    raw = Six_axis_data.Axis_data[AXIS];
    flt = Six_axis_data.Axis_lvbodata[AXIS];
    __enable_irq();

    printf("%.3f,%.3f\n",raw,flt);
}

//四元数归一化
static void Quat_Normalize(NEUQ_QUATERNION *q){
    float n = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);

    if(n < 1e-9f){     //退化保护，避免除零
        q->q0 = 1.0f; q->q1 = 0.0f; q->q2 = 0.0f; q->q3 = 0.0f;
        return;
    }
    q->q0 /= n;  q->q1 /= n;  q->q2 /= n;  q->q3 /= n;
}

//陀螺仪积分 q_dot = 0.5 * q ⊗ (0, gx, gy, gz)
static void Quat_IntegrateGyro(NEUQ_QUATERNION *q, float gx, float gy, float gz, float dt){
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;

    q->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    q->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    q->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    q->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
}

//四元数 转 欧拉角
static void Quat_ToEuler(const NEUQ_QUATERNION *q, float *roll, float *pitch, float *yaw){
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    float sinp;

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    sinp = 2.0f * (q0 * q2 - q3 * q1);

    if(sinp >  1.0f) sinp =  1.0f;
    if(sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    *roll  *= RAD_TO_DEG;
    *pitch *= RAD_TO_DEG;
    *yaw   *= RAD_TO_DEG;
}

//
void NEUQ_QUATERNION_Get(void){
    float ax, ay, az;     //单位为 g
    float gx, gy, gz;     //单位为 rad/s
    float q0, q1, q2, q3;
    float vx, vy, vz;     //估算重力方向

    ax = Six_axis_data.Axis_lvbodata[0];
    ay = Six_axis_data.Axis_lvbodata[1];
    az = Six_axis_data.Axis_lvbodata[2];
    gx = Six_axis_data.Axis_lvbodata[3] * DEG_TO_RAD;   /* °/s → rad/s */
    gy = Six_axis_data.Axis_lvbodata[4] * DEG_TO_RAD;
    gz = Six_axis_data.Axis_lvbodata[5] * DEG_TO_RAD;

    // ax = -ax;   ay = -ay;   az = -az;

    float anorm = sqrtf(ax * ax + ay * ay + az * az);//计算模长

    if(fabsf(anorm - 1.0f) > wuca){//误差过大,直接积分
        Quat_IntegrateGyro(&Quaternion, gx, gy, gz, ATTITUDE_DT);
        Quat_Normalize(&Quaternion);
        Quat_ToEuler(&Quaternion, &Attitude.Roll, &Attitude.Pitch, &Attitude.Yaw);

        return;
    }

    ax /= anorm;   ay /= anorm;   az /= anorm;

    q0 = Quaternion.q0;
    q1 = Quaternion.q1;
    q2 = Quaternion.q2;
    q3 = Quaternion.q3;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex, ey, ez;         //实测重力与预估重力之间的误差
    //误差 = 实测 × 预估
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    //用误差修正角速度，再积分
    gx += ATTITUDE_BETA * ex;
    gy += ATTITUDE_BETA * ey;
    gz += ATTITUDE_BETA * ez;

    Quat_IntegrateGyro(&Quaternion, gx, gy, gz, ATTITUDE_DT);
    Quat_Normalize(&Quaternion);

    Quat_ToEuler(&Quaternion, &Attitude.Roll, &Attitude.Pitch, &Attitude.Yaw);
}

void NEUQ_ATTITUDE_NameDisplay(void){
    NEUQ_LCD200_Clear();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);

    NEUQ_LCD200_DisplayString(8,   8, "Attitude (deg)");
    NEUQ_LCD200_DisplayString(8,  48, "Roll :");
    NEUQ_LCD200_DisplayString(8,  80, "Pitch:");
    NEUQ_LCD200_DisplayString(8, 112, "Yaw  :");

    NEUQ_LCD200_DisplayString(8, 138, "System: X-Y-Z");
    NEUQ_LCD200_DisplayString(8, 160, "Update:10ms");
}

void NEUQ_ATTITUDE_DataDisplay(void){
    char line[16];
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetPenColor(LCD_BLACK);
    NEUQ_LCD200_SetBackColor(LCD_WHITE);

    snprintf(line, sizeof(line), "%.2f", Attitude.Pitch);
    NEUQ_LCD200_ClearRect(100, 48, 130, 16);
    NEUQ_LCD200_DisplayString(100,  48, line);

    snprintf(line, sizeof(line), "%.2f", Attitude.Roll);
    NEUQ_LCD200_ClearRect(100, 80, 130, 16);
    NEUQ_LCD200_DisplayString(100, 80, line);

    snprintf(line, sizeof(line), "%.2f", Attitude.Yaw);
    NEUQ_LCD200_ClearRect(100, 112, 130, 16);
    NEUQ_LCD200_DisplayString(100, 112, line);
}
