#include "NEUQ_lib_include.h"
#include <math.h>

#define PI         3.14159265f
#define DEG2RAD    (PI/180.0f)
#define RAD2DEG    (180.0f/PI)
#define MAHONY_KP  1.0f

// 零偏掉电保存：独立扇区(509)，与菜单配置扇区(510)分开，避免整扇区擦除互相清空
#define ATTITUDE_BIAS_MAGIC   0x42494153UL   // "BIAS"，用于校验保存数据是否有效
#define ATTITUDE_BIAS_SECTOR  509

// 零偏保存结构：magic 校验有效性，gx/gy/gz 为三轴零偏(°/s)
typedef struct{
    uint32_t magic;
    float gx;
    float gy;
    float gz;
} AttitudeBiasBlock;

float attitude_roll=0.0f;
float attitude_pitch=0.0f;
float attitude_yaw=0.0f;

static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float gyro_bias[3]={0.0f,0.0f,0.0f};   // 三轴陀螺零偏(°/s)，供姿态解算与六轴数据共同使用
static uint32_t last_tick=0;

// 上电静止采样标定三轴陀螺零偏并写入 Flash 保存（标定期间必须保持板子静止）
static void Attitude_Bias_Calibrate(void){
    const int N=3000;
    int32_t sum[3]={0,0,0};
    for(int i=0;i<N;i++){
        NEUQ_MPU6050_read_data();
        sum[0]+=mpu6050_gyro_x;
        sum[1]+=mpu6050_gyro_y;
        sum[2]+=mpu6050_gyro_z;
        NEUQ_Delay_ms(1);
    }
    gyro_bias[0]=MPU6050_gyro_transition((int16_t)(sum[0]/N));
    gyro_bias[1]=MPU6050_gyro_transition((int16_t)(sum[1]/N));
    gyro_bias[2]=MPU6050_gyro_transition((int16_t)(sum[2]/N));

    AttitudeBiasBlock bias;
    bias.magic=ATTITUDE_BIAS_MAGIC;
    bias.gx=gyro_bias[0];
    bias.gy=gyro_bias[1];
    bias.gz=gyro_bias[2];
    NEUQ_Flash_write_sector(ATTITUDE_BIAS_SECTOR,(uint8_t*)&bias,sizeof(bias));
}

// 手动重标定零偏并保存（菜单 GYRO CAL 触发），标定后重新对齐姿态
void NEUQ_Attitude_Calibrate(void){
    Attitude_Bias_Calibrate();
    q0=1.0f; q1=0.0f; q2=0.0f; q3=0.0f;
    last_tick=NEUQ_Get_Timer_Value_ms();
}

void NEUQ_Attitude_Init(void){
    AttitudeBiasBlock bias;

    // 优先加载上次断电保存的零偏，避免每次上电都要保持静止重新标定
    if(NEUQ_Flash_read_sector(ATTITUDE_BIAS_SECTOR,(uint8_t*)&bias,sizeof(bias))==NEUQ_FLASH_OK
       && bias.magic==ATTITUDE_BIAS_MAGIC){
        gyro_bias[0]=bias.gx;
        gyro_bias[1]=bias.gy;
        gyro_bias[2]=bias.gz;
    }else{
        Attitude_Bias_Calibrate();   // 无有效保存数据：现场标定并保存
    }

    q0=1.0f; q1=0.0f; q2=0.0f; q3=0.0f;
    last_tick=NEUQ_Get_Timer_Value_ms();
}

void NEUQ_Attitude_Task(void){
    uint32_t now=NEUQ_Get_Timer_Value_ms();
    float dt=(now-last_tick)/1000.0f;
    last_tick=now;
    if(dt<=0.0f || dt>0.1f) dt=0.01f;
    // sensor_raw[3..5] 已在 NEUQ_Sensor_Task 中减过零偏，这里不要再减一次
    float gx=sensor_raw[3]*DEG2RAD;
    float gy=sensor_raw[4]*DEG2RAD;
    float gz=sensor_raw[5]*DEG2RAD;
    float ax=sensor_raw[0], ay=sensor_raw[1], az=sensor_raw[2];
    float n=sqrtf(ax*ax+ay*ay+az*az);
    if(n>1e-6f){ ax/=n; ay/=n; az/=n; }
    float vx=2.0f*(q1*q3 - q0*q2);
    float vy=2.0f*(q0*q1 + q2*q3);
    float vz=q0*q0 - q1*q1 - q2*q2 + q3*q3;
    float ex=ay*vz - az*vy;
    float ey=az*vx - ax*vz;
    float ez=ax*vy - ay*vx;
    gx += MAHONY_KP*ex;
    gy += MAHONY_KP*ey;
    gz += MAHONY_KP*ez;
    float hdt=0.5f*dt;
    q0 += (-q1*gx - q2*gy - q3*gz)*hdt;
    q1 += ( q0*gx + q2*gz - q3*gy)*hdt;
    q2 += ( q0*gy - q1*gz + q3*gx)*hdt;
    q3 += ( q0*gz + q1*gy - q2*gx)*hdt;
    n=sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    if(n>1e-6f){ q0/=n; q1/=n; q2/=n; q3/=n; }
    attitude_roll = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))*RAD2DEG;
    float sp=2.0f*(q0*q2-q3*q1);
    if(sp>1.0f) sp=1.0f; else if(sp<-1.0f) sp=-1.0f;
    attitude_pitch = asinf(sp)*RAD2DEG;
    attitude_yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))*RAD2DEG;
}
