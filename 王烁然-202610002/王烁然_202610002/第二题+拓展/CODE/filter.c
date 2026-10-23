
#include"NEUQ_lib_include.h"

volatile FLITER_STR filters[FILTER_NUM];

 Calcu_Str Calcu_Value={0};
// NEUQ_Flash_read(uint32_t address,uint8_t *buffer,uint32_t length);
// NEUQ_Flash_page_program(uint32_t address,const uint8_t *buffer,uint16_t length);


volatile uint8_t dataWrite[6] = {0};
volatile uint8_t dataRead[6] = {0};

// // // 上电后先初始化（只做一次），失败则不能继续
// // if (NEUQ_Flash_init() != NEUQ_FLASH_OK) {
// //     // 处理初始化失败
// // }

// // 写入：擦除整个扇区后，从该扇区第 0 字节写入
// NEUQ_Flash_write_sector(FLASH_DATA_STORE, data, 6);   // 用扇区 511

// // 断电重启后，重新 init，再读回
// NEUQ_Flash_read_sector(FLASH_DATA_STORE, rd, 6);

// NEUQ_Flash_get_sector_address(FLASH_DATA_STORE);






void Filter_Init(void)//需要判断flash里有没有存货
{
    uint8_t read=1;
    NEUQ_Flash_read(NEUQ_Flash_get_sector_address(FLASH_DATA_STORE)
        ,(uint8_t*)dataRead,sizeof(dataRead));
    for(uint8_t i=0;i<FILTER_NUM;i++)
    {
        if(dataRead[i]>10)read=0;
    }
    if(read){
        for(uint8_t i=0;i<FILTER_NUM;i++)
        {
            filters[i].F_value=dataRead[i];
            filters[i].final=filters[i].new=filters[i].last=0;
        }
    }
    else
    {
        for(uint8_t i=0;i<FILTER_NUM;i++)
        {
            filters[i].F_value=filters[i].final=filters[i].new=filters[i].last=0;
        }
    }
    
}



//AI写的四元解算
/**
 * @brief 六轴四元数姿态解算（最简版）
 * @param gx, gy, gz  陀螺仪角速度，单位 °/s
 * @param ax, ay, az  加速度计，单位 g
 * @param roll        输出横滚角，单位 °
 * @param pitch       输出俯仰角，单位 °
 * @param yaw         输出航向角（相对），单位 °
 * @param dt          解算周期，单位 s
 */
#define ATTITUDE_DEG2RAD  (0.0174532925f)
#define ATTITUDE_KP       (0.5f)   // 加速度修正系数：越大收敛越快、抗零偏越强，但越容易抖

void Attitude_Solve(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float *roll, float *pitch, float *yaw,
                    float dt)
{
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float norm, vx, vy, vz, ex, ey, ez;
    float gx_r, gy_r, gz_r;
    float q0_t, q1_t, q2_t, q3_t;
    float sp;

    //处理零偏
    gx -= NB_PREV.gx_bias;
    gy -= NB_PREV.gy_bias;
    gz -= NB_PREV.gz_bias;


    // 1) 加速度归一化
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    ax /= norm; ay /= norm; az /= norm;

    // 2) 由四元数估算重力方向（机体坐标系下）
    vx = 2.0f*(q1*q3 - q0*q2);
    vy = 2.0f*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3) 加速度实测方向与估算重力方向的叉积 = 姿态误差
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    // 4) 陀螺仪积分（°/s 转 rad/s），并用误差修正（互补滤波，此处仅 P 项）
    //    注：未做陀螺零偏标定，yaw 会持续漂移、roll/pitch 有残余静差，属正常
    gx_r = (gx * ATTITUDE_DEG2RAD) + ATTITUDE_KP * ex;
    gy_r = (gy * ATTITUDE_DEG2RAD) + ATTITUDE_KP * ey;
    gz_r = (gz * ATTITUDE_DEG2RAD) + ATTITUDE_KP * ez;

    // 5) 一阶四元数更新（全部用旧值 q0~q3，避免顺序错）
    q0_t = q0 + (-q1*gx_r - q2*gy_r - q3*gz_r) * dt * 0.5f;
    q1_t = q1 + ( q0*gx_r + q2*gz_r - q3*gy_r) * dt * 0.5f;
    q2_t = q2 + ( q0*gy_r - q1*gz_r + q3*gx_r) * dt * 0.5f;
    q3_t = q3 + ( q0*gz_r + q1*gy_r - q2*gx_r) * dt * 0.5f;

    q0 = q0_t; q1 = q1_t; q2 = q2_t; q3 = q3_t;

    // 6) 四元数归一化
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.0f) {
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }

    // 7) 转欧拉角（ZYX 顺序）
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.29578f;
    sp     = 2.0f*(q0*q2 - q3*q1);
    if (sp >  1.0f) sp =  1.0f;   // 防数值越界导致 asinf 返回 NaN
    if (sp < -1.0f) sp = -1.0f;
    *pitch = asinf(sp) * 57.29578f;
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.29578f;
}



// extern int16_t mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z;            // 三轴陀螺仪数据   gyro (陀螺仪)
// extern int16_t mpu6050_acc_x, mpu6050_acc_y , mpu6050_acc_z ; 

void Fliter_update(void)
{
    NEUQ_MPU6050_read_data();

    for(uint8_t i=0;i<FILTER_NUM;i++)
    {
        filters[i].final=filters[i].new*(10.0f-filters[i].F_value)
            /10.0f+filters[i].last*filters[i].F_value/10.0f;
        filters[i].last=filters[i].final;
    }
    filters[FILTER_AX].new=mpu6050_acc_x;
    filters[FILTER_AY].new=mpu6050_acc_y;
    filters[FILTER_AZ].new=mpu6050_acc_z;
    filters[FILTER_GX].new=mpu6050_gyro_x;
    filters[FILTER_GY].new=mpu6050_gyro_y;
    filters[FILTER_GZ].new=mpu6050_gyro_z;
    Attitude_Solve(MPU6050_gyro_transition(filters[FILTER_GX].final),
    MPU6050_gyro_transition(filters[FILTER_GY].final),
    MPU6050_gyro_transition(filters[FILTER_GZ].final),
    MPU6050_acc_transition(filters[FILTER_AX].final),
    MPU6050_acc_transition(filters[FILTER_AY].final),
    MPU6050_acc_transition(filters[FILTER_AZ].final),
    &Calcu_Value.roll,&Calcu_Value.pitch,&Calcu_Value.yaw,0.01);//四元解算
    if(NB_PREV.editing_NullBias_Flag==1)
    {
    
        if((NB_PREV.count<300)&&(NB_PREV.editing_NullBias_Flag==1))
        {
            NB_PREV.sum_gx+=MPU6050_acc_transition(filters[FILTER_GX].final);
            NB_PREV.sum_gy+=MPU6050_acc_transition(filters[FILTER_GY].final);
            NB_PREV.sum_gz+=MPU6050_acc_transition(filters[FILTER_GZ].final);
            NB_PREV.count++;
            NEUQ_Delay_ms(1);
        }
        else
        {
            NB_PREV.gx_bias=NB_PREV.sum_gx/NB_PREV.count;
            NB_PREV.gy_bias=NB_PREV.sum_gy/NB_PREV.count;
            NB_PREV.gz_bias=NB_PREV.sum_gz/NB_PREV.count;
            NB_PREV.count=NB_PREV.sum_gx=NB_PREV.sum_gy=NB_PREV.sum_gz=0;
            NB_PREV.editing_NullBias_Flag=0;
            Menu_Show();
        }
    }
    else if(NB_PREV.count!=0)
    {
        NB_PREV.count=NB_PREV.sum_gx=NB_PREV.sum_gy=NB_PREV.sum_gz=0;
        NB_PREV.editing_NullBias_Flag==0;
    }
}
