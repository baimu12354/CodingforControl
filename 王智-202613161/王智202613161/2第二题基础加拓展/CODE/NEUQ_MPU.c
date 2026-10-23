#include "NEUQ_lib_include.h"

//宏
#define MPU_FLASH_SECTOR        510           // 参数存第 510 扇区（511 是库的掉电演示区）
#define MPU_FLASH_MAGIC         0x004D5055u   // "MPU" 校验头，判断是否写入过有效参数
#define MPU_PARAM_DEFAULT       10            // 默认滤波参数 -> alpha = 0.10
#define MPU_SAMPLE_PERIOD_MS    5          // 采样周期 200Hz（与 MadgwickAHRS.h 的 sampleFreq 对应）

//全局状态
static float   filt_acc[3]  = {0,0,0};      // 滤波后加速度 (g)      [0]=X [1]=Y [2]=Z
static float   filt_gyro[3] = {0,0,0};      // 滤波后角速度 (deg/s)  [0]=X [1]=Y [2]=Z
static float   mpu_bias[3] = {0,0,0};      // 陀螺零偏 (deg/s)
static float   roll  = 0;             // 横滚角 (deg)
static float   pitch = 0;             // 俯仰角 (deg)
static float   yaw   = 0;             // 航向角 (deg，相对初始方向)
static Madgwick ahrs;                       // Madgwick 四元数对象
int8_t i;
extern volatile uint32_t cnt_ms; // 计数器，单位 ms，由 PIT2 中断累加
//Flash 读
static void MPU_LoadParam(void){
    NEUQ_MPU_FilterConfig config;
        params.FilterParams = MPU_PARAM_DEFAULT;                     // 默认兜底
    if (NEUQ_Flash_read_sector(MPU_FLASH_SECTOR, (uint8_t *)&config, sizeof(config)) == NEUQ_FLASH_OK) {  
            if (config.magic == MPU_FLASH_MAGIC && config.param>= 1 && config.param <= 100) {                 // 合法范围 1~100
                params.FilterParams = config.param;
            }
        }
    }


//Flash 写
void NEUQ_MPU_SaveParam(void){
    NEUQ_MPU_FilterConfig config;
    config.magic = MPU_FLASH_MAGIC;
           config.param = params.FilterParams;
    NEUQ_Flash_write_sector(MPU_FLASH_SECTOR, (uint8_t *)&config, sizeof(config));
}



//一次采样 + 六轴滤波
static void MPU_SampleFilter(void){
    float alpha;
        alpha = params.FilterParams / 100.0f;   // 参数 1~100 -> alpha 0.01~1.00
        if (alpha < 0.01f) alpha = 0.01f;             // 防止参数为 0 时滤波被冻结
        if (alpha > 1.00f) alpha = 1.00f;             // 防止参数大于 100 时滤波过弱
    
    
    NEUQ_MPU6050_read_data();                     // 一次读取，得到六轴原始值

    MPU_LowPass(&filt_acc[0], MPU6050_acc_transition(mpu6050_acc_x), alpha);
    MPU_LowPass(&filt_acc[1], MPU6050_acc_transition(mpu6050_acc_y), alpha);
    MPU_LowPass(&filt_acc[2], MPU6050_acc_transition(mpu6050_acc_z), alpha);

    MPU_LowPass(&filt_gyro[0],NEUQ_MPU_GetCalibratedGyro(4), alpha);
    MPU_LowPass(&filt_gyro[1],NEUQ_MPU_GetCalibratedGyro(5), alpha);
    MPU_LowPass(&filt_gyro[2],NEUQ_MPU_GetCalibratedGyro(6), alpha);
}

//陀螺零偏校准（上电静置取平均）
// 注意：调用时务必保持模块静止
extern void MPU_Calibrate(void)
{   
    NEUQ_Delay_ms(500); 
    const int N = 2000;  // 采样次数
    float sum[3] = {0, 0, 0};
    NEUQ_LCD200_ClearRect(8,144,120,24);
    NEUQ_LCD200_DisplayString(8,144,"Pending!!!"); 
    for (int i = 0; i < N; i++) {
        NEUQ_MPU6050_read_data();
        sum[0] += MPU6050_gyro_transition(mpu6050_gyro_x);  //单位均为 deg/s
        sum[1] += MPU6050_gyro_transition(mpu6050_gyro_y);  //单位均为 deg/s
        sum[2] += MPU6050_gyro_transition(mpu6050_gyro_z);  //单位均为 deg/s
        NEUQ_Delay_us(500);         
// 相邻采样间隔，总共约 3秒！！！
    }
    mpu_bias[0] = sum[0] / N;  //取平均值
    mpu_bias[1] = sum[1] / N;  //取平均值
    mpu_bias[2] = sum[2] / N;  //取平均值
    NEUQ_LCD200_ClearRect(8,144,120,24);
    NEUQ_LCD200_DisplayString(8,144,"OK");      
}
//  陀螺零偏校准值
float NEUQ_MPU_GetCalibratedGyro(uint8_t axis){
    switch (axis) {
        case 4: return MPU6050_gyro_transition(mpu6050_gyro_x) - mpu_bias[0];
        case 5: return MPU6050_gyro_transition(mpu6050_gyro_y) - mpu_bias[1];
        case 6: return MPU6050_gyro_transition(mpu6050_gyro_z) - mpu_bias[2];
        default: return 0;
    }
}
//姿态更新（四元数 -> 欧拉角）
static void MPU_AttitudeUpdate(void)
{
    // 陀螺仪：去零偏，deg/s 转 rad/s
    float gx = (MPU6050_gyro_transition(mpu6050_gyro_x) - mpu_bias[0]) * DEG2RAD;
    float gy = (MPU6050_gyro_transition(mpu6050_gyro_y) - mpu_bias[1]) * DEG2RAD;
    float gz = (MPU6050_gyro_transition(mpu6050_gyro_z) - mpu_bias[2]) * DEG2RAD;

    // 加速度：单位 g
    float ax = MPU6050_acc_transition(mpu6050_acc_x);
    float ay = MPU6050_acc_transition(mpu6050_acc_y);
    float az = MPU6050_acc_transition(mpu6050_acc_z);

    // IMU 模式四元数更新（含四元数归一化）
    Madgwick_updateIMU(&ahrs, gx, gy, gz, ax, ay, az);

    // 四元数 -> 欧拉角（弧度转度）
    roll  = Madgwick_getRoll(&ahrs)  * RAD2DEG; //横滚，前进方向；+-180
    pitch = Madgwick_getPitch(&ahrs) * RAD2DEG;//俯仰，抬头低头；+-90
    yaw   = Madgwick_getYaw(&ahrs)   * RAD2DEG;//航向角
}

//取指定轴  原始值  （1=Ax 2=Ay 3=Az 4=Gx 5=Gy 6=Gz）
float NEUQ_MPU_GetRawValue(uint8_t axis)
{
    switch (axis) {
        case 1: return MPU6050_acc_transition(mpu6050_acc_x);
        case 2: return MPU6050_acc_transition(mpu6050_acc_y);
        case 3: return MPU6050_acc_transition(mpu6050_acc_z);
        case 4: return NEUQ_MPU_GetCalibratedGyro(4);  // 返回去零偏值
        case 5: return NEUQ_MPU_GetCalibratedGyro(5);  // 返回去零偏值
        case 6: return NEUQ_MPU_GetCalibratedGyro(6);  // 返回去零偏值
        default: return 0;
    }
}
//取指定轴  滤波值  （1=Ax 2=Ay 3=Az 4=Gx 5=Gy 6=Gz）
float NEUQ_MPU_GetFilteredValue(uint8_t axis)
{
    switch (axis) {
        case 1: return filt_acc[0];
        case 2: return filt_acc[1];
        case 3: return filt_acc[2];
        case 4: return filt_gyro[0];
        case 5: return filt_gyro[1];
        case 6: return filt_gyro[2];
        default: return 0;
    }
}

// VOFA+ 上传FireWater
static void MPU_Upload(void){
    if (params.UploadOn == 0 || params.UploadAxis == 0) {
        return;              // 未开启或未选轴 -> 不发送
    }
    float raw  = NEUQ_MPU_GetRawValue((uint8_t)params.UploadAxis);      // 原始值
    float filt = NEUQ_MPU_GetFilteredValue((uint8_t)params.UploadAxis); // 滤波值（同一份采样滤波得到）

    printf("%f,%f\n", raw, filt);  // VOFA+ 协议
}


//姿态页显示
void NEUQ_MPU_ShowAttitude(void)
{
    NEUQ_LCD200_DisplayString(8,48,"Roll:");
    NEUQ_LCD200_ClearRect(120,48,112,24);
    NEUQ_LCD200_DisplayFloat(120,48,roll,6,2);
    NEUQ_LCD200_DisplayString(232,48, "deg");
    NEUQ_LCD200_DisplayString(8,80,"Pitch:");
    NEUQ_LCD200_ClearRect(120,80,112,24);
    NEUQ_LCD200_DisplayFloat(120,80,pitch,6,2);
    NEUQ_LCD200_DisplayString(232,80,"deg");
    NEUQ_LCD200_DisplayString(8,112,"Yaw:");
    NEUQ_LCD200_ClearRect(120,112,112,24);
    NEUQ_LCD200_DisplayFloat(120,112,yaw,6,2);
    NEUQ_LCD200_DisplayString(232,112,"deg");
}


static void MPU_SampleTask(void)
{
    MPU_SampleFilter();      // 读数据 + 六轴滤波
    MPU_AttitudeUpdate();    // 姿态解算
    MPU_Upload();         
}

//上电初始化
void NEUQ_MPU_Init(void)
{
    NEUQ_MPU6050_init();    
    NEUQ_Flash_init();       
    MPU_LoadParam();         
    Madgwick_Init(&ahrs, betaDef);     
    MPU_Calibrate();     
}


void NEUQ_MPU_Task(void)
{
    static uint32_t last_smp = 0;
    uint32_t now = cnt_ms;
    if ((uint32_t)(now - last_smp) >= MPU_SAMPLE_PERIOD_MS) {
        last_smp = now;
        MPU_SampleTask();
    }
}
