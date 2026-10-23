#include "../NEUQ_lib_include.h"

volatile float Q2_raw[Q2_AXIS_NUM]  = {0};
volatile float Q2_raw_o[Q2_AXIS_NUM]  = {0};
volatile float Q2_filt[Q2_AXIS_NUM] = {0};
float    Q2_kalman_q = Q2_Q_DEF;
float    Q2_kalman_r = Q2_R_DEF;
uint8_t  Q2_axis    = Q2_AXIS_AX;
uint8_t  Q2_send_on = 0;
uint8_t  Q2_cal_ok  = 0;
uint8_t  Q2_imu_still = 0;   /* 零速判定结果(1=已连续静置满 Q2_ZVS_STILL_TICKS 拍), 供航向积分冻结闸门使用 */


const char *Q2_AXIS_NAMES[Q2_AXIS_NUM] = {"AX", "AY", "AZ", "GX", "GY", "GZ"};

static KALMAN_STRUCT kalman_ax, kalman_ay, kalman_az;
static KALMAN_STRUCT kalman_gx, kalman_gy, kalman_gz;

float raw_out, filt_out;

/* 
* 三轴角速度都低于阈值且持续满时间 -> 判定静置, 此时读数只含残余零偏,
* 慢速把残余并入零偏 -> 残余 GZ 偏置(航向漂移率)被持续抵消;
* 一旦有转动(|角速度|超过阈值)立即停更, 不影响正常姿态 */
#define Q2_ZVS_THRESH_DPS   1.0f    /* 静置判定阈值(°/s) */
#define Q2_ZVS_STILL_TICKS  200     /* 静置持续时间(5ms 一拍, 200=1s) */
#define Q2_ZVS_GAIN         0.001f  /* 零偏更新增益(每拍), 等效时间常数约 5s */


/* ------------------------------- Flash 参数 ------------------------------- */
#define Q2_CONFIG_MAGIC   0x4E513231UL   /* "NQ21" */
#define Q2_FLASH_ADDR     0x1F0000UL     /* W25Q16 末端扇区, 避开其他数据 */

typedef struct
{
    uint32_t magic;       /* 有效性校验: 有效记录才使用 */
    float    r_kalman;    /* 卡尔曼观测噪声(滤波参数) */
    float    q_kalman;    /* 卡尔曼过程噪声(滤波参数) */
    uint8_t  axis;        /* 发送轴 0..5 */
    uint8_t  send_on;     /* 发送开关 */
    uint8_t  res[2];
} Q2_Config_t;


void Q2_Config_Load(void)
{
    Q2_Config_t cfg;

    if (NEUQ_Flash_read(Q2_FLASH_ADDR, (uint8_t *)&cfg, sizeof(cfg)) == NEUQ_FLASH_OK &&
        cfg.magic == Q2_CONFIG_MAGIC)
    {
        Q2_kalman_r = (cfg.r_kalman >= Q2_R_MIN && cfg.r_kalman <= Q2_R_MAX) ? cfg.r_kalman : Q2_R_DEF;
        Q2_kalman_q = (cfg.q_kalman >= Q2_Q_MIN && cfg.q_kalman <= Q2_Q_MAX) ? cfg.q_kalman : Q2_Q_DEF;
        Q2_axis     = (cfg.axis < Q2_AXIS_NUM) ? cfg.axis : Q2_AXIS_AX;
        Q2_send_on  = (cfg.send_on <= 1) ? cfg.send_on : 0;
    }
    else
    {
        /* 无有效记录, 采用合理默认值(考核要求) */
        Q2_kalman_r = Q2_R_DEF;
        Q2_kalman_q = Q2_Q_DEF;
        Q2_axis     = Q2_AXIS_AX;
        Q2_send_on  = 0;
    }
}

uint8_t Q2_Config_Save(void)
{
    Q2_Config_t cfg;

    cfg.magic    = Q2_CONFIG_MAGIC;
    cfg.r_kalman = Q2_kalman_r;
    cfg.q_kalman = Q2_kalman_q;
    cfg.axis     = Q2_axis;
    cfg.send_on  = Q2_send_on;
    cfg.res[0]   = 0;
    cfg.res[1]   = 0;

    return (NEUQ_Flash_save(Q2_FLASH_ADDR, (uint8_t *)&cfg, sizeof(cfg)) == NEUQ_FLASH_OK);
    // return (NEUQ_Flash_init() == NEUQ_FLASH_OK);
}

static void VOFA_Send_Frame(float raw, float filt)
{
    raw_out = raw;
    filt_out = filt;
    
    printf("%f,%f,%f,%f,%f\n",raw_out,filt_out,IMU.angle.yaw_a,IMU.angle.pitch_a,IMU.angle.roll_a);
    // printf("%f,%f\n",raw_out,filt_out);
}

/* --------------------------- 六轴卡尔曼滤波器初始化 --------------------------- */
void Q2_Filter_Init(void)
{
    Kanman_Init(&kalman_ax);
    Kanman_Init(&kalman_ay);
    Kanman_Init(&kalman_az);
    Kanman_Init(&kalman_gx);
    Kanman_Init(&kalman_gy);
    Kanman_Init(&kalman_gz);
}

/* ------------------------------- 5ms 周期任务 ------------------------------- */
/* 由 NEUQ_PIT_2_CALLBACK 每 5ms 调用, 单条数据链:
 *   采集 -> 去零偏(Q2_raw) -> 卡尔曼滤波(Q2_filt)
 *        -> 同一份 Q2_filt: ① 数据页显示 + VOFA 波形发送
 *                          ② 喂给 Madgwick 四元数姿态更新
 *   因此滤波参数 Q/R 同时决定六轴滤波效果与姿态平滑程度,
 *   两者数据链一致、效果连贯 */
// static uint16_t still_cnt = 0;
void Q2_IMU_Tick(void)
{
    static uint8_t first = 1;   /* 首拍用当前值初始化卡尔曼估计, 避免上电收敛跳变 */
    static uint8_t cnt   = 0;
    static uint16_t still_cnt = 0;   /* 连续静置拍数(零速偏置补偿用) */

    /* 1. 原始六轴(去零偏; AZ 保留重力参考, 静止时约为 +1g) */
    NEUQ_MPU6050_read_data();
    Q2_raw_o[Q2_AXIS_AX]=MPU6050_acc_transition(mpu6050_acc_x);
    Q2_raw_o[Q2_AXIS_AY]=MPU6050_acc_transition(mpu6050_acc_x);
    Q2_raw_o[Q2_AXIS_AZ]=MPU6050_acc_transition(mpu6050_acc_x);
    Q2_raw_o[Q2_AXIS_GX]=MPU6050_acc_transition(mpu6050_acc_x);
    Q2_raw_o[Q2_AXIS_GY]=MPU6050_acc_transition(mpu6050_acc_x);
    Q2_raw_o[Q2_AXIS_GZ]=MPU6050_acc_transition(mpu6050_acc_x);





    Q2_raw[Q2_AXIS_AX] = MPU6050_acc_transition(mpu6050_acc_x)  - IMU.data.offset.acce_x;
    Q2_raw[Q2_AXIS_AY] = MPU6050_acc_transition(mpu6050_acc_y)  - IMU.data.offset.acce_y;
    Q2_raw[Q2_AXIS_AZ] = MPU6050_acc_transition(mpu6050_acc_z)  - IMU.data.offset.acce_z;
    Q2_raw[Q2_AXIS_GX] = MPU6050_gyro_transition(mpu6050_gyro_x) - IMU.data.offset.gyro_x;
    Q2_raw[Q2_AXIS_GY] = MPU6050_gyro_transition(mpu6050_gyro_y) - IMU.data.offset.gyro_y;
    Q2_raw[Q2_AXIS_GZ] = MPU6050_gyro_transition(mpu6050_gyro_z) - IMU.data.offset.gyro_z;


/* 1.5 零速在线偏置补偿(航向漂移治本, 参数见 Q2_IMU_Task.h):
* 三轴角速度都低于阈值且持续满 1s -> 判定静置, 此时读数只含残余零偏,
* 慢速把残余并入零偏 -> 残余 GZ 偏置(航向漂移率)被持续抵消;
* 下一拍的 Q2_raw 会自动减去更新后的零偏, 形成闭环。
* 一旦有转动立即清零计数、停更, 不会吃掉真实运动 */
    if (fabsf(Q2_raw[Q2_AXIS_GX]) < Q2_ZVS_THRESH_DPS &&
                fabsf(Q2_raw[Q2_AXIS_GY]) < Q2_ZVS_THRESH_DPS &&
                fabsf(Q2_raw[Q2_AXIS_GZ]) < Q2_ZVS_THRESH_DPS)
    {
        still_cnt++;
        if (still_cnt >= Q2_ZVS_STILL_TICKS)
        {
            IMU.data.offset.gyro_x += Q2_ZVS_GAIN * Q2_raw[Q2_AXIS_GX];
            IMU.data.offset.gyro_y += Q2_ZVS_GAIN * Q2_raw[Q2_AXIS_GY];
            IMU.data.offset.gyro_z += Q2_ZVS_GAIN * Q2_raw[Q2_AXIS_GZ];
            }
        }
    else
    {
        still_cnt = 0;
    }



    /* 2. 六轴独立卡尔曼滤波(一维标量, 每通道一个实例) */
    if (first)                              /* 首拍预填估计值, 避免上电收敛跳变 */
    {
        kalman_ax.Angel = Q2_raw[Q2_AXIS_AX];
        kalman_ay.Angel = Q2_raw[Q2_AXIS_AY];
        kalman_az.Angel = Q2_raw[Q2_AXIS_AZ];
        kalman_gx.Angel = Q2_raw[Q2_AXIS_GX];
        kalman_gy.Angel = Q2_raw[Q2_AXIS_GY];
        kalman_gz.Angel = Q2_raw[Q2_AXIS_GZ];
        first = 0;
    }

    /* 滤波参数 Q/R 每拍写入, 界面调参实时生效 */
    kalman_ax.Q_Angle = Q2_kalman_q;
    kalman_ay.Q_Angle = Q2_kalman_q;
    kalman_az.Q_Angle = Q2_kalman_q;
    kalman_gx.Q_Angle = Q2_kalman_q;
    kalman_gy.Q_Angle = Q2_kalman_q;
    kalman_gz.Q_Angle = Q2_kalman_q;
    kalman_ax.R_Angle = Q2_kalman_r;
    kalman_ay.R_Angle = Q2_kalman_r;
    kalman_az.R_Angle = Q2_kalman_r;
    kalman_gx.R_Angle = Q2_kalman_r;
    kalman_gy.R_Angle = Q2_kalman_r;
    kalman_gz.R_Angle = Q2_kalman_r;

    Kanman_Filter_Scalar(&kalman_ax, Q2_raw[Q2_AXIS_AX], 5);
    Kanman_Filter_Scalar(&kalman_ay, Q2_raw[Q2_AXIS_AY], 5);
    Kanman_Filter_Scalar(&kalman_az, Q2_raw[Q2_AXIS_AZ], 5);
    Kanman_Filter_Scalar(&kalman_gx, Q2_raw[Q2_AXIS_GX], 5);
    Kanman_Filter_Scalar(&kalman_gy, Q2_raw[Q2_AXIS_GY], 5);
    Kanman_Filter_Scalar(&kalman_gz, Q2_raw[Q2_AXIS_GZ], 5);

    Q2_filt[Q2_AXIS_AX] = kalman_ax.Angel;
    Q2_filt[Q2_AXIS_AY] = kalman_ay.Angel;
    Q2_filt[Q2_AXIS_AZ] = kalman_az.Angel;
    Q2_filt[Q2_AXIS_GX] = kalman_gx.Angel;
    Q2_filt[Q2_AXIS_GY] = kalman_gy.Angel;
    Q2_filt[Q2_AXIS_GZ] = kalman_gz.Angel;

    /* 3. 四元数姿态更新(Madgwick, 5ms -> 200Hz):
     *    直接使用第 2 步的卡尔曼滤波后数据, 与显示/VOFA 同源 */
    Madgwick_Update(Q2_filt[Q2_AXIS_AX], Q2_filt[Q2_AXIS_AY], Q2_filt[Q2_AXIS_AZ],
                    Q2_filt[Q2_AXIS_GX], Q2_filt[Q2_AXIS_GY], Q2_filt[Q2_AXIS_GZ],
                    5.0f);

    /* 4. VOFA+ 波形发送: 100Hz, 只发选中轴, 两条曲线同帧同采样 */
    cnt++;
    if (cnt >= 2)
    {
        cnt = 0;
        if (Q2_send_on)
            VOFA_Send_Frame(Q2_raw[Q2_axis], Q2_filt[Q2_axis]);
    }
}
