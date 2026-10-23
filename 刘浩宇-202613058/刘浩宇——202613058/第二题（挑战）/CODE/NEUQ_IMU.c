/*********************************************************************************************************************
* 功能说明
*   1. MPU6050 三轴加速度 + 三轴角速度采集与一阶低通滤波（供六轴显示、VOFA+ 波形使用）
*   2. 基于四元数的姿态更新（Mahony 互补滤波）：陀螺仪积分 + 加速度计重力方向校正
*   3. 四元数 -> 欧拉角（横滚 roll、俯仰 pitch、相对航向 yaw），供“姿态解算”页面显示
*   4. 静态陀螺仪零偏校准（上电自动）与静止残余零偏跟踪
*   5. 每一步更新后对四元数归一化，长期运行不发散
*
* 坐标系与单位
*   机体坐标系：右手系，X 轴指向模块正前方，Y 轴指向左侧，Z 轴垂直向上
*   世界坐标系：Z 轴沿重力反方向（竖直向上），X/Y 水平
*   陀螺仪：dps（度/秒）；加速度计：g；姿态角：deg（度）
*   姿态更新周期：10ms 任务（100Hz），积分步长按实际节拍计算，最大 50ms
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include <math.h>

extern volatile uint32_t sys_tick_10ms;
#define IMU_PI                 3.14159265358979f
#define IMU_DEG2RAD            (IMU_PI / 180.0f)
#define IMU_RAD2DEG            (180.0f / IMU_PI)

#define IMU_KP                 0.8f     // 加速度计比例校正增益 (1/s)
#define IMU_KI                 0.02f    // 加速度计积分校正增益 (1/s^2)
#define IMU_INT_LIMIT          0.15f    // 积分项限幅 (rad/s)，约 8.6dps，防止积分饱和

#define IMU_DT_MAX             0.05f    // 单步积分最大步长 (s)，防止界面刷新阻塞后积分爆炸

#define IMU_CALIB_SAMPLES      200      // 按键静态校准：200 x 10ms = 2s
#define IMU_CALIB_STD_MAX      3.0f     // 校准判据：三轴角速度标准差上限 (dps)

#define IMU_STATIC_TAU         3.0f     // 静止残余零偏跟踪时间常数 (s)
#define IMU_STATIC_GYRO_MAX    1.0f     // 判定为静止的角速度模值上限 (dps)
#define IMU_STATIC_ACC_ERR     0.05f    // 判定为静止的加速度模值误差 (g)
#define IMU_STATIC_CNT_MAX     50       // 连续静止 0.5s 后才开始跟踪零偏

float filtered_acc_x  = 0.0f;   // 滤波后加速度 X (g)
float filtered_acc_y  = 0.0f;   // 滤波后加速度 Y (g)
float filtered_acc_z  = 0.0f;   // 滤波后加速度 Z (g)
float filtered_gyro_x = 0.0f;   // 滤波后角速度 X (dps)
float filtered_gyro_y = 0.0f;   // 滤波后角速度 Y (dps)
float filtered_gyro_z = 0.0f;   // 滤波后角速度 Z (dps)

float attitude_pitch = 0.0f;    // 俯仰角 (deg)
float attitude_roll  = 0.0f;    // 横滚角 (deg)
float attitude_yaw   = 0.0f;    // 航向角 (deg)，相对最近一次校准时刻

float imu_q0 = 1.0f;            // 四元数 w
float imu_q1 = 0.0f;            // 四元数 x
float imu_q2 = 0.0f;            // 四元数 y
float imu_q3 = 0.0f;            // 四元数 z

float gyro_bias_x = 0.0f;       // 陀螺仪零偏 X (dps)
float gyro_bias_y = 0.0f;       // 陀螺仪零偏 Y (dps)
float gyro_bias_z = 0.0f;       // 陀螺仪零偏 Z (dps)

float imu_dt_ms = 10.0f;        // 最近一次积分步长 (ms)

volatile uint8_t imu_calib_state   = IMU_CALIB_IDLE;  // 校准状态
volatile uint8_t imu_calib_percent = 0;               // 校准进度 0~100

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 机体 -> 世界 四元数
static float ex_int = 0.0f, ey_int = 0.0f, ez_int = 0.0f; // 加速度校正积分项 (rad/s)
static float yaw_accum  = 0.0f;   // 展开后的连续航向角 (deg)
static float last_yaw   = 0.0f;   // 上一次单项欧拉航向 (deg)，用于展开
static float acc_norm_lp = 1.0f;  // 加速度模值低通，用于静止判定
static uint16_t static_cnt = 0;   // 连续静止计数
static uint8_t  is_mpu_first_read = 1;
static uint8_t  solve_started = 0;
static uint32_t last_solve_tick = 0;

static uint16_t calib_n = 0;      // 校准采样计数
static float calib_sx = 0.0f, calib_sy = 0.0f, calib_sz = 0.0f;
static float calib_sxx = 0.0f, calib_syy = 0.0f, calib_szz = 0.0f;

static float IMU_Abs(float v){
    return (v < 0.0f) ? -v : v;
}

static void IMU_QuatNormalize(void)
{
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-4f && norm < 1e4f) {
        float inv = 1.0f / norm;
        q0 *= inv;
        q1 *= inv;
        q2 *= inv;
        q3 *= inv;
    } else {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }
}

/* 四元数 -> 欧拉角（ZYX 顺序）：
 * roll  = atan2(2(q0q1+q2q3), 1-2(q1^2+q2^2))
 * pitch = asin(2(q0q2-q1q3))
 * yaw   = atan2(2(q1q2+q0q3), q0^2+q1^2-q2^2-q3^2)
 * 航向角在 [-180,180] 处展开成连续值，便于观察转动趋势
 */
static void IMU_UpdateEuler(void)
{
    float roll, pitch, yaw, sp, dy;

    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * IMU_RAD2DEG;

    sp = 2.0f * (q0 * q2 - q3 * q1);
    if (sp > 1.0f)  sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    pitch = asinf(sp) * IMU_RAD2DEG;

    yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * IMU_RAD2DEG;

    dy = yaw - last_yaw;
    if (dy > 180.0f)  dy -= 360.0f;
    if (dy < -180.0f) dy += 360.0f;
    yaw_accum += dy;
    last_yaw = yaw;

    attitude_roll  = roll;
    attitude_pitch = pitch;
    attitude_yaw   = yaw_accum;

    imu_q0 = q0;
    imu_q1 = q1;
    imu_q2 = q2;
    imu_q3 = q3;
}

/* 用加速度计当前读数确定横滚/俯仰，航向清零：
 * 校准结束时调用，使航向以“校准时刻的方向”为相对零点。
 */
static void IMU_ResetFromAccel(void)
{
    float ax = MPU6050_acc_transition(mpu6050_acc_x);
    float ay = MPU6050_acc_transition(mpu6050_acc_y);
    float az = MPU6050_acc_transition(mpu6050_acc_z);
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    /* q = qz(0) * qy(pitch) * qx(roll) */
    q0 = cp * cr;
    q1 = cp * sr;
    q2 = sp * cr;
    q3 = -sp * sr;
    IMU_QuatNormalize();
    ex_int = 0.0f;
    ey_int = 0.0f;
    ez_int = 0.0f;
    yaw_accum = 0.0f;
    last_yaw  = 0.0f;
    acc_norm_lp = 1.0f;
    static_cnt = 0;
    IMU_UpdateEuler();
}

/* 上电静态零偏校准：连续采样求均值，并用标准差判断模块是否静止。
 */
static uint8_t IMU_CalibrateAtBoot(void)
{
    float bx = 0.0f, by = 0.0f, bz = 0.0f;
    uint8_t ok = 0;
    uint8_t attempt;

    for (attempt = 0; (attempt < 3) && (ok == 0); attempt++) {//最多试3次
        float sx = 0.0f, sy = 0.0f, sz = 0.0f;
        float sxx = 0.0f, syy = 0.0f, szz = 0.0f;
        float vmax, std;
        uint16_t i;

        for (i = 0; i < 2000; i++) {
            float gx, gy, gz;
            NEUQ_MPU6050_read_data();
            gx = MPU6050_gyro_transition(mpu6050_gyro_x);
            gy = MPU6050_gyro_transition(mpu6050_gyro_y);
            gz = MPU6050_gyro_transition(mpu6050_gyro_z);
            sx += gx;  
            sxx += gx * gx;
            sy += gy;  
            syy += gy * gy;
            sz += gz;  
            szz += gz * gz;
            NEUQ_Delay_ms(2);
        }

        bx = sx / 2000.0f;
        by = sy / 2000.0f;
        bz = sz / 2000.0f;

        vmax = sxx / 2000.0f - bx * bx;
        if (syy / 2000.0f - by * by > vmax) {
            vmax = syy / 2000.0f - by * by;
        }
        if (szz / 2000.0f - bz * bz > vmax) {
            vmax = szz / 2000.0f - bz * bz;
        }
        std = (vmax > 0.0f) ? sqrtf(vmax) : 0.0f;

        if (std < IMU_CALIB_STD_MAX) {
            ok = 1;
        }
    }

    gyro_bias_x = bx;
    gyro_bias_y = by;
    gyro_bias_z = bz;
    return ok;
}
 
void NEUQ_IMU_Init(void)
{
    uint8_t ok;
    NEUQ_MPU6050_init();
    NEUQ_Delay_ms(500);
    ok = IMU_CalibrateAtBoot();         // 上电自动静态零偏校准（约 0.6s/次）
    IMU_ResetFromAccel();               // 初始横滚/俯仰由加速度计给出，航向清零
    imu_calib_percent = 100;
    imu_calib_state = (ok != 0) ? IMU_CALIB_OK : IMU_CALIB_FAIL;
    is_mpu_first_read = 1;
    solve_started = 0;

    printf("[IMU] calib %s, bias*100 = %d %d %d (dps*100)\r\n",
           (ok != 0) ? "OK" : "FAIL",
           (int)(gyro_bias_x * 100.0f),
           (int)(gyro_bias_y * 100.0f),
           (int)(gyro_bias_z * 100.0f));
}

/* 六轴数据采集 + 一阶低通滤波（10ms 任务）
 * 加速度直接滤波，角速度先扣除零偏再滤波；滤波系数由菜单参数决定。
 */
void MPU6050_Filter_Task(void)
{
    float alpha, beta;
    float raw_acc_x, raw_acc_y, raw_acc_z;
    float raw_gyro_x, raw_gyro_y, raw_gyro_z;

    NEUQ_MPU6050_read_data();

    alpha = (params.filter_alpha >= 0 && params.filter_alpha <= 100) ? (params.filter_alpha / 100.0f) : 0.5f;
    beta  = 1.0f - alpha;

    raw_acc_x = MPU6050_acc_transition(mpu6050_acc_x);
    raw_acc_y = MPU6050_acc_transition(mpu6050_acc_y);
    raw_acc_z = MPU6050_acc_transition(mpu6050_acc_z);

    raw_gyro_x = MPU6050_gyro_transition(mpu6050_gyro_x) - gyro_bias_x;
    raw_gyro_y = MPU6050_gyro_transition(mpu6050_gyro_y) - gyro_bias_y;
    raw_gyro_z = MPU6050_gyro_transition(mpu6050_gyro_z) - gyro_bias_z;

    /* 滤波系数为 0 表示“不滤波”：直接输出原始值。
     * 若仍按公式计算，得到 filtered = filtered（会永远停在第一次采样的值），
     * 把参数调到 0 时六轴显示会被“冻住”，这里单独处理成直通。 */
    if (alpha <= 0.0f) {
        filtered_acc_x  = raw_acc_x;
        filtered_acc_y  = raw_acc_y;
        filtered_acc_z  = raw_acc_z;
        filtered_gyro_x = raw_gyro_x;
        filtered_gyro_y = raw_gyro_y;
        filtered_gyro_z = raw_gyro_z;
        is_mpu_first_read = 0;
        return;
    }

    if (is_mpu_first_read) {
        filtered_acc_x  = raw_acc_x;
        filtered_acc_y  = raw_acc_y;
        filtered_acc_z  = raw_acc_z;
        filtered_gyro_x = raw_gyro_x;
        filtered_gyro_y = raw_gyro_y;
        filtered_gyro_z = raw_gyro_z;
        is_mpu_first_read = 0;
        return;
    }

    filtered_acc_x  = alpha * raw_acc_x  + beta * filtered_acc_x;
    filtered_acc_y  = alpha * raw_acc_y  + beta * filtered_acc_y;
    filtered_acc_z  = alpha * raw_acc_z  + beta * filtered_acc_z;
    filtered_gyro_x = alpha * raw_gyro_x + beta * filtered_gyro_x;
    filtered_gyro_y = alpha * raw_gyro_y + beta * filtered_gyro_y;
    filtered_gyro_z = alpha * raw_gyro_z + beta * filtered_gyro_z;
}

void Attitude_Solve_Task(void)
{
    uint32_t now = sys_tick_10ms;
    uint32_t dtick;
    float dt;
    float ax, ay, az, an;
    float gx_dps, gy_dps, gz_dps, gmag;
    float gx, gy, gz;
    float ex, ey, ez;
    float halfvx, halfvy, halfvz;

    if (solve_started == 0) {
        solve_started = 1;
        last_solve_tick = now;
        return;
    }

    dtick = now - last_solve_tick;
    if (dtick == 0) {
        return;
    }
    last_solve_tick = now;

    dt = (float)dtick * 0.01f;
    if (dt > IMU_DT_MAX) dt = IMU_DT_MAX;
    imu_dt_ms = dt * 1000.0f;

    if (imu_calib_state == IMU_CALIB_RUNNING) {
        return;                         // 校准期间冻结姿态，校准结束后重新给出基准
    }

    ax = MPU6050_acc_transition(mpu6050_acc_x);
    ay = MPU6050_acc_transition(mpu6050_acc_y);
    az = MPU6050_acc_transition(mpu6050_acc_z);

    gx_dps = MPU6050_gyro_transition(mpu6050_gyro_x) - gyro_bias_x;
    gy_dps = MPU6050_gyro_transition(mpu6050_gyro_y) - gyro_bias_y;
    gz_dps = MPU6050_gyro_transition(mpu6050_gyro_z) - gyro_bias_z;

    gx = gx_dps * IMU_DEG2RAD;
    gy = gy_dps * IMU_DEG2RAD;
    gz = gz_dps * IMU_DEG2RAD;

    an = sqrtf(ax * ax + ay * ay + az * az);
    acc_norm_lp = 0.98f * acc_norm_lp + 0.02f * an;

    /* 加速度计重力方向校正（误差为机体坐标系下的叉乘） */
    ex = 0.0f;
    ey = 0.0f;
    ez = 0.0f;
    if (an > 0.2f) {
        ax /= an;
        ay /= an;
        az /= an;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        ex = ay * halfvz - az * halfvy;
        ey = az * halfvx - ax * halfvz;
        ez = ax * halfvy - ay * halfvx;

        ex_int += IMU_KI * ex * dt;
        ey_int += IMU_KI * ey * dt;
        ez_int += IMU_KI * ez * dt;
        if (ex_int >  IMU_INT_LIMIT) {
            ex_int =  IMU_INT_LIMIT;
        }
        if (ex_int < -IMU_INT_LIMIT) {
            ex_int = -IMU_INT_LIMIT;
        }
        if (ey_int >  IMU_INT_LIMIT) {
            ey_int =  IMU_INT_LIMIT;
        }
        if (ey_int < -IMU_INT_LIMIT) {
            ey_int = -IMU_INT_LIMIT;
        }
        if (ez_int >  IMU_INT_LIMIT) {
            ez_int =  IMU_INT_LIMIT;
        }
        if (ez_int < -IMU_INT_LIMIT) {
            ez_int = -IMU_INT_LIMIT;
        }

        gx += IMU_KP * ex + ex_int;
        gy += IMU_KP * ey + ey_int;
        gz += IMU_KP * ez + ez_int;
    }

    /* 静止残余零偏跟踪：仅在“上电校准成功 + 加速度模值≈1g + 角速度很小”时生效 */
    gmag = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);
    if (imu_calib_state == IMU_CALIB_OK &&
        IMU_Abs(acc_norm_lp - 1.0f) < IMU_STATIC_ACC_ERR &&
        gmag < IMU_STATIC_GYRO_MAX) {
        if (static_cnt < IMU_STATIC_CNT_MAX) static_cnt++;
    } else {
        static_cnt = 0;
    }
    if (static_cnt >= IMU_STATIC_CNT_MAX) {
        float k = dt / IMU_STATIC_TAU;
        if (k > 0.05f) k = 0.05f;
        gyro_bias_x += k * gx_dps;
        gyro_bias_y += k * gy_dps;
        gyro_bias_z += k * gz_dps;
    }

    /* 四元数微分方程：q_dot = 0.5 * q ⊗ (0, gx, gy, gz) */
    {
        float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
        float dq1 = 0.5f * ( q0 * gx - q3 * gy + q2 * gz) * dt;
        float dq2 = 0.5f * ( q3 * gx + q0 * gy - q1 * gz) * dt;
        float dq3 = 0.5f * (-q2 * gx + q1 * gy + q0 * gz) * dt;

        q0 += dq0;
        q1 += dq1;
        q2 += dq2;
        q3 += dq3;
    }

    IMU_QuatNormalize();
    IMU_UpdateEuler();
}

/* 请求重新静态校准（非阻塞）：真正的采样在 NEUQ_IMU_CalibTask 中完成 */
void NEUQ_IMU_StartCalib(void)
{
    if (imu_calib_state == IMU_CALIB_RUNNING) {
        return;
    }
    calib_n = 0;
    calib_sx = calib_sy = calib_sz = 0.0f;
    calib_sxx = calib_syy = calib_szz = 0.0f;
    imu_calib_percent = 0;
    imu_calib_state = IMU_CALIB_RUNNING;
}

/* 校准状态机：每 10ms 调用一次；运行中返回 1，结束返回 0 */
uint8_t NEUQ_IMU_CalibTask(void)
{
    float gx, gy, gz;

    if (imu_calib_state != IMU_CALIB_RUNNING) {
        return 0;
    }

    gx = MPU6050_gyro_transition(mpu6050_gyro_x);
    gy = MPU6050_gyro_transition(mpu6050_gyro_y);
    gz = MPU6050_gyro_transition(mpu6050_gyro_z);

    calib_sx += gx;
    calib_sy += gy;
    calib_sz += gz;
    calib_sxx += gx * gx;
    calib_syy += gy * gy;
    calib_szz += gz * gz;
    calib_n++;

    imu_calib_percent = (uint8_t)(((uint32_t)calib_n * 100u) / IMU_CALIB_SAMPLES);

    if (calib_n >= IMU_CALIB_SAMPLES) {
        float mx = calib_sx / (float)IMU_CALIB_SAMPLES;
        float my = calib_sy / (float)IMU_CALIB_SAMPLES;
        float mz = calib_sz / (float)IMU_CALIB_SAMPLES;
        float vx = calib_sxx / (float)IMU_CALIB_SAMPLES - mx * mx;
        float vy = calib_syy / (float)IMU_CALIB_SAMPLES - my * my;
        float vz = calib_szz / (float)IMU_CALIB_SAMPLES - mz * mz;
        float vmax = vx;
        float std;

        if (vy > vmax) vmax = vy;
        if (vz > vmax) vmax = vz;
        std = (vmax > 0.0f) ? sqrtf(vmax) : 0.0f;

        if (std < IMU_CALIB_STD_MAX) {
            gyro_bias_x = mx;
            gyro_bias_y = my;
            gyro_bias_z = mz;
            imu_calib_state = IMU_CALIB_OK;
            /* 以校准结束时的姿态为基准：横滚/俯仰取加速度计结果，航向归零 */
            IMU_ResetFromAccel();
        } else {
            /* 校准时模块没有静止：保持原零偏与原有姿态，提示重新校准 */
            imu_calib_state = IMU_CALIB_FAIL;
        }
        imu_calib_percent = 100;
        return 0;
    }

    return 1;
}

uint8_t NEUQ_IMU_IsCalibRunning(void)
{
    return (imu_calib_state == IMU_CALIB_RUNNING) ? 1u : 0u;
}
