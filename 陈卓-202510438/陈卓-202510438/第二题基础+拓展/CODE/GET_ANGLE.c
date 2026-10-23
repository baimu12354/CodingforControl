#include "GET_ANGLE.h"
#include <math.h>

float M_PI = 3.141592653549f;
float PI = 3.141592653549f;
//规定圆周率
/***********************************************************/

void Euler_ANGLE_To_REG(imu_Data_All_s *imu_in);
void Euler_REG_To_ANGLE(void);
void MPU6050_get_value(void);
/***********************************************************/
//巴特沃夫滤波算法
Butter_BufferData gyro_filter_buf[3],accel_filter_buf[3];
Butter_Parameter Accel_Parameter,Gyro_Parameter;
//IMU相关数据总结构体
imu_Data_All_s IMU;
/***********************************************************/
typedef struct {
    double a0, a1, a2; // Coefficients of numerator
    double b0, b1, b2; // Coefficients of denominator
    double x1, x2;     // Input delay elements
    double y1, y2;     // Output delay elements
} Filter;
Filter filter;
// Initialize filter coefficients
void initFilter(Filter* filter, double fs, double f0, double Q) {
    double w0 = 2 * PI * f0 / fs;
    double alpha = sin(w0) / (2 * Q);

    filter->b0 = Q * alpha;
    filter->b1 = 0;
    filter->b2 = -Q * alpha;
    filter->a0 = 1 + alpha;
    filter->a1 = -2 * cos(w0);
    filter->a2 = 1 - alpha;
    filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0;
}
// Apply the filter to a sample
double applyFilter(Filter* filter, double input) {
    double output = (filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                    - filter->a1 * filter->y1 - filter->a2 * filter->y2) / filter->a0;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

//巴特沃夫滤波器初始化
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI / fr);
  float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
}

//二阶低通巴特沃夫滤波器
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
    if(Buffer->Output_Butter[0]==0&&
         Buffer->Output_Butter[1]==0&&
         Buffer->Output_Butter[2]==0&&
         Buffer->Input_Butter[0]==0&&
         Buffer->Input_Butter[1]==0&&
         Buffer->Input_Butter[2]==0)
    {
        Buffer->Output_Butter[0]=curr_input;
        Buffer->Output_Butter[1]=curr_input;
        Buffer->Output_Butter[2]=curr_input;
        Buffer->Input_Butter[0]=curr_input;
        Buffer->Input_Butter[1]=curr_input;
        Buffer->Input_Butter[2]=curr_input;
        return curr_input;
    }

  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=Parameter->b[0] * Buffer->Input_Butter[2]
                                                    +Parameter->b[1] * Buffer->Input_Butter[1]
                                                    +Parameter->b[2] * Buffer->Input_Butter[0]
                                                    -Parameter->a[1] * Buffer->Output_Butter[1]
                                                    -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];

    for(uint16_t i=0;i<3;i++)
    {
      if(isnan(Buffer->Output_Butter[i])==1
            ||isnan(Buffer->Input_Butter[i])==1)
            {
                Buffer->Output_Butter[0]=curr_input;
                Buffer->Output_Butter[1]=curr_input;
                Buffer->Output_Butter[2]=curr_input;
                Buffer->Input_Butter[0]=curr_input;
                Buffer->Input_Butter[1]=curr_input;
                Buffer->Input_Butter[2]=curr_input;
                return curr_input;
            }
    }
  return Buffer->Output_Butter[2];
}
#define Kp 17.0f        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f       // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f   // half the sample period采样周期的一半


//学长的代码
void IMU_Offset_Init(void)
{
    // Set_Cutoff_Frequency(200,35,&Accel_Parameter);
    // Set_Cutoff_Frequency(200,11,&Gyro_Parameter);

    float a_x=0,a_y=0,a_z=0,gx_a=0,gy_a=0,gz_a=0;
    float gx_a_t=0,gy_a_t=0,gz_a_t=0;
    int n=500;


        for(int i=0;i<n;i++){

            MPU6050_get_value();

            a_x += LPButterworth(MPU6050_acc_transition(mpu6050_acc_x),&accel_filter_buf[0],&Accel_Parameter);
            a_y += LPButterworth(MPU6050_acc_transition(mpu6050_acc_y),&accel_filter_buf[1],&Accel_Parameter);
            a_z += LPButterworth(MPU6050_acc_transition(mpu6050_acc_z),&accel_filter_buf[2],&Accel_Parameter);

            gx_a_t= MPU6050_gyro_transition(mpu6050_gyro_x);
            gx_a_t = roundf(gx_a_t * 100.0f) / 100.0f;
            gx_a_t =LPButterworth(gx_a_t,&gyro_filter_buf[0],&Gyro_Parameter);
            gx_a+=gx_a_t;

            gy_a_t= MPU6050_gyro_transition(mpu6050_gyro_y);
            gy_a_t = roundf(gy_a_t * 100.0f) / 100.0f;
            gy_a_t= LPButterworth(gy_a_t,&gyro_filter_buf[1],&Gyro_Parameter);
            gy_a+=gy_a_t;

            gz_a_t= MPU6050_gyro_transition(mpu6050_gyro_z);
            gz_a_t = roundf(gz_a_t * 100.0f) / 100.0f;
            gz_a_t= LPButterworth(gz_a_t,&gyro_filter_buf[2],&Gyro_Parameter);
            gz_a+=gz_a_t;
            NEUQ_Delay_ms(1);
        }

    IMU.data.offset.acce_x = a_x/n;
    IMU.data.offset.acce_y = a_y/n;
    IMU.data.offset.acce_z = a_z/n;

    IMU.data.offset.gyro_x = gx_a/n;
    IMU.data.offset.gyro_y = gy_a/n;
    IMU.data.offset.gyro_z = gz_a/n;
}






//四元数转换为欧拉角
void QuaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw)
{
    // *roll = atan2f(2 * (q->q0 * q->q1 + q->q2 * q->q3), 1 - 2 * (q->q1 * q->q1 + q->q2 * q->q2));
    // *pitch = asinf(2 * (q->q0 * q->q2 - q->q3 * q->q1));
    // *yaw = atan2f(2 * (q->q0 * q->q3 + q->q1 * q->q2), 1 - 2 * (q->q2 * q->q2 + q->q3 * q->q3));

    *roll = atan2f(2.0f * (q->q0 * q->q1 + q->q2 * q->q3), 
                  1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2));
    
    *pitch = asinf(2.0f * (q->q0 * q->q2 - q->q3 * q->q1));
    
    *yaw = atan2f(2.0f * (q->q0 * q->q3 + q->q1 * q->q2), 
                1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3));

}

void Euler_ANGLE_To_REG(imu_Data_All_s *imu_in){
    imu_in->angle.pitch_r   =   imu_in->angle.pitch_a*PI/180.;
    imu_in->angle.roll_r    =   imu_in->angle.roll_a*PI/180.;
    imu_in->angle.yaw_r     =   imu_in->angle.yaw_a*PI/180.;
}

void Euler_REG_To_ANGLE(void){
    IMU.angle.pitch_a   =   IMU.angle.pitch_r*180./PI;
    IMU.angle.roll_a    =   IMU.angle.roll_r*180./PI;
    IMU.angle.yaw_a     =   IMU.angle.yaw_r*180./PI;


}

float invSqrt(float a){
    return 1./sqrtf(a);
}

double restrict_the_BETA(double b){
    if(b>0.0001){
        b=0.0001;
    }
    else if(b<0.00001){
        b=0.00001;
    }
    return b;
}

/* beta就是四元数微分方程求解的姿态算法的收敛速度，
 * 这个值就是陀螺仪的测量误差，
 * 通过查询IMU器件的PDF手册就可以知道，
 * 一般是一个很小的值。
 * */
Quaternion q={1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0};
Quaternion Q_info={1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0};
double BETA=0.04;
//-------------------------------------------------------------------------------------------------------------------
//  gyro                是弧度制
//  beta                是收敛速度
//  betaIn_is_true      是beta传入是否为有效值，0为有效值，否则beta为默认值
//  delta_t             为采样时间，单位是ms
//-------------------------------------------------------------------------------------------------------------------
void Madgwick_AHRS_6_DOF_Get_Angle(double beta, uint8_t beta_in_is_true, uint8_t delta_t) {

    // printf("q0: %f, q1: %f, q2: %f, q3: %f\n", q.q0, q.q1, q.q2, q.q3);

    float sampleFreq = 1000.0f / delta_t;
    if (beta_in_is_true != 0) {
        beta = 0.0001;  // 默认的较小 beta 值
    }

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // printf("q0: %f, q1: %f, q2: %f, q3: %f\n", q.q0, q.q1, q.q2, q.q3);

    

    // 储存旧的陀螺仪数据
    IMU.data.gx_r_old = IMU.data.gx_r;
    IMU.data.gy_r_old = IMU.data.gy_r;
    IMU.data.gz_r_old = IMU.data.gz_r;
    IMU.data.ax_old = IMU.data.ax;
    IMU.data.ay_old = IMU.data.ay;
    IMU.data.az_old = IMU.data.az;

    // 获取最新的IMU数据
    MPU6050_get_value();

    // 修正：不再对陀螺仪数据进行不必要的单位转换（不除以100）
    qDot1 = 0.5f * (-q.q1 * IMU.data.gx_r_old - q.q2 * IMU.data.gy_r_old - q.q3 * IMU.data.gz_r_old);
    qDot2 = 0.5f * (q.q0 * IMU.data.gx_r_old + q.q2 * IMU.data.gz_r_old - q.q3 * IMU.data.gy_r_old);
    qDot3 = 0.5f * (q.q0 * IMU.data.gy_r_old - q.q1 * IMU.data.gz_r_old + q.q3 * IMU.data.gx_r_old);
    qDot4 = 0.5f * (q.q0 * IMU.data.gz_r_old + q.q1 * IMU.data.gy_r_old - q.q2 * IMU.data.gx_r_old);
    

    // 只有在加速度计数据有效时，才计算反馈步骤
    if (!((IMU.data.ax_old == 0.0f) && (IMU.data.ay_old == 0.0f) && (IMU.data.az_old == 0.0f))) {
        float ax = IMU.data.ax_old, ay = IMU.data.ay_old, az = IMU.data.az_old;

        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 避免重复计算的辅助变量
        _2q0 = 2.0f * q.q0;
        _2q1 = 2.0f * q.q1;
        _2q2 = 2.0f * q.q2;
        _2q3 = 2.0f * q.q3;
        _4q0 = 4.0f * q.q0;
        _4q1 = 4.0f * q.q1;
        _4q2 = 4.0f * q.q2;
        _8q1 = 8.0f * q.q1;
        _8q2 = 8.0f * q.q2;
        q0q0 = q.q0 * q.q0;
        q1q1 = q.q1 * q.q1;
        q2q2 = q.q2 * q.q2;
        q3q3 = q.q3 * q.q3;

        // 计算梯度下降算法的修正步骤
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一化步骤大小
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用反馈步骤
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 保存旧的四元数
    q.q0_old = q.q0;
    q.q1_old = q.q1;
    q.q2_old = q.q2;
    q.q3_old = q.q3;

    // 积分四元数的变化速率，得到更新后的四元数
    q.q0 += qDot1 * (1.0f / sampleFreq);
    q.q1 += qDot2 * (1.0f / sampleFreq);
    q.q2 += qDot3 * (1.0f / sampleFreq);
    q.q3 += qDot4 * (1.0f / sampleFreq);

    // **四元数归一化**：确保四元数保持单位长度
    recipNorm = invSqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;


    // 将四元数转换为欧拉角
    QuaternionToEuler(&q, &IMU.angle.roll_r, &IMU.angle.pitch_r, &IMU.angle.yaw_r);

    // 储存旧的姿态角
    IMU.angle.yaw_a_old=IMU.angle.yaw_a;
    IMU.angle.pitch_a_old=IMU.angle.pitch_a;
    IMU.angle.roll_a_old=IMU.angle.roll_a;
    
    Euler_REG_To_ANGLE();

    double d_gx_r = abs((IMU.data.ax-IMU.data.ax_old)/(float)delta_t);
    double d_gy_r = abs((IMU.data.ay-IMU.data.ay_old)/(float)delta_t);
    double d_gz_r = abs((IMU.data.az-IMU.data.az_old)/(float)delta_t);

    BETA=restrict_the_BETA(d_gx_r*d_gx_r*d_gx_r*0.000006+d_gy_r*d_gy_r*d_gy_r*0.000002+d_gz_r*d_gz_r*d_gz_r*0.000002);



}



float param_Kp = 50.0;   // 加速度计(磁力计)的收敛速率比例增益50
float param_Ki = 0.20;   //陀螺仪收敛速率的积分增益 0.2
float I_ex, I_ey, I_ez;  // 误差积分
/**
  * brief IMU_AHRSupdate_noMagnetic  姿态解算融合，是Crazepony和核心算法
  * 使用的是互补滤波算法，没有使用Kalman滤波算法
  */
//-------------------------------------------------------------------------------------------------------------------
//  IMU_MODE            选择对应的IMU
//  delta_t             为采样时间，单位是ms
//  数据存储位置：        结构体Q_info
//   数据内容：           四元数   
//-------------------------------------------------------------------------------------------------------------------

//学长的代码
void AHRS_6_DOF_Get_Angle(float delta_T)
{
    delta_T = delta_T/1000.;

    MPU6050_get_value();

    // I_ex = fmaxf(fminf(I_ex, 0.1f), -0.1f);
    // I_ey = fmaxf(fminf(I_ey, 0.1f), -0.1f);
    // I_ez = fmaxf(fminf(I_ez, 0.1f), -0.1f);
    

    float gx=IMU.data.gx_a;
    float gy=IMU.data.gy_a;
    float gz=IMU.data.gz_a;
    float ax=IMU.data.ax;
    float ay=IMU.data.ay;
    float az=IMU.data.az;

//    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    //对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    //用叉乘误差来做PI修正陀螺零偏，
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += delta_T * ex;   // integral error scaled by Ki
    I_ey += delta_T * ey;
    I_ez += delta_T * ez;

    gx = gx+ param_Kp*ex + param_Ki*I_ex;
    gy = gy+ param_Kp*ey + param_Ki*I_ey;
    gz = gz+ param_Kp*ez + param_Ki*I_ez;


    /*数据修正完成，下面是四元数微分方程*/


    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;
    //    float delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;
    // normalise quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}




void MPU6050_get_value(void){
    
    NEUQ_MPU6050_read_data();


    IMU.data.ax = MPU6050_acc_transition(mpu6050_acc_x);
    IMU.data.ay = MPU6050_acc_transition(mpu6050_acc_y);
    IMU.data.az = MPU6050_acc_transition(mpu6050_acc_z);
    IMU.data.gx_a = MPU6050_gyro_transition(mpu6050_gyro_x);
    IMU.data.gy_a = MPU6050_gyro_transition(mpu6050_gyro_y);
    IMU.data.gz_a = MPU6050_gyro_transition(mpu6050_gyro_z);

    //校正误差
        //加速度计
        IMU.data.ax = IMU.data.ax - IMU.data.offset.acce_x;
        IMU.data.ax = LPButterworth(IMU.data.ax ,&accel_filter_buf[0],&Accel_Parameter);
        IMU.data.ax = IMU.data.ax/10.;
        IMU.data.ax = IMU.data.ax*10.;

        IMU.data.ay = IMU.data.ay - IMU.data.offset.acce_y;
        IMU.data.ay = LPButterworth(IMU.data.ay ,&accel_filter_buf[1],&Accel_Parameter);
        IMU.data.ay = IMU.data.ay/10.;
        IMU.data.ay = IMU.data.ay*10.;

        IMU.data.az = IMU.data.az - IMU.data.offset.acce_z;
        IMU.data.az = LPButterworth(IMU.data.az ,&accel_filter_buf[2],&Accel_Parameter);
        IMU.data.az = IMU.data.az/10.;
        IMU.data.az = IMU.data.az*10.;
//        //加速度计
//        IMU.data.ax = LPButterworth(IMU.data.ax - IMU.data.offset.acce_x,&accel_filter_buf[0],&Accel_Parameter);
//        IMU.data.ay = LPButterworth(IMU.data.ay - IMU.data.offset.acce_y,&accel_filter_buf[1],&Accel_Parameter);
//        IMU.data.az = LPButterworth(IMU.data.az - IMU.data.offset.acce_z,&accel_filter_buf[2],&Accel_Parameter);

        //陀螺仪
        IMU.data.gx_a = IMU.data.gx_a - IMU.data.offset.gyro_x;
        IMU.data.gx_a = LPButterworth(IMU.data.gx_a,&gyro_filter_buf[0],&Gyro_Parameter);
    //    IMU.data.gx_a = applyFilter(&filter,IMU.data.gx_a);
        IMU.data.gx_a = IMU.data.gx_a/100.;
        IMU.data.gx_a = IMU.data.gx_a*100.;

        IMU.data.gy_a = IMU.data.gy_a - IMU.data.offset.gyro_y;
        IMU.data.gy_a = LPButterworth(IMU.data.gy_a,&gyro_filter_buf[1],&Gyro_Parameter);
    //    IMU.data.gy_a = applyFilter(&filter,IMU.data.gy_a);
        IMU.data.gy_a = IMU.data.gy_a/100.;
        IMU.data.gy_a = IMU.data.gy_a*100.;

        IMU.data.gz_a = IMU.data.gz_a - IMU.data.offset.gyro_z;
        IMU.data.gz_a = LPButterworth(IMU.data.gz_a,&gyro_filter_buf[2],&Gyro_Parameter);
    //    IMU.data.gz_a = applyFilter(&filter,IMU.data.gz_a);
        IMU.data.gz_a = IMU.data.gz_a/100.;
        IMU.data.gz_a = IMU.data.gz_a*100.;

    //陀螺仪弧度转换
    IMU.data.gx_r = IMU.data.gx_a*PI/180.;
    IMU.data.gy_r = IMU.data.gy_a*PI/180.;
    IMU.data.gz_r = IMU.data.gz_a*PI/180.;
}





void print_angle(void) {
    // 通过串口输出姿态角
    printf("Roll: %.1f degrees\n", IMU.angle.roll_a);
    printf("Pitch: %.1f degrees\n", IMU.angle.pitch_a);
    printf("Yaw: %.1f degrees\n", IMU.angle.yaw_a);
    


}

void init_system(void) {

    // 初始化偏移量
    IMU_Offset_Init();
    
    // 初始化四元数滤波器
    // initFilter(&filter, 200.0, 40.0, 0.707);  // 设置滤波器的采样频率、截止频率和Q因子
}


void compute_orientation(int j) {

    // AHRS_6_DOF_Get_Angle(j);

    Madgwick_AHRS_6_DOF_Get_Angle(0.0001f,0,j); 

}

/* 用外部已滤波数据完成一次 Madgwick 四元数姿态更新(每 5ms 调用)
 * 输入: 三轴加速度(g, 已去零偏)、三轴角速度(°/s, 已去零偏)、更新周期(ms)
 * 输出: IMU.angle.roll_a / pitch_a / yaw_a(°)
 * 说明: 与六轴显示/VOFA 共用同一份一阶低通滤波后的数据(Q2_filt),
 *       滤波参数 alpha 直接决定姿态解算的平滑程度, 数据链保持一致 */
void Madgwick_Update(float ax, float ay, float az,
                     float gx_dps, float gy_dps, float gz_dps,
                     float dt_ms)
{
    float sampleFreq = 1000.0f / dt_ms;   /* 采样频率(Hz) */
    float beta = 0.1f;                 /* 梯度下降增益(固定) */

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* 角速度换算为 rad/s */
    float gx_r = gx_dps * PI / 180.0f;
    float gy_r = gy_dps * PI / 180.0f;
    float gz_r = gz_dps * PI / 180.0f;

    /* 由角速度计算四元数变化率 */
    qDot1 = 0.5f * (-q.q1 * gx_r - q.q2 * gy_r - q.q3 * gz_r);
    qDot2 = 0.5f * ( q.q0 * gx_r + q.q2 * gz_r - q.q3 * gy_r);
    qDot3 = 0.5f * ( q.q0 * gy_r - q.q1 * gz_r + q.q3 * gx_r);
    qDot4 = 0.5f * ( q.q0 * gz_r + q.q1 * gy_r - q.q2 * gx_r);

    /* 加速度计梯度下降修正(仅在加速度有效时) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * q.q0;
        _2q1 = 2.0f * q.q1;
        _2q2 = 2.0f * q.q2;
        _2q3 = 2.0f * q.q3;
        _4q0 = 4.0f * q.q0;
        _4q1 = 4.0f * q.q1;
        _4q2 = 4.0f * q.q2;
        _8q1 = 8.0f * q.q1;
        _8q2 = 8.0f * q.q2;
        q0q0 = q.q0 * q.q0;
        q1q1 = q.q1 * q.q1;
        q2q2 = q.q2 * q.q2;
        q3q3 = q.q3 * q.q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    /* 保存旧四元数(兼容原接口) */
    q.q0_old = q.q0;
    q.q1_old = q.q1;
    q.q2_old = q.q2;
    q.q3_old = q.q3;

    /* 积分得到新四元数 */
    q.q0 += qDot1 * (1.0f / sampleFreq);
    q.q1 += qDot2 * (1.0f / sampleFreq);
    q.q2 += qDot3 * (1.0f / sampleFreq);
    q.q3 += qDot4 * (1.0f / sampleFreq);

    /* 四元数归一化 */
    recipNorm = invSqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;

    /* 四元数转欧拉角并输出角度(°) */
    QuaternionToEuler(&q, &IMU.angle.roll_r, &IMU.angle.pitch_r, &IMU.angle.yaw_r);

    // static float yaw_decoupled_rad = 0.0f;
    // yaw_decoupled_rad += gz_r * (dt_ms / 1000.0f);          /* gz_r: 入口换算的 rad/s(已去零偏+滤波+零+速补偿) */
    // IMU.angle.yaw_a = yaw_decoupled_rad * 180.0f / PI;      /* 覆盖四元数的 yaw 显示输出 */

    IMU.angle.yaw_a_old   = IMU.angle.yaw_a;
    IMU.angle.pitch_a_old = IMU.angle.pitch_a;
    IMU.angle.roll_a_old  = IMU.angle.roll_a;

        Euler_REG_To_ANGLE();

    /* 航向解耦 + 双闸门防漂移(六轴 IMU 无磁力计):
     * (a) 解耦 —— 加速度计的 beta 修正会把噪声/振动注入四元数的航向分量,
     *     而航向本就没有绝对参考(六轴算法的航向本质都是陀螺积分),
     *     显示航向改用去零偏+滤波+零速补偿后的 GZ 独立积分(上电从 0 起,
     *     以初始方向为参考), 四元数只负责 roll/pitch 融合;
     * (b) 静止冻结 —— Q2_imu_still(零速判定成立, 见 Q2_IMU_Tick)时不积分,
     *     静置噪声与残余零偏一律进不了积分器, 静态漂移被结构性消除;
     * (c) 死区 —— |GZ| 低于 Q2_YAW_DEADBAND_DPS 视为"没有在转", 同样不积分,
     *     兜住零速判定 1s 确认期与台面振动干扰下的缓慢爬行。
     * 手持转动(几十 °/s)远高于两道闸门, 正常转动一点角度都不吃。 */
    static float yaw_decoupled_rad = 0.0f;
    float cos_pitch = cosf(IMU.angle.pitch_r);   /* 本拍四元数解算的俯仰(重力参考), rad */
    // if (Q2_imu_still == 0 && fabsf(gz_dps) > Q2_YAW_DEADBAND_DPS)
    //     yaw_decoupled_rad += gz_r * (dt_ms / 1000.0f);     
    if (cos_pitch > 0.17f)                       /* |pitch| < ~80°, 远离万向锁公式才可信 */
    {
        float yaw_rate_dps = (gy_dps * sinf(IMU.angle.roll_r)
                               + gz_dps * cosf(IMU.angle.roll_r)) / cos_pitch;
        if (Q2_imu_still == 0 && fabsf(yaw_rate_dps) > Q2_YAW_DEADBAND_DPS)
            yaw_decoupled_rad += yaw_rate_dps * (PI / 180.0f) * (dt_ms / 1000.0f);
    }
    //     yaw_decoupled_rad += gz_r * (dt_ms / 1000.0f);      /* gz_r: 入口换算的 rad/s */
    if (yaw_decoupled_rad > PI)                             /* ±180° 环绕, 长时间运行数值不发散 */
        yaw_decoupled_rad -= 2.0f * PI;
    else if (yaw_decoupled_rad < -PI)
        yaw_decoupled_rad += 2.0f * PI;
    IMU.angle.yaw_a = yaw_decoupled_rad * 180.0f / PI;      /* 覆盖四元数的 yaw 显示输出 */

    
}