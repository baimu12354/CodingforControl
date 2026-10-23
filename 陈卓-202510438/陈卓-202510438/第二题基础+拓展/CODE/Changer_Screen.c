#include "NEUQ_lib_include.h"

float accel_x,accel_y,accel_z;
float gyro_x,gyro_y,gyro_z;
float accel_x_last,accel_y_last,accel_z_last;
float gyro_x_last,gyro_y_last,gyro_z_last;

void SCREEN_CLEAR(void)
{
    NEUQ_LCD200_Clear();
}

void Changer_MPU6050_Data_UPdate(void)
{
    accel_x = MPU6050_acc_transition(mpu6050_acc_x);
    accel_y = MPU6050_acc_transition(mpu6050_acc_y);
    accel_z = MPU6050_acc_transition(mpu6050_acc_z);
    gyro_x = MPU6050_gyro_transition(mpu6050_gyro_x);
    gyro_y = MPU6050_gyro_transition(mpu6050_gyro_y);
    gyro_z = MPU6050_gyro_transition(mpu6050_gyro_z);
}

/*初始化*/

void SCREEN_SHOW_INIT(void)
{
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font20);
}

/*光标*/
void POINTER(uint8_t change)
{

    NEUQ_LCD200_DisplayString(200,40*change,"<<<");
}


/*菜单*/
void MENU_SHOW()
{
    
    NEUQ_LCD200_DisplayString(10,10,"MENU");
    NEUQ_LCD200_DisplayString(10,40,"LED_SHOW");
    NEUQ_LCD200_DisplayString(10,80,"MPU6050");

}

//LED闪烁方案四个方案
void LED_PLAN_SHOW(void)
{
    NEUQ_LCD200_DisplayString(10,10,"LED_SHOW");
    NEUQ_LCD200_DisplayString(10,40,"PLAN_1");
    NEUQ_LCD200_DisplayString(10,80,"PLAN_2");
    NEUQ_LCD200_DisplayString(10,120,"PLAN_3");
    NEUQ_LCD200_DisplayString(10,160,"PLAN_4");

    return;
}

void MPU6050_MENU_SHOW()
{
    NEUQ_LCD200_DisplayString(10,10,"MPU6050_SHOW");
    NEUQ_LCD200_DisplayString(10,40,"DATA_SHOW");
    NEUQ_LCD200_DisplayString(10,80,"SETTING");
    NEUQ_LCD200_DisplayString(10,120,"ATTITUDE_RESOLUTION");

}



void MPU6050_DATA_SHOW()
{
    SCREEN_CLEAR();

    NEUQ_LCD200_DisplayString(10,10,"A_X");
    NEUQ_LCD200_DisplayString(10,40,"A_Y");
    NEUQ_LCD200_DisplayString(10,80,"A_Z");
    NEUQ_LCD200_DisplayString(10,120,"G_X");
    NEUQ_LCD200_DisplayString(10,160,"G_Y");
    NEUQ_LCD200_DisplayString(10,200,"G_Z");

    NEUQ_LCD200_DisplayFloat(100,10,Q2_filt[Q2_AXIS_AX],6,2);
    NEUQ_LCD200_DisplayFloat(100,40,Q2_filt[Q2_AXIS_AY],6,2);
    NEUQ_LCD200_DisplayFloat(100,80,Q2_filt[Q2_AXIS_AZ],6,2);
    NEUQ_LCD200_DisplayFloat(100,120,Q2_filt[Q2_AXIS_GX],6,2);
    NEUQ_LCD200_DisplayFloat(100,160,Q2_filt[Q2_AXIS_GY],6,2);
    NEUQ_LCD200_DisplayFloat(100,200,Q2_filt[Q2_AXIS_GZ],6,2);

    accel_x_last = accel_x;
    accel_y_last = accel_y;
    accel_z_last = accel_z;
    gyro_x_last = gyro_x;
    gyro_y_last = gyro_y;
    gyro_z_last = gyro_z;
}

void MPU6050_SETTING_SHOW()
{
    NEUQ_LCD200_DisplayString(10,10,"MPU6050_SETTING");
    NEUQ_LCD200_DisplayString(10,40,"ACCELEROMETER");
    NEUQ_LCD200_DisplayString(10,80,"GYROSCOPE");
    NEUQ_LCD200_DisplayString(10,120,"SLEEP_MODE");
}

void ALPHA_SETTING_R_SHOW(void)
{
    NEUQ_LCD200_DisplayString(10,10,"ALPHA_R_SETTING :");
    NEUQ_LCD200_DisplayFloat(170,10,Q2_kalman_r,6,2);
    NEUQ_LCD200_DisplayString(10,40,"UP");
    NEUQ_LCD200_DisplayString(10,80,"DOWN");
}

void ALPHA_SETTING_Q_SHOW(void)
{
    NEUQ_LCD200_DisplayString(10,10,"ALPHA_Q_SETTING :");
    NEUQ_LCD200_DisplayFloat(170,10,Q2_kalman_q,6,4);
    NEUQ_LCD200_DisplayString(10,40,"UP");
    NEUQ_LCD200_DisplayString(10,80,"DOWN");
}

/* --------------------- 第二题: 波形与滤波设置页 --------------------- */
/* 光标项: 1 发送轴 / 2 滤波系数 / 3 发送开关 / 4 保存到 Flash
 * editing: 0 未编辑; 1..3 正在编辑对应项(行首显示 ">")
 * saved_show: 1 时显示 "SAVED!" 保存反馈 */
void Q2_SETTING_SHOW()
{

    NEUQ_LCD200_DisplayString(10, 10, "FILTER_SET");

    /* 1. 发送轴 */
    NEUQ_LCD200_DisplayString(30, 40, "AXIS:");
    NEUQ_LCD200_DisplayString(90, 40, (char *)Q2_AXIS_NAMES[Q2_axis]);

    /* 2. 滤波系数 alpha(0.05~1.00) */
    NEUQ_LCD200_DisplayString(30, 80, "ALPH_R:");
    NEUQ_LCD200_DisplayFloat(120, 80, Q2_kalman_r, 6, 2);

    NEUQ_LCD200_DisplayString(30, 120, "ALPH_Q:");
    NEUQ_LCD200_DisplayFloat(120, 120, Q2_kalman_q, 6, 4);

    /* 3. 发送开关 */
    NEUQ_LCD200_DisplayString(30, 160, "SEND:");
    NEUQ_LCD200_DisplayString(90, 160, Q2_send_on ? "ON" : "OFF");

    /* 4. 保存 */
    NEUQ_LCD200_DisplayString(30, 200, "SAVE");

}

/* 四元数(Madgwick)解算出的欧拉角, 单位 °, 更新周期 5ms(200Hz) */
void Q2_ATTITUDE_SHOW(void)
{
    NEUQ_LCD200_DisplayString(10, 10, "ATTITUDE");

    NEUQ_LCD200_DisplayString(10, 40, "ROLL :");
    NEUQ_LCD200_DisplayFloat(70, 40, IMU.angle.roll_a, 7, 1);
    NEUQ_LCD200_DisplayString(150, 40, "d");

    NEUQ_LCD200_DisplayString(10, 80, "PITCH:");
    NEUQ_LCD200_DisplayFloat(70, 80, IMU.angle.pitch_a, 7, 1);
    NEUQ_LCD200_DisplayString(150, 80, "d");

    NEUQ_LCD200_DisplayString(10, 120, "YAW  :");
    NEUQ_LCD200_DisplayFloat(70, 120, IMU.angle.yaw_a, 7, 1);
    NEUQ_LCD200_DisplayString(150, 120, "d");

    /* 说明: 姿态更新周期(见验收要求) */
    NEUQ_LCD200_DisplayString(10, 160, "UPD:200Hz");
    NEUQ_LCD200_DisplayString(10, 200, Q2_cal_ok ? "CAL:OK" : "CAL:---");
}

void LED_WAIT(uint8_t g_next,uint8_t g_cur){
    if(g_cur != g_next){
    NEUQ_LCD200_DisplayString(120,g_next*40+40,"wait");
    NEUQ_LCD200_DisplayString(120,g_cur*40+40,"ing");    
    }
    else{
    // NEUQ_LCD200_Clear();
    NEUQ_LCD200_DisplayString(120,g_cur*40+40,"ing");  
    }
    
}
